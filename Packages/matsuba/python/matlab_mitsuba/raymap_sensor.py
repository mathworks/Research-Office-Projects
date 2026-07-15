"""Custom Mitsuba sensor that uses pre-computed per-pixel ray maps.

This module defines a sensor plugin ('raymap') that looks up ray origins and
directions from NumPy arrays stored in a global registry.  The arrays are
indexed by pixel coordinates, enabling arbitrary camera/lens models to be
defined externally (e.g., from MATLAB).

Supports depth of field via aperture sampling when aperture_radius > 0.
"""

import numpy as np
import mitsuba as mi
import drjit as dr

# Global registry: raymap_id -> dict with ray data + optional DOF params
_RAYMAP_REGISTRY: dict[int, dict] = {}
_NEXT_RAYMAP_ID: int = 1
_REGISTERED: bool = False


def register_raymap(origins: np.ndarray, directions: np.ndarray,
                    aperture_radius: float = 0.0,
                    focus_distance=None,
                    vignetting: np.ndarray = None) -> int:
    """Store ray arrays in the registry and return a raymap ID.

    Parameters
    ----------
    origins : np.ndarray
        Ray origins, shape (H, W, 3).
    directions : np.ndarray
        Ray directions, shape (H, W, 3). Will be normalized.
    aperture_radius : float, optional
        Exit pupil radius for DOF. 0 = pinhole (no DOF).
    focus_distance : float or np.ndarray, optional
        Focus distance along chief ray. Scalar for uniform, (H, W) array
        for field curvature.
    vignetting : np.ndarray, optional
        Per-pixel vignetting weights, shape (H, W), values in [0, 1].

    Returns
    -------
    int
        Raymap ID for referencing in the sensor plugin.
    """
    global _NEXT_RAYMAP_ID

    origins = np.asarray(origins, dtype=np.float32)
    directions = np.asarray(directions, dtype=np.float32)

    if origins.ndim != 3 or origins.shape[2] != 3:
        raise ValueError(f"origins must have shape (H, W, 3), got {origins.shape}")
    if directions.ndim != 3 or directions.shape[2] != 3:
        raise ValueError(f"directions must have shape (H, W, 3), got {directions.shape}")
    if origins.shape[:2] != directions.shape[:2]:
        raise ValueError(
            f"origins and directions must have same H, W dimensions, "
            f"got {origins.shape[:2]} vs {directions.shape[:2]}"
        )

    # Normalize directions (preserve zero vectors as mask sentinel)
    norms = np.linalg.norm(directions, axis=2, keepdims=True)
    valid = norms > 1e-12
    norms = np.where(valid, norms, 1.0)
    directions = np.where(valid, directions / norms, 0.0)

    entry = {
        "origins": origins,
        "directions": directions,
        "aperture_radius": float(aperture_radius),
    }

    # Focus distance: scalar or per-pixel array
    if focus_distance is not None:
        if isinstance(focus_distance, (int, float)):
            entry["focus_distance"] = float(focus_distance)
        else:
            entry["focus_distance"] = np.asarray(focus_distance, dtype=np.float32)
    elif aperture_radius > 0:
        raise ValueError("focus_distance is required when aperture_radius > 0")

    # Vignetting map
    if vignetting is not None:
        entry["vignetting"] = np.asarray(vignetting, dtype=np.float32)

    rid = _NEXT_RAYMAP_ID
    _RAYMAP_REGISTRY[rid] = entry
    _NEXT_RAYMAP_ID += 1
    return rid


def update_raymap(raymap_id: int, origins: np.ndarray = None,
                  directions: np.ndarray = None) -> bool:
    """Update ray arrays for an existing raymap ID."""
    if raymap_id not in _RAYMAP_REGISTRY:
        raise KeyError(f"Raymap ID {raymap_id} not found in registry.")

    entry = _RAYMAP_REGISTRY[raymap_id]

    if origins is not None:
        origins = np.asarray(origins, dtype=np.float32)
        if origins.shape != entry["origins"].shape:
            raise ValueError(
                f"New origins shape {origins.shape} doesn't match "
                f"existing shape {entry['origins'].shape}"
            )
        entry["origins"] = origins

    if directions is not None:
        directions = np.asarray(directions, dtype=np.float32)
        if directions.shape != entry["directions"].shape:
            raise ValueError(
                f"New directions shape {directions.shape} doesn't match "
                f"existing shape {entry['directions'].shape}"
            )
        norms = np.linalg.norm(directions, axis=2, keepdims=True)
        valid = norms > 1e-12
        norms = np.where(valid, norms, 1.0)
        entry["directions"] = np.where(valid, directions / norms, 0.0)

    return True


def release_raymap(raymap_id: int) -> bool:
    """Remove a raymap from the registry."""
    return _RAYMAP_REGISTRY.pop(raymap_id, None) is not None


def get_raymap(raymap_id: int) -> dict:
    """Retrieve raymap entry by ID."""
    if raymap_id not in _RAYMAP_REGISTRY:
        raise KeyError(f"Raymap ID {raymap_id} not found in registry.")
    return _RAYMAP_REGISTRY[raymap_id]


def ensure_registered():
    """Register the 'raymap' sensor plugin with Mitsuba (idempotent)."""
    global _REGISTERED
    if _REGISTERED:
        return
    try:
        mi.register_sensor("raymap", lambda props: RaymapSensor(props))
    except Exception:
        pass  # Already registered (e.g., after module reload)
    _REGISTERED = True


def _is_scalar_variant() -> bool:
    """Check if the current Mitsuba variant is scalar (non-vectorized)."""
    variant = mi.variant()
    return variant is not None and variant.startswith("scalar")


class RaymapSensor(mi.Sensor):
    """Mitsuba sensor that generates rays from pre-computed per-pixel arrays.

    Supports depth of field when aperture_radius > 0. DOF is implemented
    by sampling the aperture disk (via sample3) and aiming rays at the
    focus point along the chief ray.

    Optimizations:
    - Uniform origins detected: skips origin gather, uses constant position
    - to_world pre-applied: directions transformed at construction
    - Vectorized Dr.Jit gather for LLVM/CUDA variants
    """

    def __init__(self, props=mi.Properties()):
        super().__init__(props)
        self.m_needs_sample_2 = True

        self.raymap_id = props.get("raymap_id", 1)
        entry = get_raymap(int(self.raymap_id))
        self.height = int(entry["origins"].shape[0])
        self.width = int(entry["origins"].shape[1])

        origins = entry["origins"]
        directions = entry["directions"]

        # DOF configuration
        self._aperture_radius = entry.get("aperture_radius", 0.0)
        self._has_dof = self._aperture_radius > 0
        self.m_needs_sample_3 = self._has_dof

        # Focus distance: scalar or per-pixel
        if self._has_dof:
            fd = entry["focus_distance"]
            if isinstance(fd, (int, float)):
                self._focus_uniform = True
                self._focus_distance = float(fd)
            else:
                self._focus_uniform = False
                self._focus_map = fd.reshape(-1)  # flat H*W

        # Vignetting map (optional)
        self._has_vignetting = "vignetting" in entry
        if self._has_vignetting:
            self._vignetting = entry["vignetting"].reshape(-1)  # flat H*W

        # Detect uniform origins (all same, e.g., pinhole cameras)
        self._uniform_origin = bool(np.all(origins == origins[0, 0]))

        # Extract to_world as a 4x4 numpy matrix
        trafo = self.world_transform()
        if _is_scalar_variant():
            trafo_np = np.array(trafo.matrix, dtype=np.float32)
        else:
            # LLVM/CUDA: matrix entries are Dr.Jit arrays
            m = trafo.matrix
            trafo_np = np.array([[m[i, j].numpy()[0] for j in range(4)]
                                 for i in range(4)], dtype=np.float32)

        if self._uniform_origin:
            o = origins[0, 0].astype(np.float32)
            # Apply to_world to the constant origin
            rot = trafo_np[:3, :3]
            trans = trafo_np[:3, 3]
            origin_world = rot @ o + trans
            self._const_origin = mi.Point3f(
                float(origin_world[0]), float(origin_world[1]),
                float(origin_world[2]))
        else:
            flat_o = origins.reshape(-1, 3)
            rot = trafo_np[:3, :3]
            trans = trafo_np[:3, 3]
            self._origins_world = ((flat_o @ rot.T) + trans).astype(np.float32)

        # Pre-apply to_world rotation to directions
        flat_d = directions.reshape(-1, 3)
        trafo_rot = trafo_np[:3, :3]
        world_dirs = (flat_d @ trafo_rot.T).astype(np.float32)
        # Re-normalize (preserve zeros)
        norms = np.linalg.norm(world_dirs, axis=1, keepdims=True)
        valid = norms > 1e-12
        norms = np.where(valid, norms, 1.0)
        self._dirs_world = np.ascontiguousarray(
            np.where(valid, world_dirs / norms, 0.0), dtype=np.float32)

        # For DOF: pre-compute basis vectors perpendicular to each direction
        # (needed to offset origin on the aperture disk)
        if self._has_dof:
            self._compute_aperture_bases()

        # For vectorized variants, pre-load Dr.Jit arrays
        # Dr.Jit requires 1D, float32, C-contiguous arrays
        self._scalar = _is_scalar_variant()
        if not self._scalar:
            self._dx = mi.Float(self._dirs_world[:, 0].copy())
            self._dy = mi.Float(self._dirs_world[:, 1].copy())
            self._dz = mi.Float(self._dirs_world[:, 2].copy())
            if not self._uniform_origin:
                self._ox = mi.Float(self._origins_world[:, 0].copy())
                self._oy = mi.Float(self._origins_world[:, 1].copy())
                self._oz = mi.Float(self._origins_world[:, 2].copy())
            if self._has_dof:
                self._bx_x = mi.Float(self._basis_x[:, 0].copy())
                self._bx_y = mi.Float(self._basis_x[:, 1].copy())
                self._bx_z = mi.Float(self._basis_x[:, 2].copy())
                self._by_x = mi.Float(self._basis_y[:, 0].copy())
                self._by_y = mi.Float(self._basis_y[:, 1].copy())
                self._by_z = mi.Float(self._basis_y[:, 2].copy())
                if not self._focus_uniform:
                    self._focus_dr = mi.Float(self._focus_map.copy())
                if self._has_vignetting:
                    self._vignetting_dr = mi.Float(self._vignetting.copy())

    def _compute_aperture_bases(self):
        """Compute per-ray orthonormal basis perpendicular to direction.

        These basis vectors define the aperture disk plane for DOF sampling.
        """
        dirs = self._dirs_world  # (N, 3)
        N = dirs.shape[0]

        # Choose an arbitrary vector not parallel to direction
        up = np.zeros((N, 3), dtype=np.float32)
        # Use (0,1,0) unless direction is nearly parallel to it
        up[:, 1] = 1.0
        parallel = np.abs(dirs[:, 1]) > 0.99
        if np.any(parallel):
            up[parallel] = [1.0, 0.0, 0.0]

        # basis_x = normalize(cross(direction, up))
        bx = np.cross(dirs, up)
        bx_norms = np.linalg.norm(bx, axis=1, keepdims=True)
        bx_norms = np.maximum(bx_norms, 1e-12)
        bx = bx / bx_norms

        # basis_y = cross(direction, basis_x)
        by = np.cross(dirs, bx)

        self._basis_x = bx  # (N, 3)
        self._basis_y = by  # (N, 3)

    def sample_ray(self, time, sample1, sample2, sample3, active=True):
        if self._scalar:
            return self._sample_ray_scalar(time, sample2, sample3)
        else:
            return self._sample_ray_vectorized(time, sample2, sample3)

    def _sample_ray_scalar(self, time, sample2, sample3):
        """Scalar variant: one ray at a time."""
        px_f = float(sample2.x) * self.width - 0.5
        py_f = float(sample2.y) * self.height - 0.5
        px_f = max(0.0, min(px_f, self.width - 1.0))
        py_f = max(0.0, min(py_f, self.height - 1.0))

        # Bilinear interpolation
        px0 = int(px_f)
        py0 = int(py_f)
        px1 = min(px0 + 1, self.width - 1)
        py1 = min(py0 + 1, self.height - 1)
        fx = px_f - px0
        fy = py_f - py0

        w00 = (1.0 - fx) * (1.0 - fy)
        w01 = fx * (1.0 - fy)
        w10 = (1.0 - fx) * fy
        w11 = fx * fy

        idx00 = py0 * self.width + px0
        idx01 = py0 * self.width + px1
        idx10 = py1 * self.width + px0
        idx11 = py1 * self.width + px1

        # Interpolate direction
        d = (w00 * self._dirs_world[idx00] + w01 * self._dirs_world[idx01] +
             w10 * self._dirs_world[idx10] + w11 * self._dirs_world[idx11])
        d_norm = np.linalg.norm(d)

        if d_norm < 1e-12:
            ray = mi.Ray3f(mi.Point3f(0, 0, 0), mi.Vector3f(0, 0, 1), time=time)
            return ray, mi.Color3f(0.0)

        d = d / d_norm

        # Origin
        if self._uniform_origin:
            origin = np.array([self._const_origin.x, self._const_origin.y,
                               self._const_origin.z], dtype=np.float32)
        else:
            origin = (w00 * self._origins_world[idx00] +
                      w01 * self._origins_world[idx01] +
                      w10 * self._origins_world[idx10] +
                      w11 * self._origins_world[idx11])

        # Depth of field: perturb origin on aperture disk
        if self._has_dof:
            # Get focus distance for this pixel
            if self._focus_uniform:
                fd = self._focus_distance
            else:
                fd = (w00 * self._focus_map[idx00] + w01 * self._focus_map[idx01] +
                      w10 * self._focus_map[idx10] + w11 * self._focus_map[idx11])

            # Focus point along chief ray
            focus_point = origin + d * fd

            # Sample aperture disk using sample3
            # Map sample3 from [0,1]^2 to uniform disk
            s3x = float(sample3.x)
            s3y = float(sample3.y)
            r = self._aperture_radius * np.sqrt(s3x)
            theta = 2.0 * np.pi * s3y

            # Aperture offset in the plane perpendicular to direction
            bx = (w00 * self._basis_x[idx00] + w01 * self._basis_x[idx01] +
                  w10 * self._basis_x[idx10] + w11 * self._basis_x[idx11])
            by = (w00 * self._basis_y[idx00] + w01 * self._basis_y[idx01] +
                  w10 * self._basis_y[idx10] + w11 * self._basis_y[idx11])

            aperture_offset = r * (np.cos(theta) * bx + np.sin(theta) * by)
            origin = origin + aperture_offset

            # New direction: from perturbed origin toward focus point
            d = focus_point - origin
            d_norm = np.linalg.norm(d)
            if d_norm > 1e-12:
                d = d / d_norm

        # Vignetting weight
        weight = 1.0
        if self._has_vignetting:
            weight = (w00 * self._vignetting[idx00] +
                      w01 * self._vignetting[idx01] +
                      w10 * self._vignetting[idx10] +
                      w11 * self._vignetting[idx11])

        origin_pt = mi.Point3f(float(origin[0]), float(origin[1]), float(origin[2]))
        direction = mi.Vector3f(float(d[0]), float(d[1]), float(d[2]))
        ray = mi.Ray3f(origin_pt, direction, time=time)
        return ray, mi.Color3f(weight)

    def _sample_ray_vectorized(self, time, sample2, sample3):
        """Vectorized variant: Dr.Jit gather for parallel ray generation."""
        px = mi.Float(sample2.x) * self.width - 0.5
        py_coord = mi.Float(sample2.y) * self.height - 0.5
        px = dr.clamp(px, 0.0, float(self.width - 1))
        py_coord = dr.clamp(py_coord, 0.0, float(self.height - 1))

        # Bilinear indices
        px0 = mi.UInt32(dr.clamp(mi.Int32(px), 0, self.width - 1))
        py0 = mi.UInt32(dr.clamp(mi.Int32(py_coord), 0, self.height - 1))
        px1 = dr.minimum(px0 + 1, mi.UInt32(self.width - 1))
        py1 = dr.minimum(py0 + 1, mi.UInt32(self.height - 1))

        fx = px - mi.Float(mi.Int32(px))
        fy = py_coord - mi.Float(mi.Int32(py_coord))

        idx00 = py0 * self.width + px0
        idx01 = py0 * self.width + px1
        idx10 = py1 * self.width + px0
        idx11 = py1 * self.width + px1

        w00 = (1.0 - fx) * (1.0 - fy)
        w01 = fx * (1.0 - fy)
        w10 = (1.0 - fx) * fy
        w11 = fx * fy

        # Gather directions
        dx = (w00 * dr.gather(mi.Float, self._dx, idx00) +
              w01 * dr.gather(mi.Float, self._dx, idx01) +
              w10 * dr.gather(mi.Float, self._dx, idx10) +
              w11 * dr.gather(mi.Float, self._dx, idx11))
        dy = (w00 * dr.gather(mi.Float, self._dy, idx00) +
              w01 * dr.gather(mi.Float, self._dy, idx01) +
              w10 * dr.gather(mi.Float, self._dy, idx10) +
              w11 * dr.gather(mi.Float, self._dy, idx11))
        dz = (w00 * dr.gather(mi.Float, self._dz, idx00) +
              w01 * dr.gather(mi.Float, self._dz, idx01) +
              w10 * dr.gather(mi.Float, self._dz, idx10) +
              w11 * dr.gather(mi.Float, self._dz, idx11))

        direction = dr.normalize(mi.Vector3f(dx, dy, dz))

        # Origin
        if self._uniform_origin:
            ox = mi.Float(self._const_origin.x)
            oy = mi.Float(self._const_origin.y)
            oz = mi.Float(self._const_origin.z)
        else:
            ox = (w00 * dr.gather(mi.Float, self._ox, idx00) +
                  w01 * dr.gather(mi.Float, self._ox, idx01) +
                  w10 * dr.gather(mi.Float, self._ox, idx10) +
                  w11 * dr.gather(mi.Float, self._ox, idx11))
            oy = (w00 * dr.gather(mi.Float, self._oy, idx00) +
                  w01 * dr.gather(mi.Float, self._oy, idx01) +
                  w10 * dr.gather(mi.Float, self._oy, idx10) +
                  w11 * dr.gather(mi.Float, self._oy, idx11))
            oz = (w00 * dr.gather(mi.Float, self._oz, idx00) +
                  w01 * dr.gather(mi.Float, self._oz, idx01) +
                  w10 * dr.gather(mi.Float, self._oz, idx10) +
                  w11 * dr.gather(mi.Float, self._oz, idx11))

        origin = mi.Point3f(ox, oy, oz)
        weight = mi.Color3f(1.0)

        # Depth of field
        if self._has_dof:
            # Focus distance
            if self._focus_uniform:
                fd = mi.Float(self._focus_distance)
            else:
                fd = (w00 * dr.gather(mi.Float, self._focus_dr, idx00) +
                      w01 * dr.gather(mi.Float, self._focus_dr, idx01) +
                      w10 * dr.gather(mi.Float, self._focus_dr, idx10) +
                      w11 * dr.gather(mi.Float, self._focus_dr, idx11))

            # Focus point along chief ray
            focus_point = mi.Point3f(
                ox + direction.x * fd,
                oy + direction.y * fd,
                oz + direction.z * fd,
            )

            # Sample aperture disk (concentric disk mapping)
            s3x = mi.Float(sample3.x)
            s3y = mi.Float(sample3.y)
            r = mi.Float(self._aperture_radius) * dr.sqrt(s3x)
            theta = 2.0 * dr.pi * s3y

            # Gather basis vectors for aperture plane
            bx_x = (w00 * dr.gather(mi.Float, self._bx_x, idx00) +
                     w01 * dr.gather(mi.Float, self._bx_x, idx01) +
                     w10 * dr.gather(mi.Float, self._bx_x, idx10) +
                     w11 * dr.gather(mi.Float, self._bx_x, idx11))
            bx_y = (w00 * dr.gather(mi.Float, self._bx_y, idx00) +
                     w01 * dr.gather(mi.Float, self._bx_y, idx01) +
                     w10 * dr.gather(mi.Float, self._bx_y, idx10) +
                     w11 * dr.gather(mi.Float, self._bx_y, idx11))
            bx_z = (w00 * dr.gather(mi.Float, self._bx_z, idx00) +
                     w01 * dr.gather(mi.Float, self._bx_z, idx01) +
                     w10 * dr.gather(mi.Float, self._bx_z, idx10) +
                     w11 * dr.gather(mi.Float, self._bx_z, idx11))
            by_x = (w00 * dr.gather(mi.Float, self._by_x, idx00) +
                     w01 * dr.gather(mi.Float, self._by_x, idx01) +
                     w10 * dr.gather(mi.Float, self._by_x, idx10) +
                     w11 * dr.gather(mi.Float, self._by_x, idx11))
            by_y = (w00 * dr.gather(mi.Float, self._by_y, idx00) +
                     w01 * dr.gather(mi.Float, self._by_y, idx01) +
                     w10 * dr.gather(mi.Float, self._by_y, idx10) +
                     w11 * dr.gather(mi.Float, self._by_y, idx11))
            by_z = (w00 * dr.gather(mi.Float, self._by_z, idx00) +
                     w01 * dr.gather(mi.Float, self._by_z, idx01) +
                     w10 * dr.gather(mi.Float, self._by_z, idx10) +
                     w11 * dr.gather(mi.Float, self._by_z, idx11))

            cos_t = dr.cos(theta)
            sin_t = dr.sin(theta)

            # Offset origin on aperture disk
            offset_x = r * (cos_t * bx_x + sin_t * by_x)
            offset_y = r * (cos_t * bx_y + sin_t * by_y)
            offset_z = r * (cos_t * bx_z + sin_t * by_z)

            origin = mi.Point3f(ox + offset_x, oy + offset_y, oz + offset_z)

            # Direction: from perturbed origin toward focus point
            direction = dr.normalize(focus_point - origin)

        # Vignetting
        if self._has_vignetting:
            vig = (w00 * dr.gather(mi.Float, self._vignetting_dr, idx00) +
                   w01 * dr.gather(mi.Float, self._vignetting_dr, idx01) +
                   w10 * dr.gather(mi.Float, self._vignetting_dr, idx10) +
                   w11 * dr.gather(mi.Float, self._vignetting_dr, idx11))
            weight = mi.Color3f(vig)

        ray = mi.Ray3f(origin, direction, time=time)
        return ray, weight

    def sample_ray_differential(self, time, sample1, sample2, sample3, active=True):
        ray, weight = self.sample_ray(time, sample1, sample2, sample3, active)
        return mi.RayDifferential3f(ray), weight

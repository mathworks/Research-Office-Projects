"""Python adapter for MATSUBA.

This module provides bridge functions that wrap Mitsuba 3's Python API,
exposing scene loading, parameter manipulation, rendering, and lifecycle
management through a simple procedural interface suitable for calling
from MATLAB via the MATLAB-Python interop layer.
"""

import os
import tempfile
import traceback

import drjit as dr
import mitsuba as mi
import numpy as np

_SCENES: dict = {}
_PARAMS: dict = {}  # Cached mi.traverse() results per scene ID
_NEXT_ID: int = 1
_TEMP_FILES: dict[int, list[str]] = {}
_MITRANSIENT_LOADED: bool = False
_RAYMAP_SENSOR_LOADED: bool = False


def is_valid(scene_id: int) -> bool:
    """Check if a scene ID is still valid in the registry."""
    return scene_id in _SCENES


def _get_scene(scene_id: int):
    """Look up a scene by its integer ID.

    Raises
    ------
    KeyError
        If *scene_id* is not present in the registry.
    """
    try:
        return _SCENES[scene_id]
    except KeyError:
        raise KeyError(
            f"Scene ID {scene_id} not found in registry. "
            "The Python process may have been restarted (terminate(pyenv) "
            "invalidates all scene IDs)."
        ) from None


def _get_params(scene_id: int):
    """Get cached mi.traverse() result for a scene, creating if needed."""
    if scene_id not in _PARAMS:
        scene = _get_scene(scene_id)
        _PARAMS[scene_id] = mi.traverse(scene)
    return _PARAMS[scene_id]


def _invalidate_params(scene_id: int):
    """Remove cached params for a scene (call after structural changes)."""
    _PARAMS.pop(scene_id, None)


def available_variants() -> list[str]:
    """Return a sorted list of compiled Mitsuba variants.

    Example return value: ``['scalar_rgb', 'scalar_spectral', ...]``
    """
    return sorted(mi.variants())


# Preferred variant order: GPU AD > GPU > LLVM AD > LLVM > scalar (rgb preferred over spectral)
_VARIANT_PRIORITY = [
    "cuda_ad_rgb",
    "cuda_rgb",
    "cuda_ad_spectral",
    "cuda_spectral",
    "llvm_ad_rgb",
    "llvm_rgb",
    "llvm_ad_spectral",
    "llvm_spectral",
    "scalar_rgb",
    "scalar_spectral",
]


def best_variant() -> str:
    """Return the best available Mitsuba variant for the current system.

    Selection priority: CUDA AD RGB > CUDA RGB > LLVM AD RGB > LLVM RGB >
    scalar RGB, with spectral variants as fallbacks within each tier.

    Each candidate is tested by calling ``mi.set_variant()`` **and** verifying
    the backend is functional.  For LLVM/CUDA variants, a minimal Dr.Jit
    operation is attempted to ensure the shared library is actually available
    (e.g., ``libLLVM.dylib`` on macOS).  The first variant that passes both
    checks is returned.

    Falls back to the first available variant if none from the priority
    list is found.  Raises ``RuntimeError`` if no variants are available.
    """
    installed = set(mi.variants())
    if not installed:
        raise RuntimeError("No Mitsuba variants are available.")

    candidates = [v for v in _VARIANT_PRIORITY if v in installed]
    # Also include any installed variants not in the priority list
    candidates.extend(sorted(installed - set(candidates)))

    for v in candidates:
        try:
            mi.set_variant(v)
            # Verify the backend is actually functional — set_variant only
            # checks compilation, not runtime availability (e.g., LLVM lib).
            if "llvm" in v:
                _ = dr.llvm.Float(1.0)
            elif "cuda" in v:
                _ = dr.cuda.Float(1.0)
            return v
        except Exception:
            continue

    raise RuntimeError(
        "No Mitsuba variant could be activated. "
        f"Compiled variants: {sorted(installed)}"
    )


def set_variant(name: str) -> bool:
    """Activate a Mitsuba variant (e.g. ``'scalar_rgb'``).

    When switching to a scalar variant, any pending Dr.Jit JIT state from a
    previous LLVM/CUDA variant is flushed to avoid stale backend references.

    Returns ``True`` on success.

    Raises
    ------
    RuntimeError
        If the variant cannot be set.
    """
    try:
        mi.set_variant(name)
        # Flush stale JIT state when switching to scalar — a previous
        # LLVM/CUDA variant probe (e.g. from best_variant) may have
        # initialized thread state that causes errors later.
        if name.startswith("scalar"):
            dr.flush_malloc_cache()
        return True
    except Exception as exc:
        raise RuntimeError(f"Failed to set variant '{name}': {exc}") from exc


def load_file(path: str) -> int:
    """Load a Mitsuba scene from an XML file.

    Returns an integer scene ID that can be used with other bridge functions.

    Raises
    ------
    FileNotFoundError
        If *path* does not exist on disk.
    RuntimeError
        If Mitsuba fails to parse the file.
    """
    global _NEXT_ID
    if not os.path.exists(path):
        raise FileNotFoundError(f"Scene file not found: {path}")
    try:
        scene = mi.load_file(path)
    except Exception as exc:
        raise RuntimeError(f"Failed to load scene file '{path}': {exc}") from exc
    sid = _NEXT_ID
    _SCENES[sid] = scene
    _NEXT_ID += 1
    return sid


def load_cornell_box() -> int:
    """Load Mitsuba's built-in Cornell box scene.

    Returns an integer scene ID.
    """
    global _NEXT_ID
    try:
        scene = mi.load_dict(mi.cornell_box())
    except Exception as exc:
        raise RuntimeError(f"Failed to load Cornell box: {exc}") from exc
    sid = _NEXT_ID
    _SCENES[sid] = scene
    _NEXT_ID += 1
    return sid


def load_dict(scene_dict: dict) -> int:
    """Load a Mitsuba scene from a Python dictionary.

    Returns an integer scene ID.

    Raises
    ------
    RuntimeError
        If Mitsuba fails to construct the scene.
    """
    global _NEXT_ID
    try:
        scene = mi.load_dict(scene_dict)
    except Exception as exc:
        raise RuntimeError(f"Failed to load scene from dict: {exc}") from exc
    sid = _NEXT_ID
    _SCENES[sid] = scene
    _NEXT_ID += 1
    return sid


def list_params(scene_id: int) -> list[str]:
    """Return a sorted list of traversable parameter names for a scene.

    Raises
    ------
    KeyError
        If *scene_id* is not in the registry.
    """
    params = _get_params(scene_id)
    return sorted(params.keys())


def set_params(scene_id: int, updates: dict) -> bool:
    """Update one or more scene parameters.

    Values that are lists or numpy arrays of length 3 are automatically
    converted to ``mi.Color3f``.  Scalar floats are passed through
    unchanged.

    Returns ``True`` on success.

    Raises
    ------
    KeyError
        If *scene_id* is not in the registry.
    RuntimeError
        If a parameter update fails.
    """
    try:
        params = _get_params(scene_id)
        for key, value in updates.items():
            if isinstance(value, (list, np.ndarray)):
                if len(value) == 3:
                    value = mi.Color3f(value)
                elif len(value) == 2:
                    value = mi.ScalarPoint2u(int(value[0]), int(value[1]))
            params[key] = value
        params.update()
        return True
    except KeyError:
        raise
    except Exception as exc:
        raise RuntimeError(f"Failed to set params: {exc}") from exc


def set_transform(scene_id: int, param_name: str, matrix4x4) -> bool:
    """Set a transform parameter from a 4×4 matrix.

    *matrix4x4* may be a nested list or a numpy array of shape ``(4, 4)``.

    Returns ``True`` on success.

    Raises
    ------
    KeyError
        If *scene_id* is not in the registry.
    RuntimeError
        If the transform update fails.
    """
    try:
        if isinstance(matrix4x4, np.ndarray):
            matrix4x4 = matrix4x4.tolist()
        transform = mi.ScalarTransform4f(matrix4x4)
        params = _get_params(scene_id)
        params[param_name] = transform
        params.update()
        return True
    except KeyError:
        raise
    except Exception as exc:
        raise RuntimeError(
            f"Failed to set transform '{param_name}': {exc}"
        ) from exc


def render(scene_id: int, spp: int = 16, seed: int = 0) -> np.ndarray:
    """Render a scene and return the image as a float32 numpy array.

    The returned array has shape ``(H, W, C)``.  GPU/LLVM variants
    return ``TensorXf`` objects; these are automatically converted to
    NumPy via ``np.array(img)`` (which calls ``img.numpy()`` internally).

    Parameters
    ----------
    seed : int
        Random seed for the sampler.  Use different seeds when averaging
        multiple low-spp passes for progressive rendering.

    Raises
    ------
    KeyError
        If *scene_id* is not in the registry.
    RuntimeError
        If rendering fails.
    """
    scene = _get_scene(scene_id)
    try:
        img = mi.render(scene, spp=spp, seed=seed)
        # GPU/LLVM variants return TensorXf; convert to numpy.
        # np.array() handles both raw TensorXf and regular arrays.
        arr = np.array(img, dtype=np.float32)
        # Ensure 3-channel output (drop alpha if present)
        if arr.ndim == 3 and arr.shape[2] > 3:
            arr = arr[:, :, :3]
        return arr
    except KeyError:
        raise
    except Exception as exc:
        raise RuntimeError(f"Render failed: {exc}") from exc


def render_all_channels(scene_id: int, spp: int = 16, seed: int = 0) -> np.ndarray:
    """Render a scene and return ALL channels (including AOVs).

    Unlike :func:`render`, this does not clip to 3 channels.  When the
    scene uses an ``aov`` integrator, the extra AOV channels are included
    in the returned array.

    Returns a float32 numpy array of shape ``(H, W, C)`` where *C* is
    the total number of output channels (3 for RGB plus AOV channels).
    """
    scene = _get_scene(scene_id)
    try:
        img = mi.render(scene, spp=spp, seed=seed)
        return np.array(img, dtype=np.float32)
    except KeyError:
        raise
    except Exception as exc:
        raise RuntimeError(f"Render failed: {exc}") from exc


def release(scene_id: int) -> bool:
    """Remove a scene from the registry.

    Returns ``True`` if the scene was found and removed, ``False`` otherwise.
    """
    cleanup_temp_files(scene_id)
    _invalidate_params(scene_id)
    return _SCENES.pop(scene_id, None) is not None


# ---------------------------------------------------------------------------
# Transient rendering (mitransient)
# ---------------------------------------------------------------------------

def ensure_mitransient() -> bool:
    """Import mitransient, registering its transient plugins with Mitsuba.

    Must be called after ``mi.set_variant()`` and before loading a scene
    that uses transient film or integrator types.

    Returns ``True`` if mitransient was successfully imported.

    Raises
    ------
    ImportError
        If mitransient is not installed.
    """
    global _MITRANSIENT_LOADED
    if _MITRANSIENT_LOADED:
        return True
    import mitransient  # noqa: F401 — registers plugins on import
    _MITRANSIENT_LOADED = True
    return True


# ---------------------------------------------------------------------------
# Custom raymap sensor
# ---------------------------------------------------------------------------

def ensure_raymap_sensor() -> bool:
    """Register the 'raymap' sensor plugin with Mitsuba (idempotent).

    Must be called after ``mi.set_variant()`` and before loading a scene
    that uses a raymap sensor.

    Returns ``True`` if registration succeeded.
    """
    global _RAYMAP_SENSOR_LOADED
    if _RAYMAP_SENSOR_LOADED:
        return True
    from .raymap_sensor import ensure_registered
    ensure_registered()
    _RAYMAP_SENSOR_LOADED = True
    return True


def register_raymap(origins, directions) -> int:
    """Store per-pixel ray arrays and return a raymap ID.

    Parameters
    ----------
    origins : array-like
        Ray origins, shape (H, W, 3).
    directions : array-like
        Ray directions, shape (H, W, 3). Will be normalized.

    Returns
    -------
    int
        Raymap ID for use in sensor construction.
    """
    from .raymap_sensor import register_raymap as _register
    origins = np.asarray(origins, dtype=np.float32)
    directions = np.asarray(directions, dtype=np.float32)
    return _register(origins, directions)


def update_raymap(raymap_id: int, origins=None, directions=None) -> bool:
    """Update ray arrays for an existing raymap.

    Parameters
    ----------
    raymap_id : int
        Existing raymap ID.
    origins : array-like, optional
        New origins array, shape (H, W, 3).
    directions : array-like, optional
        New directions array, shape (H, W, 3).

    Returns
    -------
    bool
        True on success.
    """
    from .raymap_sensor import update_raymap as _update
    if origins is not None:
        origins = np.asarray(origins, dtype=np.float32)
    if directions is not None:
        directions = np.asarray(directions, dtype=np.float32)
    return _update(raymap_id, origins, directions)


def release_raymap(raymap_id: int) -> bool:
    """Release a raymap from the registry."""
    from .raymap_sensor import release_raymap as _release
    return _release(raymap_id)


def render_transient(
    scene_id: int, spp: int = 16, seed: int = 0
) -> tuple[np.ndarray, np.ndarray]:
    """Render a transient scene, returning steady-state and transient images.

    For scenes with a ``transient_hdr_film``, Mitsuba returns a tuple of
    ``(steady_state, transient)``.  The steady-state image has shape
    ``(H, W, 3)`` and the transient data has shape ``(H, W, T, 3)`` where
    *T* is the number of temporal bins.

    Returns
    -------
    steady : np.ndarray
        Steady-state image, shape ``(H, W, 3)``.
    transient : np.ndarray
        Transient data, shape ``(H, W, T, 3)``.
    """
    ensure_mitransient()
    scene = _get_scene(scene_id)
    try:
        result = mi.render(scene, spp=spp, seed=seed)
        steady = np.array(result[0], dtype=np.float32)
        transient = np.array(result[1], dtype=np.float32)
        if steady.ndim == 3 and steady.shape[2] > 3:
            steady = steady[:, :, :3]
        return steady, transient
    except KeyError:
        raise
    except Exception as exc:
        raise RuntimeError(f"Transient render failed: {exc}") from exc


def py_eval(scene_id: int, code: str) -> bool:
    """Execute arbitrary Python code with access to a scene.

    The code is run with ``mi``, ``scene``, and ``np`` available in its
    namespace.

    Returns ``True`` on success.

    Raises
    ------
    KeyError
        If *scene_id* is not in the registry.
    RuntimeError
        If execution of *code* raises an exception.
    """
    scene = _get_scene(scene_id)
    try:
        exec(code, {"mi": mi, "scene": scene, "np": np})
        return True
    except Exception as exc:
        raise RuntimeError(str(exc)) from exc


def _is_matrix4x4(val):
    """Check if val looks like a 4x4 nested list."""
    if not isinstance(val, list) or len(val) != 4:
        return False
    return all(isinstance(row, list) and len(row) == 4 for row in val)


_TRANSIENT_TYPES = {"transient_hdr_film", "transient_path", "transient_nlos_path",
                     "transient_prbvolpath", "phasor_hdr_film"}


def _needs_mitransient(obj) -> bool:
    """Check if a normalized scene dict uses any mitransient plugin types."""
    if isinstance(obj, dict):
        if obj.get("type") in _TRANSIENT_TYPES:
            return True
        return any(_needs_mitransient(v) for v in obj.values())
    return False


def _needs_raymap_sensor(obj) -> bool:
    """Check if a scene dict uses the raymap sensor plugin."""
    if isinstance(obj, dict):
        if obj.get("type") == "raymap":
            return True
        return any(_needs_raymap_sensor(v) for v in obj.values())
    return False


def _normalize(obj, temp_files):
    """Recursively normalize a scene dict for mi.load_dict."""
    if isinstance(obj, dict):
        result = {}
        for key, value in obj.items():
            # Strip builder-only keys
            if key in ('category_', 'key_'):
                continue
            # Handle deferred raymap sensor data
            if key == 'raymap_data_':
                ensure_raymap_sensor()
                origins = np.asarray(value.get('origins'), dtype=np.float32)
                directions = np.asarray(value.get('directions'), dtype=np.float32)
                from .raymap_sensor import register_raymap as _reg_raymap
                # DOF parameters (optional)
                aperture_radius = value.get('aperture_radius', 0.0)
                if hasattr(aperture_radius, '__float__'):
                    aperture_radius = float(aperture_radius)
                focus_distance = value.get('focus_distance', None)
                if focus_distance is not None:
                    focus_distance = np.asarray(focus_distance, dtype=np.float32)
                    if focus_distance.ndim == 0:
                        focus_distance = float(focus_distance)
                vignetting = value.get('vignetting', None)
                if vignetting is not None:
                    vignetting = np.asarray(vignetting, dtype=np.float32)
                rid = _reg_raymap(origins, directions,
                                  aperture_radius=aperture_radius,
                                  focus_distance=focus_distance,
                                  vignetting=vignetting)
                result['raymap_id'] = rid
                continue
            # Handle deferred mesh data
            if key == 'mesh_data_':
                verts = value.get('vertices')
                faces = value.get('faces')
                if isinstance(verts, np.ndarray):
                    verts = verts.tolist()
                if isinstance(faces, np.ndarray):
                    faces = faces.tolist()
                # Compute smooth vertex normals unless face_normals requested
                normals = None
                if not obj.get('face_normals', False):
                    verts, faces, normals = _compute_smooth_normals(
                        verts, faces)
                    verts = verts.tolist() if isinstance(verts, np.ndarray) else verts
                    faces = faces.tolist() if isinstance(faces, np.ndarray) else faces
                    normals = normals.tolist() if isinstance(normals, np.ndarray) else normals
                fd, path = tempfile.mkstemp(suffix='.obj', prefix='matsuba_')
                os.close(fd)
                _write_obj(path, verts, faces, normals=normals)
                temp_files.append(path)
                result['filename'] = path
                continue
            # Recursively normalize the value
            nval = _normalize(value, temp_files)
            # Convert 4x4 matrices for transform keys
            if key in ('to_world', 'to_object') and _is_matrix4x4(nval):
                nval = mi.ScalarTransform4f(nval)
            result[key] = nval
        return result
    elif isinstance(obj, list):
        return [_normalize(item, temp_files) for item in obj]
    else:
        return obj


def _compute_smooth_normals(vertices, faces, angle_threshold_deg=45.0):
    """Compute smooth vertex normals with edge-splitting at sharp angles.

    Vertices shared by faces whose normals differ by more than
    *angle_threshold_deg* are duplicated so each side gets its own normal.
    Returns (new_vertices, new_faces_1indexed, new_normals).
    """
    from collections import defaultdict

    verts = np.array(vertices, dtype=np.float64)
    fcs = np.array(faces, dtype=np.int64) - 1  # 0-indexed

    # Pre-compute unit face normals
    v0s, v1s, v2s = verts[fcs[:, 0]], verts[fcs[:, 1]], verts[fcs[:, 2]]
    face_normals = np.cross(v1s - v0s, v2s - v0s)
    fn_lens = np.linalg.norm(face_normals, axis=1, keepdims=True)
    face_normals = face_normals / np.maximum(fn_lens, 1e-12)

    cos_thresh = np.cos(np.radians(angle_threshold_deg))

    # Build per-vertex face adjacency
    vert_faces = defaultdict(list)
    for fi in range(len(fcs)):
        for vi in fcs[fi]:
            vert_faces[int(vi)].append(fi)

    new_verts = list(verts)
    new_normals_list = [np.zeros(3)] * len(verts)
    new_fcs = fcs.copy()

    for vi, fi_list in vert_faces.items():
        if len(fi_list) == 1:
            new_normals_list[vi] = face_normals[fi_list[0]]
            continue

        # Cluster adjacent faces by normal similarity
        groups, assigned = [], set()
        for fi in fi_list:
            if fi in assigned:
                continue
            group = [fi]
            assigned.add(fi)
            fn_i = face_normals[fi]
            for fj in fi_list:
                if fj not in assigned and np.dot(fn_i, face_normals[fj]) >= cos_thresh:
                    group.append(fj)
                    assigned.add(fj)
            groups.append(group)

        for gi, group in enumerate(groups):
            avg_n = np.sum(face_normals[group], axis=0)
            length = np.linalg.norm(avg_n)
            avg_n = avg_n / length if length > 1e-12 else face_normals[group[0]]

            if gi == 0:
                new_normals_list[vi] = avg_n
            else:
                new_vi = len(new_verts)
                new_verts.append(verts[vi].copy())
                new_normals_list.append(avg_n)
                for fi in group:
                    for k in range(3):
                        if new_fcs[fi, k] == vi:
                            new_fcs[fi, k] = new_vi

    return np.array(new_verts), new_fcs + 1, np.array(new_normals_list)


def _write_obj(filepath, vertices, faces, normals=None):
    """Write a Wavefront OBJ file, optionally with vertex normals."""
    with open(filepath, 'w') as f:
        for v in vertices:
            f.write(f"v {v[0]} {v[1]} {v[2]}\n")
        if normals is not None:
            for n in normals:
                f.write(f"vn {n[0]} {n[1]} {n[2]}\n")
            for face in faces:
                i0, i1, i2 = int(face[0]), int(face[1]), int(face[2])
                f.write(f"f {i0}//{i0} {i1}//{i1} {i2}//{i2}\n")
        else:
            for face in faces:
                f.write(f"f {int(face[0])} {int(face[1])} {int(face[2])}\n")


def normalize_and_load(scene_dict: dict) -> int:
    """Normalize a MATLAB-originated scene dict and load it via mi.load_dict.

    Returns an integer scene ID.

    Raises
    ------
    RuntimeError
        If Mitsuba fails to load the normalized dict.
    """
    global _NEXT_ID
    temp_files = []
    normalized = _normalize(scene_dict, temp_files)
    # Auto-import mitransient if transient plugins are used
    if _needs_mitransient(normalized):
        ensure_mitransient()
    # Auto-register raymap sensor plugin if used
    if _needs_raymap_sensor(normalized):
        ensure_raymap_sensor()
    try:
        scene = mi.load_dict(normalized)
    except Exception as exc:
        # Clean up temp files on failure
        for f in temp_files:
            try:
                os.remove(f)
            except OSError:
                pass
        tb = traceback.format_exc()
        raise RuntimeError(
            f"Failed to load normalized scene dict: {exc}\n\n"
            f"Python traceback:\n{tb}"
        ) from exc
    sid = _NEXT_ID
    _SCENES[sid] = scene
    _NEXT_ID += 1
    if temp_files:
        _TEMP_FILES[sid] = temp_files
    return sid


def cleanup_temp_files(scene_id: int) -> bool:
    """Remove temporary files associated with a scene."""
    files = _TEMP_FILES.pop(scene_id, [])
    for f in files:
        try:
            os.remove(f)
        except OSError:
            pass
    return bool(files)


# ---------------------------------------------------------------------------
# Differentiable rendering
# ---------------------------------------------------------------------------

_LOSS_FUNCTIONS = {
    "mse": lambda image, ref: dr.mean(dr.square(image - ref)),
    "l1": lambda image, ref: dr.mean(dr.abs(image - ref)),
}


def get_param(scene_id: int, param_name: str):
    """Read a single scene parameter value as a numpy array.

    Returns a numpy array (e.g. shape ``(3,)`` for Color3f).

    Raises
    ------
    KeyError
        If *scene_id* or *param_name* is not found.
    """
    params = _get_params(scene_id)
    value = params[param_name]
    return np.array(value, dtype=np.float32).ravel()


def render_diff(
    scene_id: int,
    ref_image,
    param_names: list[str],
    loss_fn: str = "mse",
    spp: int = 4,
    seed: int = 0,
) -> tuple[np.ndarray, float, dict[str, np.ndarray]]:
    """Perform a differentiable render and return gradients.

    Renders the scene, computes the loss against *ref_image*, back-propagates
    through the rendering process, and extracts gradients for each parameter
    in *param_names*.

    Parameters
    ----------
    scene_id : int
        Registry ID of the scene.
    ref_image : array-like
        Reference image as a numpy array of shape ``(H, W, C)``.
    param_names : list[str]
        Parameter names to differentiate (e.g. ``["red.reflectance.value"]``).
    loss_fn : str
        Loss function name: ``"mse"`` or ``"l1"``.
    spp : int
        Samples per pixel for the differentiable render.
    seed : int
        Random seed for the sampler.

    Returns
    -------
    image : np.ndarray
        Rendered image as float32 array of shape ``(H, W, C)``.
    loss : float
        Scalar loss value.
    gradients : dict[str, np.ndarray]
        Mapping of parameter name to its gradient as a numpy array.

    Raises
    ------
    KeyError
        If *scene_id* is not in the registry.
    ValueError
        If *loss_fn* is not recognized.
    RuntimeError
        If the differentiable render fails.
    """
    if loss_fn not in _LOSS_FUNCTIONS:
        raise ValueError(
            f"Unknown loss function '{loss_fn}'. "
            f"Available: {sorted(_LOSS_FUNCTIONS.keys())}"
        )

    scene = _get_scene(scene_id)
    params = _get_params(scene_id)

    # Enable gradient tracking on requested parameters
    for name in param_names:
        dr.enable_grad(params[name])
    params.update()

    try:
        # Convert reference image to a Dr.Jit tensor
        if isinstance(ref_image, np.ndarray):
            ref_tensor = mi.TensorXf(ref_image)
        else:
            ref_tensor = mi.TensorXf(np.array(ref_image, dtype=np.float32))

        # Differentiable render
        image = mi.render(scene, params, spp=spp, seed=seed)

        # Compute loss
        loss = _LOSS_FUNCTIONS[loss_fn](image, ref_tensor)

        # Back-propagate
        dr.backward(loss)

        # Extract gradients as plain numpy arrays
        gradients = {}
        for name in param_names:
            grad = dr.grad(params[name])
            gradients[name] = np.array(grad, dtype=np.float32).ravel()

        # Convert image to numpy
        img_np = np.array(image, dtype=np.float32)
        if img_np.ndim == 3 and img_np.shape[2] > 3:
            img_np = img_np[:, :, :3]

        loss_val = float(loss.array[0])

        return img_np, loss_val, gradients

    except (KeyError, ValueError):
        raise
    except Exception as exc:
        tb = traceback.format_exc()
        raise RuntimeError(
            f"Differentiable render failed: {exc}\n\n"
            f"Python traceback:\n{tb}"
        ) from exc
    finally:
        # Always clean up the AD graph
        for name in param_names:
            dr.disable_grad(params[name])
        params.update()


def forward_grad(
    scene_id: int,
    param_name: str,
    spp: int = 128,
    seed: int = 0,
) -> np.ndarray:
    """Forward-mode differentiable render: visualize parameter sensitivity.

    Computes how the rendered image changes with respect to a single scene
    parameter.  Returns a gradient image of the same shape as the rendered
    image.

    Parameters
    ----------
    scene_id : int
        Registry ID of the scene.
    param_name : str
        The parameter to differentiate with respect to.
    spp : int
        Samples per pixel.
    seed : int
        Random seed for the sampler.

    Returns
    -------
    grad_image : np.ndarray
        Gradient image as float32 array of shape ``(H, W, C)``.

    Raises
    ------
    KeyError
        If *scene_id* or *param_name* is not found.
    RuntimeError
        If the forward-mode render fails.
    """
    scene = _get_scene(scene_id)
    params = _get_params(scene_id)

    # Enable gradient tracking and set unit gradient
    dr.enable_grad(params[param_name])
    params.update()

    try:
        # Render (records the AD graph)
        image = mi.render(scene, params, spp=spp, seed=seed)

        # Forward-propagate gradients from the parameter to the image
        dr.forward(params[param_name])
        grad_image = dr.grad(image)

        # Convert to numpy
        grad_np = np.array(grad_image, dtype=np.float32)
        if grad_np.ndim == 3 and grad_np.shape[2] > 3:
            grad_np = grad_np[:, :, :3]

        return grad_np

    except KeyError:
        raise
    except Exception as exc:
        raise RuntimeError(f"Forward-mode gradient failed: {exc}") from exc
    finally:
        # Always clean up the AD graph
        dr.disable_grad(params[param_name])
        params.update()

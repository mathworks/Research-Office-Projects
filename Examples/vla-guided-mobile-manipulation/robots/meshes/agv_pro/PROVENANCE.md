# myAGV Pro base geometry - provenance and derived values

## The meshes are not included

The mobile base is Elephant Robotics' **myAGV Pro**. Its six STL meshes are Elephant
Robotics' files and are **not redistributed** with this example. They only change how the
base looks in the render; without them `assembleMobileManipulator` draws it as primitives
of the same size. To use them, download them unmodified into this folder
(`robots/meshes/agv_pro/`); README.md, section **Robot Base Meshes**, has the commands.

| | |
|---|---|
| Repository | https://github.com/elephantrobotics/agv_pro_ros2 |
| Branch | `humble` |
| Package | `agv_pro_description`, version 1.0.8 |
| Files to download | `meshes/base_link.stl`, `laser_link.stl`, `wheel_lf_link.stl`, `wheel_lb_link.stl`, `wheel_rf_link.stl`, `wheel_rb_link.stl` |
| Licence | BSD-3-Clause, Copyright (c) 2024 Elephant Robotics (in their repository) |

## Values this example takes from them

No Elephant Robotics file is included. A few physical dimensions and masses of the myAGV Pro
were measured from their meshes or read from their `urdf/agv_pro.urdf`, and are typed into:

- the base dimensions, wheel radius, wheel positions and chassis mass in
  `configuration/hmmConfig.m` (section *Mobile base - myAGV Pro*);
- the base frames, the wheel-to-corner mapping and the collision box in
  `robots/assembleMobileManipulator.m`;
- the tables in this file.

On first setup, `hmmMirrorTree` writes y-reflected copies of the downloaded meshes to
`robots/meshes/mirrored/`, so that the Simulation 3D Robot block renders the base the right
way round. Those copies are generated on your machine and are not redistributed either
(`.gitignore` excludes them).

## Units: metres

Measured directly from the STL bounding boxes. The URDF declares no `scale`
attribute on any mesh, which is only correct if the files are already in metres, and
they are. This is worth stating because a SolidWorks-exported ROS mesh in millimetres
with a `scale 0.001` is the far more common case, and getting it wrong yields a robot
1000x too large with no error message.

## Measured geometry

Bounding boxes, in metres, from the STL triangle vertices:

| Mesh | Triangles | x | y | z | size |
|---|---|---|---|---|---|
| `base_link.stl` | 89064 | −0.2656 … 0.2651 | −0.1760 … 0.1750 | 0.0000 … 0.2200 | 0.531 x 0.351 x 0.220 |
| `laser_link.stl` | 18378 | −0.0373 … 0.0294 | −0.0332 … 0.0332 | −0.0114 … 0.0246 | 0.067 x 0.067 x 0.036 |
| `wheel_lf_link.stl` | 49006 | −0.0761 … 0.0761 | −0.0463 … 0.0000 | −0.0761 … 0.0761 | radius 0.0761, width 0.0463 |
| `wheel_lb_link.stl` | 47686 | −0.0761 … 0.0761 | 0.0000 … 0.0463 | −0.0761 … 0.0761 | radius 0.0761, width 0.0463 |
| `wheel_rf_link.stl` | 47686 | −0.0760 … 0.0762 | −0.0463 … 0.0000 | −0.0760 … 0.0762 | radius 0.0761, width 0.0463 |
| `wheel_rb_link.stl` | 49006 | −0.0761 … 0.0761 | 0.0000 … 0.0463 | −0.0761 … 0.0761 | radius 0.0761, width 0.0463 |

`base_link.stl` rests on z = 0, so the `base_link` origin is at the **underside of
the chassis shell**, not at its centre and not on the floor.

## Frames, from the URDF

Wheel joint origins on `base_link`, axis `[0 1 0]`, type `continuous`:

| Vendor link | Origin xyz |
|---|---|
| `left_front_wheel_link` | 0.17128, 0.1799, 0.052 |
| `left_rear_wheel_link` | 0.17128, −0.1799, 0.052 |
| `right_front_wheel_link` | −0.17181, 0.1799, 0.051884 |
| `right_rear_wheel_link` | −0.171806, −0.1799, 0.0518837 |

Fixed sensor frames on `base_link`: `laser_link` at `[0.17891 0 0.20928]`,
`camera_link` at `[0.23191 0 0.14928]`, `imu_link` at `[−0.17181 −0.0270532 0.14928]`
with rpy `[0 0 1.5707]`. `base_joint` is fixed and puts `base_link` 0.020 above
`base_footprint`.

Vendor inertial mass, `base_link`: 19.236 kg, centre of mass
`[−0.0076254 −0.00023134 0.06693]`.

## THE VENDOR WHEEL NAMES ARE MISLABELLED. DO NOT TRUST THEM.

`left_front` and `left_rear` share `x = +0.17128` and differ only in the sign of `y`.
So in this URDF "left/right" selects **±X** and "front/rear" selects **±Y**. But the
lidar sits at `x = +0.179` and the camera at `x = +0.232`, and sensors point forward,
so forward is **+X** — under which "left" and "right" would have to be ±Y. The naming
is self-contradictory, and a corner assignment read off the names is wrong.

The corner mapping used by `assembleMobileManipulator` was therefore **measured, not
read**. Each wheel mesh is offset to one side of its own origin, spanning either
`y ∈ [−0.0463, 0]` or `y ∈ [0, +0.0463]`, and the tyre must sit **inboard** of its
mount. That fixes the sign of `y` for every mesh independently of both its filename
and its joint origin, and all four agree with the joint origins:

| Our body | Corner (xSign, ySign) | Mesh | Mesh spans y | Inboard of |
|---|---|---|---|---|
| `wheel_fl` | (+, +) | `wheel_lf_link.stl` | −0.0463 … 0 | a +y mount |
| `wheel_fr` | (+, −) | `wheel_lb_link.stl` | 0 … +0.0463 | a −y mount |
| `wheel_rl` | (−, +) | `wheel_rf_link.stl` | −0.0463 … 0 | a +y mount |
| `wheel_rr` | (−, −) | `wheel_rb_link.stl` | 0 … +0.0463 | a −y mount |

Two independent facts — the joint origin and the mesh asymmetry — agree on all four
corners, which is what makes this a measurement rather than a guess.

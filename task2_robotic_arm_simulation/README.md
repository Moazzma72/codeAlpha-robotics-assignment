# 4-DOF Robotic Arm – CAD Model and Pick-and-Place Simulation

Built with **CadQuery** (open-source parametric CAD kernel) and **Python**. The CAD files are standard formats
(STEP / STL / DXF / URDF), so they open in SolidWorks, AutoCAD, Fusion 360, FreeCAD, Blender, ROS/RViz and PyBullet.
All CAD geometry is in **millimetres**.

## 1. Degrees of freedom

| Joint | Name | Type | Axis | Range | Function |
|---|---|---|---|---|---|
| J1 | Base yaw | Revolute | Z (vertical) | -170° to +170° | Swings the arm toward the target |
| J2 | Shoulder | Revolute | Y (horizontal) | -10° to +170° | Raises/lowers the upper arm |
| J3 | Elbow | Revolute | Y | -150° to 0° | Extends/folds the forearm |
| J4 | Wrist pitch | Revolute | Y | -120° to +120° | Keeps the gripper pointing down |
| G1 | Gripper | Prismatic (two coupled fingers) | Y | 0 – 15 mm per finger | Opens/closes to grip the 50 mm cube |

* **Arm DOF = 4** (all revolute); the gripper adds 1 end-effector DOF (**5 actuated joints in total**).
* A 4-DOF arm can place the tool at any (x, y, z) in reach and set its pitch (tool pointing straight down),
  but cannot set roll or yaw independently — enough for top-down pick-and-place.
* Link lengths: D1 = 150 mm (base to shoulder), L2 = 300 mm, L3 = 250 mm, L4 = 100 mm (wrist to tool centre point).
  Maximum reach from the shoulder axis = L2 + L3 = 550 mm.

**Forward kinematics** (planar angles from horizontal: a1 = q2, a2 = q2+q3, a3 = q2+q3+q4; r = horizontal reach):

    r = L2 cos(a1) + L3 cos(a2) + L4 cos(a3)
    z = D1 + L2 sin(a1) + L3 sin(a2) + L4 sin(a3)
    x = r cos(q1),  y = r sin(q1)

**Inverse kinematics** (tool pointing down, elbow-up, closed form): q1 = atan2(y, x); wrist point = (r, z + L4 − D1);
cos(q3) = (r² + wz² − L2² − L3²) / (2·L2·L3), q3 = −acos(·); q2 = atan2(wz, r) − atan2(L3 sin q3, L2 + L3 cos q3);
q4 = −90° − q2 − q3. Every waypoint is verified by running FK on the IK result (error < 1e-6 m).

## 2. Pick-and-place simulation (10.4 s, 20 Hz)

Home → above pick → descend → close gripper → lift → transfer → lower → open gripper → retreat → home.
Pick cube at (300, 200, 25) mm, place at (−100, 350, 25) mm. Straight-line Cartesian paths with minimum-jerk
(smooth start/stop) timing. All joint angles stay inside the limits above (asserted in code).
Joint ranges used: q1 0–106°, q2 39–99°, q3 −126 to −89°, q4 −67 to −31°.

## 3. Files

| Folder | File | Contents |
|---|---|---|
| cad/ | `robotic_arm_assembly.step` | Coloured multi-body assembly, ready pose (open in SolidWorks/AutoCAD/Fusion) |
| cad/ | `robotic_arm_assembly.stl` | Same assembly as a mesh |
| cad/ | `base / turret / upper_arm / forearm / wrist_gripper / finger` `.step` + `.stl` | Individual parts |
| cad/ | `robotic_arm_2D_drawing.dxf` | 2D side view + top view with dimensions (AutoCAD) |
| cad/ | `robot_arm.urdf` | Kinematic model with joint limits (ROS, PyBullet, Gazebo, RViz) |
| simulation/ | `pick_and_place.gif` | Animated simulation |
| simulation/ | `trajectory.csv` | Time, joint angles, gripper travel and TCP position for every step |
| simulation/ | `arm_sim.py`, `make_extras.py` | Source: CAD generation, kinematics, simulation, rendering |
| screenshots/ | `01…16 .png` | Key poses, orthographic views, joint plots, TCP path, 2D drawing, kinematic diagram |

Re-run: `cd simulation && python arm_sim.py && python make_extras.py`.

## 4. Using it in SolidWorks / AutoCAD

* **SolidWorks:** File > Open > `robotic_arm_assembly.step`. STEP carries geometry, not mates or motion, so add
  revolute (concentric) mates at J1–J4 and a Motion Study with rotary motors if you want the animation inside SolidWorks;
  `trajectory.csv` gives the joint angle for each time step to use as motor data.
* **AutoCAD:** `OPEN` the `.dxf` for the 2D drawing, or `IMPORT` a `.step` for 3D.
* **Simulators:** load `robot_arm.urdf` in PyBullet (`p.loadURDF`) or RViz; keep the STL files in the same folder.

## 5. Notes and limitations

The simulation is kinematic (positions over time); it does not model dynamics, torques or collisions.
The cube grasp is a simple attach/release at the closed/open gripper moments. The URDF masses and inertias are placeholders.

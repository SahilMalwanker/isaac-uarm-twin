# Engineering Design Log

How the real-time digital twin was built, what failed in early prototypes, and why the final architecture uses a 1 Hz non-blocking HTTP telemetry tap paired with a 60 Hz PhysX PD follower inside NVIDIA Isaac Sim.

---

## 1 · Why a Simple Serial URDF Import Was Not Enough

The uArm Swift Pro is a **parallel-linkage desktop robot**:

1. `Joint1` rotates the base turret around $+Z$.
2. `Joint2` drives the main lower arm (`Link2`) around $+Y$.
3. `Joint6` (coaxial with `Joint2`) drives a short crank (`Link6`), which pulls the forearm (`Link3`) through a vertical drive rod (`Link7`).
4. A rear parallelogram (`Link5`–`Link4`) and a front parallelogram (`Link9`–`Link4`) keep the tool head (`Link8`) horizontal throughout the workspace.

When the upstream URDF (`pro_model.urdf`) is imported into Isaac Sim as an articulation tree:

- **PhysX articulations must be trees**, so the URDF leaves the three mechanical loops open. Without closing those loops, `Link4`, `Link5`, `Link7`, `Link8` and `Link9` hang loose or require artificial joint drives.
- **Mesh-to-mesh self-collisions lock the linkage.** Full STL collision meshes on adjacent parallel links bind against each other at the pin joints.

### Solution (`isaac/tools/rig_swiftpro.py`)

- Replace mesh colliders in the URDFs with clean primitive cylinders and boxes on the main links (`Base`, `Link1`, `Link2`, `Link3`, `Link8`, `GripperExt`) and disable articulation self-collision (`physxArticulation:enabledSelfCollisions = False`).
- Author **PD angular force drives** only on the actuated joints (`Joint1`, `Joint2`, `Joint6`, and `JointGripperExt` on arm 2).
- Remove `UsdPhysics.DriveAPI` from the six passive joints (`Joint3`, `Joint4`, `Joint5`, `Joint7`, `Joint8`, `Joint9`) and set joint friction to `0.1`.
- Close the three kinematic loops with `UsdPhysics.RevoluteJoint` prims (`Loop_Link5_Link4`, `Loop_Link7_Link3`, `Loop_Link9_Link4`) with **`physics:excludeFromArticulation = True`**, and raise the articulation solver to **32 position / 1 velocity iterations**.

Checked against the recorded PhysX passive joint states at `Joint2 = 14.798°`, `Joint6 = 36.026°`, the analytical loop closure in `isaac_uarm_twin/kinematics.py` agrees with PhysX within **0.42°** across all six passive joints (`Joint3` through `Joint9`).

---

## 2 · Evolution of the Real-Time Plant-to-Twin Bridge

Five approaches were prototyped before arriving at the final architecture:

| Stage | Architecture | What Happened on the Cell |
| :--- | :--- | :--- |
| **Prototype 1: ROS 2 Humble + WSL rosbridge** | Windows `SwiftAPI` node published `/RobotN/joint_states` at 20 Hz -> WSL 2 `rosbridge_websocket` (`:9090`, `ROS_DOMAIN_ID=42`) -> Isaac Sim Script Editor subscriber | Worked for single-arm tests, but required WSL networking and `rclpy` DLL workarounds on Windows. Early URDF drives (`stiffness ≈ 2.27`) sagged under gravity until stiffness was scaled up. When a second script tried to command the arm, two processes fought over the serial port. |
| **Prototype 2: TCP / UDP socket bridges & cuMotion** | Custom TCP (`:5555`) and UDP (`:5000`) broadcasters; Lula / cuMotion `cuMotion.xrdf` c-space config (`Joint1`, `Joint2`, `Joint6`) | Useful for offline motion-planning experiments, but the real plant is already driven by the PLC and the robot controller's Cartesian waypoints. |
| **Prototype 3: HTTP v1 — sample after each move** | Robot script sampled `get_servo_angle()` only between blocking `set_position(..., wait=True)` calls | The twin jumped from waypoint to waypoint after each move finished instead of moving continuously. |
| **Prototype 4: HTTP v2/v3 — async polling + arm-2 wrist** | Background thread polled `get_servo_angle(wait=False, callback=...)`; arm 2 added wrist tracking on a separate port | Smooth continuous motion on arms 1 and 3, but running three separate servers (`:8001`, `:8002`, `:8003`) complicated startup and port ownership. |
| **Final (v4/v5): Unified `ServoTelemetry` + 60 Hz PhysX follower** | Single FastAPI router (`GET /api/servo_angles` on `:8001`) attached directly to the process that owns the serial ports, polled by `isaac/follower.py` inside Isaac Sim | Zero serial contention, one endpoint for all three arms, non-blocking async callbacks on arms 1 & 3, and software wrist tracking on arm 2. |

---

## 3 · Why Arms 1 & 3 Use Async Callbacks While Arm 2 Uses Sync + Tracked Wrist

1. **Serial link contention (`_pending` guard):**
   During `set_position(..., wait=True)`, the uArm SDK's serial write thread is busy streaming G-code moves. Firing unbounded `get_servo_angle()` requests floods the command queue (`cmd_pend_size=2`). `ServoTelemetry.request_async()` keeps a per-arm `_pending` lock so at most **one** angle query is in flight at any time.
2. **Firmware wrist reporting limitation:**
   On the uArm Swift Pro, `get_servo_angle()` returns the three main joints (`[base, shoulder, elbow]`), not the 4th hobby servo on the end-effector wrist. `ServoTelemetry` wraps `arm.set_wrist(angle)` so every commanded wrist angle is recorded in software and appended as the 4th element (`[s0, s1, s2, wrist]`) for arm 2.
3. **E21 / E22 firmware errors at boundary coordinates:**
   When a Cartesian waypoint lies outside the Swift Pro workspace, the firmware replies `E21` (point out of range) or `E22` and does not move. All waypoints in `isaac_uarm_twin/sequences.py` are verified reachable through `kinematics.motor_angles()`.

---

## 4 · Two-Point Calibration & 60 Hz Smoothing (`isaac/follower.py`)

Raw servo angles reported by `get_servo_angle()` do not share the same zero or sign convention as the URDF joint axes:

- **Shoulder (`Joint2`) is inverted:** extending the arm forward lowers the physical servo reading (`-6.44°`) while increasing the URDF joint angle (`+89.954°`).
- **Wrist (`JointGripperExt`) is centered at 90°:** `180°` maps to `+90°` and `0°` maps to `-90°`.

Using `python -m isaac_uarm_twin.calibrate`, each arm's servos were detached (`set_servo_detach`), posed at their physical extended and retracted stops, and captured (`POST /api/capture`). `follower.py` maps each sample through:

$$\theta_{\text{isaac}} = \text{clamp}\!\left(y_0 + \frac{s - x_0}{x_1 - x_0}(y_1 - y_0),\; \theta_{\min},\; \theta_{\max}\right)$$

followed on every 60 Hz physics step by:

1. **Slew-rate limit** (`360°/s` base, `220°/s` shoulder, `260°/s` crank, `200°/s` wrist) so a large step never snaps the articulation;
2. **Exponential moving average** ($\alpha = 0.28$) to turn the 1 Hz telemetry staircase into a smooth trajectory;
3. **Deadband** (`0.05°`) to avoid waking the solver for sub-tenth-degree noise;
4. **Per-step wake-up** (`wake_up_articulation`) so PhysX never puts an idle arm to sleep mid-cycle and then jumps on the next sample.

---

## 5 · Conveyors, Light Barriers and "Pick-as-Is" Vacuum / Gripper Tools

- **Conveyors without moving belt meshes (`isaac/graphs/conveyor_controller.py`):**
  Both belts are static `Cube` colliders with `UsdPhysics.RigidBodyAPI(kinematicEnabled=True)` and `PhysxSchema.PhysxSurfaceVelocityAPI` (`±0.15 m/s` along $X$). Parts resting on the belt are carried by contact friction while the belt geometry stays stationary.
- **Light barriers (`S1`–`S4`) via PhysX sphere sweeps:**
  Every simulation tick, `get_physx_scene_query_interface().sweep_sphere_all` sweeps a `0.5 mm` sphere across the belt width along $-Y$. Belt 1 latches on when `S1` sees the holder and stops when `S2` sees it; Belt 2 latches on at `S3` and stops at `S4`—mirroring the PLC start/stop latch on the physical line.
- **Why "pick-as-is" fixed joints replaced snap-to-grip (`isaac/graphs/vacuum_autograb.py`):**
  PhysX has no native vacuum suction, and standard surface grippers or zeroed fixed joints caused parts to snap or yaw-jump when picked up slightly off-center. The custom `vacuum_autograb.py` script node computes both bodies' local anchors from the same world-space tool point (`m_hold.GetInverse().Transform(tool_world)` and `m_part.GetInverse().Transform(tool_world)`) and preserves their exact relative quaternion at the moment of capture. Releasing disables and removes the joint and applies a small downward velocity (`-0.005 m/s` on the holder, `-0.02 m/s` on the PCB) so the part separates cleanly.

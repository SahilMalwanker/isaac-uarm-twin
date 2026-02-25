# Physics, Calibration and Verification

## Joint Model

The URDF importer produces a tree. The rig adds three excluded-from-articulation revolute constraints to close the mechanical loops:

| Constraint | Bodies | Role |
| :--- | :--- | :--- |
| `Loop_Link5_Link4` | Link4, Link5 | Rear parallelogram |
| `Loop_Link7_Link3` | Link3, Link7 | Elbow drive rod |
| `Loop_Link9_Link4` | Link4, Link9 | Tool-level linkage |

`Joint3`, `Joint4`, `Joint5`, `Joint7`, `Joint8` and `Joint9` are passive. Their drive APIs are removed. Local anchor coordinates come from the public robot geometry and linkage reconstruction, not proprietary cell CAD.

| Driven joint | Limits, degrees | Stiffness | Damping | Max force |
| :--- | :--- | ---: | ---: | ---: |
| Joint1 | -90 to 90 | 10,000 | 1,000 | 10,000,000 |
| Joint2 | -29.972 to 89.954 | 10,000 | 1,000 | 10,000,000 |
| Joint6 | -17 to 110 | 2,000 | 200 | 1,000,000 |
| JointGripperExt | -90 to 90 | 20,000 | 5,000 | 100,000 |

These are reconstructed simulation tuning values, not physical actuator ratings. They are intentionally strong and need retuning/validation in a metre-scale rebuild with the imported inertias. Do not use them to size hardware or estimate real joint torque.

## Following Telemetry

For each driven joint, two measured servo anchors map to the corresponding simulator angles. Input and output are clamped; each physics step applies a rate limit, followed by exponential smoothing with alpha 0.28. A 0.05-degree deadband avoids repeated tiny target writes. Targets are converted to radians for dynamic control.

The first sample is applied without a prior-state ramp. During communication loss, the follower retains its last sample. Neither behaviour should be mistaken for a physical safety control. Higher HTTP polling rates cannot compensate for the default 1 Hz measurement rate.

To calibrate, stop the ordinary controller, support the arm, and start:

```bash
python -m isaac_uarm_twin.calibrate --arm arm1
```

The local service on port 8002 provides `POST /api/servos/detach`, `POST /api/capture` and `POST /api/servos/attach`; individual servo routes accept IDs 0-3. Capture two supported poses per driven joint, compare with the imported model and update `CALIBRATION` in the follower. Detaching servos can let the arm fall under gravity.

## Contact and Transport

Belts are kinematic rigid bodies with collision and surface-velocity APIs. Their geometry does not translate. A 0.15 m/s tangential surface velocity moves contacting parts. Sphere sweeps at the four sensor origins decide when to enable or disable each belt.

Parts have 0.1 kg mass and simplified collision shapes. The pickup graph creates a fixed joint when the tool and grip marker are close, then removes it on release. Suction uses a 10 mm capture threshold; the recreated wrist gripper uses 15 mm. This is a proximity attachment approximation, not seal simulation or frictional finger grasping. Tool toggles are manual and separate from HTTP servo telemetry.

## Evidence Boundaries

- Hardware-free tests check known servo anchors, joint-limit mappings, filter response and sequence calls.
- Linkage tests compare reconstructed passive angles with saved reference values, with a 0.5-degree tolerance. They do not run PhysX.
- Firmware IK followed by the linkage model reaches the included SDK waypoints within the tested 2.5 mm tolerance. This is model consistency, not measured hardware accuracy.
- Scene tests check structure and intended placement, not collision-free trajectories or runtime schema compatibility.
- The new scene must still be checked inside Isaac Sim for articulation-root discovery, contact stability, loop constraint errors, grip frame alignment, reset/reload behaviour and real telemetry following.

The shaded README animation is an illustrative kinematic replay with accelerated timing and visual hand-offs. It must not be used as evidence that the physical or simulated plant completed that cycle.
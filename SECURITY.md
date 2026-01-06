# Security and Safety

`isaac-uarm-twin` is a research and demonstration project for a three-arm lab assembly cell. Read this before connecting it to physical robots.

## Machine Safety

- **The digital twin follower is not a safety controller.** It mirrors servo telemetry from the robot PC into NVIDIA Isaac Sim for online visualization and physics replay. It has no watchdog, collision-stop interlock or emergency-stop output back to the plant.
- **Standalone motion sequences move real hardware.** `python -m isaac_uarm_twin.cell_demo` runs the three-arm waypoint sequences unless `TWIN_RUN_MOTION=0` is set. Those waypoints are tuned for a specific cell layout. Keep hands clear, verify clearances and keep the hard-wired emergency stop active whenever the arms are powered.
- **Servo detachment drops the arm under gravity.** The calibration helper (`python -m isaac_uarm_twin.calibrate`) exposes `POST /api/servos/detach`, which removes holding torque on the selected servos so the arm can be posed by hand. Support the arm physically before detaching any servo.

## Network Exposure

- **No authentication is enabled on the HTTP services.** Both the telemetry service (`127.0.0.1:8001` by default) and the calibration helper (`127.0.0.1:8002` by default) bind to localhost (`TWIN_HOST=127.0.0.1`).
- Keep `TWIN_HOST=127.0.0.1` when Isaac Sim runs on the same workstation. If you bind to a LAN interface to reach a separate simulation workstation, restrict ports `8001` and `8002` to an isolated machine network.
- Only one process can own each USB serial port (`COM3`, `COM5`, `COM4`). When the companion [TIA S7 Copilot](../../../tia-s7-copilot) robot controller is running, attach `ServoTelemetry` inside that process rather than starting `cell_demo` alongside it.

## Secrets and Data Hygiene

- Configuration is read from environment variables or a local `.env` file, which is git-ignored. Commit only `.env.example`.
- The telemetry and calibration services log only the arm name, port and firmware version—full device info payloads (which include hardware serial numbers) are never printed or served over HTTP.
- No proprietary plant CAD, factory USD stages or internal network addresses are stored in this repository.

## Reporting a Vulnerability

Where private vulnerability reporting is enabled on GitHub (*Security → Advisories → Report a vulnerability*), use that channel. Otherwise open an issue requesting a private contact without posting credentials, internal network details or machine data.

# Connecting the Real Plant

## One Serial Owner

The controller that already moves the physical robots should also provide telemetry. Do not start `cell_demo` alongside it: both processes would compete for the USB serial ports.

Inside the existing FastAPI robot-controller process, after its arm objects and application exist:

```python
from isaac_uarm_twin.telemetry import ServoTelemetry

twin = ServoTelemetry({"arm1": arm_1, "arm2": arm_2, "arm3": arm_3}, rate_hz=1.0)
twin.attach(app)
```

Add the repository root to that process's Python import path or place it alongside the controller. Call `twin.stop()` in the controller's existing shutdown lifecycle. Check that `/api/state` and `/api/servo_angles` are not already defined: do not register duplicate routes. The telemetry helper does not replace motion control, PLC logic or chat functionality.

## Payload

`GET /api/servo_angles` returns degrees:

```json
{
  "arm1": [90.0, 45.0, 60.0],
  "arm2": [80.0, 50.0, 70.0, 90.0],
  "arm3": [120.0, 30.0, 40.0]
}
```

An arm is `null` until its first usable sample. The fourth arm-2 value is the last commanded wrist angle, tracked by wrapping `set_wrist`; it is not encoder feedback. `GET /api/state` includes each arm's sampling timestamp. The compact servo endpoint has no per-arm timestamps.

Arms 1 and 3 permit only one outstanding asynchronous query. Arm 2 queries its three main servos sequentially. An unreadable individual servo currently falls back to 90 degrees. This can create a misleading pose; treat communication errors as invalid operational evidence, not a safe state.

## Starting the Twin

Set `TWIN_TELEMETRY_URL` in the environment used to start the simulator. The default is local-only:

```powershell
$env:TWIN_TELEMETRY_URL = "http://127.0.0.1:8001/api/servo_angles"
python -m isaac_uarm_twin.launcher
```

The launcher waits for an HTTP response, starts Isaac Sim and asks `isaac/bootstrap.py` to open the prepared cell and load the follower. Keep the repository path free of spaces for the current `--exec` argument handling. Isaac Sim's own environment must provide the modules used by the follower, including `requests`.

The bootstrap aborts without pressing Play if stage opening or follower loading fails. During physics steps, the follower retries unresolved arms until their articulation and required joints are available; connected arms keep their existing filter state. After replacing the stage, rerun the bootstrap or follower.

The bootstrap and dynamic-control paths require validation against the installed Isaac Sim release. If automatic launch fails, open the stage manually, run the follower in the Script Editor and inspect its articulation report before playing.

## Scope and Security

Only robot joint motion follows the physical plant automatically. Belt sensor sweeps and tool toggles are local simulation logic. PLC I/O, vacuum state, gripper state, timestamps and safety status are not a complete synchronized plant-state feed in this implementation.

HTTP services bind to `127.0.0.1` by default and have no authentication. Do not expose them publicly. The calibration service can detach servos; use it only in a controlled local environment with the arm supported. Existing hardware guards, emergency stops and controller interlocks remain authoritative.
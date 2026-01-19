"""Calibration helper: let one arm go limp, pose it by hand and capture its servo angles.

    python -m isaac_uarm_twin.calibrate --arm arm1

The joint calibration in ``isaac/follower.py`` maps two measured servo angles per joint (arm fully
extended and fully retracted) onto the Isaac Sim joint angles that show the same pose. This service
produces the servo side of those pairs (see docs/calibration.md):

    POST /api/servos/detach               base, shoulder and elbow go limp (the wrist stays attached)
    POST /api/servos/detach/{servo_id}    one servo: 0 base, 1 shoulder, 2 elbow, 3 wrist
    POST /api/servos/attach               base, shoulder and elbow hold again
    POST /api/servos/attach/{servo_id}
    POST /api/capture                     sample servo angles and the Cartesian position
    GET  /api/state                       last capture
"""

import argparse
import logging
import threading
import time

from fastapi import FastAPI, Path

from . import settings

log = logging.getLogger("isaac_uarm_twin.calibrate")

SERVO_BASE, SERVO_SHOULDER, SERVO_ELBOW, SERVO_WRIST = 0, 1, 2, 3  # uarm.swift.protocol ids
MAIN_SERVOS = (SERVO_BASE, SERVO_SHOULDER, SERVO_ELBOW)


class CaptureSession:
    """Servo attach/detach and pose capture for one arm."""

    def __init__(self, arm):
        self.arm = arm
        self._lock = threading.Lock()
        self._latest = {"timestamp": 0.0, "servo_deg": None, "position_mm": None}

    def detach(self, servo_ids=MAIN_SERVOS):
        results = []
        for servo_id in servo_ids:
            try:
                result = self.arm.set_servo_detach(servo_id=servo_id, wait=True)
            except Exception as exc:  # noqa: BLE001
                log.warning("detach servo %d failed: %s", servo_id, exc)
                result = None
            log.info("detach servo %d: %s", servo_id, result)
            results.append(result == "OK")
        return all(results)

    def attach(self, servo_ids=MAIN_SERVOS):
        results = []
        for servo_id in servo_ids:
            try:
                result = self.arm.set_servo_attach(servo_id=servo_id, wait=True)
            except Exception as exc:  # noqa: BLE001
                log.warning("attach servo %d failed: %s", servo_id, exc)
                result = None
            log.info("attach servo %d: %s", servo_id, result)
            results.append(result == "OK")
        return all(results)

    def capture(self):
        servo = self.arm.get_servo_angle()
        position = self.arm.get_position()
        values = list(servo) if isinstance(servo, (list, tuple)) else []
        with self._lock:
            self._latest = {
                "timestamp": time.time(),
                "servo_deg": {
                    "base": values[0] if len(values) > 0 else None,
                    "shoulder": values[1] if len(values) > 1 else None,
                    "elbow": values[2] if len(values) > 2 else None,
                },
                "position_mm": position,
            }
            return dict(self._latest)

    def state(self):
        with self._lock:
            return dict(self._latest)


def create_app(session):
    app = FastAPI(title="Isaac uArm Twin - calibration helper", version="1.0")

    @app.post("/api/servos/detach")
    def detach_all():
        return {"success": session.detach()}

    @app.post("/api/servos/detach/{servo_id}")
    def detach_one(servo_id: int = Path(..., ge=0, le=3, description="0 base, 1 shoulder, 2 elbow, 3 wrist")):
        return {"success": session.detach((servo_id,)), "servo_id": servo_id}

    @app.post("/api/servos/attach")
    def attach_all():
        return {"success": session.attach()}

    @app.post("/api/servos/attach/{servo_id}")
    def attach_one(servo_id: int = Path(..., ge=0, le=3, description="0 base, 1 shoulder, 2 elbow, 3 wrist")):
        return {"success": session.attach((servo_id,)), "servo_id": servo_id}

    @app.post("/api/capture")
    def capture():
        return session.capture()

    @app.get("/api/state")
    def state():
        return session.state()

    return app


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--arm", choices=sorted(settings.ARM_PORTS), default="arm1", help="arm to calibrate")
    parser.add_argument("--port", help="serial port (overrides the arm's configured port)")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    from uarm.wrapper import SwiftAPI
    import uvicorn

    port = args.port or settings.ARM_PORTS[args.arm]
    arm = SwiftAPI(port=port)
    arm.waiting_ready()
    firmware = (arm.get_device_info() or {}).get("firmware_version")
    log.info("%s ready on %s (firmware %s)", args.arm, port, firmware)
    if firmware and not firmware.startswith(("0.", "1.", "2.", "3.")):
        arm.set_speed_factor(0.00001)
    uvicorn.run(create_app(CaptureSession(arm)), host=settings.HOST, port=settings.CALIBRATION_PORT)


if __name__ == "__main__":
    main()

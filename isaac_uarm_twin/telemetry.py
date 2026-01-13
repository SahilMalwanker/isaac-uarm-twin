"""Servo-angle telemetry for the Isaac Sim twin.

One sampling thread per arm keeps the latest servo angles in memory; ``GET /api/servo_angles``
serves them to the follower inside Isaac Sim. Two sampling styles are used, as on the cell:

* arms 1 and 3 ask for all servo angles asynchronously and never queue more than one request,
  so telemetry cannot build a backlog on the serial link shared with motion commands;
* arm 2 reads servos 0-2 one by one and appends its wrist angle. The firmware does not report the
  wrist servo, so the last commanded ``set_wrist`` angle is tracked in software.

The service can run standalone (``isaac_uarm_twin.cell_demo``) or be attached to the FastAPI app of
the TIA S7 Copilot robot controller, which owns the serial ports:

    twin = ServoTelemetry({"arm1": arm_1, "arm2": arm_2, "arm3": arm_3})
    twin.attach(app)
"""

import functools
import logging
import threading
import time

from fastapi import APIRouter

log = logging.getLogger("isaac_uarm_twin.telemetry")

WRIST_SERVOS = 3  # servos read individually on the wrist arm before the tracked wrist angle
FALLBACK_SERVO_DEG = 90.0  # used for a servo that does not answer


def parse_servo_result(res):
    """Normalise the SDK callback payload (list, tuple or dict, depending on SDK version)."""
    if res is None:
        return None
    if isinstance(res, (list, tuple)):
        return list(res)
    if isinstance(res, dict):
        for key in ("servo_deg", "data", "result"):
            value = res.get(key)
            if isinstance(value, (list, tuple)):
                return list(value)
    return None


class ServoTelemetry:
    """Samples servo angles of several uArm Swift Pro arms and serves them over HTTP."""

    def __init__(self, arms, wrist_arm="arm2", rate_hz=1.0, wrist_home=90.0):
        if rate_hz <= 0:
            raise ValueError("rate_hz must be positive")
        self.arms = dict(arms)
        self.wrist_arm = wrist_arm if wrist_arm in self.arms else None
        self.period = 1.0 / float(rate_hz)
        self._latest = {name: {"timestamp": 0.0, "servo_deg": None, "position_mm": None} for name in self.arms}
        self._locks = {name: threading.Lock() for name in self.arms}
        self._pending = {name: False for name in self.arms}
        self._pending_locks = {name: threading.Lock() for name in self.arms}
        self._stop = threading.Event()
        self._threads = []
        self._wrist_angle = float(wrist_home)
        if self.wrist_arm:
            self._latest[self.wrist_arm]["wrist_angle"] = None
            self._track_wrist(self.arms[self.wrist_arm])
        self.router = self._build_router()

    # -- wrist tracking ----------------------------------------------------
    def _track_wrist(self, arm):
        original = arm.set_wrist

        @functools.wraps(original)
        def set_wrist(angle=90, *args, **kwargs):
            self._wrist_angle = float(angle)
            return original(angle, *args, **kwargs)

        arm.set_wrist = set_wrist

    @property
    def wrist_angle(self):
        return self._wrist_angle

    # -- sampling ----------------------------------------------------------
    def start(self):
        """Start one daemon sampling thread per arm (idempotent)."""
        if self._threads:
            return
        for name in self.arms:
            target = self._sample_sync_loop if name == self.wrist_arm else self._sample_async_loop
            thread = threading.Thread(target=target, args=(name,), name=f"telemetry-{name}", daemon=True)
            thread.start()
            self._threads.append(thread)

    def stop(self):
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=2.0)
        self._threads = []

    def _sleep_rest(self, t0):
        rest = self.period - (time.time() - t0)
        if rest > 0:
            self._stop.wait(rest)

    def _sample_async_loop(self, name):
        while not self._stop.is_set():
            t0 = time.time()
            self.request_async(name)
            self._sleep_rest(t0)

    def request_async(self, name):
        """Ask arm ``name`` for its servo angles unless a request is still outstanding."""
        with self._pending_locks[name]:
            if self._pending[name]:
                return False
            self._pending[name] = True
        try:
            self.arms[name].get_servo_angle(wait=False, callback=functools.partial(self._on_servo_angles, name))
        except Exception:  # noqa: BLE001 - a failed request must not stop the loop
            with self._pending_locks[name]:
                self._pending[name] = False
            return False
        return True

    def _on_servo_angles(self, name, res):
        try:
            servo = parse_servo_result(res)
            if servo is None:
                return
            with self._locks[name]:
                self._latest[name]["timestamp"] = time.time()
                self._latest[name]["servo_deg"] = servo
        finally:
            with self._pending_locks[name]:
                self._pending[name] = False

    def _sample_sync_loop(self, name):
        while not self._stop.is_set():
            t0 = time.time()
            self.sample_sync(name)
            self._sleep_rest(t0)

    def sample_sync(self, name):
        """Read servos 0-2 one by one and append the tracked wrist angle."""
        arm = self.arms[name]
        try:
            angles = []
            for servo_id in range(WRIST_SERVOS):
                try:
                    angles.append(float(arm.get_servo_angle(servo_id)))
                except Exception as exc:  # noqa: BLE001
                    log.warning("%s: servo %d not readable (%s)", name, servo_id, exc)
                    angles.append(FALLBACK_SERVO_DEG)
            wrist = self._wrist_angle
            angles.append(float(wrist))
            with self._locks[name]:
                self._latest[name]["timestamp"] = time.time()
                self._latest[name]["servo_deg"] = angles
                self._latest[name]["wrist_angle"] = wrist
            log.debug("%s: serving %s", name, angles)
        except Exception as exc:  # noqa: BLE001
            log.warning("%s: telemetry error (%s)", name, exc)

    # -- views ---------------------------------------------------------------
    def servo_angles(self):
        """Latest servo angles per arm, e.g. ``{"arm1": [...], "arm2": [...], "arm3": [...]}``."""
        out = {}
        for name in self.arms:
            with self._locks[name]:
                servo = self._latest[name]["servo_deg"]
            out[name] = list(servo) if servo is not None else None
        return out

    def state(self):
        """Latest sample per arm including timestamps (and the wrist angle of the wrist arm)."""
        out = {}
        for name in self.arms:
            with self._locks[name]:
                out[name] = dict(self._latest[name])
        return out

    def _build_router(self):
        router = APIRouter()
        router.add_api_route("/api/servo_angles", self.servo_angles, methods=["GET"],
                             summary="Latest servo angles per arm (polled by the Isaac Sim follower)")
        router.add_api_route("/api/state", self.state, methods=["GET"], summary="Latest samples with timestamps")
        return router

    def attach(self, app):
        """Add the telemetry routes to an existing FastAPI app and start sampling."""
        app.include_router(self.router)
        self.start()
        return self

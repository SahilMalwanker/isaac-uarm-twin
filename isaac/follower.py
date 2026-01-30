"""Digital-twin follower: mirrors the servo angles of three uArm Swift Pro arms onto Isaac Sim joints.

Runs inside Isaac Sim - loaded by ``bootstrap.py``, or pasted into the Script Editor while the twin
stage is open. Two threads share the latest sample through a lock:

* an HTTP thread polls ``GET TWIN_TELEMETRY_URL`` at ``POLL_HZ`` and keeps the newest servo angles;
* a PhysX physics-step callback (every simulation step, about 60 Hz) turns them into joint targets.

Per servo and step: two-point calibration -> joint-limit clamp -> rate limit -> exponential smoothing
-> deadband -> ``set_dof_position_target`` in radians. The PhysX PD drive of each joint then produces
the torque, so motion stays physical instead of snapping. No inverse kinematics or trajectory planning
is involved. If telemetry stops, the twin holds its last pose.
"""

import math
import os
import threading
import time

API_URL = os.environ.get("TWIN_TELEMETRY_URL", "http://127.0.0.1:8001/api/servo_angles")
POLL_HZ = 120
HTTP_TIMEOUT_S = 0.1

ALPHA = 0.28  # smoothing per physics step: higher = more responsive, lower = smoother
DEADBAND_DEG = 0.05  # skip target updates smaller than this (reduces visible jitter)
WAKE_EVERY_STEP = True  # keep articulations awake, otherwise they "sleep then jump"

# Servo index -> Isaac joint. Joint6 is the elbow crank; the forearm follows it through a closed loop.
SERVO_TO_DOF = {0: "Joint1", 1: "Joint2", 2: "Joint6"}
WRIST_TO_DOF = {3: "JointGripperExt"}  # arm 2 only: wrist angle tracked by the robot service

JOINT_LIMITS_DEG = {
    "Joint1": (-90.0, 90.0),
    "Joint2": (-30.0, 90.0),
    "Joint6": (-17.0, 110.0),
    "JointGripperExt": (-90.0, 90.0),
}

# Two measured anchor poses per joint: servo angle with the arm fully extended / fully retracted, and
# the Isaac joint angle that shows the same pose. Determined with isaac_uarm_twin.calibrate.
CALIBRATION = {
    "Joint1": {"servo_ext": 0.0, "servo_ret": 180.0, "isaac_ext": -90.0, "isaac_ret": 90.0},
    "Joint2": {"servo_ext": -6.44, "servo_ret": 129.966, "isaac_ext": 89.954, "isaac_ret": -29.972},
    "Joint6": {"servo_ext": -22.0, "servo_ret": 113.0, "isaac_ext": -17.0, "isaac_ret": 110.0},
    "JointGripperExt": {"servo_ext": 180.0, "servo_ret": 0.0, "isaac_ext": 90.0, "isaac_ret": -90.0},
}

RATE_LIMIT_DEG_PER_S = {"Joint1": 360.0, "Joint2": 220.0, "Joint6": 260.0, "JointGripperExt": 200.0}


def arm_config(index, wrist=False):
    mapping = dict(SERVO_TO_DOF)
    if wrist:
        mapping.update(WRIST_TO_DOF)
    joints = list(mapping.values())
    return {
        "art_path": f"/World/uarm{index}/swiftpro/Base",
        "api_key": f"arm{index}",
        "servo_to_dof": mapping,
        "isaac_limits_deg": {name: JOINT_LIMITS_DEG[name] for name in joints},
        "cal": {name: CALIBRATION[name] for name in joints},
        "rate_limit_deg_per_s": {name: RATE_LIMIT_DEG_PER_S[name] for name in joints},
        "expected_servo_count": len(mapping),
    }


ARM_CONFIGS = {"arm1": arm_config(1), "arm2": arm_config(2, wrist=True), "arm3": arm_config(3)}


# -----------------------------------------------------------------------------
# Pure helpers
# -----------------------------------------------------------------------------
def clamp(x, lo, hi):
    lo, hi = (lo, hi) if lo <= hi else (hi, lo)
    return max(lo, min(hi, x))


def map_linear(x, x0, x1, y0, y1):
    """Map ``x`` from [x0, x1] to [y0, y1]; inputs outside the anchors are clamped first."""
    lo, hi = (x0, x1) if x0 <= x1 else (x1, x0)
    x = max(lo, min(hi, x))
    if abs(x1 - x0) < 1e-9:
        return y0
    return y0 + (x - x0) / (x1 - x0) * (y1 - y0)


def servo_to_isaac_deg(config, dof_name, servo_deg):
    cal = config["cal"].get(dof_name)
    if cal is None:
        deg = float(servo_deg)
    else:
        deg = map_linear(float(servo_deg), cal["servo_ext"], cal["servo_ret"], cal["isaac_ext"], cal["isaac_ret"])
    lo, hi = config["isaac_limits_deg"].get(dof_name, (-1e9, 1e9))
    return clamp(deg, lo, hi)


def rate_limit(prev_deg, target_deg, max_rate_deg_s, dt):
    if prev_deg is None or dt <= 0.0:
        return target_deg
    max_step = max_rate_deg_s * dt
    delta = target_deg - prev_deg
    if delta > max_step:
        return prev_deg + max_step
    if delta < -max_step:
        return prev_deg - max_step
    return target_deg


# -----------------------------------------------------------------------------
# Follower
# -----------------------------------------------------------------------------
class TwinFollower:
    """Latest telemetry in, smoothed PhysX joint targets out."""

    def __init__(self, configs, dc, alpha=ALPHA, deadband_deg=DEADBAND_DEG, wake_every_step=WAKE_EVERY_STEP,
                 invalid_handle=0):
        self.configs = configs
        self.dc = dc
        self.alpha = alpha
        self.deadband_rad = math.radians(deadband_deg)
        self.wake_every_step = wake_every_step
        self.invalid_handle = invalid_handle
        self._reported_missing = set()
        self.lock = threading.Lock()
        self.latest = {arm: {"t": 0.0, "servo_deg": None} for arm in configs}
        self.arts = {}
        self.dof_handles = {}
        self.last_apply_rad = {arm: {d: None for d in cfg["servo_to_dof"].values()} for arm, cfg in configs.items()}
        self.smoothed_deg = {arm: {d: None for d in cfg["servo_to_dof"].values()} for arm, cfg in configs.items()}

    def ingest(self, payload):
        """Store one ``/api/servo_angles`` payload; arms with missing or short data keep their last sample."""
        for arm_name, config in self.configs.items():
            servo = payload.get(config["api_key"])
            count = config["expected_servo_count"]
            if isinstance(servo, (list, tuple)) and len(servo) >= count:
                servo = [float(s) for s in servo[:count]]
                with self.lock:
                    self.latest[arm_name]["t"] = float(payload.get("timestamp", time.time()))
                    self.latest[arm_name]["servo_deg"] = servo

    def init_articulations(self):
        dc = self.dc
        for arm_name, config in self.configs.items():
            if arm_name in self.arts:
                continue
            art = dc.get_articulation(config["art_path"])
            if art is None or art == self.invalid_handle:
                if arm_name not in self._reported_missing:
                    print(f"[DT] No articulation at {config['art_path']} for {arm_name}")
                    self._reported_missing.add(arm_name)
                continue
            handles = {}
            for dof_index in range(dc.get_articulation_dof_count(art)):
                dof = dc.get_articulation_dof(art, dof_index)
                if dof is not None and dof != self.invalid_handle:
                    handles[dc.get_dof_name(dof)] = dof
            missing = [name for name in config["servo_to_dof"].values() if name not in handles]
            if missing:
                if arm_name not in self._reported_missing:
                    print(f"[DT] {arm_name} WARNING: missing DOFs: {missing}")
                    self._reported_missing.add(arm_name)
                continue
            dc.wake_up_articulation(art)
            self.dof_handles[arm_name] = handles
            self.arts[arm_name] = art
            self._reported_missing.discard(arm_name)
            print(f"[DT] {arm_name} DOFs: {list(handles)}")

    def on_physics_step(self, dt):
        if len(self.arts) < len(self.configs):
            self.init_articulations()
            if not self.arts:
                return
        sim_dt = float(dt) if dt and dt > 0 else 1.0 / 60.0
        for arm_name, config in self.configs.items():
            art = self.arts.get(arm_name)
            if art is None:
                continue
            if self.wake_every_step:
                self.dc.wake_up_articulation(art)
            with self.lock:
                servo_deg = self.latest[arm_name]["servo_deg"]
            if not servo_deg or len(servo_deg) < config["expected_servo_count"]:
                continue
            for servo_idx, dof_name in config["servo_to_dof"].items():
                dof = self.dof_handles[arm_name].get(dof_name)
                if dof is None:
                    continue
                target_deg = servo_to_isaac_deg(config, dof_name, servo_deg[servo_idx])
                prev_deg = self.smoothed_deg[arm_name][dof_name]
                limited = rate_limit(prev_deg, target_deg, config["rate_limit_deg_per_s"].get(dof_name, 9999.0), sim_dt)
                smooth = limited if prev_deg is None else prev_deg + self.alpha * (limited - prev_deg)
                self.smoothed_deg[arm_name][dof_name] = smooth
                target_rad = math.radians(smooth)
                prev_rad = self.last_apply_rad[arm_name][dof_name]
                if prev_rad is not None and abs(target_rad - prev_rad) < self.deadband_rad:
                    continue
                self.last_apply_rad[arm_name][dof_name] = target_rad
                self.dc.set_dof_position_target(dof, target_rad)


def poll_http(follower, url=API_URL, hz=POLL_HZ, stop_event=None, session=None):
    """Poll the telemetry endpoint at ``hz`` until ``stop_event`` is set."""
    if session is None:
        import requests

        session = requests.Session()
    period = 1.0 / float(hz)
    while not (stop_event is not None and stop_event.is_set()):
        t0 = time.time()
        try:
            follower.ingest(session.get(url, timeout=HTTP_TIMEOUT_S).json())
        except Exception:  # noqa: BLE001 - gaps are expected; the twin holds its pose
            pass
        rest = period - (time.time() - t0)
        if rest > 0:
            time.sleep(rest)


# Survives a re-run from the Script Editor so the previous callback can be removed first.
_RUNTIME = globals().get("_RUNTIME") or {}


def main():
    import omni.physx
    from omni.isaac.dynamic_control import _dynamic_control

    if _RUNTIME.get("stop") is not None:
        _RUNTIME["stop"].set()
        _RUNTIME["subscription"] = None
    dc = _dynamic_control.acquire_dynamic_control_interface()
    follower = TwinFollower(ARM_CONFIGS, dc, invalid_handle=_dynamic_control.INVALID_HANDLE)
    stop = threading.Event()
    thread = threading.Thread(target=poll_http, args=(follower,), kwargs={"stop_event": stop}, daemon=True)
    thread.start()
    subscription = omni.physx.get_physx_interface().subscribe_physics_step_events(follower.on_physics_step)
    _RUNTIME.update(follower=follower, thread=thread, stop=stop, subscription=subscription)
    print("[DT] Multi-arm digital twin follower running. Press Stop to end.")
    print(f"[DT] HTTP poll: {API_URL} at {POLL_HZ} Hz | ALPHA={ALPHA} | WAKE_EVERY_STEP={WAKE_EVERY_STEP}")


# The bootstrap and the Script Editor execute this file; tests and tools import it as ``follower``.
if __name__ != "follower":
    main()

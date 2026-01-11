"""Configuration of the robot-PC services, read from environment variables or a ``.env`` file.

    python -m isaac_uarm_twin.settings     # print the configuration in use
"""

import os
from pathlib import Path

PACKAGE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = PACKAGE_DIR.parent

try:
    from dotenv import load_dotenv
except ImportError:  # python-dotenv is optional; plain environment variables still work
    load_dotenv = None

if load_dotenv is not None:
    load_dotenv(PROJECT_ROOT / ".env")


def _env_int(name, default):
    value = os.getenv(name)
    return int(value) if value not in (None, "") else default


def _env_float(name, default):
    value = os.getenv(name)
    return float(value) if value not in (None, "") else default


def _env_bool(name, default):
    value = os.getenv(name)
    if value in (None, ""):
        return default
    return value.strip().lower() in ("1", "true", "yes", "on")


# Serial ports of the three arms (same defaults as the TIA S7 Copilot robot controller)
ARM_PORTS = {
    "arm1": os.getenv("ARM1_PORT", "COM3"),
    "arm2": os.getenv("ARM2_PORT", "COM5"),
    "arm3": os.getenv("ARM3_PORT", "COM4"),
}

# Telemetry service of the standalone cell demo
HOST = os.getenv("TWIN_HOST", "127.0.0.1")
PORT = _env_int("TWIN_PORT", 8001)
TELEMETRY_HZ = _env_float("TWIN_TELEMETRY_HZ", 1.0)
RUN_MOTION = _env_bool("TWIN_RUN_MOTION", True)

# Endpoint polled by the Isaac Sim follower
TELEMETRY_URL = os.getenv("TWIN_TELEMETRY_URL") or f"http://127.0.0.1:{PORT}/api/servo_angles"

# Calibration helper
CALIBRATION_PORT = _env_int("TWIN_CALIBRATION_PORT", 8002)

# Isaac Sim launcher
ISAAC_SIM_EXE = os.getenv("ISAAC_SIM_EXE", "isaacsim")
SCENE_PATH = Path(os.getenv("TWIN_SCENE") or PROJECT_ROOT / "scene" / "cell.usda")
BOOTSTRAP_PATH = PROJECT_ROOT / "isaac" / "bootstrap.py"
FOLLOWER_PATH = PROJECT_ROOT / "isaac" / "follower.py"
WAIT_TIMEOUT = _env_float("TWIN_WAIT_TIMEOUT", 60.0)


def summary():
    """Configuration as a list of (name, value) pairs for display."""
    return [
        ("ARM_PORTS", ARM_PORTS),
        ("HOST:PORT", f"{HOST}:{PORT}"),
        ("TELEMETRY_HZ", TELEMETRY_HZ),
        ("RUN_MOTION", RUN_MOTION),
        ("TELEMETRY_URL", TELEMETRY_URL),
        ("CALIBRATION_PORT", CALIBRATION_PORT),
        ("ISAAC_SIM_EXE", ISAAC_SIM_EXE),
        ("SCENE_PATH", SCENE_PATH),
        ("WAIT_TIMEOUT", WAIT_TIMEOUT),
    ]


if __name__ == "__main__":
    for key, value in summary():
        print(f"{key:18s} {value}")
    if not SCENE_PATH.is_file():
        print(f"warning: scene not found at {SCENE_PATH} - run tools/build_scene.py")

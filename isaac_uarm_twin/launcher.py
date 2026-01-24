"""Start Isaac Sim with the digital twin once the servo telemetry answers.

    python -m isaac_uarm_twin.launcher [--isaac PATH] [--scene scene/cell.usda] [--no-wait]

1. finds the Isaac Sim executable (``isaacsim`` from the pip install, or a full path),
2. waits until ``GET TWIN_TELEMETRY_URL`` answers - served by the TIA S7 Copilot robot controller
   or by ``isaac_uarm_twin.cell_demo``,
3. starts Isaac Sim as a separate, non-blocking process that runs ``isaac/bootstrap.py``: open the
   stage, load the follower into the running simulation and press Play.

Isaac Sim runs on its own. Closing it does not affect the robot services, and if telemetry stops the
twin simply holds its last pose.
"""

import argparse
import os
import shutil
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

from . import settings


def resolve_executable(exe):
    """Absolute path of the Isaac Sim executable, or ``None``."""
    found = shutil.which(exe)
    if found:
        return found
    path = Path(exe)
    return str(path.resolve()) if path.is_file() else None


def wait_for_service(url, timeout, interval=1.0, opener=urllib.request.urlopen, clock=time.monotonic,
                     sleep=time.sleep):
    """Poll ``url`` until it answers with HTTP 200 or ``timeout`` seconds have passed."""
    if not url.startswith(("http://", "https://")):
        raise ValueError(f"telemetry URL must be http(s): {url!r}")
    deadline = clock() + timeout
    while True:
        try:
            with opener(url, timeout=2.0) as response:
                if response.status == 200:
                    return True
        except (urllib.error.URLError, OSError):
            pass
        if clock() >= deadline:
            return False
        sleep(interval)


def build_command(exe, bootstrap, scene, follower):
    """Command line that makes Isaac Sim run the bootstrap with the scene and follower as arguments."""
    return [str(exe), "--exec", f"{bootstrap} {scene} {follower}"]


def main(argv=None):
    parser = argparse.ArgumentParser(description="Start Isaac Sim with the Isaac uArm Twin.")
    parser.add_argument("--isaac", default=settings.ISAAC_SIM_EXE, help="Isaac Sim executable (default: %(default)s)")
    parser.add_argument("--scene", default=str(settings.SCENE_PATH), help="USD stage to open")
    parser.add_argument("--follower", default=str(settings.FOLLOWER_PATH), help="script loaded into the simulation")
    parser.add_argument("--telemetry-url", default=settings.TELEMETRY_URL, help="servo telemetry endpoint")
    parser.add_argument("--timeout", type=float, default=settings.WAIT_TIMEOUT, help="seconds to wait for telemetry")
    parser.add_argument("--no-wait", action="store_true", help="start Isaac Sim without waiting for telemetry")
    args = parser.parse_args(argv)

    exe = resolve_executable(args.isaac)
    if exe is None:
        print(f"Isaac Sim executable not found: {args.isaac!r}. Set ISAAC_SIM_EXE or pass --isaac.", file=sys.stderr)
        return 1
    scene = Path(args.scene).resolve()
    if not scene.is_file():
        print(f"Scene not found: {scene}. Run: python tools/build_scene.py", file=sys.stderr)
        return 1
    if " " in str(settings.PROJECT_ROOT):
        print("warning: the project path contains spaces; Isaac Sim may split the --exec arguments "
              "and fall back to the default scene and follower.", file=sys.stderr)

    if not args.no_wait:
        print(f"Waiting up to {args.timeout:.0f} s for {args.telemetry_url} ...")
        if wait_for_service(args.telemetry_url, args.timeout):
            print("Telemetry is up.")
        else:
            print("warning: no telemetry yet - the twin will hold its pose until samples arrive.", file=sys.stderr)

    env = os.environ.copy()
    env["TWIN_TELEMETRY_URL"] = args.telemetry_url
    command = build_command(exe, settings.BOOTSTRAP_PATH, scene, Path(args.follower).resolve())
    process = subprocess.Popen(command, env=env, cwd=str(settings.PROJECT_ROOT))
    print(f"Isaac Sim started (pid {process.pid}). The twin goes live after the stage has loaded.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

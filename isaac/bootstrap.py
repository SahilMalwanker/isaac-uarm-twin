"""Isaac Sim bootstrap: open the twin stage, load the follower into the simulation and press Play.

Runs inside Isaac Sim; ``python -m isaac_uarm_twin.launcher`` starts it like this:

    isaacsim --exec "isaac/bootstrap.py scene/cell.usda isaac/follower.py"

Kit consumes some command-line entries, so the last ``.usd``/``.usda`` and the last ``.py`` other
than this file are taken from ``sys.argv``. Without them, ``../scene/cell.usda`` and ``follower.py``
next to this file are used.

The follower registers a PhysX physics-step callback, which only fires for code running inside the
Isaac Sim process. Its namespace stays referenced while the app runs. Unresolved articulations are
retried on later physics steps; rerun the bootstrap after replacing the stage.
"""

import asyncio
import os
import sys
import time
import traceback

SETTLE_SECONDS = 6.0  # articulations, meshes and textures need a moment after the stage opens
SCENE_EXTENSIONS = (".usd", ".usda", ".usdc")

_KEEP_ALIVE = {}


def script_dir():
    try:
        return os.path.dirname(os.path.abspath(__file__))
    except NameError:  # some Kit exec paths do not define __file__
        return os.getcwd()


def this_script():
    try:
        return os.path.normpath(os.path.abspath(__file__))
    except NameError:
        return ""


def find_last_arg(argv, extensions, base_dir, exclude=""):
    """Last argument ending in one of ``extensions`` that exists (relative to ``base_dir`` or the CWD)."""
    for arg in reversed(argv):
        if not arg.lower().endswith(extensions):
            continue
        candidates = [arg] if os.path.isabs(arg) else [os.path.join(base_dir, arg), os.path.abspath(arg)]
        for candidate in candidates:
            if os.path.isfile(candidate):
                resolved = os.path.normpath(os.path.abspath(candidate))
                if resolved != exclude:
                    return resolved
                break
    return None


def resolve_inputs(argv, base_dir, exclude=""):
    """(scene, follower) paths from the command line, or the defaults next to this script."""
    scene = find_last_arg(argv, SCENE_EXTENSIONS, base_dir)
    if scene is None:
        fallback = os.path.normpath(os.path.join(base_dir, "..", "scene", "cell.usda"))
        if os.path.isfile(fallback):
            scene = fallback
            print(f"[Bootstrap] Scene fallback: {scene}")
    follower = find_last_arg(argv, (".py",), base_dir, exclude=exclude)
    if follower is None:
        fallback = os.path.join(base_dir, "follower.py")
        if os.path.isfile(fallback):
            follower = fallback
            print(f"[Bootstrap] Follower fallback: {follower}")
    return scene, follower


def open_stage(path):
    import omni.usd

    context = omni.usd.get_context()
    print(f"[Bootstrap] Opening stage: {path}")
    result = context.open_stage(path)
    if not result:
        print("[Bootstrap] Sync open_stage returned False - trying async ...")
        try:
            async def _open_async():
                ok, error = await context.open_stage_async(path)
                if error:
                    print(f"[Bootstrap] Async open error: {error}")
                return ok

            result = asyncio.get_event_loop().run_until_complete(_open_async())
        except Exception as exc:  # noqa: BLE001
            print(f"[Bootstrap] Async open failed: {exc}")
    print("[Bootstrap] Stage opened." if result else "[Bootstrap] ERROR: stage failed to open.")
    return result


def load_follower(path):
    """Execute the follower in this process; its physics callback is registered while it runs."""
    print(f"[Bootstrap] Loading follower into the simulation: {path}")
    namespace = {"__name__": "__main__", "__file__": path}
    try:
        with open(path, "r", encoding="utf-8") as handle:
            code = handle.read()
        exec(compile(code, path, "exec"), namespace)
        print("[Bootstrap] Follower loaded - physics callback registered.")
    except Exception as exc:  # noqa: BLE001
        print(f"[Bootstrap] ERROR loading follower: {exc}")
        traceback.print_exc()
        return None
    _KEEP_ALIVE["follower"] = namespace
    return namespace


def main(argv=None):
    argv = sys.argv if argv is None else argv
    base = script_dir()
    scene, follower = resolve_inputs(argv, base, exclude=this_script())
    print("=" * 60)
    print("  Isaac uArm Twin - Isaac Sim bootstrap")
    print("=" * 60)
    print(f"  argv     : {argv}")
    print(f"  scene    : {scene}")
    print(f"  follower : {follower}")
    print("=" * 60)
    if not scene:
        print("[Bootstrap] ERROR: no USD stage found - aborting.")
        return False
    if not follower:
        print("[Bootstrap] ERROR: no follower script found - aborting.")
        return False

    import omni.timeline

    try:
        if not open_stage(scene):
            print("[Bootstrap] ERROR: stage failed to open - aborting.")
            return False
        print(f"[Bootstrap] Waiting {SETTLE_SECONDS:.0f} s for the stage to settle ...")
        time.sleep(SETTLE_SECONDS)
        if load_follower(follower) is None:
            print("[Bootstrap] ERROR: follower failed to load - aborting.")
            return False
        time.sleep(1.0)
        omni.timeline.get_timeline_interface().play()
    except Exception as exc:
        print(f"[Bootstrap] ERROR: startup failed - aborting: {exc}")
        return False
    print("[Bootstrap] Timeline playing - the digital twin is live.")
    print("[Bootstrap] Stop the timeline or close Isaac Sim to end.")
    return True


# Kit runs --exec scripts under varying module names, so only the test suite skips the start.
if "pytest" not in sys.modules:
    main()

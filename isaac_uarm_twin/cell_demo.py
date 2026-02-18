"""Standalone cell demo: drives the three arms through their sequences and serves servo telemetry.

    python -m isaac_uarm_twin.cell_demo

Use it to run the twin without the TIA S7 Copilot. When the copilot's robot controller drives the
arms, attach :class:`~isaac_uarm_twin.telemetry.ServoTelemetry` to it instead (see docs/integration.md);
only one process can own the serial ports.
"""

import logging
import threading
import time

import uvicorn
from fastapi import FastAPI

from . import settings
from .sequences import PHASES, run_steps
from .telemetry import ServoTelemetry

log = logging.getLogger("isaac_uarm_twin.cell_demo")


def connect_arm(name, port):
    """Open one uArm Swift Pro with async callbacks and a write thread (needed for async telemetry)."""
    from uarm.wrapper import SwiftAPI

    arm = SwiftAPI(
        port=port,
        cmd_pend_size=2,
        callback_thread_pool_size=1,
        enable_write_thread=True,
        enable_handle_report_thread=False,
    )
    arm.waiting_ready()
    info = arm.get_device_info() or {}
    firmware = info.get("firmware_version")
    log.info("%s ready on %s (firmware %s)", name, port, firmware)
    if firmware and not firmware.startswith(("0.", "1.", "2.", "3.")):
        arm.set_speed_factor(0.00001)
    return arm


def motion_loop(arms, stop, sleep=time.sleep):
    """Alternate the two assembly phases until ``stop`` is set."""
    while not stop.is_set():
        for title, steps in PHASES:
            log.info("[SEQUENCE] %s", title)
            for name, sequence in steps:
                if stop.is_set():
                    return
                run_steps(arms[name], sequence, sleep=sleep)
        log.info("[SEQUENCE] Cycle complete, looping...")
        sleep(1)


def create_app(telemetry):
    app = FastAPI(title="Isaac uArm Twin - cell demo", version="1.0")
    app.include_router(telemetry.router)
    return app


def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    arms = {name: connect_arm(name, port) for name, port in settings.ARM_PORTS.items()}
    telemetry = ServoTelemetry(arms, wrist_arm="arm2", rate_hz=settings.TELEMETRY_HZ)
    telemetry.start()
    stop = threading.Event()
    if settings.RUN_MOTION:
        threading.Thread(target=motion_loop, args=(arms, stop), name="motion", daemon=True).start()
    else:
        log.info("TWIN_RUN_MOTION=0: serving telemetry only")
    try:
        uvicorn.run(create_app(telemetry), host=settings.HOST, port=settings.PORT)
    finally:
        stop.set()
        telemetry.stop()


if __name__ == "__main__":
    main()

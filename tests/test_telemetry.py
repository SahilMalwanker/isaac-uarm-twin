import time

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from fakes import FakeSwift
from isaac_uarm_twin import telemetry
from isaac_uarm_twin.telemetry import ServoTelemetry, parse_servo_result


def make(**kwargs):
    arms = {"arm1": FakeSwift(), "arm2": FakeSwift(servo=(80.0, 50.0, 70.0)), "arm3": FakeSwift()}
    return ServoTelemetry(arms, **kwargs), arms


@pytest.mark.parametrize("payload, expected", [
    ([1, 2, 3], [1, 2, 3]),
    ((1, 2), [1, 2]),
    ({"servo_deg": [4, 5]}, [4, 5]),
    ({"data": (6,)}, [6]),
    ({"result": [7]}, [7]),
    ({"other": 1}, None),
    (None, None),
    ("OK", None),
])
def test_parse_servo_result(payload, expected):
    assert parse_servo_result(payload) == expected


def test_rate_must_be_positive():
    with pytest.raises(ValueError):
        ServoTelemetry({"arm1": FakeSwift()}, rate_hz=0)


def test_unknown_wrist_arm_disables_wrist_tracking():
    twin = ServoTelemetry({"arm1": FakeSwift()}, wrist_arm="arm2")
    assert twin.wrist_arm is None


def test_async_request_never_queues_a_second_one():
    twin, arms = make()
    assert twin.request_async("arm1") is True
    assert twin.request_async("arm1") is False  # first request still pending
    assert len(arms["arm1"].callbacks) == 1
    arms["arm1"].callbacks[0]([91.0, 46.0, 61.0])
    assert twin.servo_angles()["arm1"] == [91.0, 46.0, 61.0]
    assert twin.request_async("arm1") is True


def test_unusable_callback_payload_still_clears_pending():
    twin, arms = make()
    twin.request_async("arm3")
    arms["arm3"].callbacks[0](None)
    assert twin.servo_angles()["arm3"] is None
    assert twin.request_async("arm3") is True


def test_failed_request_clears_pending():
    twin, arms = make()

    def broken(**kwargs):
        raise OSError("port closed")

    arms["arm1"].get_servo_angle = broken
    assert twin.request_async("arm1") is False
    assert twin._pending["arm1"] is False


def test_wrist_arm_reads_servos_and_appends_tracked_wrist():
    twin, arms = make()
    arms["arm2"].set_wrist(150)
    assert arms["arm2"].calls[-1] == ("set_wrist", 150)  # the real call still goes through
    twin.sample_sync("arm2")
    assert twin.servo_angles()["arm2"] == [80.0, 50.0, 70.0, 150.0]
    assert twin.state()["arm2"]["wrist_angle"] == 150.0


def test_wrist_starts_at_home():
    twin, _ = make(wrist_home=90.0)
    twin.sample_sync("arm2")
    assert twin.servo_angles()["arm2"][-1] == 90.0


def test_unreadable_servo_falls_back():
    arms = {"arm2": FakeSwift(failing_servo=1)}
    twin = ServoTelemetry(arms)
    twin.sample_sync("arm2")
    assert twin.servo_angles()["arm2"][1] == telemetry.FALLBACK_SERVO_DEG


def test_endpoints():
    twin, arms = make()
    app = FastAPI()
    app.include_router(twin.router)
    client = TestClient(app)
    assert client.get("/api/servo_angles").json() == {"arm1": None, "arm2": None, "arm3": None}
    twin.request_async("arm1")
    arms["arm1"].callbacks[0]([90.0, 45.0, 60.0])
    twin.sample_sync("arm2")
    body = client.get("/api/servo_angles").json()
    assert body["arm1"] == [90.0, 45.0, 60.0]
    assert body["arm2"] == [80.0, 50.0, 70.0, 90.0]
    state = client.get("/api/state").json()
    assert state["arm1"]["timestamp"] > 0
    assert state["arm2"]["wrist_angle"] == 90.0


def test_attach_adds_routes_and_starts_sampling(monkeypatch):
    twin, _ = make()
    started = []
    monkeypatch.setattr(twin, "start", lambda: started.append(True))
    app = FastAPI()
    assert twin.attach(app) is twin
    assert started == [True]
    client = TestClient(app)
    assert client.get("/api/servo_angles").status_code == 200
    assert client.get("/api/state").status_code == 200


def test_sampling_threads_start_and_stop():
    twin, arms = make(rate_hz=50.0)
    twin.start()
    twin.start()  # idempotent
    assert len(twin._threads) == 3
    deadline = time.monotonic() + 2.0
    while not (arms["arm1"].callbacks and twin.servo_angles()["arm2"]) and time.monotonic() < deadline:
        time.sleep(0.01)
    twin.stop()
    assert twin._threads == []
    assert arms["arm1"].callbacks  # async request issued
    assert twin.servo_angles()["arm2"] == [80.0, 50.0, 70.0, 90.0]

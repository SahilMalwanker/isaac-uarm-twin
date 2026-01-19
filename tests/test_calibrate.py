from fastapi.testclient import TestClient

from fakes import FakeSwift
from isaac_uarm_twin import calibrate


def client_for(arm):
    return TestClient(calibrate.create_app(calibrate.CaptureSession(arm)))


def test_detach_and_attach_main_servos():
    arm = FakeSwift()
    client = client_for(arm)
    assert client.post("/api/servos/detach").json() == {"success": True}
    assert client.post("/api/servos/attach").json() == {"success": True}
    assert arm.calls == [("detach", 0), ("detach", 1), ("detach", 2), ("attach", 0), ("attach", 1), ("attach", 2)]


def test_single_servo_routes_validate_the_id():
    arm = FakeSwift()
    client = client_for(arm)
    assert client.post("/api/servos/detach/3").json() == {"success": True, "servo_id": 3}
    assert client.post("/api/servos/attach/3").json() == {"success": True, "servo_id": 3}
    assert client.post("/api/servos/detach/4").status_code == 422
    assert arm.calls == [("detach", 3), ("attach", 3)]


def test_failing_servo_reports_failure():
    arm = FakeSwift()

    def broken(servo_id=None, wait=True):
        raise OSError("port closed")

    arm.set_servo_detach = broken
    assert calibrate.CaptureSession(arm).detach() is False


def test_capture_names_the_servos():
    arm = FakeSwift(servo=(12.5, 80.0, 30.0), position=(200.0, 10.0, 50.0))
    client = client_for(arm)
    assert client.get("/api/state").json()["servo_deg"] is None
    body = client.post("/api/capture").json()
    assert body["servo_deg"] == {"base": 12.5, "shoulder": 80.0, "elbow": 30.0}
    assert body["position_mm"] == [200.0, 10.0, 50.0]
    assert client.get("/api/state").json()["servo_deg"]["base"] == 12.5

import importlib

import pytest

from isaac_uarm_twin import settings


@pytest.fixture
def reload_settings(monkeypatch):
    for name in ("TWIN_HOST", "TWIN_PORT", "TWIN_TELEMETRY_URL", "TWIN_RUN_MOTION", "TWIN_TELEMETRY_HZ", "TWIN_SCENE"):
        monkeypatch.delenv(name, raising=False)

    def reload(**env):
        for name, value in env.items():
            monkeypatch.setenv(name, value)
        return importlib.reload(settings)

    yield reload
    monkeypatch.undo()
    importlib.reload(settings)


def test_defaults_bind_to_localhost(reload_settings):
    s = reload_settings()
    assert s.HOST == "127.0.0.1"
    assert s.SCENE_PATH.name == "cell.usda"


def test_environment_overrides(reload_settings):
    s = reload_settings(TWIN_PORT="9001", TWIN_RUN_MOTION="0", ARM2_PORT="/dev/ttyACM1", TWIN_TELEMETRY_HZ="2.5")
    assert s.PORT == 9001
    assert s.TELEMETRY_URL == "http://127.0.0.1:9001/api/servo_angles"
    assert s.RUN_MOTION is False
    assert s.ARM_PORTS["arm2"] == "/dev/ttyACM1"
    assert s.TELEMETRY_HZ == 2.5
    assert dict(s.summary())["HOST:PORT"] == "127.0.0.1:9001"

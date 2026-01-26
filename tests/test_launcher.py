import io
import urllib.error

import pytest

from isaac_uarm_twin import launcher, settings


class FakeResponse(io.BytesIO):
    def __init__(self, status):
        super().__init__(b"{}")
        self.status = status


def test_build_command_passes_scene_and_follower_to_the_bootstrap():
    command = launcher.build_command("isaacsim", "isaac/bootstrap.py", "scene/cell.usda", "isaac/follower.py")
    assert command == ["isaacsim", "--exec", "isaac/bootstrap.py scene/cell.usda isaac/follower.py"]


def test_wait_for_service_retries_until_ready():
    answers = [urllib.error.URLError("down"), FakeResponse(503), FakeResponse(200)]
    slept = []

    def opener(url, timeout):
        answer = answers.pop(0)
        if isinstance(answer, Exception):
            raise answer
        return answer

    assert launcher.wait_for_service("http://127.0.0.1:8001/api/servo_angles", timeout=10, interval=0.5,
                                     opener=opener, clock=lambda: 0.0, sleep=slept.append)
    assert slept == [0.5, 0.5]


def test_wait_for_service_times_out():
    now = [0.0]

    def opener(url, timeout):
        raise ConnectionRefusedError()

    def sleep(seconds):
        now[0] += seconds

    assert not launcher.wait_for_service("http://127.0.0.1:8001/x", timeout=3, interval=1.0, opener=opener,
                                         clock=lambda: now[0], sleep=sleep)
    assert now[0] == 3.0


def test_wait_for_service_rejects_other_schemes():
    with pytest.raises(ValueError):
        launcher.wait_for_service("file:///etc/passwd", timeout=1)


def test_resolve_executable(tmp_path):
    exe = tmp_path / "isaac-sim.sh"
    exe.write_text("")
    assert launcher.resolve_executable(str(exe)) == str(exe.resolve())
    assert launcher.resolve_executable(str(tmp_path / "missing")) is None


def test_main_requires_isaac_sim(monkeypatch, capsys):
    monkeypatch.setattr(launcher, "resolve_executable", lambda exe: None)
    assert launcher.main(["--no-wait"]) == 1
    assert "not found" in capsys.readouterr().err


def test_main_requires_the_scene(monkeypatch, tmp_path):
    monkeypatch.setattr(launcher, "resolve_executable", lambda exe: "isaacsim")
    assert launcher.main(["--no-wait", "--scene", str(tmp_path / "missing.usda")]) == 1


def test_main_starts_isaac_sim_without_blocking(monkeypatch, tmp_path):
    scene = tmp_path / "cell.usda"
    scene.write_text("#usda 1.0\n")
    started = {}

    class Process:
        pid = 4242

    def popen(command, env, cwd):
        started.update(command=command, env=env, cwd=cwd)
        return Process()

    monkeypatch.setattr(launcher, "resolve_executable", lambda exe: "/opt/isaacsim/isaac-sim.sh")
    monkeypatch.setattr(launcher, "wait_for_service", lambda url, timeout: False)
    monkeypatch.setattr(launcher.subprocess, "Popen", popen)
    url = "http://127.0.0.1:9001/api/servo_angles"
    assert launcher.main(["--scene", str(scene), "--telemetry-url", url, "--timeout", "1"]) == 0
    assert started["command"][:2] == ["/opt/isaacsim/isaac-sim.sh", "--exec"]
    assert str(settings.BOOTSTRAP_PATH) in started["command"][2]
    assert str(scene.resolve()) in started["command"][2]
    assert started["env"]["TWIN_TELEMETRY_URL"] == url

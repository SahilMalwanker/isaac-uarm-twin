import os
import sys
import types
from unittest.mock import Mock

import pytest

import bootstrap


def touch(path):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("")
    return path


def test_last_existing_argument_wins(tmp_path):
    first = touch(tmp_path / "a.usda")
    second = touch(tmp_path / "b.usd")
    argv = ["kit", str(first), "--flag", str(second), str(tmp_path / "missing.usda")]
    assert bootstrap.find_last_arg(argv, bootstrap.SCENE_EXTENSIONS, str(tmp_path)) == os.path.normpath(str(second))


def test_relative_arguments_resolve_against_the_base_dir(tmp_path):
    touch(tmp_path / "scene" / "cell.usda")
    found = bootstrap.find_last_arg(["scene/cell.usda"], bootstrap.SCENE_EXTENSIONS, str(tmp_path))
    assert found == os.path.normpath(str(tmp_path / "scene" / "cell.usda"))


def test_the_bootstrap_itself_is_not_taken_as_follower(tmp_path):
    me = touch(tmp_path / "bootstrap.py")
    follower = touch(tmp_path / "follower.py")
    argv = [str(follower), str(me)]
    exclude = os.path.normpath(str(me))
    assert bootstrap.find_last_arg(argv, (".py",), str(tmp_path), exclude=exclude) == os.path.normpath(str(follower))


def test_defaults_next_to_the_bootstrap(tmp_path):
    base = tmp_path / "isaac"
    follower = touch(base / "follower.py")
    scene = touch(tmp_path / "scene" / "cell.usda")
    assert bootstrap.resolve_inputs(["kit"], str(base)) == (os.path.normpath(str(scene)), str(follower))


def test_missing_inputs(tmp_path, monkeypatch):
    assert bootstrap.resolve_inputs(["kit"], str(tmp_path)) == (None, None)
    monkeypatch.setattr(bootstrap, "script_dir", lambda: str(tmp_path))
    assert bootstrap.main(["kit"]) is False  # aborts before touching Isaac Sim modules


@pytest.fixture
def startup(monkeypatch):
    timeline = types.ModuleType("omni.timeline")
    play = Mock()
    timeline.get_timeline_interface = lambda: types.SimpleNamespace(play=play)
    omni = types.ModuleType("omni")
    omni.timeline = timeline
    monkeypatch.setitem(sys.modules, "omni", omni)
    monkeypatch.setitem(sys.modules, "omni.timeline", timeline)
    monkeypatch.setattr(bootstrap, "resolve_inputs", lambda *args, **kwargs: ("scene.usda", "follower.py"))
    opened = Mock(return_value=True)
    loaded = Mock(return_value={"__name__": "__main__"})
    slept = Mock()
    monkeypatch.setattr(bootstrap, "open_stage", opened)
    monkeypatch.setattr(bootstrap, "load_follower", loaded)
    monkeypatch.setattr(bootstrap.time, "sleep", slept)
    return types.SimpleNamespace(play=play, opened=opened, loaded=loaded, slept=slept)


def test_failed_stage_never_loads_follower_or_plays(startup, capsys):
    startup.opened.return_value = False
    assert bootstrap.main([]) is False
    startup.loaded.assert_not_called()
    startup.play.assert_not_called()
    startup.slept.assert_not_called()
    assert "digital twin is live" not in capsys.readouterr().out


def test_stage_exception_aborts_startup(startup):
    startup.opened.side_effect = RuntimeError("stage unavailable")
    assert bootstrap.main([]) is False
    startup.loaded.assert_not_called()
    startup.play.assert_not_called()


def test_failed_follower_never_plays(startup, capsys):
    startup.loaded.return_value = None
    assert bootstrap.main([]) is False
    startup.play.assert_not_called()
    assert "digital twin is live" not in capsys.readouterr().out


def test_successful_startup_plays_only_after_loading(startup):
    events = []
    startup.opened.side_effect = lambda path: events.append("open") or True
    startup.loaded.side_effect = lambda path: events.append("load") or {"ready": True}
    startup.play.side_effect = lambda: events.append("play")
    assert bootstrap.main([]) is True
    assert events == ["open", "load", "play"]
    startup.play.assert_called_once()


@pytest.mark.parametrize("source", [None, "def broken(:", "raise RuntimeError('failed to register')"])
def test_load_follower_failure_returns_none_without_retaining_namespace(source, tmp_path, monkeypatch, capsys):
    path = tmp_path / "follower.py"
    if source is not None:
        path.write_text(source, encoding="utf-8")
    monkeypatch.setattr(bootstrap, "_KEEP_ALIVE", {})
    assert bootstrap.load_follower(str(path)) is None
    assert "follower" not in bootstrap._KEEP_ALIVE
    assert "Follower loaded" not in capsys.readouterr().out


def test_load_follower_retains_successful_namespace(tmp_path, monkeypatch):
    path = tmp_path / "follower.py"
    path.write_text("ready = True", encoding="utf-8")
    monkeypatch.setattr(bootstrap, "_KEEP_ALIVE", {})
    namespace = bootstrap.load_follower(str(path))
    assert namespace["ready"] is True
    assert bootstrap._KEEP_ALIVE["follower"] is namespace

import hashlib

import fetch_swiftpro_meshes as fetch


def use_meshes(monkeypatch, **files):
    monkeypatch.setattr(fetch, "MESHES", {name: hashlib.sha256(data).hexdigest() for name, data in files.items()})


def test_download_is_verified_and_written(monkeypatch, tmp_path):
    use_meshes(monkeypatch, **{"Base.STL": b"solid base"})
    assert fetch.fetch(tmp_path, loader=lambda name: b"solid base") == []
    assert (tmp_path / "Base.STL").read_bytes() == b"solid base"
    assert not list(tmp_path.glob("*.part"))


def test_hash_mismatch_is_not_written(monkeypatch, tmp_path):
    use_meshes(monkeypatch, **{"Base.STL": b"solid base"})
    assert fetch.fetch(tmp_path, loader=lambda name: b"tampered") == ["Base.STL"]
    assert not (tmp_path / "Base.STL").exists()


def test_verified_files_are_not_downloaded_again(monkeypatch, tmp_path):
    use_meshes(monkeypatch, **{"Base.STL": b"solid base"})
    (tmp_path / "Base.STL").write_bytes(b"solid base")

    def loader(name):
        raise AssertionError("should not download")

    assert fetch.fetch(tmp_path, loader=loader) == []


def test_copy_from_a_local_clone(monkeypatch, tmp_path):
    use_meshes(monkeypatch, **{"Link1.STL": b"link"})
    source = tmp_path / "pro_links"
    source.mkdir()
    (source / "Link1.STL").write_bytes(b"link")
    assert fetch.main(["--source", str(source), "--dest", str(tmp_path / "meshes")]) == 0
    assert (tmp_path / "meshes" / "Link1.STL").read_bytes() == b"link"


def test_pinned_source():
    assert len(fetch.COMMIT) == 40
    assert fetch.BASE_URL.startswith("https://raw.githubusercontent.com/")
    assert fetch.COMMIT in fetch.BASE_URL
    assert len(fetch.MESHES) == 11
    assert all(len(digest) == 64 for digest in fetch.MESHES.values())

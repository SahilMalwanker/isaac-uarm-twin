"""Download the uArm Swift Pro meshes that the twin URDFs reference.

The meshes come from the uArm Swift Pro ROS package (uArm-Developer/RosForSwiftAndSwiftPro,
Copyright (c) 2017 UFactory; C++ source headers carry a BSD notice while package.xml leaves
``<license>TODO</license>``) and are not redistributed in this repository. The script downloads
them from a pinned upstream commit into ``assets/swiftpro/meshes`` and checks every file against
a SHA-256 hash.

    python tools/fetch_swiftpro_meshes.py                 # download from GitHub
    python tools/fetch_swiftpro_meshes.py --source DIR    # copy from a local clone (.../swiftpro/urdf/pro_links)
"""

from __future__ import annotations

import argparse
import hashlib
import shutil
import sys
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT / "assets" / "swiftpro" / "meshes"
COMMIT = "a735297d586a1120f410ff9c1ea4ecdc21dcbb86"
BASE_URL = f"https://raw.githubusercontent.com/uArm-Developer/RosForSwiftAndSwiftPro/{COMMIT}/swiftpro/urdf/pro_links/"

MESHES = {
    "Base.STL": "2df99c71b8e7b88f27e70e66199bb229d6b966e8846fc5b495b2d0d0921b7e3f",
    "Gripper.STL": "7fd8fa2f161fd71014f47884a1dbc0410a07a298a2ec6ce51ad359d3fa38c23a",
    "Link1.STL": "f2e375b9b53b2007aff9f33f91319a64bc213cd39b4b385c76e4f4cb1136a637",
    "Link2.STL": "d058fc34ab378c49b33ea1d5a0975b7d395d4ef79a48e52d90296b146471bdfa",
    "Link3.STL": "953a3b3507b1404b19c87726d3a21838171bed27f91a032a9a5cef3e219687e0",
    "Link4.STL": "26bc1d0b582b6b1dcd62c5799df0c5d8c5f224b5e5ae41bb13e5c04f867efd6e",
    "Link5.STL": "2352202f71ff320008df7b77f6f8bab60b895dcc0b274852590476d7b060227c",
    "Link6.STL": "53d60fb0f4b5c85e2bbbc1d1a21e78e51c64555db8b085b62efdfb99102d4f4b",
    "Link7.STL": "636b26fab1da638261b9f3dc07987060995dad494f457948a66fdd18904cee74",
    "Link9.STL": "2e9fa1bf3883f3c6684cec3b9f283b9ed575a0467a8c9bd08f510382abb75a70",
    "Suction.STL": "1678efe1394f38794e3532ba0f478c08c832e0c35d7a8d9911b511ec6f1cd54a",
}


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def download(name: str, timeout: float = 60.0) -> bytes:
    with urllib.request.urlopen(BASE_URL + name, timeout=timeout) as response:  # noqa: S310 - fixed https URL
        return response.read()


def fetch(dest: Path = DEST, source: Path | None = None, loader=download) -> list[str]:
    """Fetch every mesh into ``dest``; returns the names of files that failed verification."""
    dest.mkdir(parents=True, exist_ok=True)
    failed = []
    for name, expected in MESHES.items():
        target = dest / name
        if target.is_file() and sha256(target.read_bytes()) == expected:
            print(f"  ok      {name}")
            continue
        data = (source / name).read_bytes() if source else loader(name)
        if sha256(data) != expected:
            print(f"  MISMATCH {name} (not written)")
            failed.append(name)
            continue
        partial = target.with_suffix(".part")
        partial.write_bytes(data)
        shutil.move(str(partial), str(target))
        print(f"  fetched {name} ({len(data) // 1024} KiB)")
    return failed


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--source", type=Path, help="local pro_links folder to copy from instead of downloading")
    parser.add_argument("--dest", type=Path, default=DEST, help=f"output folder (default {DEST})")
    args = parser.parse_args(argv)
    print(f"uArm Swift Pro meshes -> {args.dest}")
    failed = fetch(args.dest, args.source)
    if failed:
        print("Hash check failed for: " + ", ".join(failed), file=sys.stderr)
        return 1
    print("All meshes verified.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

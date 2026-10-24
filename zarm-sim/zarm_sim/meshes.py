"""Hitbot's CAD meshes for the 3D view, made locally from its STEP by ``scripts/fetch_cad_meshes.py``.

The vendor CAD has no published license, so neither it nor meshes derived from
it are committed: the script writes them to ``zarm-sim/cache/meshes/`` (git
ignores ``cache/``). The view draws the arm with them when they are present and
were made for the current URDF; otherwise it falls back to the URDF's primitive
shapes and shows :func:`fetch_command`, which makes them.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Dict, Optional, Tuple

SIM_ROOT = Path(__file__).resolve().parents[1]
MESH_DIR = SIM_ROOT / "cache" / "meshes"
FETCH_SCRIPT = SIM_ROOT / "scripts" / "fetch_cad_meshes.py"
MANIFEST = "manifest.json"
MESH_LINKS = ("base_link", "carriage", "j1_arm", "j2_arm", "flange")
MESH_URL = "/meshes/"


def fetch_command(script: Path = FETCH_SCRIPT) -> str:
    """The command that makes the meshes for this checkout. The path is absolute: the cache belongs to the checkout
    the server runs from, so the command must run this checkout's script, wherever it's typed."""
    path = str(script)
    return f'uv run "{path}"' if " " in path else f"uv run {path}"


def load_meshes(urdf_bytes: bytes, directory: Path = MESH_DIR) -> Tuple[Dict[str, bytes], Optional[str]]:
    """({link: binary STL in metres, in the link's frame}, why not). Empty with no reason when there's no cache."""
    manifest_path = directory / MANIFEST
    if not manifest_path.is_file():
        return {}, None
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if manifest.get("format") != 1:
            return {}, f"{manifest_path} has an unknown format"
        if manifest.get("urdf_sha256") != hashlib.sha256(urdf_bytes).hexdigest():
            return {}, "the CAD meshes were made for a different model"
        meshes = {}
        for link in MESH_LINKS:
            entry = manifest["links"][link]
            name = Path(entry["file"]).name
            data = (directory / name).read_bytes()
            if hashlib.sha256(data).hexdigest() != entry["sha256"]:
                return {}, f"{name} doesn't match its manifest"
            meshes[link] = data
    except (OSError, ValueError, KeyError, TypeError, AttributeError) as error:
        return {}, f"couldn't read the CAD meshes ({error})"
    return meshes, None

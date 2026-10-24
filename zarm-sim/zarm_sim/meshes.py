"""Optional CAD meshes for the 3D view, made locally from Hitbot's STEP by ``scripts/fetch_cad_meshes.py``.

The vendor CAD has no published license, so neither it nor meshes derived from
it are committed: the script writes them to ``zarm-sim/cache/meshes/`` (git
ignores ``cache/``). The view uses them when they are present and were made for
the current URDF; otherwise it draws the URDF's primitives.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Dict, Optional, Tuple

MESH_DIR = Path(__file__).resolve().parents[1] / "cache" / "meshes"
MANIFEST = "manifest.json"
MESH_LINKS = ("base_link", "carriage", "j1_arm", "j2_arm", "flange")
MESH_URL = "/meshes/"


def load_meshes(urdf_bytes: bytes, directory: Path = MESH_DIR) -> Tuple[Dict[str, bytes], Optional[str]]:
    """({link: binary STL in metres, in the link's frame}, note). Empty, with the reason, if unusable."""
    manifest_path = directory / MANIFEST
    if not manifest_path.is_file():
        return {}, None
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if manifest.get("format") != 1:
            return {}, f"{manifest_path} has an unknown format; re-run scripts/fetch_cad_meshes.py"
        if manifest.get("urdf_sha256") != hashlib.sha256(urdf_bytes).hexdigest():
            return {}, "the CAD meshes were made for a different model; re-run scripts/fetch_cad_meshes.py"
        meshes = {}
        for link in MESH_LINKS:
            entry = manifest["links"][link]
            name = Path(entry["file"]).name
            data = (directory / name).read_bytes()
            if hashlib.sha256(data).hexdigest() != entry["sha256"]:
                return {}, f"{name} doesn't match its manifest; re-run scripts/fetch_cad_meshes.py"
            meshes[link] = data
    except (OSError, ValueError, KeyError, TypeError, AttributeError) as error:
        return {}, f"couldn't read the CAD meshes ({error}); re-run scripts/fetch_cad_meshes.py"
    return meshes, None

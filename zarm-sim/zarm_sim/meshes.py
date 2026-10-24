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
import struct
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple

SIM_ROOT = Path(__file__).resolve().parents[1]
MESH_DIR = SIM_ROOT / "cache" / "meshes"
FETCH_SCRIPT = SIM_ROOT / "scripts" / "fetch_cad_meshes.py"
MANIFEST = "manifest.json"
MESH_LINKS = ("base_link", "carriage", "j1_arm", "j2_arm", "flange")
MESH_URL = "/meshes/"
# Down the middle of the column's two front slots, 2.8 mm behind its face: x and +/-y in mm, in the robot frame. The
# CAD's front opening is 88 mm wide and its centre strip 52, so the slots run from y = 26 to 44 mm either side. At this
# depth the line meets both ends of the open slot: the bracket's plates below (x 60.25 to 61.75, their top edge
# rounded, so 171.9 here against 172 flat) and the cap's front lip above (x over 61; behind that the cap is recessed).
SLOT_PROBE_MM = (61.2, 35.0)

Point = Tuple[float, float, float]
Triangle = Tuple[Point, Point, Point]


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


def column_lift(low_mm: float, high_mm: float, added_mm: float) -> Callable[[float], float]:
    """Heights in the CAD's column (mm) mapped onto a column ``added_mm`` taller: the slice ``low_mm..high_mm`` is
    stretched evenly, everything above it moves up unchanged, and everything below stays. Adding 0 changes nothing,
    which is the stock column (any stroke up to 318.1 mm)."""
    scale = (high_mm - low_mm + added_mm) / (high_mm - low_mm)

    def lift(z: float) -> float:
        if z <= low_mm:
            return z
        if z <= high_mm:
            return low_mm + (z - low_mm) * scale
        return z + added_mm

    return lift


def stl_triangles(data: bytes) -> List[Triangle]:
    """The triangles of a binary STL."""
    (count,) = struct.unpack_from("<I", data, 80)
    if len(data) != 84 + 50 * count:
        raise ValueError("not a binary STL")
    out = []
    for index in range(count):
        v = struct.unpack_from("<9f", data, 84 + 50 * index + 12)
        out.append(((v[0], v[1], v[2]), (v[3], v[4], v[5]), (v[6], v[7], v[8])))
    return out


def vertical_hits(triangles: Sequence[Triangle], x: float, y: float) -> List[float]:
    """Sorted heights where the vertical line through (x, y) crosses the triangles: where a mesh has material along
    it. Used to measure the column's slot and the carriage in it, in the STEP and in the meshes made from it. A line
    along a shared edge counts for both triangles: STL stores float32, which can open hairline cracks there."""
    hits = set()
    for (ax, ay, az), (bx, by, bz), (cx, cy, cz) in triangles:
        if x < min(ax, bx, cx) or x > max(ax, bx, cx) or y < min(ay, by, cy) or y > max(ay, by, cy):
            continue
        det = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy)
        if abs(det) < 1e-18:
            continue  # a vertical face, seen edge-on
        u = ((by - cy) * (x - cx) + (cx - bx) * (y - cy)) / det
        v = ((cy - ay) * (x - cx) + (ax - cx) * (y - cy)) / det
        if min(u, v, 1.0 - u - v) < -1e-4:
            continue
        hits.add(round(u * az + v * bz + (1.0 - u - v) * cz, 9))
    return sorted(hits)

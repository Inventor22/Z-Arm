# /// script
# requires-python = ">=3.10,<3.14"
# dependencies = ["build123d>=0.9,<0.14"]
# ///
"""Make CAD meshes for the simulator view from Hitbot's own Z-Arm 2442 STEP. Opt-in; run it on your PC.

    uv run zarm-sim/scripts/fetch_cad_meshes.py

Hitbot's download centre publishes the arm's CAD (2442-3D-Diagram.zip) with no license, so neither the CAD nor
anything made from it is committed. This script downloads the zip into zarm-sim/cache/cad/ (git ignores cache/),
checks it against the pinned size and SHA-256, converts the STEP locally with build123d (OpenCascade), and writes one
binary STL per URDF link to zarm-sim/cache/meshes/, in metres and in that link's frame, with a manifest. The
simulator view uses them when they're present and were made for the current URDF; the primitive model stays the
default and always works.

What the script does to the CAD, and checks before trusting it:

* Finds the J1, elbow and J4 axes (coaxial cylinders shared by the parts) and the base plate's four R15 corners.
  It requires the axes 220 and 200 mm apart and J1 133 mm from the base centre, square to the base, as measured
  when the model was made; otherwise it stops.
* Moves everything into the robot frame of zarm_sim/dimensions.py (origin at the centre of the base's underside,
  +X toward J1, +Z up). The CAD's arm sits at J1 = J2 = 0, which the axes confirm; J4 is assumed to be at 0.
* Splits the solids into the URDF's links: the column part's solids at the J1 housing's height go to the carriage,
  the rest to base_link; the J2 part's 84 mm boss on the J4 axis is the flange.
* Stretches the column to the model's height: the CAD is the 593 mm-tall, 210 mm-stroke arm; the model assumes
  696 mm for the 310 mm stroke. Heights between 130 and 470 mm are stretched evenly and everything above moves up,
  so the connector panel and the top stay as drawn. That part is an approximation, not Hitbot's geometry.
"""

from __future__ import annotations

import hashlib
import io
import json
import math
import struct
import sys
import urllib.request
import zipfile
from collections import defaultdict
from pathlib import Path

SIM_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_ROOT))

from zarm_sim.generate import MODELS_DIR, URDF_NAME  # noqa: E402
from zarm_sim.meshes import MANIFEST, MESH_DIR, MESH_LINKS  # noqa: E402
from zarm_sim.model import build_model  # noqa: E402

CAD_URL = "https://www.hitbotrobot.com/wp-content/uploads/2021/09/2442-3D-Diagram.zip"
CAD_SIZE = 543_873
CAD_SHA256 = "845fe21f9e84b228fd19c22c895b4dd9ef723d266b00ede5dbb432d8e9260998"
STEP_SHA256 = "4ca2a31aca98205fdf67f7a3fb3c34005ff4539dc586bf0c9aa7e4ac40f4de29"
CAD_DIR = SIM_ROOT / "cache" / "cad"
CAD_ZIP = CAD_DIR / "2442-3D-Diagram.zip"
CAD_STEP = CAD_DIR / "zarm_2442_v1.0.0.stp"
STEP_COLUMN_HEIGHT_MM = 593.0  # the CAD's column, measured
STRETCH_FROM_MM, STRETCH_TO_MM = 130.0, 470.0
TOLERANCE_MM, ANGULAR_TOLERANCE_RAD = 0.15, 0.25
CHECK_MM = 0.05


def fail(message: str) -> None:
    print(f"Error: {message}", file=sys.stderr)
    raise SystemExit(1)


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def fetch_cad() -> bytes:
    if CAD_ZIP.is_file() and sha256(CAD_ZIP.read_bytes()) == CAD_SHA256:
        return CAD_ZIP.read_bytes()
    print(f"Downloading {CAD_URL} ...")
    with urllib.request.urlopen(CAD_URL, timeout=60) as reply:
        data = reply.read(CAD_SIZE + 1)
    if len(data) != CAD_SIZE or sha256(data) != CAD_SHA256:
        fail(f"the download isn't the pinned file ({len(data)} bytes, sha256 {sha256(data)}); not using it")
    CAD_DIR.mkdir(parents=True, exist_ok=True)
    CAD_ZIP.write_bytes(data)
    return data


def extract_step(zip_bytes: bytes) -> Path:
    if CAD_STEP.is_file() and sha256(CAD_STEP.read_bytes()) == STEP_SHA256:
        return CAD_STEP
    with zipfile.ZipFile(io.BytesIO(zip_bytes)) as archive:
        steps = [info for info in archive.infolist() if info.filename.lower().endswith((".stp", ".step"))]
        if len(steps) != 1:
            fail(f"expected one STEP file in the zip, found {len(steps)}")
        data = archive.read(steps[0])
    if sha256(data) != STEP_SHA256:
        fail("the STEP inside the zip isn't the pinned file")
    CAD_STEP.write_bytes(data)
    return CAD_STEP


def vertical_axes(part):
    """{(y, z) rounded: [(radius, y, z, solid index), ...]} for cylinders along the CAD's X axis (vertical)."""
    from build123d import GeomType

    found = defaultdict(list)
    for index, solid in enumerate(part.solids()):
        for face in solid.faces():
            if face.geom_type != GeomType.CYLINDER:
                continue
            cylinder = face.geom_adaptor().Cylinder()
            direction = cylinder.Axis().Direction()
            if abs(abs(direction.X()) - 1.0) > 1e-6:
                continue
            location = cylinder.Axis().Location()
            found[(round(location.Y(), 1), round(location.Z(), 1))].append(
                (cylinder.Radius(), location.Y(), location.Z(), index))
    return found


def mean_axis(entries):
    return (sum(e[1] for e in entries) / len(entries), sum(e[2] for e in entries) / len(entries))


def close_key(axes, key):
    for candidate in axes:
        if abs(candidate[0] - key[0]) <= 0.2 and abs(candidate[1] - key[1]) <= 0.2:
            return candidate
    return None


def main() -> int:
    model = build_model()
    d = model.dimensions
    urdf_bytes = (MODELS_DIR / URDF_NAME).read_bytes()
    step_path = extract_step(fetch_cad())
    print(f"Reading {step_path.name} (this takes a few seconds) ...")
    from build123d import GeomType, import_step

    cad = import_step(str(step_path))
    parts = {child.label: child for child in cad.children}
    try:
        column = next(part for label, part in parts.items() if label.endswith("5_AllCATPart"))
        upper = parts["02_AllCATPart"]
        lower = parts["101-012-10-03_AllCATPart"]
    except (KeyError, StopIteration):
        fail(f"the CAD's parts aren't the expected three: {sorted(parts)}")

    # The axes, in the CAD's own frame (its X axis points down the column).
    column_axes, upper_axes, lower_axes = vertical_axes(column), vertical_axes(upper), vertical_axes(lower)
    shared = [key for key in upper_axes if close_key(column_axes, key)]
    if not shared:
        fail("no J1 axis shared by the column part and the J1 arm")
    j1 = mean_axis(upper_axes[max(shared, key=lambda k: len(upper_axes[k]))])
    elbow_keys = [key for key in upper_axes if close_key(lower_axes, key)]
    if not elbow_keys:
        fail("no elbow axis shared by the two arms")
    elbow = mean_axis(upper_axes[max(elbow_keys, key=lambda k: len(upper_axes[k]))])
    tool = mean_axis(lower_axes[max(lower_axes, key=lambda k: len(lower_axes[k]))])
    plate = max(column.solids(), key=lambda s: s.bounding_box().size.Y * s.bounding_box().size.Z / s.bounding_box().size.X)
    corners = []
    for face in plate.faces():
        if face.geom_type == GeomType.CYLINDER and abs(face.geom_adaptor().Cylinder().Radius() - 15.0) < 1e-6:
            location = face.geom_adaptor().Cylinder().Axis().Location()
            corners.append((location.Y(), location.Z()))
    if len(corners) != 4:
        fail(f"expected the base plate's four R15 corners, found {len(corners)}")
    centre = (sum(c[0] for c in corners) / 4.0, sum(c[1] for c in corners) / 4.0)
    bottom = plate.bounding_box().max.X  # the mounting surface (the CAD's X points down)
    offset = math.dist(centre, j1)
    forward = ((j1[0] - centre[0]) / offset, (j1[1] - centre[1]) / offset)
    left = (forward[1], -forward[0])
    checks = {
        "J1 to elbow": (math.dist(j1, elbow), d.j1_arm_length_mm),
        "elbow to J4": (math.dist(elbow, tool), d.j2_arm_length_mm),
        "J1 from the base centre": (offset, d.j1_axis_offset_mm),
    }
    for name, (measured, expected) in checks.items():
        print(f"  {name}: {measured:.3f} mm (model {expected:g})")
        if abs(measured - expected) > CHECK_MM:
            fail(f"{name} is {measured:.3f} mm in this CAD, not {expected:g}; it isn't the CAD the model was made from")
    square = max(abs(math.remainder(math.atan2(b[1] - a[1], b[0] - a[0]) - math.atan2(forward[1], forward[0]), math.pi / 2))
                 for a in corners for b in corners if a != b and 150 < math.dist(a, b) < 190)
    if square > math.radians(0.05):
        fail("the J1 axis isn't square to the base plate")

    def to_robot(x, y, z):
        u, v = y - centre[0], z - centre[1]
        return (forward[0] * u + forward[1] * v, left[0] * u + left[1] * v, bottom - x)

    for name, point in (("elbow", elbow), ("J4", tool)):
        rx, ry, _ = to_robot(0.0, *point)
        if abs(ry) > CHECK_MM:
            fail(f"the CAD's arm isn't at J1 = J2 = 0 (the {name} axis is {ry:.3f} mm off the X axis)")

    # Which link each solid belongs to. The J2 part's 84 mm boss on the J4 axis is the flange; its underside is the
    # flange face, which places the arm's height in the CAD.
    groups = defaultdict(list)
    tool_x = to_robot(0.0, *tool)[0]
    flange_face = math.inf
    for solid in lower.solids():
        box = solid.bounding_box()
        cx, cy, _ = to_robot(box.center().X, box.center().Y, box.center().Z)
        boss = abs(cx - tool_x) < 1.0 and abs(cy) < 1.0 and max(box.size.Y, box.size.Z) < d.flange_diameter_mm + 2.0
        groups["flange" if boss else "j2_arm"].append(solid)
        if boss:
            flange_face = min(flange_face, bottom - box.max.X)
    groups["j1_arm"] = list(upper.solids())
    # The column part also holds the J1 housing (at its height in the CAD) and bits of the carriage behind it.
    band = (flange_face + 120.0, flange_face + d.housing_top_above_flange_mm + 10.0)
    for solid in column.solids():
        box = solid.bounding_box()
        z_low, z_high = bottom - box.max.X, bottom - box.min.X
        cx, _, _ = to_robot(box.center().X, box.center().Y, box.center().Z)
        moving = band[0] <= z_low and z_high <= band[1] and cx > 0.0
        groups["carriage" if moving else "base_link"].append(solid)
    if len(groups["flange"]) != 1 or not groups["carriage"]:
        fail("couldn't find the flange or the carriage in the CAD")
    print(f"  flange face in the CAD: {flange_face:.2f} mm above the mounting surface; J1 at x = {to_robot(0.0, *j1)[0]:.3f}")

    # Each link's frame at the CAD's pose (mm, robot frame): every moving frame sits at the flange face's height.
    frames = {"base_link": (0.0, 0.0, 0.0), "carriage": (0.0, 0.0, flange_face)}
    x = 0.0
    for link, joint in (("j1_arm", "j1_shoulder"), ("j2_arm", "j2_elbow"), ("flange", "j4_wrist")):
        x += model.joint(joint).xyz[0] * 1000.0
        frames[link] = (x, 0.0, flange_face)
    stretch = d.overall_height_mm - STEP_COLUMN_HEIGHT_MM
    scale = (STRETCH_TO_MM - STRETCH_FROM_MM + stretch) / (STRETCH_TO_MM - STRETCH_FROM_MM)

    def lift(z: float) -> float:
        if z <= STRETCH_FROM_MM:
            return z
        if z <= STRETCH_TO_MM:
            return STRETCH_FROM_MM + (z - STRETCH_FROM_MM) * scale
        return z + stretch

    MESH_DIR.mkdir(parents=True, exist_ok=True)
    links = {}
    for link in MESH_LINKS:
        ox, oy, oz = frames[link]
        triangles = []
        for solid in groups[link]:
            vertices, faces = solid.tessellate(TOLERANCE_MM, ANGULAR_TOLERANCE_RAD)
            moved = []
            for vertex in vertices:
                rx, ry, rz = to_robot(vertex.X, vertex.Y, vertex.Z)
                if link == "base_link":
                    rz = lift(rz)
                moved.append(((rx - ox) / 1000.0, (ry - oy) / 1000.0, (rz - oz) / 1000.0))
            triangles.extend((moved[a], moved[b], moved[c]) for a, b, c in faces)
        data = binary_stl(triangles, f"Z-Arm 2442 {link}, made locally from Hitbot's STEP; do not redistribute")
        (MESH_DIR / f"{link}.stl").write_bytes(data)
        links[link] = {"file": f"{link}.stl", "sha256": sha256(data), "triangles": len(triangles),
                       "solids": len(groups[link])}
        print(f"  {link}: {len(groups[link])} solids, {len(triangles)} triangles")
    manifest = {
        "format": 1,
        "source": {"url": CAD_URL, "zip_sha256": CAD_SHA256, "step_sha256": STEP_SHA256},
        "urdf_sha256": sha256(urdf_bytes),
        "units": "m, in each URDF link's frame",
        "column_stretch_mm": {"from": STRETCH_FROM_MM, "to": STRETCH_TO_MM, "added": round(stretch, 3)},
        "links": links,
    }
    (MESH_DIR / MANIFEST).write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote {len(links)} meshes to {MESH_DIR}. Restart the simulator view to use them.")
    return 0


def binary_stl(triangles, header: str) -> bytes:
    out = bytearray(header.encode("ascii", "replace")[:80].ljust(80, b" "))
    out += struct.pack("<I", len(triangles))
    for a, b, c in triangles:
        ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
        vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
        nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
        length = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
        out += struct.pack("<12fH", nx / length, ny / length, nz / length, *a, *b, *c, 0)
    return bytes(out)


if __name__ == "__main__":
    sys.exit(main())

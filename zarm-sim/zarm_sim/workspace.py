"""Where the tool can go, in zarm-sdk's Cartesian frame (mm, origin on the J1 axis).

* The reach envelope: tool positions with a J1/J2 solution inside the given
  joint ranges (zarm-sdk's soft limits in the server), in either hand.
* The blind zone (Hitbot's drawing hatches it): positions the joint ranges
  allow, but only with the J2 arm overlapping the column. The SDK doesn't check
  this; the view warns about it. The J1 arm can't reach the column, and the
  J2 arm is always within the column's height, so a plan-view check suffices.

Geometry comes from the model (the J2 arm's primitives and the column box), so
it follows any change to :mod:`zarm_sim.dimensions`.
"""

from __future__ import annotations

import math
import struct
import zlib
from dataclasses import dataclass
from functools import lru_cache
from typing import List, Optional, Sequence, Tuple

from .model import Box, Cylinder, RobotModel

Point = Tuple[float, float]
REACHABLE, UNREACHABLE, BLIND = 1, 0, 2


def _point_rect_distance(px: float, py: float, rect: Tuple[float, float, float, float]) -> float:
    x0, x1, y0, y1 = rect
    dx = max(x0 - px, 0.0, px - x1)
    dy = max(y0 - py, 0.0, py - y1)
    return math.hypot(dx, dy)


def _point_segment_distance(p: Point, a: Point, b: Point) -> float:
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    length_sq = dx * dx + dy * dy
    t = 0.0 if length_sq == 0.0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / length_sq))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def _projections(points: Sequence[Point], axis: Point) -> Tuple[float, float]:
    values = [p[0] * axis[0] + p[1] * axis[1] for p in points]
    return min(values), max(values)


def _quad_rect_distance(quad: Sequence[Point], rect: Tuple[float, float, float, float]) -> float:
    """Distance between a convex quadrilateral and an axis-aligned rectangle; 0 when they overlap."""
    x0, x1, y0, y1 = rect
    corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    axes = [(1.0, 0.0), (0.0, 1.0)]
    for i in range(2):
        ex, ey = quad[i + 1][0] - quad[i][0], quad[i + 1][1] - quad[i][1]
        norm = math.hypot(ex, ey)
        axes.append((-ey / norm, ex / norm))
    separated = False
    for axis in axes:
        a0, a1 = _projections(quad, axis)
        b0, b1 = _projections(corners, axis)
        if a1 < b0 or b1 < a0:
            separated = True
            break
    if not separated:
        return 0.0
    best = min(_point_rect_distance(px, py, rect) for px, py in quad)
    for corner in corners:
        for i in range(4):
            best = min(best, _point_segment_distance(corner, quad[i], quad[(i + 1) % 4]))
    return best


@dataclass(frozen=True)
class Workspace:
    l1_mm: float
    l2_mm: float
    j1_range_deg: Tuple[float, float]
    j2_range_deg: Tuple[float, float]
    column_rect_mm: Tuple[float, float, float, float]  # x0, x1, y0, y1 in the SDK frame
    j2_circles_mm: Tuple[Tuple[float, float], ...]  # (distance along the J2 arm from the elbow, radius)
    j2_box_mm: Tuple[float, float, float]  # (start, end) along the arm, half width

    @classmethod
    def from_model(
        cls,
        model: RobotModel,
        j1_range_deg: Optional[Tuple[float, float]] = None,
        j2_range_deg: Optional[Tuple[float, float]] = None,
    ) -> "Workspace":
        """The model's workspace; joint ranges default to the hardware ranges in the model."""
        d = model.dimensions
        j1 = model.joint("j1_shoulder")
        column = next(part for part in model.link("base_link").parts if part.name == "column")
        assert isinstance(column.shape, Box)
        half_x, half_y = column.shape.size[0] / 2.0 * 1000.0, column.shape.size[1] / 2.0 * 1000.0
        cx, cy = column.xyz[0] * 1000.0 - j1.xyz[0] * 1000.0, column.xyz[1] * 1000.0
        circles = []
        box = None
        for part in model.link("j2_arm").parts:
            if not part.collision:
                continue
            if isinstance(part.shape, Cylinder):
                circles.append((part.xyz[0] * 1000.0, part.shape.radius * 1000.0))
            elif isinstance(part.shape, Box):
                half_len = part.shape.size[0] / 2.0 * 1000.0
                box = (part.xyz[0] * 1000.0 - half_len, part.xyz[0] * 1000.0 + half_len, part.shape.size[1] / 2.0 * 1000.0)
        assert box is not None
        return cls(
            l1_mm=d.j1_arm_length_mm,
            l2_mm=d.j2_arm_length_mm,
            j1_range_deg=j1_range_deg if j1_range_deg is not None else (-d.j1_range_deg, d.j1_range_deg),
            j2_range_deg=j2_range_deg if j2_range_deg is not None else (-d.j2_range_deg, d.j2_range_deg),
            column_rect_mm=(cx - half_x, cx + half_x, cy - half_y, cy + half_y),
            j2_circles_mm=tuple(circles),
            j2_box_mm=box,
        )

    # Kinematics (the SDK's conventions: J1 = J2 = 0 along +X, counter-clockwise positive)

    def solutions(self, x: float, y: float) -> List[Tuple[float, float]]:
        """(J1, J2) in degrees, inside the joint ranges, that put the tool at (x, y)."""
        l1, l2 = self.l1_mm, self.l2_mm
        cos2 = (x * x + y * y - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
        if not -1.0 - 1e-12 <= cos2 <= 1.0 + 1e-12:
            return []
        cos2 = max(-1.0, min(1.0, cos2))
        found = []
        for sign in (1.0, -1.0):
            sin2 = sign * math.sqrt(max(0.0, 1.0 - cos2 * cos2))
            j2 = math.degrees(math.atan2(sin2, cos2))
            j1 = math.degrees(math.atan2(y, x) - math.atan2(l2 * sin2, l1 + l2 * cos2))
            j1 = (j1 + 180.0) % 360.0 - 180.0
            if self._inside(j1, j2) and (j1, j2) not in found:
                found.append((j1, j2))
        return found

    def _inside(self, j1: float, j2: float, margin: float = 1e-9) -> bool:
        (a1, b1), (a2, b2) = self.j1_range_deg, self.j2_range_deg
        return a1 - margin <= j1 <= b1 + margin and a2 - margin <= j2 <= b2 + margin

    def j2_clearance_mm(self, j1_deg: float, j2_deg: float) -> float:
        """Plan-view gap between the J2 arm and the column; 0 or less means they overlap."""
        a1 = math.radians(j1_deg)
        a12 = math.radians(j1_deg + j2_deg)
        ex, ey = self.l1_mm * math.cos(a1), self.l1_mm * math.sin(a1)
        ux, uy = math.cos(a12), math.sin(a12)
        best = math.inf
        for along, radius in self.j2_circles_mm:
            best = min(best, _point_rect_distance(ex + along * ux, ey + along * uy, self.column_rect_mm) - radius)
        start, end, half = self.j2_box_mm
        nx, ny = -uy, ux
        quad = [
            (ex + start * ux + half * nx, ey + start * uy + half * ny),
            (ex + end * ux + half * nx, ey + end * uy + half * ny),
            (ex + end * ux - half * nx, ey + end * uy - half * ny),
            (ex + start * ux - half * nx, ey + start * uy - half * ny),
        ]
        return min(best, _quad_rect_distance(quad, self.column_rect_mm))

    def classify(self, x: float, y: float) -> int:
        found = self.solutions(x, y)
        if not found:
            return UNREACHABLE
        return REACHABLE if any(self.j2_clearance_mm(j1, j2) > 0.0 for j1, j2 in found) else BLIND

    # Pictures

    def bounds_mm(self) -> Tuple[float, float, float, float]:
        reach = self.l1_mm + self.l2_mm + 8.0
        return (-reach, reach, -reach, reach)

    def raster(self, step_mm: float = 4.0) -> Tuple[int, int, bytes]:
        """(width, height, classes) on a grid over bounds_mm(); row 0 is the largest y."""
        x0, x1, y0, y1 = self.bounds_mm()
        width, height = int(round((x1 - x0) / step_mm)), int(round((y1 - y0) / step_mm))
        cells = bytearray(width * height)
        for row in range(height):
            y = y1 - (row + 0.5) * step_mm
            for col in range(width):
                cells[row * width + col] = self.classify(x0 + (col + 0.5) * step_mm, y)
        return width, height, bytes(cells)

    def _arcs(self) -> List[List[Point]]:
        """Each hand's boundary as four arcs: J2 = 0, J1 = max, J2 = limit, J1 = min (the union's outline is a subset)."""
        (a1, b1), (a2, b2) = self.j1_range_deg, self.j2_range_deg
        l1, l2 = self.l1_mm, self.l2_mm

        def tool(j1: float, j2: float) -> Point:
            r1, r12 = math.radians(j1), math.radians(j1 + j2)
            return (l1 * math.cos(r1) + l2 * math.cos(r12), l1 * math.sin(r1) + l2 * math.sin(r12))

        def sweep(fixed_j1: Optional[float], fixed_j2: Optional[float], start: float, end: float) -> List[Point]:
            count = max(8, int(abs(end - start) * math.pi / 180.0 * (l1 + l2)))  # about 1 mm apart or closer
            points = []
            for k in range(count + 1):
                value = start + (end - start) * k / count
                points.append(tool(value, fixed_j2) if fixed_j2 is not None else tool(fixed_j1, value))  # type: ignore[arg-type]
            return points

        arcs = [sweep(None, 0.0, a1, b1)]
        for limit in (b2, a2):
            if limit == 0.0:
                continue
            arcs.append(sweep(b1, None, 0.0, limit))
            arcs.append(sweep(None, limit, b1, a1))
            arcs.append(sweep(a1, None, limit, 0.0))
        return arcs

    def outline(self, probe_mm: float = 0.25, min_length_mm: float = 3.0) -> List[List[Point]]:
        """Polylines (mm) along the envelope's edge: arc points with an unreachable point within probe_mm."""
        directions = [(math.cos(k * math.pi / 4.0), math.sin(k * math.pi / 4.0)) for k in range(8)]

        def on_edge(p: Point) -> bool:
            return any(not self.solutions(p[0] + probe_mm * dx, p[1] + probe_mm * dy) for dx, dy in directions)

        def keep(line: List[Point]) -> bool:
            return sum(math.dist(a, b) for a, b in zip(line, line[1:])) >= min_length_mm

        lines: List[List[Point]] = []
        for arc in self._arcs():
            current: List[Point] = []
            for point in arc:
                if on_edge(point):
                    current.append(point)
                    continue
                if keep(current):
                    lines.append(current)
                current = []
            if keep(current):
                lines.append(current)
        return lines


def png_rgba(width: int, height: int, rgba_rows: bytes) -> bytes:
    """A minimal PNG (8-bit RGBA, no filtering) from width*height*4 bytes, top row first."""
    stride = width * 4
    raw = b"".join(b"\x00" + rgba_rows[row * stride:(row + 1) * stride] for row in range(height))

    def chunk(kind: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF)

    header = struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0)
    return b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", header) + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b"")


REACH_RGBA = (170, 166, 157, 30)  # Printed Grey: amber is kept for motion and goals
BLIND_RGBA = (255, 110, 96, 150)
BLIND_HATCH_RGBA = (255, 110, 96, 60)


@lru_cache(maxsize=8)
def envelope_png(workspace: Workspace, step_mm: float = 4.0) -> bytes:
    """The envelope as a picture for the floor: reachable cells tinted, the blind zone hatched."""
    width, height, cells = workspace.raster(step_mm)
    out = bytearray(width * height * 4)
    clear = REACH_RGBA[:3] + (0,)  # transparent but with the tint's colour, so filtering leaves no dark fringe
    for row in range(height):
        for col in range(width):
            kind = cells[row * width + col]
            if kind == REACHABLE:
                colour = REACH_RGBA
            elif kind == BLIND:
                colour = BLIND_RGBA if (row + col) % 4 == 0 else BLIND_HATCH_RGBA
            else:
                colour = clear
            index = (row * width + col) * 4
            out[index:index + 4] = bytes(colour)
    return png_rgba(width, height, bytes(out))


@lru_cache(maxsize=8)
def envelope_outline(workspace: Workspace) -> Tuple[Tuple[Point, ...], ...]:
    return tuple(tuple((round(x, 2), round(y, 2)) for x, y in line) for line in workspace.outline())

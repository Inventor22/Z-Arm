"""Planar kinematics for the Z-Arm's J1/J2 links.

Assumed conventions (not verified on hardware; see the README's "Default
assumptions"):

* J1 = J2 = 0 puts the arm fully extended along +X, and positive angles turn
  counter-clockwise seen from above. This is the usual SCARA convention, and
  nothing in the legacy C# code contradicts it.
* ``l1_mm`` is the J1 (shoulder) link and ``l2_mm`` the J2 (elbow) link: 220 mm
  and 200 mm on a Z-Arm 2442, per Hitbot's spec sheet. These are physical
  lengths, not the arguments of ``robot_set_arm_length``.
* Z is 0 at the top of travel and negative going down, as in the legacy slider
  app (-310 to 0) and the jjh1214/hitbot_sim URDF.
* ``r`` is the J4 joint angle relative to the forearm (the vendor header calls
  it "angle value of joint 4"), so the tool's yaw in the base frame is
  J1 + J2 + r.

The controller does its own kinematics; this module serves the simulator, the
SDK's reach checks and motion-time estimates.
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple


def forward_xy(j1_deg: float, j2_deg: float, l1_mm: float, l2_mm: float) -> Tuple[float, float]:
    a1 = math.radians(j1_deg)
    a12 = math.radians(j1_deg + j2_deg)
    return (l1_mm * math.cos(a1) + l2_mm * math.cos(a12), l1_mm * math.sin(a1) + l2_mm * math.sin(a12))


def tool_yaw_deg(j1_deg: float, j2_deg: float, r_deg: float) -> float:
    """The tool's rotation about Z in the base frame, assuming ``r`` is relative to the forearm."""
    return j1_deg + j2_deg + r_deg


def reach_mm(l1_mm: float, l2_mm: float) -> Tuple[float, float]:
    """(minimum, maximum) planar distance from the J1 axis that the tool can reach."""
    return (abs(l1_mm - l2_mm), l1_mm + l2_mm)


def _wrap_deg(angle: float) -> float:
    wrapped = math.fmod(angle + 180.0, 360.0)
    if wrapped <= 0.0:
        wrapped += 360.0
    return wrapped - 180.0


def inverse_xy(x_mm: float, y_mm: float, l1_mm: float, l2_mm: float, elbow: int) -> Optional[Tuple[float, float]]:
    """(J1, J2) in degrees for ``elbow`` +1 (J2 >= 0) or -1 (J2 <= 0); None if unreachable."""
    cos2 = (x_mm * x_mm + y_mm * y_mm - l1_mm * l1_mm - l2_mm * l2_mm) / (2.0 * l1_mm * l2_mm)
    if cos2 > 1.0 + 1e-9 or cos2 < -1.0 - 1e-9:
        return None
    cos2 = max(-1.0, min(1.0, cos2))
    sin2 = (1.0 if elbow >= 0 else -1.0) * math.sqrt(max(0.0, 1.0 - cos2 * cos2))
    j2 = math.atan2(sin2, cos2)
    j1 = math.atan2(y_mm, x_mm) - math.atan2(l2_mm * sin2, l1_mm + l2_mm * cos2)
    return (_wrap_deg(math.degrees(j1)), math.degrees(j2))


def inverse_xy_solutions(x_mm: float, y_mm: float, l1_mm: float, l2_mm: float) -> List[Tuple[float, float]]:
    solutions = []
    for elbow in (1, -1):
        solution = inverse_xy(x_mm, y_mm, l1_mm, l2_mm, elbow)
        if solution is not None and solution not in solutions:
            solutions.append(solution)
    return solutions


def elbow_of(j2_deg: float) -> int:
    return -1 if j2_deg < 0.0 else 1


def alternate_hand(j1_deg: float, j2_deg: float, l1_mm: float, l2_mm: float) -> Tuple[float, float]:
    """The other (J1, J2) solution that reaches the same tool position."""
    x, y = forward_xy(j1_deg, j2_deg, l1_mm, l2_mm)
    solution = inverse_xy(x, y, l1_mm, l2_mm, -elbow_of(j2_deg))
    if solution is None:
        return (j1_deg, j2_deg)
    return solution


def segment_distance_to_origin(x0: float, y0: float, x1: float, y1: float) -> float:
    """Smallest distance from the J1 axis to the segment (x0, y0)-(x1, y1)."""
    dx, dy = x1 - x0, y1 - y0
    length_sq = dx * dx + dy * dy
    if length_sq == 0.0:
        return math.hypot(x0, y0)
    t = max(0.0, min(1.0, -(x0 * dx + y0 * dy) / length_sq))
    return math.hypot(x0 + t * dx, y0 + t * dy)

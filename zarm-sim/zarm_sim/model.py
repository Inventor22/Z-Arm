"""The Z-Arm as links, joints and primitive shapes, in SI units.

One parametric source for both writers: :mod:`zarm_sim.urdf` and
:mod:`zarm_sim.mjcf` only translate this model. Frames:

* ``base_link``: the robot frame of :mod:`zarm_sim.dimensions` (origin at the
  centre of the base's underside, +Z up, +X toward the arm).
* ``carriage``: J3, a prismatic joint on the column axis. Its frame is at the
  flange face's height, so J3 = 0 is the top of travel and J3 < 0 is lower.
* ``j1_arm``, ``j2_arm``, ``flange``: J1, J2 and J4 turn about vertical axes,
  every frame at the flange face's height. J1 = J2 = 0 points the arm along +X;
  positive angles turn counter-clockwise seen from above. J4 (the vendor's
  ``r``) is relative to the J2 arm, so the tool's yaw is J1 + J2 + J4.
* ``tool0``: the centre of the flange face.
* ``sdk_origin``: the frame zarm-sdk's Cartesian poses are assumed to be in: on
  the J1 axis at the flange face's height at the top of travel. There,
  tool0 = (forward_xy(J1, J2), J3) and yaw = J1 + J2 + J4.

Shapes are axis-aligned boxes and cylinders (along their part's Z). Visual-only
parts (``collision=False``) mark features that have no physical extent worth
colliding with, and a J4 index tab that exists only to make J4 visible.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple, Union

from .dimensions import ZArmDimensions

Vec3 = Tuple[float, float, float]
MM = 0.001


@dataclass(frozen=True)
class Box:
    size: Vec3

    def volume(self) -> float:
        return self.size[0] * self.size[1] * self.size[2]

    def principal_inertia(self, mass: float) -> Vec3:
        x, y, z = self.size
        return (mass * (y * y + z * z) / 12.0, mass * (x * x + z * z) / 12.0, mass * (x * x + y * y) / 12.0)


@dataclass(frozen=True)
class Cylinder:
    radius: float
    length: float  # along the part's Z axis

    def volume(self) -> float:
        return math.pi * self.radius * self.radius * self.length

    def principal_inertia(self, mass: float) -> Vec3:
        r, h = self.radius, self.length
        side = mass * (3.0 * r * r + h * h) / 12.0
        return (side, side, mass * r * r / 2.0)


Shape = Union[Box, Cylinder]


@dataclass(frozen=True)
class Part:
    name: str
    shape: Shape
    xyz: Vec3
    material: str
    collision: bool = True


@dataclass(frozen=True)
class Inertial:
    mass: float
    com: Vec3
    # Inertia tensor entries about the centre of mass: ixx, ixy, ixz, iyy, iyz, izz.
    inertia: Tuple[float, float, float, float, float, float]


@dataclass(frozen=True)
class Link:
    name: str
    description: str
    parts: Tuple[Part, ...] = ()
    inertial: Optional[Inertial] = None


@dataclass(frozen=True)
class Joint:
    name: str
    kind: str  # "prismatic", "revolute" or "fixed"
    parent: str
    child: str
    xyz: Vec3
    axis: Vec3 = (0.0, 0.0, 1.0)
    lower: float = 0.0
    upper: float = 0.0
    velocity: float = 0.0
    effort: float = 0.0
    sdk_field: Optional[str] = None  # the zarm.JointPositions field this joint carries
    sdk_scale: float = 1.0  # model value (rad or m) = SDK value (deg or mm) * sdk_scale


@dataclass(frozen=True)
class Material:
    name: str
    rgba: Tuple[float, float, float, float]


@dataclass(frozen=True)
class RobotModel:
    name: str
    dimensions: ZArmDimensions
    materials: Tuple[Material, ...]
    links: Tuple[Link, ...]
    joints: Tuple[Joint, ...]

    def link(self, name: str) -> Link:
        for link in self.links:
            if link.name == name:
                return link
        raise KeyError(name)

    def joint(self, name: str) -> Joint:
        for joint in self.joints:
            if joint.name == name:
                return joint
        raise KeyError(name)

    def movable_joints(self) -> Tuple[Joint, ...]:
        """Movable joints in tree order (J3, J1, J2, J4)."""
        return tuple(joint for joint in self.joints if joint.kind != "fixed")

    def sdk_joints(self) -> Tuple[Joint, ...]:
        """Movable joints in the SDK's order (J1, J2, J3, J4)."""
        return tuple(sorted(self.movable_joints(), key=lambda joint: SDK_FIELDS.index(joint.sdk_field or "")))


SDK_FIELDS = ("j1_deg", "j2_deg", "j3_mm", "j4_deg")

# Visual only: approximate colours. The writers prefix the names ("white").
MATERIALS = (
    Material("white", (0.92, 0.92, 0.9, 1.0)),
    Material("column", (0.8, 0.81, 0.82, 1.0)),
    Material("dark", (0.22, 0.23, 0.25, 1.0)),
    Material("trim", (0.36, 0.37, 0.39, 1.0)),
    Material("metal", (0.62, 0.64, 0.66, 1.0)),
    Material("hole", (0.06, 0.06, 0.07, 1.0)),
    Material("index", (0.95, 0.67, 0.24, 1.0)),
)


def _inertial(parts: Iterable[Part], mass: float) -> Inertial:
    """Mass spread over the link's collision parts at uniform density (overlaps count twice)."""
    solids = [part for part in parts if part.collision]
    volume = sum(part.shape.volume() for part in solids)
    com = tuple(sum(part.xyz[i] * part.shape.volume() for part in solids) / volume for i in range(3))
    tensor = [[0.0] * 3 for _ in range(3)]
    for part in solids:
        m = mass * part.shape.volume() / volume
        principal = part.shape.principal_inertia(m)
        d = [part.xyz[i] - com[i] for i in range(3)]
        dd = sum(v * v for v in d)
        for i in range(3):
            for j in range(3):
                tensor[i][j] += (principal[i] if i == j else 0.0) + m * ((dd if i == j else 0.0) - d[i] * d[j])
    return Inertial(
        mass,
        (com[0], com[1], com[2]),
        (tensor[0][0], tensor[0][1], tensor[0][2], tensor[1][1], tensor[1][2], tensor[2][2]),
    )


def _link(name: str, description: str, parts: Sequence[Part], mass: Optional[float]) -> Link:
    return Link(name, description, tuple(parts), None if mass is None else _inertial(parts, mass))


def _between(low_mm: float, high_mm: float) -> Tuple[float, float]:
    """(centre, length) in metres of the span low..high mm."""
    return ((low_mm + high_mm) / 2.0 * MM, (high_mm - low_mm) * MM)


def build_model(dims: Optional[ZArmDimensions] = None) -> RobotModel:
    d = dims if dims is not None else ZArmDimensions()
    d.validate()
    stack = d.stack_mm()
    height = d.overall_height_mm
    j1_x = d.j1_axis_offset_mm * MM
    l1, l2 = d.j1_arm_length_mm * MM, d.j2_arm_length_mm * MM

    plate_t = d.base_plate_thickness_mm
    column_z, column_h = _between(plate_t, height - d.column_cap_height_mm)
    cap_z, cap_h = _between(height - d.column_cap_height_mm, height)
    half_holes = d.mount_hole_spacing_mm / 2.0 * MM
    base_parts: List[Part] = [
        Part("base_plate", Box((d.base_plate_size_mm * MM, d.base_plate_size_mm * MM, plate_t * MM)),
             (0.0, 0.0, plate_t / 2.0 * MM), "dark"),
        Part("column", Box((d.column_depth_mm * MM, d.column_width_mm * MM, column_h)), (0.0, 0.0, column_z),
             "column"),
        Part("column_cap", Box((d.column_depth_mm * MM, d.column_width_mm * MM, cap_h)), (0.0, 0.0, cap_z),
             "trim"),
    ]
    for index, (sx, sy) in enumerate(((1, 1), (-1, 1), (-1, -1), (1, -1)), start=1):
        base_parts.append(Part(
            f"mount_hole_{index}", Cylinder(d.mount_counterbore_diameter_mm / 2.0 * MM, 0.5 * MM),
            (sx * half_holes, sy * half_holes, (plate_t + 0.25) * MM), "hole", collision=False,
        ))

    housing_z, housing_h = _between(*stack["housing"])
    column_front = d.column_depth_mm / 2.0 * MM
    housing_parts = [
        Part("housing", Box((j1_x - column_front, d.housing_width_mm * MM, housing_h)),
             ((column_front + j1_x) / 2.0, 0.0, housing_z), "white"),
        Part("housing_nose", Cylinder(d.housing_width_mm / 2.0 * MM, housing_h), (j1_x, 0.0, housing_z), "white"),
    ]

    j1_z, j1_h = _between(*stack["j1_arm"])
    r1 = d.j1_arm_end_radius_mm * MM
    j1_parts = [
        Part("j1_arm_body", Box((l1, 2.0 * r1, j1_h)), (l1 / 2.0, 0.0, j1_z), "white"),
        Part("j1_arm_shoulder_end", Cylinder(r1, j1_h), (0.0, 0.0, j1_z), "white"),
        Part("j1_arm_elbow_end", Cylinder(r1, j1_h), (l1, 0.0, j1_z), "white"),
    ]

    j2_z, j2_h = _between(*stack["j2_arm"])
    r2a, r2b = d.j2_arm_elbow_radius_mm * MM, d.j2_arm_tool_radius_mm * MM
    j2_parts = [
        # The arm tapers from R53 to R47.5; the box takes the mean width.
        Part("j2_arm_body", Box((l2, r2a + r2b, j2_h)), (l2 / 2.0, 0.0, j2_z), "white"),
        Part("j2_arm_elbow_end", Cylinder(r2a, j2_h), (0.0, 0.0, j2_z), "white"),
        Part("j2_arm_tool_end", Cylinder(r2b, j2_h), (l2, 0.0, j2_z), "white"),
    ]

    flange_z, flange_h = _between(*stack["flange"])
    flange_r = d.flange_diameter_mm / 2.0 * MM
    tab_inner, tab_outer = 35.0 * MM, d.j2_arm_tool_radius_mm * MM + 12.5 * MM
    flange_parts = [
        Part("flange_disc", Cylinder(flange_r, flange_h), (0.0, 0.0, flange_z), "metal"),
        # Not on the arm: a tab along the flange's +X that makes J4 visible past the J2 arm's end.
        Part("flange_index", Box((tab_outer - tab_inner, 6.0 * MM, flange_h)),
             ((tab_inner + tab_outer) / 2.0, 0.0, flange_z), "index", collision=False),
    ]

    links = (
        _link("base_link", "Base plate and column (fixed)", base_parts, d.base_mass_kg),
        _link("carriage", "Z carriage with the J1 housing; moves with J3", housing_parts, d.carriage_mass_kg),
        _link("j1_arm", "J1 arm (shoulder link), 220 mm between axes", j1_parts, d.j1_arm_mass_kg),
        _link("j2_arm", "J2 arm (elbow link), 200 mm between axes", j2_parts, d.j2_arm_mass_kg),
        _link("flange", "J4 output flange (the vendor's r axis)", flange_parts, d.flange_mass_kg),
        _link("tool0", "Centre of the flange face; tool frame", (), None),
        _link("sdk_origin", "Frame of zarm-sdk's Cartesian poses (assumed): J1 axis, flange face at J3 = 0", (), None),
    )
    top = d.flange_face_at_top_mm * MM
    deg = math.pi / 180.0
    joints = (
        Joint("j3_vertical", "prismatic", "base_link", "carriage", (0.0, 0.0, top),
              lower=-d.z_stroke_mm * MM, upper=0.0, velocity=d.j3_velocity_m_s, effort=d.j3_effort_n,
              sdk_field="j3_mm", sdk_scale=MM),
        Joint("j1_shoulder", "revolute", "carriage", "j1_arm", (j1_x, 0.0, 0.0),
              lower=-d.j1_range_deg * deg, upper=d.j1_range_deg * deg, velocity=d.j1_j2_velocity_rad_s,
              effort=d.j1_j2_effort_nm, sdk_field="j1_deg", sdk_scale=deg),
        Joint("j2_elbow", "revolute", "j1_arm", "j2_arm", (l1, 0.0, 0.0),
              lower=-d.j2_range_deg * deg, upper=d.j2_range_deg * deg, velocity=d.j1_j2_velocity_rad_s,
              effort=d.j1_j2_effort_nm, sdk_field="j2_deg", sdk_scale=deg),
        Joint("j4_wrist", "revolute", "j2_arm", "flange", (l2, 0.0, 0.0),
              lower=-d.j4_range_deg * deg, upper=d.j4_range_deg * deg, velocity=d.j4_velocity_rad_s,
              effort=d.j4_effort_nm, sdk_field="j4_deg", sdk_scale=deg),
        Joint("flange_tool0", "fixed", "flange", "tool0", (0.0, 0.0, 0.0)),
        Joint("base_sdk_origin", "fixed", "base_link", "sdk_origin", (j1_x, 0.0, top)),
    )
    return RobotModel("zarm_2442", d, MATERIALS, links, joints)


def sdk_to_model(model: RobotModel, sdk: Mapping[str, float]) -> Dict[str, float]:
    """zarm-sdk joint values ({"j1_deg": ..., "j3_mm": ...}) as model joint values (rad, m) by joint name."""
    return {joint.name: float(sdk[joint.sdk_field]) * joint.sdk_scale for joint in model.sdk_joints()}  # type: ignore[index]


def model_to_sdk(model: RobotModel, values: Mapping[str, float]) -> Dict[str, float]:
    """Model joint values (rad, m) by joint name as zarm-sdk joint values (deg, mm)."""
    return {joint.sdk_field: float(values[joint.name]) / joint.sdk_scale for joint in model.sdk_joints()}  # type: ignore[misc]

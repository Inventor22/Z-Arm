"""Forward kinematics over a URDF, with the standard library only.

The tests use it to check the generated URDF against zarm-sdk's kinematics, and
the server uses it to compare the model with the poses the SDK reports.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Mapping, Sequence, Tuple, Union

Matrix = Tuple[Tuple[float, float, float, float], ...]  # 4x4, row-major

IDENTITY: Matrix = ((1.0, 0.0, 0.0, 0.0), (0.0, 1.0, 0.0, 0.0), (0.0, 0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def matmul(a: Matrix, b: Matrix) -> Matrix:
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)) for i in range(4))  # type: ignore[return-value]


def inverse(t: Matrix) -> Matrix:
    """Inverse of a rigid transform."""
    r = [[t[j][i] for j in range(3)] for i in range(3)]  # transpose of the rotation
    p = [-sum(r[i][k] * t[k][3] for k in range(3)) for i in range(3)]
    return (
        (r[0][0], r[0][1], r[0][2], p[0]),
        (r[1][0], r[1][1], r[1][2], p[1]),
        (r[2][0], r[2][1], r[2][2], p[2]),
        (0.0, 0.0, 0.0, 1.0),
    )


def transform(xyz: Sequence[float], rotation: Sequence[Sequence[float]]) -> Matrix:
    return (
        (rotation[0][0], rotation[0][1], rotation[0][2], xyz[0]),
        (rotation[1][0], rotation[1][1], rotation[1][2], xyz[1]),
        (rotation[2][0], rotation[2][1], rotation[2][2], xyz[2]),
        (0.0, 0.0, 0.0, 1.0),
    )


def rpy_matrix(roll: float, pitch: float, yaw: float) -> List[List[float]]:
    """URDF roll-pitch-yaw: fixed-axis X, Y, Z rotations, R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def axis_angle_matrix(axis: Sequence[float], angle: float) -> List[List[float]]:
    x, y, z = axis
    norm = math.sqrt(x * x + y * y + z * z)
    x, y, z = x / norm, y / norm, z / norm
    c, s = math.cos(angle), math.sin(angle)
    k = 1.0 - c
    return [
        [c + x * x * k, x * y * k - z * s, x * z * k + y * s],
        [y * x * k + z * s, c + y * y * k, y * z * k - x * s],
        [z * x * k - y * s, z * y * k + x * s, c + z * z * k],
    ]


@dataclass(frozen=True)
class UrdfJoint:
    name: str
    kind: str
    parent: str
    child: str
    origin: Matrix
    axis: Tuple[float, float, float]
    lower: float
    upper: float

    def motion(self, value: float) -> Matrix:
        if self.kind in ("revolute", "continuous"):
            return transform((0.0, 0.0, 0.0), axis_angle_matrix(self.axis, value))
        if self.kind == "prismatic":
            return transform(tuple(a * value for a in self.axis), rpy_matrix(0.0, 0.0, 0.0))
        return IDENTITY


def _floats(text: str, count: int) -> Tuple[float, ...]:
    values = tuple(float(part) for part in text.split())
    if len(values) != count:
        raise ValueError(f"expected {count} numbers, got {text!r}")
    return values


class UrdfRobot:
    """A URDF's tree of joints, enough for forward kinematics."""

    def __init__(self, source: Union[str, Path]) -> None:
        text = Path(source).read_text(encoding="utf-8") if isinstance(source, Path) else source
        root = ET.fromstring(text)
        if root.tag != "robot":
            raise ValueError("not a URDF: the root element isn't <robot>")
        self.name = root.get("name", "")
        self.links = [link.get("name", "") for link in root.findall("link")]
        if len(set(self.links)) != len(self.links):
            raise ValueError("duplicate link names")
        self.joints: Dict[str, UrdfJoint] = {}
        for element in root.findall("joint"):
            origin = element.find("origin")
            xyz = _floats(origin.get("xyz", "0 0 0"), 3) if origin is not None else (0.0, 0.0, 0.0)
            rpy = _floats(origin.get("rpy", "0 0 0"), 3) if origin is not None else (0.0, 0.0, 0.0)
            axis_element = element.find("axis")
            axis = _floats(axis_element.get("xyz", "1 0 0"), 3) if axis_element is not None else (1.0, 0.0, 0.0)
            limit = element.find("limit")
            joint = UrdfJoint(
                name=element.get("name", ""),
                kind=element.get("type", ""),
                parent=element.find("parent").get("link", ""),  # type: ignore[union-attr]
                child=element.find("child").get("link", ""),  # type: ignore[union-attr]
                origin=transform(xyz, rpy_matrix(*rpy)),
                axis=(axis[0], axis[1], axis[2]),
                lower=float(limit.get("lower", "0")) if limit is not None else 0.0,
                upper=float(limit.get("upper", "0")) if limit is not None else 0.0,
            )
            if joint.name in self.joints:
                raise ValueError(f"duplicate joint name {joint.name}")
            self.joints[joint.name] = joint
        children = {joint.child for joint in self.joints.values()}
        unknown = {name for joint in self.joints.values() for name in (joint.parent, joint.child)} - set(self.links)
        if unknown:
            raise ValueError(f"joints refer to undefined links: {sorted(unknown)}")
        roots = [link for link in self.links if link not in children]
        if len(roots) != 1:
            raise ValueError(f"expected one root link, found {roots}")
        if len(children) != len(self.joints):
            raise ValueError("a link has more than one parent joint")
        self.root = roots[0]

    def movable(self) -> List[UrdfJoint]:
        return [joint for joint in self.joints.values() if joint.kind != "fixed"]

    def link_poses(self, values: Mapping[str, float]) -> Dict[str, Matrix]:
        """Every link's pose in the root link's frame, for joint values by name (rad or m; missing ones are 0)."""
        poses: Dict[str, Matrix] = {self.root: IDENTITY}
        pending = [self.root]
        while pending:
            parent = pending.pop()
            for joint in self.joints.values():
                if joint.parent == parent:
                    local = matmul(joint.origin, joint.motion(float(values.get(joint.name, 0.0))))
                    poses[joint.child] = matmul(poses[parent], local)
                    pending.append(joint.child)
        return poses

    def relative(self, values: Mapping[str, float], frame: str, base: str) -> Matrix:
        """The pose of ``frame`` in ``base``."""
        poses = self.link_poses(values)
        return matmul(inverse(poses[base]), poses[frame])


def xyz_yaw(t: Matrix) -> Tuple[float, float, float, float]:
    """Position and the rotation about Z (rad) of a transform whose Z axis is vertical."""
    return (t[0][3], t[1][3], t[2][3], math.atan2(t[1][0], t[0][0]))

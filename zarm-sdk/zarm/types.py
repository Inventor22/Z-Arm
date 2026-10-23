"""Typed values for the Z-Arm SDK.

Units are part of every field name. J1, J2 and J4 (and the tool rotation ``r``)
are in degrees; J3 and Cartesian coordinates are in millimetres. Those are the
units Hitbot's native library uses, so values cross the vendor boundary without
conversion. The ``to_si()`` helpers convert to radians and metres.
"""

from __future__ import annotations

import enum
import math
import numbers
import struct
from dataclasses import dataclass
from typing import List, Optional, Tuple

from .errors import InvalidArgumentError

FLOAT32_MAX = 3.4028234663852886e38
C_INT_MIN, C_INT_MAX = -(2**31), 2**31 - 1


class JointKind(enum.Enum):
    REVOLUTE = "revolute"
    PRISMATIC = "prismatic"


class Joint(enum.IntEnum):
    """Joint numbers as the vendor library uses them (``joint_home``, ``get_joint_state``)."""

    J1_SHOULDER = 1
    J2_ELBOW = 2
    J3_VERTICAL = 3
    J4_WRIST = 4


@dataclass(frozen=True)
class JointSpec:
    joint: Joint
    name: str
    kind: JointKind
    unit: str
    field: str


JOINT_SPECS: Tuple[JointSpec, ...] = (
    JointSpec(Joint.J1_SHOULDER, "shoulder", JointKind.REVOLUTE, "deg", "j1_deg"),
    JointSpec(Joint.J2_ELBOW, "elbow", JointKind.REVOLUTE, "deg", "j2_deg"),
    JointSpec(Joint.J3_VERTICAL, "vertical", JointKind.PRISMATIC, "mm", "j3_mm"),
    JointSpec(Joint.J4_WRIST, "wrist", JointKind.REVOLUTE, "deg", "j4_deg"),
)


def joint_spec(joint: Joint) -> JointSpec:
    return JOINT_SPECS[int(Joint(joint)) - 1]


@dataclass(frozen=True)
class JointPositions:
    """Joint-space position.

    ``j3_mm`` is the vertical axis: 0 at the top of travel, negative going down.
    ``j4_deg`` is the vendor's ``r`` value, which the library also uses as the
    tool rotation in Cartesian commands (see the README for the caveat).
    """

    j1_deg: float
    j2_deg: float
    j3_mm: float
    j4_deg: float

    def as_tuple(self) -> Tuple[float, float, float, float]:
        return (self.j1_deg, self.j2_deg, self.j3_mm, self.j4_deg)

    def value(self, joint: Joint) -> float:
        return float(getattr(self, joint_spec(joint).field))

    def to_si(self) -> Tuple[float, float, float, float]:
        """(J1 rad, J2 rad, J3 m, J4 rad)."""
        return (
            math.radians(self.j1_deg),
            math.radians(self.j2_deg),
            self.j3_mm / 1000.0,
            math.radians(self.j4_deg),
        )


@dataclass(frozen=True)
class CartesianPose:
    """Tool position in the arm's base frame, plus the vendor's rotation ``r``."""

    x_mm: float
    y_mm: float
    z_mm: float
    r_deg: float

    def as_tuple(self) -> Tuple[float, float, float, float]:
        return (self.x_mm, self.y_mm, self.z_mm, self.r_deg)

    def to_si(self) -> Tuple[float, float, float, float]:
        """(x m, y m, z m, r rad)."""
        return (self.x_mm / 1000.0, self.y_mm / 1000.0, self.z_mm / 1000.0, math.radians(self.r_deg))


@dataclass(frozen=True)
class ArmState:
    """One ``get_scara_param`` sample."""

    joints: JointPositions
    pose: CartesianPose
    communicating: bool
    initialized: bool
    moving: bool
    timestamp_s: float


@dataclass(frozen=True)
class EncoderReading:
    """One ``get_encoder_param`` sample (positions derived from the joint encoders)."""

    joints: JointPositions
    pose: CartesianPose
    timestamp_s: float


@dataclass(frozen=True)
class JointLimits:
    """SDK-side soft limits, checked before a joint move is sent.

    The defaults are the slider ranges of the legacy "Z-Arm Sliders" app in this
    repository. The controller enforces its own limits as well (it rejects
    unreachable targets with ``MoveResult.UNREACHABLE``).
    """

    j1_deg: Tuple[float, float] = (-90.0, 90.0)
    j2_deg: Tuple[float, float] = (-164.0, 164.0)
    j3_mm: Tuple[float, float] = (-310.0, 0.0)
    j4_deg: Tuple[float, float] = (-180.0, 180.0)

    @classmethod
    def for_z_travel(cls, z_travel_mm: float) -> "JointLimits":
        return cls(j3_mm=(-float(z_travel_mm), 0.0))

    def range_of(self, joint: Joint) -> Tuple[float, float]:
        return getattr(self, joint_spec(joint).field)

    def violations(self, target: JointPositions) -> List[str]:
        problems = []
        for spec in JOINT_SPECS:
            low, high = getattr(self, spec.field)
            value = getattr(target, spec.field)
            if not low <= value <= high:
                problems.append(f"{spec.field}={value:g} is outside [{low:g}, {high:g}] {spec.unit}")
        return problems


def require_number(name: str, value: object) -> float:
    """Return ``value`` as a finite float or raise InvalidArgumentError."""
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        raise InvalidArgumentError(f"{name} must be a real number, got {value!r}")
    number = float(value)
    if not math.isfinite(number):
        raise InvalidArgumentError(f"{name} must be finite, got {number!r}")
    return number


def to_float32(value: float) -> float:
    """``value`` as the DLL will see it after conversion to a C float."""
    return float(struct.unpack("<f", struct.pack("<f", value))[0])


def require_c_float(name: str, value: object) -> float:
    """A finite number that stays finite as a C float (ctypes turns 1e300 into inf)."""
    number = require_number(name, value)
    if abs(number) > FLOAT32_MAX:
        raise InvalidArgumentError(f"{name}={number:g} does not fit in a C float")
    return number


def require_c_int(name: str, value: object, low: int = C_INT_MIN, high: int = C_INT_MAX) -> int:
    """An int within [low, high] (ctypes silently wraps ints that do not fit in a C int)."""
    if isinstance(value, bool) or not isinstance(value, int):
        raise InvalidArgumentError(f"{name} must be an int, got {value!r}")
    if not max(low, C_INT_MIN) <= value <= min(high, C_INT_MAX):
        raise InvalidArgumentError(f"{name} must be in {max(low, C_INT_MIN)}..{min(high, C_INT_MAX)}, got {value}")
    return value


def require_card_id(value: object) -> int:
    return require_c_int("card_id", value, 0, 255)


@dataclass(frozen=True)
class ZArmConfig:
    """Connection, safety and timing settings for one arm.

    ``card_id`` is the arm's id on the vendor server (the legacy code used 59).
    Leave it ``None`` to discover it; discovery fails if more than one arm answers.
    """

    card_id: Optional[int] = None
    generation: int = 1
    z_travel_mm: int = 310
    arm_lengths_mm: Tuple[float, float] = (200.0, 200.0)
    joint_limits: Optional[JointLimits] = None
    max_joint_speed: float = 100.0
    max_linear_speed_mm_s: float = 100.0
    arrival_tolerance_deg: float = 1.0
    arrival_tolerance_mm: float = 1.0
    poll_interval_s: float = 0.02
    discovery_poll_interval_s: float = 0.25
    motion_start_grace_s: float = 0.5
    motion_timeout_factor: float = 2.0
    motion_timeout_margin_s: float = 3.0
    max_motion_timeout_s: float = 300.0

    def resolved_joint_limits(self) -> JointLimits:
        if self.joint_limits is not None:
            return self.joint_limits
        return JointLimits.for_z_travel(self.z_travel_mm)

    def validate(self) -> None:
        if self.card_id is not None:
            require_card_id(self.card_id)
        require_c_int("generation", self.generation, 1, 8)
        require_c_int("z_travel_mm (the C API takes an int)", self.z_travel_mm, 1, 100_000)
        if len(self.arm_lengths_mm) != 2:
            raise InvalidArgumentError("arm_lengths_mm must be (l1_mm, l2_mm)")
        for index, length in enumerate(self.arm_lengths_mm):
            if require_c_float(f"arm_lengths_mm[{index}]", length) <= 0:
                raise InvalidArgumentError("arm lengths must be positive")
        for name in (
            "max_joint_speed",
            "max_linear_speed_mm_s",
            "arrival_tolerance_deg",
            "arrival_tolerance_mm",
            "poll_interval_s",
            "discovery_poll_interval_s",
            "motion_timeout_factor",
            "max_motion_timeout_s",
        ):
            if require_c_float(name, getattr(self, name)) <= 0:
                raise InvalidArgumentError(f"{name} must be > 0")
        for name in ("motion_start_grace_s", "motion_timeout_margin_s"):
            if require_number(name, getattr(self, name)) < 0:
                raise InvalidArgumentError(f"{name} must be >= 0")
        limits = self.resolved_joint_limits()
        for spec in JOINT_SPECS:
            low, high = getattr(limits, spec.field)
            if math.isnan(low) or math.isnan(high) or low > high:
                raise InvalidArgumentError(f"joint_limits.{spec.field} must be (low, high) with low <= high")

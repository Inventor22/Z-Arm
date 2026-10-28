"""Vendor return codes as typed enums.

Where each value comes from is recorded in ``docs/native-api.md``. In short:
values marked *verified* were read out of the x64 ``small_scara_interface.dll``
by static disassembly; the rest come from Hitbot's header comments, the legacy
C# code in this repository, or the community wrappers.
"""

from __future__ import annotations

import enum
from typing import Dict, Type, TypeVar, Union


class ServerStartResult(enum.IntEnum):
    """``net_port_initial`` (verified)."""

    FAILED = -1
    OK = 1


class ConnectStatus(enum.IntEnum):
    """``card_number_is_connect`` / ``robot_is_connect`` (verified)."""

    NOT_CONNECTED = 0
    CONNECTED = 1
    CARD_OUT_OF_RANGE = 2


class InitResult(enum.IntEnum):
    """``robot_initial``."""

    NOT_CONNECTED = 0
    OK = 1
    GENERATION_ERROR = 2
    ENCODER_ERROR = 3
    MOBILE_TERMINAL_IN_CONTROL = 11
    Z_TRAVEL_ERROR = 12
    JOINT_FAILURE = 105


class MoveResult(enum.IntEnum):
    """``robot_movej_*`` / ``robot_movel_*`` / ``robot_change_attitude``."""

    BUSY = 0
    OK = 1
    INVALID_SPEED = 2
    NOT_INITIALIZED = 3
    UNREACHABLE = 4
    SERVO_OFF = 6
    HAND_CONFIGURATION_UNREACHABLE = 7
    MOBILE_TERMINAL_IN_CONTROL = 11
    INVALID_PARAMETER = 101
    COLLISION = 102
    JOINT_NEEDS_REINITIALIZATION = 103
    JOINT_FAULT = 105


class JointState(enum.IntEnum):
    """``robot_get_joint_state`` (values verified; meanings of 4 and 6 unknown)."""

    NEEDS_REINITIALIZATION = 0
    NORMAL = 1
    INVALID_JOINT = 2
    NOT_INITIALIZED = 3
    UNAVAILABLE = 4
    COLLISION = 5
    SPECIAL_MODE = 6
    FAULT = 7
    FAULT_8 = 8


class HomeResult(enum.IntEnum):
    """``robot_joint_home`` (0, 2, 3 verified; 1 inferred)."""

    NOT_CONNECTED = 0
    OK = 1
    INVALID_JOINT = 2
    REFUSED = 3


# Keyed by enum class: IntEnum members compare and hash equal to their int values,
# so members of different enums would collide in one dict.
DESCRIPTIONS: Dict[Type[enum.IntEnum], Dict[int, str]] = {
    ServerStartResult: {
        ServerStartResult.FAILED: "server failed to start (share.dll not loadable, or server.exe not running within 3 s)",
        ServerStartResult.OK: "server running",
    },
    ConnectStatus: {
        ConnectStatus.NOT_CONNECTED: "no arm connected with this card id",
        ConnectStatus.CONNECTED: "arm connected",
        ConnectStatus.CARD_OUT_OF_RANGE: "card id outside 0..255",
    },
    InitResult: {
        InitResult.NOT_CONNECTED: "communication not established",
        InitResult.OK: "initialization accepted",
        InitResult.GENERATION_ERROR: "generation parameter outside 1..8",
        InitResult.ENCODER_ERROR: "encoder value error",
        InitResult.MOBILE_TERMINAL_IN_CONTROL: "arm is controlled by the mobile terminal",
        InitResult.Z_TRAVEL_ERROR: "z_travel must be > 0",
        InitResult.JOINT_FAILURE: "a joint reported a fault (the vendor library shows a message box)",
    },
    MoveResult: {
        MoveResult.BUSY: "arm is executing another command",
        MoveResult.OK: "command accepted; motion started",
        MoveResult.INVALID_SPEED: "speed must be > 0",
        MoveResult.NOT_INITIALIZED: "arm is not initialized",
        MoveResult.UNREACHABLE: "target (or a point on the path) is out of range",
        MoveResult.SERVO_OFF: "servo is off",
        MoveResult.HAND_CONFIGURATION_UNREACHABLE: "target is unreachable in the requested hand configuration",
        MoveResult.MOBILE_TERMINAL_IN_CONTROL: "arm is controlled by the mobile terminal",
        MoveResult.INVALID_PARAMETER: "a parameter is NaN",
        MoveResult.COLLISION: "arm is in a collision state",
        MoveResult.JOINT_NEEDS_REINITIALIZATION: "a joint was reset and the arm must be initialized again",
        MoveResult.JOINT_FAULT: "a joint reported a fault",
    },
    JointState: {
        JointState.NEEDS_REINITIALIZATION: "joint was reset; initialize the arm again",
        JointState.NORMAL: "normal",
        JointState.INVALID_JOINT: "joint number outside 1..4",
        JointState.NOT_INITIALIZED: "arm is not initialized",
        JointState.UNAVAILABLE: "joint state unavailable (meaning unverified)",
        JointState.COLLISION: "collision detected",
        JointState.SPECIAL_MODE: "controller is in a special mode (meaning unverified)",
        JointState.FAULT: "joint fault: communication error or missing-axis alarm",
        JointState.FAULT_8: "joint fault (code 8; treated as a fault by the vendor move wrappers)",
    },
    HomeResult: {
        HomeResult.NOT_CONNECTED: "arm not connected",
        HomeResult.OK: "homing started",
        HomeResult.INVALID_JOINT: "joint number outside 1..4",
        HomeResult.REFUSED: "refused by a controller status flag (meaning unverified)",
    },
}

E = TypeVar("E", bound=enum.IntEnum)


def coerce(enum_cls: Type[E], raw: int) -> Union[E, int]:
    """Map a raw vendor value to ``enum_cls``; unknown values stay plain ints."""
    try:
        return enum_cls(int(raw))
    except ValueError:
        return int(raw)


def describe(code: Union[enum.IntEnum, int]) -> str:
    if isinstance(code, enum.IntEnum):
        return DESCRIPTIONS.get(type(code), {}).get(int(code), code.name.lower().replace("_", " "))
    return f"unknown vendor code {int(code)}"

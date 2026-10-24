"""Typed Python SDK for the Hitbot Z-Arm, layered over Hitbot's native library.

Start with :class:`ZArm`. ``ZArm.simulated()`` needs no hardware;
``ZArm.native(library_dir)`` drives a real arm through ``small_scara_interface.dll``.
"""

import logging

from .arm import ArmPhase, ZArm
from .backend import Backend, EncoderParam, ScaraParam
from .codes import (
    ConnectStatus,
    HomeResult,
    InitResult,
    JointState,
    MoveResult,
    ServerStartResult,
    describe,
)
from .errors import (
    AmbiguousArmError,
    ArmBusyError,
    ArmNotFoundError,
    CollisionError,
    CommandError,
    ConnectionLostError,
    InitializationError,
    InvalidArgumentError,
    InvalidTelemetryError,
    JointFaultError,
    LibraryError,
    LimitExceededError,
    MotionIncompleteError,
    MotionTimeoutError,
    NotConnectedError,
    NotInitializedError,
    OwnershipError,
    ServerStartError,
    SoftLimitError,
    UnreachableError,
    ZArmConnectionError,
    ZArmError,
)
from .fake import FakeBackend, FakeBackendMisuseError, ManualClock
from .types import (
    DEFAULT_ARM_LENGTHS_MM,
    JOINT_SPECS,
    LEGACY_CARD_ID,
    VENDOR_ARM_LENGTHS_MM,
    ArmState,
    CartesianPose,
    EncoderReading,
    Joint,
    JointKind,
    JointLimits,
    JointPositions,
    JointSpec,
    ZArmConfig,
)

__version__ = "0.1.0"

logging.getLogger("zarm").addHandler(logging.NullHandler())

__all__ = [
    "AmbiguousArmError",
    "ArmBusyError",
    "ArmNotFoundError",
    "ArmPhase",
    "ArmState",
    "Backend",
    "CartesianPose",
    "CollisionError",
    "CommandError",
    "ConnectStatus",
    "ConnectionLostError",
    "DEFAULT_ARM_LENGTHS_MM",
    "EncoderParam",
    "EncoderReading",
    "FakeBackend",
    "FakeBackendMisuseError",
    "HomeResult",
    "InitResult",
    "InitializationError",
    "InvalidArgumentError",
    "InvalidTelemetryError",
    "JOINT_SPECS",
    "Joint",
    "JointFaultError",
    "JointKind",
    "JointLimits",
    "JointPositions",
    "JointSpec",
    "JointState",
    "LEGACY_CARD_ID",
    "LibraryError",
    "LimitExceededError",
    "ManualClock",
    "MotionIncompleteError",
    "MotionTimeoutError",
    "MoveResult",
    "NotConnectedError",
    "NotInitializedError",
    "OwnershipError",
    "ScaraParam",
    "ServerStartError",
    "ServerStartResult",
    "SoftLimitError",
    "UnreachableError",
    "VENDOR_ARM_LENGTHS_MM",
    "ZArm",
    "ZArmConfig",
    "ZArmConnectionError",
    "ZArmError",
    "describe",
]

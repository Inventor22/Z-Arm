"""Exceptions raised by the Z-Arm SDK."""

from __future__ import annotations

from typing import TYPE_CHECKING, Iterable, Optional, Tuple, Type, Union

from .codes import InitResult, MoveResult, coerce, describe

if TYPE_CHECKING:
    from .types import ArmState


class ZArmError(Exception):
    """Base class for every SDK error."""


class InvalidArgumentError(ZArmError, ValueError):
    """A value was rejected by the SDK before anything was sent to the arm."""


class SoftLimitError(InvalidArgumentError):
    """A target lies outside the configured soft limits or the arm's reach."""


class LibraryError(ZArmError):
    """The vendor library is missing, unsupported, or failed its preflight checks."""


class OwnershipError(ZArmError):
    """Another ZArm instance in this process already owns the arm."""


class ZArmConnectionError(ZArmError):
    """Base class for server and connection problems."""


class ServerStartError(ZArmConnectionError):
    """``net_port_initial`` failed."""


class ArmNotFoundError(ZArmConnectionError):
    """No arm (or not the configured arm) connected before the timeout."""


class AmbiguousArmError(ZArmConnectionError):
    """Several arms are connected and no ``card_id`` was configured."""

    def __init__(self, card_ids: Iterable[int]):
        self.card_ids: Tuple[int, ...] = tuple(card_ids)
        super().__init__(
            f"several arms are connected (card ids {list(self.card_ids)}); set ZArmConfig.card_id to choose one"
        )


class NotConnectedError(ZArmConnectionError):
    """The operation needs a connected arm."""


class ConnectionLostError(ZArmConnectionError):
    """The arm stopped communicating during an operation."""

    def __init__(self, message: str, state: "Optional[ArmState]" = None):
        super().__init__(message)
        self.state = state


class InvalidTelemetryError(ZArmConnectionError):
    """The vendor library reported non-finite positions."""


class InitializationError(ZArmError):
    """``robot_initial`` did not succeed."""

    def __init__(self, code: int, attempts: int):
        self.code: Union[InitResult, int] = coerce(InitResult, code)
        self.attempts = attempts
        super().__init__(
            f"initialization failed after {attempts} attempt(s): {describe(self.code)} (code {int(code)})"
        )


class CommandError(ZArmError):
    """A command was rejected, or its motion did not complete as expected.

    ``code`` is the raw vendor result mapped to ``MoveResult`` where possible,
    or ``None`` when the SDK raised the error itself.
    """

    def __init__(
        self,
        command: str,
        code: Optional[int] = None,
        message: Optional[str] = None,
        state: "Optional[ArmState]" = None,
    ):
        self.command = command
        self.code: Union[MoveResult, int, None] = None if code is None else coerce(MoveResult, code)
        self.state = state
        if message is None:
            if code is None:
                message = f"{command} failed"
            else:
                message = f"{command} rejected: {describe(self.code)} (vendor code {int(code)})"
        super().__init__(message)


class ArmBusyError(CommandError):
    """The arm is still executing another command."""


class NotInitializedError(CommandError):
    """The arm must be initialized first."""


class UnreachableError(CommandError):
    """The target, or a point on the path, is out of range."""


class CollisionError(CommandError):
    """The arm reported a collision."""


class JointFaultError(CommandError):
    """A joint reported a fault or needs re-initialization."""


class MotionTimeoutError(CommandError):
    """The motion did not finish in time. The SDK has already sent a stop."""


class MotionIncompleteError(CommandError):
    """The arm stopped short of the target. The SDK has already sent a stop."""


class LimitExceededError(CommandError):
    """During motion, a joint moved further outside the soft limits. The SDK has already sent a stop."""


_MOVE_ERRORS = {
    MoveResult.BUSY: ArmBusyError,
    MoveResult.NOT_INITIALIZED: NotInitializedError,
    MoveResult.UNREACHABLE: UnreachableError,
    MoveResult.HAND_CONFIGURATION_UNREACHABLE: UnreachableError,
    MoveResult.COLLISION: CollisionError,
    MoveResult.JOINT_NEEDS_REINITIALIZATION: JointFaultError,
    MoveResult.JOINT_FAULT: JointFaultError,
}


def error_for_move_code(command: str, code: int) -> CommandError:
    """Build the most specific CommandError for a non-OK move result."""
    cls: Type[CommandError] = _MOVE_ERRORS.get(coerce(MoveResult, code), CommandError)  # type: ignore[arg-type]
    return cls(command, code)

"""High-level client for one Hitbot Z-Arm."""

from __future__ import annotations

import atexit
import enum
import logging
import math
import os
import threading
import time
import weakref
from typing import Callable, Dict, Optional, Tuple, Union

from . import kinematics
from .backend import Backend
from .codes import ConnectStatus, HomeResult, InitResult, JointState, MoveResult, ServerStartResult, coerce, describe
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
    LimitExceededError,
    MotionIncompleteError,
    MotionTimeoutError,
    NotConnectedError,
    NotInitializedError,
    ServerStartError,
    SoftLimitError,
    ZArmError,
    error_for_move_code,
)
from .types import (
    JOINT_SPECS,
    ArmState,
    CartesianPose,
    EncoderReading,
    Joint,
    JointKind,
    JointPositions,
    ZArmConfig,
    require_c_float,
    require_c_int,
    require_number,
    to_float32,
)

_log = logging.getLogger("zarm")

_RETRYABLE_INIT_CODES = frozenset({InitResult.NOT_CONNECTED, InitResult.ENCODER_ERROR})
LR_VALUES = (-1, 0, 1)
MAX_IO_INDEX = 63


class ArmPhase(enum.Enum):
    CREATED = "created"
    CONNECTED = "connected"
    INITIALIZED = "initialized"
    CLOSING = "closing"
    CLOSED = "closed"


_ACTIVE = (ArmPhase.CONNECTED, ArmPhase.INITIALIZED)
_OPEN_ARMS: "weakref.WeakSet[ZArm]" = weakref.WeakSet()


@atexit.register
def _stop_open_arms_at_exit() -> None:
    for arm in list(_OPEN_ARMS):
        arm._failsafe_stop("interpreter exit")


class _Guard:
    """Sends a stop if an exception escapes once a command may have reached the arm.

    Set ``armed`` immediately before the vendor call; everything after it, up to
    the end of the wait, is then covered.
    """

    def __init__(self, arm: "ZArm", command: str, armed: bool = False) -> None:
        self.arm = arm
        self.command = command
        self.armed = armed

    def __enter__(self) -> "_Guard":
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> bool:
        if exc_type is not None and self.armed:
            self.arm._failsafe_stop(self.command)
        return False


class ZArm:
    """One Hitbot Z-Arm, owned by this object from :meth:`connect` until :meth:`close`.

    ::

        config = ZArmConfig(card_id=59)
        with ZArm.native(r"C:\\hitbot\\vendor", config) as arm:
            arm.connect()
            arm.initialize()
            arm.move_joints(JointPositions(j1_deg=0, j2_deg=0, j3_mm=-50, j4_deg=0), speed=30)

    Every wait has a timeout. Once a command may have reached the arm, any
    exception (a rejection, timeout, collision, lost communication, the arm
    leaving the soft limits or stopping short, Ctrl+C) makes the SDK send a
    stop before the exception propagates.
    """

    def __init__(
        self,
        backend: Backend,
        config: Optional[ZArmConfig] = None,
        *,
        clock: Optional[Callable[[], float]] = None,
        sleep: Optional[Callable[[float], None]] = None,
        owns_backend: bool = False,
    ) -> None:
        self._backend = backend
        self._config = config if config is not None else ZArmConfig()
        self._config.validate()
        self._limits = self._config.resolved_joint_limits()
        self._clock: Callable[[], float] = clock or time.monotonic
        self._sleep: Callable[[float], None] = sleep or time.sleep
        self._owns_backend = owns_backend
        self._lock = threading.RLock()
        self._phase_lock = threading.Lock()
        self._phase = ArmPhase.CREATED
        self._card: Optional[int] = None

    @classmethod
    def native(
        cls,
        library_dir: Union[str, "os.PathLike[str]"],
        config: Optional[ZArmConfig] = None,
        *,
        allow_unverified_build: bool = False,
        **kwargs: object,
    ) -> "ZArm":
        """A ZArm over Hitbot's DLL in ``library_dir`` (see ``zarm.native``)."""
        from .native import NativeBackend

        backend = NativeBackend(library_dir, allow_unverified_build=allow_unverified_build)
        try:
            return cls(backend, config, owns_backend=True, **kwargs)  # type: ignore[arg-type]
        except BaseException:
            backend.close()
            raise

    @classmethod
    def simulated(
        cls,
        config: Optional[ZArmConfig] = None,
        *,
        clock: Optional[Callable[[], float]] = None,
        sleep: Optional[Callable[[float], None]] = None,
        **backend_options: object,
    ) -> "ZArm":
        """A ZArm over :class:`zarm.fake.FakeBackend` with one simulated arm."""
        from .fake import FakeBackend

        config = config if config is not None else ZArmConfig()
        card = config.card_id if config.card_id is not None else 1
        backend = FakeBackend(
            (card,),
            clock=clock,
            arm_lengths_mm=config.arm_lengths_mm,
            z_travel_mm=config.z_travel_mm,
            **backend_options,  # type: ignore[arg-type]
        )
        return cls(backend, config, clock=clock, sleep=sleep, owns_backend=True)

    def __enter__(self) -> "ZArm":
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()

    def __repr__(self) -> str:
        return f"ZArm(card_id={self._card}, phase={self._phase.value})"

    @property
    def backend(self) -> Backend:
        return self._backend

    @property
    def config(self) -> ZArmConfig:
        return self._config

    @property
    def phase(self) -> ArmPhase:
        return self._phase

    @property
    def card_id(self) -> Optional[int]:
        return self._card

    # Lifecycle

    def connect(self, *, timeout_s: float = 10.0) -> int:
        """Start the vendor server and wait for the arm; returns its card id.

        ``net_port_initial`` itself can block for about 3 s (and indefinitely if
        the vendor library shows a message box) before the timeout applies.
        """
        timeout_s = self._positive("timeout_s", timeout_s)
        with self._lock:
            phase = self._phase
            if phase in (ArmPhase.CLOSING, ArmPhase.CLOSED):
                raise ZArmError("this ZArm is closed; create a new one")
            if phase in _ACTIVE:
                assert self._card is not None
                return self._card
            deadline = self._clock() + timeout_s
            result = self._backend.start_server()
            if result != ServerStartResult.OK:
                raise ServerStartError(
                    f"net_port_initial returned {result}: {describe(coerce(ServerStartResult, result))}"
                )
            card = self._find_card(deadline, timeout_s)
            self._backend.claim(card, self)
            with self._phase_lock:
                if self._phase is not ArmPhase.CREATED:
                    self._backend.release(card, self)
                    raise ZArmError("this ZArm was closed while connecting")
                self._card = card
                self._phase = ArmPhase.CONNECTED
            _OPEN_ARMS.add(self)
            _log.info("connected to Z-Arm card %d", card)
            return card

    def _find_card(self, deadline: float, timeout_s: float) -> int:
        configured = self._config.card_id
        while True:
            if configured is not None:
                if self._backend.card_connect_status(configured) == ConnectStatus.CONNECTED:
                    return configured
            else:
                found = [
                    card for card in range(256) if self._backend.card_connect_status(card) == ConnectStatus.CONNECTED
                ]
                if len(found) == 1:
                    return found[0]
                if found:
                    raise AmbiguousArmError(found)
            if self._clock() >= deadline:
                which = f"the arm with card id {configured}" if configured is not None else "no arm"
                raise ArmNotFoundError(
                    f"{which} connected within {timeout_s:g} s; check power, the Ethernet cable, the PC's "
                    "address (192.168.0.100) and the firewall prompt for server.exe"
                )
            self._sleep(self._config.discovery_poll_interval_s)

    def initialize(self, *, attempts: int = 4, retry_delay_s: float = 1.0) -> None:
        """Run the vendor initialization, then unlock and set the arm lengths.

        Retries only the codes the legacy app saw recover (0 and 3). This
        energizes the arm; treat it like a motion command.
        """
        attempts = require_c_int("attempts", attempts, 1, 100)
        retry_delay_s = require_number("retry_delay_s", retry_delay_s)
        if retry_delay_s < 0:
            raise InvalidArgumentError(f"retry_delay_s must be >= 0, got {retry_delay_s:g}")
        with self._lock:
            card = self._require_connected("initialize")
            if self.get_state().moving:
                raise ArmBusyError(
                    "initialize", message="initialize: the arm is moving (e.g. homing); wait until it stops"
                )
            config = self._config
            self._set_phase(ArmPhase.CONNECTED)
            with _Guard(self, "initialize", armed=True):
                for attempt in range(1, attempts + 1):
                    code = self._backend.initial(card, config.generation, config.z_travel_mm)
                    if code == InitResult.OK:
                        break
                    if code not in _RETRYABLE_INIT_CODES or attempt == attempts:
                        raise InitializationError(code, attempt)
                    _log.warning("initial returned %d (%s); retrying", code, describe(coerce(InitResult, code)))
                    self._sleep(retry_delay_s)
                unlocked = self._backend.unlock_position(card)
                if unlocked != 1:
                    raise NotConnectedError(f"unlock_position returned {unlocked}: the arm is not connected")
                l1_mm, l2_mm = config.arm_lengths_mm
                self._backend.set_arm_length(card, float(l1_mm), float(l2_mm))
                self._set_phase(ArmPhase.INITIALIZED)

    def close(self, *, stop_server: bool = False) -> None:
        """Stop the arm, release it and optionally shut down the vendor server.

        The stop is sent before waiting for any command in progress (for example
        one blocked behind a vendor message box). ``stop_server`` is skipped
        with a warning while another ZArm still owns an arm on the same backend.
        """
        with self._phase_lock:
            if self._phase in (ArmPhase.CLOSING, ArmPhase.CLOSED):
                return
            connected = self._phase in _ACTIVE
            self._phase = ArmPhase.CLOSING
        card = self._card
        try:
            if connected and card is not None:
                self._stop_quietly(card, "close")
            with self._lock:
                if connected and card is not None:
                    self._backend.release(card, self)
                    if stop_server:
                        others = [other for other in self._backend.owned_cards() if other != card]
                        if others:
                            _log.warning("not stopping the vendor server: cards %s are still in use", others)
                        else:
                            self._backend.close_server()
        finally:
            with self._phase_lock:
                self._phase = ArmPhase.CLOSED
            _OPEN_ARMS.discard(self)
            if self._owns_backend:
                self._backend.close()

    # State

    def get_state(self) -> ArmState:
        card = self._require_connected("get_state")
        param = self._backend.get_scara_param(card)
        self._require_finite_telemetry(
            "get_scara_param",
            (param.x_mm, param.y_mm, param.z_mm, param.angle1_deg, param.angle2_deg, param.r_deg),
        )
        return ArmState(
            joints=JointPositions(param.angle1_deg, param.angle2_deg, param.z_mm, param.r_deg),
            pose=CartesianPose(param.x_mm, param.y_mm, param.z_mm, param.r_deg),
            communicating=bool(param.communicate_success),
            initialized=bool(param.initial_finish),
            moving=bool(param.move_flag),
            timestamp_s=self._clock(),
        )

    def get_encoder_reading(self) -> EncoderReading:
        card = self._require_connected("get_encoder_reading")
        param = self._backend.get_encoder_param(card)
        self._require_finite_telemetry(
            "get_encoder_param",
            (param.x_mm, param.y_mm, param.z_mm, param.angle1_deg, param.angle2_deg, param.r_deg),
        )
        return EncoderReading(
            joints=JointPositions(param.angle1_deg, param.angle2_deg, param.z_mm, param.r_deg),
            pose=CartesianPose(param.x_mm, param.y_mm, param.z_mm, param.r_deg),
            timestamp_s=self._clock(),
        )

    def get_joint_state(self, joint: Joint) -> Union[JointState, int]:
        """Vendor joint state. On some faults the vendor library also shows a modal message box."""
        number = self._joint_number(joint)
        card = self._require_connected("get_joint_state")
        return coerce(JointState, self._backend.get_joint_state(card, number))

    def get_joint_states(self) -> Dict[Joint, Union[JointState, int]]:
        return {joint: self.get_joint_state(joint) for joint in Joint}

    def is_moving(self) -> bool:
        return not self._backend.is_stop(self._require_connected("is_moving"))

    def is_collision(self) -> bool:
        return bool(self._backend.is_collision(self._require_connected("is_collision")))

    # Motion

    def move_joints(
        self,
        target: JointPositions,
        *,
        speed: float,
        roughly: float = 0.0,
        wait: bool = True,
        timeout_s: Optional[float] = None,
    ) -> Optional[ArmState]:
        """Joint-space move to absolute joint values (``robot_movej_angle``).

        ``speed`` is the vendor joint speed: deg/s for J1, J2 and J4 and mm/s for
        J3; the slowest joint sets the duration. ``roughly`` (0..1) trades
        accuracy for blending; arrival is only checked when it is 0.
        Returns the final state, or None when ``wait`` is False.
        """
        target = self._checked_joints(target)
        speed = self._checked_speed("speed", speed, self._config.max_joint_speed)
        roughly = self._checked_roughly(roughly)
        timeout_s = self._checked_timeout(timeout_s)
        with _Guard(self, "move_joints") as guard:
            with self._lock:
                card = self._require_initialized("move_joints")
                start = self.get_state()
                guard.armed = True
                self._check_move_result(
                    "movej_angle", self._backend.movej_angle(card, *target.as_tuple(), speed, roughly)
                )
            if not wait:
                return None
            estimate = max(abs(b - a) for a, b in zip(start.joints.as_tuple(), target.as_tuple())) / speed
            at_target = (lambda state: self._joints_close(state.joints, target)) if roughly == 0 else None
            return self._wait_for_motion("move_joints", self._motion_timeout(estimate, timeout_s), at_target, start)

    def move_to_pose(
        self,
        target: CartesianPose,
        *,
        speed: float,
        roughly: float = 0.0,
        lr: Optional[int] = None,
        wait: bool = True,
        timeout_s: Optional[float] = None,
    ) -> Optional[ArmState]:
        """Joint-interpolated move to a Cartesian target (``robot_movej_xyz``).

        The path is generally not a straight line. Without ``lr`` the target must
        be within the soft limits in the current hand configuration. Passing
        ``lr`` (-1, 0 or 1) uses ``robot_movej_xyz_lr``; because the value-to-hand
        mapping is unverified, both hand solutions must then be within the limits.
        """
        target = self._checked_pose(target)
        speed = self._checked_speed("speed", speed, self._config.max_joint_speed)
        roughly = self._checked_roughly(roughly)
        timeout_s = self._checked_timeout(timeout_s)
        lr = self._checked_lr(lr)
        with _Guard(self, "move_to_pose") as guard:
            with self._lock:
                card = self._require_initialized("move_to_pose")
                start = self.get_state()
                self._check_pose_solutions(start, target, lr)
                guard.armed = True
                if lr is None:
                    code = self._backend.movej_xyz(card, *target.as_tuple(), speed, roughly)
                    self._check_move_result("movej_xyz", code)
                else:
                    code = self._backend.movej_xyz_lr(card, *target.as_tuple(), speed, roughly, lr)
                    self._check_move_result("movej_xyz_lr", code)
            if not wait:
                return None
            estimate = self._pose_move_seconds(start, target, speed)
            at_target = (lambda state: self._pose_close(state.pose, target)) if roughly == 0 else None
            return self._wait_for_motion("move_to_pose", self._motion_timeout(estimate, timeout_s), at_target, start)

    def move_linear(
        self,
        target: CartesianPose,
        *,
        speed_mm_s: float,
        wait: bool = True,
        timeout_s: Optional[float] = None,
    ) -> Optional[ArmState]:
        """Straight-line Cartesian move (``robot_movel_xyz``) in the current hand configuration."""
        target = self._checked_pose(target)
        speed_mm_s = self._checked_speed("speed_mm_s", speed_mm_s, self._config.max_linear_speed_mm_s)
        timeout_s = self._checked_timeout(timeout_s)
        with _Guard(self, "move_linear") as guard:
            with self._lock:
                card = self._require_initialized("move_linear")
                start = self.get_state()
                self._check_linear_path(start, target)
                guard.armed = True
                self._check_move_result(
                    "movel_xyz", self._backend.movel_xyz(card, *target.as_tuple(), speed_mm_s)
                )
            if not wait:
                return None
            distance = math.sqrt(
                sum((b - a) ** 2 for a, b in zip(start.pose.as_tuple()[:3], target.as_tuple()[:3]))
            )
            estimate = max(distance, abs(target.r_deg - start.pose.r_deg)) / speed_mm_s
            return self._wait_for_motion(
                "move_linear",
                self._motion_timeout(estimate, timeout_s),
                lambda state: self._pose_close(state.pose, target),
                start,
            )

    def change_hand(self, *, speed: float, wait: bool = True, timeout_s: Optional[float] = None) -> Optional[ArmState]:
        """Switch to the other elbow configuration at the same tool position (``robot_change_attitude``)."""
        speed = self._checked_speed("speed", speed, self._config.max_joint_speed)
        timeout_s = self._checked_timeout(timeout_s)
        l1_mm, l2_mm = self._config.arm_lengths_mm
        with _Guard(self, "change_hand") as guard:
            with self._lock:
                card = self._require_initialized("change_hand")
                start = self.get_state()
                j1, j2 = kinematics.alternate_hand(start.joints.j1_deg, start.joints.j2_deg, l1_mm, l2_mm)
                problems = self._limits.violations(JointPositions(j1, j2, start.joints.j3_mm, start.joints.j4_deg))
                if problems:
                    raise SoftLimitError("the other hand configuration is outside the soft limits: " + "; ".join(problems))
                guard.armed = True
                self._check_move_result("change_attitude", self._backend.change_attitude(card, speed))
            if not wait:
                return None
            estimate = max(abs(j1 - start.joints.j1_deg), abs(j2 - start.joints.j2_deg)) / speed
            tolerance = self._config.arrival_tolerance_deg
            start_elbow = kinematics.elbow_of(start.joints.j2_deg)

            def flipped(state: ArmState) -> bool:
                same_place = self._pose_close(state.pose, start.pose, check_r=False)
                elbow_changed = abs(start.joints.j2_deg) <= tolerance or (
                    kinematics.elbow_of(state.joints.j2_deg) != start_elbow
                )
                return same_place and elbow_changed

            return self._wait_for_motion("change_hand", self._motion_timeout(estimate, timeout_s), flipped, start)

    def wait_until_stopped(self, *, timeout_s: float) -> ArmState:
        """Wait until the arm reports it is not moving; stops it and raises on timeout."""
        timeout_s = self._positive("timeout_s", timeout_s)
        self._require_connected("wait_until_stopped")
        with _Guard(self, "wait_until_stopped", armed=True):
            return self._wait_for_motion("wait_until_stopped", timeout_s, None, None, grace_s=0.0)

    def stop(self) -> None:
        """Stop the current motion (``robot_stop_move``). Safe to call from another thread."""
        self._backend.stop_move(self._require_connected("stop"))

    def emergency_stop(self) -> None:
        """``robot_emergency_stop``. Its exact effect on the servos is unverified; the hardware E-stop is the real one."""
        self._backend.emergency_stop(self._require_connected("emergency_stop"))

    def pause(self) -> None:
        """Pause the current motion. If the vendor refuses, the SDK stops the arm instead."""
        card = self._require_connected("pause")
        with _Guard(self, "pause", armed=True):
            result = self._backend.pause_move(card)
            if result != 1:
                raise CommandError("pause_move", message=f"pause_move returned {result}; the arm was stopped instead")

    def resume(self) -> None:
        """Resume a paused motion. If the vendor refuses, the SDK stops the arm."""
        card = self._require_connected("resume")
        with _Guard(self, "resume", armed=True):
            result = self._backend.resume_move(card)
            if result != 1:
                raise CommandError("resume_move", message=f"resume_move returned {result}; the arm was stopped")

    def start_homing(self, joint: Joint) -> None:
        """Start homing one joint (``robot_joint_home``); returns once the controller accepts. The joint moves.

        The vendor clears initialization as soon as this is called, even if it
        then refuses, so the arm must be initialized again afterwards. Whether
        the vendor reports homing as motion is unverified: wait a generous,
        bounded time (for example ``wait_until_stopped``) before ``initialize()``.
        """
        number = self._joint_number(joint)
        with _Guard(self, "start_homing") as guard:
            with self._lock:
                card = self._require_connected("start_homing")
                if self.get_state().moving:
                    raise ArmBusyError("start_homing", message="start_homing: the arm is moving; stop it first")
                self._set_phase(ArmPhase.CONNECTED)
                guard.armed = True
                result = coerce(HomeResult, self._backend.joint_home(card, number))
                if result == HomeResult.OK:
                    return
                if result == HomeResult.NOT_CONNECTED:
                    raise NotConnectedError("joint_home: the arm is not connected")
                if result == HomeResult.INVALID_JOINT:
                    raise InvalidArgumentError(f"joint_home rejected joint {number}")
                raise CommandError(
                    "joint_home", message=f"joint_home refused: {describe(result)} (code {int(result)})"
                )

    # Modes and I/O

    def set_drag_teach(self, enabled: bool) -> None:
        """Enable or disable hand-guiding (drag teach)."""
        enabled = self._checked_flag("enabled", enabled)
        if not self._backend.set_drag_teach(self._require_connected("set_drag_teach"), enabled):
            raise CommandError("set_drag_teach", message="set_drag_teach was refused by the vendor library")

    def get_drag_teach(self) -> bool:
        return bool(self._backend.get_drag_teach(self._require_connected("get_drag_teach")))

    def set_cooperation_mode(self, enabled: bool) -> None:
        """The vendor's "cooperation function" (collaborative mode; exact behaviour unverified)."""
        enabled = self._checked_flag("enabled", enabled)
        if not self._backend.set_cooperation_fun_state(self._require_connected("set_cooperation_mode"), enabled):
            raise CommandError("set_cooperation_fun_state", message="set_cooperation_fun_state was refused")

    def get_cooperation_mode(self) -> bool:
        return bool(self._backend.get_cooperation_fun_state(self._require_connected("get_cooperation_mode")))

    def read_digital_input(self, io: int) -> int:
        """Input level as the vendor reports it (0 or 1).

        The library returns the inverse of the controller bit. Legacy notes in
        this repository say 0 means a 24 V signal is present; confirm on hardware.
        """
        io = self._checked_io(io)
        card = self._require_connected("read_digital_input")
        return self._io_level("get_digital_io_in", io, self._backend.get_digital_io_in(card, io))

    def read_digital_output(self, io: int) -> int:
        io = self._checked_io(io)
        card = self._require_connected("read_digital_output")
        return self._io_level("get_digital_io_out", io, self._backend.get_digital_io_out(card, io))

    def write_digital_output(self, io: int, on: bool) -> None:
        io = self._checked_io(io)
        on = self._checked_flag("on", on)
        if not self._backend.set_digital_io_out(self._require_connected("write_digital_output"), io, on):
            raise CommandError(
                "set_digital_io_out", message=f"set_digital_io_out refused io {io} (invalid io, or not connected)"
            )

    # Internals: lifecycle

    def _set_phase(self, phase: ArmPhase) -> None:
        with self._phase_lock:
            if self._phase not in _ACTIVE:
                raise NotConnectedError(f"the arm was closed during the operation (phase {self._phase.value})")
            self._phase = phase

    def _require_connected(self, command: str) -> int:
        card = self._card
        if self._phase not in _ACTIVE or card is None:
            raise NotConnectedError(f"{command}: call connect() first (phase is {self._phase.value})")
        return card

    def _require_initialized(self, command: str) -> int:
        card = self._require_connected(command)
        if self._phase is not ArmPhase.INITIALIZED:
            raise NotInitializedError(command, message=f"{command}: call initialize() first")
        return card

    def _failsafe_stop(self, reason: str) -> None:
        card = self._card
        if card is None or self._phase in (ArmPhase.CREATED, ArmPhase.CLOSED):
            return
        self._stop_quietly(card, reason)

    def _stop_quietly(self, card: int, reason: str) -> None:
        """Send a stop. Never raises, so it cannot replace the error being handled."""
        try:
            _log.warning("fail-safe stop for card %d (%s)", card, reason)
            self._backend.stop_move(card)
        except BaseException:
            _log.exception("fail-safe stop for card %d failed (%s)", card, reason)

    # Internals: motion

    @staticmethod
    def _check_move_result(command: str, code: int) -> None:
        if code != MoveResult.OK:
            raise error_for_move_code(command, code)

    def _wait_for_motion(
        self,
        command: str,
        timeout_s: float,
        at_target: Optional[Callable[[ArmState], bool]],
        start: Optional[ArmState],
        grace_s: Optional[float] = None,
    ) -> ArmState:
        """Poll until the arm stops. Callers run this inside an armed _Guard, which sends the stop on errors."""
        if not (math.isfinite(timeout_s) and timeout_s > 0):
            raise ZArmError(f"{command}: internal error, invalid timeout {timeout_s!r}")
        config = self._config
        started = self._clock()
        deadline = started + timeout_s
        grace_deadline = started + (config.motion_start_grace_s if grace_s is None else grace_s)
        baseline = None if start is None else self._limit_excess(start.joints)
        seen_moving = False
        while True:
            state = self.get_state()
            if baseline is None:
                baseline = self._limit_excess(state.joints)
            if not state.communicating:
                raise ConnectionLostError(f"{command}: the arm stopped communicating", state)
            if self._backend.is_collision(self._require_connected(command)):
                raise CollisionError(command, message=f"{command}: collision reported during motion", state=state)
            self._check_envelope(command, state, baseline)
            if state.moving:
                seen_moving = True
            elif seen_moving or state.timestamp_s >= grace_deadline or (at_target is not None and at_target(state)):
                break
            if state.timestamp_s >= deadline:
                raise MotionTimeoutError(command, message=f"{command}: still moving after {timeout_s:.2f} s", state=state)
            self._sleep(config.poll_interval_s)
        if at_target is not None and not at_target(state):
            raise MotionIncompleteError(command, message=f"{command}: the arm stopped away from the target", state=state)
        return state

    def _limit_excess(self, joints: JointPositions) -> Tuple[float, ...]:
        excess = []
        for spec in JOINT_SPECS:
            low, high = getattr(self._limits, spec.field)
            value = getattr(joints, spec.field)
            excess.append(max(0.0, low - value, value - high))
        return tuple(excess)

    def _check_envelope(self, command: str, state: ArmState, baseline: Tuple[float, ...]) -> None:
        """Raise if a joint has moved further outside the soft limits than it started."""
        for spec, now, before in zip(JOINT_SPECS, self._limit_excess(state.joints), baseline):
            margin = (
                self._config.arrival_tolerance_mm
                if spec.kind is JointKind.PRISMATIC
                else self._config.arrival_tolerance_deg
            )
            if now > before + margin:
                low, high = getattr(self._limits, spec.field)
                value = getattr(state.joints, spec.field)
                raise LimitExceededError(
                    command,
                    message=f"{command}: {spec.field}={value:g} left the soft limits [{low:g}, {high:g}] {spec.unit}",
                    state=state,
                )

    def _motion_timeout(self, estimate_s: float, explicit_s: Optional[float]) -> float:
        if explicit_s is not None:
            return explicit_s
        computed = self._config.motion_timeout_factor * estimate_s + self._config.motion_timeout_margin_s
        if not math.isfinite(computed):
            return self._config.max_motion_timeout_s
        return min(computed, self._config.max_motion_timeout_s)

    def _pose_move_seconds(self, start: ArmState, target: CartesianPose, speed: float) -> float:
        l1_mm, l2_mm = self._config.arm_lengths_mm
        other_axes = max(abs(target.z_mm - start.pose.z_mm), abs(target.r_deg - start.pose.r_deg))
        deltas = [
            max(abs(j1 - start.joints.j1_deg), abs(j2 - start.joints.j2_deg))
            for j1, j2 in kinematics.inverse_xy_solutions(target.x_mm, target.y_mm, l1_mm, l2_mm)
        ]
        planar = min(deltas) if deltas else 360.0
        return max(planar, other_axes) / speed

    def _joints_close(self, actual: JointPositions, target: JointPositions) -> bool:
        tol_deg = self._config.arrival_tolerance_deg
        tol_mm = self._config.arrival_tolerance_mm
        return (
            abs(actual.j1_deg - target.j1_deg) <= tol_deg
            and abs(actual.j2_deg - target.j2_deg) <= tol_deg
            and abs(actual.j3_mm - target.j3_mm) <= tol_mm
            and abs(actual.j4_deg - target.j4_deg) <= tol_deg
        )

    def _pose_close(self, actual: CartesianPose, target: CartesianPose, check_r: bool = True) -> bool:
        tol_mm = self._config.arrival_tolerance_mm
        close = (
            abs(actual.x_mm - target.x_mm) <= tol_mm
            and abs(actual.y_mm - target.y_mm) <= tol_mm
            and abs(actual.z_mm - target.z_mm) <= tol_mm
        )
        if check_r:
            close = close and abs(actual.r_deg - target.r_deg) <= self._config.arrival_tolerance_deg
        return close

    def _check_pose_solutions(self, start: ArmState, target: CartesianPose, lr: Optional[int]) -> None:
        """Check the J1/J2 solution(s) the controller may use against the soft limits (assumed kinematics)."""
        l1_mm, l2_mm = self._config.arm_lengths_mm
        if lr is None:
            elbow = kinematics.elbow_of(start.joints.j2_deg)
            solution = kinematics.inverse_xy(target.x_mm, target.y_mm, l1_mm, l2_mm, elbow)
            candidates = [] if solution is None else [solution]
        else:
            candidates = kinematics.inverse_xy_solutions(target.x_mm, target.y_mm, l1_mm, l2_mm)
        if not candidates:
            raise SoftLimitError("the target is out of the arm's reach")
        for j1, j2 in candidates:
            problems = self._limits.violations(JointPositions(j1, j2, target.z_mm, target.r_deg))
            if problems:
                hint = "" if lr is not None else "; try change_hand() first"
                raise SoftLimitError(f"the target needs joints outside the soft limits: {'; '.join(problems)}{hint}")

    def _check_linear_path(self, start: ArmState, target: CartesianPose) -> None:
        """Sample the straight path (every 2 mm at most) in the current hand configuration."""
        l1_mm, l2_mm = self._config.arm_lengths_mm
        elbow = kinematics.elbow_of(start.joints.j2_deg)
        baseline = self._limit_excess(start.joints)
        x0, y0, z0, r0 = start.pose.as_tuple()
        distance = math.hypot(target.x_mm - x0, target.y_mm - y0)
        steps = max(2, min(1000, int(math.ceil(distance / 2.0))))
        for step in range(1, steps + 1):
            t = step / steps
            x, y = x0 + (target.x_mm - x0) * t, y0 + (target.y_mm - y0) * t
            solution = kinematics.inverse_xy(x, y, l1_mm, l2_mm, elbow)
            if solution is None:
                raise SoftLimitError(f"the straight path leaves the arm's reach near ({x:.1f}, {y:.1f}) mm")
            joints = JointPositions(solution[0], solution[1], z0 + (target.z_mm - z0) * t, r0 + (target.r_deg - r0) * t)
            if any(now > before + 1e-9 for now, before in zip(self._limit_excess(joints), baseline)):
                problems = self._limits.violations(joints)
                raise SoftLimitError(
                    f"the straight path leaves the soft limits near ({x:.1f}, {y:.1f}) mm: {'; '.join(problems)}"
                )

    # Internals: validation

    @staticmethod
    def _require_finite_telemetry(source: str, values: Tuple[float, ...]) -> None:
        if not all(math.isfinite(value) for value in values):
            raise InvalidTelemetryError(f"{source} returned non-finite values {values}")

    def _checked_joints(self, target: JointPositions) -> JointPositions:
        if not isinstance(target, JointPositions):
            raise InvalidArgumentError(f"target must be a JointPositions, got {type(target).__name__}")
        checked = JointPositions(*(require_c_float(spec.field, getattr(target, spec.field)) for spec in JOINT_SPECS))
        problems = self._limits.violations(checked)
        if problems:
            raise SoftLimitError("target is outside the soft limits: " + "; ".join(problems))
        return checked

    def _checked_pose(self, target: CartesianPose) -> CartesianPose:
        if not isinstance(target, CartesianPose):
            raise InvalidArgumentError(f"target must be a CartesianPose, got {type(target).__name__}")
        checked = CartesianPose(
            require_c_float("x_mm", target.x_mm),
            require_c_float("y_mm", target.y_mm),
            require_c_float("z_mm", target.z_mm),
            require_c_float("r_deg", target.r_deg),
        )
        problems = []
        z_low, z_high = self._limits.j3_mm
        if not z_low <= checked.z_mm <= z_high:
            problems.append(f"z_mm={checked.z_mm:g} is outside [{z_low:g}, {z_high:g}] mm")
        r_low, r_high = self._limits.j4_deg
        if not r_low <= checked.r_deg <= r_high:
            problems.append(f"r_deg={checked.r_deg:g} is outside [{r_low:g}, {r_high:g}] deg")
        inner, outer = kinematics.reach_mm(*self._config.arm_lengths_mm)
        radius = math.hypot(checked.x_mm, checked.y_mm)
        if not inner - 1e-6 <= radius <= outer + 1e-6:
            problems.append(f"(x, y) is {radius:.1f} mm from the J1 axis; reach is [{inner:g}, {outer:g}] mm")
        if problems:
            raise SoftLimitError("target is outside the soft limits: " + "; ".join(problems))
        return checked

    @staticmethod
    def _checked_speed(name: str, value: float, cap: float) -> float:
        speed = require_c_float(name, value)
        if speed <= 0 or to_float32(speed) <= 0:
            raise InvalidArgumentError(f"{name} must be > 0 (as a C float), got {speed:g}")
        if speed > cap:
            raise InvalidArgumentError(f"{name}={speed:g} exceeds the configured cap {cap:g}; raise it in ZArmConfig")
        return speed

    @staticmethod
    def _checked_roughly(value: float) -> float:
        roughly = require_c_float("roughly", value)
        if not 0.0 <= roughly <= 1.0:
            raise InvalidArgumentError(f"roughly must be within [0, 1], got {roughly:g}")
        return roughly

    @staticmethod
    def _checked_lr(value: Optional[int]) -> Optional[int]:
        if value is None:
            return None
        if isinstance(value, bool) or not isinstance(value, int) or value not in LR_VALUES:
            raise InvalidArgumentError(f"lr must be None or one of {LR_VALUES}, got {value!r}")
        return value

    @staticmethod
    def _positive(name: str, value: float) -> float:
        number = require_number(name, value)
        if number <= 0:
            raise InvalidArgumentError(f"{name} must be > 0, got {number:g}")
        return number

    def _checked_timeout(self, value: Optional[float]) -> Optional[float]:
        return None if value is None else self._positive("timeout_s", value)

    @staticmethod
    def _checked_flag(name: str, value: bool) -> bool:
        if not isinstance(value, bool):
            raise InvalidArgumentError(f"{name} must be a bool, got {value!r}")
        return value

    @staticmethod
    def _checked_io(io: int) -> int:
        return require_c_int("io", io, 0, MAX_IO_INDEX)

    @staticmethod
    def _joint_number(joint: Joint) -> int:
        try:
            return int(Joint(joint))
        except (ValueError, TypeError):
            raise InvalidArgumentError(f"joint must be a Joint (1..4), got {joint!r}") from None

    @staticmethod
    def _io_level(command: str, io: int, value: int) -> int:
        if value == -1:
            raise InvalidArgumentError(f"{command}: io {io} does not exist on this arm")
        if value == 3:
            raise NotConnectedError(f"{command}: the arm is not connected")
        if value not in (0, 1):
            raise CommandError(command, message=f"{command} returned unexpected value {value}")
        return value

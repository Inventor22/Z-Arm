"""In-memory simulator of Hitbot's native library, for tests and offline work.

It follows the vendor behaviour the SDK relies on, as read from the x64 DLL
(see docs/native-api.md): result codes and the order the library checks them
in, the moving and initialized flags (homing clears the latter), card-id
handling, and the rule that every ``robot_*`` call needs a successful
``net_port_initial`` first. Calls that would crash or misbehave with the real
DLL raise :class:`FakeBackendMisuseError` instead.

Where the real behaviour is unknown, the simulator follows the SDK's default
assumptions (README, "Default assumptions"):

* The arm is a Z-Arm 2442: 220 mm and 200 mm links, and the rated joint ranges
  (``JointLimits.rated``) are the controller's hard limits.
* ``set_arm_length`` arguments are recorded but don't change the kinematics.
* ``joint_home`` moves the joint to 0 without setting the moving flag.
* ``emergency_stop`` latches: the arm is uninitialized afterwards.
* ``movej_xyz_lr``: lr > 0 selects J2 >= 0, lr < 0 selects J2 <= 0, 0 keeps the current hand.
* 9 digital inputs and 3 outputs. An input with no signal reads 1 (0 means 24 V present);
  an output reads back what was last written.

Motion follows the vendor header's speed model: each joint needs
``|delta| / speed`` seconds and the slowest joint sets the duration.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, Iterable, List, Optional, Tuple

from . import kinematics
from .backend import Backend, EncoderParam, ScaraParam
from .types import DEFAULT_ARM_LENGTHS_MM, JointLimits, JointPositions

Joints = Tuple[float, float, float, float]

INPUT_NO_SIGNAL = 1  # assumed: the DLL reports 0 when 24 V is present


class FakeBackendMisuseError(AssertionError):
    """The SDK used the backend in a way that would crash or misbehave with the real DLL."""


class ManualClock:
    """A clock that only moves when told to. Pass ``clock=c`` and ``sleep=c.sleep``."""

    def __init__(self, start_s: float = 0.0) -> None:
        self._now = float(start_s)
        self._lock = threading.Lock()

    def __call__(self) -> float:
        with self._lock:
            return self._now

    def advance(self, seconds: float) -> None:
        if seconds < 0:
            raise ValueError("a ManualClock cannot go backwards")
        with self._lock:
            self._now += seconds

    def sleep(self, seconds: float) -> None:
        if seconds < 0:
            raise ValueError("sleep length must be non-negative")
        self.advance(seconds)


@dataclass
class _Motion:
    start_s: float
    duration_s: float
    start: Joints
    end: Joints
    linear_xy: Optional[Tuple[Tuple[float, float], Tuple[float, float], int]] = None
    paused_at_s: Optional[float] = None
    homing: bool = False


@dataclass
class FakeArm:
    """State of one simulated arm. Tests may read and change it directly.

    ``arm_lengths_mm`` are the physical link lengths the simulation uses;
    ``vendor_arm_lengths_mm`` records the last ``set_arm_length`` call.
    """

    card: int
    present: bool = True
    initialized: bool = False
    servo_off: bool = False
    mobile_terminal: bool = False
    collision: bool = False
    stalled: bool = False
    drag_teach: bool = False
    cooperation: bool = False
    joints: Joints = (0.0, 0.0, 0.0, 0.0)
    arm_lengths_mm: Tuple[float, float] = DEFAULT_ARM_LENGTHS_MM
    vendor_arm_lengths_mm: Optional[Tuple[float, float]] = None
    joint_state_overrides: Dict[int, int] = field(default_factory=dict)
    init_results: List[int] = field(default_factory=list)
    forced_results: Dict[str, int] = field(default_factory=dict)
    digital_in: Dict[int, int] = field(default_factory=dict)
    digital_out: Dict[int, int] = field(default_factory=dict)
    motion: Optional[_Motion] = None
    stop_count: int = 0
    emergency_stop_count: int = 0


def _lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


class FakeBackend(Backend):
    """Simulated arms keyed by card id.

    ``hard_limits`` defaults to ``JointLimits.rated(z_travel_mm)``.
    ``digital_io_count``, if given, sets both I/O counts (kept for older callers).
    """

    def __init__(
        self,
        cards: Iterable[int] = (1,),
        *,
        clock: Optional[Callable[[], float]] = None,
        arm_lengths_mm: Tuple[float, float] = DEFAULT_ARM_LENGTHS_MM,
        z_travel_mm: float = 310.0,
        hard_limits: Optional[JointLimits] = None,
        server_start_result: int = 1,
        connect_delay_s: float = 0.0,
        digital_input_count: int = 9,
        digital_output_count: int = 3,
        digital_io_count: Optional[int] = None,
        home_speed: float = 30.0,
    ) -> None:
        super().__init__()
        self.clock: Callable[[], float] = clock or time.monotonic
        self.arm_lengths_mm = (float(arm_lengths_mm[0]), float(arm_lengths_mm[1]))
        self.arms: Dict[int, FakeArm] = {
            card: FakeArm(card, arm_lengths_mm=self.arm_lengths_mm) for card in cards
        }
        self.hard_limits = hard_limits or JointLimits.rated(z_travel_mm)
        self.server_start_result = server_start_result
        self.connect_delay_s = connect_delay_s
        if digital_io_count is not None:
            digital_input_count = digital_output_count = digital_io_count
        self.digital_input_count = digital_input_count
        self.digital_output_count = digital_output_count
        self.home_speed = home_speed
        self.server_running = False
        self.calls: List[Tuple[str, tuple]] = []
        self._server_started_at: Optional[float] = None
        self._lock = threading.RLock()

    # Helpers

    def _record(self, name: str, *args: object) -> None:
        self.calls.append((name, args))

    def call_names(self) -> List[str]:
        return [name for name, _ in self.calls]

    def _require_server(self, name: str) -> None:
        if not self.server_running:
            raise FakeBackendMisuseError(
                f"{name} called before a successful start_server(); the real DLL dereferences "
                "shared memory that net_port_initial creates"
            )

    def _arm(self, name: str, card: int) -> FakeArm:
        self._require_server(name)
        if isinstance(card, bool) or not isinstance(card, int) or not 0 <= card <= 255:
            raise FakeBackendMisuseError(
                f"{name} called with card={card!r}; the real DLL indexes a 256-entry table without bounds checks"
            )
        arm = self.arms.get(card)
        if arm is None:
            arm = FakeArm(card, present=False, arm_lengths_mm=self.arm_lengths_mm)
            self.arms[card] = arm
        return arm

    def _connected(self, arm: FakeArm) -> bool:
        if not (self.server_running and arm.present and self._server_started_at is not None):
            return False
        return self.clock() - self._server_started_at >= self.connect_delay_s

    def _fraction(self, arm: FakeArm, motion: _Motion, now: float) -> float:
        at = motion.paused_at_s if motion.paused_at_s is not None else now
        fraction = 1.0 if motion.duration_s <= 0 else (at - motion.start_s) / motion.duration_s
        fraction = max(0.0, min(1.0, fraction))
        if arm.stalled:
            fraction = min(fraction, 0.5)
        return fraction

    def _joints_at(self, arm: FakeArm, motion: _Motion, fraction: float) -> Joints:
        if motion.linear_xy is None:
            return tuple(_lerp(a, b, fraction) for a, b in zip(motion.start, motion.end))  # type: ignore[return-value]
        (x0, y0), (x1, y1), elbow = motion.linear_xy
        l1, l2 = arm.arm_lengths_mm
        solution = kinematics.inverse_xy(_lerp(x0, x1, fraction), _lerp(y0, y1, fraction), l1, l2, elbow)
        if solution is None:
            j1, j2 = _lerp(motion.start[0], motion.end[0], fraction), _lerp(motion.start[1], motion.end[1], fraction)
        else:
            j1, j2 = solution
        return (
            j1,
            j2,
            _lerp(motion.start[2], motion.end[2], fraction),
            _lerp(motion.start[3], motion.end[3], fraction),
        )

    def _settle(self, arm: FakeArm, now: float) -> Joints:
        """Advance ``arm`` to ``now`` and return its joints."""
        motion = arm.motion
        if motion is None:
            return arm.joints
        fraction = self._fraction(arm, motion, now)
        if fraction >= 1.0 and motion.paused_at_s is None and not arm.stalled:
            arm.joints = motion.end
            arm.motion = None
            return arm.joints
        return self._joints_at(arm, motion, fraction)

    def _freeze(self, arm: FakeArm) -> None:
        arm.joints = self._settle(arm, self.clock())
        arm.motion = None

    @staticmethod
    def _moving(arm: FakeArm) -> bool:
        """The vendor's moving flag. Assumed: homing does not set it."""
        return arm.motion is not None and not arm.motion.homing

    @staticmethod
    def _homing(arm: FakeArm) -> bool:
        return arm.motion is not None and arm.motion.homing

    def _violates(self, target: Joints) -> bool:
        return bool(self.hard_limits.violations(JointPositions(*target)))

    def _joint_state(self, arm: FakeArm, joint: int) -> int:
        if joint not in (1, 2, 3, 4):
            return 2
        if joint in arm.joint_state_overrides:
            return arm.joint_state_overrides[joint]
        if joint == 4:
            return 1
        if not arm.initialized:
            return 3
        if arm.collision:
            return 5
        return 1

    def _preamble(
        self,
        arm: FakeArm,
        name: str,
        values: Tuple[float, ...],
        speed: float,
        roughly: Optional[float],
        wrapper_checks: bool = True,
    ) -> Optional[int]:
        """Checks in the order the vendor library performs them; None means proceed."""
        if name in arm.forced_results:
            return arm.forced_results.pop(name)
        if wrapper_checks:
            if arm.collision:
                return 102
            for joint in (1, 2, 3, 4):
                state = self._joint_state(arm, joint)
                if state == 0:
                    return 103
                if state in (7, 8):
                    return 105
        checked = values + (speed,) + (() if roughly is None else (roughly,))
        if any(math.isnan(value) for value in checked):
            return 101
        self._settle(arm, self.clock())
        if self._moving(arm):
            return 0
        if roughly is not None and not 0.0 <= roughly <= 1.0:
            # The real library returns garbage (the card id) here; mimic it.
            return arm.card
        if speed <= 0:
            return 2
        if not arm.initialized or not self._connected(arm):
            return 3
        if arm.servo_off:
            return 6
        if arm.mobile_terminal:
            return 11
        return None

    def _start_joint_motion(self, arm: FakeArm, target: Joints, speed: float, homing: bool = False) -> None:
        now = self.clock()
        start = self._settle(arm, now)
        duration = max(abs(b - a) for a, b in zip(start, target)) / speed
        if duration <= 0:
            arm.joints = target
            arm.motion = None
        else:
            arm.motion = _Motion(now, duration, start, target, homing=homing)

    def _solve(self, arm: FakeArm, x: float, y: float, z: float, r: float, elbow: int) -> Optional[Joints]:
        l1, l2 = arm.arm_lengths_mm
        solution = kinematics.inverse_xy(x, y, l1, l2, elbow)
        if solution is None:
            return None
        target = (solution[0], solution[1], z, r)
        return None if self._violates(target) else target

    # Server and discovery

    def start_server(self) -> int:
        with self._lock:
            self._record("start_server")
            if self.server_start_result == 1:
                if not self.server_running:
                    self._server_started_at = self.clock()
                self.server_running = True
            return self.server_start_result

    def close_server(self) -> None:
        with self._lock:
            self._record("close_server")
            self._require_server("close_server")
            self.server_running = False
            self._server_started_at = None

    def card_connect_status(self, card: int) -> int:
        with self._lock:
            self._record("card_connect_status", card)
            self._require_server("card_connect_status")
            if not 0 <= card <= 255:
                return 2
            arm = self.arms.get(card)
            return 1 if arm is not None and self._connected(arm) else 0

    # Setup

    def initial(self, card: int, generation: int, z_travel_mm: int) -> int:
        with self._lock:
            self._record("initial", card, generation, z_travel_mm)
            arm = self._arm("initial", card)
            for name, value in (("generation", generation), ("z_travel_mm", z_travel_mm)):
                if isinstance(value, bool) or not isinstance(value, int):
                    raise FakeBackendMisuseError(f"initial: {name} must be an int (C API int), got {value!r}")
            if not self._connected(arm):
                return 0
            if arm.init_results:
                code = arm.init_results.pop(0)
                if code == 1:
                    arm.initialized = True
                return code
            if not 1 <= generation <= 8:
                return 2
            if arm.mobile_terminal:
                return 11
            if z_travel_mm <= 0:
                return 12
            if any(arm.joint_state_overrides.get(joint) == 7 for joint in (1, 2, 3)):
                return 105
            self._settle(arm, self.clock())
            if self._homing(arm):
                # Legacy notes saw initial() return 3 (encoder error) straight after joint_home.
                return 3
            arm.initialized = True
            return 1

    def unlock_position(self, card: int) -> int:
        with self._lock:
            self._record("unlock_position", card)
            return 1 if self._connected(self._arm("unlock_position", card)) else 0

    def set_arm_length(self, card: int, l1_mm: float, l2_mm: float) -> None:
        """Records the arguments. The simulated kinematics keep using the physical lengths."""
        with self._lock:
            self._record("set_arm_length", card, l1_mm, l2_mm)
            arm = self._arm("set_arm_length", card)
            self._settle(arm, self.clock())
            if arm.motion is not None:
                raise FakeBackendMisuseError(
                    "set_arm_length while the arm is moving or homing (legacy code warns against this)"
                )
            arm.vendor_arm_lengths_mm = (float(l1_mm), float(l2_mm))

    # State

    def get_scara_param(self, card: int) -> ScaraParam:
        with self._lock:
            arm = self._arm("get_scara_param", card)
            j1, j2, z, r = self._settle(arm, self.clock())
            x, y = kinematics.forward_xy(j1, j2, *arm.arm_lengths_mm)
            return ScaraParam(
                x_mm=x,
                y_mm=y,
                z_mm=z,
                angle1_deg=j1,
                angle2_deg=j2,
                r_deg=r,
                communicate_success=self._connected(arm),
                initial_finish=arm.initialized,
                move_flag=self._moving(arm),
            )

    def get_encoder_param(self, card: int) -> EncoderParam:
        with self._lock:
            param = self.get_scara_param(card)
            return EncoderParam(param.x_mm, param.y_mm, param.z_mm, param.angle1_deg, param.angle2_deg, param.r_deg)

    def get_joint_state(self, card: int, joint: int) -> int:
        with self._lock:
            return self._joint_state(self._arm("get_joint_state", card), joint)

    def is_collision(self, card: int) -> bool:
        with self._lock:
            return self._arm("is_collision", card).collision

    def is_stop(self, card: int) -> bool:
        with self._lock:
            arm = self._arm("is_stop", card)
            self._settle(arm, self.clock())
            return not self._moving(arm)

    # Motion

    def movej_angle(
        self, card: int, angle1_deg: float, angle2_deg: float, z_mm: float, r_deg: float, speed: float, roughly: float
    ) -> int:
        with self._lock:
            self._record("movej_angle", card, angle1_deg, angle2_deg, z_mm, r_deg, speed, roughly)
            arm = self._arm("movej_angle", card)
            target = (angle1_deg, angle2_deg, z_mm, r_deg)
            code = self._preamble(arm, "movej_angle", target, speed, roughly)
            if code is not None:
                return code
            if self._violates(target):
                return 4
            self._start_joint_motion(arm, target, speed)
            return 1

    def movej_xyz(
        self, card: int, x_mm: float, y_mm: float, z_mm: float, r_deg: float, speed: float, roughly: float
    ) -> int:
        with self._lock:
            self._record("movej_xyz", card, x_mm, y_mm, z_mm, r_deg, speed, roughly)
            arm = self._arm("movej_xyz", card)
            code = self._preamble(arm, "movej_xyz", (x_mm, y_mm, z_mm, r_deg), speed, roughly)
            if code is not None:
                return code
            elbow = kinematics.elbow_of(self._settle(arm, self.clock())[1])
            target = self._solve(arm, x_mm, y_mm, z_mm, r_deg, elbow) or self._solve(
                arm, x_mm, y_mm, z_mm, r_deg, -elbow
            )
            if target is None:
                return 4
            self._start_joint_motion(arm, target, speed)
            return 1

    def movej_xyz_lr(
        self,
        card: int,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        r_deg: float,
        speed: float,
        roughly: float,
        lr: int,
    ) -> int:
        with self._lock:
            self._record("movej_xyz_lr", card, x_mm, y_mm, z_mm, r_deg, speed, roughly, lr)
            arm = self._arm("movej_xyz_lr", card)
            if isinstance(lr, bool) or not isinstance(lr, int):
                raise FakeBackendMisuseError(f"movej_xyz_lr: lr must be an int (C API int), got {lr!r}")
            code = self._preamble(arm, "movej_xyz_lr", (x_mm, y_mm, z_mm, r_deg), speed, roughly)
            if code is not None:
                return code
            # Assumed convention (unverified): lr > 0 selects J2 >= 0, lr < 0 selects J2 <= 0, 0 keeps the hand.
            elbow = 1 if lr > 0 else -1 if lr < 0 else kinematics.elbow_of(arm.joints[1])
            target = self._solve(arm, x_mm, y_mm, z_mm, r_deg, elbow)
            if target is None:
                return 7
            self._start_joint_motion(arm, target, speed)
            return 1

    def movel_xyz(self, card: int, x_mm: float, y_mm: float, z_mm: float, r_deg: float, speed_mm_s: float) -> int:
        with self._lock:
            self._record("movel_xyz", card, x_mm, y_mm, z_mm, r_deg, speed_mm_s)
            arm = self._arm("movel_xyz", card)
            code = self._preamble(arm, "movel_xyz", (x_mm, y_mm, z_mm, r_deg), speed_mm_s, None)
            if code is not None:
                return code
            now = self.clock()
            start = self._settle(arm, now)
            l1, l2 = arm.arm_lengths_mm
            x0, y0 = kinematics.forward_xy(start[0], start[1], l1, l2)
            elbow = kinematics.elbow_of(start[1])
            steps = 50
            end: Optional[Joints] = None
            for step in range(1, steps + 1):
                t = step / steps
                end = self._solve(
                    arm, _lerp(x0, x_mm, t), _lerp(y0, y_mm, t), _lerp(start[2], z_mm, t), _lerp(start[3], r_deg, t), elbow
                )
                if end is None:
                    return 4
            assert end is not None
            distance = math.sqrt((x_mm - x0) ** 2 + (y_mm - y0) ** 2 + (z_mm - start[2]) ** 2)
            duration = max(distance, abs(r_deg - start[3])) / speed_mm_s
            if duration <= 0:
                arm.joints = end
            else:
                arm.motion = _Motion(now, duration, start, end, linear_xy=((x0, y0), (x_mm, y_mm), elbow))
            return 1

    def change_attitude(self, card: int, speed: float) -> int:
        with self._lock:
            self._record("change_attitude", card, speed)
            arm = self._arm("change_attitude", card)
            code = self._preamble(arm, "change_attitude", (), speed, None, wrapper_checks=False)
            if code is not None:
                return code
            j1, j2, z, r = self._settle(arm, self.clock())
            new_j1, new_j2 = kinematics.alternate_hand(j1, j2, *arm.arm_lengths_mm)
            target = (new_j1, new_j2, z, r)
            if self._violates(target):
                return 4
            self._start_joint_motion(arm, target, speed)
            return 1

    def joint_home(self, card: int, joint: int) -> int:
        with self._lock:
            self._record("joint_home", card, joint)
            arm = self._arm("joint_home", card)
            arm.initialized = False  # the DLL clears this before any check, even when it refuses
            if "joint_home" in arm.forced_results:
                return arm.forced_results.pop("joint_home")
            if not self._connected(arm):
                return 0
            if joint not in (1, 2, 3, 4):
                return 2
            self._freeze(arm)
            target = list(arm.joints)
            target[joint - 1] = 0.0
            self._start_joint_motion(arm, tuple(target), self.home_speed, homing=True)  # type: ignore[arg-type]
            return 1

    def stop_move(self, card: int) -> None:
        with self._lock:
            self._record("stop_move", card)
            arm = self._arm("stop_move", card)
            arm.stop_count += 1
            self._freeze(arm)

    def emergency_stop(self, card: int) -> None:
        """Stops the arm and latches: it stays uninitialized until initial() succeeds again (assumed)."""
        with self._lock:
            self._record("emergency_stop", card)
            arm = self._arm("emergency_stop", card)
            arm.emergency_stop_count += 1
            self._freeze(arm)
            arm.initialized = False

    def pause_move(self, card: int) -> int:
        with self._lock:
            self._record("pause_move", card)
            arm = self._arm("pause_move", card)
            if not self._connected(arm):
                return 0
            self._settle(arm, self.clock())
            if arm.motion is not None and arm.motion.paused_at_s is None:
                arm.motion.paused_at_s = self.clock()
            return 1

    def resume_move(self, card: int) -> int:
        with self._lock:
            self._record("resume_move", card)
            arm = self._arm("resume_move", card)
            if not self._connected(arm):
                return 0
            motion = arm.motion
            if motion is not None and motion.paused_at_s is not None:
                motion.start_s += self.clock() - motion.paused_at_s
                motion.paused_at_s = None
            return 1

    # Modes and I/O

    def set_drag_teach(self, card: int, enable: bool) -> bool:
        with self._lock:
            self._record("set_drag_teach", card, enable)
            arm = self._arm("set_drag_teach", card)
            if not self._connected(arm):
                return False
            arm.drag_teach = bool(enable)
            return True

    def get_drag_teach(self, card: int) -> bool:
        with self._lock:
            return self._arm("get_drag_teach", card).drag_teach

    def set_cooperation_fun_state(self, card: int, enable: bool) -> bool:
        with self._lock:
            self._record("set_cooperation_fun_state", card, enable)
            arm = self._arm("set_cooperation_fun_state", card)
            if not self._connected(arm):
                return False
            arm.cooperation = bool(enable)
            return True

    def get_cooperation_fun_state(self, card: int) -> bool:
        with self._lock:
            return self._arm("get_cooperation_fun_state", card).cooperation

    def get_digital_io_in(self, card: int, io: int) -> int:
        with self._lock:
            arm = self._arm("get_digital_io_in", card)
            if not self._connected(arm):
                return 3
            if not 0 <= io < self.digital_input_count:
                return -1
            return arm.digital_in.get(io, INPUT_NO_SIGNAL)

    def get_digital_io_out(self, card: int, io: int) -> int:
        with self._lock:
            arm = self._arm("get_digital_io_out", card)
            if not self._connected(arm):
                return 3
            if not 0 <= io < self.digital_output_count:
                return -1
            return arm.digital_out.get(io, 0)

    def set_digital_io_out(self, card: int, io: int, value: bool) -> bool:
        with self._lock:
            self._record("set_digital_io_out", card, io, value)
            arm = self._arm("set_digital_io_out", card)
            if not self._connected(arm) or not 0 <= io < self.digital_output_count:
                return False
            arm.digital_out[io] = 1 if value else 0
            return True

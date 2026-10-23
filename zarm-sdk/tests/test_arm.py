"""ZArm behaviour against the simulator, with a deterministic clock."""

import dataclasses
import threading
import time
import unittest

from zarm import (
    AmbiguousArmError,
    ArmBusyError,
    ArmNotFoundError,
    ArmPhase,
    CartesianPose,
    CollisionError,
    CommandError,
    ConnectionLostError,
    FakeBackend,
    InitializationError,
    InitResult,
    InvalidArgumentError,
    InvalidTelemetryError,
    Joint,
    JointFaultError,
    JointLimits,
    JointPositions,
    JointState,
    LimitExceededError,
    ManualClock,
    MotionIncompleteError,
    MotionTimeoutError,
    NotConnectedError,
    NotInitializedError,
    OwnershipError,
    ServerStartError,
    SoftLimitError,
    UnreachableError,
    ZArm,
    ZArmConfig,
    ZArmError,
)
from zarm.arm import _stop_open_arms_at_exit

from .support import ArmTestCase, make_arm, ready_arm

TARGET = JointPositions(30.0, 45.0, -50.0, 10.0)


class ConnectTests(ArmTestCase):
    def test_connect_with_configured_card(self):
        arm, fake, _ = make_arm()
        self.assertEqual(arm.connect(), 59)
        self.assertIs(arm.phase, ArmPhase.CONNECTED)
        self.assertEqual(fake.call_names()[:2], ["start_server", "card_connect_status"])
        self.assertEqual(arm.connect(), 59)  # idempotent

    def test_discovery_of_a_single_arm(self):
        arm, _, _ = make_arm(cards=(17,), config=ZArmConfig())
        self.assertEqual(arm.connect(), 17)

    def test_discovery_refuses_to_guess(self):
        arm, _, _ = make_arm(cards=(3, 59), config=ZArmConfig())
        with self.assertRaises(AmbiguousArmError) as caught:
            arm.connect()
        self.assertEqual(caught.exception.card_ids, (3, 59))
        self.assertIs(arm.phase, ArmPhase.CREATED)

    def test_arm_not_found_after_timeout(self):
        arm, _, clock = make_arm(cards=(1,))
        with self.assertRaises(ArmNotFoundError):
            arm.connect(timeout_s=2.0)
        self.assertGreaterEqual(clock(), 2.0)

    def test_waits_for_the_arm_to_connect(self):
        arm, _, clock = make_arm(connect_delay_s=1.2)
        self.assertEqual(arm.connect(timeout_s=5.0), 59)
        self.assertGreaterEqual(clock(), 1.2)

    def test_server_start_failure(self):
        arm, _, _ = make_arm(server_start_result=-1)
        with self.assertRaises(ServerStartError):
            arm.connect()

    def test_single_owner(self):
        arm, fake, clock = make_arm()
        other = ZArm(fake, ZArmConfig(card_id=59), clock=clock, sleep=clock.sleep)
        arm.connect()
        with self.assertRaises(OwnershipError):
            other.connect()
        arm.close()
        self.assertEqual(other.connect(), 59)
        other.close()

    def test_closed_arm_cannot_reconnect(self):
        arm, _, _ = make_arm()
        arm.close()
        with self.assertRaises(ZArmError):
            arm.connect()

    def test_operations_need_a_connection(self):
        arm, fake, _ = make_arm()
        for call in (arm.get_state, arm.stop, arm.is_moving, lambda: arm.read_digital_input(0)):
            with self.subTest(call=call), self.assertRaises(NotConnectedError):
                call()
        self.assertEqual(fake.calls, [])


class InitializeTests(ArmTestCase):
    def test_follows_the_legacy_sequence(self):
        arm, fake, _ = make_arm()
        arm.connect()
        arm.initialize()
        self.assertIs(arm.phase, ArmPhase.INITIALIZED)
        names = [name for name in fake.call_names() if name in ("initial", "unlock_position", "set_arm_length")]
        self.assertEqual(names, ["initial", "unlock_position", "set_arm_length"])
        self.assertIn(("initial", (59, 1, 310)), fake.calls)
        self.assertIn(("set_arm_length", (59, 200.0, 200.0)), fake.calls)

    def test_retries_transient_codes(self):
        arm, fake, clock = make_arm()
        fake.arms[59].init_results.extend([0, 3])
        arm.connect()
        arm.initialize(retry_delay_s=1.0)
        self.assertEqual(fake.call_names().count("initial"), 3)
        self.assertGreaterEqual(clock(), 2.0)

    def test_gives_up_after_the_attempts(self):
        arm, fake, _ = make_arm()
        fake.arms[59].init_results.extend([3] * 10)
        arm.connect()
        with self.assertRaises(InitializationError) as caught:
            arm.initialize(attempts=4)
        self.assertEqual(caught.exception.attempts, 4)
        self.assertIs(caught.exception.code, InitResult.ENCODER_ERROR)
        self.assertIs(arm.phase, ArmPhase.CONNECTED)

    def test_does_not_retry_configuration_errors(self):
        arm, fake, _ = make_arm(config=ZArmConfig(card_id=59, generation=1))
        fake.arms[59].init_results.append(12)
        arm.connect()
        with self.assertRaises(InitializationError) as caught:
            arm.initialize()
        self.assertEqual(caught.exception.attempts, 1)
        self.assertEqual(fake.call_names().count("initial"), 1)

    def test_refuses_while_moving(self):
        arm, fake, _ = ready_arm()
        arm.move_joints(TARGET, speed=10.0, wait=False)
        with self.assertRaises(ArmBusyError):
            arm.initialize()
        self.assertEqual(fake.call_names().count("initial"), 1)
        self.assertEqual(fake.call_names().count("set_arm_length"), 1)

    def test_moves_need_initialization(self):
        arm, fake, _ = make_arm()
        arm.connect()
        with self.assertRaises(NotInitializedError):
            arm.move_joints(TARGET, speed=10.0)
        self.assertNotIn("movej_angle", fake.call_names())


class MoveTests(ArmTestCase):
    def test_move_joints(self):
        arm, _, clock = ready_arm()
        state = arm.move_joints(TARGET, speed=50.0)
        self.assertEqual(state.joints, TARGET)
        self.assertFalse(state.moving)
        self.assertGreaterEqual(clock(), 1.0)
        self.assertEqual(arm.get_state().joints, TARGET)

    def test_move_to_current_position_returns_at_once(self):
        arm, _, _ = ready_arm()
        here = arm.get_state().joints
        self.assertEqual(arm.move_joints(here, speed=10.0).joints, here)

    def test_move_to_pose(self):
        arm, _, _ = ready_arm()
        target = CartesianPose(250.0, -100.0, -20.0, 5.0)
        state = arm.move_to_pose(target, speed=50.0)
        self.assertAlmostEqual(state.pose.x_mm, 250.0)
        self.assertAlmostEqual(state.pose.y_mm, -100.0)

    def test_move_to_pose_with_hand(self):
        arm, fake, _ = ready_arm()
        arm.move_to_pose(CartesianPose(250.0, -100.0, -20.0, 0.0), speed=50.0, lr=-1)
        self.assertIn("movej_xyz_lr", fake.call_names())
        self.assertLessEqual(arm.get_state().joints.j2_deg, 0.0)
        with self.assertRaises(InvalidArgumentError):
            arm.move_to_pose(CartesianPose(250.0, 0.0, 0.0, 0.0), speed=50.0, lr=True)

    def test_move_linear(self):
        arm, _, _ = ready_arm()
        arm.move_to_pose(CartesianPose(250.0, -100.0, -20.0, 0.0), speed=50.0)
        state = arm.move_linear(CartesianPose(250.0, 100.0, -20.0, 0.0), speed_mm_s=50.0)
        self.assertAlmostEqual(state.pose.y_mm, 100.0)

    def test_change_hand(self):
        arm, _, _ = ready_arm()
        arm.move_joints(JointPositions(30.0, 45.0, 0.0, 0.0), speed=50.0)
        state = arm.change_hand(speed=50.0)
        self.assertLess(state.joints.j2_deg, 0.0)

    def test_roughly_skips_the_arrival_check(self):
        arm, _, _ = ready_arm()
        self.assertIsNotNone(arm.move_joints(TARGET, speed=50.0, roughly=0.5))

    def test_no_wait(self):
        arm, _, clock = ready_arm()
        self.assertIsNone(arm.move_joints(TARGET, speed=10.0, wait=False))
        self.assertTrue(arm.is_moving())
        state = arm.wait_until_stopped(timeout_s=10.0)
        self.assertEqual(state.joints, TARGET)
        self.assertFalse(arm.is_moving())


class ValidationTests(ArmTestCase):
    def assert_rejected_locally(self, arm, fake, call, error=InvalidArgumentError):
        before = len(fake.calls)
        with self.assertRaises(error):
            call()
        self.assertEqual(len(fake.calls), before, "nothing may be sent to the arm")

    def test_rejected_targets(self):
        arm, fake, _ = ready_arm()
        cases = [
            (lambda: arm.move_joints(JointPositions(float("nan"), 0, 0, 0), speed=10), InvalidArgumentError),
            (lambda: arm.move_joints(JointPositions(float("inf"), 0, 0, 0), speed=10), InvalidArgumentError),
            (lambda: arm.move_joints(JointPositions(True, 0, 0, 0), speed=10), InvalidArgumentError),
            (lambda: arm.move_joints((0, 0, 0, 0), speed=10), InvalidArgumentError),
            (lambda: arm.move_joints(JointPositions(95, 0, 0, 0), speed=10), SoftLimitError),
            (lambda: arm.move_joints(JointPositions(0, 0, 10, 0), speed=10), SoftLimitError),
            (lambda: arm.move_to_pose(CartesianPose(450, 0, 0, 0), speed=10), SoftLimitError),
            (lambda: arm.move_to_pose(CartesianPose(200, 0, -400, 0), speed=10), SoftLimitError),
            (lambda: arm.move_linear(CartesianPose(200, 0, 0, 270), speed_mm_s=10), SoftLimitError),
        ]
        for call, error in cases:
            with self.subTest(error=error):
                self.assert_rejected_locally(arm, fake, call, error)

    def test_rejected_speeds_and_roughly(self):
        arm, fake, _ = ready_arm()
        for kwargs in ({"speed": 0}, {"speed": -5}, {"speed": 101}, {"speed": 10, "roughly": 1.5}, {"speed": 10, "roughly": -0.1}):
            with self.subTest(kwargs=kwargs):
                self.assert_rejected_locally(arm, fake, lambda: arm.move_joints(TARGET, **kwargs))
        self.assert_rejected_locally(arm, fake, lambda: arm.move_linear(CartesianPose(250, 0, 0, 0), speed_mm_s=500))
        self.assert_rejected_locally(arm, fake, lambda: arm.move_joints(TARGET, speed=10, timeout_s=0))

    def test_inner_reach_for_unequal_links(self):
        config = ZArmConfig(card_id=59, arm_lengths_mm=(250.0, 150.0))
        arm, fake, _ = ready_arm(config=config, arm_lengths_mm=(250.0, 150.0))
        self.assert_rejected_locally(arm, fake, lambda: arm.move_to_pose(CartesianPose(50, 0, 0, 0), speed=10), SoftLimitError)
        arm.move_to_pose(CartesianPose(200.0, -150.0, 0.0, 0.0), speed=50.0)
        self.assert_rejected_locally(
            arm, fake, lambda: arm.move_linear(CartesianPose(-200.0, 150.0, 0.0, 0.0), speed_mm_s=50.0), SoftLimitError
        )

    def test_speed_cap_is_configurable(self):
        arm, _, _ = ready_arm(config=ZArmConfig(card_id=59, max_joint_speed=200.0))
        arm.move_joints(TARGET, speed=150.0)


class FailSafeTests(ArmTestCase):
    def test_rejections_map_to_errors_and_stop(self):
        cases = [
            ("collision", CollisionError),
            ("joint_fault", JointFaultError),
            ("forced_4", UnreachableError),
            ("forced_6", CommandError),
        ]
        for case, error in cases:
            with self.subTest(case=case):
                arm, fake, _ = ready_arm()
                if case == "collision":
                    fake.arms[59].collision = True
                elif case == "joint_fault":
                    fake.arms[59].joint_state_overrides[2] = 7
                else:
                    fake.arms[59].forced_results["movej_angle"] = int(case.split("_")[1])
                with self.assertRaises(error):
                    arm.move_joints(TARGET, speed=10.0)
                self.assertEqual(fake.call_names()[-1], "stop_move")

    def test_busy_is_reported(self):
        arm, fake, _ = ready_arm()
        arm.move_joints(TARGET, speed=10.0, wait=False)
        with self.assertRaises(ArmBusyError) as caught:
            arm.move_joints(JointPositions(0, 0, 0, 0), speed=10.0)
        self.assertEqual(caught.exception.code, 0)
        self.assertEqual(fake.arms[59].stop_count, 1)

    def test_timeout_stops_the_arm(self):
        arm, fake, clock = ready_arm()
        fake.arms[59].stalled = True
        with self.assertLogs("zarm", level="WARNING") as logs, self.assertRaises(MotionTimeoutError) as caught:
            arm.move_joints(TARGET, speed=50.0, timeout_s=2.0)
        self.assertIn("fail-safe stop for card 59 (move_joints)", logs.output[-1])
        self.assertGreaterEqual(clock(), 2.0)
        self.assertEqual(fake.arms[59].stop_count, 1)
        self.assertIsNotNone(caught.exception.state)
        self.assertFalse(arm.is_moving())

    def test_default_timeout_scales_with_the_move(self):
        arm, fake, clock = ready_arm()
        fake.arms[59].stalled = True
        with self.assertRaises(MotionTimeoutError):
            arm.move_joints(TARGET, speed=50.0)
        # The slowest axis is J3 (50 mm at 50 mm/s = 1 s), so the default timeout is 2 * 1 s + 3 s.
        self.assertAlmostEqual(clock(), 5.0, delta=0.05)

    def _with_sleep_hook(self, arm, clock, hook):
        calls = []

        def sleep(seconds):
            calls.append(seconds)
            clock.advance(seconds)
            hook(len(calls))

        arm._sleep = sleep

    def test_collision_during_motion(self):
        arm, fake, clock = ready_arm()

        def hook(count):
            if count == 5:
                fake.arms[59].collision = True

        self._with_sleep_hook(arm, clock, hook)
        with self.assertRaises(CollisionError) as caught:
            arm.move_joints(TARGET, speed=10.0)
        self.assertIsNone(caught.exception.code)
        self.assertEqual(fake.arms[59].stop_count, 1)

    def test_lost_communication(self):
        arm, fake, clock = ready_arm()

        def hook(count):
            if count == 5:
                fake.arms[59].present = False

        self._with_sleep_hook(arm, clock, hook)
        with self.assertRaises(ConnectionLostError):
            arm.move_joints(TARGET, speed=10.0)
        self.assertEqual(fake.arms[59].stop_count, 1)

    def test_keyboard_interrupt_stops_the_arm(self):
        arm, fake, clock = ready_arm()

        def hook(count):
            if count == 3:
                raise KeyboardInterrupt

        self._with_sleep_hook(arm, clock, hook)
        with self.assertRaises(KeyboardInterrupt):
            arm.move_joints(TARGET, speed=10.0)
        self.assertEqual(fake.arms[59].stop_count, 1)

    def test_stopping_short_is_an_error(self):
        arm, fake, clock = ready_arm()

        def hook(count):
            if count == 10:
                fake.stop_move(59)

        self._with_sleep_hook(arm, clock, hook)
        with self.assertRaises(MotionIncompleteError) as caught:
            arm.move_joints(TARGET, speed=10.0)
        self.assertNotEqual(caught.exception.state.joints, TARGET)

    def test_motion_that_never_starts(self):
        arm, fake, _ = ready_arm()
        fake.arms[59].forced_results["movej_angle"] = 1  # accepted, but nothing moves
        with self.assertRaises(MotionIncompleteError):
            arm.move_joints(TARGET, speed=10.0)
        self.assertEqual(fake.arms[59].stop_count, 1)

    def test_wait_until_stopped_timeout(self):
        arm, fake, _ = ready_arm()
        fake.arms[59].stalled = True
        arm.move_joints(TARGET, speed=10.0, wait=False)
        with self.assertRaises(MotionTimeoutError):
            arm.wait_until_stopped(timeout_s=1.0)
        self.assertEqual(fake.arms[59].stop_count, 1)

    def test_backend_exception_stops_the_arm(self):
        arm, fake, _ = ready_arm()

        def broken(*args):
            raise RuntimeError("ctypes failure")

        fake.movej_angle = broken
        with self.assertRaises(RuntimeError):
            arm.move_joints(TARGET, speed=10.0)
        self.assertEqual(fake.arms[59].stop_count, 1)

    def test_stop_from_another_thread(self):
        fake = FakeBackend((59,))
        arm = ZArm(fake, ZArmConfig(card_id=59, poll_interval_s=0.005))
        arm.connect()
        arm.initialize()
        errors = []

        def run():
            try:
                arm.move_joints(JointPositions(80.0, 0.0, 0.0, 0.0), speed=40.0)
            except MotionIncompleteError as exc:
                errors.append(exc)

        worker = threading.Thread(target=run)
        worker.start()
        time.sleep(0.3)
        arm.stop()
        worker.join(timeout=5.0)
        self.assertFalse(worker.is_alive())
        self.assertEqual(len(errors), 1)
        self.assertLess(arm.get_state().joints.j1_deg, 80.0)
        arm.close()


class ReviewRegressionTests(ArmTestCase):
    """One test per finding from the design review."""

    def assert_nothing_sent(self, fake, call, error=InvalidArgumentError):
        before = len(fake.calls)
        with self.assertRaises(error):
            call()
        self.assertEqual(len(fake.calls), before, "nothing may be sent to the arm")

    def test_failure_after_an_accepted_command_still_stops(self):
        arm, fake, _ = ready_arm()

        def interrupted(*args):
            raise KeyboardInterrupt

        arm._motion_timeout = interrupted
        with self.assertRaises(KeyboardInterrupt):
            arm.move_joints(TARGET, speed=10.0)
        self.assertEqual(fake.arms[59].stop_count, 1)

    def test_values_that_ctypes_would_narrow_are_rejected(self):
        wide = JointLimits(j1_deg=(-1e300, 1e300), j2_deg=(-1e300, 1e300), j3_mm=(-1e300, 1e300), j4_deg=(-1e300, 1e300))
        arm, fake, _ = ready_arm(config=ZArmConfig(card_id=59, joint_limits=wide))
        self.assert_nothing_sent(fake, lambda: arm.move_joints(JointPositions(1e300, 0, 0, 0), speed=10))
        self.assert_nothing_sent(fake, lambda: arm.move_joints(TARGET, speed=1e-300))
        self.assert_nothing_sent(fake, lambda: arm.write_digital_output(2**32, True))
        self.assert_nothing_sent(fake, lambda: arm.read_digital_input(64))
        self.assert_nothing_sent(fake, lambda: arm.move_to_pose(CartesianPose(250, 0, 0, 0), speed=10, lr=2))
        with self.assertRaises(InvalidArgumentError):
            ZArmConfig(max_joint_speed=1e300).validate()
        with self.assertRaises(InvalidArgumentError):
            ZArmConfig(z_travel_mm=2**40).validate()
        with self.assertRaises(InvalidArgumentError):
            ZArmConfig(arm_lengths_mm=(1e-300, 200.0)).validate()  # positive, but 0.0 as a C float

    def test_cartesian_targets_respect_joint_limits(self):
        limits = JointLimits(j1_deg=(-10.0, 10.0))
        arm, fake, _ = ready_arm(config=ZArmConfig(card_id=59, joint_limits=limits))
        # (250, 100) needs J1 of about -26 degrees with the current hand.
        self.assert_nothing_sent(fake, lambda: arm.move_to_pose(CartesianPose(250, 100, 0, 0), speed=10), SoftLimitError)
        self.assert_nothing_sent(
            fake, lambda: arm.move_to_pose(CartesianPose(250, 100, 0, 0), speed=10, lr=1), SoftLimitError
        )

    def test_change_hand_checks_the_other_hand(self):
        arm, fake, _ = ready_arm(config=ZArmConfig(card_id=59, joint_limits=JointLimits(j1_deg=(-50.0, 50.0))))
        arm.move_joints(JointPositions(30.0, 45.0, 0.0, 0.0), speed=50.0)
        self.assert_nothing_sent(fake, lambda: arm.change_hand(speed=10.0), SoftLimitError)

    def test_linear_path_respects_joint_limits(self):
        arm, fake, _ = ready_arm(config=ZArmConfig(card_id=59, joint_limits=JointLimits(j1_deg=(-60.0, 60.0))))
        arm.move_joints(JointPositions(-20.0, 90.0, 0.0, 0.0), speed=50.0)
        start = arm.get_state().pose
        target = CartesianPose(start.x_mm, -start.y_mm, 0.0, 0.0)  # needs J1 of about -70 degrees
        self.assert_nothing_sent(fake, lambda: arm.move_linear(target, speed_mm_s=50.0), SoftLimitError)

    def test_leaving_the_envelope_during_motion_stops_the_arm(self):
        arm, fake, _ = ready_arm(config=ZArmConfig(card_id=59, joint_limits=JointLimits(j1_deg=(-20.0, 20.0))))
        fake.movej_angle(59, 60.0, 0.0, 0.0, 0.0, 30.0, 0.0)  # a motion the SDK did not plan
        with self.assertRaises(LimitExceededError) as caught:
            arm.wait_until_stopped(timeout_s=10.0)
        self.assertLess(caught.exception.state.joints.j1_deg, 22.0)
        self.assertEqual(fake.arms[59].stop_count, 1)

    def test_moving_back_into_the_envelope_is_allowed(self):
        arm, fake, _ = ready_arm()
        fake.arms[59].joints = (95.0, 0.0, 0.0, 0.0)  # starts outside the default J1 limit of 90
        state = arm.move_joints(JointPositions(80.0, 0.0, 0.0, 0.0), speed=10.0)
        self.assertAlmostEqual(state.joints.j1_deg, 80.0)

    def test_non_finite_telemetry(self):
        arm, fake, clock = ready_arm()
        fake.arms[59].joints = (float("nan"), 0.0, 0.0, 0.0)
        with self.assertRaises(InvalidTelemetryError):
            arm.get_state()
        fake.arms[59].joints = (0.0, 0.0, 0.0, 0.0)
        original = fake.get_scara_param
        calls = []

        def corrupt_after_a_while(card):
            calls.append(card)
            param = original(card)
            return dataclasses.replace(param, x_mm=float("nan")) if len(calls) > 5 else param

        fake.get_scara_param = corrupt_after_a_while
        with self.assertRaises(InvalidTelemetryError):
            arm.move_joints(TARGET, speed=10.0)
        self.assertEqual(fake.arms[59].stop_count, 1)

    def test_default_timeout_is_capped(self):
        arm, fake, clock = ready_arm()
        fake.arms[59].stalled = True
        with self.assertRaises(MotionTimeoutError):
            arm.move_joints(TARGET, speed=0.001)
        self.assertAlmostEqual(clock(), 300.0, delta=0.05)

    def test_negative_retry_delay(self):
        arm, _, _ = make_arm()
        arm.connect()
        with self.assertRaises(InvalidArgumentError):
            arm.initialize(retry_delay_s=-1.0)

class LifecycleTests(ArmTestCase):
    def test_close_stops_a_moving_arm_and_releases_it(self):
        arm, fake, clock = ready_arm()
        arm.move_joints(TARGET, speed=10.0, wait=False)
        arm.close()
        self.assertIs(arm.phase, ArmPhase.CLOSED)
        self.assertTrue(fake.is_stop(59))
        self.assertEqual(fake.arms[59].stop_count, 2)  # once at once, once after commands in progress
        self.assertIsNone(fake.owner_of(59))
        self.assertFalse(fake.has_clients())
        arm.close()
        self.assertNotIn("close_server", fake.call_names())

    def test_close_always_sends_a_stop(self):
        arm, fake, _ = ready_arm()
        arm.close(stop_server=True)
        self.assertEqual(fake.arms[59].stop_count, 2)
        self.assertEqual(fake.call_names()[-1], "close_server")

    def test_a_command_racing_close_is_stopped(self):
        arm, fake, _ = ready_arm()
        original = fake.movej_angle
        closers = []

        def racing_movej_angle(*args):
            closer = threading.Thread(target=arm.close)
            closers.append(closer)
            closer.start()
            for _ in range(500):  # let close() send its first stop before the motion starts
                if fake.arms[59].stop_count:
                    break
                time.sleep(0.01)
            return original(*args)

        fake.movej_angle = racing_movej_angle
        arm.move_joints(TARGET, speed=10.0, wait=False)
        closers[0].join(5.0)
        self.assertIs(arm.phase, ArmPhase.CLOSED)
        self.assertTrue(fake.is_stop(59), "the motion that started after the first stop must be stopped too")
        self.assertEqual(fake.arms[59].stop_count, 2)

    def test_owned_backend_outlives_its_owner_while_shared(self):
        owner = ZArm.simulated(ZArmConfig(card_id=1), clock=ManualClock())
        fake = owner.backend
        fake.arms[2] = type(fake.arms[1])(2)
        closed = []
        fake.close = lambda: closed.append(True)
        other = ZArm(fake, ZArmConfig(card_id=2))
        owner.connect()
        other.connect()
        owner.close()
        self.assertEqual(closed, [], "the other arm still uses the backend")
        other.close()
        self.assertEqual(closed, [True])

    def test_close_stops_before_waiting_for_a_blocked_call(self):
        arm, fake, _ = ready_arm()
        entered, release = threading.Event(), threading.Event()
        original_initial = fake.initial

        def blocking_initial(*args):
            entered.set()
            release.wait(5.0)  # stands in for a vendor message box
            return original_initial(*args)

        fake.initial = blocking_initial
        outcome = []

        def run_initialize():
            try:
                arm.initialize()
            except ZArmError as exc:
                outcome.append(exc)

        worker = threading.Thread(target=run_initialize)
        worker.start()
        self.assertTrue(entered.wait(5.0))
        closer = threading.Thread(target=arm.close)
        closer.start()
        for _ in range(500):
            if fake.arms[59].stop_count:
                break
            time.sleep(0.01)
        self.assertEqual(fake.arms[59].stop_count, 1, "the stop must not wait behind the blocked call")
        self.assertTrue(closer.is_alive(), "close() waits for the blocked call before releasing the arm")
        release.set()
        worker.join(5.0)
        closer.join(5.0)
        self.assertIs(arm.phase, ArmPhase.CLOSED)
        self.assertEqual(len(outcome), 1, "initialize() must fail once the arm is closing")

    def test_stop_server_is_skipped_while_another_arm_is_owned(self):
        clock = ManualClock()
        fake = FakeBackend((1, 2), clock=clock)
        first = ZArm(fake, ZArmConfig(card_id=1), clock=clock, sleep=clock.sleep)
        second = ZArm(fake, ZArmConfig(card_id=2), clock=clock, sleep=clock.sleep)
        first.connect()
        second.connect()
        with self.assertLogs("zarm", level="WARNING") as logs:
            first.close(stop_server=True)
        self.assertNotIn("close_server", fake.call_names())
        self.assertTrue(any("still in use" in line for line in logs.output))
        second.close(stop_server=True)
        self.assertEqual(fake.call_names()[-1], "close_server")

    def test_context_manager(self):
        arm, fake, _ = make_arm()
        with arm:
            arm.connect()
        self.assertIs(arm.phase, ArmPhase.CLOSED)

    def test_exit_hook_stops_moving_arms(self):
        arm, fake, _ = ready_arm()
        arm.move_joints(TARGET, speed=10.0, wait=False)
        _stop_open_arms_at_exit()
        self.assertEqual(fake.arms[59].stop_count, 1)
        arm.close()

    def test_homing_requires_reinitialization(self):
        arm, fake, _ = ready_arm()
        arm.move_joints(TARGET, speed=50.0)
        arm.start_homing(Joint.J1_SHOULDER)
        self.assertIs(arm.phase, ArmPhase.CONNECTED)
        with self.assertRaises(NotInitializedError):
            arm.move_joints(TARGET, speed=10.0)
        arm.wait_until_stopped(timeout_s=10.0)
        arm.initialize()
        self.assertAlmostEqual(arm.get_state().joints.j1_deg, 0.0)
        with self.assertRaises(InvalidArgumentError):
            arm.start_homing(7)

    def test_refused_homing_still_needs_reinitialization(self):
        arm, fake, _ = ready_arm()
        fake.arms[59].forced_results["joint_home"] = 3
        with self.assertRaises(CommandError):
            arm.start_homing(Joint.J2_ELBOW)
        self.assertIs(arm.phase, ArmPhase.CONNECTED)
        self.assertFalse(arm.get_state().initialized)
        self.assertEqual(fake.arms[59].stop_count, 1)

    def test_homing_is_refused_while_moving(self):
        arm, fake, _ = ready_arm()
        arm.move_joints(TARGET, speed=10.0, wait=False)
        with self.assertRaises(ArmBusyError):
            arm.start_homing(Joint.J1_SHOULDER)
        self.assertNotIn("joint_home", fake.call_names())
        self.assertIs(arm.phase, ArmPhase.INITIALIZED)

    def test_simulated_factory(self):
        with ZArm.simulated(ZArmConfig(card_id=5)) as arm:
            self.assertEqual(arm.connect(), 5)
            arm.initialize()
            arm.move_joints(JointPositions(1.0, 0.0, 0.0, 0.0), speed=100.0)


class StateAndIoTests(ArmTestCase):
    def test_state_fields(self):
        arm, _, clock = ready_arm()
        state = arm.get_state()
        self.assertTrue(state.communicating)
        self.assertTrue(state.initialized)
        self.assertFalse(state.moving)
        self.assertEqual(state.pose.x_mm, 400.0)
        self.assertEqual(state.timestamp_s, clock())
        self.assertEqual(arm.get_encoder_reading().joints, state.joints)

    def test_joint_states(self):
        arm, fake, _ = ready_arm()
        fake.arms[59].joint_state_overrides[3] = 42
        states = arm.get_joint_states()
        self.assertIs(states[Joint.J1_SHOULDER], JointState.NORMAL)
        self.assertEqual(states[Joint.J3_VERTICAL], 42)
        self.assertFalse(arm.is_collision())

    def test_io(self):
        arm, fake, _ = ready_arm(digital_io_count=4)
        fake.arms[59].digital_in[1] = 1
        self.assertEqual(arm.read_digital_input(1), 1)
        arm.write_digital_output(2, True)
        self.assertEqual(arm.read_digital_output(2), 1)
        with self.assertRaises(InvalidArgumentError):
            arm.read_digital_input(9)
        with self.assertRaises(CommandError):
            arm.write_digital_output(9, True)
        with self.assertRaises(InvalidArgumentError):
            arm.write_digital_output(1, 1)

    def test_modes(self):
        arm, _, _ = ready_arm()
        arm.set_drag_teach(True)
        self.assertTrue(arm.get_drag_teach())
        arm.set_cooperation_mode(True)
        self.assertTrue(arm.get_cooperation_mode())
        with self.assertRaises(InvalidArgumentError):
            arm.set_drag_teach("yes")

    def test_pause_resume(self):
        arm, _, _ = ready_arm()
        arm.move_joints(TARGET, speed=10.0, wait=False)
        arm.pause()
        arm.resume()
        self.assertEqual(arm.wait_until_stopped(timeout_s=30.0).joints, TARGET)


if __name__ == "__main__":
    unittest.main()

"""The simulator must behave like the DLL wherever the SDK depends on it."""

import math
import unittest

from zarm import FakeBackend, FakeBackendMisuseError, JointLimits, ManualClock


def started(cards=(59,), **options):
    clock = ManualClock()
    fake = FakeBackend(cards, clock=clock, **options)
    fake.start_server()
    return fake, clock


def initialized(cards=(59,), **options):
    fake, clock = started(cards, **options)
    for card in cards:
        assert fake.initial(card, 1, 310) == 1
    return fake, clock


class PreconditionTests(unittest.TestCase):
    def test_robot_calls_need_the_server(self):
        fake = FakeBackend((59,), clock=ManualClock())
        with self.assertRaises(FakeBackendMisuseError):
            fake.get_scara_param(59)
        with self.assertRaises(FakeBackendMisuseError):
            fake.card_connect_status(59)

    def test_card_bounds(self):
        fake, _ = started()
        self.assertEqual(fake.card_connect_status(256), 2)
        self.assertEqual(fake.card_connect_status(-1), 2)
        for bad in (256, -1, True):
            with self.subTest(card=bad), self.assertRaises(FakeBackendMisuseError):
                fake.movej_angle(bad, 0, 0, 0, 0, 10, 0)

    def test_close_server_ends_robot_calls(self):
        fake, _ = started()
        fake.close_server()
        with self.assertRaises(FakeBackendMisuseError):
            fake.is_stop(59)

    def test_server_start_failure(self):
        fake = FakeBackend((59,), clock=ManualClock(), server_start_result=-1)
        self.assertEqual(fake.start_server(), -1)
        self.assertFalse(fake.server_running)

    def test_manual_clock_rejects_negative_sleep(self):
        with self.assertRaises(ValueError):
            ManualClock().sleep(-0.1)

    def test_connect_delay(self):
        fake, clock = started(connect_delay_s=1.2)
        self.assertEqual(fake.card_connect_status(59), 0)
        clock.advance(1.2)
        self.assertEqual(fake.card_connect_status(59), 1)
        self.assertEqual(fake.card_connect_status(60), 0)


class InitialTests(unittest.TestCase):
    def test_codes(self):
        fake, _ = started()
        self.assertEqual(fake.initial(7, 1, 310), 0)  # no arm with this id
        self.assertEqual(fake.initial(59, 9, 310), 2)
        self.assertEqual(fake.initial(59, 1, 0), 12)
        fake.arms[59].mobile_terminal = True
        self.assertEqual(fake.initial(59, 1, 310), 11)
        fake.arms[59].mobile_terminal = False
        fake.arms[59].joint_state_overrides[2] = 7
        self.assertEqual(fake.initial(59, 1, 310), 105)
        del fake.arms[59].joint_state_overrides[2]
        self.assertEqual(fake.initial(59, 1, 310), 1)
        self.assertTrue(fake.get_scara_param(59).initial_finish)

    def test_queued_results_and_int_arguments(self):
        fake, _ = started()
        fake.arms[59].init_results.extend([0, 3])
        self.assertEqual([fake.initial(59, 1, 310) for _ in range(3)], [0, 3, 1])
        with self.assertRaises(FakeBackendMisuseError):
            fake.initial(59, 1, 310.0)


class MoveCheckOrderTests(unittest.TestCase):
    def move(self, fake, **overrides):
        args = dict(angle1_deg=10.0, angle2_deg=10.0, z_mm=-10.0, r_deg=0.0, speed=10.0, roughly=0.0)
        args.update(overrides)
        return fake.movej_angle(59, **args)

    def test_codes_in_vendor_order(self):
        fake, _ = initialized()
        arm = fake.arms[59]
        arm.collision = True
        self.assertEqual(self.move(fake, angle1_deg=math.nan), 102)  # wrapper checks come first
        arm.collision = False
        arm.joint_state_overrides[3] = 0
        self.assertEqual(self.move(fake), 103)
        arm.joint_state_overrides[3] = 8
        self.assertEqual(self.move(fake), 105)
        arm.joint_state_overrides.clear()
        self.assertEqual(self.move(fake, angle1_deg=math.nan), 101)
        self.assertEqual(self.move(fake, roughly=1.5), 59)  # the DLL returns the card id here
        self.assertEqual(self.move(fake, speed=0.0), 2)
        arm.servo_off = True
        self.assertEqual(self.move(fake), 6)
        arm.servo_off = False
        arm.mobile_terminal = True
        self.assertEqual(self.move(fake), 11)
        arm.mobile_terminal = False
        self.assertEqual(self.move(fake, angle1_deg=120.0), 4)
        self.assertEqual(self.move(fake), 1)
        self.assertEqual(self.move(fake), 0)  # busy while moving

    def test_not_initialized(self):
        fake, _ = started()
        self.assertEqual(self.move(fake), 3)

    def test_forced_result(self):
        fake, _ = initialized()
        fake.arms[59].forced_results["movej_angle"] = 7
        self.assertEqual(self.move(fake), 7)
        self.assertEqual(self.move(fake), 1)


class MotionTests(unittest.TestCase):
    def test_slowest_joint_sets_duration(self):
        fake, clock = initialized()
        self.assertEqual(fake.movej_angle(59, 90.0, 30.0, -10.0, 0.0, 45.0, 0.0), 1)
        clock.advance(1.0)
        halfway = fake.get_scara_param(59)
        self.assertTrue(halfway.move_flag)
        self.assertAlmostEqual(halfway.angle1_deg, 45.0)
        self.assertAlmostEqual(halfway.angle2_deg, 15.0)
        clock.advance(1.0)
        done = fake.get_scara_param(59)
        self.assertFalse(done.move_flag)
        self.assertTrue(fake.is_stop(59))
        self.assertEqual((done.angle1_deg, done.angle2_deg, done.z_mm), (90.0, 30.0, -10.0))
        self.assertAlmostEqual(done.x_mm, 200 * math.cos(math.radians(90)) + 200 * math.cos(math.radians(120)))

    def test_stop_freezes_motion(self):
        fake, clock = initialized()
        fake.movej_angle(59, 40.0, 0.0, 0.0, 0.0, 10.0, 0.0)
        clock.advance(1.0)
        fake.stop_move(59)
        clock.advance(10.0)
        param = fake.get_scara_param(59)
        self.assertFalse(param.move_flag)
        self.assertAlmostEqual(param.angle1_deg, 10.0)
        self.assertEqual(fake.arms[59].stop_count, 1)

    def test_pause_and_resume(self):
        fake, clock = initialized()
        fake.movej_angle(59, 20.0, 0.0, 0.0, 0.0, 10.0, 0.0)
        clock.advance(1.0)
        self.assertEqual(fake.pause_move(59), 1)
        clock.advance(5.0)
        self.assertAlmostEqual(fake.get_scara_param(59).angle1_deg, 10.0)
        self.assertEqual(fake.resume_move(59), 1)
        clock.advance(1.0)
        self.assertTrue(fake.is_stop(59))
        self.assertAlmostEqual(fake.get_scara_param(59).angle1_deg, 20.0)

    def test_movej_xyz_and_lr(self):
        fake, clock = initialized()
        self.assertEqual(fake.movej_xyz(59, 250.0, -100.0, -20.0, 5.0, 50.0, 0.0), 1)
        clock.advance(10.0)
        param = fake.get_scara_param(59)
        self.assertAlmostEqual(param.x_mm, 250.0)
        self.assertAlmostEqual(param.y_mm, -100.0)
        self.assertEqual(fake.movej_xyz(59, 500.0, 0.0, 0.0, 0.0, 50.0, 0.0), 4)
        self.assertEqual(fake.movej_xyz_lr(59, 250.0, -100.0, -20.0, 5.0, 50.0, 0.0, -1), 1)
        clock.advance(10.0)
        self.assertLessEqual(fake.get_scara_param(59).angle2_deg, 0.0)
        self.assertEqual(fake.movej_xyz_lr(59, 500.0, 0.0, 0.0, 0.0, 50.0, 0.0, 1), 7)
        with self.assertRaises(FakeBackendMisuseError):
            fake.movej_xyz_lr(59, 250.0, 0.0, 0.0, 0.0, 50.0, 0.0, 1.0)

    def test_movel_follows_a_straight_line(self):
        fake, clock = initialized()
        fake.movej_xyz(59, 250.0, -100.0, -20.0, 0.0, 100.0, 0.0)
        clock.advance(10.0)
        self.assertEqual(fake.movel_xyz(59, 250.0, 100.0, -20.0, 0.0, 50.0), 1)
        for _ in range(3):
            clock.advance(1.0)
            self.assertAlmostEqual(fake.get_scara_param(59).x_mm, 250.0, places=6)
        clock.advance(10.0)
        self.assertAlmostEqual(fake.get_scara_param(59).y_mm, 100.0, places=6)

    def test_movel_rejects_a_path_through_the_base(self):
        fake, clock = initialized()
        fake.movej_xyz(59, 150.0, 0.0, 0.0, 0.0, 100.0, 0.0)
        clock.advance(10.0)
        self.assertEqual(fake.movel_xyz(59, -150.0, 0.0, 0.0, 0.0, 50.0), 4)

    def test_change_attitude(self):
        fake, clock = initialized()
        fake.movej_angle(59, 30.0, 45.0, 0.0, 0.0, 100.0, 0.0)
        clock.advance(5.0)
        before = fake.get_scara_param(59)
        self.assertEqual(fake.change_attitude(59, 50.0), 1)
        clock.advance(10.0)
        after = fake.get_scara_param(59)
        self.assertLess(after.angle2_deg, 0.0)
        self.assertAlmostEqual(after.x_mm, before.x_mm)
        self.assertAlmostEqual(after.y_mm, before.y_mm)

    def test_joint_home_clears_initialization(self):
        fake, clock = initialized()
        fake.movej_angle(59, 30.0, 0.0, 0.0, 0.0, 100.0, 0.0)
        clock.advance(5.0)
        self.assertEqual(fake.joint_home(59, 5), 2)
        self.assertFalse(fake.get_scara_param(59).initial_finish, "cleared even when refused")
        fake.initial(59, 1, 310)
        self.assertEqual(fake.joint_home(59, 1), 1)
        self.assertFalse(fake.get_scara_param(59).initial_finish)
        clock.advance(10.0)
        self.assertAlmostEqual(fake.get_scara_param(59).angle1_deg, 0.0)
        self.assertEqual(fake.movej_angle(59, 1.0, 0.0, 0.0, 0.0, 10.0, 0.0), 3)

    def test_hard_limits_are_configurable(self):
        fake, _ = initialized(hard_limits=JointLimits(j1_deg=(-10.0, 10.0)))
        self.assertEqual(fake.movej_angle(59, 20.0, 0.0, 0.0, 0.0, 10.0, 0.0), 4)


class StateAndIoTests(unittest.TestCase):
    def test_joint_states(self):
        fake, _ = started()
        self.assertEqual(fake.get_joint_state(59, 1), 3)
        self.assertEqual(fake.get_joint_state(59, 4), 1)
        self.assertEqual(fake.get_joint_state(59, 9), 2)
        fake.initial(59, 1, 310)
        self.assertEqual(fake.get_joint_state(59, 1), 1)
        fake.arms[59].collision = True
        self.assertEqual(fake.get_joint_state(59, 2), 5)
        self.assertTrue(fake.is_collision(59))

    def test_absent_arm_reports_no_communication(self):
        fake, _ = started()
        self.assertFalse(fake.get_scara_param(12).communicate_success)

    def test_io(self):
        fake, _ = started(digital_io_count=4)
        fake.arms[59].digital_in[2] = 1
        self.assertEqual(fake.get_digital_io_in(59, 2), 1)
        self.assertEqual(fake.get_digital_io_in(59, 4), -1)
        self.assertTrue(fake.set_digital_io_out(59, 1, True))
        self.assertEqual(fake.get_digital_io_out(59, 1), 1)
        self.assertFalse(fake.set_digital_io_out(59, 9, True))
        self.assertEqual(fake.get_digital_io_in(12, 0), 3)

    def test_modes(self):
        fake, _ = started()
        self.assertTrue(fake.set_drag_teach(59, True))
        self.assertTrue(fake.get_drag_teach(59))
        self.assertTrue(fake.set_cooperation_fun_state(59, True))
        self.assertTrue(fake.get_cooperation_fun_state(59))
        self.assertFalse(fake.set_drag_teach(12, True))


if __name__ == "__main__":
    unittest.main()

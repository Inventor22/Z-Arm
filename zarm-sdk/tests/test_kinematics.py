import itertools
import math
import unittest

from zarm import kinematics


class KinematicsTests(unittest.TestCase):
    L1 = L2 = 200.0

    def test_zero_angles_point_along_x(self):
        x, y = kinematics.forward_xy(0.0, 0.0, self.L1, self.L2)
        self.assertAlmostEqual(x, 400.0)
        self.assertAlmostEqual(y, 0.0)

    def test_assumed_z_arm_2442_links(self):
        self.assertEqual(kinematics.forward_xy(0.0, 0.0, 220.0, 200.0), (420.0, 0.0))
        x, y = kinematics.forward_xy(90.0, 0.0, 220.0, 200.0)
        self.assertAlmostEqual(x, 0.0)
        self.assertAlmostEqual(y, 420.0, msg="counter-clockwise is positive")
        self.assertEqual(kinematics.reach_mm(220.0, 200.0), (20.0, 420.0))
        for elbow in (1, -1):
            j1, j2 = kinematics.inverse_xy(250.0, -100.0, 220.0, 200.0, elbow)
            x, y = kinematics.forward_xy(j1, j2, 220.0, 200.0)
            self.assertAlmostEqual(x, 250.0)
            self.assertAlmostEqual(y, -100.0)

    def test_tool_yaw_adds_the_joint_angles(self):
        self.assertEqual(kinematics.tool_yaw_deg(30.0, 45.0, -10.0), 65.0)

    def test_round_trip_both_elbows(self):
        for j1, j2 in itertools.product((-80.0, -30.0, 0.0, 45.0, 89.0), (-150.0, -60.0, 20.0, 120.0)):
            x, y = kinematics.forward_xy(j1, j2, self.L1, self.L2)
            solution = kinematics.inverse_xy(x, y, self.L1, self.L2, kinematics.elbow_of(j2))
            self.assertIsNotNone(solution)
            self.assertAlmostEqual(solution[0], j1, places=6)
            self.assertAlmostEqual(solution[1], j2, places=6)

    def test_unreachable(self):
        self.assertIsNone(kinematics.inverse_xy(401.0, 0.0, self.L1, self.L2, 1))
        self.assertIsNone(kinematics.inverse_xy(10.0, 0.0, 250.0, 150.0, 1))
        self.assertEqual(kinematics.reach_mm(250.0, 150.0), (100.0, 400.0))

    def test_alternate_hand_keeps_position_and_flips_elbow(self):
        j1, j2 = kinematics.alternate_hand(30.0, 45.0, self.L1, self.L2)
        self.assertLess(j2, 0.0)
        before = kinematics.forward_xy(30.0, 45.0, self.L1, self.L2)
        after = kinematics.forward_xy(j1, j2, self.L1, self.L2)
        self.assertAlmostEqual(before[0], after[0])
        self.assertAlmostEqual(before[1], after[1])

    def test_solutions_and_segment_distance(self):
        self.assertEqual(len(kinematics.inverse_xy_solutions(300.0, 0.0, self.L1, self.L2)), 2)
        self.assertAlmostEqual(kinematics.segment_distance_to_origin(-100.0, 50.0, 100.0, 50.0), 50.0)
        self.assertAlmostEqual(kinematics.segment_distance_to_origin(30.0, 40.0, 30.0, 40.0), 50.0)
        self.assertAlmostEqual(kinematics.segment_distance_to_origin(100.0, 0.0, 200.0, 0.0), 100.0)

    def test_angles_are_wrapped(self):
        j1, _ = kinematics.inverse_xy(-300.0, -1e-9, self.L1, self.L2, 1)
        self.assertTrue(-180.0 < j1 <= 180.0)
        self.assertFalse(math.isnan(j1))


if __name__ == "__main__":
    unittest.main()

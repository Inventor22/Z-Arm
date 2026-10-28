import math
import unittest

from zarm import JOINT_SPECS, CartesianPose, InvalidArgumentError, Joint, JointKind, JointLimits, JointPositions, ZArmConfig
from zarm.types import joint_spec, require_c_float, require_c_int, require_card_id, require_number, to_float32


class JointModelTests(unittest.TestCase):
    def test_joint_specs_describe_the_arm(self):
        kinds = [(spec.joint, spec.kind, spec.unit) for spec in JOINT_SPECS]
        self.assertEqual(
            kinds,
            [
                (Joint.J1_SHOULDER, JointKind.REVOLUTE, "deg"),
                (Joint.J2_ELBOW, JointKind.REVOLUTE, "deg"),
                (Joint.J3_VERTICAL, JointKind.PRISMATIC, "mm"),
                (Joint.J4_WRIST, JointKind.REVOLUTE, "deg"),
            ],
        )
        self.assertEqual(joint_spec(Joint.J3_VERTICAL).field, "j3_mm")

    def test_joint_positions_accessors_and_si(self):
        joints = JointPositions(90.0, -45.0, -120.0, 180.0)
        self.assertEqual(joints.as_tuple(), (90.0, -45.0, -120.0, 180.0))
        self.assertEqual(joints.value(Joint.J3_VERTICAL), -120.0)
        j1, j2, j3, j4 = joints.to_si()
        self.assertAlmostEqual(j1, math.pi / 2)
        self.assertAlmostEqual(j2, -math.pi / 4)
        self.assertAlmostEqual(j3, -0.12)
        self.assertAlmostEqual(j4, math.pi)

    def test_pose_si(self):
        self.assertEqual(CartesianPose(100.0, -50.0, -20.0, 0.0).to_si(), (0.1, -0.05, -0.02, 0.0))


class LimitTests(unittest.TestCase):
    def test_defaults_match_legacy_slider_ranges(self):
        limits = JointLimits()
        self.assertEqual(limits.j1_deg, (-90.0, 90.0))
        self.assertEqual(limits.j2_deg, (-164.0, 164.0))
        self.assertEqual(limits.j3_mm, (-310.0, 0.0))
        self.assertEqual(limits.j4_deg, (-180.0, 180.0))

    def test_violations_name_the_joint_and_unit(self):
        problems = JointLimits().violations(JointPositions(95.0, 0.0, 5.0, 0.0))
        self.assertEqual(len(problems), 2)
        self.assertIn("j1_deg=95", problems[0])
        self.assertIn("mm", problems[1])

    def test_z_limits_follow_z_travel(self):
        self.assertEqual(ZArmConfig(z_travel_mm=210).resolved_joint_limits().j3_mm, (-210.0, 0.0))

    def test_rated_ranges_of_the_assumed_model(self):
        rated = JointLimits.rated(310)
        self.assertEqual((rated.j1_deg, rated.j2_deg, rated.j3_mm), ((-90.0, 90.0), (-164.0, 164.0), (-310.0, 0.0)))
        self.assertEqual(rated.j4_deg, (-1080.0, 1080.0))
        soft = JointLimits()
        self.assertEqual((soft.j1_deg, soft.j2_deg), (rated.j1_deg, rated.j2_deg))


class ValidationTests(unittest.TestCase):
    def test_require_number(self):
        self.assertEqual(require_number("x", 3), 3.0)
        for bad in (float("nan"), float("inf"), True, "1", None):
            with self.assertRaises(InvalidArgumentError):
                require_number("x", bad)

    def test_require_card_id(self):
        self.assertEqual(require_card_id(59), 59)
        for bad in (-1, 256, True, 1.0, None):
            with self.assertRaises(InvalidArgumentError):
                require_card_id(bad)

    def test_c_ranges(self):
        self.assertEqual(require_c_int("x", 2**31 - 1), 2**31 - 1)
        for bad in (2**31, -(2**31) - 1, 2**32, True, 1.0):
            with self.subTest(value=bad), self.assertRaises(InvalidArgumentError):
                require_c_int("x", bad)
        with self.assertRaises(InvalidArgumentError):
            require_c_int("io", 64, 0, 63)
        self.assertEqual(require_c_float("x", 3.0e38), 3.0e38)
        with self.assertRaises(InvalidArgumentError):
            require_c_float("x", 1e39)
        self.assertEqual(to_float32(1e-300), 0.0)
        self.assertEqual(to_float32(0.5), 0.5)

    def test_default_config_is_valid(self):
        ZArmConfig().validate()
        ZArmConfig(card_id=59).validate()

    def test_invalid_configs(self):
        bad_configs = [
            ZArmConfig(card_id=300),
            ZArmConfig(generation=0),
            ZArmConfig(generation=9),
            ZArmConfig(z_travel_mm=310.0),  # the C API takes an int
            ZArmConfig(z_travel_mm=0),
            ZArmConfig(arm_lengths_mm=(200.0, -1.0)),
            ZArmConfig(vendor_arm_lengths_mm=(200.0,)),
            ZArmConfig(vendor_arm_lengths_mm=(200.0, 0.0)),
            ZArmConfig(max_joint_speed=0.0),
            ZArmConfig(poll_interval_s=float("nan")),
            ZArmConfig(motion_start_grace_s=-1.0),
            ZArmConfig(joint_limits=JointLimits(j1_deg=(10.0, -10.0))),
        ]
        for config in bad_configs:
            with self.subTest(config=config), self.assertRaises(InvalidArgumentError):
                config.validate()

    def test_invalid_argument_error_is_a_value_error(self):
        self.assertTrue(issubclass(InvalidArgumentError, ValueError))


if __name__ == "__main__":
    unittest.main()

"""The URDF's kinematics against zarm-sdk's, and the SDK-to-model joint mapping."""

from __future__ import annotations

import math
import random
import unittest

from tests.support import zarm
from zarm_sim.generate import MODELS_DIR, URDF_NAME
from zarm_sim.kinematics import UrdfRobot, xyz_yaw
from zarm_sim.model import SDK_FIELDS, build_model, model_to_sdk, sdk_to_model

L1_MM, L2_MM = 220.0, 200.0


def wrap_deg(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


class UrdfKinematicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.urdf = UrdfRobot(MODELS_DIR / URDF_NAME)
        cls.model = build_model()

    def tool(self, j1, j2, j3, j4, base="sdk_origin"):
        values = sdk_to_model(self.model, {"j1_deg": j1, "j2_deg": j2, "j3_mm": j3, "j4_deg": j4})
        x, y, z, yaw = xyz_yaw(self.urdf.relative(values, "tool0", base))
        return x * 1000.0, y * 1000.0, z * 1000.0, math.degrees(yaw)

    def test_tool_matches_the_sdks_forward_kinematics_for_random_joints(self):
        rng = random.Random(2442)
        for _ in range(500):
            j1, j2 = rng.uniform(-90, 90), rng.uniform(-164, 164)
            j3, j4 = rng.uniform(-310, 0), rng.uniform(-1080, 1080)
            x, y, z, yaw = self.tool(j1, j2, j3, j4)
            ex, ey = zarm.kinematics.forward_xy(j1, j2, L1_MM, L2_MM)
            with self.subTest(j1=j1, j2=j2, j3=j3, j4=j4):
                self.assertAlmostEqual(x, ex, delta=1e-6)
                self.assertAlmostEqual(y, ey, delta=1e-6)
                self.assertAlmostEqual(z, j3, delta=1e-6)
                expected_yaw = zarm.kinematics.tool_yaw_deg(j1, j2, j4)
                self.assertAlmostEqual(wrap_deg(yaw - expected_yaw), 0.0, delta=1e-6)

    def test_known_poses(self):
        self.assertEqual(tuple(round(v, 9) for v in self.tool(0, 0, 0, 0)), (420.0, 0.0, 0.0, 0.0))
        x, y, _, _ = self.tool(90, 0, 0, 0)
        self.assertAlmostEqual(x, 0.0, delta=1e-9)
        self.assertAlmostEqual(y, 420.0, delta=1e-9)  # counter-clockwise seen from above
        x, y, _, yaw = self.tool(0, 90, 0, 0)
        self.assertAlmostEqual(x, 220.0, delta=1e-9)
        self.assertAlmostEqual(y, 200.0, delta=1e-9)
        self.assertAlmostEqual(yaw, 90.0, delta=1e-9)

    def test_heights_above_the_mounting_surface(self):
        _, _, top, _ = self.tool(0, 0, 0, 0, base="base_link")
        _, _, bottom, _ = self.tool(0, 0, -310, 0, base="base_link")
        self.assertAlmostEqual(top, 374.0, delta=1e-9)
        self.assertAlmostEqual(bottom, 64.0, delta=1e-9)  # the drawing's 64 +/- 1 mm

    def test_sdk_origin_is_on_the_j1_axis_at_the_top_of_travel(self):
        poses = self.urdf.link_poses({})
        origin, shoulder = poses["sdk_origin"], poses["j1_arm"]
        for row in range(3):
            self.assertAlmostEqual(origin[row][3], shoulder[row][3], delta=1e-12)
        self.assertAlmostEqual(origin[0][3], 0.133, delta=1e-12)
        self.assertAlmostEqual(origin[2][3], 0.374, delta=1e-12)

    def test_joint_ranges_are_the_rated_ranges_and_contain_the_sdk_soft_limits(self):
        joints = {joint.name: joint for joint in self.urdf.movable()}
        self.assertEqual(set(joints), {"j1_shoulder", "j2_elbow", "j3_vertical", "j4_wrist"})
        expected = {
            "j1_shoulder": (-math.radians(90), math.radians(90)),
            "j2_elbow": (-math.radians(164), math.radians(164)),
            "j3_vertical": (-0.31, 0.0),
            "j4_wrist": (-math.radians(1080), math.radians(1080)),
        }
        for name, (low, high) in expected.items():
            with self.subTest(joint=name):
                self.assertAlmostEqual(joints[name].lower, low, delta=1e-9)
                self.assertAlmostEqual(joints[name].upper, high, delta=1e-9)
        soft = zarm.JointLimits()
        for joint in self.model.sdk_joints():
            low, high = getattr(soft, joint.sdk_field)
            urdf_joint = joints[joint.name]
            with self.subTest(joint=joint.name):
                self.assertLessEqual(urdf_joint.lower, low * joint.sdk_scale + 1e-9)
                self.assertGreaterEqual(urdf_joint.upper, high * joint.sdk_scale - 1e-9)

    def test_tree(self):
        self.assertEqual(self.urdf.root, "base_link")
        chain = ["base_link", "carriage", "j1_arm", "j2_arm", "flange", "tool0"]
        for parent, child in zip(chain, chain[1:]):
            self.assertTrue(any(j.parent == parent and j.child == child for j in self.urdf.joints.values()))


class JointMappingTests(unittest.TestCase):
    def setUp(self):
        self.model = build_model()

    def test_round_trip(self):
        rng = random.Random(42)
        for _ in range(200):
            sdk = {"j1_deg": rng.uniform(-90, 90), "j2_deg": rng.uniform(-164, 164),
                   "j3_mm": rng.uniform(-310, 0), "j4_deg": rng.uniform(-1080, 1080)}
            back = model_to_sdk(self.model, sdk_to_model(self.model, sdk))
            for field in SDK_FIELDS:
                self.assertAlmostEqual(back[field], sdk[field], delta=1e-9)

    def test_matches_the_sdks_own_si_conversion(self):
        positions = zarm.JointPositions(j1_deg=12.5, j2_deg=-40.0, j3_mm=-123.4, j4_deg=270.0)
        values = sdk_to_model(self.model, dict(zip(SDK_FIELDS, positions.as_tuple())))
        expected = dict(zip(("j1_shoulder", "j2_elbow", "j3_vertical", "j4_wrist"), positions.to_si()))
        self.assertEqual(set(values), set(expected))
        for name, value in expected.items():
            self.assertAlmostEqual(values[name], value, delta=1e-15)

    def test_sdk_order(self):
        self.assertEqual([joint.sdk_field for joint in self.model.sdk_joints()], list(SDK_FIELDS))
        self.assertEqual([joint.name for joint in self.model.movable_joints()],
                         ["j3_vertical", "j1_shoulder", "j2_elbow", "j4_wrist"])


if __name__ == "__main__":
    unittest.main()

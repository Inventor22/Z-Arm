"""The MJCF: reusable pieces (prefix and base pose), and, when MuJoCo is installed, that it loads and agrees."""

from __future__ import annotations

import math
import random
import unittest
import xml.etree.ElementTree as ET

from tests.support import HAS_MUJOCO, zarm
from zarm_sim.generate import MJCF_NAME, MODELS_DIR
from zarm_sim.mjcf import arm_mjcf, scene, yaw_quat
from zarm_sim.model import build_model
from zarm_sim.xmlfmt import to_text

NAME_ATTRIBUTES = ("name", "joint", "material", "objname", "refname", "body1", "body2")


def names_in(elements):
    for element in elements:
        for node in element.iter():
            for attribute in NAME_ATTRIBUTES:
                if attribute in node.attrib:
                    yield attribute, node.attrib[attribute]


class MjcfPiecesTests(unittest.TestCase):
    def setUp(self):
        self.model = build_model()

    def test_every_name_carries_the_prefix(self):
        arm = arm_mjcf(self.model, prefix="cell_zarm_", pos=(1.0, -0.5, 0.75), quat=yaw_quat(math.pi / 2))
        elements = [arm.body, *arm.materials, *arm.actuators, *arm.sensors, *arm.excludes]
        found = list(names_in(elements))
        self.assertGreater(len(found), 30)
        for attribute, value in found:
            with self.subTest(attribute=attribute, value=value):
                self.assertTrue(value.startswith("cell_zarm_"))
        self.assertTrue(all(name.startswith("cell_zarm_") for name in arm.joints))
        self.assertEqual(arm.sites, {"tool0": "cell_zarm_tool0", "sdk_origin": "cell_zarm_sdk_origin"})

    def test_base_pose_is_applied_to_the_root_body(self):
        arm = arm_mjcf(self.model, prefix="z_", pos=(1.0, -0.5, 0.75), quat=yaw_quat(math.pi / 2))
        self.assertEqual(arm.body.get("name"), "z_base_link")
        self.assertEqual(arm.body.get("pos"), "1 -0.5 0.75")
        self.assertEqual(arm.body.get("quat"), "0.707106781 0 0 0.707106781")

    def test_bad_prefixes_are_refused(self):
        for prefix in ("", "a b", "x/"):
            with self.subTest(prefix=prefix), self.assertRaises(ValueError):
                arm_mjcf(self.model, prefix=prefix)

    def test_two_arms_compose_without_name_clashes(self):
        arms = [arm_mjcf(self.model, "left_"), arm_mjcf(self.model, "right_", pos=(0.8, 0.0, 0.0), quat=yaw_quat(math.pi))]
        root = scene(arms, "two_zarms")
        by_kind = {}
        for node in root.iter():
            if "name" in node.attrib:  # MuJoCo keeps a namespace per kind of element
                by_kind.setdefault(node.tag, []).append(node.attrib["name"])
        for kind, names in by_kind.items():
            with self.subTest(kind=kind):
                self.assertEqual(len(names), len(set(names)))
        self.assertEqual(len(by_kind["joint"]), 8)
        key = root.find("keyframe/key")
        self.assertEqual(len(key.get("qpos").split()), 8)

    def test_committed_files_are_strict_xml(self):
        for name in (MJCF_NAME, "zarm_2442.urdf"):
            with self.subTest(file=name):
                ET.parse(MODELS_DIR / name)

    def test_joint_order_and_servo_order(self):
        arm = arm_mjcf(self.model)
        self.assertEqual(arm.joints, ("zarm_j3_vertical", "zarm_j1_shoulder", "zarm_j2_elbow", "zarm_j4_wrist"))
        self.assertEqual([a.get("joint") for a in arm.actuators],
                         ["zarm_j1_shoulder", "zarm_j2_elbow", "zarm_j3_vertical", "zarm_j4_wrist"])

    def test_committed_file_parses(self):
        root = ET.parse(MODELS_DIR / MJCF_NAME).getroot()
        self.assertEqual(root.tag, "mujoco")
        self.assertIsNotNone(root.find(".//body[@name='zarm_base_link']"))
        self.assertIsNotNone(root.find(".//exclude[@body1='zarm_base_link'][@body2='zarm_carriage']"))


@unittest.skipUnless(HAS_MUJOCO, "mujoco isn't installed (pip install mujoco to run these)")
class MujocoTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import mujoco

        cls.mujoco = mujoco
        cls.mj_model = mujoco.MjModel.from_xml_path(str(MODELS_DIR / MJCF_NAME))

    def qpos_index(self, name):
        return self.mj_model.jnt_qposadr[self.mujoco.mj_name2id(self.mj_model, self.mujoco.mjtObj.mjOBJ_JOINT, name)]

    def test_loads_with_four_joints_and_servos(self):
        self.assertEqual(self.mj_model.nq, 4)
        self.assertEqual(self.mj_model.nu, 4)

    def test_forward_kinematics_matches_the_sdk(self):
        mujoco, m = self.mujoco, self.mj_model
        data = mujoco.MjData(m)
        rng = random.Random(7)
        for _ in range(100):
            j1, j2, j3, j4 = rng.uniform(-90, 90), rng.uniform(-164, 164), rng.uniform(-310, 0), rng.uniform(-1080, 1080)
            data.qpos[self.qpos_index("zarm_j1_shoulder")] = math.radians(j1)
            data.qpos[self.qpos_index("zarm_j2_elbow")] = math.radians(j2)
            data.qpos[self.qpos_index("zarm_j3_vertical")] = j3 / 1000.0
            data.qpos[self.qpos_index("zarm_j4_wrist")] = math.radians(j4)
            mujoco.mj_kinematics(m, data)
            tool, origin = data.site("zarm_tool0"), data.site("zarm_sdk_origin")
            dx, dy, dz = (tool.xpos - origin.xpos) * 1000.0
            ex, ey = zarm.kinematics.forward_xy(j1, j2, 220.0, 200.0)
            yaw = math.degrees(math.atan2(tool.xmat[3], tool.xmat[0]))
            expected = zarm.kinematics.tool_yaw_deg(j1, j2, j4)
            with self.subTest(j1=j1, j2=j2, j3=j3, j4=j4):
                self.assertAlmostEqual(dx, ex, delta=1e-6)
                self.assertAlmostEqual(dy, ey, delta=1e-6)
                self.assertAlmostEqual(dz, j3, delta=1e-6)
                self.assertAlmostEqual((yaw - expected + 180.0) % 360.0 - 180.0, 0.0, delta=1e-6)

    def test_position_servos_reach_their_targets_without_contacts(self):
        mujoco, m = self.mujoco, self.mj_model
        data = mujoco.MjData(m)
        mujoco.mj_resetDataKeyframe(m, data, 0)
        targets = {"zarm_j1_shoulder": math.radians(40), "zarm_j2_elbow": math.radians(-60),
                   "zarm_j3_vertical": -0.2, "zarm_j4_wrist": math.radians(120)}
        for index in range(m.nu):
            data.ctrl[index] = targets[m.actuator(index).name]
        for _ in range(1500):  # 3 s
            mujoco.mj_step(m, data)
        for name, target in targets.items():
            with self.subTest(joint=name):
                self.assertAlmostEqual(data.qpos[self.qpos_index(name)], target, delta=1e-3)
        self.assertEqual(data.ncon, 0)

    def test_a_two_arm_scene_loads(self):
        model = build_model()
        arms = [arm_mjcf(model, "a_"), arm_mjcf(model, "b_", pos=(0.9, 0.0, 0.0), quat=yaw_quat(math.pi))]
        mj = self.mujoco.MjModel.from_xml_string(to_text(scene(arms, "two_zarms")))
        self.assertEqual(mj.nq, 8)
        self.assertEqual(mj.nu, 8)


if __name__ == "__main__":
    unittest.main()

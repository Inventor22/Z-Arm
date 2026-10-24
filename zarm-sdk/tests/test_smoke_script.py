"""The hardware smoke test's safety gate. These tests never touch hardware."""

import importlib.util
import io
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
from unittest import mock

from zarm import ArmPhase, ManualClock, ZArm

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "hardware_smoke_test.py"
_spec = importlib.util.spec_from_file_location("hardware_smoke_test", SCRIPT)
smoke = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(smoke)


def parse(*argv):
    return smoke.build_parser().parse_args(["--library-dir", "unused", *argv])


class RefusalTests(unittest.TestCase):
    def test_refusals(self):
        cases = {
            (): "nothing to do",
            ("--connect", "--card-id", "59"): "--i-am-at-the-estop",
            ("--connect", "--i-am-at-the-estop", "--card-id", "59", "--allow-unverified-build"): "--preflight-only",
            ("--connect", "--i-am-at-the-estop"): "--card-id",
            ("--connect", "--i-am-at-the-estop", "--card-id", "300"): "0..255",
            ("--initialize", "--i-am-at-the-estop", "--card-id", "59"): "requires --connect",
            ("--connect", "--allow-motion", "--i-am-at-the-estop", "--card-id", "59"): "requires --initialize",
            (
                "--connect", "--initialize", "--allow-motion", "--i-am-at-the-estop", "--card-id", "59",
                "--j4-delta-deg", "90", "--yes",
            ): "--j4-delta-deg",
            (
                "--connect", "--initialize", "--allow-motion", "--i-am-at-the-estop", "--card-id", "59",
                "--speed", "100", "--yes",
            ): "--speed",
        }
        for argv, expected in cases.items():
            with self.subTest(argv=argv):
                reason = smoke.refusal_reason(parse(*argv))
                self.assertIsNotNone(reason)
                self.assertIn(expected, reason)

    def test_motion_needs_a_terminal_or_yes(self):
        args = parse("--connect", "--initialize", "--allow-motion", "--i-am-at-the-estop", "--card-id", "59")
        with mock.patch.object(smoke.sys, "stdin", io.StringIO()):
            self.assertIn("--yes", smoke.refusal_reason(args))

    def test_fully_flagged_request_is_allowed(self):
        args = parse("--connect", "--initialize", "--i-am-at-the-estop", "--card-id", "59")
        self.assertIsNone(smoke.refusal_reason(args))

    def test_main_refuses_before_touching_the_library(self):
        with mock.patch.object(smoke.ZArm, "native", side_effect=AssertionError("must not be reached")):
            with redirect_stderr(io.StringIO()) as err:
                code = smoke.main(["--library-dir", "unused", "--connect", "--card-id", "59"])
        self.assertEqual(code, smoke.EXIT_REFUSED)
        self.assertIn("REFUSED", err.getvalue())

    def test_preflight_only_is_static(self):
        with tempfile.TemporaryDirectory() as directory:
            with mock.patch.object(smoke.ZArm, "native", side_effect=AssertionError("must not be reached")):
                with redirect_stdout(io.StringIO()) as out:
                    code = smoke.main(["--library-dir", directory, "--preflight-only"])
        self.assertEqual(code, smoke.EXIT_FAILED)
        self.assertIn("MISSING", out.getvalue())


class SimulatedRunTests(unittest.TestCase):
    """The script's hardware path, run against the simulator instead of the DLL."""

    ARGV = [
        "--library-dir", "unused", "--card-id", "59", "--i-am-at-the-estop",
        "--connect", "--initialize", "--allow-motion", "--yes",
    ]

    def run_script(self, start_joints=None):
        clock = ManualClock()
        arms = []

        def simulated(library_dir, config):
            arm = ZArm.simulated(config, clock=clock, sleep=clock.sleep)
            if start_joints is not None:
                arm.backend.arms[59].joints = start_joints
            arms.append(arm)
            return arm

        with mock.patch.object(smoke.ZArm, "native", side_effect=simulated), mock.patch.object(
            smoke.logging, "basicConfig"
        ), redirect_stdout(io.StringIO()) as out, redirect_stderr(io.StringIO()) as err:
            code = smoke.main(self.ARGV)
        return code, arms[0], out.getvalue() + err.getvalue()

    def test_motion_round_trip(self):
        code, arm, output = self.run_script()
        self.assertEqual(code, smoke.EXIT_OK, output)
        calls = arm.backend.call_names()
        self.assertEqual(calls.count("movej_angle"), 2)
        self.assertAlmostEqual(arm.backend.arms[59].joints[3], 0.0)
        self.assertIs(arm.phase, ArmPhase.CLOSED)
        self.assertIn("forward_xy with arm_lengths_mm (220.0, 200.0): (420.0, 0.0) mm, off by 0.0 mm", output)
        self.assertIn("forward_xy with vendor_arm_lengths_mm (200.0, 200.0): (400.0, 0.0) mm, off by 20.0 mm", output)

    def test_refuses_motion_when_the_return_position_is_outside_the_limits(self):
        code, arm, output = self.run_script(start_joints=(0.0, 0.0, 0.0, 200.0))
        self.assertEqual(code, smoke.EXIT_FAILED)
        self.assertNotIn("movej_angle", arm.backend.call_names())
        self.assertIn("no motion sent", output)


if __name__ == "__main__":
    unittest.main()

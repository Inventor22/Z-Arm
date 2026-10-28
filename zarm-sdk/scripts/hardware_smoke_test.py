"""Opt-in hardware smoke test for a real Hitbot Z-Arm.

Nothing touches the arm unless you ask for each step explicitly:

    --i-am-at-the-estop   required for any hardware step: you are at the emergency stop
    --card-id N           the arm's id on the vendor server (assumed 59, the legacy id)
    --connect             start the vendor server and connect; reads state, no motion
    --initialize          also run the vendor initialization (energizes the arm)
    --allow-motion        also rotate J4 by +/- --j4-delta-deg at --speed and back
                          (asks you to type MOVE unless --yes is given)

``--preflight-only`` checks the vendor directory statically and touches nothing.

After connecting (and again after initializing) it prints a read-only check of
the SDK's kinematic assumptions: the vendor's reported (x, y) against
``forward_xy`` with the assumed link lengths.

Example (connect and read state only):

    python scripts/hardware_smoke_test.py --library-dir C:\\hitbot\\vendor --card-id 59 --i-am-at-the-estop --connect

Exit codes: 0 success, 1 failure, 2 refused.
"""

from __future__ import annotations

import argparse
import logging
import math
import sys
from pathlib import Path
from typing import List, Optional

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from zarm import ArmState, JointPositions, ZArm, ZArmConfig, ZArmError, kinematics  # noqa: E402
from zarm.native import LibraryReport, describe_python, inspect_library_dir  # noqa: E402

EXIT_OK, EXIT_FAILED, EXIT_REFUSED = 0, 1, 2
MAX_SPEED = 30.0
MAX_J4_DELTA_DEG = 15.0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Opt-in Z-Arm hardware smoke test. Refuses to touch the arm without explicit flags.",
    )
    parser.add_argument("--library-dir", required=True, help="directory with small_scara_interface.dll, share.dll, server.exe")
    parser.add_argument("--preflight-only", action="store_true", help="static checks only; touches nothing")
    parser.add_argument("--card-id", type=int, help="the arm's card id on the vendor server (assumed 59)")
    parser.add_argument("--i-am-at-the-estop", action="store_true", help="confirm you are at the hardware E-stop")
    parser.add_argument("--connect", action="store_true", help="start the vendor server and connect (no motion)")
    parser.add_argument("--initialize", action="store_true", help="run the vendor initialization (energizes the arm)")
    parser.add_argument("--allow-motion", action="store_true", help="rotate J4 a few degrees and back")
    parser.add_argument("--j4-delta-deg", type=float, default=5.0, help=f"J4 rotation, 0 < deg <= {MAX_J4_DELTA_DEG:g}")
    parser.add_argument("--speed", type=float, default=10.0, help=f"joint speed, 0 < speed <= {MAX_SPEED:g}")
    parser.add_argument("--yes", action="store_true", help="skip the interactive MOVE confirmation")
    parser.add_argument(
        "--allow-unverified-build",
        action="store_true",
        help="with --preflight-only: report on a vendor set that is not in VERIFIED_BUNDLES",
    )
    parser.add_argument("--connect-timeout-s", type=float, default=15.0)
    return parser


def refusal_reason(args: argparse.Namespace) -> Optional[str]:
    """Why the requested hardware steps must not run, or None if they may."""
    if not (args.connect or args.initialize or args.allow_motion):
        return "nothing to do: pass --connect (and optionally --initialize, --allow-motion), or --preflight-only"
    if args.allow_unverified_build:
        return "--allow-unverified-build is only accepted with --preflight-only; hardware steps need the verified set"
    if not args.i_am_at_the_estop:
        return "refusing to touch the arm without --i-am-at-the-estop"
    if args.card_id is None:
        return "refusing without --card-id; the smoke test does not guess which arm to drive"
    if not 0 <= args.card_id <= 255:
        return "--card-id must be in 0..255"
    if args.initialize and not args.connect:
        return "--initialize requires --connect"
    if args.allow_motion and not args.initialize:
        return "--allow-motion requires --initialize"
    if args.allow_motion:
        if not 0 < args.j4_delta_deg <= MAX_J4_DELTA_DEG:
            return f"--j4-delta-deg must be in (0, {MAX_J4_DELTA_DEG:g}]"
        if not 0 < args.speed <= MAX_SPEED:
            return f"--speed must be in (0, {MAX_SPEED:g}]"
        if not args.yes and not sys.stdin.isatty():
            return "--allow-motion needs an interactive terminal for the MOVE confirmation, or --yes"
    return None


def print_report(report: LibraryReport) -> None:
    print(f"vendor directory: {report.library_dir}")
    for name, present in report.files_present.items():
        print(f"  {name:28} {'present' if present else 'MISSING'}")
    print(f"  machine: {report.machine}; verified set: {report.verified_build}")
    for name, digest in report.sha256.items():
        print(f"  sha256 {name:26} {digest}")
    for problem in report.problems:
        print(f"  problem: {problem}")


def print_kinematics_check(state: ArmState, config: ZArmConfig) -> None:
    """Compare the vendor's (x, y) with forward_xy under the physical and the vendor link lengths."""
    joints, pose = state.joints, state.pose
    print(f"kinematics check: J1 {joints.j1_deg:.2f} deg, J2 {joints.j2_deg:.2f} deg; "
          f"vendor reports (x, y) = ({pose.x_mm:.1f}, {pose.y_mm:.1f}) mm")
    candidates = (("arm_lengths_mm", config.arm_lengths_mm), ("vendor_arm_lengths_mm", config.vendor_arm_lengths_mm))
    for label, lengths in candidates:
        x, y = kinematics.forward_xy(joints.j1_deg, joints.j2_deg, *lengths)
        error = math.hypot(x - pose.x_mm, y - pose.y_mm)
        print(f"  forward_xy with {label} {lengths}: ({x:.1f}, {y:.1f}) mm, off by {error:.1f} mm")
    print("  (a match within about 1 mm for arm_lengths_mm confirms the link lengths and angle conventions)")


def run_hardware_steps(args: argparse.Namespace) -> int:
    config = ZArmConfig(card_id=args.card_id, max_joint_speed=MAX_SPEED)
    arm = ZArm.native(args.library_dir, config)
    try:
        print(f"connecting to card {args.card_id} (starts server.exe if needed)...")
        arm.connect(timeout_s=args.connect_timeout_s)
        state = arm.get_state()
        print("state:", state)
        print_kinematics_check(state, config)
        print("joint states:", {joint.name: state for joint, state in arm.get_joint_states().items()})
        if not args.initialize:
            return EXIT_OK

        print("initializing...")
        arm.initialize()
        start = arm.get_state()
        print("state after initialize:", start)
        print_kinematics_check(start, config)
        if not args.allow_motion:
            return EXIT_OK

        here = start.joints
        limits = config.resolved_joint_limits()
        problems = limits.violations(here)
        if problems:
            print("the arm is outside the soft limits, so the return move would be refused; no motion sent:", file=sys.stderr)
            for problem in problems:
                print(f"  {problem}", file=sys.stderr)
            return EXIT_FAILED
        delta = args.j4_delta_deg
        low, high = limits.j4_deg
        if here.j4_deg + delta > high:
            delta = -delta
        if not low <= here.j4_deg + delta <= high:
            print("J4 is too close to its soft limits to rotate; no motion sent", file=sys.stderr)
            return EXIT_FAILED
        target = JointPositions(here.j1_deg, here.j2_deg, here.j3_mm, here.j4_deg + delta)
        print(f"about to rotate J4 from {here.j4_deg:.2f} to {target.j4_deg:.2f} deg at speed {args.speed:g}, then back")
        if not args.yes and input("type MOVE to continue: ").strip() != "MOVE":
            print("not confirmed; no motion sent")
            return EXIT_REFUSED
        print("after J4 move:", arm.move_joints(target, speed=args.speed))
        print("after return:", arm.move_joints(here, speed=args.speed))
        return EXIT_OK
    except ZArmError as exc:
        print(f"FAILED: {type(exc).__name__}: {exc}", file=sys.stderr)
        return EXIT_FAILED
    finally:
        arm.close()


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.preflight_only:
        print(describe_python())
        report = inspect_library_dir(args.library_dir, allow_unverified_build=args.allow_unverified_build)
        print_report(report)
        return EXIT_OK if report.ok else EXIT_FAILED

    reason = refusal_reason(args)
    if reason is not None:
        print(f"REFUSED: {reason}", file=sys.stderr)
        return EXIT_REFUSED

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    return run_hardware_steps(args)


if __name__ == "__main__":
    sys.exit(main())

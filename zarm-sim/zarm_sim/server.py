"""The simulator view: a web page on this PC showing a Z-Arm driven by zarm-sdk's simulated backend.

    cd zarm-sim
    py -3.12 -m zarm_sim.server          # then open http://127.0.0.1:8770

The server owns one ``ZArm.simulated(...)``; the page never talks to the SDK
itself, and every command goes through the server one at a time. It refuses to
drive anything but zarm's FakeBackend: it never imports the native backend, and
:func:`zarm_sim.sdk.require_simulated` checks the arm it's given.

Jogs follow umbratek-code's jog panel: each click moves one exact step from the
last commanded goal (not from the lagging measured position), so quick clicks
add up instead of being lost; the goal may run at most ``MAX_LEAD_STEPS`` steps
ahead of the arm. The simulated controller, like the real one, refuses a move
while another is running, so clicks made during a step are queued and sent as
one move when it ends. Stop works at any time. A move under way is stopped if
the page goes quiet. Commands are sent with ``wait=False``; a supervisor thread
then waits with the SDK's ``wait_until_stopped`` (which stops the arm on
timeouts and faults) and checks the arm arrived.

The server listens on 127.0.0.1 only, checks the Host header, and requires a
per-start token on every POST, so other web pages can't drive it.
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
import os
import re
import secrets
import socket
import socketserver
import sys
import threading
import time
import urllib.parse
import webbrowser
from collections import deque
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable, Deque, Dict, List, Mapping, Optional, Tuple

from .generate import MJCF_NAME, MODELS_DIR, URDF_NAME, stale_files
from .kinematics import UrdfRobot, xyz_yaw
from .meshes import MESH_URL, fetch_command, load_meshes
from .model import RobotModel, build_model, sdk_to_model
from .sdk import import_zarm, require_simulated
from .three_js import THREE_URL, THREE_VERSION, load_three
from .workspace import Workspace, envelope_outline, envelope_png

zarm = import_zarm()
from zarm import (  # noqa: E402  (zarm-sdk is found by import_zarm above)
    ArmPhase,
    CartesianPose,
    JointPositions,
    SoftLimitError,
    ZArm,
    ZArmConfig,
    ZArmError,
)
from zarm import kinematics as sdk_kinematics  # noqa: E402

DEFAULT_PORT = 8770  # umbratek-code's jog panel uses 8765
ARM_LENGTHS_MM = (220.0, 200.0)  # Z-Arm 2442; passed explicitly so it holds whatever the SDK's default
WEB_DIR = Path(__file__).resolve().parent / "web"
PAGE = WEB_DIR / "index.html"
STATIC_FILES = {"app.js", "view.js", "urdf.js", "plan.js", "style.css"}
STATIC_TYPES = {".js": "text/javascript; charset=utf-8", ".css": "text/css; charset=utf-8"}
SDK_FIELDS = ("j1_deg", "j2_deg", "j3_mm", "j4_deg")
JOINT_STEPS = (0.1, 0.5, 1.0, 5.0, 10.0)  # deg per click for J1, J2, J4; mm for J3
JOINT_SPEEDS = (5.0, 10.0, 20.0, 30.0, 50.0, 100.0)  # deg/s (J1, J2, J4) and mm/s (J3), the SDK's joint speed
LINEAR_SPEEDS = (5.0, 10.0, 20.0, 50.0, 100.0)  # mm/s
DEFAULTS = {"jointStep": 1.0, "jointSpeed": 30.0, "linearSpeed": 50.0}
HANDS = {"none": None, "plus": 1, "minus": -1, "keep": 0}  # move_to_pose's lr; see the README
MAX_LEAD_STEPS = 3
PAGE_SILENT_S = 3.0
POLL_MS = 33  # about 30 Hz
FK_TOLERANCE_MM = 0.5
MAX_BODY_BYTES = 4096
LOG_SIZE = 12


class PanelError(Exception):
    """A request the simulator view refuses by itself (the SDK's own refusals are ZArmErrors)."""


def _number(body: Mapping[str, Any], key: str) -> float:
    value = body.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise PanelError(f"{key} must be a number, got {value!r}")
    return float(value)


def _pick(body: Mapping[str, Any], key: str, allowed: Tuple[float, ...]) -> float:
    value = _number(body, key)
    if value not in allowed:
        raise PanelError(f"{key} must be one of {', '.join(f'{a:g}' for a in allowed)}, got {value:g}")
    return value


def _rounded(values: Mapping[str, float], digits: int = 3) -> Dict[str, float]:
    return {key: round(float(value), digits) for key, value in values.items()}


def describe_error(error: BaseException) -> str:
    return f"{type(error).__name__}: {error}"


def _n(value: float) -> str:
    """A number for messages, with a typographic minus."""
    return f"{value:g}".replace("-", "\u2212")


@dataclass
class Motion:
    kind: str  # "jog", "joints", "pose", "linear" or "hand"
    label: str
    started_s: float
    timeout_s: float
    target_joints: Optional[JointPositions] = None
    target_pose: Optional[CartesianPose] = None


class Simulator:
    """Owns the one simulated ZArm. Every command goes through here, one at a time."""

    def __init__(
        self,
        arm: ZArm,
        model: RobotModel,
        urdf: UrdfRobot,
        *,
        clock: Callable[[], float] = time.monotonic,
        page_silent_s: float = PAGE_SILENT_S,
    ) -> None:
        require_simulated(arm)
        self.arm = arm
        self.model = model
        self.urdf = urdf
        self.clock = clock
        self.page_silent_s = page_silent_s
        self.config = arm.config
        self.limits = self.config.resolved_joint_limits()
        self.workspace = Workspace.from_model(model, self.limits.j1_deg, self.limits.j2_deg)
        self.lock = threading.RLock()
        self.goal: Optional[JointPositions] = None  # the commanded joint goal that jogs build on
        self.pending: Optional[Tuple[JointPositions, float, str]] = None  # a jog waiting for the step under way
        self.active: Optional[Motion] = None  # the move the supervisor should watch
        self.waiting: Optional[Motion] = None  # the move the supervisor is waiting on right now
        self.log: Deque[Dict[str, Any]] = deque(maxlen=LOG_SIZE)
        self.log_seq = 0
        self.heard_at = clock()
        self.closing = False
        self.warnings: List[str] = []
        self._wake = threading.Event()
        self._threads: List[threading.Thread] = []
        lengths = tuple(float(v) for v in self.config.arm_lengths_mm)
        model_lengths = (model.dimensions.j1_arm_length_mm, model.dimensions.j2_arm_length_mm)
        if lengths != model_lengths:
            self.warnings.append(
                f"the SDK's arm lengths {lengths} differ from the model's {model_lengths}; the view will disagree"
            )

    # Lifecycle

    def start(self) -> None:
        self.arm.connect()
        self.arm.initialize()
        self.say("Connected to the simulated arm (zarm FakeBackend) and initialized.")
        for target in (self._supervise, self._watch_page):
            thread = threading.Thread(target=target, daemon=True)
            thread.start()
            self._threads.append(thread)

    def close(self) -> None:
        self.closing = True
        self._wake.set()
        try:
            if self.arm.phase in (ArmPhase.CONNECTED, ArmPhase.INITIALIZED):
                self.arm.stop()
        finally:
            self.arm.close()
        for thread in self._threads:
            thread.join(timeout=2.0)

    # Status

    def say(self, text: str, level: str = "ok") -> str:
        with self.lock:
            self.log_seq += 1
            self.log.append({"seq": self.log_seq, "time": time.strftime("%H:%M:%S"), "level": level, "text": text})
        return text

    def heard(self) -> None:
        self.heard_at = self.clock()

    def tool_mm(self, joints: JointPositions) -> Tuple[float, float, float, float]:
        """The model's tool0 in the SDK's frame (mm, deg), from the committed URDF."""
        values = sdk_to_model(self.model, dict(zip(SDK_FIELDS, joints.as_tuple())))
        x, y, z, yaw = xyz_yaw(self.urdf.relative(values, "tool0", "sdk_origin"))
        return (x * 1000.0, y * 1000.0, z * 1000.0, math.degrees(yaw))

    def status(self) -> Dict[str, Any]:
        self.heard()
        with self.lock:
            phase = self.arm.phase
            out: Dict[str, Any] = {
                "simulated": True,
                "backend": type(self.arm.backend).__name__,
                "phase": phase.value,
                "ready": phase is ArmPhase.INITIALIZED,
                "busy": self.active is not None or self.waiting is not None,
                "queued": self.pending is not None,
                "log": list(self.log),
                "warnings": list(self.warnings),
            }
            active = self.active
            goal = self.goal
        if phase not in (ArmPhase.CONNECTED, ArmPhase.INITIALIZED):
            return out
        try:
            state = self.arm.get_state()
        except ZArmError as error:
            out["stateError"] = describe_error(error)
            return out
        joints, pose = state.joints, state.pose
        out["joints"] = _rounded(dict(zip(SDK_FIELDS, joints.as_tuple())), 4)
        out["pose"] = _rounded({"x_mm": pose.x_mm, "y_mm": pose.y_mm, "z_mm": pose.z_mm, "r_deg": pose.r_deg}, 4)
        out["toolYawDeg"] = round(sdk_kinematics.tool_yaw_deg(joints.j1_deg, joints.j2_deg, joints.j4_deg), 3)
        out["moving"] = state.moving
        out["initialized"] = state.initialized
        tool = self.tool_mm(joints)
        error_mm = math.dist(tool[:3], (pose.x_mm, pose.y_mm, pose.z_mm))
        out["modelCheck"] = {"errorMm": round(error_mm, 3), "ok": error_mm <= FK_TOLERANCE_MM}
        clearance = self.workspace.j2_clearance_mm(joints.j1_deg, joints.j2_deg)
        out["blindZone"] = {"hit": clearance <= 0.0, "clearanceMm": round(max(clearance, 0.0), 1)}
        if active is not None and active.target_joints is not None:
            x, y, z, _ = self.tool_mm(active.target_joints)
            out["goal"] = {"label": active.label, "joints": _rounded(dict(zip(SDK_FIELDS, active.target_joints.as_tuple())), 3),
                           "tool": _rounded({"x_mm": x, "y_mm": y, "z_mm": z}, 2)}
        elif active is not None and active.target_pose is not None:
            p = active.target_pose
            out["goal"] = {"label": active.label, "tool": _rounded({"x_mm": p.x_mm, "y_mm": p.y_mm, "z_mm": p.z_mm}, 2)}
        if goal is not None and "goal" in out:
            out["goal"]["jogGoal"] = _rounded(dict(zip(SDK_FIELDS, goal.as_tuple())), 3)
        return out

    # Commands

    def _require_ready(self) -> None:
        phase = self.arm.phase
        if phase is ArmPhase.CONNECTED:
            raise PanelError("The arm needs Initialize first (after an emergency stop, or when the controller said so).")
        if phase is not ArmPhase.INITIALIZED:
            raise PanelError(f"The arm isn't available (phase {phase.value}).")

    def _require_idle(self) -> None:
        if self.active is not None or self.waiting is not None:
            raise PanelError("A move is under way. Wait for it to finish, or press Stop.")

    def _timeout(self, estimate_s: float) -> float:
        config = self.config
        computed = config.motion_timeout_factor * estimate_s + config.motion_timeout_margin_s
        return min(computed, config.max_motion_timeout_s) if math.isfinite(computed) else config.max_motion_timeout_s

    def _start(self, motion: Motion) -> None:
        self.active = motion
        self._wake.set()

    def _send_joints(self, target: JointPositions, speed: float, kind: str, label: str) -> None:
        start = self.arm.get_state().joints
        self.arm.move_joints(target, speed=speed, wait=False)
        estimate = max(abs(a - b) for a, b in zip(start.as_tuple(), target.as_tuple())) / speed
        self._start(Motion(kind, label, self.clock(), self._timeout(estimate), target_joints=target))

    def _close_to(self, actual: JointPositions, target: JointPositions) -> bool:
        tol_deg, tol_mm = self.config.arrival_tolerance_deg, self.config.arrival_tolerance_mm
        return all(
            abs(a - b) <= (tol_mm if field == "j3_mm" else tol_deg)
            for field, a, b in zip(SDK_FIELDS, actual.as_tuple(), target.as_tuple())
        )

    def jog(self, body: Mapping[str, Any]) -> Tuple[bool, str]:
        joint = body.get("joint")
        if isinstance(joint, bool) or joint not in (1, 2, 3, 4):
            raise PanelError(f"joint must be 1, 2, 3 or 4, got {joint!r}")
        direction = body.get("dir")
        if isinstance(direction, bool) or direction not in (1, -1):
            raise PanelError(f"dir must be 1 or -1, got {direction!r}")
        step = _pick(body, "step", JOINT_STEPS)
        speed = _number(body, "speed")
        field = SDK_FIELDS[joint - 1]
        unit = " mm" if field == "j3_mm" else "\u00b0"
        with self.lock:
            self._require_ready()
            state = self.arm.get_state()
            active = self.active
            if self.waiting is not None and active is None:
                raise PanelError("Still stopping; try again in a moment.")
            if active is not None and active.kind != "jog":
                raise PanelError("A move is under way. Wait for it to finish, or press Stop, before jogging.")
            busy = active is not None
            # Exact steps from the commanded goal while it's still valid; otherwise from where the arm is.
            if self.goal is not None and (busy or self._close_to(state.joints, self.goal)):
                base = self.goal
            else:
                base = state.joints
            values = dict(zip(SDK_FIELDS, base.as_tuple()))
            values[field] += direction * step
            target = JointPositions(**values)
            if abs(values[field] - getattr(state.joints, field)) > MAX_LEAD_STEPS * step + 1e-6:
                return False, self.say("Holding: the arm is still catching up with earlier clicks.", "warn")
            sign = "+" if direction > 0 else "\u2212"
            reached = f"{values[field]:.2f}".replace("-", "\u2212")
            label = f"J{joint} {sign}{step:g}{unit} \u2192 {reached}{unit}"
            if busy:
                problems = self.limits.violations(target)
                if problems:  # the SDK would refuse it when sent; say so now
                    raise SoftLimitError("target is outside the soft limits: " + "; ".join(problems))
                self.goal = target
                self.pending = (target, speed, label)
                return True, self.say(label + " (queued behind the step under way)")
            self._send_joints(target, speed, "jog", label)
            self.goal = target
            return True, self.say(label)

    def move_joints(self, body: Mapping[str, Any]) -> Tuple[bool, str]:
        target = JointPositions(*(_number(body, field) for field in SDK_FIELDS))
        speed = _number(body, "speed")
        with self.lock:
            self._require_ready()
            self._require_idle()
            label = (f"Joint move to J1 {_n(target.j1_deg)}\u00b0, J2 {_n(target.j2_deg)}\u00b0, J3 {_n(target.j3_mm)} mm, "
                     f"J4 {_n(target.j4_deg)}\u00b0 at {_n(speed)}\u00b0/s (J3 mm/s)")
            self._send_joints(target, speed, "joints", label)
            self.goal = target
            return True, self.say(label + "\u2026")

    def _pose(self, body: Mapping[str, Any]) -> CartesianPose:
        return CartesianPose(_number(body, "x_mm"), _number(body, "y_mm"), _number(body, "z_mm"), _number(body, "r_deg"))

    def move_pose(self, body: Mapping[str, Any]) -> Tuple[bool, str]:
        target = self._pose(body)
        speed = _number(body, "speed")
        hand = body.get("hand", "none")
        if hand not in HANDS:
            raise PanelError(f"hand must be one of {', '.join(HANDS)}, got {hand!r}")
        lr = HANDS[hand]
        with self.lock:
            self._require_ready()
            self._require_idle()
            start = self.arm.get_state()
            self.arm.move_to_pose(target, speed=speed, lr=lr, wait=False)
            # A bound for the supervisor's timeout: the slowest joint of the farther hand solution.
            l1, l2 = self.config.arm_lengths_mm
            deltas = [max(abs(j1 - start.joints.j1_deg), abs(j2 - start.joints.j2_deg))
                      for j1, j2 in sdk_kinematics.inverse_xy_solutions(target.x_mm, target.y_mm, l1, l2)]
            other = max(abs(target.z_mm - start.pose.z_mm), abs(target.r_deg - start.pose.r_deg))
            estimate = max(max(deltas) if deltas else 360.0, other) / speed
            hand_text = "current hand" if lr is None else f"lr {lr:+d}" if lr else "lr 0"
            label = (f"Move to x {_n(target.x_mm)}, y {_n(target.y_mm)}, z {_n(target.z_mm)} mm, r {_n(target.r_deg)}\u00b0 "
                     f"({hand_text}) at {_n(speed)}\u00b0/s (J3 mm/s)")
            self._start(Motion("pose", label, self.clock(), self._timeout(estimate), target_pose=target))
            self.goal = None
            return True, self.say(label + "\u2026")

    def move_linear(self, body: Mapping[str, Any]) -> Tuple[bool, str]:
        target = self._pose(body)
        speed = _number(body, "speed_mm_s")
        with self.lock:
            self._require_ready()
            self._require_idle()
            start = self.arm.get_state().pose
            self.arm.move_linear(target, speed_mm_s=speed, wait=False)
            distance = math.dist(start.as_tuple()[:3], target.as_tuple()[:3])
            estimate = max(distance, abs(target.r_deg - start.r_deg)) / speed
            label = (f"Linear move to x {_n(target.x_mm)}, y {_n(target.y_mm)}, z {_n(target.z_mm)} mm, "
                     f"r {_n(target.r_deg)}\u00b0 at {_n(speed)} mm/s")
            self._start(Motion("linear", label, self.clock(), self._timeout(estimate), target_pose=target))
            self.goal = None
            return True, self.say(label + "\u2026")

    def change_hand(self, body: Mapping[str, Any]) -> Tuple[bool, str]:
        speed = _number(body, "speed")
        with self.lock:
            self._require_ready()
            self._require_idle()
            start = self.arm.get_state().joints
            self.arm.change_hand(speed=speed, wait=False)
            j1, j2 = sdk_kinematics.alternate_hand(start.j1_deg, start.j2_deg, *self.config.arm_lengths_mm)
            target = JointPositions(j1, j2, start.j3_mm, start.j4_deg)
            estimate = max(abs(j1 - start.j1_deg), abs(j2 - start.j2_deg)) / speed
            label = f"Change hand: J1 {_n(round(j1, 2))}\u00b0, J2 {_n(round(j2, 2))}\u00b0 at {_n(speed)}\u00b0/s"
            self._start(Motion("hand", label, self.clock(), self._timeout(estimate), target_joints=target))
            self.goal = None
            return True, self.say(label + "\u2026")

    def _drop_motion(self) -> None:
        self.pending = None
        self.goal = None
        self.active = None

    def stop(self, cause: Optional[str] = None) -> Tuple[bool, str]:
        self.arm.stop()  # at once, without waiting for anything (lock-free in the SDK)
        with self.lock:
            self._drop_motion()
            self.arm.stop()  # and again, for a move sent in between
            if cause is None:
                return True, self.say("Stopped.")
            return True, self.say(f"Stopped because {cause}.")

    def emergency_stop(self) -> Tuple[bool, str]:
        self.arm.emergency_stop()  # lock-free; latches: the arm needs initialize() again
        with self.lock:
            self._drop_motion()
            return True, self.say(
                "Emergency stop sent. It latches: press Initialize before the arm will move again."
            )

    def initialize(self) -> Tuple[bool, str]:
        with self.lock:
            self._require_idle()
            self.arm.initialize()
            self.goal = None
            return True, self.say("Initialized: the arm accepts moves again.")

    # Background work

    def _arrived(self, motion: Motion, joints: JointPositions, pose: CartesianPose) -> bool:
        if motion.target_joints is not None:
            return self._close_to(joints, motion.target_joints)
        if motion.target_pose is not None:
            target = motion.target_pose
            return (math.dist(pose.as_tuple()[:3], target.as_tuple()[:3]) <= self.config.arrival_tolerance_mm
                    and abs(pose.r_deg - target.r_deg) <= self.config.arrival_tolerance_deg)
        return True

    def _supervise(self) -> None:
        while not self.closing:
            self._wake.wait(0.1)
            self._wake.clear()
            with self.lock:
                motion = self.active
                self.waiting = motion
            if motion is None:
                continue
            state, error = None, None
            try:
                state = self.arm.wait_until_stopped(timeout_s=motion.timeout_s)  # stops the arm on errors
            except Exception as caught:  # ZArmError, or anything unexpected: report it, don't die
                error = caught
            with self.lock:
                self.waiting = None
                if self.active is not motion:  # stopped (and maybe replaced) meanwhile
                    self._wake.set()
                    continue
                self.active = None
                if error is not None or state is None:
                    self.pending = None
                    self.goal = None
                    self.say(describe_error(error) if error is not None else "the move ended without a state", "error")
                    continue
                if not self._arrived(motion, state.joints, state.pose):
                    self.pending = None
                    self.goal = None
                    self.say(f"{motion.label}: the arm stopped away from the target.", "error")
                    continue
                if self.pending is not None:
                    target, speed, label = self.pending
                    self.pending = None
                    try:
                        self._send_joints(target, speed, "jog", label)
                    except ZArmError as caught:
                        self.goal = None
                        self.say(describe_error(caught), "error")
                elif motion.kind != "jog":
                    self.say(f"Arrived. ({motion.label})")

    def _watch_page(self) -> None:
        while not self.closing:
            time.sleep(0.2)
            with self.lock:
                busy = self.active is not None
            if busy and self.clock() - self.heard_at > self.page_silent_s:
                try:
                    self.stop(f"the page went quiet for {self.page_silent_s:g} s")
                except ZArmError:
                    pass

    # The page's view of the setup

    def page_config(self, token: str, three: bool, three_missing: Optional[str], meshes: Mapping[str, bytes],
                    mesh_note: Optional[str]) -> Dict[str, Any]:
        model = self.model
        sdk_origin = model.joint("base_sdk_origin").xyz
        hard = {joint.sdk_field: [round(joint.lower / joint.sdk_scale, 6), round(joint.upper / joint.sdk_scale, 6)]
                for joint in model.sdk_joints()}
        return {
            "token": token,
            "simulated": True,
            "backend": type(self.arm.backend).__name__,
            "urdf": f"/model/{URDF_NAME}",
            "mjcf": f"/model/{MJCF_NAME}",
            "three": three,
            "threeVersion": THREE_VERSION,
            "threeMissing": three_missing,
            "meshes": {link: f"{MESH_URL}{link}.stl" for link in meshes} or None,
            "meshNote": mesh_note,
            "cadCommand": fetch_command(),
            "jointMap": [
                {"field": joint.sdk_field, "joint": joint.name, "scale": joint.sdk_scale,
                 "label": f"J{SDK_FIELDS.index(joint.sdk_field) + 1}", "unit": "mm" if joint.kind == "prismatic" else "deg"}
                for joint in model.sdk_joints()
            ],
            "softLimits": {field: list(self.limits.range_of(index + 1)) for index, field in enumerate(SDK_FIELDS)},
            "hardLimits": hard,
            "armLengthsMm": list(self.config.arm_lengths_mm),
            "sdkOriginM": list(sdk_origin),
            "floorZMm": round(-sdk_origin[2] * 1000.0, 3),
            "steps": JOINT_STEPS,
            "jointSpeeds": [s for s in JOINT_SPEEDS if s <= self.config.max_joint_speed],
            "linearSpeeds": [s for s in LINEAR_SPEEDS if s <= self.config.max_linear_speed_mm_s],
            "defaults": DEFAULTS,
            "maxLeadSteps": MAX_LEAD_STEPS,
            "pollMs": POLL_MS,
            "pageSilentS": self.page_silent_s,
            "envelope": "/api/envelope.json",
            # For the page's plan checks: the column and the J2 arm in plan view, as workspace.py checks them (mm).
            "blindZone": {
                "columnRectMm": list(self.workspace.column_rect_mm),
                "j2CirclesMm": [list(circle) for circle in self.workspace.j2_circles_mm],
                "j2BoxMm": list(self.workspace.j2_box_mm),
                "l1Mm": self.workspace.l1_mm,
                "l2Mm": self.workspace.l2_mm,
            },
        }

    def envelope(self) -> Dict[str, Any]:
        x0, x1, y0, y1 = self.workspace.bounds_mm()
        lines = [[list(p) for p in line[::3] + (line[-1],)] for line in envelope_outline(self.workspace)]
        return {"png": "/api/envelope.png", "boundsMm": [x0, x1, y0, y1], "outlineMm": lines,
                "j1RangeDeg": list(self.workspace.j1_range_deg), "j2RangeDeg": list(self.workspace.j2_range_deg)}


# ---- HTTP -------------------------------------------------------------------------------------------------------

class LoopbackServer(ThreadingHTTPServer):
    """ThreadingHTTPServer without the reverse DNS lookup HTTPServer.server_bind makes (as umbratek-code's).

    On Windows, SO_REUSEADDR would let a second server bind a port that's already listening (and the browser would
    keep talking to the first), so there the port is bound exclusively and a busy port is an error.
    """

    daemon_threads = True
    allow_reuse_address = os.name != "nt"

    def server_bind(self) -> None:
        if os.name == "nt" and hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        socketserver.TCPServer.server_bind(self)
        self.server_name, self.server_port = self.server_address[:2]


IMPORT_MAP = re.compile(r'<script type="importmap">(.*?)</script>', re.S)


def render_page(template: str, config: Mapping[str, Any]) -> Tuple[bytes, str]:
    """(page bytes, the import map's CSP hash source). The config goes into a JSON script element."""
    data = json.dumps(config, separators=(",", ":")).replace("<", "\\u003c")
    page = template.replace("__CONFIG_JSON__", data)
    found = IMPORT_MAP.search(page)
    if found is None:
        raise ValueError("the page template has no import map")
    digest = base64.b64encode(hashlib.sha256(found.group(1).encode("utf-8")).digest()).decode("ascii")
    return page.encode("utf-8"), f"'sha256-{digest}'"


def make_handler(sim: Simulator, token: str, page: bytes, script_hash: str, three: Mapping[str, bytes],
                 meshes: Mapping[str, bytes], models: Mapping[str, bytes]) -> type:
    csp = ("default-src 'none'; script-src 'self' " + script_hash + "; style-src 'self'; img-src 'self' data:; "
           "connect-src 'self'; frame-ancestors 'none'; base-uri 'none'; form-action 'none'")
    posts: Dict[str, Callable[[Mapping[str, Any]], Tuple[bool, str]]] = {
        "/api/jog": sim.jog,
        "/api/move_joints": sim.move_joints,
        "/api/move_pose": sim.move_pose,
        "/api/move_linear": sim.move_linear,
        "/api/change_hand": sim.change_hand,
        "/api/stop": lambda body: sim.stop(),
        "/api/estop": lambda body: sim.emergency_stop(),
        "/api/initialize": lambda body: sim.initialize(),
    }

    class Handler(BaseHTTPRequestHandler):
        server_version = "ZArmSimView/1"
        protocol_version = "HTTP/1.1"  # keep-alive: the page polls about 30 times a second
        timeout = 30

        def log_message(self, fmt: str, *args: object) -> None:
            pass

        def _send(self, code: int, payload: Any, kind: str = "application/json") -> None:
            data = payload if isinstance(payload, bytes) else json.dumps(payload).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            if code >= 400:  # the request body may be unread, so don't reuse the connection
                self.send_header("Connection", "close")
                self.close_connection = True
            if kind.startswith("text/html"):
                self.send_header("Content-Security-Policy", csp)
                self.send_header("X-Frame-Options", "DENY")
                self.send_header("Referrer-Policy", "no-referrer")
            self.end_headers()
            self.wfile.write(data)

        def _host_ok(self) -> bool:
            port = self.server.server_address[1]
            return self.headers.get("Host") in (f"127.0.0.1:{port}", f"localhost:{port}")

        def do_GET(self) -> None:  # noqa: N802 (http.server's naming)
            if not self._host_ok():
                port = self.server.server_address[1]
                return self._send(403, {"ok": False, "message": f"Open the view at http://127.0.0.1:{port}"})
            path = urllib.parse.urlsplit(self.path).path
            if path == "/":
                return self._send(200, page, "text/html; charset=utf-8")
            if path == "/api/status":
                return self._send(200, sim.status())
            if path == "/api/envelope.json":
                return self._send(200, sim.envelope())
            if path == "/api/envelope.png":
                return self._send(200, envelope_png(sim.workspace), "image/png")
            if path.startswith("/model/") and path[len("/model/"):] in models:
                return self._send(200, models[path[len("/model/"):]], "application/xml; charset=utf-8")
            if path.startswith("/web/") and path[len("/web/"):] in STATIC_FILES:
                name = path[len("/web/"):]
                return self._send(200, (WEB_DIR / name).read_bytes(), STATIC_TYPES[Path(name).suffix])
            if path.startswith(THREE_URL) and path.endswith(".js"):  # only the checked copy, and only its scripts
                data = three.get(path[len(THREE_URL):])
                if data is not None:
                    return self._send(200, data, STATIC_TYPES[".js"])
            if path.startswith(MESH_URL) and path.endswith(".stl"):
                data = meshes.get(path[len(MESH_URL):-len(".stl")])
                if data is not None:
                    return self._send(200, data, "model/stl")
            return self._send(404, {"ok": False, "message": "Not found."})

        def do_POST(self) -> None:  # noqa: N802
            if not self._host_ok() or not secrets.compare_digest(self.headers.get("X-Panel-Token", ""), token):
                return self._send(403, {"ok": False, "message": "Forbidden."})
            try:
                length = int(self.headers.get("Content-Length") or 0)
            except ValueError:
                length = -1
            if not 0 <= length <= MAX_BODY_BYTES:
                return self._send(400, {"ok": False, "message": "Bad request."})
            try:
                body = json.loads(self.rfile.read(length) or b"{}")
            except ValueError:
                return self._send(400, {"ok": False, "message": "Bad request: the body isn't JSON."})
            if not isinstance(body, dict):
                return self._send(400, {"ok": False, "message": "Bad request: the body must be a JSON object."})
            action = posts.get(urllib.parse.urlsplit(self.path).path)
            if action is None:
                return self._send(404, {"ok": False, "message": "Not found."})
            sim.heard()
            try:
                moved, message = action(body)
            except (PanelError, ZArmError) as error:
                text = str(error) if isinstance(error, PanelError) else describe_error(error)
                return self._send(200, {"ok": False, "message": sim.say(text, "error"), "error": type(error).__name__})
            except Exception as error:  # a bug: say so instead of dropping the connection
                print(f"Unexpected error in {self.path}: {describe_error(error)}", file=sys.stderr)
                return self._send(500, {"ok": False, "message": sim.say(describe_error(error), "error")})
            return self._send(200, {"ok": True, "moved": moved, "message": message})

    return Handler


def make_server(
    sim: Simulator,
    *,
    port: int = DEFAULT_PORT,
    three: Optional[Mapping[str, bytes]] = None,
    three_missing: Optional[str] = None,
    meshes: Optional[Mapping[str, bytes]] = None,
    mesh_note: Optional[str] = None,
    token: Optional[str] = None,
) -> LoopbackServer:
    """An HTTP server for ``sim`` on 127.0.0.1:``port`` (0 picks a free port). Call serve_forever() on it."""
    three = dict(three or {})
    meshes = dict(meshes or {})
    token = token or secrets.token_urlsafe(24)
    models = {name: (MODELS_DIR / name).read_bytes() for name in (URDF_NAME, MJCF_NAME)}
    config = sim.page_config(token, bool(three), three_missing, meshes, mesh_note)
    page, script_hash = render_page(PAGE.read_text(encoding="utf-8"), config)
    handler = make_handler(sim, token, page, script_hash, three, meshes, models)
    return LoopbackServer(("127.0.0.1", port), handler)


def open_simulator(page_silent_s: float = PAGE_SILENT_S) -> Simulator:
    """A started Simulator over ``ZArm.simulated`` with the Z-Arm 2442's link lengths."""
    model = build_model()
    urdf = UrdfRobot(MODELS_DIR / URDF_NAME)
    arm = ZArm.simulated(ZArmConfig(arm_lengths_mm=ARM_LENGTHS_MM))
    sim = Simulator(arm, model, urdf, page_silent_s=page_silent_s)
    sim.start()
    return sim


def main(argv: Optional[List[str]] = None) -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(line_buffering=True)  # type: ignore[union-attr]
    parser = argparse.ArgumentParser(description="Serve the Z-Arm simulator view at http://127.0.0.1:PORT.")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"local port (default {DEFAULT_PORT})")
    parser.add_argument("--open", action="store_true", help="open the view in the default browser")
    args = parser.parse_args(argv)
    if not 1024 <= args.port <= 65535:
        parser.error("--port must be between 1024 and 65535")
    for name, why in stale_files().items():
        print(f"Warning: models/{name} {why}; run python -m zarm_sim.generate", file=sys.stderr)
    three, missing = load_three()
    if missing:
        print(f"Warning: couldn't download three.js {THREE_VERSION}, which draws the 3D view ({missing}). The page "
              "works without it; to get it, check this PC can reach the internet and start the server again.",
              file=sys.stderr)
    meshes, mesh_note = load_meshes((MODELS_DIR / URDF_NAME).read_bytes())
    if meshes:
        print("Drawing the arm with Hitbot's CAD meshes, from cache/meshes (made on this PC from its STEP).")
    else:
        why = mesh_note or "the CAD meshes haven't been made on this PC"
        print(f"Note: {why}, so the view draws the arm's simple shapes. For Hitbot's CAD shapes, run this, then "
              f"restart the server:\n  {fetch_command()}", file=sys.stderr)
    sim = open_simulator()
    try:
        server = make_server(sim, port=args.port, three=three, three_missing=missing, meshes=meshes,
                             mesh_note=mesh_note)
    except OSError as error:
        sim.close()
        print(f"Error: cannot listen on 127.0.0.1:{args.port} ({error}). Try --port.", file=sys.stderr)
        return 1
    threading.Thread(target=envelope_png, args=(sim.workspace,), daemon=True).start()  # warm the cache
    url = f"http://127.0.0.1:{server.server_address[1]}"
    for warning in sim.warnings:
        print(f"Warning: {warning}", file=sys.stderr)
    print(f"Z-Arm simulator view: open {url}  (SIMULATED: zarm's FakeBackend, no hardware; Ctrl+C to quit)")
    if args.open:
        webbrowser.open(url)
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        sim.close()
    print("Simulator view closed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

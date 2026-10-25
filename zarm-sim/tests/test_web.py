"""The browser files: JavaScript syntax and plan checks (when node is available), and the page's contract with the
server and its scripts."""

from __future__ import annotations

import json
import random
import re
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

from tests.support import NODE, zarm
from zarm_sim.model import build_model
from zarm_sim.server import PAGE, STATIC_FILES, WEB_DIR, render_page
from zarm_sim.three_js import THREE_FILES, THREE_URL
from zarm_sim.workspace import Workspace

# Runs plan.js's checks on the JSON it reads from stdin: {zone, joints: [[j1, j2]...], paths: [[from, to]...]}.
PLAN_DRIVER = """
import {readFileSync} from "node:fs";
import {j2ClearanceMm, pathCheck} from "./plan.mjs";
const input = JSON.parse(readFileSync(0, "utf8"));
process.stdout.write(JSON.stringify({
  clearance: input.joints.map(([j1, j2]) => j2ClearanceMm(input.zone, j1, j2)),
  paths: input.paths.map(([from, to]) => pathCheck(input.zone, from, to)),
}));
"""


def joints(j1, j2):
    return {"j1_deg": j1, "j2_deg": j2, "j3_mm": 0.0, "j4_deg": 0.0}


# Runs kinematics.js on the JSON it reads from stdin and prints what it found.
KINEMATICS_DRIVER = """
import {readFileSync} from "node:fs";
import {elbowOf, forwardXy, inverseXy, solveTool, toolPath, toolPose} from "./kinematics.mjs";
const input = JSON.parse(readFileSync(0, "utf8"));
const [l1, l2] = input.lengths;
process.stdout.write(JSON.stringify({
  forward: input.forward.map(([j1, j2]) => forwardXy(j1, j2, l1, l2)),
  inverse: input.inverse.map(([x, y, elbow]) => inverseXy(x, y, l1, l2, elbow)),
  elbow: input.elbow.map(elbowOf),
  pose: input.pose.map((j) => toolPose(j, input.lengths)),
  solve: input.solve.map(([base, axis, amount]) => solveTool(base, axis, amount, input.lengths, input.limits)),
  path: input.path.map(([from, to]) => toolPath(from, to, input.lengths)),
}));
"""
FIELDS = ("j1_deg", "j2_deg", "j3_mm", "j4_deg")


class WebFilesTests(unittest.TestCase):
    @unittest.skipUnless(NODE, "node isn't installed")
    def test_javascript_parses(self):
        with tempfile.TemporaryDirectory() as folder:
            for script in sorted(WEB_DIR.glob("*.js")):
                with self.subTest(script=script.name):
                    module = Path(folder) / (script.stem + ".mjs")  # parse as an ES module, as the browser does
                    shutil.copyfile(script, module)
                    result = subprocess.run([NODE, "--check", str(module)], capture_output=True, encoding="utf-8")
                    self.assertEqual(result.returncode, 0, result.stderr)

    def test_every_web_file_is_served_and_nothing_else(self):
        on_disk = {path.name for path in WEB_DIR.iterdir() if path.suffix in (".js", ".css")}
        self.assertEqual(on_disk, STATIC_FILES)

    def test_import_map_points_at_the_checked_three_js_files(self):
        template = PAGE.read_text(encoding="utf-8")
        found = re.search(r'<script type="importmap">(.*?)</script>', template, re.S)
        imports = json.loads(found.group(1))["imports"]
        self.assertEqual(imports["three"], THREE_URL + "build/three.module.js")
        self.assertIn("build/three.module.js", THREE_FILES)
        addons = imports["three/addons/"]
        for path in ("examples/jsm/controls/OrbitControls.js", "examples/jsm/loaders/STLLoader.js"):
            self.assertIn(path, THREE_FILES)
            self.assertTrue((THREE_URL + path).startswith(addons))

    def test_config_is_embedded_as_inert_json(self):
        page, script_hash = render_page(PAGE.read_text(encoding="utf-8"), {"token": "</script><script>alert(1)</script>"})
        text = page.decode("utf-8")
        self.assertNotIn("<script>alert(1)", text)
        config = re.search(r'<script type="application/json" id="zarm-config">(.*?)</script>', text, re.S).group(1)
        self.assertEqual(json.loads(config)["token"], "</script><script>alert(1)</script>")
        self.assertTrue(script_hash.startswith("'sha256-"))

    def test_page_always_says_simulated(self):
        template = PAGE.read_text(encoding="utf-8")
        self.assertIn('<p class="placard" role="note"><strong>SIMULATED</strong>', template)
        self.assertIn('class="watermark"', template)
        self.assertIn("<title>Z-Arm simulator · SIMULATED</title>", template)

    def test_cad_is_the_view_with_no_toggle(self):
        template = PAGE.read_text(encoding="utf-8")
        self.assertNotIn("data-mode", template)
        self.assertNotIn("Primitives", template)
        self.assertIn("Drag a ring or the Z handle to plan a pose, then press Move.", template)
        for name in ("cadNote", "cadCommand", "cadCopy", "plan", "planJoints", "planNote", "planClear", "planMove",
                     "dragTag"):
            self.assertIn(f'id="{name}"', template)

    def test_joints_and_tool_keys(self):
        template = PAGE.read_text(encoding="utf-8")
        keys = re.search(r'<div class="mode-keys" role="group" aria-label="Dragging in the 3D view moves">(.*?)</div>',
                         template, re.S)
        self.assertIsNotNone(keys)
        self.assertEqual(re.findall(r'id="(mode\w+)"', keys.group(1)), ["modeJoints", "modeTool"])
        self.assertIn('aria-pressed="true" disabled>Joints</button>', keys.group(1))  # until the 3D view is up
        for name in ("aimX", "aimY", "aimZ", "aimR", "aimYaw"):
            self.assertIn(f'id="{name}"', template)

    def test_every_element_the_page_script_looks_up_is_on_the_page(self):
        template = PAGE.read_text(encoding="utf-8")
        ids = set(re.findall(r'\bid="([^"]+)"', template))
        script = (WEB_DIR / "app.js").read_text(encoding="utf-8")
        wanted = set(re.findall(r'\$\("([^"]+)"\)', script)) | set(re.findall(r'getElementById\("([^"]+)"\)', script))
        self.assertGreater(len(wanted), 20)
        self.assertEqual(wanted - ids, set())


class PlanCheckTests(unittest.TestCase):
    """plan.js, which flags plans whose path runs the J2 arm into the column, against workspace.py."""

    @classmethod
    def setUpClass(cls):
        cls.ws = Workspace.from_model(build_model(), (-90.0, 90.0), (-164.0, 164.0))
        ws = cls.ws
        cls.zone = {"columnRectMm": list(ws.column_rect_mm), "j2CirclesMm": [list(c) for c in ws.j2_circles_mm],
                    "j2BoxMm": list(ws.j2_box_mm), "l1Mm": ws.l1_mm, "l2Mm": ws.l2_mm}

    def run_plan_js(self, pairs, paths):
        with tempfile.TemporaryDirectory() as folder:
            shutil.copyfile(WEB_DIR / "plan.js", Path(folder) / "plan.mjs")
            (Path(folder) / "driver.mjs").write_text(PLAN_DRIVER, encoding="utf-8")
            data = json.dumps({"zone": self.zone, "joints": pairs, "paths": paths})
            result = subprocess.run([NODE, "driver.mjs"], cwd=folder, input=data, capture_output=True, encoding="utf-8")
        self.assertEqual(result.returncode, 0, result.stderr)
        return json.loads(result.stdout)

    def boundary_at_j1_90(self):
        """J2 values either side of where, with J1 at 90, the J2 arm meets the column: (clear, touching)."""
        low, high = 90.0, 164.0
        self.assertGreater(self.ws.j2_clearance_mm(90.0, low), 0.0)
        self.assertLessEqual(self.ws.j2_clearance_mm(90.0, high), 0.0)
        while high - low > 1e-6:
            middle = (low + high) / 2
            if self.ws.j2_clearance_mm(90.0, middle) > 0.0:
                low = middle
            else:
                high = middle
        return low, high

    @unittest.skipUnless(NODE, "node isn't installed")
    def test_clearance_matches_workspace_py(self):
        rng = random.Random(11)
        pairs = [[rng.uniform(-90, 90), rng.uniform(-164, 164)] for _ in range(600)]
        pairs += [[90.0, 164.0], [-90.0, -164.0], [90.0, 90.0], [0.0, 0.0], [90.0, 120.0], [-90.0, -125.0]]
        found = self.run_plan_js(pairs, [])["clearance"]
        for (j1, j2), js in zip(pairs, found):
            with self.subTest(j1=j1, j2=j2):
                self.assertAlmostEqual(js, self.ws.j2_clearance_mm(j1, j2), delta=1e-9)

    @unittest.skipUnless(NODE, "node isn't installed")
    def test_paths(self):
        clear, touching = self.boundary_at_j1_90()
        gap = self.ws.j2_clearance_mm
        # With J1 at 90, folding J2 back from 164 first sweeps further into the column (the gap bottoms out near 145).
        self.assertTrue(gap(90, 164) < 0 and gap(90, 145) < gap(90, 164) - 0.5 and gap(90, 125) > gap(90, 164))
        self.assertTrue(gap(90, 110) > 0 and gap(40, 164) > 0)
        self.assertLess(min(gap(90 - 50 * k / 100, 110 + 54 * k / 100) for k in range(101)), 0)  # dips in on the way
        cases = [
            ("a move well clear", joints(0, 0), joints(30, 20), None),
            ("into the column on the way", joints(90, 0), joints(90, 164), "on the way"),
            ("ending just inside the column", joints(90, clear), joints(90, touching), "end up in the column"),
            ("through the column and out", joints(90, 110), joints(40, 164), "on the way"),
            ("out of the column, through deeper", joints(90, 164), joints(90, 0), None),
            ("less far in", joints(90, 164), joints(90, 125), None),
            ("further into the column", joints(90, 125), joints(90, 145), "further into the column"),
            ("staying put inside it", joints(90, 164), joints(90, 164), None),
        ]
        found = self.run_plan_js([], [[start, end] for _, start, end, _ in cases])["paths"]
        for (name, _, _, reason), result in zip(cases, found):
            with self.subTest(name):
                if reason is None:
                    self.assertEqual(result, {"clear": True, "reason": None})
                else:
                    self.assertFalse(result["clear"])
                    self.assertIn(reason, result["reason"])
                    self.assertIn("Hitbot's blind zone", result["reason"])


@unittest.skipUnless(NODE, "node isn't installed")
class KinematicsTests(unittest.TestCase):
    """kinematics.js, the tool gizmo's inverse kinematics, against zarm-sdk's zarm.kinematics."""

    LENGTHS = (220.0, 200.0)
    LIMITS = {"j1_deg": [-90.0, 90.0], "j2_deg": [-164.0, 164.0], "j3_mm": [-310.0, 0.0], "j4_deg": [-180.0, 180.0]}

    def run_js(self, **inputs):
        data = {"lengths": list(self.LENGTHS), "limits": self.LIMITS, "forward": [], "inverse": [], "elbow": [],
                "pose": [], "solve": [], "path": [], **inputs}
        with tempfile.TemporaryDirectory() as folder:
            shutil.copyfile(WEB_DIR / "kinematics.js", Path(folder) / "kinematics.mjs")
            (Path(folder) / "driver.mjs").write_text(KINEMATICS_DRIVER, encoding="utf-8")
            result = subprocess.run([NODE, "driver.mjs"], cwd=folder, input=json.dumps(data), capture_output=True,
                                    encoding="utf-8")
        self.assertEqual(result.returncode, 0, result.stderr)
        return json.loads(result.stdout)

    def expected_solve(self, base, axis, amount):
        """What the gizmo should do, from the SDK's kinematics: ("ok", joints), ("reach", None) or ("limit", field)."""
        k = zarm.kinematics
        if axis == "z":
            found = {**base, "j3_mm": base["j3_mm"] + amount}
        elif axis == "yaw":
            found = {**base, "j4_deg": base["j4_deg"] + amount}
        else:
            x, y = k.forward_xy(base["j1_deg"], base["j2_deg"], *self.LENGTHS)
            x, y = (x + amount, y) if axis == "x" else (x, y + amount)
            solution = (base["j1_deg"], base["j2_deg"]) if amount == 0 else k.inverse_xy(x, y, *self.LENGTHS,
                                                                                         k.elbow_of(base["j2_deg"]))
            if solution is None:
                return "reach", None
            j1, j2 = solution
            found = {"j1_deg": j1, "j2_deg": j2, "j3_mm": base["j3_mm"],
                     "j4_deg": base["j4_deg"] + (base["j1_deg"] + base["j2_deg"]) - (j1 + j2)}
        for field in FIELDS:
            low, high = self.LIMITS[field]
            if not low <= found[field] <= high:
                return "limit", field
        return "ok", found

    def test_inverse_and_forward_match_the_sdk(self):
        rng = random.Random(5)
        points = [[rng.uniform(-460, 460), rng.uniform(-460, 460), rng.choice((1, -1))] for _ in range(800)]
        points += [[420.0, 0.0, 1], [420.0, 0.0, -1], [0.0, 420.0, 1], [20.0, 0.0, 1], [-300.0, -1e-12, 1],
                   [419.999999, 0.0, -1], [0.0, 0.0, 1]]
        pairs = [[rng.uniform(-180, 180), rng.uniform(-180, 180)] for _ in range(300)]
        found = self.run_js(inverse=points, forward=pairs, elbow=[0.0, -0.0, 1e-9, -1e-9, 5.0, -5.0])
        for (x, y, elbow), js in zip(points, found["inverse"]):
            expected = zarm.kinematics.inverse_xy(x, y, *self.LENGTHS, elbow)
            with self.subTest(x=x, y=y, elbow=elbow):
                if expected is None:
                    self.assertIsNone(js)
                else:
                    self.assertIsNotNone(js)
                    self.assertAlmostEqual(js[0], expected[0], delta=1e-9)
                    self.assertAlmostEqual(js[1], expected[1], delta=1e-9)
        for (j1, j2), (x, y) in zip(pairs, found["forward"]):
            ex, ey = zarm.kinematics.forward_xy(j1, j2, *self.LENGTHS)
            self.assertAlmostEqual(x, ex, delta=1e-9)
            self.assertAlmostEqual(y, ey, delta=1e-9)
        self.assertEqual(found["elbow"], [zarm.kinematics.elbow_of(j2) for j2 in (0.0, -0.0, 1e-9, -1e-9, 5.0, -5.0)])

    def test_tool_pose_is_the_sdks(self):
        rng = random.Random(6)
        poses = [dict(zip(FIELDS, (rng.uniform(-90, 90), rng.uniform(-164, 164), rng.uniform(-310, 0),
                                   rng.uniform(-180, 180)))) for _ in range(100)]
        for j, pose in zip(poses, self.run_js(pose=poses)["pose"]):
            x, y = zarm.kinematics.forward_xy(j["j1_deg"], j["j2_deg"], *self.LENGTHS)
            self.assertAlmostEqual(pose["x_mm"], x, delta=1e-9)
            self.assertAlmostEqual(pose["y_mm"], y, delta=1e-9)
            self.assertEqual((pose["z_mm"], pose["r_deg"]), (j["j3_mm"], j["j4_deg"]))
            self.assertAlmostEqual(pose["yaw_deg"], zarm.kinematics.tool_yaw_deg(j["j1_deg"], j["j2_deg"], j["j4_deg"]),
                                   delta=1e-9)

    def test_the_gizmo_solves_as_the_sdk_would(self):
        rng = random.Random(7)
        cases = []
        for _ in range(400):
            base = dict(zip(FIELDS, (rng.uniform(-85, 85), rng.choice((-1, 1)) * rng.uniform(5, 160),
                                     rng.uniform(-305, -5), rng.uniform(-175, 175))))
            axis = rng.choice(("x", "y", "z", "yaw"))
            amount = rng.choice((1, -1)) * rng.choice((1, 10, 37, 150, 400)) * (0.5 if axis == "yaw" else 1)
            cases.append([base, axis, amount])
        found = self.run_js(solve=cases)["solve"]
        outcomes = set()
        for (base, axis, amount), js in zip(cases, found):
            kind, expected = self.expected_solve(base, axis, amount)
            if kind == "ok" and any(min(abs(expected[f] - lim) for lim in self.LIMITS[f]) < 1e-6 for f in FIELDS):
                continue  # on a limit to within rounding: either answer is right
            outcomes.add((axis, kind))
            with self.subTest(base=base, axis=axis, amount=amount):
                if kind == "ok":
                    self.assertTrue(js["ok"], js)
                    for field in FIELDS:
                        self.assertAlmostEqual(js["joints"][field], expected[field], delta=1e-9)
                    pose = zarm.kinematics.forward_xy(js["joints"]["j1_deg"], js["joints"]["j2_deg"], *self.LENGTHS)
                    start = zarm.kinematics.forward_xy(base["j1_deg"], base["j2_deg"], *self.LENGTHS)
                    moved = {"x": (amount, 0.0), "y": (0.0, amount)}.get(axis, (0.0, 0.0))
                    self.assertAlmostEqual(pose[0], start[0] + moved[0], delta=1e-6)  # the tool went where it was dragged
                    self.assertAlmostEqual(pose[1], start[1] + moved[1], delta=1e-6)
                    yaw = lambda j: zarm.kinematics.tool_yaw_deg(j["j1_deg"], j["j2_deg"], j["j4_deg"])  # noqa: E731
                    self.assertAlmostEqual(yaw(js["joints"]), yaw(base) + (amount if axis == "yaw" else 0.0), delta=1e-9)
                    self.assertEqual(zarm.kinematics.elbow_of(js["joints"]["j2_deg"]), zarm.kinematics.elbow_of(base["j2_deg"]))
                elif kind == "reach":
                    self.assertEqual((js["ok"], js["field"]), (False, None))
                    self.assertTrue(js["reason"].startswith("Out of reach"), js["reason"])
                else:
                    self.assertEqual((js["ok"], js["field"]), (False, expected))
                    self.assertIn(f"J{FIELDS.index(expected) + 1} is at its limit", js["reason"])
        for outcome in [(axis, "ok") for axis in ("x", "y", "z", "yaw")] + [("x", "reach"), ("z", "limit"), ("yaw", "limit")]:
            self.assertIn(outcome, outcomes)  # the random cases cover each way it can go

    def test_named_cases(self):
        home = dict(zip(FIELDS, (0.0, 0.0, 0.0, 0.0)))
        cases = [
            ("home: out along X", [home, "x", 1], (False, None, "Out of reach (420 mm from the J1 axis at most)")),
            ("home: sideways", [home, "y", 1], (False, None, "Out of reach")),
            ("home: up past the top of travel", [home, "z", 1], (False, "j3_mm", "J3 is at its limit (\u2212310 to 0 mm)")),
            ("home: yaw past J4's limit", [home, "yaw", 180.5], (False, "j4_deg", "J4 is at its limit (\u2212180 to 180\u00b0)")),
            ("J4 near its limit, sliding", [dict(zip(FIELDS, (0.0, 90.0, -100.0, 179.0))), "x", 50],
             (False, "j4_deg", "J4 is at its limit (\u2212180 to 180\u00b0) holding the tool's yaw")),
            ("J1 near its limit", [dict(zip(FIELDS, (89.0, 60.0, -100.0, 0.0))), "x", -60], (False, "j1_deg", "J1 is at its limit")),
            ("home: in along X", [home, "x", -10], (True, None, None)),
        ]
        found = self.run_js(solve=[case for _, case, _ in cases])["solve"]
        for (name, (base, axis, amount), (ok, field, reason)), js in zip(cases, found):
            with self.subTest(name):
                self.assertEqual(js["ok"], ok, js)
                if ok:
                    self.assertGreater(js["joints"]["j2_deg"], 0.0)  # straightened at home, it folds on the J2 >= 0 side
                    self.assertAlmostEqual(js["joints"]["j1_deg"] + js["joints"]["j2_deg"] + js["joints"]["j4_deg"], 0.0,
                                           delta=1e-9)  # holding the tool's yaw
                else:
                    self.assertEqual(js["field"], field)
                    self.assertIn(reason, js["reason"])

    def test_the_tool_path_samples_the_joint_move(self):
        start = dict(zip(FIELDS, (-60.0, 100.0, -20.0, 0.0)))
        end = dict(zip(FIELDS, (45.0, 30.0, -250.0, 90.0)))
        same = dict(zip(FIELDS, (10.0, 20.0, -30.0, 40.0)))
        path, still = self.run_js(path=[[start, end], [same, same]])["path"]
        self.assertEqual(len(path), 116)  # every 1 deg of the larger J1/J2 turn (105) or 2 mm of J3 (115): 115 steps
        for k in (0, 40, 115):
            t = k / 115
            j1, j2 = start["j1_deg"] + (end["j1_deg"] - start["j1_deg"]) * t, start["j2_deg"] + (end["j2_deg"] - start["j2_deg"]) * t
            x, y = zarm.kinematics.forward_xy(j1, j2, *self.LENGTHS)
            self.assertAlmostEqual(path[k][0], x, delta=1e-9)
            self.assertAlmostEqual(path[k][1], y, delta=1e-9)
            self.assertAlmostEqual(path[k][2], start["j3_mm"] + (end["j3_mm"] - start["j3_mm"]) * t, delta=1e-9)
        straight = [path[0][i] + (path[-1][i] - path[0][i]) * 0.5 for i in range(2)]
        self.assertGreater(abs(path[58][0] - straight[0]) + abs(path[58][1] - straight[1]), 20.0)  # a curve, not a line
        self.assertEqual(len(still), 2)  # no move: the start and the end


if __name__ == "__main__":
    unittest.main()

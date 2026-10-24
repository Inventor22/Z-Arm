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

from tests.support import NODE
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


class WebFilesTests(unittest.TestCase):
    @unittest.skipUnless(NODE, "node isn't installed")
    def test_javascript_parses(self):
        with tempfile.TemporaryDirectory() as folder:
            for script in sorted(WEB_DIR.glob("*.js")):
                with self.subTest(script=script.name):
                    module = Path(folder) / (script.stem + ".mjs")  # parse as an ES module, as the browser does
                    shutil.copyfile(script, module)
                    result = subprocess.run([NODE, "--check", str(module)], capture_output=True, text=True)
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
            result = subprocess.run([NODE, "driver.mjs"], cwd=folder, input=data, capture_output=True, text=True)
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


if __name__ == "__main__":
    unittest.main()

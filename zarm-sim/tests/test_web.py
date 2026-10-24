"""The browser files: JavaScript syntax (when node is available) and the page template's contract with the server."""

from __future__ import annotations

import json
import re
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

from tests.support import NODE
from zarm_sim.server import PAGE, STATIC_FILES, WEB_DIR, render_page
from zarm_sim.three_js import THREE_FILES, THREE_URL


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


if __name__ == "__main__":
    unittest.main()

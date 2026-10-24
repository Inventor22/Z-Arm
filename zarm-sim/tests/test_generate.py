"""The generated URDF and MJCF: committed, deterministic, and every dimension accounted for."""

from __future__ import annotations

import contextlib
import hashlib
import io
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from tests.support import SIM_ROOT
from zarm_sim.dimensions import ZArmDimensions, dimension_table
from zarm_sim.generate import MJCF_NAME, MODELS_DIR, URDF_NAME, main, render_all, stale_files

DIGEST_SCRIPT = (
    "import hashlib\n"
    "from zarm_sim.generate import render_all\n"
    "for name, data in sorted(render_all().items()):\n"
    "    print(name, hashlib.sha256(data).hexdigest())\n"
)


class GeneratedFilesTests(unittest.TestCase):
    def test_committed_models_are_byte_identical_to_the_generator(self):
        for name, data in render_all().items():
            with self.subTest(file=name):
                committed = (MODELS_DIR / name).read_bytes()
                self.assertEqual(committed, data, f"models/{name} is stale: run python -m zarm_sim.generate")
        self.assertEqual(stale_files(), {})

    def test_generation_is_deterministic_within_and_across_interpreters(self):
        self.assertEqual(render_all(), render_all())
        digests = set()
        for seed in ("1", "2"):
            env = dict(os.environ, PYTHONHASHSEED=seed)
            result = subprocess.run([sys.executable, "-c", DIGEST_SCRIPT], cwd=SIM_ROOT, env=env, capture_output=True,
                                    text=True, check=True)
            digests.add(result.stdout)
        self.assertEqual(len(digests), 1)
        expected = "".join(f"{name} {hashlib.sha256(data).hexdigest()}\n" for name, data in sorted(render_all().items()))
        self.assertEqual(digests.pop().replace("\r\n", "\n"), expected)

    def test_files_are_utf8_with_lf_line_endings(self):
        for name, data in render_all().items():
            with self.subTest(file=name):
                self.assertNotIn(b"\r", data)
                data.decode("utf-8")
                self.assertTrue(data.endswith(b"\n"))

    def test_check_mode_reports_stale_files(self):
        with tempfile.TemporaryDirectory() as folder, contextlib.redirect_stdout(io.StringIO()), \
                contextlib.redirect_stderr(io.StringIO()):
            out = Path(folder)
            self.assertEqual(main(["--out", str(out)]), 0)
            self.assertEqual(main(["--check", "--out", str(out)]), 0)
            (out / URDF_NAME).write_bytes((out / URDF_NAME).read_bytes() + b"<!-- edited -->\n")
            (out / MJCF_NAME).unlink()
            self.assertEqual(main(["--check", "--out", str(out)]), 1)
            self.assertEqual(set(stale_files(out)), {URDF_NAME, MJCF_NAME})

    def test_gitattributes_keeps_models_lf(self):
        attributes = (SIM_ROOT / ".gitattributes").read_text(encoding="utf-8")
        self.assertIn("models/* text eol=lf", attributes)


class DimensionTests(unittest.TestCase):
    def test_every_dimension_has_a_source(self):
        table = dimension_table()
        self.assertGreater(len(table), 30)
        for name, value, source in table:
            with self.subTest(dimension=name):
                self.assertGreater(value, 0)
                self.assertGreater(len(source.strip()), 10)
                self.assertTrue(any(tag in source for tag in ("SPEC", "DRAWING", "STEP", "HITBOT_SIM", "LEGACY", "ASSUMED")))

    def test_readme_documents_every_dimension(self):
        readme = (SIM_ROOT / "README.md").read_text(encoding="utf-8")
        for name, _, _ in dimension_table():
            with self.subTest(dimension=name):
                self.assertIn(f"`{name}`", readme)

    def test_derived_heights(self):
        d = ZArmDimensions()
        self.assertEqual(d.overall_height_mm, 696.0)  # 596 for the 210 mm stroke, plus 100 mm
        self.assertEqual(d.flange_face_at_top_mm, 374.0)  # 64 at the bottom of a 310 mm stroke
        self.assertAlmostEqual(d.base_mass_kg, 8.6)
        stack = d.stack_mm()
        self.assertEqual(stack["housing"], (133.9, 195.9))  # 62 mm tall, as the drawing shows
        self.assertAlmostEqual(stack["housing"][1] - stack["housing"][0], 62.0)

    def test_validation_rejects_impossible_dimensions(self):
        with self.assertRaises(ValueError):
            ZArmDimensions(j1_axis_offset_mm=100.0).validate()  # the J1 arm's R53 end would cut into the column
        with self.assertRaises(ValueError):
            ZArmDimensions(housing_top_above_flange_mm=120.0).validate()
        with self.assertRaises(ValueError):
            ZArmDimensions(z_stroke_mm=-1.0).validate()


if __name__ == "__main__":
    unittest.main()

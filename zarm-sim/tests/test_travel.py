"""The Z travel: the J1 housing sweeps the column's slot from 26.9 mm above its bottom to 8.1 mm under the cap, on
Hitbot's column as drawn, in the primitive model and in the CAD meshes; and a stroke the slot can't hold grows the
column by exactly the shortfall."""

from __future__ import annotations

import json
import unittest
import xml.etree.ElementTree as ET

from zarm_sim.dimensions import ZArmDimensions
from zarm_sim.generate import MODELS_DIR, URDF_NAME
from zarm_sim.meshes import (MANIFEST, MESH_DIR, SLOT_PROBE_MM, column_lift, load_meshes, stl_triangles,
                             vertical_hits)
from zarm_sim.model import Box, build_model

MM = 1000.0
STROKES = (110.0, 210.0, 240.0, 310.0, 410.0, 510.0)  # SPEC's options, and the drawn 240


def numbers(text):
    return [float(value) for value in text.split()]


def housing_sweep_mm(dims):
    """(bottom, top) in mm the model's J1 housing sweeps over J3's range."""
    model = build_model(dims)
    j3 = model.joint("j3_vertical")
    housing = next(part for part in model.link("carriage").parts if part.name == "housing")
    assert isinstance(housing.shape, Box)
    half = housing.shape.size[2] / 2.0
    return (j3.xyz[2] + j3.lower + housing.xyz[2] - half) * MM, (j3.xyz[2] + j3.upper + housing.xyz[2] + half) * MM


def cap_mm(dims):
    """(underside, top) of the model's column cap in mm."""
    cap = next(part for part in build_model(dims).link("base_link").parts if part.name == "column_cap")
    assert isinstance(cap.shape, Box)
    half = cap.shape.size[2] / 2.0
    return (cap.xyz[2] - half) * MM, (cap.xyz[2] + half) * MM


class PrimitiveTravelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.d = ZArmDimensions()
        cls.urdf = ET.parse(MODELS_DIR / URDF_NAME).getroot()

    def visual_span(self, link, name):
        """(bottom, top) in m of a box visual in the generated URDF, in its link's frame."""
        visual = self.urdf.find(f"link[@name='{link}']/visual[@name='{name}']")
        z = numbers(visual.find("origin").get("xyz"))[2]
        height = numbers(visual.find("geometry/box").get("size"))[2]
        return z - height / 2.0, z + height / 2.0

    def test_the_housing_sweeps_the_travel_zone(self):
        joint = self.urdf.find("joint[@name='j3_vertical']")
        origin = numbers(joint.find("origin").get("xyz"))[2]
        limit = joint.find("limit")
        lower, upper = float(limit.get("lower")), float(limit.get("upper"))
        bottom, top = self.visual_span("carriage", "housing")
        swept = ((origin + lower + bottom) * MM, (origin + upper + top) * MM)
        for got, want in zip(swept, self.d.travel_zone_mm()):
            self.assertAlmostEqual(got, want, delta=1e-6)
        self.assertAlmostEqual((upper - lower) * MM, 310.0, delta=1e-9)
        self.assertAlmostEqual(origin * MM, 374.0, delta=1e-9)  # J3 = 0 puts the flange face 374 mm up

    def test_the_310_mm_stroke_fits_the_stock_column(self):
        _, cap_top = self.visual_span("base_link", "column_cap")
        _, column_top = self.visual_span("base_link", "column")  # where the cap closes the slots
        self.assertAlmostEqual(cap_top * MM, 593.0, delta=1e-6)  # 596 with the drawing's lid, which isn't modelled
        self.assertAlmostEqual(cap_top * MM + self.d.cad_lid_height_mm, 596.0, delta=1e-6)
        bottom, top = self.d.travel_zone_mm()
        # The J1 housing stays in the slot: 26.9 mm above where it opens, 8.1 mm under the cap at J3 = 0.
        self.assertAlmostEqual(bottom - self.d.cad_slot_bottom_mm, 26.9, delta=1e-6)
        self.assertAlmostEqual(column_top * MM - top, 8.1, delta=1e-6)

    def test_every_stroke_fits_its_column(self):
        for stroke in STROKES:
            with self.subTest(stroke=stroke):
                d = ZArmDimensions(z_stroke_mm=stroke)
                swept = housing_sweep_mm(d)
                for got, want in zip(swept, d.travel_zone_mm()):
                    self.assertAlmostEqual(got, want, delta=1e-6)
                underside, top = cap_mm(d)
                grown = max(0.0, stroke - 318.1)  # what the stock column's slot can't hold
                self.assertAlmostEqual(top, 593.0 + grown, delta=1e-6)
                self.assertAlmostEqual(swept[0] - d.cad_slot_bottom_mm, 26.9, delta=1e-6)
                if grown:
                    self.assertAlmostEqual(swept[1], underside, delta=1e-6)  # grown just enough: it meets the cap
                else:
                    self.assertGreaterEqual(underside - swept[1], 8.1 - 1e-6)


class ColumnLiftTests(unittest.TestCase):
    """The stretch that grows the CAD column for strokes over 318.1 mm (410 and 510 mm, say)."""

    def test_no_growth_changes_nothing(self):
        d = ZArmDimensions()
        lift = column_lift(*d.stretch_zone_mm, d.column_growth_mm)
        for z in (0.0, 10.0, 172.0, 198.9, 300.0, 499.9, 578.0, 593.0):
            self.assertEqual(lift(z), z)

    def test_a_longer_stroke_stretches_only_the_middle(self):
        for stroke in (410.0, 510.0):
            with self.subTest(stroke=stroke):
                d = ZArmDimensions(z_stroke_mm=stroke)
                low, high = d.stretch_zone_mm
                grown = d.column_growth_mm
                lift = column_lift(low, high, grown)
                for z in (0.0, 10.0, 172.0, low):  # the base, the bracket and the slot's bottom stay
                    self.assertAlmostEqual(lift(z), z, delta=1e-9)
                for z in (high, 541.0, 578.0, 593.0):  # the slot's top end and the cap move up unchanged
                    self.assertAlmostEqual(lift(z), z + grown, delta=1e-9)
                heights = [low + (high - low) * k / 50 for k in range(51)]
                lifted = [lift(z) for z in heights]
                self.assertEqual(lifted, sorted(lifted))
                self.assertAlmostEqual(lift(578.0), d.travel_zone_mm()[1], delta=1e-9)  # J3 = 0 meets the cap


class CadTravelTests(unittest.TestCase):
    """Run only where scripts/fetch_cad_meshes.py has made the meshes for this model (they're never committed)."""

    @classmethod
    def setUpClass(cls):
        meshes, _ = load_meshes((MODELS_DIR / URDF_NAME).read_bytes())
        if not meshes:
            raise unittest.SkipTest("no CAD meshes for this model on this PC (run scripts/fetch_cad_meshes.py)")
        cls.manifest = json.loads((MESH_DIR / MANIFEST).read_text(encoding="utf-8"))
        cls.d = ZArmDimensions()
        x, y = SLOT_PROBE_MM
        cls.probes = [(x / MM, y / MM), (x / MM, -y / MM)]
        # Only the triangles around the probes matter; keep those (and the column's top for its height).
        cls.carriage = cls.near_probes(stl_triangles(meshes["carriage"]))
        column = stl_triangles(meshes["base_link"])
        cls.column_top = max(z for triangle in column for _, _, z in triangle) * MM
        cls.column = cls.near_probes(column)

    @classmethod
    def near_probes(cls, triangles):
        return [t for t in triangles if any(min(p[0] for p in t) <= x <= max(p[0] for p in t)
                                            and min(p[1] for p in t) <= y <= max(p[1] for p in t)
                                            for x, y in cls.probes)]

    def housing_hits(self, x, y):
        """Where the J1 housing meets the column, in the carriage's frame (m): its underside and top."""
        hits = vertical_hits(self.carriage, x, y)
        return min(hits), max(hits)

    def slot(self, triangles, x, y, inside_mm):
        """(bottom, top) in mm of the column's open slot around a height."""
        hits = vertical_hits(triangles, x, y)
        return max(z for z in hits if z * MM < inside_mm) * MM, min(z for z in hits if z * MM > inside_mm) * MM

    def test_the_carriage_sweeps_the_manifests_travel_zone(self):
        zone = self.manifest["travel_zone_mm"]
        for got, want in zip(zone, self.d.travel_zone_mm()):
            self.assertAlmostEqual(got, want, delta=0.01)
        j3 = build_model(self.d).joint("j3_vertical")
        for x, y in self.probes:
            with self.subTest(y=y):
                bottom, top = self.housing_hits(x, y)
                swept = ((j3.xyz[2] + j3.lower + bottom) * MM, (j3.xyz[2] + j3.upper + top) * MM)
                self.assertAlmostEqual(swept[0], zone[0], delta=1.0)
                self.assertAlmostEqual(swept[1], zone[1], delta=1.0)

    def test_the_column_is_hitbots_and_the_travel_fills_its_slot(self):
        self.assertEqual(self.manifest["column_growth_mm"]["added"], 0.0)
        self.assertAlmostEqual(self.column_top, 593.0, delta=0.5)
        zone = self.manifest["travel_zone_mm"]
        for x, y in self.probes:
            with self.subTest(y=y):
                slot = self.slot(self.column, x, y, sum(zone) / 2.0)
                self.assertAlmostEqual(slot[0], self.d.cad_slot_bottom_mm, delta=0.5)
                self.assertAlmostEqual(slot[1], self.d.cad_slot_top_mm, delta=0.5)
                self.assertEqual([round(v, 1) for v in slot], [round(v, 1) for v in self.manifest["slot_mm"]])
                # The J1 housing's sweep lies within the slot, with 26.9 mm to spare below and 8.1 mm above.
                self.assertAlmostEqual(zone[0] - slot[0], 26.9, delta=0.5)
                self.assertAlmostEqual(slot[1] - zone[1], 8.1, delta=0.5)

    def test_longer_strokes_grow_the_cad_column_by_the_shortfall(self):
        if self.manifest["column_growth_mm"]["added"]:
            self.skipTest("the meshes were made for a grown column; this check grows the stock one")
        for stroke in (410.0, 510.0):
            with self.subTest(stroke=stroke):
                d = ZArmDimensions(z_stroke_mm=stroke)
                lift = column_lift(*d.stretch_zone_mm, d.column_growth_mm)
                grown = [tuple((px, py, lift(pz * MM) / MM) for px, py, pz in t) for t in self.column]
                j3 = build_model(d).joint("j3_vertical")
                zone = d.travel_zone_mm()
                for x, y in self.probes:
                    slot = self.slot(grown, x, y, sum(zone) / 2.0)
                    self.assertAlmostEqual(slot[0], d.cad_slot_bottom_mm, delta=0.5)
                    self.assertAlmostEqual(slot[1], d.cad_slot_top_mm + d.column_growth_mm, delta=0.5)
                    bottom, top = self.housing_hits(x, y)
                    swept = ((j3.xyz[2] + j3.lower + bottom) * MM, (j3.xyz[2] + j3.upper + top) * MM)
                    self.assertAlmostEqual(swept[0] - slot[0], 26.9, delta=0.5)
                    self.assertAlmostEqual(slot[1] - swept[1], 0.0, delta=0.5)  # just reaches the cap


if __name__ == "__main__":
    unittest.main()

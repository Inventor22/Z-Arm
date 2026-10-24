"""The reach envelope and the column's blind zone."""

from __future__ import annotations

import math
import random
import struct
import unittest
import zlib

from tests.support import zarm
from zarm_sim.model import build_model
from zarm_sim.workspace import BLIND, REACHABLE, UNREACHABLE, Workspace, envelope_outline, envelope_png


class WorkspaceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ws = Workspace.from_model(build_model(), (-90.0, 90.0), (-164.0, 164.0))

    def test_geometry_comes_from_the_model(self):
        self.assertEqual(self.ws.column_rect_mm, (-197.0, -69.0, -74.0, 74.0))  # the column, from the J1 axis
        self.assertEqual(self.ws.j2_circles_mm, ((0.0, 53.0), (200.0, 47.5)))
        self.assertEqual(self.ws.j2_box_mm, (0.0, 200.0, 50.25))

    def test_solutions_agree_with_the_sdks_inverse_kinematics(self):
        rng = random.Random(3)
        limits = zarm.JointLimits()
        for _ in range(400):
            x, y = rng.uniform(-430, 430), rng.uniform(-430, 430)
            expected = [
                (j1, j2) for j1, j2 in zarm.kinematics.inverse_xy_solutions(x, y, 220.0, 200.0)
                if limits.j1_deg[0] <= j1 <= limits.j1_deg[1] and limits.j2_deg[0] <= j2 <= limits.j2_deg[1]
            ]
            found = self.ws.solutions(x, y)
            with self.subTest(x=x, y=y):
                self.assertEqual(len(found), len(expected))
                for (a1, a2), (b1, b2) in zip(sorted(found), sorted(expected)):
                    self.assertAlmostEqual(a1, b1, delta=1e-9)
                    self.assertAlmostEqual(a2, b2, delta=1e-9)

    def test_reach(self):
        self.assertEqual(self.ws.classify(419.0, 0.0), REACHABLE)
        self.assertEqual(self.ws.classify(421.0, 0.0), UNREACHABLE)
        inner = math.sqrt(220.0 ** 2 + 200.0 ** 2 + 2 * 220.0 * 200.0 * math.cos(math.radians(164.0)))
        self.assertEqual(self.ws.classify(inner - 0.5, 0.0), UNREACHABLE)
        self.assertEqual(self.ws.classify(inner + 0.5, 0.0), REACHABLE)
        self.assertEqual(self.ws.classify(-100.0, -300.0), REACHABLE)  # behind the J1 axis, J1 at -90

    def test_blind_zone(self):
        self.assertLess(self.ws.j2_clearance_mm(90.0, 164.0), 0.0)  # folded back onto the column
        self.assertAlmostEqual(self.ws.j2_clearance_mm(0.0, 0.0), 69.0 + 220.0 - 53.0, delta=1e-9)
        self.assertGreater(self.ws.j2_clearance_mm(90.0, 90.0), 0.0)
        x, y = zarm.kinematics.forward_xy(90.0, 164.0, 220.0, 200.0)
        self.assertEqual(self.ws.classify(x, y), BLIND)

    def test_picture_is_a_valid_png(self):
        data = envelope_png(self.ws)
        self.assertEqual(data[:8], b"\x89PNG\r\n\x1a\n")
        width, height = struct.unpack(">II", data[16:24])
        self.assertEqual((width, height), (214, 214))
        self.assertEqual(zlib.crc32(data[12:29]) & 0xFFFFFFFF, struct.unpack(">I", data[29:33])[0])  # IHDR's CRC
        self.assertEqual(data[-12:], b"\x00\x00\x00\x00IEND\xaeB`\x82")

    def test_outline_follows_the_edge(self):
        lines = envelope_outline(self.ws)
        self.assertGreaterEqual(len(lines), 3)
        radii = [math.hypot(x, y) for line in lines for x, y in line]
        self.assertAlmostEqual(max(radii), 420.0, delta=0.01)
        near = [(0.0, 0.0)] + [(0.05 * math.cos(k * math.pi / 4), 0.05 * math.sin(k * math.pi / 4)) for k in range(8)]
        for line in lines:
            for x, y in line[::25]:
                with self.subTest(point=(x, y)):
                    reachable = [bool(self.ws.solutions(x + dx, y + dy)) for dx, dy in near]
                    self.assertTrue(any(reachable))  # on the edge: reachable on one side...
                    far = [bool(self.ws.solutions(x + 20 * dx, y + 20 * dy)) for dx, dy in near[1:]]
                    self.assertFalse(all(far))  # ...and not on the other


if __name__ == "__main__":
    unittest.main()

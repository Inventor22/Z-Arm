"""The simulator view's HTTP API, on an ephemeral port, against the SDK's simulated backend."""

from __future__ import annotations

import http.client
import json
import re
import sys
import threading
import time
import unittest
from http.server import BaseHTTPRequestHandler

from tests.support import zarm
from zarm_sim.generate import MODELS_DIR, URDF_NAME
from zarm_sim.kinematics import UrdfRobot
from zarm_sim.model import build_model
from zarm_sim.sdk import NotSimulatedError
from zarm_sim.server import ARM_LENGTHS_MM, LoopbackServer, Simulator, make_server, open_simulator

TOKEN = "test-token-0123456789abcdef"


class Client:
    def __init__(self, port):
        self.port = port

    def request(self, method, path, body=None, token=TOKEN, host=None):
        connection = http.client.HTTPConnection("127.0.0.1", self.port, timeout=10)
        headers = {"Host": host or f"127.0.0.1:{self.port}"}
        data = None
        if body is not None:
            data = json.dumps(body).encode()
            headers["Content-Type"] = "application/json"
        if token is not None and method == "POST":
            headers["X-Panel-Token"] = token
        connection.request(method, path, body=data, headers=headers)
        response = connection.getresponse()
        payload = response.read()
        connection.close()
        return response.status, dict(response.getheaders()), payload

    def get_json(self, path):
        status, _, payload = self.request("GET", path)
        assert status == 200, (status, payload)
        return json.loads(payload)

    def post(self, path, body=None, **kwargs):
        status, _, payload = self.request("POST", path, body if body is not None else {}, **kwargs)
        return status, json.loads(payload)

    def wait_idle(self, timeout_s=10.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            status = self.get_json("/api/status")
            if not status["busy"] and not status.get("moving") and not status["queued"]:
                return status
            time.sleep(0.02)
        raise AssertionError("the simulated arm didn't settle")


class ServerTestCase(unittest.TestCase):
    page_silent_s = 30.0

    @classmethod
    def setUpClass(cls):
        cls.sim = open_simulator(page_silent_s=cls.page_silent_s)
        cls.server = make_server(cls.sim, port=0, token=TOKEN)
        cls.thread = threading.Thread(target=cls.server.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True)
        cls.thread.start()
        cls.client = Client(cls.server.server_address[1])

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.sim.close()

    def setUp(self):
        self.client.post("/api/stop")
        status = self.client.wait_idle()
        if status["phase"] != "initialized":
            self.client.post("/api/initialize")
        _, reply = self.client.post("/api/move_joints", {"j1_deg": 0, "j2_deg": 30, "j3_mm": -50, "j4_deg": 0,
                                                         "speed": 100})
        self.assertTrue(reply["ok"], reply)
        self.client.wait_idle()


class PageAndStatusTests(ServerTestCase):
    def test_page_is_marked_simulated_and_locked_down(self):
        status, headers, body = self.client.request("GET", "/")
        self.assertEqual(status, 200)
        page = body.decode("utf-8")
        self.assertIn("SIMULATED", page)
        self.assertIn(TOKEN, page)
        self.assertNotIn("__CONFIG_JSON__", page)
        csp = headers["Content-Security-Policy"]
        self.assertIn("default-src 'none'", csp)
        self.assertIn("script-src 'self' 'sha256-", csp)
        self.assertEqual(headers["X-Frame-Options"], "DENY")

    def test_status_reports_the_simulated_backend(self):
        status = self.client.get_json("/api/status")
        self.assertTrue(status["simulated"])
        self.assertEqual(status["backend"], "FakeBackend")
        self.assertEqual(status["phase"], "initialized")
        self.assertEqual(set(status["joints"]), {"j1_deg", "j2_deg", "j3_mm", "j4_deg"})
        self.assertTrue(status["modelCheck"]["ok"], status["modelCheck"])
        x, y = zarm.kinematics.forward_xy(status["joints"]["j1_deg"], status["joints"]["j2_deg"], *ARM_LENGTHS_MM)
        self.assertAlmostEqual(status["pose"]["x_mm"], x, delta=0.01)
        self.assertAlmostEqual(status["pose"]["y_mm"], y, delta=0.01)

    def test_host_header_is_checked(self):
        status, _, _ = self.client.request("GET", "/api/status", host="evil.example:80")
        self.assertEqual(status, 403)

    def test_posts_need_the_token(self):
        for token in (None, "wrong-token"):
            with self.subTest(token=token):
                status, reply = self.client.post("/api/jog", {"joint": 1, "dir": 1, "step": 1, "speed": 10}, token=token)
                self.assertEqual(status, 403)
                self.assertFalse(reply["ok"])

    def test_static_files_models_and_envelope(self):
        status, _, body = self.client.request("GET", f"/model/{URDF_NAME}")
        self.assertEqual((status, body), (200, (MODELS_DIR / URDF_NAME).read_bytes()))
        for path in ("/web/app.js", "/web/view.js", "/web/urdf.js", "/web/plan.js", "/web/kinematics.js", "/web/gizmo.js",
                     "/web/style.css"):
            self.assertEqual(self.client.request("GET", path)[0], 200, path)
        for path in ("/web/../server.py", "/web/index.html", "/vendor/three/build/three.module.js", "/meshes/j1_arm.stl",
                     "/model/../zarm_sim/server.py"):
            self.assertEqual(self.client.request("GET", path)[0], 404, path)
        envelope = self.client.get_json("/api/envelope.json")
        self.assertGreaterEqual(len(envelope["outlineMm"]), 3)
        status, headers, png = self.client.request("GET", envelope["png"])
        self.assertEqual((status, headers["Content-Type"], png[:4]), (200, "image/png", b"\x89PNG"))

    def test_never_loads_the_native_backend(self):
        self.assertNotIn("zarm.native", sys.modules)

    def test_page_config_carries_the_plan_checks_and_the_cad_command(self):
        _, _, body = self.client.request("GET", "/")
        found = re.search(r'<script type="application/json" id="zarm-config">(.*?)</script>', body.decode("utf-8"), re.S)
        config = json.loads(found.group(1))
        ws = self.sim.workspace
        self.assertEqual(config["blindZone"], {
            "columnRectMm": list(ws.column_rect_mm), "j2CirclesMm": [list(c) for c in ws.j2_circles_mm],
            "j2BoxMm": list(ws.j2_box_mm), "l1Mm": 220.0, "l2Mm": 200.0,
        })
        self.assertIsNone(config["meshes"])  # no meshes given: the page draws the primitives and says how to get CAD
        self.assertTrue(config["cadCommand"].startswith("uv run "))
        self.assertIn("fetch_cad_meshes.py", config["cadCommand"])


class MotionTests(ServerTestCase):
    def test_jogs_accumulate_exact_steps(self):
        start = self.client.get_json("/api/status")["joints"]
        replies = [self.client.post("/api/jog", {"joint": 1, "dir": 1, "step": 1, "speed": 5})[1] for _ in range(3)]
        self.assertTrue(all(reply["ok"] and reply["moved"] for reply in replies), replies)
        self.assertIn("queued", replies[1]["message"])
        end = self.client.wait_idle()["joints"]
        self.assertAlmostEqual(end["j1_deg"], start["j1_deg"] + 3.0, delta=1e-9)
        for field in ("j2_deg", "j3_mm", "j4_deg"):
            self.assertAlmostEqual(end[field], start[field], delta=1e-9)

    def test_jogs_hold_when_too_far_ahead(self):
        replies = [self.client.post("/api/jog", {"joint": 3, "dir": -1, "step": 0.5, "speed": 5})[1] for _ in range(8)]
        self.assertTrue(any(reply["ok"] and not reply["moved"] and "Holding" in reply["message"] for reply in replies))
        self.client.wait_idle()

    def test_soft_limit_refusals_are_reported_in_the_sdks_words(self):
        _, reply = self.client.post("/api/move_joints", {"j1_deg": 95, "j2_deg": 0, "j3_mm": 0, "j4_deg": 0, "speed": 50})
        self.assertFalse(reply["ok"])
        self.assertEqual(reply["error"], "SoftLimitError")
        self.assertIn("j1_deg=95 is outside [-90, 90] deg", reply["message"])
        log = self.client.get_json("/api/status")["log"]
        self.assertEqual(log[-1]["level"], "error")
        self.assertIn("SoftLimitError", log[-1]["text"])

    def test_a_jog_past_the_soft_limit_is_refused(self):
        self.client.post("/api/move_joints", {"j1_deg": 89.5, "j2_deg": 0, "j3_mm": 0, "j4_deg": 0, "speed": 100})
        self.client.wait_idle()
        _, reply = self.client.post("/api/jog", {"joint": 1, "dir": 1, "step": 1, "speed": 10})
        self.assertFalse(reply["ok"])
        self.assertEqual(reply["error"], "SoftLimitError")
        self.assertAlmostEqual(self.client.get_json("/api/status")["joints"]["j1_deg"], 89.5, delta=1e-9)

    def test_invalid_requests(self):
        cases = [
            ("/api/jog", {"joint": 5, "dir": 1, "step": 1, "speed": 10}),
            ("/api/jog", {"joint": 1, "dir": 2, "step": 1, "speed": 10}),
            ("/api/jog", {"joint": 1, "dir": 1, "step": 3, "speed": 10}),
            ("/api/move_joints", {"j1_deg": "ten", "j2_deg": 0, "j3_mm": 0, "j4_deg": 0, "speed": 10}),
            ("/api/move_pose", {"x_mm": 300, "y_mm": 0, "z_mm": 0, "r_deg": 0, "speed": 10, "hand": "left"}),
            ("/api/move_joints", {"j1_deg": 0, "j2_deg": 0, "j3_mm": 0, "j4_deg": 0, "speed": 500}),
        ]
        for path, body in cases:
            with self.subTest(path=path, body=body):
                status, reply = self.client.post(path, body)
                self.assertEqual(status, 200)
                self.assertFalse(reply["ok"])
        self.assertEqual(self.client.post("/api/nothing", {})[0], 404)
        status, _, _ = self.client.request("POST", "/api/stop", body=None)
        self.assertEqual(status, 200)

    def test_pose_and_linear_moves_arrive(self):
        for hand in ("none", "plus", "minus", "keep"):
            with self.subTest(hand=hand):
                _, reply = self.client.post("/api/move_pose", {"x_mm": 300, "y_mm": 80, "z_mm": -100, "r_deg": 15,
                                                               "hand": hand, "speed": 100})
                self.assertTrue(reply["ok"], reply)
                status = self.client.wait_idle()
                self.assertAlmostEqual(status["pose"]["x_mm"], 300, delta=0.01)
                self.assertAlmostEqual(status["pose"]["y_mm"], 80, delta=0.01)
                if hand == "plus":
                    self.assertGreaterEqual(status["joints"]["j2_deg"], 0)
                if hand == "minus":
                    self.assertLessEqual(status["joints"]["j2_deg"], 0)
        _, reply = self.client.post("/api/move_linear", {"x_mm": 250, "y_mm": -50, "z_mm": -20, "r_deg": 0,
                                                         "speed_mm_s": 100})
        self.assertTrue(reply["ok"], reply)
        status = self.client.wait_idle()
        self.assertAlmostEqual(status["pose"]["x_mm"], 250, delta=0.01)
        self.assertIn("Arrived", status["log"][-1]["text"])

    def test_change_hand(self):
        before = self.client.get_json("/api/status")
        _, reply = self.client.post("/api/change_hand", {"speed": 100})
        self.assertTrue(reply["ok"], reply)
        after = self.client.wait_idle()
        self.assertLess(after["joints"]["j2_deg"] * before["joints"]["j2_deg"], 0)
        self.assertAlmostEqual(after["pose"]["x_mm"], before["pose"]["x_mm"], delta=0.01)

    def test_stop_halts_a_move(self):
        _, reply = self.client.post("/api/move_joints", {"j1_deg": 80, "j2_deg": 30, "j3_mm": -50, "j4_deg": 0,
                                                         "speed": 10})
        self.assertTrue(reply["ok"])
        time.sleep(0.3)
        busy = self.client.get_json("/api/status")
        self.assertTrue(busy["busy"])
        self.assertIn("goal", busy)
        _, reply = self.client.post("/api/stop")
        self.assertTrue(reply["ok"])
        status = self.client.wait_idle(2.0)
        self.assertGreater(status["joints"]["j1_deg"], 0.5)
        self.assertLess(status["joints"]["j1_deg"], 79)
        self.assertNotIn("goal", status)

    def test_moves_are_refused_while_one_is_under_way(self):
        self.client.post("/api/move_joints", {"j1_deg": 60, "j2_deg": 30, "j3_mm": -50, "j4_deg": 0, "speed": 10})
        _, reply = self.client.post("/api/move_joints", {"j1_deg": 0, "j2_deg": 0, "j3_mm": 0, "j4_deg": 0, "speed": 10})
        self.assertFalse(reply["ok"])
        self.assertIn("under way", reply["message"])
        self.client.post("/api/stop")

    def test_emergency_stop_latches_until_initialize(self):
        self.client.post("/api/move_joints", {"j1_deg": 60, "j2_deg": 30, "j3_mm": -50, "j4_deg": 0, "speed": 10})
        _, reply = self.client.post("/api/estop")
        self.assertTrue(reply["ok"])
        status = self.client.wait_idle(2.0)
        self.assertEqual(status["phase"], "connected")
        self.assertFalse(status["ready"])
        _, reply = self.client.post("/api/jog", {"joint": 1, "dir": 1, "step": 1, "speed": 10})
        self.assertFalse(reply["ok"])
        self.assertIn("Initialize", reply["message"])
        _, reply = self.client.post("/api/initialize")
        self.assertTrue(reply["ok"], reply)
        self.assertEqual(self.client.get_json("/api/status")["phase"], "initialized")


class PageQuietTests(ServerTestCase):
    page_silent_s = 0.6

    def test_a_move_stops_when_the_page_goes_quiet(self):
        self.client.post("/api/move_joints", {"j1_deg": 80, "j2_deg": 30, "j3_mm": -50, "j4_deg": 0, "speed": 10})
        time.sleep(1.5)  # no requests at all
        status = self.client.get_json("/api/status")
        self.assertFalse(status["busy"])
        self.assertIn("went quiet", " ".join(entry["text"] for entry in status["log"]))
        self.assertLess(status["joints"]["j1_deg"], 30)


class SimulatedOnlyTests(unittest.TestCase):
    def test_a_busy_port_is_an_error(self):
        first = LoopbackServer(("127.0.0.1", 0), BaseHTTPRequestHandler)
        try:
            with self.assertRaises(OSError):  # not a silent second listener (SO_REUSEADDR on Windows)
                LoopbackServer(("127.0.0.1", first.server_address[1]), BaseHTTPRequestHandler)
        finally:
            first.server_close()
    def test_refuses_anything_but_the_fake_backend(self):
        model = build_model()
        urdf = UrdfRobot(MODELS_DIR / URDF_NAME)

        class LookAlike(zarm.FakeBackend):
            pass

        for backend in (LookAlike((1,)),):
            arm = zarm.ZArm(backend, zarm.ZArmConfig(arm_lengths_mm=ARM_LENGTHS_MM))
            with self.subTest(backend=type(backend).__name__), self.assertRaises(NotSimulatedError):
                Simulator(arm, model, urdf)
        with self.assertRaises(NotSimulatedError):
            Simulator(object(), model, urdf)  # type: ignore[arg-type]

    def test_accepts_the_fake_backend(self):
        arm = zarm.ZArm.simulated(zarm.ZArmConfig(arm_lengths_mm=ARM_LENGTHS_MM))
        sim = Simulator(arm, build_model(), UrdfRobot(MODELS_DIR / URDF_NAME))
        self.assertEqual(sim.warnings, [])
        arm.close()


if __name__ == "__main__":
    unittest.main()

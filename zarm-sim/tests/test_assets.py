"""three.js's checked download and the optional CAD meshes' integrity checks."""

from __future__ import annotations

import hashlib
import json
import tempfile
import unittest
import urllib.error
from pathlib import Path
from unittest import mock

from zarm_sim import three_js
from zarm_sim.meshes import MANIFEST, MESH_LINKS, load_meshes

FAKE_FILES = {"build/three.module.js": b"export const REVISION = '186';\n", "LICENSE": b"MIT\n"}


class ThreeDownloadTests(unittest.TestCase):
    def setUp(self):
        hashes = {path: hashlib.sha256(data).hexdigest() for path, data in FAKE_FILES.items()}
        patcher = mock.patch.object(three_js, "THREE_FILES", hashes)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.folder = Path(tempfile.mkdtemp())
        self.fetched = []

    def fetch(self, url):
        self.fetched.append(url)
        for path, data in FAKE_FILES.items():
            if url.endswith(path):
                return data
        raise urllib.error.HTTPError(url, 404, "not found", None, None)

    def test_downloads_checks_and_caches(self):
        files, why = three_js.load_three(self.folder, say=lambda text: None, fetch=self.fetch)
        self.assertIsNone(why)
        self.assertEqual(files, FAKE_FILES)
        self.assertTrue(all(url.startswith(three_js.THREE_MIRRORS[0]) for url in self.fetched))
        self.fetched.clear()
        files, why = three_js.load_three(self.folder, say=lambda text: None, fetch=self.fetch)
        self.assertEqual((files, why, self.fetched), (FAKE_FILES, None, []))  # from the cache, nothing fetched

    def test_altered_files_are_refused(self):
        def tampered(url):
            return b"alert('not three.js')"

        files, why = three_js.load_three(self.folder, say=lambda text: None, fetch=tampered)
        self.assertEqual(files, {})
        self.assertIn("isn't the published file", why)
        self.assertFalse(any(self.folder.rglob("*.js")))

    def test_a_dead_mirror_falls_back_to_the_next(self):
        def first_dead(url):
            if url.startswith(three_js.THREE_MIRRORS[0]):
                raise urllib.error.URLError("no route to host")
            return self.fetch(url)

        files, why = three_js.load_three(self.folder, say=lambda text: None, fetch=first_dead)
        self.assertEqual((files, why), (FAKE_FILES, None))


class PinnedThreeTests(unittest.TestCase):
    def test_pins(self):
        self.assertEqual(three_js.THREE_VERSION, "0.186.1")  # the same version and hashes as umbratek-code
        self.assertEqual(three_js.THREE_FILES["build/three.module.js"],
                         "9052042d676cb0fdc1ddfefe193053f34b7ac0513a616fdac4535d49987812ea")
        self.assertEqual(three_js.THREE_FILES["examples/jsm/loaders/STLLoader.js"],
                         "023ed97f848b633d8bcd53d4db3b996d29d0c644088700691297c552257d480b")
        self.assertEqual(len(three_js.THREE_FILES), 5)


class MeshCacheTests(unittest.TestCase):
    def write_cache(self, folder, urdf, damage=None):
        links = {}
        for link in MESH_LINKS:
            data = f"solid {link}\nendsolid {link}\n".encode()
            (folder / f"{link}.stl").write_bytes(data)
            links[link] = {"file": f"{link}.stl", "sha256": hashlib.sha256(data).hexdigest()}
        manifest = {"format": 1, "urdf_sha256": hashlib.sha256(urdf).hexdigest(), "links": links}
        (folder / MANIFEST).write_text(json.dumps(manifest), encoding="utf-8")
        if damage:
            (folder / f"{damage}.stl").write_bytes(b"changed")

    def test_absent_cache_means_primitives_without_a_note(self):
        with tempfile.TemporaryDirectory() as folder:
            self.assertEqual(load_meshes(b"<robot/>", Path(folder)), ({}, None))

    def test_valid_cache_loads(self):
        with tempfile.TemporaryDirectory() as folder:
            self.write_cache(Path(folder), b"<robot/>")
            meshes, note = load_meshes(b"<robot/>", Path(folder))
            self.assertIsNone(note)
            self.assertEqual(set(meshes), set(MESH_LINKS))

    def test_meshes_for_another_model_are_refused(self):
        with tempfile.TemporaryDirectory() as folder:
            self.write_cache(Path(folder), b"<robot name='old'/>")
            meshes, note = load_meshes(b"<robot/>", Path(folder))
            self.assertEqual(meshes, {})
            self.assertIn("different model", note)

    def test_damaged_meshes_are_refused(self):
        with tempfile.TemporaryDirectory() as folder:
            self.write_cache(Path(folder), b"<robot/>", damage="j2_arm")
            meshes, note = load_meshes(b"<robot/>", Path(folder))
            self.assertEqual(meshes, {})
            self.assertIn("j2_arm.stl", note)


if __name__ == "__main__":
    unittest.main()

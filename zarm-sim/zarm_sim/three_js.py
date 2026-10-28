"""three.js for the 3D view, fetched and checked the way umbratek-code's jog panel does it.

three.js isn't kept in the repository. The first time the server starts, it
downloads these files of three 0.186.1 into ``zarm-sim/vendor/three/`` (which
git ignores) from the first mirror that has them, and every start checks them
against the SHA-256 hashes of the files as published on npm. The page gets
them from this server only, served from the copy checked at startup.
"""

from __future__ import annotations

import hashlib
import http.client
import os
import sys
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Callable, Dict, Optional, Tuple

THREE_VERSION = "0.186.1"
# The same files and hashes as umbratek-code's scripts/jog_server.py.
THREE_FILES = {
    "build/three.module.js": "9052042d676cb0fdc1ddfefe193053f34b7ac0513a616fdac4535d49987812ea",
    "build/three.core.js": "9edde002b066a9a05676a6127f67735b62baf399bdea529f2f7e31657da769e6",
    "examples/jsm/controls/OrbitControls.js": "3d79d07ecb686b4e5d93232eedab255331c1beef711e13164eaa1f68655a5f2b",
    "examples/jsm/loaders/STLLoader.js": "023ed97f848b633d8bcd53d4db3b996d29d0c644088700691297c552257d480b",
    "LICENSE": "8b378ebe60e2fe500158cb0ac71cb5e8b7d92953c2abcc63a0eb90499653b5bc",  # MIT: kept with the copy, not served
}
THREE_MIRRORS = (f"https://cdn.jsdelivr.net/npm/three@{THREE_VERSION}/", f"https://unpkg.com/three@{THREE_VERSION}/")
THREE_DIR = Path(__file__).resolve().parents[1] / "vendor" / "three"
THREE_URL = "/vendor/three/"
DOWNLOAD_TIMEOUT_S = 10.0


def load_three(
    directory: Path = THREE_DIR,
    say: Callable[[str], None] = print,
    fetch: Optional[Callable[[str], bytes]] = None,
) -> Tuple[Dict[str, bytes], Optional[str]]:
    """three.js's files as {path: bytes}, each matching its hash: read from ``directory``, or downloaded there.

    Returns (files, None), or ({}, why) when a file can't be had (say, with no internet).
    """

    def download(url: str) -> bytes:
        with urllib.request.urlopen(url, timeout=DOWNLOAD_TIMEOUT_S) as reply:
            return reply.read()

    fetch = fetch or download
    files: Dict[str, bytes] = {}
    dead: Dict[str, str] = {}
    announced = False
    for path, digest in THREE_FILES.items():
        local = directory.joinpath(*path.split("/"))
        try:
            data = local.read_bytes()
        except OSError:
            data = b""
        if hashlib.sha256(data).hexdigest() == digest:
            files[path] = data
            continue
        if not announced:
            say(f"Downloading three.js {THREE_VERSION} for the 3D view (the first time only)...")
            announced = True
        problems = []
        for base in THREE_MIRRORS:
            host = urllib.parse.urlsplit(base).hostname
            if base in dead:  # it didn't answer for an earlier file, so don't wait on it again
                problems.append(f"{host}: {dead[base]}")
                continue
            try:
                data = fetch(base + path)
            except (OSError, http.client.HTTPException) as error:  # URLError, HTTPError and timeouts are OSErrors
                if isinstance(error, urllib.error.HTTPError):
                    problems.append(f"{host}: HTTP {error.code}")
                else:
                    dead[base] = str(getattr(error, "reason", None) or error)
                    problems.append(f"{host}: {dead[base]}")
                continue
            if hashlib.sha256(data).hexdigest() == digest:
                break
            problems.append(f"{host}'s copy of {path} isn't the published file")
        else:
            return {}, "; ".join(problems)
        files[path] = data
        try:  # write it whole or not at all, so a failed save can't leave a half file
            local.parent.mkdir(parents=True, exist_ok=True)
            partial = local.with_name(local.name + ".part")
            partial.write_bytes(data)
            os.replace(partial, local)
        except OSError as error:
            print(f"Warning: couldn't save {path} ({error}); it'll be downloaded again next time.", file=sys.stderr)
    if announced:
        say(f"Saved three.js in {directory}. From now on the 3D view works offline.")
    return files, None

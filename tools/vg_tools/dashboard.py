"""A local dashboard for the mapping workflow (vg dashboard): recordings ->
walks (map packages) -> sites -> maps, as the app will run it on the phone.

    vg dashboard --recordings DIR --sites DIR [--port 8765]

Serves a page on http://127.0.0.1:PORT (this computer only) that lists the
recordings and sites, runs vg replay / vg site / vg mcap stats as jobs (one
at a time, with their logs), and shows any map of a site or walk with the
walks' paths over it.

A site is a directory under --sites: walks/<name>/ holds its walks (map
packages, made from recordings by vg replay), and vg site keeps site.csv and
current/ beside them (see include/vg_core/site.hpp). Pure standard library.
"""

from __future__ import annotations

import csv
import json
import math
import re
import subprocess
import sys
import threading
import time
import webbrowser
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

from vg_tools import heatmap, semantics

PAGE = Path(__file__).with_name("dashboard.html")
RECORDING_SUFFIX = ".vgcap.mcap"
# Names of sites and walks: one directory level, nothing that climbs out.
NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,80}$")
STAMP = re.compile(r"(\d{8}-\d{6})")
# Path points kept per walk for drawing.
MAX_PATH_POINTS = 600


def valid_name(name: str) -> bool:
    return bool(NAME.match(name)) and ".." not in name


@dataclass
class Workspace:
    recordings: Path
    sites: Path
    root: Path  # The vg_core checkout, where `vg` runs from.

    def recording(self, name: str) -> Path:
        path = self.recordings / name
        if not name.endswith(RECORDING_SUFFIX) or "/" in name or "\\" in name or not path.is_file():
            raise ValueError(f"no recording {name}")
        return path

    def site(self, name: str) -> Path:
        if not valid_name(name):
            raise ValueError(f"bad site name {name!r}")
        return self.sites / name


# --- Reading the workspace -------------------------------------------------------------------


def _stamp(name: str) -> str:
    m = STAMP.search(name)
    return m.group(1) if m else ""


def list_recordings(ws: Workspace) -> list[dict]:
    """Recordings, newest first, with the tag of a renamed video beside them
    (e.g. shed-recording-20261109-120459.mp4 -> shed) and which walks use them."""
    if not ws.recordings.is_dir():
        return []
    tags = {}
    for mp4 in ws.recordings.glob("*.mp4"):
        stamp = _stamp(mp4.name)
        prefix = mp4.name.split("recording-")[0].rstrip("-")
        if stamp and prefix:
            tags[stamp] = prefix
    used: dict[str, list[str]] = {}
    for site in list_sites(ws):
        for walk in site["walks"]:
            used.setdefault(walk["session"], []).append(f"{site['name']}/{walk['name']}")
    out = []
    for path in ws.recordings.glob("*" + RECORDING_SUFFIX):
        stamp = _stamp(path.name)
        labels_csv = path.with_name(path.name.replace(RECORDING_SUFFIX, ".labels.csv"))
        out.append({
            "name": path.name,
            "stamp": stamp,
            "tag": tags.get(stamp, ""),
            "size_mb": round(path.stat().st_size / 1e6, 1),
            "labels_csv": labels_csv.is_file(),
            "walks": used.get(path.name, []),
        })
    out.sort(key=lambda r: r["stamp"], reverse=True)
    return out


def _manifest(package: Path) -> dict:
    try:
        return json.loads((package / "manifest.json").read_text())
    except (OSError, ValueError):
        return {}


def walk_summary(package: Path) -> dict:
    m = _manifest(package)
    stages = m.get("stages", {})
    build = stages.get("build", {})
    geo = m.get("geo_reference") or {}
    return {
        "name": package.name,
        "session": m.get("session", ""),
        "stamp": _stamp(m.get("session", "")),
        "stages": list(stages),
        "frames": m.get("frames_integrated"),
        "label_consistency": build.get("label_consistency"),
        "sky_frames": build.get("sky_frames"),
        "gps_bearing_sigma_deg": geo.get("bearing_sigma_deg"),
        "phone_height_m": stages.get("terrain", {}).get("phone_height_m"),
        "canopy_cells": stages.get("sky", {}).get("canopy_cells"),
        "maps": sorted(p.name for p in package.glob("*.asc")),
    }


def read_visits(site: Path) -> list[dict]:
    path = site / "site.csv"
    if not path.is_file():
        return []
    visits = []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            package = Path(row["package"])
            visits.append({
                "walk": package.name,
                "package": row["package"],
                "recorded": row["recorded"],
                "yaw_deg": float(row["yaw_deg"]),
                "x": float(row["x"]),
                "y": float(row["y"]),
                "z": float(row["z"]),
                "used_gps": row.get("used_gps") == "1",
                "inliers_30cm": float(row["inliers_30cm"]) if row.get("inliers_30cm") else None,
                "overlap": float(row["overlap"]) if row.get("overlap") else None,
                "rival_ratio": float(row["rival_ratio"]) if row.get("rival_ratio") else None,
            })
    return visits


def list_sites(ws: Workspace) -> list[dict]:
    if not ws.sites.is_dir():
        return []
    sites = []
    for d in sorted(p for p in ws.sites.iterdir() if p.is_dir() and valid_name(p.name)):
        walks_dir = d / "walks"
        if not (d / "site.csv").is_file() and not walks_dir.is_dir():
            continue
        walks = [walk_summary(p) for p in sorted(walks_dir.glob("*")) if p.is_dir()] \
            if walks_dir.is_dir() else []
        sites.append({
            "name": d.name,
            "walks": walks,
            "visits": read_visits(d),
            "maps": sorted(p.name for p in (d / "current").glob("*.asc")),
        })
    return sites


def site_paths(ws: Workspace, site_name: str) -> list[dict]:
    """Each visit's walked path in the site frame, thinned for drawing."""
    site = ws.site(site_name)
    out = []
    for v in read_visits(site):
        package = Path(v["package"])
        package = package if package.is_absolute() else site / package
        path = package / "trajectory.csv"
        if not path.is_file():
            continue
        yaw = math.radians(v["yaw_deg"])
        c, s = math.cos(yaw), math.sin(yaw)
        with open(path, newline="") as f:
            rows = [(float(r["x"]), float(r["y"])) for r in csv.DictReader(f)]
        step = max(1, len(rows) // MAX_PATH_POINTS)
        out.append({
            "walk": v["walk"],
            "points": [[round(c * x - s * y + v["x"], 2), round(s * x + c * y + v["y"], 2)]
                       for x, y in rows[::step]],
        })
    return out


def grid_json(ws: Workspace, site_name: str, layer: str, walk: str | None) -> dict:
    """A map of a site (current/) or of one of its walks, as plain numbers."""
    base = ws.site(site_name)
    if walk:
        if not valid_name(walk):
            raise ValueError(f"bad walk name {walk!r}")
        base = base / "walks" / walk
    else:
        base = base / "current"
    if not valid_name(layer) or not layer.endswith(".asc"):
        raise ValueError(f"bad map {layer!r}")
    grid = heatmap.read_ascii_grid(base / layer)
    values = grid.values()
    return {
        "ncols": grid.ncols,
        "nrows": grid.nrows,
        "xll": grid.xllcorner,
        "yll": grid.yllcorner,
        "cell": grid.cellsize,
        "min": min(values) if values else 0.0,
        "max": max(values) if values else 0.0,
        # Top row first, as in the file; null where there's no data.
        "values": [None if v is None else round(v, 3) for row in grid.rows for v in row],
    }


# --- Jobs -----------------------------------------------------------------------------------


@dataclass
class Job:
    id: int
    title: str
    args: list[str]
    state: str = "queued"  # queued, running, done, failed
    returncode: int | None = None
    started: float | None = None
    ended: float | None = None
    lines: list[str] = field(default_factory=list)

    def summary(self) -> dict:
        return {"id": self.id, "title": self.title, "state": self.state,
                "returncode": self.returncode, "started": self.started, "ended": self.ended,
                "command": "vg " + " ".join(self.args)}


class Jobs:
    """vg commands run one at a time (they build the C++ apps, which mustn't
    race), output kept for the page."""

    def __init__(self, root: Path, command: list[str] | None = None):
        self.root = root
        # How to run `vg`: this Python running the CLI module.
        self.command = command or [sys.executable, "-m", "vg_tools.cli"]
        self.jobs: list[Job] = []
        self.lock = threading.Lock()
        self.wake = threading.Condition(self.lock)
        threading.Thread(target=self._work, daemon=True).start()

    def submit(self, title: str, args: list[str]) -> Job:
        with self.lock:
            job = Job(len(self.jobs) + 1, title, args)
            self.jobs.append(job)
            self.wake.notify()
            return job

    def get(self, job_id: int) -> Job | None:
        with self.lock:
            return self.jobs[job_id - 1] if 0 < job_id <= len(self.jobs) else None

    def summaries(self) -> list[dict]:
        with self.lock:
            return [j.summary() for j in reversed(self.jobs)]

    def _work(self) -> None:
        while True:
            with self.lock:
                job = next((j for j in self.jobs if j.state == "queued"), None)
                if job is None:
                    self.wake.wait()
                    continue
                job.state = "running"
                job.started = time.time()
            try:
                proc = subprocess.Popen(
                    [*self.command, *job.args], cwd=self.root, stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT, text=True, errors="replace", bufsize=1)
                assert proc.stdout is not None
                for line in proc.stdout:
                    with self.lock:
                        job.lines.append(line.rstrip("\n"))
                rc = proc.wait()
            except OSError as e:
                with self.lock:
                    job.lines.append(f"dashboard: cannot run: {e}")
                rc = -1
            with self.lock:
                job.returncode = rc
                job.state = "done" if rc == 0 else "failed"
                job.ended = time.time()


# --- Actions --------------------------------------------------------------------------------


def start_replay(ws: Workspace, jobs: Jobs, recording: str, site: str, walk: str) -> Job:
    rec = ws.recording(recording)
    if not valid_name(walk):
        raise ValueError(f"bad walk name {walk!r}")
    out = ws.site(site) / "walks" / walk
    return jobs.submit(f"replay {recording} -> {site}/{walk}", ["replay", str(rec), str(out)])


def start_site_add(ws: Workspace, jobs: Jobs, site: str, walks: list[str], force: bool) -> Job:
    base = ws.site(site)
    for w in walks:
        if not valid_name(w) or not (base / "walks" / w).is_dir():
            raise ValueError(f"no walk {w!r} in {site}")
    if not walks:
        raise ValueError("no walks given")
    args = ["site", "add", str(base), *[str(base / "walks" / w) for w in walks]]
    if force:
        args.append("--force")
    return jobs.submit(f"site add {site}: {', '.join(walks)}", args)


def start_site_maps(ws: Workspace, jobs: Jobs, site: str) -> Job:
    return jobs.submit(f"site maps {site}", ["site", "maps", str(ws.site(site))])


def start_stats(ws: Workspace, jobs: Jobs, recording: str) -> Job:
    return jobs.submit(f"stats {recording}", ["mcap", "stats", str(ws.recording(recording))])


def create_site(ws: Workspace, name: str) -> None:
    (ws.site(name) / "walks").mkdir(parents=True, exist_ok=True)


# --- HTTP -----------------------------------------------------------------------------------


def make_handler(ws: Workspace, jobs: Jobs):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args) -> None:  # Quiet.
            pass

        def _send(self, status: int, body: bytes, kind: str) -> None:
            self.send_response(status)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, value, status: int = 200) -> None:
            self._send(status, json.dumps(value).encode(), "application/json")

        def do_GET(self) -> None:
            url = urlparse(self.path)
            q = {k: v[0] for k, v in parse_qs(url.query).items()}
            try:
                if url.path == "/":
                    self._send(200, PAGE.read_bytes(), "text/html; charset=utf-8")
                elif url.path == "/api/state":
                    self._json({
                        "workspace": {"recordings": str(ws.recordings), "sites": str(ws.sites)},
                        "recordings": list_recordings(ws),
                        "sites": list_sites(ws),
                        "jobs": jobs.summaries(),
                        "classes": [{"name": n, "color": c} for n, c in semantics.CLASSES],
                        "palette": [[t, list(c)] for t, c in heatmap.STOPS],
                    })
                elif url.path == "/api/jobs":
                    self._json(jobs.summaries())
                elif url.path == "/api/job":
                    job = jobs.get(int(q.get("id", "0")))
                    if job is None:
                        self._json({"error": "no such job"}, 404)
                        return
                    start = int(q.get("from", "0"))
                    with jobs.lock:
                        self._json({**job.summary(), "lines": job.lines[start:],
                                    "next": len(job.lines)})
                elif url.path == "/api/grid":
                    self._json(grid_json(ws, q["site"], q["map"], q.get("walk")))
                elif url.path == "/api/paths":
                    self._json(site_paths(ws, q["site"]))
                else:
                    self._json({"error": "not found"}, 404)
            except (ValueError, KeyError, OSError) as e:
                self._json({"error": str(e)}, 400)

        def do_POST(self) -> None:
            url = urlparse(self.path)
            try:
                length = int(self.headers.get("Content-Length", "0"))
                body = json.loads(self.rfile.read(length) or b"{}")
                if url.path == "/api/replay":
                    job = start_replay(ws, jobs, body["recording"], body["site"], body["walk"])
                elif url.path == "/api/site/add":
                    job = start_site_add(ws, jobs, body["site"], list(body["walks"]),
                                         bool(body.get("force")))
                elif url.path == "/api/site/maps":
                    job = start_site_maps(ws, jobs, body["site"])
                elif url.path == "/api/stats":
                    job = start_stats(ws, jobs, body["recording"])
                elif url.path == "/api/site/create":
                    create_site(ws, body["site"])
                    self._json({"ok": True})
                    return
                else:
                    self._json({"error": "not found"}, 404)
                    return
                self._json(job.summary())
            except (ValueError, KeyError, OSError, TypeError) as e:
                self._json({"error": str(e)}, 400)

    return Handler


def serve(ws: Workspace, port: int = 8765, open_browser: bool = True) -> None:
    jobs = Jobs(ws.root)
    server = ThreadingHTTPServer(("127.0.0.1", port), make_handler(ws, jobs))
    url = f"http://127.0.0.1:{server.server_address[1]}/"
    print(f"vg dashboard: {url}  (recordings {ws.recordings}, sites {ws.sites}; Ctrl+C stops)",
          flush=True)
    if open_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()

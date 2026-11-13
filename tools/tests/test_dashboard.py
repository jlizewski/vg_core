import json
import sys
import threading
import time
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

import pytest

from vg_tools import dashboard

GRID = """ncols 2
nrows 2
xllcorner 10
yllcorner 20
cellsize 0.5
NODATA_value -9999
1 -9999
3 4
"""


def make_workspace(tmp_path: Path) -> dashboard.Workspace:
    rec = tmp_path / "recordings"
    rec.mkdir()
    (rec / "recording-20261109-120459.vgcap.mcap").write_bytes(b"x" * 2000)
    (rec / "shed-recording-20261109-120459.mp4").write_bytes(b"")
    (rec / "recording-20261109-115106.vgcap.mcap").write_bytes(b"")
    site = tmp_path / "sites" / "backyard"
    walk = site / "walks" / "shed"
    walk.mkdir(parents=True)
    (walk / "manifest.json").write_text(json.dumps({
        "session": "recording-20261109-120459.vgcap.mcap",
        "frames_integrated": 518,
        "geo_reference": {"bearing_sigma_deg": 13.4},
        "stages": {"build": {"label_consistency": 0.84, "sky_frames": 510},
                   "terrain": {"phone_height_m": 1.55}},
    }))
    (walk / "surface.asc").write_text(GRID)
    (walk / "trajectory.csv").write_text("t,x,y,z\n0,0,0,1.5\n1,1,0,1.5\n")
    (site / "walks" / "fence").mkdir()
    (site / "site.csv").write_text(
        "package,recorded,yaw_deg,x,y,z,used_gps,peak,rival_ratio,overlap,inliers_10cm,"
        "inliers_30cm,median_m\n"
        "walks/shed,20261109-120459,90,10,20,0,,,,,,,\n")
    (site / "current").mkdir()
    (site / "current" / "semantic.asc").write_text(GRID)
    return dashboard.Workspace(rec, tmp_path / "sites", tmp_path)


def test_lists_recordings_with_tags_and_walks(tmp_path: Path):
    ws = make_workspace(tmp_path)
    recs = dashboard.list_recordings(ws)
    assert [r["stamp"] for r in recs] == ["20261109-120459", "20261109-115106"]  # Newest first.
    assert recs[0]["tag"] == "shed"
    assert recs[0]["walks"] == ["backyard/shed"]
    assert recs[1]["tag"] == "" and recs[1]["walks"] == []


def test_lists_sites_walks_and_visits(tmp_path: Path):
    ws = make_workspace(tmp_path)
    [site] = dashboard.list_sites(ws)
    assert site["name"] == "backyard"
    assert [w["name"] for w in site["walks"]] == ["fence", "shed"]
    shed = site["walks"][1]
    assert shed["label_consistency"] == 0.84 and shed["sky_frames"] == 510
    assert shed["maps"] == ["surface.asc"]
    assert site["visits"][0]["walk"] == "shed" and site["visits"][0]["inliers_30cm"] is None
    assert site["maps"] == ["semantic.asc"]


def test_paths_in_the_site_frame(tmp_path: Path):
    ws = make_workspace(tmp_path)
    [path] = dashboard.site_paths(ws, "backyard")
    # Turned 90 degrees, shifted to (10, 20): (1, 0) -> (10, 21).
    assert path["points"] == [[10.0, 20.0], [10.0, 21.0]]


def test_grid_json(tmp_path: Path):
    ws = make_workspace(tmp_path)
    g = dashboard.grid_json(ws, "backyard", "surface.asc", "shed")
    assert (g["ncols"], g["nrows"], g["xll"], g["cell"]) == (2, 2, 10.0, 0.5)
    assert g["values"] == [1.0, None, 3.0, 4.0]
    assert (g["min"], g["max"]) == (1.0, 4.0)
    assert dashboard.grid_json(ws, "backyard", "semantic.asc", None)["ncols"] == 2


@pytest.mark.parametrize("bad", ["..", "../x", "a/b", "", ".hidden", "a\\b"])
def test_refuses_names_that_leave_the_workspace(tmp_path: Path, bad: str):
    ws = make_workspace(tmp_path)
    with pytest.raises(ValueError):
        dashboard.grid_json(ws, bad, "surface.asc", None)
    with pytest.raises(ValueError):
        dashboard.grid_json(ws, "backyard", "surface.asc", bad or "/")
    with pytest.raises(ValueError):
        ws.recording(bad + ".vgcap.mcap" if bad else "")


def test_jobs_run_one_at_a_time_and_keep_output(tmp_path: Path):
    jobs = dashboard.Jobs(tmp_path, command=[sys.executable, "-c",
                                             "import sys; print('hi', sys.argv[1:]); "
                                             "sys.exit(len(sys.argv) > 2)"])
    ok = jobs.submit("ok", ["a"])
    bad = jobs.submit("bad", ["a", "b"])
    deadline = time.time() + 20
    while time.time() < deadline and bad.state in ("queued", "running"):
        time.sleep(0.05)
    assert ok.state == "done" and ok.lines == ["hi ['a']"]
    assert bad.state == "failed" and bad.returncode == 1
    assert ok.ended <= bad.started


def test_actions_check_their_inputs(tmp_path: Path):
    ws = make_workspace(tmp_path)
    jobs = dashboard.Jobs(tmp_path, command=[sys.executable, "-c", "pass"])
    job = dashboard.start_replay(ws, jobs, "recording-20261109-115106.vgcap.mcap", "backyard",
                                 "fence-walk")
    assert job.args[0] == "replay" and job.args[2].endswith("fence-walk")
    with pytest.raises(ValueError):
        dashboard.start_replay(ws, jobs, "missing.vgcap.mcap", "backyard", "x")
    with pytest.raises(ValueError):
        dashboard.start_site_add(ws, jobs, "backyard", ["nope"], False)
    add = dashboard.start_site_add(ws, jobs, "backyard", ["fence"], True)
    assert add.args[:2] == ["site", "add"] and add.args[-1] == "--force"


def test_serves_the_page_and_api(tmp_path: Path):
    ws = make_workspace(tmp_path)
    jobs = dashboard.Jobs(tmp_path, command=[sys.executable, "-c", "print('done')"])
    server = ThreadingHTTPServer(("127.0.0.1", 0), dashboard.make_handler(ws, jobs))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{server.server_address[1]}"
    try:
        page = urllib.request.urlopen(base + "/").read().decode()
        assert "vg dashboard" in page
        state = json.loads(urllib.request.urlopen(base + "/api/state").read())
        assert state["sites"][0]["name"] == "backyard"
        assert state["classes"][5]["name"] == "sky"
        grid = json.loads(urllib.request.urlopen(
            base + "/api/grid?site=backyard&map=surface.asc&walk=shed").read())
        assert grid["values"][0] == 1.0
        req = urllib.request.Request(base + "/api/site/create", method="POST",
                                     data=json.dumps({"site": "front"}).encode())
        urllib.request.urlopen(req)
        assert (ws.sites / "front" / "walks").is_dir()
        with pytest.raises(urllib.error.HTTPError) as e:
            urllib.request.urlopen(base + "/api/grid?site=..&map=surface.asc")
        assert e.value.code == 400
    finally:
        server.shutdown()
        server.server_close()

import json
import os
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


def make_layers(ws: dashboard.Workspace) -> Path:
    layers = ws.site("backyard") / "layers"
    for name in ("ground", "boundary", "features", "guidance"):
        (layers / name).mkdir(parents=True)
    (layers / "ground" / "state.asc").write_text(GRID)
    (layers / "ground" / "edits.csv").write_text(
        "id,time,kind,points,note\n"
        "edit-1,2026-11-18T18:00:00Z,not_ground,1 1;2 1;2 2,leaves, in a pile\n"
        "edit-3,2026-11-18T18:05:00Z,ground,0 0;1 0;1 1\n")
    (layers / "ground" / "edit_reviews.csv").write_text(
        "id,cells,disagreeing_cells,walks\nedit-1,12,4,fill3;fill4\n")
    (layers / "boundary" / "edits.csv").write_text(
        "id,time,kind,points,note\nedit-2,2026-11-18T18:01:00Z,boundary,0 0;3 0,gate\n")
    (layers / "boundary" / "boundary.csv").write_text(
        "x,y,edge\n0,0,structure\n4,0,assumed\n4,3,ground_end\n")
    (layers / "boundary" / "walks.csv").write_text(
        "walk,closed,site,loop_m,gap_m,area_m2,median_apart_m,overlap,structure,ground_end,assumed\n"
        "b1,1,1,14.0,0.2,6.0,,,1,1,1\n")
    (layers / "features" / "elements.csv").write_text(
        "id,kind,class,status,seen,center_x,center_y,length_m,width_m,axis_deg,height_m\n"
        "1,post,structure,matched,3,1.5,2.5,0.1,0.1,0,1.2\n")
    (layers / "guidance" / "targets.csv").write_text(
        "target,order,cells,area_m2,seen,center_x,center_y,spot,spot_x,spot_y,heading_deg,"
        "distance_m,sees\n"
        "1,2,40,2.5,1.0,3,3,1,1,1,45,2.0,1.0\n"
        "2,1,8,0.5,0.0,-3,-3,,,,,,\n")
    (layers / "guidance" / "guidance.json").write_text('{"done": false, "unknown_m2": 3.0}')
    return layers


def test_site_overlays(tmp_path: Path):
    ws = make_workspace(tmp_path)
    make_layers(ws)
    o = dashboard.site_overlays(ws, "backyard")
    assert o["boundary"][1] == {"x": 4.0, "y": 0.0, "edge": "assumed"}
    [walk] = o["boundary_walks"]
    assert walk["site"] and walk["closed"] and walk["median_apart_m"] is None
    assert walk["edges"] == {"structure": 1, "ground_end": 1, "assumed": 1}
    [post] = o["features"]
    assert post["kind"] == "post" and post["seen"] == 3 and (post["x"], post["y"]) == (1.5, 2.5)
    g = o["guidance"]
    assert g["done"] is False and g["unknown_m2"] == 3.0
    assert [t["rank"] for t in g["targets"]] == [2, 1]  # In visiting order.
    assert g["targets"][0]["spots"] == []
    assert g["targets"][1]["spots"][0]["heading_deg"] == 45.0
    edits = {e["id"]: e for e in o["edits"]}
    assert edits["edit-1"]["note"] == "leaves, in a pile"  # Commas kept in notes.
    assert edits["edit-1"]["points"] == [[1.0, 1.0], [2.0, 1.0], [2.0, 2.0]]
    assert edits["edit-1"]["review"] == {"cells": 12, "disagreeing": 4, "walks": ["fill3", "fill4"]}
    assert edits["edit-3"]["note"] == "" and edits["edit-3"]["review"] is None
    assert edits["edit-2"]["layer"] == "boundary" and edits["edit-2"]["kind"] == "boundary"


def test_site_overlays_without_layers(tmp_path: Path):
    ws = make_workspace(tmp_path)
    o = dashboard.site_overlays(ws, "backyard")
    assert o["boundary"] == [] and o["features"] == [] and o["edits"] == []
    assert o["guidance"] == {"done": None, "unknown_m2": None, "targets": []}


def test_ground_layer_maps(tmp_path: Path):
    ws = make_workspace(tmp_path)
    make_layers(ws)
    [site] = dashboard.list_sites(ws)
    assert site["layer_maps"] == ["state.asc"]
    g = dashboard.grid_json(ws, "backyard", "state.asc", "@ground")
    assert g["values"] == [1.0, None, 3.0, 4.0]


def test_site_edits(tmp_path: Path):
    ws = make_workspace(tmp_path)
    layers = make_layers(ws)
    jobs = dashboard.Jobs(tmp_path, command=[sys.executable, "-c", "pass"])
    job = dashboard.start_site_edit(ws, jobs, "backyard", "not_ground",
                                    [[1, 2], [3, 4.5], [5, 6]], "  chairs\n stacked ")
    assert job.args[:2] == ["site", "edit"] and job.args[2].endswith("backyard")
    assert job.args[3:] == ["not_ground", "1.000 2.000;3.000 4.500;5.000 6.000", "chairs stacked"]
    line = dashboard.start_site_edit(ws, jobs, "backyard", "boundary", [[0, 0], [1, 1]], "")
    assert len(line.args) == 5  # No note.
    for kind, points in [("fence", [[0, 0], [1, 1], [2, 2]]), ("ground", [[0, 0], [1, 1]]),
                         ("ground", [[0, 0], [1, 1], [float("nan"), 2]])]:
        with pytest.raises(ValueError):
            dashboard.start_site_edit(ws, jobs, "backyard", kind, points, "")

    remake = dashboard.delete_site_edit(ws, jobs, "backyard", "edit-1")
    assert remake.args[:2] == ["site", "ground"]
    assert (layers / "ground" / "edits.csv").read_text().splitlines() == [
        "id,time,kind,points,note", "edit-3,2026-11-18T18:05:00Z,ground,0 0;1 0;1 1"]
    dashboard.delete_site_edit(ws, jobs, "backyard", "edit-2")
    assert (layers / "boundary" / "edits.csv").read_text() == "id,time,kind,points,note\n"
    for bad in ["edit-1", "../x", "a,b", ""]:
        with pytest.raises(ValueError):
            dashboard.delete_site_edit(ws, jobs, "backyard", bad)


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


def test_foxglove_makes_the_file_once_then_opens_it(tmp_path: Path, monkeypatch):
    ws = make_workspace(tmp_path)
    opened: list[Path] = []
    monkeypatch.setattr(dashboard.foxglove, "open_file",
                        lambda path: opened.append(path) or f"opened {path}")
    jobs = dashboard.Jobs(tmp_path, command=[sys.executable, "-c", "pass"])
    walk = ws.sites / "backyard" / "walks" / "shed"
    out = walk / "foxglove.mcap"

    started = dashboard.open_in_foxglove(ws, jobs, "backyard", "shed")
    job = jobs.get(started["id"])
    assert job.args[:3] == ["foxglove", str(ws.recordings / "recording-20261109-120459.vgcap.mcap"),
                            str(out)]
    assert job.args[3:] == ["--site", str(ws.sites / "backyard"), "--walk", str(walk)]
    deadline = time.time() + 20
    while time.time() < deadline and job.state in ("queued", "running"):
        time.sleep(0.05)
    assert job.state == "done" and opened == [out] and job.lines[-1] == f"opened {out}"

    # Made after everything it comes from: opened straight away.
    out.write_bytes(b"")
    later = time.time() + 60
    os.utime(out, (later, later))
    again = dashboard.open_in_foxglove(ws, jobs, "backyard", "shed")
    assert again == {"opened": True, "message": f"opened {out}"}

    with pytest.raises(ValueError):  # No manifest: no recording to show.
        dashboard.open_in_foxglove(ws, jobs, "backyard", "fence")


def test_foxglove_can_be_named(monkeypatch):
    from vg_tools import foxglove

    monkeypatch.setenv("VG_FOXGLOVE", "/opt/fox/foxglove")
    assert foxglove.find() == ["/opt/fox/foxglove"]
    assert foxglove.LAYOUT.is_file()
    assert "3D!vgwalk" in json.loads(foxglove.LAYOUT.read_text())["configById"]

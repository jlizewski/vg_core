"""The C API's site layers (vg_site_layers_*) read through ctypes, the way an
app would, against what the dashboard reads from the files itself
(dashboard.site_overlays): the two must agree. Skipped unless vg_core_c has
been built (vg build)."""

import ctypes as C
import math
import os
import sys
from pathlib import Path

import pytest

from vg_tools import dashboard

ROOT = Path(__file__).resolve().parents[2]


def find_library() -> Path | None:
    names = ["vg_core_c.dll"] if sys.platform == "win32" else ["libvg_core_c.so",
                                                               "libvg_core_c.dylib"]
    for preset in ("debug", "release"):
        for sub in ("", "Debug", "Release"):
            for name in names:
                path = ROOT / "build" / preset / sub / name
                if path.is_file():
                    return path
    return None


class BoundaryVertex(C.Structure):
    _fields_ = [("x", C.c_double), ("y", C.c_double), ("edge", C.c_int32), ("push", C.c_double)]


class Feature(C.Structure):
    _fields_ = [("id", C.c_char_p), ("kind", C.c_int32), ("semantic_class", C.c_int32),
                ("x", C.c_double), ("y", C.c_double), ("length", C.c_double),
                ("width", C.c_double), ("axis", C.c_double), ("height", C.c_double),
                ("seen", C.c_int32), ("missed", C.c_int32), ("retired", C.c_int32),
                ("spread", C.c_double)]


class LookSpot(C.Structure):
    _fields_ = [("x", C.c_double), ("y", C.c_double), ("heading", C.c_double),
                ("distance", C.c_double), ("sees", C.c_double)]


class Target(C.Structure):
    _fields_ = [("rank", C.c_int32), ("order", C.c_int32), ("area", C.c_double),
                ("center_x", C.c_double), ("center_y", C.c_double), ("seen", C.c_double),
                ("first_spot", C.c_size_t), ("spot_count", C.c_size_t)]


class Edit(C.Structure):
    _fields_ = [("id", C.c_char_p), ("time", C.c_char_p), ("kind", C.c_int32),
                ("points", C.POINTER(C.c_double)), ("point_count", C.c_size_t),
                ("note", C.c_char_p), ("reviewed", C.c_int32), ("cells", C.c_size_t),
                ("disagreeing_cells", C.c_size_t), ("disagreeing_walks", C.c_char_p)]


class Info(C.Structure):
    _fields_ = [("has_ground", C.c_int32), ("has_guidance", C.c_int32), ("done", C.c_int32),
                ("unknown_area", C.c_double), ("boundary_vertex_count", C.c_size_t),
                ("feature_count", C.c_size_t), ("target_count", C.c_size_t),
                ("spot_count", C.c_size_t), ("edit_count", C.c_size_t)]


EDGES = ["structure", "ground_end", "assumed", "edited"]
KINDS = ["post", "line", "block"]
EDIT_KINDS = ["ground", "not_ground", "boundary"]


def read_layers(lib, site: Path) -> dict:
    """The dashboard's overlays, from the C API alone."""
    handle = C.c_void_p()
    assert lib.vg_site_layers_open(str(site).encode(), C.byref(handle)) == 0, lib.vg_last_error()
    try:
        info = Info()
        assert lib.vg_site_layers_get_info(handle, C.byref(info)) == 0
        n = C.c_size_t()
        verts = C.POINTER(BoundaryVertex)()
        assert lib.vg_site_layers_boundary(handle, C.byref(verts), C.byref(n)) == 0
        boundary = [{"x": verts[i].x, "y": verts[i].y, "edge": EDGES[verts[i].edge]}
                    for i in range(n.value)]
        feats = C.POINTER(Feature)()
        assert lib.vg_site_layers_features(handle, C.byref(feats), C.byref(n)) == 0
        features = [{"id": feats[i].id.decode(), "kind": KINDS[feats[i].kind],
                     "seen": feats[i].seen, "x": feats[i].x, "y": feats[i].y,
                     "axis_deg": math.degrees(feats[i].axis), "length": feats[i].length,
                     "status": "retired" if feats[i].retired else "current"}
                    for i in range(n.value)]
        targets_p, spots_p = C.POINTER(Target)(), C.POINTER(LookSpot)()
        nt, ns = C.c_size_t(), C.c_size_t()
        assert lib.vg_site_layers_guidance(handle, C.byref(targets_p), C.byref(nt),
                                           C.byref(spots_p), C.byref(ns)) == 0
        targets = []
        for t in (targets_p[i] for i in range(nt.value)):
            spots = [spots_p[t.first_spot + k] for k in range(t.spot_count)]
            targets.append({"rank": t.rank, "order": t.order, "area_m2": t.area,
                            "x": t.center_x, "y": t.center_y,
                            "spots": [{"x": s.x, "y": s.y, "heading_deg": math.degrees(s.heading)}
                                      for s in spots]})
        edits_p = C.POINTER(Edit)()
        assert lib.vg_site_layers_edits(handle, C.byref(edits_p), C.byref(n)) == 0
        edits = []
        for e in (edits_p[i] for i in range(n.value)):
            edits.append({"id": e.id.decode(), "kind": EDIT_KINDS[e.kind],
                          "note": e.note.decode(),
                          "points": [[e.points[2 * k], e.points[2 * k + 1]]
                                     for k in range(e.point_count)],
                          "review": ({"cells": e.cells, "disagreeing": e.disagreeing_cells,
                                      "walks": [w for w in e.disagreeing_walks.decode().split(";")
                                                if w]}
                                     if e.reviewed else None)})
        return {"info": info, "boundary": boundary, "features": features, "targets": targets,
                "edits": edits}
    finally:
        lib.vg_site_layers_free(handle)


def load(path: Path):
    if sys.platform == "win32":
        os.add_dll_directory(str(path.parent))
    lib = C.CDLL(str(path))
    lib.vg_last_error.restype = C.c_char_p
    lib.vg_site_layers_free.restype = None
    for name in ("vg_site_layers_open", "vg_site_layers_get_info", "vg_site_layers_boundary",
                 "vg_site_layers_features", "vg_site_layers_guidance", "vg_site_layers_edits"):
        getattr(lib, name).restype = C.c_int32
    return lib


def assert_agree(c_api: dict, overlays: dict) -> None:
    close = lambda a, b: a == pytest.approx(b, abs=1e-3)  # noqa: E731 (files keep 10 digits)
    assert len(c_api["boundary"]) == len(overlays["boundary"])
    for a, b in zip(c_api["boundary"], overlays["boundary"], strict=True):
        assert close(a["x"], b["x"]) and close(a["y"], b["y"]) and a["edge"] == b["edge"]
    assert [f["id"] for f in c_api["features"]] == [f["id"] for f in overlays["features"]]
    for a, b in zip(c_api["features"], overlays["features"], strict=True):
        assert (a["kind"], a["seen"], a["status"]) == (b["kind"], b["seen"], b["status"])
        assert close(a["x"], b["x"]) and close(a["axis_deg"], b["axis_deg"])
    guidance = overlays["guidance"]
    assert [t["rank"] for t in c_api["targets"]] == [t["rank"] for t in guidance["targets"]]
    for a, b in zip(c_api["targets"], guidance["targets"], strict=True):
        assert a["order"] == b["order"] and close(a["area_m2"], b["area_m2"])
        assert len(a["spots"]) == len(b["spots"])
        for sa, sb in zip(a["spots"], b["spots"], strict=True):
            assert close(sa["x"], sb["x"]) and close(sa["heading_deg"], sb["heading_deg"])
    if guidance["done"] is not None:
        assert bool(c_api["info"].done) == guidance["done"]
        assert close(c_api["info"].unknown_area, guidance["unknown_m2"])
    assert [e["id"] for e in c_api["edits"]] == [e["id"] for e in overlays["edits"]]
    for a, b in zip(c_api["edits"], overlays["edits"], strict=True):
        assert (a["kind"], a["note"], a["review"]) == (b["kind"], b["note"], b["review"])
        flat = [v for p in b["points"] for v in p]
        assert [v for p in a["points"] for v in p] == pytest.approx(flat)


def write_site(site: Path) -> None:
    """A site's layer files as vg site ground writes them (columns and all)."""
    layers = site / "layers"
    for name in ("ground", "boundary", "features", "guidance"):
        (layers / name).mkdir(parents=True)
    grid = "ncols 2\nnrows 1\nxllcorner 0\nyllcorner 0\ncellsize 0.5\nNODATA_value -9999\n1 4\n"
    for name in ("ground.asc", "state.asc"):
        (layers / "ground" / name).write_text(grid)
    (layers / "ground" / "edits.csv").write_text(
        "id,time,kind,polygon,note\n"
        "edit-1,2026-11-18T18:00:00Z,not_ground,1 1;2 1;2 2,leaves, in a pile\n")
    (layers / "ground" / "edit_reviews.csv").write_text(
        "id,kind,cells,disagreeing_cells,walks\nedit-1,not_ground,12,4,fill3;fill4\n")
    (layers / "boundary" / "edits.csv").write_text(
        "id,time,kind,polygon,note\nedit-2,2026-11-18T18:01:00Z,boundary,0 0;3 0,gate\n")
    (layers / "boundary" / "boundary.csv").write_text(
        "x,y,edge,push_m\n0,0,structure,1.2\n4,0,assumed,1\n4,3,edited,0\n")
    (layers / "features" / "elements.csv").write_text(
        "id,kind,class,center_x,center_y,length_m,width_m,axis_deg,height_m,seen,missed,"
        "first_seen,last_seen,status,spread_m\n"
        "post-1,post,structure,1.5,2.5,0.1,0.1,0,1.2,3,0,20261118-122828,20261118-123800,"
        "current,0.08\n"
        "line-1,line,structure,0,1.5,3,0.2,90,1.1,1,2,20261118-122828,20261118-122828,"
        "retired,0\n")
    (layers / "guidance" / "targets.csv").write_text(
        "target,order,area_m2,cells,center_x,center_y,seen,spot,spot_x,spot_y,heading_deg,"
        "distance_m,sees\n"
        "1,2,2.5,40,3,3,1,1,1,1,45,2,0.6\n"
        "1,2,2.5,40,3,3,1,2,5,1,135,2,0.4\n"
        "2,1,0.5,8,-3,-3,0,,,,,,\n")
    (layers / "guidance" / "guidance.json").write_text(
        '{\n  "done": false,\n  "targets": 2,\n  "spots": 2,\n  "unknown_m2": 3\n}\n')


@pytest.fixture(scope="module")
def lib():
    path = find_library()
    if path is None:
        pytest.skip("vg_core_c isn't built (vg build)")
    return load(path)


def test_the_c_api_reads_what_the_dashboard_shows(tmp_path: Path, lib):
    sites = tmp_path / "sites"
    write_site(sites / "garden")
    ws = dashboard.Workspace(tmp_path, sites, tmp_path)
    c_api = read_layers(lib, sites / "garden")
    assert c_api["info"].has_ground == 1 and c_api["info"].has_guidance == 1
    assert [t["order"] for t in c_api["targets"]] == [1, 2]  # Visiting order.
    assert c_api["features"][1]["status"] == "retired"
    assert_agree(c_api, dashboard.site_overlays(ws, "garden"))


def test_a_site_without_layers(tmp_path: Path, lib):
    c_api = read_layers(lib, tmp_path)
    assert c_api["info"].has_ground == 0 and c_api["info"].has_guidance == 0
    assert not (c_api["boundary"] or c_api["features"] or c_api["targets"] or c_api["edits"])

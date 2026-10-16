import struct
import zlib
from pathlib import Path

import pytest

from vg_tools import cli, heatmap, labels, semantics

HEADER = (
    "depth_timestamp_ns,elapsed_s,model,accelerator,rotation_deg,skipped_before,preprocess_ms,"
    "inference_ms,postprocess_ms,latency_ms,battery_c,thermal_status,thermal_headroom,"
    "unknown,ground,plant,tree,structure,sky,dynamic\n"
)


def write_csv(path: Path) -> None:
    rows = [
        "1,1.0,DEEPLABV3_MOBILENETV2,GPU,90,0,10,50,20,100,28.0,none,0.40,0.1,0.6,0.1,0.1,0.1,0,0",
        "2,2.0,DEEPLABV3_MOBILENETV2,GPU,90,2,12,60,22,120,30.5,light,0.70,0.1,0.4,0.2,0.1,0.1,0.1,0",
        "3,3.0,DEEPLABV3_MOBILENETV2,GPU,90,1,14,70,24,140,30.0,light,0.60,0.1,0.5,0.1,0.1,0.1,0,0.1",
    ]
    path.write_text(HEADER + "\n".join(rows) + "\n")


def test_summarizes_phone_performance(tmp_path: Path):
    csv_path = tmp_path / "rec.labels.csv"
    write_csv(csv_path)
    s = labels.summarize_perf(csv_path)
    assert (s.frames, s.skipped, s.model, s.accelerator) == (3, 3, "DEEPLABV3_MOBILENETV2", "GPU")
    assert s.duration_s == pytest.approx(2.0)
    assert s.timings_ms["inference"] == (60.0, 70.0, 70.0)
    assert s.battery_c == (28.0, 30.5, 30.0)
    assert s.thermal_statuses == ["none", "light"]
    assert s.peak_headroom == pytest.approx(0.70)
    assert s.class_share["ground"] == pytest.approx(0.5)
    text = labels.format_perf(s)
    assert "3 recorded without labels (50.0%)" in text
    assert "peak headroom 0.70" in text


def test_labels_perf_command(tmp_path: Path, capsys: pytest.CaptureFixture[str]):
    csv_path = tmp_path / "rec.labels.csv"
    write_csv(csv_path)
    assert cli.main(["labels", "perf", str(csv_path)]) == 0
    assert "DEEPLABV3_MOBILENETV2 on GPU" in capsys.readouterr().out
    assert cli.main(["labels", "perf", str(tmp_path / "missing.csv")]) == 1


def test_depth_box():
    # ARCore: a 4:3 camera image and 16:9 depth.
    assert labels.depth_box((640, 480), (160, 90), "stretch") == (0, 0, 640, 480)
    assert labels.depth_box((640, 480), (160, 90), "crop") == (0, 60, 640, 420)
    assert labels.depth_box((400, 100), (160, 90), "crop") == pytest.approx(
        (200 - 88.888, 0, 200 + 88.888, 100), abs=1e-2)
    with pytest.raises(ValueError):
        labels.depth_box((640, 480), (160, 90), "zoom")


def test_semantic_map_renders_in_class_colors(tmp_path: Path):
    (tmp_path / "semantic.asc").write_text(
        "ncols 3\nnrows 1\nxllcorner 0\nyllcorner 0\ncellsize 0.15\nNODATA_value -9999\n"
        "1 4 -9999\n")
    (png,) = heatmap.write_previews(tmp_path, scale=1)
    data = png.read_bytes()
    idat = b""
    pos = 8
    while pos < len(data):
        (length,) = struct.unpack(">I", data[pos : pos + 4])
        if data[pos + 4 : pos + 8] == b"IDAT":
            idat += data[pos + 8 : pos + 8 + length]
        pos += 12 + length
    raw = zlib.decompress(idat)[1:]  # Skip the row's filter byte.
    assert tuple(raw[0:3]) == semantics.color(1)  # ground
    assert tuple(raw[4:7]) == semantics.color(4)  # structure
    assert raw[11] == 0  # No data: transparent.


def test_panel_draws_labels_and_depth_over_the_image():
    np = pytest.importorskip("numpy")
    image_module = pytest.importorskip("PIL.Image")
    image = image_module.new("RGB", (64, 48), (255, 255, 255))
    label_image = np.zeros((9, 16), dtype=np.uint8)
    label_image[:, 8:] = 4  # Right half: structure.
    depth = np.full((9, 16), 2.0, dtype=np.float32)
    panel = labels.render_panel(image, label_image, depth)
    assert panel.size == (3 * 64 + 16, 48)
    # The labels panel is blended toward the structure color on its right half.
    r, g, b = panel.getpixel((64 + 8 + 56, 10))
    assert r > g and r > b
    left = panel.getpixel((64 + 8 + 4, 10))
    assert left[0] < 255  # Unknown (gray) blended in too.

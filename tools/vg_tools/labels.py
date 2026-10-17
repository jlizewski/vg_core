"""Checking semantic labels: the phone's segmentation cost, and how the labels
line up with the camera image and depth.

    vg labels perf recording.labels.csv     speed, latency and heat while labeling
    vg labels frames recording.mcap out/    image | labels | depth panels per frame
    vg labels compare a.mcap b.mcap         how two labelings of a recording differ

`perf` reads the CSV the app writes beside each recording and needs nothing
extra; `frames` reads the MCAP and needs the analysis extra
(pip install -e "tools[analysis]").
"""

from __future__ import annotations

import csv
import statistics
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

from vg_tools.semantics import CLASSES

# ---------------------------------------------------------------------------
# perf: the app's labels.csv


@dataclass
class PerfSummary:
    frames: int
    model: str
    accelerator: str
    skipped: int  # Depth images recorded without labels (the worker was busy).
    duration_s: float
    timings_ms: dict[str, tuple[float, float, float]]  # name: (median, p90, max)
    battery_c: tuple[float, float, float]  # first, max, last
    thermal_statuses: list[str]  # In the order first seen.
    peak_headroom: float | None  # Highest thermal headroom; 1.0 = throttling.
    class_share: dict[str, float]  # Mean share of depth pixels per class.


def _percentile(values: Sequence[float], q: float) -> float:
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(q * (len(ordered) - 1) + 0.5))]


def summarize_perf(path: Path) -> PerfSummary:
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"{path}: no labeled frames")

    def column(name: str) -> list[float]:
        return [float(r[name]) for r in rows if r.get(name) not in (None, "")]

    timings = {}
    for name in ("preprocess_ms", "inference_ms", "postprocess_ms", "latency_ms"):
        values = column(name)
        if values:
            timings[name.removesuffix("_ms")] = (
                statistics.median(values), _percentile(values, 0.9), max(values))
    battery = column("battery_c")
    statuses: list[str] = []
    for r in rows:
        if r.get("thermal_status") and r["thermal_status"] not in statuses:
            statuses.append(r["thermal_status"])
    headroom = [h for h in column("thermal_headroom") if h == h]  # Drop NaN.
    elapsed = column("elapsed_s")
    return PerfSummary(
        frames=len(rows),
        model=rows[0].get("model", "?"),
        accelerator=rows[0].get("accelerator", "?"),
        skipped=int(sum(column("skipped_before"))),
        duration_s=(max(elapsed) - min(elapsed)) if elapsed else 0.0,
        timings_ms=timings,
        battery_c=(battery[0], max(battery), battery[-1]) if battery else (0.0, 0.0, 0.0),
        thermal_statuses=statuses,
        peak_headroom=max(headroom) if headroom else None,
        class_share={name: statistics.fmean(column(name)) for name, _ in CLASSES
                     if column(name)},
    )


def format_perf(s: PerfSummary) -> str:
    total = s.frames + s.skipped
    lines = [
        f"{s.model} on {s.accelerator}: {s.frames} depth images labeled over "
        f"{s.duration_s:.0f} s ({s.frames / s.duration_s if s.duration_s else 0:.1f} Hz), "
        f"{s.skipped} recorded without labels ({100 * s.skipped / total if total else 0:.1f}%)",
        "",
        f"  {'ms':<12}{'median':>8}{'p90':>8}{'max':>8}",
    ]
    for name, (median, p90, worst) in s.timings_ms.items():
        lines.append(f"  {name:<12}{median:>8.1f}{p90:>8.1f}{worst:>8.1f}")
    first, peak, last = s.battery_c
    lines += [
        "",
        f"  battery {first:.1f} C at start, {peak:.1f} C peak, {last:.1f} C at end",
        f"  thermal status: {' -> '.join(s.thermal_statuses) or 'not logged'}"
        + (f"; peak headroom {s.peak_headroom:.2f} (1.0 = throttling)"
           if s.peak_headroom is not None else ""),
        "",
        "  mean share of depth pixels: "
        + ", ".join(f"{name} {100 * share:.1f}%" for name, share in s.class_share.items()),
    ]
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# frames: labels and depth over the camera image


def depth_box(image_size: tuple[int, int], depth_size: tuple[int, int], fov: str):
    """The part of the camera image the depth image covers, as (left, top,
    right, bottom) pixels: all of it ("stretch", as the recorded depth
    intrinsics say), or a centered crop of the depth's aspect ratio ("crop")."""
    w, h = image_size
    if fov == "stretch":
        return (0, 0, w, h)
    if fov != "crop":
        raise ValueError(f"unknown depth field of view {fov!r} (stretch or crop)")
    dw, dh = depth_size
    if w / h > dw / dh:  # Image wider than depth: crop the sides.
        cw = h * dw / dh
        return ((w - cw) / 2, 0, (w + cw) / 2, h)
    ch = w * dh / dw  # Image taller than depth: crop top and bottom.
    return (0, (h - ch) / 2, w, (h + ch) / 2)


def _colorize_labels(labels):
    np = __import__("numpy")
    palette = np.array([color for _, color in CLASSES], dtype=np.uint8)
    return palette[np.minimum(labels, len(CLASSES) - 1)]


def _colorize_depth(depth, max_depth: float = 5.0):
    np = __import__("numpy")
    t = np.clip(depth / max_depth, 0.0, 1.0)
    rgb = np.stack([255 * (1 - t), 255 * (1 - abs(t - 0.5) * 2), 255 * t], axis=-1)
    rgb[~(depth > 0)] = 0
    return rgb.astype(np.uint8)


def render_panel(image, labels, depth, fov: str = "stretch"):
    """One frame as three panels side by side: the camera image; the labels
    blended over it; the depth (near red, far blue, none black) over it. Edges
    in the image, labels and depth should line up."""
    from PIL import Image

    np = __import__("numpy")
    box = depth_box(image.size, (labels.shape[1], labels.shape[0]), fov)
    size = (round(box[2] - box[0]), round(box[3] - box[1]))
    offset = (round(box[0]), round(box[1]))

    def overlay(rgb, alpha):
        layer = Image.fromarray(rgb).resize(size, Image.NEAREST)
        out = image.copy()
        region = out.crop((*offset, offset[0] + size[0], offset[1] + size[1]))
        out.paste(Image.blend(region, layer, alpha), offset)
        return out

    panels = [image, overlay(_colorize_labels(labels), 0.5),
              overlay(_colorize_depth(depth), 0.6)]
    w, h = image.size
    canvas = Image.new("RGB", (3 * w + 16, h), (255, 255, 255))
    for i, panel in enumerate(panels):
        canvas.paste(panel, (i * (w + 8), 0))
    # A legend strip of the class colors along the bottom of the labels panel.
    swatch = w // len(CLASSES)
    legend = np.zeros((10, swatch * len(CLASSES), 3), dtype=np.uint8)
    for i, (_, color) in enumerate(CLASSES):
        legend[:, i * swatch:(i + 1) * swatch] = color
    canvas.paste(Image.fromarray(legend), (w + 8, h - 10))
    return canvas


@dataclass
class Frame:
    time: int
    image: object  # PIL image
    labels: object  # numpy uint8
    depth: object  # numpy float32, meters


def labeled_frames(path: Path, times_s: Sequence[float] | None = None, count: int = 8):
    """Labeled depth frames with the camera image nearest in time: at the given
    times (seconds from the first depth frame), or count evenly spread ones."""
    from vg_tools import recording

    depth_topic, label_topic, image_topic = (
        "/cam/rgb/depth", "/cam/rgb/depth/labels", "/cam/rgb/image")
    depth: dict[int, object] = {}
    labels: dict[int, object] = {}
    images: list[tuple[int, object]] = []
    for m in recording.messages(path, [depth_topic, label_topic, image_topic]):
        if m.topic == depth_topic:
            depth[m.time] = m.proto
        elif m.topic == label_topic:
            labels[m.time] = m.proto
        else:
            images.append((m.time, m.proto))
    times = sorted(t for t in labels if t in depth)
    if not times or not images:
        raise ValueError(f"{path}: no labeled depth frames with camera images")
    images.sort(key=lambda item: item[0])
    image_times = [t for t, _ in images]

    start = times[0]
    if times_s:
        wanted = [start + int(s * 1e9) for s in times_s]
        chosen = [min(times, key=lambda t, w=w: abs(t - w)) for w in wanted]
    else:
        step = max(1, len(times) // max(1, count))
        chosen = times[step // 2::step][:count]

    import bisect

    for t in chosen:
        i = bisect.bisect_left(image_times, t)
        j = min((k for k in (i - 1, i) if 0 <= k < len(images)),
                key=lambda k: abs(image_times[k] - t))
        yield Frame(t - start, recording.compressed_image(images[j][1]),
                    recording.raw_image_array(labels[t]), recording.raw_image_array(depth[t]))


def write_frames(path: Path, out_dir: Path, times_s: Sequence[float] | None = None,
                 count: int = 8, fov: str = "stretch") -> list[Path]:
    out_dir.mkdir(parents=True, exist_ok=True)
    written = []
    for frame in labeled_frames(path, times_s, count):
        png = out_dir / f"frame_{frame.time / 1e9:07.2f}s_{fov}.png"
        render_panel(frame.image, frame.labels, frame.depth, fov).save(png)
        written.append(png)
    return written


# ---------------------------------------------------------------------------
# compare: two labelings of the same recording


@dataclass
class Comparison:
    frames: int  # Depth images labeled in both.
    pixels: int  # Depth pixels compared.
    agree: int
    # confusion[a][b]: pixels labeled class a in A and class b in B.
    confusion: list[list[int]]

    def agreement(self) -> float:
        return self.agree / self.pixels if self.pixels else 0.0

    def class_iou(self, c: int) -> float | None:
        """Intersection over union of class c between A and B (None if neither has it)."""
        both = self.confusion[c][c]
        a = sum(self.confusion[c])
        b = sum(row[c] for row in self.confusion)
        union = a + b - both
        return both / union if union else None


def read_labels(path: Path) -> dict[int, object]:
    """Each labeled depth image's labels (numpy uint8), by timestamp."""
    from vg_tools import recording

    return {m.time: recording.raw_image_array(m.proto)
            for m in recording.messages(path, ["/cam/rgb/depth/labels"])}


def compare_labels(a: dict[int, object], b: dict[int, object]) -> Comparison:
    np = __import__("numpy")
    n = len(CLASSES)
    confusion = np.zeros((n, n), dtype=np.int64)
    frames = 0
    for t in sorted(a.keys() & b.keys()):
        la, lb = a[t], b[t]
        if la.shape != lb.shape:
            continue
        frames += 1
        pairs = np.minimum(la, n - 1).astype(np.int64) * n + np.minimum(lb, n - 1)
        confusion += np.bincount(pairs.ravel(), minlength=n * n).reshape(n, n)
    pixels = int(confusion.sum())
    return Comparison(frames, pixels, int(np.trace(confusion)), confusion.tolist())


def format_comparison(c: Comparison, name_a: str = "A", name_b: str = "B") -> str:
    names = [name for name, _ in CLASSES]
    lines = [
        f"A: {name_a}",
        f"B: {name_b}",
        f"{c.frames} depth images labeled in both, {c.pixels} pixels: "
        f"{100 * c.agreement():.1f}% labeled the same",
        "",
        f"  {'class':<10}{'IoU':>7}{'A %':>9}{'B %':>9}",
    ]
    for i, name in enumerate(names):
        iou = c.class_iou(i)
        share_a = 100 * sum(c.confusion[i]) / c.pixels if c.pixels else 0.0
        share_b = 100 * sum(row[i] for row in c.confusion) / c.pixels if c.pixels else 0.0
        lines.append(f"  {name:<10}{'-' if iou is None else f'{iou:.2f}':>7}"
                     f"{share_a:>9.1f}{share_b:>9.1f}")
    lines += ["", "  % of pixels, rows A, columns B:",
              "  " + " " * 10 + "".join(f"{name[:7]:>8}" for name in names)]
    for i, name in enumerate(names):
        cells = "".join(f"{100 * v / c.pixels:>8.2f}" if c.pixels else f"{0:>8}"
                        for v in c.confusion[i])
        lines.append(f"  {name:<10}{cells}")
    return "\n".join(lines)

"""Render an ESRI ASCII grid (.asc), such as a vg_replay sun map, as a heatmap PNG.

Pure standard library: the PNG is written with zlib, so no imaging packages are
needed. Cells with no data are transparent. North (highest y) is at the top.
"""

from __future__ import annotations

import struct
import zlib
from dataclasses import dataclass
from pathlib import Path

# Inferno-like color stops: dark (least) to pale yellow (most).
STOPS = (
    (0.0, (0, 0, 4)),
    (0.25, (87, 16, 110)),
    (0.5, (188, 55, 84)),
    (0.75, (249, 142, 9)),
    (1.0, (252, 255, 164)),
)


@dataclass
class Grid:
    ncols: int
    nrows: int
    cellsize: float
    # Top row first, as in the file. None where there is no data.
    rows: list[list[float | None]]

    def values(self) -> list[float]:
        return [v for row in self.rows for v in row if v is not None]


def read_ascii_grid(path: Path) -> Grid:
    with open(path, encoding="ascii") as f:
        tokens = f.read().split()
    header: dict[str, float] = {}
    i = 0
    while i < len(tokens) and tokens[i][0].isalpha():
        header[tokens[i].lower()] = float(tokens[i + 1])
        i += 2
    try:
        ncols = int(header["ncols"])
        nrows = int(header["nrows"])
        cellsize = header["cellsize"]
    except KeyError as e:
        raise ValueError(f"{path}: missing {e.args[0]} in the grid header") from e
    nodata = header.get("nodata_value")
    data = tokens[i:]
    if len(data) != ncols * nrows:
        raise ValueError(f"{path}: expected {ncols * nrows} values, found {len(data)}")
    rows = []
    for r in range(nrows):
        row: list[float | None] = []
        for c in range(ncols):
            v = float(data[r * ncols + c])
            row.append(None if v != v or (nodata is not None and v == nodata) else v)
        rows.append(row)
    return Grid(ncols, nrows, cellsize, rows)


def color(t: float) -> tuple[int, int, int]:
    """Color for a value normalized to [0, 1]."""
    t = min(max(t, 0.0), 1.0)
    for (t0, c0), (t1, c1) in zip(STOPS, STOPS[1:], strict=False):
        if t <= t1:
            a = (t - t0) / (t1 - t0)
            r, g, b = (round(x0 + a * (x1 - x0)) for x0, x1 in zip(c0, c1, strict=True))
            return r, g, b
    return STOPS[-1][1]


def encode_png(width: int, height: int, rgba_rows: list[bytes]) -> bytes:
    def chunk(kind: bytes, payload: bytes) -> bytes:
        return (
            struct.pack(">I", len(payload))
            + kind
            + payload
            + struct.pack(">I", zlib.crc32(kind + payload) & 0xFFFFFFFF)
        )

    raw = b"".join(b"\x00" + row for row in rgba_rows)
    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(raw, 9))
        + chunk(b"IEND", b"")
    )


def render(
    grid: Grid, scale: int = 1, vmin: float | None = None, vmax: float | None = None
) -> tuple[bytes, float, float]:
    """PNG bytes of the grid as a heatmap, with the value range used."""
    values = grid.values()
    lo = vmin if vmin is not None else (min(values) if values else 0.0)
    hi = vmax if vmax is not None else (max(values) if values else 1.0)
    span = hi - lo if hi > lo else 1.0
    rows = []
    for row in grid.rows:
        line = bytearray()
        for v in row:
            px = b"\x00\x00\x00\x00" if v is None else bytes((*color((v - lo) / span), 255))
            line += px * scale
        rows.extend([bytes(line)] * scale)
    return encode_png(grid.ncols * scale, grid.nrows * scale, rows), lo, hi


def write_heatmap(
    src: Path,
    dst: Path,
    scale: int = 1,
    vmin: float | None = None,
    vmax: float | None = None,
) -> tuple[float, float]:
    png, lo, hi = render(read_ascii_grid(src), scale, vmin, vmax)
    dst.write_bytes(png)
    return lo, hi

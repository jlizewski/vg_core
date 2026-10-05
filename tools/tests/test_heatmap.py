import struct
import zlib
from pathlib import Path

import pytest

from vg_tools import cli, heatmap

GRID = """ncols 3
nrows 2
xllcorner 0
yllcorner 0
cellsize 0.05
NODATA_value -9999
0 5 -9999
10 2.5 7.5
"""


def decode_png(data: bytes) -> tuple[int, int, bytes]:
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    pos = 8
    width = height = 0
    idat = b""
    while pos < len(data):
        (length,) = struct.unpack(">I", data[pos : pos + 4])
        kind = data[pos + 4 : pos + 8]
        payload = data[pos + 8 : pos + 8 + length]
        (crc,) = struct.unpack(">I", data[pos + 8 + length : pos + 12 + length])
        assert crc == zlib.crc32(kind + payload) & 0xFFFFFFFF
        if kind == b"IHDR":
            width, height = struct.unpack(">II", payload[:8])
        elif kind == b"IDAT":
            idat += payload
        pos += 12 + length
    return width, height, zlib.decompress(idat)


def test_reads_grid(tmp_path: Path):
    path = tmp_path / "sun.asc"
    path.write_text(GRID)
    grid = heatmap.read_ascii_grid(path)
    assert (grid.ncols, grid.nrows) == (3, 2)
    assert grid.rows[0] == [0.0, 5.0, None]
    assert sorted(grid.values()) == [0.0, 2.5, 5.0, 7.5, 10.0]


def test_rejects_short_grid(tmp_path: Path):
    path = tmp_path / "bad.asc"
    path.write_text(GRID.replace("10 2.5 7.5\n", ""))
    with pytest.raises(ValueError):
        heatmap.read_ascii_grid(path)


def test_colors_run_dark_to_bright():
    assert heatmap.color(0.0) == heatmap.STOPS[0][1]
    assert heatmap.color(1.0) == heatmap.STOPS[-1][1]
    assert sum(heatmap.color(0.3)) < sum(heatmap.color(0.8))


def test_cli_writes_png(tmp_path: Path, capsys):
    src = tmp_path / "sun.asc"
    src.write_text(GRID)
    dst = tmp_path / "sun.png"
    assert cli.main(["heatmap", str(src), str(dst), "--scale", "2"]) == 0
    width, height, raw = decode_png(dst.read_bytes())
    assert (width, height) == (6, 4)
    stride = 1 + width * 4
    first_row = raw[1:stride]
    # Top-left cell is the minimum (darkest, opaque); top-right has no data.
    assert first_row[0:4] == bytes((*heatmap.STOPS[0][1], 255))
    assert first_row[-4:] == b"\x00\x00\x00\x00"
    second_grid_row = raw[2 * stride + 1 : 3 * stride]
    assert second_grid_row[0:4] == bytes((*heatmap.STOPS[-1][1], 255))
    assert "0 (dark) to 10 (bright)" in capsys.readouterr().out

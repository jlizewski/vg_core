from pathlib import Path

import pytest

from vg_tools import cli, path


def write_grid(file: Path, rows: list[list[float]], cellsize: float = 0.5) -> None:
    header = (f"ncols {len(rows[0])}\nnrows {len(rows)}\nxllcorner 0\nyllcorner 0\n"
              f"cellsize {cellsize}\nNODATA_value -9999\n")
    file.write_text(header + "\n".join(" ".join(f"{v:g}" for v in r) for r in rows) + "\n")


def make_package(dir: Path) -> Path:
    """A 4 x 2 m strip (0.5 m cells): flat ground at z = 0, except the right
    half, where the 'ground' is foliage 0.2 m up among plants and the last
    column wasn't seen (patched). A walk along y = 0.75 at z = 1.5."""
    dir.mkdir()
    ground = [[0, 0, 0, 0, 0.2, 0.2, 0.2, 0.2]] * 4
    patched = [[0, 0, 0, 0, 0, 0, 0, 1]] * 4
    semantic = [[1, 1, 1, 1, 2, 2, 2, 2]] * 4
    write_grid(dir / "ground.asc", ground)
    write_grid(dir / "ground_patched.asc", patched)
    write_grid(dir / "semantic.asc", semantic)
    write_grid(dir / "terrain.asc", [[0] * 8] * 4)
    rows = ["t,x,y,z"] + [f"{i},{0.25 + 0.5 * i},0.75,1.5" for i in range(8)]
    (dir / "trajectory.csv").write_text("\n".join(rows) + "\n")
    return dir


def test_path_stats(tmp_path: Path):
    s = path.path_stats(make_package(tmp_path / "pkg"))
    assert s.points == 8
    assert s.over_seen_ground == 7  # The last point is over patched ground.
    assert s.ground_over_1m == 0
    assert s.height_over_ground.median == pytest.approx(1.5)
    # Over the foliage the phone reads 0.2 m lower.
    assert s.by_plants["open"].median == pytest.approx(1.5)
    assert s.by_plants["overgrown"].median == pytest.approx(1.3)
    assert s.height_over_terrain.count == 8
    assert s.height_over_terrain.median == pytest.approx(1.5)
    assert len(s.gaps) == 1
    assert s.gaps[0].start_s == 7 and s.gaps[0].farthest_ground_m == pytest.approx(0.5)


def test_path_stats_command(tmp_path: Path, capsys):
    package = make_package(tmp_path / "pkg")
    assert cli.main(["path", "stats", str(package)]) == 0
    out = capsys.readouterr().out
    assert "8 path points" in out and "overgrown" in out and "over the terrain" in out
    assert cli.main(["path", "stats", str(tmp_path / "missing")]) == 1

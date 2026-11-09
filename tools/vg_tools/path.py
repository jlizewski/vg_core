"""How the walk relates to the ground, from a map package (vg path stats).

The terrain stage uses the phone's height over the ground as evidence of the
ground underfoot. This checks how well that holds on a walk:

- how steady the phone's height over the seen ground is;
- whether it reads lower where plants are around (foliage taken for ground);
- how often ground was seen under the phone, and the longest stretches
  walked without it (an overgrown path);
- with terrain.asc, the phone's height over the fitted terrain.

Reads ground.asc, ground_patched.asc, trajectory.csv and, if present,
semantic.asc and terrain.asc. Pure Python: no extras needed.
"""

from __future__ import annotations

import csv
import math
import statistics
from dataclasses import dataclass, field
from pathlib import Path

from vg_tools.heatmap import Grid, read_ascii_grid

PLANT_CLASSES = (2, 3)  # plant, tree (semantics.hpp)
# Plant share around a path point (within PLANT_RADIUS) that counts as some
# plants, and as overgrown.
SOME_PLANTS = 0.1
OVERGROWN = 0.4
PLANT_RADIUS = 1.0


@dataclass
class Spread:
    count: int
    median: float
    p10: float
    p90: float
    std: float

    @staticmethod
    def of(values: list[float]) -> Spread | None:
        if not values:
            return None
        v = sorted(values)

        def q(f: float) -> float:
            return v[int(f * (len(v) - 1))]

        return Spread(len(v), q(0.5), q(0.1), q(0.9), statistics.pstdev(v))


@dataclass
class Gap:
    """A stretch of the walk without seen ground under the phone."""

    start_s: float
    end_s: float
    walked_m: float
    # Farthest the nearest seen ground got along it (None: none within reach).
    farthest_ground_m: float | None


@dataclass
class PathStats:
    points: int
    over_seen_ground: int
    # Points with no seen ground within 1 m, and 2 m.
    ground_over_1m: int
    ground_over_2m: int
    height_over_ground: Spread | None
    # Height over the seen ground by plants around: open, some, overgrown.
    by_plants: dict[str, Spread | None] = field(default_factory=dict)
    height_over_terrain: Spread | None = None
    gaps: list[Gap] = field(default_factory=list)


def _seen(ground: Grid, patched: Grid | None, x: float, y: float) -> float | None:
    """Seen (not patched) ground height at (x, y)."""
    g = ground.at(x, y)
    if g is None or (patched is not None and patched.at(x, y) not in (None, 0.0)):
        return None
    return g


def _nearest_seen(ground: Grid, patched: Grid | None, x: float, y: float,
                  reach: float = 2.5) -> float | None:
    cs = ground.cellsize
    steps = int(reach / cs)
    best = None
    for radius in range(steps + 1):
        for i in range(-radius, radius + 1):
            for j in range(-radius, radius + 1):
                if max(abs(i), abs(j)) != radius:
                    continue
                if _seen(ground, patched, x + i * cs, y + j * cs) is not None:
                    d = math.hypot(i, j) * cs
                    best = d if best is None else min(best, d)
        if best is not None and best <= radius * cs:
            return best
    return best


def _plant_share(semantic: Grid, x: float, y: float) -> float | None:
    cs = semantic.cellsize
    steps = int(PLANT_RADIUS / cs)
    n = plants = 0
    for i in range(-steps, steps + 1):
        for j in range(-steps, steps + 1):
            v = semantic.at(x + i * cs, y + j * cs)
            if v is not None:
                n += 1
                plants += int(v) in PLANT_CLASSES
    return plants / n if n else None


def read_path(path: Path) -> list[tuple[float, float, float, float]]:
    """trajectory.csv as (t, x, y, z) rows."""
    with open(path, newline="") as f:
        return [tuple(float(r[k]) for k in ("t", "x", "y", "z")) for r in csv.DictReader(f)]


def path_stats(package: Path, gaps: int = 3) -> PathStats:
    ground = read_ascii_grid(package / "ground.asc")
    patched_path = package / "ground_patched.asc"
    patched = read_ascii_grid(patched_path) if patched_path.is_file() else None
    semantic_path = package / "semantic.asc"
    semantic = read_ascii_grid(semantic_path) if semantic_path.is_file() else None
    terrain_path = package / "terrain.asc"
    terrain = read_ascii_grid(terrain_path) if terrain_path.is_file() else None
    walk = read_path(package / "trajectory.csv")

    heights: list[float] = []
    by_plants: dict[str, list[float]] = {"open": [], "some": [], "overgrown": []}
    over_terrain: list[float] = []
    over_1m = over_2m = 0
    status = []  # (t, x, y, seen underfoot, nearest seen ground)
    for t, x, y, z in walk:
        g = _seen(ground, patched, x, y)
        near = 0.0 if g is not None else _nearest_seen(ground, patched, x, y)
        status.append((t, x, y, g is not None, near))
        if near is None or near > 1.0:
            over_1m += 1
        if near is None or near > 2.0:
            over_2m += 1
        if terrain is not None and (tz := terrain.at(x, y)) is not None:
            over_terrain.append(z - tz)
        if g is None:
            continue
        heights.append(z - g)
        if semantic is not None and (share := _plant_share(semantic, x, y)) is not None:
            key = "open" if share < SOME_PLANTS else "some" if share < OVERGROWN else "overgrown"
            by_plants[key].append(z - g)

    # Stretches without seen ground underfoot, longest first.
    stretches: list[Gap] = []
    start = None
    for i, s in enumerate([*status, (None, None, None, True, 0.0)]):
        if not s[3] and start is None:
            start = i
        elif s[3] and start is not None:
            part = status[start:i]
            walked = sum(math.hypot(b[1] - a[1], b[2] - a[2])
                         for a, b in zip(part, part[1:], strict=False))
            nears = [p[4] for p in part]
            farthest = None if any(n is None for n in nears) else max(nears)
            stretches.append(Gap(part[0][0], part[-1][0], walked, farthest))
            start = None
    stretches.sort(key=lambda g: g.end_s - g.start_s, reverse=True)

    return PathStats(
        points=len(walk),
        over_seen_ground=len(heights),
        ground_over_1m=over_1m,
        ground_over_2m=over_2m,
        height_over_ground=Spread.of(heights),
        by_plants={k: Spread.of(v) for k, v in by_plants.items()} if semantic else {},
        height_over_terrain=Spread.of(over_terrain),
        gaps=stretches[:gaps],
    )


def _spread(s: Spread) -> str:
    return (f"median {s.median:.2f} m, 10-90% {s.p10:.2f}-{s.p90:.2f} m, std {s.std:.2f} m "
            f"({s.count} points)")


def format_path_stats(s: PathStats) -> str:
    def pct(n: int) -> str:
        return f"{100 * n / s.points:.0f}%" if s.points else "-"

    lines = [
        f"{s.points} path points; seen ground right under the phone at {pct(s.over_seen_ground)}; "
        f"nearest seen ground over 1 m away at {pct(s.ground_over_1m)}, over 2 m at "
        f"{pct(s.ground_over_2m)}",
    ]
    if s.height_over_ground:
        lines.append(f"phone height over seen ground: {_spread(s.height_over_ground)}")
    if s.by_plants:
        lines.append(f"... by plants (plant or tree) within {PLANT_RADIUS:g} m; lower where "
                     "plants are around suggests foliage taken for ground:")
        names = {"open": f"under {SOME_PLANTS:.0%}", "some": f"{SOME_PLANTS:.0%}-{OVERGROWN:.0%}",
                 "overgrown": f"over {OVERGROWN:.0%}"}
        for key, spread in s.by_plants.items():
            text = _spread(spread) if spread else "no points"
            lines.append(f"  {key:9s} ({names[key]:>7s}): {text}")
    if s.height_over_terrain:
        lines.append(f"phone height over the terrain: {_spread(s.height_over_terrain)}")
    if s.gaps:
        lines.append("longest stretches without seen ground underfoot:")
        for g in s.gaps:
            far = ("none within 2.5 m" if g.farthest_ground_m is None
                   else f"seen ground within {g.farthest_ground_m:.1f} m")
            lines.append(f"  {g.start_s:7.1f}-{g.end_s:7.1f} s ({g.end_s - g.start_s:4.1f} s, "
                         f"{g.walked_m:4.1f} m walked): {far}")
    return "\n".join(lines)

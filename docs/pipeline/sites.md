# Sites

A walk is one map in its own frame: the tracker picks a new origin and
heading every time. A **site** lines up many walks of one yard in one frame,
so the garden can be mapped bit by bit, and its change followed over days
and seasons. It all runs on the phone, with no cloud service.

Code: [site.hpp](../../include/vg_core/site.hpp),
[registration.hpp](../../include/vg_core/registration.hpp). Command: `vg site`.
Settings: the `site` section of [config/mapping.yaml](../../config/mapping.yaml).
Usage: the README's [Sites](../../README.md#sites-many-walks-of-one-place).

## The site frame

The first walk added defines the site frame. Each later walk gets a
`MapTransform` into it: a turn about vertical (`yaw`) and a shift in x, y and
z. Four degrees of freedom are enough because every walk already knows which
way is down (gravity).

The site frame is the frame the garden model will use: beds and plantings
are drawn in site coordinates, so they stay put as walks are added (see
[design/garden-model.md](../design/garden-model.md)).

## Registration

`register_map(reference, map)` finds where a new walk sits in the site, from
the maps themselves. The reference is every visit so far, combined.

1. **Coarse search.** For each candidate heading (every 2°), rasterize the
   outlines of the features (`features.asc`: things standing at least 25 cm
   above the terrain, such as beds, posts, walls and plants) on a 30 cm
   grid, and cross-correlate them with the reference's using an FFT. The peak
   gives the best shift for that heading. Flat ground is left out because it
   matches anywhere.
2. **Height.** Line up the terrains where they overlap to get the z shift.
3. **Fine fit.** From the best 12 headings (at least 5° apart, since the
   coarse peak can be a few degrees off or a mirror image), run trimmed ICP on
   the voxels standing over the terrain, turning about z and shifting, with
   the pairing distance shrinking from 1 m to 15 cm over 30 rounds.
4. **Choose.** The fit lining up the most points wins.

### Walks from one tracking session

A walk recorded in the same tracking session as a visit already in the site
(their recordings' `tracking_frame` match, capture format 0.2; guided mapping
records each step this way) is in that visit's frame already: it's placed as
that visit, with no search, after checking that it fits there
(`evaluate_alignment()`). If it doesn't fit (the tracker jumped within the
session), it's searched for like any walk; if there's too little to judge,
the shared session is trusted. `site.use_tracking_frame: false` turns this
off.

On five guided walks of one yard (two runs, 2026-11-18), a ground fill
searched for in the usual way was accepted 2.6 m from where its session put
it (71% of its overlap within 30 cm, a weak lead over its rival); placed in
its session's frame it fit far better (84% within 30 cm, median 9.6 cm). A
ground fill from the other run fit a little better searched (68% vs 62%),
the tracker's drift over a few minutes, well within the checks.

### GPS as a hint, never the judge

When both walks have GPS, it narrows the heading search (within 3 sigmas of
the GPS heading, at least ±10°) and weighs fits that are nearly tied by how
well they agree with the GPS heading and position. It never overrules a
clearly better geometric fit. Walks minutes apart share most of their GPS
error: three walks on one day landed 2.7–4.2 m from where GPS put them.

### Refusing doubtful fits

A wrongly placed walk would mislead every later one, so a fit is refused
unless:

- it lines up at least 1.3× as many points as its best rival (another heading
  or place): `min_rival_ratio`;
- at least 20% of the new walk's points lie within 1 m of the reference (the
  maps overlap): `min_overlap`;
- at least half of those lie within 0.3 m (they line up): `min_inliers`. Right
  fits scored 0.61–0.82 on real walks; a wrong one scored 0.41.

The result says why a fit was refused. `--force` adds it anyway.

### How well it works

On three real walks of one yard, the same alignment came out with and without
GPS, with about 0.6 m and 2° of disagreement around the loop of three. That
is the walks' own drift, which one rigid transform can't remove. A sweep of a
single thing (just the shed) fits several ways from geometry alone and is
refused as ambiguous without GPS.

Matching each walk against the whole site so far means adding short walks
after a long one works best.

## The site directory

```
garden-site/
  site.csv     the visits, oldest first
  current/     the site's maps in the site frame
    surface.asc, ground.asc, terrain.asc, features.asc, semantic.asc
    visit.asc     which visit (1 = first) each cell came from
    changes.asc   surface height change since the visit before (m)
    canopy.asc, canopy_top.asc, canopy.ply
  layers/      the site's layers (design/guided-mapping.md)
    ground/       the ground from every visit, with the user's edits
```

`site.csv` lists each visit's package path (relative to the site when
inside it, so the site can be moved), when it was recorded (from the
`YYYYMMDD-HHMMSS` stamp in the recording's name), its transform, and its
fit. See [formats.md](../formats.md#site).

## Current maps

`Site::make_maps()` writes `current/`:

- **Latest view.** Each visit's maps are moved into the site frame (heights
  shifted by the visit's z offset; classes and heights above the terrain
  aren't). Each cell takes its value from the latest visit that saw it, by
  recorded time.
- **`visit.asc`** says which visit that was.
- **`changes.asc`** is the surface height change at each cell since the visit
  before the latest one that saw it: mostly growth, plus misalignment along
  walls.
- **Canopy.** Every visit's sky evidence, moved into the site frame and added
  together, then read out as in the
  [sky stage](sun-and-sky.md#canopy-the-sky-stage). Many walks see far more
  of the sky than one.

## The ground layer

`vg site ground` (`Site::make_ground_layer()`) builds the first of the
site's layers ([design/guided-mapping.md](../design/guided-mapping.md)):
the ground as an entity of its own, from every visit's ground stage outputs
(`ground_state.asc`, `ground_weight.asc`), in `layers/ground/`
([formats.md](../formats.md#layersground)).

- **State per cell**: *measured* if any walk saw ground there. Otherwise, by
  what any walk saw standing on it (`objects.csv`): *covered* under a
  structure, a tree, or anything at least `cover_min_height` (0.5 m) tall;
  *filled* under anything lower (plants). Overhangs (a canopy, an eave) and
  objects labeled ground (a stretch of ground not joined to the rest) don't
  stand on it. Cells left are *outside* if no walk knew anything of them and
  they reach the map's edge; otherwise a hole: *filled* up to
  `max_fill_area` (1 m²), else *unknown*, a place to go and look. Filled and
  covered ground runs on smoothly from the measured ground around it.
- **Where each value came from**: *measured*, *inferred* (patched), *edited*,
  and, later, *confirmed*; how many walks measured each cell, and which one
  the height came from.
- **Edits** in `edits.csv` are applied on every build, in order: an area
  marked `ground` is filled where it wasn't measured, one marked `not_ground`
  is covered; either way the ground under it is filled smoothly from the
  ground around it.
- **Aligning** walks first. Phone tracking drifts in height along a walk,
  so walks registered by a turn and a shift can still sit centimeters apart,
  or tilted by a degree or two. Each walk's ground (but the one that set the
  site frame) is moved up or down and tilted to meet the walks before it,
  where both measured the ground, leaving out cells far off the fit
  (something changed there). With little overlap it's only moved up or down;
  a narrow strip of overlap fixes no tilt across it. On the yard's three
  walks: moved 11 and 12 cm, tilted 0.5 and 1.9 degrees; the walks went from
  13 and 19 cm apart (median) to 6 and 9 cm.
- **Combining** walks that measured a cell: their median, weighted by how
  much each saw it (`ground_weight.asc`), then the mean of those within
  `agree_within` (10 cm) of it. One bad walk can't make a dip, and walks that
  agree average out their noise. `spread.asc` is how far the farthest walk is
  from the result: where walks still disagree.
- **Flicker**: on the yard, adding a newer walk changed 2 to 6 times fewer
  cells than letting the latest walk win (by more than 20 cm: 377 instead of
  2267, and 531 instead of 1159). Settings: the `ground_layer` section of
  [config/mapping.yaml](../../config/mapping.yaml); `latest_wins` and
  `align: false` give the old behavior for comparison.

The same evidence and edits always give byte-identical files. Packages made
before the ground stage wrote its per-cell outputs get them first.

## The boundary

A **boundary walk** (a recording whose session type is `boundary_walk`) is a
walk along the site's edge, back past its start. `vg site ground` turns each
into a boundary ([boundary.hpp](../../include/vg_core/boundary.hpp)), in
`layers/boundary/`:

- **The loop**: where the walk comes back to its start, having come well
  nearer to it than it had been. How far from the start that is, and how far
  the walk's overlap at the end sits from its beginning, is the tracker's
  drift round the loop.
- **Pushed out to the edge**: the walker keeps a meter or two inside the
  edge, so each point of the loop (every 0.5 m) is pushed outward across the
  ground layer to the first covered cell (a fence, a wall, a hedge) or to
  where the seen ground ends, within 3 m. With neither, it's put 1 m out and
  marked *assumed*, for the user to confirm. A running median smooths gates
  and gaps; small loops at tight corners are cut.
- The **site's boundary** is its first boundary walk's; later ones are
  compared with it. Ground layer cells outside it are *outside*.

On the yard's two boundary walks (2026-11-18, from different corners):
loops of 103 and 100 m, back within 20 and 8 cm of their starts; 365 and
361 m² inside; the two outlines 8 cm apart (median), 27 cm (90%), their
areas overlapping 96%. The edge was a structure at 41% of the points, the
end of the seen ground at 52%, assumed at 7%.

## What sites don't do yet

- Sun maps over the site's combined maps and canopy.
- A site-wide geo-reference from every walk's GPS.
- Correcting each walk's internal drift (non-rigid alignment).
- Growth per plant rather than per cell, which needs the garden model's
  plantings.

See the [roadmap](../roadmap.md).

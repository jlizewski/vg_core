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

## What sites don't do yet

- Sun maps over the site's combined maps and canopy.
- A site-wide geo-reference from every walk's GPS.
- Correcting each walk's internal drift (non-rigid alignment).
- Growth per plant rather than per cell, which needs the garden model's
  plantings.

See the [roadmap](../roadmap.md).

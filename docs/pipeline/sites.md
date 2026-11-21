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
    ground_state.asc  the ground layer's state per cell
    model.glb     the site in 3D
  layers/      the site's layers (design/guided-mapping.md)
    ground/       the ground from every visit, with the user's edits
    boundary/     the site's edge, from its boundary walks, with edits
    features/     posts, lines and blocks, matched across visits
    guidance/     where to go and look to finish the ground
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
- **The layers.** `vg site add` and `vg site maps` then remake the layers
  (`vg site ground`, below), and `ground.asc`, `ground_state.asc` and
  `model.glb` come from the ground layer rather than the latest walk.

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

- **Smoothing**: the measured ground is the median of its neighborhood
  (`smooth_cells`, 5 x 5 cells): cell-to-cell noise and lone spikes go, real
  steps stay sharp.
- **The site's model**: with a ground layer, `current/model.glb` shows its
  clean ground (by class where measured, a lighter tint where filled, dark
  where covered, **yellow over unknown holes**: places to go and look), what
  stands on it (the surface where it's over the ground and not seen as
  ground, with sides down to it), the canopy, and the boundary as a low white
  kerb. `current/ground.asc` and `current/ground_state.asc` are the layer's,
  and adding walks or remaking the maps remakes the layers too.

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

## Feature elements

`vg site ground` also finds the things on the ground a person recognizes and
relocalization can match ([features.hpp](../../include/vg_core/features.hpp)),
in `layers/features/`:

- **From each walk**: what stands through waist height (the ground stage's
  `elements.csv`, 0.6-1.4 m above the ground, so a fence is just the fence,
  without the shrubs at its foot or the canopy over it), as oriented boxes:
  a **post** (small both ways: a trunk, a post), a **line** (long and thin: a
  fence, a wall) or a **block** (the rest: a raised bed, a bin).
- **Matched** into the site's elements, oldest walk first, by position (and
  for lines, direction); an element's position is the mean of its sightings.
  A walk that saw the spot but not the element misses it; missed by
  `features.retire_after_misses` (2) walks in a row, it's kept but retired.
- `elements.csv` has every element with its sightings, misses and spread
  (its farthest sighting); `observations.csv` every sighting.

On the yard's five guided walks: 32 posts, 5 lines and 8 blocks; seen by
more than one walk, 6 posts within 10 cm (median of their farthest
sightings), 3 lines within 17 cm, 4 blocks within 13 cm.

## Ground-fill guidance

`vg site ground` also plans where to go and look to finish the ground
([guidance.hpp](../../include/vg_core/guidance.hpp)), in `layers/guidance/`:

- **Targets**: each connected region of unknown ground inside the boundary,
  largest first.
- **Spots to look from**: measured ground (somewhere to stand) 1-3 m from the
  hole, each seeing as much of it as it can in a straight line not crossing
  a covered cell (a wall, a fence, a bed); added one at a time until 90% of
  the hole is in view (at most 3), each with the way to face.
- **Order**: nearest next, from the site's origin (where its first walk
  began).
- **Done**: no unknown hole left (of at least `guidance.min_area`).

On the yard after its guided walks: 3 holes (8.1 m²), 4 spots, each hole
98-100% in view from its spots.

## Live guidance

While a ground fill is being walked, the holes close as the phone sees them
([live_guidance.hpp](../../include/vg_core/live_guidance.hpp)): a walk
recorded in the same tracking session as one of the site's walks is in the
site frame already (D17), so its live map (keyframes fused as it walks) is
read against the site's holes as it grows. A hole cell is seen when the live
map has a surface there no more than 2 m over the hole's expected ground (a
plane through the measured ground around it); a canopy higher up doesn't
count. A hole is done when 90% of it is seen. Next is the nearest spot of a
hole not yet done, with its distance and which way to turn.

The live map must keep the stages' minimum voxel weight (0.05 for depth with
confidence, as ARCore's), or faint far readings count as seeing holes.
Checked on the yard: a site of its first four walks (5 holes, 18.7 m²), with
its last ground fill played through live guidance (`vg live guidance`), then
added to the site as usual, cell by cell:

| Live map | Live: unseen at the end | Seen live, still unknown once added | Unseen live, resolved once added | Agree |
| --- | --- | --- | --- | --- |
| no minimum weight | 0.3 m² | 335 of 807 cells | 0 | 58% |
| minimum weight 0.05 | 10.1 m² (added: 8.1 m²) | 13 | 100 | 86% |

So live guidance errs the safe way: it rarely says a hole is seen that the
walk didn't really see, and adding the walk after the step settles the rest.

```sh
vg live guidance garden-site rec.vgcap.mcap     # a ground fill played as the phone would
vg live guidance garden-site rec.vgcap.mcap --every 1 --cells-out cells.csv
```

## Edits

The user's corrections win over what walks measure (D16). Each is a line in
an edits file, made with `vg site edit` (or by hand), and applied on every
build:

```sh
vg site edit garden-site not_ground "x y;x y;x y;..." the deck   # an area that isn't ground
vg site edit garden-site ground "x y;x y;x y;..." lawn           # an area that is
vg site edit garden-site boundary "x y;x y" across the gate      # where the boundary runs
vg site delete-edit garden-site edit-3                          # take one back
```

- **Areas** (`layers/ground/edits.csv`): *not ground* covers them, *ground*
  fills what wasn't measured; either way the ground beneath is filled afresh
  from the ground around.
- **Boundary lines** (`layers/boundary/edits.csv`): a line replaces the stretch
  of the boundary between the points nearest its ends (of the two ways round,
  the one nearer the line). They're placed by where they are, so they still
  apply when the boundary is made again from more walks.
- **Reviews**: walks recorded *after* an area edit that disagree with it
  (ground measured under *not ground*, something standing under *ground*)
  are counted per edit in `layers/ground/edit_reviews.csv` and reported; the
  edit stands until the user changes it.

On the yard, a boundary walk's path went round a raised bed against the
fence and came back through a gate, so its boundary dipped 2 m into the yard
there (all assumed points); one line across the gate put it back on the
fence.

In the dashboard (`vg dashboard`, the Sites tab) the site's map opens on the
ground layer's states, with the boundary (assumed points magenta, edited
stretches blue), feature elements, the guidance (numbered holes, spots and
which way to face) and each edit drawn over it. Pick *not ground*, *ground* or
*boundary* beside **Edit**, click the points on the map, and **Save edit**:
that runs `vg site edit`, which remakes the layers. The Edits card lists each
edit with its review and a **Delete** button (`vg site delete-edit`). Edit
ids are never reused: a new one is numbered after the highest so far.

Apps read the layers through the C API (`vg_site_layers_*`, built on
[site_layers.hpp](../../include/vg_core/site_layers.hpp)) and make and delete
edits with its jobs; see [c-api.md](../c-api.md#guided-mapping-a-sites-layers).

## On the phone

The whole site pipeline runs on a phone. Measured on a Pixel 10a
(2026-11-20) with `vg_replay` and `vg_site` built for Android (README,
"Building"), on yard-guided's five walks of 1.5-2.5 minutes each, replaying
each walk then adding it to the site, oldest first, as the app would after
each step of a run:

| Walk | Replay (phone / PC) | Add to the site (phone / PC) |
| --- | --- | --- |
| boundary1 | 10.1 / 4.1 s | 0.7 / 0.9 s |
| fill1 | 11.6 / 4.4 s | 1.2 / 1.3 s |
| fill2 | 8.9 / 3.2 s | 2.1 / 2.0 s |
| boundary2 | 8.4 / 3.2 s | 2.8 / 2.8 s |
| fill3 | 9.2 / 3.2 s | 2.4 / 2.8 s |

So about 10-14 s after each step on the phone (2-3 times the PC for a
replay; adding is as fast), growing slowly with the site's size. The
phone's site matched the PC's: the same placements and fits, the same
ground layer, states, features and guidance cell for cell, and the same
boundary (once ties in its running median were broken the same way on every
standard library); the surface maps differ by about 1e-6 m in a few dozen
cells.

## What sites don't do yet

- Sun maps over the site's combined maps and canopy.
- A site-wide geo-reference from every walk's GPS.
- Correcting each walk's internal drift (non-rigid alignment).
- Growth per plant rather than per cell, which needs the garden model's
  plantings.

See the [roadmap](../roadmap.md).

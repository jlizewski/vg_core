# Guided mapping: a layered site model

Status: Proposed; stage 1 built ([pipeline/sites.md](../pipeline/sites.md#the-ground-layer)) · Updated: 2026-11-18

## Summary

Today a site is built from free walks: every walk becomes one 3D map, the
derived maps come from it, and a site keeps each cell from the latest walk
that saw it. The results are noisy where it matters most (dips and holes that
come and go in the ground, blobs past depth's reach) because one generic
reconstruction is asked to capture very different things.

This design maps a site as **separate layers**, each with a shape that suits
what it is (a smooth ground surface, structures as footprints with heights,
the canopy overhead, the skyline as horizon outlines), built in a **guided
sequence**: the boundary and ground first, by walking, then skyline,
structures and canopy from guided **stations** where the user stands still
and points the camera. Each layer keeps its evidence, knows where it's
uncertain, and turns that uncertainty into guidance ("go here, look at
that"). Layers are computed as the user maps and shown as overlays to
confirm, and the user can correct any of them.

## Goals

- **Clean, consistent site models**: the same yard mapped by different
  people, or on different days, gives the same layers within stated
  uncertainty.
- **A ground layer that is the garden's base**: smooth, complete, stable
  between walks, and independent of the 3D model. The user plans the garden
  on it.
- **Guidance from uncertainty**: every step knows what's still unknown, leads
  the user to it, and knows when it's done.
- **Live**: layers update while mapping, and the user confirms them against
  the camera view.
- **User edits win**: a correction is never overwritten by a new measurement.
- **Each layer updates at its own pace**: the ground settles, structures
  persist until changed, the canopy follows the seasons.
- **Useful after each step**: a site with only its ground is already usable
  for drawing beds.

## Non-goals

- **iOS for now.** Android (ARCore) only; the design shouldn't rule ARKit
  out (D12), but doesn't use its saved maps.
- **Cloud services**, including ARCore Cloud Anchors (D2: the phone stands
  alone).
- **Plant-level modeling** (one plant's shape or growth): the garden model's
  plantings ([garden-model.md](garden-model.md)).
- **Screen design.** This defines what the app must show and ask; the
  screens are prototyped first on a computer (D13).

## Background

What exists and is reused:

- **Recordings and map packages** ([capture-format.md](../capture-format.md),
  [map_package.hpp](../../include/vg_core/map_package.hpp)): one recording
  becomes a 3D voxel map (`map.vgm`), and stages derive 2D maps from it (D8).
- **Ground and terrain stages**
  ([derived-maps.md](../pipeline/derived-maps.md)): the ground found by slope
  and labels, objects split off and labeled, holes under objects and small
  unseen holes patched smoothly; `terrain.asc` a smooth land contour and
  `features.asc` what stands on it.
- **Sky evidence and canopy**
  ([sun-and-sky.md](../pipeline/sun-and-sky.md#sky-and-canopy)): labeled rays
  through 0.5 m cells, open or blocked; the canopy's underside is measured,
  its top only bounded.
- **Sites** ([sites.md](../pipeline/sites.md)): walks registered into the
  first walk's frame (D9) by matching `features.asc` and voxels standing on
  the terrain; doubtful fits refused. Current maps: latest walk wins per cell.
- **Live mapping**: `MapBuilder` updates the height map and voxels
  incrementally as depth arrives; the C API exposes it (D11).
- **Viewing**: `Site::model()` / `model.glb`, and `vg foxglove` to replay a
  walk over the site.

What these lack, for this design: a ground that's an entity of its own
rather than a by-product of each walk; any idea of what is still unknown;
structures and the far skyline at all (beyond depth's reach); and a way to
place a short capture in the site. The last was measured (see
[Relocalization: what we measured](#relocalization-what-we-measured)).

## Design

### The mapping sequence

| Step | The user | Builds | Needs |
| --- | --- | --- | --- |
| 1. **Boundary walk** | walks the edge of the site, back to the start | the site frame, the boundary, the ground near the edges, the first features | — |
| 2. **Ground fill** | is guided to holes and unseen areas | the ground layer, complete; more features | 1 |
| 3. **Skyline** | stands at a few stations, pans around | horizon outlines | 1 |
| 4. **Structures** | stands at stations, traces rooflines and edges | structures: footprints, heights, rooflines | 2 |
| 5. **Canopy** | stands under and near trees, looks up | the canopy's underside and extent | 2 |

Steps 1–2 walk; 3–5 are **stationary**: the user stands at a station chosen
by the guidance and moves only the camera. The whole sequence runs in **one
ARCore session** when it can (see [Sessions and frames](#sessions-and-frames)),
so every step shares the frame of step 1 with nothing to relocalize. Steps can
be repeated later on their own (the canopy in another season, the ground after
re-grading) in a new session that relocalizes first.

A site is usable after step 2. Steps 3–5 improve what depends on them (sun,
shade, what's buildable) and can be done in any order after it.

### Layers

Every layer has the same parts:

| Part | What it is |
| --- | --- |
| **Shape** | how the clean layer is stored (below) |
| **Evidence** | what sessions observed, dated and kept; **never replaced** |
| **Estimator** | evidence + edits → the clean shape; holds the layer's update policy |
| **Uncertainty** | where the layer is unknown or doubtful, as a map or per element |
| **Guidance** | uncertainty → targets ("stand here, look there"), and when the step is done |

| Layer | Shape | Evidence | Update policy |
| --- | --- | --- | --- |
| **Boundary** | a polygon in the site frame | the boundary walk's path, pushed out to the edge it follows; edits | edits only, after step 1 |
| **Ground** | a height grid with a **state per cell**; a smooth mesh from it | per-walk ground heights with their quality; hole decisions; edits | each walk aligned vertically (offset + tilt) to the frame's walk, then robust over all walks (median); settles, doesn't flicker |
| **Features** | point and line elements: trunks, posts, bed edges, wall and fence lines, footprint outlines | per-walk detections | persistent; an element seen missing several times is retired |
| **Structures** | footprint polygon + height + roofline profile per structure; near or far | footprints from ground holes, roofline and edge traces from stations; edits | persistent until edited or re-traced |
| **Skyline** | horizon elevation by azimuth, per station | station panoramas (sky boundary per direction) | rarely changes; newest station data per direction |
| **Canopy** | underside height (and extent) grid over the terrain | sky evidence (open and blocked rays) | per season |

**Features are elements, not maps**, because they serve two purposes: they're
what relocalization matches against (the ground alone can't be matched, see
below), and they're the things the user recognizes ("the post by the shed").

**The ground cell states**:

| State | Meaning | Shown as |
| --- | --- | --- |
| **measured** | seen as ground by enough walks | ground |
| **filled** | a small hole or under a low object, filled smoothly from around it | ground, marked filled |
| **covered** | under a structure or other object standing on the ground; the surface runs on smoothly beneath | ground, marked covered |
| **unknown** | a big hole not yet explained: a guidance target | gap or hatched |
| **outside** | beyond the boundary | not shown |

The ground surface is continuous over *filled* and *covered* cells, so the
mesh is one smooth, clean piece, while the states keep track of what was
actually seen. A **border** between ground and a structure is the edge of a
covered region: kept as a feature (a footprint outline) even though the mesh
runs under it.

**Every value records where it came from**: *measured* (with how many
observations and when), *filled* or *inferred* (from what), *edited* (by the
user), or *confirmed* (shown to the user and accepted). The app can always
show what's real and what's assumed.

### Steps in detail

**1. Boundary walk.** The user walks the site's edge and closes the loop.

- The path closes the loop: the drift around it is measured, and corrected
  if it's large.
- The boundary polygon is the path pushed outward to the edge the user walked
  along (a fence, wall, hedge or the ground's end), shown for confirmation
  and editable.
- It decides **near or far**: what's inside is mapped in detail (structures
  get footprints); what's outside belongs to the skyline.
- Fences, walls and hedges along the edge are the first features, and the
  most useful ones for relocalizing.

**2. Ground fill.** The ground layer shows its unknown cells; guidance leads
the user to each unexplained hole (nearest first, largest first) and says
what to look at. A hole is resolved as:

- **ground**: seen as ground on a closer look → *measured*;
- **small**: under `max_hole_area`, or under a low object → *filled*;
- **structure border**: vertical surfaces at its edge in the 3D map, labels
  saying structure, an object standing on it → *covered*, and a footprint is
  proposed for step 4;
- **unresolved**: still unclear after a look → left *unknown* for the user to
  decide (ground or not), never guessed silently.

Step 2 is done when no unknown hole above a size threshold is left inside the
boundary. The existing ground and terrain stages do most of the per-walk work;
what's new is combining walks robustly, the cell states, and the hole
classification.

**3. Skyline.** At a few stations (chosen to see past the boundary in every
direction: corners, open spots), the user pans a full circle. The sky
boundary in each direction gives the horizon's elevation by azimuth. The sun
map uses it for everything outside the boundary (neighbors' trees and
buildings), which a walk never maps.

**4. Structures.** For each covered region big enough to be a structure:

- The **footprint** comes from the ground layer: the covered region's
  outline, refined by the 3D map's vertical surfaces at its edge.
- **Stations** are chosen from the footprint: about as far back as the
  structure is tall, so the roofline is in view, preferably with open sky
  behind it, from one or two sides.
- At each station the user **traces the roofline** with the camera. Its
  elevation angle at a known distance to the footprint's edge gives the
  height directly; the profile along it gives the roof's shape.
- The structure is its footprint extruded to that height, with that
  roofline, and the overlay is shown to confirm.

This replaces carving space with depth rays past depth's reach: a
structure's top is usually beyond phone depth's range (3–5 m), but its
outline against the sky is easy to see and to segment. Open and overhanging
structures (decks, pergolas, carports, trellises) don't fit footprint plus
height; they're entered by the user (a shape from a short list, sized on the
overlay) until there's a better model.

**5. Canopy.** Stations under and around trees, looking up. The existing sky
evidence becomes a guided step: guidance aims at canopy cells with too little
evidence to judge, until each tree's underside and extent are measured. The
top stays an upper bound. Repeated in leaf and out of leaf, it gives the
seasonal canopy the sun map needs.

### Sessions and frames

- **One ARCore session per mapping run.** ARCore keeps one consistent frame
  for as long as it tracks, so steps 1–5 in one run share step 1's frame: no
  relocalization, no registration error between layers. The app keeps the
  session alive between steps.
- **Every session is placed in the site frame first.** The first run defines
  it (D9). A later run, or a run whose tracking was lost, **relocalizes**:
  - a short guided walk (about 40 m) through a part of the site known to
    register well (a **localizability map**, computed by the site from its
    own walks, as in the experiment below), not a look from where the user
    happens to stand;
  - registration against the ground and the feature elements, with GPS and
    the compass (when recorded) as hints that order the candidates;
  - **confirmation by overlay**: the best candidate's layers drawn over the
    camera view; the user accepts it or switches to the next candidate.
    Matching usually narrows to 2–3 candidates; the user sees at a glance
    which one lines up.
- Sessions have a **type** (boundary walk, ground fill, skyline station,
  structure station, canopy station, relocalization), recorded in the
  capture's session info, so each kind of evidence goes to its layer.

### Live computation and confirmation

Layers are computed as the user maps, on the phone:

- The 3D map and height map already update live (`MapBuilder`).
- Each layer's estimator runs incrementally, or on a checkpoint a few times a
  second; the batch stages (ground cleanup, hole classification) need
  incremental or windowed versions.
- **Overlays** show each layer over the camera view (ground states colored,
  footprints and rooflines outlined, the horizon line drawn), so the user
  confirms what's right and spots what's wrong.
- **Confirming is evidence**: an accepted overlay marks those values
  *confirmed*, and closes guidance targets.

The C API grows to expose layer state, uncertainty and guidance targets, not
only finished maps.

### Edits

- An edit (move a boundary vertex, reclassify a hole, reshape a footprint,
  set a structure's height) is stored as its own dated evidence and has the
  **highest priority**: every recompute honors it.
- New measurements that disagree with an edit don't override it; they're
  flagged ("this was edited; new data disagrees") for the user to look at.
- Edits are kept in the site's edit log, like the garden model's (D10).

### Storage

A site directory gains a `layers/` folder, one subfolder per layer, each with
its evidence, its edits and its current clean shape (and how it was made).
Walks and map packages stay as they are (D8): they're the evidence the ground
and features come from. `current/` stays, written from the layers, so the
dashboard, `model.glb` and the sun map keep working throughout.

## Accuracy

Every layer reports its uncertainty, and every value its provenance:

- **Ground**: per-cell spread between walks, the number of observations, and
  the state; filled and covered cells say so.
- **Structures**: height from the roofline angle has an error that grows with
  distance and with the footprint's edge error; reported per structure.
- **Skyline**: angular, from the segmentation of the sky boundary.
- **Placement**: each session's registration fit and, when relocalized, how it
  was confirmed.

Segmentation quality underlies the structure, skyline and canopy steps:
structure against sky is reliable today; structure against tree and tree
against plant are not, and the current model is a stopgap
([semantics.md](../pipeline/semantics.md#before-shipping-re-evaluate-the-model)).

## Relocalization: what we measured

On 2026-11-16, short captures were registered with today's site registration
against a site built from another walk of the same yard (two walks, 11-09).
Pieces of each walk (depth fused only within the window) were registered
against a site of the other walk, against the full-walk registration as
truth; "right" is within 0.5 m and 3°.

| Piece | GPS | Accepted | Accepted and right | Accepted but wrong | Right but refused |
| --- | --- | --- | --- | --- | --- |
| 20 s (~12 m walked) | off | 1 / 14 | 1 | 0 | 1 |
| | on | 10 / 14 | 4 | 6 | 0 |
| 40 s (~27 m) | off | 3 / 13 | 3 | 0 | 4 |
| | on | 12 / 13 | 7 | 5 | 0 |
| 60 s (~40 m) | off | 7 / 12 | 7 | 0 | 1 |
| | on | 11 / 12 | 8 | 3 | 0 |

Six separate short sessions recorded minutes later (each its own ARCore
session) were all refused as ambiguous without GPS; with GPS, 5 were accepted,
and 4 of those were placed 2.5–7 m from where the GPS-free fit put them.

What it shows:

- **Flat ground can't be matched**: pieces over open lawn had nothing standing
  on the terrain to match. Relocalization needs feature elements.
- **Where matters more than how long**: some stretches of the yard registered
  at every length, others at none. Hence the localizability map.
- **Without GPS, the checks are honest**: nothing wrong was accepted; refused
  fits had genuine rivals 44–158° away fitting within 3–17%.
- **GPS's failure is the right heading in the wrong place**, 6–12 m off along
  repeating structure, and GPS weighting inflates the confidence so the checks
  don't catch it. GPS can order candidates; it can't decide.
- When a fit is right, it's accurate: median 0.1–0.2 m, under 1°.

Caveats: one yard, one day, two walks; the pieces borrowed their whole walk's
GPS heading (optimistic); pieces of walks see more than a stationary look.

## Alternatives considered

- **Keep free walks, clean up afterwards** (today's approach, with better
  combining). Simplest, and robust combining fixes the flicker; but it can't
  map what depth doesn't reach (rooflines, the skyline), has no notion of
  done, and results depend on how well the user happened to walk. Its pieces
  (robust combining, cell states) are kept, as step 2.
- **One fused 3D model of everything** (merge every walk's voxels). The most
  faithful geometry, but heavy on a phone, and still blind past depth's reach;
  a garden plan needs a clean ground and simple shapes more than dense
  geometry.
- **Relocalize from a stationary look.** Measured: not reliable with map
  matching (above). Visual place recognition from saved keyframes might be;
  it's a candidate for the relocalization stage, not a starting assumption.
- **ARCore Cloud Anchors / ARKit saved maps.** Cloud anchors break D2; ARKit's
  saved maps aren't available on Android.

## Rollout

Each stage is useful on its own, prototyped first on recordings with the
dashboard (D13), then moved to the phone.

1. **Ground layer.** Layer framework (evidence, estimator, uncertainty,
   provenance) on the ground first: robust multi-walk combining, cell states,
   hole classification, the clean ground mesh, feature elements, the boundary
   from a walk. Guidance offline: holes listed and shown on the dashboard's
   map. *Delivers*: the ground stops flickering; a clean ground to plan on.
   **Built** (2026-11-18): `vg site ground`, the dashboard's layers and edits;
   see [pipeline/sites.md](../pipeline/sites.md#the-ground-layer).
2. **Capture for guided steps** (first part specified in [guided-capture.md](guided-capture.md)). The app records session types, the
   magnetometer, and runs steps in one session (boundary walk, then stations);
   guidance targets and overlays on the phone, starting with the ground.
   *Delivers*: guided boundary and ground mapping in the app.
3. **Structures.** Footprints from covered regions, station planning,
   roofline tracing (prototyped first on existing recordings' structure-against-
   sky frames), extrusion, edits. *Delivers*: structures beyond depth's reach.
4. **Skyline and canopy.** Station panoramas → horizon outlines; guided sky
   evidence → canopy; both into a site-wide sun map. *Delivers*: sun and shade
   over the whole site, from what's outside it too.
5. **Relocalization.** Started early as a research track alongside 1–4 (it
   only blocks return visits): a localizability map, matching on feature
   elements and classes, compass prior, candidates with overlay confirmation,
   and an evaluation harness built from the experiment above. *Delivers*:
   return visits that update layers.

## Open questions

- **Loop closure**: how much drift does a boundary walk accumulate, and is
  correcting it (non-rigid) worth it, or is measuring it enough?
- **Hole classification**: which evidence decides ground vs. border reliably,
  and what size makes a hole worth a guided look?
- **Stations**: how many does each step need on a typical yard, and how are
  they chosen (coverage of unknowns vs. walking distance)?
- **Roofline tracing**: free camera sweep, or the user following a guide line
  on the screen? How is the trace tied to the footprint's edges?
- **Session length**: can one ARCore session hold a whole run (battery, heat,
  tracking loss)? What happens when tracking is lost mid-run?
- **Incremental estimators**: which ground and hole computations can run live
  on a phone, and at what rate?
- **Migration**: existing sites become a ground layer built from their walks;
  does the boundary come from their union, or a new boundary walk?

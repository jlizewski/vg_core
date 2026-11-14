# Derived maps

The stages that turn one walk's 3D map into 2D maps of the yard: what's
there, where the ground is, and what stands on it. Each stage reads the
package's files and writes its own, so any one can be re-run with new
settings (`vg map <package> <stage>`).

Code: [map_package.hpp](../../include/vg_core/map_package.hpp) (the stages),
[ground_segmentation.hpp](../../include/vg_core/ground_segmentation.hpp),
[terrain.hpp](../../include/vg_core/terrain.hpp),
[height_map.hpp](../../include/vg_core/height_map.hpp),
[diagnostics.hpp](../../include/vg_core/diagnostics.hpp).

All 2D maps share one grid: cells of `map.resolution` (6 in) in the map's
x/y plane, saved as ESRI ASCII grids (`.asc`, see [formats.md](../formats.md#ascii-grids-asc)).

## voxels

`voxels.ply`: the occupied voxels as cubes, for viewing in MeshLab, Blender
or CloudCompare. With labels, `labels.ply` shows the same voxels colored by
class. Nothing downstream reads these; they are for people.

## surface

`surface.asc`: the height of the highest surface point in each cell. Tops of
plants, beds, fences, roofs. It's what a drone would see.

## ground

`ground.asc`: the bare ground height in each cell. `objects.ply`: everything
on or over it. `ground_patched.asc`: 1 where the ground was patched rather
than seen. Function: `segment_ground()`.

Finding the ground from geometry alone:

1. **Columns.** Group the occupied voxels into vertical columns (one per
   voxel x/y) and take each column's lowest voxel: its **bottom**.
2. **Regions.** Join neighboring bottoms whose height step is no steeper than
   `max_slope` (rise over run, 1.0 = 45°) plus a small `step_tolerance`
   (3 cm). Each connected group is a candidate stretch of ground.
3. **The main ground.** The region with the most columns labeled ground (or,
   without labels, the largest).
4. **Other patches.** Regions cut off by unseen space (behind an obstacle)
   join the ground if they line up with it within `max_gap` (0.5 m), or if
   they are large on their own (`min_isolated_area`, 0.5 m², e.g. a second
   scanned area).
5. **What's left out.** The tops of raised beds, decks and canopies drop off
   steeply to the ground around them, so step 2 never joins them to it.

Labels help where geometry can't tell (see [semantics.md](semantics.md)): a
column whose bottom is labeled plant, tree, structure or dynamic isn't
ground. So a low plant isn't read as a bump in the ground, and the foot of a
wall isn't read as a ledge. Fusion also rounds the corner where a wall meets
the ground, so a column just inside a wall can bottom out a voxel or two
high; those are left out too.

Then:

- **Segments.** All the non-ground voxels are split into 26-connected groups:
  a plant, a bed, a fence, a branch. Each records whether it touches the
  ground (**grounded**; otherwise an **overhang** such as a canopy or an
  eave), its height above the ground, and the class most of its voxels have.
- **Patching.** Holes in the ground map under segments are always filled;
  unseen holes enclosed by ground are filled up to `max_hole_area` (1 m²).
  Filling solves Laplace's equation (each cell the mean of its neighbors), so
  the patch is smooth and meets the ground around it.

## terrain

`terrain.asc`: a smooth contour of the land. `features.asc`: the surface
minus the terrain, i.e. how high things stand on the land. Function:
`fit_terrain()`.

The ground map is precise where the ground was really seen, but in overgrown
spots "ground" can be the top of low foliage. The terrain fits two kinds of
evidence together:

- **Seen ground**: `ground.asc` cells that weren't patched.
- **The walked path**: the phone is carried at a fairly steady height (1.56 m
  over the ground on one measured walk, ±18 cm), so the camera's height
  minus that height is the ground underfoot. One point is rough; hundreds pin
  down the ground where it's hidden.

The terrain is a grid of nodes every 0.5 m (`node_spacing`), heights read
between them bilinearly. All node heights are solved at once as a sparse
weighted least-squares problem, with a bending penalty so the terrain
follows the lie of the land rather than every bump. The phone's height is
solved for too, from where the path crosses seen ground.

The fit starts from the path alone, then adds the seen ground with **robust
weights**, re-solved over 8 rounds: a ground cell well above the fit counts
for little (it's probably foliage), and one below it counts for more (dips
are usually real). A node's area of seen ground counts as one observation,
so a large patch of foliage can't outvote the path by its size.

The manifest records the solved phone height and how many ground cells were
discounted. `vg path stats` shows how the path relates to the ground.

`features.asc` is what registration matches between walks
([sites.md](sites.md)) and what the garden model will summarize per bed.

## semantic

`semantic.asc`: per cell, the class of the highest labeled voxel, i.e. what's
seen from above (1 ground … 6 dynamic, see
[semantics.hpp](../../include/vg_core/semantics.hpp)). Skipped when the
recording has no labels.

## layers (diagnostic)

`layers.asc`: in each voxel column, the gap between the lowest run of voxels
and the next one above it. A small gap over a wide area (a second ground a
few centimeters up) is a ghost surface, usually from a tracking jump; large
gaps are canopies and eaves. Not part of the default pipeline. Function:
`find_surface_layers()`.

## What these maps don't know yet

- **Monthly or seasonal values**: the derived maps are a snapshot of one walk;
  change over time comes from sites ([sites.md](sites.md)).
- **What the objects are** beyond the seven classes: which segment is a
  raised bed, which is a tomato. Beds will be drawn by the user first
  ([design/garden-model.md](../design/garden-model.md)).

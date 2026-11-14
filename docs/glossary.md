# Glossary

The words the code and docs use, in roughly the order you meet them.

## Recording

**Walk, session, recording, capture.** One pass around the yard with the
phone recording. Stored as one MCAP file (`.vgcap.mcap`). See
[capture-format.md](capture-format.md).

**Producer.** Whatever wrote a recording: vg_android, an iOS app, a stereo
rig, a robot.

**Raw layer / derived layer.** In a recording, raw is the sensor data itself
(images, IMU, GPS); derived is what the platform already computed from it
(depth, poses). vg_core uses the derived layer when present.

**Pose.** Where the phone is and which way it faces, as a rigid transform
(`world_from_body`). Comes from the platform's motion tracker (ARCore, ARKit).

**Tracker, VIO.** The platform's visual-inertial odometry: it tracks the
phone's motion from the camera and IMU. It drifts slowly and sometimes
corrects itself with a sudden **pose jump**.

**Depth frame.** An image where each pixel is the distance to the surface
along the camera's view axis, in meters. Phones produce it from LiDAR (some
iPhones) or from motion and the camera (ARCore Depth API). May carry a
**confidence** (0–255) and **labels** per pixel.

**Extrinsics / intrinsics.** Extrinsics: where a camera sits on the phone
body (`body_from_camera`). Intrinsics: the camera's focal lengths and
principal point, in pixels.

## Frames

**World frame.** The tracker's frame for one walk: z up (gravity aligned),
origin and heading arbitrary. Each walk has its own.

**Site frame.** The shared frame of a site: the first walk's world frame.
Later walks are moved into it by registration.

**ENU.** East-North-Up: a local frame on the Earth's surface. The
**geo-reference** relates a world frame to ENU (and so to latitude and
longitude).

**Heading.** The rotation about up between the world frame and ENU, found by
comparing the GPS track with the pose track.

## The 3D map

**Voxel.** A small cube of space (by default half the 2D cell size, about
7.6 cm). The 3D map is a sparse grid of voxels.

**Block.** 8×8×8 voxels, the unit the 3D map allocates and updates.

**TSDF.** Truncated signed distance field: each voxel stores how far it is
from the nearest observed surface (positive in front, negative behind,
clamped at the **truncation distance**) and a **weight** (how much evidence).
Averaging many depth frames into it cancels noise. See
[pipeline/building.md](pipeline/building.md).

**Fuse, integrate.** Add a depth frame's evidence to the TSDF.

**Occupied voxels.** The voxels just behind each observed surface: the solid
shell of everything seen. This is "the voxel map".

**Keyframe.** A depth frame taken after the phone has moved or turned enough
since the last one fused. With keyframing on, only keyframes are fused live;
the rest are **deferred** to a **spool** and fused after recording stops.

**Carving.** Using each depth ray to also mark the space it passes through as
empty, so stray readings fade out.

**Speck.** A tiny group of occupied voxels connected to nothing else, usually
noise; removed before the derived stages.

**Ghost surface.** A doubled copy of a surface (a second ground a few cm up),
usually from a pose jump.

## Derived maps

**Cell.** One square of a 2D map, 0.1524 m (6 inches) by default.

**Height map.** A 2D grid of heights, one per cell (`HeightMap`). Saved as an
ESRI ASCII grid (`.asc`).

**Surface.** The highest thing in each cell: the top of plants, beds, roofs.

**Ground.** The bare ground, found from the 3D map by geometry (gentle slope,
lowest in each column) and helped by labels. Where something stood on it or
hid it, the ground is **patched** by interpolation.

**Segment, object.** A connected group of non-ground voxels: a plant, a bed,
a fence. **Grounded** if it touches the ground; otherwise an **overhang**
(canopy, eave).

**Terrain.** A smooth contour of the ground fitted to the seen ground and the
walked path. Coarser than the ground map, but not fooled by low foliage.

**Features.** Surface minus terrain: how high things stand above the land.
Used to line up walks.

**Semantic class.** What a voxel or pixel is: ground, plant, tree, structure,
sky, dynamic (people, animals), or unknown. See
[pipeline/semantics.md](pipeline/semantics.md).

**Label votes.** Per voxel, how many times it was seen as each class. The
voxel's class is the one with the most votes.

## Sun and sky

**Sun hours.** Hours of direct sun a cell gets over a period (a year by
default), with shadows from everything mapped. Clear sky assumed.

**Irradiation, sun energy.** The direct solar energy reaching a cell over the
period, in kWh/m², allowing for the ground's slope and the sun's angle.

**Daylight hours.** The hours the sun was up in the period: the most sun hours
any cell could get.

**Sky evidence.** Per coarse cell overhead, how many frames saw sky through it
and how many saw it blocked beyond depth's reach. Stored as `sky.vgs`.

**Canopy.** Coarse cells overhead judged occupied from sky evidence: trees and
rooflines too far for depth. Gives `canopy.asc` (bottom) and `canopy_top.asc`
(an upper bound on the top).

## Sites

**Map package.** A directory with one walk's 3D map and every map derived
from it. See [formats.md](formats.md#map-package).

**Stage.** One step that derives maps in a package (voxels, surface, ground,
terrain, sky, sun, semantic, layers). Each can be re-run alone.

**Site.** One place (a yard) mapped over many walks. See
[pipeline/sites.md](pipeline/sites.md).

**Visit.** One walk added to a site, with the transform into the site frame.

**Registration.** Finding where a new walk's map sits in the site, from the
maps themselves (GPS only narrows the search).

**Current maps.** The site's maps where each cell comes from the latest visit
that saw it (`current/`).

**Changes.** How much the surface rose or fell in each cell since the
previous visit that saw it: growth, mostly.

## The garden (designed, not built)

See [design/garden-model.md](design/garden-model.md).

**Place.** An area the gardener grows in or cares about: an in-ground bed, a
raised bed, a container, a grow bag, a lawn area.

**Planting.** A plant (or a row or group of one kind) in a place, from
planned to removed.

**Observation.** Something recorded about a place or planting at a time: a
photo, a sensor reading, a health check, a harvest.

**Goal.** What the gardener wants from the garden ("salad greens all
summer", "attract pollinators"), for recommendations later.

**vg_data.** The separate dataset of plant knowledge: species, companions,
pests, diseases, organic remedies. See [design/plant-data.md](design/plant-data.md).

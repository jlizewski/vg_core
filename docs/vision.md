# Vision

Victory Garden (VG) helps people grow food and flowers well by knowing their
garden as precisely as a surveyor would: where the sun falls through the
year, where water collects or runs short, what is planted where, and how it
is doing. It turns sensors people already have (a phone, and later a robot or
home sensors) into that knowledge, and turns the knowledge into advice and
warnings.

## The products

vg_core is the shared engine. Three kinds of app wrap it:

| App | What it does | Runs on |
| --- | --- | --- |
| **VG phone apps** (Android, iOS) | Map and track the garden with the phone's own sensors: camera, depth, motion tracking, GPS. | The phone, fully on its own |
| **VG ROS2 package** | The same on a robot: map, track and monitor the garden on a schedule, and report to the app or VG Home. | A robot |
| **VG Home** | Software on the home network that holds the garden's maps and records, and orchestrates peripherals (valves, sensors) and robots from them. Similar in spirit to Home Assistant, with a Home Assistant integration. | A home server or hub |

All the logic lives in vg_core so the three behave the same. The apps provide
sensors, storage, a network connection, a user interface and device control.
See [architecture](architecture.md) for where the line falls.

## What vg_core provides

1. **Maps from sensor data.** A 3D map of the yard from depth and motion
   tracking, and the 2D maps derived from it: ground, terrain, what stands on
   it, the canopy overhead. Many walks of one yard line up into one site, so
   change shows over time.
2. **Environmental maps.** Sun hours and solar energy per spot over any
   period, the water a spot is likely to receive, and other factors that
   decide what grows well where.
3. **The garden itself.** Beds, containers and plots; what is planted in each;
   observations over time (photos, readings, harvests).
4. **Advice.** Which plants suit a spot, which grow well together, what is at
   risk (overwatering from forecast rain, frost, heat), and organic remedies
   for problems found.
5. **Warnings.** Hazards to the garden from the forecast or from sensors in
   the system, as events the apps turn into notifications.

## Who it's for

Hobby gardeners first: people with a yard, some beds or containers, and a
phone. They want to know where to put the tomatoes and why the lettuce bolted,
without becoming agronomists.

That audience never lowers the bar for accuracy. **vg_core computes as
precisely as the data allows and carries its uncertainty with every result.**
When the app needs to keep things simple, it leaves detail out at the point
of display; it never computes something loosely because the user is a
hobbyist. A sun map that is 40% mapped says so. A companion pairing backed only
by folklore is labeled as folklore. See [decisions](decisions.md#d3-precise-results-simplified-only-for-display).

## Priorities

In order:

1. **Map a garden with phone data.** Largely built: 3D map, ground and terrain,
   semantic labels, canopy, sun maps, and multi-walk sites. Remaining: monthly
   and seasonal sun, an iOS recorder, and running the pipeline on the phone.
2. **Garden planning and tracking.** Beds and plantings drawn by the user, tied
   to the maps: "this bed gets 5.2 h of sun in June", "this grow bag will be
   waterlogged after Thursday's rain", "these two grow well together", "this
   plant suits the sunnier end of the lawn". Later, recommendations from goals
   the user sets for the garden.
3. **Plant health detection** from images and sensors.
4. **Remedy recommendations**: how to deal with a pest or a disease. Organic
   only, US first.

The [roadmap](roadmap.md) breaks these into milestones.

## Principles

- **One engine, many hosts.** Logic is written once, in vg_core, in portable
  C++. Apps don't reimplement it.
- **Precise first, simple at the surface.** See above.
- **Works offline.** The phone app does everything on its own. VG Home adds
  coordination, not capability the phone lacks.
- **The garden belongs to the user.** Data stays on their devices and their
  home network unless they choose otherwise. No cloud service is required.
- **Knowledge is data, not code.** Plant facts, companion pairings, pests and
  remedies live in a separate dataset (vg_data) with sources for every claim.
  See [design/plant-data.md](design/plant-data.md).
- **Organic only.** Remedies are those allowed in organic growing in the US.

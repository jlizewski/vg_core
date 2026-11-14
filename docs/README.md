# vg_core documentation

These pages explain what Victory Garden is for, how vg_core is built, and how
each part of the code works, so the code can be followed without reading it
all first. The top-level [README](../README.md) is the hands-on guide (building,
running the tools); these pages are the why and the how.

## Reading order

New to the project? Read these in order:

1. [Vision](vision.md): what Victory Garden is, who it's for, and the
   priorities.
2. [Architecture](architecture.md): where vg_core sits among the apps, what
   belongs in it and what doesn't, and how data flows from a phone to a
   recommendation.
3. [Glossary](glossary.md): the words the code and docs use (walk, map
   package, site, terrain, features, ...).
4. [Code tour](code-tour.md): every module, what it does, and the order to
   read them in.
5. [Decisions](decisions.md): choices already made, and why.

## How the mapping works

The pipeline from sensor data to garden maps, one stage per page. Each page
says which headers and source files hold the code.

| Page | Covers |
| --- | --- |
| [Pipeline overview](pipeline/README.md) | The whole path, start to finish |
| [Capture](capture-format.md) | The recording format (MCAP) every source writes |
| [Building the 3D map](pipeline/building.md) | Poses, depth cleaning, the TSDF volume, keyframes, labels |
| [Derived maps](pipeline/derived-maps.md) | Surface, ground, objects, terrain, features, semantic map |
| [Sun and sky](pipeline/sun-and-sky.md) | Geo-reference, sun position, sun maps, sky evidence, canopy |
| [Sites](pipeline/sites.md) | Lining up many walks of one yard, and tracking change |
| [Semantic labels](pipeline/semantics.md) | Segmentation models, classes, label quality |
| [File formats](formats.md) | Every file in a map package and a site |

## Designs

Designs for work that hasn't been built yet. See [design/README.md](design/README.md)
for how design docs are written and reviewed.

| Design | Status |
| --- | --- |
| [Garden model](design/garden-model.md): places, plantings, observations | Proposed |
| [Plant knowledge data (vg_data)](design/plant-data.md): species, companions, pests, organic remedies | Proposed |

## Planning

- [Roadmap](roadmap.md): the priorities as milestones, and the designs each
  one still needs.

## Keeping these docs current

A change to a module updates its page here in the same commit: the pipeline
page for its stage, the [code tour](code-tour.md) entry, [formats](formats.md)
if a file changes, and [decisions](decisions.md) if it settles a question.

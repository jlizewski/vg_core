# Decisions

Choices that shape the project, with the reason for each. New entries go at
the end with the next number. An entry is never deleted; if a decision
changes, it's marked **Replaced by Dn** and the new one explains why.

Status: **Decided** (in force), **Proposed** (recommended, awaiting
agreement), **Open** (still being discussed).

---

## D1. vg_core is computation; apps own I/O

**Decided** · 2026-11-14

vg_core holds computation, data models and file formats. Apps own sensors,
networking, the user interface, scheduling and device control.
Forecasts come in as data; warnings go out as typed events; vg_core never
calls a web service or opens a valve.

*Why:* the same inputs give the same results on the phone, the robot and VG
Home, and vg_core can be tested fully offline. See
[architecture](architecture.md#the-rule).

## D2. The phone app stands alone; VG Home owns the data when present

**Decided** · 2026-11-14

The phone app does everything on its own: mapping, sites, the garden model,
advice. When VG Home is on the network, it holds the master copy of maps and
the garden; phones and robots send it recordings and edits.

*Why:* most users start with just a phone, and a garden app that fails
without a server would be a poor one. A single master keeps sync simple.
See [design/garden-model.md](design/garden-model.md#sync).

## D3. Precise results, simplified only for display

**Decided** · 2026-11-14

Hobby gardeners are the first audience, but vg_core never computes something
loosely for them. Results carry their uncertainty, coverage, age and
assumptions. The app may show less; vg_core never drops it.

*Why:* advice is only trustworthy when it's right, and knowing how sure a
result is matters as much as the result. A sun map from 40% of a bed is
reported as such. Folklore is labeled as folklore.

## D4. Organic-only remedies, US first

**Decided** · 2026-11-14

Remedies are limited to those allowed in organic growing under the USDA
National Organic Program, with cultural and mechanical methods first. US
rules and regions first; other countries later.

*Why:* the user's choice for the product. It also limits the risk of
recommending products that harm people, pollinators or soil. See
[design/plant-data.md](design/plant-data.md#remedies).

## D5. Plant knowledge in a separate repository

**Decided** · 2026-11-14

Species, companion relations, pests, diseases and remedies live in a separate
data repository, **vg_data** (public), released as a versioned SQLite file.

*Why:* different contributors, a faster release pace, a separate license,
and every app shares it. See [design/plant-data.md](design/plant-data.md).

## D6. Beds drawn by the user first, recommendations from goals later

**Decided** · 2026-11-14

Places (beds, containers, plots) are drawn by the user at first. Later, a
recommendation system will suggest layouts and plantings from goals the user
sets for the garden.

*Why:* drawing is reliable and quick to build; automatic detection would be
wrong often enough to frustrate. Goals give recommendations something to aim
at.

## D7. One capture format for every source

**Decided** · before 2026-11 (recorded here 2026-11-14)

Every source (Android, iOS, stereo rigs, robots) writes the same MCAP +
protobuf format, using Foxglove schemas where they fit. vg_core's mapping
code takes plain structs and never sees the format.

*Why:* the core needs no platform code, recordings open in Foxglove, and
every source can be replayed on a PC. See [capture-format.md](capture-format.md).

## D8. Derived maps come from the saved 3D map, never the recording

**Decided** · before 2026-11 (recorded here 2026-11-14)

The build turns a recording into `map.vgm` once. Every other stage reads only
the package's files, so it can be re-run in seconds with new settings or new
code.

*Why:* recordings are large and slow to replay; tuning a stage shouldn't
need one. See [pipeline/README.md](pipeline/README.md#stages).

## D9. The site frame is the first walk's frame

**Decided** · before 2026-11 (recorded here 2026-11-14)

A site's frame is its first walk's world frame. Later walks are registered
into it from the maps themselves; GPS narrows the search but never overrules
geometry. Doubtful fits are refused.

*Why:* positions in the site frame stay put forever, so the garden model can
store them. GPS is meters off; map registration is about half a meter.
See [pipeline/sites.md](pipeline/sites.md).

## D10. Garden data in the site frame and in SQLite

**Proposed** · 2026-11-14

Places, plantings and observations are positioned in the site frame (meters)
and stored in `garden/garden.sqlite` in the site directory, as entities plus
an ordered edit log.

*Why:* see [design/garden-model.md](design/garden-model.md#storage).

## D11. A C API for the app bindings

**Proposed** · 2026-11-14

A small, stable `extern "C"` API (`vg_core_c`) over the C++ classes, used by
Swift, Dart and other FFI callers; pybind11 for Python (VG Home); JNI and
direct C++ where they already work.

*Why:* one binding layer instead of one per platform. See
[architecture](architecture.md#bindings-planned).

## D12. Phone apps in Flutter, with native capture

**Decided** · 2026-11-14

The phone apps move from the native Android app (vg_android) to one Flutter
app, **vg_flutter** (a private repository). Capture stays native: a Kotlin
plugin on ARCore and a Swift plugin on ARKit, each writing recordings through
vg_core. The UI is Dart and reaches vg_core through the C API (D11) with
`dart:ffi`.

*Why:* iOS recordings can be tested alongside Android while features are
developed, and the planning UI (priority 2 on) is written once. Building for
iOS needs macOS with Xcode (a Mac, or a macOS CI runner), and measured depth
on iOS needs a LiDAR iPhone (Pro models).

## D13. A desktop dev interface before the phone UI

**Decided** (where it's built: **Open**) · 2026-11-14

Before building phone screens, work out on a computer how data is presented,
how maps look and how a user interacts with them, on real sites and walks.
Settled designs then go into the phone app.

*Why:* trying presentation and interaction where iteration is fast reduces
churn in the app's features and code.

Where to build it is open:

- **Flutter desktop (Windows) in vg_flutter**, calling vg_core through the C
  API. The prototyped screens become the app's screens, and the C API and FFI
  are exercised from day one. (Recommended.)
- **Extending the Python dashboard** (`vg dashboard`). Fastest to change and
  already shows sites and maps, but everything settled there is rebuilt in
  Flutter.

# TODO

The working list: what's waiting on Jacob, and what's next on the active
plan. Kept current as work lands; the long view is in
[docs/roadmap.md](docs/roadmap.md), the plan behind it in
[docs/design/guided-mapping.md](docs/design/guided-mapping.md#rollout).

Updated: 2026-11-20

## For Jacob

Things only you can do, most pressing first.

- [ ] **Run the vg_flutter phone test** (blocks 2.3). On `vg_flutter`'s
      `app/recorder-parity` branch:
  1. `flutter run` with the phone attached (vg_flutter's README, "Quick start").
     Open Recorder, allow the camera.
  2. **Next:** *Boundary walk* (allow location); record a short loop, stop.
     Then, without leaving the recorder, *Ground fill*: record, stop.
  3. Press Home, come back, record a third.
  4. `adb pull /sdcard/Android/data/com.victorygarden.vg_flutter/files/recordings`
  5. `vg mcap stats` on each `.vgcap.mcap` (PowerShell: `python -m vg_tools.cli mcap stats <file>`).
     Expect each recording's session type, the first two sharing a tracking
     frame and the third a new one, magnetometer at about 50 Hz.
  6. Also: the preview shows; the list shows types, lengths and sizes; tag,
     share and delete work; logs appear in `files/logs`.
  - Send the stats output (or the pulled folder's path) back.
- [ ] **Push and merge vg_core's `stage2/c-api-site-layers`** (2.1, 2.2, the
      Foxglove fix, the PowerShell README; it holds `stage2/c-api-capture-0.2`).
- [ ] **vg_flutter: commit your README quick start** (uncommitted on
      `app/recorder-parity`), then push the branch. The README is saved as
      UTF-16, so git shows it as binary and won't diff it; re-save it as UTF-8.
- [ ] **Send your design considerations for the Flutter app** (needed before
      2.5, the guided run screens).
- [ ] *Optional:* mark the leaf pile and the lawn chairs on `yard-guided` as
      *not ground* in the dashboard (Sites → Edit → not ground → click round
      each → Save edit). Guidance then stops sending you to them.
- [ ] *Optional:* make `vg` work directly in PowerShell: add
      `C:\Users\jacob\AppData\Local\Python\pythoncore-3.15-64\Scripts` to
      `PATH`, or use a virtual environment (README, "On Windows"; activating
      it needs `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned`, which is
      your call).
- [ ] *Optional:* `flutter doctor` reports missing Android cmdline-tools and
      unaccepted licenses; the build doesn't need them yet, Play/release
      builds will.

## Active plan: stage 2, guided capture on the phone

| # | Task | State | Done when |
| --- | --- | --- | --- |
| 2.1 | C API recorder: session type, tracking frame, magnetometer (format 0.2) | ✅ `2ac146f` | The Flutter plugin records guided steps through the C API |
| 2.2 | C API for a site's layers (ground grids, boundary, features, guidance, edits) and their jobs; `vg site delete-edit` | ✅ `eb1e196` | The dashboard's layer views could be rebuilt from it (a ctypes test does) |
| — | vg_flutter recorder parity (agent) | ✅ built, ⏳ phone test | The phone test above passes |
| 2.3 | **Next.** The site pipeline on the phone after each step (add the walk, remake the layers), timed; plus `vg_session_open` / `vg_session_info` / `vg_session_free` in the C API so the app can read any recording's session type | ⏳ waits on the phone test | After a boundary walk and a ground fill, the phone shows the same layers as the PC, in acceptable time |
| 2.4 | Live guidance during a ground fill: the walk's ground merged with the site's as you walk | | Walking to a guidance spot visibly closes that hole within seconds |
| 2.5 | Flutter screens for a guided run: boundary walk (loop closed), ground fill with holes and spots on a map, done check, edits on the phone | waits on your design notes | You can map the yard's ground on the phone without the PC |
| 2.6 | Confirmation overlay: ground states over the camera view | | You can confirm what the phone thinks is ground by looking |
| 2.7 | Wrap-up: docs; D14 for the ground steps and D17 within a run | | Docs match the code |

## Follow-ups

Small things noticed along the way, not yet scheduled.

- vg_flutter: opening Recordings from the recorder doesn't pause the AR
  session (going Home does); the whole app is portrait-only.
- vg_flutter: tagging a recording renames its files, so walks already built
  from it name the old file in their manifest.
- vg_flutter: the preview is a platform view around vg_android's GL renderer;
  if 2.6's overlay needs more, render to a Flutter texture instead.
- `recording-20261118-122823` (the false start before boundary1) has its own
  tracking frame and isn't in `yard-guided`.
- The release build is rebuilt by the first `vg` app run after C++ changes
  (several minutes after a big change); the dashboard's first job shows it.

## Later stages

From [the rollout](docs/design/guided-mapping.md#rollout): 3 structures
(footprints, stations, roofline tracing), 4 skyline and canopy, 5
relocalization (a research track alongside). D14 and D17 stay Proposed until
their stages are built.

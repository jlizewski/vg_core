# TODO

The working list: what's waiting on Jacob, and what's next on the active
plan. Kept current as work lands; the long view is in
[docs/roadmap.md](docs/roadmap.md), the plan behind it in
[docs/design/guided-mapping.md](docs/design/guided-mapping.md#rollout).

Updated: 2026-11-20

## For Jacob

Things only you can do, most pressing first.

- [x] ~~Run the vg_flutter phone test~~ (2026-11-20, at night, Pixel 10a):
      session types ✅, magnetometer at 50 Hz ✅, every message type ✅. The
      boundary walk and the first ground fill got **different** tracking
      frames because the app left the screen for ~7 s between them (logcat
      18:43:24-31: UI hidden, then background), which restarts the AR
      session; the frame logic itself is right.
- [ ] **Push vg_core's `stage2/live-guidance` first, then vg_flutter's
      `app/recorder-parity`**: vg_flutter's submodule points at vg_core
      `9b307c8`, only on your machine until you push. (vg_core's branch has
      everything since main: 2.1-2.4, the Foxglove fix, the PowerShell
      README and this TODO.)
- [ ] **Next daylight test, on the phone** (`flutter run` from
      `app/recorder-parity`), screen on, app in front the whole time:
  1. **Boundary walk** round the yard. When it stops, wait for the status
     line's "Added to the site in N s: … holes (… m²) to look at" (10-20 s).
  2. **Ground fill**, without leaving the recorder. The status line should
     say "Live guidance: the holes of the site "garden" close as you see
     them", and a panel over the camera shows the holes left, an arrow and
     distance to the next spot, and each hole's progress. Follow the arrow:
     a hole should fill up within seconds of looking at it.
  3. Afterwards: `adb pull /sdcard/Android/data/com.victorygarden.vg_flutter/files/sites`
     and the two recordings, and send me the paths (I'll compare what live
     guidance said with what adding the fill did, as for the yard).
- [ ] **vg_flutter: commit your README quick start** (uncommitted). The
      README is saved as UTF-16, so git shows it as binary and won't diff it;
      re-save it as UTF-8. It also still says the app remembers recording
      types because the C API can't read them: it can now (`vg_session_*`).
- [ ] *Note:* the 11-18 recordings moved to `vg_sites/recordings_11182026`;
      for yard-guided's Foxglove buttons start the dashboard with
      `--recordings ..\vg_sites\recordings_11182026`.
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
| — | vg_flutter recorder parity (agent), the JNI crash fix, keep the screen on while recording | ✅ `fd02a47`..`7beef6e`; shared frame to re-check outdoors | The phone test passes |
| 2.3 | The site pipeline on the phone after each step: `vg_session_*` in the C API; vg_core's apps built for Android and timed on the Pixel 10a (8-12 s replay + 1-3 s add per step; same layers as the PC once a boundary tie-break was made portable); the app adds each guided step to a site and reports it | ✅ vg_core `8620b22`, `26bd64d`; vg_flutter `27dd2fa`; ⏳ the app flow on the phone (next daylight test) | After a boundary walk and a ground fill, the phone shows the same layers as the PC, in acceptable time |
| 2.4 | Live guidance during a ground fill: holes closing as the live map sees them, the next spot and which way to turn (`LiveGuidance`, `vg_live_guidance_*`, `vg live guidance`); on the phone, natively on the recorded frames, with a panel over the camera | ✅ vg_core `808b797`, `9b307c8`; vg_flutter `3339c1a`; ⏳ the daylight test | Walking to a guidance spot visibly closes that hole within seconds |
| 2.5 | **Next.** Flutter screens for a guided run: boundary walk (loop closed), ground fill with holes and spots on a map, done check, edits on the phone | waits on your design notes | You can map the yard's ground on the phone without the PC |
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
- Night recordings (2026-11-20): depth confidence mostly under 64/255,
  tracking limited by light several times, a 2.6 m pose jump on recovery.
  Fine for testing the app; not for mapping. Worth knowing for 2.4: guidance
  should say when it's too dark to map.
- Does ARCore keep its world frame across a short pause? Its logs mention
  pause/resume handling. If it does, the app could keep the tracking frame
  over brief pauses instead of always making a new one; needs a test
  (record, pause a few seconds, record the same spot, compare poses).
- Live guidance is cautious: on the yard it ended with 10.1 m² unseen where
  adding the fill left 8.1 m² (it misses some cells the full pipeline
  resolves from every frame, not just keyframes). Fine for guidance; could
  be tuned with phone data.
- Live guidance can't start for a ground fill until the boundary walk (or
  another walk of the run) has been added to the site; the app doesn't wait
  for it yet.
- vg_flutter: no screen to name or switch the site yet (Settings has
  `siteName`, default "garden"); part of 2.5.
- The release build is rebuilt by the first `vg` app run after C++ changes
  (several minutes after a big change); the dashboard's first job shows it.

## Later stages

From [the rollout](docs/design/guided-mapping.md#rollout): 3 structures
(footprints, stations, roofline tracing), 4 skyline and canopy, 5
relocalization (a research track alongside). D14 and D17 stay Proposed until
their stages are built.

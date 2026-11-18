# Guided capture in vg_android (stage 2, part 1)

Status: Proposed · Updated: 2026-11-18

## Summary

Guided mapping ([guided-mapping.md](guided-mapping.md)) maps a site in steps:
a boundary walk, ground fill, then stations for the skyline, structures and
canopy, each step its own recording, all in one tracking session. This is the
recorder's first part of that: make **vg_android** record which step a
recording is, tie recordings of one tracking session together, record the
magnetometer, and let the user take several steps in a row without the AR
session restarting. No guidance or overlays yet: those need vg_core work
that isn't done.

It's written to be handed to a developer (or an agent) working in vg_android.

## What vg_core already provides

Capture format **0.2** ([capture-format.md](../capture-format.md#guided-mapping-sessions)):

- `vg::io::SessionInfo::session_type`: what the recording is for. Values in
  `vg::io::session_type` (`include/vg_core/io/session.hpp`):
  `boundary_walk`, `ground_fill`, `skyline_station`, `structure_station`,
  `canopy_station`, `relocalization`; empty for a free walk.
- `vg::io::SessionInfo::tracking_frame`: an identifier of the tracker's world
  frame, the same for every recording made while one AR session keeps
  running.
- `vg::MagnetometerSample` (`include/vg_core/sensor_data.hpp`) and
  `vg::io::SessionWriter::write(const MagnetometerSample&)`: the `/mag` topic,
  field in microtesla in the body frame, with Android's accuracy (0-3).
- `vg mcap stats <file>` prints a recording's session type, tracking frame
  and how many magnetometer samples it has: the quickest check of a new
  recording.

vg_android builds vg_core from its `vg_core` submodule; **update the submodule
to a vg_core commit with capture format 0.2** first (`kFormatVersion` is
`"0.2"` in `session.hpp`).

## Changes in vg_android

Paths are under `app/src/main/`.

1. **Choose the step.** The Recorder gets a choice of what the next recording
   is: *Free walk* (the default, today's behavior), *Boundary walk*, *Ground
   fill*, *Skyline station*, *Structure station*, *Canopy station*,
   *Relocalization*. It's remembered between recordings (preferences, like
   `PREF_RECORD_SESSION`), and shown while recording. The Recordings list
   shows each recording's type.
2. **Write it.** `SessionInfo` in `java/.../core/SessionWriter.kt` gains
   `sessionType` and `trackingFrame`; `cpp/session_writer_jni.cpp` passes
   them into `vg::io::SessionInfo` (as it does `notes`); `LiveMapper.openSession`
   fills them.
3. **Tracking frame.** Make a new identifier (a random UUID) whenever the AR
   `Session` is created or resumed (`RecorderActivity.ensureSession` /
   `onResume`), and use it for every recording until the next pause or
   restart. Recordings in a row without leaving the Recorder therefore share
   it; one after the app was paused gets a new one. Don't make a new one per
   recording.
4. **Steps in a row.** The AR session already stays up between recordings
   (`startRecording`/`stopRecording` on one `Session`); keep it that way, and
   make it quick to start the next step: after stopping, the step choice is
   right there, and starting again doesn't recreate the session. Don't pause
   the session between steps (that would change the tracking frame).
5. **Magnetometer.** In `java/.../SensorFeed.kt`, also register
   `Sensor.TYPE_MAGNETIC_FIELD` (calibrated, microtesla) at the IMU's rate or
   at least 25 Hz, with the same handler. Pass each sample to the mapper like
   `onImu` (`LiveMapper.onImu` adds the clock offset; do the same), and write
   it with a new JNI call wrapping `SessionWriter::write(const
   MagnetometerSample&)`. Use the event's `accuracy` (and
   `onAccuracyChanged`) for the accuracy. The field is in Android's sensor
   frame, the same frame as the accelerometer and gyroscope samples written
   to `/imu`: no conversion beyond what `/imu` gets (check
   `ArCoreConventions.kt` and `writeImu`; if the IMU samples are converted,
   convert these the same way). A device without a magnetometer just writes
   no `/mag`.
6. **Session data on for guided steps.** With any type other than *Free
   walk*, record the MCAP session data whatever the "Save session data"
   switch says (guided steps are useless without it); the switch keeps
   governing free walks.
7. **File names stay as they are** (`recording-YYYYMMDD-HHMMSS.*`): the type is
   in the file. The desktop tools find recordings by that pattern.

## Out of scope

- Guidance (go here, look at that), overlays on the camera feed, and
  computing the site's layers on the phone: they need vg_core to expose layer
  state and guidance targets, which comes later.
- The C API (`vg_core_c`) and vg_flutter: the Flutter app comes later (D12);
  it will get these fields through the C API then.
- Relocalizing a later run into an existing site, and any station-specific
  capture behavior (e.g. tracing a roofline): later stages.

## How to check it

- Build and run on an ARCore depth device. Record two steps in a row (e.g. a
  short *Boundary walk*, then a *Ground fill*), then background the app,
  return and record a third.
- Copy the three `.vgcap.mcap` files off the phone and run `vg mcap stats` on
  each (from a vg_core checkout: `vg mcap stats <file>`):
  - each shows its `session type`;
  - the first two show the **same** `tracking frame`, the third a different
    one;
  - each counts `magnetometer samples` at roughly the rate asked for.
- A *Free walk* with the switch off records no MCAP, as today; with it on, an
  MCAP with no session type.
- Replay (the app's **Replay** sub-app) still plays the new recordings.
- On the desktop, `vg replay <file> <dir>` still builds a map package from
  them (vg_core ignores what it doesn't use).

## Acceptance

- [ ] The step can be chosen, is remembered, and is shown while recording and
      in the Recordings list.
- [ ] Recordings carry `session_type` (empty for free walks) and a
      `tracking_frame` shared across steps in one AR session, new after a
      pause.
- [ ] `/mag` is recorded at 25 Hz or more with accuracy, on the session clock.
- [ ] Guided steps always record session data.
- [ ] The checks above pass; existing behavior for free walks is unchanged.

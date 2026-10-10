# vg capture format (v0.1, draft)

This is the input format for the vg_core map builder. A **capture session** is a
single [MCAP](https://mcap.dev) file that holds everything recorded during one
mapping walk. Every source (vg_android on ARCore, an iOS app on ARKit, a
standalone stereo + IMU + GPS rig) writes the same format, so the core never
needs platform code.

A session has two layers:

- **Raw** (required): the sensor data itself plus calibration. With only this
  layer, vg_core can in principle do all the work itself.
- **Derived** (optional): depth and poses that the platform already computed
  (ARCore, ARKit, a stereo camera SDK). When present, vg_core uses them instead
  of computing its own.

## Where this runs

The map builder has to run on device (Android, iOS) as well as on desktop
(Windows, Linux, macOS), so the file format is kept out of the mapping code:

- **`vg_core`** is the map builder. Its API takes plain C++ structs (image,
  IMU sample, GPS fix, depth, pose). It has no MCAP or protobuf dependency.
- **`vg_core_io`** is an optional module that reads and writes capture
  sessions (this format) and converts them to and from those structs. It uses
  the header-only [MCAP C++ library](https://github.com/foxglove/mcap/tree/main/cpp)
  (zstd, lz4) and the protobuf lite runtime, all of which build for the
  Android NDK and iOS.

vg_core_io provides `SessionWriter`, `SessionReader`, a `play()` function that
replays a session at its recorded speed (or scaled, or as fast as possible),
and the `vg_replay` command-line tool that rebuilds a map package (3D map,
ground, objects, sun maps) from a recording.

On a phone, the app can feed live sensor data straight into `vg_core`, or
record a session through `vg_core_io`'s writer (via JNI / Objective-C++), so
Android and iOS share one writer instead of each implementing the format. On a
desktop, recorded sessions are replayed through `vg_core_io` for development
and inspected in Foxglove Studio, which is a viewer only and not a dependency.

## Container

- One `.mcap` file per session, extension `.vgcap.mcap` recommended.
- Messages are encoded as **protobuf**. Where a standard
  [Foxglove schema](https://docs.foxglove.dev/docs/visualization/message-schemas/introduction)
  fits, it is used unchanged so sessions open in Foxglove Studio with no plugins.
  vg-specific messages live in [`schemas/vg/`](../schemas/vg).
- Chunk compression: `zstd`. (vg_core_io is built without LZ4, so LZ4-compressed
  files from other tools can't be read yet.)
- The file carries an MCAP metadata record named `vg_capture` with keys
  `format_version` (e.g. `0.1`) and `producer` (e.g. `vg_android 0.3.0`).

## Conventions

**Units.** SI throughout: meters, seconds, radians. Geographic positions are
WGS84 degrees, altitude in meters above the ellipsoid.

**Time.** Every message carries a `timestamp` in its payload, and the MCAP
`log_time` is set to the same value. All timestamps in a session come from **one
monotonic clock** on the producer (Android `SystemClock.elapsedRealtimeNanos` /
sensor event time, iOS `ARFrame.timestamp` / mach continuous time, rig hardware
clock). `vg.SessionInfo` records the offset from that clock to UTC so GPS and
weather data can be aligned. Producers must not mix clocks; if a sensor reports
on its own clock, convert before writing.

**Frames.** Right-handed. Frame ids used by this spec:

| Frame id | Meaning |
| --- | --- |
| `body` | The device body. Equal to the IMU frame. |
| `cam/<name>` | Camera optical frame, OpenCV convention: x right, y down, z forward. |
| `gps` | GPS antenna phase center. |
| `world` | Gravity-aligned, **z up**, origin and heading arbitrary (set by the tracker at session start). |

ARCore and ARKit use a y-up world and a camera frame with z pointing backwards;
producers convert to the conventions above before writing.

## Topics

`<name>` is a short camera name such as `rgb`, `left`, or `right`.

### Session and calibration (required)

| Topic | Schema | Notes |
| --- | --- | --- |
| `/vg/session` | `vg.SessionInfo` | Exactly one message, written first. |
| `/tf_static` | `foxglove.FrameTransforms` | Fixed extrinsics: `body` → each `cam/<name>`, `body` → `gps`. |
| `/cam/<name>/calibration` | `foxglove.CameraCalibration` | Intrinsics per camera. Rewrite if they change (e.g. autofocus). |

### Raw layer (required)

| Topic | Schema | Notes |
| --- | --- | --- |
| `/cam/<name>/image` | `foxglove.CompressedImage` (`jpeg`/`png`) or `foxglove.RawImage` | One per camera. Stereo rigs write `left` and `right`. |
| `/imu` | `vg.Imu` | Gyro + accelerometer in `body`, at native rate (100 Hz or more preferred). |
| `/gps` | `foxglove.LocationFix` | With `position_covariance` filled from reported accuracy. |

### Derived layer (optional)

| Topic | Schema | Notes |
| --- | --- | --- |
| `/cam/<name>/depth` | `foxglove.RawImage`, encoding `16UC1` (millimeters) or `32FC1` (meters) | Registered to `cam/<name>`. 0 / NaN means no data. |
| `/cam/<name>/depth/calibration` | `foxglove.CameraCalibration` | Intrinsics of the depth image. Needed when they differ from the color image (usual on ARCore/ARKit); vg_core_io always writes it, again whenever it changes. |
| `/cam/<name>/depth/confidence` | `foxglove.RawImage`, encoding `mono8` | 0 = none, 255 = full. ARKit's low/medium/high map to 0/128/255. |
| `/pose` | `foxglove.PoseInFrame` | Pose of `body` in `world` from the platform tracker (VIO). |
| `/pose/status` | `vg.TrackingStatus` | Tracker state, so vg_core can drop poses taken while tracking was limited or lost. |
| `/geo_pose` | `vg.GeoPose` | Optional. ARCore Geospatial / ARKit geo tracking output. |

## What each source writes

| Source | Raw | Derived |
| --- | --- | --- |
| vg_android (ARCore) | `rgb` image, IMU, GPS, calibration from `Camera.getImageIntrinsics()` | Depth from the Depth API (+ confidence), pose from `Frame.getAndroidSensorPose()` / camera pose, tracking state |
| iOS (ARKit) | `capturedImage`, CoreMotion IMU, CoreLocation GPS, `ARCamera.intrinsics` | `sceneDepth` + `confidenceMap` on LiDAR devices, `ARCamera.transform`, `trackingState` |
| Stereo + IMU + GPS rig | `left` + `right` images, IMU, GPS, full stereo calibration | Only if the camera SDK provides depth / VIO (ZED, RealSense, OAK-D) |

## Minimum for mapping

A session is mappable if it has session info, calibration, GPS (recommended, not
required), and **one** of:

1. a camera with depth **and** poses (typical phone session), or
2. a stereo pair **and** IMU (vg_core computes depth and poses), or
3. a single camera **and** IMU (monocular VIO; lowest quality, scale from IMU).

## Versioning

`format_version` is `MAJOR.MINOR`. Adding topics or optional fields bumps
MINOR; readers ignore topics they don't know. Changing or removing anything
bumps MAJOR. Until 1.0 the format may still change without a MAJOR bump.

## Open questions

- Video: storing images as `foxglove.CompressedVideo` (H.264/H.265) would cut
  file size a lot but makes per-frame access harder. Starting with JPEG.
- Whether to also record magnetometer and barometer (useful for heading and
  altitude). Easy to add as MINOR topics later.
- Whether vg_core should read protobuf directly or use a lighter decoder for
  the handful of schemas it needs.

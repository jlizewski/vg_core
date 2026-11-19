/*
 * vg_core_c: the C API of vg_core, for apps written in other languages
 * (Dart through dart:ffi, Swift, Kotlin through JNI, Python through ctypes).
 * docs/c-api.md is the guide: the design, examples, and how to bind it.
 *
 * Conventions
 * -----------
 * - Every function that can fail returns a vg_status: VG_OK (0), or an
 *   error whose message vg_last_error() returns on the same thread.
 * - Results come back through out-parameters, written only on VG_OK.
 * - Objects are opaque handles: made by a *_open / *_new function, released
 *   by the matching *_free (which accepts NULL). A handle may be used from
 *   any thread, but from one thread at a time, except vg_recorder, which
 *   serializes writes from several threads itself.
 * - Arrays a function returns are owned by the handle they came from, and
 *   stay valid until the next call on that handle that returns the same kind
 *   of array, or until the handle is freed. Copy what you keep.
 * - Arrays and strings passed in are copied; the caller keeps ownership.
 * - Strings are UTF-8 and NUL-terminated. Paths are UTF-8 too.
 * - Units are SI: meters, seconds, radians. Times are nanoseconds on the
 *   recording's monotonic clock (int64). Frames follow docs/capture-format.md:
 *   world z up; camera frames x right, y down, z forward (OpenCV).
 * - Booleans are int32: 0 false, anything else true.
 */
#ifndef VG_CORE_C_H
#define VG_CORE_C_H

#include <stddef.h>
#include <stdint.h>

#if defined(_WIN32)
#if defined(VG_CORE_C_BUILD)
#define VG_API __declspec(dllexport)
#elif defined(VG_CORE_C_SHARED)
#define VG_API __declspec(dllimport)
#else
#define VG_API
#endif
#else
#define VG_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ===== Library ========================================================= */

/* Version of this header. Compare with vg_api_version() at start-up: a
 * library built from a different header may not match these declarations. */
#define VG_C_API_VERSION 2

typedef int32_t vg_status;
enum {
  VG_OK = 0,
  VG_ERROR_INVALID_ARGUMENT = 1, /* NULL where not allowed, bad size, bad value */
  VG_ERROR_CONFIG = 2,           /* a bad setting (unknown, or doesn't parse) */
  VG_ERROR_FAILED = 3,           /* the work failed: a file, a recording, ... */
  VG_ERROR_INTERNAL = 4          /* a bug or out of memory */
};

/* VG_C_API_VERSION of the library. */
VG_API int32_t vg_api_version(void);

/* vg_core version, "major.minor.patch". Static; never freed. */
VG_API const char* vg_version_string(void);

/* Message of the last error on this thread ("" if none). Valid until the
 * next vg_ call on this thread. */
VG_API const char* vg_last_error(void);

/* Receives log output from long jobs, one line at a time (no newline). */
typedef void (*vg_log_fn)(void* user_data, const char* line);

/* ===== Shared types ==================================================== */

/* Pinhole intrinsics in pixels; pixel centers at integer coordinates. */
typedef struct vg_intrinsics {
  int32_t width;
  int32_t height;
  double fx;
  double fy;
  double cx;
  double cy;
} vg_intrinsics;

/* A rigid transform: rotation as a unit quaternion (x, y, z, w), then
 * translation. Takes points from the child frame to the parent frame (e.g.
 * world_from_body). */
typedef struct vg_pose {
  double orientation[4]; /* x, y, z, w */
  double position[3];    /* meters */
} vg_pose;

/* A depth image registered to a camera, row-major with no row padding
 * (width * height values). Depth is along the camera's z axis; 0 (or NaN)
 * means no reading. Give depth either in meters (depth_m) or in millimeters
 * (depth_mm, as ARCore delivers it): exactly one of them non-NULL. */
typedef struct vg_depth_frame {
  int64_t timestamp_ns;
  vg_intrinsics intrinsics;
  const float* depth_m;
  const uint16_t* depth_mm;
  const uint8_t* confidence; /* 0 none .. 255 full, or NULL */
  const uint8_t* labels;     /* a vg_semantic_class per pixel, or NULL */
} vg_depth_frame;

/* Semantic classes (include/vg_core/semantics.hpp). */
enum {
  VG_CLASS_UNKNOWN = 0,
  VG_CLASS_GROUND = 1,
  VG_CLASS_PLANT = 2,
  VG_CLASS_TREE = 3,
  VG_CLASS_STRUCTURE = 4,
  VG_CLASS_SKY = 5,
  VG_CLASS_DYNAMIC = 6
};

/* Where a map's frame sits on the Earth. */
typedef struct vg_geo_reference {
  double latitude;  /* degrees, WGS84 */
  double longitude; /* degrees */
  double altitude;  /* meters above the ellipsoid */
  /* Compass bearing of the map's +x axis, degrees clockwise from north, and
   * its 1-sigma uncertainty (infinite when the heading wasn't observed). */
  double x_bearing_deg;
  double bearing_sigma_deg;
} vg_geo_reference;

/* ===== Recording a capture session (docs/capture-format.md) ============ */

typedef struct vg_recorder vg_recorder;

/* What a recording is for in guided mapping (vg_session_info.session_type;
 * docs/capture-format.md#guided-mapping-sessions). NULL or "" is a free walk. */
#define VG_SESSION_BOUNDARY_WALK "boundary_walk"
#define VG_SESSION_GROUND_FILL "ground_fill"
#define VG_SESSION_SKYLINE_STATION "skyline_station"
#define VG_SESSION_STRUCTURE_STATION "structure_station"
#define VG_SESSION_CANOPY_STATION "canopy_station"
#define VG_SESSION_RELOCALIZATION "relocalization"

typedef struct vg_session_info {
  const char* producer;     /* e.g. "vg_flutter 0.1.0" */
  const char* platform;     /* "arcore", "arkit", "rig", ... */
  const char* device_model; /* may be NULL */
  int64_t start_time_ns;
  int64_t utc_offset_ns; /* UTC = session clock + this */
  const char* notes;     /* may be NULL */
  /* Since API version 2. A VG_SESSION_* value, or NULL / "" for a free walk;
   * anything else is refused. */
  const char* session_type;
  /* Since API version 2. The tracker's world frame: a new unique string (a
   * UUID) each time the AR session starts or restarts, the same in every
   * recording made while it keeps running. NULL or "" if unknown. */
  const char* tracking_frame;
} vg_session_info;

enum {
  VG_TRACKING_UNKNOWN = 0,
  VG_TRACKING_NORMAL = 1,
  VG_TRACKING_LIMITED = 2,
  VG_TRACKING_LOST = 3
};

typedef struct vg_static_transform {
  const char* parent_frame; /* e.g. "body" */
  const char* child_frame;  /* e.g. "cam/rgb" */
  vg_pose parent_from_child;
} vg_static_transform;

typedef struct vg_imu_sample {
  int64_t timestamp_ns;
  double angular_velocity[3];    /* rad/s, body frame */
  double linear_acceleration[3]; /* m/s^2, body frame */
} vg_imu_sample;

/* Magnetometer (since API version 2): the platform-calibrated field in the
 * body frame, microtesla, with the platform's accuracy: 0 unreliable, 1 low,
 * 2 medium, 3 high (Android's SENSOR_STATUS_*), 0 if unknown. */
typedef struct vg_magnetometer_sample {
  int64_t timestamp_ns;
  double field[3]; /* microtesla, body frame */
  int32_t accuracy;
} vg_magnetometer_sample;

typedef struct vg_gps_fix {
  int64_t timestamp_ns;
  double latitude;  /* degrees */
  double longitude; /* degrees */
  double altitude;  /* meters above the ellipsoid */
  /* East-North-Up position covariance, m^2, row-major. All zero if unknown. */
  double position_covariance[9];
} vg_gps_fix;

/* Starts a recording at path (replacing any file there). Write messages in
 * roughly timestamp order. */
VG_API vg_status vg_recorder_open(const char* path, const vg_session_info* info,
                                  vg_recorder** out);
VG_API vg_status vg_recorder_write_calibration(vg_recorder* recorder, int64_t timestamp_ns,
                                               const char* camera,
                                               const vg_intrinsics* intrinsics);
VG_API vg_status vg_recorder_write_static_transforms(vg_recorder* recorder,
                                                     int64_t timestamp_ns,
                                                     const vg_static_transform* transforms,
                                                     size_t count);
/* An encoded camera image; format "jpeg" or "png". */
VG_API vg_status vg_recorder_write_image(vg_recorder* recorder, int64_t timestamp_ns,
                                         const char* camera, const char* format,
                                         const uint8_t* data, size_t size);
/* Depth, with its confidence and labels when given. */
VG_API vg_status vg_recorder_write_depth(vg_recorder* recorder, const char* camera,
                                         const vg_depth_frame* frame);
VG_API vg_status vg_recorder_write_pose(vg_recorder* recorder, int64_t timestamp_ns,
                                        const vg_pose* world_from_body);
VG_API vg_status vg_recorder_write_tracking(vg_recorder* recorder, int64_t timestamp_ns,
                                            int32_t state, const char* reason);
VG_API vg_status vg_recorder_write_imu(vg_recorder* recorder, const vg_imu_sample* sample);
VG_API vg_status vg_recorder_write_magnetometer(vg_recorder* recorder,
                                                const vg_magnetometer_sample* sample);
VG_API vg_status vg_recorder_write_gps(vg_recorder* recorder, const vg_gps_fix* fix);
/* Finishes the file (writes its index). Writing after this fails. */
VG_API vg_status vg_recorder_close(vg_recorder* recorder);
/* Closes the recording if still open (errors are lost: call
 * vg_recorder_close first to see them) and frees the handle. */
VG_API void vg_recorder_free(vg_recorder* recorder);

/* ===== Building a map live (include/vg_core/map_builder.hpp) =========== */

typedef struct vg_map_builder vg_map_builder;

typedef struct vg_map_builder_config {
  /* 3D map (TsdfConfig). */
  double voxel_size;          /* meters */
  double truncation_distance; /* meters */
  double min_depth;           /* readings outside [min_depth, max_depth] are */
  double max_depth;           /* ignored, meters */
  int32_t min_confidence;     /* 0-255 */
  double edge_ratio;          /* depth edges dropped; 0 keeps them */
  double max_incidence_deg;   /* surfaces seen edge-on dropped; 90 keeps all */
  int32_t quality_weights;    /* weigh readings by confidence, range, angle */
  int32_t carve;              /* free-space carving */
  int32_t pixel_stride;       /* place blocks from every Nth pixel */
  int32_t threads;            /* 0: all hardware threads */
  float min_weight;           /* fused weight a voxel needs to show */
  uint32_t drop_classes;      /* bit i set: readings of class i left out */
  /* Live height map and keyframes (MapBuilderConfig). */
  double height_cell_size;     /* meters */
  int64_t max_pose_age_ns;     /* pose must be this close to a depth frame */
  double keyframe_translation; /* meters; 0 with keyframe_rotation 0: */
  double keyframe_rotation;    /* radians; fuse every frame live */
  const char* deferred_path;   /* spool file for deferred frames; NULL: memory */
  /* Undo the tracker's pose jumps (pose_jumps.hpp), as vg replay does. */
  int32_t remove_pose_jumps;
} vg_map_builder_config;

/* Fills config with the settings vg replay uses by default
 * (config/mapping.yaml). min_weight is 0: set it with
 * vg_map_builder_set_min_weight once it's known whether the depth has
 * confidence (vg replay uses 0.05 with confidence, 0.5 without). */
VG_API vg_status vg_map_builder_config_defaults(vg_map_builder_config* config);

VG_API vg_status vg_map_builder_new(const vg_map_builder_config* config, vg_map_builder** out);
VG_API void vg_map_builder_free(vg_map_builder* builder);

/* Fixed pose of a camera on the body (body_from_camera); identity if unset. */
VG_API vg_status vg_map_builder_set_camera_extrinsics(vg_map_builder* builder,
                                                      const char* camera,
                                                      const vg_pose* body_from_camera);
VG_API vg_status vg_map_builder_add_pose(vg_map_builder* builder, int64_t timestamp_ns,
                                         const vg_pose* world_from_body);
/* Whether the tracker trusts its poses; depth arriving while not is skipped. */
VG_API vg_status vg_map_builder_set_tracking_ok(vg_map_builder* builder, int32_t ok);
/* While not live, every depth frame is deferred (e.g. the caller is behind). */
VG_API vg_status vg_map_builder_set_live(vg_map_builder* builder, int32_t live);
VG_API vg_status vg_map_builder_add_depth(vg_map_builder* builder, const char* camera,
                                          const vg_depth_frame* frame);
/* Settles the last depth frame still waiting for a pose. Call at the end. */
VG_API vg_status vg_map_builder_flush(vg_map_builder* builder);
/* Fuses up to max_frames deferred frames; remaining (may be NULL) gets how
 * many are still deferred. */
VG_API vg_status vg_map_builder_integrate_deferred(vg_map_builder* builder, size_t max_frames,
                                                   size_t* remaining);
VG_API vg_status vg_map_builder_set_min_weight(vg_map_builder* builder, float min_weight);

/* One live height map cell: it covers [x, x + 1) * cell_size by
 * [y, y + 1) * cell_size in the world frame. height is NaN when the cell no
 * longer has a surface. */
typedef struct vg_height_cell {
  int32_t x;
  int32_t y;
  float height;
} vg_height_cell;

/* What changed since the previous update, for a live view to redraw. */
typedef struct vg_map_update {
  const vg_height_cell* cells;
  size_t cell_count;
  /* Voxel blocks whose contents may have changed: 3 int32 (x, y, z) each.
   * Re-read each with vg_map_builder_block_voxels. */
  const int32_t* blocks;
  size_t block_count;
} vg_map_update;

/* Brings the live height map up to date. The arrays in out are owned by the
 * builder until the next vg_map_builder_update. */
VG_API vg_status vg_map_builder_update(vg_map_builder* builder, vg_map_update* out);
/* The occupied voxels of one block, 3 int32 (i, j, k) each: voxel (i, j, k)
 * spans [i, j, k] * voxel_size to [i + 1, j + 1, k + 1] * voxel_size. Owned
 * by the builder until the next vg_map_builder_block_voxels. */
VG_API vg_status vg_map_builder_block_voxels(vg_map_builder* builder, const int32_t block[3],
                                             const int32_t** voxels, size_t* count);

typedef struct vg_map_builder_stats {
  uint64_t integrated;
  uint64_t skipped_no_pose;
  uint64_t skipped_tracking;
  uint64_t deferred; /* ever deferred */
  uint64_t dropped;
  uint64_t waiting; /* deferred and not yet integrated */
  uint64_t height_cells;
} vg_map_builder_stats;

VG_API vg_status vg_map_builder_get_stats(const vg_map_builder* builder,
                                          vg_map_builder_stats* out);

typedef struct vg_grid vg_grid;

/* A copy of the live height map as of the last update, as a new grid (free
 * it with vg_grid_free). */
VG_API vg_status vg_map_builder_height_map(const vg_map_builder* builder, vg_grid** out);

/* ===== 2D maps (.asc grids) ============================================ */

typedef struct vg_grid_info {
  int32_t width;
  int32_t height;
  double origin_x;  /* lower-left corner of cell (0, 0), meters */
  double origin_y;
  double cell_size; /* meters */
} vg_grid_info;

/* Loads an ESRI ASCII grid such as a package's ground.asc. */
VG_API vg_status vg_grid_load(const char* path, vg_grid** out);
VG_API void vg_grid_free(vg_grid* grid);
VG_API vg_status vg_grid_get_info(const vg_grid* grid, vg_grid_info* out);
/* width * height values, row-major from the bottom row (y = 0) up: cell
 * (x, y) is values[y * width + x]. NaN where there is no data. Owned by the
 * grid. */
VG_API const float* vg_grid_values(const vg_grid* grid);
/* Smallest and largest value, and how many cells have data (lowest and
 * highest are NaN when none do). Any out-parameter may be NULL. */
VG_API vg_status vg_grid_range(const vg_grid* grid, float* lowest, float* highest, size_t* cells);

/* ===== Map packages (include/vg_core/map_package.hpp) ================== */

typedef struct vg_package vg_package;

/* Opens a package and loads its 3D map (map.vgm). */
VG_API vg_status vg_package_open(const char* dir, vg_package** out);
VG_API void vg_package_free(vg_package* package);
/* The occupied voxels (specks left out, as the stages see them): 3 int32
 * (i, j, k) each, and per voxel its vg_semantic_class (VG_CLASS_UNKNOWN when
 * unlabeled). voxel_size in meters. Arrays owned by the package. */
VG_API vg_status vg_package_voxels(vg_package* package, const int32_t** voxels,
                                   const uint8_t** classes, size_t* count, double* voxel_size);
/* has is set to 0 (and out left alone) when the recording had no GPS. */
VG_API vg_status vg_package_geo_reference(const vg_package* package, vg_geo_reference* out,
                                          int32_t* has);

/* ===== Sites (include/vg_core/site.hpp) ================================ */

typedef struct vg_site vg_site;

typedef struct vg_site_visit {
  const char* package_dir; /* resolved; owned by the site */
  const char* recorded;    /* YYYYMMDD-HHMMSS, or "" if unknown */
  /* Transform into the site frame: turn about z, then shift. */
  double yaw;              /* radians, counter-clockwise */
  double x, y, z;          /* meters */
  int32_t has_fit;         /* 0 for the first visit, which sets the frame */
  double overlap;          /* the fit when added (see registration.hpp) */
  double inliers_30cm;
  double median_distance;
} vg_site_visit;

/* Opens the site in dir (created if needed). Reflects site.csv when opened;
 * open again after adding visits. */
VG_API vg_status vg_site_open(const char* dir, vg_site** out);
VG_API void vg_site_free(vg_site* site);
VG_API vg_status vg_site_visit_count(const vg_site* site, size_t* count);
/* Visits are oldest first. */
VG_API vg_status vg_site_get_visit(const vg_site* site, size_t index, vg_site_visit* out);

/* ===== Site layers (include/vg_core/site_layers.hpp) ================= */

/* A site's layers as vg_site_make_ground (or vg_site_add, vg_site_make_maps)
 * last made them: the ground with a state per cell, the boundary, feature
 * elements, ground-fill guidance and the user's edits. All in the site frame,
 * meters. See docs/pipeline/sites.md. Since API version 2. */
typedef struct vg_site_layers vg_site_layers;

/* Ground cell states (the VG_GROUND_STATE grid's values; NaN is outside). */
enum {
  VG_GROUND_OUTSIDE = 0,
  VG_GROUND_MEASURED = 1, /* seen as ground */
  VG_GROUND_FILLED = 2,   /* a small hole or under something low, filled in */
  VG_GROUND_COVERED = 3,  /* under something standing; the ground runs on beneath */
  VG_GROUND_UNKNOWN = 4   /* a hole not yet seen: go and look */
};

/* Where a ground cell's value came from (the VG_GROUND_PROVENANCE grid). */
enum {
  VG_PROVENANCE_NONE = 0,
  VG_PROVENANCE_MEASURED = 1,
  VG_PROVENANCE_INFERRED = 2,
  VG_PROVENANCE_EDITED = 3,
  VG_PROVENANCE_CONFIRMED = 4
};

/* The ground layer's grids (vg_site_layers_ground_grid). */
enum {
  VG_GROUND_GRID_HEIGHT = 0,       /* m; NaN outside and over unknown cells */
  VG_GROUND_GRID_STATE = 1,        /* VG_GROUND_*; NaN outside */
  VG_GROUND_GRID_PROVENANCE = 2,   /* VG_PROVENANCE_*; NaN where none */
  VG_GROUND_GRID_OBSERVATIONS = 3, /* how many walks measured the cell */
  VG_GROUND_GRID_SPREAD = 4        /* m: how far the farthest walk is from the height */
};

/* How a boundary vertex was found. */
enum {
  VG_EDGE_STRUCTURE = 0,  /* a fence, a wall, a hedge */
  VG_EDGE_GROUND_END = 1, /* where the seen ground ends */
  VG_EDGE_ASSUMED = 2,    /* neither nearby: assumed, for the user to confirm */
  VG_EDGE_EDITED = 3      /* drawn by the user */
};

typedef struct vg_boundary_vertex {
  double x, y;
  int32_t edge; /* VG_EDGE_* */
  double push;  /* how far out from the walk's path, m */
} vg_boundary_vertex;

enum { VG_FEATURE_POST = 0, VG_FEATURE_LINE = 1, VG_FEATURE_BLOCK = 2 };

/* A feature element: its footprint (a box) and what it is. */
typedef struct vg_feature {
  const char* id;         /* e.g. "post-3"; owned by the layers */
  int32_t kind;           /* VG_FEATURE_* */
  int32_t semantic_class; /* VG_CLASS_* */
  double x, y;            /* center */
  double length, width;   /* along and across its axis, m */
  double axis;            /* radians from x, [0, pi) */
  double height;          /* top above the ground, m */
  int32_t seen;           /* walks that saw it */
  int32_t missed;         /* walks in a row since that covered it but didn't */
  int32_t retired;        /* missed too often; kept, not current */
  double spread;          /* farthest sighting from its position, m */
} vg_feature;

/* A spot to stand on to see a hole, and which way to face. */
typedef struct vg_look_spot {
  double x, y;
  double heading;  /* radians from x, toward what it sees */
  double distance; /* to the hole's nearest cell, m */
  double sees;     /* share of the hole in view from here, 0-1 */
} vg_look_spot;

/* A hole in the ground (unknown cells) to go and look at. */
typedef struct vg_guidance_target {
  int32_t rank;  /* 1 = largest */
  int32_t order; /* when to visit it, 1 = first (nearest next) */
  double area;   /* m^2 */
  double center_x, center_y;
  double seen;       /* share of it in view from its spots together */
  size_t first_spot; /* its spots: spots[first_spot .. first_spot + spot_count) */
  size_t spot_count;
} vg_guidance_target;

enum { VG_EDIT_GROUND = 0, VG_EDIT_NOT_GROUND = 1, VG_EDIT_BOUNDARY = 2 };

/* A user's edit, and (area edits) how walks recorded after it agree. */
typedef struct vg_site_edit {
  const char* id;       /* "edit-N" */
  const char* time;     /* ISO 8601 UTC */
  int32_t kind;         /* VG_EDIT_* */
  const double* points; /* x0, y0, x1, y1, ...: an area, or a boundary line */
  size_t point_count;
  const char* note; /* "" if none */
  int32_t reviewed; /* 0 for boundary lines and before the layers are made again */
  size_t cells;     /* the area's cells */
  size_t disagreeing_cells;
  const char* disagreeing_walks; /* walks since the edit that disagree, ';'-separated */
} vg_site_edit;

typedef struct vg_site_layers_info {
  int32_t has_ground; /* the ground layer has been made */
  int32_t has_guidance;
  int32_t done;        /* no unknown hole left worth looking at */
  double unknown_area; /* m^2 of unknown ground, over all targets */
  size_t boundary_vertex_count;
  size_t feature_count;
  size_t target_count;
  size_t spot_count;
  size_t edit_count;
} vg_site_layers_info;

/* Reads the layers of the site in dir (none made yet: all empty). Open again
 * after a job changes them. Strings and arrays below belong to the handle
 * and live until it's freed. */
VG_API vg_status vg_site_layers_open(const char* dir, vg_site_layers** out);
VG_API void vg_site_layers_free(vg_site_layers* layers);
VG_API vg_status vg_site_layers_get_info(const vg_site_layers* layers, vg_site_layers_info* out);
/* One of the ground layer's grids (VG_GROUND_GRID_*), as a new grid: free it
 * with vg_grid_free. VG_ERROR_FAILED if the ground layer isn't made. */
VG_API vg_status vg_site_layers_ground_grid(const vg_site_layers* layers, int32_t which,
                                            vg_grid** out);
/* The site's boundary, counter-clockwise (count 0 with no boundary walk). */
VG_API vg_status vg_site_layers_boundary(const vg_site_layers* layers,
                                         const vg_boundary_vertex** vertices, size_t* count);
VG_API vg_status vg_site_layers_features(const vg_site_layers* layers, const vg_feature** features,
                                         size_t* count);
/* Targets in visiting order; spots as indexed by the targets. */
VG_API vg_status vg_site_layers_guidance(const vg_site_layers* layers,
                                         const vg_guidance_target** targets, size_t* target_count,
                                         const vg_look_spot** spots, size_t* spot_count);
/* Area edits, then boundary lines, each in the order made. */
VG_API vg_status vg_site_layers_edits(const vg_site_layers* layers, const vg_site_edit** edits,
                                      size_t* count);

/* ===== Pipeline jobs (include/vg_core/config/jobs.hpp) ================= */

/* Each job takes settings as command-line arguments, applied in order over
 * the defaults: "--config", FILE, "--set", "PATH=VALUE", or a short flag and
 * its value. settings may be NULL when settings_count is 0. Progress goes to
 * log (may be NULL). Jobs run on the calling thread and can take minutes:
 * call them from a worker thread. */

/* vg replay. complete (may be NULL) is 0 if a stage couldn't run (the sun
 * stage, with no location). */
VG_API vg_status vg_replay(const char* session, const char* package_dir,
                           const char* const* settings, size_t settings_count, vg_log_fn log,
                           void* user_data, int32_t* complete);
/* vg map: re-run stages ("surface", "ground", ..., or "all"). */
VG_API vg_status vg_map_stages(const char* package_dir, const char* const* stages,
                               size_t stage_count, const char* const* settings,
                               size_t settings_count, vg_log_fn log, void* user_data,
                               int32_t* complete);
/* vg site add. all_added (may be NULL) is 0 if a walk didn't line up. */
VG_API vg_status vg_site_add(const char* site_dir, const char* const* package_dirs,
                             size_t package_count, int32_t force, const char* const* settings,
                             size_t settings_count, vg_log_fn log, void* user_data,
                             int32_t* all_added);
/* vg site maps. */
VG_API vg_status vg_site_make_maps(const char* site_dir, const char* const* settings,
                                   size_t settings_count, vg_log_fn log, void* user_data);
/* vg site ground: remakes the site's layers (ground, boundary, features,
 * guidance) from its walks and edits. Since API version 2. */
VG_API vg_status vg_site_make_ground(const char* site_dir, const char* const* settings,
                                     size_t settings_count, vg_log_fn log, void* user_data);
/* vg site edit: adds an edit (kind VG_EDIT_*; points x0, y0, x1, y1, ... in
 * the site frame: an area of 3 or more, a boundary line of 2 or more; note
 * one line, may be NULL), then remakes the layers. The new edit's id
 * ("edit-N") is copied to id (may be NULL) if it fits in id_size bytes with
 * its terminating NUL. Since API version 2. */
VG_API vg_status vg_site_add_edit(const char* site_dir, int32_t kind, const double* points,
                                  size_t point_count, const char* note, char* id, size_t id_size,
                                  vg_log_fn log, void* user_data);
/* vg site delete-edit: takes the edit out, then remakes the layers.
 * VG_ERROR_CONFIG if the site has no such edit. Since API version 2. */
VG_API vg_status vg_site_delete_edit(const char* site_dir, const char* id, vg_log_fn log,
                                     void* user_data);

#ifdef __cplusplus
}
#endif

#endif /* VG_CORE_C_H */

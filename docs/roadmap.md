# Roadmap

The [priorities](vision.md#priorities) as milestones. Each milestone is
something a gardener could use, and lists what it still needs. ✅ built,
🔨 in progress or partly built, ⬜ not started.

## M1. Map a garden with a phone

*Priority 1.* A walk around the yard gives accurate maps of ground, what
stands on it, the canopy and sun, and many walks combine into one site.

| Item | Status | Notes |
| --- | --- | --- |
| Capture format and recorder (Android) | ✅ | [capture-format.md](capture-format.md) |
| 3D map from depth and poses, cleanup | ✅ | [pipeline/building.md](pipeline/building.md) |
| Ground, terrain, features, objects | ✅ | [pipeline/derived-maps.md](pipeline/derived-maps.md) |
| Semantic labels, label quality | ✅ | Model choice must be revisited before shipping ([semantics.md](pipeline/semantics.md#before-shipping-re-evaluate-the-model)) |
| Sky evidence and canopy | ✅ | [pipeline/sun-and-sky.md](pipeline/sun-and-sky.md#sky-and-canopy) |
| Annual sun hours and energy per walk | ✅ | |
| Sites: register walks, current maps, change | ✅ | [pipeline/sites.md](pipeline/sites.md) |
| Dashboard for the workflow | ✅ | README |
| iOS recorder | ⬜ | In vg_flutter ([D12](decisions.md#d12-phone-apps-in-flutter-with-native-capture)); vg_core needs an iOS build |
| Sun by month | ⬜ | Needed for planning ([garden-model.md](design/garden-model.md#what-this-needs-from-the-maps)) |
| Sun over the site, with the canopy as occluder | ⬜ | Site-wide geo-reference from all walks' GPS |
| Diffuse light (sky-view fraction) | ⬜ | Same rays as the sun map |
| Run the pipeline on the phone after a walk | ⬜ | Today it runs on a PC |
| C API for bindings | ⬜ | [D11](decisions.md#d11-a-c-api-for-the-app-bindings) |
| Commercially usable segmentation model | ⬜ | Licenses, yard accuracy, phone cost |

## M2. Plan and track the garden

*Priority 2.* Beds and plantings drawn by the user, checked against the maps
and plant knowledge; history kept.

| Item | Status | Notes |
| --- | --- | --- |
| Garden model design | 🔨 | [design/garden-model.md](design/garden-model.md) (Proposed) |
| Places and summaries | ⬜ | Phase 1 of the garden model |
| vg_data repository: taxa, regions | ⬜ | [design/plant-data.md](design/plant-data.md) |
| Plantings, light suitability | ⬜ | |
| Companion relations, with evidence levels | ⬜ | |
| Observations, photos, harvests | ⬜ | |
| Water design | ⬜ | Rain exposure (rain shadow from the canopy and eaves, like sun), a water balance per place by kind and soil, forecasts as input. Design doc needed. |
| Warnings design | ⬜ | Frost, heat, waterlogging, disease-favoring weather, as typed events. Design doc needed. |
| Sync with VG Home | ⬜ | |
| Goals and recommendations | ⬜ | Design doc needed |

## M3. Plant health detection

*Priority 3.* Spot problems from photos and sensors.

| Item | Status | Notes |
| --- | --- | --- |
| Health detection design | ⬜ | Models, training data and licenses, on-phone cost, how results become observations with confidence |
| Image datasets | ⬜ | Separate from vg_data |
| Sensor readings as observations | ⬜ | Via VG Home devices |

## M4. Organic remedy recommendations

*Priority 4.* For a problem found, what to do about it, organically.

| Item | Status | Notes |
| --- | --- | --- |
| Problems in vg_data | ⬜ | Pests, diseases, disorders, conditions favoring them |
| Remedies in vg_data | ⬜ | Cultural and mechanical first; NOP status for materials |
| Recommendation logic | ⬜ | Design doc needed |

## Apps

Outside vg_core, but they decide what vg_core must support.

| App | Status | Notes |
| --- | --- | --- |
| vg_android: recorder | ✅ | Depth, poses, GPS, and labels from an on-device model. To be replaced by vg_flutter. |
| Desktop dev interface | ⬜ | Settle presentation and map interaction before phone screens ([D13](decisions.md#d13-a-desktop-dev-interface-before-the-phone-ui)) |
| vg_flutter: Android + iOS recorder | ⬜ | Native capture plugins, Flutter UI ([D12](decisions.md#d12-phone-apps-in-flutter-with-native-capture)) |
| VG Home | ⬜ | Python service + Home Assistant integration; needs Python bindings |
| VG ROS2 package | ⬜ | C++ directly; needs a ROS-topic producer of the capture format or live structs |

## Design docs still to write

In rough order of need:

1. **Water**: rain exposure map, per-place water balance.
2. **Warnings**: event types, thresholds, how forecasts come in.
3. **C API**: the binding surface.
4. **Recommendations**: goals, suitability, layout suggestions.
5. **Health detection**.

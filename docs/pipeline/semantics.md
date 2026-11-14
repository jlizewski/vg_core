# Semantic labels

Labels tell the map what things are: ground, plant, tree, structure, sky,
people and animals. A segmentation model labels each camera image, the labels
are carried onto the depth pixels, and the map collects them per voxel.

Code: [semantics.hpp](../../include/vg_core/semantics.hpp),
[segmentation.hpp](../../include/vg_core/segmentation.hpp),
[onnx_backend.hpp](../../include/vg_core/onnx_backend.hpp),
[label_consistency.hpp](../../include/vg_core/label_consistency.hpp).
Usage: the README's [Semantic labels](../../README.md#semantic-labels).

## The classes

| Value | Class | Covers | Used by |
| --- | --- | --- | --- |
| 0 | unknown | Unlabeled, or nothing the model knows | – |
| 1 | ground | Lawn, soil, mulch, paths | Ground stage, terrain |
| 2 | plant | Plants, flowers, shrubs: low vegetation | Ground stage (not ground) |
| 3 | tree | Trunks and canopy | Ground stage (not ground) |
| 4 | structure | House, walls, fences, decks, sheds | Ground stage (not ground) |
| 5 | sky | Sky | Sky evidence, canopy; dropped from the map |
| 6 | dynamic | People, animals | Dropped from the map |

The set is small on purpose: what mapping and sun modeling need to tell
apart, not everything a model can name. A model's own classes (ADE20K has
150) are mapped onto these where the model runs. Identifying plant species or
diseases is a separate job (priority 3, [roadmap](../roadmap.md)) with its
own models; it won't be done by growing this list.

## Labeling an image

`Segmenter::segment()` does the same steps on every platform:

1. **Turn upright.** Models are trained on upright photos, but phone cameras
   deliver images in the sensor's orientation. `upright_rotation()` works out
   from the camera's pose which edge of the image is up.
2. **Prepare.** Stretch the image to the model's input size and normalize it
   (`SegmentationModelInfo`: mean and standard deviation per channel).
3. **Run** the model through an `InferenceBackend`: LiteRT on Android, ONNX
   Runtime on a PC, Core ML on iOS later. This is the only platform-specific
   part, and each backend is its own small library.
4. **Classify.** Each output cell takes its best-scoring class, mapped to a vg
   class (e.g. `ade20k_classes()`).
5. **Shrink to depth.** Each depth pixel takes the class most of the output
   cells it covers have, back in the camera's orientation.

The phone writes the result into the recording as
`/cam/<name>/depth/labels` ([capture-format.md](../capture-format.md)). On a
PC, `vg segment` relabels a recording with another model so models can be
compared on the same walk.

## Labels in the map

During fusion, each voxel near a surface counts the classes it was seen as
(`TsdfVolume::label_votes`). Votes are 8-bit and saturate; when one would
overflow, all of that voxel's counts are halved together, so the ratios hold.
A voxel's class is the one with the most votes, if it has at least
`semantics.min_votes` (2). Single-frame mistakes are outvoted.

Readings of the dropped classes (people, animals and sky by default) don't
enter the map at all, so a person walking through doesn't leave a ghost.

## Label quality

Per-pixel agreement between two runs of the same model is only about 80%
(JPEG versus raw camera images, GPU precision), so comparing pixels says
little. Instead the build measures **label consistency**: the share of each
frame's labeled readings that agree with the map's settled class where they
land. Steady labels agree with the map; labels that flip from frame to frame
don't. It needs no ground truth.

On the October 16 walk, DeepLabV3 scored 85% overall: ground 95%, structure
88%, tree 52%, plant 44%. Plants and trees are where a better model would help
most.

## Before shipping: re-evaluate the model

The current models (DeepLabV3 MobileNetV2 and SegFormer-B0, both trained on
ADE20K) were chosen to get started:

- **Licenses.** ADE20K's terms and some weights (NVIDIA's SegFormer) are
  research or non-commercial only. A shipped app needs commercially usable
  weights and training data.
- **Accuracy on yards.** Measure where it fails (mulch vs. soil vs. low
  plants, raised beds, decks, furniture), and fine-tune on our own frames if
  it matters.
- **Cost on the phone.** Inference time, label frame rate, heat and battery
  over a whole walk.

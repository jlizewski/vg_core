"""Writes the tiny ONNX "segmentation models" the ONNX backend tests load.

Each takes a 1 x 8 x 8 x 3 float image (NHWC) and scores 151 ADE20K labels on
a 4 x 4 grid: label 10 (grass, vg ground) where the 2 x 2 block's red is above
0, label 1 (wall, vg structure) where it's below. tiny_seg_nhwc.onnx outputs
1 x 4 x 4 x 151, as DeepLabV3 does; tiny_seg_nchw.onnx 1 x 151 x 4 x 4, as
SegFormer does.

    pip install onnx && python tests/data/make_tiny_segmenters.py
"""

from pathlib import Path

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper

LABELS = 151
GRASS = 10
WALL = 1


def model(channels_first_output: bool) -> onnx.ModelProto:
    weights = np.zeros((LABELS, 3, 1, 1), dtype=np.float32)
    weights[GRASS, 0] = 1.0  # Red above 0 scores grass...
    weights[WALL, 0] = -1.0  # ...below 0, wall.
    nodes = [
        helper.make_node("Transpose", ["image"], ["nchw"], perm=[0, 3, 1, 2]),
        helper.make_node("AveragePool", ["nchw"], ["pooled"], kernel_shape=[2, 2], strides=[2, 2]),
        helper.make_node("Conv", ["pooled", "weights"], ["scores_nchw"]),
    ]
    out_shape = [1, LABELS, 4, 4] if channels_first_output else [1, 4, 4, LABELS]
    if channels_first_output:
        nodes.append(helper.make_node("Identity", ["scores_nchw"], ["scores"]))
    else:
        nodes.append(helper.make_node("Transpose", ["scores_nchw"], ["scores"], perm=[0, 2, 3, 1]))
    graph = helper.make_graph(
        nodes,
        "tiny_segmenter",
        [helper.make_tensor_value_info("image", TensorProto.FLOAT, [1, 8, 8, 3])],
        [helper.make_tensor_value_info("scores", TensorProto.FLOAT, out_shape)],
        [numpy_helper.from_array(weights, "weights")],
    )
    m = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 17)])
    m.ir_version = 8
    onnx.checker.check_model(m)
    return m


here = Path(__file__).parent
onnx.save(model(False), here / "tiny_seg_nhwc.onnx")
onnx.save(model(True), here / "tiny_seg_nchw.onnx")
print("wrote tiny_seg_nhwc.onnx and tiny_seg_nchw.onnx")

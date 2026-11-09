import pytest

from vg_tools import labels

np = pytest.importorskip("numpy")


def test_compares_two_labelings():
    ground, plant, structure = 1, 2, 4
    a = {
        1: np.full((2, 2), ground, dtype=np.uint8),
        2: np.array([[plant, plant], [structure, structure]], dtype=np.uint8),
        3: np.full((2, 2), ground, dtype=np.uint8),  # Only in A.
    }
    b = {
        1: np.array([[ground, ground], [ground, plant]], dtype=np.uint8),
        2: np.array([[plant, ground], [structure, structure]], dtype=np.uint8),
    }
    c = labels.compare_labels(a, b)
    assert (c.frames, c.pixels, c.agree) == (2, 8, 6)
    assert c.agreement() == pytest.approx(0.75)
    assert c.confusion[ground][plant] == 1
    assert c.confusion[plant][ground] == 1
    # Ground: 3 in both, A has 4, B has 4 -> 3 / 5.
    assert c.class_iou(ground) == pytest.approx(3 / 5)
    assert c.class_iou(structure) == pytest.approx(1.0)
    assert c.class_iou(5) is None  # Sky in neither.
    text = labels.format_comparison(c, "phone.mcap", "desktop.mcap")
    assert "A: phone.mcap" in text and "75.0% labeled the same" in text


def test_fixes_fractional_resizes():
    onnx = pytest.importorskip("onnx")
    from onnx import TensorProto, helper, numpy_helper

    from vg_tools import models

    # 65 x 65 resized by 129/65, as tf2onnx writes TensorFlow's resize to 129.
    scales = np.array([1, 1, 129 / 65, 129 / 65], dtype=np.float32)
    graph = helper.make_graph(
        [helper.make_node("Resize", ["x", "", "scales"], ["y"], name="up", mode="linear")],
        "resize",
        [helper.make_tensor_value_info("x", TensorProto.FLOAT, [1, 4, 65, 65])],
        [helper.make_tensor_value_info("y", TensorProto.FLOAT, [1, 4, 129, 129])],
        [numpy_helper.from_array(scales, "scales")],
    )
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 17)])
    assert models.fix_fractional_resizes(model) == ["up"]
    sizes = [numpy_helper.to_array(i) for i in model.graph.initializer]
    assert [s.tolist() for s in sizes] == [[1, 4, 129, 129]]  # Exact sizes; scales dropped.
    onnx.checker.check_model(model)

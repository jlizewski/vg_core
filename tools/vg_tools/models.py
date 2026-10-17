"""Converting segmentation models to ONNX for the desktop (vg segment).

    vg models export deeplabv3_mnv2_ade20k path/to/deeplabv3_mnv2_ade20k.tflite

writes models/deeplabv3_mnv2_ade20k.onnx in vg_core, where vg segment looks
by default. The phone keeps running the .tflite; the ONNX copy is checked to
give the same classes on random input before it's written.

Needs the `models` extra, which is large (TensorFlow): pip install -e "tools[models]".

tf2onnx turns TensorFlow's exact-size resizes into fractional scales (65 x
129/65), which ONNX Runtime floors one pixel short (128), breaking the model;
those resizes get their exact sizes back here.
"""

from __future__ import annotations

from pathlib import Path


def _require(module: str):
    try:
        return __import__(module)
    except ImportError as e:
        raise SystemExit(
            f'vg: this needs the models extra ({e.name} is missing): pip install -e "tools[models]"'
        ) from e


def fix_fractional_resizes(model) -> list[str]:
    """Give Resize nodes with fractional scales their exact output sizes, and
    drop the scales left unused. Returns the fixed nodes' names."""
    np = _require("numpy")
    from onnx import numpy_helper, shape_inference

    inferred = shape_inference.infer_shapes(model, strict_mode=False)
    shapes = {}
    for vi in list(inferred.graph.value_info) + list(inferred.graph.input):
        shapes[vi.name] = [d.dim_value if d.HasField("dim_value") else None
                           for d in vi.type.tensor_type.shape.dim]
    inits = {i.name: numpy_helper.to_array(i) for i in model.graph.initializer}
    fixed = []
    for node in model.graph.node:
        if node.op_type != "Resize" or len(node.input) < 3 or node.input[2] not in inits:
            continue
        scales = inits[node.input[2]]
        if np.allclose(scales, np.round(scales)):
            continue
        in_shape = shapes.get(node.input[0])
        if not in_shape or any(d is None for d in in_shape):
            continue
        sizes = np.array([int(round(d * s)) for d, s in zip(in_shape, scales, strict=True)],
                         dtype=np.int64)
        name = node.name + "_sizes"
        model.graph.initializer.append(numpy_helper.from_array(sizes, name))
        node.input[2] = ""
        while len(node.input) < 4:
            node.input.append("")
        node.input[3] = name
        fixed.append(node.name)
    used = {i for n in model.graph.node for i in n.input}
    for init in [i for i in model.graph.initializer if i.name not in used]:
        model.graph.initializer.remove(init)
    return fixed


def check_against_tflite(tflite: Path, onnx_path: Path, trials: int = 2) -> float:
    """Share of output cells where the ONNX model picks the .tflite's class, on
    random images scaled to -1..1."""
    np = _require("numpy")
    ort = _require("onnxruntime")
    tf = _require("tensorflow")

    interpreter = tf.lite.Interpreter(model_path=str(tflite))
    interpreter.allocate_tensors()
    inp = interpreter.get_input_details()[0]
    out = interpreter.get_output_details()[0]
    session = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    name = session.get_inputs()[0].name
    rng = np.random.default_rng(0)
    agree = []
    for _ in range(trials):
        x = rng.uniform(-1, 1, size=inp["shape"]).astype(np.float32)
        interpreter.set_tensor(inp["index"], x)
        interpreter.invoke()
        a = interpreter.get_tensor(out["index"])
        b = session.run(None, {name: x})[0]
        agree.append(float((a.argmax(-1) == b.argmax(-1)).mean()))
    return min(agree)


def export_tflite(tflite: Path, out: Path, opset: int = 17) -> tuple[list[str], float]:
    """Convert a .tflite segmentation model to ONNX at out, fix its resizes and
    check it. Returns the fixed resize nodes and the agreement with the .tflite."""
    onnx = _require("onnx")
    tf2onnx = _require("tf2onnx")

    model, _ = tf2onnx.convert.from_tflite(str(tflite), opset=opset)
    fixed = fix_fractional_resizes(model)
    onnx.checker.check_model(model)
    out.parent.mkdir(parents=True, exist_ok=True)
    onnx.save(model, str(out))
    return fixed, check_against_tflite(tflite, out)

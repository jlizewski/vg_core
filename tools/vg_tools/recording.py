"""Reading capture sessions (docs/capture-format.md) from Python, for analysis.

Needs the `analysis` extra: pip install -e "tools[analysis]" (mcap with zstd,
numpy, Pillow). The C++ reader (vg_core_io) is what mapping uses; this is for
looking at recordings: plotting, checking labels against images, and so on.
"""

from __future__ import annotations

import io
from collections.abc import Iterable, Iterator
from dataclasses import dataclass
from pathlib import Path


def _require(module: str):
    try:
        return __import__(module)
    except ImportError as e:
        raise SystemExit(
            f'vg: this needs the analysis extra ({e.name} is missing): '
            f'pip install -e "tools[analysis]"'
        ) from e


@dataclass
class Message:
    topic: str
    time: int  # ns on the session clock
    proto: object  # The decoded protobuf message


def messages(path: Path, topics: Iterable[str] | None = None) -> Iterator[Message]:
    """Every message of the given topics (all if None), in file order.

    Works on recordings that were never closed (no index), as the app leaves
    them when it's killed, by reading the file front to back.
    """
    _require("mcap")
    _require("mcap_protobuf")
    from mcap.reader import NonSeekingReader
    from mcap_protobuf.decoder import DecoderFactory

    wanted = set(topics) if topics is not None else None
    with open(path, "rb") as f:
        reader = NonSeekingReader(f, decoder_factories=[DecoderFactory()])
        for _schema, channel, message, proto in reader.iter_decoded_messages(
            topics=list(wanted) if wanted else None
        ):
            yield Message(channel.topic, message.log_time, proto)


def raw_image_array(proto):
    """A foxglove.RawImage as a numpy array: float32 meters for depth (16UC1
    millimeters or 32FC1 meters), uint8 for mono8 (confidence, labels)."""
    np = _require("numpy")
    kinds = {"16UC1": (np.uint16, 2), "32FC1": (np.float32, 4), "mono8": (np.uint8, 1),
             "8UC1": (np.uint8, 1)}
    if proto.encoding not in kinds:
        raise ValueError(f"unsupported image encoding {proto.encoding}")
    dtype, size = kinds[proto.encoding]
    step = proto.step or proto.width * size
    rows = np.frombuffer(proto.data, dtype=np.uint8).reshape(proto.height, step)
    image = rows[:, : proto.width * size].copy().view(dtype).reshape(proto.height, proto.width)
    if proto.encoding == "16UC1":
        return image.astype(np.float32) * 0.001
    return image


def compressed_image(proto):
    """A foxglove.CompressedImage (JPEG / PNG) as a Pillow RGB image."""
    pil = _require("PIL")
    from PIL import Image  # noqa: F401  (imported by _require)

    return pil.Image.open(io.BytesIO(proto.data)).convert("RGB")

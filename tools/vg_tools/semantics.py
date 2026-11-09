"""vg_core's semantic classes (include/vg_core/semantics.hpp), for the tools:
the class id is the index, with its name and display color. Keep in step with
semantics.hpp and semantics.cpp."""

from __future__ import annotations

CLASSES: tuple[tuple[str, tuple[int, int, int]], ...] = (
    ("unknown", (128, 128, 128)),
    ("ground", (150, 110, 60)),
    ("plant", (90, 200, 60)),
    ("tree", (20, 110, 30)),
    ("structure", (200, 60, 50)),
    ("sky", (110, 170, 240)),
    ("dynamic", (240, 200, 0)),
)


def color(class_id: int) -> tuple[int, int, int]:
    return CLASSES[class_id][1] if 0 <= class_id < len(CLASSES) else CLASSES[0][1]

from __future__ import annotations

import numpy as np

from f1tenth_raceline.edit_model import apply_profile


def test_empty_edit_profile_is_pixel_identical() -> None:
    base = np.arange(100, dtype=np.uint8).reshape(10, 10)
    edited = apply_profile(base, {"operations": []})
    assert np.array_equal(edited, base)
    assert edited is not base


def test_free_and_occupied_strokes_only_change_masked_pixels() -> None:
    base = np.full((20, 20), 127, dtype=np.uint8)
    profile = {
        "operations": [
            {"tool": "free", "radius": 2, "points": [[5, 5]]},
            {"tool": "occupied", "radius": 2, "points": [[15, 15]]},
        ]
    }
    edited = apply_profile(base, profile)
    assert edited[5, 5] == 255
    assert edited[15, 15] == 0
    assert edited[10, 10] == 127

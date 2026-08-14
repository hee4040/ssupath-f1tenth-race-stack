from __future__ import annotations

import numpy as np

_PATCH_ATTR = "_f1tenth_scipy_vector_compat"


def apply_tph_spline_approximation_compat() -> bool:
    """Patch TPH 0.80 for modern SciPy vector validation."""
    import trajectory_planning_helpers as tph

    module = tph.spline_approximation
    current = module.dist_to_p
    if getattr(current, _PATCH_ATTR, False):
        return False

    original = current

    def dist_to_p_1d(t_glob: np.ndarray, path: list, p: np.ndarray):
        t = np.asarray(t_glob, dtype=float).reshape(-1)
        if t.size != 1:
            raise ValueError(
                "TPH spline compatibility patch expected one spline parameter, "
                f"got shape {np.asarray(t_glob).shape}."
            )
        s = module.interpolate.splev(float(t[0]), path)
        p_vec = np.asarray(p, dtype=float).reshape(-1)
        s_vec = np.asarray(s, dtype=float).reshape(-1)
        return module.spatial.distance.euclidean(p_vec, s_vec)

    setattr(dist_to_p_1d, _PATCH_ATTR, True)
    setattr(dist_to_p_1d, "_f1tenth_original", original)
    module.dist_to_p = dist_to_p_1d
    return True


def verify_tph_spline_approximation_compat() -> None:
    """Small numerical smoke test for the patched scalar/array behavior."""
    import trajectory_planning_helpers as tph

    apply_tph_spline_approximation_compat()
    module = tph.spline_approximation

    x = np.array([0.0, 1.0, 1.0, 0.0, 0.0], dtype=float)
    y = np.array([0.0, 0.0, 1.0, 1.0, 0.0], dtype=float)
    tck, _ = module.interpolate.splprep([x, y], s=0.0, per=True)
    p = np.array([0.5, 0.2], dtype=float)
    d_from_array = module.dist_to_p(np.array([0.2]), tck, p)
    d_from_scalar = module.dist_to_p(0.2, tck, p)

    if not np.isfinite(d_from_array):
        raise RuntimeError("TPH compatibility distance is not finite.")
    if not np.isclose(d_from_array, d_from_scalar, rtol=0.0, atol=1e-12):
        raise RuntimeError(
            "TPH compatibility patch changed scalar-vs-array distance behavior."
        )

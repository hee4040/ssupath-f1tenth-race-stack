import numpy as np


class CCMA:
    """Small centered moving-average smoother compatible with the ccma package API used here."""

    def __init__(self, w_ma=5, w_cc=3):
        self.w_ma = max(1, int(w_ma))
        self.w_cc = max(1, int(w_cc))

    def _smooth_once(self, values, window):
        if len(values) == 0 or window <= 1:
            return values
        radius = window // 2
        padded = np.pad(values, ((radius, radius), (0, 0)), mode="edge")
        result = np.empty_like(values, dtype=float)
        for idx in range(len(values)):
            result[idx] = padded[idx:idx + window].mean(axis=0)
        return result

    def filter(self, points):
        smoothed = np.asarray(points, dtype=float)
        smoothed = self._smooth_once(smoothed, self.w_ma)
        smoothed = self._smooth_once(smoothed, self.w_cc)
        return smoothed

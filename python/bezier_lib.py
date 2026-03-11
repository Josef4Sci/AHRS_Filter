from __future__ import annotations

from dataclasses import dataclass
import numpy as np


def _barycentric_weights(x: np.ndarray) -> np.ndarray:
    """
    Compute barycentric weights for distinct nodes x.
    O(n^2) time, O(n) memory.
    """
    x = np.asarray(x, dtype=float)
    n = x.size
    w = np.ones(n, dtype=float)
    for j in range(n):
        diff = x[j] - np.delete(x, j)
        if np.any(diff == 0.0):
            raise ValueError("x values must be distinct")
        w[j] = 1.0 / np.prod(diff)
    return w


def _barycentric_eval(x_nodes: np.ndarray, y_nodes: np.ndarray, w: np.ndarray, xq):
    """
    Evaluate barycentric interpolant defined on (x_nodes, y_nodes) with weights w.
    Vectorized in xq.
    """
    x_nodes = np.asarray(x_nodes, dtype=float)
    y_nodes = np.asarray(y_nodes, dtype=float)
    w = np.asarray(w, dtype=float)

    xq = np.asarray(xq, dtype=float)
    xq_flat = xq.ravel()

    out = np.empty_like(xq_flat, dtype=float)

    for k, x in enumerate(xq_flat):
        # If x hits a node exactly, return that y (avoid division by zero)
        idx = np.where(x == x_nodes)[0]
        if idx.size:
            out[k] = y_nodes[idx[0]]
            continue

        diff = x - x_nodes
        t = w / diff
        out[k] = np.dot(t, y_nodes) / np.sum(t)

    return out.reshape(xq.shape)


@dataclass(frozen=True)
class PolynomialInterpolator:
    """
    Polynomial interpolator with slow init (precompute) and fast eval.

    Parameters
    ----------
    x, y : 1D arrays of data points (x must be distinct)
    order : int or None
        - None: global polynomial through all points (degree n-1)
        - k (>=0): local polynomial of degree k using k+1 nearest nodes per query
                  (precomputes weights for all sliding windows of size k+1)
    assume_sorted : bool
        If False, points are sorted by x at init.
    """
    x: np.ndarray
    y: np.ndarray
    order: int | None = None
    assume_sorted: bool = False

    def __post_init__(self):
        x = np.asarray(self.x, dtype=float).copy()
        y = np.asarray(self.y, dtype=float).copy()

        if x.ndim != 1 or y.ndim != 1:
            raise ValueError("x and y must be 1D arrays")
        if x.size != y.size:
            raise ValueError("x and y must have the same length")
        if x.size < 2:
            raise ValueError("need at least 2 points")

        if not self.assume_sorted:
            p = np.argsort(x)
            x, y = x[p], y[p]

        if np.any(np.diff(x) == 0.0):
            raise ValueError("x values must be distinct")

        object.__setattr__(self, "x", x)
        object.__setattr__(self, "y", y)

        n = x.size
        if self.order is None:
            # global: degree n-1
            w = _barycentric_weights(x)
            object.__setattr__(self, "_mode", "global")
            object.__setattr__(self, "_w_global", w)
        else:
            k = int(self.order)
            if k < 0:
                raise ValueError("order must be >= 0 or None")
            m = k + 1
            if m > n:
                raise ValueError(f"order={k} requires at least {m} points (got {n})")

            # Precompute weights for every contiguous window of size m (slow init).
            # Evaluation uses nearest window index (fast).
            w_windows = np.empty((n - m + 1, m), dtype=float)
            for i in range(n - m + 1):
                w_windows[i] = _barycentric_weights(x[i:i + m])

            object.__setattr__(self, "_mode", "local")
            object.__setattr__(self, "_m", m)
            object.__setattr__(self, "_w_windows", w_windows)
            
           

    def __call__(self, xq):
        xq = np.asarray(xq, dtype=float)

        if getattr(self, "_mode") == "global":
            return _barycentric_eval(self.x, self.y, getattr(self, "_w_global"), xq)

        # local mode: choose a contiguous window of size m near xq
        m = getattr(self, "_m")
        w_windows = getattr(self, "_w_windows")
        x_nodes = self.x
        y_nodes = self.y
        n = x_nodes.size

        xq_flat = xq.ravel()
        out = np.empty_like(xq_flat, dtype=float)

        for k, x in enumerate(xq_flat):
            # Find insertion point
            j = int(np.searchsorted(x_nodes, x, side="left"))

            # Center a window of size m around x
            start = j - m // 2
            start = max(0, min(start, n - m))

            xs = x_nodes[start:start + m]
            ys = y_nodes[start:start + m]
            ws = w_windows[start]

            # Evaluate local barycentric interpolant
            idx = np.where(x == xs)[0]
            if idx.size:
                out[k] = ys[idx[0]]
            else:
                diff = x - xs
                t = ws / diff
                out[k] = np.dot(t, ys) / np.sum(t)

        return out.reshape(xq.shape)
    
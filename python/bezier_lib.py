
from __future__ import annotations

import numpy as np
from numpy.typing import NDArray
from numba import jit

ArrayF = NDArray[np.floating]

@jit(nopython=True, cache=True)
def _as_points(P) -> ArrayF:
    P = np.asarray(P, dtype=float)
    if P.ndim != 2:
        raise ValueError(f"P must have shape (n, d). Got {P.shape=}")
    if P.shape[0] < 2:
        raise ValueError("Need at least 2 control points.")
    return P

@jit(nopython=True, cache=True)
def de_casteljau(P: ArrayF, u: float) -> ArrayF:
    """Evaluate Bézier at scalar u using De Casteljau (stable)."""
    Q = P.copy()
    n = Q.shape[0]
    for r in range(1, n):
        Q[: n - r] = (1.0 - u) * Q[: n - r] + u * Q[1 : n - r + 1]
    return Q[0]


@jit(nopython=True, cache=True)
def derivative_control_points(P: ArrayF) -> ArrayF:
    """
    Control points of the first derivative curve w.r.t. normalized parameter u in [0,1]:
      dB/du is a Bézier of degree n-2 with control points:
        D_i = (n-1) * (P_{i+1} - P_i)
    """
    P = _as_points(P)
    deg = P.shape[0] - 1
    return deg * (P[1:] - P[:-1])


@jit(nopython=True, cache=True)
def bezier_linear_extrap_pointscale_vector(P, t) -> ArrayF:
    """
    Evaluate Bézier with linear extrapolation, where t is in *point index scale*.

    Control points correspond to:
      t = 0 -> start
      t = n-1 -> end

    Returns:
      (d,) for scalar t
      (m, d) for array t of length m
    """
    P = _as_points(P)
    n = P.shape[0]
    L = float(n - 1)  # max t in point scale

    t = np.asarray(t, dtype=float)

    # Precompute endpoints in curve space
    B0 = de_casteljau(P, 0.0)
    B1 = de_casteljau(P, 1.0)

    # Endpoint derivatives with respect to normalized u
    D = derivative_control_points(P)
    dBdu_0 = de_casteljau(D, 0.0)
    dBdu_1 = de_casteljau(D, 1.0)

    # Convert to derivatives w.r.t point-scale t:
    # u = t/L  =>  dB/dt = (dB/du) * (du/dt) = (dB/du) * (1/L)
    dBdt_0 = dBdu_0 / L
    dBdt_1 = dBdu_1 / L

    def _eval_scalar(ts: float) -> ArrayF:
        if ts < 0.0:
            return B0 + ts * dBdt_0
        if ts > L:
            return B1 + (ts - L) * dBdt_1

        u = ts / L  # normalize for in-range evaluation
        return de_casteljau(P, u)

    out = np.empty((t.size, P.shape[1]), dtype=float)
    for i, ti in enumerate(t):
        out[i] = _eval_scalar(float(ti))
    return out


@jit(nopython=True, cache=True)
def bezier_linear_extrap_pointscale_scalar(P, t) -> ArrayF:
    """
    Evaluate Bézier with linear extrapolation, where t is in *point index scale*.

    Control points correspond to:
      t = 0 -> start
      t = n-1 -> end

    Returns:
      (d,) for scalar t
      (m, d) for array t of length m
    """
    P = _as_points(P)
    n = P.shape[0]
    L = float(n - 1)  # max t in point scale

    t = np.asarray(t, dtype=float)

    # Precompute endpoints in curve space
    B0 = de_casteljau(P, 0.0)
    B1 = de_casteljau(P, 1.0)

    # Endpoint derivatives with respect to normalized u
    D = derivative_control_points(P)
    dBdu_0 = de_casteljau(D, 0.0)
    dBdu_1 = de_casteljau(D, 1.0)

    # Convert to derivatives w.r.t point-scale t:
    # u = t/L  =>  dB/dt = (dB/du) * (du/dt) = (dB/du) * (1/L)
    dBdt_0 = dBdu_0 / L
    dBdt_1 = dBdu_1 / L

    ts  = float(t)
    if ts < 0.0:
        return B0 + ts * dBdt_0
    if ts > L:
        return B1 + (ts - L) * dBdt_1

    u = ts / L  # normalize for in-range evaluation
    return de_casteljau(P, u)

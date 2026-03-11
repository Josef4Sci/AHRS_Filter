"""
Bézier curve evaluation with *linear* extrapolation outside the control-point
parameter range, plus plotting.

Key behavior (what you asked for):
- You pass control points P (shape (n, d)) and a parameter t in the *point scale*
  where control points are indexed at t = 0, 1, ..., n-1.
- Internally we normalize to u in [0, 1] using: u = t / (n-1).
- Inside the range: standard Bézier evaluation at u.
- Outside the range: linear extrapolation using endpoint derivative, but in the
  original t scale, so the slope is correct in "points per index".

Meaning:
  If t < 0:
     B(t) = B(0) + t * dB/dt|_{t=0}
  If t > n-1:
     B(t) = B(n-1) + (t-(n-1)) * dB/dt|_{t=n-1}
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

ArrayF = NDArray[np.floating]
from bezier_lib import BezierLinearExtrapPointScale, bezier_linear_extrap_pointscale_scalar


def main():
 # Example usage + plot
    P = np.array(
        [
            [0.0, 0.0],
            [0.05, 1.0],
            [0.1, 1.0],
            [1.0, 1.0],
        ],
        dtype=np.float64,
    )

    curve = BezierLinearExtrapPointScale(P)

    ts = np.array([ 0.0, 0.25, 0.5, 4.0], dtype=np.float64)
    #pts = curve(ts)

    # Plot
    import matplotlib.pyplot as plt

    t_dense = np.linspace(ts.min(), ts.max(), 400)
    pts_dense = curve(t_dense)

    plt.figure()
    plt.plot(pts_dense[:, 0], pts_dense[:, 1], label="Bezier (linear extrap)")
    # plt.plot(ts, pts[:, 1], "o", label="Samples at ts")
    plt.plot(P[:, 0], P[:, 1], "k--o", alpha=0.5, label="Control polygon")
    plt.axis("equal")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.title("Cached Bézier with linear extrapolation (point-scale parameter)")
    plt.show()


if __name__ == "__main__":
    main()
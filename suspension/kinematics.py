"""
Position and velocity analysis of the four-bar rear suspension.

The rocker rotation (about the frame pivot D) is used as the independent
coordinate. For every rocker angle the loop-closure equations of the four-bar
A-B-C-D are solved in closed form (intersection of two circles), and the
positions of the rear axle and the shock eye follow from rigid-body motion.

Velocities are obtained analytically from the velocity loop equations, which
gives an exact leverage ratio without numerical differentiation.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.optimize import brentq

from .geometry import BikeGeometry


# ---------------------------------------------------------------------------
# Small 2D helpers
# ---------------------------------------------------------------------------
def rotate(v: np.ndarray, angle: float) -> np.ndarray:
    """Rotate a 2D vector counter-clockwise by `angle` (rad)."""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


def omega_cross(omega: float, r: np.ndarray) -> np.ndarray:
    """Velocity of a point at position r on a body rotating at omega (planar)."""
    return omega * np.array([-r[1], r[0]])


def circle_intersections(c0, r0, c1, r1) -> tuple[np.ndarray, np.ndarray]:
    """Both intersection points of two circles (centre, radius)."""
    c0, c1 = np.asarray(c0, float), np.asarray(c1, float)
    d_vec = c1 - c0
    d = np.linalg.norm(d_vec)
    if d > r0 + r1 or d < abs(r0 - r1) or d == 0.0:
        raise ValueError("Linkage cannot be assembled at this position.")
    a = (r0**2 - r1**2 + d**2) / (2.0 * d)
    h = np.sqrt(max(r0**2 - a**2, 0.0))
    base = c0 + a * d_vec / d
    perp = np.array([-d_vec[1], d_vec[0]]) / d
    return base + h * perp, base - h * perp


def line_intersection(p1, p2, q1, q2) -> np.ndarray:
    """Intersection of line p1-p2 with line q1-q2 (NaN if parallel)."""
    p1, p2, q1, q2 = (np.asarray(x, float) for x in (p1, p2, q1, q2))
    r, s = p2 - p1, q2 - q1
    denom = r[0] * s[1] - r[1] * s[0]
    if abs(denom) < 1e-12:
        return np.array([np.nan, np.nan])
    t = ((q1 - p1)[0] * s[1] - (q1 - p1)[1] * s[0]) / denom
    return p1 + t * r


# ---------------------------------------------------------------------------
# Linkage state
# ---------------------------------------------------------------------------
@dataclass
class LinkageState:
    rocker_angle: float      # rocker rotation from top-out (rad)
    B: np.ndarray            # Horst pivot
    C: np.ndarray            # seatstay-rocker pivot
    F: np.ndarray            # shock eye on rocker
    axle: np.ndarray         # rear axle
    seatstay_angle: float    # seatstay rotation from top-out (rad)

    def shock_length(self, geom: BikeGeometry) -> float:
        return float(np.linalg.norm(self.F - geom.shock_frame_mount))


class FourBarSuspension:
    def __init__(self, geom: BikeGeometry):
        self.g = geom
        self._sign = self._compression_direction()

    # -- position analysis --------------------------------------------------
    def solve(self, rocker_angle: float, B_guess: np.ndarray | None = None) -> LinkageState:
        g = self.g
        D = g.rocker_pivot
        C = D + rotate(g.seatstay_rocker_pivot - D, rocker_angle)
        F = D + rotate(g.shock_rocker_mount - D, rocker_angle)

        b1, b2 = circle_intersections(g.main_pivot, g.chainstay_length,
                                      C, g.seatstay_length)
        ref = g.horst_pivot if B_guess is None else B_guess
        B = b1 if np.linalg.norm(b1 - ref) < np.linalg.norm(b2 - ref) else b2

        v0 = g.seatstay_rocker_pivot - g.horst_pivot
        v1 = C - B
        phi = np.arctan2(v1[1], v1[0]) - np.arctan2(v0[1], v0[0])
        axle = B + rotate(g.rear_axle - g.horst_pivot, phi)
        return LinkageState(rocker_angle, B, C, F, axle, phi)

    def _compression_direction(self) -> float:
        """+1 if a positive rocker rotation compresses the shock, else -1."""
        s0 = self.solve(0.0).shock_length(self.g)
        s1 = self.solve(1e-4).shock_length(self.g)
        return 1.0 if s1 < s0 else -1.0

    def bottom_out_angle(self) -> float:
        """Rocker rotation at which the shock has used its full stroke."""
        g = self.g
        target = g.shock_eye_to_eye - g.shock_stroke

        def f(t):
            return self.solve(self._sign * t).shock_length(g) - target

        t_max = 0.05
        while f(t_max) > 0:
            t_max += 0.05
            if t_max > np.pi / 2:
                raise ValueError("Shock stroke cannot be reached by this linkage.")
        return self._sign * brentq(f, 0.0, t_max, xtol=1e-12)

    def sweep(self, n: int = 201) -> list[LinkageState]:
        """Linkage states from top-out to bottom-out (branch-tracked)."""
        angles = np.linspace(0.0, self.bottom_out_angle(), n)
        states, B_prev = [], None
        for a in angles:
            st = self.solve(a, B_prev)
            states.append(st)
            B_prev = st.B
        return states

    # -- velocity analysis --------------------------------------------------
    def velocities(self, st: LinkageState, omega_rocker: float = 1.0) -> dict:
        """
        Exact velocity analysis for a given rocker angular velocity.

        Unknowns: chainstay (w_cs) and seatstay (w_ss) angular velocities,
        from the closure condition v_B(chainstay) = v_C + w_ss x (B - C).
        """
        g = self.g
        A, D, E = g.main_pivot, g.rocker_pivot, g.shock_frame_mount
        vC = omega_cross(omega_rocker, st.C - D)
        rAB, rCB = st.B - A, st.B - st.C
        # w_cs * perp(rAB) - w_ss * perp(rCB) = vC
        M = np.column_stack(([-rAB[1], rAB[0]], [rCB[1], -rCB[0]]))
        w_cs, w_ss = np.linalg.solve(M, vC)

        v_axle = vC + omega_cross(w_ss, st.axle - st.C)
        vF = omega_cross(omega_rocker, st.F - D)
        e = (st.F - E) / np.linalg.norm(st.F - E)
        shock_rate = float(e @ vF)  # d(shock length)/dt
        return dict(w_chainstay=w_cs, w_seatstay=w_ss,
                    v_axle=v_axle, shock_rate=shock_rate)

    def instant_center(self, st: LinkageState) -> np.ndarray:
        """Instant centre of the seatstay relative to the frame (lines AB and DC)."""
        return line_intersection(self.g.main_pivot, st.B, self.g.rocker_pivot, st.C)

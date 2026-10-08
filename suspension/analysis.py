"""
Suspension characteristics computed over the full travel.

* Vertical wheel travel and axle path
* Leverage ratio (wheel travel rate / shock compression rate) and progression
* Instant centre of the rear-wheel-carrying link (seatstay)
* Anti-squat, using the chain-force / instant-centre construction
* Chain growth (bottom bracket to rear axle distance)

Simplifications: planar model, frame held fixed (no pitch or sag of the
front end), rigid links, ideal pin joints. These are the same assumptions
used by common suspension-design tools for comparing linkages.
"""

from __future__ import annotations

import numpy as np

from .geometry import BikeGeometry
from .kinematics import FourBarSuspension, line_intersection


def upper_chain_line(geom: BikeGeometry, axle: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    Tangent points of the upper (driving) chain run between chainring and cog.

    For two circles below a common tangent with upward unit normal n:
        n . (c_cog - c_ring) = r_ring - r_cog
    """
    c1, r1 = np.zeros(2), geom.drivetrain.chainring_radius
    c2, r2 = axle, geom.drivetrain.cog_radius
    d = c2 - c1
    L = np.linalg.norm(d)
    u = d / L
    u_perp = np.array([-u[1], u[0]])
    cos_a = (r1 - r2) / L
    sin_a = np.sqrt(1.0 - cos_a**2)
    n = cos_a * u + sin_a * u_perp
    if n[1] < 0:
        n = cos_a * u - sin_a * u_perp
    return c1 + r1 * n, c2 + r2 * n


def anti_squat(geom: BikeGeometry, axle: np.ndarray, ic: np.ndarray) -> float:
    """
    Anti-squat percentage.

    1. Intersect the upper chain line with the line from the rear axle through
       the instant centre of the seatstay.
    2. Draw a line from the rear tyre contact patch through that point.
    3. Anti-squat = height of that line above the front contact patch divided
       by the centre of mass height (x100).
    """
    t_ring, t_cog = upper_chain_line(geom, axle)
    p = line_intersection(t_ring, t_cog, axle, ic)
    rear_contact = axle - np.array([0.0, geom.wheel_radius])
    front_axle = geom.rear_axle + np.array([geom.wheelbase, 0.0])
    front_contact = front_axle - np.array([0.0, geom.wheel_radius])

    if np.any(np.isnan(p)):
        return float("nan")
    slope = (p[1] - rear_contact[1]) / (p[0] - rear_contact[0])
    h = rear_contact[1] + slope * (front_contact[0] - rear_contact[0]) - front_contact[1]
    return 100.0 * h / geom.cog_height


def analyse(geom: BikeGeometry, n: int = 201) -> dict[str, np.ndarray]:
    """Run the full kinematic analysis and return arrays over the travel."""
    model = FourBarSuspension(geom)
    states = model.sweep(n)

    axle = np.array([s.axle for s in states])
    shock_len = np.array([s.shock_length(geom) for s in states])
    ic = np.array([model.instant_center(s) for s in states])

    lr, as_pct = np.empty(n), np.empty(n)
    for i, s in enumerate(states):
        v = model.velocities(s)
        lr[i] = v["v_axle"][1] / -v["shock_rate"]
        as_pct[i] = anti_squat(geom, s.axle, ic[i])

    travel = axle[:, 1] - axle[0, 1]
    shock_travel = shock_len[0] - shock_len
    progression = 100.0 * (lr[0] - lr[-1]) / lr[0]

    return dict(
        states=states,
        wheel_travel=travel,
        axle_x=axle[:, 0],
        axle_y=axle[:, 1],
        shock_travel=shock_travel,
        shock_length=shock_len,
        leverage_ratio=lr,
        instant_center=ic,
        anti_squat=as_pct,
        chain_growth=np.linalg.norm(axle, axis=1) - np.linalg.norm(axle[0]),
        summary=dict(
            rear_travel_mm=float(travel[-1]),
            shock_stroke_mm=float(shock_travel[-1]),
            mean_leverage_ratio=float(travel[-1] / shock_travel[-1]),
            leverage_ratio_start=float(lr[0]),
            leverage_ratio_end=float(lr[-1]),
            progression_pct=float(progression),
            anti_squat_at_sag_pct=float(np.interp(0.30 * travel[-1], travel, as_pct)),
            axle_rearward_shift_mm=float(axle[0, 0] - axle[:, 0].min()),
        ),
    )

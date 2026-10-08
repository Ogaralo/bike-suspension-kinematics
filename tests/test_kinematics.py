"""
Consistency checks of the kinematic model.

Run with:  python -m pytest tests      (or simply: python tests/test_kinematics.py)
"""

import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from suspension import BikeGeometry, FourBarSuspension, analyse  # noqa: E402
from suspension.analysis import upper_chain_line  # noqa: E402
from suspension.kinematics import omega_cross  # noqa: E402

GEOM = BikeGeometry.from_json(ROOT / "geometries" / "trail_29_horst_link.json")


def test_link_lengths_are_constant():
    """Rigid links must keep their length over the whole travel."""
    g = GEOM
    for st in FourBarSuspension(g).sweep(101):
        assert abs(np.linalg.norm(st.B - g.main_pivot) - g.chainstay_length) < 1e-9
        assert abs(np.linalg.norm(st.C - st.B) - g.seatstay_length) < 1e-9
        assert abs(np.linalg.norm(st.C - g.rocker_pivot) - g.rocker_length) < 1e-9
        # the axle is rigidly attached to the seatstay
        assert abs(np.linalg.norm(st.axle - st.B)
                   - np.linalg.norm(g.rear_axle - g.horst_pivot)) < 1e-9


def test_full_shock_stroke_is_used():
    states = FourBarSuspension(GEOM).sweep(51)
    used = states[0].shock_length(GEOM) - states[-1].shock_length(GEOM)
    assert abs(used - GEOM.shock_stroke) < 0.05  # mounts are rounded to 0.1 mm


def test_leverage_ratio_matches_finite_differences():
    """Analytic velocity analysis vs. numerical derivative of positions."""
    res = analyse(GEOM, n=2001)
    lr_fd = np.gradient(res["wheel_travel"], res["shock_travel"])
    inner = slice(5, -5)
    assert np.max(np.abs(lr_fd[inner] - res["leverage_ratio"][inner])) < 1e-4


def test_instant_centre_has_zero_velocity():
    """The seatstay point located at the instant centre must not move."""
    model = FourBarSuspension(GEOM)
    for st in model.sweep(21):
        ic = model.instant_center(st)
        v = model.velocities(st)
        v_ic = omega_cross(1.0, st.C - GEOM.rocker_pivot) \
            + omega_cross(v["w_seatstay"], ic - st.C)
        assert np.linalg.norm(v_ic) < 1e-6 * np.linalg.norm(ic)


def test_single_pivot_limit_gives_circular_axle_path():
    """With the axle on the Horst pivot the bike becomes a single pivot."""
    g = replace(GEOM, rear_axle=GEOM.horst_pivot.copy())
    r = [np.linalg.norm(st.axle - g.main_pivot) for st in FourBarSuspension(g).sweep(51)]
    assert np.ptp(r) < 1e-9


def test_chain_line_is_tangent_to_both_sprockets():
    axle = GEOM.rear_axle
    t1, t2 = upper_chain_line(GEOM, axle)
    d = (t2 - t1) / np.linalg.norm(t2 - t1)
    n = np.array([-d[1], d[0]])
    assert abs(abs(n @ (np.zeros(2) - t1)) - GEOM.drivetrain.chainring_radius) < 1e-9
    assert abs(abs(n @ (axle - t1)) - GEOM.drivetrain.cog_radius) < 1e-9
    assert t1[1] > 0 and t2[1] > axle[1]  # upper run, above both centres


def test_reference_results():
    """Regression values for the example geometry."""
    s = analyse(GEOM)["summary"]
    assert abs(s["rear_travel_mm"] - 139.3) < 0.5
    assert abs(s["progression_pct"] - 19.8) < 0.5
    assert abs(s["anti_squat_at_sag_pct"] - 105.4) < 1.0


if __name__ == "__main__":
    tests = [v for k, v in dict(globals()).items() if k.startswith("test_")]
    for t in tests:
        t()
        print(f"PASS  {t.__name__}")
    print(f"\n{len(tests)} tests passed")

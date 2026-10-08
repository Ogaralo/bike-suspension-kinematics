"""Figures and animation of the suspension analysis."""

from __future__ import annotations

from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Circle
import numpy as np

from .analysis import upper_chain_line
from .geometry import BikeGeometry

# Palette (colour-blind-safe pair) and neutral inks
BLUE = "#2a78d6"
ORANGE = "#eb6834"
INK = "#0b0b0b"
INK_2 = "#52514e"
GRID = "#e4e3df"
SURFACE = "#fcfcfb"
FRAME = "#8a8984"

plt.rcParams.update({
    "figure.facecolor": SURFACE,
    "axes.facecolor": SURFACE,
    "axes.edgecolor": INK_2,
    "axes.labelcolor": INK,
    "axes.titlecolor": INK,
    "axes.titleweight": "bold",
    "axes.grid": True,
    "grid.color": GRID,
    "grid.linewidth": 0.8,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "xtick.color": INK_2,
    "ytick.color": INK_2,
    "font.size": 10,
    "lines.linewidth": 2.0,
    "savefig.dpi": 160,
    "savefig.bbox": "tight",
})


def _sag_marker(ax, travel, values, sag_fraction=0.30):
    x = sag_fraction * travel[-1]
    y = float(np.interp(x, travel, values))
    ax.axvline(x, color=INK_2, lw=1, ls="--")
    ax.plot(x, y, "o", ms=8, color=BLUE, mec=SURFACE, mew=2, zorder=5)
    ax.annotate(f"{y:.2f}" if abs(y) < 10 else f"{y:.0f} %",
                (x, y), xytext=(8, 8), textcoords="offset points", color=INK)
    ax.text(x, ax.get_ylim()[1], " 30 % sag", va="top", ha="left",
            color=INK_2, fontsize=9)


def plot_summary(res: dict, out: Path) -> Path:
    t = res["wheel_travel"]
    fig, axs = plt.subplots(2, 2, figsize=(10, 7.2))

    ax = axs[0, 0]
    ax.plot(t, res["leverage_ratio"], color=BLUE)
    ax.set(title="Leverage ratio", xlabel="Vertical wheel travel [mm]",
           ylabel="Wheel travel / shock travel [-]")
    _sag_marker(ax, t, res["leverage_ratio"])

    ax = axs[0, 1]
    ax.plot(t, res["anti_squat"], color=BLUE)
    ax.axhline(100, color=INK_2, lw=1)
    ax.set(title="Anti-squat (32T x 21T)", xlabel="Vertical wheel travel [mm]",
           ylabel="Anti-squat [%]")
    _sag_marker(ax, t, res["anti_squat"])

    ax = axs[1, 0]
    ax.plot(res["axle_x"] - res["axle_x"][0], t, color=BLUE)
    ax.set(title="Axle path", xlabel="Horizontal axle displacement [mm]\n(negative = rearward)",
           ylabel="Vertical wheel travel [mm]")

    ax = axs[1, 1]
    ax.plot(t, res["chain_growth"], color=BLUE)
    ax.set(title="Chain growth (BB-to-axle distance)", xlabel="Vertical wheel travel [mm]",
           ylabel="Growth from top-out [mm]")

    s = res["summary"]
    fig.suptitle(
        f"Rear travel {s['rear_travel_mm']:.0f} mm  ·  shock stroke {s['shock_stroke_mm']:.0f} mm  ·  "
        f"progression {s['progression_pct']:.0f} %", color=INK, fontsize=11)
    fig.tight_layout()
    path = out / "results_summary.png"
    fig.savefig(path)
    plt.close(fig)
    return path


def _draw_linkage(ax, geom: BikeGeometry, st, ic=None, alpha=1.0, color=INK):
    A, D, E = geom.main_pivot, geom.rocker_pivot, geom.shock_frame_mount
    kw = dict(color=color, alpha=alpha, solid_capstyle="round")
    ax.plot(*np.c_[A, st.B], lw=4, **kw)                 # chainstay
    ax.plot(*np.c_[st.B, st.C], lw=4, **kw)              # seatstay
    ax.plot(*np.c_[st.B, st.axle], lw=4, **kw)           # dropout
    ax.fill(*np.c_[D, st.C, st.F], color=color, alpha=0.15 * alpha)
    ax.plot(*np.c_[D, st.C, st.F, D], lw=3, **kw)        # rocker
    ax.plot(*np.c_[E, st.F], lw=6, color=ORANGE, alpha=alpha, solid_capstyle="butt")  # shock
    for p in (A, D, E, st.B, st.C, st.F):
        ax.plot(*p, "o", ms=7, color=SURFACE, mec=color, mew=1.8, alpha=alpha, zorder=6)
    ax.add_patch(Circle(st.axle, geom.wheel_radius, fill=False, ec=FRAME,
                        lw=1.5, alpha=0.6 * alpha))
    if ic is not None and np.all(np.isfinite(ic)):
        ax.plot(*ic, "D", ms=7, color=BLUE, mec=SURFACE, mew=1.5, zorder=7)


def _draw_frame(ax, geom: BikeGeometry):
    """Simplified front triangle, for context only."""
    A, D, E = geom.main_pivot, geom.rocker_pivot, geom.shock_frame_mount
    bb = np.zeros(2)
    head_tube = np.array([420.0, 600.0])
    seat_top = D * (520.0 / D[1])            # seat tube passes through the rocker pivot
    for p, q in ((bb, head_tube), (bb, seat_top), (head_tube, seat_top),
                 (bb, A), (bb, E)):            # last one: shock-mount strut
        ax.plot(*np.c_[p, q], lw=7, color=FRAME, alpha=0.35, solid_capstyle="round")
    ax.add_patch(Circle((0, 0), geom.drivetrain.chainring_radius, fill=False,
                        ec=FRAME, lw=1.2))


def _setup_axes(ax, geom):
    ax.set_aspect("equal")
    ax.set_xlim(-860, 520)
    ax.set_ylim(-420, 720)
    ax.grid(False)
    ax.set_xticks([]); ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)


def plot_linkage(res: dict, geom: BikeGeometry, out: Path) -> Path:
    states = res["states"]
    fig, ax = plt.subplots(figsize=(9, 7.4))
    _setup_axes(ax, geom)
    _draw_frame(ax, geom)
    _draw_linkage(ax, geom, states[0], alpha=0.3, color=INK_2)
    _draw_linkage(ax, geom, states[-1], ic=res["instant_center"][-1])
    ax.plot(res["axle_x"], res["axle_y"], color=BLUE, lw=2)
    ic = res["instant_center"]
    ax.plot(ic[:, 0], ic[:, 1], color=BLUE, lw=1.5, ls=":")
    t_ring, t_cog = upper_chain_line(geom, states[-1].axle)
    ax.plot(*np.c_[t_ring, t_cog], color=INK_2, lw=1, ls="--")

    ax.annotate("Axle path", (res["axle_x"][-1], res["axle_y"][-1]),
                xytext=(-120, 40), textcoords="offset points", color=INK,
                arrowprops=dict(arrowstyle="-", color=INK_2, lw=1))
    ax.annotate("Instant centre path", tuple(ic[len(ic) // 2]), xytext=(10, -60),
                textcoords="offset points", color=INK,
                arrowprops=dict(arrowstyle="-", color=INK_2, lw=1))
    ax.annotate("Shock", tuple((geom.shock_frame_mount + states[-1].F) / 2),
                xytext=(40, 30), textcoords="offset points", color=INK,
                arrowprops=dict(arrowstyle="-", color=INK_2, lw=1))
    ax.set_title("Four-bar (Horst link) rear suspension: top-out (grey) and bottom-out (black)",
                 fontsize=10)
    path = out / "linkage.png"
    fig.savefig(path)
    plt.close(fig)
    return path


def animate(res: dict, geom: BikeGeometry, out: Path, frames: int = 50) -> Path:
    states = res["states"]
    idx = np.r_[np.linspace(0, len(states) - 1, frames // 2),
                np.linspace(len(states) - 1, 0, frames // 2)].astype(int)
    fig, ax = plt.subplots(figsize=(6.4, 5.3), dpi=90)

    def draw(k):
        ax.clear()
        _setup_axes(ax, geom)
        _draw_frame(ax, geom)
        i = idx[k]
        ax.plot(res["axle_x"], res["axle_y"], color=BLUE, lw=1.5, alpha=0.5)
        _draw_linkage(ax, geom, states[i], ic=res["instant_center"][i])
        ax.text(-840, 680, f"Wheel travel  {res['wheel_travel'][i]:5.0f} mm\n"
                           f"Leverage ratio {res['leverage_ratio'][i]:5.2f}\n"
                           f"Anti-squat   {res['anti_squat'][i]:5.0f} %",
                va="top", family="monospace", color=INK, fontsize=10)

    anim = FuncAnimation(fig, draw, frames=len(idx))
    path = out / "suspension.gif"
    anim.save(path, writer=PillowWriter(fps=15))
    plt.close(fig)
    return path

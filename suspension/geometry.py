"""
Geometry definition for a planar four-bar (Horst-link style) rear suspension.

Coordinate system
-----------------
* Origin at the bottom bracket (BB) centre.
* x axis pointing forwards (towards the front wheel), y axis pointing up.
* All lengths in millimetres.

Linkage topology (frame is the ground link)
-------------------------------------------
    A  main pivot (frame)          ── chainstay ──>  B  Horst pivot
    D  rocker pivot (frame)        ── rocker    ──>  C  seatstay/rocker pivot
    B ── seatstay (coupler) ── C   ; the rear axle is rigidly attached to the seatstay
    E  shock frame mount           ── shock     ──>  F  shock eye on the rocker

The rear axle sits on the coupler (seatstay), which is what makes it a
Horst-link / four-bar layout instead of a single pivot.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np


@dataclass
class Drivetrain:
    chainring_teeth: int = 32
    cog_teeth: int = 21
    chain_pitch: float = 12.7  # mm (1/2 inch)

    @staticmethod
    def pitch_radius(teeth: int, pitch: float) -> float:
        """Pitch radius of a sprocket: r = p / (2 sin(pi / N))."""
        return pitch / (2.0 * np.sin(np.pi / teeth))

    @property
    def chainring_radius(self) -> float:
        return self.pitch_radius(self.chainring_teeth, self.chain_pitch)

    @property
    def cog_radius(self) -> float:
        return self.pitch_radius(self.cog_teeth, self.chain_pitch)


@dataclass
class BikeGeometry:
    name: str
    # Hard points at top-out (fully extended shock), in mm
    main_pivot: np.ndarray          # A
    horst_pivot: np.ndarray         # B
    seatstay_rocker_pivot: np.ndarray  # C
    rocker_pivot: np.ndarray        # D
    shock_frame_mount: np.ndarray   # E
    shock_rocker_mount: np.ndarray  # F
    rear_axle: np.ndarray           # axle position at top-out
    # Shock
    shock_eye_to_eye: float = 210.0
    shock_stroke: float = 55.0
    # Whole-bike data used for anti-squat
    wheel_radius: float = 370.0       # 29" wheel with tyre
    wheelbase: float = 1230.0         # at top-out
    cog_height: float = 1150.0        # centre of mass height above ground
    drivetrain: Drivetrain = field(default_factory=Drivetrain)

    def __post_init__(self) -> None:
        for attr in ("main_pivot", "horst_pivot", "seatstay_rocker_pivot",
                     "rocker_pivot", "shock_frame_mount", "shock_rocker_mount",
                     "rear_axle"):
            setattr(self, attr, np.asarray(getattr(self, attr), dtype=float))

        measured = np.linalg.norm(self.shock_rocker_mount - self.shock_frame_mount)
        if abs(measured - self.shock_eye_to_eye) > 0.5:
            raise ValueError(
                f"Shock mounts are {measured:.1f} mm apart but the shock "
                f"eye-to-eye length is {self.shock_eye_to_eye:.1f} mm."
            )

    # Link lengths (constant while the suspension moves)
    @property
    def chainstay_length(self) -> float:
        return float(np.linalg.norm(self.horst_pivot - self.main_pivot))

    @property
    def seatstay_length(self) -> float:
        return float(np.linalg.norm(self.seatstay_rocker_pivot - self.horst_pivot))

    @property
    def rocker_length(self) -> float:
        return float(np.linalg.norm(self.seatstay_rocker_pivot - self.rocker_pivot))

    @classmethod
    def from_json(cls, path: str | Path) -> "BikeGeometry":
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        hp = data["hard_points_mm"]
        shock = data["shock_mm"]
        bike = data["bike"]
        dt = data.get("drivetrain", {})
        return cls(
            name=data["name"],
            main_pivot=hp["main_pivot"],
            horst_pivot=hp["horst_pivot"],
            seatstay_rocker_pivot=hp["seatstay_rocker_pivot"],
            rocker_pivot=hp["rocker_pivot"],
            shock_frame_mount=hp["shock_frame_mount"],
            shock_rocker_mount=hp["shock_rocker_mount"],
            rear_axle=hp["rear_axle"],
            shock_eye_to_eye=shock["eye_to_eye"],
            shock_stroke=shock["stroke"],
            wheel_radius=bike["wheel_radius"],
            wheelbase=bike["wheelbase"],
            cog_height=bike["cog_height"],
            drivetrain=Drivetrain(**dt),
        )

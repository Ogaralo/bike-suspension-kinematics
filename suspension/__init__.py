"""Planar kinematic analysis of a four-bar (Horst link) bicycle rear suspension."""

from .geometry import BikeGeometry, Drivetrain
from .kinematics import FourBarSuspension
from .analysis import analyse

__all__ = ["BikeGeometry", "Drivetrain", "FourBarSuspension", "analyse"]
__version__ = "1.0.0"

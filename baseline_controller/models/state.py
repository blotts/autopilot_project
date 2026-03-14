from dataclasses import dataclass


@dataclass
class AircraftState:
    theta: float = 0.0   # pitch angle [deg]
    q: float = 0.0       # pitch rate [deg/s]
    phi: float = 0.0     # roll angle [deg]
    p: float = 0.0       # roll rate [deg/s]
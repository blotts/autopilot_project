from dataclasses import dataclass

@dataclass
class Command:
    theta_cmd: float = 5.0   # desired pitch angle [deg]
    phi_cmd: float = 0.0     # desired roll angle [deg]

@dataclass
class ControlOutput:
    elevator: float = 0.0
    aileron: float = 0.0
    rudder: float = 0.0
    throttle: float = 0.0

@dataclass
class AircraftState:
    theta: float = 0.0   # pitch angle [deg]
    q: float = 0.0       # pitch rate [deg/s]
    phi: float = 0.0     # roll angle [deg]
    p: float = 0.0       # roll rate [deg/s]

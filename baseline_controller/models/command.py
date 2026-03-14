from dataclasses import dataclass


@dataclass
class Command:
    theta_cmd: float = 0.0   # desired pitch angle [deg]
    phi_cmd: float = 0.0     # desired roll angle [deg]
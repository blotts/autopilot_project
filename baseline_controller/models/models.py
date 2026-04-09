from dataclasses import dataclass


@dataclass
class Command:
    theta_cmd: float = 5.0            # desired pitch angle [deg]
    phi_cmd: float = 0.0              # desired roll angle [deg]
    heading_deg_cmd: float = 0.0      # desired heading [deg]
    altitude_ft_cmd: float = 0.0      # desired pressure altitude [ft]
    airspeed_kts_cmd: float = 0.0     # desired indicated airspeed [kt]


@dataclass
class ControlOutput:
    elevator: float = 0.0
    aileron: float = 0.0
    rudder: float = 0.0
    throttle: float = 0.0


@dataclass
class AircraftState:
    theta: float = 0.0                # pitch angle [deg]
    q: float = 0.0                    # pitch rate [deg/s]
    phi: float = 0.0                  # roll angle [deg]
    p: float = 0.0                    # roll rate [deg/s]
    psi: float = 0.0                  # heading [deg]
    r: float = 0.0                    # yaw rate [deg/s]
    altitude_ft: float = 0.0          # altitude [ft]
    vertical_speed_fpm: float = 0.0   # vertical speed [ft/min]
    airspeed_kts: float = 0.0         # indicated airspeed [kt]
    slip_deg: float = 0.0             # beta/slip indication [deg]
    throttle: float = 0.0             # throttle ratio [0,1]

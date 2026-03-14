from dataclasses import dataclass


@dataclass
class ControlOutput:
    elevator: float = 0.0
    aileron: float = 0.0
    rudder: float = 0.0
    throttle: float = 0.0
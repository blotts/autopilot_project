from XPPython3 import xp

from baseline_controller.models.state import AircraftState
from baseline_controller.models.control_output import ControlOutput


class XPlaneInterface:
    def __init__(self):
        # state datarefs
        self.theta_ref = xp.findDataRef('sim/flightmodel/position/theta')
        self.q_ref = xp.findDataRef('sim/flightmodel/position/Q')
        self.phi_ref = xp.findDataRef('sim/flightmodel/position/phi')
        self.p_ref = xp.findDataRef('sim/flightmodel/position/P')

        # control input datarefs
        self.yoke_pitch_ref = xp.findDataRef('sim/cockpit2/controls/yoke_pitch_ratio')
        self.yoke_roll_ref = xp.findDataRef('sim/cockpit2/controls/yoke_roll_ratio')

        self._check_datarefs()

    def _check_datarefs(self):
        required = {
            'theta_ref': self.theta_ref,
            'q_ref': self.q_ref,
            'phi_ref': self.phi_ref,
            'p_ref': self.p_ref,
            'yoke_pitch_ref': self.yoke_pitch_ref,
            'yoke_roll_ref': self.yoke_roll_ref,
        }

        missing = [name for name, ref in required.items() if ref is None]
        if missing:
            raise RuntimeError(f'Missing X-Plane datarefs: {missing}')

    def read_state(self) -> AircraftState:
        return AircraftState(
            theta=xp.getDataf(self.theta_ref),
            q=xp.getDataf(self.q_ref),
            phi=xp.getDataf(self.phi_ref),
            p=xp.getDataf(self.p_ref),
        )

    def write_control(self, output: ControlOutput):
        xp.setDataf(self.yoke_pitch_ref, float(output.elevator))
        xp.setDataf(self.yoke_roll_ref, float(output.aileron))
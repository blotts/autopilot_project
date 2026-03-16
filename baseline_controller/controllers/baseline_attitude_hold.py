from baseline_controller.controllers.base_controller import BaseController
from baseline_controller.utils.helpers import clamp


class BaselineAttitudeHoldController(BaseController):
    def __init__(self, name='baseline_attitude_hold'):
        super().__init__(name=name)

        # textbook baseline gains
        self.k_theta = 4.0
        self.k_q = 2.5
        self.k_phi = 5.0

        # limits
        self.max_elevator = 1.0
        self.max_aileron = 1.0

    def initialize(self, state=None, command=None):
        super().initialize(state=state, command=command)

    def compute_output(self, state, command, dt):
        output = self.zero_output()

        output.elevator = self.compute_elevator(state, command)
        output.aileron = self.compute_aileron(state, command)

        return output

    def compute_elevator(self, state, command):
        pitch_error = command.theta_cmd - state.theta
        elevator_cmd = self.k_theta * pitch_error - self.k_q * state.q
        return clamp(elevator_cmd, -self.max_elevator, self.max_elevator)

    def compute_aileron(self, state, command):
        roll_error = command.phi_cmd - state.phi
        aileron_cmd = self.k_phi * roll_error
        return clamp(aileron_cmd, -self.max_aileron, self.max_aileron)
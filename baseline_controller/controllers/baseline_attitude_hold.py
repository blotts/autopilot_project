from baseline_controller.controllers.base_controller import BaseController
from baseline_controller.utils.helpers import clamp


class BaselineAttitudeHoldController(BaseController):
    def __init__(self, name='baseline_attitude_hold'):
        super().__init__(name=name)

        # Tested gains
        self.k_theta = 0.06
        self.k_q = 0.12
        self.k_phi = 0.05

        # Max inputs to the aircraft
        self.max_elevator = 1.0
        self.max_aileron = 1.0

        # Output smoothing / rate limiting to stop elevator chatter.
        self.output_alpha = 0.25          # low-pass blend factor
        self.max_elevator_rate = 1.50     # yoke ratio per second
        self.max_aileron_rate = 1.80      # yoke ratio per second

        self.prev_elevator = 0.0
        self.prev_aileron = 0.0

    def reset(self):
        super().reset()
        self.prev_elevator = 0.0
        self.prev_aileron = 0.0

    def initialize(self, state=None, command=None):
        super().initialize(state=state, command=command)
        self.prev_elevator = 0.0
        self.prev_aileron = 0.0

    def compute_output(self, state, command, dt):
        output = self.zero_output()

        raw_elevator = self.compute_elevator(state, command)
        raw_aileron = self.compute_aileron(state, command)

        output.elevator = self._smooth_axis(
            target=raw_elevator,
            previous=self.prev_elevator,
            max_rate=self.max_elevator_rate,
            dt=dt,
        )
        output.aileron = self._smooth_axis(
            target=raw_aileron,
            previous=self.prev_aileron,
            max_rate=self.max_aileron_rate,
            dt=dt,
        )

        self.prev_elevator = output.elevator
        self.prev_aileron = output.aileron

        return output

    # Textbook controllers
    def compute_elevator(self, state, command):
        pitch_error = command.theta_cmd - state.theta
        elevator_cmd = self.k_theta * pitch_error - self.k_q * state.q       # PD controller
        return clamp(elevator_cmd, -self.max_elevator, self.max_elevator)

    def compute_aileron(self, state, command):
        roll_error = command.phi_cmd - state.phi
        aileron_cmd = self.k_phi * roll_error                                # P controller
        return clamp(aileron_cmd, -self.max_aileron, self.max_aileron)

    def _smooth_axis(self, target, previous, max_rate, dt):
        dt = max(dt, 1.0e-3)

        # First-order low-pass filter.
        filtered = previous + self.output_alpha * (target - previous)

        # Then rate-limit the change.
        max_step = max_rate * dt
        delta = clamp(filtered - previous, -max_step, max_step)

        return previous + delta
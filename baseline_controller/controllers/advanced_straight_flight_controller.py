from baseline_controller.controllers.base_controller import BaseController


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class AdvancedStraightFlightController(BaseController):
    """
    Cascaded straight-and-level autopilot for disturbance rejection.

    Outer loops:
      altitude error   -> pitch command
      heading error    -> bank command
      airspeed error   -> throttle command

    Inner loops:
      pitch command    -> elevator
      bank command     -> aileron
      slip / yaw rate  -> rudder

    This is designed to be practical with X-Plane datarefs rather than an exact
    aircraft model. Gains are intentionally conservative so it is more likely to
    work as a first test on a generic aircraft.
    """

    def __init__(self, name='advanced_straight_flight'):
        super().__init__(name=name)

        # -----------------------------
        # Outer-loop gains
        # -----------------------------
        # Altitude -> pitch command
        self.k_alt = 0.0035           # deg pitch per ft altitude error
        self.k_vs = 0.0022            # deg pitch per fpm vertical speed
        self.k_i_alt = 0.00045

        # Heading -> bank command
        self.k_psi = 1.35             # deg bank per deg heading error
        self.k_r_heading = 0.18       # deg bank per deg/s yaw rate
        self.k_i_psi = 0.030

        # Airspeed -> throttle
        self.trim_throttle = 0.55
        self.k_v = 0.018              # throttle per knot error
        self.k_i_v = 0.0035
        self.k_theta_ff = 0.010       # extra throttle when commanding nose-up

        # -----------------------------
        # Inner-loop gains
        # -----------------------------
        # Pitch inner loop
        self.k_theta = 0.070
        self.k_q = 0.140
        self.k_i_theta = 0.010

        # Roll inner loop
        self.k_phi = 0.080
        self.k_p = 0.180
        self.k_i_phi = 0.012

        # Rudder damping / slip suppression
        self.k_beta = 0.060
        self.k_r_rudder = 0.050

        # -----------------------------
        # Command limits
        # -----------------------------
        self.max_theta_cmd = 10.0     # deg
        self.max_phi_cmd = 20.0       # deg
        self.max_elevator = 1.0
        self.max_aileron = 1.0
        self.max_rudder = 1.0
        self.min_throttle = 0.0
        self.max_throttle = 1.0

        # -----------------------------
        # Integrator and anti-windup
        # -----------------------------
        self.integrator_leak = 0.12
        self.alt_int = 0.0
        self.psi_int = 0.0
        self.v_int = 0.0
        self.theta_int = 0.0
        self.phi_int = 0.0

        self.alt_int_limit = 3000.0
        self.psi_int_limit = 60.0
        self.v_int_limit = 40.0
        self.theta_int_limit = 20.0
        self.phi_int_limit = 20.0

        self.altitude_deadband_ft = 10.0
        self.heading_deadband_deg = 0.5
        self.speed_deadband_kts = 0.5
        self.theta_deadband_deg = 0.05
        self.phi_deadband_deg = 0.05

        # -----------------------------
        # Output smoothing / rate limiting
        # -----------------------------
        self.surface_alpha = 0.35
        self.throttle_alpha = 0.20
        self.max_elevator_rate = 1.50
        self.max_aileron_rate = 1.80
        self.max_rudder_rate = 1.50
        self.max_throttle_rate = 0.35

        self.prev_elevator = 0.0
        self.prev_aileron = 0.0
        self.prev_rudder = 0.0
        self.prev_throttle = self.trim_throttle

    def reset(self):
        super().reset()
        self.alt_int = 0.0
        self.psi_int = 0.0
        self.v_int = 0.0
        self.theta_int = 0.0
        self.phi_int = 0.0
        self.prev_elevator = 0.0
        self.prev_aileron = 0.0
        self.prev_rudder = 0.0
        self.prev_throttle = self.trim_throttle

    def initialize(self, state=None, command=None):
        super().initialize(state=state, command=command)
        self.alt_int = 0.0
        self.psi_int = 0.0
        self.v_int = 0.0
        self.theta_int = 0.0
        self.phi_int = 0.0
        self.prev_elevator = 0.0
        self.prev_aileron = 0.0
        self.prev_rudder = 0.0
        self.prev_throttle = clamp(self.trim_throttle, self.min_throttle, self.max_throttle)

    def compute_output(self, state, command, dt):
        dt = max(dt, 1.0e-3)
        output = self.zero_output()

        # ---------- Outer loops ----------
        alt_error = command.altitude_ft_cmd - state.altitude_ft
        if abs(alt_error) < self.altitude_deadband_ft:
            alt_error = 0.0
        self.alt_int = self._update_integrator(self.alt_int, alt_error, dt, self.alt_int_limit)
        theta_cmd = (
            self.k_alt * alt_error
            - self.k_vs * state.vertical_speed_fpm
            + self.k_i_alt * self.alt_int
        )
        theta_cmd = clamp(theta_cmd, -self.max_theta_cmd, self.max_theta_cmd)

        heading_error = self._wrap_angle_deg(command.heading_deg_cmd - state.psi)
        if abs(heading_error) < self.heading_deadband_deg:
            heading_error = 0.0
        self.psi_int = self._update_integrator(self.psi_int, heading_error, dt, self.psi_int_limit)
        phi_cmd = (
            self.k_psi * heading_error
            - self.k_r_heading * state.r
            + self.k_i_psi * self.psi_int
        )
        phi_cmd = clamp(phi_cmd, -self.max_phi_cmd, self.max_phi_cmd)

        speed_error = command.airspeed_kts_cmd - state.airspeed_kts
        if abs(speed_error) < self.speed_deadband_kts:
            speed_error = 0.0
        self.v_int = self._update_integrator(self.v_int, speed_error, dt, self.v_int_limit)
        throttle_cmd = (
            self.trim_throttle
            + self.k_v * speed_error
            + self.k_i_v * self.v_int
            + self.k_theta_ff * max(theta_cmd, 0.0)
        )
        throttle_cmd = clamp(throttle_cmd, self.min_throttle, self.max_throttle)

        # ---------- Inner loops ----------
        theta_error = theta_cmd - state.theta
        if abs(theta_error) < self.theta_deadband_deg:
            theta_error = 0.0
        self.theta_int = self._update_integrator(self.theta_int, theta_error, dt, self.theta_int_limit)
        elevator_cmd = (
            self.k_theta * theta_error
            - self.k_q * state.q
            + self.k_i_theta * self.theta_int
        )
        elevator_cmd = clamp(elevator_cmd, -self.max_elevator, self.max_elevator)

        phi_error = phi_cmd - state.phi
        if abs(phi_error) < self.phi_deadband_deg:
            phi_error = 0.0
        self.phi_int = self._update_integrator(self.phi_int, phi_error, dt, self.phi_int_limit)
        aileron_cmd = (
            self.k_phi * phi_error
            - self.k_p * state.p
            + self.k_i_phi * self.phi_int
        )
        aileron_cmd = clamp(aileron_cmd, -self.max_aileron, self.max_aileron)

        rudder_cmd = -self.k_beta * state.slip_deg - self.k_r_rudder * state.r
        rudder_cmd = clamp(rudder_cmd, -self.max_rudder, self.max_rudder)

        # ---------- Output shaping ----------
        output.elevator = self._smooth_axis(elevator_cmd, self.prev_elevator, self.surface_alpha, self.max_elevator_rate, dt)
        output.aileron = self._smooth_axis(aileron_cmd, self.prev_aileron, self.surface_alpha, self.max_aileron_rate, dt)
        output.rudder = self._smooth_axis(rudder_cmd, self.prev_rudder, self.surface_alpha, self.max_rudder_rate, dt)
        output.throttle = self._smooth_axis(throttle_cmd, self.prev_throttle, self.throttle_alpha, self.max_throttle_rate, dt)

        self.prev_elevator = output.elevator
        self.prev_aileron = output.aileron
        self.prev_rudder = output.rudder
        self.prev_throttle = output.throttle
        return output

    def _update_integrator(self, integrator, error, dt, limit):
        integrator_dot = error - self.integrator_leak * integrator
        integrator += integrator_dot * dt
        return clamp(integrator, -limit, limit)

    def _smooth_axis(self, target, previous, alpha, max_rate, dt):
        filtered = previous + alpha * (target - previous)
        max_step = max_rate * dt
        delta = clamp(filtered - previous, -max_step, max_step)
        return previous + delta

    def _wrap_angle_deg(self, angle):
        while angle > 180.0:
            angle -= 360.0
        while angle < -180.0:
            angle += 360.0
        return angle

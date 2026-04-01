from XPPython3 import xp

from baseline_controller.models.models import AircraftState, ControlOutput


class XPlaneInterface:
    def __init__(self):
        # Primary state datarefs
        self.theta_ref = self._find_first(['sim/flightmodel/position/theta'])
        self.q_ref = self._find_first(['sim/flightmodel/position/Q'])
        self.phi_ref = self._find_first(['sim/flightmodel/position/phi'])
        self.p_ref = self._find_first(['sim/flightmodel/position/P'])
        self.psi_ref = self._find_first([
            'sim/flightmodel/position/psi',
            'sim/cockpit/gyros/psi_ind_degm',
        ])
        self.r_ref = self._find_first(['sim/flightmodel/position/R'])
        self.altitude_ref = self._find_first([
            'sim/cockpit2/gauges/indicators/altitude_ft_pilot',
            'sim/flightmodel/position/elevation',
        ])
        self.vertical_speed_ref = self._find_first([
            'sim/cockpit2/gauges/indicators/vvi_fpm_pilot',
            'sim/cockpit2/gauges/indicators/vvi_fpm_copilot',
        ], required=False)
        self.airspeed_ref = self._find_first([
            'sim/cockpit2/gauges/indicators/airspeed_kts_pilot',
            'sim/cockpit2/gauges/indicators/airspeed_kts_copilot',
        ])
        self.slip_ref = self._find_first([
            'sim/cockpit2/gauges/indicators/slip_deg',
            'sim/cockpit2/gauges/indicators/sideslip_degrees',
        ], required=False)

        # Writable control datarefs
        self.yoke_pitch_ref = self._find_first([
            'sim/joystick/yoke_pitch_ratio',
            'sim/cockpit2/controls/yoke_pitch_ratio',
        ])
        self.yoke_roll_ref = self._find_first([
            'sim/joystick/yoke_roll_ratio',
            'sim/cockpit2/controls/yoke_roll_ratio',
        ])
        self.yoke_heading_ref = self._find_first([
            'sim/joystick/yoke_heading_ratio',
            'sim/cockpit2/controls/yoke_heading_ratio',
        ], required=False)
        self.throttle_ref = self._find_first([
            'sim/cockpit2/engine/actuators/throttle_ratio_all',
            'sim/cockpit2/engine/actuators/throttle_ratio[0]',
            'sim/cockpit2/engine/actuators/throttle_ratio',
        ], required=False)

        # Overrides so hardware input stops fighting the plugin
        self.override_pitch_ref = self._find_first(['sim/operation/override/override_joystick_pitch'])
        self.override_roll_ref = self._find_first(['sim/operation/override/override_joystick_roll'])
        self.override_heading_ref = self._find_first(['sim/operation/override/override_joystick_heading'], required=False)
        self.override_throttles_ref = self._find_first([
            'sim/operation/override/override_throttles',
            'sim/operation/override/override_throttle',
        ], required=False)

    def _find_first(self, names, required=True):
        for name in names:
            ref = xp.findDataRef(name)
            if ref is not None:
                return ref
        if required:
            raise RuntimeError(f'Missing X-Plane datarefs: {names}')
        return None

    def _getf(self, ref, default=0.0):
        if ref is None:
            return default
        return xp.getDataf(ref)

    def _setf(self, ref, value):
        if ref is not None:
            xp.setDataf(ref, float(value))

    def _seti(self, ref, value):
        if ref is not None:
            xp.setDatai(ref, int(value))

    def read_state(self) -> AircraftState:
        return AircraftState(
            theta=self._getf(self.theta_ref),
            q=self._getf(self.q_ref),
            phi=self._getf(self.phi_ref),
            p=self._getf(self.p_ref),
            psi=self._getf(self.psi_ref),
            r=self._getf(self.r_ref),
            altitude_ft=self._getf(self.altitude_ref),
            vertical_speed_fpm=self._getf(self.vertical_speed_ref),
            airspeed_kts=self._getf(self.airspeed_ref),
            slip_deg=self._getf(self.slip_ref),
            throttle=self._getf(self.throttle_ref),
        )

    def set_control_override(self, enabled: bool):
        value = 1 if enabled else 0
        self._seti(self.override_pitch_ref, value)
        self._seti(self.override_roll_ref, value)
        self._seti(self.override_heading_ref, value)
        self._seti(self.override_throttles_ref, value)

    def write_control(self, output: ControlOutput):
        self._setf(self.yoke_pitch_ref, output.elevator)
        self._setf(self.yoke_roll_ref, output.aileron)
        self._setf(self.yoke_heading_ref, output.rudder)
        self._setf(self.throttle_ref, output.throttle)

    def write_control_zero(self, throttle=0.0):
        self.write_control(ControlOutput(throttle=throttle))

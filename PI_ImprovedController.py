from XPPython3 import xp

from baseline_controller.controllers.advanced_straight_flight_controller import AdvancedStraightFlightController
from baseline_controller.interfaces.xplane_interface import XPlaneInterface
from baseline_controller.models.models import Command


class PythonInterface:
    def XPluginStart(self):
        self.Name = 'Advanced Straight Flight Hold'
        self.Sig = 'braden.advanced.straightflight'
        self.Desc = 'Straight-and-level controller with heading, altitude, and airspeed hold'

        self.controller = None
        self.xplane = None
        self.command = None
        self.flight_loop = None
        self.debug_print = False

        self.active = False
        self.loop_interval_s = 0.05   # 20 Hz

        self.toggle_command_ref = None
        self.enable_command_ref = None
        self.disable_command_ref = None

        try:
            self.controller = AdvancedStraightFlightController()
            self.xplane = XPlaneInterface()
            self.command = Command()

            self.flight_loop = xp.createFlightLoop(self.flight_loop_callback)

            self.toggle_command_ref = xp.createCommand(
                'braden/advanced_straight_flight/toggle',
                'Toggle the advanced straight-flight controller on/off',
            )
            self.enable_command_ref = xp.createCommand(
                'braden/advanced_straight_flight/enable',
                'Enable the advanced straight-flight controller',
            )
            self.disable_command_ref = xp.createCommand(
                'braden/advanced_straight_flight/disable',
                'Disable the advanced straight-flight controller',
            )

            print('[AdvancedStraightFlight] Plugin started successfully.')

        except Exception as exc:
            print(f'[AdvancedStraightFlight] Startup failed: {exc}')

        return self.Name, self.Sig, self.Desc

    def XPluginStop(self):
        try:
            self.disengage_controller()
        except Exception as exc:
            print(f'[AdvancedStraightFlight] Stop cleanup warning: {exc}')

        if self.toggle_command_ref is not None:
            xp.unregisterCommandHandler(self.toggle_command_ref, self.command_handler, 1, None)
        if self.enable_command_ref is not None:
            xp.unregisterCommandHandler(self.enable_command_ref, self.command_handler, 1, None)
        if self.disable_command_ref is not None:
            xp.unregisterCommandHandler(self.disable_command_ref, self.command_handler, 1, None)

        if self.flight_loop is not None:
            xp.destroyFlightLoop(self.flight_loop)
            self.flight_loop = None

        print('[AdvancedStraightFlight] Plugin stopped.')

    def XPluginEnable(self):
        if self.flight_loop is None or self.controller is None or self.xplane is None:
            print('[AdvancedStraightFlight] Cannot enable: plugin not fully initialized.')
            return 0

        xp.registerCommandHandler(self.toggle_command_ref, self.command_handler, 1, None)
        xp.registerCommandHandler(self.enable_command_ref, self.command_handler, 1, None)
        xp.registerCommandHandler(self.disable_command_ref, self.command_handler, 1, None)

        xp.scheduleFlightLoop(self.flight_loop, 0.0, 1)

        print('[AdvancedStraightFlight] Plugin enabled. Bind one of these commands in X-Plane:')
        print('  braden/advanced_straight_flight/toggle')
        print('  braden/advanced_straight_flight/enable')
        print('  braden/advanced_straight_flight/disable')
        return 1

    def XPluginDisable(self):
        try:
            self.disengage_controller()
        except Exception as exc:
            print(f'[AdvancedStraightFlight] Disable cleanup warning: {exc}')

        if self.toggle_command_ref is not None:
            xp.unregisterCommandHandler(self.toggle_command_ref, self.command_handler, 1, None)
        if self.enable_command_ref is not None:
            xp.unregisterCommandHandler(self.enable_command_ref, self.command_handler, 1, None)
        if self.disable_command_ref is not None:
            xp.unregisterCommandHandler(self.disable_command_ref, self.command_handler, 1, None)

        print('[AdvancedStraightFlight] Plugin disabled.')

    def XPluginReceiveMessage(self, inFromWho, inMessage, inParam):
        pass

    def command_handler(self, command_ref, phase, refcon):
        if phase != xp.CommandBegin:
            return 1

        if command_ref == self.toggle_command_ref:
            if self.active:
                self.disengage_controller()
            else:
                self.engage_controller()

        elif command_ref == self.enable_command_ref:
            if not self.active:
                self.engage_controller()

        elif command_ref == self.disable_command_ref:
            if self.active:
                self.disengage_controller()

        return 0

    def engage_controller(self):
        state = self.xplane.read_state()

        self.command = Command(
            theta_cmd=state.theta,
            phi_cmd=state.phi,
            heading_deg_cmd=state.psi,
            altitude_ft_cmd=state.altitude_ft,
            airspeed_kts_cmd=state.airspeed_kts,
        )

        self.controller.trim_throttle = max(0.0, min(1.0, state.throttle))
        self.controller.reset()
        self.controller.enable()
        self.controller.initialize(state=state, command=self.command)

        self.xplane.set_control_override(True)
        self.xplane.write_control_zero(throttle=self.controller.trim_throttle)

        self.active = True
        xp.scheduleFlightLoop(self.flight_loop, -1.0, 1)

        print(
            '[AdvancedStraightFlight] ACTIVE '
            f'(hdg={self.command.heading_deg_cmd:.1f}, alt={self.command.altitude_ft_cmd:.1f} ft, '
            f'ias={self.command.airspeed_kts_cmd:.1f} kt)'
        )

    def disengage_controller(self):
        self.active = False

        if self.flight_loop is not None:
            xp.scheduleFlightLoop(self.flight_loop, 0.0, 1)

        if self.controller is not None:
            self.controller.reset()
            self.controller.disable()

        if self.xplane is not None:
            self.xplane.write_control_zero()
            self.xplane.set_control_override(False)

        print('[AdvancedStraightFlight] INACTIVE')

    def flight_loop_callback(self, since_last_call, elapsed_time, counter, refcon):
        if not self.active:
            return 0.0

        try:
            dt = since_last_call if since_last_call > 0.0 else self.loop_interval_s
            state = self.xplane.read_state()
            output = self.controller.update(state, self.command, dt)
            self.xplane.write_control(output)

            if self.debug_print:
                print(f'[AdvancedStraightFlight] state={state} output={output}')

        except Exception as exc:
            print(f'[AdvancedStraightFlight] Flight loop error: {exc}')
            self.disengage_controller()
            return 0.0

        return self.loop_interval_s

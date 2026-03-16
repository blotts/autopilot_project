from XPPython3 import xp

from baseline_controller.controllers.baseline_attitude_hold import BaselineAttitudeHoldController
from baseline_controller.interfaces.xplane_interface import XPlaneInterface
from baseline_controller.models.command import Command


class PythonInterface:
    def XPluginStart(self):
        self.Name = 'Baseline Attitude Hold'
        self.Sig = 'braden.baseline.attitudehold'
        self.Desc = 'Baseline pitch/roll attitude-hold controller'

        self.controller = None
        self.xplane = None
        self.command = None
        self.flight_loop = None
        self.debug_print = False

        # Plugin logic state.
        self.active = False
        self.loop_interval_s = 0.05   # 20 Hz, intentionally slower/smoother

        # X-Plane command refs for keybinding.
        self.toggle_command_ref = None
        self.enable_command_ref = None
        self.disable_command_ref = None

        try:
            self.controller = BaselineAttitudeHoldController()
            self.xplane = XPlaneInterface()
            self.command = Command()

            self.flight_loop = xp.createFlightLoop(self.flight_loop_callback)

            self.toggle_command_ref = xp.createCommand(
                'braden/baseline_attitude_hold/toggle',
                'Toggle the baseline attitude-hold controller on/off',
            )
            self.enable_command_ref = xp.createCommand(
                'braden/baseline_attitude_hold/enable',
                'Enable the baseline attitude-hold controller',
            )
            self.disable_command_ref = xp.createCommand(
                'braden/baseline_attitude_hold/disable',
                'Disable the baseline attitude-hold controller',
            )

            print('[BaselineController] Plugin started successfully.')

        except Exception as exc:
            print(f'[BaselineController] Startup failed: {exc}')

        return self.Name, self.Sig, self.Desc

    def XPluginStop(self):
        try:
            self.disengage_controller()
        except Exception as exc:
            print(f'[BaselineController] Stop cleanup warning: {exc}')

        if self.toggle_command_ref is not None:
            xp.unregisterCommandHandler(self.toggle_command_ref, self.command_handler, 1, None)
        if self.enable_command_ref is not None:
            xp.unregisterCommandHandler(self.enable_command_ref, self.command_handler, 1, None)
        if self.disable_command_ref is not None:
            xp.unregisterCommandHandler(self.disable_command_ref, self.command_handler, 1, None)

        if self.flight_loop is not None:
            xp.destroyFlightLoop(self.flight_loop)
            self.flight_loop = None

        print('[BaselineController] Plugin stopped.')

    def XPluginEnable(self):
        if self.flight_loop is None or self.controller is None or self.xplane is None:
            print('[BaselineController] Cannot enable: plugin not fully initialized.')
            return 0

        xp.registerCommandHandler(self.toggle_command_ref, self.command_handler, 1, None)
        xp.registerCommandHandler(self.enable_command_ref, self.command_handler, 1, None)
        xp.registerCommandHandler(self.disable_command_ref, self.command_handler, 1, None)

        # Do not schedule the controller loop yet. It will run only when active.
        xp.scheduleFlightLoop(self.flight_loop, 0.0, 1)

        print('[BaselineController] Plugin enabled. Bind one of these commands in X-Plane:')
        print('  braden/baseline_attitude_hold/toggle')
        print('  braden/baseline_attitude_hold/enable')
        print('  braden/baseline_attitude_hold/disable')
        return 1

    def XPluginDisable(self):
        try:
            self.disengage_controller()
        except Exception as exc:
            print(f'[BaselineController] Disable cleanup warning: {exc}')

        if self.toggle_command_ref is not None:
            xp.unregisterCommandHandler(self.toggle_command_ref, self.command_handler, 1, None)
        if self.enable_command_ref is not None:
            xp.unregisterCommandHandler(self.enable_command_ref, self.command_handler, 1, None)
        if self.disable_command_ref is not None:
            xp.unregisterCommandHandler(self.disable_command_ref, self.command_handler, 1, None)

        print('[BaselineController] Plugin disabled.')

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

        # Capture the current aircraft attitude so there is no step jump when engaged.
        self.command.theta_cmd = state.theta
        self.command.phi_cmd = state.phi

        self.controller.reset()
        self.controller.enable()
        self.controller.initialize(state=state, command=self.command)

        # Override pilot pitch/roll hardware so the joystick no longer fights the plugin.
        self.xplane.set_control_override(True)
        self.xplane.write_control_zero()

        self.active = True
        xp.scheduleFlightLoop(self.flight_loop, -1.0, 1)

        print(
            '[BaselineController] ACTIVE '
            f'(theta_cmd={self.command.theta_cmd:.2f}, phi_cmd={self.command.phi_cmd:.2f})'
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

        print('[BaselineController] INACTIVE')

    def flight_loop_callback(self, since_last_call, elapsed_time, counter, refcon):
        if not self.active:
            return 0.0

        try:
            dt = since_last_call if since_last_call > 0.0 else self.loop_interval_s

            state = self.xplane.read_state()
            output = self.controller.update(state, self.command, dt)
            self.xplane.write_control(output)

            if self.debug_print:
                print(f'[BaselineController] state={state} output={output}')

        except Exception as exc:
            print(f'[BaselineController] Flight loop error: {exc}')
            self.disengage_controller()
            return 0.0

        return self.loop_interval_s
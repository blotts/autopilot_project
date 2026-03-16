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

        try:
            self.controller = BaselineAttitudeHoldController()
            self.xplane = XPlaneInterface()

            self.command = Command()

            self.flight_loop = xp.createFlightLoop(self.flight_loop_callback)

            print('[BaselineController] Plugin started successfully.')

        except Exception as exc:
            print(f'[BaselineController] Startup failed: {exc}')

        return self.Name, self.Sig, self.Desc

    def XPluginStop(self):
        try:
            if self.xplane is not None:
                self.xplane.write_control_zero()
        except Exception as exc:
            print(f'[BaselineController] Stop cleanup warning: {exc}')

        if self.flight_loop is not None:
            xp.destroyFlightLoop(self.flight_loop)
            self.flight_loop = None

        print('[BaselineController] Plugin stopped.')

    def XPluginEnable(self):
        if self.flight_loop is None or self.controller is None or self.xplane is None:
            print('[BaselineController] Cannot enable: plugin not fully initialized.')
            return 0

        xp.scheduleFlightLoop(self.flight_loop, -1.0, 1)
        print('[BaselineController] Plugin enabled.')
        return 1

    def XPluginDisable(self):
        try:
            if self.xplane is not None:
                self.xplane.write_control_zero()
        except Exception as exc:
            print(f'[BaselineController] Disable cleanup warning: {exc}')

        print('[BaselineController] Plugin disabled.')

    def XPluginReceiveMessage(self, inFromWho, inMessage, inParam):
        pass

    def flight_loop_callback(self, since_last_call, elapsed_time, counter, refcon):
        try:
            dt = since_last_call if since_last_call > 0.0 else 0.02

            state = self.xplane.read_state()
            output = self.controller.update(state, self.command, dt)
            self.xplane.write_control(output)

            if self.debug_print:
                print(f'[BaselineController] state={state} output={output}')

        except Exception as exc:
            print(f'[BaselineController] Flight loop error: {exc}')

        return -1.0
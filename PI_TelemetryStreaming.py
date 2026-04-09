from XPPython3 import xp

from baseline_controller.interfaces.xplane_interface import XPlaneInterface
from baseline_controller.models.models import Command
from network_utils.telemetry import TelemetryManager
from dataclasses import asdict
import json

DEST_ADDR = "localhost"
DEST_PORT = 8000

class PythonInterface:
    def XPluginStart(self):
        self.Name = 'Telemetry Streaming Manager'
        self.Sig = 'braden.telemetry.streaming'
        self.Desc = 'Simple UDP streaming code that sends aircraft data periodically to a server'

        self.xplane = None
        self.flight_loop = None
        self.debug_print = False

        self.active = False
        self.loop_interval_s = 0.05   # 20 Hz

        self.toggle_command_ref = None
        self.enable_command_ref = None
        self.disable_command_ref = None
        self._handlers_registered = False

        self.telman = None

        try:
            self.xplane = XPlaneInterface()

            self.flight_loop = xp.createFlightLoop(self.flight_loop_callback)

            self.toggle_command_ref = xp.createCommand(
                'braden/telemetry_streaming_manager/toggle',
                'Toggle streaming telemetry',
            )
            self.enable_command_ref = xp.createCommand(
                'braden/telemetry_streaming_manager/enable',
                'Enable streaming telemetry',
            )
            self.disable_command_ref = xp.createCommand(
                'braden/telemetry_streaming_manager/disable',
                'Disable streaming telemetry',
            )

            print('[TelemetryStreamingManager] Plugin started successfully.')

        except Exception as exc:
            print(f'[TelemetryStreamingManager] Startup failed: {exc}')

        return self.Name, self.Sig, self.Desc

    def XPluginStop(self):
        try:
            self.disengage_telem_dump()
        except Exception as exc:
            print(f'[TelemetryStreamingManager] Stop cleanup warning: {exc}')

        if self._handlers_registered:
            if self.toggle_command_ref is not None:
                xp.unregisterCommandHandler(self.toggle_command_ref, self.command_handler, 1, None)
            if self.enable_command_ref is not None:
                xp.unregisterCommandHandler(self.enable_command_ref, self.command_handler, 1, None)
            if self.disable_command_ref is not None:
                xp.unregisterCommandHandler(self.disable_command_ref, self.command_handler, 1, None)
            self._handlers_registered = False

        if self.flight_loop is not None:
            xp.destroyFlightLoop(self.flight_loop)
            self.flight_loop = None

        print('[TelemetryStreamingManager] Plugin stopped.')

    def XPluginEnable(self):
        if self.flight_loop is None or self.xplane is None:
            print('[TelemetryStreamingManager] Cannot enable: plugin not fully initialized.')
            return 0

        try:
            self.telman = TelemetryManager(DEST_ADDR, DEST_PORT)
        except Exception as e:
            print(f'[TelemetryStreamingManager] Connection failed: {e}')
            return 0

        xp.registerCommandHandler(self.toggle_command_ref, self.command_handler, 1, None)
        xp.registerCommandHandler(self.enable_command_ref, self.command_handler, 1, None)
        xp.registerCommandHandler(self.disable_command_ref, self.command_handler, 1, None)
        self._handlers_registered = True

        xp.scheduleFlightLoop(self.flight_loop, 0.0, 1)

        print('[TelemetryStreamingManager] Plugin enabled. Bind one of these commands in X-Plane:')
        print('  braden/telemetry_streaming_manager/toggle')
        print('  braden/telemetry_streaming_manager/enable')
        print('  braden/telemetry_streaming_manager/disable')
        return 1

    def XPluginDisable(self):
        try:
            self.telman = None # call destructor to clean up
            self.disengage_telem_dump()
        except Exception as exc:
            print(f'[TelemetryStreamingManager] Disable cleanup warning: {exc}')

        if self._handlers_registered:
            if self.toggle_command_ref is not None:
                xp.unregisterCommandHandler(self.toggle_command_ref, self.command_handler, 1, None)
            if self.enable_command_ref is not None:
                xp.unregisterCommandHandler(self.enable_command_ref, self.command_handler, 1, None)
            if self.disable_command_ref is not None:
                xp.unregisterCommandHandler(self.disable_command_ref, self.command_handler, 1, None)
            self._handlers_registered = False

        print('[TelemetryStreamingManager] Plugin disabled.')

    def XPluginReceiveMessage(self, inFromWho, inMessage, inParam):
        pass

    def command_handler(self, command_ref, phase, refcon):
        if phase != xp.CommandBegin:
            return 1

        if command_ref == self.toggle_command_ref:
            if self.active:
                self.disengage_telem_dump()
            else:
                self.engage_telem_dump()

        elif command_ref == self.enable_command_ref:
            if not self.active:
                self.engage_telem_dump()

        elif command_ref == self.disable_command_ref:
            if self.active:
                self.disengage_telem_dump()

        return 0

    def engage_telem_dump(self):
        self.active = True
        xp.scheduleFlightLoop(self.flight_loop, -1.0, 1)

        print("[TelemetryStreamingManager] Active")

    def disengage_telem_dump(self):
        self.active = False

        if self.flight_loop is not None:
            xp.scheduleFlightLoop(self.flight_loop, 0.0, 1)
        
        print("[TelemetryStreamingManager] Inactive")

    def flight_loop_callback(self, since_last_call, elapsed_time, counter, refcon):
        if not self.active:
            return 0.0

        try:
            dt = since_last_call if since_last_call > 0.0 else self.loop_interval_s
            state = self.xplane.read_state()
            self.telman.send_data(json.dumps(asdict(state)))

        except Exception as exc:
            print(f'[AdvancedStraightFlight] Flight loop error: {exc}')
            self.disengage_telem_dump()
            return 0.0

        return self.loop_interval_s

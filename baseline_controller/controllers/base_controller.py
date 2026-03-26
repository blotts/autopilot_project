from baseline_controller.models.models import ControlOutput


class BaseController:
    def __init__(self, name='controller'):
        self.name = name
        self.enabled = True
        self.initialized = False

    def reset(self):
        self.initialized = False

    def initialize(self, state=None, command=None):
        self.initialized = True

    def update(self, state, command, dt):
        if not self.enabled:
            return self.zero_output()

        if not self.initialized:
            self.initialize(state=state, command=command)

        return self.compute_output(state, command, dt)

    def compute_output(self, state, command, dt):
        raise NotImplementedError('Subclasses must implement compute_output().')

    def zero_output(self):
        return ControlOutput()

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False
"""Test doubles for the uArm SDK and the Isaac Sim dynamic-control interface."""


class FakeSwift:
    """Records calls like ``uarm.wrapper.SwiftAPI`` would receive them."""

    def __init__(self, servo=(90.0, 45.0, 60.0), position=(180.0, 0.0, 100.0), failing_servo=None):
        self.servo = list(servo)
        self.position = list(position)
        self.failing_servo = failing_servo
        self.calls = []
        self.callbacks = []

    def get_servo_angle(self, servo_id=None, wait=True, timeout=None, callback=None):
        if callback is not None:
            self.callbacks.append(callback)
            return None
        if servo_id is None:
            return list(self.servo)
        if servo_id == self.failing_servo:
            raise RuntimeError("servo does not answer")
        return self.servo[servo_id]

    def get_position(self):
        return list(self.position)

    def set_position(self, **kwargs):
        self.calls.append(("set_position", kwargs))

    def set_gripper(self, catch=False):
        self.calls.append(("set_gripper", catch))

    def set_wrist(self, angle=90, wait=True, timeout=None, callback=None):
        self.calls.append(("set_wrist", angle))
        return "OK"

    def set_servo_detach(self, servo_id=None, wait=True):
        self.calls.append(("detach", servo_id))
        return "OK"

    def set_servo_attach(self, servo_id=None, wait=True):
        self.calls.append(("attach", servo_id))
        return "OK"


class FakeDynamicControl:
    """Articulations by prim path, each a list of DOF names; records position targets."""

    def __init__(self, articulations):
        self.articulations = articulations
        self.targets = []
        self.wakes = 0

    def get_articulation(self, path):
        return path if path in self.articulations else 0

    def wake_up_articulation(self, art):
        self.wakes += 1

    def get_articulation_dof_count(self, art):
        return len(self.articulations[art])

    def get_articulation_dof(self, art, index):
        return (art, self.articulations[art][index])

    def get_dof_name(self, dof):
        return dof[1]

    def set_dof_position_target(self, dof, value):
        self.targets.append((dof[0], dof[1], value))

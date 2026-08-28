#!/usr/bin/env python3

from sensor_msgs.msg import JointState


class GripperClient:
    HOLD_WIDTH_MIN = 0.005
    SETTLE_TICKS = 3
    TARGET_TOPIC = 'control/gripper_target'

    def __init__(self, node, joint_name, open_width=0.1, closed_width=0.0,
                 force_threshold=0.5, width_tolerance=0.002, timeout=3.0):
        self.node = node
        self.joint_name = joint_name
        self.open_width = open_width
        self.closed_width = closed_width
        self.force_threshold = force_threshold
        self.width_tolerance = width_tolerance
        self.timeout = timeout

        self.current_width = None
        self.current_force = 0.0
        self.target = None
        self.done = False
        self._closing = False
        self._ok_ticks = 0
        self._force_ticks = 0
        self._elapsed = 0.0

        self.pub = node.create_publisher(JointState, self.TARGET_TOPIC, 1)

    def feedback(self, width, force):
        self.current_width = width
        self.current_force = force

    def open(self):
        self._closing = False
        self._send(self.open_width)

    def close(self):
        self._closing = True
        self._send(self.closed_width)

    def _send(self, width):
        self.target = width
        self.done = False
        self._ok_ticks = 0
        self._force_ticks = 0
        self._elapsed = 0.0
        msg = JointState()
        msg.name = [self.joint_name]
        msg.position = [float(width)]
        self.pub.publish(msg)

    def update(self, dt):
        if self.done or self.target is None:
            return
        self._elapsed += dt
        if self.current_width is not None:
            if self._closing:
                if abs(self.current_force) > self.force_threshold:
                    self._force_ticks += 1
                else:
                    self._force_ticks = 0
                if self._force_ticks >= self.SETTLE_TICKS:
                    self.done = True
                    return
            if abs(self.current_width - self.target) <= self.width_tolerance:
                self._ok_ticks += 1
            else:
                self._ok_ticks = 0
            if self._ok_ticks >= self.SETTLE_TICKS:
                self.done = True
                return
        if self._elapsed >= self.timeout:
            self.done = True

    def holding(self):
        return (self.current_width is not None
                and self.current_width > self.HOLD_WIDTH_MIN
                and self._force_ticks >= self.SETTLE_TICKS)

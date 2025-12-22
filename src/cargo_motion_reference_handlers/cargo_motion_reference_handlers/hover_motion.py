#!/usr/bin/env python3

"""Implementation of a motion reference handler for speed motion."""

from cargo_motion_reference_handlers.basic_motion_references import BasicMotionReferenceHandler
from cargo_msgs.msg import ControlMode
from rclpy.node import Node


class HoverMotion(BasicMotionReferenceHandler):
    """Send hover motion command."""

    def __init__(self, node: Node):
        """Initialize hover motion handler."""
        super().__init__(node)
        self.desired_control_mode_.yaw_mode = ControlMode.NONE
        self.desired_control_mode_.control_mode = ControlMode.HOVER
        self.desired_control_mode_.reference_frame = ControlMode.UNDEFINED_FRAME

    def send_hover(self):
        """Send hover command."""
        self.desired_control_mode_.yaw_mode = ControlMode.NONE
        self.desired_control_mode_.control_mode = ControlMode.HOVER
        self.desired_control_mode_.reference_frame = ControlMode.UNDEFINED_FRAME
        return self.check_mode()

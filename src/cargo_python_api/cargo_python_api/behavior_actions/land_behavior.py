"""Land Behavior."""

import typing

from cargo_msgs.action import Land

from ..behavior_actions.behavior_handler import BehaviorHandler

if typing.TYPE_CHECKING:
    from ..drone_interface_base import DroneInterfaceBase


class LandBehavior(BehaviorHandler):
    """Land Behavior."""

    def __init__(self, drone: 'DroneInterfaceBase') -> None:
        self.__drone = drone

        try:
            super().__init__(drone, Land, 'LandBehavior')
        except self.BehaviorNotAvailable as err:
            self.__drone.get_logger().warn(str(err))

    def start(self, speed: float, wait_result: bool = True) -> bool:
        """Start landing."""
        goal_msg = Land.Goal()
        goal_msg.land_speed = speed

        try:
            return super().start(goal_msg, wait_result)
        except self.GoalRejected as err:
            self.__drone.get_logger().warn(str(err))
        return False

    def modify(self, speed: float):
        """Modify landing."""
        goal_msg = Land.Goal()
        goal_msg.land_speed = speed

        return super().modify(goal_msg)

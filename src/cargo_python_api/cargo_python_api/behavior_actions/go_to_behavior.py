"""Go To Behavior."""

import typing

from cargo_msgs.action import GoToWaypoint
from cargo_python_api.behavior_actions.behavior_handler import BehaviorHandler
from geographic_msgs.msg import GeoPose, GeoPoseStamped
from geometry_msgs.msg import Pose, PoseStamped
from pymap3d import geodetic2enu

if typing.TYPE_CHECKING:
    from ..drone_interface_base import DroneInterfaceBase


class GoToBehavior(BehaviorHandler):
    """GoTo Behavior."""

    def __init__(self, drone: 'DroneInterfaceBase') -> None:
        self.__drone = drone

        try:
            super().__init__(drone, GoToWaypoint, 'GoToBehavior')
        except self.BehaviorNotAvailable as err:
            self.__drone.get_logger().warn(str(err))

    def start(self, pose: tuple[Pose, PoseStamped, GeoPose, GeoPoseStamped],
              speed: float, yaw_mode: int, yaw_angle: float, frame_id: str = 'earth',
              wait_result: bool = True) -> bool:
        """Start behavior."""
        goal_msg = GoToWaypoint.Goal()
        pose_stamped = self.__get_pose(pose)
        goal_msg.target_pose.header.stamp = self.__drone.get_clock().now().to_msg()
        goal_msg.target_pose.header.frame_id = frame_id
        goal_msg.target_pose.point.x = pose_stamped.position.x
        goal_msg.target_pose.point.y = pose_stamped.position.y
        goal_msg.target_pose.point.z = pose_stamped.position.z

        goal_msg.max_speed = speed
        goal_msg.yaw.mode = yaw_mode
        if yaw_angle:
            goal_msg.yaw.angle = yaw_angle

        try:
            return super().start(goal_msg, wait_result)
        except self.GoalRejected as err:
            self.__drone.get_logger().warn(str(err))
        return False

    def modify(self, pose: tuple[Pose, PoseStamped, GeoPose, GeoPoseStamped],
               speed: float, yaw_mode: int, yaw_angle: float, frame_id: str = 'earth'):
        """Modify behavior."""
        goal_msg = GoToWaypoint.Goal()
        pose_stamped = self.__get_pose(pose)
        goal_msg.target_pose.header.stamp = self.__drone.get_clock().now().to_msg()
        goal_msg.target_pose.header.frame_id = frame_id
        goal_msg.target_pose.point.x = pose_stamped.position.x
        goal_msg.target_pose.point.y = pose_stamped.position.y
        goal_msg.target_pose.point.z = pose_stamped.position.z

        goal_msg.max_speed = speed
        goal_msg.yaw.mode = yaw_mode
        if yaw_angle:
            goal_msg.yaw.angle = yaw_angle
        return super().modify(goal_msg)

    def __get_pose(self, pose: tuple[Pose, PoseStamped, GeoPose, GeoPoseStamped]):
        """Get pose msg."""
        if isinstance(pose, Pose):
            return pose
        if isinstance(pose, PoseStamped):
            return pose.pose
        if isinstance(pose, GeoPose):
            geopose = GeoPoseStamped()
            # TODO: frame id and stamp
            geopose.pose = pose
            return self.__get_pose(geopose)
        if isinstance(pose, GeoPoseStamped):
            lat0, lon0, h0 = self.__drone.gps.origin
            x, y, z = geodetic2enu(pose.pose.position.latitude,
                                   pose.pose.position.longitude,
                                   pose.pose.position.altitude,
                                   lat0, lon0, h0)
            mypose = Pose()
            mypose.position.x = float(x)
            mypose.position.y = float(y)
            # CAUTION: using height from origin
            mypose.position.z = float(pose.pose.position.altitude)
            return mypose

        raise self.GoalRejected('Goal format invalid')

"""Go to Module."""

from __future__ import annotations

from typing import TYPE_CHECKING

from cargo_msgs.msg import YawMode
from cargo_python_api.behavior_actions.go_to_behavior import GoToBehavior
from cargo_python_api.modules.module_base import ModuleBase
from geometry_msgs.msg import Pose

if TYPE_CHECKING:
    from ..drone_interface import DroneInterface


class GoToModule(ModuleBase, GoToBehavior):
    """Go to Module."""

    __alias__ = 'go_to'

    def __init__(self, drone: 'DroneInterface') -> None:
        super().__init__(drone, self.__alias__)

    def _create_goal(self, x: int, y: int, z: int) -> Pose:
        """
        Create a goal pose.

        :param x: x coordinate (m)
        :type x: int
        :param y: y coordinate (m)
        :type y: int
        :param z: z coordinate (m)
        :type z: int
        :return: Pose with the goal position
        :rtype: Pose
        """
        msg = Pose()
        msg.position.x = (float)(x)
        msg.position.y = (float)(y)
        msg.position.z = (float)(z)
        return msg

    def __call__(
        self,
        x: float,
        y: float,
        z: float,
        speed: float,
        yaw_mode: int = YawMode.KEEP_YAW,
        yaw_angle: float = None,
        frame_id: str = 'earth',
        wait: bool = True,
    ) -> bool:
        """
        Go to point.

        :param x: x coordinate (m) to go to
        :type x: float
        :param y: y coordinate (m) to go to
        :type y: float
        :param z: z coordinate (m) to go to
        :type z: float
        :param speed: speed (m/s) to go to the point
        :type speed: float
        :param yaw_mode: yaw mode, defaults to YawMode.KEEP_YAW
        :type yaw_mode: int, optional
        :param yaw_angle: yaw angle (rad) when fixed yaw is set, defaults to None
        :type yaw_angle: float, optional
        :param frame_id: reference frame of the coordinates, defaults to "earth"
        :type frame_id: str, optional
        :param wait: blocking call, defaults to True
        :type wait: bool, optional
        :return: True if was accepted, False otherwise
        :rtype: bool
        """
        return self.__go_to(x, y, z, speed, yaw_mode, yaw_angle, frame_id, wait)

    def __go_to(
        self,
        x: float,
        y: float,
        z: float,
        speed: float,
        yaw_mode: int,
        yaw_angle: float,
        frame_id: str = 'earth',
        wait: bool = True,
    ) -> bool:
        """
        Go to point.

        :param x: x coordinate (m) to go to
        :type x: float
        :param y: y coordinate (m) to go to
        :type y: float
        :param z: z coordinate (m) to go to
        :type z: float
        :param speed: speed (m/s) to go to the point
        :type speed: float
        :param yaw_mode: yaw mode
        :type yaw_mode: int
        :param yaw_angle: yaw angle (rad) when fixed yaw is set
        :type yaw_angle: float
        :param frame_id: reference frame of the coordinates, defaults to "earth"
        :type frame_id: str, optional
        :param wait: blocking call, defaults to True
        :type wait: bool, optional
        :return: True if was accepted, False otherwise
        :rtype: bool
        """
        msg = self._create_goal(x, y, z)
        return self.start(msg, speed, yaw_mode, yaw_angle, frame_id, wait)

    def modify(
        self,
        x: float,
        y: float,
        z: float,
        speed: float,
        yaw_mode: int = YawMode.KEEP_YAW,
        yaw_angle: float = None,
        frame_id: str = 'earth',
    ) -> bool:
        """
        Modify the go to point.

        :param x: x coordinate (m) to go to
        :type x: float
        :param y: y coordinate (m) to go to
        :type y: float
        :param z: z coordinate (m) to go to
        :type z: float
        :param speed: speed (m/s) to go to the point
        :type speed: float
        :param yaw_mode: yaw mode, defaults to YawMode.KEEP_YAW
        :type yaw_mode: int, optional
        :return: True if was accepted, False otherwise
        :rtype: bool
        """
        msg: Pose = self._create_goal(x, y, z)
        return super().modify(msg, speed, yaw_mode, yaw_angle, frame_id)

    # Method simplifications
    def go_to(self, x: float, y: float, z: float, speed: float, frame_id: str = 'earth') -> bool:
        """
        Go to point, blocking call.

        :param x: x coordinate (m) to go to
        :type x: float
        :param y: y coordinate (m) to go to
        :type y: float
        :param z: z coordinate (m) to go to
        :type z: float
        :param speed: speed (m/s) to go to the point
        :type speed: float
        :param frame_id: reference frame of the coordinates, defaults to "earth"
        :type frame_id: str, optional
        :return: True if was accepted, False otherwise
        :rtype: bool
        """
        return self.__go_to(
            x, y, z, speed, yaw_mode=YawMode.KEEP_YAW, yaw_angle=None, frame_id=frame_id
        )

    def go_to_with_yaw(
        self, x: float, y: float, z: float, speed: float, angle: float, frame_id: str = 'earth'
    ) -> bool:
        """
        Go to point. With desired yaw angle (degrees). Blocking call.

        :param x: x coordinate (m) to go to
        :type x: float
        :param y: y coordinate (m) to go to
        :type y: float
        :param z: z coordinate (m) to go to
        :type z: float
        :param speed: speed (m/s) to go to the point
        :type speed: float
        :param yaw_angle: yaw angle
        :type yaw_angle: float
        :param frame_id: reference frame of the coordinates, defaults to "earth"
        :type frame_id: str, optional
        :return: True if was accepted, False otherwise
        :rtype: bool
        """
        return self.__go_to(
            x, y, z, speed, yaw_mode=YawMode.FIXED_YAW, yaw_angle=angle, frame_id=frame_id
        )

    def go_to_path_facing(
        self, x: float, y: float, z: float, speed: float, frame_id: str = 'earth'
    ) -> bool:
        """
        Go to point. With path facing yaw mode. Blocking call.

        :param x: x coordinate (m) to go to
        :type x: float
        :param y: y coordinate (m) to go to
        :type y: float
        :param z: z coordinate (m) to go to
        :type z: float
        :param speed: speed (m/s) to go to the point
        :type speed: float
        :param frame_id: reference frame of the coordinates, defaults to "earth"
        :type frame_id: str, optional
        :return: True if was accepted, False otherwise
        :rtype: bool
        """
        return self.__go_to(
            x, y, z, speed, yaw_mode=YawMode.PATH_FACING, yaw_angle=None, frame_id=frame_id
        )

    def go_to_point(self, point: list[float], speed: float, frame_id: str = 'earth') -> bool:
        """
        Go to point. With keep yaw mode. Blocking call.

        :param point: [x, y, z] (m) coordinates to go to
        :type point: list[float]
        :param speed: speed (m/s) to go to the point
        :type speed: float
        :param frame_id: reference frame of the coordinates, defaults to "earth"
        :type frame_id: str, optional
        :return: True if was accepted, False otherwise
        :rtype: bool
        """
        return self.__go_to(
            point[0],
            point[1],
            point[2],
            speed,
            yaw_mode=YawMode.KEEP_YAW,
            yaw_angle=None,
            frame_id=frame_id,
        )

    def go_to_point_with_yaw(
        self, point: list[float], speed: float, angle: float, frame_id: str = 'earth'
    ) -> bool:
        """
        Go to point. With desired yaw angle (degrees). Blocking call.

        :param point: [x, y, z] (m) coordinates to go to
        :type point: list[float]
        :param speed: speed (m/s) to go to the point
        :type speed: float
        :param yaw_angle: yaw angle
        :type yaw_angle: float
        :param frame_id: reference frame of the coordinates, defaults to "earth"
        :type frame_id: str, optional
        :return: True if was accepted, False otherwise
        :rtype: bool
        """
        return self.__go_to(
            point[0],
            point[1],
            point[2],
            speed,
            yaw_mode=YawMode.FIXED_YAW,
            yaw_angle=angle,
            frame_id=frame_id,
        )

    def go_to_point_path_facing(
        self, point: list[float], speed: float, frame_id: str = 'earth'
    ) -> bool:
        """
        Go to point. With path facing yaw mode. Blocking call.

        :param point: [x, y, z] (m) coordinates to go to
        :type point: list[float]
        :param speed: speed (m/s) to go to the point
        :type speed: float
        :param frame_id: reference frame of the coordinates, defaults to "earth"
        :type frame_id: str, optional
        :return: True if was accepted, False otherwise
        :rtype: bool
        """
        return self.__go_to(
            point[0],
            point[1],
            point[2],
            speed,
            yaw_mode=YawMode.PATH_FACING,
            frame_id=frame_id,
            yaw_angle=None,
        )




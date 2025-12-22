from enum import Enum


class ExtendedEnum(Enum):

    @classmethod
    def to_list(cls):
        return [c.value for c in cls]


class KeyMappings(ExtendedEnum):
    TAKE_OFF_KEY = 't'
    LAND_KEY = 'l'
    HOVER_KEY = 'space'
    EMERGENCY_KEY = 'Delete'
    UP_KEY = 'w'
    DOWN_KEY = 's'
    ROTATE_RIGHT_KEY = 'd'
    ROTATE_LEFT_KEY = 'a'
    LEFT_KEY = 'Left'
    RIGHT_KEY = 'Right'
    FORWARD_KEY = 'Up'
    BACKWARD_KEY = 'Down'


class ControlValues():
    # Default values
    SPEED_VALUE = 0.5
    VERTICAL_VALUE = 0.5
    TURN_SPEED_VALUE = 0.30
    POSITION_VALUE = 1.00
    ALTITUDE_VALUE = 1.00
    TURN_ANGLE_VALUE = 1.57

    @classmethod
    def initialize(cls, speed_value=None, altitude_speed_value=None, turn_speed_value=None,
                   position_value=None, altitude_value=None, turn_angle_value=None):
        if speed_value is not None:
            cls.SPEED_VALUE = speed_value
        if altitude_speed_value is not None:
            cls.VERTICAL_VALUE = altitude_speed_value
        if turn_speed_value is not None:
            cls.TURN_SPEED_VALUE = turn_speed_value
        if position_value is not None:
            cls.POSITION_VALUE = position_value
        if altitude_value is not None:
            cls.ALTITUDE_VALUE = altitude_value
        if turn_angle_value is not None:
            cls.TURN_ANGLE_VALUE = turn_angle_value


class ControlModes(ExtendedEnum):
    SPEED_CONTROL = '-SPEED-'
    POSE_CONTROL = '-POSE-'
    BODY_POSE_CONTROL = '-BODY-POSE-'


class Options(ExtendedEnum):
    ARM_ON_TAKE_OFF = True

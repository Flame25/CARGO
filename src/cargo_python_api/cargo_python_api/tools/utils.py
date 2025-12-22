"""Different utility methods."""
import importlib
from importlib.machinery import ModuleSpec
import inspect
from math import asin, atan2
import os
import sys
from typing import TYPE_CHECKING

from nav_msgs.msg import Path

if TYPE_CHECKING:
    from cargo_python_api.modules.module_base import ModuleBase


def get_sendgoal_action_msg(action_msg):
    """Find and return the action_SendGoal message for an action."""
    module = inspect.getmodule(action_msg)
    name = action_msg.__name__
    action_sendgoal_name = f'{name}_SendGoal'
    return getattr(module, action_sendgoal_name)


def euler_from_quaternion(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    """
    Convert a quaternion into euler angles [roll, pitch, yaw].

    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t_0 = +2.0 * (w * x + y * z)
    t_1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = atan2(t_0, t_1)

    t_2 = +2.0 * (w * y - z * x)
    t_2 = +1.0 if t_2 > +1.0 else t_2
    t_2 = -1.0 if t_2 < -1.0 else t_2
    pitch_y = asin(t_2)

    t_3 = +2.0 * (w * z + x * y)
    t_4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = atan2(t_3, t_4)

    return roll_x, pitch_y, yaw_z  # in radians


def path_to_list(path: Path) -> list[list[float]]:
    """Convert path into list."""
    return [[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in path.poses]


def get_class_from_module(module_name: str) -> 'ModuleBase':
    """
    Get class from module name.

    source: https://docs.python.org/3.10/library/importlib.html#importing-programmatically
    """
    # check if absolute name
    if 'module' not in module_name:
        module_name = f'{module_name}_module'
    spec = find_spec_in_pkg(module_name)
    if spec is None:
        spec = find_spec_in_envvar(module_name)
    if spec is None:
        raise ModuleNotFoundError(f'Module {module_name} not found in cargo_MODULES_PATH')
    print(f'spec: {spec}')
    module = importlib.util.module_from_spec(spec)  # get module from spec
    sys.modules[f'{module_name}'] = module  # adding manually to loaded modules

    spec.loader.exec_module(module)  # load module

    # get class from module
    target = [t for t in dir(module) if 'Module' in t and t != 'ModuleBase']
    return getattr(module, *target)


def find_spec_in_pkg(module_name: str) -> 'ModuleSpec':
    """Search for ModuleSpec in cargo_python_api package default modules folder."""
    spec_name = f'cargo_python_api.modules.{module_name}'
    spec = importlib.util.find_spec(spec_name)
    return spec


def find_spec_in_envvar(module_name: str) -> 'ModuleSpec':
    """Search for ModuleSpec in aerostack2 modules path environment variable."""
    cargo_modules_path_list = os.getenv('cargo_MODULES_PATH').split(':')
    for module_path in cargo_modules_path_list:
        spec = importlib.util.spec_from_file_location(
            module_name, module_path + f'/{module_name}.py'
        )
        if not os.path.exists(spec.origin):
            continue
        return spec
    return None


def get_module_call_signature(module_name: str) -> inspect.Signature:
    """
    Get call method signature from given module name.

    :rtype: inspect.Signature
    """
    class_ = get_class_from_module(module_name)

    signature = inspect.signature(class_.__call__)
    return signature

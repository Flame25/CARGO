
from cargo_python_api.drone_interface_base import DroneInterfaceBase
from cargo_python_api.modules.land_module import LandModule
from cargo_python_api.modules.takeoff_module import TakeoffModule
from cargo_python_api.modules.go_to_module import GoToModule


class DroneInterface(DroneInterfaceBase):
    """Drone interface node."""

    def __init__(self, drone_id: str = 'drone0', verbose: bool = False,
                 use_sim_time: bool = False, spin_rate: float = 20.0) -> None:
        """
        Construct method.

        :param drone_id: drone namespace, defaults to "drone0"
        :type drone_id: str, optional
        :param verbose: output mode, defaults to False
        :type verbose: bool, optional
        :param use_sim_time: use simulation time, defaults to False
        :type use_sim_time: bool, optional
        :param spin_rate: spin rate (Hz), defaults to 20
        :type spin_rate: float, optional
        """
        super().__init__(drone_id=drone_id, verbose=verbose,
                         use_sim_time=use_sim_time, spin_rate=spin_rate)

        self.takeoff = TakeoffModule(drone=self)
        self.goto = GoToModule(drone=self)
        self.land = LandModule(drone=self)

"""Interface for the Fairino FR5 collaborative robot."""

from typing import Any

from ..robot import Robot
from ..config import RobotConfig

JOINT_NAMES = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
    "joint_7",
]


class FR5(Robot):
    """Fairino FR5 robot with seven rotational joints."""

    config_class = RobotConfig
    name = "fr5"

    def __init__(self, config: RobotConfig):
        raise NotImplementedError
        super().__init__(config)

    @property
    def observation_features(self) -> dict:
        return {
            "dtype": "float32",
            "shape": (len(JOINT_NAMES),),
            "names": {"joints": JOINT_NAMES},
        }

    @property
    def action_features(self) -> dict:
        return self.observation_features

    @property
    def is_connected(self) -> bool:
        raise NotImplementedError

    def connect(self, calibrate: bool = True) -> None:
        raise NotImplementedError

    @property
    def is_calibrated(self) -> bool:
        raise NotImplementedError

    def calibrate(self) -> None:
        raise NotImplementedError

    def configure(self) -> None:
        raise NotImplementedError

    def get_observation(self) -> dict[str, Any]:
        raise NotImplementedError

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        raise NotImplementedError

    def disconnect(self) -> None:
        raise NotImplementedError
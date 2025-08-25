import logging
from typing import Any

from fairino import Robot as SDKRobot

from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from ..robot import Robot
from .config_fairino_follower import FairinoFollowerConfig
from ..fairino.fr5 import JOINT_NAMES

logger = logging.getLogger(__name__)


class FairinoFollower(Robot):
    """Follower interface for Fairino FR5 robots using the Fairino SDK."""

    config_class = FairinoFollowerConfig
    name = "fairino_follower"

    def __init__(self, config: FairinoFollowerConfig):
        super().__init__(config)
        self.config = config
        self.sdk_robot = None

    @property
    def observation_features(self) -> dict[str, type]:
        return {f"{j}.pos": float for j in JOINT_NAMES}

    @property
    def action_features(self) -> dict[str, type]:
        return self.observation_features

    @property
    def is_connected(self) -> bool:
        return self.sdk_robot is not None

    def connect(self, calibrate: bool = True) -> None:  # noqa: D401 - interface
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")
        self.sdk_robot = SDKRobot.RPC(self.config.controller_ip)
        self.sdk_robot.Mode(0)
        self.sdk_robot.RobotEnable(1)
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        return None

    def configure(self) -> None:
        return None

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        err, joints = self.sdk_robot.GetActualJointPosDegree()
        if err != 0:
            raise RuntimeError(f"SDK error {err}")
        return {f"{j}.pos": joints[i] for i, j in enumerate(JOINT_NAMES)}

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        if "joint_position" in action:
            q = [float(v) for v in action["joint_position"]]
        elif all(f"{j}.pos" in action for j in JOINT_NAMES):
            q = [float(action[f"{j}.pos"]) for j in JOINT_NAMES]
        elif all(j in action for j in JOINT_NAMES):
            q = [float(action[j]) for j in JOINT_NAMES]
        else:
            raise KeyError("joint_position")
        self.sdk_robot.MoveJ(joint_pos=q, tool=0, user=0, vel=self.config.velocity)
        return {f"{j}.pos": q[i] for i, j in enumerate(JOINT_NAMES)}

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        self.sdk_robot.CloseRPC()
        self.sdk_robot = None
        logger.info(f"{self} disconnected.")
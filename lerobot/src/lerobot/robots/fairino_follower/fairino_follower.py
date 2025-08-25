"""Fairino follower robot using the Fairino Python SDK.

This module provides a minimal :class:`Robot` implementation that wraps the
Fairino SDK's ``Robot`` class (exposed here as :class:`FairinoSDKRobot`). It is
intended to be used as a follower arm within LeRobot's teleoperation pipeline.
"""

from functools import cached_property
from typing import Any

import numpy as np

from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from ..robot import Robot
from .config_fairino_follower import FairinoFollowerConfig

from Robot import RPC as FairinoSDKRobot


class FairinoFollower(Robot):
    """Follower class for a Fairino 6‑DOF robotic arm."""

    config_class = FairinoFollowerConfig
    name = "fairino_follower"

    def __init__(self, config: FairinoFollowerConfig):
        super().__init__(config)
        self.config = config
        self.sdk_robot: FairinoSDKRobot | None = None

    # ------------------------------------------------------------------
    # Properties describing obs/action spaces
    # ------------------------------------------------------------------
    @cached_property
    def observation_features(self) -> dict[str, Any]:
        return {"joint_position": (7,)}

    @cached_property
    def action_features(self) -> dict[str, Any]:
        return {"joint_position": (7,)}

    # ------------------------------------------------------------------
    # Connectivity & lifecycle
    # ------------------------------------------------------------------
    @property
    def is_connected(self) -> bool:  # pragma: no cover - trivial
        return self.sdk_robot is not None

    def connect(self, calibrate: bool = True) -> None:  # pragma: no cover - network/hardware
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")
        self.sdk_robot = FairinoSDKRobot(self.config.ip)
        self.configure()

    @property
    def is_calibrated(self) -> bool:  # pragma: no cover - Fairino has no calibration step
        return True

    def calibrate(self) -> None:  # pragma: no cover - no-op for Fairino
        return None

    def configure(self) -> None:  # pragma: no cover - placeholder for optional config
        pass

    # ------------------------------------------------------------------
    # Observation & action methods
    # ------------------------------------------------------------------
    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        err, q = self.sdk_robot.GetActualJointPosRadian()
        if err != 0:
            raise RuntimeError(f"GetActualJointPosRadian failed: {err}")
        return {"joint_position": np.array(q, dtype=float)}

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        joints = action.get("joint_position", action.get("joint_positions"))
        if joints is None:
            raise KeyError("joint_position")
        q = np.asarray(joints, dtype=float)

        self.sdk_robot.ServoJ(q.tolist(), [0, 0, 0, 0])
        return {"joint_position": q}

    def disconnect(self) -> None:  # pragma: no cover - network/hardware
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        self.sdk_robot.CloseRPC()
        self.sdk_robot = None


# #-------------------------------------------------------------------------------------        
# # lerobot/robots/fairino_follower/fairino_follower.py
# from dataclasses import dataclass
# import numpy as np
# from lerobot.robots.robot import Robot, RobotConfig
# from Robot import RPC as FairinoSDK  # correct import

# @dataclass
# class FairinoFollowerConfig(RobotConfig):
#     ip: str
#     name: str = "fairino_follower"

# class FairinoFollower(Robot):
#     config_class = FairinoFollowerConfig
#     observation_features = ["joint_position"]
#     action_features = ["joint_position"]

#     def __init__(self, config: FairinoFollowerConfig):
#         super().__init__(config)
#         self.sdk_robot = None

#     def connect(self):
#         self.sdk_robot = FairinoSDK(self.config.ip)  # instantiate RPC

#     def get_observation(self):
#         err, q = self.sdk_robot.GetActualJointPosRadian()
#         if err == 0:
#             return {"joint_position": np.array(q)}
#         raise RuntimeError(f"GetActualJointPosRadian failed: {err}")

#     def send_action(self, action):
#         q = action["joint_position"]
#         self.sdk_robot.ServoJ(q.tolist(), [0, 0, 0, 0])  # axisPos placeholder

#     def disconnect(self):
#         if self.sdk_robot:
#             self.sdk_robot.close()



#-------------------------------------------------------------------------------------
# import numpy as np

# from ..robot import Robot
# from .config_fairino_follower import FairinoFollowerConfig

# from Robot import ROBOT_AUX_STATE as FairinoSDKRobot

# fairino_ip = "192.168.58.2"
# arm = FairinoSDKRobot.RPC(fairino_ip)
# print(arm.get_joint_positions())
# arm.servoj([0, 0, 0, 0, 0, 0])
# arm.close()

# # class FairinoFollower(Robot):
# #     """Follower class for a Fairino robotic arm."""

# #     config_class = FairinoFollowerConfig

# #     observation_features = ["joint_position"]
# #     action_features = ["joint_position"]

# #     def __init__(self, config: FairinoFollowerConfig):
# #         super().__init__(config)
# #         self.sdk_robot: FairinoSDKRobot | None = None

# #     def connect(self) -> None:
# #         self.sdk_robot = FairinoSDKRobot.RPC(self.config.ip)

# #     def configure(self) -> None:  # pragma: no cover - placeholder for optional config
# #         pass

# #     def get_observation(self) -> dict:
# #         q = np.array(self.sdk_robot.get_joint_positions())
# #         return {"joint_position": q}

# #     def send_action(self, action: dict) -> None:
# #         q = action["joint_position"]
# #         self.sdk_robot.servoj(q)

# #     def disconnect(self) -> None:
# #         if self.sdk_robot:
# #             self.sdk_robot.close()
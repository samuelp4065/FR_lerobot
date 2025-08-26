import logging
from typing import Any

from fairino import Robot as SDKRobot

from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from ..fairino.fr5 import JOINT_NAMES
from ..robot import Robot
from .config_fairino_follower import FairinoFollowerConfig

# Mapping from SO101 leader keys to FR5 joint indices.
SO101_TO_FR5_INDEX = {
    "shoulder_pan.pos": 0,
    "shoulder_lift.pos": 1,
    "elbow_flex.pos": 2,
    "wrist_flex.pos": 3,
    "wrist_roll.pos": 4,
    "gripper.pos": 5,
}
SO101_KEYS = list(SO101_TO_FR5_INDEX.keys())

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

def connect(self) -> None:
    if self.is_connected:
        raise DeviceAlreadyConnectedError(f"{self} is already connected.")

    # use controller_ip, not ip
    ip_addr = getattr(self.config, "controller_ip", None)
    if ip_addr is None:
        raise ValueError("FairinoFollowerConfig must define 'controller_ip'.")

    try:
        self.sdk_robot = SDKRobot.RPC(ip_addr)
    except AttributeError:
        self.sdk_robot = SDKRobot(ip_addr)

    self.sdk_robot.Mode(0)
    self.sdk_robot.RobotEnable(1)
    logger.info(f"{self} connected to controller at {ip_addr}.")


    @property
    def is_calibrated(self) -> bool:
        # FR controllers expose absolute encoders; nothing to do.
        return True

    def calibrate(self) -> None:
        return None

    def configure(self) -> None:
        return None

    def _pad_or_trim(self, joints: list[float]) -> list[float]:
        """Ensure joint vector length matches len(JOINT_NAMES)."""
        target_len = len(JOINT_NAMES)
        if len(joints) == target_len:
            return joints
        if len(joints) < target_len:
            # pad with default for missing tail joints (e.g., joint_7)
            pad = [self.config.joint_7_default] * (target_len - len(joints))
            return joints + pad
        # Too many → trim
        return joints[:target_len]

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        err, joints = self.sdk_robot.GetActualJointPosDegree()
        if err != 0:
            raise RuntimeError(f"SDK error {err}")
        joints = self._pad_or_trim([float(x) for x in joints])
        return {f"{j}.pos": joints[i] for i, j in enumerate(JOINT_NAMES)}

    def _parse_action_to_joint_vector(self, action: dict[str, Any]) -> list[float]:
        """
        Accepts multiple action formats and returns a joint_position list
        matching len(JOINT_NAMES).
        Supported:
          - {"joint_position": [...]}
          - {"joint_i": angle} or {"joint_i.pos": angle} for all i in JOINT_NAMES
          - SO101 leader keys (mapped via SO101_TO_FR5_INDEX)
        """
        # 1) Direct vector
        if "joint_position" in action:
            joint_position = [float(v) for v in action["joint_position"]]
            return self._pad_or_trim(joint_position)

        # 2) FR5 keys: "joint_1" or "joint_1.pos"
        if all((j in action) or (f"{j}.pos" in action) for j in JOINT_NAMES):
            joint_position = [
                float(action.get(f"{j}.pos", action.get(j))) for j in JOINT_NAMES
            ]
            return self._pad_or_trim(joint_position)
        elif any((j in action) or (f"{j}.pos" in action) for j in JOINT_NAMES):
            missing = [j for j in JOINT_NAMES if j not in action and f"{j}.pos" not in action]
            raise KeyError(f"Missing joint keys: {missing}")

        # 3) SO101 leader keys
        if all(k in action for k in SO101_KEYS):
            joint_position = [0.0] * len(JOINT_NAMES)
            for key, idx in SO101_TO_FR5_INDEX.items():
                joint_position[idx] = float(action[key])
            # Fill any remaining joints (e.g., joint 7) with default
            for i in range(len(JOINT_NAMES)):
                if joint_position[i] == 0.0 and i not in SO101_TO_FR5_INDEX.values():
                    joint_position[i] = self.config.joint_7_default
            return self._pad_or_trim(joint_position)
        elif any(k in action for k in SO101_KEYS):
            missing = [k for k in SO101_KEYS if k not in action]
            raise KeyError(f"Missing SO101 joint keys: {missing}")

        raise KeyError("Unsupported action format: expected 'joint_position', FR5 joint keys, or SO101 leader keys.")

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        joint_position = self._parse_action_to_joint_vector(action)

        # Execute motion
        # Fairino SDK: MoveJ(joint_pos=[...], tool=0, user=0, vel=...)
        self.sdk_robot.MoveJ(
            joint_pos=joint_position, tool=0, user=0, vel=self.config.velocity
        )

        return {f"{j}.pos": joint_position[i] for i, j in enumerate(JOINT_NAMES)}

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        self.sdk_robot.CloseRPC()
        self.sdk_robot = None
        logger.info(f"{self} disconnected.")

import logging
from typing import Any, Dict, List

from fairino import Robot as SDKRobot

from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from ..fairino.fr5 import JOINT_NAMES
from ..robot import Robot
from .config_fairino_follower import FairinoFollowerConfig

# Mapping from SO101 leader keys to FR5 joint indices.
SO101_TO_FR5_INDEX: Dict[str, int] = {
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

    # --- Non-abstract helpers -------------------------------------------------
# inside FairinoFollower

    @property
    def is_connected(self) -> bool:
        return self.sdk_robot is not None
    
    @property
    def observation_features(self) -> dict[str, type]:
        return {f"{j}.pos": float for j in JOINT_NAMES}

    @property
    def action_features(self) -> dict[str, type]:
        return self.observation_features

    # AFTER
    @property
    def observation_features(self) -> dict[str, type]:
        return {f"{j}.pos": float for j in JOINT_NAMES}

    @property
    def action_features(self) -> dict[str, type]:
    # same keys/types as observation
        return self.observation_features


    def _pad_or_trim(self, joints: List[float]) -> List[float]:
        target_len = len(JOINT_NAMES)
        if len(joints) == target_len:
            return joints
        if len(joints) < target_len:
            pad = [self.config.joint_7_default] * (target_len - len(joints))
            return joints + pad
        return joints[:target_len]

    def _parse_action_to_joint_vector(self, action: Dict[str, Any]) -> List[float]:
        # 1) Direct vector
        if "joint_position" in action:
            return self._pad_or_trim([float(v) for v in action["joint_position"]])

        # 2) FR5 keys: "joint_1" or "joint_1.pos"
        if all((j in action) or (f"{j}.pos" in action) for j in JOINT_NAMES):
            vec = [float(action.get(f"{j}.pos", action.get(j))) for j in JOINT_NAMES]
            return self._pad_or_trim(vec)
        elif any((j in action) or (f"{j}.pos" in action) for j in JOINT_NAMES):
            missing = [j for j in JOINT_NAMES if j not in action and f"{j}.pos" not in action]
            raise KeyError(f"Missing joint keys: {missing}")

        # 3) SO101 leader keys
        if all(k in action for k in SO101_KEYS):
            vec = [self.config.joint_7_default] * len(JOINT_NAMES)
            for key, idx in SO101_TO_FR5_INDEX.items():
                vec[idx] = float(action[key])
            return self._pad_or_trim(vec)
        elif any(k in action for k in SO101_KEYS):
            missing = [k for k in SO101_KEYS if k not in action]
            raise KeyError(f"Missing SO101 joint keys: {missing}")

        raise KeyError("Unsupported action format: expected 'joint_position', FR5 joint keys, or SO101 leader keys.")

    # --- Implement ALL abstract methods from Robot ----------------------------

    def connect(self) -> None:
        if self.sdk_robot is not None:
            raise DeviceAlreadyConnectedError(f"{self} is already connected.")

        ip_addr = getattr(self.config, "controller_ip", None)
        if not ip_addr:
            raise ValueError("FairinoFollowerConfig must define 'controller_ip' (e.g., --robot.controller_ip=192.168.x.x).")

        # Fairino SDK constructors vary by build; try RPC first, then direct.
        try:
            self.sdk_robot = SDKRobot.RPC(ip_addr)
        except AttributeError:
            self.sdk_robot = SDKRobot(ip_addr)

        # Position mode + enable
        self.sdk_robot.Mode(0)
        self.sdk_robot.RobotEnable(1)
        logger.info(f"{self} connected to controller at {ip_addr}.")

    def disconnect(self) -> None:
        if self.sdk_robot is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        self.sdk_robot.CloseRPC()
        self.sdk_robot = None
        logger.info(f"{self} disconnected.")

    def configure(self) -> None:
        # No extra configuration needed for FR controllers by default
        return None

    def calibrate(self) -> None:
        # FR controllers expose absolute encoders; nothing to do.
        return None

    def is_calibrated(self) -> bool:
        # IMPORTANT: method (not @property) to satisfy the ABC
        return True

    def get_observation(self) -> Dict[str, Any]:
        if self.sdk_robot is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        err, joints = self.sdk_robot.GetActualJointPosDegree()
        if err != 0:
            raise RuntimeError(f"SDK error {err}")
        joints = self._pad_or_trim([float(x) for x in joints])
        return {f"{j}.pos": joints[i] for i, j in enumerate(JOINT_NAMES)}

    def send_action(self, action: Dict[str, Any]) -> Dict[str, Any]:
        if self.sdk_robot is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        joint_position = self._parse_action_to_joint_vector(action)
        self.sdk_robot.MoveJ(
            joint_pos=joint_position,
            tool=0,
            user=0,
            vel=self.config.velocity,
        )
        return self.get_observation()
    
    def _so101_value_to_deg(self, key: str, v: float) -> float:
        """
        SO101 리더에서 오는 값이 '도'인지 '틱(0~4095 등)'인지 자동 판별하고 도(deg)로 변환.
        - |v| <= 720이면 도로 간주
        - 크면 틱으로 간주하고 ticks -> degrees 선형 변환
        """
        x = float(v)
        # 이미 '도'로 보임
        if -720.0 <= x <= 720.0:
            return x

        # 틱으로 간주: config에 범위 있으면 사용, 없으면 기본(0~4095, offset=0)
        # 키는 "shoulder_pan.pos" 형태를 그대로 사용
        ranges = getattr(self.config, "so101_ticks_ranges", {})
        rng = ranges.get(key, {})
        rmin = float(rng.get("range_min", 0.0))
        rmax = float(rng.get("range_max", 4095.0))
        offset = float(rng.get("homing_offset", 0.0))  # 영점 보정(도)

        span = max(1.0, (rmax - rmin))
        mid = 0.5 * (rmin + rmax)
        # ticks를 중간값 기준 ±180°로 매핑 + 오프셋
        deg = (x - mid) * (360.0 / span) + offset
        return deg

    def _clamp_safe(self, vec: list[float]) -> list[float]:
        """관절별 보수적 안전 리미트로 클램프(필요시 조정). FR5 6축 가정."""
        dof = self._target_len()
        # 축별 안전 범위(보수적으로 시작; 필요시 늘리세요)
        SAFE_MIN = [-170.0, -90.0, -90.0, -90.0, -170.0, -90.0][:dof]
        SAFE_MAX = [ 170.0,  90.0,  90.0,  90.0,  170.0,  90.0][:dof]
        return [max(SAFE_MIN[i], min(SAFE_MAX[i], vec[i])) for i in range(dof)]

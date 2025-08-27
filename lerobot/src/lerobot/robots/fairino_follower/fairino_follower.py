# ✅ Robot.py(로컬 래퍼)만 사용
import sys, time, inspect
sys.path.insert(0, '/home/sw/FR_lerobot/fairino-python-sdk-main/linux/fairino')
import Robot as fairino_sdk

import logging
from typing import Any, Dict, List

# ⛔️ 제거: C 확장(사이트패키지) 혼선 원인
# from fairino import Robot as SDKRobot

from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError



from ..fairino.fr5 import JOINT_NAMES
from ..robot import Robot
from .config_fairino_follower import FairinoFollowerConfig

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
    config_class = FairinoFollowerConfig
    name = "fairino_follower"

    def __init__(self, config: FairinoFollowerConfig):
        super().__init__(config)
        self.config = config
        self.sdk_robot = None

    # --- Non-abstract helpers -------------------------------------------------

    @property
    def is_connected(self) -> bool:
        return self.sdk_robot is not None

    @property
    def observation_features(self) -> dict[str, type]:
        return {f"{j}.pos": float for j in JOINT_NAMES}

    @property
    def action_features(self) -> dict[str, type]:
        return self.observation_features  # same keys/types as observation

    def _target_len(self) -> int:
        return len(JOINT_NAMES)

    def _get_joints_degree(self, dof: int):
        """Try multiple signatures; prefer flag-first returning (err, data)."""
        # 1) flag-first → (err, data)
        try:
            ret = self.sdk_robot.GetActualJointPosDegree(0)
            if isinstance(ret, (list, tuple)) and len(ret) >= 2:
                return ret[0], ret[1]
            if isinstance(ret, int) and ret == 0:
                # 드문 케이스: 데이터 별도 추출 필요 → out-param로 재시도
                pass
        except TypeError:
            pass

        # 2) out-param only → int
        buf = [0.0] * dof
        try:
            err = self.sdk_robot.GetActualJointPosDegree(buf)
            if isinstance(err, int):
                return err, (buf if err == 0 else None)
        except TypeError:
            pass

        # 3) out-param + flag → int
        buf = [0.0] * dof
        try:
            err = self.sdk_robot.GetActualJointPosDegree(buf, 0)
            if isinstance(err, int):
                return err, (buf if err == 0 else None)
        except TypeError:
            pass

        return -1, None

    def _pad_or_trim(self, joints: List[float]) -> List[float]:
        target_len = self._target_len()
        if len(joints) == target_len:
            return joints
        if len(joints) < target_len:
            pad = [getattr(self.config, "joint_7_default", 0.0)] * (target_len - len(joints))
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
            # 원본은 6축만 채워짐
            vec6 = [0.0] * 6
            for key, idx in SO101_TO_FR5_INDEX.items():
                # 필요 시 변환: self._so101_value_to_deg(key, action[key])
                vec6[idx] = float(action[key])

            # 1) 먼저 목표 DOF에 맞춰 pad/trim
            vec = self._pad_or_trim(vec6)
            # 2) 그 다음 안전 클램프
            vec = self._clamp_safe(vec)
            return vec
        elif any(k in action for k in SO101_KEYS):
            missing = [k for k in SO101_KEYS if k not in action]
            raise KeyError(f"Missing SO101 joint keys: {missing}")


    # --- Implement ALL abstract methods from Robot ----------------------------

    def connect(self) -> None:
        """
        Robot.py 기반 Fairino SDK 연결 + 상태패킷 인스턴스화 + 최소 준비 루틴.
        - Robot.py 일부 빌드는 robot_state_pkg가 '클래스'로만 올라와 첫 호출에 CField 에러가 나므로,
        여기서 즉시 인스턴스로 전환하고 짧게 대기 후 캐시를 예열한다.
        - 컨트롤러 준비(드래그 OFF, 서보 ON 등)는 '존재하는 메서드만' 안전하게 시도한다.
        """
        if self.sdk_robot is not None:
            raise DeviceAlreadyConnectedError(f"{self} is already connected.")

        ip_addr = getattr(self.config, "controller_ip", None)
        if not ip_addr:
            raise ValueError(
                "FairinoFollowerConfig must define 'controller_ip' (e.g., --robot.controller_ip=192.168.x.x)."
            )

        # Robot.py 래퍼 사용 (파일 상단에서 fairino_sdk = Robot 임포트되어 있어야 함)
        self.sdk_robot = fairino_sdk.RPC(ip_addr)

        # --- Robot.py 배포본 버그 회피: robot_state_pkg를 인스턴스로 만들어 둔다 ---
        import inspect, time
        if hasattr(self.sdk_robot, "robot_state_pkg") and inspect.isclass(self.sdk_robot.robot_state_pkg):
            try:
                self.sdk_robot.robot_state_pkg = self.sdk_robot.robot_state_pkg()  # ctypes.Structure 인스턴스화
            except Exception as e:
                logger.warning(f"robot_state_pkg instantiate failed: {e}")

        # 연결 직후 컨트롤러가 준비 신호를 내릴 시간을 살짝 준다
        time.sleep(0.1)

        # 일부 빌드엔 GetRobotState가 없으므로 예외 무시하고 캐시 예열만 시도
        try:
            _ = self.sdk_robot.GetRobotState()
            logger.info("GetRobotState precheck OK.")
        except Exception:
            logger.info("GetRobotState precheck skipped: method not available or ignored.")

        # --- (옵션) 존재하는 메서드만 안전하게 준비 시도 ---
        def _safe(name, *args):
            fn = getattr(self.sdk_robot, name, None)
            if callable(fn):
                try:
                    ret = fn(*args)
                    logger.debug(f"{name}{args if args else ''} -> {ret!r}")
                    return ret
                except Exception as e:
                    logger.debug(f"{name}{args if args else ''} !! {e}")
            return None

        # 드래그/교시 OFF (네 환경에선 DragTeachSwitch(0)만 있는 것으로 확인됨)
        _safe("DragTeachSwitch", 0)

        # 서보 ON/Enable (네 환경에선 RobotEnable(1) 성공 확인됨)
        for m in ("RobotEnable", "EnableRobot", "ServoOn", "SetServo"):
            _safe(m, 1)

        # 브레이크 해제 (빌드마다 다름; 있으면 실행)
        if _safe("BrakeControl", 0) is None:
            _safe("BrakeRelease")
            _safe("SetBrake", 0)

        # 주의: 일부 컨트롤러는 원격에서 모드 전환(Mode/SetRunMode/SetAutoMode)을 금지함.
        # 네 장비에선 Mode(0) 호출이 14(거부)였으므로 여기서는 시도하지 않는다.
        # 필요 시 펜던트에서 Auto/Remote 허용 설정을 해두면 MoveJ가 수락됨.

        logger.info(f"{self} connected to controller at {ip_addr}.")


    def disconnect(self) -> None:
        if self.sdk_robot is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        try:
            self.sdk_robot.CloseRPC()
        finally:
            self.sdk_robot = None
        logger.info(f"{self} disconnected.")

    def configure(self) -> None:
        return None

    def calibrate(self) -> None:
        return None

    def is_calibrated(self) -> bool:
        return True

    def get_observation(self):
        dof = self._target_len()

        # ▶ flag-first (err, data) 우선 시도
        try:
            err, joints = self._get_joints_degree(dof)
        except TypeError:
            # 연결 시퀀스가 달라졌을 때 방어
            if hasattr(self.sdk_robot, "robot_state_pkg") and inspect.isclass(self.sdk_robot.robot_state_pkg):
                try:
                    self.sdk_robot.robot_state_pkg = self.sdk_robot.robot_state_pkg()
                except Exception as e:
                    raise RuntimeError(f"robot_state_pkg instantiate failed: {e}")
            try:
                _ = self.sdk_robot.GetRobotState()
            except Exception:
                pass
            err, joints = self._get_joints_degree(dof)

        if err != 0 or joints is None:
            raise RuntimeError(f"GetActualJointPosDegree failed (err={err})")

        # 7축 패딩
        joints = self._pad_or_trim(list(joints))
        return {"joint_positions": joints}

    def _safe(self, name, *args):
        fn = getattr(self.sdk_robot, name, None)
        if callable(fn):
            try: return fn(*args)
            except Exception: return None

    def _fr_ok(self, ret, what="call"):
        if isinstance(ret, int) and ret == 0: return
        if isinstance(ret, (list, tuple)) and len(ret) >= 1 and ret[0] == 0: return
        raise RuntimeError(f"{what} failed: ret={ret!r}")

    def _pre_move_prepare(self):
        """이동 전 최소 준비: 존재하는 메서드만 안전하게 호출."""
        # Drag/Teach OFF
        self._safe("DragTeachSwitch", 0)
        # Servo ON
        for m in ("RobotEnable","EnableRobot","ServoOn","SetServo"):
            self._safe(m, 1)
        # Brake Release (빌드별 시그니처 상이)
        if self._safe("BrakeControl", 0) is None:
            self._safe("BrakeRelease"); self._safe("SetBrake", 0)

    def _try_movej_signatures(self, target, vel: float):
        """빌드별 MoveJ 시그니처를 차례로 시도, 성공(0) 또는 마지막 ret 반환."""
        # 1) 순수 위치 벡터
        try:
            ret = self.sdk_robot.MoveJ(target)
            if isinstance(ret, int) and ret == 0: return ret
        except TypeError:
            pass

        # 2) 키워드 방식 (가장 흔함)
        ret = self.sdk_robot.MoveJ(joint_pos=target, tool=0, user=0, vel=vel)
        if isinstance(ret, int) and ret == 0: return ret

        # 3) 속도/가속 수치 직접 (빌드별): vel, acc를 숫자로만 받는 경우
        try:
            ret = self.sdk_robot.MoveJ(target, vel, getattr(self.config, "acceleration", 5.0))
            if isinstance(ret, int) and ret == 0: return ret
        except TypeError:
            pass

        # 4) 아주 보수적 속도
        try:
            ret = self.sdk_robot.MoveJ(joint_pos=target, tool=0, user=0, vel=max(1.0, min(vel, 20.0)))
            return ret
        except TypeError:
            return ret  # 마지막 시도 결과 반환

    def send_action(self, action: Dict[str, Any]) -> Dict[str, Any]:
        if self.sdk_robot is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # ① 원본 → 목표(절대) 벡터
        raw_vec = self._parse_action_to_joint_vector(action)
        # 안전 여유 5° 안으로
        abs_target = self._clamp_safe(raw_vec, margin_deg=5.0)

        # ② 현재 관절 읽기
        cur = None
        try:
            rr = self.sdk_robot.GetActualJointPosDegree(0)
            if isinstance(rr, (list, tuple)) and len(rr) >= 2 and rr[0] == 0:
                cur = list(rr[1])
        except Exception:
            cur = None

        # ③ 증분 목표 만들기 (큰 점프 금지)
        #    루프 주기(fps 60) 기준으로 관절당 최대 1~2도 정도만 이동
        if cur and len(cur) == len(abs_target):
            max_step = 1.0  # 필요시 0.5~2.0 사이로 조정
            inc_target = []
            for i in range(len(cur)):
                delta = abs_target[i] - cur[i]
                if delta >  max_step: delta =  max_step
                if delta < -max_step: delta = -max_step
                inc_target.append(cur[i] + delta)
            # 다시 여유 클램프
            target = self._clamp_safe(inc_target, margin_deg=5.0)
        else:
            # 현재값을 못 읽었으면 아주 보수적으로 목표를 줄임
            target = self._clamp_safe(abs_target, margin_deg=10.0)

        # ④ 무이동 가드
        if cur and len(cur) == len(target):
            if all(abs(cur[i] - target[i]) < 1e-3 for i in range(len(target))):
                target = target[:]
                target[0] += 0.1

        # ⑤ 이동 전 준비 (있는 메서드만)
        self._pre_move_prepare()

        # ⑥ MoveJ 시도 (보수 속도)
        vel = max(1.0, min(getattr(self.config, "velocity", 10.0), 20.0))
        ret = self._try_movej_signatures(target, vel=vel)

        # ⑦ ret=14면 재시도 (cur 기준 0.5°만 이동)
        if isinstance(ret, int) and ret == 14 and cur and len(cur) == len(target):
            self._pre_move_prepare()
            tgt2 = []
            for i in range(len(cur)):
                d = max(-0.5, min(0.5, target[i] - cur[i]))
                tgt2.append(cur[i] + d)
            tgt2 = self._clamp_safe(tgt2, margin_deg=7.5)
            ret2 = self._try_movej_signatures(tgt2, vel=max(1.0, min(vel, 15.0)))
            if isinstance(ret2, int) and ret2 == 14:
                raise RuntimeError(
                    "MoveJ failed with code 14: controller not in a movable state.\n"
                    "- Set to Auto/Run mode on pendant\n"
                    "- Enable Remote control (UDP control in FR Application)\n"
                    "- Drag/Teach OFF, Servo ON, Brake Released\n"
                    "- Speed Override > 0%\n"
                    "- Clear any safety/alarm interlocks"
                )
            ret = ret2

        self._fr_ok(ret, "MoveJ")
        return self.get_observation()



    def _so101_value_to_deg(self, key: str, v: float) -> float:
        x = float(v)
        if -720.0 <= x <= 720.0:
            return x
        ranges = getattr(self.config, "so101_ticks_ranges", {})
        rng = ranges.get(key, {})
        rmin = float(rng.get("range_min", 0.0))
        rmax = float(rng.get("range_max", 4095.0))
        offset = float(rng.get("homing_offset", 0.0))
        span = max(1.0, (rmax - rmin))
        mid = 0.5 * (rmin + rmax)
        return (x - mid) * (360.0 / span) + offset

    def _target_len(self) -> int:
        return len(JOINT_NAMES)

    def _clamp_safe(self, vec: list[float], margin_deg: float = 5.0) -> list[float]:
        """
        벡터 길이를 타깃 DOF에 맞추고, 보수적 안전범위에서 margin만큼 안쪽으로 클램프.
        margin_deg=5이면 예: -90~90 범위는 -85~+85로 제한.
        """
        dof = self._target_len()

        # 길이 맞추기 (pad/trim)
        v = list(map(float, vec))
        if len(v) < dof:
            v += [getattr(self.config, "joint_7_default", 0.0)] * (dof - len(v))
        elif len(v) > dof:
            v = v[:dof]

        base_min = [-170.0, -90.0, -90.0, -90.0, -170.0, -90.0]
        base_max = [ 170.0,  90.0,  90.0,  90.0,  170.0,  90.0]
        if dof > 6:
            SAFE_MIN = base_min + ([-180.0] * (dof - 6))
            SAFE_MAX = base_max + ([ 180.0] * (dof - 6))
        else:
            SAFE_MIN = base_min[:dof]
            SAFE_MAX = base_max[:dof]

        # margin 적용
        MIN = [SAFE_MIN[i] + margin_deg for i in range(dof)]
        MAX = [SAFE_MAX[i] - margin_deg for i in range(dof)]

        return [max(MIN[i], min(MAX[i], v[i])) for i in range(dof)]


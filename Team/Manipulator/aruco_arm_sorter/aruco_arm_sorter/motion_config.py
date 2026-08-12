"""Load and validate OpenMANIPULATOR-X joint-motion YAML."""

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Mapping, Optional, Tuple

import yaml


ARM_JOINTS: Tuple[str, ...] = ("joint1", "joint2", "joint3", "joint4")


@dataclass(frozen=True)
class MotionStep:
    """One controller target and an optional Gazebo grasp event."""

    name: str
    positions: Dict[str, float]
    gripper: float
    duration: float
    hold: float
    after: Optional[str]


@dataclass(frozen=True)
class MotionConfig:
    """Validated runtime configuration."""

    marker_topic: str
    arm_action: str
    gripper_action: str
    startup_delay: float
    action_server_timeout: float
    joint_limits: Dict[str, Tuple[float, float]]
    gripper_limits: Tuple[float, float]
    initial_positions: Dict[str, float]
    initial_gripper: float
    observation_positions: Dict[str, float]
    observation_gripper: float
    observation_duration: float
    observation_hold: float
    events: Dict[str, str]
    startup_events: Tuple[str, ...]
    marker_sequences: Dict[int, str]
    sequences: Dict[str, List[MotionStep]]


def _require_mapping(value, label: str) -> Mapping:
    if not isinstance(value, Mapping):
        raise ValueError(f"{label} 항목은 key: value 형식이어야 합니다.")
    return value


def _as_float(value, label: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{label} 값은 숫자여야 합니다.")
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} 값은 숫자여야 합니다: {value!r}") from exc


def _validate_range(value: float, limits: Tuple[float, float], label: str) -> None:
    if not limits[0] <= value <= limits[1]:
        raise ValueError(
            f"{label}={value:.4f}가 허용 범위 "
            f"[{limits[0]:.4f}, {limits[1]:.4f}] 밖입니다."
        )


def _absolute_name(value, label: str) -> str:
    name = str(value or "").strip()
    if not name.startswith("/"):
        raise ValueError(
            f"{label}는 /로 시작하는 절대 이름이어야 합니다."
        )
    return name


def load_motion_config(path) -> MotionConfig:
    """Read *path* and return a validated :class:`MotionConfig`."""

    source = Path(path)
    if not source.is_file():
        raise FileNotFoundError(
            f"동작 설정 파일을 찾을 수 없습니다: {source}"
        )

    with source.open("r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream)

    root = _require_mapping(raw, "YAML 최상위")
    controller = _require_mapping(root.get("controller"), "controller")
    interfaces = _require_mapping(root.get("interfaces"), "interfaces")

    marker_topic = _absolute_name(interfaces.get("marker_id"), "interfaces.marker_id")
    arm_action = _absolute_name(interfaces.get("arm_action"), "interfaces.arm_action")
    gripper_action = _absolute_name(
        interfaces.get("gripper_action"), "interfaces.gripper_action"
    )

    limits_raw = _require_mapping(root.get("limits"), "limits")
    joint_limits = {}
    for joint in ARM_JOINTS:
        pair = limits_raw.get(joint)
        if not isinstance(pair, list) or len(pair) != 2:
            raise ValueError(
                f"limits.{joint}는 [최솟값, 최댓값]이어야 합니다."
            )
        limits = (
            _as_float(pair[0], f"limits.{joint}[0]"),
            _as_float(pair[1], f"limits.{joint}[1]"),
        )
        if limits[0] >= limits[1]:
            raise ValueError(
                f"limits.{joint}의 최솟값은 최댓값보다 작아야 합니다."
            )
        joint_limits[joint] = limits

    gripper_pair = limits_raw.get("gripper")
    if not isinstance(gripper_pair, list) or len(gripper_pair) != 2:
        raise ValueError("limits.gripper는 [최솟값, 최댓값]이어야 합니다.")
    gripper_limits = (
        _as_float(gripper_pair[0], "limits.gripper[0]"),
        _as_float(gripper_pair[1], "limits.gripper[1]"),
    )

    initial = _require_mapping(root.get("initial"), "initial")
    initial_positions = {}
    for joint in ARM_JOINTS:
        value = _as_float(initial.get(joint), f"initial.{joint}")
        _validate_range(value, joint_limits[joint], f"initial.{joint}")
        initial_positions[joint] = value
    initial_gripper = _as_float(initial.get("gripper"), "initial.gripper")
    _validate_range(initial_gripper, gripper_limits, "initial.gripper")

    observation = _require_mapping(root.get("observation"), "observation")
    observation_positions_raw = _require_mapping(
        observation.get("positions"), "observation.positions"
    )
    observation_positions = {}
    for joint in ARM_JOINTS:
        value = _as_float(
            observation_positions_raw.get(joint), f"observation.positions.{joint}"
        )
        _validate_range(value, joint_limits[joint], f"observation.positions.{joint}")
        observation_positions[joint] = value
    observation_gripper = _as_float(
        observation.get("gripper"), "observation.gripper"
    )
    _validate_range(observation_gripper, gripper_limits, "observation.gripper")
    observation_duration = _as_float(
        observation.get("duration", 3.0), "observation.duration"
    )
    observation_hold = _as_float(
        observation.get("hold", 0.5), "observation.hold"
    )
    if observation_duration <= 0.0 or observation_hold < 0.0:
        raise ValueError(
            "observation.duration은 양수, observation.hold는 "
            "0 이상이어야 합니다."
        )

    events_raw = _require_mapping(root.get("events"), "events")
    events = {
        str(name): _absolute_name(topic, f"events.{name}")
        for name, topic in events_raw.items()
    }

    startup_events_raw = controller.get("startup_events", [])
    if not isinstance(startup_events_raw, list):
        raise ValueError("controller.startup_events는 목록이어야 합니다.")
    startup_events = tuple(str(name) for name in startup_events_raw)
    for event in startup_events:
        if event not in events:
            raise ValueError(f"정의되지 않은 startup event입니다: {event}")

    marker_raw = _require_mapping(root.get("markers"), "markers")
    marker_sequences = {}
    for marker_value, sequence_name in marker_raw.items():
        try:
            marker_id = int(marker_value)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"마커 ID는 정수여야 합니다: {marker_value!r}"
            ) from exc
        marker_sequences[marker_id] = str(sequence_name)

    sequences_raw = _require_mapping(root.get("sequences"), "sequences")
    sequences = {}
    for sequence_name, step_values in sequences_raw.items():
        if not isinstance(step_values, list) or not step_values:
            raise ValueError(
                f"sequences.{sequence_name}에는 동작 단계가 필요합니다."
            )
        parsed_steps = []
        for index, value in enumerate(step_values):
            label = f"sequences.{sequence_name}[{index}]"
            step_raw = _require_mapping(value, label)
            position_raw = _require_mapping(
                step_raw.get("positions"), f"{label}.positions"
            )
            positions = {}
            for joint in ARM_JOINTS:
                position = _as_float(
                    position_raw.get(joint), f"{label}.positions.{joint}"
                )
                _validate_range(position, joint_limits[joint], f"{label}.{joint}")
                positions[joint] = position

            gripper = _as_float(step_raw.get("gripper"), f"{label}.gripper")
            _validate_range(gripper, gripper_limits, f"{label}.gripper")
            duration = _as_float(step_raw.get("duration", 1.0), f"{label}.duration")
            hold = _as_float(step_raw.get("hold", 0.0), f"{label}.hold")
            if duration <= 0.0 or hold < 0.0:
                raise ValueError(
                    f"{label}의 duration은 양수, hold는 "
                    "0 이상이어야 합니다."
                )

            after_value = step_raw.get("after")
            after = None if after_value in (None, "") else str(after_value)
            if after is not None and after not in events:
                raise ValueError(
                    f"{label}.after에 정의되지 않은 event가 있습니다: "
                    f"{after}"
                )

            parsed_steps.append(
                MotionStep(
                    name=str(step_raw.get("name", f"step_{index}")),
                    positions=positions,
                    gripper=gripper,
                    duration=duration,
                    hold=hold,
                    after=after,
                )
            )
        sequences[str(sequence_name)] = parsed_steps

    for marker_id, sequence_name in marker_sequences.items():
        if sequence_name not in sequences:
            raise ValueError(
                f"markers.{marker_id}가 존재하지 않는 sequence를 "
                "가리킵니다: "
                f"{sequence_name}"
            )

    startup_delay = _as_float(
        controller.get("startup_delay", 2.0), "controller.startup_delay"
    )
    action_server_timeout = _as_float(
        controller.get("action_server_timeout", 45.0),
        "controller.action_server_timeout",
    )
    if startup_delay < 0.0 or action_server_timeout <= 0.0:
        raise ValueError(
            "startup_delay는 0 이상, action_server_timeout은 "
            "양수여야 합니다."
        )

    return MotionConfig(
        marker_topic=marker_topic,
        arm_action=arm_action,
        gripper_action=gripper_action,
        startup_delay=startup_delay,
        action_server_timeout=action_server_timeout,
        joint_limits=joint_limits,
        gripper_limits=gripper_limits,
        initial_positions=initial_positions,
        initial_gripper=initial_gripper,
        observation_positions=observation_positions,
        observation_gripper=observation_gripper,
        observation_duration=observation_duration,
        observation_hold=observation_hold,
        events=events,
        startup_events=startup_events,
        marker_sequences=marker_sequences,
        sequences=sequences,
    )

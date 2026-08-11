"""Select OpenMANIPULATOR-X joint trajectories from an ArUco marker ID."""

import queue
import threading
import time
from pathlib import Path
from typing import Any, Dict

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Empty, Int32, String
from trajectory_msgs.msg import JointTrajectoryPoint

from aruco_arm_sorter.motion_config import ARM_JOINTS, MotionStep, load_motion_config


def _duration(seconds: float) -> Duration:
    whole = int(seconds)
    nanos = int(round((seconds - whole) * 1_000_000_000))
    if nanos >= 1_000_000_000:
        whole += 1
        nanos -= 1_000_000_000
    return Duration(sec=whole, nanosec=nanos)


class ArmSequenceController(Node):
    """Run fixed YAML motions through the official ros2_control actions."""

    def __init__(self):
        super().__init__("arm_sequence_controller")

        default_file = (
            Path(get_package_share_directory("aruco_arm_sorter"))
            / "config"
            / "motions.yaml"
        )
        self.declare_parameter("motion_file", str(default_file))
        self.config = load_motion_config(self.get_parameter("motion_file").value)

        self.arm_client = ActionClient(
            self, FollowJointTrajectory, self.config.arm_action
        )
        self.gripper_client = ActionClient(
            self, GripperCommand, self.config.gripper_action
        )
        self.event_publishers = {
            name: self.create_publisher(Empty, topic, 10)
            for name, topic in self.config.events.items()
        }
        self.status_publisher = self.create_publisher(String, "/arm/status", 10)
        self.active_id_publisher = self.create_publisher(
            Int32, "/arm/active_marker_id", 10
        )
        self.subscription = self.create_subscription(
            Int32, self.config.marker_topic, self._marker_callback, 10
        )

        self.current_gripper = self.config.initial_gripper
        self.pending = queue.Queue(maxsize=max(4, len(self.config.marker_sequences)))
        self.accepted_ids = set()
        self.processed_ids = set()
        self.stop_event = threading.Event()
        self.worker = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker.start()

        self._publish_status("INITIALIZING")
        self._publish_active_id(-1)
        self.get_logger().info(
            f"마커 입력 대기: {self.config.marker_topic}; "
            f"지원 ID: {sorted(self.config.marker_sequences)}"
        )

    def _marker_callback(self, msg: Int32) -> None:
        marker_id = int(msg.data)
        if marker_id not in self.config.marker_sequences:
            self.get_logger().warning(f"등록되지 않은 ArUco ID: {marker_id}")
            return
        if marker_id in self.accepted_ids or marker_id in self.processed_ids:
            self.get_logger().info(f"이미 접수하거나 처리한 ID: {marker_id}")
            return
        try:
            self.pending.put_nowait(marker_id)
        except queue.Full:
            self.get_logger().warning("동작 대기열이 가득 찼습니다.")
            return
        self.accepted_ids.add(marker_id)
        self.get_logger().info(f"ArUco ID {marker_id} 동작을 접수했습니다.")

    def _worker_loop(self) -> None:
        try:
            if not self._sleep(self.config.startup_delay):
                return
            self._wait_for_action_servers()

            for event_name in self.config.startup_events:
                self._send_event(event_name, repeat=3)

            home = MotionStep(
                name="startup_home",
                positions=self.config.initial_positions,
                gripper=self.config.initial_gripper,
                duration=3.0,
                hold=0.5,
                after=None,
            )
            self._execute_step(home, marker_id=-1)
            self._publish_status("READY")

            while not self.stop_event.is_set():
                try:
                    marker_id = self.pending.get(timeout=0.2)
                except queue.Empty:
                    continue
                try:
                    self._run_sequence(marker_id)
                except Exception as exc:
                    self.get_logger().error(
                        f"ID {marker_id} 실행 실패: {type(exc).__name__}: {exc}"
                    )
                    self._publish_status(f"ERROR marker={marker_id}: {exc}")
                    self.accepted_ids.discard(marker_id)
                else:
                    self.processed_ids.add(marker_id)
                    self._publish_status(f"DONE marker={marker_id}")
                finally:
                    self._publish_active_id(-1)
                    self.pending.task_done()
                    if not self.stop_event.is_set():
                        self._publish_status("READY")
        except Exception as exc:
            self.get_logger().error(f"초기화 실패: {type(exc).__name__}: {exc}")
            self._publish_status(f"ERROR initialization: {exc}")

    def _wait_for_action_servers(self) -> None:
        deadline = time.monotonic() + self.config.action_server_timeout
        clients = (
            (self.arm_client, self.config.arm_action),
            (self.gripper_client, self.config.gripper_action),
        )
        for client, name in clients:
            while not self.stop_event.is_set():
                if client.wait_for_server(timeout_sec=1.0):
                    self.get_logger().info(f"Action server 연결: {name}")
                    break
                if time.monotonic() >= deadline:
                    raise TimeoutError(f"Action server를 찾지 못했습니다: {name}")
            if self.stop_event.is_set():
                raise RuntimeError("노드 종료 중")

    def _run_sequence(self, marker_id: int) -> None:
        sequence_name = self.config.marker_sequences[marker_id]
        steps = self.config.sequences[sequence_name]
        self._publish_active_id(marker_id)
        self._publish_status(f"RUNNING marker={marker_id} sequence={sequence_name}")
        for index, step in enumerate(steps, start=1):
            self._publish_status(
                f"RUNNING marker={marker_id} step={index}/{len(steps)} "
                f"name={step.name}"
            )
            self._execute_step(step, marker_id)

    def _execute_step(self, step: MotionStep, marker_id: int) -> None:
        self.get_logger().info(
            f"ID {marker_id}: {step.name} ({step.duration:.1f}s)"
        )
        self._send_arm_goal(step.positions, step.duration, step.name)
        if abs(step.gripper - self.current_gripper) > 1.0e-6:
            self._send_gripper_goal(step.gripper, step.name)
            self.current_gripper = step.gripper
        if not self._sleep(step.hold):
            return
        if step.after is not None:
            self._send_event(step.after, repeat=2)

    def _send_arm_goal(
        self, positions: Dict[str, float], duration: float, label: str
    ) -> None:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [float(positions[joint]) for joint in ARM_JOINTS]
        point.velocities = [0.0] * len(ARM_JOINTS)
        point.time_from_start = _duration(duration)
        goal.trajectory.points = [point]
        goal.goal_time_tolerance = _duration(3.0)
        result = self._send_goal_and_wait(
            self.arm_client, goal, f"팔/{label}", duration + 8.0
        )
        if getattr(result, "error_code", 0) != 0:
            detail = getattr(result, "error_string", "")
            raise RuntimeError(f"팔 trajectory 실패: {detail}")

    def _send_gripper_goal(self, position: float, label: str) -> None:
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = 2.0
        self._send_goal_and_wait(
            self.gripper_client, goal, f"그리퍼/{label}", 6.0
        )

    def _send_goal_and_wait(
        self, client: ActionClient, goal: Any, label: str, timeout: float
    ) -> Any:
        sent = threading.Event()
        result_ready = threading.Event()
        state: Dict[str, Any] = {}

        def sent_callback(future):
            try:
                state["goal_handle"] = future.result()
            except Exception as exc:  # pragma: no cover - ROS runtime path
                state["error"] = exc
            sent.set()

        client.send_goal_async(goal).add_done_callback(sent_callback)
        self._wait_event(sent, timeout, f"{label} goal 전송")
        if "error" in state:
            raise state["error"]
        goal_handle = state["goal_handle"]
        if not goal_handle.accepted:
            raise RuntimeError(f"{label} goal이 거부됐습니다.")

        def result_callback(future):
            try:
                state["wrapped_result"] = future.result()
            except Exception as exc:  # pragma: no cover - ROS runtime path
                state["error"] = exc
            result_ready.set()

        goal_handle.get_result_async().add_done_callback(result_callback)
        self._wait_event(result_ready, timeout, f"{label} 실행")
        if "error" in state:
            raise state["error"]
        wrapped = state["wrapped_result"]
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"{label} Action 상태={wrapped.status}")
        return wrapped.result

    def _wait_event(self, event: threading.Event, timeout: float, label: str) -> None:
        deadline = time.monotonic() + timeout
        while not event.wait(0.1):
            if self.stop_event.is_set():
                raise RuntimeError("노드 종료 중")
            if time.monotonic() >= deadline:
                raise TimeoutError(f"{label} 시간 초과")

    def _send_event(self, name: str, repeat: int = 1) -> None:
        publisher = self.event_publishers[name]
        for _ in range(repeat):
            publisher.publish(Empty())
            if repeat > 1 and not self._sleep(0.12):
                return

    def _publish_status(self, value: str) -> None:
        self.status_publisher.publish(String(data=value))

    def _publish_active_id(self, marker_id: int) -> None:
        self.active_id_publisher.publish(Int32(data=marker_id))

    def _sleep(self, duration: float) -> bool:
        return not self.stop_event.wait(max(0.0, duration))

    def destroy_node(self):
        self.stop_event.set()
        if self.worker.is_alive():
            self.worker.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmSequenceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

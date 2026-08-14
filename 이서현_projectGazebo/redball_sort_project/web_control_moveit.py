# web_control.py와 똑같은 웹 대시보드인데, 박스 이동만 "미리 가르친 자리"가 아니라
# MoveIt으로 계산한 실제 마커 좌표로 정밀하게 집도록 바꾼 버전입니다.
#
# [web_control.py와 다른 점]
#   web_control.py의 /move는 항상 미리 가르친 관절 각도(config/box_slots.yaml)로만 움직여서,
#   박스가 자리에서 몇 cm만 벗어나 있어도 집기가 부정확했습니다. 이 버전은 천장 카메라가
#   실시간으로 본 마커의 world 좌표를 그대로 MoveIt에 "여기로 가줘"라고 넘겨서, 박스가 정확히
#   어디 있든 상관없이 집습니다. 대신 반드시 launch 파일로 실행해야 합니다 (아래 참고).
#
# 실행 방법 (반드시 이 순서):
#   1) 터미널 1: ros2 launch open_manipulator_bringup open_manipulator_x_gazebo.launch.py
#   2) 터미널 2: ros2 launch redball_sort_project box_sort_moveit.launch.py \
#                  use_sim_time:=true node_executable:=web_control_moveit
#   (그냥 `ros2 run redball_sort_project web_control_moveit`으로 실행하면 MoveIt 설정
#    파라미터가 없어서 "Failed to load planning pipelines"로 죽습니다.)

import queue
import threading
import time

import rclpy
from flask import Flask, Response, jsonify, render_template_string, request

from redball_sort_project.box_sort_project import TOTAL_SLOTS, BoxSortProject
from redball_sort_project.moveit_pick_place import MoveItPickPlace
from redball_sort_project.web_control import PAGE, _mjpeg_stream

BOX_Z_M = 0.025  # 박스 중심 높이 (5cm 정육면체 절반) - pick/place 목표 z좌표로 씀

_command_queue: "queue.Queue" = queue.Queue()
_node: BoxSortProject | None = None
_moveit_node: MoveItPickPlace | None = None


def _run_on_robot_thread(func, *args, timeout_sec: float = 60.0, **kwargs):
    done_event = threading.Event()
    result_box: dict = {}
    _command_queue.put((func, args, kwargs, done_event, result_box))
    finished = done_event.wait(timeout=timeout_sec)
    if not finished:
        raise TimeoutError(f"로봇 담당 스레드가 {timeout_sec:.0f}초 안에 응답하지 않았습니다.")
    if "error" in result_box:
        raise RuntimeError(result_box["error"])
    return result_box.get("value")


def run_precise_command(marker_id: int, to_slot: int) -> str:
    # web_control.py의 run_command()와 같은 역할이지만, 집을 위치는 미리 가르친 자세가 아니라
    # 천장 카메라가 "지금 이 순간" 본 마커의 실제 world 좌표를 그대로 씀.
    if to_slot not in _node.slots:
        msg = f"{to_slot}번 자리는 아직 안 가르쳤습니다."
        raise RuntimeError(msg)

    pick_xy = _node.marker_world_positions.get(marker_id)
    if pick_xy is None:
        msg = f"마커 id={marker_id} 박스를 천장 카메라에서 지금 못 보고 있습니다."
        raise RuntimeError(msg)

    place_xy = _node.slot_world_xy(to_slot)
    if place_xy is None:
        msg = f"{to_slot}번 자리의 좌표를 계산할 수 없습니다."
        raise RuntimeError(msg)

    pick_xyz = (pick_xy[0], pick_xy[1], BOX_Z_M)
    place_xyz = (place_xy[0], place_xy[1], BOX_Z_M)
    ok = _moveit_node.pick_and_place(pick_xyz, place_xyz)
    if not ok:
        raise RuntimeError(f"MoveIt 집기/옮기기 실패 (집을 위치={pick_xyz}, 놓을 위치={place_xyz})")
    return f"마커 id={marker_id} 박스: {pick_xyz[:2]} -> {to_slot}번 자리({place_xyz[:2]}) 이동 완료 (MoveIt 정밀 집기)"


def _robot_thread_main():
    global _node, _moveit_node
    rclpy.init()
    _node = BoxSortProject()
    _moveit_node = MoveItPickPlace()
    while rclpy.ok():
        rclpy.spin_once(_node, timeout_sec=0.02)
        rclpy.spin_once(_moveit_node, timeout_sec=0.02)
        try:
            func, args, kwargs, done_event, result_box = _command_queue.get_nowait()
        except queue.Empty:
            continue
        try:
            result_box["value"] = func(*args, **kwargs)
        except Exception as e:  # noqa: BLE001
            result_box["error"] = str(e)
        done_event.set()


app = Flask(__name__)


@app.route("/")
def index():
    return render_template_string(PAGE)


@app.route("/status")
def status():
    return jsonify(
        {
            "joint_position": list(_node.current_joint_position),
            "slot_occupancy": _node.slot_occupancy,
            "teach_cursor": _node.teach_cursor,
            "total_slots": TOTAL_SLOTS,
            "known_marker_ids": sorted(_node.known_marker_ids),
        }
    )


@app.route("/video/overhead")
def video_overhead():
    return Response(
        _mjpeg_stream(lambda: _node.latest_overhead_jpeg),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/video/gripper")
def video_gripper():
    return Response(
        _mjpeg_stream(lambda: _node.latest_gripper_jpeg),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/jog", methods=["POST"])
def jog():
    key = request.json["key"]
    _run_on_robot_thread(_node.handle_key, key)
    return jsonify({"ok": True})


@app.route("/pixel_to_marker", methods=["POST"])
def pixel_to_marker():
    px = float(request.json["x"])
    py = float(request.json["y"])
    marker_id = _node.nearest_marker_for_pixel(px, py)
    return jsonify({"marker_id": marker_id})


@app.route("/pixel_to_slot", methods=["POST"])
def pixel_to_slot():
    px = float(request.json["x"])
    py = float(request.json["y"])
    slot = _node.nearest_slot_for_pixel(px, py)
    return jsonify({"slot": slot})


@app.route("/move", methods=["POST"])
def move():
    marker_id = int(request.json["marker_id"])
    to_slot = int(request.json["to_slot"])
    try:
        message = _run_on_robot_thread(run_precise_command, marker_id, to_slot, timeout_sec=180.0)
    except (RuntimeError, TimeoutError) as e:
        return jsonify({"ok": False, "message": str(e)}), 400
    return jsonify({"ok": True, "message": message})


def main(args=None):
    robot_thread = threading.Thread(target=_robot_thread_main, daemon=True)
    robot_thread.start()

    while _moveit_node is None:
        time.sleep(0.05)

    print("웹 페이지(MoveIt 정밀 집기): http://localhost:8080")
    app.run(host="0.0.0.0", port=8080, threaded=True)


if __name__ == "__main__":
    main()

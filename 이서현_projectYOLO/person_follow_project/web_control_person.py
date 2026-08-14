# person_follow_project.py를 웹 페이지에서 보고 조작하는 프로그램입니다.
#
# [비유로 먼저 이해하기]
#   redball_sort_project의 web_control.py가 box_sort_project.py를 웹으로 옮겨놓은 것과
#   완전히 같은 발상입니다. 로직은 PersonFollowProject 클래스를 그대로 재사용하고, 이
#   파일은 "버튼을 눌렀을 때 어떤 키를 누른 것과 같은 동작을 할지"만 연결해줍니다.
#   박스/자리/실물 전환 같은 기능이 없는 훨씬 단순한 프로젝트라, 이 웹 페이지도
#   카메라 화면 1개 + 상태 + jog 버튼만 있는 가벼운 버전입니다.
#
# [스레드 구조 - web_control.py와 동일한 이유]
#   ROS2 노드와 Flask(웹 서버)를 한 스레드에서 같이 돌리면 서로 막히므로, "로봇 담당"
#   스레드가 rclpy.spin_once()를 돌리며 큐(queue)에 쌓인 명령을 처리하고, Flask는 별도
#   스레드에서 요청을 받아 큐에 명령을 넣고 기다립니다.
#
# 실행 방법:
#   1) colcon build --symlink-install --packages-select person_follow_project
#   2) source install/setup.bash
#   3) ros2 run person_follow_project web_control_person
#   4) 브라우저에서 http://localhost:8080 접속

import queue
import threading
import time

import rclpy
from flask import Flask, Response, jsonify, render_template_string, request

from person_follow_project.person_follow_project import PersonFollowProject

_command_queue: "queue.Queue" = queue.Queue()
_node: PersonFollowProject | None = None


def _run_on_robot_thread(func, *args, timeout_sec: float = 60.0, **kwargs):
    """웹 요청 스레드에서 호출: 로봇 담당 스레드에 작업을 맡기고 끝날 때까지 기다림."""
    done_event = threading.Event()
    result_box: dict = {}
    _command_queue.put((func, args, kwargs, done_event, result_box))
    finished = done_event.wait(timeout=timeout_sec)
    if not finished:
        raise TimeoutError(f"로봇 담당 스레드가 {timeout_sec:.0f}초 안에 응답하지 않았습니다.")
    if "error" in result_box:
        raise RuntimeError(result_box["error"])
    return result_box.get("value")


def _robot_thread_main():
    global _node
    rclpy.init()
    _node = PersonFollowProject()
    while rclpy.ok():
        rclpy.spin_once(_node, timeout_sec=0.05)
        try:
            func, args, kwargs, done_event, result_box = _command_queue.get_nowait()
        except queue.Empty:
            continue
        try:
            result_box["value"] = func(*args, **kwargs)
        except Exception as e:  # noqa: BLE001 - 웹 쪽으로 에러 메시지를 그대로 전달하기 위함
            result_box["error"] = str(e)
        done_event.set()


# ---- Flask 앱 ----
app = Flask(__name__)

PAGE = """
<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>사람 추적 로봇 조작</title>
<style>
  body { font-family: -apple-system, sans-serif; max-width: 700px; margin: 0 auto; padding: 16px; background: #f5f5f5; }
  h1 { font-size: 20px; }
  h2 { font-size: 16px; margin-top: 24px; }
  .card { background: white; border-radius: 12px; padding: 16px; margin-bottom: 16px; box-shadow: 0 1px 3px rgba(0,0,0,0.1); }
  .jog-grid { display: grid; grid-template-columns: repeat(4, 1fr); gap: 8px; }
  button { font-size: 16px; padding: 12px; border-radius: 8px; border: 1px solid #ccc; background: #fff; cursor: pointer; }
  button:active { background: #eee; }
  .row { display: flex; gap: 8px; margin-top: 8px; }
  .row button { flex: 1; }
  #status { font-size: 14px; color: #333; white-space: pre-wrap; }
  .big { padding: 16px; font-size: 18px; font-weight: bold; width: 100%; }
  .open-btn { background: #d4edda; }
  .close-btn { background: #f8d7da; }
  .mode-btn { background: #cce5ff; margin-top: 8px; }
  img#cam { width: 100%; border-radius: 8px; background: #000; display: block; }
  img#camRaw { width: 100%; min-height: 320px; object-fit: contain; border-radius: 8px; background: #000; display: block; }
</style>
</head>
<body>
  <h1>사람 추적 로봇 조작 (YOLO26)</h1>

  <div class="card">
    <h2>카메라 (인식 표시)</h2>
    <img id="cam" src="/video/gripper" alt="gripper camera">
  </div>

  <div class="card">
    <h2>카메라 (원본, 깨끗한 화면)</h2>
    <img id="camRaw" src="/video/gripper_raw" alt="gripper camera raw">
  </div>

  <div class="card">
    <h2>지금 상태</h2>
    <div id="status">불러오는 중...</div>
    <button class="big mode-btn" onclick="jog('m')">모드 전환 (MANUAL → AUTO → GESTURE)</button>
    <button class="big" style="margin-top:8px; background:#fff3cd;" onclick="resetDefault()">디폴트 위치로 옮기기</button>
    <div id="resetResult" style="font-size: 13px; margin-top: 8px;"></div>
  </div>

  <div class="card">
    <h2>수동 조작 (MANUAL)</h2>
    <div class="jog-grid">
      <button onclick="jog('q')">joint1 +</button>
      <button onclick="jog('a')">joint1 -</button>
      <button onclick="jog('w')">joint2 +</button>
      <button onclick="jog('s')">joint2 -</button>
      <button onclick="jog('e')">joint3 +</button>
      <button onclick="jog('d')">joint3 -</button>
      <button onclick="jog('r')">joint4 +</button>
      <button onclick="jog('f')">joint4 -</button>
    </div>
    <div class="row">
      <button class="open-btn" onclick="jog('z')">그리퍼 열기</button>
      <button class="close-btn" onclick="jog('x')">그리퍼 닫기</button>
    </div>
  </div>

<script>
async function refreshStatus() {
  const res = await fetch('/status');
  const data = await res.json();
  const j = data.joint_position.map(v => v.toFixed(3)).join(', ');
  document.getElementById('status').textContent =
    `모드: ${data.mode}\\n` +
    `관절 각도(joint1~4): ${j}\\n` +
    `인사 상태: ${data.greet_state}\\n` +
    `탐색(사람 찾는 중) 여부: ${data.searching ? '예' : '아니오'}`;
}

async function jog(key) {
  await fetch('/jog', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({key: key}),
  });
  refreshStatus();
}

async function resetDefault() {
  const resultDiv = document.getElementById('resetResult');
  resultDiv.textContent = '이동 중...';
  try {
    await fetch('/reset_default', { method: 'POST' });
    resultDiv.style.color = '#2e7d32';
    resultDiv.textContent = '디폴트 위치로 이동 명령을 보냈습니다.';
  } catch (e) {
    resultDiv.style.color = '#c0392b';
    resultDiv.textContent = '요청이 실패했습니다: ' + e;
  } finally {
    refreshStatus();
  }
}

refreshStatus();
setInterval(refreshStatus, 1000);
</script>
</body>
</html>
"""


@app.route("/")
def index():
    return render_template_string(PAGE)


@app.route("/status")
def status():
    return jsonify(
        {
            "mode": _node.mode,
            "joint_position": list(_node.current_joint_position),
            "greet_state": _node.greet_state,
            "searching": _node.searching,
        }
    )


def _mjpeg_stream(get_frame):
    while True:
        frame = get_frame()
        if frame is not None:
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
        time.sleep(0.05)


@app.route("/video/gripper")
def video_gripper():
    return Response(
        _mjpeg_stream(lambda: _node.latest_gripper_jpeg),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/video/gripper_raw")
def video_gripper_raw():
    return Response(
        _mjpeg_stream(lambda: _node.latest_gripper_jpeg_raw),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/jog", methods=["POST"])
def jog():
    key = request.json["key"]
    _run_on_robot_thread(_node.handle_key, key)
    return jsonify({"ok": True})


@app.route("/reset_default", methods=["POST"])
def reset_default():
    _run_on_robot_thread(_node.reset_to_default)
    return jsonify({"ok": True})


def main(args=None):
    robot_thread = threading.Thread(target=_robot_thread_main, daemon=True)
    robot_thread.start()

    while _node is None:
        time.sleep(0.05)

    print("웹 페이지: http://localhost:8080 (같은 네트워크의 다른 기기에서는 http://<이 컴퓨터의 IP>:8080)")
    # threaded=True 필수: 카메라 스트리밍(/video/gripper)은 연결을 계속 열어두고 있어서,
    # 스레드 하나짜리 서버면 스트리밍 보는 동안 다른 버튼(jog)이 전부 안 먹힘.
    app.run(host="0.0.0.0", port=8080, threaded=True)


if __name__ == "__main__":
    main()

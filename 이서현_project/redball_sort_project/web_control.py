# box_sort_project.py를 웹 페이지에서 간단히 조작하는 프로그램입니다.
#
# [비유로 먼저 이해하기]
#   box_sort_project.py가 "키보드로만 조종하는 리모컨"이라면, 이 파일은 그 리모컨을
#   웹 페이지의 버튼으로 그대로 옮겨놓은 것입니다. 로직은 box_sort_project.py의
#   BoxSortProject 클래스를 그대로 재사용하고(똑같은 그리퍼/카메라/자리 코드), 이
#   파일은 "버튼을 눌렀을 때 어떤 키를 누른 것과 같은 동작을 할지"만 연결해줍니다.
#
# [스레드 구조 - 왜 이렇게 나눴는지]
#   ROS2 노드(로봇과 통신하는 부분)와 Flask(웹 서버)는 각자 자기만의 반복문을 돌리고
#   싶어합니다. 이 둘을 한 스레드에서 같이 돌리면 서로 막혀버립니다. 그래서:
#     - "로봇 담당" 스레드 하나가 계속 rclpy.spin_once()를 돌리면서 로봇 상태(관절 각도,
#       카메라로 본 자리 점유 현황)를 최신으로 유지하고, 큐(queue)에 쌓인 명령을 처리합니다.
#     - Flask(웹 서버)는 별도 스레드에서 HTTP 요청을 받고, 로봇을 움직여야 하면 그 명령을
#       큐에 넣고 "로봇 담당" 스레드가 처리할 때까지 기다립니다.
#   이렇게 하면 "로봇과 대화하는 코드"는 항상 한 스레드에서만 실행되어서 안전합니다.
#
# 실행 방법:
#   1) colcon build --symlink-install --packages-select redball_sort_project
#   2) source install/setup.bash
#   3) ros2 run redball_sort_project web_control
#   4) 브라우저에서 http://localhost:8080 접속

import queue
import threading
import time

import rclpy
from flask import Flask, Response, jsonify, render_template_string, request

from redball_sort_project.box_sort_project import TOTAL_SLOTS, BoxSortProject

# ---- 로봇 담당 스레드와 Flask 스레드 사이를 잇는 큐 ----
# (func, args, kwargs, 완료됐다고 알려줄 Event, 결과를 담을 dict) 튜플을 넣으면
# 로봇 담당 스레드가 꺼내서 실행하고 Event를 set() 해줍니다.
_command_queue: "queue.Queue" = queue.Queue()
_node: BoxSortProject | None = None


def _run_on_robot_thread(func, *args, **kwargs):
    """웹 요청 스레드에서 호출: 로봇 담당 스레드에 작업을 맡기고 끝날 때까지 기다림."""
    done_event = threading.Event()
    result_box: dict = {}
    _command_queue.put((func, args, kwargs, done_event, result_box))
    finished = done_event.wait(timeout=60.0)
    if not finished:
        raise TimeoutError("로봇 담당 스레드가 60초 안에 응답하지 않았습니다.")
    if "error" in result_box:
        raise RuntimeError(result_box["error"])
    return result_box.get("value")


def _robot_thread_main():
    global _node
    rclpy.init()
    _node = BoxSortProject()
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
<title>아루코 박스 로봇 조작</title>
<style>
  body { font-family: -apple-system, sans-serif; max-width: 480px; margin: 0 auto; padding: 16px; background: #f5f5f5; }
  h1 { font-size: 20px; }
  h2 { font-size: 16px; margin-top: 24px; }
  .card { background: white; border-radius: 12px; padding: 16px; margin-bottom: 16px; box-shadow: 0 1px 3px rgba(0,0,0,0.1); }
  .jog-grid { display: grid; grid-template-columns: repeat(4, 1fr); gap: 8px; }
  button { font-size: 16px; padding: 12px; border-radius: 8px; border: 1px solid #ccc; background: #fff; cursor: pointer; }
  button:active { background: #eee; }
  .row { display: flex; gap: 8px; margin-top: 8px; }
  .row button, .row select { flex: 1; }
  select { font-size: 16px; padding: 10px; border-radius: 8px; border: 1px solid #ccc; }
  #status { font-size: 14px; color: #333; white-space: pre-wrap; }
  .big { padding: 16px; font-size: 18px; font-weight: bold; }
  .open-btn { background: #d4edda; }
  .close-btn { background: #f8d7da; }
  .move-btn { background: #cce5ff; width: 100%; margin-top: 8px; }
  table { width: 100%; border-collapse: collapse; font-size: 14px; }
  td, th { border-bottom: 1px solid #eee; padding: 4px; text-align: left; }
  .video-row { display: flex; gap: 8px; }
  .video-box { flex: 1; text-align: center; }
  .video-box img { width: 100%; border-radius: 8px; background: #000; display: block; }
  .video-box p { font-size: 12px; color: #666; margin: 4px 0; }
</style>
</head>
<body>
  <h1>아루코 박스 로봇 조작</h1>

  <div class="card">
    <h2>카메라</h2>
    <div class="video-row">
      <div class="video-box">
        <img src="/video/overhead" alt="overhead camera">
        <p>천장 카메라 (자리 점유 확인용)</p>
      </div>
      <div class="video-box">
        <img src="/video/gripper" alt="gripper camera">
        <p>손목 카메라</p>
      </div>
    </div>
  </div>

  <div class="card">
    <h2>지금 상태</h2>
    <div id="status">불러오는 중...</div>
  </div>

  <div class="card">
    <h2>수동 조작 (TEACH)</h2>
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
    <div class="row">
      <button onclick="jog('b')">◀ 이전 자리</button>
      <button onclick="jog('n')">다음 자리 ▶</button>
    </div>
    <button class="big" style="width:100%; margin-top:8px;" onclick="jog('g')">지금 자세를 자리로 저장</button>
  </div>

  <div class="card">
    <h2>박스 이동 (COMMAND)</h2>
    <div class="row">
      <select id="markerSelect">
        <option value="0">마커 id 0</option>
        <option value="1">마커 id 1</option>
      </select>
      <select id="slotSelect"></select>
    </div>
    <button class="move-btn big" onclick="moveBox()">이동</button>
  </div>

<script>
async function refreshStatus() {
  const res = await fetch('/status');
  const data = await res.json();
  const occ = Object.entries(data.slot_occupancy).map(([slot, mid]) => `${slot}번 자리: 마커 id=${mid}`).join('\\n') || '(비어있음)';
  const j = data.joint_position.map(v => v.toFixed(3)).join(', ');
  document.getElementById('status').textContent =
    `TEACH 커서(지금 가르칠 자리): ${data.teach_cursor}\\n` +
    `관절 각도(joint1~4): ${j}\\n` +
    `자리 점유 현황:\\n${occ}`;

  const slotSelect = document.getElementById('slotSelect');
  if (slotSelect.options.length !== data.total_slots) {
    slotSelect.innerHTML = '';
    for (let i = 1; i <= data.total_slots; i++) {
      const opt = document.createElement('option');
      opt.value = i;
      opt.textContent = `${i}번 자리`;
      slotSelect.appendChild(opt);
    }
  }
}

async function jog(key) {
  await fetch('/jog', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({key: key}),
  });
  refreshStatus();
}

async function moveBox() {
  const markerId = document.getElementById('markerSelect').value;
  const toSlot = document.getElementById('slotSelect').value;
  const btn = document.querySelector('.move-btn');
  btn.disabled = true;
  btn.textContent = '이동 중...';
  try {
    await fetch('/move', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({marker_id: parseInt(markerId), to_slot: parseInt(toSlot)}),
    });
  } finally {
    btn.disabled = false;
    btn.textContent = '이동';
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
            "joint_position": list(_node.current_joint_position),
            "slot_occupancy": _node.slot_occupancy,
            "teach_cursor": _node.teach_cursor,
            "total_slots": TOTAL_SLOTS,
        }
    )


def _mjpeg_stream(get_frame):
    # MJPEG 스트리밍: 브라우저가 <img> 태그로 이 URL을 열어두면, 새 jpg가 나올 때마다
    # 계속 이어서 보내줘서 마치 동영상처럼 보이게 하는 오래되고 아주 단순한 방식.
    while True:
        frame = get_frame()
        if frame is not None:
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
        time.sleep(0.05)


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


@app.route("/move", methods=["POST"])
def move():
    marker_id = int(request.json["marker_id"])
    to_slot = int(request.json["to_slot"])
    _run_on_robot_thread(_node.run_command, marker_id, to_slot)
    return jsonify({"ok": True})


def main(args=None):
    robot_thread = threading.Thread(target=_robot_thread_main, daemon=True)
    robot_thread.start()

    # _node가 만들어질 때까지 잠깐 대기 (로봇 담당 스레드가 rclpy.init() + 노드 생성을 마칠 시간)
    while _node is None:
        time.sleep(0.05)

    print("웹 페이지: http://localhost:8080 (같은 네트워크의 다른 기기에서는 http://<이 컴퓨터의 IP>:8080)")
    # threaded=True 필수: 카메라 스트리밍(/video/...)은 연결을 계속 열어두고 있어서,
    # 스레드 하나짜리 서버면 스트리밍 보는 동안 다른 버튼(jog/move/status)이 전부 안 먹힘.
    app.run(host="0.0.0.0", port=8080, threaded=True)


if __name__ == "__main__":
    main()

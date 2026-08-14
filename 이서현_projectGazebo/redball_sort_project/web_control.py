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

import os
import queue
import signal
import subprocess
import threading
import time

import rclpy
from flask import Flask, Response, jsonify, render_template_string, request

from redball_sort_project.box_sort_project import TOTAL_SLOTS, BoxSortProject

# ---- Gazebo(시뮬레이션) <-> 실물 로봇 전환 ----
# 둘 다 /arm_controller, /gripper_controller 같은 똑같은 토픽/액션 이름을 쓰기 때문에
# 동시에 띄우면 서로 충돌함(CLAUDE.md에 이미 기록된 사고). 그래서 "전환"은 실제로는
# "지금 떠 있는 쪽을 완전히 끄고 -> 선택한 쪽을 새로 켜고 -> 컨트롤러가 활성화될 때까지
# 기다리는" 과정임. web_control 자신(BoxSortProject 노드)은 안 죽여도 됨 - 액션
# 클라이언트가 매번 wait_for_server()로 다시 찾기 때문에 백엔드가 바뀌어도 알아서 재연결됨.
_BACKEND_PROCESS_PATTERNS = [
    "gz sim",
    "ros2_control_node",
    "robot_state_publisher",
    "parameter_bridge",
    "joint_trajectory_executor",
]
_backend_lock = threading.Lock()
_backend_proc: subprocess.Popen | None = None
_backend_mode: str | None = None  # "sim" | "real" | None


def _kill_stray_backend_processes() -> None:
    # 버튼이 아니라 터미널에서 수동으로 띄워둔 것까지 포함해서 싹 다 정리함(이번 세션
    # 내내 겪은 "좀비 프로세스가 안 죽어서 새로 띄운 것과 충돌" 문제를 버튼에서도 반복 안
    # 하기 위함). pkill로 안 죽는 경우가 있어서(CLAUDE.md 참고) 반드시 재확인함.
    for pattern in _BACKEND_PROCESS_PATTERNS:
        subprocess.run(["pkill", "-9", "-f", pattern], capture_output=True)
    time.sleep(1.0)
    remaining = subprocess.run(
        ["pgrep", "-f", "|".join(_BACKEND_PROCESS_PATTERNS)],
        capture_output=True, text=True,
    ).stdout.strip()
    if remaining:
        for pid in remaining.splitlines():
            subprocess.run(["kill", "-9", pid.strip()], capture_output=True)
        time.sleep(1.0)


def _wait_controllers_active(timeout_sec: float) -> bool:
    # 초기엔 controller_manager가 아직 안 떠 있어서 이 명령 자체가 응답 없이 오래 걸리거나
    # 타임아웃될 수 있음(정상) - 그때마다 전체를 포기하지 않고 계속 재시도해야 함.
    deadline = time.monotonic() + timeout_sec
    names = ("arm_controller", "gripper_controller", "joint_state_broadcaster")
    while time.monotonic() < deadline:
        try:
            result = subprocess.run(
                ["ros2", "control", "list_controllers"],
                capture_output=True, text=True, timeout=4.0,
            )
            lines_ok = all(
                any(n in line and "active" in line for line in result.stdout.splitlines())
                for n in names
            )
            if lines_ok:
                return True
        except subprocess.TimeoutExpired:
            pass
        time.sleep(1.0)
    return False


def switch_backend(mode: str) -> str:
    if mode not in ("sim", "real"):
        raise ValueError(f"알 수 없는 모드: {mode}")
    global _backend_proc, _backend_mode
    with _backend_lock:
        if _backend_proc is not None:
            try:
                os.killpg(os.getpgid(_backend_proc.pid), signal.SIGINT)
                _backend_proc.wait(timeout=15)
            except Exception:
                pass
            _backend_proc = None
        _kill_stray_backend_processes()

        if mode == "sim":
            launch_cmd = "ros2 launch open_manipulator_bringup open_manipulator_x_gazebo.launch.py"
            wait_sec = 40.0
        else:
            launch_cmd = "ros2 launch open_manipulator_bringup open_manipulator_x.launch.py"
            wait_sec = 25.0

        full_cmd = (
            "source /opt/ros/jazzy/setup.bash && "
            "source /home/lee/open_manipulator_ws/install/setup.bash && "
            f"exec {launch_cmd}"
        )
        _backend_proc = subprocess.Popen(
            ["bash", "-c", full_cmd],
            preexec_fn=os.setsid,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        _backend_mode = mode

        if not _wait_controllers_active(wait_sec):
            raise RuntimeError(
                f"{wait_sec:.0f}초 안에 컨트롤러가 활성화되지 않았습니다 "
                f"({'Gazebo' if mode == 'sim' else '실물 로봇'} 쪽 로그를 확인해보세요)."
            )
        return mode

# 박스 2개의 "기본(디폴트)" world 좌표. open_manipulator_bringup의 empty_world.sdf에 있는
# aruco_cube_1/aruco_cube_2의 <pose>와 반드시 같은 값이어야 함(그래야 "디폴트로 되돌리기"가
# 실제 월드 시작 상태와 일치함). y를 정확히 0으로 두면 천장 카메라 인식이 이상하게 실패하는
# 특이점이 있어서(실측) 일부러 살짝 어긋나게 둠 - empty_world.sdf 쪽 주석 참고.
DEFAULT_BOX_POSES = {
    "aruco_cube_1": (0.283, 0.02, 0.025),
    "aruco_cube_2": (-0.283, 0.02, 0.025),
}


def reset_boxes_to_default(world_name: str = "empty") -> None:
    # gz_ros2_control 쪽엔 "모델 좌표를 순간이동시키는" ROS 서비스가 없어서, Gazebo가 직접
    # 제공하는 /world/<world>/set_pose 서비스를 gz CLI로 호출함(로봇 팔로 하나씩 옮기는 것보다
    # 훨씬 빠르고 확실함 - 실제로 세션 내내 이 방법으로 정리했었음, 그걸 버튼으로 옮긴 것).
    for name, (x, y, z) in DEFAULT_BOX_POSES.items():
        req = f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{x: 0, y: 0, z: 0, w: 1}}'
        result = subprocess.run(
            [
                "gz", "service", "-s", f"/world/{world_name}/set_pose",
                "--reqtype", "gz.msgs.Pose",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "2000",
                "--req", req,
            ],
            capture_output=True, text=True, timeout=5.0,
        )
        if "data: true" not in result.stdout:
            raise RuntimeError(f"{name} 위치 초기화 실패: {result.stdout or result.stderr}")

# ---- 로봇 담당 스레드와 Flask 스레드 사이를 잇는 큐 ----
# (func, args, kwargs, 완료됐다고 알려줄 Event, 결과를 담을 dict) 튜플을 넣으면
# 로봇 담당 스레드가 꺼내서 실행하고 Event를 set() 해줍니다.
_command_queue: "queue.Queue" = queue.Queue()
_node: BoxSortProject | None = None


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
  body { font-family: -apple-system, sans-serif; max-width: 1100px; margin: 0 auto; padding: 16px; background: #f5f5f5; }
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
    <h2>조작 대상</h2>
    <div class="row">
      <button id="modeSimBtn" class="big" onclick="switchMode('sim')">Gazebo(시뮬레이션)</button>
      <button id="modeRealBtn" class="big" onclick="switchMode('real')">실물 로봇</button>
    </div>
    <div id="modeStatus" style="font-size: 13px; margin-top: 8px;">
      전환 버튼을 눌러 지금 조작할 대상을 선택하세요. 30~40초 정도 걸리고, 그 사이 팔이 멈춰있을 수 있습니다.
    </div>
  </div>

  <div class="card">
    <h2>카메라</h2>
    <div class="video-row">
      <div class="video-box">
        <img id="overheadImg" src="/video/overhead" alt="overhead camera" style="cursor: crosshair;">
        <p>천장 카메라 — 박스 클릭 → 도착 자리 클릭하면 바로 이동</p>
      </div>
      <div class="video-box">
        <img src="/video/gripper" alt="gripper camera">
        <p>손목 카메라</p>
      </div>
    </div>
    <div id="clickStatus" style="font-size: 13px; margin-top: 8px;"></div>
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
    <div class="row" style="align-items:center; gap:8px;">
      <span>닫는 정도</span>
      <input id="gripperLevel" type="range" min="1" max="10" value="10" step="1"
             oninput="document.getElementById('gripperLevelValue').textContent=this.value"
             onchange="setGripperLevel(this.value)" style="flex:1;">
      <span id="gripperLevelValue">10</span>/10
    </div>
    <div class="row">
      <button onclick="jog('b')">◀ 이전 자리</button>
      <button onclick="jog('n')">다음 자리 ▶</button>
    </div>
    <button class="big" style="width:100%; margin-top:8px;" onclick="jog('g')">지금 자세를 자리로 저장</button>
    <button class="big" style="width:100%; margin-top:8px;" onclick="resetArm()">매니퓰레이터 디폴트 위치로</button>
    <div id="resetArmResult" style="font-size: 13px; margin-top: 8px;"></div>
  </div>

  <div class="card">
    <h2>박스 이동 (COMMAND)</h2>
    <div class="row">
      <select id="markerSelect"></select>
      <select id="slotSelect"></select>
    </div>
    <button class="move-btn big" onclick="moveBox()">이동</button>
    <div id="moveResult" style="font-size: 13px; margin-top: 8px;"></div>
    <button class="big" style="width:100%; margin-top:8px;" onclick="resetBoxes()">박스 디폴트 위치로 되돌리기</button>
    <div id="resetResult" style="font-size: 13px; margin-top: 8px;"></div>
  </div>

  <div class="card">
    <h2>박스 이동 (카메라 없이, 자리→자리)</h2>
    <p style="font-size: 13px; color: #666;">천장 카메라가 없을 때(실물 로봇) 씁니다. 마커를 못 찾아도, 출발 자리에 <b>진짜로 박스가 있다고 믿고</b> 바로 집으러 가니 주의하세요.</p>
    <div class="row">
      <select id="blindFromSlot"></select>
      <span>→</span>
      <select id="blindToSlot"></select>
    </div>
    <button class="move-btn big" onclick="moveBlind()">카메라 없이 이동</button>
    <div id="moveBlindResult" style="font-size: 13px; margin-top: 8px;"></div>
  </div>

<script>
let lastStatus = null;
let pendingMarkerId = null;

async function refreshStatus() {
  const res = await fetch('/status');
  const data = await res.json();
  lastStatus = data;
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

  for (const selId of ['blindFromSlot', 'blindToSlot']) {
    const sel = document.getElementById(selId);
    if (sel.options.length !== data.total_slots) {
      sel.innerHTML = '';
      for (let i = 1; i <= data.total_slots; i++) {
        const opt = document.createElement('option');
        opt.value = i;
        opt.textContent = `${i}번 자리`;
        sel.appendChild(opt);
      }
      if (selId === 'blindToSlot' && data.total_slots > 1) sel.value = '2';
    }
  }

  // 마커 id 드롭다운: 천장 카메라가 지금까지 실제로 본 마커 id로 채움 (하드코딩 아님).
  const markerSelect = document.getElementById('markerSelect');
  const currentIds = Array.from(markerSelect.options).map(o => o.value).join(',');
  const newIds = data.known_marker_ids.join(',');
  if (currentIds !== newIds) {
    const prevValue = markerSelect.value;
    markerSelect.innerHTML = '';
    for (const id of data.known_marker_ids) {
      const opt = document.createElement('option');
      opt.value = id;
      opt.textContent = `마커 id ${id}`;
      markerSelect.appendChild(opt);
    }
    if (data.known_marker_ids.map(String).includes(prevValue)) {
      markerSelect.value = prevValue;
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

async function setGripperLevel(level) {
  await fetch('/gripper_level', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({level: parseInt(level)}),
  });
}

async function moveBox() {
  const resultDiv = document.getElementById('moveResult');
  const markerId = document.getElementById('markerSelect').value;
  const toSlot = document.getElementById('slotSelect').value;
  if (markerId === '') {
    resultDiv.style.color = '#c0392b';
    resultDiv.textContent = '아직 천장 카메라가 마커를 하나도 못 봤습니다. 박스가 카메라에 보이는지 확인하세요.';
    return;
  }
  const btn = document.querySelector('.move-btn');
  btn.disabled = true;
  btn.textContent = '이동 중...';
  resultDiv.textContent = '';
  try {
    const res = await fetch('/move', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({marker_id: parseInt(markerId), to_slot: parseInt(toSlot)}),
    });
    const data = await res.json();
    resultDiv.style.color = data.ok ? '#2e7d32' : '#c0392b';
    resultDiv.textContent = data.message || (data.ok ? '완료' : '실패');
  } catch (e) {
    resultDiv.style.color = '#c0392b';
    resultDiv.textContent = '요청이 실패했습니다: ' + e;
  } finally {
    btn.disabled = false;
    btn.textContent = '이동';
    refreshStatus();
  }
}

async function refreshBackendStatus() {
  const res = await fetch('/backend_status');
  const data = await res.json();
  const label = data.mode === 'sim' ? 'Gazebo(시뮬레이션)' : (data.mode === 'real' ? '실물 로봇' : '(아직 선택 안 됨)');
  document.getElementById('modeSimBtn').disabled = data.switching;
  document.getElementById('modeRealBtn').disabled = data.switching;
  if (!data.switching) {
    document.getElementById('modeStatus').textContent = `지금 조작 대상: ${label}`;
  }
}

async function switchMode(mode) {
  const statusDiv = document.getElementById('modeStatus');
  document.getElementById('modeSimBtn').disabled = true;
  document.getElementById('modeRealBtn').disabled = true;
  statusDiv.style.color = '#333';
  statusDiv.textContent = (mode === 'sim' ? 'Gazebo' : '실물 로봇') + ' 켜는 중... (30~40초 정도 걸립니다)';
  try {
    const res = await fetch('/switch_mode', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({mode: mode}),
    });
    const data = await res.json();
    statusDiv.style.color = data.ok ? '#2e7d32' : '#c0392b';
    statusDiv.textContent = data.message || (data.ok ? '완료' : '실패');
  } catch (e) {
    statusDiv.style.color = '#c0392b';
    statusDiv.textContent = '요청이 실패했습니다: ' + e;
  } finally {
    refreshBackendStatus();
    refreshStatus();
  }
}

async function resetArm() {
  const resultDiv = document.getElementById('resetArmResult');
  const btn = event.target;
  btn.disabled = true;
  resultDiv.textContent = '이동 중...';
  try {
    const res = await fetch('/reset_arm', { method: 'POST' });
    const data = await res.json();
    resultDiv.style.color = data.ok ? '#2e7d32' : '#c0392b';
    resultDiv.textContent = data.message || (data.ok ? '완료' : '실패');
  } catch (e) {
    resultDiv.style.color = '#c0392b';
    resultDiv.textContent = '요청이 실패했습니다: ' + e;
  } finally {
    btn.disabled = false;
    refreshStatus();
  }
}

async function moveBlind() {
  const resultDiv = document.getElementById('moveBlindResult');
  const fromSlot = document.getElementById('blindFromSlot').value;
  const toSlot = document.getElementById('blindToSlot').value;
  if (fromSlot === toSlot) {
    resultDiv.style.color = '#c0392b';
    resultDiv.textContent = '출발 자리와 도착 자리가 같습니다.';
    return;
  }
  const btn = event.target;
  btn.disabled = true;
  btn.textContent = '이동 중...';
  resultDiv.textContent = '';
  try {
    const res = await fetch('/move_blind', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({from_slot: parseInt(fromSlot), to_slot: parseInt(toSlot)}),
    });
    const data = await res.json();
    resultDiv.style.color = data.ok ? '#2e7d32' : '#c0392b';
    resultDiv.textContent = data.message || (data.ok ? '완료' : '실패');
  } catch (e) {
    resultDiv.style.color = '#c0392b';
    resultDiv.textContent = '요청이 실패했습니다: ' + e;
  } finally {
    btn.disabled = false;
    btn.textContent = '카메라 없이 이동';
    refreshStatus();
  }
}

async function resetBoxes() {
  const resultDiv = document.getElementById('resetResult');
  const btn = event.target;
  btn.disabled = true;
  resultDiv.textContent = '';
  try {
    const res = await fetch('/reset_boxes', { method: 'POST' });
    const data = await res.json();
    resultDiv.style.color = data.ok ? '#2e7d32' : '#c0392b';
    resultDiv.textContent = data.message || (data.ok ? '완료' : '실패');
  } catch (e) {
    resultDiv.style.color = '#c0392b';
    resultDiv.textContent = '요청이 실패했습니다: ' + e;
  } finally {
    btn.disabled = false;
    refreshStatus();
  }
}

// 천장 카메라 클릭으로 이동: 첫 클릭 = 박스(마커) 자체를 직접 클릭해서 선택, 두번째 클릭 = 도착 자리.
// 첫 클릭은 미리 가르친 자리 위치가 아니라, 지금 화면에 실제로 보이는 마커 중심과 비교해서 고름
// (자리에서 살짝 벗어나 있어도 눈에 보이는 그대로 클릭하면 됨).
async function onOverheadClick(e) {
  const img = e.target;
  const rect = img.getBoundingClientRect();
  const scaleX = img.naturalWidth / rect.width;
  const scaleY = img.naturalHeight / rect.height;
  const px = (e.clientX - rect.left) * scaleX;
  const py = (e.clientY - rect.top) * scaleY;
  const clickStatus = document.getElementById('clickStatus');

  if (pendingMarkerId === null) {
    const res = await fetch('/pixel_to_marker', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({x: px, y: py}),
    });
    const data = await res.json();
    if (data.marker_id === null) {
      clickStatus.style.color = '#c0392b';
      clickStatus.textContent = '박스(아루코 마커)를 정확히 클릭해주세요.';
      return;
    }
    pendingMarkerId = data.marker_id;
    clickStatus.style.color = '#333';
    clickStatus.textContent = `선택: 마커 id=${data.marker_id} → 도착 자리를 클릭하세요 (취소하려면 새로고침)`;
    return;
  }

  const slotRes = await fetch('/pixel_to_slot', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({x: px, y: py}),
  });
  const slotData = await slotRes.json();
  if (slotData.slot === null) {
    clickStatus.style.color = '#c0392b';
    clickStatus.textContent = '도착 자리 표시(십자가) 근처를 클릭해주세요.';
    return;
  }

  const markerId = pendingMarkerId;
  const toSlot = slotData.slot;
  pendingMarkerId = null;
  pendingFromSlot = null;
  pendingMarkerId = null;
  clickStatus.style.color = '#333';
  clickStatus.textContent = `마커 id=${markerId} → ${toSlot}번 자리로 이동 중...`;
  try {
    const moveRes = await fetch('/move', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({marker_id: markerId, to_slot: toSlot}),
    });
    const moveData = await moveRes.json();
    clickStatus.style.color = moveData.ok ? '#2e7d32' : '#c0392b';
    clickStatus.textContent = moveData.message || (moveData.ok ? '완료' : '실패');
  } catch (err) {
    clickStatus.style.color = '#c0392b';
    clickStatus.textContent = '요청이 실패했습니다: ' + err;
  } finally {
    refreshStatus();
  }
}
document.getElementById('overheadImg').addEventListener('click', onOverheadClick);

refreshStatus();
refreshBackendStatus();
setInterval(refreshStatus, 1000);
setInterval(refreshBackendStatus, 3000);
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
            "known_marker_ids": sorted(_node.known_marker_ids),
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


@app.route("/pixel_to_marker", methods=["POST"])
def pixel_to_marker():
    # 카메라 화면 클릭 좌표 -> 지금 그 근처에 실제로 보이는 마커 id(없으면 None).
    # 미리 가르친 자리 위치가 아니라, 이번 프레임에 실제 검출된 마커 중심과 비교함.
    px = float(request.json["x"])
    py = float(request.json["y"])
    marker_id = _node.nearest_marker_for_pixel(px, py)
    return jsonify({"marker_id": marker_id})


@app.route("/pixel_to_slot", methods=["POST"])
def pixel_to_slot():
    # 카메라 화면 클릭 좌표(px, py) -> 그 근처에 그려진 자리 번호(없으면 None).
    # 로봇을 움직이지 않고 self.slots만 읽는 조회라, 로봇 담당 스레드 큐를 거칠 필요 없음.
    px = float(request.json["x"])
    py = float(request.json["y"])
    slot = _node.nearest_slot_for_pixel(px, py)
    return jsonify({"slot": slot})


@app.route("/backend_status")
def backend_status():
    return jsonify({"mode": _backend_mode, "switching": _backend_lock.locked()})


@app.route("/switch_mode", methods=["POST"])
def switch_mode():
    mode = request.json["mode"]
    try:
        switch_backend(mode)
        _run_on_robot_thread(_node.set_gripper_reversed, mode == "real")
        label = "Gazebo(시뮬레이션)" if mode == "sim" else "실물 로봇"
        return jsonify({"ok": True, "mode": mode, "message": f"{label} 모드로 전환했습니다."})
    except Exception as e:  # noqa: BLE001 - 웹 쪽으로 에러 메시지를 그대로 전달하기 위함
        return jsonify({"ok": False, "message": str(e)}), 400


@app.route("/reset_arm", methods=["POST"])
def reset_arm():
    ok = _run_on_robot_thread(_node.reset_arm_to_default)
    if ok:
        return jsonify({"ok": True, "message": "매니퓰레이터를 디폴트 자세로 되돌렸습니다."})
    return jsonify({"ok": False, "message": "디폴트 자세로 이동하지 못했습니다."}), 400


@app.route("/gripper_level", methods=["POST"])
def gripper_level():
    level = int(request.json["level"])
    _run_on_robot_thread(_node.set_gripper_close_level, level)
    return jsonify({"ok": True, "level": level})


@app.route("/reset_boxes", methods=["POST"])
def reset_boxes():
    # 로봇을 안 움직이고 Gazebo 월드에만 직접 요청하는 거라 로봇 담당 스레드 큐를 안 거침.
    try:
        reset_boxes_to_default()
        return jsonify({"ok": True, "message": "박스 2개를 디폴트 위치로 되돌렸습니다."})
    except Exception as e:  # noqa: BLE001 - 웹 쪽으로 에러 메시지를 그대로 전달하기 위함
        return jsonify({"ok": False, "message": str(e)}), 400


@app.route("/move", methods=["POST"])
def move():
    marker_id = int(request.json["marker_id"])
    to_slot = int(request.json["to_slot"])
    try:
        # run_command()는 (그리퍼 열기 -> 출발 자리로 안전 이동(3단계) -> 집기 ->
        # 도착 자리로 안전 이동(3단계) -> 놓기)까지 전부 끝나야 반환됨. 실제로 겪은 문제:
        # 기본 60초로는 부족해서(실측 약 35~40초는 정상 범위 안이었지만 여유가 거의 없었음)
        # 로봇은 정상적으로 끝냈는데도 웹 쪽만 먼저 타임아웃으로 실패 처리되는 경우가 있었음.
        message = _run_on_robot_thread(_node.run_command, marker_id, to_slot, timeout_sec=180.0)
    except (RuntimeError, TimeoutError) as e:
        return jsonify({"ok": False, "message": str(e)}), 400
    return jsonify({"ok": True, "message": message})


@app.route("/move_blind", methods=["POST"])
def move_blind():
    # 카메라 없이(실물 로봇) 쓰는 이동: "그 자리에 박스가 있다"고 사용자가 직접 보장해야 함.
    from_slot = int(request.json["from_slot"])
    to_slot = int(request.json["to_slot"])
    try:
        message = _run_on_robot_thread(_node.run_command_blind, from_slot, to_slot, timeout_sec=180.0)
    except (RuntimeError, TimeoutError) as e:
        return jsonify({"ok": False, "message": str(e)}), 400
    return jsonify({"ok": True, "message": message})


def main(args=None):
    robot_thread = threading.Thread(target=_robot_thread_main, daemon=True)
    robot_thread.start()

    # _node가 만들어질 때까지 잠깐 대기 (로봇 담당 스레드가 rclpy.init() + 노드 생성을 마칠 시간)
    while _node is None:
        time.sleep(0.05)

    # 시작 직후 joint1=0(1번 자리와 겹침)에 그대로 머물지 않도록 한 번 비켜줌.
    _run_on_robot_thread(_node.park_away_from_home)

    print("웹 페이지: http://localhost:8080 (같은 네트워크의 다른 기기에서는 http://<이 컴퓨터의 IP>:8080)")
    # threaded=True 필수: 카메라 스트리밍(/video/...)은 연결을 계속 열어두고 있어서,
    # 스레드 하나짜리 서버면 스트리밍 보는 동안 다른 버튼(jog/move/status)이 전부 안 먹힘.
    app.run(host="0.0.0.0", port=8080, threaded=True)


if __name__ == "__main__":
    main()

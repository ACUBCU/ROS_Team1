import os
import subprocess
import time
from ament_index_python.packages import get_package_share_directory

def main():
    # Gazebo 월드가 완전히 로드되고 스폰 서비스가 열릴 때까지 8초간 대기합니다.
    # 컴퓨터 사양에 따라 시간이 더 필요할 경우 이 값을 10~15로 늘려주십시오.
    print("Gazebo 로딩 대기 중... (8초)")
    time.sleep(8)

    final_project_path = get_package_share_directory('final_project')
    base_cube_sdf_path = os.path.join(final_project_path, 'models', 'aruco_cube', 'model.sdf')
    
    with open(base_cube_sdf_path, 'r') as f:
        base_sdf_string = f.read()

    # 스폰할 마커 ID 및 위치 지정 (단위: 미터)
    markers = [
        {'id': 0, 'x': '0.1', 'y': '-0.3'},
        {'id': 1, 'x': '0.2', 'y': '-0.2'},
        {'id': 2, 'x': '0.3', 'y': '-0.1'},
        {'id': 3, 'x': '0.3', 'y': '0.1'},
        {'id': 4, 'x': '0.2', 'y': '0.2'},
        {'id': 5, 'x': '0.1', 'y': '0.3'}
    ]

    for marker in markers:
        m_id = marker['id']
        
        modified_sdf = base_sdf_string.replace(
            "<model name='aruco_cube'>", f"<model name='aruco_cube_{m_id}'>"
        ).replace(
            "aruco_0.png", f"aruco_{m_id}.png"
        )

        cmd = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-string', modified_sdf,
            '-name', f'aruco_cube_{m_id}',
            '-x', marker['x'],
            '-y', marker['y'],
            '-z', '0.025'
        ]
        
        print(f"아루코 큐브 {m_id} 생성 시도 중...")
        # 터미널 출력을 캡처하여 성공 여부를 확인합니다.
        result = subprocess.run(cmd, capture_output=True, text=True)
        
        if result.returncode != 0:
            print(f"[오류] 큐브 {m_id} 생성 실패:\n{result.stderr}")
        else:
            print(f"[성공] 큐브 {m_id} 생성 완료.")

if __name__ == '__main__':
    main()
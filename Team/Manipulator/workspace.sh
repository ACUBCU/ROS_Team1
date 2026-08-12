#!/usr/bin/env bash

set -Eeuo pipefail

PACKAGE_NAME="aruco_arm_sorter"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
PACKAGE_DIR="${SCRIPT_DIR}/${PACKAGE_NAME}"
WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd -P)"
EXPECTED_PREFIX="${WORKSPACE_ROOT}/install/${PACKAGE_NAME}"
ROS_DISTRO_NAME="${ARUCO_ROS_DISTRO:-jazzy}"
ROS_SETUP="${ARUCO_ROS_SETUP:-/opt/ros/${ROS_DISTRO_NAME}/setup.bash}"

export ARUCO_ARM_SORTER_WORKSPACE_ROOT="${WORKSPACE_ROOT}"
export ARUCO_ARM_SORTER_EXPECTED_PREFIX="${EXPECTED_PREFIX}"


die() {
  printf '오류: %s\n' "$*" >&2
  exit 1
}


source_checked() {
  local setup_file="$1"
  [[ -f "${setup_file}" ]] || die "setup 파일을 찾을 수 없습니다: ${setup_file}"

  # ROS/ament setup files intentionally probe optional variables such as
  # AMENT_TRACE_SETUP_FILES without always using a default value.  They must
  # therefore be sourced with Bash nounset temporarily disabled, even though
  # this helper keeps nounset enabled for its own code.
  local nounset_was_enabled=0
  local source_status=0
  case "$-" in
    *u*)
      nounset_was_enabled=1
      set +u
      ;;
  esac

  # shellcheck disable=SC1090
  source "${setup_file}" || source_status=$?

  if (( nounset_was_enabled )); then
    set -u
  fi
  (( source_status == 0 )) || die "setup 파일을 불러오지 못했습니다: ${setup_file}"
}


have_open_manipulator() {
  ros2 pkg prefix open_manipulator_description >/dev/null 2>&1 &&
    ros2 pkg prefix open_manipulator_bringup >/dev/null 2>&1
}


setup_contains_conflicting_package() {
  local setup_file="$1"
  local install_root
  [[ -f "${setup_file}" ]] || return 1
  install_root="$(cd -- "$(dirname -- "${setup_file}")" && pwd -P)"

  [[ -f "${install_root}/share/ament_index/resource_index/packages/${PACKAGE_NAME}" ]] ||
    [[ -f "${install_root}/${PACKAGE_NAME}/share/ament_index/resource_index/packages/${PACKAGE_NAME}" ]]
}


load_dependency_overlays() {
  source_checked "${ROS_SETUP}"

  if [[ -n "${ARUCO_DEPENDENCY_SETUP:-}" ]]; then
    local setup_file
    local -a setup_files=()
    IFS=':' read -r -a setup_files <<< "${ARUCO_DEPENDENCY_SETUP}"
    for setup_file in "${setup_files[@]}"; do
      [[ -n "${setup_file}" ]] || continue
      if setup_contains_conflicting_package "${setup_file}"; then
        die "의존성 workspace에 구버전 ${PACKAGE_NAME}가 함께 있습니다: ${setup_file}. 공식 OpenMANIPULATOR 전용 workspace를 지정하세요."
      fi
      source_checked "${setup_file}"
    done
  fi

  if have_open_manipulator; then
    return
  fi

  local candidate
  while IFS= read -r candidate; do
    [[ "${candidate}" == "${WORKSPACE_ROOT}/install/setup.bash" ]] && continue
    if setup_contains_conflicting_package "${candidate}"; then
      printf '구버전 %s 포함 workspace 건너뜀: %s\n' "${PACKAGE_NAME}" "${candidate}"
      continue
    fi
    source_checked "${candidate}"
    if have_open_manipulator; then
      printf '공식 OpenMANIPULATOR 의존성 사용: %s\n' "${candidate}"
      return
    fi
  done < <(
    find "${HOME}" -maxdepth 7 \
      \( -path "${HOME}/.cache" -o -path "${HOME}/.local" \) -prune -o \
      -type f -path '*/install/setup.bash' -print 2>/dev/null | sort
  )

  die "open_manipulator_description/bringup을 찾지 못했습니다. ARUCO_DEPENDENCY_SETUP=/의존성/워크스페이스/install/setup.bash 를 지정하세요."
}


assert_layout() {
  [[ -f "${PACKAGE_DIR}/package.xml" ]] ||
    die "패키지가 저장소의 Team/Manipulator/aruco_arm_sorter 위치에 없습니다: ${PACKAGE_DIR}"
  [[ "${WORKSPACE_ROOT}" != "/" && "${WORKSPACE_ROOT}" != "${HOME}" ]] ||
    die "안전하지 않은 워크스페이스 경로입니다: ${WORKSPACE_ROOT}"
}


source_project_install() {
  local setup_file="${WORKSPACE_ROOT}/install/setup.bash"
  [[ -f "${setup_file}" ]] ||
    die "프로젝트 설치본이 없습니다. 먼저 '$0 build'를 실행하세요."
  source_checked "${setup_file}"
}


active_prefix() {
  ros2 pkg prefix "${PACKAGE_NAME}" 2>/dev/null || true
}


verify_overlay() {
  local selected
  selected="$(active_prefix)"
  [[ -n "${selected}" ]] || die "ROS가 ${PACKAGE_NAME} 패키지를 찾지 못했습니다."

  if [[ "$(realpath -m -- "${selected}")" != "$(realpath -m -- "${EXPECTED_PREFIX}")" ]]; then
    die "구버전 또는 다른 작업공간 패키지가 선택됐습니다. 현재=${selected}, 기대=${EXPECTED_PREFIX}"
  fi

  local executables
  executables="$(ros2 pkg executables "${PACKAGE_NAME}")"
  local executable
  for executable in arm_sequence_controller aruco_detector preflight; do
    grep -Eq "^${PACKAGE_NAME}[[:space:]]+${executable}$" <<< "${executables}" ||
      die "현재 설치본에 실행 파일이 없습니다: ${executable}"
  done

  local identity="${selected}/share/${PACKAGE_NAME}/config/project.yaml"
  [[ -f "${identity}" ]] || die "프로젝트 식별 파일이 없는 구버전 설치본입니다: ${identity}"
  grep -Fq 'package_path: Team/Manipulator/aruco_arm_sorter' "${identity}" ||
    die "프로젝트 식별 파일의 패키지 경로가 올바르지 않습니다: ${identity}"

  printf '정상 설치본: %s\n' "${selected}"
  printf '등록 실행 파일: arm_sequence_controller, aruco_detector, preflight\n'
}


build_package() {
  assert_layout
  load_dependency_overlays
  command -v colcon >/dev/null 2>&1 || die "colcon 명령을 찾지 못했습니다."

  python3 "${PACKAGE_DIR}/tools/static_validate.py"

  cd -- "${WORKSPACE_ROOT}"
  printf '빌드할 소스: %s\n' "${PACKAGE_DIR}"
  printf '설치할 위치: %s\n' "${EXPECTED_PREFIX}"

  # 이 패키지의 자동 생성 결과만 제거합니다. 다른 패키지와 소스는 건드리지 않습니다.
  rm -rf -- \
    "${WORKSPACE_ROOT}/build/${PACKAGE_NAME}" \
    "${WORKSPACE_ROOT}/install/${PACKAGE_NAME}"

  local -a override_args=()
  if colcon build --help 2>&1 | grep -q -- '--allow-overriding'; then
    override_args=(--allow-overriding "${PACKAGE_NAME}")
  fi

  colcon build \
    --symlink-install \
    --base-paths "${PACKAGE_DIR}" \
    --packages-select "${PACKAGE_NAME}" \
    "${override_args[@]}"

  source_project_install
  verify_overlay
  printf "\n빌드 완료. 다음 명령으로 전체 점검을 실행하세요:\n%s doctor\n" "$0"
}


doctor() {
  assert_layout
  load_dependency_overlays
  source_project_install
  verify_overlay
  ARUCO_ARM_SORTER_WORKSPACE_ROOT="${WORKSPACE_ROOT}" \
    ARUCO_ARM_SORTER_EXPECTED_PREFIX="${EXPECTED_PREFIX}" \
    ros2 run "${PACKAGE_NAME}" preflight
}


run_project() {
  doctor
  printf '\n사전 점검 통과. Gazebo와 자동 ArUco 분류를 시작합니다.\n'
  exec ros2 launch "${PACKAGE_NAME}" aruco_arm_sorter.launch.py
}


show_status() {
  assert_layout
  load_dependency_overlays
  source_project_install
  verify_overlay
  printf '워크스페이스 루트: %s\n' "${WORKSPACE_ROOT}"
  printf '패키지 소스: %s\n' "${PACKAGE_DIR}"
  printf 'ROS 배포판: %s\n' "${ROS_DISTRO:-${ROS_DISTRO_NAME}}"
}


usage() {
  cat <<EOF
사용법: $0 <명령>

  build    저장소의 Team/Manipulator/aruco_arm_sorter만 깨끗하게 다시 빌드
  doctor   활성 설치본, 실행 파일, ROS/Gazebo/카메라 의존성 점검
  run      doctor 통과 후 Gazebo와 자동 ArUco 분류 실행
  status   현재 선택된 설치본과 저장소 경로 표시

선택 환경 변수:
  ARUCO_DEPENDENCY_SETUP  공식 OpenMANIPULATOR가 있는 install/setup.bash 경로
                          여러 개면 콜론(:)으로 구분
  ARUCO_ROS_DISTRO        기본값 jazzy
  ARUCO_ROS_SETUP         기본값 /opt/ros/<배포판>/setup.bash
EOF
}


case "${1:-help}" in
  build)
    build_package
    ;;
  doctor)
    doctor
    ;;
  run)
    run_project
    ;;
  status)
    show_status
    ;;
  help|-h|--help)
    usage
    ;;
  *)
    usage >&2
    die "알 수 없는 명령: $1"
    ;;
esac
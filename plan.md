# Vsp_to_qsp 작업 정리 (2026-02-24)

## 최종 목표
- `build.sh`를 수정하지 않고 `./scripts/run.sh ros2 build ros2_ws`에서 `vsp_to_qsp` 빌드 실패를 해결.

## 최종 수행 내용
1. `CMakeLists.txt` 수정
- `px4_msgs`를 기본 경로에서 찾지 못할 때 fallback 탐색 로직 추가.
- 탐색 우선순위:
  - `VSP_TO_QSP_PX4_MSGS_PREFIX` (CMake cache 변수)
  - `PX4_MSGS_PREFIX` (환경변수)
  - `ros2_ws` 기준 상대 경로(`../px4_ros/install`)
  - `$HOME/Documents/A4VAI-SITL/ROS2/px4_ros/install`
- `merged`/`isolated` install 레이아웃 모두 지원.
- 미탐지 시 검사한 경로와 권장 조치 커맨드를 포함해 즉시 실패하도록 메시지 강화.

2. `package.xml` 수정
- `px4_msgs` 의존성을 `build_depend`, `build_export_depend`, `exec_depend`로 명시 분리.

3. `src/velocity_to_attitude_node.cpp` 수정
- ROS2 Humble 컴파일 오류 해결을 위해 `nowMicros()`의 `const` 제거.

## 검증
- 실행 명령:
  - `./scripts/run.sh ros2 build ros2_ws`
- 결과:
  - `Finished <<< vsp_to_qsp`
  - 산출물 확인: `install/vsp_to_qsp/lib/vsp_to_qsp/velocity_to_attitude_node`

## 최종 상태
- `build.sh` 변경 없이 `vsp_to_qsp` 빌드 성공.
- 동일 유형의 `px4_msgs` 미탐지 발생 시 원인 파악 가능한 오류 메시지 제공.

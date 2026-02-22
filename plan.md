# Velocity setpoint -> Attitude setpoint 구현 계획

## 1) 목표
- `px4_msgs/msg/TrajectorySetpoint`의 **velocity setpoint**를 입력으로 받아,
- PX4 `mc_pos_control` 알고리즘(설계 문서 기준)을 ROS2 노드로 재현하여
- `px4_msgs/msg/VehicleAttitudeSetpoint`를 출력한다.

## 2) 구현 범위
- 포함:
  - Velocity PID -> Acceleration setpoint
  - Acceleration -> Thrust vector
  - Thrust saturation(수직 우선, XY margin 포함)
  - Anti-windup(수직/수평 ARW, z 적분 제한)
  - Thrust + yaw -> quaternion 변환
  - `VehicleAttitudeSetpoint` publish
- 제외(초기 버전):
  - Hover thrust estimator 연동
  - VTOL 전용 `mc_virtual_attitude_setpoint` 경로
  - 고급 fail-safe(입력 타임아웃 외 추가 상태기계)

## 3) 입출력/토픽 계획
- Subscribe:
  - `TrajectorySetpoint` (velocity 사용, pos/acc는 NAN 허용)
  - `VehicleLocalPosition` (현재 속도/가속도/heading)
  - `VehicleLandDetected` (landed 시 적분 초기화)
- Publish:
  - `VehicleAttitudeSetpoint`
  - (옵션) `OffboardControlMode` (`attitude=true`) - 파라미터로 on/off

## 4) 코드 구조 계획
- 새 ROS2 노드 생성: `offboard_velocity_to_attitude_node` (C++, `rclcpp`)
- 내부 모듈(클래스/함수):
  - `VelocityPid3D`
  - `AccelerationToThrust`
  - `ThrustSaturation`
  - `ThrustToAttitude`
  - `MathUtil` (`constrainXY`, `limitTilt`, `nan 처리`, clamp)
- 파라미터:
  - 설계 문서 2.3 항목 기본값을 그대로 선언
  - 토픽 이름 및 타이머 주기(기본 50Hz) 파라미터화

## 5) 알고리즘 적용 순서(구현 고정 순서)
1. 입력 유효성/NAN 처리 및 `dt` 계산(clamp: 0.000125~0.02s)
2. velocity setpoint 제한(XY norm, Z up/down clamp)
3. velocity PID로 `acc_sp` 계산(D항은 측정 가속도 기반)
4. `acc_sp -> thr_sp` 변환 + tilt 제한
5. thrust saturation(수직 우선 -> 수평 배분)
6. anti-windup 반영 후 적분 업데이트
7. `thr_sp + yaw_sp -> q_sp` 변환
8. `VehicleAttitudeSetpoint` publish

## 6) 구현 단계
1. 패키지/빌드 스캐폴드 생성 및 메시지 의존성 설정(`px4_msgs`, `Eigen3`)
2. 수학 유틸 및 변환 함수 단위 구현
3. PID + 포화 + anti-windup 로직 구현
4. ROS2 노드(pub/sub/timer/params) 결합
5. 런치/파라미터 파일 및 사용법 문서화
6. 기본 검증(단위 + 시뮬레이션 입력 시나리오)

## 7) 검증 계획
- 단위 테스트:
  - `limitTilt`, `constrainXY`, quaternion 변환 정합성
  - 포화 상태에서 anti-windup이 적분 과증가를 막는지 확인
- 시나리오 테스트:
  - 정지 호버(`vel_sp=0`) -> 수평 자세/hover thrust 수렴
  - 전진 속도 명령 -> 기대 pitch/roll 및 yaw 유지
  - 급상승/급하강 명령 -> z thrust 우선 포화 확인
  - `yaw=NaN` 입력 -> 현재 heading 유지

## 8) 완료 기준(승인 기준)
- `TrajectorySetpoint(vel)` 입력만으로 `VehicleAttitudeSetpoint`가 안정적으로 출력됨
- 설계 문서의 핵심 수식/순서/제약(tilt, thrust limit, anti-windup) 반영
- 빌드 및 테스트(가능한 범위) 통과
- 파라미터/토픽/실행 방법 문서 제공

## 9) 승인 요청
- 위 계획 승인 후, 바로 구현 단계(코드 생성 + 테스트)로 진행한다.

# ROS2 Offboard Velocity → Attitude 변환 설계 문서

Offboard 모드에서 외부 ROS2 노드가 보내는 velocity command(V_sp)를 attitude quaternion(q_sp)으로 변환하는 제어기를 ROS2로 구현하기 위한 설계 문서입니다.

PX4 소스코드(`mc_pos_control` 모듈)의 실제 알고리즘을 기반으로 합니다.

---

## 1. 시스템 개요

### 1.1 PX4에서의 Offboard Velocity 경로

PX4에서 offboard velocity command는 다음 경로를 따릅니다:

```
ROS2 Node
  │
  ├─ /fmu/in/offboard_control_mode   (velocity=true)
  ├─ /fmu/in/trajectory_setpoint     (pos=NAN, vel={vx,vy,vz}, acc=NAN)
  │
  v
uORB (XRCE-DDS bridge)
  │
  v
mc_pos_control
  ├─ _positionControl()  → 건너뜀 (pos_sp = NAN)
  ├─ _velocityControl()  → PID로 A_sp 생성
  ├─ _accelerationControl() → thrust 벡터 계산
  └─ thrustToAttitude()  → q_sp 출력
  │
  v
vehicle_attitude_setpoint → mc_att_control → mc_rate_control → mixer
```

### 1.2 ROS2 구현 시 대체 범위

이 설계 문서에서 구현할 부분은 **mc_pos_control 내부의 velocity → attitude 변환 전체**입니다:

```
┌──────────────────────────────────────────────────────────┐
│  ROS2 Node (구현 대상)                                    │
│                                                          │
│  V_sp (입력) ──► Velocity PID ──► A_sp                   │
│  ψ_sp (입력) ─────────────────────┐                      │
│                                    ▼                     │
│                    Acceleration → Thrust Vector           │
│                                    │                     │
│                                    ▼                     │
│                    Thrust + Yaw → q_sp (출력)            │
└──────────────────────────────────────────────────────────┘
```

---

## 2. 인터페이스 설계

### 2.1 입력 (Subscribe)

| ROS2 Topic                        | 메시지 타입                              | 설명                             | 주기      |
| --------------------------------- | ----------------------------------- | ------------------------------ | ------- |
| `velocity_setpoint`               | `geometry_msgs/msg/Twist` 또는 커스텀    | V_sp (NED 또는 body frame 속도 명령) | 외부에서 발행 |
| `/fmu/out/vehicle_local_position` | `px4_msgs/msg/VehicleLocalPosition` | 현재 위치, 속도, 속도 미분값, yaw         | ~50 Hz  |
| `/fmu/out/vehicle_land_detected`  | `px4_msgs/msg/VehicleLandDetected`  | 착지 상태 (적분 초기화용)                | 이벤트     |

### 2.2 출력 (Publish)

| ROS2 Topic                          | 메시지 타입                                 | 설명                | 주기     |
| ----------------------------------- | -------------------------------------- | ----------------- | ------ |
| `/fmu/in/offboard_control_mode`     | `px4_msgs/msg/OffboardControlMode`     | attitude=true로 설정 | ~50 Hz |
| `/fmu/in/vehicle_attitude_setpoint` | `px4_msgs/msg/VehicleAttitudeSetpoint` | q_sp + thrust     | ~50 Hz |

> **주의**: velocity offboard를 PX4에 직접 보내는 것이 아니라, 이 노드가 velocity→attitude 변환을 수행하고 **attitude 수준의 명령**을 PX4에 보내는 구조입니다.

### 2.3 파라미터

| 파라미터 | PX4 대응 | 기본값 | 설명 |
|---|---|---|---|
| `kp_vel_xy` | `MPC_XY_VEL_P_ACC` | 1.8 | 수평 속도 P 게인 |
| `ki_vel_xy` | `MPC_XY_VEL_I_ACC` | 0.4 | 수평 속도 I 게인 |
| `kd_vel_xy` | `MPC_XY_VEL_D_ACC` | 0.2 | 수평 속도 D 게인 |
| `kp_vel_z` | `MPC_Z_VEL_P_ACC` | 4.0 | 수직 속도 P 게인 |
| `ki_vel_z` | `MPC_Z_VEL_I_ACC` | 2.0 | 수직 속도 I 게인 |
| `kd_vel_z` | `MPC_Z_VEL_D_ACC` | 0.0 | 수직 속도 D 게인 |
| `hover_thrust` | `MPC_THR_HOVER` | 0.5 | 호버링 시 정규화 추력 [0,1] |
| `thr_min` | `MPC_THR_MIN` | 0.12 | 최소 추력 |
| `thr_max` | `MPC_THR_MAX` | 1.0 | 최대 추력 |
| `tilt_max_rad` | `MPC_TILTMAX_AIR` | 0.7854 (45°) | 최대 기울기 [rad] |
| `thr_xy_margin` | `MPC_THR_XY_MARG` | 0.3 | 수평 추력 마진 |
| `lim_vel_horizontal` | `MPC_XY_VEL_MAX` | 12.0 | 수평 속도 제한 [m/s] |
| `lim_vel_up` | `MPC_Z_VEL_MAX_UP` | 3.0 | 상승 속도 제한 [m/s] |
| `lim_vel_down` | `MPC_Z_VEL_MAX_DN` | 1.5 | 하강 속도 제한 [m/s] |

---

## 3. 알고리즘 상세

### 3.1 좌표계 규약

- **NED (North-East-Down)**: PX4 내부 좌표계. z축이 아래쪽이 양수.
- 중력 가속도: `g = 9.80665 m/s²`, NED에서 `(0, 0, +g)` 방향
- 추력(thrust): NED에서 음수 z방향 (위쪽). 호버링 시 `_thr_sp(2) ≈ -hover_thrust`
- 속도: NED 기준. 상승 = `vz < 0`, 하강 = `vz > 0`

### 3.2 Velocity PID → Acceleration Setpoint

**PX4 원본:** `PositionControl::_velocityControl(dt)`

#### 수식

```
e(t) = V_sp - V_actual                     (속도 오차)

A_sp = Kp ⊙ e(t) + I_term - Kd ⊙ V_dot   (PID 출력 = 원하는 가속도)

I_term += Ki ⊙ e(t) × dt                   (적분 업데이트)
```

여기서 `⊙`는 원소별(element-wise) 곱셈, `V_dot`는 속도의 시간 미분(= 가속도 측정값)입니다.

#### 구현 세부사항

**D항 — 측정값 미분 사용:**
```
D_term = -Kd ⊙ V_dot    (오차의 미분이 아님)
```
오차의 미분(`d(V_sp - V)/dt`)을 사용하면 setpoint가 갑자기 변할 때 큰 스파이크(derivative kick)가 발생합니다. 대신 측정된 가속도(`V_dot`)를 직접 사용하여 이를 방지합니다. `V_dot`는 `vehicle_local_position.ax/ay/az`에서 얻습니다.

**수직 적분 제한:**
```
I_term(2) = clamp(I_term(2), -g, +g)
```
수직 적분항은 최대 ±1g로 제한합니다. 이보다 큰 적분값은 물리적으로 의미가 없습니다.

**속도 제한 (velocity PID 이전에 적용):**
```
V_sp(0:1) = constrainXY(V_sp_xy, lim_vel_horizontal)
V_sp(2)   = clamp(V_sp(2), -lim_vel_up, +lim_vel_down)
```

### 3.3 Acceleration → Thrust Vector

**PX4 원본:** `PositionControl::_accelerationControl()`

이 단계에서 가속도 setpoint를 **기체가 기울어야 할 방향(body_z)**과 **추력 크기**로 변환합니다.

#### 3.3.1 Body z축 방향 결정

```
body_z = normalize(-a_x, -a_y, +g)
```

물리적 의미: 멀티콥터의 총 힘 = 추력 + 중력 = m × a_sp

```
F_thrust = m × (a_sp - g_vec) = m × (a_x, a_y, a_z - g)
```

NED에서 g_vec = (0, 0, g)이므로:

```
F_thrust 방향 = (a_x, a_y, a_z - g)
body_z 방향   = -F_thrust 방향 = (-a_x, -a_y, g - a_z)
```

> **Decouple 모드 (기본 활성):** `a_z` 항을 무시하고 `z_specific_force = -g`로 고정합니다. 이렇게 하면 상승/하강 가속도 변화가 기울기(tilt)에 영향을 주지 않습니다.
> ```
> body_z = normalize(-a_x, -a_y, g)    // a_z 무시
> ```

#### 3.3.2 Tilt 제한

```
angle = acos(body_z · e_z_world)       // e_z_world = (0,0,1)
if angle > tilt_max:
    // body_z를 tilt_max 각도로 제한
    rejection = body_z - (body_z · e_z_world) × e_z_world
    body_z = cos(tilt_max) × e_z_world + sin(tilt_max) × normalize(rejection)
```

기체가 너무 많이 기울어지는 것을 방지합니다 (기본 45°).

#### 3.3.3 Thrust 크기 계산

```
T_ned_z = a_z × (T_hover / g) - T_hover
```

- 호버링 시(`a_z = 0`): `T_ned_z = -T_hover` (위쪽으로 hover thrust)
- 상승 시(`a_z < 0`, NED): 추력 증가
- 하강 시(`a_z > 0`): 추력 감소

기울어진 각도 보정:
```
cos_tilt = body_z · (0, 0, 1)
T_collective = T_ned_z / cos_tilt
T_collective = min(T_collective, -thr_min)    // 최소 추력 보장
```

기체가 기울면 수직 추력 성분이 `cos(tilt)`배로 줄어드므로, 총 추력을 `1/cos(tilt)`만큼 키워야 합니다.

최종 thrust 벡터:
```
thr_sp = body_z × T_collective
```

### 3.4 Thrust 포화 처리 (수직 우선)

**PX4 원본:** `_velocityControl()` 후반부 (line 160-196)

thrust 벡터가 최대 추력을 초과할 경우, **수직을 우선**하고 남은 여력을 수평에 배분합니다:

```
1) 수평 마진 확보: allocated_xy = min(|thr_sp_xy|, thr_xy_margin)
2) 수직 최대: thr_z_max = sqrt(thr_max² - allocated_xy²)
3) 수직 포화: thr_sp_z = max(thr_sp_z, -thr_z_max)
4) 수평 최대: thr_max_xy = sqrt(thr_max² - thr_sp_z²)
5) 수평 포화: if |thr_sp_xy| > thr_max_xy → thr_sp_xy 스케일링
```

이 순서로 처리하면:
- 수직 제어가 항상 우선되어 고도 유지가 보장됨
- 수평에 최소 마진(`thr_xy_margin`)을 남겨 수평 제어력을 완전히 잃지 않음

### 3.5 Anti-Windup

세 가지 anti-windup 메커니즘이 적용됩니다:

#### (a) 수직 integrator anti-windup
```
if (thr_sp_z >= -thr_min AND vel_error_z >= 0) → vel_error_z = 0
if (thr_sp_z <= -thr_max AND vel_error_z <= 0) → vel_error_z = 0
```
추력이 최소/최대에 도달하면 해당 방향의 적분을 중단합니다.

#### (b) 수평 tracking anti-windup (ARW)
```
acc_produced_xy = thr_sp_xy × (g / T_hover)
if |A_sp_xy|² > |acc_produced_xy|²:
    arw_gain = 2 / Kp_vel_xy
    vel_error_xy -= arw_gain × (A_sp_xy - acc_produced_xy)
```
포화로 인해 실제 생산된 가속도가 원하는 가속도보다 작을 때, 적분항의 오차 입력을 보정하여 적분 과적을 방지합니다. (참고: L. Rundqwist, "Anti-Reset Windup for PID Controllers", 1990)

#### (c) 수직 적분 범위 제한
```
I_term(2) = clamp(I_term(2), -g, +g)
```

### 3.6 Thrust Vector + Yaw → Quaternion (q_sp)

**PX4 원본:** `ControlMath::thrustToAttitude()` → `bodyzToAttitude()`

thrust 벡터로 body z축 방향은 결정되었으나, z축을 중심으로 한 회전(yaw)은 아직 미정입니다. ψ_sp(yaw setpoint)를 사용하여 완전한 3축 회전을 구성합니다.

#### 단계별 과정

**Step 1: body_z 결정**
```
body_z = normalize(-thr_sp)    // thrust 반대 방향 = 기체 윗면 방향
```

**Step 2: yaw로부터 중간 벡터 생성**
```
y_C = (-sin(ψ_sp), cos(ψ_sp), 0)    // yaw 방향에서 90° 회전한 수평 벡터
```

**Step 3: 외적으로 body frame 3축 구성**
```
body_x = y_C × body_z               // 기체 전방 (nose) 방향
if body_z_down < 0: body_x = -body_x   // 뒤집힌 경우 보정
body_x = normalize(body_x)

body_y = body_z × body_x            // 기체 좌측 방향
```

**Step 4: DCM → Quaternion**
```
R = [body_x | body_y | body_z]      // 3×3 회전 행렬 (Direction Cosine Matrix)
q_sp = Quaternion(R)                 // DCM을 quaternion으로 변환
```

**Step 5: Body thrust 크기**
```
thrust_body_z = -|thr_sp|           // body frame z축 추력 (음수 = 아래쪽)
```

#### Yaw 처리 규칙
- `ψ_sp`가 NAN이면: 현재 yaw를 유지 (`ψ_sp = ψ_current`)
- `yawspeed_sp`가 NAN이면: 0으로 처리

---

## 4. 노드 구조 설계

### 4.1 클래스 구조

```
OffboardVelocityController (ROS2 Node)
│
├── VelocityPID                    // 속도 PID 제어기 (3축)
│   ├── gains: Kp, Ki, Kd         // Vector3f
│   ├── integral: Vector3f         // 적분항
│   ├── integral_limit: float      // 적분 제한
│   └── update(vel_sp, vel, vel_dot, dt) → acc_sp
│
├── AccelerationToThrust           // 가속도 → 추력 벡터 변환
│   ├── hover_thrust: float
│   ├── tilt_max: float
│   ├── thr_min, thr_max: float
│   └── compute(acc_sp) → thr_sp
│
├── ThrustToAttitude               // 추력 + yaw → quaternion 변환
│   └── compute(thr_sp, yaw_sp) → (q_sp, thrust_body_z)
│
└── ThrustSaturation               // 추력 포화 처리
    ├── thr_max: float
    ├── thr_xy_margin: float
    └── saturate(thr_sp) → thr_sp_saturated
```

### 4.2 메인 루프 (타이머 콜백, ~50 Hz)

```
function on_timer():
    // 1. 입력 수집
    vel_sp    ← latest velocity command
    vel       ← vehicle_local_position.vx/vy/vz
    vel_dot   ← vehicle_local_position.ax/ay/az
    yaw       ← vehicle_local_position.heading
    dt        ← time since last iteration

    // 2. 속도 제한
    vel_sp.xy = constrain_norm(vel_sp.xy, lim_vel_horizontal)
    vel_sp.z  = clamp(vel_sp.z, -lim_vel_up, +lim_vel_down)

    // 3. Velocity PID → Acceleration
    vel_error = vel_sp - vel
    acc_sp = Kp ⊙ vel_error + I_term - Kd ⊙ vel_dot

    // 4. Acceleration → Thrust vector
    body_z = normalize(-acc_sp.x, -acc_sp.y, +g)
    limitTilt(body_z, tilt_max)
    T_z = acc_sp.z × (T_hover / g) - T_hover
    T_col = T_z / dot(body_z, e_z)
    thr_sp = body_z × T_col

    // 5. Thrust 포화 (수직 우선)
    saturate(thr_sp)

    // 6. Anti-windup + 적분 업데이트
    apply_antiwindup(thr_sp, vel_error)
    I_term += Ki ⊙ vel_error × dt

    // 7. Thrust + Yaw → Quaternion
    (q_sp, thrust_body_z) = thrustToAttitude(thr_sp, yaw_sp)

    // 8. 발행
    publish offboard_control_mode (attitude=true)
    publish vehicle_attitude_setpoint (q_sp, thrust_body_z)
```

### 4.3 실행 순서 주의사항

PX4 코드에서 `_velocityControl()` 내부에서 `_accelerationControl()`이 호출된 **후에** anti-windup과 thrust 포화가 처리됩니다. 이 순서가 중요합니다:

```
1. PID로 acc_sp 계산
2. acc_sp → thr_sp 변환 (_accelerationControl)
3. thr_sp로 수직 anti-windup 판단
4. thr_sp에 수직/수평 포화 적용
5. 포화 결과로 수평 ARW 적용
6. 최종 vel_error로 적분 업데이트
```

thrust 포화 결과가 적분 업데이트에 피드백되는 구조이므로, **반드시 이 순서를 지켜야** anti-windup이 올바르게 동작합니다.

---

## 5. 핵심 알고리즘 구현 명세

### 5.1 constrainXY (우선순위 기반 2D 벡터 제한)

```
function constrainXY(v0, v1, max):
    // v0: 우선순위 높은 벡터, v1: 낮은 벡터
    if |v0 + v1| <= max → return v0 + v1
    if |v0| >= max      → return normalize(v0) × max
    // 이차방정식으로 v1의 스케일링 계수 s를 구함
    u = normalize(v1)
    m = dot(u, v0)
    c = dot(v0, v0) - max²
    s = -m + sqrt(m² - c)
    return v0 + u × s
```

### 5.2 limitTilt

```
function limitTilt(body_z, max_angle):
    angle = acos(dot(body_z, (0,0,1)))
    if angle <= max_angle → return
    angle = max_angle
    rejection = body_z - dot(body_z, (0,0,1)) × (0,0,1)
    body_z = cos(angle) × (0,0,1) + sin(angle) × normalize(rejection)
```

### 5.3 bodyzToAttitude

```
function bodyzToAttitude(body_z, yaw_sp):
    body_z = normalize(body_z)
    y_C = (-sin(yaw_sp), cos(yaw_sp), 0)
    body_x = cross(y_C, body_z)
    if body_z.z < 0: body_x = -body_x
    body_x = normalize(body_x)
    body_y = cross(body_z, body_x)
    R = matrix([body_x, body_y, body_z])    // 열벡터로 구성
    q = quaternion_from_rotation_matrix(R)
    return q
```

---

## 6. PX4 원본 코드 참조

| 구현 블록 | PX4 파일 | 함수 | 라인 |
|---|---|---|---|
| Velocity PID | `PositionControl/PositionControl.cpp` | `_velocityControl(dt)` | 140-202 |
| Accel→Thrust | `PositionControl/PositionControl.cpp` | `_accelerationControl()` | 204-222 |
| Tilt 제한 | `PositionControl/ControlMath.cpp` | `limitTilt()` | 53-68 |
| Thrust→Attitude | `PositionControl/ControlMath.cpp` | `thrustToAttitude()` | 47-51 |
| Body z→Quaternion | `PositionControl/ControlMath.cpp` | `bodyzToAttitude()` | 70-114 |
| XY 제한 | `PositionControl/ControlMath.cpp` | `constrainXY()` | 116-178 |
| NAN 처리 | `PositionControl/ControlMath.cpp` | `addIfNotNanVector3f()` | 242-247 |
| 입력 검증 | `PositionControl/PositionControl.cpp` | `_inputValid()` | 224-250 |
| Attitude 발행 | `PositionControl/PositionControl.cpp` | `getAttitudeSetpoint()` | 266-270 |

---

## 7. 주의사항

### 7.1 NAN 규약
PX4에서는 NAN = "이 축은 제어하지 않음"을 의미합니다. ROS2 구현 시 이 규약을 유지하면 유연한 setpoint 조합이 가능합니다:
- velocity만 제어: `pos=NAN, vel={값}, acc=NAN`
- yaw 유지: `yaw_sp = NAN` → 현재 heading 유지
- yaw rate 없음: `yawspeed = NAN` → 0으로 처리

### 7.2 시간 동기화
`vehicle_local_position` 메시지의 타임스탬프를 사용하여 dt를 계산해야 합니다. PX4는 dt를 0.000125s ~ 0.02s (0.125ms ~ 20ms)로 클램핑합니다.

### 7.3 착지 상태
착지 상태(`vehicle_land_detected.landed`)에서는 적분항을 초기화하여 이륙 시 급작스러운 동작을 방지해야 합니다.

### 7.4 Hover Thrust 추정
실제 비행에서 `hover_thrust`는 고정값이 아닌 `mc_hover_thrust_estimator` 모듈의 추정값을 사용할 수 있습니다. 이 추정값이 변경될 때 적분항을 보정하는 `updateHoverThrust()` 로직이 PX4에 있습니다:
```
// 새 hover thrust 적용 시 적분항 보정
I_z += (a_z - g) × T_hover_old / T_hover_new + g - a_z
```

### 7.5 VTOL 모드
VTOL 기체의 경우 PX4는 `vehicle_attitude_setpoint` 대신 `mc_virtual_attitude_setpoint` 토픽을 사용합니다. 필요 시 구분해야 합니다.

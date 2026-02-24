#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{

constexpr float kGravity = 9.80665F;
constexpr float kEpsilon = 1e-5F;
constexpr float kMinDtSec = 0.000125F;
constexpr float kMaxDtSec = 0.02F;
constexpr float kMinCosTilt = 1e-3F;

float clampScalar(const float value, const float low, const float high)
{
  return std::max(low, std::min(value, high));
}

float safeSqrt(const float value)
{
  return std::sqrt(std::max(0.0F, value));
}

bool isFinite(const float value)
{
  return std::isfinite(value);
}

template<typename T, typename = void>
struct HasAccelerationFields : std::false_type
{
};

template<typename T, typename = void>
struct HasPositionFields : std::false_type
{
};

template<typename T>
struct HasAccelerationFields<
  T,
  std::void_t<
    decltype(std::declval<const T &>().ax),
    decltype(std::declval<const T &>().ay),
    decltype(std::declval<const T &>().az)>> : std::true_type
{
};

template<typename T>
struct HasPositionFields<
  T,
  std::void_t<
    decltype(std::declval<const T &>().x),
    decltype(std::declval<const T &>().y),
    decltype(std::declval<const T &>().z)>> : std::true_type
{
};

template<typename LocalPositionT>
Eigen::Vector3f extractMeasuredAcceleration(
  const LocalPositionT & local_position, const Eigen::Vector3f & fallback_acceleration)
{
  if constexpr (HasAccelerationFields<LocalPositionT>::value) {
    return Eigen::Vector3f(
      local_position.ax,
      local_position.ay,
      local_position.az);
  }
  return fallback_acceleration;
}

template<typename LocalPositionT>
Eigen::Vector3f extractMeasuredPosition(const LocalPositionT & local_position)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  if constexpr (HasPositionFields<LocalPositionT>::value) {
    return Eigen::Vector3f(
      isFinite(local_position.x) ? local_position.x : nan,
      isFinite(local_position.y) ? local_position.y : nan,
      isFinite(local_position.z) ? local_position.z : nan);
  }
  return Eigen::Vector3f(nan, nan, nan);
}

Eigen::Vector3f normalizeOrDefault(
  const Eigen::Vector3f & vec, const Eigen::Vector3f & default_vec)
{
  const float norm = vec.norm();
  if (norm <= kEpsilon) {
    return default_vec;
  }
  return vec / norm;
}

Eigen::Vector2f constrainXY(
  const Eigen::Vector2f & high_priority, const Eigen::Vector2f & low_priority, const float max_norm)
{
  if (max_norm <= kEpsilon) {
    return Eigen::Vector2f::Zero();
  }

  const Eigen::Vector2f combined = high_priority + low_priority;
  const float max_sq = max_norm * max_norm;
  if (combined.squaredNorm() <= max_sq) {
    return combined;
  }

  const float high_norm = high_priority.norm();
  if (high_norm >= max_norm) {
    if (high_norm <= kEpsilon) {
      return Eigen::Vector2f::Zero();
    }
    return high_priority * (max_norm / high_norm);
  }

  const float low_norm = low_priority.norm();
  if (low_norm <= kEpsilon) {
    return high_priority;
  }

  const Eigen::Vector2f unit_low = low_priority / low_norm;
  const float m = unit_low.dot(high_priority);
  const float c = high_priority.squaredNorm() - max_sq;
  const float discriminant = std::max(0.0F, m * m - c);
  const float scale = -m + std::sqrt(discriminant);
  return high_priority + unit_low * scale;
}

Eigen::Vector3f limitTilt(const Eigen::Vector3f & body_z, const float tilt_max_rad)
{
  const Eigen::Vector3f world_z(0.0F, 0.0F, 1.0F);
  Eigen::Vector3f normalized_body_z = normalizeOrDefault(body_z, world_z);

  const float dot_product = clampScalar(normalized_body_z.dot(world_z), -1.0F, 1.0F);
  const float angle = std::acos(dot_product);
  if (angle <= tilt_max_rad) {
    return normalized_body_z;
  }

  Eigen::Vector3f rejection = normalized_body_z - dot_product * world_z;
  rejection = normalizeOrDefault(rejection, Eigen::Vector3f(1.0F, 0.0F, 0.0F));
  normalized_body_z = std::cos(tilt_max_rad) * world_z + std::sin(tilt_max_rad) * rejection;
  return normalizeOrDefault(normalized_body_z, world_z);
}

struct AttitudeCommand
{
  Eigen::Quaternionf q_sp{Eigen::Quaternionf::Identity()};
  float thrust_body_z{0.0F};
};

struct VelocityCommand
{
  Eigen::Vector3f vel_sp{Eigen::Vector3f::Zero()};
  std::array<bool, 3> axis_enabled{{false, false, false}};
};

struct InputValidationResult
{
  bool valid{false};
  std::string reason{};
};

class VelocityPid3D
{
public:
  void setGains(
    const Eigen::Vector3f & kp, const Eigen::Vector3f & ki, const Eigen::Vector3f & kd)
  {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }

  void reset()
  {
    integral_.setZero();
  }

  Eigen::Vector3f computeAcceleration(
    const Eigen::Vector3f & vel_error, const Eigen::Vector3f & vel_dot) const
  {
    return kp_.cwiseProduct(vel_error) + integral_ - kd_.cwiseProduct(vel_dot);
  }

  void applyVerticalAntiWindup(
    const Eigen::Vector3f & thr_sp, const float thr_min, const float thr_max,
    Eigen::Vector3f * vel_error) const
  {
    if (vel_error == nullptr) {
      return;
    }

    if (thr_sp.z() >= -thr_min && vel_error->z() >= 0.0F) {
      vel_error->z() = 0.0F;
    }
    if (thr_sp.z() <= -thr_max && vel_error->z() <= 0.0F) {
      vel_error->z() = 0.0F;
    }
  }

  void applyHorizontalArw(
    const Eigen::Vector3f & acc_sp, const Eigen::Vector3f & thr_sp, const float hover_thrust,
    Eigen::Vector3f * vel_error) const
  {
    if (vel_error == nullptr || kp_.x() <= kEpsilon || hover_thrust <= kEpsilon) {
      return;
    }

    const Eigen::Vector2f acc_sp_xy(acc_sp.x(), acc_sp.y());
    const Eigen::Vector2f acc_produced_xy = Eigen::Vector2f(thr_sp.x(), thr_sp.y()) * (kGravity / hover_thrust);
    if (acc_sp_xy.squaredNorm() > acc_produced_xy.squaredNorm()) {
      const float arw_gain = 2.0F / kp_.x();
      const Eigen::Vector2f correction = arw_gain * (acc_sp_xy - acc_produced_xy);
      vel_error->x() -= correction.x();
      vel_error->y() -= correction.y();
    }
  }

  void integrate(const Eigen::Vector3f & vel_error, const float dt_sec)
  {
    integral_ += ki_.cwiseProduct(vel_error) * dt_sec;
    integral_.z() = clampScalar(integral_.z(), -kGravity, kGravity);
  }

  void clearIntegralForDisabledAxes(const std::array<bool, 3> & axis_enabled)
  {
    if (!axis_enabled[0]) {
      integral_.x() = 0.0F;
    }
    if (!axis_enabled[1]) {
      integral_.y() = 0.0F;
    }
    if (!axis_enabled[2]) {
      integral_.z() = 0.0F;
    }
  }

private:
  Eigen::Vector3f kp_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f ki_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f kd_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f integral_{Eigen::Vector3f::Zero()};
};

class AccelerationToThrust
{
public:
  void setParams(
    const float hover_thrust, const float tilt_max_rad, const float thr_min,
    const bool decouple_horizontal_vertical_accel)
  {
    hover_thrust_ = hover_thrust;
    tilt_max_rad_ = tilt_max_rad;
    thr_min_ = thr_min;
    decouple_horizontal_vertical_accel_ = decouple_horizontal_vertical_accel;
  }

  Eigen::Vector3f compute(const Eigen::Vector3f & acc_sp) const
  {
    const float body_z_z = decouple_horizontal_vertical_accel_ ? kGravity : (kGravity - acc_sp.z());
    Eigen::Vector3f body_z(-acc_sp.x(), -acc_sp.y(), body_z_z);
    body_z = normalizeOrDefault(body_z, Eigen::Vector3f(0.0F, 0.0F, 1.0F));
    body_z = limitTilt(body_z, tilt_max_rad_);

    const float thrust_ned_z = acc_sp.z() * (hover_thrust_ / kGravity) - hover_thrust_;
    const float cos_tilt = std::max(body_z.dot(Eigen::Vector3f(0.0F, 0.0F, 1.0F)), kMinCosTilt);
    float collective_thrust = thrust_ned_z / cos_tilt;
    collective_thrust = std::min(collective_thrust, -thr_min_);

    return body_z * collective_thrust;
  }

private:
  float hover_thrust_{0.5F};
  float tilt_max_rad_{0.7854F};
  float thr_min_{0.12F};
  bool decouple_horizontal_vertical_accel_{true};
};

class ThrustSaturation
{
public:
  void setParams(const float thr_max, const float thr_xy_margin)
  {
    thr_max_ = thr_max;
    thr_xy_margin_ = thr_xy_margin;
  }

  Eigen::Vector3f saturate(const Eigen::Vector3f & thr_sp) const
  {
    Eigen::Vector3f saturated = thr_sp;
    const Eigen::Vector2f thr_xy(saturated.x(), saturated.y());
    const float thr_xy_norm = thr_xy.norm();

    const float allocated_xy = std::min(thr_xy_norm, thr_xy_margin_);
    const float thr_z_max = safeSqrt(thr_max_ * thr_max_ - allocated_xy * allocated_xy);
    saturated.z() = std::max(saturated.z(), -thr_z_max);

    const float thr_max_xy = safeSqrt(thr_max_ * thr_max_ - saturated.z() * saturated.z());
    if (thr_xy_norm > thr_max_xy && thr_xy_norm > kEpsilon) {
      const float scale = thr_max_xy / thr_xy_norm;
      saturated.x() *= scale;
      saturated.y() *= scale;
    }

    return saturated;
  }

private:
  float thr_max_{1.0F};
  float thr_xy_margin_{0.3F};
};

class ThrustToAttitude
{
public:
  AttitudeCommand compute(
    const Eigen::Vector3f & thr_sp, const float yaw_sp, const float yaw_current) const
  {
    const float yaw = isFinite(yaw_sp) ? yaw_sp : yaw_current;

    const Eigen::Vector3f body_z =
      normalizeOrDefault(-thr_sp, Eigen::Vector3f(0.0F, 0.0F, 1.0F));
    const Eigen::Vector3f yaw_perp(-std::sin(yaw), std::cos(yaw), 0.0F);

    Eigen::Vector3f body_x = yaw_perp.cross(body_z);
    if (body_z.z() < 0.0F) {
      body_x = -body_x;
    }

    if (body_x.norm() <= kEpsilon) {
      const Eigen::Vector3f yaw_forward(std::cos(yaw), std::sin(yaw), 0.0F);
      body_x = yaw_forward - body_z * body_z.dot(yaw_forward);
    }
    body_x = normalizeOrDefault(body_x, Eigen::Vector3f(1.0F, 0.0F, 0.0F));
    Eigen::Vector3f body_y = body_z.cross(body_x);
    body_y = normalizeOrDefault(body_y, Eigen::Vector3f(0.0F, 1.0F, 0.0F));

    Eigen::Matrix3f rotation;
    rotation.col(0) = body_x;
    rotation.col(1) = body_y;
    rotation.col(2) = body_z;
    Eigen::Quaternionf q_sp(rotation);
    q_sp.normalize();

    AttitudeCommand command;
    command.q_sp = q_sp;
    command.thrust_body_z = -thr_sp.norm();
    return command;
  }
};

}  // namespace

class OffboardVelocityToAttitudeNode : public rclcpp::Node
{
public:
  OffboardVelocityToAttitudeNode()
  : Node("offboard_velocity_to_attitude_node")
  {
    loadParameters();
    configureControllers();
    createInterfaces();

    RCLCPP_INFO(
      get_logger(),
      "Started velocity->attitude converter. input='%s' output='%s' rate=%.1fHz",
      params_.trajectory_setpoint_topic.c_str(), params_.vehicle_attitude_setpoint_topic.c_str(),
      params_.loop_rate_hz);
  }

private:
  struct Params
  {
    float kp_vel_xy{1.8F};
    float ki_vel_xy{0.4F};
    float kd_vel_xy{0.2F};
    float kp_vel_z{4.0F};
    float ki_vel_z{2.0F};
    float kd_vel_z{0.0F};
    float hover_thrust{0.5F};
    float thr_min{0.12F};
    float thr_max{1.0F};
    float tilt_max_rad{0.7854F};
    float thr_xy_margin{0.3F};
    float lim_vel_horizontal{12.0F};
    float lim_vel_up{3.0F};
    float lim_vel_down{1.5F};
    bool decouple_horizontal_vertical_acceleration{true};
    bool publish_offboard_control_mode{true};
    float loop_rate_hz{50.0F};
    float setpoint_timeout_sec{0.5F};
    std::string trajectory_setpoint_topic{"/fmu/in/trajectory_setpoint"};
    std::string vehicle_local_position_topic{"/fmu/out/vehicle_local_position"};
    std::string vehicle_land_detected_topic{"/fmu/out/vehicle_land_detected"};
    std::string vehicle_attitude_setpoint_topic{"/fmu/in/vehicle_attitude_setpoint"};
    std::string offboard_control_mode_topic{"/fmu/in/offboard_control_mode"};
  };

  void loadParameters()
  {
    params_.kp_vel_xy = static_cast<float>(declare_parameter("kp_vel_xy", 1.8));
    params_.ki_vel_xy = static_cast<float>(declare_parameter("ki_vel_xy", 0.4));
    params_.kd_vel_xy = static_cast<float>(declare_parameter("kd_vel_xy", 0.2));
    params_.kp_vel_z = static_cast<float>(declare_parameter("kp_vel_z", 4.0));
    params_.ki_vel_z = static_cast<float>(declare_parameter("ki_vel_z", 2.0));
    params_.kd_vel_z = static_cast<float>(declare_parameter("kd_vel_z", 0.0));
    params_.hover_thrust = static_cast<float>(declare_parameter("hover_thrust", 0.5));
    params_.thr_min = static_cast<float>(declare_parameter("thr_min", 0.12));
    params_.thr_max = static_cast<float>(declare_parameter("thr_max", 1.0));
    params_.tilt_max_rad = static_cast<float>(declare_parameter("tilt_max_rad", 0.7854));
    params_.thr_xy_margin = static_cast<float>(declare_parameter("thr_xy_margin", 0.3));
    params_.lim_vel_horizontal = static_cast<float>(declare_parameter("lim_vel_horizontal", 12.0));
    params_.lim_vel_up = static_cast<float>(declare_parameter("lim_vel_up", 3.0));
    params_.lim_vel_down = static_cast<float>(declare_parameter("lim_vel_down", 1.5));
    params_.decouple_horizontal_vertical_acceleration =
      declare_parameter("decouple_horizontal_vertical_acceleration", true);
    params_.publish_offboard_control_mode = declare_parameter("publish_offboard_control_mode", true);
    params_.loop_rate_hz = static_cast<float>(declare_parameter("loop_rate_hz", 50.0));
    params_.setpoint_timeout_sec = static_cast<float>(declare_parameter("setpoint_timeout_sec", 0.5));
    params_.trajectory_setpoint_topic =
      declare_parameter("trajectory_setpoint_topic", params_.trajectory_setpoint_topic);
    params_.vehicle_local_position_topic =
      declare_parameter("vehicle_local_position_topic", params_.vehicle_local_position_topic);
    params_.vehicle_land_detected_topic =
      declare_parameter("vehicle_land_detected_topic", params_.vehicle_land_detected_topic);
    params_.vehicle_attitude_setpoint_topic =
      declare_parameter("vehicle_attitude_setpoint_topic", params_.vehicle_attitude_setpoint_topic);
    params_.offboard_control_mode_topic =
      declare_parameter("offboard_control_mode_topic", params_.offboard_control_mode_topic);
  }

  void configureControllers()
  {
    const Eigen::Vector3f kp(params_.kp_vel_xy, params_.kp_vel_xy, params_.kp_vel_z);
    const Eigen::Vector3f ki(params_.ki_vel_xy, params_.ki_vel_xy, params_.ki_vel_z);
    const Eigen::Vector3f kd(params_.kd_vel_xy, params_.kd_vel_xy, params_.kd_vel_z);
    pid_.setGains(kp, ki, kd);

    acc_to_thrust_.setParams(
      params_.hover_thrust, params_.tilt_max_rad, params_.thr_min,
      params_.decouple_horizontal_vertical_acceleration);
    thrust_saturation_.setParams(params_.thr_max, params_.thr_xy_margin);

    nominal_dt_sec_ = 1.0F / std::max(1.0F, params_.loop_rate_hz);
  }

  void createInterfaces()
  {
    rclcpp::QoS sensor_qos(rclcpp::KeepLast(10));
    sensor_qos.best_effort();

    rclcpp::QoS control_qos(rclcpp::KeepLast(10));
    control_qos.best_effort();

    trajectory_setpoint_sub_ = create_subscription<px4_msgs::msg::TrajectorySetpoint>(
      params_.trajectory_setpoint_topic, sensor_qos,
      std::bind(&OffboardVelocityToAttitudeNode::onTrajectorySetpoint, this, std::placeholders::_1));
    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      params_.vehicle_local_position_topic, sensor_qos,
      std::bind(&OffboardVelocityToAttitudeNode::onVehicleLocalPosition, this, std::placeholders::_1));
    land_detected_sub_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
      params_.vehicle_land_detected_topic, sensor_qos,
      std::bind(&OffboardVelocityToAttitudeNode::onVehicleLandDetected, this, std::placeholders::_1));

    attitude_setpoint_pub_ = create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
      params_.vehicle_attitude_setpoint_topic, control_qos);
    if (params_.publish_offboard_control_mode) {
      offboard_control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
        params_.offboard_control_mode_topic, control_qos);
    }

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(1.0F, params_.loop_rate_hz)));
    timer_ = create_wall_timer(period, std::bind(&OffboardVelocityToAttitudeNode::onTimer, this));
    last_traj_rx_time_ = now();
  }

  void onTrajectorySetpoint(const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg)
  {
    latest_trajectory_setpoint_ = *msg;
    has_trajectory_setpoint_ = true;
    last_traj_rx_time_ = now();
  }

  void onVehicleLocalPosition(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    latest_local_position_ = *msg;
    has_local_position_ = true;
  }

  void onVehicleLandDetected(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
  {
    landed_ = msg->landed;
    if (landed_) {
      pid_.reset();
    }
  }

  InputValidationResult validateInputSetpoint(
    const px4_msgs::msg::TrajectorySetpoint & setpoint, const Eigen::Vector3f & pos,
    const Eigen::Vector3f & vel, const Eigen::Vector3f & vel_dot) const
  {
    std::array<bool, 3> pos_finite{{false, false, false}};
    std::array<bool, 3> vel_finite{{false, false, false}};
    std::array<bool, 3> acc_finite{{false, false, false}};

    // PX4 _inputValid(): each axis must have at least one finite setpoint among pos/vel/acc.
    for (int i = 0; i < 3; ++i) {
      pos_finite[static_cast<size_t>(i)] = isFinite(setpoint.position[i]);
      vel_finite[static_cast<size_t>(i)] = isFinite(setpoint.velocity[i]);
      acc_finite[static_cast<size_t>(i)] = isFinite(setpoint.acceleration[i]);

      if (!(pos_finite[static_cast<size_t>(i)] ||
        vel_finite[static_cast<size_t>(i)] ||
        acc_finite[static_cast<size_t>(i)]))
      {
        return {false, "Invalid setpoint: each axis requires pos/vel/acc (finite)."};
      }
    }

    // PX4 _inputValid(): x/y setpoints must always come in pairs.
    if (pos_finite[0] != pos_finite[1]) {
      return {false, "Invalid setpoint: position x/y must be both finite or both NaN."};
    }
    if (vel_finite[0] != vel_finite[1]) {
      return {false, "Invalid setpoint: velocity x/y must be both finite or both NaN."};
    }
    if (acc_finite[0] != acc_finite[1]) {
      return {false, "Invalid setpoint: acceleration x/y must be both finite or both NaN."};
    }

    // PX4 _inputValid(): if a state is controlled by setpoint, corresponding estimate must be finite.
    for (int i = 0; i < 3; ++i) {
      const size_t idx = static_cast<size_t>(i);
      if (pos_finite[idx] && !isFinite(pos[i])) {
        return {false, "Invalid state: finite position setpoint requires finite position estimate."};
      }
      if (vel_finite[idx] && (!isFinite(vel[i]) || !isFinite(vel_dot[i]))) {
        return {false, "Invalid state: finite velocity setpoint requires finite velocity and acceleration estimate."};
      }
    }

    // Node mode constraint: this node implements velocity->attitude path only.
    if (pos_finite[0] || pos_finite[1] || pos_finite[2]) {
      return {false, "Unsupported setpoint: position setpoint must be NaN for velocity-only mode."};
    }
    if (acc_finite[0] || acc_finite[1] || acc_finite[2]) {
      return {false, "Unsupported setpoint: acceleration setpoint must be NaN for velocity-only mode."};
    }
    if (!(vel_finite[0] && vel_finite[1] && vel_finite[2])) {
      return {false, "Unsupported setpoint: velocity setpoint must be finite on all axes."};
    }

    return {true, ""};
  }

  VelocityCommand extractVelocityCommand(const Eigen::Vector3f & vel_current) const
  {
    VelocityCommand command;
    command.vel_sp = vel_current;
    for (int i = 0; i < 3; ++i) {
      const float axis_setpoint = latest_trajectory_setpoint_.velocity[i];
      if (isFinite(axis_setpoint)) {
        command.vel_sp[i] = axis_setpoint;
        command.axis_enabled[static_cast<size_t>(i)] = true;
      }
    }
    return command;
  }

  float computeDtSec()
  {
    float dt_sec = nominal_dt_sec_;
    const uint64_t timestamp_us = latest_local_position_.timestamp;
    if (last_local_position_timestamp_us_ > 0U && timestamp_us > last_local_position_timestamp_us_) {
      dt_sec = static_cast<float>(timestamp_us - last_local_position_timestamp_us_) * 1e-6F;
    }
    last_local_position_timestamp_us_ = timestamp_us;
    return clampScalar(dt_sec, kMinDtSec, kMaxDtSec);
  }

  uint64_t nowMicros()
  {
    return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000ULL);
  }

  void publishOffboardControlMode(const uint64_t timestamp_us)
  {
    if (!params_.publish_offboard_control_mode || !offboard_control_mode_pub_) {
      return;
    }

    px4_msgs::msg::OffboardControlMode msg{};
    msg.timestamp = timestamp_us;
    msg.position = false;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = true;
    msg.body_rate = false;
    offboard_control_mode_pub_->publish(msg);
  }

  void publishVehicleAttitudeSetpoint(
    const uint64_t timestamp_us, const AttitudeCommand & cmd, const float yaw_rate_sp)
  {
    px4_msgs::msg::VehicleAttitudeSetpoint msg{};
    msg.timestamp = timestamp_us;
    msg.q_d[0] = cmd.q_sp.w();
    msg.q_d[1] = cmd.q_sp.x();
    msg.q_d[2] = cmd.q_sp.y();
    msg.q_d[3] = cmd.q_sp.z();
    msg.thrust_body[0] = 0.0F;
    msg.thrust_body[1] = 0.0F;
    msg.thrust_body[2] = cmd.thrust_body_z;
    msg.yaw_sp_move_rate = yaw_rate_sp;
    attitude_setpoint_pub_->publish(msg);
  }

  void onTimer()
  {
    if (!has_local_position_ || !has_trajectory_setpoint_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Waiting for both trajectory_setpoint and vehicle_local_position inputs.");
      return;
    }

    const float traj_age_sec = static_cast<float>((now() - last_traj_rx_time_).seconds());
    const bool traj_stale = traj_age_sec > params_.setpoint_timeout_sec;

    float yaw_sp = std::numeric_limits<float>::quiet_NaN();
    float yawspeed_sp = 0.0F;
    if (!isFinite(latest_local_position_.vx) || !isFinite(latest_local_position_.vy) ||
      !isFinite(latest_local_position_.vz))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "VehicleLocalPosition velocity contains NaN/Inf; skipping control update.");
      return;
    }

    const Eigen::Vector3f vel(
      latest_local_position_.vx, latest_local_position_.vy, latest_local_position_.vz);
    const float dt_sec = computeDtSec();
    Eigen::Vector3f fallback_acceleration = Eigen::Vector3f::Zero();
    if (has_previous_velocity_ && dt_sec > kEpsilon) {
      fallback_acceleration = (vel - previous_velocity_) / dt_sec;
    }
    Eigen::Vector3f vel_dot =
      extractMeasuredAcceleration(latest_local_position_, fallback_acceleration);
    const Eigen::Vector3f pos = extractMeasuredPosition(latest_local_position_);
    previous_velocity_ = vel;
    has_previous_velocity_ = true;
    const float yaw_current = isFinite(latest_local_position_.heading) ? latest_local_position_.heading : 0.0F;

    if (!traj_stale) {
      const InputValidationResult validation = validateInputSetpoint(
        latest_trajectory_setpoint_, pos, vel, vel_dot);
      if (!validation.valid) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Rejected TrajectorySetpoint: %s", validation.reason.c_str());
        return;
      }
    }

    VelocityCommand velocity_command;
    if (!traj_stale) {
      velocity_command = extractVelocityCommand(vel);
      yaw_sp = latest_trajectory_setpoint_.yaw;
      if (isFinite(latest_trajectory_setpoint_.yawspeed)) {
        yawspeed_sp = latest_trajectory_setpoint_.yawspeed;
      }
    } else {
      velocity_command.vel_sp = Eigen::Vector3f::Zero();
      velocity_command.axis_enabled = {{true, true, true}};
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "TrajectorySetpoint timeout. Forcing zero-velocity hold.");
    }

    if (!isFinite(yaw_sp)) {
      yaw_sp = yaw_current;
    }

    Eigen::Vector3f vel_sp = velocity_command.vel_sp;
    if (velocity_command.axis_enabled[0] && velocity_command.axis_enabled[1]) {
      const Eigen::Vector2f vel_limited_xy = constrainXY(
        Eigen::Vector2f::Zero(), Eigen::Vector2f(vel_sp.x(), vel_sp.y()), params_.lim_vel_horizontal);
      vel_sp.x() = vel_limited_xy.x();
      vel_sp.y() = vel_limited_xy.y();
    } else if (velocity_command.axis_enabled[0]) {
      vel_sp.x() = clampScalar(vel_sp.x(), -params_.lim_vel_horizontal, params_.lim_vel_horizontal);
    } else if (velocity_command.axis_enabled[1]) {
      vel_sp.y() = clampScalar(vel_sp.y(), -params_.lim_vel_horizontal, params_.lim_vel_horizontal);
    }
    if (velocity_command.axis_enabled[2]) {
      vel_sp.z() = clampScalar(vel_sp.z(), -params_.lim_vel_up, params_.lim_vel_down);
    }

    Eigen::Vector3f vel_error = vel_sp - vel;
    for (size_t i = 0; i < velocity_command.axis_enabled.size(); ++i) {
      if (!velocity_command.axis_enabled[i]) {
        vel_error[static_cast<int>(i)] = 0.0F;
        vel_dot[static_cast<int>(i)] = 0.0F;
      }
    }
    pid_.clearIntegralForDisabledAxes(velocity_command.axis_enabled);
    const Eigen::Vector3f acc_sp = pid_.computeAcceleration(vel_error, vel_dot);

    Eigen::Vector3f thr_sp = acc_to_thrust_.compute(acc_sp);
    pid_.applyVerticalAntiWindup(thr_sp, params_.thr_min, params_.thr_max, &vel_error);
    thr_sp = thrust_saturation_.saturate(thr_sp);
    pid_.applyHorizontalArw(acc_sp, thr_sp, params_.hover_thrust, &vel_error);

    if (landed_) {
      pid_.reset();
    } else {
      pid_.integrate(vel_error, dt_sec);
    }

    const AttitudeCommand attitude_cmd = thrust_to_attitude_.compute(thr_sp, yaw_sp, yaw_current);
    const uint64_t timestamp_us = latest_local_position_.timestamp > 0U ?
      latest_local_position_.timestamp : nowMicros();
    publishOffboardControlMode(timestamp_us);
    publishVehicleAttitudeSetpoint(timestamp_us, attitude_cmd, yawspeed_sp);
  }

  Params params_{};
  float nominal_dt_sec_{0.02F};

  VelocityPid3D pid_{};
  AccelerationToThrust acc_to_thrust_{};
  ThrustSaturation thrust_saturation_{};
  ThrustToAttitude thrust_to_attitude_{};

  bool has_trajectory_setpoint_{false};
  bool has_local_position_{false};
  bool landed_{false};

  px4_msgs::msg::TrajectorySetpoint latest_trajectory_setpoint_{};
  px4_msgs::msg::VehicleLocalPosition latest_local_position_{};
  Eigen::Vector3f previous_velocity_{Eigen::Vector3f::Zero()};
  bool has_previous_velocity_{false};
  rclcpp::Time last_traj_rx_time_{0, 0, RCL_ROS_TIME};
  uint64_t last_local_position_timestamp_us_{0U};

  rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_detected_sub_;

  rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardVelocityToAttitudeNode>());
  rclcpp::shutdown();
  return 0;
}

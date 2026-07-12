#include "spot_micro_motion_cmd.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <eigen3/Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "i2c_pwm_board_msgs/msg/servo.hpp"
#include "i2c_pwm_board_msgs/msg/servo_array.hpp"
#include "i2c_pwm_board_msgs/msg/servo_config.hpp"
#include "i2c_pwm_board_msgs/srv/servos_config.hpp"

#include "spot_micro_kinematics/spot_micro_kinematics.h"

#include "spot_micro_idle.h"
#include "utils.h"

using namespace smk;
using namespace Eigen;

using VectorStringPairs = std::vector<std::pair<std::string, std::string>>;

template<typename T>
static void declare_and_get(
  rclcpp::Node* node,
  const std::string& name,
  T& output,
  const T& default_value)
{
  node->declare_parameter<T>(name, default_value);
  node->get_parameter(name, output);
}

// ROS 2 stores YAML decimal values, e.g. 306.0, as double parameters.
// Most of this project stores config values as float. If we declare/get those
// params as float, ROS 2 can leave the default value in place. This overload
// declares and reads the ROS parameter as double, then safely casts to float.
static void declare_and_get(
  rclcpp::Node* node,
  const std::string& name,
  float& output,
  const float& default_value)
{
  double value = static_cast<double>(default_value);
  node->declare_parameter<double>(name, value);
  node->get_parameter(name, value);
  output = static_cast<float>(value);
}

static std::vector<int> to_int_vector(const std::vector<int64_t>& input)
{
  std::vector<int> output;
  output.reserve(input.size());

  for (const auto value : input) {
    output.push_back(static_cast<int>(value));
  }

  return output;
}

SpotMicroMotionCmd::SpotMicroMotionCmd()
: rclcpp::Node("spot_micro_motion_cmd")
{
  cmd_ = Command();
  state_ = std::make_unique<SpotMicroIdleState>();

  readInConfigParameters();

  sm_ = smk::SpotMicroKinematics(0.0f, 0.0f, 0.0f, smnc_.smc);

  body_state_cmd_.euler_angs.phi = 0.0f;
  body_state_cmd_.euler_angs.theta = 0.0f;
  body_state_cmd_.euler_angs.psi = 0.0f;

  body_state_cmd_.xyz_pos.x = 0.0f;
  body_state_cmd_.xyz_pos.y = smnc_.lie_down_height;
  body_state_cmd_.xyz_pos.z = 0.0f;

  body_state_cmd_.leg_feet_pos = getLieDownStance();
  sm_.setBodyState(body_state_cmd_);

  robot_odometry_.euler_angs.phi = 0.0f;
  robot_odometry_.euler_angs.theta = 0.0f;
  robot_odometry_.euler_angs.psi = 0.0f;

  robot_odometry_.xyz_pos.x = 0.0f;
  robot_odometry_.xyz_pos.y = 0.0f;
  robot_odometry_.xyz_pos.z = 0.0f;

  // UPDATED:
  // Build the outgoing servo array directly from YAML-defined servo numbers.
  // This allows non-contiguous PCA9685 channels such as 0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14.
  // The old code assumed logical servo numbers 1..num_servos and indexed with servo_num - 1.
  servo_array_.servos.clear();

  for (const auto& [servo_name, servo_config_params] : smnc_.servo_config) {
    i2c_pwm_board_msgs::msg::Servo temp_servo;
    temp_servo.servo = static_cast<int>(std::round(servo_config_params.at("num")));
    temp_servo.value = 0.0f;
    servo_array_.servos.push_back(temp_servo);

    RCLCPP_INFO(
      this->get_logger(),
      "Servo array entry: joint=%s servo_num=%d pca9685_port_if_0_based=%d initial_value=%.3f",
      servo_name.c_str(),
      temp_servo.servo,
      temp_servo.servo - 1,
      temp_servo.value);
  }

  servo_array_absolute_.servos = servo_array_.servos;

  transform_br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  static_transform_br_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

  stand_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/stand_cmd", 10,
    std::bind(&SpotMicroMotionCmd::standCommandCallback, this, std::placeholders::_1));

  idle_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/idle_cmd", 10,
    std::bind(&SpotMicroMotionCmd::idleCommandCallback, this, std::placeholders::_1));

  walk_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/walk_cmd", 10,
    std::bind(&SpotMicroMotionCmd::walkCommandCallback, this, std::placeholders::_1));

  body_angle_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
    "/angle_cmd", 10,
    std::bind(&SpotMicroMotionCmd::angleCommandCallback, this, std::placeholders::_1));

  vel_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10,
    std::bind(&SpotMicroMotionCmd::velCommandCallback, this, std::placeholders::_1));

  servos_absolute_pub_ =
    this->create_publisher<i2c_pwm_board_msgs::msg::ServoArray>("servos_absolute_1", 10);

  servos_proportional_pub_ =
    this->create_publisher<i2c_pwm_board_msgs::msg::ServoArray>("servos_proportional_1", 10);

  servos_config_client_ =
    this->create_client<i2c_pwm_board_msgs::srv::ServosConfig>("config_servos_1");

  body_state_pub_ =
    this->create_publisher<std_msgs::msg::Float32MultiArray>("body_state", 10);

  lcd_state_pub_ =
    this->create_publisher<std_msgs::msg::String>("lcd_state", 10);

  lcd_vel_cmd_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>("lcd_vel_cmd", 10);

  lcd_angle_cmd_pub_ =
    this->create_publisher<geometry_msgs::msg::Vector3>("lcd_angle_cmd", 10);

  lcd_state_string_msg_.data = "Idle";

  lcd_vel_cmd_msg_.linear.x = 0.0f;
  lcd_vel_cmd_msg_.linear.y = 0.0f;
  lcd_vel_cmd_msg_.linear.z = 0.0f;
  lcd_vel_cmd_msg_.angular.x = 0.0f;
  lcd_vel_cmd_msg_.angular.y = 0.0f;
  lcd_vel_cmd_msg_.angular.z = 0.0f;

  lcd_angle_cmd_msg_.x = 0.0f;
  lcd_angle_cmd_msg_.y = 0.0f;
  lcd_angle_cmd_msg_.z = 0.0f;

  if (smnc_.plot_mode) {
    body_state_msg_.data.resize(18, 0.0f);
  }

  publishStaticTransforms();

  RCLCPP_INFO(this->get_logger(), "Spot Micro motion command node initialized");
}

SpotMicroMotionCmd::~SpotMicroMotionCmd()
{
}

void SpotMicroMotionCmd::runOnce()
{
  handleInputCommands();
  resetEventCommands();

  if (smnc_.plot_mode) {
    publishBodyState();
  }

  publishLcdMonitorData();
  publishDynamicTransforms();

  if (smnc_.publish_odom) {
    integrateOdometry();
  }
}

bool SpotMicroMotionCmd::publishServoConfiguration()
{
  auto request = std::make_shared<i2c_pwm_board_msgs::srv::ServosConfig::Request>();

  for (auto iter = smnc_.servo_config.begin(); iter != smnc_.servo_config.end(); ++iter) {
    const std::map<std::string, float>& servo_config_params = iter->second;

    i2c_pwm_board_msgs::msg::ServoConfig temp_servo_config;
    temp_servo_config.center = servo_config_params.at("center");
    temp_servo_config.range = servo_config_params.at("range");
    temp_servo_config.servo = static_cast<int>(std::round(servo_config_params.at("num")));
    temp_servo_config.direction = static_cast<int>(std::round(servo_config_params.at("direction")));

    RCLCPP_INFO(
      this->get_logger(),
      "Config request: joint=%s servo_num=%d pca9685_port_if_0_based=%d center=%.1f range=%.1f direction=%d",
      iter->first.c_str(),
      temp_servo_config.servo,
      temp_servo_config.servo - 1,
      static_cast<double>(temp_servo_config.center),
      static_cast<double>(temp_servo_config.range),
      temp_servo_config.direction);

    request->servos.push_back(temp_servo_config);
  }

  if (!servos_config_client_->wait_for_service(std::chrono::seconds(2))) {
    if (!smnc_.debug_mode && !smnc_.run_standalone) {
      RCLCPP_ERROR(this->get_logger(), "Service config_servos is not available");
      return false;
    }

    RCLCPP_WARN(
      this->get_logger(),
      "Service config_servos is not available; continuing because debug/run_standalone is enabled");
    return true;
  }

  auto future = servos_config_client_->async_send_request(request);

  const auto result = rclcpp::spin_until_future_complete(
    this->get_node_base_interface(), future, std::chrono::seconds(2));

  if (result != rclcpp::FutureReturnCode::SUCCESS) {
    if (!smnc_.debug_mode && !smnc_.run_standalone) {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service config_servos");
      return false;
    }

    RCLCPP_WARN(
      this->get_logger(),
      "Failed to call config_servos; continuing because debug/run_standalone is enabled");
  }

  return true;
}

void SpotMicroMotionCmd::setServoCommandMessageData()
{
  sm_.setBodyState(body_state_cmd_);
  LegsJointAngles joint_angs = sm_.getLegsJointAngles();

  servo_cmds_rad_["RF_1"] = joint_angs.right_front.ang1;
  servo_cmds_rad_["RF_2"] = joint_angs.right_front.ang2;
  servo_cmds_rad_["RF_3"] = joint_angs.right_front.ang3;

  servo_cmds_rad_["RB_1"] = joint_angs.right_back.ang1;
  servo_cmds_rad_["RB_2"] = joint_angs.right_back.ang2;
  servo_cmds_rad_["RB_3"] = joint_angs.right_back.ang3;

  servo_cmds_rad_["LF_1"] = joint_angs.left_front.ang1;
  servo_cmds_rad_["LF_2"] = joint_angs.left_front.ang2;
  servo_cmds_rad_["LF_3"] = joint_angs.left_front.ang3;

  servo_cmds_rad_["LB_1"] = joint_angs.left_back.ang1;
  servo_cmds_rad_["LB_2"] = joint_angs.left_back.ang2;
  servo_cmds_rad_["LB_3"] = joint_angs.left_back.ang3;
}

void SpotMicroMotionCmd::publishServoProportionalCommand()
{
  for (auto iter = smnc_.servo_config.begin(); iter != smnc_.servo_config.end(); ++iter) {
    const std::string& servo_name = iter->first;
    const std::map<std::string, float>& servo_config_params = iter->second;

    const int servo_num = static_cast<int>(servo_config_params.at("num"));
    const float cmd_ang_rad = servo_cmds_rad_[servo_name];

    const float center_ang_rad =
      servo_config_params.at("center_angle_deg") * static_cast<float>(M_PI) / 180.0f;

    float servo_proportional_cmd =
      (cmd_ang_rad - center_ang_rad) /
      (smnc_.servo_max_angle_deg * static_cast<float>(M_PI) / 180.0f);

    if (servo_proportional_cmd > 1.0f) {
      servo_proportional_cmd = 1.0f;
      RCLCPP_WARN(this->get_logger(), "Proportional command above +1.0 clipped to 1.0");
      RCLCPP_WARN(
        this->get_logger(), "Joint %s, Angle: %.2f",
        servo_name.c_str(), cmd_ang_rad * 180.0f / static_cast<float>(M_PI));
    } else if (servo_proportional_cmd < -1.0f) {
      servo_proportional_cmd = -1.0f;
      RCLCPP_WARN(this->get_logger(), "Proportional command below -1.0 clipped to -1.0");
      RCLCPP_WARN(
        this->get_logger(), "Joint %s, Angle: %.2f",
        servo_name.c_str(), cmd_ang_rad * 180.0f / static_cast<float>(M_PI));
    }

    // UPDATED:
    // Do not use servo_array_.servos[servo_num - 1].
    // servo_num is now the actual YAML/PCA9685 channel and may be non-contiguous or zero.
    bool found_servo = false;

    for (auto& servo : servo_array_.servos) {
      if (servo.servo == servo_num) {
        servo.value = servo_proportional_cmd;
        found_servo = true;
        break;
      }
    }

    if (!found_servo) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Servo num %d for joint %s is not in servo_array_",
        servo_num,
        servo_name.c_str());
    }

    if (smnc_.debug_mode) {
      RCLCPP_INFO(
        this->get_logger(),
        "Servo cmd: joint=%s servo_num=%d pca9685_port_if_0_based=%d ik_angle_deg=%.2f center_angle_deg=%.2f proportional=%.3f",
        servo_name.c_str(),
        servo_num,
        servo_num - 1,
        static_cast<double>(cmd_ang_rad * 180.0f / static_cast<float>(M_PI)),
        static_cast<double>(servo_config_params.at("center_angle_deg")),
        static_cast<double>(servo_proportional_cmd));
    }
  }

  servos_proportional_pub_->publish(servo_array_);
}

void SpotMicroMotionCmd::publishZeroServoAbsoluteCommand()
{
  servos_absolute_pub_->publish(servo_array_absolute_);
}

SpotMicroNodeConfig SpotMicroMotionCmd::getNodeConfig()
{
  return smnc_;
}

LegsFootPos SpotMicroMotionCmd::getNeutralStance()
{
  const float len = smnc_.smc.body_length;
  const float width = smnc_.smc.body_width;
  const float l1 = smnc_.smc.hip_link_length;
  const float f_offset = smnc_.stand_front_x_offset;
  const float b_offset = smnc_.stand_back_x_offset;

  LegsFootPos neutral_stance;

  neutral_stance.right_back.x = -len / 2.0f + b_offset;
  neutral_stance.right_back.y = 0.0f;
  neutral_stance.right_back.z = width / 2.0f + l1;

  neutral_stance.right_front.x = len / 2.0f + f_offset;
  neutral_stance.right_front.y = 0.0f;
  neutral_stance.right_front.z = width / 2.0f + l1;

  neutral_stance.left_front.x = len / 2.0f + f_offset;
  neutral_stance.left_front.y = 0.0f;
  neutral_stance.left_front.z = -width / 2.0f - l1;

  neutral_stance.left_back.x = -len / 2.0f + b_offset;
  neutral_stance.left_back.y = 0.0f;
  neutral_stance.left_back.z = -width / 2.0f - l1;

  return neutral_stance;
}

LegsFootPos SpotMicroMotionCmd::getLieDownStance()
{
  const float len = smnc_.smc.body_length;
  const float width = smnc_.smc.body_width;
  const float l1 = smnc_.smc.hip_link_length;
  const float x_off = smnc_.lie_down_feet_x_offset;

  LegsFootPos lie_down_stance;

  lie_down_stance.right_back.x = -len / 2.0f + x_off;
  lie_down_stance.right_back.y = 0.0f;
  lie_down_stance.right_back.z = width / 2.0f + l1;

  lie_down_stance.right_front.x = len / 2.0f + x_off;
  lie_down_stance.right_front.y = 0.0f;
  lie_down_stance.right_front.z = width / 2.0f + l1;

  lie_down_stance.left_front.x = len / 2.0f + x_off;
  lie_down_stance.left_front.y = 0.0f;
  lie_down_stance.left_front.z = -width / 2.0f - l1;

  lie_down_stance.left_back.x = -len / 2.0f + x_off;
  lie_down_stance.left_back.y = 0.0f;
  lie_down_stance.left_back.z = -width / 2.0f - l1;

  return lie_down_stance;
}

void SpotMicroMotionCmd::commandIdle()
{
  cmd_.idle_cmd_ = true;
}

std::string SpotMicroMotionCmd::getCurrentStateName()
{
  return state_->getCurrentStateName();
}

void SpotMicroMotionCmd::readInConfigParameters()
{
  declare_and_get(this, "hip_link_length", smnc_.smc.hip_link_length, 0.055f);
  declare_and_get(this, "upper_leg_link_length", smnc_.smc.upper_leg_link_length, 0.1075f);
  declare_and_get(this, "lower_leg_link_length", smnc_.smc.lower_leg_link_length, 0.130f);
  declare_and_get(this, "body_width", smnc_.smc.body_width, 0.078f);
  declare_and_get(this, "body_length", smnc_.smc.body_length, 0.186f);

  declare_and_get(this, "default_stand_height", smnc_.default_stand_height, 0.18f);
  declare_and_get(this, "stand_front_x_offset", smnc_.stand_front_x_offset, 0.0f);
  declare_and_get(this, "stand_back_x_offset", smnc_.stand_back_x_offset, 0.0f);
  declare_and_get(this, "lie_down_height", smnc_.lie_down_height, 0.05f);
  declare_and_get(this, "lie_down_foot_x_offset", smnc_.lie_down_feet_x_offset, 0.0f);

  declare_and_get(this, "num_servos", smnc_.num_servos, 12);
  declare_and_get(this, "servo_max_angle_deg", smnc_.servo_max_angle_deg, 90.0f);

  declare_and_get(this, "transit_tau", smnc_.transit_tau, 0.1f);
  declare_and_get(this, "transit_rl", smnc_.transit_rl, 1.0f);
  declare_and_get(this, "transit_angle_rl", smnc_.transit_angle_rl, 1.0f);
  declare_and_get(this, "dt", smnc_.dt, 0.02f);

  declare_and_get(this, "debug_mode", smnc_.debug_mode, false);
  declare_and_get(this, "run_standalone", smnc_.run_standalone, false);
  declare_and_get(this, "plot_mode", smnc_.plot_mode, false);

  declare_and_get(this, "max_fwd_velocity", smnc_.max_fwd_velocity, 0.1f);
  declare_and_get(this, "max_side_velocity", smnc_.max_side_velocity, 0.1f);
  declare_and_get(this, "max_yaw_rate", smnc_.max_yaw_rate, 0.5f);

  declare_and_get(this, "z_clearance", smnc_.z_clearance, 0.03f);
  declare_and_get(this, "alpha", smnc_.alpha, 0.5f);
  declare_and_get(this, "beta", smnc_.beta, 0.5f);
  declare_and_get(this, "num_phases", smnc_.num_phases, 4);

  std::vector<int64_t> rb_contact_phases;
  std::vector<int64_t> rf_contact_phases;
  std::vector<int64_t> lf_contact_phases;
  std::vector<int64_t> lb_contact_phases;
  std::vector<int64_t> body_shift_phases;

  declare_and_get(this, "rb_contact_phases", rb_contact_phases, std::vector<int64_t>{1, 1, 0, 0});
  declare_and_get(this, "rf_contact_phases", rf_contact_phases, std::vector<int64_t>{0, 0, 1, 1});
  declare_and_get(this, "lf_contact_phases", lf_contact_phases, std::vector<int64_t>{1, 1, 0, 0});
  declare_and_get(this, "lb_contact_phases", lb_contact_phases, std::vector<int64_t>{0, 0, 1, 1});
  declare_and_get(this, "body_shift_phases", body_shift_phases, std::vector<int64_t>{1, 2, 3, 4});

  smnc_.rb_contact_phases = to_int_vector(rb_contact_phases);
  smnc_.rf_contact_phases = to_int_vector(rf_contact_phases);
  smnc_.lf_contact_phases = to_int_vector(lf_contact_phases);
  smnc_.lb_contact_phases = to_int_vector(lb_contact_phases);
  smnc_.body_shift_phases = to_int_vector(body_shift_phases);

  declare_and_get(this, "overlap_time", smnc_.overlap_time, 0.1f);
  declare_and_get(this, "swing_time", smnc_.swing_time, 0.2f);
  declare_and_get(this, "foot_height_time_constant", smnc_.foot_height_time_constant, 0.05f);

  declare_and_get(this, "fwd_body_balance_shift", smnc_.fwd_body_balance_shift, 0.0f);
  declare_and_get(this, "back_body_balance_shift", smnc_.back_body_balance_shift, 0.0f);
  declare_and_get(this, "side_body_balance_shift", smnc_.side_body_balance_shift, 0.0f);

  declare_and_get(this, "publish_odom", smnc_.publish_odom, false);

  declare_and_get(this, "lidar_x_pos", smnc_.lidar_x_pos, 0.0f);
  declare_and_get(this, "lidar_y_pos", smnc_.lidar_y_pos, 0.0f);
  declare_and_get(this, "lidar_z_pos", smnc_.lidar_z_pos, 0.0f);
  declare_and_get(this, "lidar_yaw_angle", smnc_.lidar_yaw_angle, 0.0f);

  smnc_.overlap_ticks = static_cast<int>(std::round(smnc_.overlap_time / smnc_.dt));
  smnc_.swing_ticks = static_cast<int>(std::round(smnc_.swing_time / smnc_.dt));

  if (smnc_.num_phases == 8) {
    smnc_.stance_ticks = 7 * smnc_.swing_ticks;
    smnc_.phase_ticks = std::vector<int>{
      smnc_.swing_ticks, smnc_.swing_ticks, smnc_.swing_ticks, smnc_.swing_ticks,
      smnc_.swing_ticks, smnc_.swing_ticks, smnc_.swing_ticks, smnc_.swing_ticks};
    smnc_.phase_length = smnc_.num_phases * smnc_.swing_ticks;
  } else {
    smnc_.stance_ticks = 2 * smnc_.overlap_ticks + smnc_.swing_ticks;
    smnc_.phase_ticks = std::vector<int>{
      smnc_.overlap_ticks, smnc_.swing_ticks, smnc_.overlap_ticks, smnc_.swing_ticks};
    smnc_.phase_length = 2 * smnc_.swing_ticks + 2 * smnc_.overlap_ticks;
  }

  const std::vector<std::string> servo_names = {
    "RF_3", "RF_2", "RF_1",
    "RB_3", "RB_2", "RB_1",
    "LB_3", "LB_2", "LB_1",
    "LF_3", "LF_2", "LF_1"
  };

  smnc_.servo_config.clear();

  for (const auto& servo_name : servo_names) {
    std::map<std::string, float> servo_config;

    declare_and_get(this, servo_name + ".center", servo_config["center"], 306.0f);
    declare_and_get(this, servo_name + ".range", servo_config["range"], 385.0f);
    declare_and_get(this, servo_name + ".num", servo_config["num"], 1.0f);
    declare_and_get(this, servo_name + ".direction", servo_config["direction"], 1.0f);
    declare_and_get(this, servo_name + ".center_angle_deg", servo_config["center_angle_deg"], 0.0f);

    smnc_.servo_config[servo_name] = servo_config;

    RCLCPP_INFO(
      this->get_logger(),
      "Loaded %s: num=%.0f center=%.1f range=%.1f direction=%.0f center_angle_deg=%.2f",
      servo_name.c_str(),
      static_cast<double>(servo_config["num"]),
      static_cast<double>(servo_config["center"]),
      static_cast<double>(servo_config["range"]),
      static_cast<double>(servo_config["direction"]),
      static_cast<double>(servo_config["center_angle_deg"]));
  }
}

void SpotMicroMotionCmd::standCommandCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    cmd_.stand_cmd_ = true;
  }
}

void SpotMicroMotionCmd::idleCommandCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    cmd_.idle_cmd_ = true;
  }
}

void SpotMicroMotionCmd::walkCommandCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    cmd_.walk_cmd_ = true;
  }
}

void SpotMicroMotionCmd::angleCommandCallback(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
  cmd_.phi_cmd_ = msg->x;
  cmd_.theta_cmd_ = msg->y;
  cmd_.psi_cmd_ = msg->z;
}

void SpotMicroMotionCmd::velCommandCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  cmd_.x_vel_cmd_mps_ = msg->linear.x;
  cmd_.y_vel_cmd_mps_ = msg->linear.y;
  cmd_.yaw_rate_cmd_rps_ = msg->angular.z;
}

void SpotMicroMotionCmd::resetEventCommands()
{
  cmd_.resetEventCmds();
}

void SpotMicroMotionCmd::handleInputCommands()
{
  state_->handleInputCommands(sm_.getBodyState(), smnc_, cmd_, this, &body_state_cmd_);
}

void SpotMicroMotionCmd::changeState(std::unique_ptr<SpotMicroState> sms)
{
  state_ = std::move(sms);
  state_->init(sm_.getBodyState(), smnc_, cmd_, this);
  cmd_.resetAllCommands();
}

void SpotMicroMotionCmd::publishBodyState()
{
  if (body_state_msg_.data.size() < 18) {
    body_state_msg_.data.resize(18, 0.0f);
  }

  body_state_msg_.data[0] = body_state_cmd_.leg_feet_pos.right_back.x;
  body_state_msg_.data[1] = body_state_cmd_.leg_feet_pos.right_back.y;
  body_state_msg_.data[2] = body_state_cmd_.leg_feet_pos.right_back.z;

  body_state_msg_.data[3] = body_state_cmd_.leg_feet_pos.right_front.x;
  body_state_msg_.data[4] = body_state_cmd_.leg_feet_pos.right_front.y;
  body_state_msg_.data[5] = body_state_cmd_.leg_feet_pos.right_front.z;

  body_state_msg_.data[6] = body_state_cmd_.leg_feet_pos.left_front.x;
  body_state_msg_.data[7] = body_state_cmd_.leg_feet_pos.left_front.y;
  body_state_msg_.data[8] = body_state_cmd_.leg_feet_pos.left_front.z;

  body_state_msg_.data[9] = body_state_cmd_.leg_feet_pos.left_back.x;
  body_state_msg_.data[10] = body_state_cmd_.leg_feet_pos.left_back.y;
  body_state_msg_.data[11] = body_state_cmd_.leg_feet_pos.left_back.z;

  body_state_msg_.data[12] = body_state_cmd_.xyz_pos.x;
  body_state_msg_.data[13] = body_state_cmd_.xyz_pos.y;
  body_state_msg_.data[14] = body_state_cmd_.xyz_pos.z;

  body_state_msg_.data[15] = body_state_cmd_.euler_angs.phi;
  body_state_msg_.data[16] = body_state_cmd_.euler_angs.theta;
  body_state_msg_.data[17] = body_state_cmd_.euler_angs.psi;

  body_state_pub_->publish(body_state_msg_);
}

void SpotMicroMotionCmd::publishLcdMonitorData()
{
  lcd_state_string_msg_.data = getCurrentStateName();

  lcd_vel_cmd_msg_.linear.x = cmd_.getXSpeedCmd();
  lcd_vel_cmd_msg_.linear.y = cmd_.getYSpeedCmd();
  lcd_vel_cmd_msg_.angular.z = cmd_.getYawRateCmd();

  lcd_angle_cmd_msg_.x = cmd_.getPhiCmd();
  lcd_angle_cmd_msg_.y = cmd_.getThetaCmd();
  lcd_angle_cmd_msg_.z = cmd_.getPsiCmd();

  lcd_state_pub_->publish(lcd_state_string_msg_);
  lcd_vel_cmd_pub_->publish(lcd_vel_cmd_msg_);
  lcd_angle_cmd_pub_->publish(lcd_angle_cmd_msg_);
}

void SpotMicroMotionCmd::publishStaticTransforms()
{
  geometry_msgs::msg::TransformStamped tr_stamped;
  const rclcpp::Time stamp = this->now();

  tr_stamped = createTransform("base_link", "front_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, stamp);
  static_transform_br_->sendTransform(tr_stamped);

  tr_stamped = createTransform("base_link", "rear_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, stamp);
  static_transform_br_->sendTransform(tr_stamped);

  const float x_offset = smnc_.lidar_x_pos;
  const float y_offset = smnc_.lidar_y_pos;
  const float z_offset = smnc_.lidar_z_pos;
  const float yaw_angle = smnc_.lidar_yaw_angle * static_cast<float>(M_PI) / 180.0f;

  tr_stamped = createTransform("base_link", "lidar_link", x_offset, y_offset, z_offset, 0.0, 0.0, yaw_angle, stamp);
  static_transform_br_->sendTransform(tr_stamped);

  const VectorStringPairs leg_cover_pairs{
    {"front_left_leg_link", "front_left_leg_link_cover"},
    {"front_right_leg_link", "front_right_leg_link_cover"},
    {"rear_right_leg_link", "rear_right_leg_link_cover"},
    {"rear_left_leg_link", "rear_left_leg_link_cover"}};

  for (const auto& pair : leg_cover_pairs) {
    tr_stamped = createTransform(pair.first, pair.second, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, stamp);
    static_transform_br_->sendTransform(tr_stamped);
  }

  const VectorStringPairs foot_toe_pairs{
    {"front_left_foot_link", "front_left_toe_link"},
    {"front_right_foot_link", "front_right_toe_link"},
    {"rear_right_foot_link", "rear_right_toe_link"},
    {"rear_left_foot_link", "rear_left_toe_link"}};

  for (const auto& pair : foot_toe_pairs) {
    tr_stamped = createTransform(pair.first, pair.second, 0.0, 0.0, -0.13, 0.0, 0.0, 0.0, stamp);
    static_transform_br_->sendTransform(tr_stamped);
  }
}

void SpotMicroMotionCmd::publishDynamicTransforms()
{
  const rclcpp::Time stamp = this->now();
  LegsJointAngles joint_angs = sm_.getLegsJointAngles();

  geometry_msgs::msg::TransformStamped transform_stamped;
  Affine3d temp_trans;

  if (smnc_.publish_odom) {
    transform_stamped = eigAndFramesToTrans(getOdometryTransform(), "odom", "base_footprint", stamp);
    transform_br_->sendTransform(transform_stamped);
  }

  temp_trans = matrix4fToAffine3d(sm_.getBodyHt());
  temp_trans =
    AngleAxisd(M_PI / 2.0, Vector3d::UnitX()) *
    temp_trans *
    AngleAxisd(-M_PI / 2.0, Vector3d::UnitX());

  transform_stamped = eigAndFramesToTrans(temp_trans, "base_footprint", "base_link", stamp);
  transform_br_->sendTransform(transform_stamped);

  transform_stamped = createTransform("base_link", "front_right_shoulder_link", smnc_.smc.body_length / 2.0, -smnc_.smc.body_width / 2.0, 0.0, joint_angs.right_front.ang1, 0.0, 0.0, stamp);
  transform_br_->sendTransform(transform_stamped);
  transform_stamped = createTransform("front_right_shoulder_link", "front_right_leg_link", 0.0, -smnc_.smc.hip_link_length, 0.0, 0.0, -joint_angs.right_front.ang2, 0.0, stamp);
  transform_br_->sendTransform(transform_stamped);
  transform_stamped = createTransform("front_right_leg_link", "front_right_foot_link", 0.0, 0.0, -smnc_.smc.upper_leg_link_length, 0.0, -joint_angs.right_front.ang3, 0.0, stamp);
  transform_br_->sendTransform(transform_stamped);

  transform_stamped = createTransform("base_link", "rear_right_shoulder_link", -smnc_.smc.body_length / 2.0, -smnc_.smc.body_width / 2.0, 0.0, joint_angs.right_back.ang1, 0.0, 0.0, stamp);
  transform_br_->sendTransform(transform_stamped);
  transform_stamped = createTransform("rear_right_shoulder_link", "rear_right_leg_link", 0.0, -smnc_.smc.hip_link_length, 0.0, 0.0, -joint_angs.right_back.ang2, 0.0, stamp);
  transform_br_->sendTransform(transform_stamped);
  transform_stamped = createTransform("rear_right_leg_link", "rear_right_foot_link", 0.0, 0.0, -smnc_.smc.upper_leg_link_length, 0.0, -joint_angs.right_back.ang3, 0.0, stamp);
  transform_br_->sendTransform(transform_stamped);

  transform_stamped = createTransform("base_link", "front_left_shoulder_link", smnc_.smc.body_length / 2.0, smnc_.smc.body_width / 2.0, 0.0, -joint_angs.left_front.ang1, 0.0, 0.0, stamp);
  transform_br_->sendTransform(transform_stamped);
  transform_stamped = createTransform("front_left_shoulder_link", "front_left_leg_link", 0.0, smnc_.smc.hip_link_length, 0.0, 0.0, joint_angs.left_front.ang2, 0.0, stamp);
  transform_br_->sendTransform(transform_stamped);
  transform_stamped = createTransform("front_left_leg_link", "front_left_foot_link", 0.0, 0.0, -smnc_.smc.upper_leg_link_length, 0.0, joint_angs.left_front.ang3, 0.0, stamp);
  transform_br_->sendTransform(transform_stamped);

  transform_stamped = createTransform("base_link", "rear_left_shoulder_link", -smnc_.smc.body_length / 2.0, smnc_.smc.body_width / 2.0, 0.0, -joint_angs.left_back.ang1, 0.0, 0.0, stamp);
  transform_br_->sendTransform(transform_stamped);
  transform_stamped = createTransform("rear_left_shoulder_link", "rear_left_leg_link", 0.0, smnc_.smc.hip_link_length, 0.0, 0.0, joint_angs.left_back.ang2, 0.0, stamp);
  transform_br_->sendTransform(transform_stamped);
  transform_stamped = createTransform("rear_left_leg_link", "rear_left_foot_link", 0.0, 0.0, -smnc_.smc.upper_leg_link_length, 0.0, joint_angs.left_back.ang3, 0.0, stamp);
  transform_br_->sendTransform(transform_stamped);
}

void SpotMicroMotionCmd::integrateOdometry()
{
  const float dt = smnc_.dt;
  const float psi = robot_odometry_.euler_angs.psi;

  const float x_spd = cmd_.getXSpeedCmd();
  const float y_spd = -cmd_.getYSpeedCmd();
  const float yaw_rate = -cmd_.getYawRateCmd();

  const float x_dot = x_spd * std::cos(psi) - y_spd * std::sin(psi);
  const float y_dot = x_spd * std::sin(psi) + y_spd * std::cos(psi);
  const float yaw_dot = yaw_rate;

  robot_odometry_.xyz_pos.x += x_dot * dt;
  robot_odometry_.xyz_pos.y += y_dot * dt;
  robot_odometry_.euler_angs.psi += yaw_dot * dt;
}

Affine3d SpotMicroMotionCmd::getOdometryTransform()
{
  Translation3d translation(robot_odometry_.xyz_pos.x, robot_odometry_.xyz_pos.y, 0.0);
  AngleAxisd rotation(robot_odometry_.euler_angs.psi, Vector3d::UnitZ());
  return translation * rotation;
}

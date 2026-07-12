#pragma once

#ifndef SPOT_MICRO_MOTION_CMD
#define SPOT_MICRO_MOTION_CMD

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <eigen3/Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "i2c_pwm_board_msgs/msg/servo.hpp"
#include "i2c_pwm_board_msgs/msg/servo_array.hpp"
#include "i2c_pwm_board_msgs/msg/servo_config.hpp"
#include "i2c_pwm_board_msgs/srv/servos_config.hpp"

#include "command.h"
#include "spot_micro_kinematics/spot_micro_kinematics.h"
#include "spot_micro_state.h"


struct SpotMicroNodeConfig
{
  smk::SpotMicroConfig smc;

  float default_stand_height{0.0f};
  float stand_front_x_offset{0.0f};
  float stand_back_x_offset{0.0f};
  float lie_down_height{0.0f};
  float lie_down_feet_x_offset{0.0f};

  int num_servos{12};
  float servo_max_angle_deg{90.0f};

  std::map<std::string, std::map<std::string, float>> servo_config;

  float dt{0.02f};
  float transit_tau{0.1f};
  float transit_rl{1.0f};
  float transit_angle_rl{1.0f};

  bool debug_mode{false};
  bool run_standalone{false};
  bool plot_mode{false};

  float max_fwd_velocity{0.0f};
  float max_side_velocity{0.0f};
  float max_yaw_rate{0.0f};

  float z_clearance{0.0f};
  float alpha{0.0f};
  float beta{0.0f};

  int num_phases{4};

  std::vector<int> rb_contact_phases;
  std::vector<int> rf_contact_phases;
  std::vector<int> lf_contact_phases;
  std::vector<int> lb_contact_phases;

  float overlap_time{0.0f};
  float swing_time{0.0f};

  int overlap_ticks{0};
  int swing_ticks{0};
  int stance_ticks{0};

  std::vector<int> phase_ticks;
  int phase_length{0};

  float foot_height_time_constant{0.0f};

  std::vector<int> body_shift_phases;

  float fwd_body_balance_shift{0.0f};
  float side_body_balance_shift{0.0f};
  float back_body_balance_shift{0.0f};

  bool publish_odom{false};

  float lidar_x_pos{0.0f};
  float lidar_y_pos{0.0f};
  float lidar_z_pos{0.0f};
  float lidar_yaw_angle{0.0f};
};


class SpotMicroMotionCmd : public rclcpp::Node
{
public:
  SpotMicroMotionCmd();
  ~SpotMicroMotionCmd() override;

  void runOnce();

  bool publishServoConfiguration();

  void setServoCommandMessageData();

  void publishServoProportionalCommand();

  void publishZeroServoAbsoluteCommand();

  SpotMicroNodeConfig getNodeConfig();

  smk::LegsFootPos getNeutralStance();

  smk::LegsFootPos getLieDownStance();

  void commandIdle();

  std::string getCurrentStateName();

private:
  friend class SpotMicroState;

  std::unique_ptr<SpotMicroState> state_;

  Command cmd_;

  smk::SpotMicroKinematics sm_;

  SpotMicroNodeConfig smnc_;

  smk::BodyState body_state_cmd_;

  smk::BodyState robot_odometry_;

  std::map<std::string, float> servo_cmds_rad_ = {
    {"RF_3", 0.0f}, {"RF_2", 0.0f}, {"RF_1", 0.0f},
    {"RB_3", 0.0f}, {"RB_2", 0.0f}, {"RB_1", 0.0f},
    {"LB_3", 0.0f}, {"LB_2", 0.0f}, {"LB_1", 0.0f},
    {"LF_3", 0.0f}, {"LF_2", 0.0f}, {"LF_1", 0.0f}
  };

  void readInConfigParameters();

  i2c_pwm_board_msgs::msg::ServoArray servo_array_;

  i2c_pwm_board_msgs::msg::ServoArray servo_array_absolute_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stand_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr idle_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr walk_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr body_angle_cmd_sub_;

  rclcpp::Publisher<i2c_pwm_board_msgs::msg::ServoArray>::SharedPtr servos_absolute_pub_;
  rclcpp::Publisher<i2c_pwm_board_msgs::msg::ServoArray>::SharedPtr servos_proportional_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr body_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr lcd_vel_cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr lcd_angle_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lcd_state_pub_;

  rclcpp::Client<i2c_pwm_board_msgs::srv::ServosConfig>::SharedPtr servos_config_client_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_br_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_transform_br_;

  std_msgs::msg::Float32MultiArray body_state_msg_;

  std_msgs::msg::String lcd_state_string_msg_;
  geometry_msgs::msg::Twist lcd_vel_cmd_msg_;
  geometry_msgs::msg::Vector3 lcd_angle_cmd_msg_;

  void standCommandCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void idleCommandCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void walkCommandCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void angleCommandCallback(const geometry_msgs::msg::Vector3::SharedPtr msg);
  void velCommandCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  void resetEventCommands();

  void handleInputCommands();

  void changeState(std::unique_ptr<SpotMicroState> sms);

  void publishBodyState();

  void publishLcdMonitorData();

  void publishStaticTransforms();

  void publishDynamicTransforms();

  void integrateOdometry();

  Eigen::Affine3d getOdometryTransform();
};

#endif
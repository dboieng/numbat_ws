#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include "smov/base.h"
#include "smov/executable.h"
#include "smov/trigonometry.h"

/**
 * @brief High-level operating mode of the robot.
 */
enum Mode {
  SITTING_DOWN = 0,
  STANDING = 1,
  WAKING_UP = 2,
  WALKING = 3,
  TURNING_RIGHT = 4,
  TURNING_LEFT = 5
};

/**
 * @brief Velocity commands consumed by the gait controller.
 *
 * Linear velocities must use the same distance unit as the foot coordinates.
 *
 * If the foot coordinates are centimeters:
 *   - forward_velocity is centimeters/second.
 *   - lateral_velocity is centimeters/second.
 *
 * yaw_rate is always radians/second.
 */
struct GaitCommand {
  float forward_velocity = 0.0f;
  float lateral_velocity = 0.0f;
  float yaw_rate = 0.0f;
};

/**
 * @brief Eight-phase crawl-gait controller.
 *
 * Assumed coordinate system:
 *   - +x: forward
 *   - +y: downward / leg extension
 *   - +z: lateral
 *
 * Therefore, reducing y lifts a foot.
 *
 * Assumed leg mapping:
 *   - leg 1 / coord1: front-left
 *   - leg 2 / coord2: front-right
 *   - leg 3 / coord3: rear-right
 *   - leg 4 / coord4: rear-left
 */
class ForwardMotion {
 public:
  STATE_CLASS(ForwardMotion)

  // STATE_CLASS already declares on_start(), on_loop(), and on_quit().

  // ---------------------------------------------------------------------------
  // Existing robot-state functions
  // ---------------------------------------------------------------------------

  /**
   * @brief Moves all legs to their configured neutral positions.
   */
  void stabilize_legs();

  /**
   * @brief Logs current mode, commands, phase, and foot coordinates.
   */
  void output_coordinates();

  /**
   * @brief Executes the existing wake-up servo sequence.
   */
  void wake_up();

  // ---------------------------------------------------------------------------
  // Gait lifecycle
  // ---------------------------------------------------------------------------

  /**
   * @brief Initializes and begins the gait cycle.
   *
   * This resets the phase clock but does not modify the velocity command.
   */
  void start_gait();

  /**
   * @brief Stops the gait and returns to the neutral stance.
   */
  void stop_gait();

  /**
   * @brief Runs one timestep of the gait controller.
   */
  void step_gait();

  /**
   * @brief Updates phase_index and subphase_ticks using the gait tick counter.
   */
  void update_phase_data();

  // ---------------------------------------------------------------------------
  // Per-leg gait controllers
  // ---------------------------------------------------------------------------

  /**
   * @brief Updates a planted foot opposite the commanded body motion.
   *
   * @param foot Current foot coordinate, updated in place.
   * @param neutral_foot Neutral coordinate used to restore ground height.
   */
  void apply_stance_controller(
    smov::Vector3& foot,
    const smov::Vector3& neutral_foot);

  /**
   * @brief Moves a swinging foot toward its predicted touchdown point.
   *
   * @param foot Current foot coordinate, updated in place.
   * @param neutral_foot Neutral coordinate used to calculate touchdown.
   */
  void apply_swing_controller(
    smov::Vector3& foot,
    const smov::Vector3& neutral_foot);

  /**
   * @brief Calculates the triangular swing-foot clearance.
   *
   * @param swing_progress Swing completion from 0.0 to 1.0.
   * @return Foot-clearance distance.
   */
  float calculate_swing_height(float swing_progress) const;

  /**
   * @brief Calculates a foot touchdown target from velocity and yaw commands.
   *
   * @param neutral_foot Neutral coordinate for the selected foot.
   * @return Predicted touchdown coordinate.
   */
  smov::Vector3 calculate_touchdown(
    const smov::Vector3& neutral_foot) const;

  /**
   * @brief Returns whether a leg is in swing during the current phase.
   *
   * @param leg_index Zero-based leg index in the range 0-3.
   */
  bool is_leg_in_swing(int leg_index) const;

  // ---------------------------------------------------------------------------
  // Command and output helpers
  // ---------------------------------------------------------------------------

  /**
   * @brief Publishes all four current foot coordinates to inverse kinematics.
   */
  void publish_leg_coordinates();

  /**
   * @brief Sets a requested gait command while applying configured limits.
   *
   * Linear velocities use the same distance unit as the foot coordinates.
   * Yaw rate uses radians/second.
   */
  void set_gait_command(
    float forward_velocity,
    float lateral_velocity,
    float yaw_rate);

  /**
   * @brief Clears commanded movement and requests a safe gait stop.
   */
  void request_gait_stop();

  /**
   * @brief Creates the ROS 2 /cmd_vel subscriber.
   *
   * ForwardMotion is not an rclcpp::Node, so the subscriber is created on
   * velocity_command_node and spun from on_loop().
   */
  void create_velocity_command_subscriber();

  /**
   * @brief Handles ROS 2 geometry_msgs/Twist velocity commands.
   *
   * Standard ROS linear velocities are meters/second. When
   * cmd_vel_linear_scale is 100.0, they are converted to centimeters/second.
   */
  void velocity_command_callback(
    geometry_msgs::msg::Twist::ConstSharedPtr msg);

  /**
   * @brief Returns true when all four feet are currently in stance.
   */
  bool all_feet_in_stance() const;

  /**
   * @brief Returns a printable name for the current robot mode.
   */
  const char* mode_to_string() const;

  /**
   * @brief Restricts a foot target to the configured IK workspace.
   */
  void clamp_foot_target(smov::Vector3& foot) const;

  // ---------------------------------------------------------------------------
  // Timing configuration
  // ---------------------------------------------------------------------------

  /**
   * The node declaration at the bottom of walking_gait.cpp uses a 50 ms period.
   */
  float dt = 0.05f;

  /**
   * Duration of one phase in seconds.
   *
   * The eight-phase gait contains eight equally sized phases.
   */
  float phase_duration = 0.36f;

  int num_phases = 8;

  /**
   * Number of gait-controller updates since gait start.
   */
  int ticks = 0;

  /**
   * Current gait phase, in the range 0-7.
   */
  int phase_index = 0;

  /**
   * Current tick within the active gait phase.
   */
  int subphase_ticks = 0;

  /**
   * Derived as round(phase_duration / dt).
   */
  int phase_ticks = 1;

  /**
   * Derived as num_phases * phase_ticks.
   */
  int gait_cycle_ticks = 8;

  /**
   * Each leg spends seven phases in stance in the eight-phase gait.
   */
  int stance_ticks = 7;

  // ---------------------------------------------------------------------------
  // Neutral stance
  // ---------------------------------------------------------------------------

  /**
   * Foot coordinates appear to use centimeters.
   *
   * These values should match the real robot's stable standing position.
   */
  float default_x = 3.0f;
  float default_y = 19.0f;
  float default_z = 5.0f;

  /**
   * Additional neutral y offset for rear legs.
   */
  float back_leg_gap = 1.0f;

  // ---------------------------------------------------------------------------
  // Gait tuning
  // ---------------------------------------------------------------------------

  /**
   * Maximum amount by which y is reduced during swing.
   *
   * This assumes smaller y lifts the foot.
   */
  float foot_clearance = 7.0f;

  /**
   * Translational touchdown prediction factor.
   *
   * A value of 0.5 centers stance travel around the neutral position.
   */
  float alpha = 0.9f;

  /**
   * Rotational touchdown prediction factor.
   *
   * A value of 0.5 centers yaw-induced stance travel around neutral.
   */
  float beta = 0.9f;

  /**
   * Command limits in coordinate-units/second and radians/second.
   *
   * Start with conservative limits and increase only after testing.
   */
  float max_forward_velocity = 5.0f;
  float max_lateral_velocity = 2.0f;
  float max_yaw_rate = 0.15f;

  /**
   * Converts standard ROS cmd_vel linear commands into coordinate units.
   *
   * Use:
   *   100.0 when foot coordinates are centimeters.
   *   1.0 when foot coordinates are meters.
   */
  float cmd_vel_linear_scale = 100.0f;

  /**
   * Commands below these magnitudes are treated as zero.
   */
  float linear_command_deadband = 0.01f;
  float angular_command_deadband = 0.001f;

  // ---------------------------------------------------------------------------
  // Foot workspace safety limits
  // ---------------------------------------------------------------------------

  /**
   * Update these values to match the reachable workspace of your IK system.
   */
  float min_foot_x = -5.0f;
  float max_foot_x = 9.0f;

  float min_foot_y = 12.0f;
  float max_foot_y = 28.0f;

  float min_foot_z = 1.0f;
  float max_foot_z = 11.0f;

  // ---------------------------------------------------------------------------
  // Eight-phase crawl gait
  // ---------------------------------------------------------------------------

  /**
   * true  = swing
   * false = stance
   *
   * Assumed leg mapping:
   *   index 0: front-left  / coord1
   *   index 1: front-right / coord2
   *   index 2: rear-right  / coord3
   *   index 3: rear-left   / coord4
   *
   * Gait sequence:
   *   phase 0: all stance
   *   phase 1: rear-right swing
   *   phase 2: all stance
   *   phase 3: front-right swing
   *   phase 4: all stance
   *   phase 5: rear-left swing
   *   phase 6: all stance
   *   phase 7: front-left swing
   */
  std::array<std::array<bool, 8>, 4> swing_phases{{
    // Front-left / coord1
    {{false, false, false, false, false, false, false, true}},

    // Front-right / coord2
    {{false, false, false, true, false, false, false, false}},

    // Rear-right / coord3
    {{false, true, false, false, false, false, false, false}},

    // Rear-left / coord4
    {{false, false, false, false, false, true, false, false}}
  }};

  // ---------------------------------------------------------------------------
  // Current gait and robot state
  // ---------------------------------------------------------------------------

  Mode mode = SITTING_DOWN;

  GaitCommand gait_command;

  bool gait_running = false;
  bool request_to_stop_walk = false;

  // ---------------------------------------------------------------------------
  // IK and servo output
  // ---------------------------------------------------------------------------

  smov::TrigonometryState trig = smov::TrigonometryState(
    &front_servos,
    &back_servos,
    &front_state_publisher,
    &back_state_publisher,
    &upper_leg_length,
    &lower_leg_length,
    &hip_body_distance
  );

  // Current commanded foot positions.
  smov::Vector3 coord1;
  smov::Vector3 coord2;
  smov::Vector3 coord3;
  smov::Vector3 coord4;

  // Fixed neutral foot positions.
  smov::Vector3 neutral_coord1;
  smov::Vector3 neutral_coord2;
  smov::Vector3 neutral_coord3;
  smov::Vector3 neutral_coord4;

  // ---------------------------------------------------------------------------
  // ROS 2 interfaces
  // ---------------------------------------------------------------------------

  /**
   * Small ROS 2 node used only for receiving /cmd_vel.
   */
  rclcpp::Node::SharedPtr velocity_command_node;

  /**
   * Kept alive for the lifetime of this state. In ROS 2, subscriptions must
   * be stored; otherwise they are destroyed immediately after creation.
   */
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
    velocity_command_subscriber;

  // ---------------------------------------------------------------------------
  // Legacy members
  // ---------------------------------------------------------------------------

  /**
   * These are retained so your existing logging and other source files can
   * continue to compile during migration.
   *
   * They are not required by the phase-based gait controller and can be
   * removed after updating walking_gait.cpp.
   */
  bool leg1_motion_done = true;
  bool leg2_motion_done = true;
  bool leg3_motion_done = true;
  bool leg4_motion_done = true;

  bool has_finished_walk = false;
  bool done_once = false;
  bool has_finished_turn = false;

  float i1 = 0.0f;
  float i2 = 0.0f;
  float i3 = 0.0f;
  float i4 = 0.0f;

  int auto_walk_delay_seconds = 10;

  // Terminal configuration used by on_start() and on_quit().
  struct termios old_chars {};
  struct termios new_chars {};
};

#include "walking_gait.h"

#include <algorithm>
#include <cmath>
#include <functional>

// =============================================================================
// Lifecycle
// =============================================================================

void ForwardMotion::on_start()
{
  // Save the current terminal configuration.
  tcgetattr(STDIN_FILENO, &old_chars);

  // Configure non-blocking keyboard input.
  fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);

  new_chars = old_chars;
  new_chars.c_lflag &= ~ICANON;
  new_chars.c_lflag &= ~ECHO;

  tcsetattr(STDIN_FILENO, TCSANOW, &new_chars);

  // Derive gait timing values.
  phase_ticks = std::max(
    1,
    static_cast<int>(std::round(phase_duration / dt))
  );

  gait_cycle_ticks = num_phases * phase_ticks;
  stance_ticks = 7 * phase_ticks;

  ticks = 0;
  phase_index = 0;
  subphase_ticks = 0;

  gait_running = false;
  request_to_stop_walk = false;

  /*
   * If STATE_CLASS exposes create_subscription(), uncomment this block.
   *
   * Otherwise, create the subscription through the underlying ROS 2 node and
   * connect it to velocity_command_callback().
   */
  /*
  velocity_command_subscriber =
    this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      rclcpp::QoS(1),
      std::bind(
        &ForwardMotion::velocity_command_callback,
        this,
        std::placeholders::_1
      )
    );
  */

  create_velocity_command_subscriber();

  RCLCPP_INFO(
    rclcpp::get_logger("walking_gait"),
    "Initialized gait: dt=%.3f, phase_ticks=%d, cycle_ticks=%d",
    dt,
    phase_ticks,
    gait_cycle_ticks
  );
}

void ForwardMotion::on_loop()
{
  if (velocity_command_node) {
    rclcpp::spin_some(velocity_command_node);
  }

  const int key = getchar();

  switch (key) {
    case 65:  // Up arrow: walk forward.
      if (mode == STANDING) {
        mode = WALKING;

        set_gait_command(
          2.0f,  // Forward speed in foot-coordinate units/second.
          0.0f,
          0.0f
        );

        start_gait();
      }
      break;

    case 66:  // Down arrow: safely stop.
      if (gait_running) {
        request_gait_stop();
      }
      break;

    case 67:  // Right arrow: turn right.
      if (mode == STANDING) {
        mode = TURNING_RIGHT;

        set_gait_command(
          0.0f,
          0.0f,
          -0.12f
        );

        start_gait();
      }
      break;

    case 68:  // Left arrow: turn left.
      if (mode == STANDING) {
        mode = TURNING_LEFT;

        set_gait_command(
          0.0f,
          0.0f,
          0.12f
        );

        start_gait();
      }
      break;

    case ' ':  // Space: wake up.
      if (mode == SITTING_DOWN) {
        mode = WAKING_UP;
        wake_up();
      }
      break;

    default:
      break;
  }

  if (gait_running) {
    step_gait();
  }

  /*
   * output_coordinates() produces many log messages. Consider calling this
   * less frequently after verifying the gait.
   */
  output_coordinates();
}

void ForwardMotion::on_quit()
{
  set_gait_command(0.0f, 0.0f, 0.0f);
  gait_running = false;

  velocity_command_subscriber.reset();
  velocity_command_node.reset();

  // Restore the original terminal configuration.
  tcsetattr(STDIN_FILENO, TCSANOW, &old_chars);
}

// =============================================================================
// Neutral stance
// =============================================================================

void ForwardMotion::stabilize_legs()
{
  neutral_coord1.x = default_x;
  neutral_coord1.y = default_y;
  neutral_coord1.z = default_z;

  neutral_coord2.x = default_x;
  neutral_coord2.y = default_y;
  neutral_coord2.z = default_z;

  neutral_coord3.x = default_x;
  neutral_coord3.y = default_y + back_leg_gap;
  neutral_coord3.z = default_z;

  neutral_coord4.x = default_x;
  neutral_coord4.y = default_y + back_leg_gap;
  neutral_coord4.z = default_z;

  coord1 = neutral_coord1;
  coord2 = neutral_coord2;
  coord3 = neutral_coord3;
  coord4 = neutral_coord4;

  publish_leg_coordinates();

  RCLCPP_INFO(
    rclcpp::get_logger("walking_gait"),
    "Set neutral stance"
  );
}

// =============================================================================
// Gait lifecycle
// =============================================================================

void ForwardMotion::start_gait()
{
  if (gait_running) {
    return;
  }

  phase_ticks = std::max(
    1,
    static_cast<int>(std::round(phase_duration / dt))
  );

  gait_cycle_ticks = num_phases * phase_ticks;
  stance_ticks = 7 * phase_ticks;

  ticks = 0;
  phase_index = 0;
  subphase_ticks = 0;

  request_to_stop_walk = false;
  gait_running = true;

  RCLCPP_INFO(
    rclcpp::get_logger("walking_gait"),
    "Started gait: phase_ticks=%d, cycle_ticks=%d, stance_ticks=%d",
    phase_ticks,
    gait_cycle_ticks,
    stance_ticks
  );
}

void ForwardMotion::stop_gait()
{
  gait_running = false;
  request_to_stop_walk = false;

  set_gait_command(0.0f, 0.0f, 0.0f);

  ticks = 0;
  phase_index = 0;
  subphase_ticks = 0;

  mode = STANDING;

  /*
   * This immediately returns every foot to neutral.
   *
   * For hardware testing, only stop while all feet are in stance. If this
   * movement is still too abrupt, replace stabilize_legs() with a filtered
   * transition-to-neutral state.
   */
  stabilize_legs();

  RCLCPP_INFO(
    rclcpp::get_logger("walking_gait"),
    "Stopped gait and returned to neutral stance"
  );
}

void ForwardMotion::step_gait()
{
  if (!gait_running) {
    return;
  }

  update_phase_data();

  if (is_leg_in_swing(0)) {
    apply_swing_controller(coord1, neutral_coord1);
  } else {
    apply_stance_controller(coord1, neutral_coord1);
  }

  if (is_leg_in_swing(1)) {
    apply_swing_controller(coord2, neutral_coord2);
  } else {
    apply_stance_controller(coord2, neutral_coord2);
  }

  if (is_leg_in_swing(2)) {
    apply_swing_controller(coord3, neutral_coord3);
  } else {
    apply_stance_controller(coord3, neutral_coord3);
  }

  if (is_leg_in_swing(3)) {
    apply_swing_controller(coord4, neutral_coord4);
  } else {
    apply_stance_controller(coord4, neutral_coord4);
  }

  clamp_foot_target(coord1);
  clamp_foot_target(coord2);
  clamp_foot_target(coord3);
  clamp_foot_target(coord4);

  publish_leg_coordinates();

  /*
   * Stop only during an all-stance phase. Check before incrementing ticks,
   * because all_feet_in_stance() describes the phase just executed.
   */
  if (request_to_stop_walk && all_feet_in_stance()) {
    stop_gait();
    return;
  }

  ++ticks;
}

void ForwardMotion::update_phase_data()
{
  if (phase_ticks <= 0 || gait_cycle_ticks <= 0) {
    phase_index = 0;
    subphase_ticks = 0;
    return;
  }

  const int cycle_tick = ticks % gait_cycle_ticks;

  phase_index = cycle_tick / phase_ticks;
  subphase_ticks = cycle_tick % phase_ticks;

  phase_index = std::clamp(
    phase_index,
    0,
    num_phases - 1
  );
}

// =============================================================================
// Stance controller
// =============================================================================

void ForwardMotion::apply_stance_controller(
  smov::Vector3& foot,
  const smov::Vector3& neutral_foot)
{
  /*
   * A planted foot moves opposite the commanded body translation.
   *
   * If the body command is forward, the foot moves backward relative to the
   * body, keeping the foot approximately stationary in world coordinates.
   */
  foot.x -= gait_command.forward_velocity * dt;
  foot.z -= gait_command.lateral_velocity * dt;

  /*
   * Apply incremental yaw rotation in the horizontal x/z plane.
   *
   * If the turn direction is reversed on the physical robot, negate theta.
   */
  const float theta = gait_command.yaw_rate * dt;
  const float cosine = std::cos(theta);
  const float sine = std::sin(theta);

  const float old_x = foot.x;
  const float old_z = foot.z;

  foot.x = cosine * old_x - sine * old_z;
  foot.z = sine * old_x + cosine * old_z;

  // Stance feet remain on the ground.
  foot.y = neutral_foot.y;
}

// =============================================================================
// Swing controller
// =============================================================================

void ForwardMotion::apply_swing_controller(
  smov::Vector3& foot,
  const smov::Vector3& neutral_foot)
{
  if (phase_ticks <= 0) {
    foot = neutral_foot;
    return;
  }

  /*
   * Use subphase_ticks + 1 so the final swing update reaches progress 1.0.
   */
  const float swing_progress = std::clamp(
    static_cast<float>(subphase_ticks + 1) /
      static_cast<float>(phase_ticks),
    0.0f,
    1.0f
  );

  const smov::Vector3 touchdown =
    calculate_touchdown(neutral_foot);

  const int ticks_remaining =
    std::max(1, phase_ticks - subphase_ticks);

  /*
   * Move one remaining-tick fraction toward touchdown.
   *
   * If the touchdown target remains constant, the foot reaches it exactly on
   * the final swing update.
   */
  foot.x +=
    (touchdown.x - foot.x) /
    static_cast<float>(ticks_remaining);

  foot.z +=
    (touchdown.z - foot.z) /
    static_cast<float>(ticks_remaining);

  const float height =
    calculate_swing_height(swing_progress);

  /*
   * Assumption: reducing y lifts the foot.
   *
   * If the robot pushes the foot downward instead, change '-' to '+'.
   */
  foot.y = neutral_foot.y - height;
}

float ForwardMotion::calculate_swing_height(
  float swing_progress) const
{
  swing_progress = std::clamp(
    swing_progress,
    0.0f,
    1.0f
  );

  if (swing_progress < 0.5f) {
    return 2.0f * swing_progress * foot_clearance;
  }

  return 2.0f * (1.0f - swing_progress) * foot_clearance;
}

smov::Vector3 ForwardMotion::calculate_touchdown(
  const smov::Vector3& neutral_foot) const
{
  smov::Vector3 touchdown = neutral_foot;

  const float stance_time =
    static_cast<float>(stance_ticks) * dt;

  /*
   * Predict translation accumulated while the foot will be in stance.
   *
   * alpha=0.5 places touchdown approximately half of the stance travel ahead
   * of neutral.
   */
  touchdown.x +=
    alpha * stance_time * gait_command.forward_velocity;

  touchdown.z +=
    alpha * stance_time * gait_command.lateral_velocity;

  /*
   * Predict rotational travel during stance.
   *
   * beta=0.5 centers the rotational stance trajectory around neutral.
   */
  const float theta =
    -beta * stance_time * gait_command.yaw_rate;

  const float cosine = std::cos(theta);
  const float sine = std::sin(theta);

  const float old_x = touchdown.x;
  const float old_z = touchdown.z;

  touchdown.x = cosine * old_x - sine * old_z;
  touchdown.z = sine * old_x + cosine * old_z;

  return touchdown;
}

bool ForwardMotion::is_leg_in_swing(int leg_index) const
{
  if (leg_index < 0 || leg_index >= 4) {
    return false;
  }

  if (phase_index < 0 || phase_index >= num_phases) {
    return false;
  }

  return swing_phases[leg_index][phase_index];
}

bool ForwardMotion::all_feet_in_stance() const
{
  return
    !is_leg_in_swing(0) &&
    !is_leg_in_swing(1) &&
    !is_leg_in_swing(2) &&
    !is_leg_in_swing(3);
}

// =============================================================================
// Command handling
// =============================================================================

void ForwardMotion::set_gait_command(
  float forward_velocity,
  float lateral_velocity,
  float yaw_rate)
{
  gait_command.forward_velocity = std::clamp(
    forward_velocity,
    -max_forward_velocity,
    max_forward_velocity
  );

  gait_command.lateral_velocity = std::clamp(
    lateral_velocity,
    -max_lateral_velocity,
    max_lateral_velocity
  );

  gait_command.yaw_rate = std::clamp(
    yaw_rate,
    -max_yaw_rate,
    max_yaw_rate
  );

  if (
    std::abs(gait_command.forward_velocity) <
    linear_command_deadband
  ) {
    gait_command.forward_velocity = 0.0f;
  }

  if (
    std::abs(gait_command.lateral_velocity) <
    linear_command_deadband
  ) {
    gait_command.lateral_velocity = 0.0f;
  }

  if (
    std::abs(gait_command.yaw_rate) <
    angular_command_deadband
  ) {
    gait_command.yaw_rate = 0.0f;
  }
}

void ForwardMotion::request_gait_stop()
{
  set_gait_command(0.0f, 0.0f, 0.0f);
  request_to_stop_walk = true;

  RCLCPP_INFO(
    rclcpp::get_logger("walking_gait"),
    "Gait stop requested"
  );
}

void ForwardMotion::create_velocity_command_subscriber()
{
  if (!velocity_command_node) {
    velocity_command_node =
      std::make_shared<rclcpp::Node>("walking_gait_cmd_vel");
  }

  velocity_command_subscriber =
    velocity_command_node->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      rclcpp::QoS(10),
      std::bind(
        &ForwardMotion::velocity_command_callback,
        this,
        std::placeholders::_1));

  RCLCPP_INFO(
    velocity_command_node->get_logger(),
    "Subscribed to /cmd_vel");
}

void ForwardMotion::velocity_command_callback(
  geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  if (!msg) {
    return;
  }

  /*
   * geometry_msgs/Twist normally uses meters/second for linear velocity.
   * cmd_vel_linear_scale=100 converts meters/second to centimeters/second.
   */
  const float forward_velocity =
    static_cast<float>(msg->linear.x) *
    cmd_vel_linear_scale;

  const float lateral_velocity =
    static_cast<float>(msg->linear.y) *
    cmd_vel_linear_scale;

  const float yaw_rate =
    static_cast<float>(msg->angular.z);

  set_gait_command(
    forward_velocity,
    lateral_velocity,
    yaw_rate
  );

  const bool zero_command =
    gait_command.forward_velocity == 0.0f &&
    gait_command.lateral_velocity == 0.0f &&
    gait_command.yaw_rate == 0.0f;

  if (zero_command) {
    if (gait_running) {
      request_to_stop_walk = true;
    }

    return;
  }

  request_to_stop_walk = false;

  if (mode == SITTING_DOWN) {
    mode = WAKING_UP;
    wake_up();
    mode = STANDING;
  }

  if (!gait_running && mode == STANDING) {
    mode = WALKING;
    start_gait();
  }
}

// =============================================================================
// Output and safety helpers
// =============================================================================

void ForwardMotion::publish_leg_coordinates()
{
  trig.set_leg_to(1, coord1);
  trig.set_leg_to(2, coord2);
  trig.set_leg_to(3, coord3);
  trig.set_leg_to(4, coord4);
}

void ForwardMotion::clamp_foot_target(
  smov::Vector3& foot) const
{
  foot.x = std::clamp(
    foot.x,
    min_foot_x,
    max_foot_x
  );

  foot.y = std::clamp(
    foot.y,
    min_foot_y,
    max_foot_y
  );

  foot.z = std::clamp(
    foot.z,
    min_foot_z,
    max_foot_z
  );
}

const char* ForwardMotion::mode_to_string() const
{
  switch (mode) {
    case SITTING_DOWN:
      return "SITTING_DOWN";

    case STANDING:
      return "STANDING";

    case WAKING_UP:
      return "WAKING_UP";

    case WALKING:
      return "WALKING";

    case TURNING_RIGHT:
      return "TURNING_RIGHT";

    case TURNING_LEFT:
      return "TURNING_LEFT";

    default:
      return "UNKNOWN";
  }
}

void ForwardMotion::output_coordinates()
{
  RCLCPP_INFO(
    rclcpp::get_logger("walking_gait"),
    "Mode=%s gait=%s phase=%d/%d subphase=%d/%d "
    "command=(forward=%.3f lateral=%.3f yaw=%.3f)",
    mode_to_string(),
    gait_running ? "RUNNING" : "STOPPED",
    phase_index,
    num_phases - 1,
    subphase_ticks,
    phase_ticks - 1,
    gait_command.forward_velocity,
    gait_command.lateral_velocity,
    gait_command.yaw_rate
  );

  RCLCPP_INFO(
    rclcpp::get_logger("walking_gait"),
    "Leg 1%s: (%.3f, %.3f, %.3f)",
    is_leg_in_swing(0) ? " [SWING]" : " [STANCE]",
    coord1.x,
    coord1.y,
    coord1.z
  );

  RCLCPP_INFO(
    rclcpp::get_logger("walking_gait"),
    "Leg 2%s: (%.3f, %.3f, %.3f)",
    is_leg_in_swing(1) ? " [SWING]" : " [STANCE]",
    coord2.x,
    coord2.y,
    coord2.z
  );

  RCLCPP_INFO(
    rclcpp::get_logger("walking_gait"),
    "Leg 3%s: (%.3f, %.3f, %.3f)",
    is_leg_in_swing(2) ? " [SWING]" : " [STANCE]",
    coord3.x,
    coord3.y,
    coord3.z
  );

  RCLCPP_INFO(
    rclcpp::get_logger("walking_gait"),
    "Leg 4%s: (%.3f, %.3f, %.3f)",
    is_leg_in_swing(3) ? " [SWING]" : " [STANCE]",
    coord4.x,
    coord4.y,
    coord4.z
  );
}

// =============================================================================
// Existing wake-up sequence
// =============================================================================

void ForwardMotion::wake_up()
{
  RCLCPP_INFO(
    rclcpp::get_logger("walking_gait"),
    "[WAKING UP]"
  );

  /*
   * These delays preserve your existing behavior, but they block the state
   * loop. Consider replacing this sequence with a non-blocking timed state
   * machine after the gait is working.
   */
  smov::delay(800);

  for (int i = 0; i < 2; ++i) {
    front_servos.value[i] = 90.0f;
    front_servos.value[i + 2] = 55.0f;
    front_servos.value[i + 4] = 45.0f;

    back_servos.value[i] = 120.0f;
    back_servos.value[i + 2] = 150.0f;
    back_servos.value[i + 4] = 45.0f;
  }

  front_state_publisher->publish(front_servos);
  back_state_publisher->publish(back_servos);

  RCLCPP_INFO(
    rclcpp::get_logger("walking_gait"),
    "Executed first wake-up sequence"
  );

  smov::delay(800);

  for (int i = 0; i < 2; ++i) {
    back_servos.value[i] = 90.0f;
  }

  back_state_publisher->publish(back_servos);

  RCLCPP_INFO(
    rclcpp::get_logger("walking_gait"),
    "Executed second wake-up sequence"
  );

  smov::delay(800);

  for (int i = 0; i < 2; ++i) {
    front_servos.value[i + 2] = 45.0f;
    front_servos.value[i + 4] = 112.0f;
  }

  for (int i = 0; i < 2; ++i) {
    back_servos.value[i + 2] = 45.0f;
    back_servos.value[i + 4] = 115.0f;
  }

  front_state_publisher->publish(front_servos);
  back_state_publisher->publish(back_servos);

  RCLCPP_INFO(
    rclcpp::get_logger("walking_gait"),
    "Completed wake-up sequence"
  );

  smov::delay(2000);

  mode = STANDING;
  stabilize_legs();
}

// =============================================================================
// State registration
// =============================================================================

DECLARE_STATE_NODE_CLASS("walking_gait", ForwardMotion, 50ms)
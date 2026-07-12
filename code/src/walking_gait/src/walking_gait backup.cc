#include "walking_gait.h"

#include <chrono>
#include <cmath>

float ForwardMotion::curved(float x, float dist_from_origin, float gap) {
  float inside =
    curve_radius_squared -
    std::pow((2.0f * x - dist_from_origin), 2.0f);

  if (inside < 0.0f) {
    inside = 0.0f;
  }

  return -std::sqrt(inside) + default_y + gap;
}

const char* ForwardMotion::mode_to_string() {
  if (mode == SITTING_DOWN) {
    return "SITTING_DOWN";
  }

  if (mode == WAKING_UP) {
    return "WAKING_UP";
  }

  if (mode == STANDING) {
    return "STANDING";
  }

  if (mode == WALKING) {
    return "WALKING";
  }

  if (mode == TURNING_RIGHT) {
    return "TURNING_RIGHT";
  }

  if (mode == TURNING_LEFT) {
    return "TURNING_LEFT";
  }

  return "UNKNOWN";
}

void ForwardMotion::stabilize_legs() {
  coord1.x = default_x;
  coord1.y = default_y;
  coord1.z = default_z;

  coord2.x = default_x;
  coord2.y = default_y;
  coord2.z = default_z;

  coord3.x = default_x;
  coord3.y = default_y + back_leg_gap;
  coord3.z = default_z;

  coord4.x = default_x;
  coord4.y = default_y + back_leg_gap;
  coord4.z = default_z;

  trig.set_leg_to(1, coord1);
  trig.set_leg_to(2, coord2);
  trig.set_leg_to(3, coord3);
  trig.set_leg_to(4, coord4);

  RCLCPP_INFO(
    rclcpp::get_logger("walking_gait"),
    "Set default standing position."
  );

  RCLCPP_INFO(
    rclcpp::get_logger("walking_gait"),
    "Front legs target: (%f, %f, %f)",
    default_x,
    default_y,
    default_z
  );

  RCLCPP_INFO(
    rclcpp::get_logger("walking_gait"),
    "Back legs target: (%f, %f, %f)",
    default_x,
    default_y + back_leg_gap,
    default_z
  );
}

void ForwardMotion::output_coordinates() {
  RCLCPP_INFO(rclcpp::get_logger("walking_gait"), "\033[2J\033[;H");
  RCLCPP_INFO(rclcpp::get_logger("walking_gait"), "Robot is set to WALK STRAIGHT ONLY");

  RCLCPP_INFO(
    rclcpp::get_logger("walking_gait"),
    "Coordinates leg 1: (%f, %f, %f)",
    coord1.x,
    coord1.y,
    coord1.z
  );

  RCLCPP_INFO(
    rclcpp::get_logger("walking_gait"),
    "Coordinates leg 2: (%f, %f, %f)",
    coord2.x,
    coord2.y,
    coord2.z
  );

  RCLCPP_INFO(
    rclcpp::get_logger("walking_gait"),
    "Coordinates leg 3: (%f, %f, %f)",
    coord3.x,
    coord3.y,
    coord3.z
  );

  RCLCPP_INFO(
    rclcpp::get_logger("walking_gait"),
    "Coordinates leg 4: (%f, %f, %f)",
    coord4.x,
    coord4.y,
    coord4.z
  );

  RCLCPP_INFO(
    rclcpp::get_logger("walking_gait"),
    "Robot Mode:        %s",
    mode_to_string()
  );

  RCLCPP_INFO(
    rclcpp::get_logger("walking_gait"),
    "Motion done leg 1: %d",
    leg1_motion_done
  );

  RCLCPP_INFO(
    rclcpp::get_logger("walking_gait"),
    "Motion done leg 2: %d",
    leg2_motion_done
  );

  RCLCPP_INFO(
    rclcpp::get_logger("walking_gait"),
    "Motion done leg 3: %d",
    leg3_motion_done
  );

  RCLCPP_INFO(
    rclcpp::get_logger("walking_gait"),
    "Motion done leg 4: %d",
    leg4_motion_done
  );
}

void ForwardMotion::move_leg(
  int leg_id,
  smov::Vector3& coord,
  bool& current_leg_done,
  bool& next_leg_done,
  float y_gap
) {
  if (!current_leg_done) {
    if (coord.x > swing_end_x + step_done_threshold) {
      coord.x = smov::Functions::lerp(
        coord.x,
        swing_end_x,
        step_lerp_rate
      );

      coord.y = curved(
        coord.x,
        curve_origin_offset,
        y_gap
      );

      coord.z = default_z;

      trig.set_leg_to(leg_id, coord);
    } else {
      current_leg_done = true;

      if (!request_to_stop_walk) {
        next_leg_done = false;
      }
    }
  } else {
    if (coord.x < default_x - step_done_threshold) {
      coord.x = smov::Functions::lerp(
        coord.x,
        default_x,
        step_lerp_rate
      );

      coord.y = default_y + y_gap;
      coord.z = default_z;

      trig.set_leg_to(leg_id, coord);
    }
  }
}

void ForwardMotion::walk() {
  // Diagonal pair 1:
  // leg 1 + leg 4
  move_leg(
    1,
    coord1,
    leg1_motion_done,
    leg2_motion_done,
    0.0f
  );

  move_leg(
    4,
    coord4,
    leg4_motion_done,
    leg3_motion_done,
    back_leg_gap
  );

  // Diagonal pair 2:
  // leg 2 + leg 3
  move_leg(
    2,
    coord2,
    leg2_motion_done,
    leg1_motion_done,
    0.0f
  );

  move_leg(
    3,
    coord3,
    leg3_motion_done,
    leg4_motion_done,
    back_leg_gap
  );
}

void ForwardMotion::reset_walk_flags() {
  // Start walking with leg 1 and leg 4.
  leg1_motion_done = false;
  leg4_motion_done = false;

  // Keep leg 2 and leg 3 in support phase first.
  leg2_motion_done = true;
  leg3_motion_done = true;

  has_finished_walk = false;
}

bool ForwardMotion::all_legs_near_default_position() {
  return std::fabs(coord1.x - default_x) < 0.1f &&
         std::fabs(coord2.x - default_x) < 0.1f &&
         std::fabs(coord3.x - default_x) < 0.1f &&
         std::fabs(coord4.x - default_x) < 0.1f;
}

void ForwardMotion::turn() {
  // Turning is intentionally disabled.
}

void ForwardMotion::wake_up() {
  RCLCPP_INFO(rclcpp::get_logger("walking_gait"), "\033[2J\033[;H");
  RCLCPP_INFO(rclcpp::get_logger("walking_gait"), "[WAKING UP]");

  // Standing target positions.
  smov::Vector3 target1;
  target1.x = default_x;
  target1.y = default_y;
  target1.z = default_z;

  smov::Vector3 target2;
  target2.x = default_x;
  target2.y = default_y;
  target2.z = default_z;

  smov::Vector3 target3;
  target3.x = default_x;
  target3.y = default_y + back_leg_gap;
  target3.z = default_z;

  smov::Vector3 target4;
  target4.x = default_x;
  target4.y = default_y + back_leg_gap;
  target4.z = default_z;

  // Start close to the final standing pose, but lower.
  coord1.x = target1.x;
  coord1.y = target1.y;
  coord1.z = 0.0f;

  coord2.x = target2.x;
  coord2.y = target2.y;
  coord2.z = 0.0f;

  coord3.x = target3.x;
  coord3.y = target3.y;
  coord3.z = 0.0f;

  coord4.x = target4.x;
  coord4.y = target4.y;
  coord4.z = 0.0f;

  int steps = 80;
  int delay_ms = 30;

  // First: start the sequence with the back legs only.
  for (int i = 0; i <= steps / 4; i++) {
    float t = static_cast<float>(i) / static_cast<float>(steps / 4);

    coord3.z = smov::Functions::lerp(0.0f, target3.z, t);
    coord4.z = smov::Functions::lerp(0.0f, target4.z, t);

    trig.set_leg_to(3, coord3);
    trig.set_leg_to(4, coord4);

    smov::delay(delay_ms);
  }

  // Then: move both back and front legs together.
  for (int i = 0; i <= steps; i++) {
    float t = static_cast<float>(i) / static_cast<float>(steps);

    coord1.z = smov::Functions::lerp(0.0f, target1.z, t);
    coord2.z = smov::Functions::lerp(0.0f, target2.z, t);

    coord3.z = smov::Functions::lerp(target3.z * 0.25f, target3.z, t);
    coord4.z = smov::Functions::lerp(target4.z * 0.25f, target4.z, t);

    // Command back legs first, then front legs.
    trig.set_leg_to(3, coord3);
    trig.set_leg_to(4, coord4);
    trig.set_leg_to(1, coord1);
    trig.set_leg_to(2, coord2);

    smov::delay(delay_ms);
  }

  // Make sure final pose is exact.
  mode = STANDING;
  stabilize_legs();

  RCLCPP_INFO(
    rclcpp::get_logger("walking_gait"),
    "Robot slowly moved to stable standing pose, starting with back legs."
  );
}

void ForwardMotion::on_start() {
  // Getting the default config.
  tcgetattr(0, &old_chars);

  // Initializing the reader.
  fcntl(0, F_SETFL, O_NONBLOCK);

  new_chars = old_chars;
  new_chars.c_lflag &= ~ICANON;
  new_chars.c_lflag &= ~ECHO;

  tcsetattr(0, TCSANOW, &new_chars);
}

void ForwardMotion::on_loop() {
  static auto start_time = std::chrono::steady_clock::now();
  static bool auto_walk_started = false;

  auto now = std::chrono::steady_clock::now();

  auto elapsed =
    std::chrono::duration_cast<std::chrono::seconds>(
      now - start_time
    ).count();

  // Stage 1:
  // Wake up immediately.
  if (mode == SITTING_DOWN) {
    mode = WAKING_UP;
    wake_up();

    // Start the standing timer after the robot has finished standing up.
    start_time = std::chrono::steady_clock::now();
    auto_walk_started = false;

    return;
  }

  // Stage 2:
  // After standing for a while, start walking once.
  if (
    mode == STANDING &&
    !auto_walk_started &&
    elapsed >= auto_walk_delay_seconds
  ) {
    mode = WALKING;

    request_to_stop_walk = false;
    done_once = false;
    auto_walk_started = true;

    RCLCPP_INFO(
      rclcpp::get_logger("walking_gait"),
      "Standing complete. Starting walking gait."
    );
  }

  output_coordinates();

  if (mode == WALKING) {
    if (!done_once) {
      reset_walk_flags();
      done_once = true;
    }

    walk();

    if (
      request_to_stop_walk &&
      all_legs_near_default_position()
    ) {
      mode = STANDING;
      stabilize_legs();
    }
  }

  if (mode == STANDING) {
    done_once = false;
  }
}

void ForwardMotion::on_quit() {
  tcsetattr(STDIN_FILENO, TCSANOW, &old_chars);
}

DECLARE_STATE_NODE_CLASS("walking_gait", ForwardMotion, 50ms)
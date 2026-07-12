#pragma once

#include <cmath>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstdlib>

#include <rclcpp/rclcpp.hpp>

#include "smov/executable.h"
#include "smov/base.h"
#include "smov/trigonometry.h"

enum Mode {
  SITTING_DOWN = 0,
  STANDING = 1,
  WAKING_UP = 2,
  WALKING = 3,
  TURNING_RIGHT = 4,
  TURNING_LEFT = 5
};

class ForwardMotion {
 public:
  STATE_CLASS(ForwardMotion)

  // -----------------------------
  // Main gait functions
  // -----------------------------
  float curved(float x, float dist_from_origin, float gap);

  void stabilize_legs();
  void output_coordinates();

  void walk();
  void turn();
  void wake_up();

  // -----------------------------
  // Helper functions
  // -----------------------------
  void move_leg(
    int leg_id,
    smov::Vector3& coord,
    bool& current_leg_done,
    bool& next_leg_done,
    float y_gap
  );

  void reset_walk_flags();
  bool all_legs_near_default_position();
  const char* mode_to_string();

  // -----------------------------
  // Gait tuning values
  // -----------------------------

// [INFO] [1781328893.253834661] [walking_gait]: Coordinates leg 1: (3.500000, 23.000000, 5.000000)
// [INFO] [1781328893.254001697] [walking_gait]: Coordinates leg 2: (3.500000, 23.000000, 5.000000)
// [INFO] [1781328893.254194789] [walking_gait]: Coordinates leg 3: (3.500000, 26.000000, 5.000000)
// [INFO] [1781328893.254334289] [walking_gait]: Coordinates leg 4: (3.500000, 26.000000, 5.000000)
// float default_x = 3.5f;
// float default_y = 23.0f;
// float default_z = 5.0f;

  float default_x = 3.5f;
  float default_y = 15.0f;
  float default_z = 3.5f;
  float back_leg_gap = 1.5f;

  float swing_end_x = -1.5f;
  
  float step_lerp_rate = 0.10f;
  float step_done_threshold = 0.05f;

  float curve_radius_squared = 25.0f;
  float curve_origin_offset = 2.0f;

  int auto_walk_delay_seconds = 10;

  // -----------------------------
  // Robot mode
  // -----------------------------
  Mode mode = SITTING_DOWN;

  smov::TrigonometryState trig = smov::TrigonometryState(
    &front_servos,
    &back_servos,
    &front_state_publisher,
    &back_state_publisher,
    &upper_leg_length,
    &lower_leg_length,
    &hip_body_distance
  );

  smov::Vector3 coord1, coord2, coord3, coord4;

  // -----------------------------
  // Gait flags
  // -----------------------------
  bool leg1_motion_done = true;
  bool leg2_motion_done = true;
  bool leg3_motion_done = true;
  bool leg4_motion_done = true;

  bool has_finished_walk = false;
  bool done_once = false;
  bool request_to_stop_walk = false;
  bool has_finished_turn = false;

  float i1 = 0;
  float i2 = 0;
  float i3 = 0;
  float i4 = 0;

  // Used for reading terminal values.
  struct termios old_chars, new_chars;
};
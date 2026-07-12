#include <iostream>
#include <memory>

#include <eigen3/Eigen/Geometry>

#include "spot_micro_walk.h"

#include "spot_micro_transition_stand.h"
#include "spot_micro_motion_cmd.h"
#include "rate_limited_first_order_filter.h"

SpotMicroWalkState::SpotMicroWalkState()
{
  contact_feet_states_.right_back_in_swing = false;
  contact_feet_states_.right_front_in_swing = false;
  contact_feet_states_.left_front_in_swing = false;
  contact_feet_states_.left_back_in_swing = false;

  ticks_ = 0;
  phase_index_ = 0;
  subphase_ticks_ = 0;
}

SpotMicroWalkState::~SpotMicroWalkState()
{
}

void SpotMicroWalkState::handleInputCommands(
  const smk::BodyState& body_state,
  const SpotMicroNodeConfig& smnc,
  const Command& cmd,
  SpotMicroMotionCmd* smmc,
  smk::BodyState* body_state_cmd)
{
  if (smnc.debug_mode) {
    std::cout << "In Spot Micro Walk State" << std::endl;
  }

  // If stand command was received, transition to stand state.
  if (cmd.getStandCmd()) {
    changeState(smmc, std::make_unique<SpotMicroTransitionStandState>());
    return;
  }

  updatePhaseData();

  body_state_cmd->leg_feet_pos =
    stepGait(body_state, cmd, smnc, smmc->getNeutralStance());

  if (smnc_.num_phases == 8) {
    body_state_cmd->xyz_pos = stepBodyShift(body_state, cmd, smnc);
  }

  smmc->setServoCommandMessageData();
  smmc->publishServoProportionalCommand();

  ticks_ += 1;
}

void SpotMicroWalkState::init(
  const smk::BodyState& body_state,
  const SpotMicroNodeConfig& smnc,
  const Command& cmd,
  SpotMicroMotionCmd* smmc)
{
  (void)body_state;
  (void)cmd;

  smnc_ = smnc;

  cmd_state_.leg_feet_pos = smmc->getNeutralStance();

  cmd_state_.euler_angs.phi = 0.0f;
  cmd_state_.euler_angs.theta = 0.0f;
  cmd_state_.euler_angs.psi = 0.0f;

  cmd_state_.xyz_pos.x = 0.0f;
  cmd_state_.xyz_pos.y = smnc.default_stand_height;
  cmd_state_.xyz_pos.z = 0.0f;
}

void SpotMicroWalkState::updatePhaseData()
{
  const int phase_time = ticks_ % smnc_.phase_length;
  int phase_sum = 0;

  for (int i = 0; i < smnc_.num_phases; ++i) {
    phase_sum += smnc_.phase_ticks[i];

    if (phase_time < phase_sum) {
      phase_index_ = i;
      subphase_ticks_ = phase_time - phase_sum + smnc_.phase_ticks[i];
      break;
    }
  }

  contact_feet_states_.right_back_in_swing =
    smnc_.rb_contact_phases[phase_index_] == 0;

  contact_feet_states_.right_front_in_swing =
    smnc_.rf_contact_phases[phase_index_] == 0;

  contact_feet_states_.left_front_in_swing =
    smnc_.lf_contact_phases[phase_index_] == 0;

  contact_feet_states_.left_back_in_swing =
    smnc_.lb_contact_phases[phase_index_] == 0;
}

smk::LegsFootPos SpotMicroWalkState::stepGait(
  const smk::BodyState& body_state,
  const Command& cmd,
  const SpotMicroNodeConfig& smnc,
  const smk::LegsFootPos& default_stance_feet_pos)
{
  smk::LegsFootPos new_feet_pos;

  for (int i = 0; i < 4; ++i) {
    bool in_swing = false;
    smk::Point foot_pos;
    smk::Point default_stance_foot_pos;

    if (i == 0) {
      in_swing = contact_feet_states_.right_back_in_swing;
      foot_pos = body_state.leg_feet_pos.right_back;
      default_stance_foot_pos = default_stance_feet_pos.right_back;
    } else if (i == 1) {
      in_swing = contact_feet_states_.right_front_in_swing;
      foot_pos = body_state.leg_feet_pos.right_front;
      default_stance_foot_pos = default_stance_feet_pos.right_front;
    } else if (i == 2) {
      in_swing = contact_feet_states_.left_front_in_swing;
      foot_pos = body_state.leg_feet_pos.left_front;
      default_stance_foot_pos = default_stance_feet_pos.left_front;
    } else {
      in_swing = contact_feet_states_.left_back_in_swing;
      foot_pos = body_state.leg_feet_pos.left_back;
      default_stance_foot_pos = default_stance_feet_pos.left_back;
    }

    if (!in_swing) {
      foot_pos = stanceController(foot_pos, cmd, smnc);
    } else {
      const float swing_proportion =
        static_cast<float>(subphase_ticks_) / static_cast<float>(smnc.swing_ticks);

      foot_pos = swingLegController(
        foot_pos,
        cmd,
        smnc,
        swing_proportion,
        default_stance_foot_pos);
    }

    if (i == 0) {
      new_feet_pos.right_back = foot_pos;
    } else if (i == 1) {
      new_feet_pos.right_front = foot_pos;
    } else if (i == 2) {
      new_feet_pos.left_front = foot_pos;
    } else {
      new_feet_pos.left_back = foot_pos;
    }
  }

  return new_feet_pos;
}

smk::Point SpotMicroWalkState::stanceController(
  const smk::Point& foot_pos,
  const Command& cmd,
  const SpotMicroNodeConfig& smnc)
{
  using namespace Eigen;

  smk::Point new_foot_pos;

  const float dt = smnc.dt;
  const float h_tau = smnc.foot_height_time_constant;

  const Vector3f foot_pos_vec(foot_pos.x, foot_pos.y, foot_pos.z);

  const Vector3f delta_pos(
    -cmd.getXSpeedCmd() * dt,
    (1.0f / h_tau) * (0.0f - foot_pos.y) * dt,
    -cmd.getYSpeedCmd() * dt);

  Matrix3f rot_delta;
  rot_delta = AngleAxisf(cmd.getYawRateCmd() * dt, Vector3f::UnitY());

  const Vector3f new_foot_pos_vec = (rot_delta * foot_pos_vec) + delta_pos;

  new_foot_pos.x = new_foot_pos_vec[0];
  new_foot_pos.y = new_foot_pos_vec[1];
  new_foot_pos.z = new_foot_pos_vec[2];

  return new_foot_pos;
}

smk::Point SpotMicroWalkState::swingLegController(
  const smk::Point& foot_pos,
  const Command& cmd,
  const SpotMicroNodeConfig& smnc,
  float swing_proportion,
  const smk::Point& default_stance_foot_pos)
{
  using namespace Eigen;

  smk::Point new_foot_pos;

  const float dt = smnc.dt;
  const float alpha = smnc.alpha;
  const float beta = smnc.beta;
  const float stance_ticks = static_cast<float>(smnc.stance_ticks);

  const Vector3f default_stance_foot_pos_vec(
    default_stance_foot_pos.x,
    default_stance_foot_pos.y,
    default_stance_foot_pos.z);

  float swing_height = 0.0f;

  if (swing_proportion < 0.5f) {
    swing_height = (swing_proportion / 0.5f) * smnc.z_clearance;
  } else {
    swing_height =
      smnc.z_clearance * (1.0f - ((swing_proportion - 0.5f) / 0.5f));
  }

  const Vector3f foot_pos_vec(foot_pos.x, foot_pos.y, foot_pos.z);

  const Vector3f delta_pos(
    alpha * stance_ticks * dt * cmd.getXSpeedCmd(),
    0.0f,
    alpha * stance_ticks * dt * cmd.getYSpeedCmd());

  const float theta = beta * stance_ticks * dt * -cmd.getYawRateCmd();

  Matrix3f rot_delta;
  rot_delta = AngleAxisf(theta, Vector3f::UnitY());

  const Vector3f touchdown_location =
    (rot_delta * default_stance_foot_pos_vec) + delta_pos;

  const float time_left =
    dt * static_cast<float>(smnc.swing_ticks) * (1.0f - swing_proportion);

  Vector3f new_foot_pos_vec = foot_pos_vec;

  if (time_left > 1e-6f) {
    const Vector3f delta_pos2 =
      ((touchdown_location - foot_pos_vec) / time_left) * dt;

    new_foot_pos_vec = foot_pos_vec + delta_pos2;
  } else {
    new_foot_pos_vec = touchdown_location;
  }

  new_foot_pos_vec[1] = swing_height;

  new_foot_pos.x = new_foot_pos_vec[0];
  new_foot_pos.y = new_foot_pos_vec[1];
  new_foot_pos.z = new_foot_pos_vec[2];

  return new_foot_pos;
}

smk::Point SpotMicroWalkState::stepBodyShift(
  const smk::BodyState& body_state,
  const Command& cmd,
  const SpotMicroNodeConfig& smnc)
{
  (void)cmd;

  const float dt = smnc.dt;

  const int shift_phase = smnc.body_shift_phases[phase_index_];
  const float shift_proportion =
    static_cast<float>(subphase_ticks_) / static_cast<float>(smnc.swing_ticks);

  const float time_left =
    dt * static_cast<float>(smnc.swing_ticks) * (1.0f - shift_proportion);

  float end_x_pos = 0.0f;
  float end_z_pos = 0.0f;

  smk::Point return_point;
  return_point.y = smnc.default_stand_height;

  if (shift_phase == 2) {
    return_point.x = smnc.fwd_body_balance_shift;
    return_point.z = -smnc.side_body_balance_shift;
  } else if (shift_phase == 4) {
    return_point.x = -smnc.back_body_balance_shift;
    return_point.z = -smnc.side_body_balance_shift;
  } else if (shift_phase == 6) {
    return_point.x = smnc.fwd_body_balance_shift;
    return_point.z = smnc.side_body_balance_shift;
  } else if (shift_phase == 8) {
    return_point.x = -smnc.back_body_balance_shift;
    return_point.z = smnc.side_body_balance_shift;
  } else {
    if (shift_phase == 1) {
      end_x_pos = smnc.fwd_body_balance_shift;
      end_z_pos = -smnc.side_body_balance_shift;
    } else if (shift_phase == 3) {
      end_x_pos = -smnc.back_body_balance_shift;
      end_z_pos = -smnc.side_body_balance_shift;
    } else if (shift_phase == 5) {
      end_x_pos = smnc.fwd_body_balance_shift;
      end_z_pos = smnc.side_body_balance_shift;
    } else {
      end_x_pos = -smnc.back_body_balance_shift;
      end_z_pos = smnc.side_body_balance_shift;
    }

    if (time_left > 1e-6f) {
      const float delta_x =
        ((end_x_pos - body_state.xyz_pos.x) / time_left) * dt;

      const float delta_z =
        ((end_z_pos - body_state.xyz_pos.z) / time_left) * dt;

      return_point.x = body_state.xyz_pos.x + delta_x;
      return_point.z = body_state.xyz_pos.z + delta_z;
    } else {
      return_point.x = end_x_pos;
      return_point.z = end_z_pos;
    }
  }

  return return_point;
}
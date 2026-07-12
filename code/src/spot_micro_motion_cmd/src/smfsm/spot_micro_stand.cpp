#include <iostream>
#include <memory>

#include "spot_micro_stand.h"

#include "spot_micro_transition_idle.h"
#include "spot_micro_walk.h"
#include "spot_micro_motion_cmd.h"
#include "rate_limited_first_order_filter.h"

SpotMicroStandState::SpotMicroStandState()
{
  // Constructor does not need to do anything for now.
}

SpotMicroStandState::~SpotMicroStandState()
{
}

void SpotMicroStandState::handleInputCommands(
  const smk::BodyState& body_state,
  const SpotMicroNodeConfig& smnc,
  const Command& cmd,
  SpotMicroMotionCmd* smmc,
  smk::BodyState* body_state_cmd)
{
  (void)body_state;

  if (smnc.debug_mode) {
    std::cout << "In Spot Micro Stand State" << std::endl;
  }

  if (cmd.getIdleCmd()) {
    changeState(smmc, std::make_unique<SpotMicroTransitionIdleState>());
  } else if (cmd.getWalkCmd()) {
    changeState(smmc, std::make_unique<SpotMicroWalkState>());
  } else {
    cmd_state_.euler_angs.phi = cmd.getPhiCmd();
    cmd_state_.euler_angs.theta = cmd.getThetaCmd();
    cmd_state_.euler_angs.psi = cmd.getPsiCmd();

    angle_cmd_filters_.x.setCommand(cmd_state_.euler_angs.phi);
    angle_cmd_filters_.y.setCommand(cmd_state_.euler_angs.theta);
    angle_cmd_filters_.z.setCommand(cmd_state_.euler_angs.psi);

    body_state_cmd->euler_angs.phi =
      angle_cmd_filters_.x.runTimestepAndGetOutput();

    body_state_cmd->euler_angs.theta =
      angle_cmd_filters_.y.runTimestepAndGetOutput();

    body_state_cmd->euler_angs.psi =
      angle_cmd_filters_.z.runTimestepAndGetOutput();

    body_state_cmd->xyz_pos = cmd_state_.xyz_pos;
    body_state_cmd->leg_feet_pos = cmd_state_.leg_feet_pos;

    smmc->setServoCommandMessageData();
    smmc->publishServoProportionalCommand();
  }
}

void SpotMicroStandState::init(
  const smk::BodyState& body_state,
  const SpotMicroNodeConfig& smnc,
  const Command& cmd,
  SpotMicroMotionCmd* smmc)
{
  (void)body_state;
  (void)cmd;

  cmd_state_.leg_feet_pos = smmc->getNeutralStance();

  cmd_state_.euler_angs.phi = 0.0f;
  cmd_state_.euler_angs.theta = 0.0f;
  cmd_state_.euler_angs.psi = 0.0f;

  cmd_state_.xyz_pos.x = 0.0f;
  cmd_state_.xyz_pos.y = smnc.default_stand_height;
  cmd_state_.xyz_pos.z = 0.0f;

  const float dt = smnc.dt;
  const float tau = smnc.transit_tau;
  const float rate_limit = smnc.transit_angle_rl;

  using RLOF = RateLmtdFirstOrderFilter;

  angle_cmd_filters_.x =
    RLOF(dt, tau, cmd_state_.euler_angs.phi, rate_limit);

  angle_cmd_filters_.y =
    RLOF(dt, tau, cmd_state_.euler_angs.theta, rate_limit);

  angle_cmd_filters_.z =
    RLOF(dt, tau, cmd_state_.euler_angs.psi, rate_limit);
}
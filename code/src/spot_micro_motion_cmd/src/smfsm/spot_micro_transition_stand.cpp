#include <iostream>
#include <memory>

#include "spot_micro_transition_stand.h"

#include "spot_micro_stand.h"
#include "spot_micro_motion_cmd.h"
#include "spot_micro_state.h"

SpotMicroTransitionStandState::SpotMicroTransitionStandState()
{
  // Constructor does not need to do anything for now.
}

SpotMicroTransitionStandState::~SpotMicroTransitionStandState()
{
}

void SpotMicroTransitionStandState::init(
  const smk::BodyState& body_state,
  const SpotMicroNodeConfig& smnc,
  const Command& cmd,
  SpotMicroMotionCmd* smmc)
{
  (void)cmd;

  // Set initial state and end state.
  start_body_state_ = body_state;

  // Create end state feet positions: neutral standing stance.
  end_body_state_.leg_feet_pos = smmc->getNeutralStance();

  end_body_state_.euler_angs.phi = 0.0f;
  end_body_state_.euler_angs.theta = 0.0f;
  end_body_state_.euler_angs.psi = 0.0f;

  end_body_state_.xyz_pos.x = 0.0f;
  end_body_state_.xyz_pos.y = smnc.default_stand_height;
  end_body_state_.xyz_pos.z = 0.0f;

  const float dt = smnc.dt;
  const float tau = smnc.transit_tau;
  const float rl = smnc.transit_rl;
  const float rl_ang = smnc.transit_angle_rl;

  initBodyStateFilters(
    dt,
    tau,
    rl,
    rl_ang,
    body_state,
    &body_state_filters_);

  setBodyStateFilterCommands(
    end_body_state_,
    &body_state_filters_);
}

void SpotMicroTransitionStandState::handleInputCommands(
  const smk::BodyState& body_state,
  const SpotMicroNodeConfig& smnc,
  const Command& cmd,
  SpotMicroMotionCmd* smmc,
  smk::BodyState* body_state_cmd)
{
  (void)cmd;

  if (smnc.debug_mode) {
    std::cout << "In Spot Micro Transition Stand State" << std::endl;
  }

  // Check if desired end state was reached. If so, change to stand state.
  if (checkBodyStateEquality(body_state, end_body_state_, 0.001f)) {
    changeState(smmc, std::make_unique<SpotMicroStandState>());
  } else {
    runFilters(&body_state_filters_);

    assignFilterValuesToBodyState(
      body_state_filters_,
      body_state_cmd);

    smmc->setServoCommandMessageData();
    smmc->publishServoProportionalCommand();
  }
}
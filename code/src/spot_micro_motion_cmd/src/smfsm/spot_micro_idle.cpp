#include <iostream>
#include <memory>

#include "spot_micro_idle.h"
#include "spot_micro_motion_cmd.h"
#include "spot_micro_transition_stand.h"

SpotMicroIdleState::SpotMicroIdleState()
{
  // Constructor does not need to do anything for now.
}

SpotMicroIdleState::~SpotMicroIdleState()
{
}

void SpotMicroIdleState::handleInputCommands(
  const smk::BodyState& body_state,
  const SpotMicroNodeConfig& smnc,
  const Command& cmd,
  SpotMicroMotionCmd* smmc,
  smk::BodyState* body_state_cmd)
{
  (void)body_state;
  (void)body_state_cmd;

  if (smnc.debug_mode) {
    std::cout << "In Spot Micro Idle State" << std::endl;
  }

  // Check if stand command was issued. If so, transition to stand state.
  if (cmd.getStandCmd()) {
    changeState(smmc, std::make_unique<SpotMicroTransitionStandState>());
  } else {
    // Otherwise, command idle servo commands.
    smmc->publishZeroServoAbsoluteCommand();
  }
}
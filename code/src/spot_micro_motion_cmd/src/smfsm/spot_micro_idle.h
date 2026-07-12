#pragma once

#include <string>

#include "spot_micro_state.h"
#include "command.h"

class SpotMicroIdleState : public SpotMicroState
{
public:
  SpotMicroIdleState();
  ~SpotMicroIdleState() override;

  void handleInputCommands(
    const smk::BodyState& body_state,
    const SpotMicroNodeConfig& smnc,
    const Command& cmd,
    SpotMicroMotionCmd* smmc,
    smk::BodyState* body_state_cmd) override;

  std::string getCurrentStateName() override
  {
    return "Idle";
  }
};
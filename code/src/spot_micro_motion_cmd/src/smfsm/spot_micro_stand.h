#pragma once

#include <string>

#include "spot_micro_state.h"
#include "command.h"

class SpotMicroStandState : public SpotMicroState
{
public:
  SpotMicroStandState();
  ~SpotMicroStandState() override;

  void handleInputCommands(
    const smk::BodyState& body_state,
    const SpotMicroNodeConfig& smnc,
    const Command& cmd,
    SpotMicroMotionCmd* smmc,
    smk::BodyState* body_state_cmd) override;

  void init(
    const smk::BodyState& body_state,
    const SpotMicroNodeConfig& smnc,
    const Command& cmd,
    SpotMicroMotionCmd* smmc) override;

  std::string getCurrentStateName() override
  {
    return "Stand";
  }

private:
  smk::BodyState cmd_state_;

  // Three filters for angle commands
  XyzFilters angle_cmd_filters_;
};
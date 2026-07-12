#pragma once

#include <string>

#include "spot_micro_state.h"
#include "command.h"
#include "rate_limited_first_order_filter.h"

#include "spot_micro_kinematics/spot_micro_kinematics.h"

class SpotMicroTransitionIdleState : public SpotMicroState
{
public:
  SpotMicroTransitionIdleState();
  ~SpotMicroTransitionIdleState() override;

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
    return "Transit Idle";
  }

private:
  smk::BodyState start_body_state_;
  smk::BodyState end_body_state_;

  BodyStateFilters body_state_filters_;
};
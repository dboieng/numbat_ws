#pragma once

#include <string>

#include "spot_micro_motion_cmd.h"
#include "spot_micro_state.h"
#include "command.h"

struct ContactFeet
{
  bool right_back_in_swing{false};
  bool right_front_in_swing{false};
  bool left_front_in_swing{false};
  bool left_back_in_swing{false};
};

class SpotMicroWalkState : public SpotMicroState
{
public:
  SpotMicroWalkState();
  ~SpotMicroWalkState() override;

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
    return "Walk";
  }

private:
  SpotMicroNodeConfig smnc_;
  smk::BodyState cmd_state_;

  int ticks_{0};
  int phase_index_{0};
  int subphase_ticks_{0};

  ContactFeet contact_feet_states_;

  // Updates phase index, subphase ticks, and contact feet states.
  void updatePhaseData();

  // Steps the gait controller one timestep.
  smk::LegsFootPos stepGait(
    const smk::BodyState& body_state,
    const Command& cmd,
    const SpotMicroNodeConfig& smnc,
    const smk::LegsFootPos& default_stance_feet_pos);

  // Returns new foot position incremented by stance controller.
  smk::Point stanceController(
    const smk::Point& foot_pos,
    const Command& cmd,
    const SpotMicroNodeConfig& smnc);

  // Returns new foot position incremented by swing leg controller.
  smk::Point swingLegController(
    const smk::Point& foot_pos,
    const Command& cmd,
    const SpotMicroNodeConfig& smnc,
    float swing_proportion,
    const smk::Point& default_stance_foot_pos);

  // Steps the body shift controller for balance during the gait cycle.
  smk::Point stepBodyShift(
    const smk::BodyState& body_state,
    const Command& cmd,
    const SpotMicroNodeConfig& smnc);
};
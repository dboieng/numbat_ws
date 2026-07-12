#pragma once

#include <memory>
#include <string>

#include "spot_micro_kinematics/spot_micro_kinematics.h"

#include "command.h"
#include "rate_limited_first_order_filter.h"

// Forward declarations.
// Do not include spot_micro_motion_cmd.h here because that creates circular
// dependency compile errors.
class SpotMicroMotionCmd;
struct SpotMicroNodeConfig;

// Struct holding filters for three axes.
struct XyzFilters
{
  RateLmtdFirstOrderFilter x;
  RateLmtdFirstOrderFilter y;
  RateLmtdFirstOrderFilter z;
};

// Convenience structure holding filters for all transitory body-state values.
struct BodyStateFilters
{
  XyzFilters leg_right_back;
  XyzFilters leg_right_front;
  XyzFilters leg_left_front;
  XyzFilters leg_left_back;
  XyzFilters body_pos;
  XyzFilters body_angs;
};

class SpotMicroState
{
public:
  SpotMicroState();

  virtual ~SpotMicroState();

  virtual void handleInputCommands(
    const smk::BodyState& body_state,
    const SpotMicroNodeConfig& smnc,
    const Command& cmd,
    SpotMicroMotionCmd* smmc,
    smk::BodyState* body_state_cmd)
  {
    (void)body_state;
    (void)smnc;
    (void)cmd;
    (void)smmc;
    (void)body_state_cmd;
  }

  virtual void init(
    const smk::BodyState& body_state,
    const SpotMicroNodeConfig& smnc,
    const Command& cmd,
    SpotMicroMotionCmd* smmc)
  {
    (void)body_state;
    (void)smnc;
    (void)cmd;
    (void)smmc;
  }

  virtual std::string getCurrentStateName()
  {
    return "None";
  }

protected:
  // Calls SpotMicroMotionCmd's method to change the currently active state.
  void changeState(
    SpotMicroMotionCmd* smmc,
    std::unique_ptr<SpotMicroState> sms);

  // Initializes filters controlling body-state values.
  virtual void initBodyStateFilters(
    float dt,
    float tau,
    float rl,
    float rl_ang,
    const smk::BodyState& body_state,
    BodyStateFilters* body_state_filters);

  // Sets body-state filter commands to values contained in body_state.
  virtual void setBodyStateFilterCommands(
    const smk::BodyState& body_state,
    BodyStateFilters* body_state_filters);

  // Calls run timestep method for all body-state filters.
  virtual void runFilters(BodyStateFilters* body_state_filters);

  // Assigns current filter values to body state.
  virtual void assignFilterValuesToBodyState(
    const BodyStateFilters& body_state_filters,
    smk::BodyState* body_state);

  // Checks equality of body-state structs to an absolute tolerance.
  virtual bool checkBodyStateEquality(
    const smk::BodyState& body_state1,
    const smk::BodyState& body_state2,
    float tol);
};
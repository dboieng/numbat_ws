#pragma once

#include <algorithm>
#include <cmath>

// Encapsulates a first-order filter with rate limiting.
//
// Assumption:
//   runTimestep() is called at a fixed sample time dt.
//
// Equations:
//   y[i] = (1 - alpha) * y[i-1] + alpha * u[i]
//   alpha = dt / (tau + dt)
//
// If the calculated rate exceeds rate_limit_, the output increment is limited.
class RateLmtdFirstOrderFilter
{
public:
  RateLmtdFirstOrderFilter() = default;

  RateLmtdFirstOrderFilter(float dt, float tau, float x0, float rate_limit)
  : dt_(dt),
    tau_(tau),
    state_(x0),
    cmd_(x0),
    alpha_(dt / (tau + dt)),
    rate_limit_(rate_limit)
  {
  }

  void setCommand(float cmd)
  {
    cmd_ = cmd;
  }

  float runTimestepAndGetOutput()
  {
    runTimestep();
    return state_;
  }

  void runTimestep()
  {
    const float y_prev = state_;

    const float y_unlimited =
      (1.0f - alpha_) * y_prev + alpha_ * cmd_;

    const float max_delta = rate_limit_ * dt_;
    const float delta = std::clamp(
      y_unlimited - y_prev,
      -max_delta,
      max_delta
    );

    state_ = y_prev + delta;
  }

  float getOutput() const
  {
    return state_;
  }

  void resetState(float x0)
  {
    state_ = x0;
    cmd_ = x0;
  }

private:
  float dt_{0.001f};
  float tau_{0.001f};
  float state_{0.0f};
  float cmd_{0.0f};
  float alpha_{0.5f};
  float rate_limit_{0.0f};
};
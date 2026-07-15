// Torque-governed retiming: stretch a planned move's duration until the
// predictive torque audit fits inside the budget, making it impossible to
// command a move fast enough to exceed the torque limit.
//
// Physics: slowing a quintic profile by k divides velocities by k and
// accelerations by k², so inertial torque shrinks and the stepper's
// speed-dependent available torque grows, while the gravity load stays fixed.
// Utilization therefore falls monotonically toward its static floor as the
// duration grows. If even that floor exceeds the ceiling (the arm cannot hold
// the pose at all), no timing helps and the move is flagged instead.
//
// This is pure orchestration over plan + audit; the physics lives in the
// injected audit callback (rt::auditTrajectory), so it works identically over
// native, WASM, and the TS mirror (web/src/core/retime.ts).
#pragma once

#include <functional>

#include "../simulation/metrics.hpp"
#include "trajectory.hpp"

namespace rt {

struct RetimeResult {
  TrajectoryPlan plan;
  TrajectoryAudit audit;
  double stretch;  // duration multiplier applied (1 = planner's timing was already safe)
  bool limited;    // true when no stretch <= maxStretch fits the budget (static overload)
};

/**
 * Find the smallest duration stretch that brings peak torque utilization at
 * or below `ceiling`. Exponential search up to `maxStretch`, then bisection,
 * so moves stay as quick as the torque budget allows.
 */
RetimeResult retimeForTorque(const TrajectoryPlan& plan,
                              const std::function<TrajectoryAudit(const TrajectoryPlan&)>& audit,
                              double ceiling = 0.95, double maxStretch = 32.0);

} // namespace rt

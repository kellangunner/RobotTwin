// Point-to-point joint-space trajectories: synchronized quintic time scaling
// (zero boundary velocity/acceleration, C² smooth — jerk-bounded by design).
//   s(τ) = 10τ³ − 15τ⁴ + 6τ⁵,  max|ṡ| = 1.875/T,  max|s̈| ≈ 5.7735/T²
// Mirrors web/src/core/trajectory.ts.
#pragma once

#include "../config/config.hpp"

namespace rt {

struct TrajectoryPlan {
  JointAngles from;
  JointAngles to;
  double duration;  // s
  bool infeasible;  // some joint had no usable speed/acceleration budget
};

/**
 * Shortest duration respecting every joint's velocity and acceleration
 * limits. Limits come from the drivetrain, so the same move gets
 * slower/faster as gearboxes change.
 */
TrajectoryPlan planTrajectory(const JointAngles& from, const JointAngles& to,
                              const std::array<double, kNumJoints>& vmax,
                              const std::array<double, kNumJoints>& amax,
                              double minDuration = 0.25);

struct TrajectorySample {
  JointAngles q;
  JointAngles qd;  // rad/s
  JointAngles qdd; // rad/s²
};

TrajectorySample sampleTrajectory(const TrajectoryPlan& plan, double t);

} // namespace rt

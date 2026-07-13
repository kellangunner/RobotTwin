#include "trajectory.hpp"

#include <algorithm>
#include <cmath>

namespace rt {

namespace {
constexpr double kQuinticVmax = 1.875;
const double kQuinticAmax = 10.0 / std::sqrt(3.0); // ≈ 5.7735
} // namespace

TrajectoryPlan planTrajectory(const JointAngles& from, const JointAngles& to,
                              const std::array<double, kNumJoints>& vmax,
                              const std::array<double, kNumJoints>& amax,
                              double minDuration) {
  double T = minDuration;
  bool infeasible = false;
  for (int i = 0; i < kNumJoints; ++i) {
    const double dq = std::abs(to[i] - from[i]);
    if (dq < 1e-9) continue;
    if (vmax[i] <= 0.0 || amax[i] <= 0.0) {
      infeasible = true;
      continue;
    }
    T = std::max({T, kQuinticVmax * dq / vmax[i], std::sqrt(kQuinticAmax * dq / amax[i])});
  }
  return {from, to, T, infeasible};
}

TrajectorySample sampleTrajectory(const TrajectoryPlan& plan, double t) {
  const double T = plan.duration;
  const double tau = std::min(1.0, std::max(0.0, t / T));
  const double s = tau * tau * tau * (10.0 + tau * (-15.0 + 6.0 * tau));
  const double sd = tau * tau * (30.0 + tau * (-60.0 + 30.0 * tau)) / T;
  const double sdd = tau * (60.0 + tau * (-180.0 + 120.0 * tau)) / (T * T);

  TrajectorySample out{};
  for (int i = 0; i < kNumJoints; ++i) {
    const double dq = plan.to[i] - plan.from[i];
    out.q[i] = plan.from[i] + dq * s;
    out.qd[i] = dq * sd;
    out.qdd[i] = dq * sdd;
  }
  return out;
}

} // namespace rt

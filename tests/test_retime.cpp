#include <cmath>
#include <limits>

#include "../src/planning/retime.hpp"
#include "harness.hpp"

using namespace rt;

namespace {

const TrajectoryPlan kBasePlan{{0, 0, 0}, {1, 1, 1}, 1.0, false};

/**
 * Synthetic audit matching the real physics shape: a fixed gravity floor plus
 * an inertial term that scales with 1/duration^2 (quintic time scaling).
 */
std::function<TrajectoryAudit(const TrajectoryPlan&)> gravityPlusInertia(double grav,
                                                                          double dyn) {
  return [grav, dyn](const TrajectoryPlan& plan) -> TrajectoryAudit {
    const double util = grav + dyn / (plan.duration * plan.duration);
    return {util, Joint::Shoulder, util > 1.0};
  };
}

} // namespace

RT_TEST(retime_leaves_an_already_safe_plan_untouched) {
  const RetimeResult res = retimeForTorque(kBasePlan, gravityPlusInertia(0.3, 0.2), 0.95);
  CHECK(res.stretch == 1.0);
  CHECK(!res.limited);
  CHECK(res.plan.duration == kBasePlan.duration);
  CHECK(res.audit.peakUtilization <= 0.95);
}

RT_TEST(retime_stretches_an_overloaded_move_until_it_fits_the_ceiling) {
  const double ceiling = 0.95;
  const RetimeResult res = retimeForTorque(kBasePlan, gravityPlusInertia(0.4, 2.0), ceiling);
  CHECK(!res.limited);
  CHECK(res.audit.peakUtilization <= ceiling);
  // Minimal stretch solves grav + dyn/k^2 = ceiling -> k* = sqrt(dyn/(ceiling-grav)).
  const double kStar = std::sqrt(2.0 / (ceiling - 0.4));
  CHECK(res.stretch >= kStar);
  CHECK(res.stretch < kStar * 1.1); // bisection stays near-minimal
  CHECK_CLOSE(res.plan.duration, res.stretch * kBasePlan.duration, 1e-9);
}

RT_TEST(retime_never_returns_a_plan_above_the_ceiling_when_achievable) {
  for (double dyn : {0.1, 1.0, 10.0, 100.0, 700.0}) {
    const RetimeResult res = retimeForTorque(kBasePlan, gravityPlusInertia(0.2, dyn), 0.95);
    CHECK(!res.limited);
    CHECK(res.audit.peakUtilization <= 0.95);
  }
}

RT_TEST(retime_flags_moves_needing_a_stretch_beyond_the_cap) {
  // Needs k ~= 34.6 > default maxStretch of 32.
  const RetimeResult res = retimeForTorque(kBasePlan, gravityPlusInertia(0.2, 900.0), 0.95);
  CHECK(res.limited);
}

RT_TEST(retime_flags_a_static_overload_instead_of_stretching_forever) {
  // Gravity floor above the ceiling: no duration can fix this.
  const RetimeResult res = retimeForTorque(kBasePlan, gravityPlusInertia(1.2, 0.5), 0.95);
  CHECK(res.limited);
  CHECK(res.stretch == 1.0);
  CHECK(res.plan.duration == kBasePlan.duration); // planner timing kept
  CHECK(res.audit.skippedSteps);
}

RT_TEST(retime_recovers_from_infinite_utilization) {
  auto audit = [](const TrajectoryPlan& plan) -> TrajectoryAudit {
    const double util = plan.duration < 3.0 ? std::numeric_limits<double>::infinity() : 0.5;
    return {util, Joint::Base, util > 1.0};
  };
  const RetimeResult res = retimeForTorque(kBasePlan, audit, 0.95);
  CHECK(!res.limited);
  CHECK(res.plan.duration >= 3.0);
  CHECK(res.audit.peakUtilization <= 0.95);
}

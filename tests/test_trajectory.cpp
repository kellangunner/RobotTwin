#include <cmath>

#include "../src/planning/trajectory.hpp"
#include "../src/simulation/metrics.hpp"
#include "fixtures.hpp"
#include "harness.hpp"

using namespace rt;
using rtest::config;

RT_TEST(trajectory_respects_velocity_and_acceleration_limits) {
  const JointAngles from{0, 0.2, -0.4};
  const JointAngles to{1.5, 1.2, -1.8};
  const std::array<double, 3> vmax{1.0, 0.8, 1.2};
  const std::array<double, 3> amax{3, 2, 4};
  const TrajectoryPlan plan = planTrajectory(from, to, vmax, amax);
  CHECK(!plan.infeasible);

  std::array<double, 3> peakV{}, peakA{};
  const int n = 400;
  for (int k = 0; k <= n; ++k) {
    const TrajectorySample s = sampleTrajectory(plan, plan.duration * k / n);
    for (int i = 0; i < 3; ++i) {
      peakV[i] = std::max(peakV[i], std::abs(s.qd[i]));
      peakA[i] = std::max(peakA[i], std::abs(s.qdd[i]));
    }
  }
  for (int i = 0; i < 3; ++i) {
    CHECK(peakV[i] <= vmax[i] * 1.001);
    CHECK(peakA[i] <= amax[i] * 1.001);
  }
  const TrajectorySample end = sampleTrajectory(plan, plan.duration);
  for (int i = 0; i < 3; ++i) {
    CHECK_CLOSE(end.q[i], to[i], 1e-9);
    CHECK_CLOSE(end.qd[i], 0.0, 1e-9);
  }
}

RT_TEST(lower_vmax_lengthens_the_move) {
  const JointAngles from{0, 0, 0};
  const JointAngles to{1, 1, 1};
  const TrajectoryPlan fast = planTrajectory(from, to, {2, 2, 2}, {10, 10, 10});
  const TrajectoryPlan slow = planTrajectory(from, to, {0.5, 0.5, 0.5}, {10, 10, 10});
  CHECK(slow.duration > fast.duration * 3);
}

RT_TEST(zero_accel_budget_flags_infeasible) {
  const TrajectoryPlan plan = planTrajectory({0, 0, 0}, {1, 1, 1}, {1, 1, 1}, {1, 0, 1});
  CHECK(plan.infeasible);
}

RT_TEST(audit_predicts_skipped_steps_with_direct_shoulder) {
  const RobotConfig& cfg = config();
  // slow "move" that holds near full horizontal extension
  const TrajectoryPlan plan =
      planTrajectory({0, 0.05, -0.1}, {0, 0.1, -0.15}, {0.5, 0.5, 0.5}, {2, 2, 2});

  auto weak = cfg.gearboxes;
  weak[1].ratio = 1;
  weak[1].efficiency = 0.98;
  const TrajectoryAudit bad = auditTrajectory(cfg, weak, plan, 0.1);
  CHECK(bad.skippedSteps);
  CHECK(bad.peakJoint == Joint::Shoulder);

  const TrajectoryAudit good = auditTrajectory(cfg, cfg.gearboxes, plan, 0.1);
  CHECK(!good.skippedSteps);
  CHECK(good.peakUtilization > 0.1);
}

RT_TEST(compute_metrics_flags_hold_failure) {
  const RobotConfig& cfg = config();
  auto weak = cfg.gearboxes;
  weak[1] = {GearboxType::Direct, 1, 0.98, deg2rad(0.02), 1.0, 0.0};
  // arm horizontal: shoulder gravity ≈ 1.17 N·m > 0.44 N·m direct-drive limit
  const TwinMetrics m = computeMetrics(cfg, weak, {0, 0, 0}, 0.1);
  CHECK(m.joints[1].holdFails);
  CHECK(m.joints[1].maxAccel == 0.0);
  const TwinMetrics ok = computeMetrics(cfg, cfg.gearboxes, {0, 0, 0}, 0.1);
  CHECK(!ok.joints[1].holdFails);
  CHECK(ok.backlashErrorTcp > 0.0);
}

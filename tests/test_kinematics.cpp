#include <cmath>

#include "../src/kinematics/kinematics.hpp"
#include "fixtures.hpp"
#include "harness.hpp"

using namespace rt;
using rtest::config;
using rtest::pose;

RT_TEST(config_loads_geometry_in_si_units) {
  const RobotConfig& cfg = config();
  CHECK_CLOSE(cfg.links.baseHeight, 0.09, 1e-12);
  CHECK_CLOSE(cfg.links.upperArm, 0.12, 1e-12);
  CHECK_CLOSE(cfg.links.forearm, 0.12, 1e-12);
  CHECK_CLOSE(cfg.limits[0].min, deg2rad(-135), 1e-12);
  CHECK_CLOSE(cfg.masses.payloadMax, 2.0, 1e-12);
  CHECK(cfg.gearboxes[1].ratio > 1.0);
  CHECK(cfg.name == "rt-arm-3dof");
}

RT_TEST(fk_reference_pose_straight_out_along_x) {
  const auto& g = config().links;
  const Vec3 tcp = forwardKinematics(pose(0, 0, 0), g).tcp;
  CHECK_CLOSE(tcp[0], g.upperArm + g.forearm, 1e-12);
  CHECK_CLOSE(tcp[1], 0.0, 1e-12);
  CHECK_CLOSE(tcp[2], g.baseHeight, 1e-12);
}

RT_TEST(fk_arm_straight_up) {
  const auto& g = config().links;
  const Vec3 tcp = forwardKinematics(pose(0, 90, 0), g).tcp;
  CHECK_CLOSE(tcp[0], 0.0, 1e-12);
  CHECK_CLOSE(tcp[2], g.baseHeight + g.upperArm + g.forearm, 1e-12);
}

RT_TEST(fk_base_yaw_rotates_arm_plane) {
  const auto& g = config().links;
  const Vec3 tcp = forwardKinematics(pose(90, 0, 0), g).tcp;
  CHECK_CLOSE(tcp[0], 0.0, 1e-12);
  CHECK_CLOSE(tcp[1], g.upperArm + g.forearm, 1e-12);
}

RT_TEST(fk_elbow_folded_90_down) {
  const auto& g = config().links;
  const FkResult fk = forwardKinematics(pose(0, 0, -90), g);
  CHECK_CLOSE(fk.elbow[0], g.upperArm, 1e-12);
  CHECK_CLOSE(fk.tcp[0], g.upperArm, 1e-12);
  CHECK_CLOSE(fk.tcp[2], g.baseHeight - g.forearm, 1e-12);
}

RT_TEST(ik_rejects_unreachable_targets) {
  const RobotConfig& cfg = config();
  CHECK(!inverseKinematics({0.5, 0, cfg.links.baseHeight}, cfg.links, cfg.limits).reachable);
  CHECK(!inverseKinematics({0.3, 0.3, 0.3}, cfg.links, cfg.limits).reachable);
}

RT_TEST(ik_returns_four_branches_landing_on_target) {
  const RobotConfig& cfg = config();
  const Vec3 target{0.15, 0.05, 0.15};
  const IkResult res = inverseKinematics(target, cfg.links, cfg.limits);
  CHECK(res.reachable);
  CHECK(res.solutionCount == 4);
  int frontUp = 0, frontDown = 0;
  for (int i = 0; i < res.solutionCount; ++i) {
    const IkSolution& s = res.solutions[i];
    if (!s.baseFlipped && s.branch == IkBranch::ElbowUp) ++frontUp;
    if (!s.baseFlipped && s.branch == IkBranch::ElbowDown) ++frontDown;
    const Vec3 p = forwardKinematics(s.q, cfg.links).tcp;
    CHECK_CLOSE(p[0], target[0], 1e-9);
    CHECK_CLOSE(p[1], target[1], 1e-9);
    CHECK_CLOSE(p[2], target[2], 1e-9);
  }
  CHECK(frontUp == 1);
  CHECK(frontDown == 1);
}

RT_TEST(ik_flags_near_singular_straight_arm) {
  const RobotConfig& cfg = config();
  const double reach = cfg.links.upperArm + cfg.links.forearm;
  const IkResult res =
      inverseKinematics({reach - 1e-5, 0, cfg.links.baseHeight}, cfg.links, cfg.limits);
  CHECK(res.reachable);
  for (int i = 0; i < res.solutionCount; ++i) CHECK(res.solutions[i].nearSingularity);
}

RT_TEST(ik_flags_base_axis_targets) {
  const RobotConfig& cfg = config();
  const double z = cfg.links.baseHeight + cfg.links.upperArm + cfg.links.forearm - 0.01;
  CHECK(inverseKinematics({0, 0, z}, cfg.links, cfg.limits).baseSingular);
}

RT_TEST(ik_marks_limit_violations_behind_base) {
  const RobotConfig& cfg = config();
  const IkResult res = inverseKinematics({-0.15, -1e-9, 0.15}, cfg.links, cfg.limits);
  CHECK(res.reachable);
  bool flippedOk = false;
  for (int i = 0; i < res.solutionCount; ++i) {
    const IkSolution& s = res.solutions[i];
    if (!s.baseFlipped) {
      CHECK(s.violated[0]); // front solutions exceed the ±135° yaw limit
      CHECK(!s.withinLimits);
    } else if (!s.violated[0]) {
      flippedOk = true;
    }
  }
  CHECK(flippedOk); // over-the-top branch reaches it without violating yaw
}

RT_TEST(fk_ik_round_trip_over_pose_grid) {
  const RobotConfig& cfg = config();
  for (int a = -120; a <= 120; a += 60) {
    for (int b = 10; b <= 170; b += 40) {
      for (int c = -140; c <= 140; c += 35) {
        if (std::abs(c) < 10) continue; // skip singular straight arm
        const JointAngles q = pose(a, b, c);
        const Vec3 tcp = forwardKinematics(q, cfg.links).tcp;
        const IkResult res = inverseKinematics(tcp, cfg.links, cfg.limits);
        CHECK(res.reachable);
        const IkSolution* best = nullptr;
        double bestErr = 1e9;
        for (int i = 0; i < res.solutionCount; ++i) {
          const JointAngles& s = res.solutions[i].q;
          const double err =
              std::abs(s[0] - q[0]) + std::abs(s[1] - q[1]) + std::abs(s[2] - q[2]);
          if (err < bestErr) {
            bestErr = err;
            best = &res.solutions[i];
          }
        }
        CHECK(best != nullptr);
        CHECK_CLOSE(best->q[0], q[0], 1e-6);
        CHECK_CLOSE(best->q[1], q[1], 1e-6);
        CHECK_CLOSE(best->q[2], q[2], 1e-6);
        const Vec3 back = forwardKinematics(best->q, cfg.links).tcp;
        CHECK_CLOSE(back[0], tcp[0], 1e-9);
        CHECK_CLOSE(back[1], tcp[1], 1e-9);
        CHECK_CLOSE(back[2], tcp[2], 1e-9);
      }
    }
  }
}

RT_TEST(jacobian_matches_central_finite_differences) {
  const auto& g = config().links;
  const JointAngles poses[] = {{0.3, 0.8, -1.1}, {-1.2, 1.4, 0.7}, {0.0, 0.5, -2.0}};
  const double eps = 1e-6;
  for (const JointAngles& q : poses) {
    const Jacobian J = jacobian(q, g);
    for (int i = 0; i < kNumJoints; ++i) {
      JointAngles qp = q, qm = q;
      qp[i] += eps;
      qm[i] -= eps;
      const Vec3 fp = forwardKinematics(qp, g).tcp;
      const Vec3 fm = forwardKinematics(qm, g).tcp;
      for (int k = 0; k < 3; ++k) {
        CHECK_CLOSE(J[i][k], (fp[k] - fm[k]) / (2 * eps), 1e-6);
      }
    }
  }
}

RT_TEST(singularity_measure_vanishes_with_straight_arm) {
  const auto& g = config().links;
  CHECK_CLOSE(singularityMeasure({0, 0.5, 0}, g), 0.0, 1e-9);
  CHECK(singularityMeasure({0, 0.5, -1.2}, g) > 0.1);
}

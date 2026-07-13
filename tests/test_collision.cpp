#include <cstring>

#include "../src/geometry/collision.hpp"
#include "fixtures.hpp"
#include "harness.hpp"

using namespace rt;
using rtest::config;
using rtest::pose;

namespace {

bool hasIssue(const CollisionCheck& c, CollisionIssue issue) {
  for (int i = 0; i < c.issueCount; ++i) {
    if (c.issues[i] == issue) return true;
  }
  return false;
}

} // namespace

RT_TEST(point_segment_distance_primitives) {
  CHECK_CLOSE(pointSegmentDistance({0.5, 1, 0}, {0, 0, 0}, {1, 0, 0}), 1.0, 1e-12);
  CHECK_CLOSE(pointSegmentDistance({2, 0, 0}, {0, 0, 0}, {1, 0, 0}), 1.0, 1e-12);
  CHECK_CLOSE(pointSegmentDistance({-3, 4, 0}, {0, 0, 0}, {1, 0, 0}), 5.0, 1e-12);
}

RT_TEST(segment_segment_distance_primitives) {
  CHECK_CLOSE(segmentSegmentDistance({0, 0, 0}, {1, 0, 0}, {0.5, -1, 1}, {0.5, 1, 1}), 1.0, 1e-12);
  CHECK_CLOSE(segmentSegmentDistance({0, 0, 0}, {1, 0, 0}, {0, 0, 2}, {1, 0, 2}), 2.0, 1e-12);
  CHECK_CLOSE(segmentSegmentDistance({0, 0, 0}, {1, 0, 0}, {3, 0, 0}, {4, 0, 0}), 2.0, 1e-12);
  CHECK_CLOSE(segmentSegmentDistance({-1, 0, 0}, {1, 0, 0}, {0, -1, 0}, {0, 1, 0}), 0.0, 1e-12);
}

RT_TEST(point_cylinder_distance_has_flat_top_not_dome) {
  CHECK_CLOSE(pointCylinderDistance({2, 0, 0.5}, 1, 1), 1.0, 1e-12);
  CHECK_CLOSE(pointCylinderDistance({0, 0, 3}, 1, 1), 2.0, 1e-12);
  CHECK_CLOSE(pointCylinderDistance({0.9, 0, 1.4}, 1, 1), 0.4, 1e-12);
  CHECK_CLOSE(pointCylinderDistance({4, 0, 5}, 1, 1), 5.0, 1e-12); // rim corner
  CHECK_CLOSE(pointCylinderDistance({0.5, 0, 0.5}, 1, 1), 0.0, 1e-12);
}

RT_TEST(home_and_straight_poses_are_collision_free) {
  const RobotConfig& cfg = config();
  CHECK(!checkPose(pose(0, 90, -90), cfg.links, cfg.collision).colliding);
  CHECK(!checkPose(pose(0, 0, 0), cfg.links, cfg.collision).colliding);
}

RT_TEST(detects_forearm_below_the_table) {
  const RobotConfig& cfg = config();
  const CollisionCheck res = checkPose(pose(0, 0, -60), cfg.links, cfg.collision);
  CHECK(res.colliding);
  CHECK(hasIssue(res, CollisionIssue::ForearmGround));
}

RT_TEST(detects_forearm_folding_into_base_column) {
  const RobotConfig& cfg = config();
  const CollisionCheck res = checkPose(pose(0, 24.3, -138.6), cfg.links, cfg.collision);
  CHECK(res.colliding);
  CHECK(hasIssue(res, CollisionIssue::ForearmColumn));
}

RT_TEST(detects_deep_elbow_fold_against_shoulder) {
  const RobotConfig& cfg = config();
  const CollisionCheck deep = checkPose(pose(0, 90, -150), cfg.links, cfg.collision);
  CHECK(deep.colliding);
  CHECK(hasIssue(deep, CollisionIssue::ForearmShoulder));
  CHECK(!checkPose(pose(0, 90, -140), cfg.links, cfg.collision).colliding);
}

RT_TEST(collision_is_base_yaw_invariant) {
  const RobotConfig& cfg = config();
  for (const double q1 : {-120.0, -45.0, 60.0, 130.0}) {
    CHECK(checkPose(pose(q1, 0, -60), cfg.links, cfg.collision).colliding);
    CHECK(!checkPose(pose(q1, 90, -90), cfg.links, cfg.collision).colliding);
  }
}

RT_TEST(path_check_flags_colliding_endpoint) {
  const RobotConfig& cfg = config();
  const std::array<double, 3> vmax{2, 2, 2};
  const std::array<double, 3> amax{10, 10, 10};
  const JointAngles from = pose(0, 90, -90);
  CHECK(!checkPose(from, cfg.links, cfg.collision).colliding);
  const TrajectoryPlan plan = planTrajectory(from, pose(0, 0, -60), vmax, amax);
  CHECK(checkPath(plan, cfg.links, cfg.collision).colliding);
}

RT_TEST(path_check_passes_a_safe_move) {
  const RobotConfig& cfg = config();
  const TrajectoryPlan plan =
      planTrajectory(pose(0, 90, -90), pose(45, 60, -60), {2, 2, 2}, {10, 10, 10});
  CHECK(!checkPath(plan, cfg.links, cfg.collision).colliding);
}

RT_TEST(issue_descriptions_are_stable_strings) {
  CHECK(std::strcmp(describe(CollisionIssue::ForearmGround),
                    "forearm/gripper hits the ground") == 0);
  CHECK(std::strcmp(describe(CollisionIssue::ForearmShoulder),
                    "forearm/gripper hits the shoulder joint") == 0);
}

// Collision detection: conservative capsule/sphere envelopes around the links
// and static structure, plus the ground plane (table, z = 0).
// Checked pairs (given θ2 ∈ [0°,180°] keeps the upper arm at/above shoulder):
//   forearm+gripper vs ground / base column / base housing / shoulder joint
//   upper arm       vs ground (defensive)
// The base and column are flat-topped cylinders — a capsule would dome the
// top by its full radius, far too conservative for a squat housing.
// Mirrors web/src/core/collision.ts.
#pragma once

#include <array>

#include "../config/config.hpp"
#include "../math/vec3.hpp"
#include "../planning/trajectory.hpp"

namespace rt {

enum class CollisionIssue : int {
  ForearmGround = 0,
  UpperArmGround,
  ForearmColumn,
  ForearmBase,
  ForearmShoulder,
};

const char* describe(CollisionIssue issue);

struct CollisionCheck {
  bool colliding = false;
  std::array<CollisionIssue, 5> issues{};
  int issueCount = 0;
};

/** Minimum distance from point p to segment [a, b]. */
double pointSegmentDistance(const Vec3& p, const Vec3& a, const Vec3& b);

/** Minimum distance between segments [p1, q1] and [p2, q2]. */
double segmentSegmentDistance(const Vec3& p1, const Vec3& q1, const Vec3& p2, const Vec3& q2);

/**
 * Distance from point p to a flat-topped cylinder of radius r spanning
 * z = 0..top, centered on the world Z axis. 0 inside.
 */
double pointCylinderDistance(const Vec3& p, double r, double top);

/** Check a single pose for self-collision and ground contact. */
CollisionCheck checkPose(const JointAngles& q, const LinkGeometry& geom,
                         const CollisionModel& model);

/** Check every pose along a plan (dense sampling, endpoints included). */
CollisionCheck checkPath(const TrajectoryPlan& plan, const LinkGeometry& geom,
                         const CollisionModel& model, int samples = 60);

} // namespace rt

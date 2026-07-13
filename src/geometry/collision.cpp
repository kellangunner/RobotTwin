#include "collision.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "../kinematics/kinematics.hpp"

namespace rt {

namespace {
double clamp01(double t) { return std::min(1.0, std::max(0.0, t)); }

/** Minimum point-cylinder distance over a densely sampled segment [a, b]. */
double segmentCylinderDistance(const Vec3& a, const Vec3& b, double r, double top,
                               int samples = 32) {
  double minDist = std::numeric_limits<double>::infinity();
  for (int k = 0; k <= samples; ++k) {
    const double t = static_cast<double>(k) / samples;
    const Vec3 p = add(a, scale(sub(b, a), t));
    minDist = std::min(minDist, pointCylinderDistance(p, r, top));
  }
  return minDist;
}
} // namespace

const char* describe(CollisionIssue issue) {
  switch (issue) {
    case CollisionIssue::ForearmGround: return "forearm/gripper hits the ground";
    case CollisionIssue::UpperArmGround: return "upper arm hits the ground";
    case CollisionIssue::ForearmColumn: return "forearm/gripper hits the base column";
    case CollisionIssue::ForearmBase: return "forearm/gripper hits the base housing";
    case CollisionIssue::ForearmShoulder: return "forearm/gripper hits the shoulder joint";
  }
  return "?";
}

double pointSegmentDistance(const Vec3& p, const Vec3& a, const Vec3& b) {
  const Vec3 ab = sub(b, a);
  const double len2 = dot(ab, ab);
  const double t = len2 > 0.0 ? clamp01(dot(sub(p, a), ab) / len2) : 0.0;
  return norm(sub(p, add(a, scale(ab, t))));
}

double segmentSegmentDistance(const Vec3& p1, const Vec3& q1, const Vec3& p2, const Vec3& q2) {
  const Vec3 d1 = sub(q1, p1);
  const Vec3 d2 = sub(q2, p2);
  const Vec3 r = sub(p1, p2);
  const double a = dot(d1, d1);
  const double e = dot(d2, d2);
  const double f = dot(d2, r);

  double s, t;
  if (a <= 1e-12 && e <= 1e-12) {
    s = 0.0;
    t = 0.0;
  } else if (a <= 1e-12) {
    s = 0.0;
    t = clamp01(f / e);
  } else {
    const double c = dot(d1, r);
    if (e <= 1e-12) {
      t = 0.0;
      s = clamp01(-c / a);
    } else {
      const double b = dot(d1, d2);
      const double denom = a * e - b * b;
      s = denom > 1e-12 ? clamp01((b * f - c * e) / denom) : 0.0;
      t = clamp01((b * s + f) / e);
      s = clamp01((b * t - c) / a);
    }
  }
  return norm(sub(add(p1, scale(d1, s)), add(p2, scale(d2, t))));
}

double pointCylinderDistance(const Vec3& p, double r, double top) {
  const double radial = std::hypot(p[0], p[1]) - r;
  const double vertical = p[2] > top ? p[2] - top : (p[2] < 0.0 ? -p[2] : 0.0);
  if (radial <= 0.0) return vertical; // inside radially → 0 when also inside vertically
  return std::hypot(radial, vertical);
}

CollisionCheck checkPose(const JointAngles& q, const LinkGeometry& geom,
                         const CollisionModel& model) {
  const FkResult fk = forwardKinematics(q, geom);
  const Vec3& S = fk.shoulder;
  const Vec3& E = fk.elbow;
  const Vec3& T = fk.tcp;

  CollisionCheck out;
  const auto push = [&](CollisionIssue issue) {
    out.colliding = true;
    out.issues[out.issueCount++] = issue;
  };

  // forearm capsule, extended past the TCP to cover the gripper
  const double fLen = norm(sub(T, E));
  const Vec3 u = fLen > 1e-9 ? scale(sub(T, E), 1.0 / fLen) : Vec3{1.0, 0.0, 0.0};
  const Vec3 fEnd = add(T, scale(u, model.gripperExtent));
  const Vec3 fStartTrimmed = add(E, scale(u, fLen * model.elbowTrim));

  // ground plane
  if (std::min(E[2], fEnd[2]) - model.forearmRadius < model.groundClearance) {
    push(CollisionIssue::ForearmGround);
  }
  if (std::min(S[2], E[2]) - model.upperArmRadius < model.groundClearance) {
    push(CollisionIssue::UpperArmGround);
  }

  // static structure (flat-topped cylinders on the yaw axis)
  if (segmentCylinderDistance(E, fEnd, model.columnRadius, model.columnTop) <
      model.forearmRadius) {
    push(CollisionIssue::ForearmColumn);
  }
  if (segmentCylinderDistance(E, fEnd, model.baseRadius, model.baseTop) < model.forearmRadius) {
    push(CollisionIssue::ForearmBase);
  }

  // shoulder joint housing (adjacent-link fold; trimmed at the elbow)
  if (pointSegmentDistance(S, fStartTrimmed, fEnd) <
      model.forearmRadius + model.shoulderRadius) {
    push(CollisionIssue::ForearmShoulder);
  }

  return out;
}

CollisionCheck checkPath(const TrajectoryPlan& plan, const LinkGeometry& geom,
                         const CollisionModel& model, int samples) {
  for (int k = 0; k <= samples; ++k) {
    const TrajectorySample s = sampleTrajectory(plan, plan.duration * k / samples);
    const CollisionCheck res = checkPose(s.q, geom, model);
    if (res.colliding) return res;
  }
  return {};
}

} // namespace rt

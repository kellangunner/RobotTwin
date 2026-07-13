#include "kinematics.hpp"

#include <cmath>

#include "../math/units.hpp"

namespace rt {

namespace {
/** Angular tolerance below which singularities are flagged (≈ 2.9°). */
constexpr double kSingularityTol = 0.05;
constexpr double kReachTol = 1e-9;

double normalizeAngle(double a) { return std::atan2(std::sin(a), std::cos(a)); }
double clampUnit(double v) { return clamp(v, -1.0, 1.0); }
} // namespace

FkResult forwardKinematics(const JointAngles& q, const LinkGeometry& geom) {
  const double h = geom.baseHeight, L1 = geom.upperArm, L2 = geom.forearm;
  const double c1 = std::cos(q[0]), s1 = std::sin(q[0]);
  const double c2 = std::cos(q[1]), s2 = std::sin(q[1]);
  const double c23 = std::cos(q[1] + q[2]), s23 = std::sin(q[1] + q[2]);

  const double rElbow = L1 * c2;
  const double r = L1 * c2 + L2 * c23;
  return {
      /*shoulder=*/{0.0, 0.0, h},
      /*elbow=*/{rElbow * c1, rElbow * s1, h + L1 * s2},
      /*tcp=*/{r * c1, r * s1, h + L1 * s2 + L2 * s23},
  };
}

IkResult inverseKinematics(const Vec3& target,
                           const LinkGeometry& geom,
                           const std::array<JointLimits, kNumJoints>& limits) {
  const double h = geom.baseHeight, L1 = geom.upperArm, L2 = geom.forearm;
  const double x = target[0], y = target[1], z = target[2];
  const double r = std::hypot(x, y);

  IkResult result;
  result.baseSingular = r < 1e-6;
  const double q1Front = result.baseSingular ? 0.0 : std::atan2(y, x);

  const double dz = z - h;
  const double d2 = r * r + dz * dz;
  const double c3 = (d2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);

  if (c3 > 1.0 + kReachTol || c3 < -1.0 - kReachTol) {
    result.reachable = false;
    return result;
  }
  const double c3c = clampUnit(c3);
  result.reachable = true;

  // The same TCP is reachable facing the target (planar radius +r) or yawed
  // 180° reaching back over the top (planar radius −r, needs θ2 > 90°).
  for (const bool flipped : {false, true}) {
    const double q1 = flipped ? normalizeAngle(q1Front + kPi) : q1Front;
    const double rp = flipped ? -r : r;
    for (const double sign : {1.0, -1.0}) {
      const double q3 = sign * std::acos(c3c);
      const double q2 = normalizeAngle(
          std::atan2(dz, rp) - std::atan2(L2 * std::sin(q3), L1 + L2 * std::cos(q3)));
      const JointAngles q{q1, q2, q3};

      IkSolution sol;
      sol.q = q;
      sol.branch = sign > 0 ? IkBranch::ElbowDown : IkBranch::ElbowUp;
      sol.baseFlipped = flipped;
      sol.withinLimits = true;
      for (int i = 0; i < kNumJoints; ++i) {
        sol.violated[i] = q[i] < limits[i].min - 1e-9 || q[i] > limits[i].max + 1e-9;
        if (sol.violated[i]) sol.withinLimits = false;
      }
      sol.nearSingularity = std::abs(std::sin(q3)) < kSingularityTol || result.baseSingular;

      result.solutions[result.solutionCount++] = sol;

      // Straight arm: both elbow signs coincide — keep one per flip.
      if (std::abs(q3) < 1e-9) break;
    }
  }
  return result;
}

Jacobian jacobian(const JointAngles& q, const LinkGeometry& geom) {
  const double L1 = geom.upperArm, L2 = geom.forearm;
  const double c1 = std::cos(q[0]), s1 = std::sin(q[0]);
  const double c2 = std::cos(q[1]), s2 = std::sin(q[1]);
  const double c23 = std::cos(q[1] + q[2]), s23 = std::sin(q[1] + q[2]);

  const double r = L1 * c2 + L2 * c23;
  const double drdq2 = -L1 * s2 - L2 * s23;
  const double dzdq2 = L1 * c2 + L2 * c23;

  return {
      Vec3{-r * s1, r * c1, 0.0},
      Vec3{drdq2 * c1, drdq2 * s1, dzdq2},
      Vec3{-L2 * s23 * c1, -L2 * s23 * s1, L2 * c23},
  };
}

double singularityMeasure(const JointAngles& q, const LinkGeometry& geom) {
  const double L1 = geom.upperArm, L2 = geom.forearm;
  const double r = L1 * std::cos(q[1]) + L2 * std::cos(q[1] + q[2]);
  // det(J) for this arm factors to r · L1 · L2 · sin(q3)
  return std::abs((r / (L1 + L2)) * std::sin(q[2]));
}

std::vector<std::array<double, 2>> workspaceBoundary(
    const LinkGeometry& geom,
    const std::array<JointLimits, kNumJoints>& limits,
    int samples) {
  const double h = geom.baseHeight, L1 = geom.upperArm, L2 = geom.forearm;
  const JointLimits& s = limits[static_cast<int>(Joint::Shoulder)];
  const JointLimits& e = limits[static_cast<int>(Joint::Elbow)];

  const auto point = [&](double q2, double q3) -> std::array<double, 2> {
    return {L1 * std::cos(q2) + L2 * std::cos(q2 + q3),
            h + L1 * std::sin(q2) + L2 * std::sin(q2 + q3)};
  };

  std::vector<std::array<double, 2>> pts;
  pts.reserve(4 * (samples + 1));
  // outer edge: straight arm, sweep shoulder
  for (int i = 0; i <= samples; ++i) pts.push_back(point(s.min + (s.max - s.min) * i / samples, 0.0));
  // shoulder at max, fold elbow toward its negative limit
  for (int i = 0; i <= samples; ++i) pts.push_back(point(s.max, e.min * i / samples));
  // inner edge: elbow at its negative limit, sweep shoulder back down
  for (int i = 0; i <= samples; ++i)
    pts.push_back(point(s.max - (s.max - s.min) * i / samples, e.min));
  // shoulder at min, unfold elbow back to straight
  for (int i = 0; i <= samples; ++i) pts.push_back(point(s.min, e.min - e.min * i / samples));
  return pts;
}

} // namespace rt

// Analytical kinematics for the 3-DOF arm (base yaw θ1, shoulder pitch θ2,
// elbow pitch θ3). Conventions in docs/linkage-geometry.md:
//   world Z up, X forward; θ2 from horizontal (positive up); θ3 relative to
//   the upper arm (0 = straight arm).
//
//   r   = L1·cos θ2 + L2·cos(θ2+θ3)
//   z   = h  + L1·sin θ2 + L2·sin(θ2+θ3)
//   tcp = (r·cos θ1, r·sin θ1, z)
//
// Mirrors web/src/core/kinematics.ts. Pure math — no I/O, no state, no heap
// allocation in the hot paths (embedded-firmware friendly).
#pragma once

#include <array>
#include <vector>

#include "../config/config.hpp"
#include "../math/vec3.hpp"

namespace rt {

struct FkResult {
  Vec3 shoulder;
  Vec3 elbow;
  Vec3 tcp;
};

FkResult forwardKinematics(const JointAngles& q, const LinkGeometry& geom);

enum class IkBranch : int { ElbowUp = 0, ElbowDown = 1 };

struct IkSolution {
  JointAngles q;
  IkBranch branch;
  bool baseFlipped;   // base yawed 180°, arm reaching back over the top
  bool withinLimits;
  std::array<bool, kNumJoints> violated;
  bool nearSingularity;
};

struct IkResult {
  bool reachable = false;
  bool baseSingular = false; // target on/near the yaw axis: θ1 undefined
  /** Up to 4 solutions: {front, base-flipped} × {elbow-up, elbow-down}. */
  std::array<IkSolution, 4> solutions{};
  int solutionCount = 0;
};

IkResult inverseKinematics(const Vec3& target,
                           const LinkGeometry& geom,
                           const std::array<JointLimits, kNumJoints>& limits);

/** 3×3 position Jacobian, columns = ∂tcp/∂θi (m/rad). */
using Jacobian = std::array<Vec3, kNumJoints>;

Jacobian jacobian(const JointAngles& q, const LinkGeometry& geom);

/** Manipulability-style measure: → 0 at straight-arm / base-axis singularity. */
double singularityMeasure(const JointAngles& q, const LinkGeometry& geom);

/**
 * Boundary polyline of the reachable region in the (r, z) arm plane, honoring
 * shoulder/elbow limits. Consumed by visualization backends.
 */
std::vector<std::array<double, 2>> workspaceBoundary(
    const LinkGeometry& geom,
    const std::array<JointLimits, kNumJoints>& limits,
    int samples = 40);

} // namespace rt

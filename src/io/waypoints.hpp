// CSV waypoint parsing: rows of Cartesian TCP targets (x,y,z mm) or joint
// angles (θ1,θ2,θ3 deg), converted to validated joint-space targets.
// Cartesian rows go through the analytical IK, chaining branch selection from
// the previous waypoint so the arm doesn't flip mid-sequence.
// Mirrors web/src/core/waypoints.ts.
#pragma once

#include <string>
#include <vector>

#include "../config/config.hpp"
#include "../kinematics/kinematics.hpp"

namespace rt {

enum class WaypointMode : int { Cartesian = 0, Joints = 1 };

struct WaypointParseResult {
  std::vector<JointAngles> targets;
  int skipped = 0; // rows dropped: malformed, out of limits, unreachable, colliding
  std::string firstIssue;
};

WaypointParseResult parseWaypointCsv(const std::string& text, WaypointMode mode,
                                     const RobotConfig& config, IkBranch branch,
                                     const JointAngles& fromQ);

} // namespace rt

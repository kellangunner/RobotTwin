#include "waypoints.hpp"

#include <cmath>
#include <cstdlib>
#include <sstream>

#include "../geometry/collision.hpp"
#include "../math/units.hpp"

namespace rt {

namespace {

std::string trim(const std::string& s) {
  const auto b = s.find_first_not_of(" \t\r");
  if (b == std::string::npos) return "";
  const auto e = s.find_last_not_of(" \t\r");
  return s.substr(b, e - b + 1);
}

/** Split a CSV row on commas/semicolons/tabs/spaces into numeric fields.
 *  Uses strtod rather than std::stod: header rows and malformed tokens are
 *  expected input here, and the WASM build compiles without C++ exceptions,
 *  where std::stod's throw-on-failure would abort the whole runtime. */
bool parseThreeNumbers(const std::string& line, std::array<double, 3>& out) {
  std::string norm = line;
  for (char& c : norm) {
    if (c == ',' || c == ';' || c == '\t') c = ' ';
  }
  std::istringstream ss(norm);
  int n = 0;
  std::string tok;
  while (ss >> tok) {
    if (n >= 3) return false; // too many fields
    char* end = nullptr;
    out[n] = std::strtod(tok.c_str(), &end);
    if (end != tok.c_str() + tok.size()) return false;
    ++n;
  }
  return n == 3;
}

double manhattan(const JointAngles& a, const JointAngles& b) {
  return std::abs(a[0] - b[0]) + std::abs(a[1] - b[1]) + std::abs(a[2] - b[2]);
}

} // namespace

WaypointParseResult parseWaypointCsv(const std::string& text, WaypointMode mode,
                                     const RobotConfig& config, IkBranch branch,
                                     const JointAngles& fromQ) {
  WaypointParseResult result;
  bool sawData = false;
  JointAngles prevQ = fromQ;

  const auto issue = [&](const std::string& msg) {
    ++result.skipped;
    if (result.firstIssue.empty()) result.firstIssue = msg;
  };

  std::istringstream lines(text);
  std::string raw;
  int idx = 0;
  while (std::getline(lines, raw)) {
    ++idx;
    const std::string line = trim(raw);
    if (line.empty() || line[0] == '#') continue;

    std::array<double, 3> nums{};
    if (!parseThreeNumbers(line, nums)) {
      // tolerate one leading header row (e.g. "x,y,z") silently
      if (!sawData) {
        sawData = true;
        continue;
      }
      issue("line " + std::to_string(idx) + ": expected 3 numbers");
      continue;
    }
    sawData = true;

    if (mode == WaypointMode::Joints) {
      const JointAngles q{deg2rad(nums[0]), deg2rad(nums[1]), deg2rad(nums[2])};
      bool outOfLimits = false;
      for (int i = 0; i < kNumJoints; ++i) {
        if (q[i] < config.limits[i].min - 1e-9 || q[i] > config.limits[i].max + 1e-9) {
          issue("line " + std::to_string(idx) + ": " +
                jointName(static_cast<Joint>(i)) + " out of limits");
          outOfLimits = true;
          break;
        }
      }
      if (outOfLimits) continue;
      const CollisionCheck pose = checkPose(q, config.links, config.collision);
      if (pose.colliding) {
        issue("line " + std::to_string(idx) + ": " + describe(pose.issues[0]));
        continue;
      }
      result.targets.push_back(q);
      prevQ = q;
    } else {
      const Vec3 p{mm2m(nums[0]), mm2m(nums[1]), mm2m(nums[2])};
      const IkResult res = inverseKinematics(p, config.links, config.limits);

      const IkSolution* best = nullptr;
      const IkSolution* firstWithinLimits = nullptr;
      double bestDist = 0.0;
      bool bestPreferred = false;
      for (int i = 0; res.reachable && i < res.solutionCount; ++i) {
        const IkSolution& s = res.solutions[i];
        if (!s.withinLimits) continue;
        if (!firstWithinLimits) firstWithinLimits = &s;
        if (checkPose(s.q, config.links, config.collision).colliding) continue;
        const bool preferred = s.branch == branch;
        const double d = manhattan(s.q, prevQ);
        // preferred-branch solutions beat others; ties break on distance
        if (!best || (preferred && !bestPreferred) ||
            (preferred == bestPreferred && d < bestDist)) {
          best = &s;
          bestDist = d;
          bestPreferred = preferred;
        }
      }

      if (!best) {
        if (firstWithinLimits) {
          const CollisionCheck c = checkPose(firstWithinLimits->q, config.links, config.collision);
          issue("line " + std::to_string(idx) + ": " +
                (c.issueCount > 0 ? describe(c.issues[0]) : "pose collides"));
        } else {
          issue("line " + std::to_string(idx) + ": unreachable or outside joint limits");
        }
        continue;
      }
      result.targets.push_back(best->q);
      prevQ = best->q;
    }
  }
  return result;
}

} // namespace rt

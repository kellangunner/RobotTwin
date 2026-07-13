// Shared test fixtures: the real robot config (loaded once from the same
// config/robot.yaml the web twin uses) and common poses.
#pragma once

#include "../src/config/load.hpp"
#include "../src/math/units.hpp"

#ifndef ROBOTTWIN_CONFIG_PATH
#error "ROBOTTWIN_CONFIG_PATH must be defined by the build system"
#endif

namespace rtest {

inline const rt::RobotConfig& config() {
  static const rt::RobotConfig cfg = rt::loadRobotConfig(ROBOTTWIN_CONFIG_PATH);
  return cfg;
}

inline rt::JointAngles pose(double aDeg, double bDeg, double cDeg) {
  return {rt::deg2rad(aDeg), rt::deg2rad(bDeg), rt::deg2rad(cDeg)};
}

inline const rt::JointAngles kHome = pose(0, 90, -90);

} // namespace rtest

// Loads config/robot.yaml into an SI-unit RobotConfig (native builds).
#pragma once

#include <string>

#include "config.hpp"

namespace rt {

/** Parse the YAML text of config/robot.yaml. Throws on malformed input. */
RobotConfig parseRobotConfig(const std::string& yamlText);

/** Convenience: read + parse a config file path. */
RobotConfig loadRobotConfig(const std::string& path);

} // namespace rt

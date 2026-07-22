#include "embedded_configs.hpp"

#include "config/load.hpp"
#include "embedded_yaml_gen.hpp"  // kRobotYaml / kFirmwareYaml — generated from
                                  // config/*.yaml by scripts/gen_embedded_configs.py

namespace fw {

rt::RobotConfig loadEmbeddedRobotConfig() {
  return rt::parseRobotConfig(std::string(kRobotYaml));
}

rt::HardwareConfig loadEmbeddedHardwareConfig() {
  return rt::parseHardwareConfig(std::string(kFirmwareYaml));
}

} // namespace fw

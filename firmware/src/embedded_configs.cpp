#include "embedded_configs.hpp"

#include "config/load.hpp"

// EMBED_TXTFILES null-terminates, so the blobs read as C strings.
extern const char kRobotYaml[] asm("_binary_robot_yaml_start");
extern const char kFirmwareYaml[] asm("_binary_firmware_yaml_start");

namespace fw {

rt::RobotConfig loadEmbeddedRobotConfig() {
  return rt::parseRobotConfig(std::string(kRobotYaml));
}

rt::HardwareConfig loadEmbeddedHardwareConfig() {
  return rt::parseHardwareConfig(std::string(kFirmwareYaml));
}

} // namespace fw

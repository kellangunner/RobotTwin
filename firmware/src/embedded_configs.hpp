// Boot-time access to the YAML configs baked into the firmware image
// (src/embedded_yaml_gen.hpp, generated from config/*.yaml at build time by
// scripts/gen_embedded_configs.py). The robot has no filesystem; the config
// text lives in flash and is parsed by the same loaders the native builds use.
#pragma once

#include "config/config.hpp"
#include "hardware/hardware_config.hpp"

namespace fw {

/** Parse the embedded config/robot.yaml. Throws on malformed input. */
rt::RobotConfig loadEmbeddedRobotConfig();

/** Parse the embedded config/firmware.yaml. Throws on malformed input. */
rt::HardwareConfig loadEmbeddedHardwareConfig();

} // namespace fw

// Boot-time access to the YAML configs embedded in the firmware image
// (EMBED_TXTFILES in components/robottwin_core). The robot has no filesystem;
// the config text is baked into flash and parsed by the same loaders the
// native builds use.
#pragma once

#include "config/config.hpp"
#include "hardware/hardware_config.hpp"

namespace fw {

/** Parse the embedded config/robot.yaml. Throws on malformed input. */
rt::RobotConfig loadEmbeddedRobotConfig();

/** Parse the embedded config/firmware.yaml. Throws on malformed input. */
rt::HardwareConfig loadEmbeddedHardwareConfig();

} // namespace fw

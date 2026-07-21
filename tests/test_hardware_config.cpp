#include <stdexcept>
#include <string>

#include "../src/config/yaml_lite.hpp"
#include "../src/hardware/hardware_config.hpp"
#include "../src/math/units.hpp"
#include "harness.hpp"

#ifndef ROBOTTWIN_FIRMWARE_CONFIG_PATH
#error "ROBOTTWIN_FIRMWARE_CONFIG_PATH must be defined by the build system"
#endif

using namespace rt;

namespace {

const std::string& yamlText() {
  static const std::string text = yaml::readFile(ROBOTTWIN_FIRMWARE_CONFIG_PATH);
  return text;
}

const HardwareConfig& config() {
  static const HardwareConfig cfg = parseHardwareConfig(yamlText());
  return cfg;
}

/** The real config text with one line's value swapped — for error cases. */
std::string withReplacement(const std::string& from, const std::string& to) {
  std::string text = yamlText();
  const auto pos = text.find(from);
  if (pos == std::string::npos) throw std::runtime_error("fixture: '" + from + "' not found");
  return text.replace(pos, from.size(), to);
}

bool throws(const std::string& yaml) {
  try {
    parseHardwareConfig(yaml);
    return false;
  } catch (const std::exception&) {
    return true;
  }
}

}  // namespace

RT_TEST(firmware_config_loads_identity_and_rates) {
  const auto& cfg = config();
  CHECK(cfg.name == "rt-arm-fw");
  CHECK(cfg.serialBaud == 115200);
  CHECK_CLOSE(cfg.loopHz, 1000, 1e-12);
  CHECK_CLOSE(cfg.stepTickHz, 40000, 1e-12);
}

RT_TEST(firmware_config_loads_safety_margins) {
  const auto& cfg = config();
  CHECK_CLOSE(cfg.torqueCeiling, 0.90, 1e-12);
  CHECK_CLOSE(cfg.maxStretch, 32, 1e-12);
  CHECK_CLOSE(cfg.minMoveDuration, 0.25, 1e-12);
  CHECK_CLOSE(cfg.homingTimeout, 30, 1e-12);
}

RT_TEST(firmware_config_loads_pins) {
  const auto& cfg = config();
  CHECK(cfg.enablePin == 23);
  CHECK(cfg.enableActiveLow);
  CHECK(cfg.joints[static_cast<int>(Joint::Base)].stepPin == 4);   // moved off GPIO16 (not on 30-pin boards)
  CHECK(cfg.joints[static_cast<int>(Joint::Base)].dirPin == 27);   // moved off GPIO17 (pair-dropped with 16)
  CHECK(cfg.joints[static_cast<int>(Joint::Shoulder)].stepPin == 18);
  CHECK(cfg.joints[static_cast<int>(Joint::Elbow)].stepPin == 21);
  CHECK(cfg.joints[static_cast<int>(Joint::Elbow)].uartAddress == 2);
  CHECK(cfg.tmcUartEnabled);
}

RT_TEST(firmware_config_converts_homing_values_to_si) {
  const auto& lim = config().joints[static_cast<int>(Joint::Shoulder)].limit;
  CHECK(lim.pin == 33);
  CHECK(lim.activeLow);
  CHECK_CLOSE(lim.homeAngle, deg2rad(90), 1e-12);
  CHECK(lim.seekDir == 1);
  CHECK_CLOSE(lim.seekFast, deg2rad(10), 1e-12);
  CHECK_CLOSE(lim.seekSlow, deg2rad(2), 1e-12);
  CHECK_CLOSE(lim.backoff, deg2rad(4), 1e-12);
}

RT_TEST(firmware_config_loads_homing_order) {
  const auto& order = config().homingOrder;
  CHECK(order[0] == Joint::Elbow);
  CHECK(order[1] == Joint::Shoulder);
  CHECK(order[2] == Joint::Base);
}

RT_TEST(firmware_config_rejects_bad_values) {
  CHECK(throws(withReplacement("seek_dir: -1", "seek_dir: 2")));
  CHECK(throws(withReplacement("uart_address: 2", "uart_address: 7")));
  CHECK(throws(withReplacement("irun: 16", "irun: 40")));
  CHECK(throws(withReplacement("[elbow, shoulder, base]", "[elbow, elbow, base]")));
  CHECK(throws(withReplacement("[elbow, shoulder, base]", "[elbow, shoulder]")));
  // step generator slower than the control loop makes no sense
  CHECK(throws(withReplacement("step_tick_hz: 40000", "step_tick_hz: 500")));
}

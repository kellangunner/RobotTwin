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

RT_TEST(firmware_config_converts_gripper_values_to_si) {
  const auto& g = config().gripper;
  // pin/knob_pin address the Arduino that drives the servo; relay_pin is the
  // only one of the three the ESP32 itself configures.
  CHECK(g.pin == 9);
  CHECK(g.relayPin == 13);
  CHECK(g.relayRxPin == 2);
  CHECK(g.relayBaud == 19200);
  CHECK(g.knobPin == 14);
  CHECK_CLOSE(g.pwmHz, 50, 1e-12);
  CHECK_CLOSE(s2us(g.closedPulse), 1200, 1e-9);
  CHECK_CLOSE(s2us(g.openPulse), 1800, 1e-9);
  CHECK_CLOSE(m2mm(g.maxOpening), 30.0, 1e-12);
  CHECK_CLOSE(m2mm(g.speed), 40.0, 1e-12);
  CHECK_CLOSE(m2mm(g.bootOpening), 30.0, 1e-12);
  CHECK_CLOSE(g.releaseAfter, 0, 1e-12);
}

RT_TEST(firmware_config_rejects_bad_gripper_values) {
  // Outside a hobby servo's window, or longer than the frame it lives in.
  CHECK(throws(withReplacement("open_pulse_us: 1800", "open_pulse_us: 3000")));
  CHECK(throws(withReplacement("closed_pulse_us: 1200", "closed_pulse_us: 100")));
  CHECK(throws(withReplacement("pwm_hz: 50", "pwm_hz: 5")));
  // A calibration with no span between the two ends is a typo, not a gripper.
  CHECK(throws(withReplacement("open_pulse_us: 1800", "open_pulse_us: 1210")));
  CHECK(throws(withReplacement("max_opening_mm: 30.0", "max_opening_mm: 0")));
  CHECK(throws(withReplacement("speed_mm_s: 40.0", "speed_mm_s: 0")));
  // The jaws cannot boot wider than they open.
  CHECK(throws(withReplacement("boot_opening_mm: 30.0", "boot_opening_mm: 45")));
  CHECK(throws(withReplacement("release_after_s: 0", "release_after_s: -1")));
  // GPIO34-39 are input-only, so they cannot drive the relay's UART TX.
  CHECK(throws(withReplacement("relay_pin: 13", "relay_pin: 36")));
  CHECK(throws(withReplacement("relay_baud: 19200", "relay_baud: 300")));
  // D0/D1 carry the Uno's USB console, so the relay cannot land there.
  CHECK(throws(withReplacement("relay_rx_pin: 2", "relay_rx_pin: 0")));
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

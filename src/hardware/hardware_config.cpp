#include "hardware_config.hpp"

#include <cmath>
#include <stdexcept>

#include "../config/yaml_lite.hpp"
#include "../math/units.hpp"

namespace rt {

namespace {

Joint jointFromString(const std::string& s) {
  for (int i = 0; i < kNumJoints; ++i) {
    const auto j = static_cast<Joint>(i);
    if (s == jointName(j)) return j;
  }
  throw std::runtime_error("unknown joint name: " + s);
}

int asInt(const yaml::Node& n) { return static_cast<int>(n.asDouble()); }

bool asBool(const yaml::Node& n) {
  const std::string s = n.asString();
  if (s == "true") return true;
  if (s == "false") return false;
  throw std::runtime_error("expected true/false, got: " + s);
}

LimitSwitchConfig parseLimit(const yaml::Node& n) {
  LimitSwitchConfig lim{};
  lim.pin = asInt(n.at("pin"));
  lim.activeLow = asBool(n.at("active_low"));
  lim.homeAngle = deg2rad(n.at("home_angle_deg").asDouble());
  lim.seekDir = asInt(n.at("seek_dir"));
  if (lim.seekDir != 1 && lim.seekDir != -1)
    throw std::runtime_error("limit.seek_dir must be 1 or -1");
  lim.seekFast = deg2rad(n.at("seek_fast_deg_s").asDouble());
  lim.seekSlow = deg2rad(n.at("seek_slow_deg_s").asDouble());
  lim.backoff = deg2rad(n.at("backoff_deg").asDouble());
  return lim;
}

GripperConfig parseGripper(const yaml::Node& n) {
  GripperConfig g{};
  // pin and knob_pin belong to the Arduino that drives the servo, so this
  // parser only sanity-checks them; relay_pin is the one the ESP32 actually
  // configures, and GPIO34-39 are input-only so they cannot carry a UART TX.
  g.pin = asInt(n.at("pin"));
  if (g.pin < 0) throw std::runtime_error("gripper.pin must be a valid Arduino pin");
  g.relayPin = asInt(n.at("relay_pin"));
  if (g.relayPin < 0 || g.relayPin > 33)
    throw std::runtime_error("gripper.relay_pin must be an output-capable GPIO 0..33");
  g.relayRxPin = asInt(n.at("relay_rx_pin"));
  if (g.relayRxPin < 2)
    throw std::runtime_error("gripper.relay_rx_pin must be >= 2 (D0/D1 are the Uno's USB UART)");
  g.relayBaud = asInt(n.at("relay_baud"));
  if (g.relayBaud < 1200 || g.relayBaud > 115200)
    throw std::runtime_error("gripper.relay_baud must be 1200..115200");
  g.knobPin = asInt(n.at("knob_pin"));
  if (g.knobPin < 0) throw std::runtime_error("gripper.knob_pin must be a valid Arduino pin");
  g.pwmHz = n.at("pwm_hz").asDouble();
  if (g.pwmHz < 20 || g.pwmHz > 400)
    throw std::runtime_error("gripper.pwm_hz must be 20..400 (hobby servos want 50)");

  g.closedPulse = us2s(n.at("closed_pulse_us").asDouble());
  g.openPulse = us2s(n.at("open_pulse_us").asDouble());
  // Outside this window a hobby servo either ignores the frame or slams into
  // its internal stop and stalls; a span this small is a calibration typo.
  for (const double pulse : {g.closedPulse, g.openPulse}) {
    if (pulse < us2s(400) || pulse > us2s(2600))
      throw std::runtime_error("gripper pulse widths must be 400..2600 us");
    if (pulse >= 1.0 / g.pwmHz)
      throw std::runtime_error("gripper pulse width exceeds the PWM period");
  }
  if (std::abs(g.openPulse - g.closedPulse) < us2s(50))
    throw std::runtime_error("gripper open/closed pulses must differ by >= 50 us");

  g.maxOpening = mm2m(n.at("max_opening_mm").asDouble());
  if (g.maxOpening <= 0) throw std::runtime_error("gripper.max_opening_mm must be > 0");
  g.speed = mm2m(n.at("speed_mm_s").asDouble());
  if (g.speed <= 0) throw std::runtime_error("gripper.speed_mm_s must be > 0");
  g.bootOpening = mm2m(n.at("boot_opening_mm").asDouble());
  if (g.bootOpening < 0 || g.bootOpening > g.maxOpening)
    throw std::runtime_error("gripper.boot_opening_mm must be within 0..max_opening_mm");
  g.releaseAfter = n.at("release_after_s").asDouble();
  if (g.releaseAfter < 0) throw std::runtime_error("gripper.release_after_s must be >= 0");
  return g;
}

JointPinsConfig parseJointPins(const yaml::Node& n) {
  JointPinsConfig j{};
  j.stepPin = asInt(n.at("step_pin"));
  j.dirPin = asInt(n.at("dir_pin"));
  j.invertDir = asBool(n.at("invert_dir"));
  j.uartAddress = asInt(n.at("uart_address"));
  if (j.uartAddress < 0 || j.uartAddress > 3)
    throw std::runtime_error("uart_address must be 0..3");
  j.limit = parseLimit(n.at("limit"));
  return j;
}

} // namespace

HardwareConfig parseHardwareConfig(const std::string& yamlText) {
  const yaml::Node root = yaml::parse(yamlText);
  HardwareConfig cfg{};

  cfg.name = root.at("firmware").at("name").asString();

  const yaml::Node& serial = root.at("serial");
  cfg.serialBaud = asInt(serial.at("baud"));
  cfg.telemetryHzDefault = serial.at("telemetry_hz_default").asDouble();

  const yaml::Node& control = root.at("control");
  cfg.loopHz = control.at("loop_hz").asDouble();
  cfg.stepTickHz = control.at("step_tick_hz").asDouble();
  if (cfg.loopHz <= 0 || cfg.stepTickHz < cfg.loopHz)
    throw std::runtime_error("control rates: need step_tick_hz >= loop_hz > 0");

  const yaml::Node& safety = root.at("safety");
  cfg.torqueCeiling = safety.at("torque_ceiling").asDouble();
  cfg.maxStretch = safety.at("max_stretch").asDouble();
  cfg.minMoveDuration = safety.at("min_move_duration_s").asDouble();
  cfg.homingTimeout = safety.at("homing_timeout_s").asDouble();

  const yaml::Node& enable = root.at("driver_enable");
  cfg.enablePin = asInt(enable.at("pin"));
  cfg.enableActiveLow = asBool(enable.at("active_low"));

  const yaml::Node& tmc = root.at("tmc_uart");
  cfg.tmcUartEnabled = asBool(tmc.at("enabled"));
  cfg.tmcUartTxPin = asInt(tmc.at("tx_pin"));
  cfg.tmcUartBaud = asInt(tmc.at("baud"));
  cfg.tmcIrun = asInt(tmc.at("irun"));
  cfg.tmcIhold = asInt(tmc.at("ihold"));
  if (cfg.tmcIrun < 0 || cfg.tmcIrun > 31 || cfg.tmcIhold < 0 || cfg.tmcIhold > 31)
    throw std::runtime_error("tmc_uart currents must be 0..31");

  cfg.gripper = parseGripper(root.at("gripper"));

  const auto order = root.at("homing_order");
  {
    // homing_order is an inline list of joint names: [elbow, shoulder, base]
    const std::string raw = order.asString();
    if (raw.size() < 2 || raw.front() != '[' || raw.back() != ']')
      throw std::runtime_error("homing_order must be an inline list");
    std::array<bool, kNumJoints> seen{};
    int count = 0;
    std::string item;
    for (size_t i = 1; i < raw.size(); ++i) {
      const char c = raw[i];
      if (c == ',' || c == ']') {
        if (item.empty()) throw std::runtime_error("homing_order: empty entry");
        const Joint j = jointFromString(item);
        if (count >= kNumJoints || seen[static_cast<int>(j)])
          throw std::runtime_error("homing_order must list each joint once");
        seen[static_cast<int>(j)] = true;
        cfg.homingOrder[count++] = j;
        item.clear();
      } else if (c != ' ') {
        item.push_back(c);
      }
    }
    if (count != kNumJoints)
      throw std::runtime_error("homing_order must list all joints");
  }

  const yaml::Node& joints = root.at("joints");
  for (int i = 0; i < kNumJoints; ++i)
    cfg.joints[i] = parseJointPins(joints.at(jointName(static_cast<Joint>(i))));

  return cfg;
}

HardwareConfig loadHardwareConfig(const std::string& path) {
  return parseHardwareConfig(yaml::readFile(path));
}

} // namespace rt

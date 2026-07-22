// Typed controller/electronics configuration (SI units), loaded from
// config/firmware.yaml. Describes the board wired to the robot — pins, homing,
// control rates, safety margins — while config/robot.yaml describes the robot
// itself. Parsed with the same YAML-subset reader, natively unit-tested, and
// embedded into the ESP32 image so firmware boots from the same source of
// truth as the twin.
#pragma once

#include <array>
#include <string>

#include "../config/config.hpp"

namespace rt {

struct LimitSwitchConfig {
  int pin;
  bool activeLow;
  double homeAngle;   // rad — joint angle at the switch trip point
  int seekDir;        // +1 / -1, travel direction toward the switch
  double seekFast;    // rad/s
  double seekSlow;    // rad/s
  double backoff;     // rad
};

struct JointPinsConfig {
  int stepPin;
  int dirPin;
  bool invertDir;
  int uartAddress; // TMC2209 MS1/MS2 strap address (0..3)
  LimitSwitchConfig limit;
};

struct HardwareConfig {
  std::string name;

  int serialBaud;
  double telemetryHzDefault;

  double loopHz;     // control-loop rate
  double stepTickHz; // step-generator ISR rate (also the max step rate)

  double torqueCeiling;      // retime target for peak utilization
  double maxStretch;         // reject moves needing more than this
  double minMoveDuration;    // s
  double homingTimeout;      // s per joint

  int enablePin;
  bool enableActiveLow;

  bool tmcUartEnabled;
  int tmcUartTxPin;
  int tmcUartBaud;
  int tmcIrun;  // 0..31
  int tmcIhold; // 0..31

  std::array<Joint, kNumJoints> homingOrder;
  std::array<JointPinsConfig, kNumJoints> joints; // indexed by Joint
};

/** Parse the YAML text of config/firmware.yaml. Throws on malformed input. */
HardwareConfig parseHardwareConfig(const std::string& yamlText);

/** Convenience: read + parse a config file path (native builds). */
HardwareConfig loadHardwareConfig(const std::string& path);

} // namespace rt

// Optional boot-time configuration of the three TMC2209s over their shared
// single-wire UART (TX through a 1k resistor onto the PDN_UART line; drivers
// addressed by their MS1/MS2 straps). Write-only: nothing is read back, the
// step/dir interface stays the sole motion path.
//
// Programs, per driver: UART-controlled current (IRUN/IHOLD from
// config/firmware.yaml) and the microstep resolution from config/robot.yaml —
// so the same YAML value that sets the twin's joint resolution sets the
// silicon's. When tmc_uart is disabled this is a no-op and the MS1/MS2 pin
// straps must match robot.yaml by wiring instead.
#pragma once

#include "config/config.hpp"
#include "hardware/hardware_config.hpp"

namespace fw {

/** Configure every driver listed in hw.joints. Throws on invalid settings. */
void configureTmc2209Drivers(const rt::HardwareConfig& hw, const rt::MotorParams& motor);

}  // namespace fw

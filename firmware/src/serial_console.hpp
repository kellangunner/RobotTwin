// UART0 line console speaking the host↔robot protocol (rt::proto). Reads
// newline-terminated commands, dispatches them to the MotionController, and
// interleaves the asynchronous traffic: ST telemetry at the requested rate
// and EV notifications drained from the controller's event queue.
//
// Runs in the calling task (app_main) — parsing and motion planning both
// happen here, keeping the 1 kHz control task free of heavy math.
#pragma once

#include <string>

#include "hardware/hardware_config.hpp"
#include "motion_controller.hpp"

namespace fw {

class SerialConsole {
 public:
  /** Install the UART0 driver at the configured baud rate. */
  void init(const rt::HardwareConfig& hw, MotionController& controller);

  /** Serve the protocol forever. */
  [[noreturn]] void run();

 private:
  std::string handleLine(const std::string& line);
  void writeLine(const std::string& line);

  MotionController* controller_ = nullptr;
  std::string firmwareName_;
  double telemetryHz_ = 0;
  int64_t nextTelemetryUs_ = 0;
};

}  // namespace fw

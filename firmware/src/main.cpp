// Firmware entry point. Boot order matters:
//   1. parse the embedded YAML configs (same parsers, same source of truth
//      as the native tests and the web twin)
//   2. bring up the step generator with the drivers disabled
//   3. push current/microstep settings to the TMC2209s (when UART is wired)
//   4. start the 1 kHz motion controller
//   5. serve the serial protocol forever from this task
// A failure anywhere leaves the drivers disabled and repeats the diagnosis
// on UART0 so a connected host sees why the robot never said BOOT.
#include <cstdio>
#include <exception>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "embedded_configs.hpp"
#include "motion_controller.hpp"
#include "serial_console.hpp"
#include "step_engine.hpp"
#include "tmc2209.hpp"

namespace {

// Static: app_main's frame is not a safe home for objects the control task
// and step ISR keep using after boot finishes.
fw::StepEngine gSteps;
fw::MotionController gController;
fw::SerialConsole gConsole;

}  // namespace

extern "C" void app_main() {
  try {
    const rt::RobotConfig robot = fw::loadEmbeddedRobotConfig();
    const rt::HardwareConfig hw = fw::loadEmbeddedHardwareConfig();

    gSteps.init(hw);
    fw::configureTmc2209Drivers(hw, robot.motor);
    gController.init(robot, hw, gSteps);
    gConsole.init(hw, gController);
    gConsole.run();  // never returns
  } catch (const std::exception& e) {
    for (;;) {
      std::printf("ERR FAULT boot: %s\n", e.what());
      vTaskDelay(pdMS_TO_TICKS(2000));
    }
  }
}

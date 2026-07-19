// The firmware's brain: the robot state machine (IDLE / HOMING / MOVING /
// FAULT) plus the exact planning pipeline the digital twin runs before it
// animates a move — computeMetrics → planTrajectory → retimeForTorque →
// checkPath. A command that the twin would refuse (limits, collision,
// unreachable, static torque overload) is refused here with the same verdict,
// and a command the twin would slow down is slowed down by the same stretch.
//
// Threading model:
//   - command methods run in the console task; planning (IK, audit, retime)
//     happens there, never in the control loop
//   - a 1 kHz control task samples the active trajectory / drives the homing
//     state machine and feeds absolute microstep targets to the StepEngine
//   - shared state hops between the two under a short spinlock; completion
//     and fault notifications flow back through a small event queue
#pragma once

#include <array>
#include <cstdint>
#include <string>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "config/config.hpp"
#include "hardware/hardware_config.hpp"
#include "hardware/serial_protocol.hpp"
#include "math/vec3.hpp"
#include "planning/trajectory.hpp"
#include "step_engine.hpp"

namespace fw {

/** Outcome of one command; the console formats it onto the wire. */
struct CmdResult {
  bool ok = true;
  const char* reason = nullptr;  // rt::proto::reason::* when !ok
  std::string detail;
  bool isMove = false;           // ok + isMove → "OK <verb> T=… STRETCH=…"
  double duration = 0;           // s
  double stretch = 1;            // torque-governor duration multiplier
};

/** Asynchronous notification (HOMING/HOMED/MOVE_DONE/FAULT) → EV line. */
struct Event {
  char name[16];
  char detail[48];
};

class MotionController {
 public:
  /** Store configs, set up limit-switch inputs, start the control task. */
  void init(const rt::RobotConfig& robot, const rt::HardwareConfig& hw, StepEngine& steps);

  // ---- command API (console task only) ----
  CmdResult enable();
  CmdResult disable();
  CmdResult stop();
  CmdResult home();
  CmdResult moveJoints(const rt::JointAngles& q);
  CmdResult moveLinear(const rt::Vec3& target);
  CmdResult setPayload(double kg);

  rt::proto::StateReport state() const;

  /** Drain one pending event; false when the queue is empty. */
  bool popEvent(Event& out);

 private:
  enum class Mode { Idle, Homing, Moving, Fault };
  enum class HomePhase { SeekFast, Backoff, SeekSlow, Release };

  // control task
  [[noreturn]] void controlTask();
  void tick();
  void tickMoving(int64_t nowUs);
  void tickHoming(int64_t nowUs);
  void enterPhase(HomePhase phase);
  void fault(const char* what, const char* which);

  // helpers
  bool guard(CmdResult& out, bool needHomed);
  CmdResult planAndStart(const rt::JointAngles& to);
  rt::JointAngles currentAngles() const;
  int32_t radToSteps(int joint, double angle) const;
  bool switchActive(int joint) const;
  void postEvent(const char* name, const char* detail);
  Mode currentMode() const;

  rt::RobotConfig robot_{};
  rt::HardwareConfig hw_{};
  StepEngine* steps_ = nullptr;
  std::array<double, rt::kNumJoints> stepsPerRad_{};

  mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
  Mode mode_ = Mode::Idle;
  bool homed_ = false;
  double payload_ = 0;  // kg — console task only (plan-time input)

  // active move (written at install under mux_, read each control tick)
  rt::TrajectoryPlan plan_{};
  int64_t moveStartUs_ = 0;

  // homing state machine (control task only, except the mode transitions)
  int homingIdx_ = 0;
  HomePhase phase_ = HomePhase::SeekFast;
  bool phaseEntered_ = false;
  int64_t jointStartUs_ = 0;
  int32_t backoffTarget_ = 0;

  QueueHandle_t events_ = nullptr;
};

}  // namespace fw

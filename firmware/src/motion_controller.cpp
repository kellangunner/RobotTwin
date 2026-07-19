#include "motion_controller.hpp"

#include <cmath>
#include <cstdio>

#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/task.h"

#include "drivetrain/drivetrain.hpp"
#include "geometry/collision.hpp"
#include "kinematics/kinematics.hpp"
#include "math/units.hpp"
#include "planning/retime.hpp"
#include "simulation/metrics.hpp"

namespace fw {

namespace {

using rt::proto::reason::kBusy;
using rt::proto::reason::kCollision;
using rt::proto::reason::kDisabled;
using rt::proto::reason::kFault;
using rt::proto::reason::kLimits;
using rt::proto::reason::kNotHomed;
using rt::proto::reason::kTorque;
using rt::proto::reason::kUnreachable;

// Mirrors SPEED_PLANNING_MARGIN in web/src/state/store.ts: plan below the
// hard drivetrain ceilings so torque utilization stays finite.
constexpr double kSpeedPlanningMargin = 0.8;

// Extra tracking rate over the sampled velocity so the DDA absorbs target
// quantization and tick jitter instead of falling behind the profile.
constexpr double kRateMargin = 1.25;
constexpr double kRateFloorStepsPerS = 50.0;

// Rate used to close the last rounding step or two after a profile ends.
constexpr double kSettleRateStepsPerS = 200.0;

CmdResult err(const char* reason, std::string detail) {
  CmdResult r;
  r.ok = false;
  r.reason = reason;
  r.detail = std::move(detail);
  return r;
}

}  // namespace

void MotionController::init(const rt::RobotConfig& robot, const rt::HardwareConfig& hw,
                            StepEngine& steps) {
  robot_ = robot;
  hw_ = hw;
  steps_ = &steps;
  payload_ = robot.masses.payloadDefault;

  for (int j = 0; j < rt::kNumJoints; ++j) {
    stepsPerRad_[j] = 1.0 / rt::jointResolution(robot.motor, robot.gearboxes[j]);

    const auto& lim = hw.joints[j].limit;
    const auto pin = static_cast<gpio_num_t>(lim.pin);
    gpio_reset_pin(pin);
    gpio_set_direction(pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(pin, lim.activeLow ? GPIO_PULLUP_ONLY : GPIO_PULLDOWN_ONLY);
  }

  events_ = xQueueCreate(8, sizeof(Event));

  // Core 1 keeps the 1 kHz cadence away from the main (console/planning) task.
  const auto entry = [](void* self) { static_cast<MotionController*>(self)->controlTask(); };
  xTaskCreatePinnedToCore(entry, "rt_control", 6144, this, 10, nullptr, 1);
}

// ---------------------------------------------------------------- commands

CmdResult MotionController::enable() {
  steps_->setEnabled(true);
  return {};
}

CmdResult MotionController::disable() {
  // De-energized steppers can slip (gravity backdrives the pitch joints), so
  // the position datum is no longer trustworthy: require a re-home.
  portENTER_CRITICAL(&mux_);
  steps_->haltAll();
  if (mode_ != Mode::Fault) mode_ = Mode::Idle;
  homed_ = false;
  portEXIT_CRITICAL(&mux_);
  steps_->setEnabled(false);
  return {};
}

CmdResult MotionController::stop() {
  // Also the fault-recovery verb: freeze everything, back to IDLE.
  portENTER_CRITICAL(&mux_);
  steps_->haltAll();
  mode_ = Mode::Idle;
  portEXIT_CRITICAL(&mux_);
  return {};
}

CmdResult MotionController::home() {
  CmdResult r;
  if (!guard(r, /*needHomed=*/false)) return r;

  portENTER_CRITICAL(&mux_);
  homed_ = false;
  homingIdx_ = 0;
  phase_ = HomePhase::SeekFast;
  phaseEntered_ = false;
  jointStartUs_ = esp_timer_get_time();
  mode_ = Mode::Homing;
  portEXIT_CRITICAL(&mux_);
  return {};
}

CmdResult MotionController::moveJoints(const rt::JointAngles& q) {
  CmdResult r;
  if (!guard(r, /*needHomed=*/true)) return r;

  for (int j = 0; j < rt::kNumJoints; ++j) {
    const auto& lim = robot_.limits[j];
    if (q[j] < lim.min || q[j] > lim.max)
      return err(kLimits, std::string(rt::jointName(static_cast<rt::Joint>(j))) +
                              " outside joint limits");
  }
  const auto pose = rt::checkPose(q, robot_.links, robot_.collision);
  if (pose.colliding) return err(kCollision, rt::describe(pose.issues[0]));

  return planAndStart(q);
}

CmdResult MotionController::moveLinear(const rt::Vec3& target) {
  CmdResult r;
  if (!guard(r, /*needHomed=*/true)) return r;

  const auto ik = rt::inverseKinematics(target, robot_.links, robot_.limits);
  if (!ik.reachable) return err(kUnreachable, "target outside workspace");

  // Mirror the twin's pickIkSolution: keep limit-respecting, collision-free
  // solutions and take the one closest to the current pose (no surprise
  // base flips).
  const rt::JointAngles cur = currentAngles();
  const rt::IkSolution* best = nullptr;
  const rt::IkSolution* firstWithinLimits = nullptr;
  double bestDist = 0;
  for (int i = 0; i < ik.solutionCount; ++i) {
    const auto& s = ik.solutions[i];
    if (!s.withinLimits) continue;
    if (firstWithinLimits == nullptr) firstWithinLimits = &s;
    if (rt::checkPose(s.q, robot_.links, robot_.collision).colliding) continue;
    const double d = std::abs(s.q[0] - cur[0]) + std::abs(s.q[1] - cur[1]) +
                     std::abs(s.q[2] - cur[2]);
    if (best == nullptr || d < bestDist) {
      best = &s;
      bestDist = d;
    }
  }
  if (firstWithinLimits == nullptr) return err(kLimits, "no IK solution within joint limits");
  if (best == nullptr) {
    const auto pose = rt::checkPose(firstWithinLimits->q, robot_.links, robot_.collision);
    return err(kCollision,
               pose.issueCount > 0 ? rt::describe(pose.issues[0]) : "every IK pose collides");
  }
  return planAndStart(best->q);
}

CmdResult MotionController::setPayload(double kg) {
  if (kg > robot_.masses.payloadMax)
    return err(kLimits, "payload exceeds configured maximum");
  payload_ = kg;
  return {};
}

CmdResult MotionController::planAndStart(const rt::JointAngles& to) {
  const rt::JointAngles from = currentAngles();

  const auto metrics = rt::computeMetrics(robot_, robot_.gearboxes, from, payload_);
  auto vmax = metrics.vmax;
  for (auto& v : vmax) v *= kSpeedPlanningMargin;
  const auto raw = rt::planTrajectory(from, to, vmax, metrics.amax, hw_.minMoveDuration);
  if (raw.infeasible) return err(kTorque, "a joint has no speed/acceleration budget");

  // Torque governor, same knobs as the twin: stretch the duration until the
  // predictive audit fits the ceiling; only a static overload survives that.
  const auto retimed = rt::retimeForTorque(
      raw, [&](const rt::TrajectoryPlan& p) { return rt::auditTrajectory(robot_, robot_.gearboxes, p, payload_); },
      hw_.torqueCeiling, hw_.maxStretch);
  if (retimed.limited) return err(kTorque, "static overload: cannot hold the pose");

  const auto path = rt::checkPath(retimed.plan, robot_.links, robot_.collision);
  if (path.colliding) return err(kCollision, rt::describe(path.issues[0]));

  portENTER_CRITICAL(&mux_);
  plan_ = retimed.plan;
  moveStartUs_ = esp_timer_get_time();
  mode_ = Mode::Moving;
  portEXIT_CRITICAL(&mux_);

  CmdResult r;
  r.isMove = true;
  r.duration = retimed.plan.duration;
  r.stretch = retimed.stretch;
  return r;
}

// ------------------------------------------------------------------- state

rt::proto::StateReport MotionController::state() const {
  rt::proto::StateReport s;
  s.q = currentAngles();
  s.payloadKg = payload_;
  s.enabled = steps_->enabled();
  portENTER_CRITICAL(&mux_);
  s.homed = homed_;
  switch (mode_) {
    case Mode::Idle: s.mode = "IDLE"; break;
    case Mode::Homing: s.mode = "HOMING"; break;
    case Mode::Moving: s.mode = "MOVING"; break;
    case Mode::Fault: s.mode = "FAULT"; break;
  }
  portEXIT_CRITICAL(&mux_);
  return s;
}

bool MotionController::popEvent(Event& out) {
  return events_ != nullptr && xQueueReceive(events_, &out, 0) == pdTRUE;
}

// ----------------------------------------------------------------- helpers

bool MotionController::guard(CmdResult& out, bool needHomed) {
  const Mode m = currentMode();
  if (m == Mode::Fault) {
    out = err(kFault, "send STOP to clear the fault");
    return false;
  }
  if (m != Mode::Idle) {
    out = err(kBusy, "robot is moving; send STOP first");
    return false;
  }
  if (!steps_->enabled()) {
    out = err(kDisabled, "drivers disabled; send ENABLE");
    return false;
  }
  if (needHomed && !homed_) {
    out = err(kNotHomed, "send HOME first");
    return false;
  }
  return true;
}

MotionController::Mode MotionController::currentMode() const {
  portENTER_CRITICAL(&mux_);
  const Mode m = mode_;
  portEXIT_CRITICAL(&mux_);
  return m;
}

rt::JointAngles MotionController::currentAngles() const {
  // Steps are the ground truth. Before the first HOME the datum is the boot
  // pose (position 0 == wherever the arm was racked), which is exactly what
  // homing needs and all a host can expect.
  rt::JointAngles q{};
  for (int j = 0; j < rt::kNumJoints; ++j) q[j] = steps_->position(j) / stepsPerRad_[j];
  return q;
}

int32_t MotionController::radToSteps(int joint, double angle) const {
  return static_cast<int32_t>(std::lround(angle * stepsPerRad_[joint]));
}

bool MotionController::switchActive(int joint) const {
  const auto& lim = hw_.joints[joint].limit;
  const int level = gpio_get_level(static_cast<gpio_num_t>(lim.pin));
  return lim.activeLow ? level == 0 : level != 0;
}

void MotionController::postEvent(const char* name, const char* detail) {
  Event ev{};
  std::snprintf(ev.name, sizeof ev.name, "%s", name);
  std::snprintf(ev.detail, sizeof ev.detail, "%s", detail);
  xQueueSend(events_, &ev, 0);
}

void MotionController::fault(const char* what, const char* which) {
  portENTER_CRITICAL(&mux_);
  steps_->haltAll();
  mode_ = Mode::Fault;
  homed_ = false;
  portEXIT_CRITICAL(&mux_);
  char detail[48];
  std::snprintf(detail, sizeof detail, "%s %s", what, which);
  postEvent("FAULT", detail);
}

// ------------------------------------------------------------ control loop

void MotionController::controlTask() {
  TickType_t last = xTaskGetTickCount();
  TickType_t period = pdMS_TO_TICKS(static_cast<uint32_t>(1000.0 / hw_.loopHz));
  if (period == 0) period = 1;
  for (;;) {
    vTaskDelayUntil(&last, period);
    tick();
  }
}

void MotionController::tick() {
  switch (currentMode()) {
    case Mode::Moving: tickMoving(esp_timer_get_time()); break;
    case Mode::Homing: tickHoming(esp_timer_get_time()); break;
    case Mode::Idle:
    case Mode::Fault: break;
  }
}

void MotionController::tickMoving(int64_t nowUs) {
  // Snapshot the plan under the lock; sample outside it (software-double
  // polynomial math must not run with interrupts masked).
  portENTER_CRITICAL(&mux_);
  const rt::TrajectoryPlan plan = plan_;
  const int64_t startUs = moveStartUs_;
  portEXIT_CRITICAL(&mux_);

  const double t = static_cast<double>(nowUs - startUs) * 1e-6;

  if (t < plan.duration) {
    const auto s = rt::sampleTrajectory(plan, t);
    std::array<int32_t, rt::kNumJoints> targets;
    std::array<double, rt::kNumJoints> rates;
    for (int j = 0; j < rt::kNumJoints; ++j) {
      targets[j] = radToSteps(j, s.q[j]);
      rates[j] = std::abs(s.qd[j]) * stepsPerRad_[j] * kRateMargin + kRateFloorStepsPerS;
    }
    // Re-check the mode under the same lock that STOP/DISABLE use, so a halt
    // can never be overwritten by a tick that raced it.
    portENTER_CRITICAL(&mux_);
    if (mode_ == Mode::Moving) {
      for (int j = 0; j < rt::kNumJoints; ++j) {
        steps_->setRate(j, rates[j]);
        steps_->setTarget(j, targets[j]);
      }
    }
    portEXIT_CRITICAL(&mux_);
    return;
  }

  // Profile finished: land exactly on the goal (the DDA cannot overshoot),
  // then report completion once every joint has settled.
  bool settled = true;
  portENTER_CRITICAL(&mux_);
  if (mode_ != Mode::Moving) {
    portEXIT_CRITICAL(&mux_);
    return;
  }
  for (int j = 0; j < rt::kNumJoints; ++j) {
    const int32_t goal = radToSteps(j, plan.to[j]);
    steps_->setRate(j, kSettleRateStepsPerS);
    steps_->setTarget(j, goal);
    if (steps_->position(j) != goal) settled = false;
  }
  if (settled) mode_ = Mode::Idle;
  portEXIT_CRITICAL(&mux_);
  if (settled) postEvent("MOVE_DONE", "");
}

void MotionController::enterPhase(HomePhase phase) {
  phase_ = phase;
  phaseEntered_ = false;
}

void MotionController::tickHoming(int64_t nowUs) {
  const rt::Joint joint = hw_.homingOrder[homingIdx_];
  const int j = static_cast<int>(joint);
  const auto& lim = hw_.joints[j].limit;
  const double spr = stepsPerRad_[j];

  if (nowUs - jointStartUs_ > static_cast<int64_t>(hw_.homingTimeout * 1e6)) {
    fault("homing timeout on", rt::jointName(joint));
    return;
  }

  const bool active = switchActive(j);

  // All StepEngine writes below re-check the mode under mux_ so a concurrent
  // STOP/DISABLE halt can never be overwritten (same pattern as tickMoving).
  const auto guarded = [&](auto&& writes) {
    portENTER_CRITICAL(&mux_);
    const bool live = mode_ == Mode::Homing;
    if (live) writes();
    portEXIT_CRITICAL(&mux_);
    return live;
  };

  switch (phase_) {
    case HomePhase::SeekFast:
    case HomePhase::SeekSlow: {
      const bool fast = phase_ == HomePhase::SeekFast;
      if (!phaseEntered_) {
        // Seek toward the switch; bound the travel by the joint's full span
        // plus margin — the per-joint timeout is the real watchdog.
        const double span = robot_.limits[j].max - robot_.limits[j].min + rt::deg2rad(45);
        const auto travel = static_cast<int32_t>(std::lround(span * spr));
        if (!guarded([&] {
              steps_->setRate(j, (fast ? lim.seekFast : lim.seekSlow) * spr);
              steps_->setTarget(j, steps_->position(j) + lim.seekDir * travel);
            }))
          return;
        phaseEntered_ = true;
        if (fast) postEvent("HOMING", rt::jointName(joint));
      }
      if (!active) return;
      // Switch tripped: freeze the joint where it is.
      if (!guarded([&] { steps_->setTarget(j, steps_->position(j)); })) return;
      if (fast) {
        enterPhase(HomePhase::Backoff);
      } else {
        // The slow trip is the datum: re-zero the step counter to the
        // configured switch angle, then release the switch.
        if (!guarded([&] { steps_->setPosition(j, radToSteps(j, lim.homeAngle)); })) return;
        enterPhase(HomePhase::Release);
      }
      return;
    }

    case HomePhase::Backoff:
    case HomePhase::Release: {
      if (!phaseEntered_) {
        const auto back = static_cast<int32_t>(std::lround(lim.backoff * spr));
        backoffTarget_ = steps_->position(j) - lim.seekDir * back;
        if (!guarded([&] {
              steps_->setRate(j, lim.seekFast * spr);
              steps_->setTarget(j, backoffTarget_);
            }))
          return;
        phaseEntered_ = true;
      }
      if (steps_->position(j) != backoffTarget_) return;
      if (active) {
        fault("limit switch stuck on", rt::jointName(joint));
        return;
      }
      if (phase_ == HomePhase::Backoff) {
        enterPhase(HomePhase::SeekSlow);
        return;
      }
      // Joint homed (datum set, switch released) — next joint or done.
      ++homingIdx_;
      if (homingIdx_ < rt::kNumJoints) {
        jointStartUs_ = nowUs;
        enterPhase(HomePhase::SeekFast);
        return;
      }
      portENTER_CRITICAL(&mux_);
      if (mode_ == Mode::Homing) {
        homed_ = true;
        mode_ = Mode::Idle;
      }
      portEXIT_CRITICAL(&mux_);
      postEvent("HOMED", "");
      return;
    }
  }
}

}  // namespace fw

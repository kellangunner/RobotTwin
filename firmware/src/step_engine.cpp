#include "step_engine.hpp"

#include <stdexcept>

#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_attr.h"
#include "soc/gpio_struct.h"

namespace fw {

namespace {

constexpr uint32_t kOne = 1u << 16; // DDA overflow threshold (1 step)

bool IRAM_ATTR onTimerAlarm(gptimer_handle_t, const gptimer_alarm_event_data_t*, void* user) {
  static_cast<StepEngine*>(user)->isrTick();
  return false; // no task wakeup needed
}

gpio_num_t outputPin(int pin, const char* what) {
  if (pin < 0 || pin >= 32)
    throw std::runtime_error(std::string(what) + " pin must be GPIO 0..31");
  return static_cast<gpio_num_t>(pin);
}

void configOutput(gpio_num_t pin) {
  gpio_reset_pin(pin);
  gpio_set_direction(pin, GPIO_MODE_OUTPUT);
  gpio_set_level(pin, 0);
}

} // namespace

void StepEngine::init(const rt::HardwareConfig& hw) {
  stepTickHz_ = hw.stepTickHz;
  enablePin_ = hw.enablePin;
  enableActiveLow_ = hw.enableActiveLow;

  for (int i = 0; i < rt::kNumJoints; ++i) {
    const auto& pins = hw.joints[i];
    const gpio_num_t step = outputPin(pins.stepPin, "step");
    const gpio_num_t dir = outputPin(pins.dirPin, "dir");
    configOutput(step);
    configOutput(dir);
    channels_[i].stepMask = 1u << step;
    channels_[i].dirMask = 1u << dir;
    channels_[i].invertDir = pins.invertDir;
  }

  const gpio_num_t en = outputPin(enablePin_, "enable");
  configOutput(en);
  setEnabled(false);

  // 1 MHz timebase; alarm every (1 MHz / step_tick_hz) counts, auto-reload.
  constexpr uint32_t kTimebaseHz = 1'000'000;
  const auto alarmCount = static_cast<uint64_t>(kTimebaseHz / stepTickHz_);
  if (alarmCount == 0) throw std::runtime_error("step_tick_hz exceeds the 1 MHz timebase");

  gptimer_handle_t timer = nullptr;
  gptimer_config_t timerCfg = {};
  timerCfg.clk_src = GPTIMER_CLK_SRC_DEFAULT;
  timerCfg.direction = GPTIMER_COUNT_UP;
  timerCfg.resolution_hz = kTimebaseHz;
  ESP_ERROR_CHECK(gptimer_new_timer(&timerCfg, &timer));

  gptimer_event_callbacks_t cbs = {};
  cbs.on_alarm = onTimerAlarm;
  ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer, &cbs, this));

  gptimer_alarm_config_t alarm = {};
  alarm.alarm_count = alarmCount;
  alarm.reload_count = 0;
  alarm.flags.auto_reload_on_alarm = true;
  ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &alarm));
  ESP_ERROR_CHECK(gptimer_enable(timer));
  ESP_ERROR_CHECK(gptimer_start(timer));
}

void IRAM_ATTR StepEngine::isrTick() {
  uint32_t stepMask = 0;
  for (auto& ch : channels_) {
    ch.acc += ch.rateQ16;
    if (ch.acc < kOne) continue;
    ch.acc -= kOne;

    const int32_t delta = ch.target - ch.pos;
    if (delta == 0) continue;

    const bool forward = delta > 0;
    if (forward != ch.invertDir) {
      GPIO.out_w1ts = ch.dirMask;
    } else {
      GPIO.out_w1tc = ch.dirMask;
    }
    stepMask |= ch.stepMask;
    ch.pos += forward ? 1 : -1;
  }
  if (stepMask != 0) {
    // Rising edge steps the TMC2209. The loop bookkeeping between set and
    // clear comfortably covers the 100 ns minimum high time at 240 MHz.
    GPIO.out_w1ts = stepMask;
    GPIO.out_w1tc = stepMask;
  }
}

void StepEngine::setTarget(int joint, int32_t steps) { channels_[joint].target = steps; }

void StepEngine::setRate(int joint, double stepsPerSecond) {
  double perTick = stepsPerSecond / stepTickHz_;
  if (perTick < 0) perTick = 0;
  if (perTick > 1.0) perTick = 1.0;
  channels_[joint].rateQ16 = static_cast<uint32_t>(perTick * kOne);
}

int32_t StepEngine::position(int joint) const { return channels_[joint].pos; }

void StepEngine::setPosition(int joint, int32_t steps) {
  channels_[joint].target = steps;
  channels_[joint].pos = steps;
}

void StepEngine::haltAll() {
  for (auto& ch : channels_) ch.target = ch.pos;
}

void StepEngine::setEnabled(bool on) {
  enabled_ = on;
  const bool level = on != enableActiveLow_; // active-low: enabled -> 0
  gpio_set_level(static_cast<gpio_num_t>(enablePin_), level ? 1 : 0);
}

} // namespace fw

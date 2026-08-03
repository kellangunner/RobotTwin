// RobotTwin gripper node — an Arduino Uno driving the SG-90 parallel-jaw
// gripper, independent of the ESP32 that runs the arm.
//
// Why this board exists: a stalled SG-90 pulls ~650 mA, which is exactly the
// transient that browns out a board generating step pulses, and a hobby servo
// wants 5 V logic rather than the ESP32's marginal 3.3 V. The gripper was
// always the least-coupled part of the robot — no kinematics, no trajectory, no
// homing, no torque governor — so it costs nothing to give it its own board and
// its own supply. See docs/wiring-and-bringup.md.
//
// Two ways to command it, whichever moved last wins:
//   - the knob on the analog pin, which needs no computer at all and is how
//     the mechanism gets calibrated;
//   - "G <mm>" / "GRIP <mm>" on either serial port — USB from a host, or the
//     one-way relay from the ESP32 so scripted moves can grip mid-sequence.
//
// The slew rate limiting and the opening → pulse width mapping are NOT written
// here. They come from rt::GripperAxis, the same class the twin and the ESP32
// use, from the same calibration in config/firmware.yaml. That is what lets the
// ESP32 predict when the jaws land without ever hearing back from this board.

#include <Arduino.h>
#include <Servo.h>
#include <SoftwareSerial.h>

#include "gripper_config_gen.hpp"
#include "hardware/gripper.hpp"

namespace {

constexpr rt::GripperConfig kCfg = rt::kGripperConfig;

Servo servo;
rt::GripperAxis axis;

// D3 is not wired to anything yet — SoftwareSerial demands a TX pin, and this
// reserves the one the return path would use if this link ever grows one.
constexpr uint8_t kUnusedTxPin = 3;
SoftwareSerial relay(kCfg.relayRxPin, kUnusedTxPin);

// ---- control cadence -------------------------------------------------------
// 200 Hz: four times the servo's own frame rate, so the slew is smooth, while
// each dt stays large enough to accumulate cleanly in AVR's 32-bit double.
constexpr unsigned long kTickIntervalUs = 5000UL;
unsigned long lastTickUs = 0;

// ---- knob ------------------------------------------------------------------
// A raw ADC reading jitters by a count or two forever, which would let the knob
// silently steal control back from a script. So the knob only takes over once
// it moves decisively away from where it sat when the last serial command
// arrived; a serial command then re-anchors it and reclaims control.
constexpr int kKnobDeadband = 8;   // of 1023
constexpr int kAdcMax = 1023;
int knobAnchor = -1;
bool knobOwns = false;

// ---- drive state -----------------------------------------------------------
// Nothing is driven at boot. The jaws are only assumed to be at
// boot_opening_mm, so snapping to a knob or a stale target on power-up would
// lurch the mechanism; the first deliberate command is what energizes it.
bool energized = false;
double commanded = -1;  // m — last value handed to the axis, to avoid re-arming

// ---- line assembly ---------------------------------------------------------
constexpr uint8_t kLineMax = 24;
char usbLine[kLineMax];
uint8_t usbLen = 0;
char relayLine[kLineMax];
uint8_t relayLen = 0;

/** Hand a new destination to the axis, but only if it is actually new. */
void command(double openingM) {
  // setOpening() re-arms the idle timer on every call, so calling it every tick
  // with an unchanged value would defeat release_after_s entirely.
  if (commanded >= 0 && fabs(openingM - commanded) < 1e-5) return;
  if (!axis.setOpening(openingM)) return;  // out of range; leave the jaws alone
  commanded = openingM;
  energized = true;
}

/** Stop driving: the servo goes slack and the signal line idles. */
void release() {
  energized = false;
  commanded = -1;
  if (servo.attached()) servo.detach();
}

/**
 * Handle one received line. Accepts "R" plus "G <mm>" and its long spelling
 * "GRIP <mm>", so the ESP32's terse relay frames and a human at a serial
 * monitor can use the same parser.
 */
void handleLine(char* line, Stream& echo) {
  while (*line == ' ') ++line;
  if (*line == '\0') return;

  if (line[0] == 'R' && (line[1] == '\0' || line[1] == ' ')) {
    release();
    echo.println(F("R"));
    return;
  }

  char* arg = nullptr;
  if (strncmp(line, "GRIP", 4) == 0) {
    arg = line + 4;
  } else if (line[0] == 'G') {
    arg = line + 1;
  }
  // The separator is required, so a bare "G" is a malformed command rather than
  // a silent "close fully" — atof() would happily read the empty string as 0.
  if (arg == nullptr || *arg != ' ') {
    echo.print(F("ERR BADCMD "));
    echo.println(line);
    return;
  }
  while (*arg == ' ') ++arg;
  // Likewise reject anything atof() would quietly turn into 0.
  if (*arg != '.' && (*arg < '0' || *arg > '9')) {
    echo.print(F("ERR BADCMD "));
    echo.println(line);
    return;
  }

  const double mm = atof(arg);
  if (mm < 0 || mm > kCfg.maxOpening * 1000.0) {
    echo.print(F("ERR RANGE "));
    echo.println(mm);
    return;
  }
  command(mm / 1000.0);
  knobOwns = false;              // a command outranks the knob...
  knobAnchor = analogRead(kCfg.knobPin);  // ...until the knob moves again
  echo.print(F("G "));
  echo.println(mm);
}

/** Accumulate bytes into `buf` and dispatch on newline. */
void pump(Stream& port, char* buf, uint8_t& len, Stream& echo) {
  while (port.available() > 0) {
    const char c = static_cast<char>(port.read());
    if (c == '\n' || c == '\r') {
      if (len > 0) {
        buf[len] = '\0';
        handleLine(buf, echo);
        len = 0;
      }
      continue;
    }
    // A line this long is noise on the wire, not a command — drop it rather
    // than let a stuck sender wedge the buffer forever.
    if (len >= kLineMax - 1) {
      len = 0;
      continue;
    }
    buf[len++] = c;
  }
}

void pollKnob() {
  const int raw = analogRead(kCfg.knobPin);
  if (knobAnchor < 0) {
    knobAnchor = raw;  // first reading is the reference, not a movement
    return;
  }
  if (!knobOwns && abs(raw - knobAnchor) > kKnobDeadband) knobOwns = true;
  if (!knobOwns) return;

  knobAnchor = raw;
  command(static_cast<double>(raw) / kAdcMax * kCfg.maxOpening);
}

/** Solid while holding, blinking while the jaws are moving, off when slack. */
void updateLed(bool driving) {
  if (!driving) {
    digitalWrite(LED_BUILTIN, LOW);
    return;
  }
  if (!axis.moving()) {
    digitalWrite(LED_BUILTIN, HIGH);
    return;
  }
  digitalWrite(LED_BUILTIN, (millis() / 100) % 2 == 0 ? HIGH : LOW);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  relay.begin(kCfg.relayBaud);
  axis.init(kCfg);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  lastTickUs = micros();

  Serial.println(F("RobotTwin gripper node"));
  Serial.print(F("max opening mm: "));
  Serial.println(kCfg.maxOpening * 1000.0);
  Serial.println(F("commands: G <mm> | GRIP <mm> | R"));
}

void loop() {
  // Serial and the knob are polled every pass so the node stays responsive;
  // only the slew itself runs on the fixed tick.
  pump(Serial, usbLine, usbLen, Serial);
  pump(relay, relayLine, relayLen, Serial);  // relay traffic echoes to USB
  pollKnob();

  const unsigned long now = micros();
  if (now - lastTickUs < kTickIntervalUs) return;
  const double dt = static_cast<double>(now - lastTickUs) * 1e-6;
  lastTickUs = now;

  axis.advance(dt);

  // release_after_s is honoured locally, so a knob-only bench session still
  // stops the servo buzzing once the jaws settle.
  const bool driving = energized && axis.driving();
  if (driving) {
    if (!servo.attached()) servo.attach(kCfg.pin);
    servo.writeMicroseconds(static_cast<int>(axis.pulseWidth() * 1e6 + 0.5));
  } else if (servo.attached()) {
    servo.detach();
  }
  updateLed(driving);
}

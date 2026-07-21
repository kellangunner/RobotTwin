// Line-based ASCII serial protocol between a host and the robot firmware.
// One implementation of the codec serves both ends: the ESP32 firmware parses
// commands and formats replies with it, and host clients (Python CLI, future
// HardwareRobot backend, WebSerial from the twin) do the reverse.
//
// Pure string handling — no I/O, no hardware dependency, natively unit-tested.
//
// Wire format (human-typeable, newline-terminated, human units like the YAML):
//   host → robot:  PING | STATE | HOME | STOP | ENABLE | DISABLE
//                  MOVEJ <θ1°> <θ2°> <θ3°>
//                  MOVEL <x mm> <y mm> <z mm>
//                  SETHOME <θ1°> <θ2°> <θ3°>   (manual datum; no switches)
//                  PAYLOAD <grams>
//                  TELEM <hz>
//   robot → host:  PONG <name> <version>
//                  OK <verb> [T=<s> STRETCH=<k>]
//                  ERR <reason> <detail…>
//                  ST <θ1°> <θ2°> <θ3°> <mode> homed=<0|1> en=<0|1> payload=<g>
//                  EV <name> [detail…]
#pragma once

#include <string>

#include "../config/config.hpp"
#include "../math/vec3.hpp"

namespace rt::proto {

inline constexpr const char* kProtocolVersion = "0.1.0";

enum class CommandType : int {
  Ping = 0,
  State,
  Home,
  MoveJoints,  // q (rad, converted from degrees on the wire)
  MoveLinear,  // target (m, converted from mm on the wire)
  Stop,
  Enable,
  Disable,
  SetPayload,   // value = kg (grams on the wire)
  SetTelemetry, // value = Hz
  SetHome,      // q (rad, from degrees) — declare the arm's current pose as
                // the datum (manual homing; no limit switches required)
};

struct Command {
  CommandType type{};
  JointAngles q{};  // MoveJoints
  Vec3 target{};    // MoveLinear
  double value = 0; // SetPayload / SetTelemetry
};

struct CommandParse {
  bool ok = false;
  Command command{};
  std::string error; // human-readable when !ok
};

/** Parse one received line (verb is case-insensitive, CR/LF tolerated). */
CommandParse parseCommandLine(const std::string& line);

/** Canonical wire verb for a command type (e.g. "MOVEJ"). */
const char* verbName(CommandType type);

// ---- error reasons (robot → host, second token of an ERR line) ----
namespace reason {
inline constexpr const char* kParse = "PARSE";
inline constexpr const char* kLimits = "LIMITS";
inline constexpr const char* kUnreachable = "UNREACHABLE";
inline constexpr const char* kCollision = "COLLISION";
inline constexpr const char* kTorque = "TORQUE";
inline constexpr const char* kBusy = "BUSY";
inline constexpr const char* kNotHomed = "NOT_HOMED";
inline constexpr const char* kDisabled = "DISABLED";
inline constexpr const char* kFault = "FAULT";
} // namespace reason

struct StateReport {
  JointAngles q{};        // rad (degrees on the wire)
  const char* mode = "";  // IDLE | HOMING | MOVING | FAULT
  bool homed = false;
  bool enabled = false;
  double payloadKg = 0;
};

// ---- reply / event formatting (robot → host); no trailing newline ----
std::string formatPong(const std::string& firmwareName);
std::string formatOk(const char* verb);
std::string formatMoveAck(const char* verb, double durationS, double stretch);
std::string formatError(const char* reason, const std::string& detail);
std::string formatState(const StateReport& s);
std::string formatEvent(const char* name, const std::string& detail = "");

// ---- host-side helpers (round-trip tested against the parser) ----
std::string formatMoveJoints(const JointAngles& q);
std::string formatMoveLinear(const Vec3& target);

} // namespace rt::proto

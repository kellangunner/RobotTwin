#include "serial_protocol.hpp"

#include <cctype>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "../math/units.hpp"

namespace rt::proto {

namespace {

std::vector<std::string> tokenize(const std::string& line) {
  std::vector<std::string> out;
  std::string cur;
  for (char c : line) {
    if (std::isspace(static_cast<unsigned char>(c))) {
      if (!cur.empty()) out.push_back(std::move(cur)), cur.clear();
    } else {
      cur.push_back(c);
    }
  }
  if (!cur.empty()) out.push_back(std::move(cur));
  return out;
}

std::string upper(std::string s) {
  for (char& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  return s;
}

bool parseNumber(const std::string& tok, double& out) {
  const char* begin = tok.c_str();
  char* end = nullptr;
  out = std::strtod(begin, &end);
  return end == begin + tok.size() && std::isfinite(out);
}

CommandParse fail(const std::string& msg) {
  CommandParse r;
  r.error = msg;
  return r;
}

CommandParse okCommand(Command cmd) {
  CommandParse r;
  r.ok = true;
  r.command = cmd;
  return r;
}

/** Parse exactly `n` numeric arguments after the verb. */
bool parseArgs(const std::vector<std::string>& tok, int n, std::array<double, 3>& vals,
               std::string& err) {
  if (static_cast<int>(tok.size()) != n + 1) {
    err = tok[0] + " expects " + std::to_string(n) + " argument(s)";
    return false;
  }
  for (int i = 0; i < n; ++i) {
    if (!parseNumber(tok[i + 1], vals[i])) {
      err = "bad number: " + tok[i + 1];
      return false;
    }
  }
  return true;
}

std::string fmt(const char* format, ...) {
  char buf[160];
  va_list args;
  va_start(args, format);
  std::vsnprintf(buf, sizeof buf, format, args);
  va_end(args);
  return buf;
}

} // namespace

const char* verbName(CommandType type) {
  switch (type) {
    case CommandType::Ping: return "PING";
    case CommandType::State: return "STATE";
    case CommandType::Home: return "HOME";
    case CommandType::MoveJoints: return "MOVEJ";
    case CommandType::MoveLinear: return "MOVEL";
    case CommandType::Stop: return "STOP";
    case CommandType::Enable: return "ENABLE";
    case CommandType::Disable: return "DISABLE";
    case CommandType::SetPayload: return "PAYLOAD";
    case CommandType::SetTelemetry: return "TELEM";
    case CommandType::SetHome: return "SETHOME";
  }
  return "?";
}

CommandParse parseCommandLine(const std::string& line) {
  const auto tok = tokenize(line);
  if (tok.empty()) return fail("empty line");
  const std::string verb = upper(tok[0]);

  const auto bare = [&](CommandType t) -> CommandParse {
    if (tok.size() != 1) return fail(verb + " takes no arguments");
    return okCommand({.type = t});
  };

  std::array<double, 3> v{};
  std::string err;

  if (verb == "PING") return bare(CommandType::Ping);
  if (verb == "STATE") return bare(CommandType::State);
  if (verb == "HOME") return bare(CommandType::Home);
  if (verb == "STOP") return bare(CommandType::Stop);
  if (verb == "ENABLE") return bare(CommandType::Enable);
  if (verb == "DISABLE") return bare(CommandType::Disable);

  if (verb == "MOVEJ") {
    if (!parseArgs(tok, 3, v, err)) return fail(err);
    return okCommand({.type = CommandType::MoveJoints,
                      .q = {deg2rad(v[0]), deg2rad(v[1]), deg2rad(v[2])}});
  }
  if (verb == "MOVEL") {
    if (!parseArgs(tok, 3, v, err)) return fail(err);
    return okCommand({.type = CommandType::MoveLinear,
                      .target = {mm2m(v[0]), mm2m(v[1]), mm2m(v[2])}});
  }
  if (verb == "SETHOME") {
    if (!parseArgs(tok, 3, v, err)) return fail(err);
    return okCommand({.type = CommandType::SetHome,
                      .q = {deg2rad(v[0]), deg2rad(v[1]), deg2rad(v[2])}});
  }
  if (verb == "PAYLOAD") {
    if (!parseArgs(tok, 1, v, err)) return fail(err);
    if (v[0] < 0) return fail("payload must be >= 0");
    return okCommand({.type = CommandType::SetPayload, .value = g2kg(v[0])});
  }
  if (verb == "TELEM") {
    if (!parseArgs(tok, 1, v, err)) return fail(err);
    if (v[0] < 0) return fail("rate must be >= 0");
    return okCommand({.type = CommandType::SetTelemetry, .value = v[0]});
  }

  return fail("unknown command: " + verb);
}

std::string formatPong(const std::string& firmwareName) {
  return "PONG " + firmwareName + " " + kProtocolVersion;
}

std::string formatOk(const char* verb) { return std::string("OK ") + verb; }

std::string formatMoveAck(const char* verb, double durationS, double stretch) {
  return fmt("OK %s T=%.3f STRETCH=%.2f", verb, durationS, stretch);
}

std::string formatError(const char* reason, const std::string& detail) {
  std::string out = std::string("ERR ") + reason;
  if (!detail.empty()) out += " " + detail;
  return out;
}

std::string formatState(const StateReport& s) {
  return fmt("ST %.3f %.3f %.3f %s homed=%d en=%d payload=%.0f", rad2deg(s.q[0]),
             rad2deg(s.q[1]), rad2deg(s.q[2]), s.mode, s.homed ? 1 : 0, s.enabled ? 1 : 0,
             s.payloadKg * 1000.0);
}

std::string formatEvent(const char* name, const std::string& detail) {
  std::string out = std::string("EV ") + name;
  if (!detail.empty()) out += " " + detail;
  return out;
}

std::string formatMoveJoints(const JointAngles& q) {
  return fmt("MOVEJ %.4f %.4f %.4f", rad2deg(q[0]), rad2deg(q[1]), rad2deg(q[2]));
}

std::string formatMoveLinear(const Vec3& target) {
  return fmt("MOVEL %.3f %.3f %.3f", m2mm(target[0]), m2mm(target[1]), m2mm(target[2]));
}

} // namespace rt::proto

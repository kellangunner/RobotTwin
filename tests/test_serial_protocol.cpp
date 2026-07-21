#include <string>

#include "../src/hardware/serial_protocol.hpp"
#include "../src/math/units.hpp"
#include "harness.hpp"

using namespace rt;
using namespace rt::proto;

RT_TEST(protocol_parses_bare_verbs_case_insensitively) {
  CHECK(parseCommandLine("PING").command.type == CommandType::Ping);
  CHECK(parseCommandLine("state").command.type == CommandType::State);
  CHECK(parseCommandLine("Home").command.type == CommandType::Home);
  CHECK(parseCommandLine("STOP").command.type == CommandType::Stop);
  CHECK(parseCommandLine("enable").command.type == CommandType::Enable);
  CHECK(parseCommandLine("DISABLE").command.type == CommandType::Disable);
  CHECK(parseCommandLine("  PING  ").ok);   // stray whitespace tolerated
  CHECK(parseCommandLine("PING\r").ok);     // CRLF line endings tolerated
}

RT_TEST(protocol_rejects_arguments_on_bare_verbs) {
  CHECK(!parseCommandLine("PING 1").ok);
  CHECK(!parseCommandLine("HOME now").ok);
}

RT_TEST(protocol_parses_movej_degrees_to_radians) {
  const auto r = parseCommandLine("MOVEJ 10 90 -45.5");
  CHECK(r.ok);
  CHECK(r.command.type == CommandType::MoveJoints);
  CHECK_CLOSE(r.command.q[0], deg2rad(10), 1e-12);
  CHECK_CLOSE(r.command.q[1], deg2rad(90), 1e-12);
  CHECK_CLOSE(r.command.q[2], deg2rad(-45.5), 1e-12);
}

RT_TEST(protocol_parses_movel_millimeters_to_meters) {
  const auto r = parseCommandLine("MOVEL 120 -35.5 90");
  CHECK(r.ok);
  CHECK(r.command.type == CommandType::MoveLinear);
  CHECK_CLOSE(r.command.target[0], 0.120, 1e-12);
  CHECK_CLOSE(r.command.target[1], -0.0355, 1e-12);
  CHECK_CLOSE(r.command.target[2], 0.090, 1e-12);
}

RT_TEST(protocol_parses_sethome_degrees_to_radians) {
  const auto r = parseCommandLine("SETHOME 0 90 -30");
  CHECK(r.ok);
  CHECK(r.command.type == CommandType::SetHome);
  CHECK_CLOSE(r.command.q[0], deg2rad(0), 1e-12);
  CHECK_CLOSE(r.command.q[1], deg2rad(90), 1e-12);
  CHECK_CLOSE(r.command.q[2], deg2rad(-30), 1e-12);
  CHECK(!parseCommandLine("SETHOME").ok);          // arity: needs 3 angles
  CHECK(!parseCommandLine("SETHOME 0 90").ok);     // arity
  CHECK(!parseCommandLine("SETHOME 0 90 x").ok);   // not a number
}

RT_TEST(protocol_parses_payload_grams_to_kilograms) {
  const auto r = parseCommandLine("PAYLOAD 150");
  CHECK(r.ok);
  CHECK_CLOSE(r.command.value, 0.150, 1e-12);
  CHECK(!parseCommandLine("PAYLOAD -1").ok);
}

RT_TEST(protocol_parses_telemetry_rate) {
  const auto r = parseCommandLine("TELEM 10");
  CHECK(r.ok);
  CHECK_CLOSE(r.command.value, 10, 1e-12);
  CHECK(parseCommandLine("TELEM 0").ok);  // 0 = off
  CHECK(!parseCommandLine("TELEM -5").ok);
}

RT_TEST(protocol_rejects_malformed_input) {
  CHECK(!parseCommandLine("").ok);
  CHECK(!parseCommandLine("   ").ok);
  CHECK(!parseCommandLine("FLY 1 2 3").ok);
  CHECK(!parseCommandLine("MOVEJ 1 2").ok);        // arity
  CHECK(!parseCommandLine("MOVEJ 1 2 3 4").ok);    // arity
  CHECK(!parseCommandLine("MOVEJ 1 2 banana").ok); // not a number
  CHECK(!parseCommandLine("MOVEJ 1 2 nan").ok);    // non-finite
}

RT_TEST(protocol_host_formatters_round_trip_through_parser) {
  const JointAngles q = {deg2rad(12.3), deg2rad(-45.6), deg2rad(78.9)};
  const auto rj = parseCommandLine(formatMoveJoints(q));
  CHECK(rj.ok);
  for (int i = 0; i < kNumJoints; ++i) CHECK_CLOSE(rj.command.q[i], q[i], 1e-5);

  const Vec3 t = {0.1234, -0.0567, 0.2};
  const auto rl = parseCommandLine(formatMoveLinear(t));
  CHECK(rl.ok);
  for (int i = 0; i < 3; ++i) CHECK_CLOSE(rl.command.target[i], t[i], 1e-6);
}

RT_TEST(protocol_formats_replies) {
  CHECK(formatPong("rt-arm-fw") == std::string("PONG rt-arm-fw ") + kProtocolVersion);
  CHECK(formatOk("STOP") == "OK STOP");
  CHECK(formatMoveAck("MOVEJ", 1.5, 2.0) == "OK MOVEJ T=1.500 STRETCH=2.00");
  CHECK(formatError(reason::kLimits, "base outside joint limits") ==
        "ERR LIMITS base outside joint limits");
  CHECK(formatError(reason::kBusy, "") == "ERR BUSY");
  CHECK(formatEvent("HOMED") == "EV HOMED");
  CHECK(formatEvent("FAULT", "homing timeout on elbow") == "EV FAULT homing timeout on elbow");
}

RT_TEST(protocol_formats_state_line) {
  StateReport s;
  s.q = {deg2rad(10), deg2rad(90), deg2rad(-90)};
  s.mode = "IDLE";
  s.homed = true;
  s.enabled = true;
  s.payloadKg = 0.15;
  CHECK(formatState(s) == "ST 10.000 90.000 -90.000 IDLE homed=1 en=1 payload=150");
}

RT_TEST(protocol_verb_names_match_wire_verbs) {
  CHECK(parseCommandLine(verbName(CommandType::Ping)).command.type == CommandType::Ping);
  CHECK(std::string(verbName(CommandType::MoveJoints)) == "MOVEJ");
  CHECK(std::string(verbName(CommandType::SetTelemetry)) == "TELEM");
  CHECK(std::string(verbName(CommandType::SetHome)) == "SETHOME");
}

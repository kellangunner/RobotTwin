#include <string>

#include "../src/io/waypoints.hpp"
#include "fixtures.hpp"
#include "harness.hpp"

using namespace rt;
using rtest::config;
using rtest::kHome;

RT_TEST(joints_csv_parses_degrees_and_skips_header) {
  const auto res = parseWaypointCsv("theta1,theta2,theta3\n0,90,-90\n45,45,-45\n",
                                    WaypointMode::Joints, config(), IkBranch::ElbowUp, kHome);
  CHECK(res.targets.size() == 2);
  CHECK(res.skipped == 0);
  CHECK_CLOSE(res.targets[0][1], deg2rad(90), 1e-12);
  CHECK_CLOSE(res.targets[1][0], deg2rad(45), 1e-12);
}

RT_TEST(joints_csv_skips_rows_outside_limits) {
  const auto res = parseWaypointCsv("170,90,-90\n0,90,-90", WaypointMode::Joints, config(),
                                    IkBranch::ElbowUp, kHome);
  CHECK(res.targets.size() == 1);
  CHECK(res.skipped == 1);
  CHECK(res.firstIssue.find("base out of limits") != std::string::npos);
}

RT_TEST(joints_csv_skips_colliding_poses) {
  const auto res = parseWaypointCsv("0,0,-60\n0,90,-90", WaypointMode::Joints, config(),
                                    IkBranch::ElbowUp, kHome);
  CHECK(res.targets.size() == 1);
  CHECK(res.firstIssue.find("ground") != std::string::npos);
}

RT_TEST(joints_csv_skips_malformed_and_ignores_comments) {
  const auto res = parseWaypointCsv("# demo path\n\n0,90,-90\nnot,a,row\n10,80",
                                    WaypointMode::Joints, config(), IkBranch::ElbowUp, kHome);
  CHECK(res.targets.size() == 1);
  CHECK(res.skipped == 2);
}

RT_TEST(cartesian_csv_lands_on_targets_via_ik) {
  const auto res = parseWaypointCsv("x,y,z\n120,0,210\n150,50,150", WaypointMode::Cartesian,
                                    config(), IkBranch::ElbowUp, kHome);
  CHECK(res.targets.size() == 2);
  CHECK(res.skipped == 0);
  const Vec3 tcp = forwardKinematics(res.targets[1], config().links).tcp;
  CHECK_CLOSE(tcp[0], 0.15, 1e-6);
  CHECK_CLOSE(tcp[1], 0.05, 1e-6);
  CHECK_CLOSE(tcp[2], 0.15, 1e-6);
}

RT_TEST(cartesian_csv_skips_unreachable) {
  const auto res = parseWaypointCsv("500,0,210\n120,0,210", WaypointMode::Cartesian, config(),
                                    IkBranch::ElbowUp, kHome);
  CHECK(res.targets.size() == 1);
  CHECK(res.firstIssue.find("unreachable") != std::string::npos);
}

RT_TEST(cartesian_csv_skips_targets_where_every_branch_collides) {
  const auto res = parseWaypointCsv("60,0,30\n120,0,210", WaypointMode::Cartesian, config(),
                                    IkBranch::ElbowUp, kHome);
  CHECK(res.targets.size() == 1);
  CHECK(res.skipped == 1);
}

RT_TEST(empty_csv_yields_nothing) {
  const auto res = parseWaypointCsv("", WaypointMode::Cartesian, config(), IkBranch::ElbowUp, kHome);
  CHECK(res.targets.empty());
}

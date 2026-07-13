#include <cmath>

#include "../src/drivetrain/drivetrain.hpp"
#include "../src/drivetrain/gearbox_model.hpp"
#include "../src/dynamics/dynamics.hpp"
#include "../src/kinematics/kinematics.hpp"
#include "fixtures.hpp"
#include "harness.hpp"

using namespace rt;
using rtest::config;

namespace {

MotorParams testMotor() {
  return {0.45, rpm2radps(600), 57e-7, deg2rad(1.8), 16};
}

GearboxParams testGearbox() {
  return {GearboxType::Planetary, 5, 0.85, deg2rad(0.8), 3.0, 12e-7};
}

} // namespace

RT_TEST(motor_full_torque_at_standstill_zero_at_max_speed) {
  const MotorParams m = testMotor();
  CHECK_CLOSE(motorTorqueAtSpeed(m, 0), 0.45, 1e-12);
  CHECK_CLOSE(motorTorqueAtSpeed(m, m.maxSpeed), 0.0, 1e-12);
  CHECK_CLOSE(motorTorqueAtSpeed(m, m.maxSpeed * 2), 0.0, 1e-12);
}

RT_TEST(gearbox_multiplies_torque_by_ratio_and_efficiency) {
  CHECK_CLOSE(availableJointTorque(testMotor(), testGearbox(), 0), 0.45 * 5 * 0.85, 1e-12);
}

RT_TEST(gearbox_torque_capped_by_printed_gear_strength) {
  GearboxParams strong = testGearbox();
  strong.ratio = 50;
  CHECK_CLOSE(availableJointTorque(testMotor(), strong, 0), 3.0, 1e-12);
}

RT_TEST(gearbox_derates_with_joint_speed) {
  const MotorParams m = testMotor();
  const GearboxParams gb = testGearbox();
  const double half = availableJointTorque(m, gb, maxJointSpeed(m, gb) / 2);
  CHECK_CLOSE(half, 0.45 * 5 * 0.85 / 2, 1e-12);
}

RT_TEST(gearbox_divides_speed_and_squares_inertia) {
  const MotorParams m = testMotor();
  GearboxParams gb = testGearbox();
  gb.ratio = 10;
  CHECK_CLOSE(maxJointSpeed(m, gb), m.maxSpeed / 10, 1e-12);
  CHECK_CLOSE(reflectedInertia(m, gb), (57e-7 + 12e-7) * 100, 1e-15);
}

RT_TEST(gearbox_improves_resolution_by_ratio) {
  CHECK_CLOSE(jointResolution(testMotor(), testGearbox()), deg2rad(1.8) / 16 / 5, 1e-15);
}

RT_TEST(torque_limited_speed_zero_when_standstill_fails) {
  const MotorParams m = testMotor();
  const GearboxParams gb = testGearbox(); // 1.91 N·m at standstill
  CHECK_CLOSE(torqueLimitedSpeed(m, gb, 2.5), 0.0, 1e-12);
  CHECK_CLOSE(torqueLimitedSpeed(m, gb, 3.5), 0.0, 1e-12); // above the gear cap too
}

RT_TEST(torque_limited_speed_matches_linear_curve) {
  const MotorParams m = testMotor();
  const GearboxParams gb = testGearbox();
  const double required = 0.45 * 5 * 0.85 / 2; // half the standstill torque
  const double speed = torqueLimitedSpeed(m, gb, required);
  CHECK_CLOSE(speed, maxJointSpeed(m, gb) / 2, 1e-9);
  // consistency: at that speed the drivetrain delivers exactly `required`
  CHECK_CLOSE(availableJointTorque(m, gb, speed), required, 1e-9);
}

RT_TEST(gravity_worst_case_at_full_horizontal_extension) {
  const RobotConfig& cfg = config();
  const auto tau = gravityTorques({0, 0, 0}, cfg.links, cfg.masses, 0.1);
  CHECK_CLOSE(tau[0], 0.0, 1e-12);
  CHECK_CLOSE(tau[1], 1.166, 5e-3); // hand-computed in docs/linkage-geometry.md
  CHECK_CLOSE(tau[2], 0.294, 5e-3);
}

RT_TEST(gravity_vanishes_with_arm_vertical) {
  const RobotConfig& cfg = config();
  const auto tau = gravityTorques({0, kPi / 2, 0}, cfg.links, cfg.masses, 0.1);
  CHECK_CLOSE(tau[1], 0.0, 1e-9);
  CHECK_CLOSE(tau[2], 0.0, 1e-9);
}

RT_TEST(backlash_error_grows_with_reach) {
  const auto& g = config().links;
  const std::array<double, 3> b{deg2rad(0.8), deg2rad(0.8), deg2rad(0.8)};
  const double eExt = backlashTcpError(jacobian({0, 0, -0.1}, g), b);
  const double eFold = backlashTcpError(jacobian({0, 0.8, -2.0}, g), b);
  CHECK(eExt > eFold);
  CHECK(eExt > 0.001);
}

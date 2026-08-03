#include "../src/hardware/gripper.hpp"
#include "../src/math/units.hpp"
#include "harness.hpp"

using namespace rt;

namespace {

// A deliberately round calibration: 1000 us shut, 2000 us at 40 mm, 40 mm/s,
// so one second of slew is one full stroke and 1 mm is 25 us of pulse.
GripperConfig calibration() {
  GripperConfig g{};
  g.pin = 13;
  g.pwmHz = 50;
  g.closedPulse = us2s(1000);
  g.openPulse = us2s(2000);
  g.maxOpening = mm2m(40);
  g.speed = mm2m(40);
  g.bootOpening = mm2m(40);
  g.releaseAfter = 0;
  return g;
}

GripperAxis axisAt(double openingMm, const GripperConfig& cfg) {
  GripperConfig c = cfg;
  c.bootOpening = mm2m(openingMm);
  GripperAxis axis;
  axis.init(c);
  return axis;
}

/** Slew for `seconds` in 1 ms ticks; returns how many landings were reported. */
int run(GripperAxis& axis, double seconds) {
  int landings = 0;
  for (int i = 0; i < static_cast<int>(seconds * 1000); ++i)
    if (axis.advance(0.001)) ++landings;
  return landings;
}

}  // namespace

RT_TEST(gripper_boots_at_the_configured_opening) {
  GripperAxis axis = axisAt(40, calibration());
  CHECK_CLOSE(m2mm(axis.opening()), 40, 1e-12);
  CHECK_CLOSE(m2mm(axis.target()), 40, 1e-12);
  CHECK(!axis.moving());
}

RT_TEST(gripper_maps_opening_onto_pulse_width_linearly) {
  const auto cfg = calibration();
  CHECK_CLOSE(s2us(axisAt(0, cfg).pulseWidth()), 1000, 1e-9);
  CHECK_CLOSE(s2us(axisAt(20, cfg).pulseWidth()), 1500, 1e-9);
  CHECK_CLOSE(s2us(axisAt(40, cfg).pulseWidth()), 2000, 1e-9);
  // Duty is the pulse as a fraction of the 20 ms frame: 1.5 ms → 7.5%.
  CHECK_CLOSE(axisAt(20, cfg).dutyFraction(), 0.075, 1e-12);
}

RT_TEST(gripper_maps_a_reversed_servo_the_same_way) {
  // A servo mounted the other way round: closed is the LONGER pulse.
  GripperConfig cfg = calibration();
  cfg.closedPulse = us2s(2000);
  cfg.openPulse = us2s(1000);
  CHECK_CLOSE(s2us(axisAt(0, cfg).pulseWidth()), 2000, 1e-9);
  CHECK_CLOSE(s2us(axisAt(40, cfg).pulseWidth()), 1000, 1e-9);
}

RT_TEST(gripper_rejects_openings_outside_the_mechanism) {
  GripperAxis axis = axisAt(40, calibration());
  CHECK(!axis.setOpening(mm2m(-1)));
  CHECK(!axis.setOpening(mm2m(41)));
  CHECK_CLOSE(m2mm(axis.target()), 40, 1e-12);  // rejected commands change nothing
  CHECK(axis.setOpening(mm2m(0)));
  CHECK(axis.setOpening(mm2m(40)));
}

RT_TEST(gripper_slews_at_the_configured_speed) {
  GripperAxis axis = axisAt(40, calibration());
  CHECK(axis.setOpening(mm2m(0)));
  CHECK_CLOSE(axis.timeToTarget(), 1.0, 1e-12);  // 40 mm at 40 mm/s

  run(axis, 0.25);
  CHECK_CLOSE(m2mm(axis.opening()), 30, 1e-9);   // quarter of the stroke
  CHECK(axis.moving());

  run(axis, 0.5);
  CHECK_CLOSE(m2mm(axis.opening()), 10, 1e-9);
  CHECK(axis.moving());
}

RT_TEST(gripper_lands_exactly_once_per_command) {
  GripperAxis axis = axisAt(40, calibration());
  axis.setOpening(mm2m(0));
  CHECK(run(axis, 0.9) == 0);              // still travelling
  CHECK(run(axis, 0.2) == 1);              // lands, and reports it once
  CHECK_CLOSE(m2mm(axis.opening()), 0, 1e-12);
  CHECK(!axis.moving());
  CHECK(run(axis, 1.0) == 0);              // and never again unprompted

  // A command that asks for the opening the jaws already hold still completes.
  axis.setOpening(mm2m(0));
  CHECK(run(axis, 0.01) == 1);
}

RT_TEST(gripper_freeze_stops_the_jaws_where_they_are) {
  GripperAxis axis = axisAt(40, calibration());
  axis.setOpening(mm2m(0));
  run(axis, 0.5);
  axis.freeze();
  CHECK_CLOSE(m2mm(axis.opening()), 20, 1e-9);
  CHECK(!axis.moving());
  // Frozen mid-command: no landing event for a command that never landed.
  CHECK(run(axis, 1.0) == 0);
  CHECK_CLOSE(m2mm(axis.opening()), 20, 1e-9);
}

RT_TEST(gripper_holds_the_servo_energized_by_default) {
  GripperAxis axis = axisAt(40, calibration());
  CHECK(axis.driving());
  run(axis, 60.0);
  CHECK(axis.driving());  // release_after_s = 0 means never let go
}

RT_TEST(gripper_releases_the_servo_after_the_configured_idle_time) {
  GripperConfig cfg = calibration();
  cfg.releaseAfter = 2.0;
  GripperAxis axis = axisAt(40, cfg);

  run(axis, 1.5);
  CHECK(axis.driving());
  run(axis, 1.0);
  CHECK(!axis.driving());  // 2.5 s idle

  // A new command re-energizes it, and slewing keeps it energized.
  axis.setOpening(mm2m(0));
  CHECK(axis.driving());
  run(axis, 1.0);
  CHECK(axis.driving());
  run(axis, 2.5);
  CHECK(!axis.driving());

  // So does freeze(): STOP (and re-ENABLE) mean "hold this", which a released
  // servo cannot do.
  axis.freeze();
  CHECK(axis.driving());
}

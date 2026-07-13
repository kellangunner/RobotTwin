#include <cmath>

#include "../src/drivetrain/gearbox_model.hpp"
#include "fixtures.hpp"
#include "harness.hpp"

using namespace rt;
using rtest::config;

RT_TEST(direct_drive_locks_ratio_to_one) {
  const DerivedGearbox d = deriveGearbox(config().gearboxModels, GearboxType::Direct, 17);
  CHECK_CLOSE(d.params.ratio, 1.0, 1e-12);
  CHECK(d.stages == 0);
  CHECK_CLOSE(d.params.efficiency, 0.98, 1e-12);
}

RT_TEST(cycloidal_backlash_negligible_at_any_ratio) {
  for (const double ratio : {8.0, 15.0, 25.0, 40.0}) {
    const DerivedGearbox d = deriveGearbox(config().gearboxModels, GearboxType::Cycloidal, ratio);
    CHECK_CLOSE(d.params.backlash, 0.0, 1e-12);
    CHECK_CLOSE(d.params.efficiency, 0.75, 1e-12);
    CHECK_CLOSE(d.params.maxTorque, 8.0, 1e-12);
  }
}

RT_TEST(planetary_single_stage_up_to_stage_limit) {
  const DerivedGearbox d = deriveGearbox(config().gearboxModels, GearboxType::Planetary, 5);
  CHECK(d.stages == 1);
  CHECK_CLOSE(d.params.efficiency, 0.88, 1e-12);
  CHECK_CLOSE(d.params.backlash, deg2rad(0.6), 1e-12);
}

RT_TEST(planetary_high_ratios_compound_stages) {
  const DerivedGearbox d = deriveGearbox(config().gearboxModels, GearboxType::Planetary, 20);
  CHECK(d.stages == 2);
  CHECK_CLOSE(d.params.efficiency, 0.88 * 0.88, 1e-12);
  CHECK_CLOSE(d.params.backlash, deg2rad(1.2), 1e-12);
  CHECK_CLOSE(d.params.inertia, 2 * 12e-7, 1e-15);
}

RT_TEST(planetary_stage_boundary_exact) {
  CHECK(stagesForRatio(config().gearboxModels.planetary, 6.0) == 1);
  CHECK(stagesForRatio(config().gearboxModels.planetary, 6.5) == 2);
}

RT_TEST(ratios_clamp_to_type_ranges) {
  const GearboxModels& m = config().gearboxModels;
  CHECK_CLOSE(deriveGearbox(m, GearboxType::Cycloidal, 2).params.ratio, 8.0, 1e-12);
  CHECK_CLOSE(deriveGearbox(m, GearboxType::Planetary, 100).params.ratio, 25.0, 1e-12);
  CHECK_CLOSE(ratioRange(m, GearboxType::Direct)[0], 1.0, 1e-12);
  CHECK_CLOSE(ratioRange(m, GearboxType::Direct)[1], 1.0, 1e-12);
}

RT_TEST(config_drives_consistent_with_derived_gearboxes) {
  const RobotConfig& cfg = config();
  for (int i = 0; i < kNumJoints; ++i) {
    const DerivedGearbox d = deriveGearbox(cfg.gearboxModels, cfg.drives[i].type, cfg.drives[i].ratio);
    CHECK(cfg.gearboxes[i].type == d.params.type);
    CHECK_CLOSE(cfg.gearboxes[i].ratio, d.params.ratio, 1e-12);
    CHECK_CLOSE(cfg.gearboxes[i].efficiency, d.params.efficiency, 1e-12);
    CHECK_CLOSE(cfg.gearboxes[i].backlash, d.params.backlash, 1e-12);
    CHECK_CLOSE(cfg.gearboxes[i].maxTorque, d.params.maxTorque, 1e-12);
    CHECK_CLOSE(cfg.gearboxes[i].inertia, d.params.inertia, 1e-15);
  }
}

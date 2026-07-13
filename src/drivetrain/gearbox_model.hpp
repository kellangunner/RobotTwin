// Derives full gearbox characteristics from the two real design choices:
// drive type and reduction ratio. Per-type constants live in
// config/robot.yaml (gearbox_models). Planetary reductions above one stage's
// practical limit stack stages: efficiency compounds, backlash/inertia
// accumulate. Cycloidal backlash is negligible by design assumption.
// Mirrors web/src/core/gearboxModel.ts.
#pragma once

#include "../config/config.hpp"

namespace rt {

struct DerivedGearbox {
  GearboxParams params;
  int stages; // 0 for direct drive
};

int stagesForRatio(const StagedGearboxModel& model, double ratio);

DerivedGearbox deriveGearbox(const GearboxModels& models, GearboxType type, double ratio);

std::array<double, 2> ratioRange(const GearboxModels& models, GearboxType type);

} // namespace rt

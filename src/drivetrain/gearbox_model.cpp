#include "gearbox_model.hpp"

#include <cmath>

#include "../math/units.hpp"

namespace rt {

int stagesForRatio(const StagedGearboxModel& model, double ratio) {
  if (ratio <= 1.0) return 1;
  return std::max(1, static_cast<int>(std::ceil(
                         std::log(ratio) / std::log(model.maxStageRatio) - 1e-9)));
}

DerivedGearbox deriveGearbox(const GearboxModels& models, GearboxType type, double ratio) {
  if (type == GearboxType::Planetary) {
    const StagedGearboxModel& m = models.planetary;
    const double r = clamp(ratio, m.ratioRange[0], m.ratioRange[1]);
    const int stages = stagesForRatio(m, r);
    return {
        GearboxParams{
            type,
            r,
            std::pow(m.stageEfficiency, stages),
            m.stageBacklash * stages,
            m.maxTorque,
            m.stageInertia * stages,
        },
        stages,
    };
  }

  const FixedGearboxModel& m = type == GearboxType::Direct ? models.direct : models.cycloidal;
  const double r = clamp(ratio, m.ratioRange[0], m.ratioRange[1]);
  return {
      GearboxParams{type, r, m.efficiency, m.backlash, m.maxTorque, m.inertia},
      type == GearboxType::Direct ? 0 : 1,
  };
}

std::array<double, 2> ratioRange(const GearboxModels& models, GearboxType type) {
  switch (type) {
    case GearboxType::Planetary: return models.planetary.ratioRange;
    case GearboxType::Direct: return models.direct.ratioRange;
    case GearboxType::Cycloidal: return models.cycloidal.ratioRange;
  }
  return {1.0, 1.0};
}

} // namespace rt

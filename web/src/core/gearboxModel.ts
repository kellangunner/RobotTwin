// Derives full gearbox characteristics from the user's two real choices:
// drive type and reduction ratio. The per-type constants live in
// config/robot.yaml (gearbox_models) — nothing here is hardcoded.
//
// Planetary reductions above one stage's practical limit are modeled as
// stacked stages: efficiency compounds and backlash/inertia accumulate.
// Cycloidal backlash is negligible by design assumption.

import type { GearboxParams, GearboxType } from './config';
import { clamp } from './units';

export interface FixedGearboxModel {
  kind: 'fixed';
  ratioRange: [number, number];
  efficiency: number;
  backlash: number;   // rad at output
  maxTorque: number;  // N·m
  inertia: number;    // kg·m² input-side
}

export interface StagedGearboxModel {
  kind: 'staged';
  ratioRange: [number, number];
  maxStageRatio: number;
  stageEfficiency: number;
  stageBacklash: number; // rad at output, per stage
  maxTorque: number;
  stageInertia: number;  // kg·m² per stage
}

export interface GearboxModels {
  direct: FixedGearboxModel;
  planetary: StagedGearboxModel;
  cycloidal: FixedGearboxModel;
}

export interface DriveSelection {
  type: GearboxType;
  ratio: number;
}

export interface DerivedGearbox {
  params: GearboxParams;
  /** Number of reduction stages (0 for direct drive). */
  stages: number;
}

export function stagesForRatio(model: StagedGearboxModel, ratio: number): number {
  if (ratio <= 1) return 1;
  return Math.max(1, Math.ceil(Math.log(ratio) / Math.log(model.maxStageRatio) - 1e-9));
}

export function deriveGearbox(
  models: GearboxModels,
  type: GearboxType,
  ratio: number,
): DerivedGearbox {
  if (type === 'planetary') {
    const m = models.planetary;
    const r = clamp(ratio, m.ratioRange[0], m.ratioRange[1]);
    const stages = stagesForRatio(m, r);
    return {
      params: {
        type,
        ratio: r,
        efficiency: Math.pow(m.stageEfficiency, stages),
        backlash: m.stageBacklash * stages,
        maxTorque: m.maxTorque,
        inertia: m.stageInertia * stages,
      },
      stages,
    };
  }

  const m = type === 'direct' ? models.direct : models.cycloidal;
  const r = clamp(ratio, m.ratioRange[0], m.ratioRange[1]);
  return {
    params: {
      type,
      ratio: r,
      efficiency: m.efficiency,
      backlash: m.backlash,
      maxTorque: m.maxTorque,
      inertia: m.inertia,
    },
    stages: type === 'direct' ? 0 : 1,
  };
}

export function ratioRange(models: GearboxModels, type: GearboxType): [number, number] {
  return type === 'planetary'
    ? models.planetary.ratioRange
    : type === 'direct'
      ? models.direct.ratioRange
      : models.cycloidal.ratioRange;
}

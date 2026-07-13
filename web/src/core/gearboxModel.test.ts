import { describe, expect, it } from 'vitest';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { parseRobotConfig } from './config';
import { deriveGearbox, ratioRange, stagesForRatio } from './gearboxModel';
import { deg2rad } from './units';

const yamlPath = fileURLToPath(new URL('../../../config/robot.yaml', import.meta.url));
const config = parseRobotConfig(readFileSync(yamlPath, 'utf8'));
const models = config.gearboxModels;

describe('gearbox derivation from type + ratio', () => {
  it('direct drive locks the ratio to 1', () => {
    const { params, stages } = deriveGearbox(models, 'direct', 17);
    expect(params.ratio).toBe(1);
    expect(stages).toBe(0);
    expect(params.efficiency).toBeCloseTo(0.98);
  });

  it('cycloidal backlash is negligible at any ratio', () => {
    for (const ratio of [8, 15, 25, 40]) {
      const { params } = deriveGearbox(models, 'cycloidal', ratio);
      expect(params.backlash).toBe(0);
      expect(params.efficiency).toBeCloseTo(0.75);
      expect(params.maxTorque).toBeCloseTo(8);
    }
  });

  it('planetary stays single-stage up to the stage limit', () => {
    const { params, stages } = deriveGearbox(models, 'planetary', 5);
    expect(stages).toBe(1);
    expect(params.efficiency).toBeCloseTo(0.88);
    expect(params.backlash).toBeCloseTo(deg2rad(0.6));
  });

  it('high planetary ratios compound stages, losses, and backlash', () => {
    const { params, stages } = deriveGearbox(models, 'planetary', 20);
    expect(stages).toBe(2);
    expect(params.efficiency).toBeCloseTo(0.88 * 0.88);
    expect(params.backlash).toBeCloseTo(deg2rad(1.2));
    expect(params.inertia).toBeCloseTo(2 * 12e-7);
  });

  it('stage count boundary is exact at the stage limit', () => {
    expect(stagesForRatio(models.planetary, 6)).toBe(1);
    expect(stagesForRatio(models.planetary, 6.5)).toBe(2);
  });

  it('ratios clamp to each type feasible range', () => {
    expect(deriveGearbox(models, 'cycloidal', 2).params.ratio).toBe(8);
    expect(deriveGearbox(models, 'planetary', 100).params.ratio).toBe(25);
    expect(ratioRange(models, 'direct')).toEqual([1, 1]);
  });

  it('config drives are consistent with their derived gearboxes', () => {
    for (const j of ['base', 'shoulder', 'elbow'] as const) {
      const derived = deriveGearbox(models, config.drives[j].type, config.drives[j].ratio);
      expect(config.gearboxes[j]).toEqual(derived.params);
    }
  });
});

// The app's math API, backed by the C++ core compiled to WebAssembly.
//
// This module presents exactly the same signatures as the TypeScript modules
// in this directory (kinematics.ts, metrics.ts, …), so the store and views
// are agnostic to the implementation. The TS modules remain as the parity
// mirror exercised by vitest; the running app calls C++ through here.
//
// Marshalling notes: embind value types cross the boundary as plain JS
// objects/arrays (no handles to delete). Enums cross as singleton objects and
// are converted to the union-string types used throughout the frontend.

import createRobotTwinModule from '../wasm/robottwin.js';
import type {
  GearboxParams,
  GearboxType,
  JointLimits,
  JointName,
  RobotConfig,
} from './config';
import { JOINT_NAMES } from './config';
import type { GearboxModels, DerivedGearbox } from './gearboxModel';
import type { FkResult, IkBranch, IkResult, JointAngles, Vec3 } from './kinematics';
import type { TwinMetrics, TrajectoryAudit } from './metrics';
import type { TrajectoryPlan, TrajectorySample } from './trajectory';
import type { WaypointMode, WaypointParseResult } from './waypoints';

// eslint-disable-next-line @typescript-eslint/no-explicit-any
type WasmModule = any;

const wasm: WasmModule = await createRobotTwinModule();

// ------------------------------------------------------------- enum bridges

const GB_TYPE: Record<GearboxType, unknown> = {
  direct: wasm.GearboxType.Direct,
  planetary: wasm.GearboxType.Planetary,
  cycloidal: wasm.GearboxType.Cycloidal,
};

function gbTypeName(v: unknown): GearboxType {
  if (v === wasm.GearboxType.Direct) return 'direct';
  if (v === wasm.GearboxType.Planetary) return 'planetary';
  return 'cycloidal';
}

const BRANCH: Record<IkBranch, unknown> = {
  'elbow-up': wasm.IkBranch.ElbowUp,
  'elbow-down': wasm.IkBranch.ElbowDown,
};

function branchName(v: unknown): IkBranch {
  return v === wasm.IkBranch.ElbowUp ? 'elbow-up' : 'elbow-down';
}

const JOINT_ENUMS = [wasm.Joint.Base, wasm.Joint.Shoulder, wasm.Joint.Elbow] as const;

function jointNameOf(v: unknown): JointName {
  const i = JOINT_ENUMS.findIndex((j) => j === v);
  return JOINT_NAMES[i >= 0 ? i : 0];
}

// -------------------------------------------------------- struct conversions

const toLimitsArray = (limits: Record<JointName, JointLimits>) =>
  JOINT_NAMES.map((n) => limits[n]);

function toWasmGearbox(gb: GearboxParams) {
  return { ...gb, type: GB_TYPE[gb.type] };
}

const toGearboxArray = (gearboxes: Record<JointName, GearboxParams>) =>
  JOINT_NAMES.map((n) => toWasmGearbox(gearboxes[n]));

// The static parts of the config never change → convert once per object.
const configCache = new WeakMap<RobotConfig, unknown>();

function toWasmConfig(config: RobotConfig) {
  let cached = configCache.get(config);
  if (!cached) {
    cached = {
      name: config.name,
      links: config.links,
      limits: toLimitsArray(config.limits),
      masses: config.masses,
      motor: config.motor,
      gearboxModels: config.gearboxModels,
      drives: JOINT_NAMES.map((n) => ({
        type: GB_TYPE[config.drives[n].type],
        ratio: config.drives[n].ratio,
      })),
      gearboxes: toGearboxArray(config.gearboxes),
      collision: config.collision,
    };
    configCache.set(config, cached);
  }
  return cached;
}

// ------------------------------------------------------------------ the API

export function forwardKinematics(q: JointAngles, geom: RobotConfig['links']): FkResult {
  return wasm.forwardKinematics(q, geom);
}

export function jacobian(q: JointAngles, geom: RobotConfig['links']): [Vec3, Vec3, Vec3] {
  return wasm.jacobian(q, geom);
}

export function singularityMeasure(q: JointAngles, geom: RobotConfig['links']): number {
  return wasm.singularityMeasure(q, geom);
}

export function inverseKinematics(
  target: Vec3,
  geom: RobotConfig['links'],
  limits: Record<JointName, JointLimits>,
): IkResult {
  const raw = wasm.inverseKinematics(target, geom, toLimitsArray(limits));
  return {
    reachable: raw.reachable,
    baseSingular: raw.baseSingular,
    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    solutions: raw.solutions.map((s: any) => ({
      q: s.q as JointAngles,
      branch: branchName(s.branch),
      baseFlipped: s.baseFlipped,
      withinLimits: s.withinLimits,
      violated: JOINT_NAMES.filter((_, i) => s.violated[i]),
      nearSingularity: s.nearSingularity,
    })),
  };
}

export function workspaceBoundary(
  geom: RobotConfig['links'],
  limits: Record<JointName, JointLimits>,
  samples = 40,
): Array<[number, number]> {
  return wasm.workspaceBoundary(geom, toLimitsArray(limits), samples);
}

export function computeMetrics(
  config: RobotConfig,
  gearboxes: Record<JointName, GearboxParams>,
  q: JointAngles,
  payload: number,
): TwinMetrics {
  const raw = wasm.computeMetrics(toWasmConfig(config), toGearboxArray(gearboxes), q, payload);
  return {
    ...raw,
    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    joints: raw.joints.map((j: any, i: number) => ({ ...j, name: JOINT_NAMES[i] })),
  };
}

export function auditTrajectory(
  config: RobotConfig,
  gearboxes: Record<JointName, GearboxParams>,
  plan: TrajectoryPlan,
  payload: number,
  samples = 120,
): TrajectoryAudit {
  const raw = wasm.auditTrajectory(
    toWasmConfig(config), toGearboxArray(gearboxes), plan, payload, samples);
  return {
    peakUtilization: raw.peakUtilization,
    peakJoint: jointNameOf(raw.peakJoint),
    skippedSteps: raw.skippedSteps,
  };
}

export function planTrajectory(
  from: JointAngles,
  to: JointAngles,
  vmax: [number, number, number],
  amax: [number, number, number],
  minDuration = 0.25,
): TrajectoryPlan {
  return wasm.planTrajectory(from, to, vmax, amax, minDuration);
}

export function sampleTrajectory(plan: TrajectoryPlan, t: number): TrajectorySample {
  return wasm.sampleTrajectory(plan, t);
}

export function checkPose(
  q: JointAngles,
  geom: RobotConfig['links'],
  model: RobotConfig['collision'],
): { colliding: boolean; issues: string[] } {
  return wasm.checkPose(q, geom, model);
}

export function checkPath(
  plan: TrajectoryPlan,
  geom: RobotConfig['links'],
  model: RobotConfig['collision'],
): { colliding: boolean; issues: string[] } {
  return wasm.checkPath(plan, geom, model);
}

export function deriveGearbox(
  models: GearboxModels,
  type: GearboxType,
  ratio: number,
): DerivedGearbox {
  const raw = wasm.deriveGearbox(models, GB_TYPE[type], ratio);
  return {
    params: { ...raw.params, type: gbTypeName(raw.params.type) },
    stages: raw.stages,
  };
}

export function ratioRange(models: GearboxModels, type: GearboxType): [number, number] {
  return wasm.ratioRange(models, GB_TYPE[type]);
}

export function parseWaypointCsv(
  text: string,
  mode: WaypointMode,
  config: RobotConfig,
  branch: IkBranch,
  fromQ: JointAngles,
): WaypointParseResult {
  const raw = wasm.parseWaypointCsv(
    text,
    mode === 'cartesian' ? wasm.WaypointMode.Cartesian : wasm.WaypointMode.Joints,
    toWasmConfig(config),
    BRANCH[branch],
    fromQ,
  );
  return {
    targets: raw.targets as JointAngles[],
    skipped: raw.skipped,
    firstIssue: raw.firstIssue === '' ? null : raw.firstIssue,
  };
}

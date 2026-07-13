// Typed robot configuration, loaded from config/robot.yaml (repo root).
// The YAML uses human units; everything in RobotConfig is SI.

import { load } from 'js-yaml';
import type { CollisionModel } from './collision';
import type { DriveSelection, GearboxModels } from './gearboxModel';
import { deriveGearbox } from './gearboxModel';
import { deg2rad, g2kg, gcm2_to_kgm2, mm2m, rpm2radps } from './units';

export type JointName = 'base' | 'shoulder' | 'elbow';
export const JOINT_NAMES: readonly JointName[] = ['base', 'shoulder', 'elbow'] as const;

export interface LinkGeometry {
  baseHeight: number; // m — table to shoulder axis
  upperArm: number;   // m — shoulder axis to elbow axis
  forearm: number;    // m — elbow axis to TCP
}

export interface JointLimits {
  min: number; // rad
  max: number; // rad
}

export interface MassModel {
  upperArm: number;   // kg, CoM at mid-link
  forearm: number;    // kg, CoM at mid-link
  elbowMotor: number; // kg, at elbow joint
  gripper: number;    // kg, at TCP
  payloadDefault: number; // kg
  payloadMax: number;     // kg
}

export interface MotorParams {
  holdingTorque: number; // N·m at standstill
  maxSpeed: number;      // rad/s (usable ceiling, torque → 0 here)
  rotorInertia: number;  // kg·m²
  stepAngle: number;     // rad
  microstepping: number;
}

export type GearboxType = 'planetary' | 'cycloidal' | 'direct';

export interface GearboxParams {
  type: GearboxType;
  ratio: number;      // output:input, >= 1
  efficiency: number; // 0..1
  backlash: number;   // rad, at the output
  maxTorque: number;  // N·m — printed-gear strength cap
  inertia: number;    // kg·m² — input-side gearbox inertia
}

export interface RobotConfig {
  name: string;
  links: LinkGeometry;
  limits: Record<JointName, JointLimits>;
  masses: MassModel;
  motor: MotorParams;
  /** Per-type characteristic models the twin derives gearboxes from. */
  gearboxModels: GearboxModels;
  /** The independent variables: drive type + ratio per joint. */
  drives: Record<JointName, DriveSelection>;
  /** Full characteristics derived from drives via gearboxModels. */
  gearboxes: Record<JointName, GearboxParams>;
  /** Capsule/sphere envelopes for self- and ground-collision checks. */
  collision: CollisionModel;
}

// eslint-disable-next-line @typescript-eslint/no-explicit-any
function modelsFromRaw(raw: any): GearboxModels {
  return {
    direct: {
      kind: 'fixed',
      ratioRange: raw.direct.ratio_range,
      efficiency: raw.direct.efficiency,
      backlash: deg2rad(raw.direct.backlash_deg),
      maxTorque: raw.direct.max_torque_nm,
      inertia: gcm2_to_kgm2(raw.direct.inertia_g_cm2),
    },
    planetary: {
      kind: 'staged',
      ratioRange: raw.planetary.ratio_range,
      maxStageRatio: raw.planetary.max_stage_ratio,
      stageEfficiency: raw.planetary.stage_efficiency,
      stageBacklash: deg2rad(raw.planetary.stage_backlash_deg),
      maxTorque: raw.planetary.max_torque_nm,
      stageInertia: gcm2_to_kgm2(raw.planetary.stage_inertia_g_cm2),
    },
    cycloidal: {
      kind: 'fixed',
      ratioRange: raw.cycloidal.ratio_range,
      efficiency: raw.cycloidal.efficiency,
      backlash: deg2rad(raw.cycloidal.backlash_deg),
      maxTorque: raw.cycloidal.max_torque_nm,
      inertia: gcm2_to_kgm2(raw.cycloidal.inertia_g_cm2),
    },
  };
}

/** Parse the YAML text of config/robot.yaml into an SI-unit RobotConfig. */
export function parseRobotConfig(yamlText: string): RobotConfig {
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  const raw = load(yamlText) as any;

  const limits: Record<JointName, JointLimits> = {} as Record<JointName, JointLimits>;
  for (const j of JOINT_NAMES) {
    const [min, max] = raw.joints[j].limits_deg as [number, number];
    limits[j] = { min: deg2rad(min), max: deg2rad(max) };
  }

  const gearboxModels = modelsFromRaw(raw.gearbox_models);
  const drives: Record<JointName, DriveSelection> = {} as Record<JointName, DriveSelection>;
  const gearboxes: Record<JointName, GearboxParams> = {} as Record<JointName, GearboxParams>;
  for (const j of JOINT_NAMES) {
    const { type, ratio } = raw.gearboxes[j] as { type: GearboxType; ratio: number };
    const derived = deriveGearbox(gearboxModels, type, ratio);
    drives[j] = { type, ratio: derived.params.ratio };
    gearboxes[j] = derived.params;
  }

  return {
    name: raw.robot.name,
    links: {
      baseHeight: mm2m(raw.links.base_height_mm),
      upperArm: mm2m(raw.links.upper_arm_mm),
      forearm: mm2m(raw.links.forearm_mm),
    },
    limits,
    masses: {
      upperArm: g2kg(raw.masses.upper_arm_g),
      forearm: g2kg(raw.masses.forearm_g),
      elbowMotor: g2kg(raw.masses.elbow_motor_g),
      gripper: g2kg(raw.masses.gripper_g),
      payloadDefault: g2kg(raw.masses.payload_default_g),
      payloadMax: g2kg(raw.masses.payload_max_g),
    },
    motor: {
      holdingTorque: raw.motor.holding_torque_nm,
      maxSpeed: rpm2radps(raw.motor.max_speed_rpm),
      rotorInertia: gcm2_to_kgm2(raw.motor.rotor_inertia_g_cm2),
      stepAngle: deg2rad(raw.motor.step_angle_deg),
      microstepping: raw.motor.microstepping,
    },
    gearboxModels,
    drives,
    gearboxes,
    collision: {
      groundClearance: mm2m(raw.collision.ground_clearance_mm),
      columnRadius: mm2m(raw.collision.column_radius_mm),
      columnTop: mm2m(raw.collision.column_top_mm),
      baseRadius: mm2m(raw.collision.base_radius_mm),
      baseTop: mm2m(raw.collision.base_top_mm),
      shoulderRadius: mm2m(raw.collision.shoulder_radius_mm),
      upperArmRadius: mm2m(raw.collision.upper_arm_radius_mm),
      forearmRadius: mm2m(raw.collision.forearm_radius_mm),
      gripperExtent: mm2m(raw.collision.gripper_extent_mm),
      elbowTrim: raw.collision.elbow_trim,
    },
  };
}

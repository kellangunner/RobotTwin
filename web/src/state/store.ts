// Application state: the twin's independent variables (gearboxes, payload),
// the commanded pose, and the motion playback. All math is delegated to core/.

import { create } from 'zustand';
import robotYaml from '../../../config/robot.yaml?raw';
import type { GearboxParams, JointName, RobotConfig } from '../core/config';
import { parseRobotConfig } from '../core/config';
import type { DriveSelection } from '../core/gearboxModel';
import { deriveGearbox, ratioRange } from '../core/gearboxModel';
import type { JointAngles, Vec3, IkBranch } from '../core/kinematics';
import { forwardKinematics, inverseKinematics } from '../core/kinematics';
import { auditTrajectory, computeMetrics } from '../core/metrics';
import type { TrajectoryPlan } from '../core/trajectory';
import { planTrajectory, sampleTrajectory } from '../core/trajectory';
import { deg2rad } from '../core/units';

export const config: RobotConfig = parseRobotConfig(robotYaml);

export const HOME_POSE: JointAngles = [0, deg2rad(90), deg2rad(-90)];

export type ControlMode = 'target' | 'joints';

export type IkStatus =
  | { kind: 'ok'; nearSingularity: boolean }
  | { kind: 'unreachable' }
  | { kind: 'limits' };

export interface MoveReport {
  duration: number;         // s
  peakUtilization: number;  // fraction of available torque, worst joint
  peakJoint: JointName;
  skippedSteps: boolean;    // peak > 1 → open-loop steppers would lose position
  infeasible: boolean;      // a joint had zero speed/accel budget at plan time
}

interface Motion {
  plan: TrajectoryPlan;
  report: MoveReport; // predictive audit, computed at plan time
  startedAt: number;  // performance.now() ms
  elapsed: number;
}

export interface SequenceState {
  targets: JointAngles[];
  /** Next leg to start (leg index-1 is currently executing or just finished). */
  index: number;
  /** Dwell deadline (ms) between legs; null while a leg is in flight. */
  holdUntil: number | null;
  // aggregated across legs for the final move report
  peakUtilization: number;
  peakJoint: JointName;
  infeasible: boolean;
  totalDuration: number;
}

/** Dwell at each waypoint before moving on. */
export const WAYPOINT_HOLD_S = 0.8;

interface TwinState {
  /** The independent variables: drive type + reduction ratio per joint. */
  drives: Record<JointName, DriveSelection>;
  /** Characteristics derived from drives via config.gearboxModels. */
  gearboxes: Record<JointName, GearboxParams>;
  payload: number; // kg
  controlMode: ControlMode;
  branch: IkBranch;
  target: Vec3;      // m
  q: JointAngles;    // animated pose
  ikStatus: IkStatus;
  motion: Motion | null;
  sequence: SequenceState | null;
  lastMove: MoveReport | null;
  trace: Vec3[];
  showWorkspace: boolean;

  setDrive: (joint: JointName, patch: Partial<DriveSelection>) => void;
  setPayload: (kg: number) => void;
  setControlMode: (mode: ControlMode) => void;
  setBranch: (branch: IkBranch) => void;
  setTarget: (target: Vec3) => void;
  setJointTarget: (index: number, angle: number) => void;
  goHome: () => void;
  toggleWorkspace: () => void;
  /** Visit each joint-space target in order, dwelling between legs. */
  startSequence: (targets: JointAngles[]) => void;
  clearTrace: () => void;
}

const TRACE_CAP = 800;
/** Plan below the hard drivetrain ceilings so utilization stays finite. */
const SPEED_PLANNING_MARGIN = 0.8;

function pickIkSolution(
  target: Vec3,
  branch: IkBranch,
  current: JointAngles,
): { q: JointAngles | null; status: IkStatus } {
  const res = inverseKinematics(target, config.links, config.limits);
  if (!res.reachable) return { q: null, status: { kind: 'unreachable' } };

  const usable = res.solutions.filter((s) => s.withinLimits);
  if (usable.length === 0) return { q: null, status: { kind: 'limits' } };

  // Prefer the requested elbow branch; among those, stay closest to the
  // current pose (avoids surprise base flips while dragging the target).
  const preferred = usable.filter((s) => s.branch === branch);
  const pool = preferred.length > 0 ? preferred : usable;
  const dist = (q: JointAngles) =>
    Math.abs(q[0] - current[0]) + Math.abs(q[1] - current[1]) + Math.abs(q[2] - current[2]);
  const best = pool.reduce((p, s) => (dist(s.q) < dist(p.q) ? s : p));
  return { q: best.q, status: { kind: 'ok', nearSingularity: best.nearSingularity } };
}

export const useTwinStore = create<TwinState>((set, get) => {
  // Motion playback runs on a wall-clock interval, NOT the render loop:
  // browsers throttle requestAnimationFrame when the canvas isn't composited,
  // and simulation time must never depend on rendering cadence anyway.
  let clock: ReturnType<typeof setInterval> | null = null;

  const stopClock = () => {
    if (clock !== null) {
      clearInterval(clock);
      clock = null;
    }
  };

  // Ticks only animate; all physics conclusions were drawn at plan time.
  const tick = () => {
    const s = get();
    if (!s.motion) {
      stopClock();
      return;
    }
    const { plan, report, startedAt } = s.motion;
    const elapsed = (performance.now() - startedAt) / 1000;
    const sample = sampleTrajectory(plan, elapsed);

    const tcp = forwardKinematics(sample.q, config.links).tcp;
    const trace =
      s.trace.length >= TRACE_CAP ? [...s.trace.slice(-TRACE_CAP + 1), tcp] : [...s.trace, tcp];

    if (elapsed >= plan.duration) {
      stopClock();
      set({ q: plan.to, motion: null, trace, lastMove: report });
    } else {
      set({ q: sample.q, motion: { plan, report, startedAt, elapsed }, trace });
    }
  };

  const startMove = (to: JointAngles): Partial<TwinState> => {
    const s = get();
    const metrics = computeMetrics(config, s.gearboxes, s.q, s.payload);
    const vmax = metrics.vmax.map((v) => v * SPEED_PLANNING_MARGIN) as [number, number, number];
    const plan = planTrajectory(s.q, to, vmax, metrics.amax);
    const audit = auditTrajectory(config, s.gearboxes, plan, s.payload);
    const report: MoveReport = {
      duration: plan.duration,
      peakUtilization: audit.peakUtilization,
      peakJoint: audit.peakJoint,
      skippedSteps: audit.skippedSteps,
      infeasible: plan.infeasible,
    };
    stopClock();
    clock = setInterval(tick, 1000 / 30);
    return {
      motion: { plan, report, startedAt: performance.now(), elapsed: 0 },
      lastMove: null,
    };
  };

  return {
    drives: { ...config.drives },
    gearboxes: { ...config.gearboxes },
    payload: config.masses.payloadDefault,
    controlMode: 'target',
    branch: 'elbow-up',
    target: forwardKinematics(HOME_POSE, config.links).tcp,
    q: HOME_POSE,
    ikStatus: { kind: 'ok', nearSingularity: false },
    motion: null,
    lastMove: null,
    trace: [],
    showWorkspace: true,

    setDrive: (joint, patch) =>
      set((s) => {
        const next = { ...s.drives[joint], ...patch };
        // switching type re-clamps the ratio into the new type's feasible range
        if (patch.type !== undefined && patch.ratio === undefined) {
          const [lo, hi] = ratioRange(config.gearboxModels, next.type);
          next.ratio = Math.min(hi, Math.max(lo, next.ratio));
        }
        const derived = deriveGearbox(config.gearboxModels, next.type, next.ratio);
        return {
          drives: { ...s.drives, [joint]: { type: next.type, ratio: derived.params.ratio } },
          gearboxes: { ...s.gearboxes, [joint]: derived.params },
        };
      }),

    setPayload: (kg) => set({ payload: kg }),

    setControlMode: (mode) => set({ controlMode: mode }),

    setBranch: (branch) => {
      const s = get();
      const { q, status } = pickIkSolution(s.target, branch, s.q);
      set({ branch, ikStatus: status, ...(q ? startMove(q) : {}) });
    },

    setTarget: (target) => {
      const s = get();
      const { q, status } = pickIkSolution(target, s.branch, s.q);
      set({ target, ikStatus: status, ...(q ? startMove(q) : {}) });
    },

    setJointTarget: (index, angle) => {
      const s = get();
      const to = [...(s.motion ? s.motion.plan.to : s.q)] as JointAngles;
      to[index] = angle;
      const tcp = forwardKinematics(to, config.links).tcp;
      set({ target: tcp, ikStatus: { kind: 'ok', nearSingularity: false }, ...startMove(to) });
    },

    goHome: () => {
      const tcp = forwardKinematics(HOME_POSE, config.links).tcp;
      set({
        target: tcp,
        ikStatus: { kind: 'ok', nearSingularity: false },
        trace: [],
        ...startMove(HOME_POSE),
      });
    },

    toggleWorkspace: () => set((s) => ({ showWorkspace: !s.showWorkspace })),
  };
});

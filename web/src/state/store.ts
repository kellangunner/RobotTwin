// Application state: the twin's independent variables (gearboxes, payload),
// the commanded pose, and the motion playback. All math is delegated to core/.

// All math below goes through core/api (the C++ core via WebAssembly);
// core/*.ts modules are imported for types only.
import { create } from 'zustand';
import robotYaml from '../../../config/robot.yaml?raw';
import {
  auditTrajectory,
  checkPath,
  checkPose,
  computeMetrics,
  deriveGearbox,
  forwardKinematics,
  inverseKinematics,
  planTrajectory,
  ratioRange,
  sampleTrajectory,
  wasmReady,
} from '../core/api';
import type { GearboxParams, JointName, RobotConfig } from '../core/config';
import { parseRobotConfig } from '../core/config';
import type { DriveSelection } from '../core/gearboxModel';
import type { JointAngles, Vec3, IkBranch } from '../core/kinematics';
import type { MoveKind, ProgramMove } from '../core/program';
import { DEFAULT_DWELL, MAX_DWELL, makeMove } from '../core/program';
import { fromCartesianMm, resolveProgram } from '../core/programResolve';
import { retimeForTorque } from '../core/retime';
import type { TrajectoryPlan } from '../core/trajectory';
import { clamp, deg2rad, m2mm, rad2deg } from '../core/units';

export const config: RobotConfig = parseRobotConfig(robotYaml);

export const HOME_POSE: JointAngles = [0, deg2rad(90), deg2rad(-90)];

export type ControlMode = 'target' | 'joints';

export type IkStatus =
  | { kind: 'ok'; nearSingularity: boolean }
  | { kind: 'unreachable' }
  | { kind: 'limits' }
  /** The commanded pose itself would collide (self or ground). */
  | { kind: 'collision'; issues: string[] }
  /** Endpoints are fine but the motion between them would collide. */
  | { kind: 'path-collision'; issues: string[] };

export interface MoveReport {
  duration: number;         // s
  peakUtilization: number;  // fraction of available torque, worst joint
  peakJoint: JointName;
  skippedSteps: boolean;    // peak > 1 → open-loop steppers would lose position
  infeasible: boolean;      // a joint had zero speed/accel budget at plan time
  /** Duration multiplier the torque governor applied (1 = timing was safe). */
  stretch: number;
}

interface Motion {
  plan: TrajectoryPlan;
  report: MoveReport; // predictive audit, computed at plan time
  startedAt: number;  // performance.now() ms
  elapsed: number;
}

/** One leg of a run: a program row already resolved to a joint target. */
export interface RunStep {
  moveId: string;
  q: JointAngles;
  dwell: number;
}

export type RunStatus = 'running' | 'paused' | 'done' | 'error';

/**
 * A program execution. Steps are resolved once, at start: editing the program
 * mid-run changes the *next* run, never the one in flight — the same contract
 * the firmware gives, where a queued move is committed when it is accepted.
 */
export interface ProgramRun {
  steps: RunStep[];
  /** Step in flight, or the next one to start while dwelling. */
  index: number;
  status: RunStatus;
  /** Pause as soon as the current leg lands (single-step). */
  stepping: boolean;
  /** Dwell deadline (ms, performance.now clock); null when not dwelling. */
  holdUntil: number | null;
  /** Dwell left over when paused mid-hold, restored on resume. */
  holdRemaining: number | null;
  /** Completed passes, for looping programs. */
  cycle: number;
  error: string | null;
  // aggregated across legs for the final move report
  peakUtilization: number;
  peakJoint: JointName;
  infeasible: boolean;
  totalDuration: number;
  maxStretch: number;
}

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
  /** The editable list of commanded moves. */
  program: ProgramMove[];
  /** The run in flight (or the last one's outcome); null when never run. */
  run: ProgramRun | null;
  /** Restart the program from the top instead of stopping at the last move. */
  loop: boolean;
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
  clearTrace: () => void;

  // ------------------------------------------------------------ the program
  /** Append the arm's currently commanded pose as a move. */
  addMove: (kind: MoveKind) => void;
  updateMove: (id: string, patch: Partial<Omit<ProgramMove, 'id'>>) => void;
  removeMove: (id: string) => void;
  /** Shift a move one place earlier (-1) or later (+1) in the list. */
  reorderMove: (id: string, delta: -1 | 1) => void;
  setProgram: (moves: ProgramMove[]) => void;
  clearProgram: () => void;

  runProgram: () => void;
  /** Run exactly one move, then hold. */
  stepProgram: () => void;
  pauseRun: () => void;
  resumeRun: () => void;
  stopRun: () => void;
  setLoop: (loop: boolean) => void;
}

const TRACE_CAP = 800;
/** Plan below the hard drivetrain ceilings so utilization stays finite. */
const SPEED_PLANNING_MARGIN = 0.8;
/** The torque governor slows every move until utilization fits this ceiling. */
const TORQUE_UTILIZATION_CEILING = 0.95;

function pickIkSolution(
  target: Vec3,
  branch: IkBranch,
  current: JointAngles,
): { q: JointAngles | null; status: IkStatus } {
  const res = inverseKinematics(target, config.links, config.limits);
  if (!res.reachable) return { q: null, status: { kind: 'unreachable' } };

  const withinLimits = res.solutions.filter((s) => s.withinLimits);
  if (withinLimits.length === 0) return { q: null, status: { kind: 'limits' } };

  // Drop solutions whose pose would collide (self or ground).
  const usable = withinLimits.filter(
    (s) => !checkPose(s.q, config.links, config.collision).colliding,
  );
  if (usable.length === 0) {
    const { issues } = checkPose(withinLimits[0].q, config.links, config.collision);
    return { q: null, status: { kind: 'collision', issues } };
  }

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

  const planLeg = (
    from: JointAngles,
    to: JointAngles,
    s: TwinState,
  ): { plan: TrajectoryPlan; report: MoveReport } => {
    const metrics = computeMetrics(config, s.gearboxes, from, s.payload);
    const vmax = metrics.vmax.map((v) => v * SPEED_PLANNING_MARGIN) as [number, number, number];
    const raw = planTrajectory(from, to, vmax, metrics.amax);
    // Torque governor: stretch the duration until the predictive audit fits
    // the budget, so a commanded move can never outrun the torque limit.
    // Only a static overload (gravity alone beats the budget) stays `limited`.
    const { plan, audit, stretch } = retimeForTorque(
      raw,
      (p) => auditTrajectory(config, s.gearboxes, p, s.payload),
      TORQUE_UTILIZATION_CEILING,
    );
    return {
      plan,
      report: {
        duration: plan.duration,
        peakUtilization: audit.peakUtilization,
        peakJoint: audit.peakJoint,
        skippedSteps: audit.skippedSteps,
        infeasible: raw.infeasible,
        stretch,
      },
    };
  };

  const startClock = () => {
    stopClock();
    clock = setInterval(tick, 1000 / 30);
  };

  const finalReport = (run: ProgramRun): MoveReport => ({
    duration: run.totalDuration,
    peakUtilization: run.peakUtilization,
    peakJoint: run.peakJoint,
    skippedSteps: run.peakUtilization > 1,
    infeasible: run.infeasible,
    stretch: run.maxStretch,
  });

  /** Plan and launch the leg at `run.index`, wrapping first if looping. */
  const beginLeg = (now: number) => {
    const s = get();
    const run = s.run;
    if (!run) return;

    let { index, cycle } = run;
    if (index >= run.steps.length) {
      if (!s.loop || run.steps.length === 0) {
        stopClock();
        set({ run: { ...run, status: 'done', holdUntil: null, holdRemaining: null } });
        return;
      }
      index = 0;
      cycle += 1;
    }

    const step = run.steps[index];
    const { plan, report } = planLeg(s.q, step.q, s);
    const path = checkPath(plan, config.links, config.collision);
    if (path.colliding) {
      // Endpoints were validated when the run started, so this is a swept-path
      // conflict; stop where we are rather than driving through it.
      stopClock();
      set({
        motion: null,
        ikStatus: { kind: 'path-collision', issues: path.issues },
        lastMove: finalReport(run),
        run: {
          ...run,
          index,
          cycle,
          status: 'error',
          holdUntil: null,
          holdRemaining: null,
          error: `move ${index + 1}: path collision — ${path.issues[0]}`,
        },
      });
      return;
    }

    set({
      motion: { plan, report, startedAt: now, elapsed: 0 },
      target: forwardKinematics(step.q, config.links).tcp,
      ikStatus: { kind: 'ok', nearSingularity: false },
      run: {
        ...run,
        index,
        cycle,
        holdUntil: null,
        holdRemaining: null,
        peakUtilization: Math.max(run.peakUtilization, report.peakUtilization),
        peakJoint:
          report.peakUtilization > run.peakUtilization ? report.peakJoint : run.peakJoint,
        infeasible: run.infeasible || report.infeasible,
        totalDuration: run.totalDuration + report.duration,
        maxStretch: Math.max(run.maxStretch, report.stretch),
      },
    });
  };

  // Ticks only animate and advance the program; all physics and collision
  // conclusions for each leg are drawn at plan time.
  const tick = () => {
    const s = get();
    const now = performance.now();

    if (!s.motion) {
      const run = s.run;
      if (!run || run.status !== 'running') {
        stopClock();
        return;
      }
      if (run.holdUntil !== null && now < run.holdUntil) return; // dwelling
      beginLeg(now);
      return;
    }

    const { plan, report, startedAt } = s.motion;
    const elapsed = (now - startedAt) / 1000;
    const sample = sampleTrajectory(plan, elapsed);

    const tcp = forwardKinematics(sample.q, config.links).tcp;
    const trace =
      s.trace.length >= TRACE_CAP ? [...s.trace.slice(-TRACE_CAP + 1), tcp] : [...s.trace, tcp];

    if (elapsed < plan.duration) {
      set({ q: sample.q, motion: { plan, report, startedAt, elapsed }, trace });
      return;
    }

    // leg finished
    const run = s.run;
    if (!run) {
      stopClock();
      set({ q: plan.to, motion: null, trace, lastMove: report });
      return;
    }

    const dwellMs = Math.max(0, run.steps[run.index]?.dwell ?? 0) * 1000;
    const nextIndex = run.index + 1;

    if (nextIndex >= run.steps.length && !s.loop) {
      stopClock();
      set({
        q: plan.to,
        motion: null,
        trace,
        run: { ...run, index: nextIndex, status: 'done', holdUntil: null, holdRemaining: null },
        lastMove: finalReport(run),
      });
      return;
    }

    if (run.stepping) {
      stopClock();
      set({
        q: plan.to,
        motion: null,
        trace,
        run: {
          ...run,
          index: nextIndex,
          status: 'paused',
          stepping: false,
          holdUntil: null,
          holdRemaining: dwellMs,
        },
      });
      return;
    }

    set({
      q: plan.to,
      motion: null,
      trace,
      run: {
        ...run,
        index: nextIndex,
        holdUntil: now + dwellMs,
        holdRemaining: null,
        totalDuration: run.totalDuration + dwellMs / 1000,
      },
    });
  };

  /** The pose the arm is committed to reach — what a new move starts from. */
  const commandedPose = (s: TwinState): JointAngles => (s.motion ? s.motion.plan.to : s.q);

  /** Resolve the program and launch it from the top. */
  const launch = (stepping: boolean) => {
    const s = get();
    const { steps } = resolveProgram(s.program, config, s.branch, s.q);
    if (steps.length === 0) return;

    startClock();
    set({
      motion: null,
      lastMove: null,
      trace: [],
      run: {
        steps: steps.map((st) => ({ moveId: st.move.id, q: st.q, dwell: st.dwell })),
        index: 0,
        status: 'running',
        stepping,
        holdUntil: null,
        holdRemaining: null,
        cycle: 0,
        error: null,
        peakUtilization: 0,
        peakJoint: 'base',
        infeasible: false,
        totalDuration: 0,
        maxStretch: 1,
      },
    });
  };

  /**
   * Plan and begin a single move to `to`, cancelling any program run.
   * If the path would collide, nothing moves and the status says why.
   */
  const startMove = (to: JointAngles): Partial<TwinState> => {
    const s = get();
    const { plan, report } = planLeg(s.q, to, s);
    const path = checkPath(plan, config.links, config.collision);
    if (path.colliding) {
      return {
        ikStatus: { kind: 'path-collision', issues: path.issues },
        motion: null,
        run: null,
      };
    }
    startClock();
    return {
      motion: { plan, report, startedAt: performance.now(), elapsed: 0 },
      lastMove: null,
      run: null,
    };
  };

  const initialState = {
    drives: { ...config.drives },
    gearboxes: { ...config.gearboxes },
    payload: config.masses.payloadDefault,
    controlMode: 'target' as ControlMode,
    branch: 'elbow-up' as IkBranch,
    target: [0.12, 0.18, 0] as Vec3, // placeholder; updated when WASM ready
    q: HOME_POSE,
    ikStatus: { kind: 'ok' as const, nearSingularity: false },
    motion: null as Motion | null,
    program: [] as ProgramMove[],
    run: null as ProgramRun | null,
    loop: false,
    lastMove: null as MoveReport | null,
    trace: [] as Vec3[],
    showWorkspace: true,
  };

  // Initialize target once WASM is ready
  wasmReady.then(() => {
    set({ target: forwardKinematics(HOME_POSE, config.links).tcp });
  });

  return {
    ...initialState,

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
      const to = [...commandedPose(s)] as JointAngles;
      to[index] = angle;
      const pose = checkPose(to, config.links, config.collision);
      if (pose.colliding) {
        // block the command; the slider snaps back to the last safe target
        set({ ikStatus: { kind: 'collision', issues: pose.issues }, run: null });
        return;
      }
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

    clearTrace: () => set({ trace: [] }),

    // ------------------------------------------------------------ the program

    addMove: (kind) =>
      set((s) => {
        const pose = commandedPose(s);
        const values: [number, number, number] =
          kind === 'joints'
            ? (pose.map(rad2deg) as [number, number, number])
            : fromCartesianMm(
                kind,
                forwardKinematics(pose, config.links).tcp.map(m2mm) as [number, number, number],
              );
        return { program: [...s.program, makeMove(kind, values, DEFAULT_DWELL)] };
      }),

    updateMove: (id, patch) =>
      set((s) => ({
        program: s.program.map((m) =>
          m.id === id
            ? {
                ...m,
                ...patch,
                dwell: patch.dwell === undefined ? m.dwell : clamp(patch.dwell, 0, MAX_DWELL),
              }
            : m,
        ),
      })),

    removeMove: (id) => set((s) => ({ program: s.program.filter((m) => m.id !== id) })),

    reorderMove: (id, delta) =>
      set((s) => {
        const i = s.program.findIndex((m) => m.id === id);
        const j = i + delta;
        if (i < 0 || j < 0 || j >= s.program.length) return {};
        const program = [...s.program];
        [program[i], program[j]] = [program[j], program[i]];
        return { program };
      }),

    setProgram: (moves) => set({ program: moves }),

    clearProgram: () => set({ program: [], run: null }),

    runProgram: () => launch(false),

    stepProgram: () => {
      const s = get();
      // Paused mid-program: advance exactly one more leg from where we are.
      if (s.run && (s.run.status === 'paused' || s.run.status === 'running')) {
        const now = performance.now();
        const run = s.run;
        startClock();
        set({
          motion: s.motion ? { ...s.motion, startedAt: now - s.motion.elapsed * 1000 } : null,
          run: {
            ...run,
            status: 'running',
            stepping: true,
            holdUntil: run.holdRemaining !== null ? now + run.holdRemaining : run.holdUntil,
            holdRemaining: null,
          },
        });
        return;
      }
      launch(true);
    },

    pauseRun: () => {
      const s = get();
      const run = s.run;
      if (!run || run.status !== 'running') return;
      stopClock();
      const now = performance.now();
      set({
        run: {
          ...run,
          status: 'paused',
          holdUntil: null,
          holdRemaining:
            run.holdUntil !== null ? Math.max(0, run.holdUntil - now) : run.holdRemaining,
        },
      });
    },

    resumeRun: () => {
      const s = get();
      const run = s.run;
      if (!run || run.status !== 'paused') return;
      const now = performance.now();
      startClock();
      set({
        // Shift the clock instead of replanning, so the paused leg resumes on
        // the exact profile it was already following.
        motion: s.motion ? { ...s.motion, startedAt: now - s.motion.elapsed * 1000 } : null,
        run: {
          ...run,
          status: 'running',
          stepping: false,
          holdUntil: run.holdRemaining !== null ? now + run.holdRemaining : null,
          holdRemaining: null,
        },
      });
    },

    stopRun: () => {
      const s = get();
      stopClock();
      set({
        motion: null,
        run: null,
        lastMove: s.run ? finalReport(s.run) : s.lastMove,
      });
    },

    setLoop: (loop) => set({ loop }),
  };
});

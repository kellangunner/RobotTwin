// The move program: an ordered list of commanded moves the twin executes one
// at a time, the same way the physical arm is driven — a queue of MOVEJ/MOVEL
// commands, each planned, run to completion, then held for a dwell — rather
// than a pose that follows a dragged slider.
//
// This module is deliberately free of kinematics so it can be unit tested in
// isolation: it owns the program's *shape* (rows, units, text round-trips).
// Turning a row into a validated joint target is programResolve.ts's job.

import { clamp } from './units';

/**
 * How a move's three numbers are interpreted.
 *
 * `polar` is cylindrical (r, θ, z) about the base yaw axis, which is the arm's
 * own natural frame — FK is literally `tcp = (r·cosθ1, r·sinθ1, z)`, so r is
 * the planar reach and θ *is* the base angle. Reaching "200 mm out, 45° round,
 * 150 mm up" is one edit here and two coupled ones in Cartesian.
 */
export type MoveKind = 'cartesian' | 'polar' | 'joints';

export const MOVE_KINDS: readonly MoveKind[] = ['cartesian', 'polar', 'joints'] as const;

/** Axis names and units per kind, for labels and tooltips. */
export const KIND_AXES: Record<MoveKind, readonly [string, string, string]> = {
  cartesian: ['x (mm)', 'y (mm)', 'z (mm)'],
  polar: ['r (mm)', 'θ (deg)', 'z (mm)'],
  joints: ['θ1 (deg)', 'θ2 (deg)', 'θ3 (deg)'],
};

/** Short badge shown on each row. */
export const KIND_BADGE: Record<MoveKind, string> = {
  cartesian: 'XYZ',
  polar: 'RθZ',
  joints: 'θ',
};

export interface ProgramMove {
  id: string;
  kind: MoveKind;
  /**
   * UI units, stored as typed: cartesian = x, y, z in mm; polar = r mm, θ deg,
   * z mm; joints = θ1, θ2, θ3 in degrees. Kept out of SI so editing and
   * CSV/script export round-trip exactly instead of drifting through repeated
   * deg↔rad conversions.
   */
  values: [number, number, number];
  /** Seconds to hold at this waypoint before the next move starts. */
  dwell: number;
  /** Unchecked rows stay in the list but are skipped when the program runs. */
  enabled: boolean;
}

/** Matches the dwell the twin used for CSV waypoint sequences. */
export const DEFAULT_DWELL = 0.8;
export const MAX_DWELL = 60;

let idCounter = 0;

export function newMoveId(): string {
  idCounter += 1;
  return `mv${idCounter.toString(36)}${Date.now().toString(36)}`;
}

export function makeMove(
  kind: MoveKind,
  values: [number, number, number],
  dwell: number = DEFAULT_DWELL,
): ProgramMove {
  return {
    id: newMoveId(),
    kind,
    values: values.map(roundUi) as [number, number, number],
    dwell: clamp(dwell, 0, MAX_DWELL),
    enabled: true,
  };
}

/** One decimal is finer than the arm can resolve; more just adds noise. */
export function roundUi(v: number): number {
  return Math.round(v * 10) / 10;
}

// ------------------------------------------------------------------- CSV I/O

export interface ProgramCsvResult {
  moves: ProgramMove[];
  /** Rows that could not be read as a move. */
  skipped: number;
  firstIssue: string | null;
}

// A leading token naming the row's type, so a program with mixed Cartesian and
// joint moves survives a save/load round-trip. Rows without one fall back to
// the import mode chosen in the UI.
const KIND_TOKENS: Record<string, MoveKind> = {
  j: 'joints',
  joint: 'joints',
  joints: 'joints',
  movej: 'joints',
  deg: 'joints',
  c: 'cartesian',
  cart: 'cartesian',
  cartesian: 'cartesian',
  movel: 'cartesian',
  xyz: 'cartesian',
  mm: 'cartesian',
  p: 'polar',
  pol: 'polar',
  polar: 'polar',
  cyl: 'polar',
  cylindrical: 'polar',
  rtz: 'polar',
};

/** CSV tag written for each kind; parsed back by KIND_TOKENS. */
const KIND_TAG: Record<MoveKind, string> = { cartesian: 'C', polar: 'P', joints: 'J' };

/**
 * Read a CSV/whitespace-delimited program. Rows are
 * `[kind,] a, b, c [, dwell]`. Malformed rows are reported rather than
 * silently dropped, so the user can fix them in the editor.
 */
export function parseProgramCsv(text: string, defaultKind: MoveKind): ProgramCsvResult {
  const moves: ProgramMove[] = [];
  let skipped = 0;
  let firstIssue: string | null = null;
  let sawData = false;

  const issue = (msg: string) => {
    skipped += 1;
    if (firstIssue === null) firstIssue = msg;
  };

  text.split(/\r?\n/).forEach((raw, idx) => {
    const line = raw.trim();
    if (!line || line.startsWith('#')) return;

    let tokens = line.split(/[,;\t]+|\s+/).filter(Boolean);
    let kind = defaultKind;
    let enabled = true;

    if (!isNumeric(tokens[0])) {
      const tag = tokens[0].toLowerCase();
      // A leading "!" marks a row that is kept but skipped when running, so
      // unchecked moves survive an export/import round-trip.
      const named = KIND_TOKENS[tag.startsWith('!') ? tag.slice(1) : tag];
      if (named) {
        kind = named;
        enabled = !tag.startsWith('!');
        tokens = tokens.slice(1);
      } else if (!sawData) {
        sawData = true; // tolerate one leading header row ("x,y,z")
        return;
      } else {
        issue(`line ${idx + 1}: unrecognised move type "${tokens[0]}"`);
        return;
      }
    }

    const nums = tokens.map(Number);
    if (nums.length < 3 || nums.slice(0, 3).some((n) => !Number.isFinite(n))) {
      issue(`line ${idx + 1}: expected 3 numbers`);
      return;
    }
    sawData = true;

    const dwell = nums.length > 3 && Number.isFinite(nums[3]) ? nums[3] : DEFAULT_DWELL;
    moves.push({ ...makeMove(kind, [nums[0], nums[1], nums[2]], dwell), enabled });
  });

  return { moves, skipped, firstIssue };
}

function isNumeric(token: string | undefined): boolean {
  return token !== undefined && token !== '' && Number.isFinite(Number(token));
}

/** Human-readable one-liner for a row, used in exported script comments. */
export function describeMove(move: ProgramMove): string {
  const [a, b, c] = move.values.map((v) => v.toFixed(1));
  switch (move.kind) {
    case 'cartesian':
      return `x=${a} y=${b} z=${c} mm`;
    case 'polar':
      return `r=${a} mm θ=${b}° z=${c} mm`;
    case 'joints':
      return `θ1=${a}° θ2=${b}° θ3=${c}°`;
  }
}

/** The whole list, disabled rows included, so saving loses nothing. */
export function programToCsv(moves: ProgramMove[]): string {
  const lines = [
    '# RobotTwin move program',
    '# kind,a,b,c,dwell_s',
    '#   C: x,y,z in mm   ·   P: r mm, θ deg, z mm   ·   J: θ1,θ2,θ3 in deg',
    '# a "!" before the kind marks a move that is kept but skipped when running',
  ];
  for (const m of moves) {
    const tag = `${m.enabled ? '' : '!'}${KIND_TAG[m.kind]}`;
    const [a, b, c] = m.values.map((v) => v.toFixed(1));
    lines.push(`${tag},${a},${b},${c},${m.dwell.toFixed(2)}`);
  }
  return `${lines.join('\n')}\n`;
}

/** A move the twin has already resolved to a validated arm pose. */
export interface ScriptStep {
  /** θ1, θ2, θ3 in degrees — the pose the twin actually executed. */
  anglesDeg: [number, number, number];
  dwell: number;
  /** How the row was authored, kept as a comment (see describeMove). */
  note?: string;
}

/**
 * The program as a firmware script, so a sequence validated in the twin can be
 * replayed on the physical arm without being retyped. Verbs match
 * src/hardware/serial_protocol.hpp; SLEEP is a run_script.py host directive.
 *
 * Every step goes out as MOVEJ, including rows authored in Cartesian or polar:
 * the twin has already picked an IK branch and proved that pose reachable,
 * collision free and within the torque budget, so sending joint angles
 * guarantees the hardware reproduces exactly what was simulated instead of
 * re-solving the IK on the ESP32 and possibly landing on the other elbow
 * branch. It also lets run_script.py split each move joint-by-joint for the
 * shared-rail bench, which it cannot do for MOVEL. (There is no polar verb on
 * the wire at all, so polar rows could not round-trip any other way.) The
 * authored intent is kept as a comment.
 */
export function programToRobotScript(steps: ScriptStep[]): string {
  const lines = [
    '# RobotTwin move program — exported from the digital twin.',
    '# Joint targets are the poses the twin validated and executed.',
    '# Replay:  python python/run_script.py --port COM5 program.txt',
    '# Preview without hardware:  python python/run_script.py --dry-run program.txt',
    '#',
    '# Prelude — uncomment once the arm is parked at its datum and clear to move:',
    '# SETHOME 0 90 -90',
    '# ENABLE',
  ];
  for (const step of steps) {
    if (step.note) lines.push(`# target ${step.note}`);
    const [a, b, c] = step.anglesDeg.map((v) => v.toFixed(2));
    lines.push(`MOVEJ ${a} ${b} ${c}`);
    if (step.dwell > 0) lines.push(`SLEEP ${step.dwell.toFixed(2)}`);
  }
  return `${lines.join('\n')}\n`;
}

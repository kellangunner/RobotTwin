// The move program editor — the twin's answer to "how does the real arm get
// told what to do". You build an explicit list of commanded moves, each one
// validated against reach, joint limits and collision as you type, then run
// them: the arm plans and executes each move to completion, dwells, and moves
// on — exactly how the firmware drains a queue of MOVEJ/MOVEL commands.
//
// Sliders remain a way to *pose* the arm; this panel is how you *program* it.

import { useMemo, useRef, useState } from 'react';
import type { MoveKind, ProgramMove, ScriptStep } from '../core/program';
import {
  MAX_DWELL,
  parseProgramCsv,
  programToCsv,
  programToRobotScript,
  roundUi,
} from '../core/program';
import type { MoveResolution, ProgramResolution } from '../core/programResolve';
import { resolveProgram } from '../core/programResolve';
import { rad2deg } from '../core/units';
import { config, useTwinStore } from '../state/store';
import { Chip, Panel } from './controls';

const fmt = (v: number) => String(roundUi(v));

/**
 * A numeric cell that stays editable while you type. The value commits on
 * every keystroke that parses, so the row's reachability check updates live,
 * but the text you are mid-way through typing is never reformatted under you.
 */
function NumCell({
  value,
  onCommit,
  className,
  title,
  step = 1,
  min,
  max,
}: {
  value: number;
  onCommit: (v: number) => void;
  className?: string;
  title?: string;
  step?: number;
  min?: number;
  max?: number;
}) {
  // null = not being edited, so the prop is the source of truth.
  const [draft, setDraft] = useState<string | null>(null);

  return (
    <input
      type="number"
      step={step}
      min={min}
      max={max}
      title={title}
      value={draft ?? fmt(value)}
      onFocus={(e) => {
        setDraft(fmt(value));
        e.currentTarget.select();
      }}
      onChange={(e) => {
        setDraft(e.target.value);
        const n = Number(e.target.value);
        if (e.target.value.trim() !== '' && Number.isFinite(n)) onCommit(n);
      }}
      onBlur={() => setDraft(null)}
      className={`border border-zinc-300 bg-white px-1 py-0.5 text-right font-mono text-[10px] tabular-nums text-zinc-900 focus:border-orange-600 focus:outline-none ${
        className ?? ''
      }`}
    />
  );
}

function IconButton({
  onClick,
  label,
  disabled,
  children,
}: {
  onClick: () => void;
  label: string;
  disabled?: boolean;
  children: React.ReactNode;
}) {
  return (
    <button
      onClick={onClick}
      disabled={disabled}
      title={label}
      aria-label={label}
      className="w-4 shrink-0 text-center text-[10px] leading-none text-zinc-400 hover:text-orange-700 disabled:cursor-default disabled:text-zinc-200"
    >
      {children}
    </button>
  );
}

function MoveRow({
  move,
  index,
  resolution,
  active,
  isFirst,
  isLast,
}: {
  move: ProgramMove;
  index: number;
  resolution: MoveResolution | undefined;
  active: boolean;
  isFirst: boolean;
  isLast: boolean;
}) {
  const updateMove = useTwinStore((s) => s.updateMove);
  const removeMove = useTwinStore((s) => s.removeMove);
  const reorderMove = useTwinStore((s) => s.reorderMove);

  const bad = resolution !== undefined && !resolution.ok;
  const joints = move.kind === 'joints';
  const unit = joints ? '°' : 'mm';

  const setValue = (i: number) => (v: number) => {
    const values = [...move.values] as [number, number, number];
    values[i] = v;
    updateMove(move.id, { values });
  };

  // Switching interpretation keeps the numbers: mm and degrees are different
  // quantities, so re-deriving them would silently move the waypoint. The row
  // simply revalidates, and an unreachable result is flagged immediately.
  const toggleKind = () =>
    updateMove(move.id, { kind: joints ? 'cartesian' : ('joints' as MoveKind) });

  return (
    <li
      className={`border-b border-zinc-200 px-0.5 py-1 last:border-b-0 ${
        active ? 'bg-orange-50' : ''
      } ${move.enabled ? '' : 'opacity-45'}`}
    >
      <div className="flex items-center gap-1">
        <span
          aria-hidden
          title={bad ? 'not runnable' : 'ok'}
          className={`h-1.5 w-1.5 shrink-0 rounded-full ${
            bad ? 'bg-red-600' : active ? 'bg-orange-600' : 'bg-emerald-600'
          }`}
        />
        <input
          type="checkbox"
          checked={move.enabled}
          onChange={(e) => updateMove(move.id, { enabled: e.target.checked })}
          title="include this move in the run"
          className="h-2.5 w-2.5 shrink-0 accent-orange-600"
        />
        <span className="w-3 shrink-0 text-right font-mono text-[10px] text-zinc-400">
          {index + 1}
        </span>
        <button
          onClick={toggleKind}
          title={
            joints
              ? 'joint move (θ1, θ2, θ3 in degrees) — click for Cartesian'
              : 'Cartesian move (x, y, z in mm, solved by IK) — click for joint space'
          }
          className={`w-7 shrink-0 border px-0.5 py-0.5 text-center font-mono text-[9px] font-semibold ${
            joints
              ? 'border-sky-700 bg-sky-100 text-sky-800'
              : 'border-violet-700 bg-violet-100 text-violet-800'
          }`}
        >
          {joints ? 'θ' : 'XYZ'}
        </button>
        {[0, 1, 2].map((i) => (
          <NumCell
            key={i}
            value={move.values[i]}
            onCommit={setValue(i)}
            className="w-full min-w-0 flex-1"
            title={joints ? `θ${i + 1} (deg)` : `${'xyz'[i]} (mm)`}
          />
        ))}
        <span className="shrink-0 text-[9px] text-zinc-400">{unit}</span>
        <NumCell
          value={move.dwell}
          onCommit={(v) => updateMove(move.id, { dwell: v })}
          className="w-8 shrink-0"
          title="dwell after arriving (seconds)"
          step={0.1}
          min={0}
          max={MAX_DWELL}
        />
        <span className="shrink-0 text-[9px] text-zinc-400">s</span>
        <IconButton onClick={() => reorderMove(move.id, -1)} label="move up" disabled={isFirst}>
          ▲
        </IconButton>
        <IconButton onClick={() => reorderMove(move.id, 1)} label="move down" disabled={isLast}>
          ▼
        </IconButton>
        <IconButton onClick={() => removeMove(move.id)} label="delete move">
          ✕
        </IconButton>
      </div>
      {bad && resolution && !resolution.ok && (
        <p className="ml-4 mt-0.5 text-[10px] text-red-600">↳ {resolution.reason}</p>
      )}
    </li>
  );
}

function RunControls({ runnable }: { runnable: number }) {
  const run = useTwinStore((s) => s.run);
  const loop = useTwinStore((s) => s.loop);
  const runProgram = useTwinStore((s) => s.runProgram);
  const stepProgram = useTwinStore((s) => s.stepProgram);
  const pauseRun = useTwinStore((s) => s.pauseRun);
  const resumeRun = useTwinStore((s) => s.resumeRun);
  const stopRun = useTwinStore((s) => s.stopRun);
  const setLoop = useTwinStore((s) => s.setLoop);

  const running = run?.status === 'running';
  const paused = run?.status === 'paused';
  const live = running || paused;

  return (
    <div className="mb-2 flex flex-wrap items-center gap-1">
      {running ? (
        <Chip onClick={pauseRun}>❚❚ Pause</Chip>
      ) : (
        <Chip
          active={live}
          disabled={!paused && runnable === 0}
          title={runnable === 0 ? 'no runnable moves in the program' : 'run the whole program'}
          onClick={paused ? resumeRun : runProgram}
        >
          {paused ? '▶ Resume' : '▶ Run'}
        </Chip>
      )}
      <Chip
        onClick={stepProgram}
        disabled={!live && runnable === 0}
        title="execute one move, then hold"
      >
        ⇥ Step
      </Chip>
      <Chip onClick={stopRun} disabled={!live} title="stop where the arm is">
        ■ Stop
      </Chip>
      <Chip active={loop} onClick={() => setLoop(!loop)}>
        ↻ Loop
      </Chip>
      <span className="ml-auto font-mono text-[10px] text-zinc-500">
        {runnable} runnable
      </span>
    </div>
  );
}

function RunStatusLine() {
  const run = useTwinStore((s) => s.run);
  const loop = useTwinStore((s) => s.loop);
  const motion = useTwinStore((s) => s.motion);

  if (!run) return null;

  if (run.status === 'error') {
    return <p className="mb-1.5 font-mono text-[11px] text-red-600">■ stopped — {run.error}</p>;
  }
  if (run.status === 'done') {
    return (
      <p className="mb-1.5 font-mono text-[11px] text-emerald-700">
        ✓ program complete — {run.steps.length} moves in {run.totalDuration.toFixed(1)} s
      </p>
    );
  }

  const at = Math.min(run.index + 1, run.steps.length);
  const phase = run.status === 'paused' ? 'paused' : motion ? 'moving' : 'dwelling';
  return (
    <p className="mb-1.5 font-mono text-[11px] text-orange-700">
      ▶ move {at}/{run.steps.length} · {phase}
      {loop && run.cycle > 0 ? ` · pass ${run.cycle + 1}` : ''}
    </p>
  );
}

export function ProgramPanel() {
  const program = useTwinStore((s) => s.program);
  const branch = useTwinStore((s) => s.branch);
  const controlMode = useTwinStore((s) => s.controlMode);
  const addMove = useTwinStore((s) => s.addMove);
  const setProgram = useTwinStore((s) => s.setProgram);
  const clearProgram = useTwinStore((s) => s.clearProgram);

  // While a run is in flight the arm's pose changes 30×/s; re-resolving the
  // whole program on every frame would be pure waste, and the answer cannot
  // change — the run committed its targets when it started.
  const isRunning = useTwinStore((s) => s.run?.status === 'running');
  const motionTo = useTwinStore((s) => s.motion?.plan.to ?? null);
  const restQ = useTwinStore((s) => s.q);
  const fromQ = motionTo ?? restQ;

  const activeMoveId = useTwinStore((s) => {
    const r = s.run;
    if (!r || (r.status !== 'running' && r.status !== 'paused')) return null;
    return r.steps[r.index]?.moveId ?? null;
  });

  const fileRef = useRef<HTMLInputElement>(null);
  const [message, setMessage] = useState<string | null>(null);
  const lastResolution = useRef<ProgramResolution | null>(null);

  const resolution = useMemo(() => {
    if (isRunning && lastResolution.current) return lastResolution.current;
    lastResolution.current = resolveProgram(program, config, branch, fromQ);
    return lastResolution.current;
  }, [program, branch, fromQ, isRunning]);

  const download = (name: string, text: string, mime: string) => {
    const url = URL.createObjectURL(new Blob([text], { type: mime }));
    const a = document.createElement('a');
    a.href = url;
    a.download = name;
    a.click();
    URL.revokeObjectURL(url);
  };

  const onFile: React.ChangeEventHandler<HTMLInputElement> = (e) => {
    const file = e.target.files?.[0];
    e.target.value = '';
    if (!file) return;
    file
      .text()
      .then((text) => {
        const defaultKind: MoveKind = controlMode === 'joints' ? 'joints' : 'cartesian';
        const result = parseProgramCsv(text, defaultKind);
        if (result.moves.length === 0) {
          setMessage(`no moves found${result.firstIssue ? ` — ${result.firstIssue}` : ''}`);
          return;
        }
        setProgram([...useTwinStore.getState().program, ...result.moves]);
        setMessage(
          `imported ${result.moves.length} moves` +
            (result.skipped > 0 ? ` · ${result.skipped} skipped (${result.firstIssue})` : ''),
        );
      })
      .catch((err: unknown) => setMessage(`could not read file — ${String(err)}`));
  };

  const runnable = resolution.steps.length;

  // The hardware script carries the poses the twin proved out, not the raw
  // rows: only enabled moves that resolved, each as validated joint angles.
  const scriptSteps: ScriptStep[] = resolution.steps.map((step) => ({
    anglesDeg: step.q.map(rad2deg) as [number, number, number],
    dwell: step.dwell,
    cartesianMm: step.move.kind === 'cartesian' ? step.move.values : undefined,
  }));

  return (
    <Panel title="Move program">
      <RunControls runnable={runnable} />
      <RunStatusLine />

      {program.length === 0 ? (
        <p className="mb-2 border border-dashed border-zinc-300 px-2 py-3 text-center text-[11px] leading-relaxed text-zinc-500">
          No moves yet.
          <br />
          Pose the arm with the sliders, then add it as a step — or import a CSV.
        </p>
      ) : (
        <ol className="mb-2 max-h-72 overflow-y-auto border border-zinc-300 bg-zinc-50">
          {program.map((move, i) => (
            <MoveRow
              key={move.id}
              move={move}
              index={i}
              resolution={resolution.byId[move.id]}
              active={move.id === activeMoveId}
              isFirst={i === 0}
              isLast={i === program.length - 1}
            />
          ))}
        </ol>
      )}

      <div className="mb-1 flex flex-wrap items-center gap-1">
        <Chip onClick={() => addMove('cartesian')}>＋ XYZ move</Chip>
        <Chip onClick={() => addMove('joints')}>＋ θ move</Chip>
        <span className="text-[10px] text-zinc-500">from current pose</span>
      </div>
      <div className="flex flex-wrap items-center gap-1">
        <Chip onClick={() => fileRef.current?.click()}>⭱ Import CSV</Chip>
        <Chip
          onClick={() => download('program.csv', programToCsv(program), 'text/csv')}
          disabled={program.length === 0}
        >
          ⭳ CSV
        </Chip>
        <Chip
          onClick={() => download('program.txt', programToRobotScript(scriptSteps), 'text/plain')}
          disabled={runnable === 0}
        >
          ⭳ Robot script
        </Chip>
        <Chip onClick={clearProgram}>✕ Clear</Chip>
      </div>
      <input
        ref={fileRef}
        type="file"
        accept=".csv,.txt,text/csv,text/plain"
        className="hidden"
        onChange={onFile}
      />
      {message && <p className="mt-1 text-[10px] text-zinc-500">{message}</p>}
      {!isRunning && resolution.firstError && (
        <p className="mt-1 text-[10px] text-red-600">⚠ {resolution.firstError}</p>
      )}
    </Panel>
  );
}

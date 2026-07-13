// Small shared form controls for the side panels.

import type { ReactNode } from 'react';

export function Panel({ title, children }: { title: string; children: ReactNode }) {
  return (
    <section className="overflow-hidden rounded-lg border border-slate-800 bg-slate-900/70 p-3">
      <h2 className="mb-2 text-[11px] font-semibold uppercase tracking-wider text-slate-400">
        {title}
      </h2>
      {children}
    </section>
  );
}

export function Slider({
  label,
  value,
  min,
  max,
  step,
  unit,
  format,
  onChange,
}: {
  label: string;
  value: number;
  min: number;
  max: number;
  step: number;
  unit?: string;
  format?: (v: number) => string;
  onChange: (v: number) => void;
}) {
  const shown = format ? format(value) : value.toFixed(step < 0.1 ? 2 : step < 1 ? 1 : 0);
  return (
    <label className="mb-1.5 block">
      <div className="flex justify-between text-xs">
        <span className="text-slate-300">{label}</span>
        <span className="tabular-nums text-slate-100">
          {shown}
          {unit && <span className="ml-0.5 text-slate-500">{unit}</span>}
        </span>
      </div>
      <input
        type="range"
        className="h-1.5 w-full cursor-pointer accent-sky-400"
        min={min}
        max={max}
        step={step}
        value={value}
        onChange={(e) => onChange(Number(e.target.value))}
      />
    </label>
  );
}

export function Chip({
  active,
  onClick,
  children,
}: {
  active?: boolean;
  onClick: () => void;
  children: ReactNode;
}) {
  return (
    <button
      onClick={onClick}
      className={`rounded px-1.5 py-0.5 text-[10px] font-medium transition-colors ${
        active
          ? 'bg-sky-500/90 text-slate-950'
          : 'bg-slate-800 text-slate-300 hover:bg-slate-700'
      }`}
    >
      {children}
    </button>
  );
}

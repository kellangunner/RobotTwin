// Small shared form controls for the side panels.

import type { ReactNode } from 'react';

export function Panel({ title, children }: { title: string; children: ReactNode }) {
  return (
    <section className="overflow-hidden border border-zinc-400 bg-white p-3 shadow-sm">
      <h2 className="mb-2 flex items-center gap-1.5 border-b border-zinc-300 pb-1.5 font-mono text-[11px] font-semibold uppercase tracking-wider text-zinc-600">
        <span aria-hidden className="h-2 w-2 shrink-0 bg-orange-600" />
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
        <span className="text-zinc-700">{label}</span>
        <span className="font-mono tabular-nums text-zinc-900">
          {shown}
          {unit && <span className="ml-0.5 text-zinc-500">{unit}</span>}
        </span>
      </div>
      <input
        type="range"
        className="slider-mech w-full cursor-pointer"
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
      className={`border px-1.5 py-0.5 text-[10px] font-medium transition-colors ${
        active
          ? 'border-orange-700 bg-orange-600 text-white'
          : 'border-zinc-400 bg-zinc-100 text-zinc-700 hover:bg-zinc-200'
      }`}
    >
      {children}
    </button>
  );
}

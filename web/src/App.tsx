import React, { useEffect, useState, type ReactNode } from 'react';
import { Scene } from './three/Scene';
import { GearboxPanel } from './ui/GearboxPanel';
import { MetricsPanel } from './ui/MetricsPanel';
import { PosePanel } from './ui/PosePanel';
import { config } from './state/store';
import { wasmReady } from './core/api';

class ErrorBoundary extends React.Component<{ children: ReactNode }, { error?: Error }> {
  constructor(props: { children: ReactNode }) {
    super(props);
    this.state = {};
  }

  static getDerivedStateFromError(error: Error) {
    return { error };
  }

  render() {
    if (this.state.error) {
      return (
        <div className="flex h-screen flex-col gap-4 bg-zinc-100 p-4 text-zinc-800">
          <h2 className="text-lg font-bold text-red-600">Error</h2>
          <pre className="overflow-auto border border-zinc-400 bg-white p-2 text-sm text-red-700">
            {this.state.error.message}
            {'\n\n'}
            {this.state.error.stack}
          </pre>
        </div>
      );
    }
    return this.props.children;
  }
}

function AppContent() {
  return (
    <div className="grid-paper flex h-screen flex-col text-zinc-800">
      <header className="flex items-baseline gap-3 border-b-2 border-zinc-400 bg-zinc-50 px-4 py-2">
        <span aria-hidden className="h-2.5 w-2.5 self-center bg-orange-600" />
        <h1 className="font-mono text-sm font-bold uppercase tracking-widest">RobotTwin</h1>
        <span className="text-xs text-zinc-500">
          {config.name} · 3-DOF digital twin · gearbox trade-study
        </span>
        <span className="ml-auto font-mono text-[10px] text-zinc-500">
          geometry fixed: 90 / 120 / 120 mm · NEMA 17 · A1 Mini printable
        </span>
      </header>
      <div className="flex min-h-0 flex-1">
        <aside className="w-72 shrink-0 overflow-y-auto border-r border-zinc-400 p-3">
          <GearboxPanel />
        </aside>
        <main className="relative min-w-0 flex-1 border-r border-zinc-400">
          <Scene />
        </main>
        <aside className="w-72 shrink-0 overflow-y-auto p-3">
          <div className="flex flex-col gap-3">
            <PosePanel />
            <MetricsPanel />
          </div>
        </aside>
      </div>
    </div>
  );
}

function AppLoader() {
  return (
    <div className="grid-paper flex h-screen items-center justify-center">
      <div className="text-center">
        <div className="mb-4 font-mono text-xs uppercase tracking-widest text-zinc-500">
          Initializing...
        </div>
        <div className="h-1 w-32 border border-zinc-400 bg-zinc-200">
          <div className="h-full animate-pulse bg-orange-600" />
        </div>
      </div>
    </div>
  );
}

export default function App() {
  const [ready, setReady] = useState(false);

  useEffect(() => {
    console.log('App: waiting for WASM...');
    wasmReady.then(() => {
      console.log('App: WASM ready!');
      setReady(true);
    }).catch((err) => {
      console.error('App: WASM failed', err);
    });
  }, []);

  console.log('App rendering, ready=' + ready);
  return (
    <ErrorBoundary>
      {ready ? <AppContent /> : <AppLoader />}
    </ErrorBoundary>
  );
}

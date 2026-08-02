import { describe, expect, it } from 'vitest';
import {
  DEFAULT_DWELL,
  makeMove,
  parseProgramCsv,
  programToCsv,
  programToRobotScript,
} from './program';

describe('parseProgramCsv', () => {
  it('reads plain 3-column rows using the default kind', () => {
    const { moves, skipped } = parseProgramCsv('150,0,150\n120,60,140\n', 'cartesian');
    expect(skipped).toBe(0);
    expect(moves.map((m) => m.kind)).toEqual(['cartesian', 'cartesian']);
    expect(moves[0].values).toEqual([150, 0, 150]);
    expect(moves[0].dwell).toBe(DEFAULT_DWELL);
  });

  it('honours a per-row kind tag, so mixed programs round-trip', () => {
    const { moves, skipped } = parseProgramCsv('J,0,90,-90,0.5\nC,150,0,150,1.5\n', 'cartesian');
    expect(skipped).toBe(0);
    expect(moves.map((m) => m.kind)).toEqual(['joints', 'cartesian']);
    expect(moves[0].dwell).toBe(0.5);
    expect(moves[1].dwell).toBe(1.5);
  });

  it('skips comments and tolerates one leading header row', () => {
    const { moves, skipped } = parseProgramCsv('# note\nx,y,z\n150,0,150\n', 'cartesian');
    expect(skipped).toBe(0);
    expect(moves).toHaveLength(1);
  });

  it('accepts comma, semicolon, tab and space delimiters', () => {
    const { moves, skipped } = parseProgramCsv(
      '150,0,150\n140;10;150\n130\t20\t150\n120 30 150\n',
      'cartesian',
    );
    expect(skipped).toBe(0);
    expect(moves).toHaveLength(4);
  });

  it('reports malformed rows instead of silently dropping them', () => {
    const { moves, skipped, firstIssue } = parseProgramCsv('150,0,150\n1,2\nfoo,1,2,3\n', 'joints');
    expect(moves).toHaveLength(1);
    expect(skipped).toBe(2);
    expect(firstIssue).toContain('line 2');
  });

  it('clamps a negative dwell to zero', () => {
    const { moves } = parseProgramCsv('J,0,90,-90,-4\n', 'joints');
    expect(moves[0].dwell).toBe(0);
  });

  it('round-trips through programToCsv, disabled rows included', () => {
    const original = [
      makeMove('joints', [0, 90, -90], 0.5),
      { ...makeMove('cartesian', [150, 0, 150], 1.2), enabled: false },
    ];
    const { moves, skipped } = parseProgramCsv(programToCsv(original), 'cartesian');
    expect(skipped).toBe(0);
    expect(moves.map((m) => [m.kind, m.values, m.dwell, m.enabled])).toEqual(
      original.map((m) => [m.kind, m.values, m.dwell, m.enabled]),
    );
  });
});

describe('programToRobotScript', () => {
  it('emits MOVEJ per step with a SLEEP for each dwell', () => {
    const script = programToRobotScript([
      { anglesDeg: [0, 90, -90], dwell: 0.5 },
      { anglesDeg: [30, 60, -45], dwell: 0 },
    ]);
    const commands = script.split('\n').filter((l) => l && !l.startsWith('#'));
    expect(commands).toEqual(['MOVEJ 0.00 90.00 -90.00', 'SLEEP 0.50', 'MOVEJ 30.00 60.00 -45.00']);
  });

  it('sends validated joint angles for a Cartesian row, keeping the target as a note', () => {
    const script = programToRobotScript([
      { anglesDeg: [0, 45, -30], dwell: 0, cartesianMm: [150, 0, 150] },
    ]);
    expect(script).toContain('# target x=150.0 y=0.0 z=150.0 mm');
    expect(script).toContain('MOVEJ 0.00 45.00 -30.00');
    expect(script).not.toContain('MOVEL');
  });

  it('leaves the enabling prelude commented out', () => {
    const script = programToRobotScript([{ anglesDeg: [0, 90, -90], dwell: 0 }]);
    expect(script).toContain('# ENABLE');
    expect(script.split('\n')).not.toContain('ENABLE');
  });
});

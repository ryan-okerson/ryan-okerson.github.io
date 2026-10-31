// The analytic Jacobian must match central finite differences for every joint type.
import { describe, expect, it } from 'vitest';
import { build } from './helpers';
import { applyIncrement, compileSystem, evaluateSystem, type Pose } from '../src/solver/system';
import { quatFromRotVec } from '../src/math/vec';

const EVERY_JOINT = `
  plate('g', { points: { o: [0, 0, 0], p: [0.3, 0.1, 0] } });
  ground('g', { pos: [0.1, 0.2, 0.3], rot: { axis: [1, 2, 3], deg: 20 } });
  bar('A', { length: 1, points: { off: [0.2, 0.05, 0.01] } });
  bar('B', { length: 0.8 });
  bar('C', { length: 0.6 });
  bar('D', { length: 0.5 });
  pin('g.o', 'A.a', { axis: [0.2, 0.3, 1] });
  ball('A.b', 'B.a');
  ujoint('B.b', 'C.a', { axes: ['y', 'z'] });
  fix('C', 'D', { offset: [0.1, 0.2, 0], rot: { axis: 'x', deg: 30 } });
  gear('A', 'C', { ratio: -1.5, ref: 'g', offset: 0.2 });
  custom((q) => [q.point('D.b')[0] - 0.5, q.axis('B', 'x')[2]]);
  drive('g', 'A', { profile: ramp({ from: 10, to: 50, deg: true, time: 1 }) });
`;

function randomPoses(n: number, seed: number): Pose[] {
  let s = seed;
  const rand = () => ((s = (s * 16807) % 2147483647) / 2147483647) * 2 - 1;
  return Array.from({ length: n }, () => ({
    r: [rand(), rand(), rand()] as [number, number, number],
    q: quatFromRotVec([rand() * 2, rand() * 2, rand() * 2]),
  }));
}

describe('analytic Jacobian', () => {
  it('matches finite differences', () => {
    const system = compileSystem(build(EVERY_JOINT));
    const poses = randomPoses(system.bodyNames.length, 7);
    const { J } = evaluateSystem(system, poses, 0.3, true);
    const h = 1e-6;
    let worst = 0;
    for (let col = 0; col < system.columnCount; col++) {
      const dx = new Float64Array(system.columnCount);
      dx[col] = h;
      const plus = evaluateSystem(system, applyIncrement(poses, dx, 1), 0.3, false).res;
      const minus = evaluateSystem(system, applyIncrement(poses, dx, -1), 0.3, false).res;
      for (let row = 0; row < system.rowCount; row++) {
        const numeric = (plus[row] - minus[row]) / (2 * h);
        worst = Math.max(worst, Math.abs(numeric - J!.get(row, col)));
      }
    }
    expect(worst).toBeLessThan(1e-6);
  });
});

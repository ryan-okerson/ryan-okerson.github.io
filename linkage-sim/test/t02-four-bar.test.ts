import { describe, expect, it } from 'vitest';
import { solve } from './helpers';
import { measureAngle, framesFromPoses } from '../src/solver/system';

const a = 0.4, b = 1.0, c = 0.8, d = 1.0;

const FOUR_BAR = `
  plate('g', { points: { o2: [0, 0, 0], o4: [${d}, 0, 0] } });
  ground('g');
  bar('crank', { length: ${a} });
  bar('coupler', { length: ${b} });
  bar('rocker', { length: ${c} });
  pin('g.o2', 'crank.a');
  pin('crank.b', 'coupler.a');
  pin('g.o4', 'rocker.a');
  pin('coupler.b', 'rocker.b');
  drive('g', 'crank', { profile: ramp({ from: 30, to: 150, deg: true, time: 1 }) });
`;

// Closed-form rocker angle (Freudenstein), both assembly branches.
function rockerAngles(theta2: number): [number, number] {
  const K1 = d / a, K2 = d / c, K3 = (a * a - b * b + c * c + d * d) / (2 * a * c);
  const A = Math.cos(theta2) - K1 - K2 * Math.cos(theta2) + K3;
  const B = -2 * Math.sin(theta2);
  const C = K1 - (K2 + 1) * Math.cos(theta2) + K3;
  const root = Math.sqrt(B * B - 4 * A * C);
  return [2 * Math.atan2(-B - root, 2 * A), 2 * Math.atan2(-B + root, 2 * A)];
}

const wrap = (x: number) => Math.atan2(Math.sin(x), Math.cos(x));

describe('test 2: planar four-bar', () => {
  it('has 1 free motion and matches the closed-form output angle within 1e-9 rad', () => {
    const { model, result } = solve(FOUR_BAR, { steps: 25 });
    const g = model.bodies.findIndex((x) => x.name === 'g');
    const rocker = model.bodies.findIndex((x) => x.name === 'rocker');
    let branch = -1;
    let worst = 0;
    for (const step of result.steps) {
      expect(step.converged).toBe(true);
      expect(step.freeMotions).toBe(1);
      const frames = framesFromPoses(step.poses);
      const theta4 = measureAngle(frames[g], [0, 0, 1], frames[rocker]).angle;
      const exact = rockerAngles(step.drivenAngle);
      if (branch < 0) branch = Math.abs(wrap(theta4 - exact[0])) < Math.abs(wrap(theta4 - exact[1])) ? 0 : 1;
      worst = Math.max(worst, Math.abs(wrap(theta4 - exact[branch])));
    }
    console.log(`four-bar: branch ${branch}, worst output-angle error ${worst.toExponential(2)} rad`);
    expect(worst).toBeLessThan(1e-9);
  });
});

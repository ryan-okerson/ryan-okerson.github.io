import { describe, expect, it } from 'vitest';
import { deg, solve } from './helpers';
import { GEARED_ROOT } from './models';

// The expected answer is deliberately not asserted: the test prints what happens.
describe('test 4: lazy tong on two fixed root pins with a gear (ratio -1)', () => {
  it('reports whether it moves or locks, and the residual through the stroke', () => {
    const { result } = solve(GEARED_ROOT, { steps: 12, stopOnFailure: false });
    const rows = result.steps.map((s) =>
      `${deg(s.drivenAngle).toFixed(1).padStart(6)} deg  free=${s.freeMotions}  converged=${s.converged}  maxResidual=${s.maxResidual.toExponential(3)}`);
    console.log(['geared root, Kutzbach ' + result.kutzbach, ...rows].join('\n'));
    expect(result.steps.length).toBe(12);
  });
});

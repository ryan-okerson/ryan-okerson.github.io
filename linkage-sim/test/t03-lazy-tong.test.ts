import { describe, expect, it } from 'vitest';
import { solve } from './helpers';
import { LAZY_TONG } from './models';


describe('test 3: planar lazy tong, 3 units', () => {
  it('Kutzbach = -5 but rank-based free motions = 1', () => {
    const { model, result } = solve(LAZY_TONG, { steps: 21 });
    expect(model.bodies.length).toBe(6);
    expect(model.joints.filter((j) => j.type === 'pin').length).toBe(7);
    expect(result.kutzbach).toBe(-5);
    for (const step of result.steps) {
      expect(step.converged).toBe(true);
      expect(step.freeMotions).toBe(1);
    }
    expect(result.steps.length).toBe(21);
  });
});

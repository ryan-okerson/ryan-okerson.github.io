import { describe, expect, it } from 'vitest';
import { solve } from './helpers';
import { tongOfUnits } from './models';

describe('test 9: repeat + chain', () => {
  for (const n of [1, 3, 20]) {
    it(`n = ${n}: 2n bodies and 1 free motion every step`, () => {
      const { model, result } = solve(tongOfUnits(n), { steps: 6 });
      expect(model.bodies.length).toBe(2 * n);
      expect(result.steps.length).toBe(6);
      for (const step of result.steps) {
        expect(step.converged).toBe(true);
        expect(step.freeMotions).toBe(1);
      }
    });
  }
});

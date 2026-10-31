import { describe, expect, it } from 'vitest';
import { deg, solve } from './helpers';
import { doubleTong } from './models';

describe('test 5: two-unit spatial double tong, ball-ended crossed webs with crossing pins', () => {
  it('has 1 free motion at 60, 30 and 7.5 deg', () => {
    const { model, result } = solve(doubleTong('{}'), { steps: 8 });
    console.log(result.steps.map((s) => `${deg(s.drivenAngle).toFixed(1)} deg free=${s.freeMotions} res=${s.maxResidual.toExponential(2)} sigma=${s.smallestSingular.slice(0, 3).map((v) => v.toExponential(2)).join(' ')}`).join('\n'));
    console.log(`bodies ${model.bodies.length}, Kutzbach ${result.kutzbach}`);
    for (const target of [60, 30, 7.5]) {
      const step = result.steps.find((s) => Math.abs(deg(s.drivenAngle) - target) < 1e-6);
      expect(step, `step at ${target} deg`).toBeDefined();
      expect(step!.converged).toBe(true);
      expect(step!.freeMotions).toBe(1);
    }
  });
});

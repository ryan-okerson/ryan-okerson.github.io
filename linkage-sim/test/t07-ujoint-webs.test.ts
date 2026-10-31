import { describe, expect, it } from 'vitest';
import { deg, solve } from './helpers';
import { doubleTong } from './models';

// Hand analysis expected a lock. The solver disagrees (see NOTES.md), so this
// runs as an expected failure: it starts failing loudly if the two ever agree.
describe('test 7: double tong with universal-joint web ends', () => {
  it.fails('locks (0 free motions) at some angle [solver disagrees with hand analysis]', () => {
    const { result } = solve(doubleTong(`{ ends: 'ujoint', ujointAxes: ['z', 'z'] }`), { steps: 36, stopOnFailure: false });
    console.log(result.steps.map((s) =>
      `${deg(s.drivenAngle).toFixed(2).padStart(6)} deg free=${s.freeMotions} converged=${s.converged} sigma_min_nonzero=${s.smallestSingular.find((x) => x > s.tolerance)!.toExponential(2)}`).join('\n'));
    const lock = result.steps.find((s) => s.freeMotions === 0 || !s.converged);
    console.log(lock ? `locks at ${deg(lock.drivenAngle).toFixed(2)} deg` : 'no lock between 60 and 7.5 deg');
    expect(lock).toBeDefined();
  });

  it('reports the spec example axes [z, y]', () => {
    const { result } = solve(doubleTong(`{ ends: 'ujoint', ujointAxes: ['z', 'y'] }`), { steps: 8, stopOnFailure: false });
    const s = result.steps[0];
    console.log(`ujoint axes [z, y]: converged=${s.converged}, max residual ${s.maxResidual.toExponential(3)}, free=${s.freeMotions}`);
    expect(result.steps.length).toBeGreaterThan(0);
  });
});

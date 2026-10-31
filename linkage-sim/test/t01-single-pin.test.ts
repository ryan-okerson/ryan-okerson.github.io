import { describe, expect, it } from 'vitest';
import { solve } from './helpers';

describe('test 1: single pin between ground and one bar', () => {
  it('has 1 free motion', () => {
    const { result } = solve(`
      plate('g', { points: { o: [0, 0, 0] } });
      ground('g');
      bar('crank', { length: 1 });
      pin('g.o', 'crank.a', { axis: 'z' });
    `);
    const step = result.steps[0];
    expect(step.converged).toBe(true);
    expect(step.freeMotions).toBe(1);
  });
});

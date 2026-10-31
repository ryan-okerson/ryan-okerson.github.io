import { describe, expect, it } from 'vitest';
import { build } from './helpers';
import { LAZY_TONG } from './models';

describe('test 8: group scaling', () => {
  it('scaling group tong by 1.2 scales every tagged bar; fractional pivots keep their fraction', () => {
    const base = build(LAZY_TONG);
    const scaled = build(LAZY_TONG, { groupScales: { tong: 1.2 } });
    expect(scaled.groups.find((g) => g.name === 'tong')!.scale).toBe(1.2);
    for (const bar of scaled.bodies) {
      const original = base.bodies.find((b) => b.name === bar.name)!;
      expect(bar.tags).toContain('tong');
      expect(bar.length).toBeCloseTo(original.length * 1.2, 12);
      for (const [name, fraction] of Object.entries(bar.fractions)) {
        expect(fraction).toBe(original.fractions[name]);
        expect(bar.points[name][0]).toBeCloseTo(fraction * bar.length, 12);
      }
    }
  });
});

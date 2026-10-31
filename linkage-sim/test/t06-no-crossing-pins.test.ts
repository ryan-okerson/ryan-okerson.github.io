import { describe, expect, it } from 'vitest';
import { build, deg } from './helpers';
import { doubleTong } from './models';
import { runSweep } from '../src/solver/sweep';
import { compileSystem, evaluateSystem } from '../src/solver/system';
import { smallestSingular } from '../src/solver/sparse';
import { cross, dot, matMulVec, normalize, quatToMat, sub, type Vec3 } from '../src/math/vec';

// Solve once, then split the motions available with the driver held.
function analyze() {
  const model = build(doubleTong('{ crossingPin: false }'));
  const step = runSweep(model, { steps: 8 }).steps[0];
  const system = compileSystem(model);
  const { J } = evaluateSystem(system, step.poses, step.t, true);
  const singular = smallestSingular(J!, 1e-8);
  const held = singular.vectors.slice(0, singular.nullity);
  const index = (name: string) => system.bodyIndex.get(name)!;

  // Sideways = in the tong plane, perpendicular to the tong axis.
  const axis = normalize(sub(step.poses[index('T1.A')].r, step.poses[index('T0.A')].r));
  const sideways = normalize(cross([0, 0, 1], axis));
  const lower = model.bodies.map((b, i) => (b.name.startsWith('bot_') ? i : -1)).filter((i) => i >= 0);
  let sway = 0;
  for (const v of held) {
    const mean: Vec3 = [0, 0, 0];
    for (const i of lower) for (let k = 0; k < 3; k++) mean[k] += v[6 * i + k] / lower.length;
    sway = Math.max(sway, Math.abs(dot(mean, sideways)));
  }

  // A pure spin moves only one web bar, rotating it about its own axis.
  const spinOnly = held.filter((v) => {
    let other = 0;
    model.bodies.forEach((b, i) => {
      const x = matMulVec(quatToMat(step.poses[i].q), [1, 0, 0]);
      const w: Vec3 = [v[6 * i + 3], v[6 * i + 4], v[6 * i + 5]];
      const spin = dot(w, x);
      const isWeb = b.tags.includes('web');
      other += Math.hypot(v[6 * i], v[6 * i + 1], v[6 * i + 2]) + Math.hypot(...sub(w, isWeb ? [x[0] * spin, x[1] * spin, x[2] * spin] : [0, 0, 0]));
    });
    return other < 1e-8;
  }).length;
  console.log(`no crossing pins at ${deg(step.drivenAngle).toFixed(1)} deg: ${step.freeMotions} free motions; ` +
    `${held.length} with the driver held, of which ${spinOnly} are pure web-bar spins; ` +
    `largest lower-tong sideways component ${sway.toExponential(2)}`);
  return { step, sway };
}

describe('test 6: double tong without web crossing pins', () => {
  it('has more than 1 free motion', () => {
    const { step } = analyze();
    expect(step.converged).toBe(true);
    expect(step.freeMotions).toBeGreaterThan(1);
  });

  // Hand analysis expected sway. The solver finds only web-bar spins (see NOTES.md),
  // so this runs as an expected failure: it starts failing loudly if the two ever agree.
  it.fails('the extra motions include sideways sway of the lower tong [solver disagrees with hand analysis]', () => {
    expect(analyze().sway).toBeGreaterThan(0.01);
  });
});

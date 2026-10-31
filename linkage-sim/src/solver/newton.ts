// Position solve: Gauss-Newton with least-squares steps (damped normal equations) and backtracking.
import { type Pose, type System, applyIncrement, evaluateSystem } from './system';
import { maxAbs, solveLeastSquares, sumSquares } from './sparse';

export interface NewtonOptions {
  tolerance: number;
  maxIterations: number;
}

// Damping of the normal equations, relative to their largest diagonal entry.
const NEWTON_MU = 1e-12;

export interface NewtonResult {
  poses: Pose[];
  converged: boolean;
  iterations: number;
  maxResidual: number;
}

export function solvePositions(system: System, start: Pose[], t: number, opts: NewtonOptions): NewtonResult {
  let poses = start;
  let res = evaluateSystem(system, poses, t, false).res;
  for (let iter = 0; iter < opts.maxIterations; iter++) {
    const err = maxAbs(res);
    if (err < opts.tolerance) return { poses, converged: true, iterations: iter, maxResidual: err };
    const { J } = evaluateSystem(system, poses, t, true);
    const step = solveLeastSquares(J!, res.map((r) => -r), NEWTON_MU);
    const next = backtrack(system, poses, res, step, t);
    // No step reduced the residual: stuck at a least-squares minimum.
    if (!next) break;
    poses = next.poses;
    res = next.res;
  }
  const err = maxAbs(res);
  return { poses, converged: err < opts.tolerance, iterations: opts.maxIterations, maxResidual: err };
}

function backtrack(system: System, poses: Pose[], res: Float64Array, step: Float64Array, t: number): { poses: Pose[]; res: Float64Array } | null {
  const base = sumSquares(res);
  let alpha = 1;
  for (let k = 0; k < 12; k++) {
    const trial = applyIncrement(poses, step, alpha);
    const trialRes = evaluateSystem(system, trial, t, false).res;
    if (sumSquares(trialRes) < base) return { poses: trial, res: trialRes };
    alpha /= 2;
  }
  return null;
}

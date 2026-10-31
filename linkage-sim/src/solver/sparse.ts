// Sparse Jacobian and the linear algebra built on it.
//
// Least squares: regularized normal equations (J^T J + mu I) x = J^T b, factored
// with a profile (skyline) Cholesky after a reverse Cuthill-McKee ordering of the
// columns. Each body touches few joints, so the profile stays narrow and the cost
// grows roughly linearly with the number of bars.
//
// Smallest singular values: block inverse iteration with that factor, then a
// Rayleigh-Ritz step on J itself (not J^T J), so values near zero keep full precision.
import { Matrix, SVD } from 'ml-matrix';

export class SparseJacobian {
  readonly cols: number[][];
  readonly vals: number[][];

  constructor(readonly rowCount: number, readonly columnCount: number) {
    this.cols = Array.from({ length: rowCount }, () => []);
    this.vals = Array.from({ length: rowCount }, () => []);
  }

  add(row: number, col: number, value: number): void {
    const cols = this.cols[row];
    const at = cols.indexOf(col);
    if (at >= 0) {
      this.vals[row][at] += value;
      return;
    }
    cols.push(col);
    this.vals[row].push(value);
  }

  get(row: number, col: number): number {
    const at = this.cols[row].indexOf(col);
    return at >= 0 ? this.vals[row][at] : 0;
  }

  // y = J x, skipping one row (its entry is left 0).
  multiply(x: ArrayLike<number>, skipRow = -1): Float64Array {
    const y = new Float64Array(this.rowCount);
    for (let r = 0; r < this.rowCount; r++) {
      if (r === skipRow) continue;
      const cols = this.cols[r];
      const vals = this.vals[r];
      let s = 0;
      for (let k = 0; k < cols.length; k++) s += vals[k] * x[cols[k]];
      y[r] = s;
    }
    return y;
  }

  // x = J^T y, skipping one row.
  multiplyTransposed(y: ArrayLike<number>, skipRow = -1): Float64Array {
    const x = new Float64Array(this.columnCount);
    for (let r = 0; r < this.rowCount; r++) {
      if (r === skipRow || y[r] === 0) continue;
      const cols = this.cols[r];
      const vals = this.vals[r];
      for (let k = 0; k < cols.length; k++) x[cols[k]] += vals[k] * y[r];
    }
    return x;
  }
}

// Cholesky factor of P (J^T J + mu I) P^T in skyline storage.
export class NormalFactor {
  private order: Int32Array;
  private first: Int32Array;
  private rows: Float64Array[];
  readonly mu: number;

  constructor(J: SparseJacobian, muRel: number, skipRow = -1) {
    const n = J.columnCount;
    this.order = reverseCuthillMcKee(J, skipRow);
    const position = new Int32Array(n);
    this.order.forEach((col, i) => (position[col] = i));
    this.first = Int32Array.from({ length: n }, (_, i) => i);
    for (let r = 0; r < J.rowCount; r++) {
      if (r === skipRow) continue;
      const ps = J.cols[r].map((c) => position[c]);
      const lo = Math.min(...ps);
      for (const p of ps) this.first[p] = Math.min(this.first[p], lo);
    }
    this.rows = Array.from({ length: n }, (_, i) => new Float64Array(i - this.first[i] + 1));
    let maxDiag = 0;
    for (let r = 0; r < J.rowCount; r++) {
      if (r === skipRow) continue;
      const cols = J.cols[r];
      const vals = J.vals[r];
      for (let a = 0; a < cols.length; a++) {
        const pa = position[cols[a]];
        for (let b = 0; b < cols.length; b++) {
          const pb = position[cols[b]];
          if (pb > pa) continue;
          this.rows[pa][pb - this.first[pa]] += vals[a] * vals[b];
        }
      }
    }
    for (let i = 0; i < n; i++) maxDiag = Math.max(maxDiag, this.rows[i][i - this.first[i]]);
    this.mu = muRel * Math.max(maxDiag, 1);
    for (let i = 0; i < n; i++) this.rows[i][i - this.first[i]] += this.mu;
    this.factor();
  }

  private factor(): void {
    const n = this.rows.length;
    for (let i = 0; i < n; i++) {
      const fi = this.first[i];
      const Li = this.rows[i];
      for (let j = fi; j <= i; j++) {
        const fj = this.first[j];
        const Lj = this.rows[j];
        let s = Li[j - fi];
        for (let k = Math.max(fi, fj); k < j; k++) s -= Li[k - fi] * Lj[k - fj];
        if (j < i) {
          Li[j - fi] = s / Lj[j - fj];
        } else {
          Li[i - fi] = Math.sqrt(Math.max(s, this.mu * 1e-3));
        }
      }
    }
  }

  // x = (J^T J + mu I)^-1 b
  solve(b: ArrayLike<number>): Float64Array {
    const n = this.rows.length;
    const y = new Float64Array(n);
    for (let i = 0; i < n; i++) y[i] = b[this.order[i]];
    for (let i = 0; i < n; i++) {
      const fi = this.first[i];
      const Li = this.rows[i];
      let s = y[i];
      for (let k = fi; k < i; k++) s -= Li[k - fi] * y[k];
      y[i] = s / Li[i - fi];
    }
    for (let i = n - 1; i >= 0; i--) {
      const fi = this.first[i];
      const Li = this.rows[i];
      y[i] /= Li[i - fi];
      for (let k = fi; k < i; k++) y[k] -= Li[k - fi] * y[i];
    }
    const x = new Float64Array(n);
    for (let i = 0; i < n; i++) x[this.order[i]] = y[i];
    return x;
  }
}

// Least-squares solution of J x = b (close to minimum norm for small mu).
export function solveLeastSquares(J: SparseJacobian, b: ArrayLike<number>, muRel = 1e-13, factor?: NormalFactor): Float64Array {
  const f = factor ?? new NormalFactor(J, muRel);
  return f.solve(J.multiplyTransposed(b));
}

// Column order that keeps the J^T J profile narrow.
function reverseCuthillMcKee(J: SparseJacobian, skipRow: number): Int32Array {
  const n = J.columnCount;
  const neighbors: Set<number>[] = Array.from({ length: n }, () => new Set());
  for (let r = 0; r < J.rowCount; r++) {
    if (r === skipRow) continue;
    const cols = J.cols[r];
    for (const a of cols) for (const b of cols) if (a !== b) neighbors[a].add(b);
  }
  const degree = neighbors.map((s) => s.size);
  const visited = new Uint8Array(n);
  const order: number[] = [];
  const byDegree = Array.from({ length: n }, (_, i) => i).sort((a, b) => degree[a] - degree[b]);
  for (const start of byDegree) {
    if (visited[start]) continue;
    visited[start] = 1;
    const queue = [start];
    for (let head = 0; head < queue.length; head++) {
      const c = queue[head];
      order.push(c);
      const next = [...neighbors[c]].filter((x) => !visited[x]).sort((a, b) => degree[a] - degree[b]);
      for (const x of next) {
        visited[x] = 1;
        queue.push(x);
      }
    }
  }
  return Int32Array.from(order.reverse());
}

export interface SingularInfo {
  // Smallest singular values found, ascending.
  sigma: number[];
  // Right singular vectors matching sigma.
  vectors: Float64Array[];
  largest: number;
  tolerance: number;
  // Number of singular values at or below tolerance (dimension of the null space).
  nullity: number;
}

// Smallest singular values of J (optionally without one row) and their vectors.
export function smallestSingular(J: SparseJacobian, rankTolRel: number, skipRow = -1, minBlock = 8): SingularInfo {
  const n = J.columnCount;
  const largest = largestSingular(J, skipRow);
  const tolerance = rankTolRel * Math.max(largest, 1);
  const factor = new NormalFactor(J, 1e-14, skipRow);
  let block = Math.min(n, minBlock);
  for (;;) {
    const info = inverseIteration(J, factor, block, skipRow);
    const nullity = info.sigma.filter((s) => s <= tolerance).length;
    // The block must reach past the null space to see the first nonzero value.
    if (nullity < block - 2 || block === n) return { ...info, largest, tolerance, nullity };
    block = Math.min(n, block * 2);
  }
}

function inverseIteration(J: SparseJacobian, factor: NormalFactor, block: number, skipRow: number): { sigma: number[]; vectors: Float64Array[] } {
  const n = J.columnCount;
  let seed = 12345;
  const rand = () => ((seed = (seed * 16807) % 2147483647) / 2147483647) - 0.5;
  let V: Float64Array[] = Array.from({ length: block }, () => Float64Array.from({ length: n }, rand));
  orthonormalize(V);
  let previous: number[] = [];
  for (let iter = 0; iter < 60; iter++) {
    V = V.map((v) => factor.solve(v));
    orthonormalize(V);
    const { sigma } = rayleighRitz(J, V, skipRow);
    const settled = previous.length === sigma.length && sigma.every((s, i) => Math.abs(s - previous[i]) <= 1e-9 * Math.max(s, 1e-3));
    previous = sigma;
    if (settled && iter >= 3) break;
  }
  return rayleighRitz(J, V, skipRow);
}

// Best singular values and vectors of J within span(V), ascending.
function rayleighRitz(J: SparseJacobian, V: Float64Array[], skipRow: number): { sigma: number[]; vectors: Float64Array[] } {
  const b = V.length;
  const columns = V.map((v) => J.multiply(v, skipRow));
  const m = Math.max(J.rowCount, b);
  const Y = Matrix.zeros(m, b);
  columns.forEach((col, j) => col.forEach((value, i) => Y.set(i, j, value)));
  const svd = new SVD(Y, { computeLeftSingularVectors: false, computeRightSingularVectors: true, autoTranspose: false });
  const W = svd.rightSingularVectors;
  const pairs = svd.diagonal.map((s, k) => {
    const vec = new Float64Array(V[0].length);
    for (let j = 0; j < b; j++) {
      const w = W.get(j, k);
      if (w === 0) continue;
      const v = V[j];
      for (let i = 0; i < vec.length; i++) vec[i] += w * v[i];
    }
    return { s, vec };
  });
  pairs.sort((x, y) => x.s - y.s);
  return { sigma: pairs.map((p) => p.s), vectors: pairs.map((p) => p.vec) };
}

// Largest singular value by power iteration on J^T J.
function largestSingular(J: SparseJacobian, skipRow: number): number {
  let v = Float64Array.from({ length: J.columnCount }, (_, i) => 1 + (i % 7) * 0.1);
  let estimate = 0;
  for (let iter = 0; iter < 50; iter++) {
    const w = J.multiplyTransposed(J.multiply(v, skipRow), skipRow);
    const norm = Math.hypot(...w);
    if (norm === 0) return 0;
    const next = Math.sqrt(norm / Math.hypot(...v));
    v = w.map((x) => x / norm);
    if (Math.abs(next - estimate) < 1e-6 * next) return next;
    estimate = next;
  }
  return estimate;
}

function orthonormalize(V: Float64Array[]): void {
  for (let j = 0; j < V.length; j++) {
    const v = V[j];
    for (let pass = 0; pass < 2; pass++) {
      for (let k = 0; k < j; k++) {
        const u = V[k];
        let d = 0;
        for (let i = 0; i < v.length; i++) d += u[i] * v[i];
        for (let i = 0; i < v.length; i++) v[i] -= d * u[i];
      }
    }
    let norm = 0;
    for (let i = 0; i < v.length; i++) norm += v[i] * v[i];
    norm = Math.sqrt(norm);
    if (norm < 1e-300) {
      v.fill(0);
      v[j % v.length] = 1;
      continue;
    }
    for (let i = 0; i < v.length; i++) v[i] /= norm;
  }
}

export function maxAbs(v: ArrayLike<number>): number {
  let m = 0;
  for (let i = 0; i < v.length; i++) m = Math.max(m, Math.abs(v[i]));
  return m;
}

export function sumSquares(v: ArrayLike<number>): number {
  let s = 0;
  for (let i = 0; i < v.length; i++) s += v[i] * v[i];
  return s;
}

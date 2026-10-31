// Small 3D vector, matrix and quaternion helpers. Quaternions are [w, x, y, z].

export type Vec3 = [number, number, number];
export type Quat = [number, number, number, number];
// Row-major 3x3 matrix.
export type Mat3 = [number, number, number, number, number, number, number, number, number];

export const ZERO: Vec3 = [0, 0, 0];
export const EX: Vec3 = [1, 0, 0];
export const EY: Vec3 = [0, 1, 0];
export const EZ: Vec3 = [0, 0, 1];
export const QUAT_IDENTITY: Quat = [1, 0, 0, 0];

export function add(a: Vec3, b: Vec3): Vec3 {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

export function sub(a: Vec3, b: Vec3): Vec3 {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

export function scale(a: Vec3, s: number): Vec3 {
  return [a[0] * s, a[1] * s, a[2] * s];
}

export function dot(a: Vec3, b: Vec3): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

export function cross(a: Vec3, b: Vec3): Vec3 {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}

export function norm(a: Vec3): number {
  return Math.hypot(a[0], a[1], a[2]);
}

export function normalize(a: Vec3): Vec3 {
  const n = norm(a);
  if (n === 0) return [0, 0, 0];
  return scale(a, 1 / n);
}

// Any unit vector perpendicular to a (a need not be unit).
export function perpendicular(a: Vec3): Vec3 {
  const u = normalize(a);
  const helper: Vec3 = Math.abs(u[0]) < 0.9 ? EX : EY;
  return normalize(cross(u, helper));
}

export function matMulVec(m: Mat3, v: Vec3): Vec3 {
  return [
    m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
    m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
    m[6] * v[0] + m[7] * v[1] + m[8] * v[2],
  ];
}

export function matMul(a: Mat3, b: Mat3): Mat3 {
  const out = new Array(9).fill(0) as Mat3;
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      out[i * 3 + j] = a[i * 3] * b[j] + a[i * 3 + 1] * b[3 + j] + a[i * 3 + 2] * b[6 + j];
    }
  }
  return out;
}

export function quatMul(a: Quat, b: Quat): Quat {
  return [
    a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
    a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
    a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
  ];
}

export function quatConj(q: Quat): Quat {
  return [q[0], -q[1], -q[2], -q[3]];
}

export function quatNormalize(q: Quat): Quat {
  const n = Math.hypot(q[0], q[1], q[2], q[3]);
  return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
}

export function quatFromAxisAngle(axis: Vec3, angle: number): Quat {
  const u = normalize(axis);
  const s = Math.sin(angle / 2);
  return [Math.cos(angle / 2), u[0] * s, u[1] * s, u[2] * s];
}

// Exponential map: rotation vector (axis * angle) to quaternion.
export function quatFromRotVec(v: Vec3): Quat {
  const angle = norm(v);
  if (angle < 1e-12) return quatNormalize([1, v[0] / 2, v[1] / 2, v[2] / 2]);
  return quatFromAxisAngle(v, angle);
}

// Smallest rotation that turns direction a into direction b.
export function quatFromTwoVectors(a: Vec3, b: Vec3): Quat {
  const u = normalize(a);
  const v = normalize(b);
  const c = dot(u, v);
  if (c < -0.999999999) return quatFromAxisAngle(perpendicular(u), Math.PI);
  const axis = cross(u, v);
  return quatNormalize([1 + c, axis[0], axis[1], axis[2]]);
}

export function quatToMat(q: Quat): Mat3 {
  const [w, x, y, z] = q;
  return [
    1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
    2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
    2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y),
  ];
}

export function quatFromMat(m: Mat3): Quat {
  const trace = m[0] + m[4] + m[8];
  if (trace > 0) {
    const s = Math.sqrt(trace + 1) * 2;
    return quatNormalize([0.25 * s, (m[7] - m[5]) / s, (m[2] - m[6]) / s, (m[3] - m[1]) / s]);
  }
  if (m[0] > m[4] && m[0] > m[8]) {
    const s = Math.sqrt(1 + m[0] - m[4] - m[8]) * 2;
    return quatNormalize([(m[7] - m[5]) / s, 0.25 * s, (m[1] + m[3]) / s, (m[2] + m[6]) / s]);
  }
  if (m[4] > m[8]) {
    const s = Math.sqrt(1 + m[4] - m[0] - m[8]) * 2;
    return quatNormalize([(m[2] - m[6]) / s, (m[1] + m[3]) / s, 0.25 * s, (m[5] + m[7]) / s]);
  }
  const s = Math.sqrt(1 + m[8] - m[0] - m[4]) * 2;
  return quatNormalize([(m[3] - m[1]) / s, (m[2] + m[6]) / s, (m[5] + m[7]) / s, 0.25 * s]);
}

export function quatRotate(q: Quat, v: Vec3): Vec3 {
  return matMulVec(quatToMat(q), v);
}

// Angle of the relative rotation between two orientations, in [0, pi].
export function quatAngleBetween(a: Quat, b: Quat): number {
  const d = Math.abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
  return 2 * Math.acos(Math.min(1, d));
}

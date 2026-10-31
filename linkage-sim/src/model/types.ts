// Plain JSON model produced by the authoring API and consumed by the solver.
// Units: meters and radians.
import type { Quat, Vec3 } from '../math/vec';

export interface ParamDef {
  name: string;
  value: number;
  min: number;
  max: number;
  unit?: string;
  group?: string;
  int?: boolean;
}

export interface GroupDef {
  name: string;
  scale: number;
}

export interface PoseDef {
  pos: Vec3;
  quat: Quat;
}

export interface BodyDef {
  name: string;
  kind: 'bar' | 'plate';
  // Bar length after group scaling (0 for plates).
  length: number;
  tags: string[];
  // Named points in the body's local frame (meters).
  points: Record<string, Vec3>;
  // For points given as a fraction of bar length: the fraction.
  fractions: Record<string, number>;
  hint?: PoseDef;
  // Set for bodies made by copy()/mirror(): start pose = transform of the source body's pose.
  copyOf?: CopySource;
}

export interface CopySource {
  source: string;
  // Row-major 3x3 linear part (rotation or reflection) and shift of the copy transform.
  linear: number[];
  shift: Vec3;
  improper: boolean;
}

export interface PointRef {
  body: string;
  point: string;
}

export interface GroundJoint {
  type: 'ground';
  body: string;
  pose: PoseDef;
}

export interface PinJoint {
  type: 'pin';
  a: PointRef;
  b: PointRef;
  // Axis in body a's local frame, and the matching axis in body b's local frame.
  axisA: Vec3;
  axisB: Vec3;
}

export interface BallJoint {
  type: 'ball';
  a: PointRef;
  b: PointRef;
}

export interface UJoint {
  type: 'ujoint';
  a: PointRef;
  b: PointRef;
  axisA: Vec3;
  axisB: Vec3;
}

export interface FixJoint {
  type: 'fix';
  a: string;
  b: string;
  // Pose of b's frame in a's frame.
  offset: Vec3;
  quat: Quat;
}

export interface GearJoint {
  type: 'gear';
  a: string;
  b: string;
  ratio: number;
  // Angles are measured relative to this body about its local axis.
  ref: string;
  axis: Vec3;
  offset: number;
  about?: [PointRef, PointRef];
}

export interface CustomJoint {
  type: 'custom';
  label: string;
  source: string;
  // Live function; dropped when the model is serialized.
  fn?: (q: CustomAccess) => number[];
}

export type JointDef = GroundJoint | PinJoint | BallJoint | UJoint | FixJoint | GearJoint | CustomJoint;

export type ProfileDef =
  | { kind: 'ramp'; from: number; to: number; time: number }
  | { kind: 'sCurve'; from: number; to: number; time: number }
  | { kind: 'table'; points: [number, number][] };

export interface DriverDef {
  a: string;
  b: string;
  axis: Vec3;
  profile: ProfileDef;
}

export interface Model {
  params: ParamDef[];
  groups: GroupDef[];
  bodies: BodyDef[];
  joints: JointDef[];
  driver: DriverDef | null;
}

// What a custom() residual function receives.
export interface CustomAccess {
  t: number;
  point(ref: string): Vec3;
  axis(body: string, localAxis: Vec3 | string): Vec3;
  pose(body: string): { pos: Vec3; quat: Quat };
}

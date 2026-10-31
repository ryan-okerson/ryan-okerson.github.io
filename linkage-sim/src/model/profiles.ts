// Motion profiles: value, first and second time derivative. Values in radians.
import type { ProfileDef } from './types';

export interface ProfileSample {
  value: number;
  rate: number;
  accel: number;
}

export function profileDuration(profile: ProfileDef): number {
  if (profile.kind === 'table') return profile.points[profile.points.length - 1][0];
  return profile.time;
}

export function sampleProfile(profile: ProfileDef, t: number): ProfileSample {
  if (profile.kind === 'ramp') return sampleRamp(profile.from, profile.to, profile.time, t);
  if (profile.kind === 'sCurve') return sampleSCurve(profile.from, profile.to, profile.time, t);
  return sampleTable(profile.points, t);
}

function sampleRamp(from: number, to: number, time: number, t: number): ProfileSample {
  const s = clamp01(t / time);
  const inside = t >= 0 && t <= time;
  return { value: from + (to - from) * s, rate: inside ? (to - from) / time : 0, accel: 0 };
}

// Half-cosine blend: zero rate at start and stop.
function sampleSCurve(from: number, to: number, time: number, t: number): ProfileSample {
  const s = clamp01(t / time);
  const span = to - from;
  const inside = t >= 0 && t <= time;
  return {
    value: from + (span * (1 - Math.cos(Math.PI * s))) / 2,
    rate: inside ? ((span * Math.PI) / (2 * time)) * Math.sin(Math.PI * s) : 0,
    accel: inside ? ((span * Math.PI * Math.PI) / (2 * time * time)) * Math.cos(Math.PI * s) : 0,
  };
}

function sampleTable(points: [number, number][], t: number): ProfileSample {
  if (t <= points[0][0]) return { value: points[0][1], rate: 0, accel: 0 };
  for (let i = 1; i < points.length; i++) {
    const [t0, v0] = points[i - 1];
    const [t1, v1] = points[i];
    if (t > t1) continue;
    const rate = (v1 - v0) / (t1 - t0);
    return { value: v0 + rate * (t - t0), rate, accel: 0 };
  }
  return { value: points[points.length - 1][1], rate: 0, accel: 0 };
}

function clamp01(s: number): number {
  return Math.min(1, Math.max(0, s));
}

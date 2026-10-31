// Example model scripts. Each uses the macros from library.js.

export const EXAMPLES: Record<string, string> = {
  'Lazy tong': `// Planar lazy tong: 3 scissor units, first A bar grounded.
const L = param('L', 1.0, { min: 0.2, max: 4, unit: 'm', group: 'tong' });
const n = param('units', 3, { int: true, min: 1, max: 40 });

const units = lazyTong(n, L);
ground(units[0].A);
drive(units[0].A, units[0].B, {
  profile: sCurve({ from: 85, to: 8, deg: true, time: 10 }),
});
`,

  'Geared root': `// Lazy tong whose two root bars sit on fixed, separate pins with a -1 gear.
// The pin spacing matches the starting angle, so the first step assembles.
const L = param('L', 1.0, { min: 0.2, max: 4, unit: 'm', group: 'tong' });
const n = param('units', 3, { int: true, min: 1, max: 40 });
const start = param('startAngle', 85, { min: 10, max: 120, unit: 'deg' });

const units = lazyTong(n, L);
gearedRoot(units, { spacing: L * Math.sin((start / 2) * Math.PI / 180) });
drive(units[0].A, units[0].B, {
  profile: ramp({ from: start, to: 8, deg: true, time: 10 }),
});
`,

  'Double tong': `// Spatial double tong: two lazy tongs joined by crossed, ball-ended webs
// with crossing pins. Try crossingPin: false or ends: 'ujoint'.
const L    = param('L', 1.0, { min: 0.2, max: 4, unit: 'm', group: 'tong' });
const Lweb = param('Lweb', 0.6, { min: 0.51, max: 2, unit: 'm', group: 'web' });
const n    = param('units', 2, { int: true, min: 1, max: 20 });

doubleTong(n, L, Lweb, { ends: 'ball', crossingPin: true });
drive('T0.A', 'T0.B', {
  profile: ramp({ from: 60, to: 7.5, deg: true, time: 7 }),
});
`,
};

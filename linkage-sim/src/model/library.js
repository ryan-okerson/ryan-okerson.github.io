// library.js: worked-example macros. Edit freely; they are ordinary functions
// built on the authoring API (bar, pin, ball, repeat, chain, copy, ...).

// One scissor unit: two bars of length L crossing at their midpoints.
// Bar A runs from the +y node down to the -y node, bar B the other way.
function scissorUnit(name, { L, tags = ['tong'] }) {
  const A = bar(name + '.A', { length: L, tags });
  const B = bar(name + '.B', { length: L, tags });
  pin(A + '.mid', B + '.mid', { axis: 'z' });
  return { A, B };
}

// Planar lazy tong of n scissor units along +x, pins about local z.
function lazyTong(n, L, { prefix = 'U', tags = ['tong'] } = {}) {
  const units = repeat(n, (i) => scissorUnit(`${prefix}${i}`, { L, tags }));
  chain(units, (u, next) => {
    pin(u.A + '.b', next.B + '.a', { axis: 'z' });
    pin(u.B + '.b', next.A + '.a', { axis: 'z' });
  });
  return units;
}

// Geared root: the first unit's two root ends sit on fixed, separate pins on a
// grounded root plate, and a gear (ratio -1) synchronizes the two root bars.
function gearedRoot(units, { spacing, ratio = -1, name = 'root' }) {
  plate(name, { points: { p1: [0, spacing / 2, 0], p2: [0, -spacing / 2, 0] }, tags: ['root'] });
  ground(name);
  pin(units[0].A + '.a', name + '.p1', { axis: 'z' });
  pin(units[0].B + '.a', name + '.p2', { axis: 'z' });
  gear(units[0].A, units[0].B, { ratio, about: [name + '.p1', name + '.p2'] });
}

// Crossed web pair between two tongs, aimed at one node of a unit:
// Wa runs from the top unit's crossing point to the bottom tong's node,
// Wb from the bottom unit's crossing point to the top tong's node. Both are
// L/2 from crossing point to node horizontally, so their length is constant.
// ends: 'ball' or 'ujoint' (with ujointAxes); crossingPin pins the two midpoints.
function webCross(name, topMid, botNode, botMid, topNode, { Lweb, ends = 'ball', ujointAxes = ['z', 'z'], crossingPin = true, tags = ['web'] }) {
  const Wa = bar(name + '.Wa', { length: Lweb, tags });
  const Wb = bar(name + '.Wb', { length: Lweb, tags });
  const join = (webPoint, node) => {
    if (ends === 'ujoint') ujoint(webPoint, node, { axes: ujointAxes });
    else ball(webPoint, node);
  };
  join(Wa + '.a', topMid);
  join(Wa + '.b', botNode);
  join(Wb + '.a', botMid);
  join(Wb + '.b', topNode);
  if (crossingPin) pin(Wa + '.mid', Wb + '.mid', { axis: 'z' });
  return { Wa, Wb };
}

// Spatial double tong: two lazy tongs, the lower one an offset copy, joined in
// every unit by two crossed web pairs aimed at the unit's outer nodes (A.b, B.b).
// Lweb must exceed L/2; the gap between tongs is sqrt(Lweb^2 - L^2/4).
// The top tong's first A bar is grounded.
function doubleTong(n, L, Lweb, { ends = 'ball', ujointAxes, crossingPin = true } = {}) {
  const gap = Math.sqrt(Math.max(Lweb * Lweb - (L * L) / 4, 0));
  const top = lazyTong(n, L, { prefix: 'T' });
  const bot = copy(() => lazyTong(n, L, { prefix: 'T' }), { prefix: 'bot_', translate: [0, 0, -gap] });
  const opts = { Lweb, ends, ujointAxes, crossingPin };
  const webs = [];
  for (let k = 0; k < n; k++) {
    const t = top[k], b = bot[k];
    webs.push(webCross(`W${k}a`, t.A + '.mid', b.A + '.b', b.A + '.mid', t.A + '.b', opts));
    webs.push(webCross(`W${k}b`, t.A + '.mid', b.B + '.b', b.A + '.mid', t.B + '.b', opts));
  }
  ground(top[0].A);
  return { top, bot, webs };
}

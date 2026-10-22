// Rebuild from the CURRENT branch's depot paths, preserving its non-shooting constraints.
const fs = require('node:fs');
const path = require('node:path');
const assert = require('node:assert/strict');
const cp = require('node:child_process');
const root = path.resolve(__dirname, '..');
const source = path.join(root, 'src/main/deploy/autos');
const output = path.join(root, 'exports/short-return-depot/autos');
const names = [
  ['left-center-line-depot.json', 'right-center-line-swipe.json'],
  ['depot-2.json', 'right-short-return-shoot.json'],
  ['depot-3.json', 'right-shoot-exit.json'],
];
const read = file => JSON.parse(fs.readFileSync(file, 'utf8'));
const write = (file, value) => {
  fs.mkdirSync(path.dirname(file), { recursive: true });
  fs.writeFileSync(file, JSON.stringify(value, null, 2) + '\n');
};
const round = n => Number(n.toFixed(5));
function mirror(v) {
  if (Array.isArray(v)) return v.map(mirror);
  if (!v || typeof v !== 'object') return v;
  return Object.fromEntries(Object.entries(v).map(([k, x]) => [k,
    k === 'y_meters' ? round(8.07 - x) : k === 'rotation_radians' ? (x === 0 ? 0 : -x) : mirror(x)]));
}
const originals = names.map(([name]) => read(path.join(source, 'paths', name)));
const result = originals.map(mirror);
const points = p => p.path_elements.filter(e => e.type === 'waypoint');
const returning = points(result[1]);
const exit = points(result[2]);
const end = returning[3].translation_target;
const approach = returning[2].translation_target;
for (const axis of ['x_meters', 'y_meters']) end[axis] = round(approach[axis] + 0.25 * (end[axis] - approach[axis]));
exit[0] = Object.assign(exit[0], structuredClone(returning[3]));
exit[1].translation_target.y_meters = 1.2;
// Hug the low-Y wall through the final trench crossing, then retain the exit turn.
// With the editor's 0.99 m robot width and heading 0, this leaves 0.055 m nominal clearance.
exit[2].translation_target.y_meters = 0.55;
exit[3].translation_target.y_meters = 0.55;
const beforeSpeedChanges = structuredClone(result);
const at = (ranges, ordinal) => ranges?.find(c => ordinal >= c.start_ordinal && ordinal <= c.end_ordinal)?.value;
function slowShooting(p, active) {
  const elements = [];
  const mapping = [];
  const shooting = [];
  let previous;
  let events = [];
  let oldOrdinal = 0;
  for (const element of p.path_elements) {
    if (element.type === 'event_trigger') { events.push(element); continue; }
    assert.equal(element.type, 'waypoint', 'Review newly introduced source element types');
    if (previous) {
      const boundary = events.find(e => ['startShooting', 'stopScoring'].includes(e.lib_key));
      if (boundary) {
        const t = boundary.t_ratio;
        assert.ok(t > 0 && t < 1);
        const a = previous.translation_target;
        const b = element.translation_target;
        elements.push(...events.filter(e => e !== boundary && e.t_ratio < t)
          .sort((a, b) => a.t_ratio - b.t_ratio).map(e => ({ ...e, t_ratio: e.t_ratio / t })));
        elements.push({ type: 'translation', x_meters: a.x_meters + t * (b.x_meters - a.x_meters),
          y_meters: a.y_meters + t * (b.y_meters - a.y_meters), intermediate_handoff_radius_meters: 0.001 });
        mapping.push(oldOrdinal);
        shooting.push(active);
        active = boundary.lib_key === 'startShooting';
        elements.push({ ...boundary, t_ratio: 0 });
        elements.push(...events.filter(e => e !== boundary && e.t_ratio >= t)
          .sort((a, b) => a.t_ratio - b.t_ratio).map(e => ({ ...e, t_ratio: (e.t_ratio - t) / (1 - t) })));
      } else elements.push(...events);
    }
    elements.push(element);
    mapping.push(oldOrdinal++);
    shooting.push(active);
    previous = element;
    events = [];
  }
  const originalConstraints = structuredClone(p.constraints);
  // Translation ranges follow the new translation ordinals. Rotation-only ordinals
  // are unaffected by translation-only splits, and scalar settings stay unchanged.
  for (const [key, ranges] of Object.entries(originalConstraints)) {
    if (!Array.isArray(ranges) || key.includes('deg')) continue;
    p.constraints[key] = mapping.flatMap((old, ordinal) => {
      const value = at(ranges, old);
      if (value === undefined) return [];
      return [{ value: key === 'max_velocity_meters_per_sec' && shooting[ordinal] ? Math.min(value, 0.3) : value,
        start_ordinal: ordinal, end_ordinal: ordinal }];
    });
  }
  // Check every remapped non-shooting value, including the newer acceleration limit.
  for (const [key, ranges] of Object.entries(originalConstraints)) {
    if (!Array.isArray(ranges) || key.includes('deg')) {
      assert.deepEqual(p.constraints[key], ranges);
      continue;
    }
    mapping.forEach((old, ordinal) => {
      const original = at(ranges, old);
      const expected = key === 'max_velocity_meters_per_sec' && shooting[ordinal] && original !== undefined
        ? Math.min(original, 0.3) : original;
      assert.equal(at(p.constraints[key], ordinal), expected, `${key} at ${ordinal}`);
    });
  }
  p.path_elements = elements;
}
slowShooting(result[1], false);
slowShooting(result[2], true);
function eventPositions(p) {
  let previous;
  let pending = [];
  const positions = {};
  for (const e of p.path_elements) {
    if (e.type === 'event_trigger') { pending.push(e); continue; }
    const target = e.translation_target ?? e;
    for (const event of pending) positions[event.lib_key] = ['x_meters', 'y_meters'].map(axis =>
      previous[axis] + event.t_ratio * (target[axis] - previous[axis]));
    pending = [];
    previous = target;
  }
  return positions;
}
assert.deepEqual(result[0], mirror(originals[0]));
assert.deepEqual(points(result[1]).at(-1), points(result[2])[0]);
for (let i = 0; i < result.length; i++) {
  const a = eventPositions(beforeSpeedChanges[i]);
  const b = eventPositions(result[i]);
  assert.deepEqual(Object.keys(a).sort(), Object.keys(b).sort());
  for (const key of Object.keys(a)) a[key].forEach((v, axis) => assert.ok(Math.abs(v - b[key][axis]) < 1e-10));
  const count = result[i].path_elements.filter(e => e.type === 'waypoint' || e.type === 'translation').length;
  for (const [key, ranges] of Object.entries(result[i].constraints)) {
    if (Array.isArray(ranges) && !key.includes('deg')) for (const c of ranges)
      assert.ok(c.start_ordinal >= 0 && c.end_ordinal < count && c.end_ordinal >= c.start_ordinal);
  }
  write(path.join(source, 'paths', names[i][1]), result[i]);
  write(path.join(output, 'paths', names[i][1]), result[i]);
  assert.deepEqual(read(path.join(output, 'paths', names[i][1])), result[i]);
}
write(path.join(output, 'config.json'), read(path.join(source, 'config.json')));
const state = read(path.join(source, '.bline-web/state.json'));
state.editor_config.gui.field = { selected_field_id: 'frc2026-rebuilt', custom_fields: [] };
write(path.join(output, '.bline-web/state.json'), {
  schema_version: 1, editor_config: state.editor_config,
  active_path_file_name: names[0][1], active_path_group_id: 'group-right-short-return',
  path_groups: [{ group_id: 'group-right-short-return', display_name: 'Right Short Return + Slow Shooting', path_file_names: names.map(p => p[1]) }],
  linked_targets: [], paths: Object.fromEntries(names.map((p, i) => [p[1], {
    display_name: ['Right Collection', 'Short Return + Shoot', 'Slow Shooting Exit'][i], editor_metadata: null,
  }])), field_assets: {},
});
const revision = cp.execFileSync('git', ['rev-parse', '--short', 'HEAD'], { cwd: root, encoding: 'utf8' }).trim();
console.log(`Built from ${revision}: verified current-branch constraints, event positions, collection mirror, and return/exit continuity.`);

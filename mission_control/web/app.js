// CERLAB UAV Mission Control page.
// Simulation runs as four steps on the main screen: Map (pick a world: 3D preview, details, a voxel wave as it
// loads) -> Drone (the drone up close with labelled components, LiDAR tilt, flight settings) -> Mission (exploration,
// navigation, inspection or manual keyboard) -> Live (simulator, planner and view running; Start/Pause/Home/Land).
// The left panel mirrors the steps as collapsible sections. The bottom data bar opens into Mission, Topics,
// Processes (logs and terminals) and Storage. Server API: server.py; live data: rosbridge.
'use strict';
const $ = id => document.getElementById(id);
const $$ = (sel, root) => Array.from((root || document).querySelectorAll(sel));
const Q = new URLSearchParams(location.search);
const store = {
  get(k, d) { try { const v = localStorage.getItem('mc.' + k); return v === null ? d : JSON.parse(v); } catch (e) { return d; } },
  set(k, v) { try { localStorage.setItem('mc.' + k, JSON.stringify(v)); } catch (e) { /* private window */ } },
};
const fmtTime = s => { s = Math.max(0, Math.floor(s)); const h = Math.floor(s / 3600), m = Math.floor(s / 60) % 60, x = String(s % 60).padStart(2, '0'); return h ? `${h}:${String(m).padStart(2, '0')}:${x}` : `${m}:${x}`; };
const pct = v => (v === undefined || v === null || v < 0) ? '—' : Math.round(v) + '%';
const nice = s => s.replace(/[_-]+/g, ' ').replace(/\b(\d+)\b/g, ' $1').replace(/\s+/g, ' ').trim();
const esc = s => String(s).replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
const icon = (id, cls) => `<svg${cls ? ` class="${cls}"` : ''}><use href="#${id}"/></svg>`;
const sleep = ms => new Promise(r => setTimeout(r, ms));

// ------------------------------------------------------------------ state
let S = null;                                                     // server state
if (Q.get('tab') === 'history' || store.get('group', 'sim') === 'history') store.set('simStep', 'history');   // History was a tab
const initialTab = (t => (t === 'history' ? 'sim' : t))(Q.get('tab') || store.get('group', 'sim'));
let group = initialTab;
let WORLDS = [], DRONES = [], worldGroups = {}, groupOrder = [];
const pick = {
  world: store.get('world', null), drone: Q.get('drone') || store.get('drone', null),
  pitch: +(Q.get('pitch') || store.get('pitch', 23.5)), mode: store.get('mode', 0),
};
let step = Q.get('step') || store.get('step', 'map');            // map | drone | mission | live
const live = { status: null, pose: null, pathLen: 0, speed: 0, lastPose: null, points: 0 };
let ros = null, rosUrl = null, rosOk = false, subs = [];
let procSel = store.get('proc', null), logTimer = null;
const terms = {};
let settingsFields = [], settingsDir = null;
let launching = null;                                             // text while the launch sequence runs
let wave = null;                                                  // the map step's load wave, while it plays
let trailT = [];                                                  // time of each trail point (s), for the replay
let trailQ = [];                                                  // drone orientation at each trail point
let summary = null;                                               // the last session's numbers, shown after it ends
let replay = null;                                                // flight replay in progress
let lastFlightState = null;
let lastSpin = 0;                                                 // propeller animation clock
let historyMode = false, RUNS = [], histRun = null, histWorld = null;   // History section (step 'history'): past runs and the one on screen

async function api(path, body) {
  const r = await fetch(path, body === undefined ? {} : { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
  return r.json();
}
function toast(text, level) {
  const t = document.createElement('div'); t.className = 'toast ' + (level || ''); t.textContent = text;
  $('toasts').appendChild(t); setTimeout(() => t.remove(), level === 'error' ? 7000 : 3000);
}
const tasks = g => (S ? S.groups[g || group].tasks : []);
const task = (id, g) => tasks(g).find(t => t.id === id);
const running = t => !!t && ['running', 'starting'].includes(t.state);
const simTask = () => task('sim', 'sim');
const simUp = () => running(simTask());
const curWorld = () => (historyMode && histWorld) || WORLDS.find(x => x.file === (simUp() ? simTask().choice.world_file : pick.world));
const curDrone = () => DRONES.find(x => x.file === (simUp() ? simTask().choice.drone_file : pick.drone));
const simPitch = () => (simUp() && simTask().choice.pitch != null ? simTask().choice.pitch : pick.pitch);
// a drone's settings (xacro args, config.json sim.xargs): {name: {label, min, max, default}}
const xarg = (d, k) => (d && d.xargs && d.xargs[k]) || null;
// the 360 camera's lens centre {x, z} in base_link and layout (CAD 360 drone): the URDF defaults, then this drone's pick
function camPick(d) {
  if (!xarg(d, 'insta360_z')) return null;
  const def = { x: (xarg(d, 'insta360_x') || { default: 0 }).default, z: xarg(d, 'insta360_z').default, side: (xarg(d, 'insta360_sideways') || { default: 0 }).default > 0.5 };
  return Object.assign(def, store.get('cam.' + d.file, null) || {});
}
// ... as drawn: the running sim's, else the pick. Left-right puts the lens on the drone's axis, leaned forward the URDF's
// insta360_side_tilt_deg (make_cad_urdf.py); front-back is upright
function simCam(d) {
  d = d || curDrone(); const p = camPick(d); if (!p) return null;
  const xa = simUp() && simTask().choice.xargs;
  const r = xa && xa.insta360_z != null ? { x: xa.insta360_x || 0, z: xa.insta360_z, side: xa.insta360_sideways > 0.5 } : p;
  return Object.assign({}, r, r.side ? { x: 0, tilt: d.cam_side_tilt } : { tilt: 0 });
}
// what the simulator gets for this drone: the picks for the settings it has
function pickXargs(d) {
  const out = {}, c = camPick(d);
  if (xarg(d, 'lidar_pitch_deg')) out.lidar_pitch_deg = pick.pitch;
  if (c) {
    out.insta360_z = c.z;
    if (xarg(d, 'insta360_x')) out.insta360_x = c.x;
    if (xarg(d, 'insta360_sideways')) out.insta360_sideways = c.side ? 1 : 0;
  }
  return out;
}
// the X4's turn: lenses left-right (yaw 90 deg), then tilted about +y (top forward); three.js 'YZX' = R_y(tilt) R_z(yaw)
function camRotation(c) { return new THREE.Euler(0, THREE.MathUtils.degToRad(c.tilt || 0), c.side ? Math.PI / 2 : 0, 'YZX'); }
function placeCam(g, c) { g.position.set(c.x, 0, c.z); g.rotation.copy(camRotation(c)); }
const missionTask = g => task('mission', g || 'sim');
const vinfo = i => (((missionTask() || {}).variant_info) || [])[i] || {};          // mode, about, learned, own_sim, dir, manual
const modeOf = i => vinfo(i).mode || (((missionTask() || {}).variants || [])[i]) || 'Exploration';
const plannerOf = i => (((missionTask() || {}).variants || [])[i]) || '';
const curVariant = () => (running(missionTask()) ? missionTask().variant : pick.mode);
const isManual = () => !!vinfo(curVariant()).manual;
const sessionUp = () => simUp() || running(missionTask());

// ------------------------------------------------------------------ chrome: tabs, panel, resizer, keys, drawer
$$('.tabs button').forEach(b => b.onclick = () => selectGroup(b.dataset.group));
function selectGroup(g) {
  group = g; store.set('group', g);
  $$('.tabs button').forEach(b => b.classList.toggle('on', b.dataset.group === g));
  $$('.pane').forEach(p => p.hidden = p.dataset.pane !== g);
  $('pillDrone').hidden = g !== 'drone';
  if (S) { render(); updateViewTarget(); setStep(g === 'sim' ? store.get('simStep', 'map') : 'live', true); refreshProcList(); }
}
function setLeft(on) { $('shell').classList.toggle('no-left', !on); $('tglLeft').classList.toggle('on', on); store.set('left', on); setTimeout(resize, 30); }
$('tglLeft').onclick = () => setLeft($('shell').classList.contains('no-left'));
{ // drag the panel edge to resize the left side
  const set = (w, first) => { w = Math.max(280, Math.min(640, w)); document.documentElement.style.setProperty('--left', w + 'px'); store.set('leftW', w); if (!first) resize(); };
  set(store.get('leftW', 360), true);
  $('resizer').addEventListener('pointerdown', e => {
    if (e.target.closest('.edge-toggle')) return;
    e.preventDefault(); $('resizer').classList.add('drag'); $('resizer').setPointerCapture(e.pointerId);
    const x0 = e.clientX, w0 = $('left').getBoundingClientRect().width;
    const move = ev => set(w0 + ev.clientX - x0);
    const up = () => { $('resizer').classList.remove('drag'); $('resizer').removeEventListener('pointermove', move); $('resizer').removeEventListener('pointerup', up); };
    $('resizer').addEventListener('pointermove', move); $('resizer').addEventListener('pointerup', up);
  });
}
function setDock(open) { $('drawer').hidden = !open; $('dock').classList.toggle('open', open); store.set('dock', open); if (open) onDrawerTab(); }
$('btnDock').onclick = () => setDock($('drawer').hidden);
$$('#dtabs button').forEach(b => b.onclick = () => { $$('#dtabs button').forEach(x => x.classList.toggle('on', x === b)); store.set('dtab', b.dataset.tab); onDrawerTab(); });
const drawerTab = () => ($$('#dtabs button').find(b => b.classList.contains('on')) || {}).dataset.tab;
function onDrawerTab() {
  const tab = drawerTab();
  $$('.dpane').forEach(p => p.hidden = p.dataset.tab !== tab);
  if (tab === 'topics') loadTopics();
  if (tab === 'procs') refreshProcList();
  if (tab === 'storage') loadBags();
  if (tab !== 'topics') stopEcho();
  fitTerm();
}
{ const t = Q.get('data') || store.get('dtab', 'mission'); $$('#dtabs button').forEach(b => b.classList.toggle('on', b.dataset.tab === t)); }

// left sections fold; clicking a step's head also takes the main screen to that step
for (const sec of $$('.sect.fold')) {
  const k = 'fold.' + sec.dataset.step; sec.classList.toggle('closed', store.get(k, false));
  for (const b of $$('.sect-head, .sect-toggle', sec).filter(b => b.tagName === 'BUTTON')) {
    b.onclick = () => {
      const st = sec.dataset.step;
      if (b.classList.contains('chev-btn') || (st === step && !sec.classList.contains('closed'))) { sec.classList.toggle('closed'); store.set(k, sec.classList.contains('closed')); return; }
      sec.classList.remove('closed'); store.set(k, false);
      if (!(simUp() && st !== 'mission')) setStep(st);
    };
  }
}

// ------------------------------------------------------------------ server state
let lastState = 0;
async function poll() { try { onState(await api('/api/state')); } catch (e) { toast('Mission Control server is not responding', 'error'); } }
function connectEvents() {
  poll();
  const es = new EventSource('/api/events');
  es.onmessage = e => onState(JSON.parse(e.data));
  setInterval(() => { if (Date.now() - lastState > 2500) poll(); }, 1000);
}
const seenNotices = new Set();
function onState(s) {
  const first = !S; S = s; lastState = Date.now();
  for (const n of s.notices) { const k = n.t + n.text; if (!seenNotices.has(k)) { seenNotices.add(k); if (!first) toast(n.text, n.level); } }
  if (first) { selectGroup(initialTab); if (sessionUp() && group === 'sim') setStep('live'); }
  render(); updateViewTarget();
}

// ------------------------------------------------------------------ worlds and drones
async function loadAssets() {
  const [w, d] = await Promise.all([api('/api/worlds'), api('/api/drones')]);
  WORLDS = w.worlds; DRONES = d.drones;
  for (const x of DRONES) if (x.model && x.extra_parts) x.model.extra = x.extra_parts.map(p => p.mesh);
  await Promise.all(DRONES.filter(x => x.model).map(x => loadCad(x.model)).concat(DRONES.filter(x => x.mesh).map(x => loadMesh(x.mesh.url))));
  worldGroups = {};
  for (const x of WORLDS) (worldGroups[x.group] = worldGroups[x.group] || []).push(x);
  groupOrder = Object.keys(worldGroups);
  const def = S && S.groups.sim;
  if (!WORLDS.find(x => x.file === pick.world)) pick.world = (def && def.default_world) || (WORLDS[0] || {}).file;
  if (!DRONES.find(x => x.file === pick.drone)) pick.drone = (def && def.default_drone) || (DRONES[0] || {}).file;
  buildGalleries();
  rebuildScene(); setStep(step);                                   // not quiet: frame the camera on what this step shows
  loadSettings();
}
const worldLabel = g => nice(g);
function variantLabel(w) {
  const g = nice(w.group), n = nice(w.name);
  const short = n.toLowerCase().startsWith(g.toLowerCase()) ? n.slice(g.length).trim() : n;
  return short || 'default';
}
const sizeText = e => e && e[0] ? `${Math.round(e[0])} × ${Math.round(e[1])} × ${Math.round(e[2])} m` : 'no preview';
let worldFilter = store.get('worldFilter', null);
const matchFilter = x => !worldFilter || (worldFilter === 'dynamic' ? x.dynamic > 0 : !x.dynamic);
function matchSearch(x) { const q = $('worldSearch').value.trim().toLowerCase(); return !q || (x.group + ' ' + x.name).replace(/_/g, ' ').toLowerCase().includes(q); }
const shownVariants = g => worldGroups[g].filter(x => matchFilter(x) && matchSearch(x));
function buildGalleries() {
  const wg = $('worldGallery'); wg.innerHTML = '';
  $$('#worldFilter button').forEach(b => b.classList.toggle('on', b.dataset.f === worldFilter));
  for (const g of groupOrder) {
    const list = shownVariants(g); if (!list.length) continue;
    const rep = list.find(x => x.items.length) || list[0];
    const c = document.createElement('button'); c.className = 'card'; c.dataset.group = g;
    c.innerHTML = `<img alt=""><div class="cap"><div class="nm"></div><div class="meta"></div></div>` +
      (list.some(x => x.dynamic) ? `<span class="walk" title="moving obstacles">${icon('i-walk')}</span>` : '');
    c.querySelector('.nm').textContent = worldLabel(g);
    c.querySelector('.meta').textContent = list.length > 1 ? `${sizeText(rep.extent)} · ${list.length} variants` : sizeText(rep.extent);
    thumbLater(c.querySelector('img'), () => worldThumb(rep), 'w:' + rep.file);
    c.onclick = () => { const cur = WORLDS.find(x => x.file === pick.world); if (!cur || cur.group !== g || !list.includes(cur)) chooseWorld((list.find(x => /static/.test(x.name)) || list[0]).file); if (step !== 'map') setStep('map'); };
    wg.appendChild(c);
  }
  const dg = $('droneGallery'); dg.innerHTML = '';
  for (const d of DRONES) {
    const c = document.createElement('button'); c.className = 'card'; c.dataset.file = d.file; c.title = d.note || '';
    c.innerHTML = `<img alt=""><div class="cap"><div class="nm"></div><div class="meta"></div></div>`;
    c.querySelector('.nm').textContent = d.label; c.querySelector('.meta').textContent = d.note || d.file;
    thumbLater(c.querySelector('img'), () => droneThumb(d));
    c.onclick = () => { chooseDrone(d.file); if (step !== 'drone') setStep('drone'); };
    dg.appendChild(c);
  }
  markPicks();
}
function chooseWorld(f) {
  if (simUp()) return;
  pick.world = f; store.set('world', f); markPicks(); rebuildScene(); playWave(); renderInfo(); renderMissionPanel();
}
function chooseDrone(f) {
  if (simUp()) return;
  pick.drone = f; store.set('drone', f); markPicks(); rebuildScene(); renderInfo();
}
function markPicks() {
  const w = curWorld(), d = curDrone(), locked = simUp();
  $$('#worldGallery .card').forEach(c => c.classList.toggle('on', !!w && c.dataset.group === w.group));
  $$('#droneGallery .card').forEach(c => c.classList.toggle('on', !!d && c.dataset.file === d.file));
  $('worldGallery').classList.toggle('locked', locked); $('droneGallery').classList.toggle('locked', locked);
  $('mapSub').textContent = w ? `${nice(w.name)} · ${sizeText(w.extent)}` : '—';
  $('droneSub').textContent = d ? d.label + (hasTilt(d) ? ` · LiDAR ${simPitch()}°` : '') : '—';
  const sk = (w && w.file) + '|' + (d && d.file);                // only when the pick changes: never fight the user's scrolling
  if (sk !== markPicks.last) { markPicks.last = sk; for (const c of $$('#worldGallery .card.on, #droneGallery .card.on')) { const p = c.parentElement; p.scrollTop = Math.max(0, c.offsetTop - p.offsetTop - 8); } }
}
const hasTilt = d => !!xarg(d, 'lidar_pitch_deg');

// thumbnails: one offscreen renderer, one per frame
const thumbQueue = []; let thumbR = null;
const THUMBS = {};                                                // drawn once, reused when the gallery is filtered
function thumbLater(img, make, key) {
  if (key && THUMBS[key]) { img.src = THUMBS[key]; return; }
  const job = key ? () => (THUMBS[key] = make()) : make;
  thumbQueue.push([img, job]); if (thumbQueue.length === 1) requestAnimationFrame(drainThumbs);
}
function drainThumbs() {
  const job = thumbQueue.shift(); if (!job) return;
  try { job[0].src = job[1](); } catch (e) { job[0].replaceWith(Object.assign(document.createElement('div'), { className: 'ph', textContent: 'no preview' })); }
  if (thumbQueue.length) requestAnimationFrame(drainThumbs);
}
function snapshot(object, w, h, dir) {
  if (!thumbR) { thumbR = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true }); thumbR.setPixelRatio(1); }
  thumbR.setSize(w, h, false); thumbR.setClearColor(0x0e0f11);
  const sc = new THREE.Scene(); sc.add(object); sc.add(new THREE.HemisphereLight(0xffffff, 0x222428, 0.95));
  const dl = new THREE.DirectionalLight(0xffffff, 0.7); dl.position.set(-0.5, -1, 1.4); sc.add(dl);
  const box = new THREE.Box3().setFromObject(object), c = box.getCenter(new THREE.Vector3()), s = box.getSize(new THREE.Vector3());
  const cam = new THREE.PerspectiveCamera(30, w / h, 0.01, 5000); cam.up.set(0, 0, 1);
  cam.position.copy(c).add(new THREE.Vector3(...(dir || [-0.75, -1.05, 0.85])).normalize().multiplyScalar(Math.max(s.x, s.y, s.z * 1.4, 0.3) * 1.9)); cam.lookAt(c);
  thumbR.render(sc, cam);
  return thumbR.domElement.toDataURL('image/jpeg', 0.86);
}
const worldThumb = w => { if (!w.items.length) throw new Error('empty'); return snapshot(worldMesh(w, 'solid'), 320, 200); };
const droneThumb = d => snapshot(droneModel(d, (xarg(d, 'lidar_pitch_deg') || { default: 0 }).default).root, 220, 165, d.model ? [-1, -1.1, 0.55] : [-1, -1.25, 0.8]);

// ------------------------------------------------------------------ 3D models
const MAT = {
  wall: new THREE.MeshLambertMaterial({ color: 0x9aa0a8 }),
  ghost: new THREE.MeshBasicMaterial({ color: 0x8a93a3, transparent: true, opacity: 0.05, depthWrite: false, side: THREE.DoubleSide }),
  edge: new THREE.LineBasicMaterial({ color: 0xc8d2e0, transparent: true, opacity: 0.22 }),
  moving: new THREE.MeshLambertMaterial({ color: 0xff9f43 }),
  movingGhost: new THREE.MeshBasicMaterial({ color: 0xff9f43, transparent: true, opacity: 0.4 }),
};
function itemGeometry(it) {
  let g;
  if (it.t === 'box') g = new THREE.BoxGeometry(...it.s);
  else if (it.t === 'cyl') { g = new THREE.CylinderGeometry(it.s[0], it.s[0], it.s[1], 20); g.rotateX(Math.PI / 2); }
  else g = new THREE.SphereGeometry(it.s[0], 16, 12);
  g.applyMatrix4(new THREE.Matrix4().fromArray(it.m));
  return g;
}
function worldMesh(w, mode) {
  const grp = new THREE.Group();
  for (const it of w.items) {
    const g = itemGeometry(it);
    if (it.d) { grp.add(new THREE.Mesh(g, mode === 'solid' ? MAT.moving : MAT.movingGhost)); continue; }
    if (mode === 'solid') { grp.add(new THREE.Mesh(g, MAT.wall)); continue; }
    grp.add(new THREE.Mesh(g, MAT.ghost)); grp.add(new THREE.LineSegments(new THREE.EdgesGeometry(g, 30), MAT.edge));
  }
  return grp;
}
// points on the world's surfaces (for the load wave): area-weighted samples on every box face
function surfacePoints(w, n) {
  const faces = [];
  for (const it of w.items) {
    if (it.t !== 'box') continue;
    const M = new THREE.Matrix4().fromArray(it.m), [a, b, c] = it.s;
    for (const [ax, sz, u, v] of [[0, a, b, c], [1, b, a, c], [2, c, a, b]]) for (const sgn of [-1, 1]) faces.push({ M, ax, half: sz / 2 * sgn, u, v, it, area: u * v });
  }
  const total = faces.reduce((s, f) => s + f.area, 0) || 1, out = [];
  for (const f of faces) {
    const k = Math.round(n * f.area / total);
    for (let i = 0; i < k; i++) {
      const p = [0, 0, 0], du = (Math.random() - 0.5) * f.u, dv = (Math.random() - 0.5) * f.v;
      p[f.ax] = f.half; p[(f.ax + 1) % 3] = f.ax === 1 ? dv : du; p[(f.ax + 2) % 3] = f.ax === 1 ? du : dv;
      if (f.ax === 0) { p[1] = du; p[2] = dv; } else if (f.ax === 1) { p[0] = du; p[2] = dv; } else { p[0] = du; p[1] = dv; }
      out.push(new THREE.Vector3(...p).applyMatrix4(f.M));
    }
  }
  return out;
}
// Drones from CAD: one STL per group, tilting groups on the hinge
const CAD = {};
const CAD_MAT = {
  frame: new THREE.MeshStandardMaterial({ color: 0x3b3f46, metalness: 0.35, roughness: 0.55 }),
  blades: new THREE.MeshStandardMaterial({ color: 0x5a616d, metalness: 0.35, roughness: 0.4, side: THREE.DoubleSide }),
  blur: new THREE.MeshBasicMaterial({ map: (() => { const c = document.createElement('canvas'); c.width = c.height = 128; const x = c.getContext('2d');
    const g = x.createRadialGradient(64, 64, 6, 64, 64, 64); g.addColorStop(0, 'rgba(40,44,52,0.85)'); g.addColorStop(0.18, 'rgba(60,66,78,0.35)');
    g.addColorStop(0.85, 'rgba(120,140,170,0.22)'); g.addColorStop(1, 'rgba(120,140,170,0)'); x.fillStyle = g; x.fillRect(0, 0, 128, 128); return new THREE.CanvasTexture(c); })(),
    transparent: true, depthWrite: false, side: THREE.DoubleSide }),
  guards: new THREE.MeshStandardMaterial({ color: 0x55595f, metalness: 0.3, roughness: 0.6, side: THREE.DoubleSide }),
  camera: new THREE.MeshStandardMaterial({ color: 0x15171a, metalness: 0.2, roughness: 0.4 }),
  mount: new THREE.MeshStandardMaterial({ color: 0x9b2226, metalness: 0.2, roughness: 0.5 }),
  lidar_bracket: new THREE.MeshStandardMaterial({ color: 0x9b2226, metalness: 0.2, roughness: 0.5 }),
  lidar_adapter: new THREE.MeshStandardMaterial({ color: 0x2c3036, metalness: 0.3, roughness: 0.5 }),
  lidar_links: new THREE.MeshStandardMaterial({ color: 0x9b2226, metalness: 0.2, roughness: 0.5 }),
  lidar: new THREE.MeshStandardMaterial({ color: 0xe6e9ee, metalness: 0.1, roughness: 0.35 }),
  insta360_x4: new THREE.MeshStandardMaterial({ color: 0x1b1d21, metalness: 0.25, roughness: 0.35 }),
};
// drones drawn from one mesh (the original CERLAB quadcopter)
const MESHES = {};
// load url into cache[key] once (null while loading or when it fails)
function loadStl(cache, key, url) {
  if (key in cache) return Promise.resolve();
  cache[key] = null;
  return new Promise(res => new THREE.STLLoader().load(url, geo => { geo.computeVertexNormals(); cache[key] = geo; res(); }, undefined, () => res()));
}
const loadMesh = url => loadStl(MESHES, url, url);
// baked colours (tools/bake_mesh_colors.py: the mesh's texture, one colour a face), else a plain red
const MESH_MAT = { colors: new THREE.MeshStandardMaterial({ vertexColors: true, metalness: 0.2, roughness: 0.55 }),
  plain: new THREE.MeshStandardMaterial({ color: 0x9b2226, metalness: 0.25, roughness: 0.5 }) };
function loadCad(m) {
  const have = CAD[m.dir] = CAD[m.dir] || {};                       // drones from one CAD model share its meshes
  return Promise.all(Object.keys(m.groups).concat(m.extra || []).map(g => loadStl(have, g, `/models/${m.dir}/meshes/${g}.stl`)));
}
// {root, hinge}: hinge is the group that turns with the LiDAR tilt (null for drones without one)
function droneModel(d, pitch) {
  if (d.model) {
    const m = d.model, root = new THREE.Group(), hinge = new THREE.Group();
    hinge.position.set(...m.hinge_xyz); hinge.rotation.y = THREE.MathUtils.degToRad(pitch);
    const rotors = [];
    for (const [g, gi] of Object.entries(m.groups)) {
      const geo = (CAD[m.dir] || {})[g]; if (!geo) continue;
      if (gi.rotor) {                                              // a propeller: spins about its hub; a blur disc in flight
        const hub = new THREE.Group(); hub.position.set(...gi.origin);
        const blades = new THREE.Mesh(geo, CAD_MAT.blades); hub.add(blades);
        const r = Math.hypot(gi.bounds[1][0] - gi.bounds[0][0], gi.bounds[1][1] - gi.bounds[0][1]) / 2.6;
        const blur = new THREE.Mesh(new THREE.CircleGeometry(r, 48), CAD_MAT.blur); blur.visible = false; hub.add(blur);
        root.add(hub); rotors.push({ hub, blades, blur }); continue;
      }
      (gi.moving ? hinge : root).add(new THREE.Mesh(geo, CAD_MAT[g] || CAD_MAT.frame));
    }
    root.add(hinge);
    let cam = null;                                                // the 360 camera: a fixed part placed by the mount choice, level
    for (const p of d.extra_parts || []) {
      const geo = (CAD[m.dir] || {})[p.mesh]; if (!geo) continue;
      const g = new THREE.Group(), c = p.edit === 'mount' ? simCam(d) : null;
      if (c) placeCam(g, c); else g.position.set(...p.xyz);
      g.add(new THREE.Mesh(geo, CAD_MAT[p.mesh] || CAD_MAT.frame)); root.add(g);
      if (p.edit === 'mount') cam = g;
    }
    return { root, hinge, rotors, cam };
  }
  // stand-in drawn from the URDF's size and sensors
  const g = new THREE.Group(), span = d.span || 0.5, arm = span / 2 * 0.82;
  const dark = new THREE.MeshLambertMaterial({ color: 0x2c2f35 }), light = new THREE.MeshLambertMaterial({ color: 0xe8ebf0 });
  const geo = d.mesh && MESHES[d.mesh.url];
  if (geo) {                                                       // the drone's own mesh, as Gazebo draws it
    const m = new THREE.Mesh(geo, geo.hasColors ? MESH_MAT.colors : MESH_MAT.plain); m.scale.setScalar(d.mesh.scale); m.position.set(...d.mesh.xyz); g.add(m);
  } else {                                                         // a box, four arms and rotor discs
    const body = new THREE.Mesh(new THREE.BoxGeometry(span * 0.36, span * 0.28, span * 0.12), dark); body.position.z = span * 0.12; g.add(body);
    const prop = new THREE.MeshBasicMaterial({ color: 0x9fc4ff, transparent: true, opacity: 0.3, side: THREE.DoubleSide, depthWrite: false });
    for (const a of [45, 135, 225, 315]) {
      const r = THREE.MathUtils.degToRad(a), x = Math.cos(r) * arm, y = Math.sin(r) * arm;
      const am = new THREE.Mesh(new THREE.BoxGeometry(arm, span * 0.04, span * 0.03), dark); am.position.set(x / 2, y / 2, span * 0.14); am.rotation.z = r; g.add(am);
      const disc = new THREE.Mesh(new THREE.CircleGeometry(span * 0.2, 32), prop); disc.position.set(x, y, span * 0.2); g.add(disc);
    }
  }
  let hinge = null;
  for (const s of d.sensors || []) {
    const h = new THREE.Group(); h.position.set(...s.xyz);
    h.rotation.set(s.rpy[0], s.kind === 'lidar' ? THREE.MathUtils.degToRad(pitch) : s.rpy[1], s.rpy[2], 'ZYX');
    if (s.kind === 'lidar') { const m = new THREE.Mesh(new THREE.CylinderGeometry(0.034, 0.034, 0.06, 24), light); m.rotation.x = Math.PI / 2; h.add(m); hinge = h; }
    else if (s.kind === 'fisheye camera') { const m = new THREE.Mesh(new THREE.SphereGeometry(0.022, 16, 12, 0, Math.PI), new THREE.MeshLambertMaterial({ color: 0x111316 })); m.rotation.z = -Math.PI / 2; h.add(m); }
    else if (s.kind === 'depth camera') h.add(new THREE.Mesh(new THREE.BoxGeometry(0.025, 0.09, 0.025), dark));
    else if (s.kind.startsWith('360 depth')) h.add(new THREE.Mesh(new THREE.SphereGeometry(0.03, 16, 12), new THREE.MeshLambertMaterial({ color: 0x7db2ff })));
    g.add(h);
  }
  return { root: g, hinge, standIn: true };
}
// labelled parts: from the CAD model, or the stand-in's sensors
const SENSOR_LABEL = /mid-360|lidar|realsense|depth camera|360 camera/i;
function components(d) {
  if (d.model && d.model.components) return d.model.components.filter(c => c.edit === 'tilt' || c.edit === 'mount' || SENSOR_LABEL.test(c.label));
  const names = { lidar: ['Tilted LiDAR', 'Mid-360 (simulated)'], 'fisheye camera': ['360 camera', 'fisheye lens'], 'depth camera': ['Depth camera', 'forward RGB-D'], '360 depth (ideal)': ['Ideal 360 depth', 'research sensor'] };
  const out = [], seen = new Set();
  for (const s of d.sensors) { if (!['lidar', 'fisheye camera', 'depth camera'].includes(s.kind)) continue; const n = names[s.kind] || [s.kind, '']; if (seen.has(n[0])) continue; seen.add(n[0]); out.push({ label: n[0], detail: n[1], xyz: s.kind === 'lidar' ? [0, 0, 0.03] : s.xyz, moving: s.kind === 'lidar', edit: s.kind === 'lidar' ? 'tilt' : null }); }
  return out;
}
// Mid-360 coverage: elevation -7..+52 deg about the sensor axis, as a translucent shell
function lidarCoverage(xyz, radius) {
  const geo = new THREE.SphereGeometry(radius, 64, 10, 0, Math.PI * 2, THREE.MathUtils.degToRad(38), THREE.MathUtils.degToRad(59)); geo.rotateX(Math.PI / 2);
  const m = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ color: 0x3e8bff, transparent: true, opacity: 0.06, side: THREE.DoubleSide, depthWrite: false }));
  const lines = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.SphereGeometry(radius, 24, 2, 0, Math.PI * 2, THREE.MathUtils.degToRad(38), THREE.MathUtils.degToRad(59)).rotateX(Math.PI / 2), 1),
    new THREE.LineBasicMaterial({ color: 0x3e8bff, transparent: true, opacity: 0.22 }));
  const h = new THREE.Group(); h.position.set(...xyz); h.add(m); h.add(lines); return h;
}

// ------------------------------------------------------------------ main 3D view
const view = $('view');
const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1)); renderer.setClearColor(0x000000, 0);
view.appendChild(renderer.domElement);
const scene = new THREE.Scene(); scene.fog = new THREE.FogExp2(0x0a0b0d, 0.012);
const camera = new THREE.PerspectiveCamera(50, 1, 0.02, 3000); camera.up.set(0, 0, 1); camera.position.set(-20, -26, 18);
const controls = new THREE.OrbitControls(camera, renderer.domElement); controls.enableDamping = true; controls.dampingFactor = 0.08; controls.autoRotateSpeed = 0.8;
controls.mouseButtons = { LEFT: THREE.MOUSE.ROTATE, MIDDLE: THREE.MOUSE.PAN, RIGHT: THREE.MOUSE.PAN };   // wheel zooms, middle drags
controls.addEventListener('start', () => { controls.autoRotate = false; tween = null; });
scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x101216, 0.85));
const sun = new THREE.DirectionalLight(0xffffff, 0.65); sun.position.set(-30, -40, 60); scene.add(sun);
const grid = new THREE.GridHelper(400, 200, 0x1f2329, 0x15181c); grid.rotation.x = Math.PI / 2; grid.material.transparent = true; grid.material.opacity = 0.6; scene.add(grid);
const dotTex = (() => { const c = document.createElement('canvas'); c.width = c.height = 32; const x = c.getContext('2d'); const g = x.createRadialGradient(16, 16, 0, 16, 16, 16); g.addColorStop(0, 'rgba(255,255,255,1)'); g.addColorStop(0.55, 'rgba(255,255,255,0.9)'); g.addColorStop(1, 'rgba(255,255,255,0)'); x.fillStyle = g; x.fillRect(0, 0, 32, 32); return new THREE.CanvasTexture(c); })();

const layers = { world: new THREE.Group(), fov: new THREE.Group() };
scene.add(layers.world);
const mapGeo = new THREE.BufferGeometry(), mapMat = new THREE.PointsMaterial({ size: 0.22, vertexColors: true, map: dotTex, transparent: true, alphaTest: 0.2 });
layers.map = new THREE.Points(mapGeo, mapMat); scene.add(layers.map);
const realGeo = new THREE.BufferGeometry(), realMat = new THREE.PointsMaterial({ size: 0.2, vertexColors: true, map: dotTex, transparent: true, alphaTest: 0.2 });
layers.reality = new THREE.Points(realGeo, realMat); scene.add(layers.reality);
const trailMax = 20000, trailPos = new Float32Array(trailMax * 3); let trailN = 0;
const trailGeo = new THREE.BufferGeometry(); trailGeo.setAttribute('position', new THREE.BufferAttribute(trailPos, 3)); trailGeo.setDrawRange(0, 0);
layers.trail = new THREE.Line(trailGeo, new THREE.LineBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.55 })); scene.add(layers.trail);
const pathGeo = new THREE.BufferGeometry(); layers.path = new THREE.Line(pathGeo, new THREE.LineBasicMaterial({ color: 0x3e8bff })); scene.add(layers.path);
const drone = new THREE.Group(); scene.add(drone);
const halo = new THREE.Mesh(new THREE.RingGeometry(0.55, 0.68, 48), new THREE.MeshBasicMaterial({ color: 0x3e8bff, transparent: true, opacity: 0.5, side: THREE.DoubleSide, depthWrite: false }));
drone.add(halo);
const home = new THREE.Mesh(new THREE.RingGeometry(0.6, 0.72, 48), new THREE.MeshBasicMaterial({ color: 0xffd60a, side: THREE.DoubleSide, transparent: true, opacity: 0.8 }));
home.visible = false; scene.add(home);
let droneParts = null;                                            // {root, hinge} of the drone on screen
// the drone step's stage: a soft shadow under the drone and a blue glow around it, on the ground plane
const pad = new THREE.Group(); pad.visible = false; scene.add(pad);
{
  const tex = (stops) => { const c = document.createElement('canvas'); c.width = c.height = 256; const x = c.getContext('2d');
    const g = x.createRadialGradient(128, 128, 0, 128, 128, 128); stops.forEach(([o, col]) => g.addColorStop(o, col)); x.fillStyle = g; x.fillRect(0, 0, 256, 256); return new THREE.CanvasTexture(c); };
  const glow = new THREE.Mesh(new THREE.CircleGeometry(0.9, 64), new THREE.MeshBasicMaterial({ map: tex([[0, 'rgba(62,139,255,0.42)'], [0.45, 'rgba(62,139,255,0.16)'], [1, 'rgba(62,139,255,0)']]), transparent: true, depthWrite: false }));
  const shadow = new THREE.Mesh(new THREE.CircleGeometry(0.42, 64), new THREE.MeshBasicMaterial({ map: tex([[0, 'rgba(0,0,0,0.85)'], [0.6, 'rgba(0,0,0,0.45)'], [1, 'rgba(0,0,0,0)']]), transparent: true, depthWrite: false }));
  glow.position.z = 0.001; shadow.position.z = 0.002; pad.add(glow); pad.add(shadow);
}
// the CAD bounds grown by the extra parts (the 360 camera) where they sit now
function droneBounds(d) {
  if (d && !d.model && d.mesh) return d.mesh.bounds;
  if (!d || !d.model) return null;
  const lo = [...d.model.bounds[0]], hi = [...d.model.bounds[1]], c = simCam(d);
  for (const p of d.extra_parts || []) {
    const m = new THREE.Matrix4();
    if (p.edit === 'mount' && c) m.compose(new THREE.Vector3(c.x, 0, c.z), new THREE.Quaternion().setFromEuler(camRotation(c)), new THREE.Vector3(1, 1, 1));
    else m.makeTranslation(...p.xyz);
    const b = new THREE.Box3(new THREE.Vector3(...p.bounds[0]), new THREE.Vector3(...p.bounds[1])).applyMatrix4(m);
    for (let i = 0; i < 3; i++) { lo[i] = Math.min(lo[i], b.min.getComponent(i)); hi[i] = Math.max(hi[i], b.max.getComponent(i)); }
  }
  return [lo, hi];
}
// height of the drone's centre when its feet are on the ground
function footZ() { const d = curDrone(), b = droneBounds(d); return b ? -b[0][2] : (d ? 0.02 : 0); }
// the drone step's view: aimed at the middle of the drone, backed off for taller drones
function frameDrone(dur) {
  const b = droneBounds(curDrone()), z = footZ(), mid = b ? z + (b[0][2] + b[1][2]) / 2 : z;
  const k = b ? Math.max(1, Math.max(b[1][0] - b[0][0], b[1][1] - b[0][1], (b[1][2] - b[0][2]) * 1.4) / 0.52) : 1;
  flyTo(new THREE.Vector3(0, 0, mid), new THREE.Vector3(-0.66 * k, -0.82 * k, mid + 0.36 * k), dur);
}

for (const cb of $$('#layers input')) {
  const k = cb.dataset.layer; cb.checked = store.get('layer.' + k, cb.checked);
  const apply = () => { const o = layers[k]; if (o) o.visible = cb.checked; store.set('layer.' + k, cb.checked); };
  cb.onchange = apply; apply();
}
$('btnLayers').onclick = () => { $('layers').hidden = !$('layers').hidden; $('btnLayers').classList.toggle('on', !$('layers').hidden); };
$('btnReset').onclick = () => {
  follow = false; store.set('follow', false); $('btnFollow').classList.remove('on');
  const w = curWorld();
  if (step === 'live' && live.pose) { const p = drone.position; flyTo(p.clone(), new THREE.Vector3(p.x - 3.2, p.y - 3.6, p.z + 2.2), 0.6); }
  else frameBox(w && w.items.length ? new THREE.Box3().setFromObject(layers.world) : new THREE.Box3(new THREE.Vector3(-6, -6, 0), new THREE.Vector3(6, 6, 3)), 0.6);
};
let follow = store.get('follow', false) || Q.get('view') === 'drone';
$('btnFollow').classList.toggle('on', follow);
$('btnFollow').onclick = () => { follow = !follow; store.set('follow', follow); $('btnFollow').classList.toggle('on', follow); if (follow) closeUp(); };
let colorMode = store.get('colorMode', 'mono'); if (colorMode === 'camera') colorMode = 'mono';
$$('#colorMode button').forEach(b => { b.classList.toggle('on', b.dataset.mode === colorMode); b.onclick = () => { colorMode = b.dataset.mode; store.set('colorMode', colorMode); $$('#colorMode button').forEach(x => x.classList.toggle('on', x === b)); redrawMap(); }; });

function resize() { const w = view.clientWidth, h = view.clientHeight; renderer.setSize(w, h, false); renderer.domElement.style.width = w + 'px'; renderer.domElement.style.height = h + 'px'; camera.aspect = w / Math.max(h, 1); camera.updateProjectionMatrix(); fitTerm(); }
new ResizeObserver(resize).observe(view);

// camera moves: ease from where it is to (target, position) over dur seconds
let tween = null;
function flyTo(target, pos, dur) { tween = { t0: performance.now(), dur: (dur || 0.8) * 1000, a: [controls.target.clone(), camera.position.clone()], b: [target.clone(), pos.clone()] }; }
function frameBox(box, dur, dir) {
  const c = box.getCenter(new THREE.Vector3()), s = box.getSize(new THREE.Vector3()), r = Math.max(s.x, s.y, s.z, 6);
  scene.fog.density = 0.9 / Math.max(r * 4, 40);
  flyTo(c, c.clone().add(new THREE.Vector3(...(dir || [-0.62, -0.95, 0.72])).normalize().multiplyScalar(r * 1.3)), dur);
}
function closeUp() { const p = drone.position; flyTo(p.clone(), new THREE.Vector3(p.x - 1.0, p.y - 1.1, p.z + 0.55), 0.6); }

(function loop(now) {
  requestAnimationFrame(loop);
  if (tween) {
    const k = Math.min(1, (now - tween.t0) / tween.dur), e = k < 0.5 ? 4 * k * k * k : 1 - Math.pow(-2 * k + 2, 3) / 2;
    controls.target.lerpVectors(tween.a[0], tween.b[0], e); camera.position.lerpVectors(tween.a[1], tween.b[1], e);
    if (k >= 1) tween = null;
  } else if (follow && live.pose && step === 'live') { const d = drone.position.clone().sub(controls.target); controls.target.add(d); camera.position.add(d); }
  halo.material.opacity = 0.3 + 0.2 * Math.sin((now || 0) / 500);
  if (wave) stepWave(now);
  spinRotors(now);
  if (replay) stepReplay(now);
  controls.update(); renderer.render(scene, camera);
  if (step === 'drone') drawAnnotations();
})();

// propellers: still on the ground; spinning under a blur disc while the drone flies

function spinRotors(now) {
  const dt = Math.min(0.05, (now - lastSpin) / 1000 || 0); lastSpin = now;
  if (!droneParts || !droneParts.rotors) return;
  const st = live.status && live.status.state;
  const flying = (step === 'live' && live.pose && (live.pose.z - (curWorld() ? curWorld().start[2] : 0) > 0.15 || ['TAKEOFF', 'ENTRY', 'EXPLORING', 'INSPECTING', 'RETURNING', 'LANDING', 'PAUSED'].includes(st))) || (replay && replay.flying);
  droneParts.rotors.forEach((r, i) => {
    r.blur.visible = !!flying;
    if (flying) r.hub.rotation.z += (i % 2 ? -1 : 1) * 60 * dt;
  });
}

// ------------------------------------------------------------------ the load wave (map step)
// Blue voxels sweep across the new world in ~0.5 s and fade as its wireframe appears.
const waveMat = new THREE.ShaderMaterial({
  uniforms: { t: { value: 0 }, size: { value: 6 } }, transparent: true, depthWrite: false, blending: THREE.AdditiveBlending,
  vertexShader: `attribute float d; uniform float t; uniform float size; varying float a;
    void main() { float k = t - d; a = smoothstep(0.0, 0.06, k) * (1.0 - smoothstep(0.08, 0.45, k));
      vec4 mv = modelViewMatrix * vec4(position, 1.0); gl_PointSize = size * (1.0 + 1.5 * a) * (60.0 / -mv.z); gl_Position = projectionMatrix * mv; }`,
  fragmentShader: `varying float a; void main() { vec2 p = gl_PointCoord - 0.5; if (max(abs(p.x), abs(p.y)) > 0.42) discard;
      gl_FragColor = vec4(0.35, 0.65, 1.0, a * 0.9); }`,
});
function playWave() {
  const w = curWorld(); if (!w || !w.items.length || step !== 'map') return;
  if (wave) scene.remove(wave.pts);
  const pts = surfacePoints(w, 14000); if (!pts.length) return;
  const box = new THREE.Box3().setFromPoints(pts), sz = box.getSize(new THREE.Vector3()), axis = sz.x >= sz.y ? 'x' : 'y';
  const pos = new Float32Array(pts.length * 3), d = new Float32Array(pts.length);
  pts.forEach((p, i) => { pos.set([p.x, p.y, p.z], i * 3); d[i] = 0.5 * (p[axis] - box.min[axis]) / Math.max(sz[axis], 1e-3) + Math.random() * 0.04; });
  const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.BufferAttribute(pos, 3)); g.setAttribute('d', new THREE.BufferAttribute(d, 1));
  const mat = waveMat.clone(); mat.uniforms.size.value = Math.max(2.5, Math.min(7, 0.012 * Math.max(sz.x, sz.y) * 6)) / 3;
  wave = { pts: new THREE.Points(g, mat), t0: performance.now() };
  scene.add(wave.pts);
  MAT.edge.opacity = 0; MAT.ghost.opacity = 0;
}
function stepWave(now) {
  const t = (now - wave.t0) / 1000;
  wave.pts.material.uniforms.t.value = t;
  const k = Math.min(1, Math.max(0, (t - 0.15) / 0.6));                // wireframe fades in behind the wave
  MAT.edge.opacity = 0.22 * k; MAT.ghost.opacity = 0.05 * k;
  if (t > 1.1) { scene.remove(wave.pts); wave.pts.geometry.dispose(); wave = null; MAT.edge.opacity = 0.22; MAT.ghost.opacity = 0.05; }
}

// ------------------------------------------------------------------ scene contents per step
let sceneKey = null;
function rebuildScene() {
  const w = group === 'sim' ? curWorld() : null;
  const d = (group === 'sim' ? curDrone() : null) || DRONES.find(x => x.file === (S && S.groups.sim.default_drone)) || DRONES[0];
  const key = (w && w.file) + '|' + (d && d.file);
  if (key !== sceneKey) {
    sceneKey = key;
    layers.world.clear(); if (w) layers.world.add(worldMesh(w, 'ghost'));
    drone.children.filter(c => c !== halo).forEach(c => drone.remove(c)); layers.fov.clear(); droneParts = null;
    if (d) {
      droneParts = droneModel(d, simPitch()); drone.add(droneParts.root); drone.add(layers.fov);
      const lid = d.model ? { xyz: d.model.lidar_from_hinge } : null;
      if (droneParts.hinge) { const cov = lidarCoverage(lid ? lid.xyz : [0, 0, 0], 2.2); layers.fov.add(cov); }
    }
  }
  applyTilt();
  placeDrone();
}
// the LiDAR tilt is a rotation of the hinge group (and its coverage shell), not a rebuild
function applyTilt() {
  if (!droneParts || !droneParts.hinge) return;
  const r = THREE.MathUtils.degToRad(simPitch()), d = curDrone();
  if (droneParts.standIn) droneParts.hinge.rotation.y = r; else droneParts.hinge.rotation.y = r;
  layers.fov.position.copy(droneParts.hinge.position); layers.fov.rotation.copy(droneParts.hinge.rotation);
  if (droneParts.standIn) layers.fov.children.forEach(c => c.position.set(0, 0, 0));
  markPicks();
}
function placeDrone() {
  if ((live.pose && step === 'live') || replay) return;
  if (step === 'history') { pad.visible = false; halo.visible = true; layers.world.visible = true; if (trailN) drone.position.set(trailPos[0], trailPos[1], trailPos[2]); return; }
  const w = group === 'sim' ? curWorld() : null;
  pad.visible = step === 'drone';
  if (step === 'drone') { drone.position.set(0, 0, footZ()); drone.rotation.set(0, 0, 0); layers.world.visible = false; halo.visible = false; return; }
  const st = w ? w.start : [0, 0, 0.1]; drone.position.set(st[0], st[1], st[2]); drone.rotation.set(0, 0, 0);
  layers.world.visible = step !== 'live' || $$('#layers input').find(c => c.dataset.layer === 'world').checked; halo.visible = true;
}

// ------------------------------------------------------------------ steps
const STEPS = ['map', 'drone', 'mission', 'live'];
function setStep(s, quiet) {
  if (group === 'drone') s = 'live';
  else store.set('simStep', s);
  if (s !== 'live' && sessionUp() && group === 'sim') s = 'live';
  const changed = s !== step;
  if (s === 'history' && !historyMode) { historyMode = true; replay = null; loadRuns(); }
  else if (s !== 'history' && historyMode) {                     // leaving History: back to the picked world, no replay
    historyMode = false; histRun = null; histWorld = null; replay = null; trailN = 0; trailGeo.setDrawRange(0, 0); sceneKey = null; rebuildScene();
  }
  if (changed && step === 'live' && running(task('record_mission', 'sim'))) { api('/api/task/sim/record_mission/stop', {}); toast('Recording stopped'); }
  step = s; store.set('step', s);
  $('stage').classList.toggle('setup', s !== 'live');
  $$('#steps button').forEach(b => { b.classList.toggle('on', b.dataset.step === s); b.classList.toggle('done', STEPS.indexOf(b.dataset.step) < STEPS.indexOf(s)); });
  $('steps').hidden = group !== 'sim' || s === 'history';
  $$('.sect.fold').forEach(sec => {                              // the current step's section opens, the others fold
    const cur = sec.dataset.step === (s === 'live' ? 'mission' : s);
    const i = STEPS.indexOf(sec.dataset.step), j = STEPS.indexOf(s === 'live' ? 'mission' : s);   // History is outside the steps
    sec.classList.toggle('current', cur); sec.classList.toggle('done', i >= 0 && j >= 0 && i < j);
    if (changed || !quiet) { sec.classList.toggle('closed', !cur); store.set('fold.' + sec.dataset.step, !cur); }
  });
  $('prevBtn').hidden = $('nextBtn').hidden = !(s === 'map' || s === 'drone');
  $('missionPanel').hidden = s !== 'mission';
  $('summary').hidden = !(s === 'mission' && summary);
  $('annot').innerHTML = ''; $('annotLabels').innerHTML = ''; $('annotLabels').hidden = s !== 'drone';
  controls.autoRotate = s === 'drone';
  placeDrone();
  if (changed || !quiet) {
    if (s === 'drone') { frameDrone(0.7); grid.visible = true; }
    else if (s === 'map' || s === 'mission') { const w = curWorld(); frameBox(w && w.items.length ? new THREE.Box3().setFromObject(layers.world) : new THREE.Box3(new THREE.Vector3(-6, -6, 0), new THREE.Vector3(6, 6, 3)), 0.7); if (s === 'map' && changed) playWave(); }
    else if (s === 'live' && changed && !live.pose) { const w = curWorld(); if (w && w.items.length) frameBox(new THREE.Box3().setFromObject(layers.world), 0.7); }
  }
  renderInfo(); renderCta(); renderMissionPanel();
}
$$('#steps button').forEach(b => b.onclick = () => { if (b.dataset.step === 'live' && !sessionUp()) return launch(); setStep(b.dataset.step); });
$('prevBtn').onclick = () => cycle(-1);
$('nextBtn').onclick = () => cycle(1);
function cycle(dir) {
  if (step === 'map') {
    const w = curWorld(), i = Math.max(0, groupOrder.indexOf(w ? w.group : groupOrder[0]));
    const g = groupOrder[(i + dir + groupOrder.length) % groupOrder.length], list = worldGroups[g];
    chooseWorld((list.find(x => /static/.test(x.name)) || list[0]).file);
    setStep('map');
  } else if (step === 'drone') {
    const i = Math.max(0, DRONES.findIndex(x => x.file === pick.drone));
    chooseDrone(DRONES[(i + dir + DRONES.length) % DRONES.length].file); controls.autoRotate = true;
  }
}
document.addEventListener('keydown', e => {
  if (e.target.closest('input, select, textarea, .xterm')) return;
  if (manualKey(e, true)) return;
  if (e.key === '[') $('tglLeft').click();
  if (e.key === 'l' || e.key === 'L') $('btnDock').click();
  if (e.key === 'ArrowLeft' && (step === 'map' || step === 'drone')) cycle(-1);
  if (e.key === 'ArrowRight' && (step === 'map' || step === 'drone')) cycle(1);
});
document.addEventListener('keyup', e => manualKey(e, false));

// ------------------------------------------------------------------ info card (top left)
$('worldSearch').oninput = () => buildGalleries();
$$('#worldFilter button').forEach(b => b.onclick = () => { worldFilter = worldFilter === b.dataset.f ? null : b.dataset.f; store.set('worldFilter', worldFilter); buildGalleries(); });
document.addEventListener('change', e => { if (e.target.id === 'variantPick') chooseWorld(e.target.value); });
function renderInfo() {
  const el = $('info');
  if (step === 'history') { el.innerHTML = histRun ? runCard(histRun) : `<div class="kicker">History</div><h2>Past runs</h2><div class="lead">Pick a run on the left to see its flight and replay it. Runs come from ${esc((S && S.history_dirs) || '~/mission_runs')}.</div>`; bindRunCard(); return; }
  if (group !== 'sim' || step === 'live') { el.innerHTML = ''; return; }
  const w = curWorld(), d = curDrone();
  if (step === 'map' && w) {
    const n = w.notes, moving = w.dynamic;
    const kinds = Object.keys(w.obstacles || {});
    el.innerHTML = `<div class="kicker">Map</div><h2>${esc(nice(w.name))}</h2>
      <div class="lead">${esc(n && n.kind ? n.kind : worldLabel(w.group))}</div>
      <dl>${worldGroups[w.group].length > 1 ? `<dt>Variant</dt><dd><select id="variantPick" ${simUp() ? 'disabled' : ''}>${worldGroups[w.group].map(x => `<option value="${esc(x.file)}" ${x.file === w.file ? 'selected' : ''}>${esc(variantLabel(x))}${x.dynamic ? ' · moving' : ''}</option>`).join('')}</select></dd>` : ''}
      <dt>Size</dt><dd>${sizeText(w.extent)}</dd>
      <dt>Obstacles</dt><dd><span class="badge">${moving ? icon('i-walk') + ` ${moving} moving` : 'all static'}</span></dd>
      <dt>Contains</dt><dd><div class="tags">${kinds.length ? kinds.map(k => `<span class="tag">${esc(k)}</span>`).join('') : '—'}</div></dd>
      <dt>Est. exploration time</dt><dd>${n && n.typical_s ? `${Math.round(n.typical_s[0] / 6) / 10}–${Math.round(n.typical_s[1] / 6) / 10} min to 90%` : '—'}</dd>
      ${n && n.typical_s ? `<dt></dt><dd class="sub">${esc(n.source)}</dd>` : ''}
      <dt>Compatible planners</dt><dd>${plannersHere(w)}</dd>
      <dt>Drone starts at</dt><dd>${w.start.map(v => v.toFixed(1)).join(', ')}</dd></dl>`;
  } else if (step === 'drone' && d) {
    const b = droneBounds(d), s = b ? [b[1][0] - b[0][0], b[1][1] - b[0][1], b[1][2] - b[0][2]] : [d.span, d.span, d.span * 0.5];
    const nFish = d.sensors.filter(x => x.kind === 'fisheye camera').length, cam = simCam(d);
    const sensors = [...new Set((d.model ? ['lidar'] : []).concat(d.sensors.map(x => x.kind)))].map(k => ({ lidar: 'Mid-360 LiDAR', 'depth camera': 'RGB-D camera', 'fisheye camera': nFish > 1 ? 'dual fisheye 360' : '360 camera', '360 depth (ideal)': 'ideal 360 depth' }[k] || k));
    const v = settingValue('autonomous_flight/desired_velocity'), y = settingValue('autonomous_flight/desired_angular_velocity'), r = settingValue('dynamic_map/raycast_max_length');
    el.innerHTML = `<div class="kicker">Drone</div><h2>${esc(d.label)}</h2><div class="lead">${esc(d.note || '')}</div>
      <dl><dt>Size</dt><dd>${s.map(x => (x * 100).toFixed(0)).join(' × ')} cm</dd>
      <dt>Sensors</dt><dd><div class="tags">${sensors.map(x => `<span class="tag">${esc(x)}</span>`).join('')}</div></dd>
      ${hasTilt(d) ? `<dt>LiDAR tilt</dt><dd>${simPitch()}°</dd>` : ''}
      ${cam ? `<dt>360 camera</dt><dd>${camText(cam)}</dd>` : ''}
      <dt>Flight speed</dt><dd>${v != null ? v + ' m/s' : '—'}</dd><dt>Yaw speed</dt><dd>${y != null ? y + ' rad/s' : '—'}</dd>
      <dt>Trusted range</dt><dd>${r != null ? r + ' m' : '—'}</dd>
</dl>`;
  } else if (step === 'mission' && summary) {
    el.innerHTML = '';
  } else if (step === 'mission') {
    const m = modeOf(pick.mode);
    el.innerHTML = `<div class="kicker">Mission</div><h2>${esc(m === 'Exploration' ? plannerOf(pick.mode) : m)}</h2><div class="lead">${esc(vinfo(pick.mode).details || vinfo(pick.mode).about || MODE_TEXT[m] || '')}</div>
      <dl><dt>World</dt><dd>${w ? esc(nice(w.name)) : '—'}</dd><dt>Drone</dt><dd>${d ? esc(d.label) : '—'}</dd>
      ${hasTilt(d) ? `<dt>LiDAR tilt</dt><dd>${simPitch()}°</dd>` : ''}</dl>`;
  } else el.innerHTML = '';
}
function plannersHere(w) {
  const t = missionTask(); if (!t || !w.planners) return '—';
  const names = t.variants.filter((v, i) => vinfo(i).mode === 'Exploration' && w.planners[i]);
  return names.length ? `<span class="okc">${names.map(esc).join(' · ')}</span>` : '<span class="warn">none set up</span>';
}
const MODE_TEXT = {
  Exploration: 'Maps the whole site on its own, then returns home and lands.',
  Navigation: 'Flies to goals you set, avoiding static and moving obstacles.',
  Inspection: 'Flies close passes over surfaces for a detailed look.',
  Manual: 'You fly it from this window with the keyboard. No planner runs.',
};
const MODE_ICON = { Exploration: 'i-explore', Navigation: 'i-nav', Inspection: 'i-inspect', Manual: 'i-keys' };

// ------------------------------------------------------------------ drone annotations (callouts)
function drawAnnotations() {
  const d = curDrone(); if (!d || !droneParts) return;
  const W = view.clientWidth, H = view.clientHeight, cx = W / 2;
  const comps = components(d), pts = [];
  for (const c of comps) {
    const p = new THREE.Vector3(...c.xyz);
    if (c.edit === 'mount' && droneParts.cam) droneParts.cam.localToWorld(p.set(0, 0, 0));      // follows the mount choice
    else if (c.moving && droneParts.hinge) droneParts.hinge.localToWorld(p); else droneParts.root.localToWorld(p);
    p.project(camera);                                             // keep every callout, so labels never pop in and out
    pts.push({ c, x: Math.min(W - 10, Math.max(10, (p.x + 1) / 2 * W)), y: Math.min(H - 10, Math.max(10, (1 - p.y) / 2 * H)) });
  }
  // two columns of labels, left and right of the drone, spread so they never overlap
  const onLeft = p => p.x < cx && !p.c.edit;                       // callouts with controls stay right, clear of the panels
  const left = pts.filter(onLeft).sort((a, b) => a.y - b.y), right = pts.filter(p => !onLeft(p)).sort((a, b) => a.y - b.y);
  // keep labels inside the view: below the details card on the left, clear of the right edge
  const info = $('info'), infoBottom = info.offsetHeight ? info.offsetTop + info.offsetHeight + 16 : 80;
  // labels are centred on ly: space them by their drawn heights (last frame's), so an open slider panel never overlaps the next
  const kids = $('annotLabels').children, half = p => ((kids[pts.indexOf(p)] || {}).offsetHeight || 44) / 2;
  const spread = (list, top) => {
    let y = top, prev = 0;
    for (const p of list) { const h = half(p); p.ly = Math.max(p.y, y + prev + h); prev = h; y = p.ly + 8; }
    const over = list.length ? list[list.length - 1].ly + half(list[list.length - 1]) - (H - 110) : 0;
    if (over > 0) list.forEach(p => p.ly -= over);
  };
  spread(left, infoBottom); spread(right, 70);
  const span = Math.min(W * 0.26, 300);
  let svg = '';
  for (const p of left) p.lx = Math.max(cx - span, 200);
  for (const p of right) p.lx = Math.min(cx + span, W - 340);                // room for the widest label
  for (const p of pts) {
    const ex = p.lx + (onLeft(p) ? 24 : -24);
    svg += `<polyline points="${p.x},${p.y} ${ex},${p.ly} ${p.lx},${p.ly}"/><circle cx="${p.x}" cy="${p.y}" r="4"/>`;
  }
  $('annot').innerHTML = svg;
  const host = $('annotLabels');
  if (host.childElementCount !== pts.length || host.dataset.key !== d.file) {
    host.dataset.key = d.file; host.innerHTML = '';
    for (const p of pts) {
      const el = document.createElement('div'); el.className = 'alabel';
      const tilt = p.c.edit === 'tilt' && hasTilt(d), mount = p.c.edit === 'mount' && !!simCam(d), tr = xarg(d, 'lidar_pitch_deg');
      el.innerHTML = `<b>${esc(p.c.label)}${tilt ? ` <em class="tilt-val"></em>` : ''}${mount ? ` <em class="cam-val"></em>` : ''}</b><span>${esc(p.c.detail || '')}${tilt ? ' · click to set the tilt' : ''}${mount ? ' · click to move it' : ''}</span>` +
        (tilt ? `<div class="tilt-pop" hidden><input type="range" min="${tr.min}" max="${tr.max}" step="0.5" class="tilt-range"><input type="number" step="0.5" min="${tr.min}" max="${tr.max}" class="tilt-in"><span>°</span></div>` : '') +
        (mount ? `<div class="tilt-pop cam-pop" hidden><div class="cam-side"><button data-side="0">Front-back</button><button data-side="1">Left-right</button></div>${camAxes(d).map(a => `<label data-k="${a.k}">${a.label}</label><input type="range" min="${a.lo}" max="${a.hi}" step="0.5" class="cam-range" data-k="${a.k}"><input type="number" step="0.5" min="${a.lo}" max="${a.hi}" class="cam-in" data-k="${a.k}"><span data-k="${a.k}">${a.unit}</span>`).join('')}</div>` : '');
      if (tilt || mount) {                                         // the label opens its pop-up; clicks inside it stay there
        el.classList.add('clickable');
        const pop = el.querySelector('.tilt-pop');
        el.onclick = e => { if (!e.target.closest('.tilt-pop')) pop.hidden = !pop.hidden; };
      }
      if (mount) {
        el.querySelectorAll('.cam-range, .cam-in').forEach(i => i.oninput = () => setCam(i.dataset.k, +i.value));
        el.querySelectorAll('.cam-side button').forEach(b => b.onclick = () => setCam('side', b.dataset.side === '1'));
      }
      if (tilt) {
        const inp = el.querySelector('.tilt-in'), rng = el.querySelector('.tilt-range');
        inp.oninput = () => setPitch(+inp.value); rng.oninput = () => setPitch(+rng.value);
      }
      host.appendChild(el);
    }
  }
  [...host.children].forEach((el, i) => {
    const p = pts[i]; if (!p) return;
    el.classList.toggle('left', onLeft(p));
    el.style.left = (onLeft(p) ? p.lx - 6 : p.lx + 6) + 'px'; el.style.top = p.ly + 'px';
    const inp = el.querySelector('.tilt-in'), rng = el.querySelector('.tilt-range'), tv = el.querySelector('.tilt-val');
    if (inp && document.activeElement !== inp) inp.value = simPitch();
    if (rng && document.activeElement !== rng) rng.value = simPitch();
    if (tv) tv.textContent = simPitch() + '°';
    if (inp) inp.disabled = rng.disabled = simUp();
    const cam = simCam(d), cv = el.querySelector('.cam-val');
    if (cam && cv) {
      cv.textContent = camText(cam);
      el.querySelectorAll('.cam-range, .cam-in').forEach(i => { const a = camAxes(d).find(x => x.k === i.dataset.k); if (document.activeElement !== i) i.value = +(cam[a.k] * a.f).toFixed(1); i.disabled = simUp(); });
      el.querySelectorAll('.cam-pop [data-k]').forEach(x => x.hidden = x.dataset.k === 'x' && cam.side);
      el.querySelectorAll('.cam-side button').forEach(b => { b.classList.toggle('on', (b.dataset.side === '1') === !!cam.side); b.disabled = simUp(); });
    }
  });
}

// ------------------------------------------------------------------ LiDAR tilt
function setPitch(v) {
  const r = xarg(curDrone(), 'lidar_pitch_deg'); if (simUp() || !r || !isFinite(v)) return;
  v = Math.max(r.min, Math.min(r.max, Math.round(v * 2) / 2));
  pick.pitch = v; store.set('pitch', v);
  applyTilt(); renderInfo();
}

// ------------------------------------------------------------------ 360 camera mount (the lens centre in base_link)
// front-back: x (fore-aft) and z; left-right: the lens centre on the drone's axis, z only. Ranges from sim.xargs, in cm
const camAxes = d => [['x', 'insta360_x'], ['z', 'insta360_z']].filter(([, n]) => xarg(d, n))
  .map(([k, n]) => ({ k, label: k, lo: xarg(d, n).min * 100, hi: xarg(d, n).max * 100, unit: 'cm', f: 100 }));
const camText = c => c.side ? `left-right · z ${(c.z * 100).toFixed(1)} cm · tilt ${c.tilt}°` : `x ${(c.x * 100).toFixed(1)} · z ${(c.z * 100).toFixed(1)} cm`;
function setCam(k, v) {
  const d = curDrone(), c = camPick(d); if (simUp() || !c) return;   // the raw pick: x survives a left-right spell
  if (k === 'side') c.side = !!v;
  else {
    const a = camAxes(d).find(x => x.k === k); if (!a || !isFinite(v)) return;
    c[k] = Math.max(a.lo, Math.min(a.hi, Math.round(v * 2) / 2)) / a.f;
  }
  store.set('cam.' + d.file, c);
  if (droneParts && droneParts.cam) placeCam(droneParts.cam, simCam(d));
  placeDrone(); renderInfo();
}

// ------------------------------------------------------------------ flight settings
function modeDir() { return vinfo(pick.mode).dir || null; }
async function loadSettings() {
  const dir = modeDir() || '-'; if (dir === settingsDir) return;
  settingsDir = dir;
  if (dir === '-') { settingsFields = []; renderSettings(); renderInfo(); return; }
  const r = await api('/api/settings?dir=' + dir); settingsFields = r.fields; renderSettings(); renderInfo();
}
const savedSetting = k => store.get('set.' + k, null);
function settingValue(k) { const f = settingsFields.find(x => x.key === k); if (!f) return null; const s = savedSetting(k); return s === null ? f.value : s; }
const changedSettings = () => settingsFields.filter(f => f.type !== 'info' && savedSetting(f.key) !== null && savedSetting(f.key) !== f.value);
function renderSettings() {
  const body = $('settingsBody'); body.innerHTML = '';
  if (!settingsFields.length) {
    body.innerHTML = `<div class="sub">${vinfo(pick.mode).own_sim ? 'EPIC uses its own configuration (~/EPIC/src/global_planner/exploration_manager/config).' : isManualPick() ? 'Manual flight has no planner settings.' : 'Loading…'}</div>`;
    $('settingsSave').disabled = $('settingsReset').disabled = true; $('settingsNote').textContent = ''; return;
  }
  for (const f of settingsFields) {
    const row = document.createElement('div'); row.className = 'setting' + (f.type === 'info' ? ' is-info' : '');
    row.innerHTML = `<div class="lbl">${esc(f.label)}${f.note ? `<small>${esc(f.note)}</small>` : ''}</div>`;
    if (f.type === 'info') { row.insertAdjacentHTML('beforeend', `<div class="val">${esc(f.value)}</div>`); body.appendChild(row); continue; }
    let inp;
    if (f.type === 'bool') { inp = document.createElement('label'); inp.className = 'switch'; inp.innerHTML = '<input type="checkbox"><span></span>'; const cb = inp.querySelector('input'); cb.checked = !!settingValue(f.key); cb.onchange = () => saveSetting(f, cb.checked); }
    else { inp = document.createElement('input'); inp.type = 'number'; inp.className = 'num-in'; inp.min = f.min; inp.max = f.max; inp.step = f.step; inp.value = settingValue(f.key); inp.onchange = () => saveSetting(f, +inp.value); inp.classList.toggle('changed', savedSetting(f.key) !== null && savedSetting(f.key) !== f.value); }
    row.appendChild(inp); row.insertAdjacentHTML('beforeend', `<span class="unit">${esc(f.unit || '')}</span>`);
    body.appendChild(row);
  }
  const n = changedSettings().length; $('settingsNote').textContent = n ? `${n} changed` : '';
  $('settingsSave').disabled = !n; $('settingsReset').disabled = !n;
}
function saveSetting(f, v) {
  if (f.type !== 'bool') { if (!isFinite(v)) return; v = Math.max(f.min, Math.min(f.max, v)); }
  store.set('set.' + f.key, v === f.value ? null : v); renderSettings(); renderInfo();
}
const isManualPick = () => !!vinfo(pick.mode).manual;
$('settingsReset').onclick = () => { settingsFields.forEach(f => store.set('set.' + f.key, null)); renderSettings(); renderInfo(); };
$('settingsSave').onclick = async () => {
  const changed = changedSettings(); if (!changed.length) return;
  if (!confirm(`Write ${changed.length} setting(s) into the ${modeDir()} config files? They become the defaults for every launch.`)) return;
  const values = {}; changed.forEach(f => values[f.key] = savedSetting(f.key));
  const r = await api('/api/settings/save', { dir: modeDir(), values });
  if (r.message !== 'ok') return toast(r.message, 'error');
  changed.forEach(f => store.set('set.' + f.key, null)); settingsDir = null; await loadSettings(); toast('Saved to the config files');
};
function renderMissionSub() {
  const t = missionTask(); if (!t) return;
  $('missionSub').textContent = running(t) ? `${plannerOf(t.variant)} running ${fmtTime(Date.now() / 1000 - t.since)}` : sessionUp() ? (isManual() ? 'Manual flight' : 'Simulator running') : `${modeOf(pick.mode) === 'Exploration' ? plannerOf(pick.mode) : modeOf(pick.mode)} · not started`;
}
$('rvizWindow').checked = store.get('rviz', false);
$('rvizWindow').onchange = e => { store.set('rviz', e.target.checked); renderInfo(); };

// ------------------------------------------------------------------ mission cards, launch sequence, CTA
const MODES = ['Exploration', 'Navigation', 'Inspection', 'Manual'];
// The bottom panel of the Mission step: a tab per mode; Exploration lists its planners for this world.
function renderMissionPanel() {
  if (step !== 'mission') return;
  const t = missionTask(); if (!t) return;
  const w = curWorld(), ok = i => !w || !w.planners || w.planners[i];
  const tab = modeOf(pick.mode);
  $('missionTabs').innerHTML = MODES.map(m => `<button data-m="${m}" class="${m === tab ? 'on' : ''}">${icon(MODE_ICON[m])}${m}</button>`).join('');
  $$('#missionTabs button').forEach(b => b.onclick = () => {
    const m = b.dataset.m, idx = t.variants.map((v, i) => i).filter(i => modeOf(i) === m);
    const pickI = idx.find(ok) ?? idx[0];
    if (pickI != null) choosePlanner(pickI);
  });
  const c = $('missionContent');
  if (tab === 'Exploration') {
    const idx = t.variants.map((v, i) => i).filter(i => modeOf(i) === 'Exploration');
    c.innerHTML = `<div class="planners">` + idx.map(i => {
      const v = vinfo(i), avail = ok(i);
      return `<button class="planner${i === pick.mode ? ' on' : ''}" data-i="${i}" ${avail ? '' : 'disabled'}>
        <div class="ph"><b>${esc(plannerOf(i))}</b></div>
        <span>${esc(v.about || '')}</span>${avail ? '' : '<small>Not set up for this world</small>'}</button>`;
    }).join('') + '</div>';
    $$('.planner', c).forEach(b => b.onclick = () => choosePlanner(+b.dataset.i));
  } else { c.innerHTML = tab === 'Manual' ? '<div class="keys">T take off · G land · W/S forward and back · A/D left and right · Q/E turn · R/F up and down</div>' : ''; }
  const avail = ok(pick.mode);
  $('launchBtn').disabled = !!launching || !avail;
  $('launchBtn').innerHTML = launching ? esc(launching) : 'Launch live view ' + icon('i-chev');
  $('launchNote').textContent = !avail ? 'Not set up for this world' : '';
}
function choosePlanner(i) {
  pick.mode = i; store.set('mode', i); settingsDir = null; loadSettings(); renderMissionPanel(); renderInfo(); renderMissionSub();
}
$('launchBtn').onclick = () => launch();
function renderCta() {
  const cta = $('stepCta'), sim = group === 'sim';
  const inLive = step === 'live';
  $('flightBar').hidden = !(sim && inLive && sessionUp() && !isManual() && !launching);
  $('manualBar').hidden = !(sim && inLive && sessionUp() && isManual() && !launching);
  cta.hidden = !sim || step === 'mission' || step === 'history' || (inLive && sessionUp() && !launching) || (inLive && !sessionUp() && !launching);
  cta.classList.toggle('busy', !!launching); cta.disabled = !!launching;
  $('backBtn').hidden = cta.hidden || step === 'map' || !!launching;
  if (launching) cta.textContent = launching;
  else if (step === 'map') cta.innerHTML = 'Choose drone ' + icon('i-chev');
  else if (step === 'drone') cta.innerHTML = 'Choose mission type ' + icon('i-chev');
  else cta.innerHTML = 'Launch live view ' + icon('i-chev');
}
$('backBtn').onclick = () => setStep(STEPS[Math.max(0, STEPS.indexOf(step) - 1)]);
$('missionBack').onclick = () => setStep('drone');
$('stepCta').onclick = () => { if (step === 'map') setStep('drone'); else if (step === 'drone') setStep('mission'); else launch(); };

async function waitFor(cond, ms) { const t0 = Date.now(); while (Date.now() - t0 < ms) { if (cond()) return true; await sleep(250); } return cond(); }
// Start the simulator with the chosen world and drone, connect the view, then the planner (unless manual).
// EPIC brings its own simulator (MARSIM) and RViz, so only the planner starts.
async function launch() {
  if (launching) return;
  const w = curWorld(), v = vinfo(pick.mode), mode = modeOf(pick.mode);
  if (w && w.planners && !w.planners[pick.mode]) { toast(`${plannerOf(pick.mode)} is not set up for this world`, 'error'); setStep('mission'); return; }
  newSession();
  const settings = {}; for (const f of changedSettings()) settings[f.key] = savedSetting(f.key);
  try {
    if (v.own_sim) {
      launching = `Starting ${plannerOf(pick.mode)}…`; setStep('live');
      const r = await api('/api/task/sim/mission/start', { variant: pick.mode, settings: Object.assign(settings, { _world: pick.world }) });
      if (r.message !== 'ok') throw new Error(r.message);
      launching = 'Connecting live view…'; renderCta(); await waitFor(() => rosOk, 30000);
    } else {
      if (!simUp()) {
        launching = 'Starting simulator…'; setStep('live');
        const r = await api('/api/task/sim/sim/start', { world: pick.world, drone: pick.drone, gazebo: false, planner: pick.mode, xargs: pickXargs(curDrone()) });
        if (r.message !== 'ok') throw new Error(r.message);
        if (!await waitFor(() => (simTask() || {}).state === 'running', 60000)) throw new Error('The simulator did not start: open Processes in the data bar');
      }
      setStep('live');
      if ($('rvizWindow').checked && !running(task('rviz', 'sim'))) api('/api/task/sim/rviz/start', {});
      launching = 'Connecting live view…'; renderCta();
      await waitFor(() => rosOk, 30000);
      if (!v.manual && !running(missionTask())) {
        launching = `Launching ${plannerOf(pick.mode)}…`; renderCta();
        await sleep(3000);                                           // let Gazebo publish the drone before the planner subscribes
        const r = await api('/api/task/sim/mission/start', { variant: pick.mode, settings });
        if (r.message !== 'ok') throw new Error(r.message);
      }
      if (v.manual && ros) new ROSLIB.Topic({ ros, name: '/CERLAB/quadcopter/vel_mode', messageType: 'std_msgs/Bool' }).publish({ data: true });
    }
  } catch (e) { toast(e.message || String(e), 'error'); }
  launching = null; renderCta(); render();
}
// a new session starts clean: no old trail, path, map, summary or replay
function newSession() {
  summary = null; replay = null; trailN = 0; trailT = []; trailQ = []; trailGeo.setDrawRange(0, 0); pathGeo.setDrawRange(0, 0);
  heightCloud = colorCloud = null; redrawMap(); home.visible = false; lastFlightState = null;
  Object.assign(live, { status: null, pose: null, pathLen: 0, speed: 0, lastPose: null, points: 0, maxZ: -Infinity, startT: null, explored: null });
  $('summary').hidden = true; renderBar();
}
async function endSession() {
  if (!confirm('End the session? This stops the planner, RViz and the simulator.')) return;
  summary = makeSummary();
  if (isManual() && trailN > 1) api('/api/runs/save', { start: Date.now() / 1000 - (trailT[trailN - 1] - trailT[0]), duration: trailT[trailN - 1] - trailT[0], preset: 'manual', planner: 'Manual',
    map: curWorld() ? curWorld().group : '', path_m: live.pathLen, min_z: summary.minZ, max_z: summary.maxZ, end_state: 'LANDED', world: pick.world, drone: pick.drone, pitch: pick.pitch,
    track: Array.from({ length: trailN }, (_, i) => [trailT[i] - trailT[0], trailPos[i * 3], trailPos[i * 3 + 1], trailPos[i * 3 + 2]]) });
  for (const id of ['record_mission', 'mission', 'rviz', 'sim', 'link', 'core']) if (running(task(id, 'sim'))) await api(`/api/task/sim/${id}/stop`, {});
  await waitFor(() => !sessionUp(), 30000);
  setLeft(true); setStep('mission'); showSummary(); startReplay();
}
// ------------------------------------------------------------------ after a session: summary and replay
function makeSummary() {
  const s = live.status || {}, t = trailT.length ? trailT[trailT.length - 1] - trailT[0] : 0;
  return { world: curWorld() ? nice(curWorld().name) : '—', drone: curDrone() ? curDrone().label : '—', planner: isManual() ? 'Manual' : plannerOf(curVariant()),
    duration: s.elapsed != null ? s.elapsed : t, path: live.pathLen, maxZ: isFinite(live.maxZ) ? live.maxZ : null, minZ: trailN ? Math.min(...Array.from({ length: trailN }, (_, i) => trailPos[i * 3 + 2])) : null,
    explored: s.explored_block_pct, floor: s.floor_pct, surface: s.surface_pct, state: s.state, points: live.points, samples: trailN };
}
function showSummary() {
  const m = summary; if (!m) return;
  const f = v => (v == null || v < 0 ? '—' : Math.round(v) + '%');
  $('summary').innerHTML = `<div class="kicker">Flight summary</div><h2>${esc(m.planner)}</h2><div class="sub">${esc(m.world)} · ${esc(m.drone)}</div>
    <dl><dt>Duration</dt><dd>${fmtTime(m.duration)}</dd><dt>Path flown</dt><dd>${Math.round(m.path)} m</dd>
    <dt>Average speed</dt><dd>${m.duration > 0 ? (m.path / m.duration).toFixed(2) + ' m/s' : '—'}</dd>
    <dt>Height range</dt><dd>${m.minZ != null && m.maxZ != null ? `${m.minZ.toFixed(1)} to ${m.maxZ.toFixed(1)} m` : '—'}</dd>
    <dt>Explored</dt><dd>${f(m.explored)}</dd><dt>Floor inspected</dt><dd>${f(m.floor)}</dd><dt>Surfaces up close</dt><dd>${f(m.surface)}</dd>
    <dt>Ended</dt><dd>${esc(m.state ? (STATE[m.state] || m.state) : 'by you')}</dd></dl>
    <div class="row2"><button class="btn" id="btnReplay">${icon('i-replay')}Replay</button><button class="btn ghost" id="btnCloseSum">Close</button></div>`;
  $('summary').hidden = false; $('info').innerHTML = '';
  $('btnReplay').onclick = startReplay;
  $('btnCloseSum').onclick = () => { summary = null; replay = null; $('summary').hidden = true; renderInfo(); placeDrone(); };
}
// The recorded flight played back in about 12 s: the drone follows its trail, which draws as it goes.
function startReplay() {
  if (trailN < 2) return;
  const T0 = trailT[0], T1 = trailT[trailN - 1];
  replay = { t0: performance.now(), dur: Math.min(20, Math.max(6, (T1 - T0) / 8)) * 1000, T0, T1, flying: true };
  layers.trail.visible = true;
  const box = new THREE.Box3(); for (let i = 0; i < trailN; i++) box.expandByPoint(new THREE.Vector3(trailPos[i * 3], trailPos[i * 3 + 1], trailPos[i * 3 + 2]));
  frameBox(box.expandByScalar(3), 0.8);
}
function stepReplay(now) {
  const k = Math.min(1, (now - replay.t0) / replay.dur), T = replay.T0 + k * (replay.T1 - replay.T0);
  let i = 0; while (i < trailN - 1 && trailT[i + 1] < T) i++;
  const j = Math.min(i + 1, trailN - 1), f = trailT[j] > trailT[i] ? (T - trailT[i]) / (trailT[j] - trailT[i]) : 0;
  drone.position.set(...[0, 1, 2].map(a => trailPos[i * 3 + a] + f * (trailPos[j * 3 + a] - trailPos[i * 3 + a])));
  if (trailQ[i] && trailQ[j]) drone.quaternion.copy(trailQ[i]).slerp(trailQ[j], f);      // the LiDAR shell turns with it
  trailGeo.setDrawRange(0, j + 1);
  if (k >= 1) { replay = null; trailGeo.setDrawRange(0, trailN); }
}
for (const b of $$('#flightBar button, #manualBar button')) b.onclick = () => {
  if (b.dataset.cmd === 'end') return endSession();
  if (b.dataset.cmd === 'kill') { if (confirm('Force quit the planner now? It will not shut down cleanly.')) api('/api/task/sim/mission/kill', {}); return; }
  if (b.dataset.key) return manualPress(b.dataset.key);
  flight(b.dataset.cmd);
};
function flight(c) {
  const st = live.status ? live.status.state : null;
  let name = c; if (c === 'pause' && st === 'PAUSED') name = 'resume';
  if (c === 'land' && group === 'drone' && !confirm('Land here, now?')) return;
  if (c === 'start') missionStarted();
  trigger(name);
}
function trigger(name) {
  if (!ros) return;
  new ROSLIB.Service({ ros, name: '/exploration/' + name, serviceType: 'std_srvs/Trigger' })
    .callService({}, r => toast(r.message || name), e => toast('Command failed: ' + e, 'error'));
}

// ------------------------------------------------------------------ manual keyboard flight (Manual mission)
// The same commands as uav_simulator's keyboard window: /CERLAB/quadcopter/{takeoff,land,cmd_vel}.
const held = new Set(); let manualTimer = null, cmdPub = null;
function manualActive() { return group === 'sim' && step === 'live' && isManual() && rosOk; }
function manualPress(k) {
  if (!ros) return;
  if (k === 't') { new ROSLIB.Topic({ ros, name: '/CERLAB/quadcopter/takeoff', messageType: 'std_msgs/Empty' }).publish({}); missionStarted(); }
  if (k === 'g') new ROSLIB.Topic({ ros, name: '/CERLAB/quadcopter/land', messageType: 'std_msgs/Empty' }).publish({});
}
function manualKey(e, down) {
  if (!manualActive()) return false;
  const k = e.key.toLowerCase();
  if (!'wasdqerftg'.includes(k) || k.length !== 1) return false;
  e.preventDefault();
  if (down && (k === 't' || k === 'g')) { if (!e.repeat) manualPress(k); return true; }
  if (down) held.add(k); else held.delete(k);
  if (!manualTimer) manualTimer = setInterval(sendTwist, 100);
  sendTwist();
  return true;
}
function sendTwist() {
  if (!ros) return;
  if (!cmdPub) cmdPub = new ROSLIB.Topic({ ros, name: '/CERLAB/quadcopter/cmd_vel', messageType: 'geometry_msgs/TwistStamped' });
  const v = 1.0, a = (p, n) => (held.has(p) ? 1 : 0) - (held.has(n) ? 1 : 0);
  cmdPub.publish({ header: { frame_id: 'base_link' }, twist: { linear: { x: v * a('w', 's'), y: v * a('a', 'd'), z: 0.8 * a('r', 'f') }, angular: { x: 0, y: 0, z: 0.8 * a('q', 'e') } } });
  if (!held.size) { clearInterval(manualTimer); manualTimer = null; }
}

// ------------------------------------------------------------------ live data (rosbridge, follows the tab)
function decodeCloud(m) {
  const bytes = typeof m.data === 'string' ? Uint8Array.from(atob(m.data), c => c.charCodeAt(0)) : new Uint8Array(m.data.buffer || m.data, m.data.byteOffset || 0, m.data.byteLength || m.data.length);
  const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const off = {}; for (const f of m.fields) off[f.name] = f.offset;
  const n = m.width * m.height, step_ = m.point_step, le = !m.is_bigendian, co = off.rgb !== undefined ? off.rgb : off.rgba;
  const xyz = new Float32Array(n * 3), rgb = co !== undefined ? new Float32Array(n * 3) : null;
  let k = 0;
  for (let i = 0; i < n; i++) {
    const b = i * step_, x = dv.getFloat32(b + off.x, le), y = dv.getFloat32(b + off.y, le), z = dv.getFloat32(b + off.z, le);
    if (!isFinite(x) || !isFinite(y) || !isFinite(z)) continue;
    xyz[k * 3] = x; xyz[k * 3 + 1] = y; xyz[k * 3 + 2] = z;
    if (rgb) { rgb[k * 3] = bytes[b + co + (le ? 2 : 1)] / 255; rgb[k * 3 + 1] = bytes[b + co + (le ? 1 : 2)] / 255; rgb[k * 3 + 2] = bytes[b + co + (le ? 0 : 3)] / 255; }
    k++;
  }
  return { xyz: xyz.subarray(0, k * 3), rgb: rgb && rgb.subarray(0, k * 3), n: k };
}
const RAMP = [[62, 139, 255], [64, 200, 224], [120, 220, 140], [255, 214, 10], [255, 120, 70]].map(c => c.map(v => v / 255));
function ramp(t) { t = Math.min(1, Math.max(0, t)) * (RAMP.length - 1); const i = Math.min(RAMP.length - 2, Math.floor(t)), f = t - i; return RAMP[i].map((v, k) => v + (RAMP[i + 1][k] - v) * f); }
let heightCloud = null, colorCloud = null;
function redrawMap() {
  const c = heightCloud;
  if (!c) { mapGeo.setDrawRange(0, 0); }
  else {
    const zs = []; for (let i = 2; i < c.xyz.length; i += 3 * Math.max(1, Math.floor(c.n / 4000))) zs.push(c.xyz[i]);
    zs.sort((a, b) => a - b);
    const lo = zs[Math.floor(zs.length * 0.02)] || 0, hi = zs[Math.floor(zs.length * 0.98)] || 1, span = Math.max(hi - lo, 0.1), col = new Float32Array(c.n * 3);
    for (let i = 0; i < c.n; i++) {
      const t = (c.xyz[i * 3 + 2] - lo) / span;
      if (colorMode === 'height') { const v = ramp(t); col.set(v, i * 3); }
      else { const v = 0.32 + 0.6 * Math.min(1, Math.max(0, t)); col[i * 3] = v * 0.92; col[i * 3 + 1] = v * 0.95; col[i * 3 + 2] = v; }
    }
    mapGeo.setAttribute('position', new THREE.BufferAttribute(c.xyz, 3)); mapGeo.setAttribute('color', new THREE.BufferAttribute(col, 3));
    mapGeo.setDrawRange(0, c.n); mapGeo.computeBoundingSphere();
  }
  if (colorCloud) {
    realGeo.setAttribute('position', new THREE.BufferAttribute(colorCloud.xyz, 3)); realGeo.setAttribute('color', new THREE.BufferAttribute(colorCloud.rgb, 3));
    realGeo.setDrawRange(0, colorCloud.n); realGeo.computeBoundingSphere();
  } else realGeo.setDrawRange(0, 0);
  $('realityNote').textContent = colorCloud ? `${colorCloud.n.toLocaleString()} points` : 'no colour data yet';
}
function updateViewTarget() {
  if (!S) return;
  const want = running(task('link')) ? S.groups[group].viz.ws : null;
  if (want === rosUrl && (ros || !want)) return;
  closeRos(); rosUrl = want; if (want) openRos(S.groups[group].viz);
}
function closeRos() {
  subs.forEach(s => s.unsubscribe()); subs = []; stopEcho(); cmdPub = null;
  if (ros) { ros.close(); ros = null; }
  rosOk = false; live.pose = null; live.lastPose = null;          // the flight's trail and numbers stay for the summary
  placeDrone(); renderBar();
}
function openRos(viz) {
  const r = new ROSLIB.Ros({ url: viz.ws }); ros = r;
  r.on('connection', () => { if (ros !== r) return; rosOk = true; subscribeAll(viz); renderBar(); render(); if (drawerTab() === 'topics') loadTopics(); });
  r.on('close', () => { if (ros !== r) return; rosOk = false; setTimeout(() => { if (ros === r) { ros = null; updateViewTarget(); } }, 2000); });
  r.on('error', () => {});
}
function sub(name, type, throttle, cb, extra) { const t = new ROSLIB.Topic(Object.assign({ ros, name, messageType: type, throttle_rate: throttle, queue_length: 1 }, extra || {})); t.subscribe(cb); subs.push(t); }
function subscribeAll(viz) {
  sub(viz.pose, 'geometry_msgs/PoseStamped', 80, onPose);
  sub(viz.path, 'nav_msgs/Path', 400, m => {
    const p = new Float32Array(m.poses.length * 3);
    m.poses.forEach((q, i) => { p[i * 3] = q.pose.position.x; p[i * 3 + 1] = q.pose.position.y; p[i * 3 + 2] = q.pose.position.z; });
    pathGeo.setAttribute('position', new THREE.BufferAttribute(p, 3)); pathGeo.setDrawRange(0, m.poses.length); pathGeo.computeBoundingSphere();
  });
  sub(viz.map, 'sensor_msgs/PointCloud2', 2500, m => { heightCloud = decodeCloud(m); live.points = heightCloud.n; redrawMap(); }, { compression: 'cbor' });
  if (viz.colored_map) sub(viz.colored_map, 'sensor_msgs/PointCloud2', 3000, m => { const c = decodeCloud(m); colorCloud = c.rgb ? c : null; redrawMap(); }, { compression: 'cbor' });
  sub('/exploration/status', 'std_msgs/String', 400, m => { try { live.status = JSON.parse(m.data); } catch (e) { return; } onStatus(); });
}
function onPose(m) {
  const p = m.pose.position, q = m.pose.orientation, t = performance.now() / 1000;
  if (step === 'live') { drone.position.set(p.x, p.y, p.z); drone.quaternion.set(q.x, q.y, q.z, q.w); }
  if (!live.pose && step === 'live' && follow) closeUp();
  live.pose = p;
  const lp = live.lastPose;
  if (lp) {
    const d = Math.hypot(p.x - lp.x, p.y - lp.y, p.z - lp.z);
    if (d > 0.05) { live.pathLen += d; live.speed = d / Math.max(t - lp.t, 1e-3); live.lastPose = { x: p.x, y: p.y, z: p.z, t }; addTrail(p); }
    else if (t - lp.t > 1) live.speed = 0;
  } else { live.lastPose = { x: p.x, y: p.y, z: p.z, t }; addTrail(p); }
  renderBarLite();
}
function addTrail(p) {
  if (trailN >= trailMax) { trailPos.copyWithin(0, 3); trailT.shift(); trailQ.shift(); trailN--; }
  trailPos.set([p.x, p.y, p.z], trailN * 3); trailT.push(performance.now() / 1000); trailQ.push(drone.quaternion.clone()); trailN++;
  live.maxZ = Math.max(isFinite(live.maxZ) ? live.maxZ : -Infinity, p.z);
  trailGeo.attributes.position.needsUpdate = true; trailGeo.setDrawRange(0, trailN); trailGeo.computeBoundingSphere();
}

// ------------------------------------------------------------------ data bar
const STATE = { IDLE: 'Ready', TAKEOFF: 'Taking off', ENTRY: 'Entering', EXPLORING: 'Exploring', INSPECTING: 'Inspecting', PAUSED: 'Paused', RETURNING: 'Returning home', LANDING: 'Landing', LANDED: 'Landed' };
function setMeter(id, v) { $('m' + id).style.width = (v > 0 ? Math.min(100, v) : 0) + '%'; $('b' + id).textContent = pct(v); }
// when the flight begins: hide the left panel and move the camera in a little
function missionStarted() {
  if (group !== 'sim') return;
  setLeft(false);
  const p = drone.position; flyTo(p.clone(), new THREE.Vector3(p.x - 3.2, p.y - 3.6, p.z + 2.2), 0.9);
}
function onStatus() {
  const s = live.status;
  if (lastFlightState && ['IDLE', 'LANDED'].includes(lastFlightState) && !['IDLE', 'LANDED'].includes(s.state)) missionStarted();
  lastFlightState = s.state;
  if (s.home && s.state !== 'IDLE') { home.position.set(s.home[0], s.home[1], s.home[2]); home.visible = true; }
  if (s.map_res) mapMat.size = s.map_res * 1.15;
  renderBar(); renderMission(); renderFlight();
}
function renderBarLite() {
  $('bHeight').textContent = live.pose ? live.pose.z.toFixed(1) + ' m' : '—';
  $('bSpeed').textContent = live.pose ? live.speed.toFixed(1) + ' m/s' : '—';
  $('bPath').textContent = live.pose ? Math.round(live.pathLen) + ' m' : '—';
}
function renderBar() {
  const s = live.status;
  $('bState').textContent = s ? (STATE[s.state] || s.state) : (rosOk ? (isManual() ? 'Manual' : 'Connected') : 'Offline');
  $('stateDot').className = 'dot' + (s && !['IDLE', 'LANDED'].includes(s.state) ? ' live' : '');
  $('bTime').textContent = s ? fmtTime(s.elapsed) : '—';
  setMeter('Explored', s ? s.explored_block_pct : -1); setMeter('Floor', s ? s.floor_pct : -1); setMeter('Surface', s ? s.surface_pct : -1);
  renderBarLite();
}
function renderMission() {
  const s = live.status || {};
  const rows = [['State', s.state ? (STATE[s.state] || s.state) : '—'], ['Elapsed', s.elapsed != null ? fmtTime(s.elapsed) : '—'], ['Explored', pct(s.explored_block_pct)],
    ['Floor inspected', pct(s.floor_pct)], ['Surfaces up close', pct(s.surface_pct)], ['Frontiers', s.frontiers == null || s.frontiers < 0 ? '—' : s.frontiers],
    ['Explored volume', s.explored_m3 != null ? Math.round(s.explored_m3).toLocaleString() + ' m³' : '—'], ['Map resolution', s.map_res ? s.map_res.toFixed(2) + ' m' : '—'],
    ['Map points', live.points ? live.points.toLocaleString() : '—'], ['Run', s.run || '—'], ['Preset', s.preset || '—'], ['Height', live.pose ? live.pose.z.toFixed(1) + ' m' : '—']];
  $('missionKv').innerHTML = rows.map(([k, v]) => `<div><dt>${esc(k)}</dt><dd>${esc(v)}</dd></div>`).join('');
}
// Start/Pause/Home/Land buttons, wherever they are (flight bar and both Mission sections)
function renderFlight() {
  const st = live.status ? live.status.state : null, ok = rosOk && st;
  const flying = ['TAKEOFF', 'ENTRY', 'EXPLORING', 'INSPECTING', 'RETURNING', 'LANDING'].includes(st);
  for (const b of $$('[data-cmd]')) {
    const c = b.dataset.cmd;
    const label = txt => { const sp = b.querySelector('span'); if (sp) sp.textContent = txt; else b.textContent = txt; };
    if (c === 'start') { b.disabled = !(ok && ['IDLE', 'LANDED'].includes(st)); label(st === 'LANDED' ? 'Rerun' : 'Start'); }
    if (c === 'pause') { b.disabled = !(ok && (flying || st === 'PAUSED')); label(st === 'PAUSED' ? 'Resume' : 'Pause'); }
    if (c === 'kill') b.disabled = !running(missionTask());
    if (c === 'return_home') b.disabled = !(ok && (flying || st === 'PAUSED') && !['RETURNING', 'LANDING'].includes(st));
    if (c === 'land') b.disabled = !(ok && (flying || st === 'PAUSED') && st !== 'LANDING');
  }
}

// ------------------------------------------------------------------ left panel
function render() {
  if (!S) return;
  markPicks(); renderMissionSections(); renderMissionSub(); renderRecs(); renderDrone(); renderStorage(); renderCta(); renderFlight();
  if (drawerTab() === 'procs' && !$('drawer').hidden) renderProcList();
  if (sessionUp() && group === 'sim' && step !== 'live') setStep('live');
  const k = simUp() ? (simTask().choice.world_file + '|' + simTask().choice.drone_file) : null;
  if (k && k !== sceneKey) rebuildScene();
}
function renderMissionSections() {
  for (const sec of $$('.sect.mission')) {
    const g = sec.dataset.group, t = missionTask(g); if (!t) continue;
    const run = running(t), busy = ['starting', 'stopping'].includes(t.state);
    const seg = sec.querySelector('.modes');
    const variants = g === 'drone' ? t.variants.filter(v => v !== 'Manual') : t.variants;
    if (seg.children.length !== variants.length) {
      seg.innerHTML = '';
      variants.forEach(v => { const i = t.variants.indexOf(v), b = document.createElement('button'); b.textContent = v; b.dataset.i = i;
        b.onclick = () => { if (g === 'sim') { pick.mode = i; store.set('mode', i); settingsDir = null; loadSettings(); renderMissionPanel(); renderInfo(); renderCta(); } else store.set('mode.drone', i); renderMissionSections(); }; seg.appendChild(b); });
    }
    const mode = run || busy ? t.variant : (g === 'sim' ? pick.mode : store.get('mode.drone', 0));
    $$('button', seg).forEach(b => b.classList.toggle('on', +b.dataset.i === mode));
    seg.classList.toggle('locked', run || busy || (g === 'sim' && simUp()));
    const go = sec.querySelector('.mission-go'), sub = sec.querySelector('.mission-sub');
    if (g === 'sim') {
      go.textContent = simUp() ? 'End session' : launching || 'Launch live view';
      go.className = 'cta mission-go' + (simUp() ? ' stop' : launching ? ' busy' : '');
      go.disabled = !!launching;
      go.onclick = () => (simUp() ? endSession() : launch());
      sub.textContent = run ? `${t.variants[t.variant]} running ${fmtTime(Date.now() / 1000 - t.since)}` : simUp() ? (isManual() ? 'Manual flight' : 'Simulator running') : `${modeOf(pick.mode)} · not started`;
      sec.querySelector('.keys-help').hidden = !(simUp() && isManual());
    } else {
      const blocked = t.needs.filter(n => (task(n, g) || {}).state !== 'running').map(n => task(n, g).label.toLowerCase());
      go.textContent = { running: 'Shut down planner', starting: 'Launching…', stopping: 'Shutting down…' }[t.state] || `Launch ${(t.variants[mode] || 'planner').toLowerCase()}`;
      go.className = 'cta mission-go' + (t.state === 'running' ? ' stop' : busy ? ' busy' : '');
      go.disabled = t.state === 'stopping' || (!run && blocked.length > 0);
      sub.textContent = t.state === 'running' ? `${t.variants[t.variant]} running ${fmtTime(Date.now() / 1000 - t.since)}` : t.state === 'failed' ? (t.message || 'Stopped with an error') : blocked.length ? `Start the ${blocked.join(' and ')} first` : 'Ready';
      go.onclick = async () => { go.disabled = true; const r = run ? await api(`/api/task/drone/mission/stop`, {}) : await api(`/api/task/drone/mission/start`, { variant: mode }); if (r.message !== 'ok') toast(r.message, 'error'); };
    }
    for (const b of $$('.flight button', sec)) b.onclick = () => flight(b.dataset.cmd);
    const kill = sec.querySelector('.kill');
    kill.disabled = !(run || t.state === 'stopping');
    kill.onclick = async () => {
      if (!confirm('Force quit the planner now? It will not shut down cleanly' + (g === 'drone' ? ', and the drone will hold or land under the flight controller\'s failsafe.' : '.'))) return;
      const r = await api(`/api/task/${g}/mission/kill`, {}); if (r.message !== 'ok') toast(r.message, 'error');
    };
  }
}
function renderRecs() {
  for (const el of $$('.rec[data-rec]')) {
    const g = el.dataset.group || el.closest('.pane').dataset.pane, t = task(el.dataset.rec, g);
    if (!t) { el.hidden = true; continue; }
    if (!el.firstChild) {
      el.innerHTML = '<span class="rec-time"></span><select title="What to record"></select><button class="dotbtn" title="Record"><i></i></button>';
      const sel = el.querySelector('select');
      t.variants.forEach((v, i) => { const o = document.createElement('option'); o.value = i; o.textContent = v + (t.sizes[i] ? ' · ' + t.sizes[i] : ''); sel.appendChild(o); });
      sel.value = store.get(`rec.${g}.${t.id}`, 0); sel.onchange = () => store.set(`rec.${g}.${t.id}`, +sel.value);
      sel.onclick = e => e.stopPropagation();
      el.querySelector('.dotbtn').onclick = async e => {
        e.stopPropagation();
        const cur = task(t.id, g), r = running(cur) ? await api(`/api/task/${g}/${t.id}/stop`, {}) : await api(`/api/task/${g}/${t.id}/start`, { variant: +sel.value });
        if (r.message !== 'ok') toast(r.message, 'error');
      };
    }
    const run = running(t), sel = el.querySelector('select');
    el.classList.toggle('on', run); sel.disabled = run || t.state === 'stopping'; if (run) sel.value = t.variant;
    $$('option', sel).forEach((o, i) => o.textContent = sel.matches(':focus') ? t.variants[i] + (t.sizes[i] ? ' · ' + t.sizes[i] : '') : t.variants[i]);
    el.querySelector('.rec-time').textContent = run ? fmtTime(Date.now() / 1000 - t.since) : '';
    el.querySelector('.dotbtn').title = run ? 'Stop recording' : `Record (${t.variants[+sel.value]}: ${t.sizes[+sel.value] || ''})`;
  }
}
document.addEventListener('focusin', e => { if (e.target.matches('.rec select')) renderRecs(); });
document.addEventListener('focusout', e => { if (e.target.matches('.rec select')) setTimeout(renderRecs, 0); });

$('btnConnect').onclick = async () => {
  const on = S && S.drone.connected; $('btnConnect').disabled = true; $('droneMsg').textContent = on ? 'Disconnecting…' : 'Connecting…';
  const r = await api('/api/drone/' + (on ? 'disconnect' : 'connect'), {}); $('btnConnect').disabled = false;
  if (!on && r.message !== 'Connected') toast(r.message, 'error');
};
function renderDrone() {
  const d = S.drone;
  $('droneHost').textContent = d.host; $('droneMsg').textContent = d.message;
  $('btnConnect').textContent = d.connected ? 'Disconnect' : 'Connect'; $('btnConnect').className = 'btn' + (d.connected ? '' : ' primary');
  $('pillDrone').className = 'pill ' + (d.connected ? 'on' : (d.checked ? 'bad' : ''));
  const ul = $('vehicleRows'), list = tasks('drone').filter(t => !t.hidden && !t.record && t.id !== 'mission');
  if (ul.children.length !== list.length) {
    ul.innerHTML = '';
    for (const t of list) {
      const li = document.createElement('li'); li.className = 'row'; li.dataset.id = t.id;
      li.innerHTML = '<span class="dot"></span><div><div class="nm"></div><div class="meta"></div></div><button class="btn"></button>';
      li.querySelector('.nm').textContent = t.label;
      li.querySelector('button').onclick = async () => { const c = task(t.id, 'drone'); const r = await api(`/api/task/drone/${t.id}/${running(c) ? 'stop' : 'start'}`, {}); if (r.message !== 'ok') toast(r.message, 'error'); };
      ul.appendChild(li);
    }
  }
  for (const li of ul.children) {
    const t = task(li.dataset.id, 'drone'); li.className = 'row ' + t.state;
    li.querySelector('.meta').textContent = !t.configured ? 'Not set up yet: add its command in config.json' : t.state === 'running' ? 'Running ' + fmtTime(Date.now() / 1000 - t.since) : t.state === 'failed' ? (t.message || 'Stopped with an error') : t.detail;
    const b = li.querySelector('button'); b.textContent = running(t) ? 'Stop' : t.state === 'stopping' ? '…' : 'Start'; b.disabled = !t.configured || t.state === 'stopping';
  }
}

// ------------------------------------------------------------------ storage
function renderStorage() {
  const k = S.disk, low = k.free_gb < k.min_free_gb * 1.5;
  $('sFree').textContent = k.free_gb.toFixed(1) + ' GB'; $('mDisk').style.width = (100 * (k.total_gb - k.free_gb) / k.total_gb) + '%';
  const rows = [['Recordings', k.bags_gb >= 1 ? k.bags_gb.toFixed(1) + ' GB' : (k.bags_gb * 1024).toFixed(0) + ' MB'], ['Logs', `${k.logs_mb.toFixed(0)} of ${k.log_budget_mb.toFixed(0)} MB`],
    ['Recording rate', k.bag_rate_mbs > 0.01 ? k.bag_rate_mbs.toFixed(1) + ' MB/s' : 'idle'],
    ['Time left', k.record_left_min == null ? '—' : k.record_left_min > 120 ? (k.record_left_min / 60).toFixed(1) + ' h' : k.record_left_min.toFixed(0) + ' min'], ['Stops recording below', k.min_free_gb + ' GB']];
  if (S.drone.free_gb != null) rows.push(['Drone free', S.drone.free_gb.toFixed(1) + ' GB']);
  $('storageKv').innerHTML = rows.map(([a, b]) => `<div><dt>${esc(a)}</dt><dd>${esc(b)}</dd></div>`).join('');
}
async function loadBags() {
  const r = await api('/api/bags'), ul = $('bags'); ul.innerHTML = '';
  if (!r.bags.length) { ul.innerHTML = '<li><span class="sub">No recordings yet</span></li>'; return; }
  for (const b of r.bags) {
    const li = document.createElement('li'); li.innerHTML = '<span></span><span class="sub"></span><button class="icon" title="Delete">&times;</button>';
    li.children[0].textContent = b.name; li.children[0].title = r.dir + '/' + b.name;
    li.children[1].textContent = b.mb >= 1024 ? (b.mb / 1024).toFixed(1) + ' GB' : b.mb.toFixed(1) + ' MB';
    li.children[2].disabled = b.name.endsWith('.active');
    li.children[2].onclick = async () => { if (!confirm('Delete ' + b.name + '? This cannot be undone.')) return; const x = await api('/api/bags/delete', { name: b.name }); if (x.message !== 'ok') toast(x.message, 'error'); loadBags(); };
    ul.appendChild(li);
  }
}

// ------------------------------------------------------------------ topics: list + live echo
let echo = null, topicList = [];
function loadTopics() {
  const ul = $('topicList');
  if (!ros || !rosOk) { ul.innerHTML = '<li class="section">No live view yet</li>'; return; }
  ros.getTopics(r => { topicList = r.topics.map((t, i) => ({ name: t, type: r.types[i] })).sort((a, b) => a.name.localeCompare(b.name)); renderTopics(); }, e => { ul.innerHTML = `<li class="section">${esc(e)}</li>`; });
}
function renderTopics() {
  const f = $('topicFilter').value.toLowerCase(), ul = $('topicList'); ul.innerHTML = '';
  for (const t of topicList.filter(t => t.name.toLowerCase().includes(f))) {
    const li = document.createElement('li'); li.innerHTML = '<span class="dot"></span><span class="t"></span><span class="ty"></span>';
    li.querySelector('.t').textContent = t.name; li.querySelector('.ty').textContent = t.type;
    if (echo && echo.name === t.name) li.classList.add('on');
    li.onclick = () => { startEcho(t); renderTopics(); }; ul.appendChild(li);
  }
}
$('topicFilter').oninput = renderTopics;
const HEAVY = /PointCloud2|Image|MarkerArray|OccupancyGrid|LaserScan/;
function shorten(v) {
  if (typeof v === 'string') return v.length > 120 ? `<${v.length} chars>` : v;
  if (Array.isArray(v)) return v.length > 24 ? `[${v.length} items: ${JSON.stringify(shorten(v.slice(0, 3))).slice(1, -1)}, …]` : v.map(shorten);
  if (v && typeof v === 'object') { const o = {}; for (const k in v) o[k] = shorten(v[k]); return o; }
  return v;
}
function startEcho(t) {
  stopEcho();
  const heavy = HEAVY.test(t.type);
  const topic = new ROSLIB.Topic({ ros, name: t.name, messageType: t.type, throttle_rate: heavy ? 1000 : 0, queue_length: 1, compression: heavy ? 'cbor' : 'none' });
  echo = { name: t.name, topic, stamps: [], last: 0 };
  $('echoName').textContent = t.name; $('echoRate').textContent = 'waiting…'; $('echoText').textContent = '';
  topic.subscribe(m => {
    const now = performance.now(); echo.stamps.push(now); echo.stamps = echo.stamps.filter(s => now - s < 5000);
    if (now - echo.last < 250) return; echo.last = now;
    const n = echo.stamps.length, hz = n > 1 ? (n - 1) / ((echo.stamps[n - 1] - echo.stamps[0]) / 1000) : 0;
    $('echoRate').textContent = heavy ? `${t.type} · large, shown about 1/s` : `${hz.toFixed(1)} Hz · ${t.type}`;
    let obj = m; if (t.type === 'std_msgs/String') { try { obj = JSON.parse(m.data); } catch (e) { /* text */ } }
    $('echoText').textContent = JSON.stringify(shorten(obj), null, 2);
  });
}
function stopEcho() { if (echo) { echo.topic.unsubscribe(); echo = null; } }

// ------------------------------------------------------------------ processes: logs + terminals
function refreshProcList() { renderProcList(); showProc(); }
function renderProcList() {
  const ul = $('procList'); if (!S) return; ul.innerHTML = '';
  const head = txt => { const li = document.createElement('li'); li.className = 'section'; li.textContent = txt; ul.appendChild(li); };
  head(group === 'sim' ? 'Processes on this laptop' : 'Processes on the drone');
  for (const t of tasks().filter(t => t.configured && (!t.hidden || ['link', 'core'].includes(t.id) || t.state !== 'stopped'))) {
    const li = document.createElement('li'); li.innerHTML = `<span class="dot ${t.state}"></span><span class="t"></span><span class="ty"></span>`;
    li.querySelector('.t').textContent = t.label; li.querySelector('.ty').textContent = t.state + (t.state === 'running' ? ' · ' + fmtTime(Date.now() / 1000 - t.since) : '');
    if (procSel === 'task:' + t.id) li.classList.add('on');
    li.onclick = () => { procSel = 'task:' + t.id; store.set('proc', procSel); refreshProcList(); }; ul.appendChild(li);
  }
  const ids = Object.keys(terms); if (ids.length) head('Terminals');
  for (const id of ids) {
    const li = document.createElement('li'); li.innerHTML = '<span class="dot running"></span><span class="t"></span><span class="ty">bash</span>';
    li.querySelector('.t').textContent = terms[id].title; if (procSel === 'term:' + id) li.classList.add('on');
    li.onclick = () => { procSel = 'term:' + id; refreshProcList(); }; ul.appendChild(li);
  }
}
function showProc() {
  clearInterval(logTimer);
  const isTerm = procSel && procSel.startsWith('term:') && terms[procSel.slice(5)];
  $('procLog').hidden = !!isTerm; $('termHost').hidden = !isTerm;
  Object.values(terms).forEach(t => t.el.hidden = true);
  if (isTerm) {
    const t = terms[procSel.slice(5)]; t.el.hidden = false;
    $('procName').textContent = t.title; $('procMeta').innerHTML = '<button class="btn ghost" id="termClose">Close terminal</button>';
    $('termClose').onclick = () => closeTerm(procSel.slice(5)); fitTerm(); t.term.focus(); return;
  }
  const id = procSel && procSel.startsWith('task:') ? procSel.slice(5) : null, t = id && task(id);
  if (!t) { $('procName').textContent = 'Pick a process'; $('procMeta').textContent = ''; $('procLog').textContent = 'Each process’s output appears here. “+ Terminal” opens a shell with the same ROS setup.'; return; }
  $('procName').textContent = t.label; $('procMeta').textContent = t.remote ? 'on the drone' : 'this laptop';
  const pull = async () => {
    const pre = $('procLog'), atEnd = pre.scrollTop + pre.clientHeight >= pre.scrollHeight - 20;
    const r = await api(`/api/log/${group}/${t.id}?n=500`);
    pre.textContent = r.lines.length ? r.lines.join('\n') : 'Nothing logged yet.'; if (atEnd) pre.scrollTop = pre.scrollHeight;
  };
  pull(); logTimer = setInterval(() => { if (!$('drawer').hidden && drawerTab() === 'procs') pull(); }, 1500);
}
$('btnTerm').onclick = async () => {
  const r = await api('/api/term/open', { group });
  const el = document.createElement('div'); el.style.height = '100%'; $('termHost').appendChild(el);
  const term = new Terminal({ fontFamily: 'SF Mono, ui-monospace, Menlo, DejaVu Sans Mono, monospace', fontSize: 12, cursorBlink: true, allowTransparency: true,
    theme: { background: '#00000000', foreground: '#d1d1d6', cursor: '#3e8bff', selectionBackground: '#3e8bff55' } });
  const fit = new FitAddon.FitAddon(); term.loadAddon(fit); term.open(el);
  const enc = new TextEncoder();
  term.onData(d => { const b = enc.encode(d); let s = ''; b.forEach(x => s += String.fromCharCode(x)); api(`/api/term/${r.id}/input`, { data: btoa(s) }); });
  const es = new EventSource(`/api/term/${r.id}/stream`);
  es.onmessage = e => { if (e.data) term.write(Uint8Array.from(atob(e.data), c => c.charCodeAt(0))); };
  es.addEventListener('exit', () => { term.write('\r\n\x1b[90m[shell exited]\x1b[0m\r\n'); es.close(); });
  terms[r.id] = { term, fit, es, el, title: r.title + ' ' + (Object.keys(terms).length + 1) };
  procSel = 'term:' + r.id; refreshProcList();
};
function fitTerm() {
  if (!procSel || !procSel.startsWith('term:')) return;
  const t = terms[procSel.slice(5)]; if (!t || $('termHost').hidden || $('drawer').hidden) return;
  try { t.fit.fit(); api(`/api/term/${procSel.slice(5)}/resize`, { cols: t.term.cols, rows: t.term.rows }); } catch (e) { /* not visible yet */ }
}
function closeTerm(id) { const t = terms[id]; if (!t) return; t.es.close(); t.term.dispose(); t.el.remove(); delete terms[id]; api(`/api/term/${id}/close`, {}); procSel = null; refreshProcList(); }

// ------------------------------------------------------------------ History: every past run, replayable
const fmtDate = t => new Date(t * 1000).toLocaleString([], { month: 'short', day: 'numeric', hour: '2-digit', minute: '2-digit' });
async function loadRuns() {
  $('histSub').textContent = 'Loading…';
  const r = await api('/api/runs'); RUNS = r.runs;
  const maps = [...new Set(RUNS.map(x => x.map))].sort(), planners = [...new Set(RUNS.map(x => x.planner))].sort();
  const keep = (sel, opts, all) => { const v = sel.value; sel.innerHTML = `<option value="">${all}</option>` + opts.map(o => `<option>${esc(o)}</option>`).join(''); sel.value = opts.includes(v) ? v : ''; };
  keep($('histMap'), maps, 'All maps'); keep($('histPlanner'), planners, 'All planners');
  renderRuns();
}
function renderRuns() {
  const q = $('histSearch').value.trim().toLowerCase(), m = $('histMap').value, p = $('histPlanner').value;
  const list = RUNS.filter(r => (!m || r.map === m) && (!p || r.planner === p) && (!q || `${r.map} ${r.planner} ${r.preset} ${r.folder} ${r.file}`.toLowerCase().includes(q)));
  $('histSub').textContent = `${list.length} of ${RUNS.length} runs`;
  const ul = $('runList'); ul.innerHTML = ''; let day = null;
  for (const r of list) {
    const d = new Date(r.start * 1000).toDateString();
    if (d !== day) { day = d; const h = document.createElement('li'); h.className = 'group'; h.textContent = new Date(r.start * 1000).toLocaleDateString([], { weekday: 'short', month: 'short', day: 'numeric' }); ul.appendChild(h); }
    const li = document.createElement('li'); if (histRun && histRun.id === r.id) li.className = 'on';
    li.innerHTML = `<span class="rt"></span><span class="rp"></span><span class="rd"></span>`;
    li.querySelector('.rt').innerHTML = `${esc(r.planner)} · ${esc(nice(r.map))}` + (r.bags.length ? '<span class="recdot" title="has a recording"></span>' : '');
    li.querySelector('.rd').textContent = `${new Date(r.start * 1000).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })} · ${fmtTime(r.duration)} · ${Math.round(r.path_m)} m`;
    li.querySelector('.rp').innerHTML = `${r.explored != null ? Math.round(r.explored) + '%' : '—'}<small>explored</small>`;
    li.onclick = () => openRun(r); ul.appendChild(li);
  }
}
$('histSearch').oninput = renderRuns; $('histMap').onchange = renderRuns; $('histPlanner').onchange = renderRuns;
$('histRefresh').onclick = loadRuns;
async function openRun(r) {
  if (sessionUp()) return toast('Stop the simulation to replay a run');
  if (step !== 'history') setStep('history');
  histRun = r; renderRuns();
  const list = WORLDS.filter(x => x.group === r.map);
  histWorld = list.find(x => /static/.test(x.name)) || list[0] || null;
  sceneKey = null; rebuildScene();
  const t = (await api('/api/runs/track?id=' + encodeURIComponent(r.id))).track;
  // the flight as the trail: positions, times, and a heading from the direction of travel
  trailN = 0; trailT = []; trailQ = [];
  let yaw = 0;
  for (let i = 0; i < t.length && trailN < trailMax; i++) {
    const [ts, x, y, z] = t[i], n = t[Math.min(i + 3, t.length - 1)];
    if (Math.hypot(n[1] - x, n[2] - y) > 0.05) yaw = Math.atan2(n[2] - y, n[1] - x);
    trailPos.set([x, y, z], trailN * 3); trailT.push(ts); trailQ.push(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), yaw)); trailN++;
  }
  trailGeo.attributes.position.needsUpdate = true; trailGeo.setDrawRange(0, trailN); trailGeo.computeBoundingSphere();
  layers.trail.visible = true; pathGeo.setDrawRange(0, 0); home.visible = false; heightCloud = colorCloud = null; redrawMap();
  placeDrone(); renderInfo(); startReplay();
}
function runCard(r) {
  const f = v => (v == null || v < 0 ? '—' : Math.round(v) + '%');
  return `<div class="kicker">${esc(fmtDate(r.start))}</div><h2>${esc(r.planner)}</h2><div class="lead">${esc(nice(r.map))} · preset ${esc(r.preset)}${r.run > 1 ? ' · run ' + r.run : ''}</div>
    <dl><dt>Duration</dt><dd>${fmtTime(r.duration)}</dd><dt>Path flown</dt><dd>${Math.round(r.path_m)} m</dd>
    <dt>Explored</dt><dd>${f(r.explored)}</dd><dt>Floor inspected</dt><dd>${f(r.floor)}</dd><dt>Surfaces up close</dt><dd>${f(r.surface)}</dd>
    <dt>Height range</dt><dd>${r.min_z != null ? `${r.min_z} to ${r.max_z} m` : '—'}</dd><dt>Ended</dt><dd>${esc(STATE[r.end_state] || r.end_state || '—')}</dd>
    <dt>Recording</dt><dd>${r.bags.length ? `${r.bags.length} file${r.bags.length > 1 ? 's' : ''} · ${Math.round(r.bags.reduce((a, b) => a + b.mb, 0))} MB` : 'none'}</dd></dl>
    ${r.has_svg ? `<img class="path" alt="planner's path plot" src="/api/runs/svg?id=${encodeURIComponent(r.id)}">` : ''}
    <div class="path-txt">${esc(r.folder + '/' + r.file)}</div>
    <div class="row2"><button class="btn" id="runReplay">${icon('i-replay')}Replay</button>
      ${r.bags.length ? `<button class="btn" id="runPlay">${icon('i-play')}Play recording</button>` : ''}
      <button class="btn ghost" id="runCopy">Copy path</button></div>`;
}
function bindRunCard() {
  if (!histRun) return;
  const b = id => document.getElementById(id);
  if (b('runReplay')) b('runReplay').onclick = startReplay;
  if (b('runCopy')) b('runCopy').onclick = () => { navigator.clipboard && navigator.clipboard.writeText(histRun.folder + '/' + histRun.file); toast('Path copied'); };
  if (b('runPlay')) b('runPlay').onclick = async () => {
    const r = await api('/api/runs/play', { id: histRun.id });
    if (r.message !== 'ok') return toast(r.message, 'error');
    toast('Playing the recording: the view shows it live'); selectGroup('sim'); setStep('live');
  };
}

// ------------------------------------------------------------------ go
setLeft(store.get('left', true));
if (Q.get('data')) setDock(true); else setDock(store.get('dock', false));
connectEvents();
loadAssets();
setInterval(() => { if (S) { renderMissionSections(); renderMissionSub(); renderRecs(); if (drawerTab() === 'procs' && !$('drawer').hidden) renderProcList(); } }, 1000);

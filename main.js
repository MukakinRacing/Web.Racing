// main.js — 画面表示(three.js)・車モデルの読み込み・操作・コース作成
import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { DRACOLoader } from 'three/addons/loaders/DRACOLoader.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import { Car, Track, CAR } from './physics.js';

window.__ok = 1;
const $ = id => document.getElementById(id), msg = $('msg');
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const fail = t => { msg.style.display = 'flex'; msg.textContent = t; };
addEventListener('error', e => fail('エラー: ' + e.message)); // iPadは画面にエラーを出す
addEventListener('unhandledrejection', e => fail('エラー: ' + ((e.reason && e.reason.message) || e.reason)));

// ---------- 画面の準備(iPad向けに軽く) ----------
const renderer = new THREE.WebGLRenderer({ antialias: false, powerPreference: 'high-performance' });
const maxPR = Math.min(devicePixelRatio, 1.25); let pr = maxPR; // 解像度。重いと自動で下げる
renderer.setPixelRatio(pr); renderer.setSize(innerWidth, innerHeight);
renderer.toneMapping = THREE.ACESFilmicToneMapping; renderer.toneMappingExposure = 0.9;
document.body.prepend(renderer.domElement);
const scene = new THREE.Scene(); scene.fog = new THREE.Fog(0xcfe3f3, 300, 2200);
const cam = new THREE.PerspectiveCamera(62, innerWidth / innerHeight, 0.4, 4000);
scene.add(new THREE.HemisphereLight(0xcfe3ff, 0x55633d, 1.2));
const sun = new THREE.DirectionalLight(0xfff2dd, 2.4); sun.position.set(-300, 400, 200); scene.add(sun);
addEventListener('resize', () => { renderer.setSize(innerWidth, innerHeight); cam.aspect = innerWidth / innerHeight; cam.updateProjectionMatrix(); });

// 空(グラデーションの球)
const skyG = new THREE.SphereGeometry(2500, 16, 10), pa = skyG.attributes.position, cols = [];
for (let i = 0; i < pa.count; i++) { const t = Math.max(0, pa.getY(i) / 2500), c = new THREE.Color(0xcfe3f3).lerp(new THREE.Color(0x4a86c8), Math.pow(t, 0.6)); cols.push(c.r, c.g, c.b); }
skyG.setAttribute('color', new THREE.Float32BufferAttribute(cols, 3));
const sky = new THREE.Mesh(skyG, new THREE.MeshBasicMaterial({ vertexColors: true, side: THREE.BackSide, fog: false, depthWrite: false }));
scene.add(sky);

// ---------- テクスチャ(画像ファイル不要。プログラムで描く) ----------
const tex = (w, h, draw, rep) => {
  const c = document.createElement('canvas'); c.width = w; c.height = h; draw(c.getContext('2d'), w, h);
  const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace; t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.anisotropy = Math.min(4, renderer.capabilities.getMaxAnisotropy()); if (rep) t.repeat.set(rep, rep); return t;
};
const noise = (base, amp) => (g, w, h) => {
  g.fillStyle = base; g.fillRect(0, 0, w, h);
  for (let i = 0; i < w * h / 3; i++) { const v = (Math.random() - .5) * amp / 255; g.fillStyle = v > 0 ? `rgba(255,255,255,${v})` : `rgba(0,0,0,${-v})`; g.fillRect(Math.random() * w, Math.random() * h, 1.5, 1.5); }
};
const ground = new THREE.Mesh(new THREE.PlaneGeometry(5000, 5000).rotateX(-Math.PI / 2), new THREE.MeshLambertMaterial({ map: tex(128, 128, noise('#44702f', 70), 500) }));
ground.position.y = -0.05; scene.add(ground);

// ---------- オリジナルコース(約3.8km) ----------
// 上から見た地図の座標 [x, z] (単位m, zが下方向)。スタートは左上から東(右)へ。時計回り。
const CP = [[-300, 0], [0, 0], [300, 0], [560, 0], [650, 35], [700, 110], [705, 200], [705, 300], [725, 375], [790, 425], [890, 455], [970, 500], [990, 580], [940, 650], [850, 655], [720, 610], [600, 590], [510, 598], [450, 640], [390, 600], [330, 632], [250, 610], [100, 615], [-100, 610], [-260, 600], [-380, 570], [-440, 500], [-445, 420], [-420, 340], [-445, 250], [-440, 150], [-400, 45], [-330, 8]];
const HW = 7, WALL = 17, N = 900; // コース半幅7m(幅14m) / 壁の位置 / 中心線の点の数
const curve = new THREE.CatmullRomCurve3(CP.map(([x, z]) => new THREE.Vector3(x, 0, z)), true, 'centripetal'); curve.arcLengthDivisions = 3000;
let pts = curve.getSpacedPoints(N).slice(0, N);
const S0 = Math.round(N * 0.03); pts = pts.slice(S0).concat(pts.slice(0, S0)); // スタートラインを index 0 にする
const track = new Track(pts, HW);

const off = (k, o, y) => { const t = track.t[k]; return [pts[k].x + t.z * o, y, pts[k].z - t.x * o]; }; // 中心線からo[m]横の点
const flat = (o1, o2, y) => k => [...off(k, o1, y), ...off(k, o2, y)];
function strip(fn, mat, vs) { // コースに沿った帯(道路・縁石・壁)を作る
  const pos = [], uv = [], idx = []; let d = 0;
  for (let i = 0; i <= N; i++) {
    const k = i % N, p = pts[(k + N - 1) % N]; if (i > 0) d += Math.hypot(pts[k].x - p.x, pts[k].z - p.z);
    pos.push(...fn(k)); uv.push(0, d / vs, 1, d / vs);
    if (i < N) { const a = i * 2; idx.push(a, a + 1, a + 2, a + 1, a + 3, a + 2); }
  }
  const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3)); g.setAttribute('uv', new THREE.Float32BufferAttribute(uv, 2)); g.setIndex(idx); g.computeVertexNormals();
  scene.add(new THREE.Mesh(g, mat));
}
const dbl = THREE.DoubleSide, po = { polygonOffset: true, polygonOffsetFactor: -1, polygonOffsetUnits: -1 };
const stripe = tex(8, 64, g => { g.fillStyle = '#d02424'; g.fillRect(0, 0, 8, 32); g.fillStyle = '#eee'; g.fillRect(0, 32, 8, 32); });
const line = new THREE.MeshBasicMaterial({ color: 0xf4f4f4, side: dbl, ...po });
strip(flat(-HW, HW, 0), new THREE.MeshLambertMaterial({ map: tex(256, 256, noise('#3a3a40', 60)), side: dbl }), 14); // 路面
strip(flat(HW - 0.55, HW - 0.2, 0.01), line, 10); strip(flat(-HW + 0.2, -HW + 0.55, 0.01), line, 10);        // 白線
const kerb = new THREE.MeshLambertMaterial({ map: stripe, side: dbl });
strip(flat(HW, HW + 1.2, 0.01), kerb, 6); strip(flat(-HW - 1.2, -HW, 0.01), kerb, 6);                          // 縁石
for (const s of [1, -1]) strip(k => [...off(k, s * WALL, 0), ...off(k, s * WALL, 1.1)], kerb, 12);               // 壁
const chk = tex(8, 2, g => { for (let i = 0; i < 8; i++) for (let j = 0; j < 2; j++) { g.fillStyle = (i + j) % 2 ? '#111' : '#fff'; g.fillRect(i, j, 1, 1); } });
chk.magFilter = chk.minFilter = THREE.NearestFilter; chk.generateMipmaps = false;
const sl = new THREE.Mesh(new THREE.PlaneGeometry(2 * HW, 2.5).rotateX(-Math.PI / 2), new THREE.MeshBasicMaterial({ map: chk, ...po })); // スタートライン
sl.position.set(pts[0].x, 0.02, pts[0].z); sl.rotation.y = Math.atan2(track.t[0].x, track.t[0].z); scene.add(sl);

// 簡易の影(車の下の丸い影)
const shT = tex(64, 64, (g, w, h) => { const r = g.createRadialGradient(w / 2, h / 2, 2, w / 2, h / 2, w / 2); r.addColorStop(0, 'rgba(0,0,0,.65)'); r.addColorStop(1, 'rgba(0,0,0,0)'); g.fillStyle = r; g.fillRect(0, 0, w, h); });
const shadow = new THREE.Mesh(new THREE.PlaneGeometry(3, 6.2).rotateX(-Math.PI / 2), new THREE.MeshBasicMaterial({ map: shT, transparent: true, depthWrite: false, ...po }));
shadow.position.y = 0.03; scene.add(shadow);

// ---------- 操作(画面ボタン / キーボード) ----------
const key = { L: 0, R: 0, G: 0, B: 0 }; let sIn = 0;
const hold = (id, k) => {
  const e = $(id);
  e.addEventListener('pointerdown', ev => { ev.preventDefault(); key[k] = 1; e.classList.add('on'); });
  for (const n of ['pointerup', 'pointercancel', 'pointerleave']) e.addEventListener(n, () => { key[k] = 0; e.classList.remove('on'); });
};
hold('bL', 'L'); hold('bR', 'R'); hold('bG', 'G'); hold('bB', 'B');
const KM = { ArrowLeft: 'L', a: 'L', ArrowRight: 'R', d: 'R', ArrowUp: 'G', w: 'G', ArrowDown: 'B', s: 'B', ' ': 'B' };
addEventListener('keydown', e => { if (KM[e.key]) key[KM[e.key]] = 1; if (e.key === 'r') resetCar(); });
addEventListener('keyup', e => { if (KM[e.key]) key[KM[e.key]] = 0; });

// ---------- 車の状態 ----------
const car = new Car(); let idx = N - 8, ready = false, simT = 0, lapStart = null, lastLap = 0, bestLap = 0, lapNo = 0;
function resetCar(atStart) { // 今の位置のコース上(またはスタート位置)に戻す
  const i = atStart ? N - 8 : idx, p = pts[i], t = track.t[i]; idx = i; sIn = 0;
  car.reset(p.x, p.z, Math.atan2(t.x, t.z));
}
resetCar(true);
$('bReset').onclick = () => resetCar();
$('bTC').onclick = e => { car.tc = !car.tc; e.target.textContent = 'TC ' + (car.tc ? 'ON' : 'OFF'); };
$('bABS').onclick = e => { car.abs = !car.abs; e.target.textContent = 'ABS ' + (car.abs ? 'ON' : 'OFF'); };

// ---------- 車モデル(GLB)の読み込みと、タイヤ等の取り付け ----------
const wheels = []; let carObj;
function buildCar(model) {
  const find = (re, root = model) => { let r = null; root.traverse(o => { if (!r && re.test(o.name)) r = o; }); return r; };
  const drop = o => o && o.parent && o.parent.remove(o);
  // 1) 重いだけで見えない室内パーツを削除(ガラス・ハンドル・シート・ダッシュは残す)
  const cp = find(/^COCKPIT_HR/); if (cp) for (const k of cp.children.slice()) if (!/Windshield|Windows|Glass|STEER|SEAT|DASH_PANEL/i.test(k.name)) cp.remove(k);
  drop(find(/^CINTURE/)); drop(find(/^WIPER/));
  for (const d of [find(/^DOOR_L/), find(/^DOOR_R/)]) if (d) for (const k of d.children.slice()) if (/INT_|REBIT|TRAV|DOBRADICA/.test(k.name)) d.remove(k);
  // 2) 大きなテクスチャを1024pxに縮小(iPadのメモリ対策) + 反射用の環境マップ
  const seen = new Set();
  const shrink = t => {
    const im = t.image; if (!im || !(im.width > 1024) || seen.has(t)) return; seen.add(t);
    const c = document.createElement('canvas'); c.width = 1024; c.height = Math.max(1, Math.round(im.height * 1024 / im.width));
    c.getContext('2d').drawImage(im, 0, 0, c.width, c.height); t.image = c; t.needsUpdate = true;
  };
  const env = new THREE.PMREMGenerator(renderer).fromScene(new RoomEnvironment(), 0.04).texture;
  model.traverse(o => {
    if (!o.isMesh) return;
    for (const m of [].concat(o.material)) { for (const k in m) if (m[k] && m[k].isTexture) shrink(m[k]); if (m.isMeshStandardMaterial) { m.envMap = env; m.envMapIntensity = 1; } }
  });
  // 3) 大きさと位置を実車に合わせる(ホイールベース2.78m、重心を原点、タイヤ底面を地面に)
  model.updateMatrixWorld(true);
  const box = o => new THREE.Box3().setFromObject(o), ctr = o => box(o).getCenter(new THREE.Vector3());
  const names = ['LF', 'RF', 'LR', 'RR'], W = {}; // 左前,右前,左後,右後
  for (const k of names) {
    W[k] = { wh: find(new RegExp('^WHEEL_' + k)), di: find(new RegExp('^DISC_' + k)), su: find(new RegExp('^SUSP_' + k)) };
    if (!W[k].wh) throw new Error('ホイール WHEEL_' + k + ' がモデルの中に見つかりません');
  }
  let wb = (ctr(W.LF.wh).z + ctr(W.RF.wh).z - ctr(W.LR.wh).z - ctr(W.RR.wh).z) / 2;
  if (wb < 0) { model.rotation.y = Math.PI; model.updateMatrixWorld(true); wb = -wb; } // 向きが逆なら回す
  model.scale.multiplyScalar(CAR.L / wb); model.updateMatrixWorld(true);
  const cs = names.map(k => ctr(W[k].wh)), bs = names.map(k => box(W[k].wh));
  model.position.set(-cs.reduce((s, v) => s + v.x, 0) / 4, -Math.min(...bs.map(b => b.min.y)), -CAR.b - (cs[2].z + cs[3].z) / 2);
  carObj = new THREE.Group(); carObj.add(model); scene.add(carObj); carObj.updateMatrixWorld(true);
  // 4) 各タイヤを「回転の軸(ホイール中心)」に付け替える。 piv=ハンドル切れ角 / spin=タイヤの回転
  for (const k of names) {
    const piv = new THREE.Group(), spin = new THREE.Group();
    piv.position.copy(ctr(W[k].wh)); piv.add(spin); carObj.add(piv); carObj.updateMatrixWorld(true);
    spin.attach(W[k].wh); if (W[k].di) spin.attach(W[k].di);   // タイヤ+ホイール+ブレーキディスクは回る
    if (W[k].su) piv.attach(W[k].su);                          // キャリパー等は回らない(前輪は一緒に切れる)
    const fr = re => find(re, W[k].wh);
    wheels.push({ piv, spin, rim: fr(new RegExp('^RIM_' + k)), blur: fr(/^RIM_BLUR/) }); // 高速時はブラー用ホイールに切替
  }
  ready = true; resetCar(true); msg.style.display = 'none';
}
const dr = new DRACOLoader(); dr.setDecoderPath('https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/libs/draco/gltf/');
const loader = new GLTFLoader(); loader.setDRACOLoader(dr);
loader.load('PorscheGT3Rrexy.glb', g => { try { buildCar(g.scene); } catch (e) { fail('車の準備でエラー: ' + e.message); } },
  e => { msg.textContent = '車を読み込み中… ' + (e.loaded / 1e6).toFixed(1) + ' MB'; },
  e => fail('車モデルの読み込みに失敗しました。\nPorscheGT3Rrexy.glb が index.html と同じ場所にあるか確認してください。\n' + (e.message || e)));

// ---------- 毎フレームの処理 ----------
function physics(dt) {
  simT += dt;
  car.step(dt, { steer: sIn, thr: key.G, brk: key.B }, (x, z) => { const a = Math.abs(track.near(x, z, idx).off); return a < HW + 0.3 ? 1 : a < HW + 2.2 ? 0.8 : 0.5; }); // 路面→グリップ(芝は低い)
  const q = track.near(car.x, car.z, idx), a = Math.abs(q.off), lim = WALL - 1.3;
  if (a > lim) { const t = track.t[q.i], s = q.off > 0 ? -1 : 1; car.bump(t.z * s, -t.x * s, a - lim); } // 壁
  if (idx > N * 0.9 && q.i < N * 0.1) { // スタートラインを通過
    if (lapStart != null) { lastLap = simT - lapStart; if (!bestLap || lastLap < bestLap) bestLap = lastLap; }
    lapStart = simT; lapNo++;
  }
  idx = q.i;
}
const ang = [0, 0, 0, 0];
function showCar(dt) {
  carObj.position.set(car.x, 0, car.z);
  carObj.rotation.set(clamp(-car.axf * 0.0035, -0.05, 0.05), car.yaw, clamp(car.ayf * 0.004, -0.06, 0.06), 'YXZ'); // 加減速・コーナーで車体が傾く
  shadow.position.x = car.x; shadow.position.z = car.z; shadow.rotation.y = car.yaw;
  wheels.forEach((w, i) => {
    ang[i] = (ang[i] + car.w[i] * dt) % (Math.PI * 2); w.spin.rotation.x = ang[i];
    if (i < 2) w.piv.rotation.y = car.steer;
    const fast = Math.abs(car.w[i]) > 60; if (w.blur) w.blur.visible = fast; if (w.rim) w.rim.visible = !fast;
  });
}
let camYaw = 0;
function showCam(dt) {
  camYaw += Math.atan2(Math.sin(car.yaw - camYaw), Math.cos(car.yaw - camYaw)) * Math.min(1, dt * 4);
  const fx = Math.sin(camYaw), fz = Math.cos(camYaw), sp = Math.min(Math.abs(car.kmh), 300), d = 6.6 + sp * 0.006;
  cam.position.set(car.x - fx * d, 2.2, car.z - fz * d); cam.lookAt(car.x + fx * 4, 1.0, car.z + fz * 4);
  const fov = 60 + sp * 0.08; if (Math.abs(cam.fov - fov) > 0.2) { cam.fov = fov; cam.updateProjectionMatrix(); }
  sky.position.copy(cam.position);
}
const fmt = t => t ? Math.floor(t / 60) + ':' + (t % 60).toFixed(3).padStart(6, '0') : '-:--.---';
let hudT = 0;
function hud(dt) {
  hudT += dt; if (hudT < 0.1) return; hudT = 0;
  $('spd').innerHTML = Math.round(Math.abs(car.kmh)) + '<small> km/h</small>';
  $('gear').textContent = 'GEAR ' + car.gear;
  $('rpmb').style.width = Math.min(100, car.rpm / 90) + '%'; $('rpmb').style.background = car.rpm > 8300 ? '#f44' : '#4fd';
  $('lap').textContent = (lapNo ? 'LAP ' + lapNo + '  ' + fmt(simT - lapStart) : 'スタートラインまで加速!') + '\nLAST ' + fmt(lastLap) + '   BEST ' + fmt(bestLap);
}
const H = 1 / 240; let acc = 0, last = performance.now(), fc = 0, ft = 0, warm = 0;
function frame(now) {
  requestAnimationFrame(frame);
  const raw = (now - last) / 1000, dt = Math.min(0.05, raw); last = now;
  if (ready) {
    const tg = key.L - key.R; sIn += (tg - sIn) * Math.min(1, dt * (tg ? 5 : 9)); // ハンドルはゆっくり切って、戻りは速く
    acc += dt; while (acc >= H) { physics(H); acc -= H; }
    showCar(dt); hud(dt);
    fc++; ft += raw; // 重い時は解像度を自動で下げる(処理落ち対策)
    if (ft > 1.5) {
      const f = fc / ft; fc = 0; ft = 0;
      if (++warm > 3) { if (f < 45 && pr > 0.6) { pr = Math.max(0.6, pr - 0.15); renderer.setPixelRatio(pr); } else if (f > 58 && pr < maxPR) { pr = Math.min(maxPR, pr + 0.05); renderer.setPixelRatio(pr); } }
    }
  }
  showCam(dt);
  renderer.render(scene, cam);
}
requestAnimationFrame(frame);

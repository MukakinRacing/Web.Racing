// physics.js — 車の動き(物理計算)とコースの判定。three.jsを使わない「計算だけ」のファイル
const G = 9.81;
export const CAR = { m: 1300, L: 2.78, wf: 0.44, h: 0.45, tf: 1.66, tr: 1.62, R: 0.34, Iw: 2.0, Iz: 1900, Ie: 0.2 };
CAR.b = CAR.wf * CAR.L; // 重心→後輪の距離
CAR.a = CAR.L - CAR.b;  // 重心→前輪の距離
const GEAR = [0, 13.0, 9.4, 7.3, 5.9, 4.8, 4.0]; // 総減速比(1〜6速)
const TQ = [[1000, .5], [3000, .8], [5000, .95], [6500, 1], [8000, .98], [9000, .88], [9400, .5]]; // エンジントルク(回転数, 相対値)
const TMAX = 470;                  // 最大トルク[Nm]
const BRK = [3000, 3000, 1700, 1700]; // ブレーキ最大トルク(左前,右前,左後,右後)
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
function torque(rpm) {
  for (let i = 1; i < TQ.length; i++) {
    if (rpm <= TQ[i][0]) { const [r0, t0] = TQ[i - 1], [r1, t1] = TQ[i]; return t0 + (t1 - t0) * clamp((rpm - r0) / (r1 - r0), 0, 1); }
  }
  return 0;
}

// 向き(yaw)=θのとき、前方=(sinθ, cosθ)、左=(cosθ, -sinθ)  ※three.jsのx,z平面と同じ
export class Car {
  constructor() { this.tc = true; this.abs = true; this.reset(0, 0, 0); }
  reset(x, z, yaw) {
    Object.assign(this, { x, z, yaw, vx: 0, vy: 0, r: 0, w: [0, 0, 0, 0], gear: 1, rpm: 1500, steer: 0, thr: 0, brk: 0, axf: 0, ayf: 0, shift: 0, slip: [0, 0, 0, 0], kmh: 0 });
  }
  // dt:秒 / inp:{steer(-1右〜+1左), thr(0〜1), brk(0〜1)} / grip(x,z):その場所の路面グリップ係数
  step(dt, inp, grip) {
    const { m, a, b, L, h, R, Iw, Iz, Ie } = CAR, s = Math.sin(this.yaw), c = Math.cos(this.yaw);
    const sp = Math.hypot(this.vx, this.vy);
    this.thr += (inp.thr - this.thr) * Math.min(1, dt * 8);
    this.brk += (inp.brk - this.brk) * Math.min(1, dt * 10);
    this.steer = inp.steer * 0.52 / (1 + (sp / 12) ** 2); // 速度が上がるほど舵角は小さく
    const brk = (inp.thr < 0.02 && Math.abs(this.vx) < 1) ? Math.max(this.brk, 0.2) : this.brk; // 停止中は自動でブレーキ保持

    // --- エンジンと6速オートマ ---
    this.shift -= dt;
    const ratio = GEAR[this.gear], wd = (this.w[2] + this.w[3]) / 2;
    let rpm = Math.max(1500, Math.abs(wd) * ratio * 9.549);
    if (this.shift <= 0) {
      if (rpm > 8600 && this.gear < 6) { this.gear++; this.shift = 0.2; }
      else if (rpm < 4000 && this.gear > 1) { this.gear--; this.shift = 0.2; }
    }
    if (this.gear === 1 && this.thr > 0.05) rpm = Math.max(rpm, 4200); // 発進時はクラッチが滑る
    this.rpm += (rpm - this.rpm) * Math.min(1, dt * 20);
    let thr = this.thr;
    const sl = Math.max(this.slip[2], this.slip[3]);
    if (this.tc && sl > 0.12) thr *= clamp(1 - (sl - 0.12) * 8, 0.1, 1); // トラクションコントロール
    let Te = torque(rpm) * TMAX * thr - (1 - thr) * TMAX * 0.1 * rpm / 9000; // エンジンブレーキ込み
    if ((rpm > 9000 && Te > 0) || this.shift > 0) Te = 0;                    // レブリミッター/変速中
    const Tw = Te * ratio * 0.92;
    const dc = clamp(20 * (this.w[2] - this.w[3]), -0.3 * Math.abs(Tw) - 150, 0.3 * Math.abs(Tw) + 150); // LSD
    const Td = [0, 0, Tw / 2 - dc, Tw / 2 + dc]; // 後輪駆動(RWD)

    // --- 4輪の接地荷重(ダウンフォースと荷重移動) ---
    const down = 0.5 * 1.2 * 1.8 * this.vx * Math.abs(this.vx);
    const lt = m * this.axf * h / L;                                   // 前後の荷重移動
    const tfs = 0.5 * m * this.ayf * h / CAR.tf, trs = 0.5 * m * this.ayf * h / CAR.tr; // 左右の荷重移動
    const Ff = m * G * b / L / 2 + down * 0.21 - lt / 2, Fr = m * G * a / L / 2 + down * 0.29 + lt / 2;
    const Fz = [Ff - tfs, Ff + tfs, Fr - trs, Fr + trs].map(v => Math.max(v, 100));

    // --- タイヤ4本ごとの力 ---
    const P = [[a, CAR.tf / 2], [a, -CAR.tf / 2], [-b, CAR.tr / 2], [-b, -CAR.tr / 2]]; // 重心から見た位置(前方,左)
    let Fx = 0, Fy = 0, Mz = 0;
    for (let i = 0; i < 4; i++) {
      const [px, py] = P[i], d = i < 2 ? this.steer : 0, cd = Math.cos(d), sd = Math.sin(d);
      const g = grip(this.x + px * s + py * c, this.z + px * c - py * s);
      const vwx = this.vx - this.r * py, vwy = this.vy + this.r * px;      // タイヤ接地点の速度
      const vl = vwx * cd + vwy * sd, vt = -vwx * sd + vwy * cd;           // タイヤの向きに変換(前後/横)
      const vr = Math.max(Math.abs(vl), 3);
      const mu = 1.55 * g * Math.pow(Fz[i] / 3200, -0.1);                  // 荷重が増えるとグリップ係数は少し下がる
      const kap = (this.w[i] * R - vl) / vr;                               // スリップ率
      const sx = kap / 0.12, sy = (vt / vr) / 0.12, ss = Math.hypot(sx, sy) + 1e-6; // 12%/約7°でピーク
      const F = Fz[i] * mu * Math.sin(1.5 * Math.atan(1.732 * ss));        // 摩擦円(前後と横の合成)
      const fxs = F * sx / ss;
      const rr = Fz[i] * (g < 0.9 ? 0.07 : 0.015) * Math.tanh(vl * 2);    // 転がり抵抗(芝は大きい)
      const fxt = fxs - rr, fyt = -F * sy / ss;
      this.slip[i] = kap;
      // ホイールの回転(安定のため陰解法)
      const I = Iw + (i > 1 ? 0.5 * Ie * ratio * ratio : 0);
      const kk = dt * R * R * Fz[i] * mu * 21.65 / (vr * I);
      let wn = this.w[i] + dt * (Td[i] - R * fxs) / I / (1 + kk);
      let bt = brk * BRK[i];
      if (this.abs && kap < -0.13) bt *= clamp(1 + (kap + 0.13) * 12, 0.05, 1); // ABS
      const db = dt * bt / Iw;
      this.w[i] = Math.abs(wn) <= db ? 0 : wn - Math.sign(wn) * db;
      const fx = fxt * cd - fyt * sd, fy = fxt * sd + fyt * cd;            // 車体の向きに戻す
      Fx += fx; Fy += fy; Mz += px * fy - py * fx;
    }
    const v2 = Math.hypot(this.vx, this.vy);
    Fx -= 0.5 * 1.2 * 0.9 * this.vx * v2; Fy -= 0.5 * 1.2 * 0.9 * this.vy * v2; // 空気抵抗
    this.axf += (Fx / m - this.axf) * Math.min(1, dt * 15);
    this.ayf += (Fy / m - this.ayf) * Math.min(1, dt * 15);
    this.vx += dt * (Fx / m + this.r * this.vy);
    this.vy += dt * (Fy / m - this.r * this.vx);
    this.r += dt * Mz / Iz;
    this.yaw += this.r * dt;
    this.x += (this.vx * s + this.vy * c) * dt;
    this.z += (this.vx * c - this.vy * s) * dt;
    this.kmh = this.vx * 3.6;
  }
  // 壁に当たった時: (nx,nz)=コース内向きの単位ベクトル, pen=めり込み量
  bump(nx, nz, pen) {
    const s = Math.sin(this.yaw), c = Math.cos(this.yaw);
    this.x += nx * pen; this.z += nz * pen;
    let vx = this.vx * s + this.vy * c, vz = this.vx * c - this.vy * s;
    const vn = vx * nx + vz * nz;
    if (vn < 0) { vx -= 1.25 * vn * nx; vz -= 1.25 * vn * nz; vx *= 0.92; vz *= 0.92; this.r *= 0.7; }
    this.vx = vx * s + vz * c; this.vy = vx * c - vz * s;
  }
}

// コース中心線(等間隔の点の並び)から、車がコースのどこにいるかを調べる
export class Track {
  constructor(pts, hw) {
    this.p = pts; this.n = pts.length; this.hw = hw;
    this.t = pts.map((_, i) => { const a = pts[(i + this.n - 1) % this.n], b = pts[(i + 1) % this.n]; const dx = b.x - a.x, dz = b.z - a.z, l = Math.hypot(dx, dz); return { x: dx / l, z: dz / l }; });
  }
  // h=前回の位置(index)。その近くだけ探すので軽い。off=中心線からの横ズレ(左が+)
  near(x, z, h) {
    let bi = h, bd = 1e18;
    for (let k = -15; k <= 15; k++) {
      const i = (h + k + this.n) % this.n, dx = x - this.p[i].x, dz = z - this.p[i].z, d = dx * dx + dz * dz;
      if (d < bd) { bd = d; bi = i; }
    }
    const t = this.t[bi];
    return { i: bi, off: (x - this.p[bi].x) * t.z - (z - this.p[bi].z) * t.x };
  }
}

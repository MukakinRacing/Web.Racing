// physics.js — 車の動き(物理計算)とコースの判定。three.jsを使わない「計算だけ」のファイル
const G = 9.81;
export const CAR = { m: 1300, L: 2.78, wf: 0.44, h: 0.42, tf: 1.66, tr: 1.62, R: 0.34, Iw: 2.0, Iz: 2300, Ie: 0.2 };
CAR.b = CAR.wf * CAR.L; // 重心→後輪の距離
CAR.a = CAR.L - CAR.b;  // 重心→前輪の距離
const GEAR = [0, 13.0, 9.4, 7.3, 5.9, 4.8, 4.0]; // 総減速比(1〜6速)
const GEAR_R = -10;                               // バック(リバース)
const TQ = [[1000, .5], [3000, .8], [5000, .95], [6500, 1], [8000, .98], [9000, .88], [9400, .5]]; // エンジントルク(回転数, 相対値)
const TMAX = 470;                  // 最大トルク[Nm]
const BRK = [3200, 3200, 1500, 1500]; // ブレーキ最大トルク(左前,右前,左後,右後)。前寄りの配分
const SK = 0.12, SA = 0.12, SC = 1.5, SB = 1.732; // タイヤ特性: グリップ最大になるスリップ率/スリップ角, 曲線の形
const SLOPE = SB * SC / SK;       // 小さい滑りでの力の増え方(計算を安定させるために使う)
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
// タイヤ(コンパウンド)の性能。ソフト/ハード等を増やす時は、ここに足してCarのtireを切り替える
// mu: 基準荷重(4000N)での路面μ / loadExp: 荷重が増えた時にμが下がる度合い(0に近いほど下がりにくい)
export const TIRES = { medium: { name: 'ミディアム', mu: 1.65, loadExp: -0.15 } };
// 挙動の調整つまみ
//  LTD: 左右の荷重移動のうち前輪が受け持つ割合(大きいほどアンダー寄り) / LTL: 前後の荷重移動の強さ
//  FG,RG: 前輪/後輪タイヤのグリップ倍率(GT3は後輪が太い) / FSA: 前輪のグリップが最大になる滑り角の倍率(大きいほど前輪がじわっと効く=アンダー寄り)
//  STA: ステア補正で許す前輪の滑り / BF,BR: ブレーキが使ってよい前輪/後輪の摩擦円の大きさ
const LTD = 0.56, LTL = 0.8, FG = 1.0, RG = 1.06, FSA = 1.35, STA = 0.16, BF = 1.05, BR = 0.95;
function torque(rpm) {
  for (let i = 1; i < TQ.length; i++) {
    if (rpm <= TQ[i][0]) { const [r0, t0] = TQ[i - 1], [r1, t1] = TQ[i]; return t0 + (t1 - t0) * clamp((rpm - r0) / (r1 - r0), 0, 1); }
  }
  return 0;
}

// 向き(yaw)=θのとき、前方=(sinθ, cosθ)、左=(cosθ, -sinθ)  ※three.jsのx,z平面と同じ
export class Car {
  constructor() {
    // アシスト(オン/オフ): tc=TCS, abs=ABS, esc=横滑り防止, sta=ステア補正
    this.tc = true; this.abs = true; this.esc = false; this.sta = true;
    this.tire = TIRES.medium; this.reset(0, 0, 0);
  }
  reset(x, z, yaw) {
    Object.assign(this, { x, z, yaw, vx: 0, vy: 0, r: 0, w: [0, 0, 0, 0], gear: 1, rpm: 1500, steer: 0, thr: 0, brk: 0, hb: 0,
      axf: 0, ayf: 0, axl: 0, ayl: 0, shift: 0, slip: [0, 0, 0, 0], comb: [0, 0, 0, 0], sa: [0, 0, 0, 0], muw: [1.6, 1.6, 1.6, 1.6], escAct: 0, kmh: 0 });
  }
  // dt:秒 / inp:{steer(-1右〜+1左), thr(0〜1), brk(0〜1), hb(サイドブレーキ0〜1), rev(バック中ならtrue)}
  // grip(x,z):その場所の路面グリップ係数
  step(dt, inp, grip) {
    const { m, a, b, L, h, R, Iw, Iz, Ie } = CAR, s = Math.sin(this.yaw), c = Math.cos(this.yaw);
    const sp = Math.hypot(this.vx, this.vy), rev = !!inp.rev;
    this.thr += (inp.thr - this.thr) * Math.min(1, dt * 8);
    this.brk += (inp.brk - this.brk) * Math.min(1, dt * (inp.brk > this.brk ? 4 : 12)); // 踏む時はじわっと、離す時は素早く
    this.hb += ((inp.hb || 0) - this.hb) * Math.min(1, dt * 12);
    // ハンドルの角度(速度が上がるほど小さく)
    let st = inp.steer * 0.52 / (1 + (sp / 14) ** 2);
    if (this.sta && this.vx > 4) { const dv = Math.atan2(this.vy + a * this.r, this.vx); st = clamp(st, Math.min(dv - STA, 0), Math.max(dv + STA, 0)); } // ステア補正: 前輪が滑りすぎない角度まで(切る向きは変えない)
    this.steer = st;
    // アクセルを離している間: 停止中はブレーキ保持、後ろに転がっている時は自動でブレーキ(バックから止まれる)
    const brk = (inp.thr < 0.02 && (Math.abs(this.vx) < 1 || this.vx < -1)) ? Math.max(this.brk, this.vx < -1 ? 0.5 : 0.2) : this.brk;
    const lat = Math.min(1, Math.abs(this.ayf) / 14.7); // 横Gの大きさ(1.5Gで1)

    // --- エンジンと6速オートマ(+バック) ---
    if (rev && this.gear > 0) this.gear = -1; else if (!rev && this.gear < 0) this.gear = 1;
    this.shift -= dt;
    const ratio = rev ? GEAR_R : GEAR[this.gear], wd = (this.w[2] + this.w[3]) / 2;
    let rpm = Math.max(1500, Math.abs(wd) * Math.abs(ratio) * 9.549);
    if (!rev && this.shift <= 0) {
      if (rpm > 8600 && this.gear < 6) { this.gear++; this.shift = 0.2; }
      else if (rpm < 4000 && this.gear > 1) { this.gear--; this.shift = 0.2; }
    }
    if ((this.gear === 1 || rev) && this.thr > 0.05) rpm = Math.max(rpm, 4200); // 発進時はクラッチが滑る
    this.rpm += (rpm - this.rpm) * Math.min(1, dt * 20);
    let thr = this.thr * (rev ? 0.7 : 1);
    const cs = Math.max(this.comb[2], this.comb[3]); // 後輪の「前後+横」を合わせた滑り具合(1でグリップの限界)
    if (this.tc && cs > 0.95) thr *= clamp(1 - (cs - 0.95) * 2.5, 0.1, 1); // TCS(コーナー中の横滑りにも効く)
    if (this.esc) thr *= 1 - 0.95 * this.escAct; // ESCが働いている間はエンジン出力も絞る
    let Te = torque(rpm) * TMAX * thr - (1 - thr) * TMAX * 0.07 * (1 - 0.6 * lat) * rpm / 9000; // エンジンブレーキ(コーナー中は弱め)
    if (rev && Te > 0) Te *= clamp((7.5 + this.vx) / 2, 0, 1); // バックの速度は約27km/hまで
    if ((rpm > 9000 && Te > 0) || this.shift > 0) Te = 0;       // レブリミッター/変速中
    const Tw = Te * ratio * 0.92;
    const dc = clamp(20 * (this.w[2] - this.w[3]), -0.3 * Math.abs(Tw) - 150, 0.3 * Math.abs(Tw) + 150); // LSD
    const Td = [0, 0, Tw / 2 - dc, Tw / 2 + dc]; // 後輪駆動(RWD)

    // --- 4輪の接地荷重(ダウンフォースと荷重移動。荷重移動は少し遅れて効く=車の重さ) ---
    const down = 0.5 * 1.2 * 2.3 * this.vx * Math.abs(this.vx);
    const lt = LTL * m * this.axl * h / L;                              // 前後の荷重移動(GT3は足が硬くアンチダイブも効くので控えめ)
    const tfs = LTD * m * this.ayl * h / CAR.tf, trs = (1 - LTD) * m * this.ayl * h / CAR.tr; // 左右の荷重移動(前を多めに=アンダー寄り)
    const Ff = m * G * b / L / 2 + down * 0.21 - lt / 2, Fr = m * G * a / L / 2 + down * 0.29 + lt / 2;
    const Fz = [Ff - tfs, Ff + tfs, Fr - trs, Fr + trs].map(v => Math.max(v, 100));

    // --- ブレーキの上限(左右同じ圧力=左右同じトルク。旋回に使いたい分を引いた、残りの摩擦円までに制限) ---
    // 前輪は少しだけ超える(=適度なアンダー)、後輪は余裕を残す(=スピンしない)
    const lus = [0, 1, 2, 3].map(i => Math.sin(SC * Math.atan(SB * Math.abs(this.sa[i])))); // 横の滑りだけで使うグリップの割合(狙っている旋回の大きさ)
    const ayReq = this.vx > 0 ? Math.min(1.5, this.vx * this.vx * Math.abs(Math.tan(this.steer)) / L / G) : 0; // ハンドルから予想される横G(旋回の強さ)
    const bscale = this.abs ? 1 - 0.72 * ayReq / 1.5 : 1; // ABS(コーナリング対応): 旋回中はブレーキの強さも自動で弱める(最大で7割減)
    const cap = [0, 1, 2, 3].map(i => {
      const lu = lus[i];
      const kb = i < 2 ? Math.min(0.95, Math.sqrt(Math.max(0, BF * BF - lu * lu))) : Math.min(0.8, Math.sqrt(Math.max(0, BR * BR - lu * lu)));
      return kb * Fz[i] * this.muw[i] * R;
    });
    const capA = [(cap[0] + cap[1]) / 2, (cap[0] + cap[1]) / 2, (cap[2] + cap[3]) / 2, (cap[2] + cap[3]) / 2];
    // --- タイヤ4本ごとの力 ---
    const P = [[a, CAR.tf / 2], [a, -CAR.tf / 2], [-b, CAR.tr / 2], [-b, -CAR.tr / 2]]; // 重心から見た位置(前方,左)
    let Fx = 0, Fy = 0, Mz = 0;
    for (let i = 0; i < 4; i++) {
      const [px, py] = P[i], d = i < 2 ? this.steer : 0, cd = Math.cos(d), sd = Math.sin(d);
      const g = grip(this.x + px * s + py * c, this.z + px * c - py * s);
      const vwx = this.vx - this.r * py, vwy = this.vy + this.r * px;      // タイヤ接地点の速度
      const vl = vwx * cd + vwy * sd, vt = -vwx * sd + vwy * cd;           // タイヤの向きに変換(前後/横)
      const vr = Math.max(Math.abs(vl), 3);
      const mu = this.tire.mu * g * Math.pow(Fz[i] / 4000, this.tire.loadExp) * (i > 1 ? RG : FG); // タイヤのμ × 路面(芝は低い) × 荷重による変化(荷重が増えるとμは下がる)
      const kap = (this.w[i] * R - vl) / vr;                               // スリップ率
      this.sa[i] += ((vt / vr) / (SA * (i < 2 ? FSA : 1) * Math.pow(Fz[i] / 4000, 0.1)) - this.sa[i]) * Math.min(1, dt * vr / 0.3); // 横の滑りは少し遅れて効く(タイヤの緩和長0.3m=重量感)
      const sx = kap / SK, sy = this.sa[i], ss = Math.hypot(sx, sy) + 1e-6;
      const F = Fz[i] * mu * Math.sin(SC * Math.atan(SB * ss));            // 摩擦円(前後と横の合成)
      const fxs = F * sx / ss;
      const rr = Fz[i] * (g < 0.9 ? 0.07 : 0.015) * Math.tanh(vl * 2);    // 転がり抵抗(芝は大きい)
      const fxt = fxs - rr, fyt = -F * sy / ss;
      this.slip[i] = kap; this.comb[i] = ss;
      // ホイールの回転(安定のため陰解法)
      const I = Iw + (i > 1 ? 0.5 * Ie * ratio * ratio : 0);
      const kk = dt * R * R * Fz[i] * mu * SLOPE / (vr * I);
      const wn = this.w[i] + dt * (Td[i] - R * fxs) / I / (1 + kk);
      this.muw[i] = mu;
      let bt = Math.min(brk * BRK[i] * bscale, capA[i]);
      if (this.abs && kap < -0.13) bt *= clamp(1 + (kap + 0.13) * 12, 0.05, 1); // ABS
      if (i > 1) bt += this.hb * 6000;                                           // サイドブレーキ(後輪ロック)
      const db = dt * bt / Iw;
      this.w[i] = Math.abs(wn) <= db ? 0 : wn - Math.sign(wn) * db;
      const fx = fxt * cd - fyt * sd, fy = fxt * sd + fyt * cd;            // 車体の向きに戻す
      Fx += fx; Fy += fy; Mz += px * fy - py * fx;
    }
    // ESC(横滑り防止): ハンドルから予想される旋回より車が回りすぎた時、逆向きに抑える(+エンジン出力を絞る)
    this.escAct = 0;
    if (this.esc && this.vx > 6) {
      const rr = this.vx * Math.tan(this.steer) / L, ex = this.r - rr;
      if (ex * rr >= 0 && Math.abs(this.r) > Math.abs(rr)) { this.escAct = clamp(Math.abs(ex) / 0.1, 0, 1); Mz -= clamp(6000 * ex, -5000, 5000) * Math.min(1, this.vx / 15); }
    }
    const v2 = Math.hypot(this.vx, this.vy);
    Fx -= 0.5 * 1.2 * 0.9 * this.vx * v2; Fy -= 0.5 * 1.2 * 0.9 * this.vy * v2; // 空気抵抗
    this.axf += (Fx / m - this.axf) * Math.min(1, dt * 15); // 車体の傾き表示用(素早い)
    this.ayf += (Fy / m - this.ayf) * Math.min(1, dt * 15);
    this.axl += (Fx / m - this.axl) * Math.min(1, dt * 10);  // 荷重移動用(少し遅れる)
    this.ayl += (Fy / m - this.ayl) * Math.min(1, dt * 10);
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

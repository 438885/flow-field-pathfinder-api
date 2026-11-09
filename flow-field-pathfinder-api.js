// flow-field-pathfinder-api.js

//#region 1: Constants & Math Utilities
/** @constant {number} Chunk width/height in cells (16x16) */
export const CS = 16;

/** @constant {number} Total cells per chunk (256) */
export const CC = CS * CS;

/** @constant {number} Default open terrain traversal cost */
export const COST_OPEN = 2;

/** @constant {number} Impassable obstacle cost threshold (255) */
export const COST_BLOCK = 255;

const SQ2 = Math.SQRT2;
const OCT = SQ2 - 1;

/** 8-way neighbor offsets, step lengths, and opposite direction lookup */
export const DX = [1, -1, 0, 0, 1, -1, 1, -1];
export const DY = [0, 0, 1, -1, 1, 1, -1, -1];
export const DL = [1, 1, 1, 1, SQ2, SQ2, SQ2, SQ2];
export const REV_D = [1, 0, 3, 2, 7, 6, 5, 4];

/** 8-way directional bitmask flags for anisotropic one-way masks */
export const DIR_FLAGS = Object.freeze({
  E: 1 << 0,
  W: 1 << 1,
  S: 1 << 2,
  N: 1 << 3,
  SE: 1 << 4,
  SW: 1 << 5,
  NE: 1 << 6,
  NW: 1 << 7,
  ALL: 0xff
});

export const clamp = (v, min, max) => Math.min(Math.max(v, min), max);

export const octile = (dx, dy) => {
  const ax = Math.abs(dx),
    ay = Math.abs(dy);
  return ax > ay ? ax + OCT * ay : ay + OCT * ax;
};
//#endregion

//#region 2: Binary Min-Heap
/**
 * TypedArray-backed binary min-heap for Eikonal, Dijkstra, and A* searches.
 */
export class MinHeap {
  constructor(cap = 128) {
    this.it = new Int32Array(cap);
    this.k = new Float64Array(cap);
    this.sz = this.lastKey = 0;
  }
  get isEmpty() {
    return !this.sz;
  }
  clear() {
    this.sz = 0;
  }
  push(item, key) {
    let { it, k, sz } = this;
    if (sz === it.length) {
      const nSz = sz * 2;
      (this.it = new Int32Array(nSz)).set(it);
      (this.k = new Float64Array(nSz)).set(k);
      it = this.it;
      k = this.k;
    }
    let c = this.sz++,
      p;
    while (c > 0 && k[(p = (c - 1) >> 1)] > key) {
      it[c] = it[p];
      k[c] = k[p];
      c = p;
    }
    it[c] = item;
    k[c] = key;
  }
  pop() {
    let { it, k } = this,
      top = it[0],
      li = --this.sz,
      item = it[li],
      key = k[li],
      p = 0,
      ch;
    this.lastKey = k[0];
    while ((ch = p * 2 + 1) < li) {
      if (ch + 1 < li && k[ch + 1] < k[ch]) ch++;
      if (k[ch] >= key) break;
      it[p] = it[ch];
      k[p] = k[ch];
      p = ch;
    }
    it[p] = item;
    k[p] = key;
    return top;
  }
}
//#endregion

//#region 3: Multi-Layer Chunked Cost Grid
/**
 * 2D chunked cost grid supporting static costs, obstacle clearance (#5),
 * wall SDF repulsion (#4), tactical overlays (#6), velocity-predictive
 * crowd congestion (#9, D2), anisotropic directional masks & currents (#11),
 * two-tier topology vs. weight dirty tracking (B1), chunk-bounded incremental
 * SDF/clearance updates (B3), and world-space supercover LOS raycasting (A5).
 */
export class GridWorld {
  /**
   * @param {number} cx - Number of chunks horizontally.
   * @param {number} cy - Number of chunks vertically.
   * @param {Uint8Array} [existingCosts] - Optional backing cost buffer.
   */
  constructor(cx, cy, existingCosts = null) {
    this.cx = cx;
    this.cy = cy;
    this.width = cx * CS;
    this.height = cy * CS;
    this.chunkCount = cx * cy;
    const len = this.width * this.height;
    this.costs = existingCosts ? existingCosts.subarray(0, len) : new Uint8Array(len).fill(COST_OPEN);

    // B1: Two-Tier Dirty Tracking (Topology vs. Dynamic Weights)
    this.dirtyChunks = new Set();
    this.topologyDirtyChunks = new Set();
    this.weightDirtyChunks = new Set();

    // Solver & feature configuration
    this.useEikonal = true;
    this.losSmoothing = true;

    // #4 & B3: Wall Repulsion / Signed Distance Field (SDF) with incremental dirty bounds
    this.wallPenaltyRadius = 2.5;
    this.wallPenaltyWeight = 0.0;
    this.wallRepulsionWeight = 0.0;
    this.wallDist = null;
    this.wallGradX = null;
    this.wallGradY = null;
    this._sdfDirty = true;
    this._sdfFullDirty = true;
    this._sdfDirtyChunks = new Set();

    // #5 & B3: Clearance-Annotated Routing (unit size in cells) with incremental dirty bounds
    this.clearance = null;
    this._clearanceDirty = true;
    this._clearanceFullDirty = true;
    this._clearanceDirtyChunks = new Set();

    // #6: Tactical / Threat-Modulated Overlay
    this.tacticalCosts = null;
    this.hasTactical = false;

    // #9 & D2: Continuum Crowd Congestion Layer + Velocity-Predictive Counter-Flow
    this.congestion = null;
    this.congestionVX = null;
    this.congestionVY = null;
    this.congestionWeight = 1.0;
    this.congestionAnisotropy = 0.65;
    this.hasCongestion = false;
    this.hasCongestionVelocity = false;
    this._activeCongestionChunks = new Set();

    // #11: Anisotropic Directional Bitmask & Vector Currents / Slopes
    this.dirMask = null;
    this.currentX = null;
    this.currentY = null;
    this.currentStrength = 0.5;
    this.hasAnisotropy = false;
  }

  index(x, y) { return y * this.width + x; }
  inBounds(x, y) { return x >= 0 && y >= 0 && x < this.width && y < this.height; }
  isWalkable(x, y, minClearance = 1) {
    if (!this.inBounds(x, y)) return false;
    const idx = this.index(x, y);
    if (this.costs[idx] >= COST_BLOCK) return false;
    if (minClearance > 1) {
      if (this._clearanceDirty || !this.clearance) this.updateClearance();
      return this.clearance[idx] >= minClearance;
    }
    return true;
  }
  chunkIdAt(x, y) { return (y / CS | 0) * this.cx + (x / CS | 0); }
  chunkOriginX(c) { return (c % this.cx) * CS; }
  chunkOriginY(c) { return (c / this.cx | 0) * CS; }
  localIndex(cell) { return ((cell / this.width | 0) % CS) * CS + (cell % this.width % CS); }

  _markChunkSet(set, x, y) {
    const c = this.chunkIdAt(x, y);
    set.add(c);
    const lx = x % CS, ly = y % CS;
    if (lx === 0 && x > 0) set.add(c - 1);
    if (lx === CS - 1 && x < this.width - 1) set.add(c + 1);
    if (ly === 0 && y > 0) set.add(c - this.cx);
    if (ly === CS - 1 && y < this.height - 1) set.add(c + this.cx);
  }

  markCellDirty(x, y, isTopology = true) {
    this._markChunkSet(this.dirtyChunks, x, y);
    if (isTopology) {
      this._markChunkSet(this.topologyDirtyChunks, x, y);
      this._sdfDirtyChunks.add(this.chunkIdAt(x, y));
      this._clearanceDirtyChunks.add(this.chunkIdAt(x, y));
      this._sdfDirty = true;
      this._clearanceDirty = true;
    } else {
      this._markChunkSet(this.weightDirtyChunks, x, y);
    }
  }

  markChunkWeightDirty(c) {
    this.dirtyChunks.add(c);
    this.weightDirtyChunks.add(c);
  }

  getCost(x, y) {
    return this.inBounds(x | 0, y | 0) ? this.costs[(y | 0) * this.width + (x | 0)] : COST_BLOCK;
  }

  setCost(x, y, cost) {
    const ix = x | 0, iy = y | 0;
    if (!this.inBounds(ix, iy)) return false;
    const idx = iy * this.width + ix;
    const prev = this.costs[idx];
    if (prev === cost) return false;
    this.costs[idx] = cost;
    const isTopo = (prev >= COST_BLOCK) !== (cost >= COST_BLOCK);
    this.markCellDirty(ix, iy, isTopo);
    return true;
  }

  setRect(x, y, w, h, cost) {
    const x0 = clamp(Math.floor(x), 0, this.width), x1 = clamp(Math.ceil(x + w), 0, this.width);
    const y0 = clamp(Math.floor(y), 0, this.height), y1 = clamp(Math.ceil(y + h), 0, this.height);
    let changed = 0;
    for (let yy = y0; yy < y1; yy++) {
      for (let xx = x0; xx < x1; xx++) {
        if (this.setCost(xx, yy, cost)) changed++;
      }
    }
    return changed;
  }

  setDisc(wx, wy, r, cost) {
    const r2 = r * r;
    const x0 = clamp(Math.floor(wx - r), 0, this.width - 1), x1 = clamp(Math.ceil(wx + r), 0, this.width - 1);
    const y0 = clamp(Math.floor(wy - r), 0, this.height - 1), y1 = clamp(Math.ceil(wy + r), 0, this.height - 1);
    let changed = 0;
    for (let y = y0; y <= y1; y++) {
      for (let x = x0; x <= x1; x++) {
        if ((x + 0.5 - wx) ** 2 + (y + 0.5 - wy) ** 2 <= r2) {
          if (this.setCost(x, y, cost)) changed++;
        }
      }
    }
    return changed;
  }

  loadCosts(srcCosts) {
    const len = Math.min(this.costs.length, srcCosts.length);
    for (let i = 0; i < len; i++) this.costs[i] = srcCosts[i];
    for (let c = 0; c < this.chunkCount; c++) {
      this.dirtyChunks.add(c);
      this.topologyDirtyChunks.add(c);
    }
    this._sdfDirty = true;
    this._sdfFullDirty = true;
    this._clearanceDirty = true;
    this._clearanceFullDirty = true;
  }

  consumeDirtyChunks() {
    const out = new Set(this.dirtyChunks);
    this.dirtyChunks.clear();
    this.topologyDirtyChunks.clear();
    this.weightDirtyChunks.clear();
    return out;
  }

  /**
   * B1: Consumes and separates topology-changing chunks from weight-only chunks.
   */
  consumeDirtyState() {
    const all = new Set(this.dirtyChunks);
    const topology = new Set(this.topologyDirtyChunks);
    const weights = new Set(this.weightDirtyChunks);
    for (const c of topology) weights.delete(c);
    this.dirtyChunks.clear();
    this.topologyDirtyChunks.clear();
    this.weightDirtyChunks.clear();
    return { all, topology, weights };
  }

  _computeDirtyBounds(chunkSet, haloCells) {
    let minCX = this.cx, minCY = this.cy, maxCX = -1, maxCY = -1;
    for (const c of chunkSet) {
      const cx = c % this.cx, cy = (c / this.cx) | 0;
      if (cx < minCX) minCX = cx;
      if (cx > maxCX) maxCX = cx;
      if (cy < minCY) minCY = cy;
      if (cy > maxCY) maxCY = cy;
    }
    if (maxCX < 0) return null;
    return {
      x0: Math.max(0, minCX * CS - haloCells),
      y0: Math.max(0, minCY * CS - haloCells),
      x1: Math.min(this.width - 1, (maxCX + 1) * CS - 1 + haloCells),
      y1: Math.min(this.height - 1, (maxCY + 1) * CS - 1 + haloCells)
    };
  }

  // #4 & B3: Wall Repulsion / Signed Distance Field (Full or Chunk-Bounded Incremental EDT)
  enableWallRepulsion({ penaltyWeight = 1.5, repulsionWeight = 0.35, radius = 2.5 } = {}) {
    this.wallPenaltyWeight = penaltyWeight;
    this.wallRepulsionWeight = repulsionWeight;
    this.wallPenaltyRadius = radius;
    this.updateWallSDF(true);
  }

  updateWallSDF(forceFull = false) {
    const { width: W, height: H, costs } = this, len = W * H;
    if (!this.wallDist) {
      this.wallDist = new Float32Array(len);
      this.wallGradX = new Float32Array(len);
      this.wallGradY = new Float32Array(len);
      this._sdfFullDirty = true;
    }
    const D = this.wallDist, GX = this.wallGradX, GY = this.wallGradY;
    const INF = 1e6;
    const halo = Math.ceil(this.wallPenaltyRadius) + CS;
    const isFull = forceFull || this._sdfFullDirty || this._sdfDirtyChunks.size === 0 || this._sdfDirtyChunks.size * 4 >= this.chunkCount;
    const bounds = isFull ? { x0: 0, y0: 0, x1: W - 1, y1: H - 1 } : this._computeDirtyBounds(this._sdfDirtyChunks, halo);
    const { x0, y0, x1, y1 } = bounds;

    for (let y = y0; y <= y1; y++) {
      const row = y * W;
      const isBorderY = !isFull && ((y === y0 && y0 > 0) || (y === y1 && y1 < H - 1));
      for (let x = x0; x <= x1; x++) {
        const i = row + x;
        if (costs[i] >= COST_BLOCK) {
          D[i] = 0;
        } else if (!isBorderY && (isFull || !((x === x0 && x0 > 0) || (x === x1 && x1 < W - 1)))) {
          D[i] = INF;
        }
      }
    }

    // Forward pass
    for (let y = y0; y <= y1; y++) {
      const row = y * W;
      for (let x = x0; x <= x1; x++) {
        const i = row + x;
        let d = D[i];
        if (d === 0) continue;
        if (x > x0 && D[i - 1] + 1 < d) d = D[i - 1] + 1;
        if (y > y0 && D[i - W] + 1 < d) d = D[i - W] + 1;
        if (x > x0 && y > y0 && D[i - W - 1] + SQ2 < d) d = D[i - W - 1] + SQ2;
        if (x < x1 && y > y0 && D[i - W + 1] + SQ2 < d) d = D[i - W + 1] + SQ2;
        D[i] = d;
      }
    }
    // Backward pass
    for (let y = y1; y >= y0; y--) {
      const row = y * W;
      for (let x = x1; x >= x0; x--) {
        const i = row + x;
        let d = D[i];
        if (d === 0) continue;
        if (x < x1 && D[i + 1] + 1 < d) d = D[i + 1] + 1;
        if (y < y1 && D[i + W] + 1 < d) d = D[i + W] + 1;
        if (x < x1 && y < y1 && D[i + W + 1] + SQ2 < d) d = D[i + W + 1] + SQ2;
        if (x > x0 && y < y1 && D[i + W - 1] + SQ2 < d) d = D[i + W - 1] + SQ2;
        D[i] = d;
      }
    }
    // Normalized outward wall gradient
    for (let y = y0; y <= y1; y++) {
      const row = y * W;
      for (let x = x0; x <= x1; x++) {
        const i = row + x;
        const dE = D[row + Math.min(W - 1, x + 1)], dW = D[row + Math.max(0, x - 1)];
        const dS = D[Math.min(H - 1, y + 1) * W + x], dN = D[Math.max(0, y - 1) * W + x];
        const gx = dE - dW, gy = dS - dN, l = Math.hypot(gx, gy);
        GX[i] = l > 1e-5 ? gx / l : 0;
        GY[i] = l > 1e-5 ? gy / l : 0;
      }
    }
    this._sdfDirty = false;
    this._sdfFullDirty = false;
    this._sdfDirtyChunks.clear();
  }

  // #5 & B3: Clearance Map (Full or Chunk-Bounded Incremental Chebyshev Brushfire)
  updateClearance(forceFull = false) {
    const { width: W, height: H, costs } = this, len = W * H;
    if (!this.clearance) {
      this.clearance = new Uint8Array(len);
      this._clearanceFullDirty = true;
    }
    const C = this.clearance;
    const MAX_CLR = CS;
    const isFull = forceFull || this._clearanceFullDirty || this._clearanceDirtyChunks.size === 0 || this._clearanceDirtyChunks.size * 4 >= this.chunkCount;
    const bounds = isFull ? { x0: 0, y0: 0, x1: W - 1, y1: H - 1 } : this._computeDirtyBounds(this._clearanceDirtyChunks, MAX_CLR + 2);
    const { x0, y0, x1, y1 } = bounds;

    for (let y = y0; y <= y1; y++) {
      const row = y * W;
      const isOuterY = !isFull && ((y === y0 && y0 > 0) || (y === y1 && y1 < H - 1));
      for (let x = x0; x <= x1; x++) {
        const i = row + x;
        if (costs[i] >= COST_BLOCK) {
          C[i] = 0;
        } else if (x === 0 || y === 0 || x === W - 1 || y === H - 1) {
          C[i] = 1;
        } else if (!isOuterY && (isFull || !((x === x0 && x0 > 0) || (x === x1 && x1 < W - 1)))) {
          C[i] = MAX_CLR;
        }
      }
    }
    const fy0 = Math.max(1, y0), fy1 = Math.min(H - 2, y1);
    const fx0 = Math.max(1, x0), fx1 = Math.min(W - 2, x1);
    for (let y = fy0; y <= fy1; y++) {
      const row = y * W;
      for (let x = fx0; x <= fx1; x++) {
        const i = row + x;
        if (!C[i]) continue;
        const m = Math.min(C[i - 1], C[i - W], C[i - W - 1], C[i - W + 1]);
        if (m + 1 < C[i]) C[i] = m + 1;
      }
    }
    for (let y = fy1; y >= fy0; y--) {
      const row = y * W;
      for (let x = fx1; x >= fx0; x--) {
        const i = row + x;
        if (!C[i]) continue;
        const m = Math.min(C[i + 1], C[i + W], C[i + W + 1], C[i + W - 1]);
        if (m + 1 < C[i]) C[i] = m + 1;
      }
    }
    this._clearanceDirty = false;
    this._clearanceFullDirty = false;
    this._clearanceDirtyChunks.clear();
  }

  // #6: Tactical / Threat-Modulated Overlay (B1: Marks Weight-Dirty Only)
  ensureTacticalLayer() {
    if (!this.tacticalCosts) this.tacticalCosts = new Float32Array(this.width * this.height);
    this.hasTactical = true;
    return this.tacticalCosts;
  }

  setTacticalCost(x, y, extraCost) {
    const ix = x | 0, iy = y | 0;
    if (!this.inBounds(ix, iy)) return;
    this.ensureTacticalLayer()[iy * this.width + ix] = Math.max(0, extraCost);
    this.markChunkWeightDirty(this.chunkIdAt(ix, iy));
  }

  addTacticalDisc(wx, wy, r, peakCost, linearFalloff = true) {
    const T = this.ensureTacticalLayer(), r2 = r * r;
    const x0 = clamp(Math.floor(wx - r), 0, this.width - 1), x1 = clamp(Math.ceil(wx + r), 0, this.width - 1);
    const y0 = clamp(Math.floor(wy - r), 0, this.height - 1), y1 = clamp(Math.ceil(wy + r), 0, this.height - 1);
    for (let y = y0; y <= y1; y++) {
      for (let x = x0; x <= x1; x++) {
        const d2 = (x + 0.5 - wx) ** 2 + (y + 0.5 - wy) ** 2;
        if (d2 > r2) continue;
        const w = linearFalloff ? 1 - Math.sqrt(d2) / r : 1;
        T[y * this.width + x] += peakCost * w;
        this.markChunkWeightDirty(this.chunkIdAt(x, y));
      }
    }
  }

  clearTacticalCosts() {
    if (this.tacticalCosts) this.tacticalCosts.fill(0);
    this.hasTactical = false;
    for (let c = 0; c < this.chunkCount; c++) this.markChunkWeightDirty(c);
  }

  // #9 & D2: Continuum Crowd Congestion Layer + Velocity-Predictive Counter-Flow
  ensureCongestionLayer() {
    const len = this.width * this.height;
    if (!this.congestion) {
      this.congestion = new Float32Array(len);
      this.congestionVX = new Float32Array(len);
      this.congestionVY = new Float32Array(len);
    }
    this.hasCongestion = true;
    return this.congestion;
  }

  addCongestion(wx, wy, amount = 1.0, vx = 0, vy = 0) {
    const ix = wx | 0, iy = wy | 0;
    if (!this.inBounds(ix, iy)) return;
    this.ensureCongestionLayer();
    const idx = iy * this.width + ix;
    this.congestion[idx] += amount;
    if (vx !== 0 || vy !== 0) {
      this.congestionVX[idx] += vx * amount;
      this.congestionVY[idx] += vy * amount;
      this.hasCongestionVelocity = true;
    }
    const c = this.chunkIdAt(ix, iy);
    this._activeCongestionChunks.add(c);
    this.markChunkWeightDirty(c);
  }

  /**
   * #9 & D2: Splats agent density and optional velocity vectors `(vx, vy)` onto the congestion grid.
   * When `vx` and `vy` are provided, opposing crowds form natural counter-flow lanes.
   */
  setCongestionFromAgents(px, py, count, unitWeight = 1.0, vx = null, vy = null) {
    const C = this.ensureCongestionLayer(), { width: W, height: H, congestionVX: CVX, congestionVY: CVY } = this;
    C.fill(0);
    CVX.fill(0);
    CVY.fill(0);
    for (const c of this._activeCongestionChunks) this.markChunkWeightDirty(c);
    this._activeCongestionChunks.clear();
    const hasVel = !!(vx && vy);
    this.hasCongestionVelocity = hasVel;

    for (let i = 0; i < count; i++) {
      const x = px[i] | 0, y = py[i] | 0;
      if (x >= 0 && x < W && y >= 0 && y < H) {
        const idx = y * W + x;
        C[idx] += unitWeight;
        if (hasVel) {
          const avx = vx[i], avy = vy[i], sp = Math.hypot(avx, avy);
          if (sp > 1e-4) {
            CVX[idx] += (avx / sp) * unitWeight;
            CVY[idx] += (avy / sp) * unitWeight;
          }
        }
        const c = this.chunkIdAt(x, y);
        this._activeCongestionChunks.add(c);
        this.markChunkWeightDirty(c);
      }
    }
  }

  decayCongestion(decayFactor = 0.85, eps = 0.05) {
    if (!this.congestion || !this.hasCongestion) return;
    const C = this.congestion, CVX = this.congestionVX, CVY = this.congestionVY, W = this.width;
    let active = false, activeVel = false;
    this._activeCongestionChunks.clear();
    for (let i = 0; i < C.length; i++) {
      if (C[i] > 0) {
        C[i] *= decayFactor;
        CVX[i] *= decayFactor;
        CVY[i] *= decayFactor;
        const c = this.chunkIdAt(i % W, (i / W) | 0);
        this.markChunkWeightDirty(c);
        if (C[i] < eps) {
          C[i] = 0;
          CVX[i] = 0;
          CVY[i] = 0;
        } else {
          active = true;
          this._activeCongestionChunks.add(c);
          if (CVX[i] !== 0 || CVY[i] !== 0) activeVel = true;
        }
      }
    }
    this.hasCongestion = active;
    this.hasCongestionVelocity = activeVel;
  }

  // #11: Anisotropic Directional Passability & Currents / Slopes
  ensureAnisotropicLayer() {
    const len = this.width * this.height;
    if (!this.dirMask) this.dirMask = new Uint8Array(len).fill(DIR_FLAGS.ALL);
    if (!this.currentX) {
      this.currentX = new Float32Array(len);
      this.currentY = new Float32Array(len);
    }
    this.hasAnisotropy = true;
  }

  setDirectionMask(x, y, mask = DIR_FLAGS.ALL) {
    const ix = x | 0, iy = y | 0;
    if (!this.inBounds(ix, iy)) return;
    this.ensureAnisotropicLayer();
    this.dirMask[iy * this.width + ix] = mask & 0xFF;
    this.markCellDirty(ix, iy, true);
  }

  setCurrent(x, y, vx, vy) {
    const ix = x | 0, iy = y | 0;
    if (!this.inBounds(ix, iy)) return;
    this.ensureAnisotropicLayer();
    const idx = iy * this.width + ix;
    this.currentX[idx] = vx;
    this.currentY[idx] = vy;
    this.markCellDirty(ix, iy, false);
  }

  setRectCurrent(x, y, w, h, vx, vy) {
    this.ensureAnisotropicLayer();
    const x0 = clamp(Math.floor(x), 0, this.width), x1 = clamp(Math.ceil(x + w), 0, this.width);
    const y0 = clamp(Math.floor(y), 0, this.height), y1 = clamp(Math.ceil(y + h), 0, this.height);
    for (let yy = y0; yy < y1; yy++) {
      for (let xx = x0; xx < x1; xx++) {
        const idx = yy * this.width + xx;
        this.currentX[idx] = vx;
        this.currentY[idx] = vy;
        this.markCellDirty(xx, yy, false);
      }
    }
  }

  setSlopeFromHeightmap(heights, slopeScale = 1.0) {
    this.ensureAnisotropicLayer();
    const { width: W, height: H, currentX: CX, currentY: CY } = this;
    for (let y = 0; y < H; y++) {
      for (let x = 0; x < W; x++) {
        const i = y * W + x;
        const hE = heights[y * W + Math.min(W - 1, x + 1)], hW = heights[y * W + Math.max(0, x - 1)];
        const hS = heights[Math.min(H - 1, y + 1) * W + x], hN = heights[Math.max(0, y - 1) * W + x];
        CX[i] = -(hE - hW) * 0.5 * slopeScale;
        CY[i] = -(hS - hN) * 0.5 * slopeScale;
      }
    }
    for (let c = 0; c < this.chunkCount; c++) this.markChunkWeightDirty(c);
  }

  /**
   * A5: Fast world-space supercover Line-of-Sight raycast across chunks.
   * Verifies walkability, clearance, terrain cost uniformity, and optional wall buffer.
   */
  hasWorldLOS(x0, y0, x1, y1, maxCost = COST_OPEN, minClearance = 1) {
    const { width: W, height: H, costs } = this;
    let cx = x0 | 0, cy = y0 | 0;
    const ex = x1 | 0, ey = y1 | 0;
    if (cx < 0 || cy < 0 || cx >= W || cy >= H || ex < 0 || ey < 0 || ex >= W || ey >= H) return false;
    if (minClearance > 1 && (this._clearanceDirty || !this.clearance)) this.updateClearance();
    const useWall = this.wallPenaltyWeight > 0;
    if (useWall && (this._sdfDirty || !this.wallDist)) this.updateWallSDF();
    const minWallDist = useWall ? Math.min(1.25, this.wallPenaltyRadius * 0.5) : 0;

    const checkCell = (x, y) => {
      const idx = y * W + x;
      const c = costs[idx];
      if (c >= COST_BLOCK || c > maxCost) return false;
      if (minClearance > 1 && this.clearance[idx] < minClearance) return false;
      if (useWall && this.wallDist[idx] < minWallDist) return false;
      if (this.hasTactical && this.tacticalCosts[idx] > 0.5) return false;
      if (this.hasCongestion && this.congestion[idx] * this.congestionWeight > 1.0) return false;
      if (this.hasAnisotropy && this.dirMask[idx] !== 0xFF) return false;
      return true;
    };

    const dx = Math.abs(ex - cx), dy = Math.abs(ey - cy);
    const sx = cx < ex ? 1 : -1, sy = cy < ey ? 1 : -1;
    let err = dx - dy;
    while (true) {
      if (!checkCell(cx, cy)) return false;
      if (cx === ex && cy === ey) return true;
      const e2 = 2 * err;
      if (e2 > -dy && e2 < dx) {
        if (!checkCell(cx + sx, cy) || !checkCell(cx, cy + sy)) return false;
      }
      if (e2 > -dy) { err -= dy; cx += sx; }
      if (e2 < dx) { err += dx; cy += sy; }
    }
  }

  findNearestWalkable(x, y, maxR = 24, minClearance = 1) {
    const { width: W, height: H } = this, sx = clamp(x | 0, 0, W - 1), sy = clamp(y | 0, 0, H - 1);
    if (this.isWalkable(sx, sy, minClearance)) return sy * W + sx;
    let bestD = Infinity, bestI = -1;
    for (let r = 1, lim = Math.min(maxR, Math.max(W, H)); r <= lim; r++) {
      const r2 = r * r;
      for (let dy = -r; dy <= r; dy++) {
        const ny = sy + dy;
        if (ny < 0 || ny >= H) continue;
        for (let dx = -r; dx <= r; dx++) {
          if (dx * dx < r2 && dy * dy < r2) continue;
          const nx = sx + dx;
          if (nx >= 0 && nx < W) {
            const idx = ny * W + nx, d = dx * dx + dy * dy;
            if (this.isWalkable(nx, ny, minClearance) && d < bestD) { bestD = d; bestI = idx; }
          }
        }
      }
      if (bestI >= 0) return bestI;
    }
    return -1;
  }
}
//#endregion

//#region 4: Portal Detection & Multi-Mode Chunk Integrator
/**
 * A3: Detects chunk boundary portals and splits wide open runs (> `maxPortalWidth`)
 * into evenly sized sub-portals to prevent midpoint macro path distortion.
 */
export function detectPortals(grid, minClearance = 1, maxPortalWidth = 8) {
  const portals = [], { cx, cy, width } = grid;
  const maxW = Math.max(2, maxPortalWidth | 0);
  for (const isH of [false, true]) {
    const countX = cx - !isH, countY = cy - isH, bOffset = isH ? cy * (cx - 1) : 0;
    const W = (u, v) => grid.isWalkable(isH ? u : v, isH ? v : u, minClearance);
    const at = (u, v) => isH ? v * width + u : u * width + v;
    for (let y = 0; y < countY; y++) {
      for (let x = 0; x < countX; x++) {
        const border = ((isH ? y : x) + 1) * CS, origin = (isH ? x : y) * CS;
        const cA = y * cx + x, cB = cA + (isH ? cx : 1), bIdx = bOffset + y * countX + x;
        let run = -1;
        for (let t = 0; t <= CS; t++) {
          if (t < CS && W(origin + t, border - 1) && W(origin + t, border)) {
            if (run < 0) run = t;
          } else if (run >= 0) {
            const totalLen = t - run;
            const numSegs = Math.ceil(totalLen / maxW);
            for (let s = 0; s < numSegs; s++) {
              const subStartOffset = run + Math.floor((s * totalLen) / numSegs);
              const subEndOffset = run + Math.floor(((s + 1) * totalLen) / numSegs);
              const len = subEndOffset - subStartOffset;
              if (len <= 0) continue;
              const start = origin + subStartOffset, mid = (len - 1) >> 1, cMid = start + mid;
              const cellsA = new Int32Array(len), cellsB = new Int32Array(len);
              for (let k = 0; k < len; k++) {
                cellsA[k] = at(start + k, border - 1);
                cellsB[k] = at(start + k, border);
              }
              portals.push({
                id: portals.length, uid: (bIdx * CS + subStartOffset) * (CS + 1) + len,
                isH, chunkA: cA, chunkB: cB, border, start, end: start + len - 1,
                cellsA, cellsB, centerA: cellsA[mid], centerB: cellsB[mid],
                anchorX: isH ? cMid + 0.5 : border, anchorY: isH ? border : cMid + 0.5
              });
            }
            run = -1;
          }
        }
      }
    }
  }
  return portals;
}

export const portalSideCells = (p, c) => c === p.chunkA ? p.cellsA : p.cellsB;
export const portalCenterCell = (p, c) => c === p.chunkA ? p.centerA : p.centerB;
export const portalExitX = (p, c) => p.isH ? 0 : c === p.chunkA ? 1 : -1;
export const portalExitY = (p, c) => p.isH ? c === p.chunkA ? 1 : -1 : 0;
export const crossingCost = (grid, p) => ((grid.costs[p.centerA] + grid.costs[p.centerB]) * 0.5) / COST_OPEN;
export const pLocal = (grid, p, c) => grid.localIndex(portalCenterCell(p, c));

/**
 * Multi-mode chunk integrator supporting:
 * - #1 & A4: Generalized Anisotropic Upwind Eikonal quadratic solver
 * - #2: Multi-source geodesic Voronoi (`sourceOut`)
 * - #4: Wall SDF penalty buffer
 * - #5: Clearance filtering (`minClearance`)
 * - #6: Tactical cost overlays
 * - #8: 2-pass Flee / Evasion safety integration (`integrateFlee`)
 * - #9 & D2: Continuum crowd congestion weights & velocity counter-flow currents
 * - #11: Anisotropic directional masks & vector currents
 */
export class ChunkIntegrator {
  heap = new MinHeap(CC * 2);
  costs = new Float32Array(CC);
  blocked = new Uint8Array(CC);
  dirMasks = new Uint8Array(CC);
  curX = new Float32Array(CC);
  curY = new Float32Array(CC);
  dist = new Float64Array(CC);
  fleeBuf = new Float32Array(CC);
  hasChunkAniso = false;

  /**
   * Extracts effective local chunk costs, passability masks, and crowd velocity currents from `grid`.
   */
  prepareChunk(grid, chunkId, minClearance = 1) {
    const { costs, blocked, dirMasks, curX, curY } = this;
    const ox = grid.chunkOriginX(chunkId), oy = grid.chunkOriginY(chunkId), W = grid.width;
    const useClearance = minClearance > 1;
    if (useClearance && (grid._clearanceDirty || !grid.clearance)) grid.updateClearance();
    const useWall = grid.wallPenaltyWeight > 0;
    if (useWall && (grid._sdfDirty || !grid.wallDist)) grid.updateWallSDF();

    const hasTac = grid.hasTactical && grid.tacticalCosts;
    const hasCong = grid.hasCongestion && grid.congestion;
    const hasCongVel = hasCong && grid.hasCongestionVelocity && grid.congestionVX;
    const hasAniso = grid.hasAnisotropy && grid.dirMask;
    const wallR = grid.wallPenaltyRadius, wallW = grid.wallPenaltyWeight;
    const congAniso = grid.congestionAnisotropy;
    let anyAniso = hasAniso;

    for (let ly = 0; ly < CS; ly++) {
      const gy = oy + ly, row = gy * W + ox, lRow = ly * CS;
      for (let lx = 0; lx < CS; lx++) {
        const gIdx = row + lx, lIdx = lRow + lx;
        const baseC = grid.costs[gIdx];
        if (baseC >= COST_BLOCK || (useClearance && grid.clearance[gIdx] < minClearance)) {
          blocked[lIdx] = 1;
          costs[lIdx] = COST_BLOCK;
          continue;
        }
        blocked[lIdx] = 0;
        let eff = baseC;
        if (hasTac) eff += grid.tacticalCosts[gIdx];
        let cDens = 0;
        if (hasCong) {
          cDens = grid.congestion[gIdx];
          eff += cDens * grid.congestionWeight;
        }
        if (useWall) {
          const wd = grid.wallDist[gIdx];
          if (wd < wallR) eff += wallW * (1 - wd / wallR);
        }
        costs[lIdx] = Math.max(0.25, eff);

        let cx = 0, cy = 0;
        if (hasAniso) {
          dirMasks[lIdx] = grid.dirMask[gIdx];
          cx = grid.currentX[gIdx];
          cy = grid.currentY[gIdx];
        } else {
          dirMasks[lIdx] = 0xFF;
        }
        // D2: Blend crowd momentum vector into local current for counter-flow lane formation
        if (hasCongVel && cDens > 1e-3) {
          const mvx = (grid.congestionVX[gIdx] / cDens) * congAniso;
          const mvy = (grid.congestionVY[gIdx] / cDens) * congAniso;
          if (mvx !== 0 || mvy !== 0) {
            cx += mvx;
            cy += mvy;
            anyAniso = true;
          }
        }
        curX[lIdx] = cx;
        curY[lIdx] = cy;
      }
    }
    this.hasChunkAniso = anyAniso;
  }

  /**
   * Integrates a 16x16 chunk from `sources`.
   * @param {GridWorld} grid
   * @param {number} chunkId
   * @param {ArrayLike<number|{cell:number, weight?:number, id?:number}>} sources
   * @param {Float32Array|Float64Array} out
   * @param {Object} [opts={}] - `{ minClearance, sourceOut, sourceWeights, eikonal, forward }`
   */
  integrate(grid, chunkId, sources, out, opts = {}) {
    const minClearance = opts.minClearance ?? 1;
    const useEikonal = opts.eikonal ?? grid.useEikonal;
    const forward = !!opts.forward;
    const sourceOut = opts.sourceOut || null;
    const sourceWeights = opts.sourceWeights || null;

    this.prepareChunk(grid, chunkId, minClearance);
    const { costs, blocked, dirMasks, curX, curY, dist, heap, hasChunkAniso } = this;
    const curStr = grid.currentStrength;

    dist.fill(Infinity);
    if (sourceOut) sourceOut.fill(-1);
    heap.clear();

    for (let i = 0; i < sources.length; i++) {
      const s = sources[i];
      const cell = typeof s === 'object' ? s.cell : s;
      const w = typeof s === 'object' ? (s.weight ?? 0) : (sourceWeights ? sourceWeights[i] : 0);
      const sid = typeof s === 'object' ? (s.id ?? i) : i;
      if (cell >= 0 && cell < CC && !blocked[cell] && w < dist[cell]) {
        heap.push(cell, dist[cell] = w);
        if (sourceOut) sourceOut[cell] = sid;
      }
    }

    while (!heap.isEmpty) {
      const u = heap.pop(), du = heap.lastKey;
      if (du > dist[u]) continue;
      const ux = u % CS, uy = (u / CS) | 0, cu = costs[u];

      for (let d = 0; d < 8; d++) {
        const nx = ux + DX[d], ny = uy + DY[d];
        if (nx >>> 0 >= CS || ny >>> 0 >= CS) continue;
        const v = ny * CS + nx;
        if (blocked[v]) continue;
        if (d >= 4 && (blocked[uy * CS + nx] || blocked[ny * CS + ux])) continue;

        const baseScale = 0.25 * (cu + costs[v]);
        let stepScale = baseScale;
        if (hasChunkAniso) {
          const travelDir = forward ? d : REV_D[d];
          const fromCell = forward ? u : v;
          if ((dirMasks[fromCell] & (1 << travelDir)) === 0) continue;
          const cx = curX[fromCell], cy = curY[fromCell];
          if (cx !== 0 || cy !== 0) {
            const dot = (DX[travelDir] * cx + DY[travelDir] * cy) / DL[d];
            stepScale *= clamp(1 - dot * curStr, 0.2, 4.0);
          }
        }

        let nd = du + DL[d] * stepScale;

        // #1 & A4: Generalized Anisotropic Upwind Eikonal quadratic refinement
        if (useEikonal && (!hasChunkAniso || dirMasks[v] === 0xFF)) {
          const dW = nx > 0 ? dist[v - 1] : Infinity;
          const dE = nx < CS - 1 ? dist[v + 1] : Infinity;
          const dN = ny > 0 ? dist[v - CS] : Infinity;
          const dS = ny < CS - 1 ? dist[v + CS] : Infinity;

          const tx = dW < dE ? dW : dE;
          const ty = dN < dS ? dN : dS;

          if (tx < Infinity || ty < Infinity) {
            let cxScale = baseScale, cyScale = baseScale;
            if (hasChunkAniso) {
              const vcx = curX[v], vcy = curY[v];
              if (vcx !== 0 || vcy !== 0) {
                const dirX = dW < dE ? (forward ? 1 : -1) : (forward ? -1 : 1);
                const dirY = dN < dS ? (forward ? 1 : -1) : (forward ? -1 : 1);
                cxScale *= clamp(1 - dirX * vcx * curStr, 0.2, 4.0);
                cyScale *= clamp(1 - dirY * vcy * curStr, 0.2, 4.0);
              }
            }
            if (tx < Infinity && ty < Infinity) {
              const diff = tx - ty;
              const cx2 = cxScale * cxScale, cy2 = cyScale * cyScale;
              const sumC2 = cx2 + cy2;
              const disc = sumC2 - diff * diff;
              if (disc > 0) {
                const eik = (cy2 * tx + cx2 * ty + cxScale * cyScale * Math.sqrt(disc)) / sumC2;
                if (eik < nd && eik >= Math.max(tx, ty)) nd = eik;
              } else {
                const eik = Math.min(tx + cxScale, ty + cyScale);
                if (eik < nd) nd = eik;
              }
            } else if (tx < Infinity) {
              const eik = tx + cxScale;
              if (eik < nd) nd = eik;
            } else {
              const eik = ty + cyScale;
              if (eik < nd) nd = eik;
            }
          }
        }

        if (nd < dist[v]) {
          heap.push(v, dist[v] = nd);
          if (sourceOut) sourceOut[v] = sourceOut[u];
        }
      }
    }
    out.set(dist);
  }

  /**
   * #8: 2-Pass Flee / Evasion Safety Integrator.
   * Pass 1 integrates distance outward from `threatSources`.
   * Pass 2 seeds the safest local maxima and integrates back so agents escape dead-ends.
   */
  integrateFlee(grid, chunkId, threatSources, out, opts = {}) {
    const { fleeBuf } = this;
    this.integrate(grid, chunkId, threatSources, fleeBuf, opts);
    let maxD = 0;
    for (let i = 0; i < CC; i++) {
      if (fleeBuf[i] < Infinity && fleeBuf[i] > maxD) maxD = fleeBuf[i];
    }
    if (maxD <= 0) {
      out.fill(Infinity);
      return;
    }
    const safeSeeds = [];
    const threshold = maxD * (opts.safeThresholdRatio ?? 0.85);
    for (let i = 0; i < CC; i++) {
      const d = fleeBuf[i];
      if (d < Infinity && d >= threshold) {
        safeSeeds.push({ cell: i, weight: (maxD - d) * 0.5, id: i });
      }
    }
    this.integrate(grid, chunkId, safeSeeds, out, opts);
  }
}
//#endregion

//#region 5: Portal Graph & Macro A* Pathfinder
/**
 * High-level abstract graph connecting chunk boundary portals in forward & reverse CSR format.
 * Supports incremental topology rebuilds, in-place weight updates (B1), wide-portal splitting (A3),
 * size-aware clearance (#5), anisotropic edges (#11), and reverse Dijkstra trees (C1/C2).
 */
export class PortalGraph {
  constructor(grid, prevGraph = null, dirtyChunks = null, minClearance = 1, maxPortalWidth = 8) {
    this.grid = grid;
    this.minClearance = minClearance;
    this.maxPortalWidth = maxPortalWidth;
    const portals = this.portals = detectPortals(grid, minClearance, maxPortalWidth), P = portals.length;
    const lists = Array.from({ length: grid.chunkCount }, () => []);
    for (let i = 0; i < P; i++) {
      const p = portals[i];
      lists[p.chunkA].push(p.id); lists[p.chunkB].push(p.id);
    }
    const chunkPortals = this.chunkPortals = lists.map(l => new Int32Array(l));
    const canReuse = prevGraph && prevGraph.minClearance === minClearance && prevGraph.maxPortalWidth === maxPortalWidth && dirtyChunks;
    this.chunkDistCache = Array.from({ length: grid.chunkCount }, (_, c) =>
      canReuse && !dirtyChunks.has(c) ? prevGraph.chunkDistCache[c] : new Map()
    );
    this._it = new ChunkIntegrator();
    this._field = new Float64Array(CC);
    this._buildCSR();
  }

  _buildCSR() {
    const { grid, portals, chunkPortals, minClearance, _it: it, _field: field } = this;
    const P = portals.length;
    const adj = Array.from({ length: P }, () => []);
    const revAdj = Array.from({ length: P }, () => []);
    const itOpts = { minClearance };

    for (let c = 0; c < grid.chunkCount; c++) {
      const ids = chunkPortals[c];
      if (ids.length < 2) continue;
      const cCache = this.chunkDistCache[c];
      for (let ia = 0; ia < ids.length; ia++) {
        const idA = ids[ia], pA = portals[idA], locA = pLocal(grid, pA, c);
        let row = cCache.get(locA);
        if (!row) {
          it.integrate(grid, c, [locA], field, itOpts);
          row = new Float64Array(field);
          cCache.set(locA, row);
        }
        const crossA = crossingCost(grid, pA);
        for (let ib = 0; ib < ids.length; ib++) {
          const idB = ids[ib];
          if (idA === idB) continue;
          const d = row[pLocal(grid, portals[idB], c)];
          if (d < Infinity) {
            const cost = d + crossA;
            adj[idB].push({ to: idA, cost, chunk: c });
            revAdj[idA].push({ from: idB, cost, chunk: c });
          }
        }
      }
    }
    const E = adj.reduce((acc, a) => acc + a.length, 0);
    this.edgeOffsets = new Int32Array(P + 1);
    this.edgeTargets = new Int32Array(E);
    this.edgeCosts = new Float64Array(E);
    this.edgeChunks = new Int32Array(E);
    for (let p = 0, e = 0; p < P; this.edgeOffsets[++p] = e) {
      for (let k = 0; k < adj[p].length; k++) {
        const edge = adj[p][k];
        this.edgeTargets[e] = edge.to; this.edgeCosts[e] = edge.cost; this.edgeChunks[e++] = edge.chunk;
      }
    }

    // C1: Reverse CSR edges for All-to-One Global Portal Tree
    this.revEdgeOffsets = new Int32Array(P + 1);
    this.revEdgeSources = new Int32Array(E);
    this.revEdgeCosts = new Float64Array(E);
    this.revEdgeChunks = new Int32Array(E);
    for (let p = 0, e = 0; p < P; this.revEdgeOffsets[++p] = e) {
      for (let k = 0; k < revAdj[p].length; k++) {
        const edge = revAdj[p][k];
        this.revEdgeSources[e] = edge.from; this.revEdgeCosts[e] = edge.cost; this.revEdgeChunks[e++] = edge.chunk;
      }
    }
  }

  /**
   * B1: Updates intra-chunk CSR edge weights in-place for chunks with weight-only changes
   * (congestion, tactical overlays, currents) without rebuilding graph topology.
   */
  updateEdgeWeights(weightDirtyChunks) {
    if (!weightDirtyChunks || weightDirtyChunks.size === 0) return;
    const { grid, portals, chunkPortals, minClearance, _it: it, _field: field } = this;
    const itOpts = { minClearance };
    for (const c of weightDirtyChunks) {
      const cCache = this.chunkDistCache[c];
      if (!cCache) continue;
      cCache.clear();
      const ids = chunkPortals[c];
      if (ids.length < 2) continue;
      for (let ia = 0; ia < ids.length; ia++) {
        const idA = ids[ia], pA = portals[idA], locA = pLocal(grid, pA, c);
        if (!cCache.has(locA)) {
          it.integrate(grid, c, [locA], field, itOpts);
          cCache.set(locA, new Float64Array(field));
        }
      }
    }
    const P = portals.length;
    for (let pB = 0; pB < P; pB++) {
      for (let e = this.edgeOffsets[pB]; e < this.edgeOffsets[pB + 1]; e++) {
        const c = this.edgeChunks[e];
        if (!weightDirtyChunks.has(c)) continue;
        const pA = this.edgeTargets[e];
        const row = this.chunkDistCache[c].get(pLocal(grid, portals[pA], c));
        if (row) {
          const d = row[pLocal(grid, portals[pB], c)];
          this.edgeCosts[e] = d < Infinity ? d + crossingCost(grid, portals[pA]) : Infinity;
        }
      }
    }
    for (let pA = 0; pA < P; pA++) {
      for (let e = this.revEdgeOffsets[pA]; e < this.revEdgeOffsets[pA + 1]; e++) {
        const c = this.revEdgeChunks[e];
        if (!weightDirtyChunks.has(c)) continue;
        const pB = this.revEdgeSources[e];
        const row = this.chunkDistCache[c].get(pLocal(grid, portals[pA], c));
        if (row) {
          const d = row[pLocal(grid, portals[pB], c)];
          this.revEdgeCosts[e] = d < Infinity ? d + crossingCost(grid, portals[pA]) : Infinity;
        }
      }
    }
  }
}

/**
 * Hierarchical A* & Global Reverse-Dijkstra pathfinder operating over `PortalGraph`.
 * Supports single goals, Multi-Source Geodesic Voronoi goals (#2), All-to-One Global Swarm Trees (C1),
 * Macro Multi-Chunk Flee & Evasion (C2), and Cross-Chunk LOS Portal String-Pulling (A5).
 */
export class MacroPathfinder {
  constructor(grid, graph, goalCacheCap = 256) {
    this.grid = grid; this.graph = graph;
    this.goalCacheCap = goalCacheCap;
    this.goalCache = new Map();
    const N = graph.portals.length + 2;
    this.g = new Float64Array(N); this.parent = new Int32Array(N);
    this.parentChunk = new Int32Array(N); this.seen = new Uint32Array(N);
    this.closed = new Uint32Array(N); this.it = new ChunkIntegrator();
    this.startField = new Float64Array(CC); this.heap = new MinHeap(256); this.stamp = 0;
  }

  invalidateChunks(dirtyChunks) {
    for (const [cell, g] of this.goalCache) {
      if (dirtyChunks.has(g.chunk)) this.goalCache.delete(cell);
    }
  }

  createGoal(cell) {
    const { grid, graph } = this, x = cell % grid.width, y = cell / grid.width | 0;
    const chunk = grid.chunkIdAt(x, y), field = new Float64Array(CC);
    this.it.integrate(grid, chunk, [grid.localIndex(cell)], field, { minClearance: graph.minClearance });
    const portalCost = new Float64Array(graph.portals.length).fill(Infinity);
    const cPortals = graph.chunkPortals[chunk];
    for (let i = 0; i < cPortals.length; i++) {
      const pid = cPortals[i];
      const d = field[pLocal(grid, graph.portals[pid], chunk)];
      if (d < Infinity) portalCost[pid] = d;
    }
    return { isMulti: false, cell, x, y, chunk, field, portalCost, graph };
  }

  /**
   * #2: Creates a Multi-Source Goal descriptor from an array of target cells or `{ cell, weight }` objects.
   */
  createMultiGoal(targets) {
    const { grid, graph } = this;
    const portalCost = new Float64Array(graph.portals.length).fill(Infinity);
    const portalGoalCell = new Int32Array(graph.portals.length).fill(-1);
    const chunkGroups = new Map();
    const goalList = [];

    for (let i = 0; i < targets.length; i++) {
      const t = targets[i];
      const cell = typeof t === 'object' ? (t.cell ?? grid.index(t.x | 0, t.y | 0)) : t;
      const weight = typeof t === 'object' ? (t.weight ?? 0) : 0;
      if (cell < 0 || cell >= grid.width * grid.height) continue;
      const x = cell % grid.width, y = (cell / grid.width) | 0, chunk = grid.chunkIdAt(x, y);
      goalList.push({ cell, x, y, chunk, weight });
      if (!chunkGroups.has(chunk)) chunkGroups.set(chunk, []);
      chunkGroups.get(chunk).push({ cell: grid.localIndex(cell), weight, globalCell: cell });
    }

    const chunkFields = new Map();
    for (const [chunk, sources] of chunkGroups) {
      const field = new Float64Array(CC);
      const sourceOut = new Int32Array(CC);
      const seeded = sources.map(s => ({ cell: s.cell, weight: s.weight, id: s.globalCell }));
      this.it.integrate(grid, chunk, seeded, field, {
        minClearance: graph.minClearance,
        sourceOut
      });
      chunkFields.set(chunk, { field, sourceOut, sources });
      const cPortals = graph.chunkPortals[chunk];
      for (let i = 0; i < cPortals.length; i++) {
        const pid = cPortals[i], loc = pLocal(grid, graph.portals[pid], chunk);
        const d = field[loc];
        if (d < portalCost[pid]) {
          portalCost[pid] = d;
          portalGoalCell[pid] = sourceOut[loc];
        }
      }
    }

    const primary = goalList[0] || { cell: 0, x: 0, y: 0, chunk: 0 };
    return {
      isMulti: true,
      cell: primary.cell,
      x: primary.x,
      y: primary.y,
      chunk: primary.chunk,
      goals: goalList,
      chunkFields,
      portalCost,
      portalGoalCell,
      graph
    };
  }

  /**
   * C1: Creates an All-to-One Reverse Portal Tree ("Global Swarm Goal") by running
   * a single backward Dijkstra pass over `PortalGraph` from `targets` to all portals.
   */
  createGlobalGoal(targets) {
    const { grid, graph, heap } = this;
    const normTargets = Array.isArray(targets) ? targets : [targets];
    const multi = this.createMultiGoal(normTargets);
    const P = graph.portals.length;
    const portalDist = new Float64Array(P).fill(Infinity);
    const portalNext = new Int32Array(P).fill(-1);
    const portalGoalCell = new Int32Array(P).fill(-1);

    heap.clear();
    for (let p = 0; p < P; p++) {
      const gc = multi.portalCost[p];
      if (gc < Infinity) {
        const w = gc + crossingCost(grid, graph.portals[p]);
        portalDist[p] = w;
        portalGoalCell[p] = multi.portalGoalCell[p];
        heap.push(p, w);
      }
    }

    while (!heap.isEmpty) {
      const v = heap.pop(), dv = heap.lastKey;
      if (dv > portalDist[v]) continue;
      for (let e = graph.revEdgeOffsets[v]; e < graph.revEdgeOffsets[v + 1]; e++) {
        const u = graph.revEdgeSources[e];
        const nd = dv + graph.revEdgeCosts[e];
        if (nd < portalDist[u]) {
          portalDist[u] = nd;
          portalNext[u] = v;
          portalGoalCell[u] = portalGoalCell[v];
          heap.push(u, nd);
        }
      }
    }

    return {
      isGlobal: true,
      isMulti: true,
      cell: multi.cell,
      x: multi.x,
      y: multi.y,
      goalX: multi.x + 0.5,
      goalY: multi.y + 0.5,
      goals: multi.goals,
      chunkFields: multi.chunkFields,
      portalDist,
      portalNext,
      portalGoalCell,
      minClearance: graph.minClearance,
      graph
    };
  }

  /**
   * C2: Macro 2-Pass Multi-Chunk Flee & Evasion.
   * Pass 1 integrates forward distance across `PortalGraph` outward from all `threatPoints`.
   * Pass 2 seeds safe-haven portals/chunks into a Global Goal Tree (or returns safe goal targets).
   */
  createGlobalFleeGoal(threatPoints, opts = {}) {
    const { grid, graph, heap } = this;
    const P = graph.portals.length;
    const threatDist = new Float64Array(P).fill(Infinity);
    const threatChunkGroups = new Map();

    for (let i = 0; i < threatPoints.length; i++) {
      const t = threatPoints[i];
      const tx = clamp(t.x | 0, 0, grid.width - 1), ty = clamp(t.y | 0, 0, grid.height - 1);
      const cell = grid.isWalkable(tx, ty, graph.minClearance)
        ? grid.index(tx, ty)
        : grid.findNearestWalkable(tx, ty, 16, graph.minClearance);
      if (cell < 0) continue;
      const chunk = grid.chunkIdAt(cell % grid.width, (cell / grid.width) | 0);
      if (!threatChunkGroups.has(chunk)) threatChunkGroups.set(chunk, []);
      threatChunkGroups.get(chunk).push(grid.localIndex(cell));
    }

    heap.clear();
    const field = new Float64Array(CC);
    for (const [chunk, localCells] of threatChunkGroups) {
      this.it.integrate(grid, chunk, localCells, field, { minClearance: graph.minClearance, forward: true });
      const cPortals = graph.chunkPortals[chunk];
      for (let i = 0; i < cPortals.length; i++) {
        const pid = cPortals[i], p = graph.portals[pid];
        const d = field[pLocal(grid, p, chunk)];
        if (d < Infinity) {
          const w = d + crossingCost(grid, p);
          if (w < threatDist[pid]) {
            threatDist[pid] = w;
            heap.push(pid, w);
          }
        }
      }
    }

    const horizon = (opts.fleeDistance ?? 48) * COST_OPEN;
    let maxD = 0;
    while (!heap.isEmpty) {
      const u = heap.pop(), du = heap.lastKey;
      if (du > threatDist[u]) continue;
      if (du > maxD) maxD = du;
      if (du >= horizon) continue;
      for (let e = graph.edgeOffsets[u]; e < graph.edgeOffsets[u + 1]; e++) {
        const v = graph.edgeTargets[e];
        const nd = du + graph.edgeCosts[e];
        if (nd < threatDist[v]) {
          threatDist[v] = nd;
          heap.push(v, nd);
        }
      }
    }

    const safeThreshold = Math.min(horizon, maxD * (opts.safeThresholdRatio ?? 0.75));
    const safeTargets = [];
    for (let p = 0; p < P; p++) {
      const d = threatDist[p];
      if (d < Infinity && d >= safeThreshold && safeThreshold > 0) {
        const pt = graph.portals[p];
        safeTargets.push({
          cell: pt.centerA,
          weight: Math.max(0, (maxD - d) * 0.5)
        });
      }
    }

    // Fallback when entire connected component is within a single chunk
    if (safeTargets.length === 0) {
      for (const [chunk, localCells] of threatChunkGroups) {
        this.it.integrate(grid, chunk, localCells, field, { minClearance: graph.minClearance });
        let localMax = 0, bestLoc = -1;
        for (let i = 0; i < CC; i++) {
          if (field[i] < Infinity && field[i] > localMax) { localMax = field[i]; bestLoc = i; }
        }
        if (bestLoc >= 0) {
          const ox = grid.chunkOriginX(chunk) + (bestLoc % CS);
          const oy = grid.chunkOriginY(chunk) + ((bestLoc / CS) | 0);
          safeTargets.push({ cell: grid.index(ox, oy), weight: 0 });
        }
      }
    }

    const globalGoal = this.createGlobalGoal(safeTargets);
    globalGoal.isFlee = true;
    globalGoal.threatChunkGroups = threatChunkGroups;
    globalGoal.safeTargets = safeTargets;
    return globalGoal;
  }

  getOrCreateGoal(cell) {
    let g = this.goalCache.get(cell);
    if (g) return g;
    g = this.createGoal(cell);
    if (this.goalCache.size >= this.goalCacheCap) {
      this.goalCache.delete(this.goalCache.keys().next().value);
    }
    this.goalCache.set(cell, g);
    return g;
  }

  findPath(startCell, goalOrCell) {
    const goal = typeof goalOrCell === 'number'
      ? this.getOrCreateGoal(goalOrCell)
      : (Array.isArray(goalOrCell) ? this.createMultiGoal(goalOrCell) : goalOrCell);
    if (!goal) return null;
    const { grid, graph, g, parent, parentChunk, seen, closed, heap } = this;
    const P = graph.portals.length, START = P, GOAL = P + 1;
    if (++this.stamp >= 0xffffffff) { seen.fill(0); closed.fill(0); this.stamp = 1; }
    const s = this.stamp;
    const sx = startCell % grid.width, sy = startCell / grid.width | 0, sc = grid.chunkIdAt(sx, sy);
    const gx = goal.x + 0.5, gy = goal.y + 0.5;

    this.it.integrate(grid, sc, [grid.localIndex(startCell)], this.startField, {
      minClearance: graph.minClearance,
      forward: true
    });

    const heuristic = (px, py) => {
      if (!goal.isMulti) return Math.max(0, octile(px - gx, py - gy) - 0.5);
      let best = Infinity;
      for (let i = 0; i < goal.goals.length; i++) {
        const tg = goal.goals[i];
        const d = Math.max(0, octile(px - (tg.x + 0.5), py - (tg.y + 0.5)) - 0.5) + tg.weight;
        if (d < best) best = d;
      }
      return best;
    };

    heap.clear(); g[START] = 0; seen[START] = s; parent[START] = -1; parentChunk[START] = sc;
    heap.push(START, heuristic(sx + 0.5, sy + 0.5));

    let chosenGoalCell = goal.cell;
    const relax = (v, cost, from, chunk, resolvedCell = -1) => {
      if (closed[v] === s || (seen[v] === s && cost >= g[v])) return;
      seen[v] = s; g[v] = cost; parent[v] = from; parentChunk[v] = chunk;
      if (v === GOAL && resolvedCell >= 0) chosenGoalCell = resolvedCell;
      const h = v < P ? heuristic(graph.portals[v].anchorX, graph.portals[v].anchorY) : 0;
      heap.push(v, cost + h);
    };

    while (!heap.isEmpty) {
      const u = heap.pop();
      if (closed[u] === s) continue;
      closed[u] = s;
      if (u === GOAL) {
        const portals = [], chunks = [parentChunk[GOAL]];
        let n = parent[GOAL];
        while (n !== START) { portals.push(n); chunks.push(parentChunk[n]); n = parent[n]; }
        portals.reverse();
        chunks.reverse();
        const fgx = (chosenGoalCell % grid.width) + 0.5, fgy = ((chosenGoalCell / grid.width) | 0) + 0.5;

        // A5: Precompute Cross-Chunk LOS String-Pulled Target Anchors for each step
        const portalArr = new Int32Array(portals);
        const chunkArr = new Int32Array(chunks);
        const targetX = new Float32Array(portalArr.length);
        const targetY = new Float32Array(portalArr.length);
        const refCost = Math.max(COST_OPEN, grid.costs[chosenGoalCell]);

        for (let i = 0; i < portalArr.length; i++) {
          const curP = graph.portals[portalArr[i]];
          let tx = i + 1 < portalArr.length ? graph.portals[portalArr[i + 1]].anchorX : fgx;
          let ty = i + 1 < portalArr.length ? graph.portals[portalArr[i + 1]].anchorY : fgy;
          if (grid.losSmoothing) {
            if (grid.hasWorldLOS(curP.anchorX, curP.anchorY, fgx, fgy, refCost, graph.minClearance)) {
              tx = fgx; ty = fgy;
            } else {
              for (let j = portalArr.length - 1; j > i + 1; j--) {
                const farP = graph.portals[portalArr[j]];
                if (grid.hasWorldLOS(curP.anchorX, curP.anchorY, farP.anchorX, farP.anchorY, refCost, graph.minClearance)) {
                  tx = farP.anchorX; ty = farP.anchorY;
                  break;
                }
              }
            }
          }
          targetX[i] = tx;
          targetY[i] = ty;
        }

        return {
          portals: portalArr,
          chunks: chunkArr,
          targetX,
          targetY,
          cost: g[GOAL],
          goalCell: chosenGoalCell,
          goalX: fgx,
          goalY: fgy,
          minClearance: graph.minClearance,
          graph
        };
      }
      const gu = g[u];
      if (u === START) {
        const cPorts = graph.chunkPortals[sc];
        for (let i = 0; i < cPorts.length; i++) {
          const pid = cPorts[i], p = graph.portals[pid], d = this.startField[pLocal(grid, p, sc)];
          if (d < Infinity) relax(pid, gu + d + crossingCost(grid, p), START, sc);
        }
        if (!goal.isMulti && sc === goal.chunk) {
          const d = goal.field[grid.localIndex(startCell)];
          if (d < Infinity) relax(GOAL, gu + d, START, sc, goal.cell);
        } else if (goal.isMulti && goal.chunkFields.has(sc)) {
          const cf = goal.chunkFields.get(sc), loc = grid.localIndex(startCell);
          const d = cf.field[loc];
          if (d < Infinity) relax(GOAL, gu + d, START, sc, cf.sourceOut[loc]);
        }
      } else {
        for (let e = graph.edgeOffsets[u]; e < graph.edgeOffsets[u + 1]; e++) {
          relax(graph.edgeTargets[e], gu + graph.edgeCosts[e], u, graph.edgeChunks[e]);
        }
        const gc = goal.portalCost[u];
        if (gc < Infinity) {
          const rCell = goal.isMulti ? goal.portalGoalCell[u] : goal.cell;
          const rChunk = grid.chunkIdAt(rCell % grid.width, (rCell / grid.width) | 0);
          relax(GOAL, gu + gc, u, rChunk, rCell);
        }
      }
    }
    return null;
  }
}
//#endregion

//#region 6: Flow Vector Extraction, LOS, Vortex, Corridor, Cache & Sampler
/**
 * #3: Fast chunk-local supercover Line-of-Sight check ensuring no obstacles or higher-cost detours lie between `(x0, y0)` and `(x1, y1)`.
 */
export function hasChunkLOS(costs, x0, y0, x1, y1, refCost = COST_OPEN) {
  let cx = clamp(x0 | 0, 0, CS - 1), cy = clamp(y0 | 0, 0, CS - 1);
  const ex = clamp(x1 | 0, 0, CS - 1), ey = clamp(y1 | 0, 0, CS - 1);
  const dx = Math.abs(ex - cx), dy = Math.abs(ey - cy);
  const sx = cx < ex ? 1 : -1, sy = cy < ey ? 1 : -1;
  let err = dx - dy;
  while (true) {
    const c = costs[cy * CS + cx];
    if (c >= COST_BLOCK || c > refCost) return false;
    if (cx === ex && cy === ey) return true;
    const e2 = 2 * err;
    if (e2 > -dy && e2 < dx) {
      if (costs[cy * CS + (cx + sx)] >= COST_BLOCK || costs[(cy + sy) * CS + cx] >= COST_BLOCK) return false;
    }
    if (e2 > -dy) { err -= dy; cx += sx; }
    if (e2 < dx) { err += dx; cy += sy; }
  }
}

/**
 * Computes normalized gradient vectors (`flowX`, `flowY`) from an integration field.
 * Supports:
 * - #3: Line-of-Sight direct-to-target smoothing (`opts.targetLocalX`, `opts.targetLocalY`, `opts.chunkCosts`)
 * - #4: Wall SDF repulsion blending (`opts.grid`)
 * - #7: Vortex / Orbit / Strafe curl modulation (`opts.vortex`)
 * - A1: Stitched portal exit vectors (`opts.portalExitMask`)
 */
export function computeFlowVectors(f, exX, exY, opts = {}) {
  const { integration: I, flowX: FX, flowY: FY } = f;
  const at = (x, y) => x >>> 0 >= CS || y >>> 0 >= CS ? Infinity : I[y * CS + x];
  const grid = opts.grid || null;
  const useWallRep = grid && grid.wallRepulsionWeight > 0 && grid.wallDist;
  const vortex = opts.vortex || f.vortex || null;
  const hasLOS = opts.losSmoothing && opts.chunkCosts && opts.targetLocalX !== undefined;
  const txL = opts.targetLocalX, tyL = opts.targetLocalY, cCosts = opts.chunkCosts;
  const refCost = hasLOS ? cCosts[clamp(tyL | 0, 0, CS - 1) * CS + clamp(txL | 0, 0, CS - 1)] : COST_OPEN;
  const portalMask = opts.portalMask || null;

  for (let y = 0; y < CS; y++) {
    for (let x = 0; x < CS; x++) {
      const id = y * CS + x, cur = I[id];
      if (cur === Infinity) { FX[id] = FY[id] = 0; continue; }
      if ((!cur || (portalMask && portalMask[id])) && !vortex && (exX || exY)) {
        FX[id] = exX; FY[id] = exY;
        continue;
      }

      let dx = 0, dy = 0, ok = false;

      // #3: Line-of-Sight direct-to-target vector
      if (hasLOS && hasChunkLOS(cCosts, x, y, txL, tyL, refCost)) {
        const vx = txL - (x + 0.5), vy = tyL - (y + 0.5), l = Math.hypot(vx, vy);
        if (l > 1e-5) { dx = vx / l; dy = vy / l; ok = true; }
      }

      if (!ok) {
        const E = at(x + 1, y), W = at(x - 1, y), S = at(x, y + 1), N = at(x, y - 1);
        const gx = E < W ? (E < cur ? E - cur : 0) : W < cur ? cur - W : 0;
        const gy = S < N ? (S < cur ? S - cur : 0) : N < cur ? cur - N : 0;
        dx = -gx; dy = -gy;
        const len = Math.hypot(dx, dy);
        ok = len > 1e-6;
        if (ok) {
          dx /= len; dy /= len;
          if (Math.abs(dx) > 0.25 && Math.abs(dy) > 0.25 && at(x + (dx > 0 ? 1 : -1), y + (dy > 0 ? 1 : -1)) === Infinity) ok = false;
        }
        if (!ok) {
          let bS = 0, bx = 0, by = 0;
          for (let d = 0; d < 8; d++) {
            const nx = x + DX[d], ny = y + DY[d], v = at(nx, ny);
            if (v === Infinity || (d >= 4 && (at(nx, y) === Infinity || at(x, ny) === Infinity))) continue;
            const sl = (cur - v) / DL[d];
            if (sl > bS) { bS = sl; bx = DX[d]; by = DY[d]; }
          }
          const l2 = Math.hypot(bx, by);
          dx = bx / l2 || exX || 0; dy = by / l2 || exY || 0;
        }
      }

      // #7: Vortex / Orbit / Strafe curl modulation
      if (vortex) {
        const cw = vortex.clockwise !== false ? 1 : -1;
        const tanX = -dy * cw, tanY = dx * cw;
        const R = (vortex.radius ?? 4) * COST_OPEN;
        const str = vortex.strength ?? 1.0;
        const wRadial = R > 0 ? clamp((cur - R) / R, -1, 1) : 1.0;
        dx = dx * wRadial + tanX * str;
        dy = dy * wRadial + tanY * str;
        const vl = Math.hypot(dx, dy);
        if (vl > 1e-6) { dx /= vl; dy /= vl; }
      }

      // #4: Wall SDF repulsion vector blending
      if (useWallRep) {
        const gIdx = (f.originY + y) * grid.width + (f.originX + x);
        const wd = grid.wallDist[gIdx], wr = grid.wallPenaltyRadius;
        if (wd > 0 && wd < wr) {
          const w = grid.wallRepulsionWeight * (1 - wd / wr);
          const rx = dx + grid.wallGradX[gIdx] * w, ry = dy + grid.wallGradY[gIdx] * w;
          const rl = Math.hypot(rx, ry);
          if (rl > 1e-6) {
            const checkX = x + Math.round(rx / rl), checkY = y + Math.round(ry / rl);
            if (at(checkX, checkY) < Infinity) { dx = rx / rl; dy = ry / rl; }
          }
        }
      }

      FX[id] = dx; FY[id] = dy;
    }
  }
}

/**
 * #10: Guide-Curve / Spline Corridor Flow Field builder.
 * Rasterizes a polyline into a chunk, integrates cross-track distance via Geodesic Voronoi,
 * and blends inward restoring gradients with the curve's forward tangent vector.
 */
export function buildCorridorField(grid, chunkId, polyline, opts = {}, targetField = null, sharedIt = null) {
  const ox = grid.chunkOriginX(chunkId), oy = grid.chunkOriginY(chunkId);
  const f = targetField || {
    chunkId, kind: 'c', originX: ox, originY: oy,
    integration: new Float32Array(CC), flowX: new Float32Array(CC), flowY: new Float32Array(CC),
    sourceId: new Int32Array(CC)
  };
  f.chunkId = chunkId; f.kind = 'c'; f.originX = ox; f.originY = oy;
  if (!f.sourceId) f.sourceId = new Int32Array(CC);

  const spineTanX = new Float32Array(CC), spineTanY = new Float32Array(CC);
  const seeded = new Uint8Array(CC), sources = [];

  for (let i = 0; i < polyline.length - 1; i++) {
    const p0 = polyline[i], p1 = polyline[i + 1];
    const x0 = Array.isArray(p0) ? p0[0] : p0.x, y0 = Array.isArray(p0) ? p0[1] : p0.y;
    const x1 = Array.isArray(p1) ? p1[0] : p1.x, y1 = Array.isArray(p1) ? p1[1] : p1.y;
    const dx = x1 - x0, dy = y1 - y0, segLen = Math.hypot(dx, dy);
    if (segLen < 1e-5) continue;
    const tx = dx / segLen, ty = dy / segLen;
    const steps = Math.max(1, Math.ceil(segLen * 2));
    for (let s = 0; s <= steps; s++) {
      const t = s / steps;
      const lx = Math.floor(x0 + dx * t - ox), ly = Math.floor(y0 + dy * t - oy);
      if (lx >>> 0 >= CS || ly >>> 0 >= CS) continue;
      const cell = ly * CS + lx;
      spineTanX[cell] = tx; spineTanY[cell] = ty;
      if (!seeded[cell]) {
        seeded[cell] = 1;
        sources.push({ cell, weight: 0, id: cell });
      }
    }
  }

  const it = sharedIt || new ChunkIntegrator();
  it.integrate(grid, chunkId, sources, f.integration, { sourceOut: f.sourceId, minClearance: opts.minClearance ?? 1 });
  computeFlowVectors(f, 0, 0, { grid });

  const corridorRadius = (opts.radius ?? 2.5) * COST_OPEN;
  const tangentWeight = opts.tangentWeight ?? 1.25;
  for (let i = 0; i < CC; i++) {
    const sid = f.sourceId[i];
    if (sid < 0 || f.integration[i] === Infinity) continue;
    const tx = spineTanX[sid], ty = spineTanY[sid];
    const attractW = clamp(f.integration[i] / Math.max(0.5, corridorRadius), 0, 1);
    const vx = f.flowX[i] * attractW + tx * tangentWeight;
    const vy = f.flowY[i] * attractW + ty * tangentWeight;
    const l = Math.hypot(vx, vy);
    if (l > 1e-6) { f.flowX[i] = vx / l; f.flowY[i] = vy / l; }
  }
  return f;
}

/**
 * B2 & A1: Unified LRU cache with zero-GC TypedArray slab pooling for all flow field types:
 * Portal (with A1 wavefront boundary stitching), Goal, Global Tree (C1), Multi-Goal (#2),
 * Vortex (#7), Flee (#8), and Corridor (#10).
 */
export class FlowFieldCache {
  constructor(grid, cap = 2048) {
    this.grid = grid; this.cap = cap; this.entries = new Map();
    this.it = new ChunkIntegrator();
    this.pool = [];
    this._portalMask = new Uint8Array(CC);
  }

  key(c, kind, tid, minClearance = 1) {
    return ((tid * 8 + (kind.charCodeAt(0) & 7)) * 16 + minClearance) * 65536 + c;
  }

  _acquire(cid, kind, needSourceId = false) {
    const { grid } = this;
    const f = this.pool.pop() || {
      chunkId: cid, kind, originX: 0, originY: 0,
      integration: new Float32Array(CC), flowX: new Float32Array(CC), flowY: new Float32Array(CC),
      sourceId: null, vortex: null
    };
    f.chunkId = cid;
    f.kind = kind;
    f.originX = grid.chunkOriginX(cid);
    f.originY = grid.chunkOriginY(cid);
    f.vortex = null;
    if (needSourceId && !f.sourceId) f.sourceId = new Int32Array(CC);
    return f;
  }

  _release(f) {
    if (f && this.pool.length < this.cap) this.pool.push(f);
  }

  _promote(k, hit) {
    if (this.entries.size >= this.cap * 0.75) {
      this.entries.delete(k);
      this.entries.set(k, hit);
    }
    return hit;
  }

  _store(k, f) {
    if (this.entries.size >= this.cap) {
      const oldestKey = this.entries.keys().next().value;
      const evicted = this.entries.get(oldestKey);
      this.entries.delete(oldestKey);
      this._release(evicted);
    }
    this.entries.set(k, f);
    return f;
  }

  /**
   * A1: Builds or retrieves a portal flow field with Wavefront-Stitched Portal Seeding.
   * When `(targetX, targetY)` or `downstreamField` is supplied, wide portal border cells are
   * seeded with smooth boundary weights toward the downstream target to eliminate 90-degree L-turns.
   */
  getPortalField(c, p, minClearance = 1, targetX = NaN, targetY = NaN, downstreamField = null) {
    const hasTarget = !Number.isNaN(targetX) && !Number.isNaN(targetY);
    const angleBucket = hasTarget
      ? ((Math.round((Math.atan2(targetY - p.anchorY, targetX - p.anchorX) / (Math.PI * 2)) * 32) & 31) + 1)
      : 0;
    const k = this.key(c, 'p', p.uid * 33 + angleBucket, minClearance);
    const hit = this.entries.get(k);
    if (hit) return this._promote(k, hit);

    const { grid, _portalMask: mask } = this;
    const pCells = portalSideCells(p, c);
    const oppCells = c === p.chunkA ? p.cellsB : p.cellsA;
    let exX = portalExitX(p, c), exY = portalExitY(p, c);
    mask.fill(0);

    const sources = new Array(pCells.length);
    let minW = Infinity;

    for (let i = 0; i < pCells.length; i++) {
      const gCell = pCells[i], loc = grid.localIndex(gCell);
      mask[loc] = 1;
      let w = 0;
      if (downstreamField) {
        const oppLoc = grid.localIndex(oppCells[i]);
        const dVal = downstreamField.integration[oppLoc];
        if (dVal < Infinity) w = dVal;
      } else if (hasTarget && pCells.length > 1) {
        const wx = (gCell % grid.width) + 0.5, wy = ((gCell / grid.width) | 0) + 0.5;
        w = octile(wx - targetX, wy - targetY) * (COST_OPEN * 0.45);
      }
      if (w < minW) minW = w;
      sources[i] = { cell: loc, weight: w, id: i };
    }
    if (minW > 0 && minW < Infinity) {
      for (let i = 0; i < sources.length; i++) sources[i].weight -= minW;
    }

    if (hasTarget) {
      const vx = targetX - p.anchorX, vy = targetY - p.anchorY, vl = Math.hypot(vx, vy);
      if (vl > 1e-5) {
        const tx = vx / vl, ty = vy / vl;
        if (tx * exX + ty * exY > 0.1) {
          const bx = exX * 0.45 + tx * 0.55, by = exY * 0.45 + ty * 0.55, bl = Math.hypot(bx, by);
          if (bl > 1e-5) { exX = bx / bl; exY = by / bl; }
        }
      }
    }

    return this.build(k, c, 'p', sources, exX, exY, { minClearance, portalMask: mask });
  }

  getGoalField(c, cell, minClearance = 1) {
    const k = this.key(c, 'g', cell, minClearance);
    const hit = this.entries.get(k);
    if (hit) return this._promote(k, hit);
    const loc = this.grid.localIndex(cell);
    return this.build(k, c, 'g', [loc], 0, 0, {
      minClearance,
      losSmoothing: this.grid.losSmoothing,
      targetLocalX: (loc % CS) + 0.5,
      targetLocalY: ((loc / CS) | 0) + 0.5
    });
  }

  getMultiGoalField(c, sources, minClearance = 1) {
    let hash = 2166136261;
    for (let i = 0; i < sources.length; i++) {
      const s = sources[i];
      const cell = typeof s === 'object' ? s.cell : s;
      const w = typeof s === 'object' ? ((s.weight ?? 0) * 16 | 0) : 0;
      hash = Math.imul(hash ^ ((cell << 8) ^ w), 16777619);
    }
    const k = `m:${c}:${minClearance}:${hash >>> 0}`;
    const hit = this.entries.get(k);
    if (hit) return this._promote(k, hit);
    return this.build(k, c, 'm', sources, 0, 0, { minClearance, trackSources: true });
  }

  getVortexField(c, centerCell, vortexOpts = {}) {
    const clr = vortexOpts.minClearance ?? 1;
    const rKey = ((vortexOpts.radius ?? 4) * 16) | 0;
    const sKey = ((vortexOpts.strength ?? 1) * 16) | 0;
    const cwKey = vortexOpts.clockwise !== false ? 1 : 0;
    const k = `v:${c}:${centerCell}:${clr}:${rKey}:${sKey}:${cwKey}`;
    const hit = this.entries.get(k);
    if (hit) return this._promote(k, hit);
    const loc = this.grid.localIndex(centerCell);
    return this.build(k, c, 'v', [loc], 0, 0, {
      minClearance: clr,
      vortex: vortexOpts,
      losSmoothing: this.grid.losSmoothing,
      targetLocalX: (loc % CS) + 0.5,
      targetLocalY: ((loc / CS) | 0) + 0.5
    });
  }

  getFleeField(c, threatLocalCells, opts = {}) {
    const clr = opts.minClearance ?? 1;
    let hash = 2166136261;
    for (let i = 0; i < threatLocalCells.length; i++) {
      hash = Math.imul(hash ^ threatLocalCells[i], 16777619);
    }
    const ratioKey = ((opts.safeThresholdRatio ?? 0.85) * 100) | 0;
    const k = `f:${c}:${clr}:${ratioKey}:${hash >>> 0}`;
    const hit = this.entries.get(k);
    if (hit) return this._promote(k, hit);

    const f = this._acquire(c, 'f', false);
    this.it.integrateFlee(this.grid, c, threatLocalCells, f.integration, opts);
    computeFlowVectors(f, 0, 0, { grid: this.grid });
    return this._store(k, f);
  }

  getCorridorField(c, polyline, opts = {}) {
    const clr = opts.minClearance ?? 1;
    let hash = 2166136261;
    for (let i = 0; i < polyline.length; i++) {
      const p = polyline[i];
      const px = ((Array.isArray(p) ? p[0] : p.x) * 8) | 0;
      const py = ((Array.isArray(p) ? p[1] : p.y) * 8) | 0;
      hash = Math.imul(hash ^ ((px << 16) ^ py), 16777619);
    }
    const rKey = ((opts.radius ?? 2.5) * 16) | 0;
    const tKey = ((opts.tangentWeight ?? 1.25) * 16) | 0;
    const k = `c:${c}:${clr}:${rKey}:${tKey}:${hash >>> 0}`;
    const hit = this.entries.get(k);
    if (hit) return this._promote(k, hit);

    const f = this._acquire(c, 'c', true);
    buildCorridorField(this.grid, c, polyline, opts, f, this.it);
    return this._store(k, f);
  }

  clear() {
    for (const f of this.entries.values()) this._release(f);
    this.entries.clear();
  }

  invalidateChunks(dirtyChunks) {
    for (const [k, f] of this.entries) {
      if (dirtyChunks.has(f.chunkId)) {
        this.entries.delete(k);
        this._release(f);
      }
    }
  }

  build(k, cid, kind, src, exX, exY, opts = {}) {
    const { grid } = this;
    const f = this._acquire(cid, kind, !!opts.trackSources);
    f.vortex = opts.vortex || null;
    this.it.integrate(grid, cid, src, f.integration, {
      minClearance: opts.minClearance ?? 1,
      sourceOut: opts.trackSources ? f.sourceId : null
    });
    computeFlowVectors(f, exX, exY, {
      grid,
      vortex: opts.vortex,
      losSmoothing: opts.losSmoothing,
      chunkCosts: this.it.costs,
      targetLocalX: opts.targetLocalX,
      targetLocalY: opts.targetLocalY,
      portalMask: opts.portalMask
    });
    return this._store(k, f);
  }
}

/**
 * A2: Bilinearly interpolates the flow direction vector `(out.x, out.y)` at world position `(px, py)`.
 * Supports cross-chunk halo interpolation via optional `neighborResolver(worldCellX, worldCellY)`
 * so velocity fields remain C1-continuous across chunk borders.
 */
export function sampleFlowBilinear(f, px, py, out = { x: 0, y: 0 }, neighborResolver = null) {
  const lx = px - f.originX - 0.5, ly = py - f.originY - 0.5;
  const x0 = Math.floor(lx), y0 = Math.floor(ly);
  const tx = lx - x0, ty = ly - y0;
  const { integration: I, flowX: FX, flowY: FY } = f;
  const w00 = (1 - tx) * (1 - ty), w10 = tx * (1 - ty), w01 = (1 - tx) * ty, w11 = tx * ty;

  let sx = 0, sy = 0, ws = 0;
  const tap = (ix, iy, w) => {
    if (w <= 0) return;
    if (ix >= 0 && ix < CS && iy >= 0 && iy < CS) {
      const id = iy * CS + ix;
      if (I[id] < Infinity) { sx += FX[id] * w; sy += FY[id] * w; ws += w; }
      return;
    }
    if (neighborResolver) {
      const gx = f.originX + ix, gy = f.originY + iy;
      const nf = neighborResolver(gx, gy);
      if (nf) {
        const nid = (gy - nf.originY) * CS + (gx - nf.originX);
        if (nf.integration[nid] < Infinity) {
          sx += nf.flowX[nid] * w; sy += nf.flowY[nid] * w; ws += w;
          return;
        }
      }
    }
    const cx = ix < 0 ? 0 : (ix > 15 ? 15 : ix);
    const cy = iy < 0 ? 0 : (iy > 15 ? 15 : iy);
    const id = cy * CS + cx;
    if (I[id] < Infinity) { sx += FX[id] * w; sy += FY[id] * w; ws += w; }
  };

  tap(x0, y0, w00);
  tap(x0 + 1, y0, w10);
  tap(x0, y0 + 1, w01);
  tap(x0 + 1, y0 + 1, w11);

  if (ws < 1e-6) return false;
  const len = Math.hypot(sx, sy);
  if (len < 0.15 * ws) {
    const c = clamp(Math.floor(py - f.originY), 0, CS - 1) * CS + clamp(Math.floor(px - f.originX), 0, CS - 1);
    if (I[c] < Infinity && (FX[c] || FY[c])) { out.x = FX[c]; out.y = FY[c]; return true; }
    if (len < 1e-6) return false;
  }
  out.x = sx / len; out.y = sy / len;
  return true;
}

/**
 * E1: Ramer-Douglas-Peucker polyline simplification for extracted flow streamlines.
 */
export function simplifyPolyline(points, tolerance = 0.25) {
  if (points.length <= 2 || tolerance <= 0) return points;
  const tol2 = tolerance * tolerance;
  const keep = new Uint8Array(points.length);
  keep[0] = keep[points.length - 1] = 1;
  const stack = [0, points.length - 1];

  while (stack.length > 0) {
    const last = stack.pop(), first = stack.pop();
    if (last - first <= 1) continue;
    const ax = points[first].x, ay = points[first].y;
    const bx = points[last].x, by = points[last].y;
    const dx = bx - ax, dy = by - ay, len2 = dx * dx + dy * dy;
    let maxD2 = 0, index = -1;

    for (let i = first + 1; i < last; i++) {
      const px = points[i].x, py = points[i].y;
      let t = len2 > 1e-8 ? ((px - ax) * dx + (py - ay) * dy) / len2 : 0;
      t = clamp(t, 0, 1);
      const d2 = (px - (ax + t * dx)) ** 2 + (py - (ay + t * dy)) ** 2;
      if (d2 > maxD2) { maxD2 = d2; index = i; }
    }
    if (maxD2 > tol2 && index >= 0) {
      keep[index] = 1;
      stack.push(first, index, index, last);
    }
  }
  const out = [];
  for (let i = 0; i < points.length; i++) if (keep[i]) out.push(points[i]);
  return out;
}
//#endregion

//#region 7: Unified FlowFieldPathfinder API
/**
 * Unified Hierarchical Flow Field Pathfinding API exposing all 11 flow field types
 * plus Global Swarm Trees (C1), Macro Multi-Chunk Flee (C2), Wavefront-Stitched Portals (A1),
 * Cross-Chunk Halo Interpolation (A2), Wide-Portal Splitting (A3), Anisotropic Eikonal (A4),
 * Cross-Chunk LOS Skipping (A5), Two-Tier Dirty Tracking (B1), Unified Slab Pool Cache (B2),
 * Incremental SDF/Clearance (B3), Counter-Flow Congestion Lanes (D2), and RK2 Streamlines (E1).
 */
export class FlowFieldPathfinder {
  constructor(opts = {}) {
    const cx = opts.width ? Math.ceil(opts.width / CS) : (opts.chunksX ?? 10);
    const cy = opts.height ? Math.ceil(opts.height / CS) : (opts.chunksY ?? 10);
    this.cacheCap = opts.cacheCap ?? 2048;
    this.minClearance = opts.minClearance ?? 1;
    this.maxPortalWidth = opts.maxPortalWidth ?? 8;
    this.grid = new GridWorld(cx, cy, opts.costs || null);
    if (opts.useEikonal !== undefined) this.grid.useEikonal = opts.useEikonal;
    if (opts.losSmoothing !== undefined) this.grid.losSmoothing = opts.losSmoothing;
    if (opts.wallRepulsion) this.grid.enableWallRepulsion(opts.wallRepulsion);

    this.graph = new PortalGraph(this.grid, null, null, this.minClearance, this.maxPortalWidth);
    this.macro = new MacroPathfinder(this.grid, this.graph);
    this.cache = new FlowFieldCache(this.grid, this.cacheCap);
    this.clearanceGraphs = new Map([[this.minClearance, { graph: this.graph, macro: this.macro }]]);
    this._globalGoalEpoch = 1;
  }

  get width() { return this.grid.width; }
  get height() { return this.grid.height; }

  getCost(x, y) { return this.grid.getCost(x, y); }
  setCost(x, y, cost) { return this.grid.setCost(x, y, cost); }
  setRect(x, y, w, h, cost) { return this.grid.setRect(x, y, w, h, cost); }
  setDisc(x, y, r, cost) { return this.grid.setDisc(x, y, r, cost); }
  loadCosts(costs) { this.grid.loadCosts(costs); }

  /**
   * D2: Convenience passthrough for splatting agent positions and optional velocity vectors `(vx, vy)`
   * onto the continuum congestion layer.
   */
  setCongestionFromAgents(px, py, count, unitWeight = 1.0, vx = null, vy = null) {
    this.grid.setCongestionFromAgents(px, py, count, unitWeight, vx, vy);
  }

  /**
   * B1 & B3: Incrementally updates portal graphs and invalidates cached flow fields.
   * Weight-only changes (congestion, tactical discs, currents) update CSR edge costs in-place
   * without reallocating `PortalGraph` or `MacroPathfinder`.
   */
  rebuild() {
    const { all, topology, weights } = this.grid.consumeDirtyState();
    if (all.size === 0) return false;
    this._globalGoalEpoch++;

    if (topology.size > 0 && (this.grid.wallPenaltyWeight > 0 || this.grid.wallRepulsionWeight > 0)) {
      this.grid.updateWallSDF();
    }
    this.cache.invalidateChunks(all);

    for (const [clr, entry] of this.clearanceGraphs) {
      if (topology.size > 0) {
        const nextGraph = new PortalGraph(this.grid, entry.graph, all, clr, this.maxPortalWidth);
        const nextMacro = new MacroPathfinder(this.grid, nextGraph);
        this.clearanceGraphs.set(clr, { graph: nextGraph, macro: nextMacro });
        if (clr === this.minClearance) {
          this.graph = nextGraph;
          this.macro = nextMacro;
        }
      } else if (weights.size > 0) {
        entry.graph.updateEdgeWeights(weights);
        entry.macro.invalidateChunks(weights);
      }
    }
    return true;
  }

  _getHierarchy(minClearance = this.minClearance) {
    if (this.grid.dirtyChunks.size > 0) this.rebuild();
    let entry = this.clearanceGraphs.get(minClearance);
    if (!entry) {
      const graph = new PortalGraph(this.grid, null, null, minClearance, this.maxPortalWidth);
      const macro = new MacroPathfinder(this.grid, graph);
      entry = { graph, macro };
      this.clearanceGraphs.set(minClearance, entry);
    }
    return entry;
  }

  /**
   * Computes a single-goal hierarchical macro path (supports `opts.minClearance`).
   */
  findPath(sx, sy, gx, gy, opts = {}) {
    const clr = opts.minClearance ?? this.minClearance;
    const { macro } = this._getHierarchy(clr);
    const { grid } = this;
    const scx = clamp(sx | 0, 0, grid.width - 1), scy = clamp(sy | 0, 0, grid.height - 1);
    const gcx = clamp(gx | 0, 0, grid.width - 1), gcy = clamp(gy | 0, 0, grid.height - 1);
    const startCell = grid.isWalkable(scx, scy, clr) ? grid.index(scx, scy) : grid.findNearestWalkable(scx, scy, 16, clr);
    const goalCell = grid.isWalkable(gcx, gcy, clr) ? grid.index(gcx, gcy) : grid.findNearestWalkable(gcx, gcy, 16, clr);
    if (startCell < 0 || goalCell < 0) return null;
    const path = macro.findPath(startCell, goalCell);
    if (path) path.minClearance = clr;
    return path;
  }

  /**
   * #2: Computes a Multi-Goal Geodesic Voronoi macro path to the nearest/best weighted target in `goals`.
   */
  findMultiGoalPath(sx, sy, goals, opts = {}) {
    const clr = opts.minClearance ?? this.minClearance;
    const { macro } = this._getHierarchy(clr);
    const { grid } = this;
    const scx = clamp(sx | 0, 0, grid.width - 1), scy = clamp(sy | 0, 0, grid.height - 1);
    const startCell = grid.isWalkable(scx, scy, clr) ? grid.index(scx, scy) : grid.findNearestWalkable(scx, scy, 16, clr);
    if (startCell < 0) return null;
    const validTargets = [];
    for (const g of goals) {
      const gx = clamp(g.x | 0, 0, grid.width - 1), gy = clamp(g.y | 0, 0, grid.height - 1);
      const cell = grid.isWalkable(gx, gy, clr) ? grid.index(gx, gy) : grid.findNearestWalkable(gx, gy, 16, clr);
      if (cell >= 0) validTargets.push({ cell, weight: g.weight ?? 0 });
    }
    if (!validTargets.length) return null;
    const path = macro.findPath(startCell, validTargets);
    if (path) path.minClearance = clr;
    return path;
  }

  /**
   * C1: Builds a Global All-to-One Reverse Portal Tree for a single goal or array of goals.
   * Any number of agents anywhere in the world can steer toward this goal in O(1) time via
   * `sampleGlobalFlow(globalGoal, wx, wy)` without calling `findPath()`.
   */
  createGlobalGoalField(goals, opts = {}) {
    const clr = opts.minClearance ?? this.minClearance;
    const { macro } = this._getHierarchy(clr);
    const { grid } = this;
    const list = Array.isArray(goals) ? goals : [goals];
    const validTargets = [];
    for (const g of list) {
      const gx = clamp((g.x ?? (g % grid.width)) | 0, 0, grid.width - 1);
      const gy = clamp((g.y ?? ((g / grid.width) | 0)) | 0, 0, grid.height - 1);
      const cell = grid.isWalkable(gx, gy, clr) ? grid.index(gx, gy) : grid.findNearestWalkable(gx, gy, 16, clr);
      if (cell >= 0) validTargets.push({ cell, weight: g.weight ?? 0 });
    }
    if (!validTargets.length) return null;
    const globalGoal = macro.createGlobalGoal(validTargets);
    globalGoal.id = ++this._globalGoalEpoch;
    return globalGoal;
  }

  /**
   * C2: Builds a Global Multi-Chunk Flee & Evasion field away from `threatPoints`.
   */
  createGlobalFleeField(threatPoints, opts = {}) {
    const clr = opts.minClearance ?? this.minClearance;
    const { macro } = this._getHierarchy(clr);
    const globalFlee = macro.createGlobalFleeGoal(threatPoints, opts);
    globalFlee.id = ++this._globalGoalEpoch;
    return globalFlee;
  }

  /**
   * C2: Computes a hierarchical multi-chunk escape path from `(sx, sy)` away from `threatPoints`.
   */
  findFleePath(sx, sy, threatPoints, opts = {}) {
    const clr = opts.minClearance ?? this.minClearance;
    const { macro } = this._getHierarchy(clr);
    const { grid } = this;
    const scx = clamp(sx | 0, 0, grid.width - 1), scy = clamp(sy | 0, 0, grid.height - 1);
    const startCell = grid.isWalkable(scx, scy, clr) ? grid.index(scx, scy) : grid.findNearestWalkable(scx, scy, 16, clr);
    if (startCell < 0) return null;
    const globalFlee = macro.createGlobalFleeGoal(threatPoints, opts);
    if (!globalFlee.safeTargets || !globalFlee.safeTargets.length) return null;
    const path = macro.findPath(startCell, globalFlee.safeTargets);
    if (path) path.minClearance = clr;
    return path;
  }

  /**
   * C1: Resolves (and caches) the 16x16 flow field for the chunk at `(wx, wy)` in a `GlobalGoal`.
   */
  getGlobalFieldAt(globalGoal, wx, wy) {
    if (!globalGoal || !globalGoal.isGlobal) return null;
    const { grid, cache } = this;
    if (!grid.inBounds(wx | 0, wy | 0)) return null;
    const cx = wx | 0, cy = wy | 0;
    const chunk = grid.chunkIdAt(cx, cy);
    const clr = globalGoal.minClearance ?? this.minClearance;
    const k = `G:${globalGoal.id}:${chunk}:${clr}`;
    const hit = cache.entries.get(k);
    if (hit) return cache._promote(k, hit);

    const graph = globalGoal.graph;
    if (globalGoal.isFlee && globalGoal.threatChunkGroups && globalGoal.threatChunkGroups.has(chunk)) {
      const cPortals = graph.chunkPortals[chunk];
      let hasSafePortal = false;
      for (let i = 0; i < cPortals.length; i++) {
        if (globalGoal.portalDist[cPortals[i]] < Infinity) { hasSafePortal = true; break; }
      }
      if (!hasSafePortal) {
        return cache.getFleeField(chunk, globalGoal.threatChunkGroups.get(chunk), { minClearance: clr });
      }
    }

    const sources = [];
    const mask = cache._portalMask;
    mask.fill(0);
    let minW = Infinity, bestExX = 0, bestExY = 0;

    if (globalGoal.chunkFields && globalGoal.chunkFields.has(chunk)) {
      const cf = globalGoal.chunkFields.get(chunk);
      for (let i = 0; i < cf.sources.length; i++) {
        const s = cf.sources[i];
        sources.push({ cell: s.cell, weight: s.weight, id: s.globalCell });
        if (s.weight < minW) minW = s.weight;
      }
    }

    const cPortals = graph.chunkPortals[chunk];
    for (let i = 0; i < cPortals.length; i++) {
      const pid = cPortals[i], pDist = globalGoal.portalDist[pid];
      if (pDist === Infinity) continue;
      const p = graph.portals[pid], pCells = portalSideCells(p, chunk);
      const nextPid = globalGoal.portalNext[pid];
      const nextP = nextPid >= 0 ? graph.portals[nextPid] : null;
      const gCell = globalGoal.portalGoalCell[pid];
      const tx = nextP ? nextP.anchorX : (gCell >= 0 ? (gCell % grid.width) + 0.5 : p.anchorX);
      const ty = nextP ? nextP.anchorY : (gCell >= 0 ? ((gCell / grid.width) | 0) + 0.5 : p.anchorY);

      if (pDist < minW) {
        minW = pDist;
        bestExX = portalExitX(p, chunk);
        bestExY = portalExitY(p, chunk);
      }

      let minBias = Infinity;
      const biases = new Float64Array(pCells.length);
      for (let j = 0; j < pCells.length; j++) {
        const gc = pCells[j];
        const b = pCells.length > 1 ? octile((gc % grid.width) + 0.5 - tx, ((gc / grid.width) | 0) + 0.5 - ty) * (COST_OPEN * 0.4) : 0;
        biases[j] = b;
        if (b < minBias) minBias = b;
      }
      for (let j = 0; j < pCells.length; j++) {
        const loc = grid.localIndex(pCells[j]);
        mask[loc] = 1;
        sources.push({ cell: loc, weight: pDist + (biases[j] - minBias), id: pid });
      }
    }

    if (!sources.length) return null;
    for (let i = 0; i < sources.length; i++) sources[i].weight -= minW;

    const singleInChunkGoal = globalGoal.chunkFields && globalGoal.chunkFields.has(chunk) && globalGoal.chunkFields.get(chunk).sources.length === 1
      ? globalGoal.chunkFields.get(chunk).sources[0].cell
      : -1;

    return cache.build(k, chunk, 'G', sources, bestExX, bestExY, {
      minClearance: clr,
      portalMask: mask,
      losSmoothing: singleInChunkGoal >= 0 && grid.losSmoothing,
      targetLocalX: singleInChunkGoal >= 0 ? (singleInChunkGoal % CS) + 0.5 : undefined,
      targetLocalY: singleInChunkGoal >= 0 ? ((singleInChunkGoal / CS) | 0) + 0.5 : undefined
    });
  }

  /**
   * C1 & A2: Samples the bilinear flow direction `(out.x, out.y)` at `(wx, wy)` from a `GlobalGoal`
   * with cross-chunk halo continuity.
   */
  sampleGlobalFlow(globalGoal, wx, wy, out = { x: 0, y: 0 }) {
    const f = this.getGlobalFieldAt(globalGoal, wx, wy);
    if (!f) { out.x = 0; out.y = 0; return false; }
    return sampleFlowBilinear(f, wx, wy, out, (gx, gy) => this.getGlobalFieldAt(globalGoal, gx, gy));
  }

  /**
   * #7: Builds (or retrieves from cache) a Vortex / Orbit / Strafe flow field around `(wx, wy)`.
   */
  getVortexField(wx, wy, opts = {}) {
    const { grid } = this;
    const cx = clamp(wx | 0, 0, grid.width - 1), cy = clamp(wy | 0, 0, grid.height - 1);
    const chunk = grid.chunkIdAt(cx, cy), cell = grid.index(cx, cy);
    return this.cache.getVortexField(chunk, cell, opts);
  }

  /**
   * #8: Builds (or retrieves from cache) a 2-Pass Flee / Evasion safety flow field for the chunk at `(wx, wy)` away from `threatPoints`.
   */
  getFleeField(wx, wy, threatPoints, opts = {}) {
    const { grid } = this;
    const cx = clamp(wx | 0, 0, grid.width - 1), cy = clamp(wy | 0, 0, grid.height - 1);
    const chunk = grid.chunkIdAt(cx, cy), ox = grid.chunkOriginX(chunk), oy = grid.chunkOriginY(chunk);
    const localThreats = [];
    for (const t of threatPoints) {
      const lx = clamp((t.x | 0) - ox, 0, CS - 1), ly = clamp((t.y | 0) - oy, 0, CS - 1);
      localThreats.push(ly * CS + lx);
    }
    return this.cache.getFleeField(chunk, localThreats, opts);
  }

  /**
   * #10: Builds (or retrieves from cache) a Guide-Curve / Spline Corridor flow field for the chunk at `(wx, wy)`.
   */
  getCorridorField(wx, wy, polyline, opts = {}) {
    const { grid } = this;
    const cx = clamp(wx | 0, 0, grid.width - 1), cy = clamp(wy | 0, 0, grid.height - 1);
    return this.cache.getCorridorField(grid.chunkIdAt(cx, cy), polyline, opts);
  }

  /**
   * Resolves the 16x16 flow field for the chunk at `(wx, wy)` along a given `path`.
   * Uses A1 Wavefront-Stitched Portal Seeding and A5 String-Pulled Target Coordinates.
   */
  getFieldForPath(path, wx, wy, hintIdx = 0) {
    if (!path) return null;
    if (path.isGlobal) {
      const field = this.getGlobalFieldAt(path, wx, wy);
      return field ? { field, chunkIndex: 0 } : null;
    }
    const clr = path.minClearance ?? this.minClearance;
    const { graph } = this._getHierarchy(clr);
    if (path.graph !== graph) return null;
    const { grid } = this;
    if (!grid.inBounds(wx | 0, wy | 0)) return null;
    const cx = wx | 0, cy = wy | 0;
    const chunk = grid.chunkIdAt(cx, cy);
    let idx = hintIdx;
    if (path.chunks[idx] !== chunk) {
      idx = path.chunks.indexOf(chunk, hintIdx);
      if (idx < 0) idx = path.chunks.lastIndexOf(chunk, hintIdx);
      if (idx < 0) return null;
    }
    let field;
    if (idx < path.portals.length) {
      const tx = path.targetX ? path.targetX[idx] : path.goalX;
      const ty = path.targetY ? path.targetY[idx] : path.goalY;
      field = this.cache.getPortalField(chunk, graph.portals[path.portals[idx]], clr, tx, ty);
    } else {
      field = this.cache.getGoalField(chunk, path.goalCell, clr);
    }
    if (!(field.integration[(cy - field.originY) * CS + (cx - field.originX)] < Infinity)) {
      return null;
    }
    return { field, chunkIndex: idx };
  }

  /**
   * A2 & A5: Samples the bilinear flow direction `(out.x, out.y)` at `(wx, wy)` along `path`
   * with cross-chunk halo interpolation and cross-chunk direct LOS smoothing.
   */
  sampleFlow(path, wx, wy, out = { x: 0, y: 0 }, hintIdx = 0) {
    if (!path) { out.x = 0; out.y = 0; return false; }
    if (path.isGlobal) return this.sampleGlobalFlow(path, wx, wy, out);
    const res = this.getFieldForPath(path, wx, wy, hintIdx);
    if (!res) { out.x = 0; out.y = 0; return false; }

    // A5: Direct Cross-Chunk Line-of-Sight steering when unobstructed to goal
    if (this.grid.losSmoothing && res.chunkIndex < path.portals.length) {
      const dx = path.goalX - wx, dy = path.goalY - wy;
      const dist = Math.hypot(dx, dy);
      if (dist > 1e-4 && dist <= CS * 3) {
        const clr = path.minClearance ?? this.minClearance;
        const refCost = this.grid.costs[path.goalCell] || COST_OPEN;
        if (this.grid.hasWorldLOS(wx, wy, path.goalX, path.goalY, refCost, clr)) {
          out.x = dx / dist;
          out.y = dy / dist;
          return true;
        }
      }
    }

    return sampleFlowBilinear(res.field, wx, wy, out, (gx, gy) => {
      const nRes = this.getFieldForPath(path, gx, gy, res.chunkIndex);
      return nRes ? nRes.field : null;
    });
  }

  /**
   * E1: Integrates a continuous streamline trajectory through the flow field from `(sx, sy)`
   * using 2nd-order Runge-Kutta (midpoint method) and Ramer-Douglas-Peucker simplification.
   * Works with both `MacroPath` and `GlobalGoal` descriptors.
   * @param {Object} pathOrGlobalGoal - Path from `findPath` or global tree from `createGlobalGoalField`.
   * @param {number} sx - Start world X.
   * @param {number} sy - Start world Y.
   * @param {Object} [opts={}] - `{ stepSize=0.5, maxSteps=1024, simplifyTolerance=0.2, arrivalRadius=0.6 }`
   * @returns {Array<{x:number, y:number}>} World-space polyline coordinates.
   */
  tracePathPolyline(pathOrGlobalGoal, sx, sy, opts = {}) {
    if (!pathOrGlobalGoal) return [];
    const stepSize = opts.stepSize ?? 0.5;
    const maxSteps = opts.maxSteps ?? 1024;
    const simplifyTol = opts.simplifyTolerance ?? 0.2;
    const arrivalR2 = (opts.arrivalRadius ?? Math.max(0.55, stepSize * 0.9)) ** 2;

    let x = sx, y = sy, hintIdx = 0;
    const pts = [{ x, y }];
    const v1 = { x: 0, y: 0 }, v2 = { x: 0, y: 0 };
    const gx = pathOrGlobalGoal.goalX, gy = pathOrGlobalGoal.goalY;
    const hasSingleGoal = gx !== undefined && gy !== undefined && (!pathOrGlobalGoal.goals || pathOrGlobalGoal.goals.length <= 1);

    for (let step = 0; step < maxSteps; step++) {
      if (hasSingleGoal && (x - gx) ** 2 + (y - gy) ** 2 <= arrivalR2) {
        pts.push({ x: gx, y: gy });
        break;
      }
      if (!pathOrGlobalGoal.isGlobal) {
        const res = this.getFieldForPath(pathOrGlobalGoal, x, y, hintIdx);
        if (!res) break;
        hintIdx = res.chunkIndex;
      }
      if (!this.sampleFlow(pathOrGlobalGoal, x, y, v1, hintIdx)) break;
      const mx = x + v1.x * (stepSize * 0.5), my = y + v1.y * (stepSize * 0.5);
      if (this.sampleFlow(pathOrGlobalGoal, mx, my, v2, hintIdx)) {
        x += v2.x * stepSize;
        y += v2.y * stepSize;
      } else {
        x += v1.x * stepSize;
        y += v1.y * stepSize;
      }
      pts.push({ x, y });
    }
    return simplifyTol > 0 ? simplifyPolyline(pts, simplifyTol) : pts;
  }
}

export default FlowFieldPathfinder;
//#endregion
// flowFieldApi.js

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
  ALL: 0xFF
});

export const clamp = (v, min, max) => Math.min(Math.max(v, min), max);

export const octile = (dx, dy) => {
  const ax = Math.abs(dx), ay = Math.abs(dy);
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
  get isEmpty() { return !this.sz; }
  clear() { this.sz = 0; }
  push(item, key) {
    let { it, k, sz } = this;
    if (sz === it.length) {
      const nSz = sz * 2;
      (this.it = new Int32Array(nSz)).set(it);
      (this.k = new Float64Array(nSz)).set(k);
      it = this.it; k = this.k;
    }
    let c = this.sz++, p;
    while (c > 0 && k[p = (c - 1) >> 1] > key) {
      it[c] = it[p]; k[c] = k[p]; c = p;
    }
    it[c] = item; k[c] = key;
  }
  pop() {
    let { it, k } = this, top = it[0], li = --this.sz, item = it[li], key = k[li], p = 0, ch;
    this.lastKey = k[0];
    while ((ch = p * 2 + 1) < li) {
      if (ch + 1 < li && k[ch + 1] < k[ch]) ch++;
      if (k[ch] >= key) break;
      it[p] = it[ch]; k[p] = k[ch]; p = ch;
    }
    it[p] = item; k[p] = key;
    return top;
  }
}
//#endregion

//#region 3: Multi-Layer Chunked Cost Grid
/**
 * 2D chunked cost grid supporting static costs, obstacle clearance (#5),
 * wall SDF repulsion (#4), tactical overlays (#6), crowd congestion (#9),
 * and anisotropic directional masks & currents (#11).
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
    this.dirtyChunks = new Set();

    // Solver & feature configuration
    this.useEikonal = true;
    this.losSmoothing = true;

    // #4: Wall Repulsion / Signed Distance Field (SDF)
    this.wallPenaltyRadius = 2.5;
    this.wallPenaltyWeight = 0.0;
    this.wallRepulsionWeight = 0.0;
    this.wallDist = null;
    this.wallGradX = null;
    this.wallGradY = null;
    this._sdfDirty = true;

    // #5: Clearance-Annotated Routing (unit size in cells)
    this.clearance = null;
    this._clearanceDirty = true;

    // #6: Tactical / Threat-Modulated Overlay
    this.tacticalCosts = null;
    this.hasTactical = false;

    // #9: Continuum Crowd Congestion Layer
    this.congestion = null;
    this.congestionWeight = 1.0;
    this.hasCongestion = false;

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

  markCellDirty(x, y) {
    const c = this.chunkIdAt(x, y);
    this.dirtyChunks.add(c);
    const lx = x % CS, ly = y % CS;
    if (lx === 0 && x > 0) this.dirtyChunks.add(c - 1);
    if (lx === CS - 1 && x < this.width - 1) this.dirtyChunks.add(c + 1);
    if (ly === 0 && y > 0) this.dirtyChunks.add(c - this.cx);
    if (ly === CS - 1 && y < this.height - 1) this.dirtyChunks.add(c + this.cx);
    this._sdfDirty = true;
    this._clearanceDirty = true;
  }

  getCost(x, y) {
    return this.inBounds(x | 0, y | 0) ? this.costs[(y | 0) * this.width + (x | 0)] : COST_BLOCK;
  }

  setCost(x, y, cost) {
    const ix = x | 0, iy = y | 0;
    if (!this.inBounds(ix, iy)) return false;
    const idx = iy * this.width + ix;
    if (this.costs[idx] === cost) return false;
    this.costs[idx] = cost;
    this.markCellDirty(ix, iy);
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
    for (let c = 0; c < this.chunkCount; c++) this.dirtyChunks.add(c);
    this._sdfDirty = true;
    this._clearanceDirty = true;
  }

  consumeDirtyChunks() {
    const out = new Set(this.dirtyChunks);
    this.dirtyChunks.clear();
    return out;
  }

  // #4: Wall Repulsion / Signed Distance Field (2-Pass Chamfer EDT + Gradient)
  enableWallRepulsion({ penaltyWeight = 1.5, repulsionWeight = 0.35, radius = 2.5 } = {}) {
    this.wallPenaltyWeight = penaltyWeight;
    this.wallRepulsionWeight = repulsionWeight;
    this.wallPenaltyRadius = radius;
    this.updateWallSDF();
  }

  updateWallSDF() {
    const { width: W, height: H, costs } = this, len = W * H;
    if (!this.wallDist) {
      this.wallDist = new Float32Array(len);
      this.wallGradX = new Float32Array(len);
      this.wallGradY = new Float32Array(len);
    }
    const D = this.wallDist, GX = this.wallGradX, GY = this.wallGradY;
    const INF = 1e6;
    for (let i = 0; i < len; i++) D[i] = costs[i] >= COST_BLOCK ? 0 : INF;

    // Forward pass
    for (let y = 0; y < H; y++) {
      for (let x = 0; x < W; x++) {
        const i = y * W + x;
        let d = D[i];
        if (d === 0) continue;
        if (x > 0 && D[i - 1] + 1 < d) d = D[i - 1] + 1;
        if (y > 0 && D[i - W] + 1 < d) d = D[i - W] + 1;
        if (x > 0 && y > 0 && D[i - W - 1] + SQ2 < d) d = D[i - W - 1] + SQ2;
        if (x < W - 1 && y > 0 && D[i - W + 1] + SQ2 < d) d = D[i - W + 1] + SQ2;
        D[i] = d;
      }
    }
    // Backward pass
    for (let y = H - 1; y >= 0; y--) {
      for (let x = W - 1; x >= 0; x--) {
        const i = y * W + x;
        let d = D[i];
        if (d === 0) continue;
        if (x < W - 1 && D[i + 1] + 1 < d) d = D[i + 1] + 1;
        if (y < H - 1 && D[i + W] + 1 < d) d = D[i + W] + 1;
        if (x < W - 1 && y < H - 1 && D[i + W + 1] + SQ2 < d) d = D[i + W + 1] + SQ2;
        if (x > 0 && y < H - 1 && D[i + W - 1] + SQ2 < d) d = D[i + W - 1] + SQ2;
        D[i] = d;
      }
    }
    // Normalized outward wall gradient
    for (let y = 0; y < H; y++) {
      for (let x = 0; x < W; x++) {
        const i = y * W + x;
        const dE = D[y * W + Math.min(W - 1, x + 1)], dW = D[y * W + Math.max(0, x - 1)];
        const dS = D[Math.min(H - 1, y + 1) * W + x], dN = D[Math.max(0, y - 1) * W + x];
        const gx = dE - dW, gy = dS - dN, l = Math.hypot(gx, gy);
        GX[i] = l > 1e-5 ? gx / l : 0;
        GY[i] = l > 1e-5 ? gy / l : 0;
      }
    }
    this._sdfDirty = false;
  }

  // #5: Clearance Map (Brushfire Chebyshev Clearance for Size-Aware Routing)
  updateClearance() {
    const { width: W, height: H, costs } = this, len = W * H;
    if (!this.clearance) this.clearance = new Uint8Array(len);
    const C = this.clearance;
    for (let y = 0; y < H; y++) {
      for (let x = 0; x < W; x++) {
        const i = y * W + x;
        if (costs[i] >= COST_BLOCK) {
          C[i] = 0;
        } else if (x === 0 || y === 0 || x === W - 1 || y === H - 1) {
          C[i] = 1;
        } else {
          C[i] = 255;
        }
      }
    }
    for (let y = 1; y < H - 1; y++) {
      for (let x = 1; x < W - 1; x++) {
        const i = y * W + x;
        if (!C[i]) continue;
        const m = Math.min(C[i - 1], C[i - W], C[i - W - 1], C[i - W + 1]);
        if (m + 1 < C[i]) C[i] = m + 1;
      }
    }
    for (let y = H - 2; y >= 1; y--) {
      for (let x = W - 2; x >= 1; x--) {
        const i = y * W + x;
        if (!C[i]) continue;
        const m = Math.min(C[i + 1], C[i + W], C[i + W + 1], C[i + W - 1]);
        if (m + 1 < C[i]) C[i] = m + 1;
      }
    }
    this._clearanceDirty = false;
  }

  // #6: Tactical / Threat-Modulated Overlay
  ensureTacticalLayer() {
    if (!this.tacticalCosts) this.tacticalCosts = new Float32Array(this.width * this.height);
    this.hasTactical = true;
    return this.tacticalCosts;
  }

  setTacticalCost(x, y, extraCost) {
    const ix = x | 0, iy = y | 0;
    if (!this.inBounds(ix, iy)) return;
    this.ensureTacticalLayer()[iy * this.width + ix] = Math.max(0, extraCost);
    this.dirtyChunks.add(this.chunkIdAt(ix, iy));
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
        this.dirtyChunks.add(this.chunkIdAt(x, y));
      }
    }
  }

  clearTacticalCosts() {
    if (this.tacticalCosts) this.tacticalCosts.fill(0);
    this.hasTactical = false;
    for (let c = 0; c < this.chunkCount; c++) this.dirtyChunks.add(c);
  }

  // #9: Continuum Crowd Congestion Layer
  ensureCongestionLayer() {
    if (!this.congestion) this.congestion = new Float32Array(this.width * this.height);
    this.hasCongestion = true;
    return this.congestion;
  }

  addCongestion(wx, wy, amount = 1.0) {
    const ix = wx | 0, iy = wy | 0;
    if (!this.inBounds(ix, iy)) return;
    this.ensureCongestionLayer()[iy * this.width + ix] += amount;
    this.dirtyChunks.add(this.chunkIdAt(ix, iy));
  }

  setCongestionFromAgents(px, py, count, unitWeight = 1.0) {
    const C = this.ensureCongestionLayer(), { width: W, height: H } = this;
    C.fill(0);
    for (let i = 0; i < count; i++) {
      const x = px[i] | 0, y = py[i] | 0;
      if (x >= 0 && x < W && y >= 0 && y < H) {
        C[y * W + x] += unitWeight;
        this.dirtyChunks.add(this.chunkIdAt(x, y));
      }
    }
  }

  decayCongestion(decayFactor = 0.85, eps = 0.05) {
    if (!this.congestion || !this.hasCongestion) return;
    const C = this.congestion, W = this.width;
    let active = false;
    for (let i = 0; i < C.length; i++) {
      if (C[i] > 0) {
        C[i] *= decayFactor;
        if (C[i] < eps) C[i] = 0;
        else active = true;
        this.dirtyChunks.add(this.chunkIdAt(i % W, (i / W) | 0));
      }
    }
    this.hasCongestion = active;
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
    this.markCellDirty(ix, iy);
  }

  setCurrent(x, y, vx, vy) {
    const ix = x | 0, iy = y | 0;
    if (!this.inBounds(ix, iy)) return;
    this.ensureAnisotropicLayer();
    const idx = iy * this.width + ix;
    this.currentX[idx] = vx;
    this.currentY[idx] = vy;
    this.markCellDirty(ix, iy);
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
        this.markCellDirty(xx, yy);
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
        // Downhill vector acts like a favorable current
        CX[i] = -(hE - hW) * 0.5 * slopeScale;
        CY[i] = -(hS - hN) * 0.5 * slopeScale;
      }
    }
    for (let c = 0; c < this.chunkCount; c++) this.dirtyChunks.add(c);
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
export function detectPortals(grid, minClearance = 1) {
  const portals = [], { cx, cy, width } = grid;
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
            const len = t - run, start = origin + run, mid = (len - 1) >> 1, cMid = start + mid;
            const cellsA = new Int32Array(len), cellsB = new Int32Array(len);
            for (let k = 0; k < len; k++) {
              cellsA[k] = at(start + k, border - 1);
              cellsB[k] = at(start + k, border);
            }
            portals.push({
              id: portals.length, uid: (bIdx * CS + run) * (CS + 1) + len,
              isH, chunkA: cA, chunkB: cB, border, start, end: start + len - 1,
              cellsA, cellsB, centerA: cellsA[mid], centerB: cellsB[mid],
              anchorX: isH ? cMid + 0.5 : border, anchorY: isH ? border : cMid + 0.5
            });
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
 * - #1: True Eikonal quadratic upwind solver
 * - #2: Multi-source geodesic Voronoi (`sourceOut`)
 * - #4: Wall SDF penalty buffer
 * - #5: Clearance filtering (`minClearance`)
 * - #6: Tactical cost overlays
 * - #8: 2-pass Flee / Evasion safety integration (`integrateFlee`)
 * - #9: Continuum crowd congestion weights
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

  /**
   * Extracts effective local chunk costs and passability masks from `grid`.
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
    const hasAniso = grid.hasAnisotropy && grid.dirMask;
    const wallR = grid.wallPenaltyRadius, wallW = grid.wallPenaltyWeight;

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
        if (hasCong) eff += grid.congestion[gIdx] * grid.congestionWeight;
        if (useWall) {
          const wd = grid.wallDist[gIdx];
          if (wd < wallR) eff += wallW * (1 - wd / wallR);
        }
        costs[lIdx] = Math.max(0.25, eff);
        if (hasAniso) {
          dirMasks[lIdx] = grid.dirMask[gIdx];
          curX[lIdx] = grid.currentX[gIdx];
          curY[lIdx] = grid.currentY[gIdx];
        } else {
          dirMasks[lIdx] = 0xFF;
        }
      }
    }
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
    const { costs, blocked, dirMasks, curX, curY, dist, heap } = this;
    const hasAniso = grid.hasAnisotropy;
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

        let stepScale = 0.5 * (cu + costs[v]) * 0.5;
        if (hasAniso) {
          // If backward-integrating from sink, agent travels from v -> u along REV_D[d]
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

        // #1: True Eikonal quadratic upwind refinement (for isotropic or mild-current cells)
        if (useEikonal && !hasAniso) {
          const tx = Math.min(
            nx > 0 ? dist[ny * CS + nx - 1] : Infinity,
            nx < CS - 1 ? dist[ny * CS + nx + 1] : Infinity
          );
          const ty = Math.min(
            ny > 0 ? dist[(ny - 1) * CS + nx] : Infinity,
            ny < CS - 1 ? dist[(ny + 1) * CS + nx] : Infinity
          );
          const diff = Math.abs(tx - ty);
          if (diff < stepScale) {
            const eik = 0.5 * (tx + ty + Math.sqrt(2 * stepScale * stepScale - diff * diff));
            if (eik < nd && eik >= Math.max(tx, ty)) nd = eik;
          } else {
            const eik = Math.min(tx, ty) + stepScale;
            if (eik < nd) nd = eik;
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
 * High-level abstract graph connecting chunk boundary portals in CSR format.
 * Supports incremental rebuilds, size-aware clearance (#5), and anisotropic edges (#11).
 */
export class PortalGraph {
  constructor(grid, prevGraph = null, dirtyChunks = null, minClearance = 1) {
    this.grid = grid;
    this.minClearance = minClearance;
    const portals = this.portals = detectPortals(grid, minClearance), P = portals.length;
    const lists = Array.from({ length: grid.chunkCount }, () => []);
    for (let i = 0; i < P; i++) {
      const p = portals[i];
      lists[p.chunkA].push(p.id); lists[p.chunkB].push(p.id);
    }
    const chunkPortals = this.chunkPortals = lists.map(l => new Int32Array(l));
    const canReuse = prevGraph && prevGraph.minClearance === minClearance && dirtyChunks;
    this.chunkDistCache = Array.from({ length: grid.chunkCount }, (_, c) =>
      canReuse && !dirtyChunks.has(c) ? prevGraph.chunkDistCache[c] : new Map()
    );
    const adj = Array.from({ length: P }, () => []), it = new ChunkIntegrator(), field = new Float64Array(CC);
    const itOpts = { minClearance };
    for (let c = 0; c < grid.chunkCount; c++) {
      const ids = chunkPortals[c];
      if (ids.length < 2) continue;
      const cCache = this.chunkDistCache[c];
      for (let ia = 0; ia < ids.length; ia++) {
        const idA = ids[ia], pA = portals[idA], locA = pLocal(grid, pA, c);
        let row = cCache.get(locA);
        if (!row) {
          // Integrating backward from sink `idA` gives cost from any `idB` -> `idA`
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
            adj[idB].push({ to: idA, cost: d + crossA, chunk: c });
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
  }
}

/**
 * Hierarchical A* pathfinder operating over `PortalGraph`.
 * Supports single goals and Multi-Source Geodesic Voronoi goals (#2).
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
        const fgx = (chosenGoalCell % grid.width) + 0.5, fgy = ((chosenGoalCell / grid.width) | 0) + 0.5;
        return {
          portals: new Int32Array(portals.reverse()),
          chunks: new Int32Array(chunks.reverse()),
          cost: g[GOAL],
          goalCell: chosenGoalCell,
          goalX: fgx,
          goalY: fgy,
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
  let cx = x0 | 0, cy = y0 | 0;
  const ex = x1 | 0, ey = y1 | 0;
  const dx = Math.abs(ex - cx), dy = Math.abs(ey - cy);
  const sx = cx < ex ? 1 : -1, sy = cy < ey ? 1 : -1;
  let err = dx - dy;
  while (true) {
    const c = costs[cy * CS + cx];
    if (c >= COST_BLOCK || c > refCost) return false;
    if (cx === ex && cy === ey) return true;
    const e2 = 2 * err;
    if (e2 > -dy && e2 < dx) {
      // Diagonal step: check both orthogonal corners
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
 */
export function computeFlowVectors(f, exX, exY, opts = {}) {
  const { integration: I, flowX: FX, flowY: FY } = f;
  const at = (x, y) => x >>> 0 >= CS || y >>> 0 >= CS ? Infinity : I[y * CS + x];
  const grid = opts.grid || null;
  const useWallRep = grid && grid.wallRepulsionWeight > 0 && grid.wallDist;
  const vortex = opts.vortex || f.vortex || null;
  const hasLOS = opts.losSmoothing && opts.chunkCosts && opts.targetLocalX !== undefined;
  const txL = opts.targetLocalX, tyL = opts.targetLocalY, cCosts = opts.chunkCosts;
  const refCost = hasLOS ? cCosts[(tyL | 0) * CS + (txL | 0)] : COST_OPEN;

  for (let y = 0; y < CS; y++) {
    for (let x = 0; x < CS; x++) {
      const id = y * CS + x, cur = I[id];
      if (cur === Infinity) { FX[id] = FY[id] = 0; continue; }
      if (!cur && !vortex) { FX[id] = exX; FY[id] = exY; continue; }

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
          dx = bx / l2 || 0; dy = by / l2 || 0;
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
export function buildCorridorField(grid, chunkId, polyline, opts = {}) {
  const ox = grid.chunkOriginX(chunkId), oy = grid.chunkOriginY(chunkId);
  const f = {
    chunkId, kind: 'c', originX: ox, originY: oy,
    integration: new Float32Array(CC), flowX: new Float32Array(CC), flowY: new Float32Array(CC),
    sourceId: new Int32Array(CC)
  };
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

  const it = new ChunkIntegrator();
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
 * LRU cache of per-chunk flow fields supporting Portal, Goal, Multi-Goal (#2),
 * Vortex (#7), Flee (#8), and Corridor (#10) fields.
 */
export class FlowFieldCache {
  constructor(grid, cap = 2048) {
    this.grid = grid; this.cap = cap; this.entries = new Map();
    this.it = new ChunkIntegrator();
  }
  key(c, kind, tid, minClearance = 1) {
    return ((tid * 8 + (kind.charCodeAt(0) & 7)) * 16 + minClearance) * 65536 + c;
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
      this.entries.delete(this.entries.keys().next().value);
    }
    this.entries.set(k, f);
    return f;
  }
  getPortalField(c, p, minClearance = 1) {
    const k = this.key(c, 'p', p.uid, minClearance);
    const hit = this.entries.get(k);
    if (hit) return this._promote(k, hit);
    return this.build(k, c, 'p', portalSideCells(p, c).map(x => this.grid.localIndex(x)), portalExitX(p, c), portalExitY(p, c), { minClearance });
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
    const { grid } = this;
    const f = {
      chunkId: c, kind: 'm', originX: grid.chunkOriginX(c), originY: grid.chunkOriginY(c),
      integration: new Float32Array(CC), flowX: new Float32Array(CC), flowY: new Float32Array(CC),
      sourceId: new Int32Array(CC)
    };
    this.it.integrate(grid, c, sources, f.integration, { minClearance, sourceOut: f.sourceId });
    computeFlowVectors(f, 0, 0, { grid });
    return f;
  }
  getVortexField(c, centerCell, vortexOpts = {}) {
    const { grid } = this, loc = grid.localIndex(centerCell);
    const f = {
      chunkId: c, kind: 'v', originX: grid.chunkOriginX(c), originY: grid.chunkOriginY(c),
      integration: new Float32Array(CC), flowX: new Float32Array(CC), flowY: new Float32Array(CC),
      vortex: vortexOpts
    };
    this.it.integrate(grid, c, [loc], f.integration, { minClearance: vortexOpts.minClearance ?? 1 });
    computeFlowVectors(f, 0, 0, {
      grid,
      vortex: vortexOpts,
      losSmoothing: grid.losSmoothing,
      chunkCosts: this.it.costs,
      targetLocalX: (loc % CS) + 0.5,
      targetLocalY: ((loc / CS) | 0) + 0.5
    });
    return f;
  }
  getFleeField(c, threatLocalCells, opts = {}) {
    const { grid } = this;
    const f = {
      chunkId: c, kind: 'f', originX: grid.chunkOriginX(c), originY: grid.chunkOriginY(c),
      integration: new Float32Array(CC), flowX: new Float32Array(CC), flowY: new Float32Array(CC)
    };
    this.it.integrateFlee(grid, c, threatLocalCells, f.integration, opts);
    computeFlowVectors(f, 0, 0, { grid });
    return f;
  }
  clear() { this.entries.clear(); }
  invalidateChunks(dirtyChunks) {
    for (const [k, f] of this.entries) {
      if (dirtyChunks.has(f.chunkId)) this.entries.delete(k);
    }
  }
  build(k, cid, kind, src, exX, exY, opts = {}) {
    const { grid } = this, f = {
      chunkId: cid, kind, originX: grid.chunkOriginX(cid), originY: grid.chunkOriginY(cid),
      integration: new Float32Array(CC), flowX: new Float32Array(CC), flowY: new Float32Array(CC),
      sourceId: opts.trackSources ? new Int32Array(CC) : null
    };
    this.it.integrate(grid, cid, src, f.integration, {
      minClearance: opts.minClearance ?? 1,
      sourceOut: f.sourceId
    });
    computeFlowVectors(f, exX, exY, {
      grid,
      vortex: opts.vortex,
      losSmoothing: opts.losSmoothing,
      chunkCosts: this.it.costs,
      targetLocalX: opts.targetLocalX,
      targetLocalY: opts.targetLocalY
    });
    return this._store(k, f);
  }
}

/**
 * Bilinearly interpolates the flow direction vector `(out.x, out.y)` at world position `(px, py)`.
 */
export function sampleFlowBilinear(f, px, py, out = { x: 0, y: 0 }) {
  const lx = px - f.originX - 0.5, ly = py - f.originY - 0.5;
  const x0 = Math.floor(lx), y0 = Math.floor(ly);
  const tx = lx - x0, ty = ly - y0;
  const { integration: I, flowX: FX, flowY: FY } = f;
  const w00 = (1 - tx) * (1 - ty), w10 = tx * (1 - ty), w01 = (1 - tx) * ty, w11 = tx * ty;
  const cx0 = x0 < 0 ? 0 : (x0 > 15 ? 15 : x0);
  const cx1 = (x0 + 1) < 0 ? 0 : ((x0 + 1) > 15 ? 15 : (x0 + 1));
  const cy0 = y0 < 0 ? 0 : (y0 > 15 ? 15 : y0);
  const cy1 = (y0 + 1) < 0 ? 0 : ((y0 + 1) > 15 ? 15 : (y0 + 1));
  let sx = 0, sy = 0, ws = 0;
  if (w00 > 0) { const id = cy0 * 16 + cx0; if (I[id] < Infinity) { sx += FX[id] * w00; sy += FY[id] * w00; ws += w00; } }
  if (w10 > 0) { const id = cy0 * 16 + cx1; if (I[id] < Infinity) { sx += FX[id] * w10; sy += FY[id] * w10; ws += w10; } }
  if (w01 > 0) { const id = cy1 * 16 + cx0; if (I[id] < Infinity) { sx += FX[id] * w01; sy += FY[id] * w01; ws += w01; } }
  if (w11 > 0) { const id = cy1 * 16 + cx1; if (I[id] < Infinity) { sx += FX[id] * w11; sy += FY[id] * w11; ws += w11; } }
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
//#endregion

//#region 7: Unified FlowFieldPathfinder API
/**
 * Unified Hierarchical Flow Field Pathfinding API exposing all 11 flow field types.
 */
export class FlowFieldPathfinder {
  constructor(opts = {}) {
    const cx = opts.width ? Math.ceil(opts.width / CS) : (opts.chunksX ?? 10);
    const cy = opts.height ? Math.ceil(opts.height / CS) : (opts.chunksY ?? 10);
    this.cacheCap = opts.cacheCap ?? 2048;
    this.minClearance = opts.minClearance ?? 1;
    this.grid = new GridWorld(cx, cy, opts.costs || null);
    if (opts.useEikonal !== undefined) this.grid.useEikonal = opts.useEikonal;
    if (opts.losSmoothing !== undefined) this.grid.losSmoothing = opts.losSmoothing;
    if (opts.wallRepulsion) this.grid.enableWallRepulsion(opts.wallRepulsion);

    this.graph = new PortalGraph(this.grid, null, null, this.minClearance);
    this.macro = new MacroPathfinder(this.grid, this.graph);
    this.cache = new FlowFieldCache(this.grid, this.cacheCap);
    this.clearanceGraphs = new Map([[this.minClearance, { graph: this.graph, macro: this.macro }]]);
  }

  get width() { return this.grid.width; }
  get height() { return this.grid.height; }

  getCost(x, y) { return this.grid.getCost(x, y); }
  setCost(x, y, cost) { return this.grid.setCost(x, y, cost); }
  setRect(x, y, w, h, cost) { return this.grid.setRect(x, y, w, h, cost); }
  setDisc(x, y, r, cost) { return this.grid.setDisc(x, y, r, cost); }
  loadCosts(costs) { this.grid.loadCosts(costs); }

  /**
   * Incrementally rebuilds portal graphs and invalidates cached flow fields for modified chunks.
   */
  rebuild() {
    const dirty = this.grid.consumeDirtyChunks();
    if (dirty.size === 0) return false;
    if (this.grid.wallPenaltyWeight > 0 || this.grid.wallRepulsionWeight > 0) this.grid.updateWallSDF();
    this.cache.invalidateChunks(dirty);
    for (const [clr, entry] of this.clearanceGraphs) {
      const nextGraph = new PortalGraph(this.grid, entry.graph, dirty, clr);
      const nextMacro = new MacroPathfinder(this.grid, nextGraph);
      this.clearanceGraphs.set(clr, { graph: nextGraph, macro: nextMacro });
      if (clr === this.minClearance) {
        this.graph = nextGraph;
        this.macro = nextMacro;
      }
    }
    return true;
  }

  _getHierarchy(minClearance = this.minClearance) {
    if (this.grid.dirtyChunks.size > 0) this.rebuild();
    let entry = this.clearanceGraphs.get(minClearance);
    if (!entry) {
      const graph = new PortalGraph(this.grid, null, null, minClearance);
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
   * @param {number} sx - Start world X.
   * @param {number} sy - Start world Y.
   * @param {Array<{x:number, y:number, weight?:number}>} goals - Candidate targets.
   * @param {Object} [opts={}] - `{ minClearance }`
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
   * #7: Builds a Vortex / Orbit / Strafe flow field around `(wx, wy)`.
   */
  getVortexField(wx, wy, opts = {}) {
    const { grid } = this;
    const cx = clamp(wx | 0, 0, grid.width - 1), cy = clamp(wy | 0, 0, grid.height - 1);
    const chunk = grid.chunkIdAt(cx, cy), cell = grid.index(cx, cy);
    return this.cache.getVortexField(chunk, cell, opts);
  }

  /**
   * #8: Builds a 2-Pass Flee / Evasion safety flow field for the chunk at `(wx, wy)` away from `threatPoints`.
   * @param {number} wx - World X inside the chunk.
   * @param {number} wy - World Y inside the chunk.
   * @param {Array<{x:number, y:number}>} threatPoints - Threat world positions.
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
   * #10: Builds a Guide-Curve / Spline Corridor flow field for the chunk at `(wx, wy)`.
   */
  getCorridorField(wx, wy, polyline, opts = {}) {
    const { grid } = this;
    const cx = clamp(wx | 0, 0, grid.width - 1), cy = clamp(wy | 0, 0, grid.height - 1);
    return buildCorridorField(grid, grid.chunkIdAt(cx, cy), polyline, opts);
  }

  /**
   * Resolves the 16x16 flow field for the chunk at `(wx, wy)` along a given `path`.
   */
  getFieldForPath(path, wx, wy, hintIdx = 0) {
    if (!path) return null;
    const clr = path.minClearance ?? this.minClearance;
    const { graph } = this._getHierarchy(clr);
    if (path.graph !== graph) return null;
    const { grid } = this;
    const cx = clamp(wx | 0, 0, grid.width - 1), cy = clamp(wy | 0, 0, grid.height - 1);
    const chunk = grid.chunkIdAt(cx, cy);
    let idx = hintIdx;
    if (path.chunks[idx] !== chunk) {
      idx = path.chunks.indexOf(chunk, hintIdx);
      if (idx < 0) idx = path.chunks.lastIndexOf(chunk, hintIdx);
      if (idx < 0) return null;
    }
    const field = idx < path.portals.length
      ? this.cache.getPortalField(chunk, graph.portals[path.portals[idx]], clr)
      : this.cache.getGoalField(chunk, path.goalCell, clr);
    if (!(field.integration[(cy - field.originY) * CS + (cx - field.originX)] < Infinity)) {
      return null;
    }
    return { field, chunkIndex: idx };
  }

  /**
   * Samples the bilinear flow direction `(out.x, out.y)` at `(wx, wy)` along `path`.
   */
  sampleFlow(path, wx, wy, out = { x: 0, y: 0 }, hintIdx = 0) {
    const res = this.getFieldForPath(path, wx, wy, hintIdx);
    if (!res) { out.x = 0; out.y = 0; return false; }
    return sampleFlowBilinear(res.field, wx, wy, out);
  }
}

export default FlowFieldPathfinder;
//#endregion
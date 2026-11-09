// flow-field-pathfinder-api-alt.js

//#region 1: Constants & Math Utilities
/** @constant {number} Chunk width/height in cells (16x16) */
export const CS = 16;

/** @constant {number} Total cells per chunk (256) */
export const CC = CS * CS;

/** @constant {number} Default open terrain traversal cost */
export const COST_OPEN = 2;

/** @constant {number} Impassable obstacle cost threshold (255) */
export const COST_BLOCK = 255;

/** @constant {number} Maximum portal width before splitting (#A2) */
export const MAX_PORTAL_LEN = 6;

/** @constant {number} Maximum tracked brushfire clearance radius in cells (#B2) */
export const MAX_CLEARANCE = 16;

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

/** Fast 32-bit FNV-1a mixer for composite cache keys (#B4) */
const hashMix = (h, v) => Math.imul((h ^ (v | 0)) >>> 0, 16777619) >>> 0;
//#endregion

//#region 2: 4-Ary Cache-Friendly Min-Heap (#B5)
/**
 * #B5: TypedArray-backed 4-ary min-heap for Eikonal, Dijkstra, and A* searches.
 * Halves tree depth and improves CPU cache-line locality during sift-down.
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
    while (c > 0 && k[p = (c - 1) >> 2] > key) {
      it[c] = it[p]; k[c] = k[p]; c = p;
    }
    it[c] = item; k[c] = key;
  }
  pop() {
    let { it, k } = this, top = it[0], li = --this.sz, item = k[li], key = k[li], p = 0, ch;
    item = it[li];
    this.lastKey = k[0];
    while ((ch = (p << 2) + 1) < li) {
      let mc = ch, mk = k[ch];
      const c1 = ch + 1, c2 = ch + 2, c3 = ch + 3;
      if (c1 < li && k[c1] < mk) { mc = c1; mk = k[c1]; }
      if (c2 < li && k[c2] < mk) { mc = c2; mk = k[c2]; }
      if (c3 < li && k[c3] < mk) { mc = c3; mk = k[c3]; }
      if (mk >= key) break;
      it[p] = it[mc]; k[p] = mk; p = mc;
    }
    it[p] = item; k[p] = key;
    return top;
  }
}
//#endregion

//#region 3: Zero-Allocation Slab Buffer Pool (#B4)
/**
 * #B4: Zero-allocation slab pool for recycling chunk-sized TypedArrays on cache eviction.
 */
export class FieldBufferPool {
  constructor(maxSlabs = 512) {
    this.maxSlabs = maxSlabs;
    this.f32Pool = [];
    this.i32Pool = [];
  }
  allocF32() {
    return this.f32Pool.length > 0 ? this.f32Pool.pop() : new Float32Array(CC);
  }
  allocI32() {
    return this.i32Pool.length > 0 ? this.i32Pool.pop() : new Int32Array(CC);
  }
  releaseField(f) {
    if (!f || f._pooled) return;
    f._pooled = true;
    if (f.integration && this.f32Pool.length < this.maxSlabs * 3) {
      this.f32Pool.push(f.integration, f.flowX, f.flowY);
      f.integration = f.flowX = f.flowY = null;
    }
    if (f.sourceId && this.i32Pool.length < this.maxSlabs) {
      this.i32Pool.push(f.sourceId);
      f.sourceId = null;
    }
  }
}
//#endregion

//#region 4: GridWorld Core & Dirty State Tracking (#B1)
/**
 * 2D chunked cost grid supporting static costs, two-tier dirty tracking (#B1),
 * localized dirty-halo SDF & clearance updates (#B2), tactical FOV shadowcasting (#E1),
 * anisotropic velocity-flux congestion (#C1), dynamic obstacle stamps (#C3),
 * and terrain ID movement profiles (#D4).
 */
export class GridWorld {
  constructor(cx, cy, existingCosts = null) {
    this.cx = cx;
    this.cy = cy;
    this.width = cx * CS;
    this.height = cy * CS;
    this.chunkCount = cx * cy;
    const len = this.width * this.height;
    this.costs = existingCosts ? existingCosts.subarray(0, len) : new Uint8Array(len).fill(COST_OPEN);

    // #B1: Two-Tier Dirty Tracking (Topology vs. Weights)
    this.dirtyChunks = new Set();
    this.topologyDirtyChunks = new Set();
    this.weightsDirtyChunks = new Set();

    // Solver & feature configuration
    this.useEikonal = true;
    this.losSmoothing = true;

    // #4 & #B2: Wall Repulsion / Signed Distance Field (SDF) with Dirty-Halo Bounds
    this.wallPenaltyRadius = 2.5;
    this.wallPenaltyWeight = 0.0;
    this.wallRepulsionWeight = 0.0;
    this.wallDist = null;
    this.wallGradX = null;
    this.wallGradY = null;
    this._sdfDirty = true;
    this._sdfFullDirty = true;
    this._sdfRect = { x0: len, y0: len, x1: -1, y1: -1 };

    // #5 & #B2: Clearance-Annotated Routing with Dirty-Halo Bounds
    this.clearance = null;
    this._clearanceDirty = true;
    this._clearanceFullDirty = true;
    this._clearanceRect = { x0: len, y0: len, x1: -1, y1: -1 };

    // #6 & #E1: Tactical / Threat-Modulated Overlay
    this.tacticalCosts = null;
    this.hasTactical = false;

    // #9 & #C1: Continuum Crowd Congestion + Velocity-Flux Layer
    this.congestion = null;
    this.congestionFluxX = null;
    this.congestionFluxY = null;
    this.congestionWeight = 1.0;
    this.congestionLaneWeight = 0.65;
    this.hasCongestion = false;
    this.hasCongestionFlux = false;

    // #11 & #A5: Anisotropic Directional Bitmask & Vector Currents / Slopes
    this.dirMask = null;
    this.currentX = null;
    this.currentY = null;
    this.currentStrength = 0.5;
    this.hasAnisotropy = false;

    // #C3: Dynamic Moving Obstacle Stamp Layer
    this.dynamicObstacles = null;
    this.hasDynamicObstacles = false;

    // #D4: Terrain ID Layer & Unit Movement Cost Profiles
    this.terrainIds = null;
    this.profiles = new Map();
  }

  index(x, y) { return y * this.width + x; }
  inBounds(x, y) { return x >= 0 && y >= 0 && x < this.width && y < this.height; }

  /** #D4: Resolves a profile's Uint8Array(256) cost lookup table if registered */
  getProfileTable(profile = null) {
    if (!profile) return null;
    return profile instanceof Uint8Array ? profile : (this.profiles.get(profile) || null);
  }

  /** #D4: Returns base cell cost respecting terrain profiles and dynamic obstacles (#C3) */
  getEffectiveBaseCost(idx, profileTable = null) {
    if (this.hasDynamicObstacles && this.dynamicObstacles[idx]) return COST_BLOCK;
    if (profileTable && this.terrainIds) return profileTable[this.terrainIds[idx]];
    return this.costs[idx];
  }

  isWalkable(x, y, minClearance = 1, profile = null) {
    if (!this.inBounds(x, y)) return false;
    const idx = this.index(x, y);
    const pTable = this.getProfileTable(profile);
    if (this.getEffectiveBaseCost(idx, pTable) >= COST_BLOCK) return false;
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

  _addChunkAndBorderNeighbors(set, x, y) {
    const c = this.chunkIdAt(x, y);
    set.add(c);
    const lx = x % CS, ly = y % CS;
    if (lx === 0 && x > 0) set.add(c - 1);
    if (lx === CS - 1 && x < this.width - 1) set.add(c + 1);
    if (ly === 0 && y > 0) set.add(c - this.cx);
    if (ly === CS - 1 && y < this.height - 1) set.add(c + this.cx);
  }

  /**
   * #B1 & #B2: Marks a cell dirty, distinguishing structural topology changes from weight-only changes.
   */
  markCellDirty(x, y, isTopology = true) {
    this._addChunkAndBorderNeighbors(this.dirtyChunks, x, y);
    if (isTopology) {
      this._addChunkAndBorderNeighbors(this.topologyDirtyChunks, x, y);
      this._sdfDirty = true;
      this._clearanceDirty = true;
      if (x < this._sdfRect.x0) this._sdfRect.x0 = x;
      if (x > this._sdfRect.x1) this._sdfRect.x1 = x;
      if (y < this._sdfRect.y0) this._sdfRect.y0 = y;
      if (y > this._sdfRect.y1) this._sdfRect.y1 = y;
      if (x < this._clearanceRect.x0) this._clearanceRect.x0 = x;
      if (x > this._clearanceRect.x1) this._clearanceRect.x1 = x;
      if (y < this._clearanceRect.y0) this._clearanceRect.y0 = y;
      if (y > this._clearanceRect.y1) this._clearanceRect.y1 = y;
    } else {
      this._addChunkAndBorderNeighbors(this.weightsDirtyChunks, x, y);
    }
  }

  markChunkWeightDirty(c) {
    this.dirtyChunks.add(c);
    this.weightsDirtyChunks.add(c);
  }

  getCost(x, y) {
    return this.inBounds(x | 0, y | 0) ? this.costs[(y | 0) * this.width + (x | 0)] : COST_BLOCK;
  }

  setCost(x, y, cost) {
    const ix = x | 0, iy = y | 0;
    if (!this.inBounds(ix, iy)) return false;
    const idx = iy * this.width + ix, prev = this.costs[idx];
    if (prev === cost) return false;
    this.costs[idx] = cost;
    const topoChanged = (prev >= COST_BLOCK) !== (cost >= COST_BLOCK);
    this.markCellDirty(ix, iy, topoChanged);
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
    this._sdfDirty = this._sdfFullDirty = true;
    this._clearanceDirty = this._clearanceFullDirty = true;
  }

  consumeDirtyChunks() {
    const out = new Set(this.dirtyChunks);
    this.dirtyChunks.clear();
    this.topologyDirtyChunks.clear();
    this.weightsDirtyChunks.clear();
    return out;
  }

  /** #B1: Consumes and categorizes dirty chunks into topology vs. weight-only sets */
  consumeDirtyState() {
    const topology = new Set(this.topologyDirtyChunks);
    const weights = new Set(this.weightsDirtyChunks);
    const all = new Set(this.dirtyChunks);
    this.dirtyChunks.clear();
    this.topologyDirtyChunks.clear();
    this.weightsDirtyChunks.clear();
    return { topology, weights, all };
  }
//#endregion

//#region 5: Terrain ID Layer & Unit Cost Profiles (#D4)
  ensureTerrainLayer() {
    if (!this.terrainIds) this.terrainIds = new Uint8Array(this.width * this.height);
    return this.terrainIds;
  }

  registerProfile(name, costMapOrArray) {
    let table;
    if (costMapOrArray instanceof Uint8Array) {
      table = costMapOrArray;
    } else {
      table = new Uint8Array(256).fill(COST_OPEN);
      table[255] = COST_BLOCK;
      for (const [k, v] of Object.entries(costMapOrArray)) table[k | 0] = clamp(v | 0, 1, COST_BLOCK);
    }
    this.profiles.set(name, table);
    return table;
  }

  setTerrain(x, y, terrainId) {
    const ix = x | 0, iy = y | 0;
    if (!this.inBounds(ix, iy)) return false;
    const T = this.ensureTerrainLayer(), idx = iy * this.width + ix;
    if (T[idx] === terrainId) return false;
    T[idx] = terrainId;
    this.markCellDirty(ix, iy, true);
    return true;
  }

  setRectTerrain(x, y, w, h, terrainId) {
    const x0 = clamp(Math.floor(x), 0, this.width), x1 = clamp(Math.ceil(x + w), 0, this.width);
    const y0 = clamp(Math.floor(y), 0, this.height), y1 = clamp(Math.ceil(y + h), 0, this.height);
    for (let yy = y0; yy < y1; yy++) {
      for (let xx = x0; xx < x1; xx++) this.setTerrain(xx, yy, terrainId);
    }
  }
//#endregion

//#region 6: Dynamic Moving Obstacle Stamp Layer (#C3)
  ensureDynamicObstacleLayer() {
    if (!this.dynamicObstacles) this.dynamicObstacles = new Uint8Array(this.width * this.height);
    return this.dynamicObstacles;
  }

  stampDynamicObstacle(wx, wy, r = 0.8, blocked = 1) {
    const D = this.ensureDynamicObstacleLayer(), r2 = r * r;
    const x0 = clamp(Math.floor(wx - r), 0, this.width - 1), x1 = clamp(Math.ceil(wx + r), 0, this.width - 1);
    const y0 = clamp(Math.floor(wy - r), 0, this.height - 1), y1 = clamp(Math.ceil(wy + r), 0, this.height - 1);
    for (let y = y0; y <= y1; y++) {
      for (let x = x0; x <= x1; x++) {
        if ((x + 0.5 - wx) ** 2 + (y + 0.5 - wy) ** 2 <= r2) {
          const idx = y * this.width + x;
          if (D[idx] !== blocked) {
            D[idx] = blocked;
            this._addChunkAndBorderNeighbors(this.weightsDirtyChunks, x, y);
            this._addChunkAndBorderNeighbors(this.dirtyChunks, x, y);
          }
        }
      }
    }
    this.hasDynamicObstacles = true;
  }

  clearDynamicObstacles() {
    if (!this.dynamicObstacles || !this.hasDynamicObstacles) return;
    const D = this.dynamicObstacles, W = this.width;
    for (let i = 0; i < D.length; i++) {
      if (D[i]) {
        D[i] = 0;
        const x = i % W, y = (i / W) | 0;
        this._addChunkAndBorderNeighbors(this.weightsDirtyChunks, x, y);
        this._addChunkAndBorderNeighbors(this.dirtyChunks, x, y);
      }
    }
    this.hasDynamicObstacles = false;
  }
//#endregion

//#region 7: Wall Repulsion & Signed Distance Field (SDF) (#4, #B2)
  enableWallRepulsion({ penaltyWeight = 1.5, repulsionWeight = 0.35, radius = 2.5 } = {}) {
    this.wallPenaltyWeight = penaltyWeight;
    this.wallRepulsionWeight = repulsionWeight;
    this.wallPenaltyRadius = radius;
    this._sdfFullDirty = true;
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
    const maxD = Math.ceil(this.wallPenaltyRadius) + 4;
    const full = forceFull || this._sdfFullDirty || this._sdfRect.x1 < this._sdfRect.x0;
    const x0 = full ? 0 : Math.max(0, this._sdfRect.x0 - maxD);
    const y0 = full ? 0 : Math.max(0, this._sdfRect.y0 - maxD);
    const x1 = full ? W - 1 : Math.min(W - 1, this._sdfRect.x1 + maxD);
    const y1 = full ? H - 1 : Math.min(H - 1, this._sdfRect.y1 + maxD);

    for (let y = y0; y <= y1; y++) {
      const row = y * W;
      for (let x = x0; x <= x1; x++) {
        const i = row + x;
        D[i] = costs[i] >= COST_BLOCK ? 0 : maxD;
      }
    }
    // Forward pass (including halo boundary neighbors if regional)
    const fy0 = Math.max(0, y0 - 1), fx0 = Math.max(0, x0 - 1);
    const fy1 = Math.min(H - 1, y1 + 1), fx1 = Math.min(W - 1, x1 + 1);
    for (let y = fy0; y <= fy1; y++) {
      const row = y * W;
      for (let x = fx0; x <= fx1; x++) {
        const i = row + x;
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
    for (let y = fy1; y >= fy0; y--) {
      const row = y * W;
      for (let x = fx1; x >= fx0; x--) {
        const i = row + x;
        let d = D[i];
        if (d === 0) continue;
        if (x < W - 1 && D[i + 1] + 1 < d) d = D[i + 1] + 1;
        if (y < H - 1 && D[i + W] + 1 < d) d = D[i + W] + 1;
        if (x < W - 1 && y < H - 1 && D[i + W + 1] + SQ2 < d) d = D[i + W + 1] + SQ2;
        if (x > 0 && y < H - 1 && D[i + W - 1] + SQ2 < d) d = D[i + W - 1] + SQ2;
        D[i] = d;
      }
    }
    // Outward gradient within updated box
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
    this._sdfDirty = this._sdfFullDirty = false;
    this._sdfRect.x0 = this._sdfRect.y0 = len;
    this._sdfRect.x1 = this._sdfRect.y1 = -1;
  }
//#endregion

//#region 8: Clearance Map & Regional Brushfire (#5, #B2)
  updateClearance(forceFull = false) {
    const { width: W, height: H, costs } = this, len = W * H;
    if (!this.clearance) {
      this.clearance = new Uint8Array(len);
      this._clearanceFullDirty = true;
    }
    const C = this.clearance;
    const halo = MAX_CLEARANCE + 2;
    const full = forceFull || this._clearanceFullDirty || this._clearanceRect.x1 < this._clearanceRect.x0;
    const x0 = full ? 0 : Math.max(0, this._clearanceRect.x0 - halo);
    const y0 = full ? 0 : Math.max(0, this._clearanceRect.y0 - halo);
    const x1 = full ? W - 1 : Math.min(W - 1, this._clearanceRect.x1 + halo);
    const y1 = full ? H - 1 : Math.min(H - 1, this._clearanceRect.y1 + halo);

    for (let y = y0; y <= y1; y++) {
      const row = y * W;
      for (let x = x0; x <= x1; x++) {
        const i = row + x;
        if (costs[i] >= COST_BLOCK) C[i] = 0;
        else if (x === 0 || y === 0 || x === W - 1 || y === H - 1) C[i] = 1;
        else C[i] = MAX_CLEARANCE;
      }
    }
    const fy0 = Math.max(1, y0), fx0 = Math.max(1, x0);
    const fy1 = Math.min(H - 2, y1), fx1 = Math.min(W - 2, x1);
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
    this._clearanceDirty = this._clearanceFullDirty = false;
    this._clearanceRect.x0 = this._clearanceRect.y0 = len;
    this._clearanceRect.x1 = this._clearanceRect.y1 = -1;
  }
//#endregion

//#region 9: Tactical Overlay, FOV Shadowcasting & Raycasting (#6, #E1)
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

  /**
   * #E1: Adds tactical threat cost only to cells within Line-of-Sight of `(wx, wy)` using
   * 8-octant recursive shadowcasting so threat zones never bleed through solid walls.
   */
  addTacticalFOV(wx, wy, r, peakCost, linearFalloff = true) {
    const ox = wx | 0, oy = wy | 0;
    if (!this.inBounds(ox, oy)) return;
    const T = this.ensureTacticalLayer(), { width: W, costs } = this, r2 = r * r, maxRow = Math.ceil(r);
    T[oy * W + ox] += peakCost;
    this.markChunkWeightDirty(this.chunkIdAt(ox, oy));

    const scanOctant = (row, startSlope, endSlope, xx, xy, yx, yy) => {
      if (startSlope < endSlope) return;
      let nextStart = startSlope;
      for (let i = row; i <= maxRow; i++) {
        let blocked = false;
        for (let dx = -i, dy = -i; dx <= 0; dx++) {
          const lSlope = (dx - 0.5) / (dy + 0.5), rSlope = (dx + 0.5) / (dy - 0.5);
          if (startSlope < rSlope) continue;
          if (endSlope > lSlope) break;
          const mx = ox + dx * xx + dy * xy, my = oy + dx * yx + dy * yy;
          if (!this.inBounds(mx, my)) continue;
          const d2 = dx * dx + dy * dy, idx = my * W + mx;
          if (d2 <= r2 && costs[idx] < COST_BLOCK) {
            const w = linearFalloff ? 1 - Math.sqrt(d2) / r : 1;
            T[idx] += peakCost * w;
            this.markChunkWeightDirty(this.chunkIdAt(mx, my));
          }
          const isWall = !this.inBounds(mx, my) || costs[idx] >= COST_BLOCK;
          if (blocked) {
            if (isWall) nextStart = rSlope;
            else { blocked = false; startSlope = nextStart; }
          } else if (isWall && i < maxRow) {
            blocked = true;
            scanOctant(i + 1, startSlope, lSlope, xx, xy, yx, yy);
            nextStart = rSlope;
          }
        }
        if (blocked) break;
      }
    };

    const M = [
      [1, 0, 0, 1], [0, 1, 1, 0], [0, -1, 1, 0], [-1, 0, 0, 1],
      [-1, 0, 0, -1], [0, -1, -1, 0], [0, 1, -1, 0], [1, 0, 0, -1]
    ];
    for (let o = 0; o < 8; o++) {
      const m = M[o];
      scanOctant(1, 1.0, 0.0, m[0], m[1], m[2], m[3]);
    }
  }

  clearTacticalCosts() {
    if (!this.tacticalCosts || !this.hasTactical) return;
    const T = this.tacticalCosts, W = this.width;
    for (let i = 0; i < T.length; i++) {
      if (T[i] > 0) {
        T[i] = 0;
        this.markChunkWeightDirty(this.chunkIdAt(i % W, (i / W) | 0));
      }
    }
    this.hasTactical = false;
  }

  /**
   * #E1: Global world-space supercover DDA raycast supporting clearance and maxCost thresholds.
   */
  raycast(x0, y0, x1, y1, opts = {}) {
    const minClearance = opts.minClearance ?? 1;
    const maxCost = opts.maxCost ?? (COST_BLOCK - 1);
    const pTable = this.getProfileTable(opts.profile);
    if (minClearance > 1 && (this._clearanceDirty || !this.clearance)) this.updateClearance();

    let cx = Math.floor(x0), cy = Math.floor(y0);
    const ex = Math.floor(x1), ey = Math.floor(y1);
    const dx = Math.abs(ex - cx), dy = Math.abs(ey - cy);
    const sx = cx < ex ? 1 : -1, sy = cy < ey ? 1 : -1;
    let err = dx - dy;

    const checkCell = (x, y) => {
      if (!this.inBounds(x, y)) return false;
      const idx = y * this.width + x;
      if (this.getEffectiveBaseCost(idx, pTable) > maxCost) return false;
      if (minClearance > 1 && this.clearance[idx] < minClearance) return false;
      return true;
    };

    while (true) {
      if (!checkCell(cx, cy)) {
        const hx = clamp(cx + 0.5, 0, this.width), hy = clamp(cy + 0.5, 0, this.height);
        return { hit: true, x: hx, y: hy, cell: this.inBounds(cx, cy) ? cy * this.width + cx : -1, distance: Math.hypot(hx - x0, hy - y0) };
      }
      if (cx === ex && cy === ey) {
        return { hit: false, x: x1, y: y1, cell: ey * this.width + ex, distance: Math.hypot(x1 - x0, y1 - y0) };
      }
      const e2 = 2 * err;
      if (e2 > -dy && e2 < dx) {
        if (!checkCell(cx + sx, cy) || !checkCell(cx, cy + sy)) {
          const hx = cx + 0.5, hy = cy + 0.5;
          return { hit: true, x: hx, y: hy, cell: cy * this.width + cx, distance: Math.hypot(hx - x0, hy - y0) };
        }
      }
      if (e2 > -dy) { err -= dy; cx += sx; }
      if (e2 < dx) { err += dx; cy += sy; }
    }
  }
//#endregion

//#region 10: Continuum Crowd Congestion & Velocity-Flux Layer (#9, #C1)
  ensureCongestionLayer(withFlux = false) {
    const len = this.width * this.height;
    if (!this.congestion) this.congestion = new Float32Array(len);
    if (withFlux && !this.congestionFluxX) {
      this.congestionFluxX = new Float32Array(len);
      this.congestionFluxY = new Float32Array(len);
    }
    this.hasCongestion = true;
    if (withFlux) this.hasCongestionFlux = true;
    return this.congestion;
  }

  addCongestion(wx, wy, amount = 1.0, vx = 0, vy = 0) {
    const ix = wx | 0, iy = wy | 0;
    if (!this.inBounds(ix, iy)) return;
    const hasVel = vx !== 0 || vy !== 0;
    const C = this.ensureCongestionLayer(hasVel), idx = iy * this.width + ix;
    C[idx] += amount;
    if (hasVel) {
      this.congestionFluxX[idx] += vx * amount;
      this.congestionFluxY[idx] += vy * amount;
    }
    this.markChunkWeightDirty(this.chunkIdAt(ix, iy));
  }

  /** #C1: Stamps directional agent velocity flux for bi-directional lane formation */
  addAgentFlux(wx, wy, vx, vy, amount = 1.0) {
    this.addCongestion(wx, wy, amount, vx, vy);
  }

  setCongestionFromAgents(px, py, count, unitWeight = 1.0, vx = null, vy = null) {
    const withFlux = !!(vx && vy);
    const C = this.ensureCongestionLayer(withFlux), { width: W, height: H } = this;
    C.fill(0);
    if (withFlux) {
      this.congestionFluxX.fill(0);
      this.congestionFluxY.fill(0);
    }
    for (let i = 0; i < count; i++) {
      const x = px[i] | 0, y = py[i] | 0;
      if (x >= 0 && x < W && y >= 0 && y < H) {
        const idx = y * W + x;
        C[idx] += unitWeight;
        if (withFlux) {
          this.congestionFluxX[idx] += vx[i] * unitWeight;
          this.congestionFluxY[idx] += vy[i] * unitWeight;
        }
        this.markChunkWeightDirty(this.chunkIdAt(x, y));
      }
    }
  }

  decayCongestion(decayFactor = 0.85, eps = 0.05) {
    if (!this.congestion || !this.hasCongestion) return;
    const C = this.congestion, FX = this.congestionFluxX, FY = this.congestionFluxY, W = this.width;
    let active = false;
    for (let i = 0; i < C.length; i++) {
      if (C[i] > 0) {
        C[i] *= decayFactor;
        if (FX) { FX[i] *= decayFactor; FY[i] *= decayFactor; }
        if (C[i] < eps) {
          C[i] = 0;
          if (FX) FX[i] = FY[i] = 0;
        } else {
          active = true;
        }
        this.markChunkWeightDirty(this.chunkIdAt(i % W, (i / W) | 0));
      }
    }
    this.hasCongestion = active;
    if (!active) this.hasCongestionFlux = false;
  }
//#endregion

//#region 11: Anisotropic Passability, Vector Currents & Nearest Walkable (#11, #A5)
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

  findNearestWalkable(x, y, maxR = 24, minClearance = 1, profile = null) {
    const { width: W, height: H } = this, sx = clamp(x | 0, 0, W - 1), sy = clamp(y | 0, 0, H - 1);
    if (this.isWalkable(sx, sy, minClearance, profile)) return sy * W + sx;
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
            if (this.isWalkable(nx, ny, minClearance, profile) && d < bestD) { bestD = d; bestI = idx; }
          }
        }
      }
      if (bestI >= 0) return bestI;
    }
    return -1;
  }
}
//#endregion

//#region 12: Chunk Boundary Portal Detection & Crossing Costs (#A2, #A5, #D4)
/**
 * #A2 & #A5: Detects chunk boundary portals, splitting wide runs (> `maxPortalLen`) into sub-portals
 * and checking anisotropic directional masks across chunk borders.
 */
export function detectPortals(grid, minClearance = 1, maxPortalLen = MAX_PORTAL_LEN, profile = null) {
  const portals = [], { cx, cy, width } = grid;
  const hasDir = grid.hasAnisotropy && grid.dirMask;
  const pushSegment = (isH, cA, cB, border, bIdx, origin, segRun, segLen) => {
    const start = origin + segRun, mid = (segLen - 1) >> 1, cMid = start + mid;
    const cellsA = new Int32Array(segLen), cellsB = new Int32Array(segLen);
    const at = (u, v) => isH ? v * width + u : u * width + v;
    const dirAB = isH ? DIR_FLAGS.S : DIR_FLAGS.E;
    const dirBA = isH ? DIR_FLAGS.N : DIR_FLAGS.W;
    let canAB = false, canBA = false;
    for (let k = 0; k < segLen; k++) {
      const ca = at(start + k, border - 1), cb = at(start + k, border);
      cellsA[k] = ca; cellsB[k] = cb;
      if (!hasDir || (grid.dirMask[ca] & dirAB)) canAB = true;
      if (!hasDir || (grid.dirMask[cb] & dirBA)) canBA = true;
    }
    if (!canAB && !canBA) return;
    portals.push({
      id: portals.length,
      uid: (bIdx * CS + segRun) * (CS + 1) + segLen,
      isH, chunkA: cA, chunkB: cB, border, start, end: start + segLen - 1,
      cellsA, cellsB, centerA: cellsA[mid], centerB: cellsB[mid],
      canAB, canBA,
      anchorX: isH ? cMid + 0.5 : border, anchorY: isH ? border : cMid + 0.5
    });
  };

  for (const isH of [false, true]) {
    const countX = cx - !isH, countY = cy - isH, bOffset = isH ? cy * (cx - 1) : 0;
    const W = (u, v) => grid.isWalkable(isH ? u : v, isH ? v : u, minClearance, profile);
    const at = (u, v) => isH ? v * width + u : u * width + v;
    const dirAB = isH ? DIR_FLAGS.S : DIR_FLAGS.E;
    const dirBA = isH ? DIR_FLAGS.N : DIR_FLAGS.W;
    for (let y = 0; y < countY; y++) {
      for (let x = 0; x < countX; x++) {
        const border = ((isH ? y : x) + 1) * CS, origin = (isH ? x : y) * CS;
        const cA = y * cx + x, cB = cA + (isH ? cx : 1), bIdx = bOffset + y * countX + x;
        let run = -1;
        for (let t = 0; t <= CS; t++) {
          let pass = t < CS && W(origin + t, border - 1) && W(origin + t, border);
          if (pass && hasDir) {
            const ca = at(origin + t, border - 1), cb = at(origin + t, border);
            pass = ((grid.dirMask[ca] & dirAB) !== 0) || ((grid.dirMask[cb] & dirBA) !== 0);
          }
          if (pass) {
            if (run < 0) run = t;
          } else if (run >= 0) {
            const totalLen = t - run;
            const splits = maxPortalLen > 0 ? Math.ceil(totalLen / maxPortalLen) : 1;
            let cur = run;
            for (let s = 0; s < splits; s++) {
              const rem = (run + totalLen) - cur;
              const segLen = Math.ceil(rem / (splits - s));
              pushSegment(isH, cA, cB, border, bIdx, origin, cur, segLen);
              cur += segLen;
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
export const pLocal = (grid, p, c) => grid.localIndex(portalCenterCell(p, c));

/** #A5 & #D4: Anisotropic and profile-aware portal boundary crossing cost */
export const crossingCost = (grid, p, fromChunk = p.chunkA, profile = null) => {
  if (fromChunk === p.chunkA && p.canAB === false) return Infinity;
  if (fromChunk === p.chunkB && p.canBA === false) return Infinity;
  const pTable = grid.getProfileTable(profile);
  const cA = grid.getEffectiveBaseCost(p.centerA, pTable);
  const cB = grid.getEffectiveBaseCost(p.centerB, pTable);
  if (cA >= COST_BLOCK || cB >= COST_BLOCK) return Infinity;
  let cost = ((cA + cB) * 0.5) / COST_OPEN;
  if (grid.hasAnisotropy && grid.currentX) {
    const fromCell = fromChunk === p.chunkA ? p.centerA : p.centerB;
    const dx = portalExitX(p, fromChunk), dy = portalExitY(p, fromChunk);
    const dot = dx * grid.currentX[fromCell] + dy * grid.currentY[fromCell];
    cost *= clamp(1 - dot * grid.currentStrength, 0.2, 4.0);
  }
  return cost;
};
//#endregion

//#region 13: Multi-Mode Eikonal & Flee Chunk Integrator (#1, #A5, #C1, #I1)
/**
 * Multi-mode chunk integrator supporting:
 * - #1, #A5 & #I1: Float32 per-cell Eikonal quadratic upwind solver
 * - #2: Multi-source geodesic Voronoi (`sourceOut`)
 * - #4: Wall SDF penalty buffer
 * - #5: Clearance filtering (`minClearance`)
 * - #6: Tactical cost overlays
 * - #8: 2-pass Flee / Evasion safety integration (`integrateFlee`)
 * - #9 & #C1: Continuum crowd congestion + directional velocity-flux lane formation
 * - #11: Anisotropic directional masks & vector currents
 * - #C3 & #D4: Dynamic moving obstacles & terrain movement profiles
 */
export class ChunkIntegrator {
  heap = new MinHeap(CC * 2);
  costs = new Float32Array(CC);
  blocked = new Uint8Array(CC);
  dirMasks = new Uint8Array(CC);
  curX = new Float32Array(CC);
  curY = new Float32Array(CC);
  fluxX = new Float32Array(CC);
  fluxY = new Float32Array(CC);
  dist = new Float32Array(CC);
  fleeBuf = new Float32Array(CC);

  prepareChunk(grid, chunkId, minClearance = 1, profile = null) {
    const { costs, blocked, dirMasks, curX, curY, fluxX, fluxY } = this;
    const ox = grid.chunkOriginX(chunkId), oy = grid.chunkOriginY(chunkId), W = grid.width;
    const useClearance = minClearance > 1;
    if (useClearance && (grid._clearanceDirty || !grid.clearance)) grid.updateClearance();
    const useWall = grid.wallPenaltyWeight > 0;
    if (useWall && (grid._sdfDirty || !grid.wallDist)) grid.updateWallSDF();

    const pTable = grid.getProfileTable(profile);
    const hasTac = grid.hasTactical && grid.tacticalCosts;
    const hasCong = grid.hasCongestion && grid.congestion;
    const hasFlux = grid.hasCongestionFlux && grid.congestionFluxX;
    const hasAniso = grid.hasAnisotropy && grid.dirMask;
    const wallR = grid.wallPenaltyRadius, wallW = grid.wallPenaltyWeight;

    for (let ly = 0; ly < CS; ly++) {
      const gy = oy + ly, row = gy * W + ox, lRow = ly * CS;
      for (let lx = 0; lx < CS; lx++) {
        const gIdx = row + lx, lIdx = lRow + lx;
        const baseC = grid.getEffectiveBaseCost(gIdx, pTable);
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
          curX[lIdx] = curY[lIdx] = 0;
        }
        if (hasFlux) {
          fluxX[lIdx] = grid.congestionFluxX[gIdx];
          fluxY[lIdx] = grid.congestionFluxY[gIdx];
        } else {
          fluxX[lIdx] = fluxY[lIdx] = 0;
        }
      }
    }
  }

  integrate(grid, chunkId, sources, out, opts = {}) {
    const minClearance = opts.minClearance ?? 1;
    const useEikonal = opts.eikonal ?? grid.useEikonal;
    const forward = !!opts.forward;
    const sourceOut = opts.sourceOut || null;
    const sourceWeights = opts.sourceWeights || null;

    this.prepareChunk(grid, chunkId, minClearance, opts.profile);
    const { costs, blocked, dirMasks, curX, curY, fluxX, fluxY, dist, heap } = this;
    const hasAniso = grid.hasAnisotropy;
    const hasFlux = grid.hasCongestionFlux;
    const curStr = grid.currentStrength;
    const laneW = grid.congestionLaneWeight;

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

        let stepScale = 0.25 * (cu + costs[v]);
        const travelDir = forward ? d : REV_D[d];
        const fromCell = forward ? u : v;
        let isMasked = false;

        if (hasAniso) {
          const mask = dirMasks[fromCell];
          if ((mask & (1 << travelDir)) === 0) continue;
          isMasked = mask !== 0xFF;
          const cx = curX[fromCell], cy = curY[fromCell];
          if (cx !== 0 || cy !== 0) {
            const dot = (DX[travelDir] * cx + DY[travelDir] * cy) / DL[d];
            stepScale *= clamp(1 - dot * curStr, 0.2, 4.0);
          }
        }

        if (hasFlux) {
          const fx = fluxX[fromCell], fy = fluxY[fromCell];
          if (fx !== 0 || fy !== 0) {
            const fl = Math.hypot(fx, fy);
            const align = (DX[travelDir] * fx + DY[travelDir] * fy) / (DL[d] * fl);
            stepScale *= clamp(1 - align * laneW, 0.45, 2.25);
          }
        }

        let nd = du + DL[d] * stepScale;

        if (useEikonal && !isMasked) {
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

//#region 14: Abstract Portal Graph, Tarjan SCC & Traffic Throttling (#B1, #D1, #G4, #H2)
/**
 * High-level abstract graph connecting chunk boundary portals in CSR format.
 * Supports:
 * - #A2: Multi-cell portal span evaluation
 * - #B1 & #I1: Flat Float32 CSR & in-place weight refreshes (`refreshWeights`)
 * - #D1 & #H2: Undirected components (`portalComp`) + Tarjan's Directed SCC & 64-Bit DAG Reachability Bitsets
 * - #G4: Dynamic Portal Flux Throttling (`portalTraffic`) for Wardrop traffic load-balancing
 */
export class PortalGraph {
  constructor(grid, prevGraph = null, dirtyChunks = null, minClearance = 1, profile = null) {
    this.grid = grid;
    this.minClearance = minClearance;
    this.profile = profile;
    this.weightVersion = prevGraph ? prevGraph.weightVersion + 1 : 1;

    const portals = this.portals = detectPortals(grid, minClearance, MAX_PORTAL_LEN, profile), P = portals.length;
    const lists = Array.from({ length: grid.chunkCount }, () => []);
    for (let i = 0; i < P; i++) {
      const p = portals[i];
      lists[p.chunkA].push(p.id); lists[p.chunkB].push(p.id);
    }
    const chunkPortals = this.chunkPortals = lists.map(l => new Int32Array(l));
    const canReuse = prevGraph && prevGraph.minClearance === minClearance && prevGraph.profile === profile && dirtyChunks;

    // #I1: Float32 cached intra-chunk rows
    this.chunkDistCache = Array.from({ length: grid.chunkCount }, (_, c) =>
      canReuse && !dirtyChunks.has(c) ? prevGraph.chunkDistCache[c] : new Map()
    );

    // #G4: Real-time portal traffic flux counters for Wardrop load-balancing
    this.portalTraffic = new Float32Array(P);
    this.trafficWeight = 1.25;

    const adj = Array.from({ length: P }, () => []);
    const it = this._it = new ChunkIntegrator();
    const field = this._field = new Float32Array(CC);
    const itOpts = { minClearance, profile };

    for (let c = 0; c < grid.chunkCount; c++) {
      const ids = chunkPortals[c];
      if (ids.length < 2) continue;
      const cCache = this.chunkDistCache[c];
      for (let ia = 0; ia < ids.length; ia++) {
        const idA = ids[ia], pA = portals[idA];
        const crossA = crossingCost(grid, pA, c, profile);
        if (crossA === Infinity) continue;
        const locA = pLocal(grid, pA, c);
        let row = cCache.get(locA);
        if (!row) {
          const seeds = portalSideCells(pA, c).map(cell => grid.localIndex(cell));
          it.integrate(grid, c, seeds, field, itOpts);
          row = new Float32Array(field);
          cCache.set(locA, row);
        }
        for (let ib = 0; ib < ids.length; ib++) {
          const idB = ids[ib];
          if (idA === idB) continue;
          const pB = portals[idB];
          const d = this._samplePortalRowDist(row, pB, c);
          if (d < Infinity) {
            adj[idB].push({ to: idA, cost: d + crossA, chunk: c });
          }
        }
      }
    }

    const E = adj.reduce((acc, a) => acc + a.length, 0);
    this.edgeOffsets = new Int32Array(P + 1);
    this.edgeTargets = new Int32Array(E);
    this.baseEdgeCosts = new Float32Array(E);
    this.edgeCosts = new Float32Array(E);
    this.edgeChunks = new Int32Array(E);
    for (let p = 0, e = 0; p < P; this.edgeOffsets[++p] = e) {
      for (let k = 0; k < adj[p].length; k++) {
        const edge = adj[p][k];
        this.edgeTargets[e] = edge.to;
        this.baseEdgeCosts[e] = this.edgeCosts[e] = edge.cost;
        this.edgeChunks[e++] = edge.chunk;
      }
    }

    this._buildReverseAndComponents();
  }

  _samplePortalRowDist(row, pB, chunk) {
    const cells = portalSideCells(pB, chunk), { grid } = this;
    const centerD = row[pLocal(grid, pB, chunk)];
    let minD = centerD;
    for (let i = 0; i < cells.length; i++) {
      const d = row[grid.localIndex(cells[i])];
      if (d < minD) minD = d;
    }
    return centerD < Infinity ? (0.65 * minD + 0.35 * centerD) : minD;
  }

  _buildReverseAndComponents() {
    const P = this.portals.length, E = this.edgeTargets.length;
    const revCounts = new Int32Array(P);
    for (let e = 0; e < E; e++) revCounts[this.edgeTargets[e]]++;
    this.revOffsets = new Int32Array(P + 1);
    for (let p = 0; p < P; p++) this.revOffsets[p + 1] = this.revOffsets[p] + revCounts[p];
    this.revSources = new Int32Array(E);
    this.revEdgeIdx = new Int32Array(E);
    const cursor = new Int32Array(this.revOffsets.subarray(0, P));
    for (let u = 0; u < P; u++) {
      for (let e = this.edgeOffsets[u]; e < this.edgeOffsets[u + 1]; e++) {
        const v = this.edgeTargets[e], pos = cursor[v]++;
        this.revSources[pos] = u;
        this.revEdgeIdx[pos] = e;
      }
    }

    // #D1: Undirected Connected Component Labeling
    const comp = this.portalComp = new Int32Array(P).fill(-1);
    const queue = new Int32Array(P);
    let compId = 0;
    for (let p = 0; p < P; p++) {
      if (comp[p] !== -1) continue;
      let head = 0, tail = 0;
      comp[p] = compId;
      queue[tail++] = p;
      while (head < tail) {
        const u = queue[head++];
        for (let e = this.edgeOffsets[u]; e < this.edgeOffsets[u + 1]; e++) {
          const v = this.edgeTargets[e];
          if (comp[v] === -1) { comp[v] = compId; queue[tail++] = v; }
        }
        for (let r = this.revOffsets[u]; r < this.revOffsets[u + 1]; r++) {
          const v = this.revSources[r];
          if (comp[v] === -1) { comp[v] = compId; queue[tail++] = v; }
        }
      }
      compId++;
    }
    this.componentCount = compId;

    // #H2: Tarjan's Strongly Connected Components (SCC) & 64-Bit DAG Reachability Signatures
    const scc = this.portalScc = new Int32Array(P).fill(-1);
    const disc = new Int32Array(P).fill(-1), low = new Int32Array(P);
    const onStack = new Uint8Array(P), stack = new Int32Array(P);
    const callNode = new Int32Array(P), callEdge = new Int32Array(P);
    let timer = 0, sTop = 0, sccCnt = 0;

    for (let i = 0; i < P; i++) {
      if (disc[i] !== -1) continue;
      let cTop = 0;
      callNode[0] = i;
      callEdge[0] = this.edgeOffsets[i];
      disc[i] = low[i] = timer++;
      stack[sTop++] = i;
      onStack[i] = 1;

      while (cTop >= 0) {
        const u = callNode[cTop], e = callEdge[cTop];
        if (e < this.edgeOffsets[u + 1]) {
          callEdge[cTop] = e + 1;
          const v = this.edgeTargets[e];
          if (disc[v] === -1) {
            cTop++;
            callNode[cTop] = v;
            callEdge[cTop] = this.edgeOffsets[v];
            disc[v] = low[v] = timer++;
            stack[sTop++] = v;
            onStack[v] = 1;
          } else if (onStack[v] && disc[v] < low[u]) {
            low[u] = disc[v];
          }
        } else {
          if (low[u] === disc[u]) {
            while (sTop > 0) {
              const w = stack[--sTop];
              onStack[w] = 0;
              scc[w] = sccCnt;
              if (w === u) break;
            }
            sccCnt++;
          }
          cTop--;
          if (cTop >= 0) {
            const pNode = callNode[cTop];
            if (low[u] < low[pNode]) low[pNode] = low[u];
          }
        }
      }
    }

    this.sccCount = sccCnt;
    const rLo = this.sccReachLo = new Uint32Array(sccCnt);
    const rHi = this.sccReachHi = new Uint32Array(sccCnt);
    for (let s = 0; s < sccCnt; s++) {
      const bit = s & 63;
      if (bit < 32) rLo[s] = 1 << bit;
      else rHi[s] = 1 << (bit - 32);
    }
    // Tarjan emits SCCs in reverse topological order (0 = sink, sccCnt-1 = source)
    for (let u = 0; u < P; u++) {
      const su = scc[u];
      for (let e = this.edgeOffsets[u]; e < this.edgeOffsets[u + 1]; e++) {
        const sv = scc[this.edgeTargets[e]];
        if (su !== sv) {
          rLo[su] |= rLo[sv];
          rHi[su] |= rHi[sv];
        }
      }
    }
    // Second sweep to propagate multi-hop DAG reachability
    for (let s = 0; s < sccCnt; s++) {
      for (let u = 0; u < P; u++) {
        if (scc[u] !== s) continue;
        for (let e = this.edgeOffsets[u]; e < this.edgeOffsets[u + 1]; e++) {
          const sv = scc[this.edgeTargets[e]];
          rLo[s] |= rLo[sv];
          rHi[s] |= rHi[sv];
        }
      }
    }
  }

  /** #G4: Records agent traversal across a portal to throttle overcrowded bottlenecks */
  addPortalTraffic(portalId, amount = 1.0) {
    if (portalId < 0 || portalId >= this.portals.length) return;
    const p = this.portals[portalId], cap = Math.max(1, p.cellsA.length);
    this.portalTraffic[portalId] += amount / cap;
    const surcharge = this.portalTraffic[portalId] * this.trafficWeight;
    for (let r = this.revOffsets[portalId]; r < this.revOffsets[portalId + 1]; r++) {
      const e = this.revEdgeIdx[r];
      this.edgeCosts[e] = this.baseEdgeCosts[e] + surcharge;
    }
  }

  /** #G4: Decays real-time portal traffic congestion surcharges */
  decayPortalTraffic(decayFactor = 0.85, eps = 0.02) {
    const T = this.portalTraffic, P = T.length;
    let changed = false;
    for (let p = 0; p < P; p++) {
      if (T[p] > 0) {
        T[p] *= decayFactor;
        if (T[p] < eps) T[p] = 0;
        const surcharge = T[p] * this.trafficWeight;
        for (let r = this.revOffsets[p]; r < this.revOffsets[p + 1]; r++) {
          const e = this.revEdgeIdx[r];
          this.edgeCosts[e] = this.baseEdgeCosts[e] + surcharge;
        }
        changed = true;
      }
    }
    if (changed) this.weightVersion++;
    return changed;
  }

  refreshWeights(dirtyChunks) {
    const { grid, portals, chunkPortals, minClearance, profile, _it: it, _field: field } = this;
    const itOpts = { minClearance, profile };
    for (const c of dirtyChunks) {
      const ids = chunkPortals[c];
      if (!ids || ids.length < 2) continue;
      const cCache = this.chunkDistCache[c];
      cCache.clear();
      for (let ia = 0; ia < ids.length; ia++) {
        const idA = ids[ia], pA = portals[idA], locA = pLocal(grid, pA, c);
        const seeds = portalSideCells(pA, c).map(cell => grid.localIndex(cell));
        it.integrate(grid, c, seeds, field, itOpts);
        cCache.set(locA, new Float32Array(field));
      }
      for (let ib = 0; ib < ids.length; ib++) {
        const idB = ids[ib], pB = portals[idB];
        for (let e = this.edgeOffsets[idB]; e < this.edgeOffsets[idB + 1]; e++) {
          if (this.edgeChunks[e] !== c) continue;
          const idA = this.edgeTargets[e], pA = portals[idA];
          const row = cCache.get(pLocal(grid, pA, c));
          const crossA = crossingCost(grid, pA, c, profile);
          const d = row ? this._samplePortalRowDist(row, pB, c) : Infinity;
          this.baseEdgeCosts[e] = d + crossA;
          this.edgeCosts[e] = this.baseEdgeCosts[e] + this.portalTraffic[idA] * this.trafficWeight;
        }
      }
    }
    this.weightVersion++;
  }
}
//#endregion

//#region 15: Hierarchical Macro Pathfinder, SSFA Funnel & Swarm Broadcast (#B3, #F5)
/**
 * Hierarchical A*, Funnel String-Puller (#F5) & Swarm Broadcast Pathfinder (#B3) operating over `PortalGraph`.
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
    this.startField = new Float32Array(CC); this.heap = new MinHeap(256); this.stamp = 0;
    this._swarmIdCounter = 1;
  }

  invalidateChunks(dirtyChunks) {
    for (const [cell, goal] of this.goalCache) {
      if (dirtyChunks.has(goal.chunk)) this.goalCache.delete(cell);
    }
  }

  createGoal(cell) {
    const { grid, graph } = this, x = cell % grid.width, y = cell / grid.width | 0;
    const chunk = grid.chunkIdAt(x, y), field = new Float32Array(CC);
    this.it.integrate(grid, chunk, [grid.localIndex(cell)], field, {
      minClearance: graph.minClearance,
      profile: graph.profile
    });
    const portalCost = new Float32Array(graph.portals.length).fill(Infinity);
    const cPortals = graph.chunkPortals[chunk];
    const compSet = new Set();
    let sccMaskLo = 0, sccMaskHi = 0;
    for (let i = 0; i < cPortals.length; i++) {
      const pid = cPortals[i], p = graph.portals[pid];
      const d = graph._samplePortalRowDist(field, p, chunk);
      if (d < Infinity) {
        portalCost[pid] = d;
        compSet.add(graph.portalComp[pid]);
        const bit = graph.portalScc[pid] & 63;
        if (bit < 32) sccMaskLo |= (1 << bit);
        else sccMaskHi |= (1 << (bit - 32));
      }
    }
    return { isMulti: false, cell, x, y, chunk, field, portalCost, compSet, sccMaskLo, sccMaskHi, graph };
  }

  createMultiGoal(targets) {
    const { grid, graph } = this;
    const portalCost = new Float32Array(graph.portals.length).fill(Infinity);
    const portalGoalCell = new Int32Array(graph.portals.length).fill(-1);
    const chunkGroups = new Map();
    const goalList = [];
    const compSet = new Set();
    let sccMaskLo = 0, sccMaskHi = 0;

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
      const field = new Float32Array(CC);
      const sourceOut = new Int32Array(CC);
      const seeded = sources.map(s => ({ cell: s.cell, weight: s.weight, id: s.globalCell }));
      this.it.integrate(grid, chunk, seeded, field, {
        minClearance: graph.minClearance,
        profile: graph.profile,
        sourceOut
      });
      chunkFields.set(chunk, { field, sourceOut, sources });
      const cPortals = graph.chunkPortals[chunk];
      for (let i = 0; i < cPortals.length; i++) {
        const pid = cPortals[i], p = graph.portals[pid], loc = pLocal(grid, p, chunk);
        const d = graph._samplePortalRowDist(field, p, chunk);
        if (d < portalCost[pid]) {
          portalCost[pid] = d;
          portalGoalCell[pid] = sourceOut[loc] >= 0 ? sourceOut[loc] : sources[0].globalCell;
          compSet.add(graph.portalComp[pid]);
          const bit = graph.portalScc[pid] & 63;
          if (bit < 32) sccMaskLo |= (1 << bit);
          else sccMaskHi |= (1 << (bit - 32));
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
      compSet,
      sccMaskLo,
      sccMaskHi,
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

  /**
   * #F5: Runs the 2D Simple Stupid Funnel Algorithm (SSFA) across `portals` to compute
   * per-chunk string-pulled visibility targets (`funnelX`, `funnelY`) for cross-chunk corner cutting.
   */
  _computePortalFunnel(portals, chunks, sx, sy, gx, gy) {
    const P = portals.length, C = chunks.length;
    const funnelX = new Float32Array(C), funnelY = new Float32Array(C);
    funnelX[C - 1] = gx; funnelY[C - 1] = gy;
    if (P === 0) return { funnelX, funnelY };

    const gPortals = this.graph.portals;
    const lx = new Float32Array(P + 1), ly = new Float32Array(P + 1);
    const rx = new Float32Array(P + 1), ry = new Float32Array(P + 1);

    for (let i = 0; i < P; i++) {
      const p = gPortals[portals[i]], fromChunk = chunks[i];
      const x0 = p.isH ? p.start : p.border, y0 = p.isH ? p.border : p.start;
      const x1 = p.isH ? p.end + 1 : p.border, y1 = p.isH ? p.border : p.end + 1;
      const pos = fromChunk === p.chunkA;
      if (p.isH ? pos : !pos) {
        lx[i] = x1; ly[i] = y1; rx[i] = x0; ry[i] = y0;
      } else {
        lx[i] = x0; ly[i] = y0; rx[i] = x1; ry[i] = y1;
      }
    }
    lx[P] = rx[P] = gx; ly[P] = ry[P] = gy;

    const triArea2 = (ax, ay, bx, by, cx, cy) => (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    let apexX = sx, apexY = sy, lIdx = 0, rIdx = 0;
    let pLx = lx[0], pLy = ly[0], pRx = rx[0], pRy = ry[0];
    let stepWrite = 0;

    for (let i = 1; i <= P; i++) {
      const nLx = lx[i], nLy = ly[i], nRx = rx[i], nRy = ry[i];
      if (triArea2(apexX, apexY, pRx, pRy, nRx, nRy) <= 0) {
        if ((apexX === pRx && apexY === pRy) || triArea2(apexX, apexY, pLx, pLy, nRx, nRy) > 0) {
          pRx = nRx; pRy = nRy; rIdx = i;
        } else {
          while (stepWrite <= lIdx && stepWrite < P) {
            funnelX[stepWrite] = pLx; funnelY[stepWrite] = pLy; stepWrite++;
          }
          apexX = pLx; apexY = pLy;
          pRx = apexX; pRy = apexY; rIdx = lIdx;
          i = lIdx;
          continue;
        }
      }
      if (triArea2(apexX, apexY, pLx, pLy, nLx, nLy) >= 0) {
        if ((apexX === pLx && apexY === pLy) || triArea2(apexX, apexY, pRx, pRy, nLx, nLy) < 0) {
          pLx = nLx; pLy = nLy; lIdx = i;
        } else {
          while (stepWrite <= rIdx && stepWrite < P) {
            funnelX[stepWrite] = pRx; funnelY[stepWrite] = pRy; stepWrite++;
          }
          apexX = pRx; apexY = pRy;
          pLx = apexX; pLy = apexY; lIdx = rIdx;
          i = rIdx;
          continue;
        }
      }
    }
    while (stepWrite < P) {
      const p = gPortals[portals[stepWrite]];
      funnelX[stepWrite] = clamp(gx, p.isH ? p.start + 0.5 : p.border, p.isH ? p.end + 0.5 : p.border);
      funnelY[stepWrite] = clamp(gy, p.isH ? p.border : p.start + 0.5, p.isH ? p.border : p.end + 0.5);
      stepWrite++;
    }
    return { funnelX, funnelY };
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
      profile: graph.profile,
      forward: true
    });

    let sameChunkDirect = false;
    if (!goal.isMulti && sc === goal.chunk && goal.field[grid.localIndex(startCell)] < Infinity) {
      sameChunkDirect = true;
    } else if (goal.isMulti && goal.chunkFields.has(sc) && goal.chunkFields.get(sc).field[grid.localIndex(startCell)] < Infinity) {
      sameChunkDirect = true;
    }

    const cPorts = graph.chunkPortals[sc];
    if (!sameChunkDirect && goal.compSet) {
      let reachableMatch = false;
      const gLo = goal.sccMaskLo, gHi = goal.sccMaskHi;
      for (let i = 0; i < cPorts.length; i++) {
        const pid = cPorts[i], p = graph.portals[pid];
        if (this.startField[pLocal(grid, p, sc)] < Infinity && crossingCost(grid, p, sc, graph.profile) < Infinity) {
          if (!goal.compSet.has(graph.portalComp[pid])) continue;
          const uScc = graph.portalScc[pid];
          // #H2: O(1) Directed SCC DAG reachability bitmask check
          if (((graph.sccReachLo[uScc] & gLo) !== 0) || ((graph.sccReachHi[uScc] & gHi) !== 0)) {
            reachableMatch = true;
            break;
          }
        }
      }
      if (!reachableMatch) return null;
    }

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
        const pArr = new Int32Array(portals.reverse());
        const cArr = new Int32Array(chunks.reverse());
        const fgx = (chosenGoalCell % grid.width) + 0.5, fgy = ((chosenGoalCell / grid.width) | 0) + 0.5;
        const { funnelX, funnelY } = this._computePortalFunnel(pArr, cArr, sx + 0.5, sy + 0.5, fgx, fgy);
        return {
          portals: pArr,
          chunks: cArr,
          funnelX,
          funnelY,
          cost: g[GOAL],
          goalCell: chosenGoalCell,
          goalX: fgx,
          goalY: fgy,
          minClearance: graph.minClearance,
          profile: graph.profile,
          graph
        };
      }
      const gu = g[u];
      if (u === START) {
        for (let i = 0; i < cPorts.length; i++) {
          const pid = cPorts[i], p = graph.portals[pid];
          const d = graph._samplePortalRowDist(this.startField, p, sc);
          const cross = crossingCost(grid, p, sc, graph.profile);
          if (d < Infinity && cross < Infinity) relax(pid, gu + d + cross, START, sc);
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

  /**
   * #B3: Builds a Global Reverse-Dijkstra Swarm Broadcast Field over `PortalGraph` in O(E log P).
   */
  createSwarmField(targets) {
    const goal = Array.isArray(targets) ? this.createMultiGoal(targets) : this.getOrCreateGoal(targets);
    const { graph, heap } = this;
    const P = graph.portals.length;
    const portalDist = new Float32Array(P).fill(Infinity);
    const portalNext = new Int32Array(P).fill(-1);
    heap.clear();

    for (let p = 0; p < P; p++) {
      const gc = goal.portalCost[p];
      if (gc < Infinity) {
        portalDist[p] = gc;
        portalNext[p] = -2;
        heap.push(p, gc);
      }
    }

    while (!heap.isEmpty) {
      const v = heap.pop(), dv = heap.lastKey;
      if (dv > portalDist[v]) continue;
      for (let r = graph.revOffsets[v]; r < graph.revOffsets[v + 1]; r++) {
        const u = graph.revSources[r], e = graph.revEdgeIdx[r];
        const nd = dv + graph.edgeCosts[e];
        if (nd < portalDist[u]) {
          portalDist[u] = nd;
          portalNext[u] = v;
          heap.push(u, nd);
        }
      }
    }

    const goalChunks = new Map();
    if (goal.isMulti) {
      for (const [c, cf] of goal.chunkFields) goalChunks.set(c, cf.sources);
    } else {
      goalChunks.set(goal.chunk, [{ cell: this.grid.localIndex(goal.cell), weight: 0, globalCell: goal.cell }]);
    }

    return {
      isSwarm: true,
      uid: this._swarmIdCounter++,
      goal,
      goalX: goal.x + 0.5,
      goalY: goal.y + 0.5,
      portalDist,
      portalNext,
      goalChunks,
      minClearance: graph.minClearance,
      profile: graph.profile,
      weightVersion: graph.weightVersion,
      graph
    };
  }
}
//#endregion

//#region 16: Chunk Line-of-Sight & Flow Vector Extraction (#3, #4, #7, #A1, #F2)
/**
 * #3: Fast chunk-local supercover Line-of-Sight check.
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

export function computeFlowVectors(f, exX, exY, opts = {}) {
  const { integration: I, flowX: FX, flowY: FY } = f;
  const at = (x, y) => x >>> 0 >= CS || y >>> 0 >= CS ? Infinity : I[y * CS + x];
  const grid = opts.grid || null;
  const useWallRep = grid && grid.wallRepulsionWeight > 0 && grid.wallDist;
  const vortex = opts.vortex || f.vortex || null;
  const hasLOS = opts.losSmoothing && opts.chunkCosts && opts.targetLocalX !== undefined;
  const txL = opts.targetLocalX, tyL = opts.targetLocalY, cCosts = opts.chunkCosts;
  const refCost = hasLOS ? cCosts[clamp(tyL | 0, 0, CS - 1) * CS + clamp(txL | 0, 0, CS - 1)] : COST_OPEN;
  const borderExitMask = opts.borderExitMask || null;
  const borderExX = opts.borderExX || null;
  const borderExY = opts.borderExY || null;

  for (let y = 0; y < CS; y++) {
    for (let x = 0; x < CS; x++) {
      const id = y * CS + x, cur = I[id];
      if (cur === Infinity) { FX[id] = FY[id] = 0; continue; }

      // #A1 & #F2: Per-cell exit normal lookup for portal and multi-portal swarm borders
      const cellExX = borderExX && borderExitMask && borderExitMask[id] ? borderExX[id] : exX;
      const cellExY = borderExY && borderExitMask && borderExitMask[id] ? borderExY[id] : exY;

      if ((!cur || (borderExitMask && borderExitMask[id])) && !vortex && (cellExX || cellExY)) {
        const E = at(x + 1, y), W = at(x - 1, y), S = at(x, y + 1), N = at(x, y - 1);
        const gx = cellExX !== 0 ? 0 : (E < W ? (E < cur ? E - cur : 0) : W < cur ? cur - W : 0);
        const gy = cellExY !== 0 ? 0 : (S < N ? (S < cur ? S - cur : 0) : N < cur ? cur - N : 0);
        const vx = cellExX - gx * 0.45, vy = cellExY - gy * 0.45, vl = Math.hypot(vx, vy) || 1;
        FX[id] = vx / vl; FY[id] = vy / vl;
        continue;
      }
      if (!cur && !vortex) { FX[id] = cellExX; FY[id] = cellExY; continue; }

      let dx = 0, dy = 0, ok = false;

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
          dx = bx / l2 || cellExX; dy = by / l2 || cellExY;
        }
      }

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
//#endregion

//#region 17: Geodesic Guide-Curve & Spline Corridor Builder (#10, #D2, #F4)
/**
 * #10, #D2 & #F4: Multi-Chunk Geodesic Guide-Curve / Spline Corridor Flow Field builder.
 * Propagates corridor distances through `PortalGraph` when the spline lies in neighboring chunks
 * so agents route around walls to join the corridor rather than pushing into solid walls.
 */
export function buildCorridorField(grid, chunkId, polyline, opts = {}) {
  const ox = grid.chunkOriginX(chunkId), oy = grid.chunkOriginY(chunkId);
  const pool = opts.pool || null;
  const f = {
    chunkId, kind: 'c', originX: ox, originY: oy,
    integration: pool ? pool.allocF32() : new Float32Array(CC),
    flowX: pool ? pool.allocF32() : new Float32Array(CC),
    flowY: pool ? pool.allocF32() : new Float32Array(CC),
    sourceId: pool ? pool.allocI32() : new Int32Array(CC),
    _pooled: false
  };
  const spineTanX = new Float32Array(CC), spineTanY = new Float32Array(CC);
  const seedDist = new Float32Array(CC).fill(Infinity), sources = [];
  const minClearance = opts.minClearance ?? 1;
  const graph = opts.graph || null;
  let hasInChunk = false;

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
      const wx = x0 + dx * t, wy = y0 + dy * t;
      const lx = Math.floor(wx - ox), ly = Math.floor(wy - oy);
      if (lx >>> 0 < CS && ly >>> 0 < CS) {
        const cell = ly * CS + lx;
        spineTanX[cell] = tx; spineTanY[cell] = ty;
        if (seedDist[cell] > 0) seedDist[cell] = 0;
        hasInChunk = true;
      }
    }
  }

  // #F4: If spline is in another chunk, seed only open portals of `chunkId` via PortalGraph
  if (!hasInChunk) {
    const cPortals = graph ? graph.chunkPortals[chunkId] : null;
    for (let i = 0; i < polyline.length - 1; i++) {
      const p0 = polyline[i], p1 = polyline[i + 1];
      const x0 = Array.isArray(p0) ? p0[0] : p0.x, y0 = Array.isArray(p0) ? p0[1] : p0.y;
      const x1 = Array.isArray(p1) ? p1[0] : p1.x, y1 = Array.isArray(p1) ? p1[1] : p1.y;
      const dx = x1 - x0, dy = y1 - y0, segLen = Math.hypot(dx, dy);
      if (segLen < 1e-5) continue;
      const tx = dx / segLen, ty = dy / segLen, mx = 0.5 * (x0 + x1), my = 0.5 * (y0 + y1);
      if (cPortals && cPortals.length > 0) {
        for (let k = 0; k < cPortals.length; k++) {
          const p = graph.portals[cPortals[k]], cells = portalSideCells(p, chunkId);
          const pd = octile(mx - p.anchorX, my - p.anchorY) * COST_OPEN;
          for (let j = 0; j < cells.length; j++) {
            const loc = grid.localIndex(cells[j]);
            if (pd < seedDist[loc]) {
              seedDist[loc] = pd;
              spineTanX[loc] = tx; spineTanY[loc] = ty;
            }
          }
        }
      } else {
        const clx = clamp(Math.floor(mx - ox), 0, CS - 1), cly = clamp(Math.floor(my - oy), 0, CS - 1);
        const loc = cly * CS + clx, pd = octile(mx - (ox + clx + 0.5), my - (oy + cly + 0.5)) * COST_OPEN;
        if (pd < seedDist[loc]) { seedDist[loc] = pd; spineTanX[loc] = tx; spineTanY[loc] = ty; }
      }
    }
  }

  for (let c = 0; c < CC; c++) {
    if (seedDist[c] < Infinity) sources.push({ cell: c, weight: seedDist[c], id: c });
  }

  const it = opts.integrator || new ChunkIntegrator();
  it.integrate(grid, chunkId, sources, f.integration, {
    sourceOut: f.sourceId,
    minClearance,
    profile: opts.profile
  });
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
//#endregion

//#region 18: LRU Flow Field Cache & Cross-Boundary Seeding (#A1, #B4, #F1, #F2, #F4, #F5)
/**
 * #A1, #B4, #F1, #F2, #F4, #F5: Collision-proof LRU cache of per-chunk flow fields backed by `FieldBufferPool`.
 */
export class FlowFieldCache {
  constructor(grid, cap = 2048) {
    this.grid = grid;
    this.cap = cap;
    this.entries = new Map();
    this.it = new ChunkIntegrator();
    this.pool = new FieldBufferPool(Math.min(cap, 1024));
    this._borderMask = new Uint8Array(CC);
    this._borderExX = new Float32Array(CC);
    this._borderExY = new Float32Array(CC);
  }

  /** #F1: Composite hash including profileId */
  key(c, kind, tid, minClearance = 1, extra = 0, profile = null) {
    const profId = this.grid.getProfileId(profile);
    let h = hashMix(2166136261, c);
    h = hashMix(h, kind.charCodeAt(0));
    h = hashMix(h, tid);
    h = hashMix(h, minClearance);
    h = hashMix(h, extra);
    return hashMix(h, profId);
  }

  /** #F1: Exact tag verification eliminating 32-bit hash collision aliasing */
  _match(hit, c, kind, tid, minClearance, extra, profile) {
    return !!(hit && hit.chunkId === c && hit.kind === kind && hit.tid === tid &&
      hit.minClearance === minClearance && hit.extra === extra && hit.profile === profile);
  }

  _tag(f, c, kind, tid, minClearance, extra, profile) {
    f.chunkId = c; f.kind = kind; f.tid = tid;
    f.minClearance = minClearance; f.extra = extra; f.profile = profile;
    return f;
  }

  _promote(k, hit) {
    if (this.entries.size >= this.cap * 0.75) {
      this.entries.delete(k);
      this.entries.set(k, hit);
    }
    return hit;
  }

  _store(k, f) {
    const existing = this.entries.get(k);
    if (existing && existing !== f) this.pool.releaseField(existing);
    if (this.entries.size >= this.cap) {
      const oldestKey = this.entries.keys().next().value;
      this.pool.releaseField(this.entries.get(oldestKey));
      this.entries.delete(oldestKey);
    }
    this.entries.set(k, f);
    return f;
  }

  _allocField(cid, kind, trackSources = false) {
    const { grid, pool } = this;
    return {
      chunkId: cid, kind,
      originX: grid.chunkOriginX(cid),
      originY: grid.chunkOriginY(cid),
      integration: pool.allocF32(),
      flowX: pool.allocF32(),
      flowY: pool.allocF32(),
      sourceId: trackSources ? pool.allocI32() : null,
      _pooled: false
    };
  }

  /**
   * #A1 & #F5: Portal field with Cross-Boundary Wavefront Seeding and optional Funnel Target LOS.
   */
  getPortalField(c, p, minClearance = 1, nextField = null, nextTid = 0, profile = null, funnelX = undefined, funnelY = undefined) {
    const k = this.key(c, 'p', p.uid, minClearance, nextTid, profile);
    const hit = this.entries.get(k);
    if (this._match(hit, c, 'p', p.uid, minClearance, nextTid, profile)) return this._promote(k, hit);

    const sideA = portalSideCells(p, c);
    const nextChunk = c === p.chunkA ? p.chunkB : p.chunkA;
    const sideB = portalSideCells(p, nextChunk);
    const { grid, _borderMask } = this;
    _borderMask.fill(0);

    let minNext = Infinity;
    if (nextField) {
      for (let i = 0; i < sideB.length; i++) {
        const d = nextField.integration[grid.localIndex(sideB[i])];
        if (d < minNext) minNext = d;
      }
    }

    const sources = new Array(sideA.length);
    for (let i = 0; i < sideA.length; i++) {
      const loc = grid.localIndex(sideA[i]);
      _borderMask[loc] = 1;
      let w = 0;
      if (nextField && minNext < Infinity) {
        const nd = nextField.integration[grid.localIndex(sideB[i])];
        w = nd < Infinity ? (nd - minNext) : (CS * COST_OPEN);
      }
      sources[i] = { cell: loc, weight: w, id: i };
    }

    const ox = grid.chunkOriginX(c), oy = grid.chunkOriginY(c);
    const hasFunnelTarget = funnelX !== undefined && funnelY !== undefined;
    const f = this.build(k, c, 'p', sources, portalExitX(p, c), portalExitY(p, c), {
      minClearance,
      profile,
      borderExitMask: nextField && minNext < Infinity ? _borderMask : null,
      losSmoothing: hasFunnelTarget && grid.losSmoothing,
      targetLocalX: hasFunnelTarget ? clamp(funnelX - ox, 0.5, CS - 0.5) : undefined,
      targetLocalY: hasFunnelTarget ? clamp(funnelY - oy, 0.5, CS - 0.5) : undefined
    });
    return this._tag(f, c, 'p', p.uid, minClearance, nextTid, profile);
  }

  getGoalField(c, cell, minClearance = 1, profile = null) {
    const k = this.key(c, 'g', cell, minClearance, 0, profile);
    const hit = this.entries.get(k);
    if (this._match(hit, c, 'g', cell, minClearance, 0, profile)) return this._promote(k, hit);
    const loc = this.grid.localIndex(cell);
    const f = this.build(k, c, 'g', [loc], 0, 0, {
      minClearance,
      profile,
      losSmoothing: this.grid.losSmoothing,
      targetLocalX: (loc % CS) + 0.5,
      targetLocalY: ((loc / CS) | 0) + 0.5
    });
    return this._tag(f, c, 'g', cell, minClearance, 0, profile);
  }

  /**
   * #B3 & #F2: Builds and caches the unified per-chunk field for a Global Swarm Broadcast Field.
   * Assigns per-portal exit normals (`_borderExX`, `_borderExY`) so bisected chunks never bleed exit vectors,
   * and seeds portal cells with downstream chunk integration values when cached (#A1).
   */
  getSwarmChunkField(c, swarm) {
    const k = this.key(c, 's', swarm.uid, swarm.minClearance, swarm.weightVersion, swarm.profile);
    const hit = this.entries.get(k);
    if (this._match(hit, c, 's', swarm.uid, swarm.minClearance, swarm.weightVersion, swarm.profile)) {
      return this._promote(k, hit);
    }

    const { grid, _borderMask, _borderExX, _borderExY } = this;
    const { graph, portalDist, goalChunks } = swarm;
    const cPortals = graph.chunkPortals[c];
    const inChunkGoals = goalChunks.get(c);
    _borderMask.fill(0);

    let minW = inChunkGoals ? 0 : Infinity;
    for (let i = 0; i < cPortals.length; i++) {
      const pd = portalDist[cPortals[i]];
      if (pd < minW) minW = pd;
    }
    if (minW === Infinity) return null;

    const sources = [];
    let domExX = 0, domExY = 0, bestPD = Infinity;

    if (inChunkGoals) {
      for (let i = 0; i < inChunkGoals.length; i++) {
        const g = inChunkGoals[i];
        sources.push({ cell: g.cell, weight: g.weight, id: g.globalCell });
      }
    }

    for (let i = 0; i < cPortals.length; i++) {
      const pid = cPortals[i], pd = portalDist[pid];
      if (pd === Infinity) continue;
      const p = graph.portals[pid], cells = portalSideCells(p, c);
      const nextChunk = c === p.chunkA ? p.chunkB : p.chunkA;
      const nextCells = portalSideCells(p, nextChunk);
      const exX = portalExitX(p, c), exY = portalExitY(p, c);
      const relW = pd - minW;
      if (pd < bestPD) { bestPD = pd; domExX = exX; domExY = exY; }

      // #F2: Check if downstream neighbor chunk is already cached for cross-boundary cell-by-cell lookahead
      const nk = this.key(nextChunk, 's', swarm.uid, swarm.minClearance, swarm.weightVersion, swarm.profile);
      const nField = this.entries.get(nk);
      let nMin = Infinity;
      if (nField && nField.chunkId === nextChunk) {
        for (let j = 0; j < nextCells.length; j++) {
          const nd = nField.integration[grid.localIndex(nextCells[j])];
          if (nd < nMin) nMin = nd;
        }
      }

      for (let j = 0; j < cells.length; j++) {
        const loc = grid.localIndex(cells[j]);
        _borderMask[loc] = 1;
        _borderExX[loc] = exX;
        _borderExY[loc] = exY;
        let cellOffset = 0;
        if (nField && nMin < Infinity) {
          const nd = nField.integration[grid.localIndex(nextCells[j])];
          if (nd < Infinity) cellOffset = (nd - nMin) * 0.5;
        }
        sources.push({ cell: loc, weight: relW + cellOffset, id: pid });
      }
    }

    const singleGoalLoc = inChunkGoals && inChunkGoals.length === 1 && bestPD > 0 ? inChunkGoals[0].cell : -1;
    const f = this.build(k, c, 's', sources, inChunkGoals ? 0 : domExX, inChunkGoals ? 0 : domExY, {
      minClearance: swarm.minClearance,
      profile: swarm.profile,
      borderExitMask: !inChunkGoals ? _borderMask : null,
      borderExX: !inChunkGoals ? _borderExX : null,
      borderExY: !inChunkGoals ? _borderExY : null,
      losSmoothing: singleGoalLoc >= 0 && grid.losSmoothing,
      targetLocalX: singleGoalLoc >= 0 ? (singleGoalLoc % CS) + 0.5 : undefined,
      targetLocalY: singleGoalLoc >= 0 ? ((singleGoalLoc / CS) | 0) + 0.5 : undefined
    });
    return this._tag(f, c, 's', swarm.uid, swarm.minClearance, swarm.weightVersion, swarm.profile);
  }

  getMultiGoalField(c, sources, minClearance = 1, profile = null) {
    let h = sources.length;
    for (let i = 0; i < sources.length; i++) {
      const s = sources[i];
      h = hashMix(h, typeof s === 'object' ? s.cell : s);
    }
    const k = this.key(c, 'm', h, minClearance, 0, profile);
    const hit = this.entries.get(k);
    if (this._match(hit, c, 'm', h, minClearance, 0, profile)) return this._promote(k, hit);

    const f = this._allocField(c, 'm', true);
    this.it.integrate(this.grid, c, sources, f.integration, { minClearance, profile, sourceOut: f.sourceId });
    computeFlowVectors(f, 0, 0, { grid: this.grid });
    this._tag(f, c, 'm', h, minClearance, 0, profile);
    return this._store(k, f);
  }

  /**
   * #7, #B4 & #F4: Cached Vortex / Orbit field using `PortalGraph` geodesic seeding for cross-chunk centers.
   */
  getVortexField(c, centerCell, vortexOpts = {}) {
    const minClearance = vortexOpts.minClearance ?? 1;
    const profile = vortexOpts.profile ?? null;
    const extra = (((vortexOpts.radius ?? 4) * 100) | 0) ^ (vortexOpts.clockwise === false ? 1 : 0);
    const k = this.key(c, 'v', centerCell, minClearance, extra, profile);
    const hit = this.entries.get(k);
    if (this._match(hit, c, 'v', centerCell, minClearance, extra, profile)) return this._promote(k, hit);

    const { grid } = this;
    const f = this._allocField(c, 'v', false);
    f.vortex = vortexOpts;
    const centerChunk = grid.chunkIdAt(centerCell % grid.width, (centerCell / grid.width) | 0);

    if (centerChunk === c) {
      const loc = grid.localIndex(centerCell);
      this.it.integrate(grid, c, [loc], f.integration, { minClearance, profile });
      computeFlowVectors(f, 0, 0, {
        grid,
        vortex: vortexOpts,
        losSmoothing: grid.losSmoothing,
        chunkCosts: this.it.costs,
        targetLocalX: (loc % CS) + 0.5,
        targetLocalY: ((loc / CS) | 0) + 0.5
      });
    } else {
      // #F4: Seed only open portals of chunk `c` via PortalGraph or macro distances
      const graph = vortexOpts.graph || null;
      const macro = vortexOpts.macro || null;
      const vcx = (centerCell % grid.width) + 0.5, vcy = ((centerCell / grid.width) | 0) + 0.5;
      const seeds = [];
      if (graph && graph.chunkPortals[c].length > 0) {
        const goal = macro ? macro.getOrCreateGoal(centerCell) : null;
        const cPortals = graph.chunkPortals[c];
        for (let i = 0; i < cPortals.length; i++) {
          const pid = cPortals[i], p = graph.portals[pid], cells = portalSideCells(p, c);
          const baseD = (goal && goal.portalCost[pid] < Infinity)
            ? goal.portalCost[pid]
            : octile(p.anchorX - vcx, p.anchorY - vcy) * COST_OPEN;
          for (let j = 0; j < cells.length; j++) {
            seeds.push({ cell: grid.localIndex(cells[j]), weight: baseD, id: pid });
          }
        }
      } else {
        const ox = f.originX, oy = f.originY;
        for (let t = 0; t < CS; t++) {
          const perim = [t, (CS - 1) * CS + t, t * CS, t * CS + (CS - 1)];
          for (let b = 0; b < 4; b++) {
            const loc = perim[b], lx = loc % CS, ly = (loc / CS) | 0;
            if (grid.isWalkable(ox + lx, oy + ly, minClearance, profile)) {
              const d = octile((ox + lx + 0.5) - vcx, (oy + ly + 0.5) - vcy) * COST_OPEN;
              seeds.push({ cell: loc, weight: d, id: loc });
            }
          }
        }
      }
      this.it.integrate(grid, c, seeds, f.integration, { minClearance, profile });
      computeFlowVectors(f, 0, 0, { grid, vortex: vortexOpts });
    }
    this._tag(f, c, 'v', centerCell, minClearance, extra, profile);
    return this._store(k, f);
  }

  /**
   * #8, #B4 & #F4: Cached 2-Pass Flee field routing out-of-chunk threats through open `PortalGraph` doorways
   * instead of bleeding straight through solid walls.
   */
  getFleeField(c, threatPointsOrCells, opts = {}) {
    const minClearance = opts.minClearance ?? 1;
    const profile = opts.profile ?? null;
    const graph = opts.graph || null;
    const { grid } = this, ox = grid.chunkOriginX(c), oy = grid.chunkOriginY(c);
    let h = threatPointsOrCells.length;
    const seeds = [];
    const cPortals = graph ? graph.chunkPortals[c] : null;

    for (let i = 0; i < threatPointsOrCells.length; i++) {
      const t = threatPointsOrCells[i];
      if (typeof t === 'number') {
        h = hashMix(h, t);
        seeds.push({ cell: t, weight: 0, id: i });
      } else {
        const tx = t.x, ty = t.y;
        h = hashMix(hashMix(h, tx * 10), ty * 10);
        const lx = (tx | 0) - ox, ly = (ty | 0) - oy;
        if (lx >>> 0 < CS && ly >>> 0 < CS) {
          seeds.push({ cell: ly * CS + lx, weight: 0, id: i });
        } else if (cPortals && cPortals.length > 0) {
          // #F4: Route external threat through chunk `c`'s open portals (doorways)
          const tChunk = grid.chunkIdAt(clamp(tx | 0, 0, grid.width - 1), clamp(ty | 0, 0, grid.height - 1));
          const tPorts = graph.chunkPortals[tChunk];
          for (let pIdx = 0; pIdx < cPortals.length; pIdx++) {
            const pid = cPortals[pIdx], p = graph.portals[pid];
            if (tPorts && tPorts.length > 0 && graph.portalComp[pid] !== graph.portalComp[tPorts[0]]) continue;
            const d = octile(tx - p.anchorX, ty - p.anchorY) * COST_OPEN;
            const cells = portalSideCells(p, c);
            for (let j = 0; j < cells.length; j++) {
              seeds.push({ cell: grid.localIndex(cells[j]), weight: d, id: i });
            }
          }
        } else {
          const clx = clamp(lx, 0, CS - 1), cly = clamp(ly, 0, CS - 1);
          const d = octile(tx - (ox + clx + 0.5), ty - (oy + cly + 0.5)) * COST_OPEN;
          seeds.push({ cell: cly * CS + clx, weight: d, id: i });
        }
      }
    }

    const k = this.key(c, 'f', h, minClearance, 0, profile);
    const hit = this.entries.get(k);
    if (this._match(hit, c, 'f', h, minClearance, 0, profile)) return this._promote(k, hit);

    const f = this._allocField(c, 'f', false);
    this.it.integrateFlee(grid, c, seeds, f.integration, opts);
    computeFlowVectors(f, 0, 0, { grid });
    this._tag(f, c, 'f', h, minClearance, 0, profile);
    return this._store(k, f);
  }

  getCorridorField(c, polyline, opts = {}) {
    const minClearance = opts.minClearance ?? 1;
    const profile = opts.profile ?? null;
    const extra = ((opts.radius ?? 2.5) * 100) | 0;
    let h = polyline.length;
    for (let i = 0; i < polyline.length; i++) {
      const p = polyline[i];
      const x = Array.isArray(p) ? p[0] : p.x, y = Array.isArray(p) ? p[1] : p.y;
      h = hashMix(hashMix(h, x * 10), y * 10);
    }
    const k = this.key(c, 'c', h, minClearance, extra, profile);
    const hit = this.entries.get(k);
    if (this._match(hit, c, 'c', h, minClearance, extra, profile)) return this._promote(k, hit);

    const f = buildCorridorField(this.grid, c, polyline, { ...opts, pool: this.pool, integrator: this.it });
    this._tag(f, c, 'c', h, minClearance, extra, profile);
    return this._store(k, f);
  }

  clear() {
    for (const f of this.entries.values()) this.pool.releaseField(f);
    this.entries.clear();
  }

  invalidateChunks(dirtyChunks) {
    for (const [k, f] of this.entries) {
      if (dirtyChunks.has(f.chunkId)) {
        this.pool.releaseField(f);
        this.entries.delete(k);
      }
    }
  }

  build(k, cid, kind, src, exX, exY, opts = {}) {
    const { grid } = this;
    const f = this._allocField(cid, kind, !!opts.trackSources);
    this.it.integrate(grid, cid, src, f.integration, {
      minClearance: opts.minClearance ?? 1,
      profile: opts.profile,
      sourceOut: f.sourceId
    });
    computeFlowVectors(f, exX, exY, {
      grid,
      vortex: opts.vortex,
      losSmoothing: opts.losSmoothing,
      chunkCosts: this.it.costs,
      targetLocalX: opts.targetLocalX,
      targetLocalY: opts.targetLocalY,
      borderExitMask: opts.borderExitMask,
      borderExX: opts.borderExX,
      borderExY: opts.borderExY
    });
    return this._store(k, f);
  }
}
//#endregion

//#region 19: Bilinear Flow Vector Sampler (#A3)
/**
 * #A3: Bilinearly interpolates the flow direction vector `(out.x, out.y)` at world position `(px, py)`.
 */
export function sampleFlowBilinear(f, px, py, out = { x: 0, y: 0 }, neighborResolver = null) {
  const lx = px - f.originX - 0.5, ly = py - f.originY - 0.5;
  const x0 = Math.floor(lx), y0 = Math.floor(ly), x1 = x0 + 1, y1 = y0 + 1;
  const tx = lx - x0, ty = ly - y0;
  const { integration: I, flowX: FX, flowY: FY } = f;
  const w00 = (1 - tx) * (1 - ty), w10 = tx * (1 - ty), w01 = (1 - tx) * ty, w11 = tx * ty;

  let sx = 0, sy = 0, ws = 0;
  const accum = (cx, cy, w) => {
    if (w <= 0) return;
    if (cx >>> 0 < CS && cy >>> 0 < CS) {
      const id = cy * CS + cx;
      if (I[id] < Infinity) { sx += FX[id] * w; sy += FY[id] * w; ws += w; }
    } else if (neighborResolver) {
      const wx = f.originX + cx, wy = f.originY + cy;
      const nf = neighborResolver(wx, wy);
      if (nf) {
        const nid = (wy - nf.originY) * CS + (wx - nf.originX);
        if (nf.integration[nid] < Infinity) {
          sx += nf.flowX[nid] * w; sy += nf.flowY[nid] * w; ws += w;
          return;
        }
      }
      const ccx = clamp(cx, 0, CS - 1), ccy = clamp(cy, 0, CS - 1), id = ccy * CS + ccx;
      if (I[id] < Infinity) { sx += FX[id] * w; sy += FY[id] * w; ws += w; }
    } else {
      const ccx = clamp(cx, 0, CS - 1), ccy = clamp(cy, 0, CS - 1), id = ccy * CS + ccx;
      if (I[id] < Infinity) { sx += FX[id] * w; sy += FY[id] * w; ws += w; }
    }
  };

  accum(x0, y0, w00);
  accum(x1, y0, w10);
  accum(x0, y1, w01);
  accum(x1, y1, w11);

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

//#region 20: SoA Crowd Steering Simulator, RVO-Lite & Settling (#C2, #G1, #G2, #G4)
/** Agent state constants for `CrowdSystem` (#G2) */
export const AGENT_STATE = Object.freeze({
  IDLE: 0,
  MOVING: 1,
  ARRIVING: 2,
  SETTLED: 3
});

/**
 * #C2, #G1, #G2, #G4: Structure-of-Arrays (SoA) Crowd Steering Simulator featuring:
 * - #G1: Predictive RVO-Lite (Time-to-Collision $\tau$ horizon avoidance) + Continuous Wall-SDF Repulsion
 * - #G2: Contagious Goal-Arrival Settling (stops destination mosh-pitting) & Stuck-Agent Auto-Repathing
 * - #G4: Automatic portal crossing traffic recording for Wardrop load-balancing
 */
export class CrowdSystem {
  constructor(pathfinder, capacity = 4096, opts = {}) {
    this.pf = pathfinder;
    this.cap = capacity;
    this.count = 0;

    this.px = new Float32Array(capacity);
    this.py = new Float32Array(capacity);
    this.vx = new Float32Array(capacity);
    this.vy = new Float32Array(capacity);
    this.radius = new Float32Array(capacity);
    this.speed = new Float32Array(capacity);
    this.hintIdx = new Int32Array(capacity);
    this.active = new Uint8Array(capacity);
    this.state = new Uint8Array(capacity);
    this.stuckTimer = new Float32Array(capacity);
    this.anchorX = new Float32Array(capacity);
    this.anchorY = new Float32Array(capacity);
    this.targets = new Array(capacity).fill(null);

    this.separationWeight = opts.separationWeight ?? 1.35;
    this.rvoWeight = opts.rvoWeight ?? 0.85;
    this.rvoHorizon = opts.rvoHorizon ?? 0.65;
    this.wallSlideWeight = opts.wallSlideWeight ?? 1.1;
    this.steeringSmooth = opts.steeringSmooth ?? 0.25;
    this.arrivalRadius = opts.arrivalRadius ?? 0.85;
    this.settleClusterRadius = opts.settleClusterRadius ?? 6.0;
    this.stuckTimeout = opts.stuckTimeout ?? 1.25;
    this.autoRepath = opts.autoRepath ?? true;
    this.autoStampCongestion = opts.autoStampCongestion ?? false;
    this.congestionUnitWeight = opts.congestionUnitWeight ?? 0.5;

    const cells = pathfinder.width * pathfinder.height;
    this.cellHead = new Int32Array(cells);
    this.agentNext = new Int32Array(capacity);
    this._vec = { x: 0, y: 0 };
  }

  _grow() {
    const nCap = this.cap * 2;
    const copy = (Arr, src) => { const a = new Arr(nCap); a.set(src); return a; };
    this.px = copy(Float32Array, this.px);
    this.py = copy(Float32Array, this.py);
    this.vx = copy(Float32Array, this.vx);
    this.vy = copy(Float32Array, this.vy);
    this.radius = copy(Float32Array, this.radius);
    this.speed = copy(Float32Array, this.speed);
    this.hintIdx = copy(Int32Array, this.hintIdx);
    this.active = copy(Uint8Array, this.active);
    this.state = copy(Uint8Array, this.state);
    this.stuckTimer = copy(Float32Array, this.stuckTimer);
    this.anchorX = copy(Float32Array, this.anchorX);
    this.anchorY = copy(Float32Array, this.anchorY);
    this.agentNext = new Int32Array(nCap);
    this.targets.length = nCap;
    this.cap = nCap;
  }

  addAgent(x, y, { speed = 3.0, radius = 0.35, path = null } = {}) {
    if (this.count >= this.cap) this._grow();
    const id = this.count++;
    this.px[id] = this.anchorX[id] = x;
    this.py[id] = this.anchorY[id] = y;
    this.vx[id] = 0; this.vy[id] = 0;
    this.speed[id] = speed; this.radius[id] = radius;
    this.hintIdx[id] = 0; this.active[id] = 1;
    this.state[id] = path ? AGENT_STATE.MOVING : AGENT_STATE.IDLE;
    this.stuckTimer[id] = 0;
    this.targets[id] = path;
    return id;
  }

  setAgentTarget(id, pathOrSwarm) {
    this.targets[id] = pathOrSwarm;
    this.hintIdx[id] = 0;
    this.state[id] = pathOrSwarm ? AGENT_STATE.MOVING : AGENT_STATE.IDLE;
    this.stuckTimer[id] = 0;
    this.anchorX[id] = this.px[id];
    this.anchorY[id] = this.py[id];
  }

  removeAgent(id) {
    if (id >= 0 && id < this.count) {
      this.active[id] = 0;
      this.state[id] = AGENT_STATE.IDLE;
      this.targets[id] = null;
    }
  }

  step(dt = 1 / 60) {
    const {
      pf, count, px, py, vx, vy, radius, speed, hintIdx, active, state,
      stuckTimer, anchorX, anchorY, targets, cellHead, agentNext, _vec
    } = this;
    const { grid } = pf, W = grid.width, H = grid.height;

    // 1. Rebuild O(N) spatial hash & optionally stamp directional velocity flux (#C1)
    cellHead.fill(-1);
    if (this.autoStampCongestion) {
      const C = grid.ensureCongestionLayer(true);
      C.fill(0);
      grid.congestionFluxX.fill(0);
      grid.congestionFluxY.fill(0);
    }

    for (let i = 0; i < count; i++) {
      if (!active[i]) continue;
      const cx = clamp(px[i] | 0, 0, W - 1), cy = clamp(py[i] | 0, 0, H - 1);
      const cell = cy * W + cx;
      agentNext[i] = cellHead[cell];
      cellHead[cell] = i;
      if (this.autoStampCongestion) {
        const uw = this.congestionUnitWeight;
        grid.congestion[cell] += uw;
        grid.congestionFluxX[cell] += vx[i] * uw;
        grid.congestionFluxY[cell] += vy[i] * uw;
        grid.markChunkWeightDirty(grid.chunkIdAt(cx, cy));
      }
    }

    const sepW = this.separationWeight, rvoW = this.rvoWeight, tauMax = this.rvoHorizon;
    const wallW = this.wallSlideWeight, smooth = this.steeringSmooth;
    const arrR2 = this.arrivalRadius * this.arrivalRadius;
    const settleR2 = this.settleClusterRadius * this.settleClusterRadius;
    const hasSDF = !!grid.wallDist;

    for (let i = 0; i < count; i++) {
      if (!active[i]) continue;
      const x = px[i], y = py[i], r = radius[i], spd = speed[i];
      const t = targets[i];

      if (!t || state[i] === AGENT_STATE.SETTLED || state[i] === AGENT_STATE.IDLE) {
        vx[i] *= 0.6; vy[i] *= 0.6;
        if (Math.hypot(vx[i], vy[i]) < 1e-3) { vx[i] = vy[i] = 0; continue; }
      }

      let fx = 0, fy = 0;
      let gx = -1, gy = -1, distToGoal2 = Infinity;

      if (t && state[i] !== AGENT_STATE.SETTLED) {
        gx = t.goalX ?? -1; gy = t.goalY ?? -1;
        if (gx >= 0) {
          distToGoal2 = (gx - x) * (gx - x) + (gy - y) * (gy - y);
          // #G2: Direct arrival within arrivalRadius
          if (distToGoal2 <= arrR2) {
            state[i] = AGENT_STATE.SETTLED;
            vx[i] = vy[i] = 0;
            continue;
          }
          state[i] = distToGoal2 <= settleR2 ? AGENT_STATE.ARRIVING : AGENT_STATE.MOVING;
        }

        if (t.isSwarm) {
          if (pf.sampleSwarmFlow(t, x, y, _vec)) { fx = _vec.x; fy = _vec.y; }
        } else {
          const prevHint = hintIdx[i];
          const res = pf.getFieldForPath(t, x, y, prevHint);
          if (res) {
            // #G4: Record portal crossing traffic when agent advances along path
            if (res.chunkIndex > prevHint && prevHint < t.portals.length && t.graph) {
              t.graph.addPortalTraffic(t.portals[prevHint], 1.0);
            }
            hintIdx[i] = res.chunkIndex;
            if (sampleFlowBilinear(res.field, x, y, _vec)) { fx = _vec.x; fy = _vec.y; }
          }
        }

        // #G2: Stuck-agent detection & automatic re-pathing around dynamic obstacles
        if (this.autoRepath && !t.isSwarm && state[i] === AGENT_STATE.MOVING) {
          if ((x - anchorX[i]) ** 2 + (y - anchorY[i]) ** 2 > 0.25) {
            anchorX[i] = x; anchorY[i] = y; stuckTimer[i] = 0;
          } else {
            stuckTimer[i] += dt;
            if (stuckTimer[i] >= this.stuckTimeout && gx >= 0) {
              stuckTimer[i] = 0;
              const newPath = pf.findPath(x, y, gx, gy, { minClearance: t.minClearance, profile: t.profile });
              if (newPath) { targets[i] = newPath; hintIdx[i] = 0; }
            }
          }
        }
      }

      // 2. Spatial-hash separation + #G1 Predictive RVO-Lite + #G2 Contagious Arrival Settling
      let sepX = 0, sepY = 0, rvoX = 0, rvoY = 0;
      const cx = clamp(x | 0, 0, W - 1), cy = clamp(y | 0, 0, H - 1);
      const yMin = Math.max(0, cy - 1), yMax = Math.min(H - 1, cy + 1);
      const xMin = Math.max(0, cx - 1), xMax = Math.min(W - 1, cx + 1);
      const vxi = vx[i], vyi = vy[i];
      let settledByNeighbor = false;

      for (let ny = yMin; ny <= yMax; ny++) {
        const row = ny * W;
        for (let nx = xMin; nx <= xMax; nx++) {
          for (let j = cellHead[row + nx]; j !== -1; j = agentNext[j]) {
            if (i === j) continue;
            const dx = x - px[j], dy = y - py[j];
            const minSep = r + radius[j];
            const d2 = dx * dx + dy * dy;
            if (d2 <= 0) continue;

            // Overlap separation
            if (d2 < minSep * minSep) {
              const d = Math.sqrt(d2), push = (minSep - d) / d;
              sepX += dx * push; sepY += dy * push;

              // #G2: Contagious settling when bumping into an already settled agent near the same goal
              if (state[i] === AGENT_STATE.ARRIVING && state[j] === AGENT_STATE.SETTLED) {
                const tj = targets[j];
                if (tj && tj.goalX === gx && tj.goalY === gy) {
                  settledByNeighbor = true;
                }
              }
            } else if (rvoW > 0 && tauMax > 0 && d2 < 9.0) {
              // #G1: Predictive RVO-Lite time-to-closest-approach avoidance
              const dvx = vxi - vx[j], dvy = vyi - vy[j];
              const dv2 = dvx * dvx + dvy * dvy;
              if (dv2 > 1e-4) {
                const dpDotDv = dx * dvx + dy * dvy;
                if (dpDotDv < 0) {
                  const tc = clamp(-dpDotDv / dv2, 0, tauMax);
                  const fxD = dx + dvx * tc, fyD = dy + dvy * tc;
                  const fd2 = fxD * fxD + fyD * fyD;
                  const safeR = minSep + 0.15;
                  if (fd2 < safeR * safeR && fd2 > 1e-6) {
                    const fd = Math.sqrt(fd2);
                    const w = ((safeR - fd) / fd) * (1 - tc / tauMax);
                    rvoX += fxD * w; rvoY += fyD * w;
                  }
                }
              }
            }
          }
        }
      }

      if (settledByNeighbor) {
        state[i] = AGENT_STATE.SETTLED;
        vx[i] = vy[i] = 0;
        continue;
      }

      // #G1: Continuous Wall-SDF boundary repulsion
      let wallX = 0, wallY = 0;
      if (hasSDF && wallW > 0) {
        const gIdx = cy * W + cx, wd = grid.wallDist[gIdx], minWallD = r + 0.45;
        if (wd > 0 && wd < minWallD) {
          const push = (1 - wd / minWallD) * wallW;
          wallX = grid.wallGradX[gIdx] * push;
          wallY = grid.wallGradY[gIdx] * push;
        }
      }

      let desX = fx + sepX * sepW + rvoX * rvoW + wallX;
      let desY = fy + sepY * sepW + rvoY * rvoW + wallY;
      const dl = Math.hypot(desX, desY);
      if (dl > 1) { desX /= dl; desY /= dl; }

      // Arrival braking ramp
      const brake = state[i] === AGENT_STATE.ARRIVING ? clamp(Math.sqrt(distToGoal2) / this.settleClusterRadius, 0.35, 1.0) : 1.0;
      const nvx = vxi + (desX * spd * brake - vxi) * smooth;
      const nvy = vyi + (desY * spd * brake - vyi) * smooth;
      vx[i] = nvx; vy[i] = nvy;

      let nx = clamp(x + nvx * dt, 0.01, W - 0.01);
      let ny = clamp(y + nvy * dt, 0.01, H - 0.01);

      if (!grid.isWalkable(nx | 0, ny | 0)) {
        if (grid.isWalkable(nx | 0, y | 0)) { ny = y; vy[i] = 0; }
        else if (grid.isWalkable(x | 0, ny | 0)) { nx = x; vx[i] = 0; }
        else { nx = x; ny = y; vx[i] = vy[i] = 0; }
      }
      px[i] = nx; py[i] = ny;
    }
  }
}
//#endregion

//#region 21: Hierarchical Squad & Formation Controller (#G3)
/**
 * #G3: Hierarchical Squad & Formation Controller (`SquadSystem`).
 * Manages virtual squad leaders, rubber-banded leader speed throttling when squad members lag in bottlenecks,
 * and formation slot presets (`'line'`, `'column'`, `'wedge'`, `'box'`, `'circle'`).
 */
export class SquadSystem {
  constructor(pathfinder, crowd = null) {
    this.pf = pathfinder;
    this.crowd = crowd || pathfinder.createCrowd();
    this.squads = new Map();
    this._nextSquadId = 1;
    this._vec = { x: 0, y: 0 };
  }

  /** Generates local `(forward, right)` slot offsets for `count` units */
  static generateSlots(count, formation = 'wedge', spacing = 1.25) {
    const slots = [];
    for (let i = 0; i < count; i++) {
      if (formation === 'line') {
        const right = (i - (count - 1) * 0.5) * spacing;
        slots.push({ forward: 0, right });
      } else if (formation === 'column') {
        slots.push({ forward: -i * spacing, right: 0 });
      } else if (formation === 'circle') {
        const angle = (2 * Math.PI * i) / Math.max(1, count);
        const rad = Math.max(spacing, (count * spacing) / (2 * Math.PI));
        slots.push({ forward: Math.cos(angle) * rad, right: Math.sin(angle) * rad });
      } else if (formation === 'box') {
        const cols = Math.ceil(Math.sqrt(count));
        const row = (i / cols) | 0, col = i % cols;
        slots.push({ forward: -row * spacing, right: (col - (cols - 1) * 0.5) * spacing });
      } else {
        // Default: 'wedge' / V-formation
        if (i === 0) slots.push({ forward: 0, right: 0 });
        else {
          const rank = ((i + 1) >> 1), side = (i & 1) ? 1 : -1;
          slots.push({ forward: -rank * spacing * 0.85, right: side * rank * spacing * 0.75 });
        }
      }
    }
    return slots;
  }

  createSquad(leaderX, leaderY, agentIds, { formation = 'wedge', spacing = 1.25, speed = 3.0, path = null, maxLagDist = 5.0 } = {}) {
    const id = this._nextSquadId++;
    const slots = SquadSystem.generateSlots(agentIds.length, formation, spacing);
    const members = agentIds.map((agentId, idx) => ({
      agentId,
      forward: slots[idx].forward,
      right: slots[idx].right
    }));
    const squad = {
      id, leaderX, leaderY, dirX: 1, dirY: 0,
      speed, formation, spacing, path,
      leaderHintIdx: 0, maxLagDist, members
    };
    this.squads.set(id, squad);
    return squad;
  }

  setSquadFormation(squadId, formation, spacing = 1.25) {
    const sq = this.squads.get(squadId);
    if (!sq) return;
    sq.formation = formation; sq.spacing = spacing;
    const slots = SquadSystem.generateSlots(sq.members.length, formation, spacing);
    for (let i = 0; i < sq.members.length; i++) {
      sq.members[i].forward = slots[i].forward;
      sq.members[i].right = slots[i].right;
    }
  }

  setSquadTarget(squadId, pathOrSwarm) {
    const sq = this.squads.get(squadId);
    if (!sq) return;
    sq.path = pathOrSwarm;
    sq.leaderHintIdx = 0;
  }

  step(dt = 1 / 60) {
    const { pf, crowd, _vec } = this;
    for (const sq of this.squads.values()) {
      const { path, members } = sq;
      if (!path || members.length === 0) continue;

      // Measure squad cohesion lag to rubber-band virtual leader speed
      let sumLag = 0;
      for (let i = 0; i < members.length; i++) {
        const aid = members[i].agentId;
        const d = Math.hypot(crowd.px[aid] - sq.leaderX, crowd.py[aid] - sq.leaderY);
        const expected = Math.hypot(members[i].forward, members[i].right);
        if (d > expected) sumLag += (d - expected);
      }
      const avgLag = sumLag / members.length;
      const throttle = clamp(1 - avgLag / Math.max(1, sq.maxLagDist), 0.15, 1.0);

      // Advance virtual squad leader along path/swarm
      let ok = false;
      if (path.isSwarm) {
        ok = pf.sampleSwarmFlow(path, sq.leaderX, sq.leaderY, _vec);
      } else {
        const res = pf.getFieldForPath(path, sq.leaderX, sq.leaderY, sq.leaderHintIdx);
        if (res) {
          sq.leaderHintIdx = res.chunkIndex;
          ok = sampleFlowBilinear(res.field, sq.leaderX, sq.leaderY, _vec);
        }
      }
      if (ok) {
        sq.dirX = _vec.x; sq.dirY = _vec.y;
        const gx = path.goalX ?? -1, gy = path.goalY ?? -1;
        if (gx < 0 || Math.hypot(gx - sq.leaderX, gy - sq.leaderY) > 0.6) {
          sq.leaderX = clamp(sq.leaderX + _vec.x * sq.speed * throttle * dt, 0.5, pf.width - 0.5);
          sq.leaderY = clamp(sq.leaderY + _vec.y * sq.speed * throttle * dt, 0.5, pf.height - 0.5);
        }
      }

      // Steer each squad member using clearance-adaptive formation flow (#C4)
      for (let i = 0; i < members.length; i++) {
        const m = members[i], aid = m.agentId;
        if (!crowd.active[aid]) continue;
        if (pf.sampleFormationFlow(path, sq.leaderX, sq.leaderY, m.forward, m.right, crowd.px[aid], crowd.py[aid], _vec, {
          leaderHintIdx: sq.leaderHintIdx,
          agentHintIdx: crowd.hintIdx[aid]
        })) {
          crowd.vx[aid] = _vec.x * crowd.speed[aid];
          crowd.vy[aid] = _vec.y * crowd.speed[aid];
        }
      }
    }
  }
}
//#endregion
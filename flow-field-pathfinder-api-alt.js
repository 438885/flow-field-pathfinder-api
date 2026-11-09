// flow-field-pathfinder-api.js (v2.0)

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

//#region 2: 4-Ary Cache-Friendly Min-Heap & Slab Buffer Pool (#B4, #B5)
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
    let { it, k } = this, top = it[0], li = --this.sz, item = it[li], key = k[li], p = 0, ch;
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

//#region 3: Multi-Layer Chunked Cost Grid
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

  // #D4: Terrain ID Layer & Unit Cost Profiles
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

  // #C3: Dynamic Moving Obstacle Stamp Layer
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

  // #4 & #B2: Wall Repulsion / Signed Distance Field with Dirty-Halo Regional Update
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

  // #5 & #B2: Clearance Map with Dirty-Halo Regional Brushfire Update
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

  // #6 & #E1: Tactical Overlay & Wall-Occluded FOV Shadowcasting
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

  // #9 & #C1: Continuum Crowd Congestion + Anisotropic Velocity-Flux Layer
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

  // #11 & #A5: Anisotropic Directional Passability & Currents / Slopes
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

//#region 4: Portal Detection & Multi-Mode Chunk Integrator (#A2, #A5, #C1, #D4)
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
            // #A2: Split wide portals into balanced sub-portals of at most `maxPortalLen`
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

/**
 * Multi-mode chunk integrator supporting:
 * - #1 & #A5: Per-cell Eikonal quadratic upwind solver (compatible with localized anisotropy)
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
  dist = new Float64Array(CC);
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

        // #C1: Anisotropic Velocity-Flux Congestion (counter-flow penalty / co-flow discount)
        if (hasFlux) {
          const fx = fluxX[fromCell], fy = fluxY[fromCell];
          if (fx !== 0 || fy !== 0) {
            const fl = Math.hypot(fx, fy);
            const align = (DX[travelDir] * fx + DY[travelDir] * fy) / (DL[d] * fl);
            stepScale *= clamp(1 - align * laneW, 0.45, 2.25);
          }
        }

        let nd = du + DL[d] * stepScale;

        // #1 & #A5: Per-cell Eikonal quadratic upwind refinement (active on all non-one-way cells)
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

//#region 5: Portal Graph, Connected Components & Macro / Swarm Pathfinder (#B1, #B3, #D1)
/**
 * High-level abstract graph connecting chunk boundary portals in CSR format.
 * Supports:
 * - #A2: Multi-cell portal span evaluation
 * - #B1: In-place CSR weight refreshes (`refreshWeights`) without rebuilding topology
 * - #D1: Connected component labeling (`portalComp`) for $O(1)$ reachability checks
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
    this.chunkDistCache = Array.from({ length: grid.chunkCount }, (_, c) =>
      canReuse && !dirtyChunks.has(c) ? prevGraph.chunkDistCache[c] : new Map()
    );

    const adj = Array.from({ length: P }, () => []);
    const it = this._it = new ChunkIntegrator();
    const field = this._field = new Float64Array(CC);
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
          row = new Float64Array(field);
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
    this.edgeCosts = new Float64Array(E);
    this.edgeChunks = new Int32Array(E);
    for (let p = 0, e = 0; p < P; this.edgeOffsets[++p] = e) {
      for (let k = 0; k < adj[p].length; k++) {
        const edge = adj[p][k];
        this.edgeTargets[e] = edge.to; this.edgeCosts[e] = edge.cost; this.edgeChunks[e++] = edge.chunk;
      }
    }

    // Build reverse CSR for #B3 (Global Reverse-Dijkstra Swarm Field) & #D1 (Components)
    this._buildReverseAndComponents();
  }

  /** #A2: Evaluates multi-sampled distance across a portal's cells (blending min and center) */
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

    // #D1: Connected Component Labeling via undirected BFS over portals
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
  }

  /**
   * #B1: Surgical in-place refresh of CSR edge costs for chunks with modified weights
   * (congestion, tactical costs, currents, non-blocking terrain edits).
   */
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
        cCache.set(locA, new Float64Array(field));
      }
      for (let ib = 0; ib < ids.length; ib++) {
        const idB = ids[ib], pB = portals[idB];
        for (let e = this.edgeOffsets[idB]; e < this.edgeOffsets[idB + 1]; e++) {
          if (this.edgeChunks[e] !== c) continue;
          const idA = this.edgeTargets[e], pA = portals[idA];
          const row = cCache.get(pLocal(grid, pA, c));
          const crossA = crossingCost(grid, pA, c, profile);
          const d = row ? this._samplePortalRowDist(row, pB, c) : Infinity;
          this.edgeCosts[e] = d + crossA;
        }
      }
    }
    this.weightVersion++;
  }
}

/**
 * Hierarchical A* & Swarm Broadcast Pathfinder operating over `PortalGraph`.
 * Supports single goals, Multi-Source Geodesic Voronoi goals (#2),
 * Connected-Component $O(1)$ unreachable rejection (#D1), and Global Reverse-Dijkstra Swarm Fields (#B3).
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
    this._swarmIdCounter = 1;
  }

  /** #B1: Selectively invalidates only cached goals residing inside modified chunks */
  invalidateChunks(dirtyChunks) {
    for (const [cell, goal] of this.goalCache) {
      if (dirtyChunks.has(goal.chunk)) this.goalCache.delete(cell);
    }
  }

  createGoal(cell) {
    const { grid, graph } = this, x = cell % grid.width, y = cell / grid.width | 0;
    const chunk = grid.chunkIdAt(x, y), field = new Float64Array(CC);
    this.it.integrate(grid, chunk, [grid.localIndex(cell)], field, {
      minClearance: graph.minClearance,
      profile: graph.profile
    });
    const portalCost = new Float64Array(graph.portals.length).fill(Infinity);
    const cPortals = graph.chunkPortals[chunk];
    const compSet = new Set();
    for (let i = 0; i < cPortals.length; i++) {
      const pid = cPortals[i], p = graph.portals[pid];
      const d = graph._samplePortalRowDist(field, p, chunk);
      if (d < Infinity) {
        portalCost[pid] = d;
        compSet.add(graph.portalComp[pid]);
      }
    }
    return { isMulti: false, cell, x, y, chunk, field, portalCost, compSet, graph };
  }

  createMultiGoal(targets) {
    const { grid, graph } = this;
    const portalCost = new Float64Array(graph.portals.length).fill(Infinity);
    const portalGoalCell = new Int32Array(graph.portals.length).fill(-1);
    const chunkGroups = new Map();
    const goalList = [];
    const compSet = new Set();

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
      profile: graph.profile,
      forward: true
    });

    // Check direct intra-chunk reachability & #D1 component overlap
    let sameChunkDirect = false;
    if (!goal.isMulti && sc === goal.chunk && goal.field[grid.localIndex(startCell)] < Infinity) {
      sameChunkDirect = true;
    } else if (goal.isMulti && goal.chunkFields.has(sc) && goal.chunkFields.get(sc).field[grid.localIndex(startCell)] < Infinity) {
      sameChunkDirect = true;
    }

    const cPorts = graph.chunkPortals[sc];
    if (!sameChunkDirect && goal.compSet) {
      let compMatch = false;
      for (let i = 0; i < cPorts.length; i++) {
        const pid = cPorts[i], p = graph.portals[pid];
        if (this.startField[pLocal(grid, p, sc)] < Infinity && goal.compSet.has(graph.portalComp[pid])) {
          compMatch = true;
          break;
        }
      }
      // #D1: O(1) instant abort if start and goal share no connected component
      if (!compMatch) return null;
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
        const fgx = (chosenGoalCell % grid.width) + 0.5, fgy = ((chosenGoalCell / grid.width) | 0) + 0.5;
        return {
          portals: new Int32Array(portals.reverse()),
          chunks: new Int32Array(chunks.reverse()),
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
   * Allows 10,000+ agents anywhere on the map to flow toward `targets` with zero per-agent `findPath` calls.
   */
  createSwarmField(targets) {
    const goal = Array.isArray(targets) ? this.createMultiGoal(targets) : this.getOrCreateGoal(targets);
    const { graph, heap } = this;
    const P = graph.portals.length;
    const portalDist = new Float64Array(P).fill(Infinity);
    const portalNext = new Int32Array(P).fill(-1);
    heap.clear();

    for (let p = 0; p < P; p++) {
      const gc = goal.portalCost[p];
      if (gc < Infinity) {
        portalDist[p] = gc;
        portalNext[p] = -2; // Direct to goal inside chunk
        heap.push(p, gc);
      }
    }

    // Reverse Dijkstra across graph edges (from target `v` backward to source `u`)
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

//#region 6: Flow Vector Extraction, Lookahead Seeding, Cache & Samplers (#A1, #A3, #B4, #D2)
/**
 * #3: Fast chunk-local supercover Line-of-Sight check.
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

  for (let y = 0; y < CS; y++) {
    for (let x = 0; x < CS; x++) {
      const id = y * CS + x, cur = I[id];
      if (cur === Infinity) { FX[id] = FY[id] = 0; continue; }
      if ((!cur || (borderExitMask && borderExitMask[id])) && !vortex && (exX || exY)) {
        // #A1: For lookahead-seeded portal cells, blend exit normal with tangent gradient along portal
        const E = at(x + 1, y), W = at(x - 1, y), S = at(x, y + 1), N = at(x, y - 1);
        const gx = exX !== 0 ? 0 : (E < W ? (E < cur ? E - cur : 0) : W < cur ? cur - W : 0);
        const gy = exY !== 0 ? 0 : (S < N ? (S < cur ? S - cur : 0) : N < cur ? cur - N : 0);
        const vx = exX - gx * 0.45, vy = exY - gy * 0.45, vl = Math.hypot(vx, vy) || 1;
        FX[id] = vx / vl; FY[id] = vy / vl;
        continue;
      }
      if (!cur && !vortex) { FX[id] = exX; FY[id] = exY; continue; }

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
          dx = bx / l2 || exX; dy = by / l2 || exY;
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

/**
 * #10 & #D2: Multi-Chunk Guide-Curve / Spline Corridor Flow Field builder.
 * Supports curves inside the chunk as well as curves in neighboring chunks by seeding
 * closest border entry cells with their cross-track distance and curve tangent.
 */
export function buildCorridorField(grid, chunkId, polyline, opts = {}) {
  const ox = grid.chunkOriginX(chunkId), oy = grid.chunkOriginY(chunkId);
  const pool = opts.pool || null;
  const f = {
    chunkId, kind: 'c', originX: ox, originY: oy,
    integration: pool ? pool.allocF32() : new Float32Array(CC),
    flowX: pool ? pool.allocF32() : new Float32Array(CC),
    flowY: pool ? pool.allocF32() : new Float32Array(CC),
    sourceId: pool ? pool.allocI32() : new Int32Array(CC)
  };
  const spineTanX = new Float32Array(CC), spineTanY = new Float32Array(CC);
  const seedDist = new Float32Array(CC).fill(Infinity), sources = [];
  const minClearance = opts.minClearance ?? 1;

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
      } else {
        // #D2: Cross-chunk corridor projection onto chunk perimeter
        const clx = clamp(lx, 0, CS - 1), cly = clamp(ly, 0, CS - 1);
        if (grid.isWalkable(ox + clx, oy + cly, minClearance, opts.profile)) {
          const cell = cly * CS + clx;
          const d = Math.hypot(wx - (ox + clx + 0.5), wy - (oy + cly + 0.5)) * COST_OPEN;
          if (d < seedDist[cell]) {
            seedDist[cell] = d;
            spineTanX[cell] = tx; spineTanY[cell] = ty;
          }
        }
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

/**
 * #A1, #B4, #D2: LRU cache of per-chunk flow fields backed by `FieldBufferPool`.
 * Caches Portal (with lookahead), Goal, Swarm (#B3), Multi-Goal (#2), Vortex (#7), Flee (#8), and Corridor (#10) fields.
 */
export class FlowFieldCache {
  constructor(grid, cap = 2048) {
    this.grid = grid;
    this.cap = cap;
    this.entries = new Map();
    this.it = new ChunkIntegrator();
    this.pool = new FieldBufferPool(Math.min(cap, 1024));
    this._borderMask = new Uint8Array(CC);
  }

  key(c, kind, tid, minClearance = 1, extra = 0) {
    let h = hashMix(2166136261, c);
    h = hashMix(h, kind.charCodeAt(0));
    h = hashMix(h, tid);
    h = hashMix(h, minClearance);
    return hashMix(h, extra);
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
   * #A1: Builds or retrieves a portal flow field with optional Cross-Boundary Wavefront Seeding (`nextField`)
   * so waves curve smoothly into the optimal crossing point before reaching the chunk seam.
   */
  getPortalField(c, p, minClearance = 1, nextField = null, nextTid = 0, profile = null) {
    const k = this.key(c, 'p', p.uid, minClearance, nextTid);
    const hit = this.entries.get(k);
    if (hit && hit.chunkId === c) return this._promote(k, hit);

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

    return this.build(k, c, 'p', sources, portalExitX(p, c), portalExitY(p, c), {
      minClearance,
      profile,
      borderExitMask: nextField && minNext < Infinity ? _borderMask : null
    });
  }

  getGoalField(c, cell, minClearance = 1, profile = null) {
    const k = this.key(c, 'g', cell, minClearance);
    const hit = this.entries.get(k);
    if (hit && hit.chunkId === c) return this._promote(k, hit);
    const loc = this.grid.localIndex(cell);
    return this.build(k, c, 'g', [loc], 0, 0, {
      minClearance,
      profile,
      losSmoothing: this.grid.losSmoothing,
      targetLocalX: (loc % CS) + 0.5,
      targetLocalY: ((loc / CS) | 0) + 0.5
    });
  }

  /** #B3: Builds and caches the unified per-chunk field for a Global Swarm Broadcast Field */
  getSwarmChunkField(c, swarm) {
    const k = this.key(c, 's', swarm.uid, swarm.minClearance, swarm.weightVersion);
    const hit = this.entries.get(k);
    if (hit && hit.chunkId === c) return this._promote(k, hit);

    const { grid, _borderMask } = this;
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
    let domExX = 0, domExY = 0, bestP = -1, bestPD = Infinity;

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
      const relW = pd - minW;
      if (pd < bestPD) {
        bestPD = pd; bestP = p;
        domExX = portalExitX(p, c); domExY = portalExitY(p, c);
      }
      for (let j = 0; j < cells.length; j++) {
        const loc = grid.localIndex(cells[j]);
        _borderMask[loc] = 1;
        sources.push({ cell: loc, weight: relW, id: pid });
      }
    }

    const singleGoalLoc = inChunkGoals && inChunkGoals.length === 1 && bestPD > 0 ? inChunkGoals[0].cell : -1;
    return this.build(k, c, 's', sources, inChunkGoals ? 0 : domExX, inChunkGoals ? 0 : domExY, {
      minClearance: swarm.minClearance,
      profile: swarm.profile,
      borderExitMask: !inChunkGoals && bestP ? _borderMask : null,
      losSmoothing: singleGoalLoc >= 0 && grid.losSmoothing,
      targetLocalX: singleGoalLoc >= 0 ? (singleGoalLoc % CS) + 0.5 : undefined,
      targetLocalY: singleGoalLoc >= 0 ? ((singleGoalLoc / CS) | 0) + 0.5 : undefined
    });
  }

  /** #B4: Cached Multi-Goal Voronoi field */
  getMultiGoalField(c, sources, minClearance = 1, profile = null) {
    let h = sources.length;
    for (let i = 0; i < sources.length; i++) {
      const s = sources[i];
      h = hashMix(h, typeof s === 'object' ? s.cell : s);
    }
    const k = this.key(c, 'm', h, minClearance);
    const hit = this.entries.get(k);
    if (hit && hit.chunkId === c) return this._promote(k, hit);

    const f = this._allocField(c, 'm', true);
    this.it.integrate(this.grid, c, sources, f.integration, { minClearance, profile, sourceOut: f.sourceId });
    computeFlowVectors(f, 0, 0, { grid: this.grid });
    return this._store(k, f);
  }

  /** #7, #B4 & #D2: Cached Vortex / Orbit field supporting both intra-chunk and cross-chunk vortex centers */
  getVortexField(c, centerCell, vortexOpts = {}) {
    const minClearance = vortexOpts.minClearance ?? 1;
    const rKey = ((vortexOpts.radius ?? 4) * 100) | 0;
    const k = this.key(c, 'v', centerCell, minClearance, rKey ^ (vortexOpts.clockwise === false ? 1 : 0));
    const hit = this.entries.get(k);
    if (hit && hit.chunkId === c) return this._promote(k, hit);

    const { grid } = this;
    const f = this._allocField(c, 'v', false);
    f.vortex = vortexOpts;
    const centerChunk = grid.chunkIdAt(centerCell % grid.width, (centerCell / grid.width) | 0);

    if (centerChunk === c) {
      const loc = grid.localIndex(centerCell);
      this.it.integrate(grid, c, [loc], f.integration, { minClearance, profile: vortexOpts.profile });
      computeFlowVectors(f, 0, 0, {
        grid,
        vortex: vortexOpts,
        losSmoothing: grid.losSmoothing,
        chunkCosts: this.it.costs,
        targetLocalX: (loc % CS) + 0.5,
        targetLocalY: ((loc / CS) | 0) + 0.5
      });
    } else {
      // #D2: Multi-chunk vortex: seed walkable chunk perimeter cells with geodesic octile distance to vortex center
      const ox = f.originX, oy = f.originY;
      const vcx = (centerCell % grid.width) + 0.5, vcy = ((centerCell / grid.width) | 0) + 0.5;
      const seeds = [];
      for (let t = 0; t < CS; t++) {
        const perim = [t, (CS - 1) * CS + t, t * CS, t * CS + (CS - 1)];
        for (let b = 0; b < 4; b++) {
          const loc = perim[b], lx = loc % CS, ly = (loc / CS) | 0;
          if (grid.isWalkable(ox + lx, oy + ly, minClearance, vortexOpts.profile)) {
            const d = octile((ox + lx + 0.5) - vcx, (oy + ly + 0.5) - vcy) * COST_OPEN;
            seeds.push({ cell: loc, weight: d, id: loc });
          }
        }
      }
      this.it.integrate(grid, c, seeds, f.integration, { minClearance, profile: vortexOpts.profile });
      computeFlowVectors(f, 0, 0, { grid, vortex: vortexOpts });
    }
    return this._store(k, f);
  }

  /** #8, #B4 & #D2: Cached 2-Pass Flee field supporting threats inside or outside the chunk */
  getFleeField(c, threatPointsOrCells, opts = {}) {
    const minClearance = opts.minClearance ?? 1;
    const { grid } = this, ox = grid.chunkOriginX(c), oy = grid.chunkOriginY(c);
    let h = threatPointsOrCells.length;
    const seeds = [];

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
        } else {
          // #D2: Threat in outside chunk — project true incoming distance onto chunk perimeter
          const clx = clamp(lx, 0, CS - 1), cly = clamp(ly, 0, CS - 1);
          const d = octile(tx - (ox + clx + 0.5), ty - (oy + cly + 0.5)) * COST_OPEN;
          seeds.push({ cell: cly * CS + clx, weight: d, id: i });
        }
      }
    }

    const k = this.key(c, 'f', h, minClearance);
    const hit = this.entries.get(k);
    if (hit && hit.chunkId === c) return this._promote(k, hit);

    const f = this._allocField(c, 'f', false);
    this.it.integrateFlee(grid, c, seeds, f.integration, opts);
    computeFlowVectors(f, 0, 0, { grid });
    return this._store(k, f);
  }

  /** #10, #B4 & #D2: Cached Multi-Chunk Guide-Curve Corridor field */
  getCorridorField(c, polyline, opts = {}) {
    const minClearance = opts.minClearance ?? 1;
    let h = polyline.length;
    for (let i = 0; i < polyline.length; i++) {
      const p = polyline[i];
      const x = Array.isArray(p) ? p[0] : p.x, y = Array.isArray(p) ? p[1] : p.y;
      h = hashMix(hashMix(h, x * 10), y * 10);
    }
    const k = this.key(c, 'c', h, minClearance, ((opts.radius ?? 2.5) * 100) | 0);
    const hit = this.entries.get(k);
    if (hit && hit.chunkId === c) return this._promote(k, hit);

    const f = buildCorridorField(this.grid, c, polyline, { ...opts, pool: this.pool, integrator: this.it });
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
      borderExitMask: opts.borderExitMask
    });
    return this._store(k, f);
  }
}

/**
 * #A3: Bilinearly interpolates the flow direction vector `(out.x, out.y)` at world position `(px, py)`.
 * Supports seamless cross-chunk boundary blending when `neighborResolver(wx, wy)` is supplied.
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
      // #A3: Sample adjacent chunk cell across seam
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

//#region 7: Built-in SoA Crowd Simulator & Spatial Hash Steering (#C2)
/**
 * #C2: High-performance Structure-of-Arrays (SoA) Crowd Steering & Spatial Hash Simulator.
 * Handles 10,000+ agents with O(N) spatial hash neighbor separation, Wall SDF sliding,
 * automatic hintIdx progression, Swarm/Macro path following, and Velocity-Flux Congestion stamping (#C1).
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
    this.targets = new Array(capacity).fill(null);

    this.separationWeight = opts.separationWeight ?? 1.35;
    this.steeringSmooth = opts.steeringSmooth ?? 0.25;
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
    this.agentNext = new Int32Array(nCap);
    this.targets.length = nCap;
    this.cap = nCap;
  }

  addAgent(x, y, { speed = 3.0, radius = 0.35, path = null } = {}) {
    if (this.count >= this.cap) this._grow();
    const id = this.count++;
    this.px[id] = x; this.py[id] = y;
    this.vx[id] = 0; this.vy[id] = 0;
    this.speed[id] = speed; this.radius[id] = radius;
    this.hintIdx[id] = 0; this.active[id] = 1;
    this.targets[id] = path;
    return id;
  }

  setAgentTarget(id, pathOrSwarm) {
    this.targets[id] = pathOrSwarm;
    this.hintIdx[id] = 0;
  }

  removeAgent(id) {
    if (id >= 0 && id < this.count) {
      this.active[id] = 0;
      this.targets[id] = null;
    }
  }

  /**
   * Advances all active agents by `dt` seconds using spatial-hash separation and flow sampling.
   */
   step(dt = 1 / 60) {
    const { pf, count, px, py, vx, vy, radius, speed, hintIdx, active, targets, cellHead, agentNext, _vec } = this;
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

    // 2. Steer, separate & slide agents
    const sepW = this.separationWeight, smooth = this.steeringSmooth;
    for (let i = 0; i < count; i++) {
      if (!active[i]) continue;
      const x = px[i], y = py[i], r = radius[i], spd = speed[i];
      let fx = 0, fy = 0;

      const t = targets[i];
      if (t) {
        if (t.isSwarm) {
          if (pf.sampleSwarmFlow(t, x, y, _vec)) { fx = _vec.x; fy = _vec.y; }
        } else {
          const res = pf.getFieldForPath(t, x, y, hintIdx[i]);
          if (res) {
            hintIdx[i] = res.chunkIndex;
            if (sampleFlowBilinear(res.field, x, y, _vec)) { fx = _vec.x; fy = _vec.y; }
          }
        }
      }

      // Short-range spatial hash separation (3x3 neighborhood)
      let sepX = 0, sepY = 0;
      const cx = clamp(x | 0, 0, W - 1), cy = clamp(y | 0, 0, H - 1);
      const yMin = Math.max(0, cy - 1), yMax = Math.min(H - 1, cy + 1);
      const xMin = Math.max(0, cx - 1), xMax = Math.min(W - 1, cx + 1);

      for (let ny = yMin; ny <= yMax; ny++) {
        const row = ny * W;
        for (let nx = xMin; nx <= xMax; nx++) {
          for (let j = cellHead[row + nx]; j !== -1; j = agentNext[j]) {
            if (i === j) continue;
            const dx = x - px[j], dy = y - py[j];
            const minSep = r + radius[j];
            const d2 = dx * dx + dy * dy;
            if (d2 > 0 && d2 < minSep * minSep) {
              const d = Math.sqrt(d2), push = (minSep - d) / d;
              sepX += dx * push; sepY += dy * push;
            }
          }
        }
      }

      let desX = fx + sepX * sepW, desY = fy + sepY * sepW;
      const dl = Math.hypot(desX, desY);
      if (dl > 1) { desX /= dl; desY /= dl; }

      const nvx = vx[i] + (desX * spd - vx[i]) * smooth;
      const nvy = vy[i] + (desY * spd - vy[i]) * smooth;
      vx[i] = nvx; vy[i] = nvy;

      let nx = clamp(x + nvx * dt, 0.01, W - 0.01);
      let ny = clamp(y + nvy * dt, 0.01, H - 0.01);

      // Axis-separated wall collision sliding
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

//#region 8: Unified FlowFieldPathfinder API
/**
 * Unified Hierarchical Flow Field Pathfinding API (v2.0) exposing all flow field types,
 * Two-Tier Incremental Rebuilds (#B1), Swarm Broadcast Fields (#B3), Cross-Boundary Lookahead (#A1),
 * Seam-Aware Sampling (#A3), Squad Formation Sampling (#C4), Terrain Profiles (#D4), and Raycast/FOV (#E1).
 */
export class FlowFieldPathfinder {
  constructor(opts = {}) {
    const cx = opts.width ? Math.ceil(opts.width / CS) : (opts.chunksX ?? 10);
    const cy = opts.height ? Math.ceil(opts.height / CS) : (opts.chunksY ?? 10);
    this.cacheCap = opts.cacheCap ?? 2048;
    this.minClearance = opts.minClearance ?? 1;
    this.portalLookahead = opts.portalLookahead ?? true;
    this.seamInterpolation = opts.seamInterpolation ?? true;

    this.grid = new GridWorld(cx, cy, opts.costs || null);
    if (opts.useEikonal !== undefined) this.grid.useEikonal = opts.useEikonal;
    if (opts.losSmoothing !== undefined) this.grid.losSmoothing = opts.losSmoothing;
    if (opts.wallRepulsion) this.grid.enableWallRepulsion(opts.wallRepulsion);

    this.graph = new PortalGraph(this.grid, null, null, this.minClearance, null);
    this.macro = new MacroPathfinder(this.grid, this.graph);
    this.cache = new FlowFieldCache(this.grid, this.cacheCap);
    this.clearanceGraphs = new Map([[this._hierarchyKey(this.minClearance, null), { graph: this.graph, macro: this.macro }]]);
    this._tmpVec = { x: 0, y: 0 };
  }

  get width() { return this.grid.width; }
  get height() { return this.grid.height; }

  _hierarchyKey(minClearance, profile) {
    return profile ? `${minClearance}:${profile}` : minClearance;
  }

  getCost(x, y) { return this.grid.getCost(x, y); }
  setCost(x, y, cost) { return this.grid.setCost(x, y, cost); }
  setRect(x, y, w, h, cost) { return this.grid.setRect(x, y, w, h, cost); }
  setDisc(x, y, r, cost) { return this.grid.setDisc(x, y, r, cost); }
  loadCosts(costs) { this.grid.loadCosts(costs); }

  // #D4: Terrain & Profile API delegates
  registerProfile(name, costMapOrArray) { return this.grid.registerProfile(name, costMapOrArray); }
  setTerrain(x, y, terrainId) { return this.grid.setTerrain(x, y, terrainId); }
  setRectTerrain(x, y, w, h, terrainId) { return this.grid.setRectTerrain(x, y, w, h, terrainId); }

  // #C3: Dynamic Moving Obstacle API delegates
  stampDynamicObstacle(wx, wy, r, blocked = 1) { this.grid.stampDynamicObstacle(wx, wy, r, blocked); }
  clearDynamicObstacles() { this.grid.clearDynamicObstacles(); }

  // #E1: World Raycast & Tactical FOV Shadowcasting delegates
  raycast(x0, y0, x1, y1, opts = {}) { return this.grid.raycast(x0, y0, x1, y1, opts); }
  addTacticalFOV(wx, wy, r, peakCost, linearFalloff = true) { this.grid.addTacticalFOV(wx, wy, r, peakCost, linearFalloff); }

  // #C2: Factory helper for SoA CrowdSystem
  createCrowd(capacity = 4096, opts = {}) { return new CrowdSystem(this, capacity, opts); }

  /**
   * #B1 & #B2: Two-Tier Incremental Rebuild:
   * - If `topologyDirtyChunks` is non-empty, updates regional SDF/Clearance and rebuilds affected `PortalGraph` chunks.
   * - If only `weightsDirtyChunks` is non-empty (e.g. congestion/tactical/currents), updates CSR edge weights in-place
   *   and preserves `PortalGraph` topology and unaffected `MacroPathfinder.goalCache` entries.
   */
  rebuild() {
    const { topology, weights, all } = this.grid.consumeDirtyState();
    if (all.size === 0) return false;

    if (topology.size > 0 && (this.grid.wallPenaltyWeight > 0 || this.grid.wallRepulsionWeight > 0)) {
      this.grid.updateWallSDF(false);
    }
    if (topology.size > 0 && this.grid.clearance) {
      this.grid.updateClearance(false);
    }

    this.cache.invalidateChunks(all);

    for (const [hKey, entry] of this.clearanceGraphs) {
      if (topology.size > 0) {
        const nextGraph = new PortalGraph(this.grid, entry.graph, all, entry.graph.minClearance, entry.graph.profile);
        const nextMacro = new MacroPathfinder(this.grid, nextGraph);
        this.clearanceGraphs.set(hKey, { graph: nextGraph, macro: nextMacro });
        if (hKey === this.minClearance) {
          this.graph = nextGraph;
          this.macro = nextMacro;
        }
      } else if (weights.size > 0) {
        // #B1: Fast path — surgical in-place CSR weight update without destroying graph or goalCache
        entry.graph.refreshWeights(weights);
        entry.macro.invalidateChunks(weights);
      }
    }
    return true;
  }

  _getHierarchy(minClearance = this.minClearance, profile = null) {
    if (this.grid.dirtyChunks.size > 0) this.rebuild();
    const hKey = this._hierarchyKey(minClearance, profile);
    let entry = this.clearanceGraphs.get(hKey);
    if (!entry) {
      const graph = new PortalGraph(this.grid, null, null, minClearance, profile);
      const macro = new MacroPathfinder(this.grid, graph);
      entry = { graph, macro };
      this.clearanceGraphs.set(hKey, entry);
    }
    return entry;
  }

  /**
   * Computes a single-goal hierarchical macro path (supports `opts.minClearance` and `opts.profile`).
   */
  findPath(sx, sy, gx, gy, opts = {}) {
    const clr = opts.minClearance ?? this.minClearance;
    const prof = opts.profile ?? null;
    const { macro } = this._getHierarchy(clr, prof);
    const { grid } = this;
    const scx = clamp(sx | 0, 0, grid.width - 1), scy = clamp(sy | 0, 0, grid.height - 1);
    const gcx = clamp(gx | 0, 0, grid.width - 1), gcy = clamp(gy | 0, 0, grid.height - 1);
    const startCell = grid.isWalkable(scx, scy, clr, prof) ? grid.index(scx, scy) : grid.findNearestWalkable(scx, scy, 16, clr, prof);
    const goalCell = grid.isWalkable(gcx, gcy, clr, prof) ? grid.index(gcx, gcy) : grid.findNearestWalkable(gcx, gcy, 16, clr, prof);
    if (startCell < 0 || goalCell < 0) return null;
    return macro.findPath(startCell, goalCell);
  }

  /**
   * #2: Computes a Multi-Goal Geodesic Voronoi macro path to the best weighted target in `goals`.
   */
  findMultiGoalPath(sx, sy, goals, opts = {}) {
    const clr = opts.minClearance ?? this.minClearance;
    const prof = opts.profile ?? null;
    const { macro } = this._getHierarchy(clr, prof);
    const { grid } = this;
    const scx = clamp(sx | 0, 0, grid.width - 1), scy = clamp(sy | 0, 0, grid.height - 1);
    const startCell = grid.isWalkable(scx, scy, clr, prof) ? grid.index(scx, scy) : grid.findNearestWalkable(scx, scy, 16, clr, prof);
    if (startCell < 0) return null;
    const validTargets = [];
    for (const g of goals) {
      const gx = clamp(g.x | 0, 0, grid.width - 1), gy = clamp(g.y | 0, 0, grid.height - 1);
      const cell = grid.isWalkable(gx, gy, clr, prof) ? grid.index(gx, gy) : grid.findNearestWalkable(gx, gy, 16, clr, prof);
      if (cell >= 0) validTargets.push({ cell, weight: g.weight ?? 0 });
    }
    if (!validTargets.length) return null;
    return macro.findPath(startCell, validTargets);
  }

  /**
   * #B3: Creates a Global Reverse-Dijkstra Swarm Broadcast Field toward one or more goals.
   * @param {Array<{x:number, y:number, weight?:number}>|{x:number, y:number}} goals
   * @param {Object} [opts={}] - `{ minClearance, profile }`
   */
  createSwarmField(goals, opts = {}) {
    const clr = opts.minClearance ?? this.minClearance;
    const prof = opts.profile ?? null;
    const { macro } = this._getHierarchy(clr, prof);
    const { grid } = this;
    const list = Array.isArray(goals) ? goals : [goals];
    const valid = [];
    for (const g of list) {
      const gx = clamp(g.x | 0, 0, grid.width - 1), gy = clamp(g.y | 0, 0, grid.height - 1);
      const cell = grid.isWalkable(gx, gy, clr, prof) ? grid.index(gx, gy) : grid.findNearestWalkable(gx, gy, 16, clr, prof);
      if (cell >= 0) valid.push({ cell, weight: g.weight ?? 0 });
    }
    if (!valid.length) return null;
    return macro.createSwarmField(valid.length === 1 && valid[0].weight === 0 ? valid[0].cell : valid);
  }

  /**
   * #B3: Resolves the cached 16x16 flow field for the chunk at `(wx, wy)` within a `swarmField`.
   */
  getFieldForSwarm(swarm, wx, wy) {
    if (!swarm) return null;
    const { graph } = this._getHierarchy(swarm.minClearance, swarm.profile);
    if (swarm.graph !== graph) return null;
    const { grid } = this;
    const cx = clamp(wx | 0, 0, grid.width - 1), cy = clamp(wy | 0, 0, grid.height - 1);
    const chunk = grid.chunkIdAt(cx, cy);
    return this.cache.getSwarmChunkField(chunk, swarm);
  }

  /**
   * #B3 & #A3: Samples the bilinear flow direction `(out.x, out.y)` at `(wx, wy)` from a `swarmField`.
   */
  sampleSwarmFlow(swarm, wx, wy, out = { x: 0, y: 0 }) {
    const field = this.getFieldForSwarm(swarm, wx, wy);
    if (!field) { out.x = 0; out.y = 0; return false; }
    const resolver = this.seamInterpolation ? (nx, ny) => {
      if (!this.grid.inBounds(nx, ny)) return null;
      return this.cache.getSwarmChunkField(this.grid.chunkIdAt(nx, ny), swarm);
    } : null;
    return sampleFlowBilinear(field, wx, wy, out, resolver);
  }

  /**
   * #7 & #D2: Builds or retrieves a cached Vortex / Orbit flow field for the chunk at `(wx, wy)`.
   * Supports multi-chunk orbits via `opts.centerX` and `opts.centerY` (defaults to `wx, wy`).
   */
  getVortexField(wx, wy, opts = {}) {
    const { grid } = this;
    const qx = clamp(wx | 0, 0, grid.width - 1), qy = clamp(wy | 0, 0, grid.height - 1);
    const chunk = grid.chunkIdAt(qx, qy);
    const cx = clamp((opts.centerX ?? wx) | 0, 0, grid.width - 1);
    const cy = clamp((opts.centerY ?? wy) | 0, 0, grid.height - 1);
    return this.cache.getVortexField(chunk, grid.index(cx, cy), opts);
  }

  /**
   * #8 & #D2: Builds or retrieves a cached 2-Pass Flee safety flow field for the chunk at `(wx, wy)`
   * supporting threats in both local and neighboring chunks.
   */
  getFleeField(wx, wy, threatPoints, opts = {}) {
    const { grid } = this;
    const cx = clamp(wx | 0, 0, grid.width - 1), cy = clamp(wy | 0, 0, grid.height - 1);
    return this.cache.getFleeField(grid.chunkIdAt(cx, cy), threatPoints, opts);
  }

  /**
   * #10 & #D2: Builds or retrieves a cached Guide-Curve / Spline Corridor flow field for the chunk at `(wx, wy)`.
   */
  getCorridorField(wx, wy, polyline, opts = {}) {
    const { grid } = this;
    const cx = clamp(wx | 0, 0, grid.width - 1), cy = clamp(wy | 0, 0, grid.height - 1);
    return this.cache.getCorridorField(grid.chunkIdAt(cx, cy), polyline, opts);
  }

  /**
   * #A1 & #D1: Resolves the 16x16 flow field for step `idx` along `path`, seeding portal exits
   * with Cross-Boundary Wavefront Lookahead and disambiguating bisected sub-chunk pockets.
   */
  _resolveStepField(path, graph, stepIdx) {
    const chunk = path.chunks[stepIdx];
    const clr = path.minClearance ?? this.minClearance;
    const prof = path.profile ?? null;
    if (stepIdx >= path.portals.length) {
      return this.cache.getGoalField(chunk, path.goalCell, clr, prof);
    }
    const p = graph.portals[path.portals[stepIdx]];
    let nextField = null, nextTid = 0;
    if (this.portalLookahead && stepIdx + 1 < path.chunks.length) {
      const nextChunk = path.chunks[stepIdx + 1];
      if (stepIdx + 1 < path.portals.length) {
        const nextP = graph.portals[path.portals[stepIdx + 1]];
        nextTid = nextP.uid;
        nextField = this.cache.getPortalField(nextChunk, nextP, clr, null, 0, prof);
      } else {
        nextTid = path.goalCell + 1;
        nextField = this.cache.getGoalField(nextChunk, path.goalCell, clr, prof);
      }
    }
    return this.cache.getPortalField(chunk, p, clr, nextField, nextTid, prof);
  }

  /**
   * Resolves the 16x16 flow field for the chunk at `(wx, wy)` along a given `path`.
   * #D1: If a bisected chunk is visited multiple times along `path`, searches for the pocket where `(wx, wy)` is reachable.
   */
  getFieldForPath(path, wx, wy, hintIdx = 0) {
    if (!path) return null;
    const clr = path.minClearance ?? this.minClearance;
    const prof = path.profile ?? null;
    const { graph } = this._getHierarchy(clr, prof);
    if (path.graph !== graph) return null;
    const { grid } = this;
    const cx = clamp(wx | 0, 0, grid.width - 1), cy = clamp(wy | 0, 0, grid.height - 1);
    const chunk = grid.chunkIdAt(cx, cy);
    const localIdx = (cy % CS) * CS + (cx % CS);

    // Check hintIdx first
    if (hintIdx >= 0 && hintIdx < path.chunks.length && path.chunks[hintIdx] === chunk) {
      const f = this._resolveStepField(path, graph, hintIdx);
      if (f.integration[localIdx] < Infinity) return { field: f, chunkIndex: hintIdx };
    }

    // #D1: Search forward then backward for a matching chunk occurrence where `localIdx` is reachable
    for (let i = Math.max(0, hintIdx); i < path.chunks.length; i++) {
      if (path.chunks[i] === chunk) {
        const f = this._resolveStepField(path, graph, i);
        if (f.integration[localIdx] < Infinity) return { field: f, chunkIndex: i };
      }
    }
    for (let i = Math.min(path.chunks.length - 1, hintIdx - 1); i >= 0; i--) {
      if (path.chunks[i] === chunk) {
        const f = this._resolveStepField(path, graph, i);
        if (f.integration[localIdx] < Infinity) return { field: f, chunkIndex: i };
      }
    }
    return null;
  }

  /**
   * #A3: Samples the bilinear flow direction `(out.x, out.y)` at `(wx, wy)` along `path`,
   * smoothly blending across chunk boundaries when within 0.5 cells of a chunk seam.
   */
  sampleFlow(path, wx, wy, out = { x: 0, y: 0 }, hintIdx = 0) {
    const res = this.getFieldForPath(path, wx, wy, hintIdx);
    if (!res) { out.x = 0; out.y = 0; return false; }
    const resolver = this.seamInterpolation ? (nx, ny) => {
      if (!this.grid.inBounds(nx, ny)) return null;
      const nRes = this.getFieldForPath(path, nx, ny, res.chunkIndex);
      return nRes ? nRes.field : null;
    } : null;
    return sampleFlowBilinear(res.field, wx, wy, out, resolver);
  }

  /**
   * #C4: Clearance-Adaptive Squad / Formation Flow Sampler.
   * Rotates the formation offset `(slotForward, slotRight)` along the leader's heading,
   * automatically compresses lateral spacing through narrow doorways/corridors using the clearance map,
   * and blends the slot-seeking vector with the agent's local path flow.
   */
  sampleFormationFlow(pathOrSwarm, leaderX, leaderY, slotForward, slotRight, agentX, agentY, out = { x: 0, y: 0 }, opts = {}) {
    const { grid, _tmpVec } = this;
    const isSwarm = pathOrSwarm && pathOrSwarm.isSwarm;
    const okLeader = isSwarm
      ? this.sampleSwarmFlow(pathOrSwarm, leaderX, leaderY, _tmpVec)
      : this.sampleFlow(pathOrSwarm, leaderX, leaderY, _tmpVec, opts.leaderHintIdx ?? 0);
    const lfx = okLeader ? _tmpVec.x : 1, lfy = okLeader ? _tmpVec.y : 0;

    if (grid._clearanceDirty || !grid.clearance) grid.updateClearance();
    const lCell = grid.index(clamp(leaderX | 0, 0, grid.width - 1), clamp(leaderY | 0, 0, grid.height - 1));
    const localClr = grid.clearance[lCell];
    const formRadius = opts.formationRadius ?? Math.max(1.5, Math.abs(slotRight));
    const scale = clamp(localClr / formRadius, opts.minCompression ?? 0.2, 1.0);

    // Rotate compressed local slot into world space relative to leader flow tangent
    const rightX = -lfy, rightY = lfx;
    const targetX = leaderX + lfx * slotForward + rightX * (slotRight * scale);
    const targetY = leaderY + lfy * slotForward + rightY * (slotRight * scale);

    const okAgent = isSwarm
      ? this.sampleSwarmFlow(pathOrSwarm, agentX, agentY, out)
      : this.sampleFlow(pathOrSwarm, agentX, agentY, out, opts.agentHintIdx ?? 0);
    const afx = okAgent ? out.x : lfx, afy = okAgent ? out.y : lfy;

    const dx = targetX - agentX, dy = targetY - agentY, dist = Math.hypot(dx, dy);
    const springW = clamp(dist * (opts.slotGain ?? 0.6), 0, opts.maxSlotWeight ?? 1.25);
    const sx = dist > 1e-4 ? (dx / dist) * springW : 0;
    const sy = dist > 1e-4 ? (dy / dist) * springW : 0;

    const bx = afx + sx, by = afy + sy, bl = Math.hypot(bx, by);
    if (bl < 1e-6) { out.x = 0; out.y = 0; return false; }
    out.x = bx / bl; out.y = by / bl;
    return true;
  }
}

export default FlowFieldPathfinder;
//#endregion
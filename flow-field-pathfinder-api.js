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
 * P4: TypedArray-backed 4-ary min-heap for Eikonal, Dijkstra, and A* searches.
 * Halves sift-up tree depth and improves cache locality over a 2-ary heap.
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
    while (c > 0 && k[(p = (c - 1) >> 2)] > key) {
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
    while ((ch = (p << 2) + 1) < li) {
      let best = ch,
        bKey = k[ch];
      if (ch + 1 < li && k[ch + 1] < bKey) {
        best = ch + 1;
        bKey = k[ best];
      }
      if (ch + 2 < li && k[ch + 2] < bKey) {
        best = ch + 2;
        bKey = k[best];
      }
      if (ch + 3 < li && k[ch + 3] < bKey) {
        best = ch + 3;
        bKey = k[best];
      }
      if (bKey >= key) break;
      it[p] = it[best];
      k[p] = bKey;
      p = best;
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
 * exact separable Euclidean wall SDF repulsion (#4, P3), tactical overlays (#6),
 * velocity-predictive crowd congestion (#9, D2), anisotropic directional masks & currents (#11),
 * two-tier topology vs. weight dirty tracking (B1), chunk topology versioning (D3),
 * off-grid custom jump/teleporter portals (D1), multi-modal terrain cost profiles (A3),
 * and world-space supercover LOS raycasting (A5).
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

    // B1 & D3: Two-Tier Dirty Tracking + Per-Chunk Topology Version Stamps
    this.dirtyChunks = new Set();
    this.topologyDirtyChunks = new Set();
    this.weightDirtyChunks = new Set();
    this.chunkTopoVersion = new Uint32Array(this.chunkCount);

    // Solver & feature configuration
    this.useEikonal = true;
    this.losSmoothing = true;

    // #4, B3 & P3: Wall Repulsion / Exact Separable Euclidean SDF with incremental dirty bounds
    this.wallPenaltyRadius = 2.5;
    this.wallPenaltyWeight = 0.0;
    this.wallRepulsionWeight = 0.0;
    this.wallDist = null;
    this.wallGradX = null;
    this.wallGradY = null;
    this._sdfDirty = true;
    this._sdfFullDirty = true;
    this._sdfDirtyChunks = new Set();
    this._edtF = null;
    this._edtV = null;
    this._edtZ = null;

    // #5 & B3: Clearance-Annotated Routing (unit size in cells) with incremental dirty bounds
    this.clearance = null;
    this._clearanceDirty = true;
    this._clearanceFullDirty = true;
    this._clearanceDirtyChunks = new Set();

    // A3: Multi-Modal Terrain Tags & Unit Traversal Cost Profiles
    this.terrainTags = null;
    this.costProfiles = new Map();
    this._profileClearance = new Map();

    // D1: Off-Grid Jump Links, Teleporters, Bridges & Multi-Layer Portals
    this.customPortals = [];
    this._nextCustomPortalUid = 1 << 20;

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

  /**
   * A3: Returns the effective base traversal cost at linear cell index `idx` for `profileId`.
   */
  getEffectiveBaseCost(idx, profileId = 0) {
    if (profileId !== 0 && this.terrainTags) {
      const table = this.costProfiles.get(profileId);
      if (table) {
        const mapped = table[this.terrainTags[idx]];
        if (mapped > 0) return mapped;
      }
    }
    return this.costs[idx];
  }

  isWalkable(x, y, minClearance = 1, profileId = 0) {
    if (!this.inBounds(x, y)) return false;
    const idx = this.index(x, y);
    if (this.getEffectiveBaseCost(idx, profileId) >= COST_BLOCK) return false;
    if (minClearance > 1) {
      const clrMap = this.getClearanceMap(profileId);
      return clrMap[idx] >= minClearance;
    }
    return true;
  }

  chunkIdAt(x, y) { return (y / CS | 0) * this.cx + (x / CS | 0); }
  chunkOriginX(c) { return (c % this.cx) * CS; }
  chunkOriginY(c) { return (c / this.cx | 0) * CS; }
  localIndex(cell) { return ((cell / this.width | 0) % CS) * CS + (cell % this.width % CS); }

  _markChunkSet(set, x, y, bumpTopo = false) {
    const c = this.chunkIdAt(x, y);
    set.add(c);
    if (bumpTopo) this.chunkTopoVersion[c]++;
    const lx = x % CS, ly = y % CS;
    if (lx === 0 && x > 0) {
      set.add(c - 1);
      if (bumpTopo) this.chunkTopoVersion[c - 1]++;
    }
    if (lx === CS - 1 && x < this.width - 1) {
      set.add(c + 1);
      if (bumpTopo) this.chunkTopoVersion[c + 1]++;
    }
    if (ly === 0 && y > 0) {
      set.add(c - this.cx);
      if (bumpTopo) this.chunkTopoVersion[c - this.cx]++;
    }
    if (ly === CS - 1 && y < this.height - 1) {
      set.add(c + this.cx);
      if (bumpTopo) this.chunkTopoVersion[c + this.cx]++;
    }
  }

  markCellDirty(x, y, isTopology = true) {
    this._markChunkSet(this.dirtyChunks, x, y, false);
    if (isTopology) {
      this._markChunkSet(this.topologyDirtyChunks, x, y, true);
      const cid = this.chunkIdAt(x, y);
      this._sdfDirtyChunks.add(cid);
      this._clearanceDirtyChunks.add(cid);
      this._sdfDirty = true;
      this._clearanceDirty = true;
      this._profileClearance.clear();
    } else {
      this._markChunkSet(this.weightDirtyChunks, x, y, false);
    }
  }

  markChunkWeightDirty(c) {
    this.dirtyChunks.add(c);
    this.weightDirtyChunks.add(c);
  }

  markChunkTopologyDirty(c) {
    if (c < 0 || c >= this.chunkCount) return;
    this.dirtyChunks.add(c);
    this.topologyDirtyChunks.add(c);
    this.chunkTopoVersion[c]++;
  }

  getCost(x, y, profileId = 0) {
    return this.inBounds(x | 0, y | 0)
      ? this.getEffectiveBaseCost((y | 0) * this.width + (x | 0), profileId)
      : COST_BLOCK;
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
      this.chunkTopoVersion[c]++;
    }
    this._sdfDirty = true;
    this._sdfFullDirty = true;
    this._clearanceDirty = true;
    this._clearanceFullDirty = true;
    this._profileClearance.clear();
  }

  // A3: Multi-Modal Terrain Tag Layer & Cost Profile Registration
  ensureTerrainTags() {
    if (!this.terrainTags) this.terrainTags = new Uint8Array(this.width * this.height);
    return this.terrainTags;
  }

  setTerrainTag(x, y, tag) {
    const ix = x | 0, iy = y | 0;
    if (!this.inBounds(ix, iy)) return false;
    const tags = this.ensureTerrainTags();
    const idx = iy * this.width + ix;
    if (tags[idx] === (tag & 0xff)) return false;
    tags[idx] = tag & 0xff;
    this.markCellDirty(ix, iy, true);
    return true;
  }

  setRectTerrainTag(x, y, w, h, tag) {
    const x0 = clamp(Math.floor(x), 0, this.width), x1 = clamp(Math.ceil(x + w), 0, this.width);
    const y0 = clamp(Math.floor(y), 0, this.height), y1 = clamp(Math.ceil(y + h), 0, this.height);
    let changed = 0;
    for (let yy = y0; yy < y1; yy++) {
      for (let xx = x0; xx < x1; xx++) {
        if (this.setTerrainTag(xx, yy, tag)) changed++;
      }
    }
    return changed;
  }

  /**
   * A3: Registers a unit traversal cost profile mapping terrain tag IDs (0..255) to traversal costs (1..255).
   * A table entry of 0 falls back to `this.costs[idx]`.
   */
  registerCostProfile(profileId, tagCostMap) {
    if (!profileId || profileId < 1 || profileId > 15) {
      throw new RangeError('profileId must be an integer in 1..15');
    }
    const table = new Uint8Array(256);
    if (tagCostMap instanceof Uint8Array) {
      table.set(tagCostMap.subarray(0, 256));
    } else if (tagCostMap && typeof tagCostMap === 'object') {
      for (const [k, v] of Object.entries(tagCostMap)) {
        const tag = Number(k) & 0xff;
        table[tag] = clamp(v | 0, 0, COST_BLOCK);
      }
    }
    this.costProfiles.set(profileId, table);
    this._profileClearance.delete(profileId);
    for (let c = 0; c < this.chunkCount; c++) this.markChunkTopologyDirty(c);
    return table;
  }

  // D1: Off-Grid Jump Links, Teleporters, Bridges & Multi-Layer Portals
  addCustomPortal(spec) {
    const { width: W, height: H } = this;
    const xA = clamp(spec.xA | 0, 0, W - 1), yA = clamp(spec.yA | 0, 0, H - 1);
    const xB = clamp(spec.xB | 0, 0, W - 1), yB = clamp(spec.yB | 0, 0, H - 1);
    const span = Math.max(1, spec.width | 0);
    const isH = !!spec.isH;
    const cellsA = new Int32Array(span);
    const cellsB = new Int32Array(span);
    for (let k = 0; k < span; k++) {
      const ax = clamp(xA + (isH ? k : 0), 0, W - 1);
      const ay = clamp(yA + (isH ? 0 : k), 0, H - 1);
      const bx = clamp(xB + (isH ? k : 0), 0, W - 1);
      const by = clamp(yB + (isH ? 0 : k), 0, H - 1);
      cellsA[k] = ay * W + ax;
      cellsB[k] = by * W + bx;
    }
    const mid = (span - 1) >> 1;
    const centerA = cellsA[mid], centerB = cellsB[mid];
    const chunkA = this.chunkIdAt(centerA % W, (centerA / W) | 0);
    const chunkB = this.chunkIdAt(centerB % W, (centerB / W) | 0);
    const uid = this._nextCustomPortalUid++;
    const dxAB = (centerB % W) - (centerA % W), dyAB = ((centerB / W) | 0) - ((centerA / W) | 0);
    const distAB = Math.hypot(dxAB, dyAB) || 1;
    const cp = {
      uid,
      isCustom: true,
      bidirectional: spec.bidirectional !== false,
      customCost: spec.cost ?? null,
      isH,
      chunkA,
      chunkB,
      border: isH ? yA : xA,
      start: isH ? xA : yA,
      end: (isH ? xA : yA) + span - 1,
      cellsA,
      cellsB,
      centerA,
      centerB,
      anchorX: (centerA % W) + 0.5,
      anchorY: ((centerA / W) | 0) + 0.5,
      anchorBX: (centerB % W) + 0.5,
      anchorBY: ((centerB / W) | 0) + 0.5,
      exitAX: spec.exitAX ?? (dxAB / distAB),
      exitAY: spec.exitAY ?? (dyAB / distAB),
      exitBX: spec.exitBX ?? (-dxAB / distAB),
      exitBY: spec.exitBY ?? (-dyAB / distAB)
    };
    this.customPortals.push(cp);
    this.markChunkTopologyDirty(chunkA);
    this.markChunkTopologyDirty(chunkB);
    return uid;
  }

  removeCustomPortal(uid) {
    const idx = this.customPortals.findIndex(p => p.uid === uid);
    if (idx < 0) return false;
    const cp = this.customPortals[idx];
    this.customPortals.splice(idx, 1);
    this.markChunkTopologyDirty(cp.chunkA);
    this.markChunkTopologyDirty(cp.chunkB);
    return true;
  }

  clearCustomPortals() {
    if (!this.customPortals.length) return;
    for (const cp of this.customPortals) {
      this.markChunkTopologyDirty(cp.chunkA);
      this.markChunkTopologyDirty(cp.chunkB);
    }
    this.customPortals.length = 0;
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

  // #4, B3 & P3: Wall Repulsion / Exact Linear-Time Separable Euclidean Distance Transform (Meijster/Felzenszwalb EDT)
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
      const maxDim = Math.max(W, H) + 2;
      this._edtF = new Float64Array(maxDim);
      this._edtV = new Int32Array(maxDim);
      this._edtZ = new Float64Array(maxDim + 1);
      this._sdfFullDirty = true;
    }
    const D = this.wallDist, GX = this.wallGradX, GY = this.wallGradY;
    const F = this._edtF, V = this._edtV, Z = this._edtZ;
    const INF = 1e6;
    const halo = Math.ceil(this.wallPenaltyRadius) + CS;
    const isFull = forceFull || this._sdfFullDirty || this._sdfDirtyChunks.size === 0 || this._sdfDirtyChunks.size * 4 >= this.chunkCount;
    const bounds = isFull ? { x0: 0, y0: 0, x1: W - 1, y1: H - 1 } : this._computeDirtyBounds(this._sdfDirtyChunks, halo);
    const { x0, y0, x1, y1 } = bounds;

    // Pass 1: Exact 1D Vertical Distance Transform per column x in [x0..x1]
    for (let x = x0; x <= x1; x++) {
      const isBorderX = !isFull && ((x === x0 && x0 > 0) || (x === x1 && x1 < W - 1));
      let d = INF;
      for (let y = y0; y <= y1; y++) {
        const i = y * W + x;
        if (costs[i] >= COST_BLOCK) {
          d = 0;
        } else if (!isFull && (isBorderX || (y === y0 && y0 > 0) || (y === y1 && y1 < H - 1))) {
          d = Math.min(d + 1, D[i]);
        } else {
          d = d + 1;
        }
        D[i] = d;
      }
      d = INF;
      for (let y = y1; y >= y0; y--) {
        const i = y * W + x;
        if (D[i] < d + 1) d = D[i];
        else D[i] = ++d;
      }
    }

    // Pass 2: Exact 1D Horizontal Parabolic Lower-Envelope per row y in [y0..y1]
    const spanX = x1 - x0 + 1;
    for (let y = y0; y <= y1; y++) {
      const row = y * W + x0;
      for (let q = 0; q < spanX; q++) {
        const dy = D[row + q];
        F[q] = dy * dy;
      }
      let k = 0;
      V[0] = 0;
      Z[0] = -1e20;
      Z[1] = 1e20;
      for (let q = 1; q < spanX; q++) {
        const fq = F[q] + q * q;
        let r = V[k];
        let s = (fq - (F[r] + r * r)) / (2 * (q - r));
        while (k > 0 && s <= Z[k]) {
          k--;
          r = V[k];
          s = (fq - (F[r] + r * r)) / (2 * (q - r));
        }
        k++;
        V[k] = q;
        Z[k] = s;
        Z[k + 1] = 1e20;
      }
      k = 0;
      for (let q = 0; q < spanX; q++) {
        while (Z[k + 1] < q) k++;
        const dx = q - V[k];
        D[row + q] = Math.sqrt(dx * dx + F[V[k]]);
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

  // #5, B3 & A3: Clearance Map (Full or Chunk-Bounded Incremental Chebyshev Brushfire + Profile Clearance)
  getClearanceMap(profileId = 0) {
    if (profileId === 0 || !this.terrainTags || !this.costProfiles.has(profileId)) {
      if (this._clearanceDirty || !this.clearance) this.updateClearance();
      return this.clearance;
    }
    let pc = this._profileClearance.get(profileId);
    if (!pc) {
      pc = new Uint8Array(this.width * this.height);
      this._computeClearanceBuffer(pc, { x0: 0, y0: 0, x1: this.width - 1, y1: this.height - 1 }, true, profileId);
      this._profileClearance.set(profileId, pc);
    }
    return pc;
  }

  _computeClearanceBuffer(C, bounds, isFull, profileId = 0) {
    const { width: W, height: H } = this;
    const MAX_CLR = CS;
    const { x0, y0, x1, y1 } = bounds;
    for (let y = y0; y <= y1; y++) {
      const row = y * W;
      const isOuterY = !isFull && ((y === y0 && y0 > 0) || (y === y1 && y1 < H - 1));
      for (let x = x0; x <= x1; x++) {
        const i = row + x;
        if (this.getEffectiveBaseCost(i, profileId) >= COST_BLOCK) {
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
  }

  updateClearance(forceFull = false) {
    const { width: W, height: H } = this, len = W * H;
    if (!this.clearance) {
      this.clearance = new Uint8Array(len);
      this._clearanceFullDirty = true;
    }
    const MAX_CLR = CS;
    const isFull = forceFull || this._clearanceFullDirty || this._clearanceDirtyChunks.size === 0 || this._clearanceDirtyChunks.size * 4 >= this.chunkCount;
    const bounds = isFull ? { x0: 0, y0: 0, x1: W - 1, y1: H - 1 } : this._computeDirtyBounds(this._clearanceDirtyChunks, MAX_CLR + 2);
    this._computeClearanceBuffer(this.clearance, bounds, isFull, 0);
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
   * A5 & A3: Fast world-space supercover Line-of-Sight raycast across chunks.
   * Verifies walkability, clearance, terrain cost uniformity, and optional wall buffer.
   */
  hasWorldLOS(x0, y0, x1, y1, maxCost = COST_OPEN, minClearance = 1, profileId = 0) {
    const { width: W, height: H } = this;
    let cx = x0 | 0, cy = y0 | 0;
    const ex = x1 | 0, ey = y1 | 0;
    if (cx < 0 || cy < 0 || cx >= W || cy >= H || ex < 0 || ey < 0 || ex >= W || ey >= H) return false;
    const clrMap = minClearance > 1 ? this.getClearanceMap(profileId) : null;
    const useWall = this.wallPenaltyWeight > 0;
    if (useWall && (this._sdfDirty || !this.wallDist)) this.updateWallSDF();
    const minWallDist = useWall ? Math.min(1.25, this.wallPenaltyRadius * 0.5) : 0;

    const checkCell = (x, y) => {
      const idx = y * W + x;
      const c = this.getEffectiveBaseCost(idx, profileId);
      if (c >= COST_BLOCK || c > maxCost) return false;
      if (clrMap && clrMap[idx] < minClearance) return false;
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

  /**
   * A2: Geometric obstacle visibility raycast (ignores tactical/congestion/cost variations)
   * used to test whether hard obstacles block Line-of-Sight from threats for Cover fields.
   */
  hasLineOfSight(x0, y0, x1, y1, profileId = 0) {
    const { width: W, height: H } = this;
    let cx = x0 | 0, cy = y0 | 0;
    const ex = x1 | 0, ey = y1 | 0;
    if (cx < 0 || cy < 0 || cx >= W || cy >= H || ex < 0 || ey < 0 || ex >= W || ey >= H) return false;
    const dx = Math.abs(ex - cx), dy = Math.abs(ey - cy);
    const sx = cx < ex ? 1 : -1, sy = cy < ey ? 1 : -1;
    let err = dx - dy;
    while (true) {
      if (this.getEffectiveBaseCost(cy * W + cx, profileId) >= COST_BLOCK) return false;
      if (cx === ex && cy === ey) return true;
      const e2 = 2 * err;
      if (e2 > -dy && e2 < dx) {
        if (this.getEffectiveBaseCost(cy * W + cx + sx, profileId) >= COST_BLOCK &&
            this.getEffectiveBaseCost((cy + sy) * W + cx, profileId) >= COST_BLOCK) return false;
      }
      if (e2 > -dy) { err -= dy; cx += sx; }
      if (e2 < dx) { err += dx; cy += sy; }
    }
  }

  findNearestWalkable(x, y, maxR = 24, minClearance = 1, profileId = 0) {
    const { width: W, height: H } = this, sx = clamp(x | 0, 0, W - 1), sy = clamp(y | 0, 0, H - 1);
    if (this.isWalkable(sx, sy, minClearance, profileId)) return sy * W + sx;
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
            if (this.isWalkable(nx, ny, minClearance, profileId) && d < bestD) { bestD = d; bestI = idx; }
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
 * P2 & A3: Detects sub-portals along a single chunk border `(isH, x, y, bIdx)`.
 */
export function detectBorderPortals(grid, isH, x, y, bIdx, minClearance = 1, maxW = 8, profileId = 0) {
  const { cx, width } = grid;
  const border = ((isH ? y : x) + 1) * CS, origin = (isH ? x : y) * CS;
  const cA = y * cx + x, cB = cA + (isH ? cx : 1);
  const out = [];
  let run = -1;
  for (let t = 0; t <= CS; t++) {
    const u = origin + t;
    const walk = t < CS &&
      grid.isWalkable(isH ? u : border - 1, isH ? border - 1 : u, minClearance, profileId) &&
      grid.isWalkable(isH ? u : border, isH ? border : u, minClearance, profileId);
    if (walk) {
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
          const pos = start + k;
          cellsA[k] = isH ? (border - 1) * width + pos : pos * width + (border - 1);
          cellsB[k] = isH ? border * width + pos : pos * width + border;
        }
        out.push({
          id: 0,
          uid: (bIdx * CS + subStartOffset) * (CS + 1) + len,
          isCustom: false,
          bidirectional: true,
          isH,
          chunkA: cA,
          chunkB: cB,
          border,
          start,
          end: start + len - 1,
          cellsA,
          cellsB,
          centerA: cellsA[mid],
          centerB: cellsB[mid],
          anchorX: isH ? cMid + 0.5 : border,
          anchorY: isH ? border : cMid + 0.5
        });
      }
      run = -1;
    }
  }
  return out;
}

/**
 * A3, P2 & D1: Detects chunk boundary portals (incrementally reusing clean borders from `prevBorderPortals`
 * when `dirtyChunks` is supplied) and appends valid custom jump/teleporter portals (D1).
 */
export function detectPortals(grid, minClearance = 1, maxPortalWidth = 8, profileId = 0, prevBorderPortals = null, dirtyChunks = null) {
  const portals = [], { cx, cy } = grid;
  const maxW = Math.max(2, maxPortalWidth | 0);
  const vBorders = cy * (cx - 1);
  const hBorders = (cy - 1) * cx;
  const borderPortals = new Array(vBorders + hBorders);

  for (const isH of [false, true]) {
    const countX = cx - !isH, countY = cy - isH, bOffset = isH ? vBorders : 0;
    for (let y = 0; y < countY; y++) {
      for (let x = 0; x < countX; x++) {
        const cA = y * cx + x, cB = cA + (isH ? cx : 1), bIdx = bOffset + y * countX + x;
        let list;
        if (prevBorderPortals && dirtyChunks && !dirtyChunks.has(cA) && !dirtyChunks.has(cB)) {
          list = prevBorderPortals[bIdx];
        } else {
          list = detectBorderPortals(grid, isH, x, y, bIdx, minClearance, maxW, profileId);
        }
        borderPortals[bIdx] = list;
        for (let i = 0; i < list.length; i++) {
          const p = list[i];
          const clone = prevBorderPortals ? { ...p, id: portals.length } : p;
          clone.id = portals.length;
          portals.push(clone);
        }
      }
    }
  }

  // D1: Append off-grid custom jump/bridge/teleporter portals
  if (grid.customPortals && grid.customPortals.length > 0) {
    const W = grid.width;
    for (let i = 0; i < grid.customPortals.length; i++) {
      const cp = grid.customPortals[i];
      const ax = cp.centerA % W, ay = (cp.centerA / W) | 0;
      const bx = cp.centerB % W, by = (cp.centerB / W) | 0;
      if (grid.isWalkable(ax, ay, minClearance, profileId) && grid.isWalkable(bx, by, minClearance, profileId)) {
        portals.push({ ...cp, id: portals.length });
      }
    }
  }

  portals._borderPortals = borderPortals;
  return portals;
}

export const portalSideCells = (p, c) => c === p.chunkA ? p.cellsA : p.cellsB;
export const portalCenterCell = (p, c) => c === p.chunkA ? p.centerA : p.centerB;
export const portalExitX = (p, c) => p.isCustom ? (c === p.chunkA ? p.exitAX : p.exitBX) : (p.isH ? 0 : c === p.chunkA ? 1 : -1);
export const portalExitY = (p, c) => p.isCustom ? (c === p.chunkA ? p.exitAY : p.exitBY) : (p.isH ? c === p.chunkA ? 1 : -1 : 0);
export const crossingCost = (grid, p, profileId = 0) => p.customCost != null
  ? p.customCost
  : ((grid.getEffectiveBaseCost(p.centerA, profileId) + grid.getEffectiveBaseCost(p.centerB, profileId)) * 0.5) / COST_OPEN;
export const pLocal = (grid, p, c) => grid.localIndex(portalCenterCell(p, c));

/**
 * Q3: Sub-Cell Fractional Goal Seeder.
 * Seeds up to the 2x2 walkable local cell neighborhood around continuous world position `(wx, wy)`
 * with exact Euclidean sub-cell distance offsets so goal fields have zero grid-snapping bias.
 */
export function buildFractionalSources(grid, chunkId, wx, wy, baseWeight = 0, id = 0, minClearance = 1, profileId = 0) {
  const ox = grid.chunkOriginX(chunkId), oy = grid.chunkOriginY(chunkId);
  const lx = clamp(wx - ox, 0.001, CS - 0.001), ly = clamp(wy - oy, 0.001, CS - 0.001);
  const pcx = clamp(Math.floor(lx), 0, CS - 1), pcy = clamp(Math.floor(ly), 0, CS - 1);
  const primaryLoc = pcy * CS + pcx;
  const gPrimary = (oy + pcy) * grid.width + (ox + pcx);
  const refCost = Math.max(0.25, grid.getEffectiveBaseCost(gPrimary, profileId)) * 0.5;

  // If virtually exact cell center, seed single cell directly
  if (Math.abs(lx - (pcx + 0.5)) < 1e-3 && Math.abs(ly - (pcy + 0.5)) < 1e-3) {
    return [{ cell: primaryLoc, weight: baseWeight, id }];
  }

  const x0 = clamp(Math.floor(lx - 0.5), 0, CS - 1), x1 = clamp(x0 + 1, 0, CS - 1);
  const y0 = clamp(Math.floor(ly - 0.5), 0, CS - 1), y1 = clamp(y0 + 1, 0, CS - 1);
  const candidates = [];
  let minD = Infinity;

  for (let cy = y0; cy <= y1; cy++) {
    for (let cx = x0; cx <= x1; cx++) {
      const gx = ox + cx, gy = oy + cy;
      if (!grid.isWalkable(gx, gy, minClearance, profileId)) continue;
      // Prevent diagonal seeding across a pinched corner obstacle
      if (cx !== pcx && cy !== pcy) {
        if (!grid.isWalkable(ox + pcx, gy, minClearance, profileId) ||
            !grid.isWalkable(gx, oy + pcy, minClearance, profileId)) continue;
      }
      const d = Math.hypot(cx + 0.5 - lx, cy + 0.5 - ly) * refCost;
      if (d < minD) minD = d;
      candidates.push({ cell: cy * CS + cx, weight: d, id });
    }
  }

  if (candidates.length === 0) {
    return [{ cell: primaryLoc, weight: baseWeight, id }];
  }
  for (let i = 0; i < candidates.length; i++) {
    candidates[i].weight = baseWeight + (candidates[i].weight - minD);
  }
  return candidates;
}

/**
 * Multi-mode chunk integrator supporting:
 * - #1 & A4: Generalized Anisotropic Upwind Eikonal quadratic solver
 * - #2: Multi-source geodesic Voronoi (`sourceOut`)
 * - #4 & P3: Exact Wall SDF penalty buffer
 * - #5 & A3: Clearance & Multi-Modal Cost Profile filtering (`minClearance`, `profileId`)
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
  prepareChunk(grid, chunkId, minClearance = 1, profileId = 0) {
    const { costs, blocked, dirMasks, curX, curY } = this;
    const ox = grid.chunkOriginX(chunkId), oy = grid.chunkOriginY(chunkId), W = grid.width;
    const clrMap = minClearance > 1 ? grid.getClearanceMap(profileId) : null;
    const useWall = grid.wallPenaltyWeight > 0;
    if (useWall && (grid._sdfDirty || !grid.wallDist)) grid.updateWallSDF();

    const hasTac = grid.hasTactical && grid.tacticalCosts;
    const hasCong = grid.hasCongestion && grid.congestion;
    const hasCongVel = hasCong && grid.hasCongestionVelocity && grid.congestionVX;
    const hasAniso = grid.hasAnisotropy && grid.dirMask;
    const wallR = grid.wallPenaltyRadius, wallW = grid.wallPenaltyWeight;
    const congAniso = grid.congestionAnisotropy;
    const profileTable = profileId !== 0 && grid.terrainTags ? grid.costProfiles.get(profileId) : null;
    const tags = grid.terrainTags;
    let anyAniso = hasAniso;

    for (let ly = 0; ly < CS; ly++) {
      const gy = oy + ly, row = gy * W + ox, lRow = ly * CS;
      for (let lx = 0; lx < CS; lx++) {
        const gIdx = row + lx, lIdx = lRow + lx;
        let baseC = grid.costs[gIdx];
        if (profileTable) {
          const mapped = profileTable[tags[gIdx]];
          if (mapped > 0) baseC = mapped;
        }
        if (baseC >= COST_BLOCK || (clrMap && clrMap[gIdx] < minClearance)) {
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
   * @param {Object} [opts={}] - `{ minClearance, profileId, sourceOut, sourceWeights, eikonal, forward }`
   */
  integrate(grid, chunkId, sources, out, opts = {}) {
    const minClearance = opts.minClearance ?? 1;
    const profileId = opts.profileId ?? 0;
    const useEikonal = opts.eikonal ?? grid.useEikonal;
    const forward = !!opts.forward;
    const sourceOut = opts.sourceOut || null;
    const sourceWeights = opts.sourceWeights || null;

    this.prepareChunk(grid, chunkId, minClearance, profileId);
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
 * Q1: Clearance-Inset 2D Simple Stupid Funnel Algorithm (SSFA) across portal openings.
 * Computes optimal corner-hugging target anchors `(targetX, targetY)` and portal crossing points
 * `(crossingX, crossingY)` for every portal along a macro path.
 */
export function computeFunnelTargets(grid, graph, portals, chunks, sx, sy, fgx, fgy, minClearance = 1, profileId = 0) {
  const K = portals.length;
  const targetX = new Float32Array(K);
  const targetY = new Float32Array(K);
  const crossingX = new Float32Array(K);
  const crossingY = new Float32Array(K);
  if (K === 0) return { targetX, targetY, crossingX, crossingY };

  const leftX = new Float64Array(K + 1), leftY = new Float64Array(K + 1);
  const rightX = new Float64Array(K + 1), rightY = new Float64Array(K + 1);
  const pMinU = new Float64Array(K), pMaxU = new Float64Array(K);

  for (let i = 0; i < K; i++) {
    const p = graph.portals[portals[i]];
    const cFrom = chunks[i];
    if (p.isCustom) {
      const ax = cFrom === p.chunkA ? p.anchorX : p.anchorBX;
      const ay = cFrom === p.chunkA ? p.anchorY : p.anchorBY;
      leftX[i] = rightX[i] = crossingX[i] = ax;
      leftY[i] = rightY[i] = crossingY[i] = ay;
      pMinU[i] = pMaxU[i] = p.isH ? ax : ay;
      continue;
    }
    const span = p.end + 1 - p.start;
    const inset = Math.min(span * 0.45, Math.max(0.5, minClearance * 0.5));
    const u0 = p.start + inset, u1 = (p.end + 1) - inset;
    pMinU[i] = u0;
    pMaxU[i] = u1;
    const forward = cFrom === p.chunkA;
    if (p.isH) {
      // Horizontal border (normal is +Y when chunkA -> chunkB): left is +X (u1), right is -X (u0)
      leftX[i] = forward ? u1 : u0;
      leftY[i] = p.border;
      rightX[i] = forward ? u0 : u1;
      rightY[i] = p.border;
    } else {
      // Vertical border (normal is +X when chunkA -> chunkB): left is -Y (u0), right is +Y (u1)
      leftX[i] = p.border;
      leftY[i] = forward ? u0 : u1;
      rightX[i] = p.border;
      rightY[i] = forward ? u1 : u0;
    }
  }
  leftX[K] = rightX[K] = fgx;
  leftY[K] = rightY[K] = fgy;

  const triArea2 = (ax, ay, bx, by, cx, cy) => (bx - ax) * (cy - ay) - (cx - ax) * (by - ay);

  // Extract funnel corners per segments separated by custom jump portals
  const corners = [{ x: sx, y: sy, step: -1 }];
  let apexX = sx, apexY = sy, apexIdx = 0;
  let lX = sx, lY = sy, lIdx = 0;
  let rX = sx, rY = sy, rIdx = 0;

  for (let i = 0; i <= K; i++) {
    if (i < K && graph.portals[portals[i]].isCustom) {
      const p = graph.portals[portals[i]];
      const cFrom = chunks[i];
      const entX = cFrom === p.chunkA ? p.anchorX : p.anchorBX;
      const entY = cFrom === p.chunkA ? p.anchorY : p.anchorBY;
      const extX = cFrom === p.chunkA ? p.anchorBX : p.anchorX;
      const extY = cFrom === p.chunkA ? p.anchorBY : p.anchorY;
      corners.push({ x: entX, y: entY, step: i });
      corners.push({ x: extX, y: extY, step: i });
      apexX = lX = rX = extX;
      apexY = lY = rY = extY;
      apexIdx = lIdx = rIdx = i;
      continue;
    }

    const nlX = leftX[i], nlY = leftY[i];
    const nrX = rightX[i], nrY = rightY[i];

    // Update right vertex
    if (triArea2(apexX, apexY, rX, rY, nrX, nrY) <= 0.0) {
      if ((apexX === rX && apexY === rY) || triArea2(apexX, apexY, lX, lY, nrX, nrY) > 0.0) {
        rX = nrX; rY = nrY; rIdx = i;
      } else {
        apexX = lX; apexY = lY; apexIdx = lIdx;
        corners.push({ x: apexX, y: apexY, step: apexIdx });
        lX = rX = apexX; lY = rY = apexY;
        lIdx = rIdx = apexIdx;
        i = apexIdx;
        continue;
      }
    }

    // Update left vertex
    if (triArea2(apexX, apexY, lX, lY, nlX, nlY) >= 0.0) {
      if ((apexX === lX && apexY === lY) || triArea2(apexX, apexY, rX, rY, nlX, nlY) < 0.0) {
        lX = nlX; lY = nlY; lIdx = i;
      } else {
        apexX = rX; apexY = rY; apexIdx = rIdx;
        corners.push({ x: apexX, y: apexY, step: apexIdx });
        lX = rX = apexX; lY = rY = apexY;
        lIdx = rIdx = apexIdx;
        i = apexIdx;
        continue;
      }
    }
  }
  corners.push({ x: fgx, y: fgy, step: K });

  // Intersect funnel segments with each portal to determine crossingX/Y and downstream targetX/Y
  let cPtr = 0;
  for (let i = 0; i < K; i++) {
    const p = graph.portals[portals[i]];
    while (cPtr + 1 < corners.length - 1 && corners[cPtr + 1].step <= i) cPtr++;
    const prevC = corners[cPtr];
    const nextC = corners[Math.min(corners.length - 1, cPtr + 1)];

    if (p.isCustom) {
      targetX[i] = crossingX[i];
      targetY[i] = crossingY[i];
      continue;
    }

    // Compute intersection of segment (prevC -> nextC) with portal border line
    if (p.isH) {
      const dy = nextC.y - prevC.y;
      const t = Math.abs(dy) > 1e-6 ? clamp((p.border - prevC.y) / dy, 0, 1) : 0.5;
      const ix = clamp(prevC.x + (nextC.x - prevC.x) * t, pMinU[i], pMaxU[i]);
      crossingX[i] = ix;
      crossingY[i] = p.border;
    } else {
      const dx = nextC.x - prevC.x;
      const t = Math.abs(dx) > 1e-6 ? clamp((p.border - prevC.x) / dx, 0, 1) : 0.5;
      const iy = clamp(prevC.y + (nextC.y - prevC.y) * t, pMinU[i], pMaxU[i]);
      crossingX[i] = p.border;
      crossingY[i] = iy;
    }

    targetX[i] = nextC.x;
    targetY[i] = nextC.y;
  }

  // Verify/refine with world LOS when non-uniform costs or tactical hazards exist
  if (grid.losSmoothing) {
    const goalCell = clamp(fgy | 0, 0, grid.height - 1) * grid.width + clamp(fgx | 0, 0, grid.width - 1);
    const refCost = Math.max(COST_OPEN, grid.getEffectiveBaseCost(goalCell, profileId));
    for (let i = 0; i < K; i++) {
      const p = graph.portals[portals[i]];
      if (p.isCustom) continue;
      const cx = crossingX[i], cy = crossingY[i];
      if (!grid.hasWorldLOS(cx, cy, targetX[i], targetY[i], refCost, minClearance, profileId)) {
        targetX[i] = i + 1 < K ? crossingX[i + 1] : fgx;
        targetY[i] = i + 1 < K ? crossingY[i + 1] : fgy;
      }
    }
  }

  return { targetX, targetY, crossingX, crossingY };
}

/**
 * High-level abstract graph connecting chunk boundary portals and custom jump portals (D1) in forward & reverse CSR format.
 * Supports chunk-local incremental portal detection (P2), in-place weight updates (B1), wide-portal splitting (A3),
 * size-aware clearance (#5), multi-modal terrain profiles (A3), anisotropic edges (#11), and reverse Dijkstra trees (C1/C2).
 */
export class PortalGraph {
  constructor(grid, prevGraph = null, dirtyChunks = null, minClearance = 1, maxPortalWidth = 8, profileId = 0) {
    this.grid = grid;
    this.minClearance = minClearance;
    this.maxPortalWidth = maxPortalWidth;
    this.profileId = profileId;

    const canReuse = !!(
      prevGraph &&
      prevGraph.minClearance === minClearance &&
      prevGraph.maxPortalWidth === maxPortalWidth &&
      prevGraph.profileId === profileId &&
      dirtyChunks
    );

    const portals = this.portals = detectPortals(
      grid,
      minClearance,
      maxPortalWidth,
      profileId,
      canReuse ? prevGraph.borderPortals : null,
      canReuse ? dirtyChunks : null
    );
    this.borderPortals = portals._borderPortals;

    const P = portals.length;
    this.uidToId = new Map();
    const lists = Array.from({ length: grid.chunkCount }, () => []);
    this.customPortalIds = [];

    for (let i = 0; i < P; i++) {
      const p = portals[i];
      this.uidToId.set(p.uid, p.id);
      lists[p.chunkA].push(p.id);
      if (p.chunkB !== p.chunkA) lists[p.chunkB].push(p.id);
      if (p.isCustom) this.customPortalIds.push(p.id);
    }
    this.chunkPortals = lists.map(l => new Int32Array(l));
    this.chunkDistCache = Array.from({ length: grid.chunkCount }, (_, c) =>
      canReuse && !dirtyChunks.has(c) ? prevGraph.chunkDistCache[c] : new Map()
    );
    this._it = new ChunkIntegrator();
    this._field = new Float64Array(CC);
    this._buildCSR();
  }

  _buildCSR() {
    const { grid, portals, chunkPortals, minClearance, profileId, _it: it, _field: field } = this;
    const P = portals.length;
    const adj = Array.from({ length: P }, () => []);
    const revAdj = Array.from({ length: P }, () => []);
    const itOpts = { minClearance, profileId };

    for (let c = 0; c < grid.chunkCount; c++) {
      const ids = chunkPortals[c];
      if (ids.length < 2) continue;
      const cCache = this.chunkDistCache[c];
      for (let ia = 0; ia < ids.length; ia++) {
        const idA = ids[ia], pA = portals[idA];
        // D1: If pA is a one-way custom portal (chunkA -> chunkB), it can only be entered in chunkA
        if (pA.isCustom && !pA.bidirectional && c !== pA.chunkA) continue;

        const locA = pLocal(grid, pA, c);
        let row = cCache.get(locA);
        if (!row) {
          it.integrate(grid, c, [locA], field, itOpts);
          row = new Float64Array(field);
          cCache.set(locA, row);
        }
        const crossA = crossingCost(grid, pA, profileId);
        for (let ib = 0; ib < ids.length; ib++) {
          const idB = ids[ib];
          if (idA === idB) continue;
          const pB = portals[idB];
          // D1: If pB is a one-way custom portal (chunkA -> chunkB), it can only be exited into chunkB
          if (pB.isCustom && !pB.bidirectional && c !== pB.chunkB) continue;

          const d = row[pLocal(grid, pB, c)];
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
    const { grid, portals, chunkPortals, minClearance, profileId, _it: it, _field: field } = this;
    const itOpts = { minClearance, profileId };
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
          this.edgeCosts[e] = d < Infinity ? d + crossingCost(grid, portals[pA], profileId) : Infinity;
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
          this.revEdgeCosts[e] = d < Infinity ? d + crossingCost(grid, portals[pA], profileId) : Infinity;
        }
      }
    }
  }
}

/**
 * Hierarchical A* & Global Reverse-Dijkstra pathfinder operating over `PortalGraph`.
 * Supports single goals with sub-cell fractional seeding (Q3), Multi-Source Geodesic Voronoi goals (#2),
 * All-to-One Global Swarm Trees (C1), Moving-Target Warm-Start Pursuit (D2),
 * Macro Multi-Chunk Flee & Evasion (C2), Clearance-Inset 2D Portal Funnel String-Pulling (Q1),
 * Off-Grid Jump Links (D1), and Lazy Path Splicing & Migration (D3).
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
    for (const [key, g] of this.goalCache) {
      if (dirtyChunks.has(g.chunk)) this.goalCache.delete(key);
    }
  }

  /**
   * Q3: Creates a single-goal descriptor supporting exact continuous sub-cell coordinates `(exactX, exactY)`.
   */
  createGoal(cell, exactX = NaN, exactY = NaN) {
    const { grid, graph } = this;
    const x = cell % grid.width, y = (cell / grid.width) | 0;
    const gx = Number.isNaN(exactX) ? x + 0.5 : exactX;
    const gy = Number.isNaN(exactY) ? y + 0.5 : exactY;
    const chunk = grid.chunkIdAt(x, y), field = new Float64Array(CC);
    const seeds = buildFractionalSources(grid, chunk, gx, gy, 0, cell, graph.minClearance, graph.profileId);
    this.it.integrate(grid, chunk, seeds, field, {
      minClearance: graph.minClearance,
      profileId: graph.profileId
    });
    const portalCost = new Float64Array(graph.portals.length).fill(Infinity);
    const cPortals = graph.chunkPortals[chunk];
    for (let i = 0; i < cPortals.length; i++) {
      const pid = cPortals[i], p = graph.portals[pid];
      if (p.isCustom && !p.bidirectional && chunk !== p.chunkB) continue;
      const d = field[pLocal(grid, p, chunk)];
      if (d < Infinity) portalCost[pid] = d;
    }
    return {
      isMulti: false,
      cell,
      x,
      y,
      goalX: gx,
      goalY: gy,
      chunk,
      field,
      portalCost,
      minClearance: graph.minClearance,
      profileId: graph.profileId,
      graph
    };
  }

  /**
   * #2 & Q3: Creates a Multi-Source Goal descriptor from an array of target cells or `{ cell, x, y, weight }` objects.
   */
  createMultiGoal(targets) {
    const { grid, graph } = this;
    const portalCost = new Float64Array(graph.portals.length).fill(Infinity);
    const portalGoalCell = new Int32Array(graph.portals.length).fill(-1);
    const chunkGroups = new Map();
    const goalList = [];

    for (let i = 0; i < targets.length; i++) {
      const t = targets[i];
      let cell, gx, gy, weight = 0;
      if (typeof t === 'object') {
        weight = t.weight ?? 0;
        if (t.x !== undefined && t.y !== undefined) {
          gx = clamp(t.x, 0.001, grid.width - 0.001);
          gy = clamp(t.y, 0.001, grid.height - 0.001);
          cell = t.cell ?? grid.index(gx | 0, gy | 0);
        } else {
          cell = t.cell;
          gx = (cell % grid.width) + 0.5;
          gy = ((cell / grid.width) | 0) + 0.5;
        }
      } else {
        cell = t;
        gx = (cell % grid.width) + 0.5;
        gy = ((cell / grid.width) | 0) + 0.5;
      }
      if (cell < 0 || cell >= grid.width * grid.height) continue;
      const x = cell % grid.width, y = (cell / grid.width) | 0, chunk = grid.chunkIdAt(x, y);
      goalList.push({ cell, x, y, goalX: gx, goalY: gy, chunk, weight });
      if (!chunkGroups.has(chunk)) chunkGroups.set(chunk, []);
      const group = chunkGroups.get(chunk);
      const fracSeeds = buildFractionalSources(grid, chunk, gx, gy, weight, cell, graph.minClearance, graph.profileId);
      for (let k = 0; k < fracSeeds.length; k++) {
        group.push({
          cell: fracSeeds[k].cell,
          weight: fracSeeds[k].weight,
          globalCell: cell,
          goalX: gx,
          goalY: gy
        });
      }
    }

    const chunkFields = new Map();
    for (const [chunk, sources] of chunkGroups) {
      const field = new Float64Array(CC);
      const sourceOut = new Int32Array(CC);
      const seeded = sources.map(s => ({ cell: s.cell, weight: s.weight, id: s.globalCell }));
      this.it.integrate(grid, chunk, seeded, field, {
        minClearance: graph.minClearance,
        profileId: graph.profileId,
        sourceOut
      });
      chunkFields.set(chunk, { field, sourceOut, sources });
      const cPortals = graph.chunkPortals[chunk];
      for (let i = 0; i < cPortals.length; i++) {
        const pid = cPortals[i], p = graph.portals[pid];
        if (p.isCustom && !p.bidirectional && chunk !== p.chunkB) continue;
        const loc = pLocal(grid, p, chunk);
        const d = field[loc];
        if (d < portalCost[pid]) {
          portalCost[pid] = d;
          portalGoalCell[pid] = sourceOut[loc];
        }
      }
    }

    const primary = goalList[0] || { cell: 0, x: 0, y: 0, goalX: 0.5, goalY: 0.5, chunk: 0 };
    return {
      isMulti: true,
      cell: primary.cell,
      x: primary.x,
      y: primary.y,
      goalX: primary.goalX,
      goalY: primary.goalY,
      chunk: primary.chunk,
      goals: goalList,
      chunkFields,
      portalCost,
      portalGoalCell,
      minClearance: graph.minClearance,
      profileId: graph.profileId,
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
        const w = gc + crossingCost(grid, graph.portals[p], graph.profileId);
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
      goalX: multi.goalX,
      goalY: multi.goalY,
      goals: multi.goals,
      chunkFields: multi.chunkFields,
      portalCost: multi.portalCost,
      portalDist,
      portalNext,
      portalGoalCell,
      minClearance: graph.minClearance,
      profileId: graph.profileId,
      graph,
      _lastChunk: -1,
      _lastField: null
    };
  }

  /**
   * D2: Warm-Start Pursuit & Velocity-Lead Update for an existing `GlobalGoal`.
   * Re-integrates only the target chunk(s) and skips the global Reverse Dijkstra pass
   * when normalized boundary portal distances shift by less than `portalTolerance`.
   * Returns the set of chunk IDs whose flow fields need invalidation.
   */
  updateGlobalGoal(globalGoal, newTargets, opts = {}) {
    const { grid, graph, heap } = this;
    const portalTol = opts.portalTolerance ?? 0.35;
    const prevChunks = new Set(globalGoal.chunkFields ? globalGoal.chunkFields.keys() : []);
    const multi = this.createMultiGoal(Array.isArray(newTargets) ? newTargets : [newTargets]);
    const P = graph.portals.length;

    let sameChunks = prevChunks.size === multi.chunkFields.size;
    if (sameChunks) {
      for (const c of multi.chunkFields.keys()) {
        if (!prevChunks.has(c)) { sameChunks = false; break; }
      }
    }

    const dirtyChunks = new Set(prevChunks);
    for (const c of multi.chunkFields.keys()) dirtyChunks.add(c);

    // Check if relative portal distances on the goal chunk boundary stayed within tolerance
    if (sameChunks && globalGoal.portalCost && globalGoal.portalCost.length === P) {
      let minDiff = Infinity, maxDiff = -Infinity, count = 0;
      for (const c of multi.chunkFields.keys()) {
        const cPortals = graph.chunkPortals[c];
        for (let i = 0; i < cPortals.length; i++) {
          const pid = cPortals[i];
          const oldC = globalGoal.portalCost[pid], newC = multi.portalCost[pid];
          if ((oldC === Infinity) !== (newC === Infinity)) { minDiff = 0; maxDiff = Infinity; break; }
          if (newC < Infinity) {
            const diff = newC - oldC;
            if (diff < minDiff) minDiff = diff;
            if (diff > maxDiff) maxDiff = diff;
            count++;
          }
        }
      }
      if (count === 0 || (maxDiff - minDiff) <= portalTol) {
        globalGoal.cell = multi.cell;
        globalGoal.x = multi.x;
        globalGoal.y = multi.y;
        globalGoal.goalX = multi.goalX;
        globalGoal.goalY = multi.goalY;
        globalGoal.goals = multi.goals;
        globalGoal.chunkFields = multi.chunkFields;
        globalGoal.portalCost = multi.portalCost;
        globalGoal._lastChunk = -1;
        globalGoal._lastField = null;
        return { dirtyChunks, skippedMacro: true };
      }
    }

    // Re-run Reverse Dijkstra reusing existing TypedArrays and track chunks whose portal routing changed
    const { portalDist, portalNext, portalGoalCell } = globalGoal;
    const prevNext = new Int32Array(portalNext);
    portalDist.fill(Infinity);
    portalNext.fill(-1);
    portalGoalCell.fill(-1);

    heap.clear();
    for (let p = 0; p < P; p++) {
      const gc = multi.portalCost[p];
      if (gc < Infinity) {
        const w = gc + crossingCost(grid, graph.portals[p], graph.profileId);
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

    for (let p = 0; p < P; p++) {
      if (portalNext[p] !== prevNext[p]) {
        const pt = graph.portals[p];
        dirtyChunks.add(pt.chunkA);
        dirtyChunks.add(pt.chunkB);
      }
    }

    globalGoal.cell = multi.cell;
    globalGoal.x = multi.x;
    globalGoal.y = multi.y;
    globalGoal.goalX = multi.goalX;
    globalGoal.goalY = multi.goalY;
    globalGoal.goals = multi.goals;
    globalGoal.chunkFields = multi.chunkFields;
    globalGoal.portalCost = multi.portalCost;
    globalGoal._lastChunk = -1;
    globalGoal._lastField = null;
    return { dirtyChunks, skippedMacro: false };
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
      const cell = grid.isWalkable(tx, ty, graph.minClearance, graph.profileId)
        ? grid.index(tx, ty)
        : grid.findNearestWalkable(tx, ty, 16, graph.minClearance, graph.profileId);
      if (cell < 0) continue;
      const chunk = grid.chunkIdAt(cell % grid.width, (cell / grid.width) | 0);
      if (!threatChunkGroups.has(chunk)) threatChunkGroups.set(chunk, []);
      threatChunkGroups.get(chunk).push(grid.localIndex(cell));
    }

    heap.clear();
    const field = new Float64Array(CC);
    for (const [chunk, localCells] of threatChunkGroups) {
      this.it.integrate(grid, chunk, localCells, field, {
        minClearance: graph.minClearance,
        profileId: graph.profileId,
        forward: true
      });
      const cPortals = graph.chunkPortals[chunk];
      for (let i = 0; i < cPortals.length; i++) {
        const pid = cPortals[i], p = graph.portals[pid];
        const d = field[pLocal(grid, p, chunk)];
        if (d < Infinity) {
          const w = d + crossingCost(grid, p, graph.profileId);
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
        this.it.integrate(grid, chunk, localCells, field, {
          minClearance: graph.minClearance,
          profileId: graph.profileId
        });
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

  getOrCreateGoal(cell, exactX = NaN, exactY = NaN) {
    // Q3: Quantize sub-cell offset to 1/8th cell for cache reuse
    const qx = Number.isNaN(exactX) ? 4 : clamp(Math.round((exactX - Math.floor(exactX)) * 8), 0, 8);
    const qy = Number.isNaN(exactY) ? 4 : clamp(Math.round((exactY - Math.floor(exactY)) * 8), 0, 8);
    const key = cell * 81 + qy * 9 + qx;
    let g = this.goalCache.get(key);
    if (g) return g;
    g = this.createGoal(cell, exactX, exactY);
    if (this.goalCache.size >= this.goalCacheCap) {
      this.goalCache.delete(this.goalCache.keys().next().value);
    }
    this.goalCache.set(key, g);
    return g;
  }

  findPath(startCell, goalOrCell, startExactX = NaN, startExactY = NaN, goalExactX = NaN, goalExactY = NaN) {
    const goal = typeof goalOrCell === 'number'
      ? this.getOrCreateGoal(goalOrCell, goalExactX, goalExactY)
      : (Array.isArray(goalOrCell) ? this.createMultiGoal(goalOrCell) : goalOrCell);
    if (!goal) return null;
    const { grid, graph, g, parent, parentChunk, seen, closed, heap } = this;
    const P = graph.portals.length, START = P, GOAL = P + 1;
    if (++this.stamp >= 0xffffffff) { seen.fill(0); closed.fill(0); this.stamp = 1; }
    const s = this.stamp;
    const sx = startCell % grid.width, sy = (startCell / grid.width) | 0, sc = grid.chunkIdAt(sx, sy);
    const startWX = Number.isNaN(startExactX) ? sx + 0.5 : startExactX;
    const startWY = Number.isNaN(startExactY) ? sy + 0.5 : startExactY;
    const gx = goal.goalX ?? (goal.x + 0.5), gy = goal.goalY ?? (goal.y + 0.5);

    const startSeeds = buildFractionalSources(grid, sc, startWX, startWY, 0, startCell, graph.minClearance, graph.profileId);
    this.it.integrate(grid, sc, startSeeds, this.startField, {
      minClearance: graph.minClearance,
      profileId: graph.profileId,
      forward: true
    });

    const hasCustomLinks = graph.customPortalIds.length > 0;
    const baseHeuristic = (px, py) => {
      if (!goal.isMulti) return Math.max(0, octile(px - gx, py - gy) - 0.5);
      let best = Infinity;
      for (let i = 0; i < goal.goals.length; i++) {
        const tg = goal.goals[i];
        const tgx = tg.goalX ?? (tg.x + 0.5), tgy = tg.goalY ?? (tg.y + 0.5);
        const d = Math.max(0, octile(px - tgx, py - tgy) - 0.5) + tg.weight;
        if (d < best) best = d;
      }
      return best;
    };

    // D1: Admissible teleporter-aware heuristic when custom off-grid jump portals exist
    const heuristic = (px, py) => {
      let h = baseHeuristic(px, py);
      if (hasCustomLinks) {
        for (let i = 0; i < graph.customPortalIds.length; i++) {
          const cp = graph.portals[graph.customPortalIds[i]];
          const cCost = crossingCost(grid, cp, graph.profileId);
          const dAB = Math.max(0, octile(px - cp.anchorX, py - cp.anchorY) - 0.5) + cCost + baseHeuristic(cp.anchorBX, cp.anchorBY);
          if (dAB < h) h = dAB;
          if (cp.bidirectional) {
            const dBA = Math.max(0, octile(px - cp.anchorBX, py - cp.anchorBY) - 0.5) + cCost + baseHeuristic(cp.anchorX, cp.anchorY);
            if (dBA < h) h = dBA;
          }
        }
      }
      return h;
    };

    heap.clear(); g[START] = 0; seen[START] = s; parent[START] = -1; parentChunk[START] = sc;
    heap.push(START, heuristic(startWX, startWY));

    let chosenGoalCell = goal.cell;
    let chosenGoalX = gx, chosenGoalY = gy;
    const relax = (v, cost, from, chunk, resolvedCell = -1) => {
      if (closed[v] === s || (seen[v] === s && cost >= g[v])) return;
      seen[v] = s; g[v] = cost; parent[v] = from; parentChunk[v] = chunk;
      if (v === GOAL && resolvedCell >= 0) chosenGoalCell = resolvedCell;
      let h = 0;
      if (v < P) {
        const pv = graph.portals[v];
        const ax = pv.isCustom && chunk === pv.chunkA ? pv.anchorBX : pv.anchorX;
        const ay = pv.isCustom && chunk === pv.chunkA ? pv.anchorBY : pv.anchorY;
        h = heuristic(ax, ay);
      }
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

        if (!goal.isMulti) {
          chosenGoalX = gx;
          chosenGoalY = gy;
        } else {
          chosenGoalX = (chosenGoalCell % grid.width) + 0.5;
          chosenGoalY = ((chosenGoalCell / grid.width) | 0) + 0.5;
          for (let i = 0; i < goal.goals.length; i++) {
            if (goal.goals[i].cell === chosenGoalCell) {
              chosenGoalX = goal.goals[i].goalX;
              chosenGoalY = goal.goals[i].goalY;
              break;
            }
          }
        }

        const portalArr = new Int32Array(portals);
        const chunkArr = new Int32Array(chunks);

        // D3: Snapshot portal UIDs and chunk topology versions for lazy path migration & splicing
        const portalUids = new Int32Array(portalArr.length);
        for (let i = 0; i < portalArr.length; i++) portalUids[i] = graph.portals[portalArr[i]].uid;
        const chunkVersions = new Uint32Array(chunkArr.length);
        for (let i = 0; i < chunkArr.length; i++) chunkVersions[i] = grid.chunkTopoVersion[chunkArr[i]];

        // Q1: Clearance-Inset 2D Portal Funnel (SSFA) String-Pulling
        const { targetX, targetY, crossingX, crossingY } = computeFunnelTargets(
          grid, graph, portalArr, chunkArr, startWX, startWY, chosenGoalX, chosenGoalY, graph.minClearance, graph.profileId
        );

        return {
          portals: portalArr,
          portalUids,
          chunks: chunkArr,
          chunkVersions,
          targetX,
          targetY,
          crossingX,
          crossingY,
          cost: g[GOAL],
          goalCell: chosenGoalCell,
          goalX: chosenGoalX,
          goalY: chosenGoalY,
          minClearance: graph.minClearance,
          profileId: graph.profileId,
          graph
        };
      }
      const gu = g[u];
      if (u === START) {
        const cPorts = graph.chunkPortals[sc];
        for (let i = 0; i < cPorts.length; i++) {
          const pid = cPorts[i], p = graph.portals[pid];
          if (p.isCustom && !p.bidirectional && sc !== p.chunkA) continue;
          const d = this.startField[pLocal(grid, p, sc)];
          if (d < Infinity) relax(pid, gu + d + crossingCost(grid, p, graph.profileId), START, sc);
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
   * D3: Transparently migrates an existing `path` to the current `graph` when unaffected by topology edits,
   * or splices a fresh repair path from `(wx, wy)` to `(path.goalX, path.goalY)` in-place.
   */
  repairOrMigratePath(path, wx, wy, hintIdx = 0) {
    const { grid, graph } = this;
    if (!path) return false;
    if (path.graph === graph) return true;

    const cx = clamp(wx | 0, 0, grid.width - 1), cy = clamp(wy | 0, 0, grid.height - 1);
    const curChunk = grid.chunkIdAt(cx, cy);
    let startIdx = hintIdx;
    if (path.chunks[startIdx] !== curChunk) {
      startIdx = path.chunks.indexOf(curChunk, hintIdx);
      if (startIdx < 0) startIdx = path.chunks.lastIndexOf(curChunk, hintIdx);
      if (startIdx < 0) startIdx = 0;
    }

    // 1. Check if all remaining chunks & portals from startIdx onward are untouched
    let canMigrate = !!(path.chunkVersions && path.portalUids);
    if (canMigrate) {
      for (let i = startIdx; i < path.chunks.length; i++) {
        const c = path.chunks[i];
        if (grid.chunkTopoVersion[c] !== path.chunkVersions[i]) {
          canMigrate = false;
          break;
        }
      }
      if (canMigrate) {
        for (let i = startIdx; i < path.portals.length; i++) {
          if (!graph.uidToId.has(path.portalUids[i])) {
            canMigrate = false;
            break;
          }
        }
      }
    }

    if (canMigrate) {
      for (let i = startIdx; i < path.portals.length; i++) {
        path.portals[i] = graph.uidToId.get(path.portalUids[i]);
      }
      path.graph = graph;
      return true;
    }

    // 2. Otherwise splice/repair path from current agent position (wx, wy) to goal
    const clr = path.minClearance ?? graph.minClearance;
    const prof = path.profileId ?? graph.profileId;
    const startCell = grid.isWalkable(cx, cy, clr, prof)
      ? grid.index(cx, cy)
      : grid.findNearestWalkable(cx, cy, 16, clr, prof);
    const gcx = clamp(path.goalX | 0, 0, grid.width - 1), gcy = clamp(path.goalY | 0, 0, grid.height - 1);
    const goalCell = grid.isWalkable(gcx, gcy, clr, prof)
      ? grid.index(gcx, gcy)
      : grid.findNearestWalkable(gcx, gcy, 16, clr, prof);
    if (startCell < 0 || goalCell < 0) return false;

    const repaired = this.findPath(startCell, goalCell, wx, wy, path.goalX, path.goalY);
    if (!repaired) return false;

    path.portals = repaired.portals;
    path.portalUids = repaired.portalUids;
    path.chunks = repaired.chunks;
    path.chunkVersions = repaired.chunkVersions;
    path.targetX = repaired.targetX;
    path.targetY = repaired.targetY;
    path.crossingX = repaired.crossingX;
    path.crossingY = repaired.crossingY;
    path.cost = repaired.cost;
    path.goalCell = repaired.goalCell;
    path.graph = graph;
    return true;
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
 * - #3 & Q3: Sub-cell Line-of-Sight direct-to-target smoothing (`opts.targetLocalX`, `opts.targetLocalY`, `opts.chunkCosts`)
 * - #4 & P3: Exact Wall SDF repulsion blending (`opts.grid`)
 * - #7: Vortex / Orbit / Strafe curl modulation (`opts.vortex`)
 * - A1 & Q2: Stitched portal exit vectors (`opts.portalMask`)
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

      // #3 & Q3: Line-of-Sight direct-to-target vector (to exact sub-cell target coordinate)
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
  it.integrate(grid, chunkId, sources, f.integration, {
    sourceOut: f.sourceId,
    minClearance: opts.minClearance ?? 1,
    profileId: opts.profileId ?? 0
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

const KIND_ID = Object.freeze({ p: 1, g: 2, G: 3, m: 4, v: 5, f: 6, c: 7 });

/**
 * B2, A1, P1 & Q2: Unified LRU cache with zero-GC TypedArray slab pooling and
 * 53-bit homogeneous integer keys (P1) for all flow field types:
 * Portal (with A1/Q2 downstream wavefront cascading), Goal (with Q3 sub-cell seeding),
 * Global Tree (C1), Multi-Goal (#2), Vortex (#7), Flee (#8), and Corridor (#10).
 */
export class FlowFieldCache {
  constructor(grid, cap = 2048) {
    this.grid = grid; this.cap = cap; this.entries = new Map();
    this.it = new ChunkIntegrator();
    this.pool = [];
    this._portalMask = new Uint8Array(CC);
    this._nextFieldToken = 1;
  }

  /**
   * P1: Packs `(chunkId, kind, payload26, minClearance, profileId)` into a 53-bit safe integer key.
   * Zero string allocations; keeps `this.entries` a homogeneous integer-keyed Map.
   */
  key(c, kind, tid, minClearance = 1, profileId = 0) {
    const kId = typeof kind === 'number' ? kind : (KIND_ID[kind] || (kind.charCodeAt(0) & 7));
    const low27 = (((profileId & 15) * 16 + (minClearance & 15)) * 8 + kId) * 65536 + (c & 0xffff);
    return ((tid >>> 0) & 0x3ffffff) * 134217728 + low27;
  }

  _acquire(cid, kind, needSourceId = false) {
    const { grid } = this;
    const f = this.pool.pop() || {
      chunkId: cid, kind, originX: 0, originY: 0, token: 0, goalId: 0,
      integration: new Float32Array(CC), flowX: new Float32Array(CC), flowY: new Float32Array(CC),
      sourceId: null, vortex: null
    };
    f.chunkId = cid;
    f.kind = kind;
    f.originX = grid.chunkOriginX(cid);
    f.originY = grid.chunkOriginY(cid);
    f.token = (this._nextFieldToken = (this._nextFieldToken + 1) & 0x3fffff);
    f.goalId = 0;
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
   * A1 & Q2: Builds or retrieves a portal flow field with True Downstream Wavefront Cascading.
   * When `downstreamField` is supplied (Q2), boundary cells are seeded directly with the exact
   * Eikonal integration values from the opposite side of the portal border.
   */
  getPortalField(c, p, minClearance = 1, targetX = NaN, targetY = NaN, downstreamField = null, profileId = 0) {
    const anchorX = p.isCustom && c === p.chunkB ? p.anchorBX : p.anchorX;
    const anchorY = p.isCustom && c === p.chunkB ? p.anchorBY : p.anchorY;
    const hasTarget = !Number.isNaN(targetX) && !Number.isNaN(targetY);
    const angleBucket = hasTarget
      ? ((Math.round((Math.atan2(targetY - anchorY, targetX - anchorX) / (Math.PI * 2)) * 64) & 63) + 1)
      : 0;
    const downToken = downstreamField && !p.isCustom ? (downstreamField.token & 0x3ff) : 0;
    const payload = ((p.uid * 65 + angleBucket) ^ (downToken << 14)) >>> 0;
    const k = this.key(c, 1, payload, minClearance, profileId);
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
      if (downstreamField && !p.isCustom) {
        const oppLoc = grid.localIndex(oppCells[i]);
        const dVal = downstreamField.integration[oppLoc];
        if (dVal < Infinity) {
          w = dVal;
        } else if (hasTarget && pCells.length > 1) {
          const wx = (gCell % grid.width) + 0.5, wy = ((gCell / grid.width) | 0) + 0.5;
          w = octile(wx - targetX, wy - targetY) * (COST_OPEN * 0.45);
        } else {
          w = COST_OPEN * CS;
        }
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

    if (hasTarget && !p.isCustom) {
      const vx = targetX - anchorX, vy = targetY - anchorY, vl = Math.hypot(vx, vy);
      if (vl > 1e-5) {
        const tx = vx / vl, ty = vy / vl;
        if (tx * exX + ty * exY > 0.1) {
          const bx = exX * 0.4 + tx * 0.6, by = exY * 0.4 + ty * 0.6, bl = Math.hypot(bx, by);
          if (bl > 1e-5) { exX = bx / bl; exY = by / bl; }
        }
      }
    }

    return this.build(k, c, 'p', sources, exX, exY, { minClearance, profileId, portalMask: mask });
  }

  /**
   * Q3: Builds or retrieves a goal chunk flow field with sub-cell fractional Eikonal seeding.
   */
  getGoalField(c, cell, minClearance = 1, exactX = NaN, exactY = NaN, profileId = 0) {
    const { grid } = this;
    const gx = Number.isNaN(exactX) ? (cell % grid.width) + 0.5 : exactX;
    const gy = Number.isNaN(exactY) ? ((cell / grid.width) | 0) + 0.5 : exactY;
    const qx = clamp(Math.round((gx - Math.floor(gx)) * 8), 0, 8);
    const qy = clamp(Math.round((gy - Math.floor(gy)) * 8), 0, 8);
    const payload = (cell * 81 + qy * 9 + qx) >>> 0;
    const k = this.key(c, 2, payload, minClearance, profileId);
    const hit = this.entries.get(k);
    if (hit) return this._promote(k, hit);

    const ox = grid.chunkOriginX(c), oy = grid.chunkOriginY(c);
    const seeds = buildFractionalSources(grid, c, gx, gy, 0, cell, minClearance, profileId);
    return this.build(k, c, 'g', seeds, 0, 0, {
      minClearance,
      profileId,
      losSmoothing: grid.losSmoothing,
      targetLocalX: clamp(gx - ox, 0.01, CS - 0.01),
      targetLocalY: clamp(gy - oy, 0.01, CS - 0.01)
    });
  }

  getMultiGoalField(c, sources, minClearance = 1, profileId = 0) {
    let hash = 2166136261;
    for (let i = 0; i < sources.length; i++) {
      const s = sources[i];
      const cell = typeof s === 'object' ? s.cell : s;
      const w = typeof s === 'object' ? ((s.weight ?? 0) * 16 | 0) : 0;
      hash = Math.imul(hash ^ ((cell << 8) ^ w), 16777619);
    }
    const k = this.key(c, 4, hash >>> 0, minClearance, profileId);
    const hit = this.entries.get(k);
    if (hit) return this._promote(k, hit);
    return this.build(k, c, 'm', sources, 0, 0, { minClearance, profileId, trackSources: true });
  }

  getVortexField(c, centerCell, vortexOpts = {}) {
    const clr = vortexOpts.minClearance ?? 1;
    const prof = vortexOpts.profileId ?? 0;
    const rKey = ((vortexOpts.radius ?? 4) * 16) | 0;
    const sKey = ((vortexOpts.strength ?? 1) * 16) | 0;
    const cwKey = vortexOpts.clockwise !== false ? 1 : 0;
    const hash = Math.imul((centerCell << 10) ^ (rKey << 4) ^ (sKey << 1) ^ cwKey, 16777619) >>> 0;
    const k = this.key(c, 5, hash, clr, prof);
    const hit = this.entries.get(k);
    if (hit) return this._promote(k, hit);
    const loc = this.grid.localIndex(centerCell);
    return this.build(k, c, 'v', [loc], 0, 0, {
      minClearance: clr,
      profileId: prof,
      vortex: vortexOpts,
      losSmoothing: this.grid.losSmoothing,
      targetLocalX: (loc % CS) + 0.5,
      targetLocalY: ((loc / CS) | 0) + 0.5
    });
  }

  getFleeField(c, threatLocalCells, opts = {}) {
    const clr = opts.minClearance ?? 1;
    const prof = opts.profileId ?? 0;
    let hash = 2166136261;
    for (let i = 0; i < threatLocalCells.length; i++) {
      hash = Math.imul(hash ^ threatLocalCells[i], 16777619);
    }
    const ratioKey = ((opts.safeThresholdRatio ?? 0.85) * 100) | 0;
    hash = Math.imul(hash ^ ratioKey, 16777619) >>> 0;
    const k = this.key(c, 6, hash, clr, prof);
    const hit = this.entries.get(k);
    if (hit) return this._promote(k, hit);

    const f = this._acquire(c, 'f', false);
    this.it.integrateFlee(this.grid, c, threatLocalCells, f.integration, opts);
    computeFlowVectors(f, 0, 0, { grid: this.grid });
    return this._store(k, f);
  }

  getCorridorField(c, polyline, opts = {}) {
    const clr = opts.minClearance ?? 1;
    const prof = opts.profileId ?? 0;
    let hash = 2166136261;
    for (let i = 0; i < polyline.length; i++) {
      const p = polyline[i];
      const px = ((Array.isArray(p) ? p[0] : p.x) * 8) | 0;
      const py = ((Array.isArray(p) ? p[1] : p.y) * 8) | 0;
      hash = Math.imul(hash ^ ((px << 16) ^ py), 16777619);
    }
    const rKey = ((opts.radius ?? 2.5) * 16) | 0;
    const tKey = ((opts.tangentWeight ?? 1.25) * 16) | 0;
    hash = Math.imul(hash ^ ((rKey << 8) ^ tKey), 16777619) >>> 0;
    const k = this.key(c, 7, hash, clr, prof);
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

  invalidateGlobalGoalChunks(goalId, dirtyChunks = null) {
    for (const [k, f] of this.entries) {
      if (f.kind === 'G' && f.goalId === goalId && (!dirtyChunks || dirtyChunks.has(f.chunkId))) {
        this.entries.delete(k);
        this._release(f);
      }
    }
  }

  build(k, cid, kind, src, exX, exY, opts = {}) {
    const { grid } = this;
    const f = this._acquire(cid, kind, !!opts.trackSources);
    f.vortex = opts.vortex || null;
    f.goalId = opts.goalId || 0;
    this.it.integrate(grid, cid, src, f.integration, {
      minClearance: opts.minClearance ?? 1,
      profileId: opts.profileId ?? 0,
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
 * P1 & A2: Bilinearly interpolates the flow direction vector `(out.x, out.y)` at world position `(px, py)`.
 * Features a zero-allocation interior fast path for the 76.5% of chunk cells that do not touch a border,
 * and supports zero-closure context passing `(neighborResolver, resolverCtx, resolverArg)` for halo taps.
 */
export function sampleFlowBilinear(f, px, py, out = { x: 0, y: 0 }, neighborResolver = null, resolverCtx = null, resolverArg = null) {
  const lx = px - f.originX - 0.5, ly = py - f.originY - 0.5;
  const x0 = Math.floor(lx), y0 = Math.floor(ly);
  const tx = lx - x0, ty = ly - y0;
  const { integration: I, flowX: FX, flowY: FY } = f;
  const w00 = (1 - tx) * (1 - ty), w10 = tx * (1 - ty), w01 = (1 - tx) * ty, w11 = tx * ty;

  let sx = 0, sy = 0, ws = 0;

  // P1: Zero-allocation interior fast path (all 4 taps lie strictly inside [0..15]x[0..15])
  if (x0 >= 0 && x0 < CS - 1 && y0 >= 0 && y0 < CS - 1) {
    const id00 = y0 * CS + x0;
    const id10 = id00 + 1;
    const id01 = id00 + CS;
    const id11 = id01 + 1;
    if (w00 > 0 && I[id00] < Infinity) { sx += FX[id00] * w00; sy += FY[id00] * w00; ws += w00; }
    if (w10 > 0 && I[id10] < Infinity) { sx += FX[id10] * w10; sy += FY[id10] * w10; ws += w10; }
    if (w01 > 0 && I[id01] < Infinity) { sx += FX[id01] * w01; sy += FY[id01] * w01; ws += w01; }
    if (w11 > 0 && I[id11] < Infinity) { sx += FX[id11] * w11; sy += FY[id11] * w11; ws += w11; }
  } else {
    // Border halo path without closure allocation
    for (let k = 0; k < 4; k++) {
      const ix = x0 + (k & 1), iy = y0 + (k >> 1);
      const w = k === 0 ? w00 : (k === 1 ? w10 : (k === 2 ? w01 : w11));
      if (w <= 0) continue;
      if (ix >= 0 && ix < CS && iy >= 0 && iy < CS) {
        const id = iy * CS + ix;
        if (I[id] < Infinity) { sx += FX[id] * w; sy += FY[id] * w; ws += w; }
        continue;
      }
      if (neighborResolver) {
        const gx = f.originX + ix, gy = f.originY + iy;
        const nf = resolverCtx !== null ? neighborResolver(gx, gy, resolverCtx, resolverArg) : neighborResolver(gx, gy);
        if (nf) {
          const nid = (gy - nf.originY) * CS + (gx - nf.originX);
          if (nf.integration[nid] < Infinity) {
            sx += nf.flowX[nid] * w; sy += nf.flowY[nid] * w; ws += w;
            continue;
          }
        }
      }
      const cx = ix < 0 ? 0 : (ix > 15 ? 15 : ix);
      const cy = iy < 0 ? 0 : (iy > 15 ? 15 : iy);
      const id = cy * CS + cx;
      if (I[id] < Infinity) { sx += FX[id] * w; sy += FY[id] * w; ws += w; }
    }
  }

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
function _resolveGlobalNeighbor(gx, gy, pf, globalGoal) {
  return pf.getGlobalFieldAt(globalGoal, gx, gy);
}

function _resolvePathNeighbor(gx, gy, pf, path) {
  const nRes = pf.getFieldForPath(path, gx, gy, pf._activeHintIdx);
  return nRes ? nRes.field : null;
}

/**
 * Unified Hierarchical Flow Field Pathfinding API exposing all 11 base flow field types
 * plus P1 Zero-Alloc Fast-Path Sampling, P2 Incremental Border Portal Detection,
 * P3 Exact Separable Euclidean SDF, P4 4-Ary Heap, Q1 Clearance-Inset 2D Portal Funnel (SSFA),
 * Q2 True Downstream Wavefront Cascading, Q3 Sub-Cell Fractional Goal Seeding,
 * D1 Off-Grid Jump/Teleporter Portals, D2 Moving-Target Warm-Start Pursuit & Velocity Lead,
 * D3 Lazy Path Splicing & Global Goal Repair, A1 Vectorized SoA Crowd Stepper,
 * A2 Line-of-Sight Cover & Shadow-Seeking Fields, A3 Multi-Modal Terrain Cost Profiles,
 * and A4 Geodesic Formation Arrival Fields.
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

    this.graph = new PortalGraph(this.grid, null, null, this.minClearance, this.maxPortalWidth, 0);
    this.macro = new MacroPathfinder(this.grid, this.graph);
    this.cache = new FlowFieldCache(this.grid, this.cacheCap);
    this.clearanceGraphs = new Map([[this._hierarchyKey(this.minClearance, 0), {
      minClearance: this.minClearance,
      profileId: 0,
      graph: this.graph,
      macro: this.macro
    }]]);
    this._globalGoalEpoch = 1;
    this._cascadeDepth = 0;
    this._activeHintIdx = 0;
  }

  get width() { return this.grid.width; }
  get height() { return this.grid.height; }

  _hierarchyKey(minClearance, profileId = 0) {
    return ((profileId & 15) << 8) | (minClearance & 255);
  }

  getCost(x, y, profileId = 0) { return this.grid.getCost(x, y, profileId); }
  setCost(x, y, cost) { return this.grid.setCost(x, y, cost); }
  setRect(x, y, w, h, cost) { return this.grid.setRect(x, y, w, h, cost); }
  setDisc(x, y, r, cost) { return this.grid.setDisc(x, y, r, cost); }
  loadCosts(costs) { this.grid.loadCosts(costs); }

  // A3: Multi-Modal Terrain Tag & Unit Traversal Cost Profile API
  setTerrainTag(x, y, tag) { return this.grid.setTerrainTag(x, y, tag); }
  setRectTerrainTag(x, y, w, h, tag) { return this.grid.setRectTerrainTag(x, y, w, h, tag); }
  registerCostProfile(profileId, tagCostMap) { return this.grid.registerCostProfile(profileId, tagCostMap); }

  // D1: Off-Grid Jump Links, Teleporters, Bridges & Multi-Layer Portals API
  addCustomPortal(spec) { return this.grid.addCustomPortal(spec); }
  removeCustomPortal(uid) { return this.grid.removeCustomPortal(uid); }
  clearCustomPortals() { this.grid.clearCustomPortals(); }

  /**
   * D2: Convenience passthrough for splatting agent positions and optional velocity vectors `(vx, vy)`
   * onto the continuum congestion layer.
   */
  setCongestionFromAgents(px, py, count, unitWeight = 1.0, vx = null, vy = null) {
    this.grid.setCongestionFromAgents(px, py, count, unitWeight, vx, vy);
  }

  /**
   * B1, B3 & P2: Incrementally updates portal graphs and invalidates cached flow fields.
   * Topology changes only re-scan borders of dirty chunks (P2); weight-only changes update CSR edge costs in-place (B1).
   */
  rebuild() {
    const { all, topology, weights } = this.grid.consumeDirtyState();
    if (all.size === 0) return false;

    if (topology.size > 0 && (this.grid.wallPenaltyWeight > 0 || this.grid.wallRepulsionWeight > 0)) {
      this.grid.updateWallSDF();
    }
    this.cache.invalidateChunks(all);

    for (const [hKey, entry] of this.clearanceGraphs) {
      const clr = entry.minClearance ?? (hKey & 255);
      const prof = entry.profileId ?? (hKey >> 8);
      if (topology.size > 0) {
        const nextGraph = new PortalGraph(this.grid, entry.graph, all, clr, this.maxPortalWidth, prof);
        const nextMacro = new MacroPathfinder(this.grid, nextGraph);
        this.clearanceGraphs.set(hKey, { minClearance: clr, profileId: prof, graph: nextGraph, macro: nextMacro });
        if (clr === this.minClearance && prof === 0) {
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

  _getHierarchy(minClearance = this.minClearance, profileId = 0) {
    if (this.grid.dirtyChunks.size > 0) this.rebuild();
    const hKey = this._hierarchyKey(minClearance, profileId);
    let entry = this.clearanceGraphs.get(hKey);
    if (!entry) {
      const graph = new PortalGraph(this.grid, null, null, minClearance, this.maxPortalWidth, profileId);
      const macro = new MacroPathfinder(this.grid, graph);
      entry = { minClearance, profileId, graph, macro };
      this.clearanceGraphs.set(hKey, entry);
      if (profileId === 0) this.clearanceGraphs.set(minClearance, entry);
    }
    return entry;
  }

  /**
   * Q3 & A3: Computes a single-goal hierarchical macro path with sub-cell fractional start/goal coordinates
   * and optional `minClearance` and `profileId`.
   */
  findPath(sx, sy, gx, gy, opts = {}) {
    const clr = opts.minClearance ?? this.minClearance;
    const prof = opts.profileId ?? 0;
    const { macro } = this._getHierarchy(clr, prof);
    const { grid } = this;
    const scx = clamp(sx | 0, 0, grid.width - 1), scy = clamp(sy | 0, 0, grid.height - 1);
    const gcx = clamp(gx | 0, 0, grid.width - 1), gcy = clamp(gy | 0, 0, grid.height - 1);
    const sWalk = grid.isWalkable(scx, scy, clr, prof);
    const gWalk = grid.isWalkable(gcx, gcy, clr, prof);
    const startCell = sWalk ? grid.index(scx, scy) : grid.findNearestWalkable(scx, scy, 16, clr, prof);
    const goalCell = gWalk ? grid.index(gcx, gcy) : grid.findNearestWalkable(gcx, gcy, 16, clr, prof);
    if (startCell < 0 || goalCell < 0) return null;
    const exactSX = sWalk ? clamp(sx, 0.001, grid.width - 0.001) : (startCell % grid.width) + 0.5;
    const exactSY = sWalk ? clamp(sy, 0.001, grid.height - 0.001) : ((startCell / grid.width) | 0) + 0.5;
    const exactGX = gWalk ? clamp(gx, 0.001, grid.width - 0.001) : (goalCell % grid.width) + 0.5;
    const exactGY = gWalk ? clamp(gy, 0.001, grid.height - 0.001) : ((goalCell / grid.width) | 0) + 0.5;
    const path = macro.findPath(startCell, goalCell, exactSX, exactSY, exactGX, exactGY);
    if (path) {
      path.minClearance = clr;
      path.profileId = prof;
    }
    return path;
  }

  /**
   * #2, Q3 & A3: Computes a Multi-Goal Geodesic Voronoi macro path to the nearest/best weighted target in `goals`.
   */
  findMultiGoalPath(sx, sy, goals, opts = {}) {
    const clr = opts.minClearance ?? this.minClearance;
    const prof = opts.profileId ?? 0;
    const { macro } = this._getHierarchy(clr, prof);
    const { grid } = this;
    const scx = clamp(sx | 0, 0, grid.width - 1), scy = clamp(sy | 0, 0, grid.height - 1);
    const sWalk = grid.isWalkable(scx, scy, clr, prof);
    const startCell = sWalk ? grid.index(scx, scy) : grid.findNearestWalkable(scx, scy, 16, clr, prof);
    if (startCell < 0) return null;
    const exactSX = sWalk ? clamp(sx, 0.001, grid.width - 0.001) : (startCell % grid.width) + 0.5;
    const exactSY = sWalk ? clamp(sy, 0.001, grid.height - 0.001) : ((startCell / grid.width) | 0) + 0.5;
    const validTargets = [];
    for (const g of goals) {
      const gx = clamp(g.x | 0, 0, grid.width - 1), gy = clamp(g.y | 0, 0, grid.height - 1);
      const gWalk = grid.isWalkable(gx, gy, clr, prof);
      const cell = gWalk ? grid.index(gx, gy) : grid.findNearestWalkable(gx, gy, 16, clr, prof);
      if (cell >= 0) {
        validTargets.push({
          cell,
          x: gWalk ? g.x : (cell % grid.width) + 0.5,
          y: gWalk ? g.y : ((cell / grid.width) | 0) + 0.5,
          weight: g.weight ?? 0
        });
      }
    }
    if (!validTargets.length) return null;
    const path = macro.findPath(startCell, validTargets, exactSX, exactSY);
    if (path) {
      path.minClearance = clr;
      path.profileId = prof;
    }
    return path;
  }

  _normalizeTargets(goals, clr, prof) {
    const { grid } = this;
    const list = Array.isArray(goals) ? goals : [goals];
    const validTargets = [];
    for (const g of list) {
      const rawX = typeof g === 'object' ? (g.x ?? (g.cell % grid.width) + 0.5) : (g % grid.width) + 0.5;
      const rawY = typeof g === 'object' ? (g.y ?? ((g.cell / grid.width) | 0) + 0.5) : ((g / grid.width) | 0) + 0.5;
      const gx = clamp(rawX | 0, 0, grid.width - 1);
      const gy = clamp(rawY | 0, 0, grid.height - 1);
      const gWalk = grid.isWalkable(gx, gy, clr, prof);
      const cell = gWalk ? grid.index(gx, gy) : grid.findNearestWalkable(gx, gy, 16, clr, prof);
      if (cell >= 0) {
        validTargets.push({
          cell,
          x: gWalk ? clamp(rawX, 0.001, grid.width - 0.001) : (cell % grid.width) + 0.5,
          y: gWalk ? clamp(rawY, 0.001, grid.height - 0.001) : ((cell / grid.width) | 0) + 0.5,
          weight: typeof g === 'object' ? (g.weight ?? 0) : 0
        });
      }
    }
    return validTargets;
  }

  /**
   * C1, Q3 & A3: Builds a Global All-to-One Reverse Portal Tree for a single goal or array of goals.
   * Any number of agents anywhere in the world can steer toward this goal in O(1) time via
   * `sampleGlobalFlow(globalGoal, wx, wy)` without calling `findPath()`.
   */
  createGlobalGoalField(goals, opts = {}) {
    const clr = opts.minClearance ?? this.minClearance;
    const prof = opts.profileId ?? 0;
    const { macro } = this._getHierarchy(clr, prof);
    const validTargets = this._normalizeTargets(goals, clr, prof);
    if (!validTargets.length) return null;
    const globalGoal = macro.createGlobalGoal(validTargets);
    globalGoal.id = ++this._globalGoalEpoch;
    return globalGoal;
  }

  /**
   * D2: Warm-Start Pursuit & Velocity-Lead Update for an existing `globalGoal`.
   * Applies optional LOS-clamped velocity leading `(vx * leadTime, vy * leadTime)` so pursuers
   * intercept moving targets, and skips global Reverse Dijkstra when portal shifts are within `portalTolerance`.
   */
  updateGlobalGoalTarget(globalGoal, newGoals, opts = {}) {
    if (!globalGoal || !globalGoal.isGlobal) return null;
    const clr = globalGoal.minClearance ?? this.minClearance;
    const prof = globalGoal.profileId ?? 0;
    const { graph, macro } = this._getHierarchy(clr, prof);
    const { grid } = this;

    const leadTime = opts.leadTime ?? 0;
    const list = Array.isArray(newGoals) ? newGoals : [newGoals];
    const ledGoals = [];
    for (let i = 0; i < list.length; i++) {
      const g = list[i];
      if (typeof g === 'object' && leadTime > 0 && (g.vx || g.vy)) {
        const vx = g.vx || 0, vy = g.vy || 0;
        let lx = g.x, ly = g.y;
        const steps = 4;
        for (let s = 1; s <= steps; s++) {
          const tx = clamp(g.x + vx * leadTime * (s / steps), 0.5, grid.width - 0.5);
          const ty = clamp(g.y + vy * leadTime * (s / steps), 0.5, grid.height - 0.5);
          if (grid.isWalkable(tx | 0, ty | 0, clr, prof) && grid.hasLineOfSight(g.x, g.y, tx, ty, prof)) {
            lx = tx; ly = ty;
          } else {
            break;
          }
        }
        ledGoals.push({ ...g, x: lx, y: ly });
      } else {
        ledGoals.push(g);
      }
    }

    const validTargets = this._normalizeTargets(ledGoals, clr, prof);
    if (!validTargets.length) return globalGoal;

    // D3: If graph topology was rebuilt, re-create on the new graph and preserve ID
    if (globalGoal.graph !== graph) {
      const fresh = macro.createGlobalGoal(validTargets);
      Object.assign(globalGoal, fresh);
      this.cache.invalidateGlobalGoalChunks(globalGoal.id, null);
      return globalGoal;
    }

    const { dirtyChunks } = macro.updateGlobalGoal(globalGoal, validTargets, opts);
    this.cache.invalidateGlobalGoalChunks(globalGoal.id, dirtyChunks);
    return globalGoal;
  }

  /**
   * C2: Builds a Global Multi-Chunk Flee & Evasion field away from `threatPoints`.
   */
  createGlobalFleeField(threatPoints, opts = {}) {
    const clr = opts.minClearance ?? this.minClearance;
    const prof = opts.profileId ?? 0;
    const { macro } = this._getHierarchy(clr, prof);
    const globalFlee = macro.createGlobalFleeGoal(threatPoints, opts);
    globalFlee.id = ++this._globalGoalEpoch;
    return globalFlee;
  }

  /**
   * C2: Computes a hierarchical multi-chunk escape path from `(sx, sy)` away from `threatPoints`.
   */
  findFleePath(sx, sy, threatPoints, opts = {}) {
    const clr = opts.minClearance ?? this.minClearance;
    const prof = opts.profileId ?? 0;
    const { macro } = this._getHierarchy(clr, prof);
    const { grid } = this;
    const scx = clamp(sx | 0, 0, grid.width - 1), scy = clamp(sy | 0, 0, grid.height - 1);
    const sWalk = grid.isWalkable(scx, scy, clr, prof);
    const startCell = sWalk ? grid.index(scx, scy) : grid.findNearestWalkable(scx, scy, 16, clr, prof);
    if (startCell < 0) return null;
    const globalFlee = macro.createGlobalFleeGoal(threatPoints, opts);
    if (!globalFlee.safeTargets || !globalFlee.safeTargets.length) return null;
    const path = macro.findPath(startCell, globalFlee.safeTargets, sx, sy);
    if (path) {
      path.minClearance = clr;
      path.profileId = prof;
    }
    return path;
  }

  /**
   * A2: Identifies tactical cover positions occluded from `threatPoints` that sit adjacent to hard obstacles.
   * @param {Array<{x:number, y:number}>} threatPoints - Enemy threat positions.
   * @param {Object} [opts={}] - `{ originX, originY, searchRadius=20, maxCoverSpots=24, minThreatDist=3, minClearance, profileId }`
   */
  findCoverTargets(threatPoints, opts = {}) {
    const { grid } = this;
    if (!threatPoints || !threatPoints.length) return [];
    const clr = opts.minClearance ?? this.minClearance;
    const prof = opts.profileId ?? 0;
    const searchR = opts.searchRadius ?? 20;
    const minThreatD2 = (opts.minThreatDist ?? 2.5) ** 2;
    const maxSpots = opts.maxCoverSpots ?? 24;

    let cx0 = 0, cy0 = 0;
    if (opts.originX !== undefined && opts.originY !== undefined) {
      cx0 = opts.originX; cy0 = opts.originY;
    } else {
      for (let i = 0; i < threatPoints.length; i++) {
        cx0 += threatPoints[i].x; cy0 += threatPoints[i].y;
      }
      cx0 /= threatPoints.length; cy0 /= threatPoints.length;
    }

    const x0 = clamp(Math.floor(cx0 - searchR), 0, grid.width - 1);
    const x1 = clamp(Math.ceil(cx0 + searchR), 0, grid.width - 1);
    const y0 = clamp(Math.floor(cy0 - searchR), 0, grid.height - 1);
    const y1 = clamp(Math.ceil(cy0 + searchR), 0, grid.height - 1);
    const r2 = searchR * searchR;
    const candidates = [];

    for (let y = y0; y <= y1; y++) {
      for (let x = x0; x <= x1; x++) {
        const dOrigin2 = (x + 0.5 - cx0) ** 2 + (y + 0.5 - cy0) ** 2;
        if (dOrigin2 > r2) continue;
        if (!grid.isWalkable(x, y, clr, prof)) continue;

        // Check if adjacent to at least one blocking obstacle cell (wall-hugging cover)
        let wallDirX = 0, wallDirY = 0, nearWall = false;
        for (let d = 0; d < 4; d++) {
          const nx = x + DX[d], ny = y + DY[d];
          if (!grid.inBounds(nx, ny) || grid.getEffectiveBaseCost(ny * grid.width + nx, prof) >= COST_BLOCK) {
            nearWall = true;
            wallDirX += DX[d];
            wallDirY += DY[d];
          }
        }
        if (!nearWall) continue;

        let hiddenCount = 0, tooClose = false, threatVecX = 0, threatVecY = 0;
        for (let t = 0; t < threatPoints.length; t++) {
          const tp = threatPoints[t];
          const dx = tp.x - (x + 0.5), dy = tp.y - (y + 0.5);
          const td2 = dx * dx + dy * dy;
          if (td2 < minThreatD2) { tooClose = true; break; }
          const td = Math.sqrt(td2) || 1;
          threatVecX += dx / td;
          threatVecY += dy / td;
          if (!grid.hasLineOfSight(x + 0.5, y + 0.5, tp.x, tp.y, prof)) {
            hiddenCount++;
          }
        }
        if (tooClose || hiddenCount === 0) continue;

        // Prefer cells hidden from ALL threats where the adjacent wall lies between the cell and the threat
        const align = (wallDirX * threatVecX + wallDirY * threatVecY);
        const unexposedPenalty = (threatPoints.length - hiddenCount) * 25;
        const distScore = Math.sqrt(dOrigin2) * 0.35;
        const weight = Math.max(0, unexposedPenalty + distScore - (align > 0 ? 1.5 : 0));
        candidates.push({
          cell: y * grid.width + x,
          x: x + 0.5,
          y: y + 0.5,
          weight,
          hiddenCount
        });
      }
    }

    candidates.sort((a, b) => a.weight - b.weight);
    if (candidates.length > maxSpots) candidates.length = maxSpots;
    return candidates;
  }

  /**
   * A2: Builds a Line-of-Sight Cover & Shadow-Seeking Global Flow Field ("Take Cover").
   */
  createCoverField(threatPoints, opts = {}) {
    const spots = this.findCoverTargets(threatPoints, opts);
    if (!spots.length) return this.createGlobalFleeField(threatPoints, opts);
    const field = this.createGlobalGoalField(spots, opts);
    if (field) field.coverSpots = spots;
    return field;
  }

  /**
   * A2: Computes a macro path from `(sx, sy)` to the best nearby LOS-occluded cover spot.
   */
  findCoverPath(sx, sy, threatPoints, opts = {}) {
    const spots = this.findCoverTargets(threatPoints, { ...opts, originX: opts.originX ?? sx, originY: opts.originY ?? sy });
    if (!spots.length) return this.findFleePath(sx, sy, threatPoints, opts);
    return this.findMultiGoalPath(sx, sy, spots, opts);
  }

  /**
   * A4: Computes `unitCount` reachable, obstacle-free formation target slots around `(goalX, goalY)`
   * oriented toward `(facingX, facingY)` using geodesic BFS expansion.
   * @param {number} goalX
   * @param {number} goalY
   * @param {number} unitCount
   * @param {Object} [opts={}] - `{ spacing=1.5, facingX=0, facingY=1, columns, minClearance, profileId }`
   */
  computeFormationTargets(goalX, goalY, unitCount, opts = {}) {
    const { grid } = this;
    const count = Math.max(1, unitCount | 0);
    const clr = opts.minClearance ?? this.minClearance;
    const prof = opts.profileId ?? 0;
    const spacing = Math.max(1.0, opts.spacing ?? Math.max(1.25, clr));
    const fx = opts.facingX ?? 0, fy = opts.facingY ?? 1;
    const fLen = Math.hypot(fx, fy) || 1;
    const dirX = fx / fLen, dirY = fy / fLen;
    const rightX = -dirY, rightY = dirX;
    const cols = opts.columns ?? Math.max(1, Math.ceil(Math.sqrt(count * 1.5)));
    const rows = Math.ceil(count / cols);

    const gcx = clamp(goalX | 0, 0, grid.width - 1), gcy = clamp(goalY | 0, 0, grid.height - 1);
    const anchorCell = grid.isWalkable(gcx, gcy, clr, prof)
      ? grid.index(gcx, gcy)
      : grid.findNearestWalkable(gcx, gcy, 24, clr, prof);
    if (anchorCell < 0) return [];
    const ax = (anchorCell % grid.width) + 0.5, ay = ((anchorCell / grid.width) | 0) + 0.5;

    const slots = [];
    const usedCells = new Set();

    // 1. Try ideal oriented rank-and-file slots around (ax, ay)
    for (let r = 0; r < rows && slots.length < count; r++) {
      const rowOffset = (r - (rows - 1) * 0.5) * -spacing;
      for (let c = 0; c < cols && slots.length < count; c++) {
        const colOffset = (c - (cols - 1) * 0.5) * spacing;
        const sx = ax + rightX * colOffset + dirX * rowOffset;
        const sy = ay + rightY * colOffset + dirY * rowOffset;
        const ix = sx | 0, iy = sy | 0;
        if (grid.isWalkable(ix, iy, clr, prof) && grid.hasLineOfSight(ax, ay, sx, sy, prof)) {
          const cell = iy * grid.width + ix;
          if (!usedCells.has(cell)) {
            usedCells.add(cell);
            slots.push({
              slotIndex: slots.length,
              cell,
              x: clamp(sx, ix + 0.15, ix + 0.85),
              y: clamp(sy, iy + 0.15, iy + 0.85),
              weight: (r * 0.15 + Math.abs(c - (cols - 1) * 0.5) * 0.05) * COST_OPEN
            });
          }
        }
      }
    }

    // 2. If walls/corridors blocked some ideal slots, fill remaining slots via flood-fill from anchorCell
    if (slots.length < count) {
      const queue = [anchorCell];
      const visited = new Set([anchorCell]);
      const minSpacing2 = (spacing * 0.75) ** 2;
      let qHead = 0;
      while (qHead < queue.length && slots.length < count) {
        const cur = queue[qHead++];
        const cx = cur % grid.width, cy = (cur / grid.width) | 0;
        const wx = cx + 0.5, wy = cy + 0.5;
        let farEnough = !usedCells.has(cur);
        if (farEnough && spacing > 1.2) {
          for (let i = 0; i < slots.length; i++) {
            if ((slots[i].x - wx) ** 2 + (slots[i].y - wy) ** 2 < minSpacing2) {
              farEnough = false;
              break;
            }
          }
        }
        if (farEnough) {
          usedCells.add(cur);
          slots.push({
            slotIndex: slots.length,
            cell: cur,
            x: wx,
            y: wy,
            weight: Math.hypot(wx - ax, wy - ay) * 0.15 * COST_OPEN
          });
        }
        for (let d = 0; d < 8; d++) {
          const nx = cx + DX[d], ny = cy + DY[d];
          if (!grid.isWalkable(nx, ny, clr, prof)) continue;
          const nCell = ny * grid.width + nx;
          if (!visited.has(nCell)) {
            visited.add(nCell);
            queue.push(nCell);
          }
        }
      }
    }

    return slots;
  }

  /**
   * A4: Builds a Global Formation Arrival Flow Field for `unitCount` slots around `(goalX, goalY)`.
   * Returns a `GlobalGoal` with `.formationSlots` attached.
   */
  createFormationField(goalX, goalY, unitCount, opts = {}) {
    const slots = this.computeFormationTargets(goalX, goalY, unitCount, opts);
    if (!slots.length) return null;
    const field = this.createGlobalGoalField(slots, opts);
    if (field) field.formationSlots = slots;
    return field;
  }

  /**
   * C1, P1, Q2, D1 & D3: Resolves (and caches) the 16x16 flow field for the chunk at `(wx, wy)` in a `GlobalGoal`.
   */
  getGlobalFieldAt(globalGoal, wx, wy) {
    if (!globalGoal || !globalGoal.isGlobal) return null;
    const { grid, cache } = this;
    if (!grid.inBounds(wx | 0, wy | 0)) return null;
    const cx = wx | 0, cy = wy | 0;
    const chunk = grid.chunkIdAt(cx, cy);
    const clr = globalGoal.minClearance ?? this.minClearance;
    const prof = globalGoal.profileId ?? 0;

    // D3: Transparently repair GlobalGoal if graph topology was rebuilt
    const { graph, macro } = this._getHierarchy(clr, prof);
    if (globalGoal.graph !== graph) {
      const fresh = globalGoal.isFlee && globalGoal.safeTargets
        ? macro.createGlobalGoal(globalGoal.safeTargets)
        : macro.createGlobalGoal(globalGoal.goals || [{ cell: globalGoal.cell, x: globalGoal.goalX, y: globalGoal.goalY }]);
      Object.assign(globalGoal, fresh);
    }

    // P1: O(1) register fast-hit when consecutive samples fall in the same chunk
    if (globalGoal._lastChunk === chunk && globalGoal._lastField && !grid.dirtyChunks.has(chunk)) {
      return globalGoal._lastField;
    }

    // P1: 53-bit integer key (zero string allocation)
    const k = cache.key(chunk, 3, globalGoal.id, clr, prof);
    const hit = cache.entries.get(k);
    if (hit) {
      globalGoal._lastChunk = chunk;
      globalGoal._lastField = hit;
      return cache._promote(k, hit);
    }

    if (globalGoal.isFlee && globalGoal.threatChunkGroups && globalGoal.threatChunkGroups.has(chunk)) {
      const cPortals = graph.chunkPortals[chunk];
      let hasSafePortal = false;
      for (let i = 0; i < cPortals.length; i++) {
        if (globalGoal.portalDist[cPortals[i]] < Infinity) { hasSafePortal = true; break; }
      }
      if (!hasSafePortal) {
        const fleeField = cache.getFleeField(chunk, globalGoal.threatChunkGroups.get(chunk), { minClearance: clr, profileId: prof });
        globalGoal._lastChunk = chunk;
        globalGoal._lastField = fleeField;
        return fleeField;
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
      const p = graph.portals[pid];
      if (p.isCustom && !p.bidirectional && chunk !== p.chunkA) continue;

      const pCells = portalSideCells(p, chunk);
      const oppCells = chunk === p.chunkA ? p.cellsB : p.cellsA;
      const oppChunk = chunk === p.chunkA ? p.chunkB : p.chunkA;
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

      // Q2: Check if downstream neighbor chunk already has its GlobalGoal field cached for exact wavefront stitching
      const oppField = !p.isCustom ? cache.entries.get(cache.key(oppChunk, 3, globalGoal.id, clr, prof)) : null;

      let minBias = Infinity;
      const biases = new Float64Array(pCells.length);
      for (let j = 0; j < pCells.length; j++) {
        const gc = pCells[j];
        let b = 0;
        if (oppField) {
          const oppLoc = grid.localIndex(oppCells[j]);
          const dVal = oppField.integration[oppLoc];
          b = dVal < Infinity ? dVal : (COST_OPEN * CS);
        } else if (pCells.length > 1) {
          b = octile((gc % grid.width) + 0.5 - tx, ((gc / grid.width) | 0) + 0.5 - ty) * (COST_OPEN * 0.4);
        }
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

    const cfEntry = globalGoal.chunkFields && globalGoal.chunkFields.has(chunk) ? globalGoal.chunkFields.get(chunk) : null;
    const singleGoalSource = cfEntry && cfEntry.sources.length >= 1 && (!globalGoal.goals || globalGoal.goals.length === 1)
      ? cfEntry.sources[0]
      : null;
    const ox = grid.chunkOriginX(chunk), oy = grid.chunkOriginY(chunk);

    const built = cache.build(k, chunk, 'G', sources, bestExX, bestExY, {
      minClearance: clr,
      profileId: prof,
      goalId: globalGoal.id,
      portalMask: mask,
      losSmoothing: !!(singleGoalSource && grid.losSmoothing),
      targetLocalX: singleGoalSource ? clamp((singleGoalSource.goalX ?? ((singleGoalSource.globalCell % grid.width) + 0.5)) - ox, 0.01, CS - 0.01) : undefined,
      targetLocalY: singleGoalSource ? clamp((singleGoalSource.goalY ?? (((singleGoalSource.globalCell / grid.width) | 0) + 0.5)) - oy, 0.01, CS - 0.01) : undefined
    });
    globalGoal._lastChunk = chunk;
    globalGoal._lastField = built;
    return built;
  }

  /**
   * C1, A2 & P1: Samples the bilinear flow direction `(out.x, out.y)` at `(wx, wy)` from a `GlobalGoal`
   * with zero closure allocations and cross-chunk halo continuity.
   */
  sampleGlobalFlow(globalGoal, wx, wy, out = { x: 0, y: 0 }) {
    const f = this.getGlobalFieldAt(globalGoal, wx, wy);
    if (!f) { out.x = 0; out.y = 0; return false; }
    return sampleFlowBilinear(f, wx, wy, out, _resolveGlobalNeighbor, this, globalGoal);
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
   * Q1, Q2, Q3, D1 & D3: Resolves the 16x16 flow field for the chunk at `(wx, wy)` along a given `path`.
   * Automatically repairs/migrates paths across topology rebuilds (D3), uses Funnel string-pulled targets (Q1),
   * and cascades 1-hop downstream wavefronts across portals (Q2).
   */
  getFieldForPath(path, wx, wy, hintIdx = 0) {
    if (!path) return null;
    if (path.isGlobal) {
      const field = this.getGlobalFieldAt(path, wx, wy);
      return field ? { field, chunkIndex: 0 } : null;
    }
    const clr = path.minClearance ?? this.minClearance;
    const prof = path.profileId ?? 0;
    const { graph, macro } = this._getHierarchy(clr, prof);

    // D3: Lazy Path Migration & Splicing if graph topology changed
    if (path.graph !== graph) {
      if (!macro.repairOrMigratePath(path, wx, wy, hintIdx)) return null;
    }

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
      const p = graph.portals[path.portals[idx]];
      const tx = path.targetX ? path.targetX[idx] : path.goalX;
      const ty = path.targetY ? path.targetY[idx] : path.goalY;

      // Q2: 1-Hop True Downstream Wavefront Cascading across boundary portals
      let downstreamField = null;
      if (!p.isCustom && this._cascadeDepth < 1 && idx + 1 < path.chunks.length) {
        const nextChunk = path.chunks[idx + 1];
        this._cascadeDepth++;
        if (idx + 1 === path.portals.length) {
          downstreamField = this.cache.getGoalField(nextChunk, path.goalCell, clr, path.goalX, path.goalY, prof);
        } else {
          const nextP = graph.portals[path.portals[idx + 1]];
          const ntx = path.targetX ? path.targetX[idx + 1] : path.goalX;
          const nty = path.targetY ? path.targetY[idx + 1] : path.goalY;
          downstreamField = this.cache.getPortalField(nextChunk, nextP, clr, ntx, nty, null, prof);
        }
        this._cascadeDepth--;
      }

      field = this.cache.getPortalField(chunk, p, clr, tx, ty, downstreamField, prof);
    } else {
      field = this.cache.getGoalField(chunk, path.goalCell, clr, path.goalX, path.goalY, prof);
    }
    if (!(field.integration[(cy - field.originY) * CS + (cx - field.originX)] < Infinity)) {
      return null;
    }
    return { field, chunkIndex: idx, portal: idx < path.portals.length ? graph.portals[path.portals[idx]] : null };
  }

  /**
   * P1, Q1, A2 & A5: Samples the bilinear flow direction `(out.x, out.y)` at `(wx, wy)` along `path`
   * with zero closure allocations, cross-chunk halo interpolation, and direct LOS smoothing.
   */
  sampleFlow(path, wx, wy, out = { x: 0, y: 0 }, hintIdx = 0) {
    if (!path) { out.x = 0; out.y = 0; return false; }
    if (path.isGlobal) return this.sampleGlobalFlow(path, wx, wy, out);
    const res = this.getFieldForPath(path, wx, wy, hintIdx);
    if (!res) { out.x = 0; out.y = 0; return false; }

    // A5 & Q1: Direct Cross-Chunk Line-of-Sight steering when unobstructed to funnel target or goal
    if (this.grid.losSmoothing && res.chunkIndex < path.portals.length && (!res.portal || !res.portal.isCustom)) {
      const clr = path.minClearance ?? this.minClearance;
      const prof = path.profileId ?? 0;
      const refCost = this.grid.getEffectiveBaseCost(path.goalCell, prof) || COST_OPEN;
      const tx = path.targetX ? path.targetX[res.chunkIndex] : path.goalX;
      const ty = path.targetY ? path.targetY[res.chunkIndex] : path.goalY;
      const dx = tx - wx, dy = ty - wy;
      const dist = Math.hypot(dx, dy);
      if (dist > 1e-4 && dist <= CS * 3) {
        if (this.grid.hasWorldLOS(wx, wy, tx, ty, refCost, clr, prof)) {
          out.x = dx / dist;
          out.y = dy / dist;
          return true;
        }
      }
    }

    this._activeHintIdx = res.chunkIndex;
    return sampleFlowBilinear(res.field, wx, wy, out, _resolvePathNeighbor, this, path);
  }

  /**
   * A1: Creates a vectorized Structure-of-Arrays `CrowdSwarm` bound to this pathfinder.
   */
  createCrowdSwarm(maxAgents = 1024, opts = {}) {
    return new CrowdSwarm(this, maxAgents, opts);
  }

  /**
   * E1: Integrates a continuous streamline trajectory through the flow field from `(sx, sy)`
   * using 2nd-order Runge-Kutta (midpoint method) and Ramer-Douglas-Peucker simplification.
   * Works with both `MacroPath` and `GlobalGoal` descriptors (including D1 custom jump portals).
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
        // D1: Traverse custom jump/teleporter portal when reaching entrance anchor
        if (res.portal && res.portal.isCustom) {
          const p = res.portal;
          const chunk = this.grid.chunkIdAt(x | 0, y | 0);
          const entX = chunk === p.chunkA ? p.anchorX : p.anchorBX;
          const entY = chunk === p.chunkA ? p.anchorY : p.anchorBY;
          if ((x - entX) ** 2 + (y - entY) ** 2 <= Math.max(0.8, stepSize * 1.5) ** 2) {
            x = chunk === p.chunkA ? p.anchorBX : p.anchorX;
            y = chunk === p.chunkA ? p.anchorBY : p.anchorY;
            hintIdx++;
            pts.push({ x, y });
            continue;
          }
        }
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

//#region 8: Vectorized SoA Batch Crowd Stepper (A1)
/**
 * A1: High-performance Structure-of-Arrays (SoA) batch crowd simulator with:
 * - Zero-allocation 1D counting-sort spatial hash grid for O(N) neighbor collision separation
 * - Direct flow field steering (supporting per-agent paths, shared GlobalGoal trees, and D1 teleporters)
 * - Exact O(1) Wall SDF collision resolution & smooth tangential wall sliding
 * - Automatic continuum congestion splatting (`setCongestionFromAgents`)
 */
export class CrowdSwarm {
  /**
   * @param {FlowFieldPathfinder} pathfinder
   * @param {number} [maxAgents=1024]
   * @param {Object} [opts={}]
   */
  constructor(pathfinder, maxAgents = 1024, opts = {}) {
    this.pf = pathfinder;
    this.grid = pathfinder.grid;
    this.capacity = maxAgents;
    this.count = 0;

    // Flat Structure-of-Arrays agent buffers
    this.px = new Float32Array(maxAgents);
    this.py = new Float32Array(maxAgents);
    this.vx = new Float32Array(maxAgents);
    this.vy = new Float32Array(maxAgents);
    this.radius = new Float32Array(maxAgents);
    this.maxSpeed = new Float32Array(maxAgents);
    this.active = new Uint8Array(maxAgents);
    this.hintIdx = new Int32Array(maxAgents);
    this.targets = new Array(maxAgents).fill(null);

    // Simulation parameters
    this.cellSize = opts.cellSize ?? 2.0;
    this.separationWeight = opts.separationWeight ?? 2.2;
    this.steeringAccel = opts.steeringAccel ?? 12.0;
    this.arrivalRadius = opts.arrivalRadius ?? 0.65;
    this.wallMargin = opts.wallMargin ?? 0.1;
    this.syncCongestion = opts.syncCongestion ?? false;

    // Spatial hash grid buffers (2x2 cell bins)
    this.binsX = Math.max(1, Math.ceil(this.grid.width / this.cellSize));
    this.binsY = Math.max(1, Math.ceil(this.grid.height / this.cellSize));
    this.binCount = this.binsX * this.binsY;
    this.binHead = new Int32Array(this.binCount);
    this.binNext = new Int32Array(maxAgents);
    this._sampleOut = { x: 0, y: 0 };

    if (!this.grid.wallDist) this.grid.updateWallSDF(true);
  }

  /**
   * Adds an agent to the swarm and returns its agent index.
   */
  addAgent(x, y, target = null, opts = {}) {
    let id = this.count;
    if (id >= this.capacity) {
      id = this.active.indexOf(0);
      if (id < 0) return -1;
    } else {
      this.count++;
    }
    this.px[id] = x;
    this.py[id] = y;
    this.vx[id] = opts.vx ?? 0;
    this.vy[id] = opts.vy ?? 0;
    this.radius[id] = opts.radius ?? 0.35;
    this.maxSpeed[id] = opts.maxSpeed ?? 4.0;
    this.active[id] = 1;
    this.hintIdx[id] = 0;
    this.targets[id] = target;
    return id;
  }

  removeAgent(id) {
    if (id >= 0 && id < this.count) {
      this.active[id] = 0;
      this.targets[id] = null;
    }
  }

  setAgentTarget(id, target) {
    if (id >= 0 && id < this.count) {
      this.targets[id] = target;
      this.hintIdx[id] = 0;
    }
  }

  /**
   * Steps all active agents forward by `dt` seconds.
   */
  step(dt = 1 / 60, sharedTarget = null) {
    if (dt <= 0 || this.count === 0) return;
    const {
      pf,
      grid,
      count,
      px,
      py,
      vx,
      vy,
      radius,
      maxSpeed,
      active,
      hintIdx,
      targets,
      binsX,
      binsY,
      cellSize,
      binHead,
      binNext,
      _sampleOut
    } = this;
    const W = grid.width,
      H = grid.height;
    const invBin = 1 / cellSize;

    if (grid._sdfDirty || !grid.wallDist) grid.updateWallSDF();
    const wallDist = grid.wallDist,
      wallGX = grid.wallGradX,
      wallGY = grid.wallGradY;

    // 1. Populate O(N) linked-list spatial hash bins
    binHead.fill(-1);
    for (let i = 0; i < count; i++) {
      if (!active[i]) continue;
      const bx = clamp((px[i] * invBin) | 0, 0, binsX - 1);
      const by = clamp((py[i] * invBin) | 0, 0, binsY - 1);
      const b = by * binsX + bx;
      binNext[i] = binHead[b];
      binHead[b] = i;
    }

    const sepWeight = this.separationWeight;
    const accelRate = Math.min(1, this.steeringAccel * dt);
    const arriveR2 = this.arrivalRadius * this.arrivalRadius;

    // 2. Flow sampling + spatial-hash neighbor separation + velocity integration
    for (let i = 0; i < count; i++) {
      if (!active[i]) continue;
      let ax = px[i],
        ay = py[i];
      const rI = radius[i],
        maxSpd = maxSpeed[i];
      const tgt = targets[i] || sharedTarget;

      // D1: Check custom teleporter/jump portal transition on macro paths
      let desX = 0,
        desY = 0;
      if (tgt) {
        if (!tgt.isGlobal) {
          const res = pf.getFieldForPath(tgt, ax, ay, hintIdx[i]);
          if (res) {
            hintIdx[i] = res.chunkIndex;
            if (res.portal && res.portal.isCustom) {
              const p = res.portal;
              const c = grid.chunkIdAt(ax | 0, ay | 0);
              const entX = c === p.chunkA ? p.anchorX : p.anchorBX;
              const entY = c === p.chunkA ? p.anchorY : p.anchorBY;
              if ((ax - entX) ** 2 + (ay - entY) ** 2 <= (rI + 0.65) ** 2) {
                ax = px[i] = c === p.chunkA ? p.anchorBX : p.anchorX;
                ay = py[i] = c === p.chunkA ? p.anchorBY : p.anchorY;
                hintIdx[i]++;
              }
            }
          }
        }

        const hasSingleGoal = tgt.goalX !== undefined && (!tgt.goals || tgt.goals.length <= 1);
        const distGoal2 = hasSingleGoal ? (tgt.goalX - ax) ** 2 + (tgt.goalY - ay) ** 2 : Infinity;

        if (distGoal2 > arriveR2) {
          const ok = tgt.isGlobal
            ? pf.sampleGlobalFlow(tgt, ax, ay, _sampleOut)
            : pf.sampleFlow(tgt, ax, ay, _sampleOut, hintIdx[i]);
          if (ok) {
            const slowDown =
              distGoal2 < arriveR2 * 9 ? Math.sqrt(distGoal2) / (this.arrivalRadius * 3) : 1;
            desX = _sampleOut.x * maxSpd * slowDown;
            desY = _sampleOut.y * maxSpd * slowDown;
          }
        }
      }

      // Short-range neighbor separation in 3x3 surrounding spatial bins
      const bx0 = clamp((ax * invBin) | 0, 0, binsX - 1);
      const by0 = clamp((ay * invBin) | 0, 0, binsY - 1);
      let sepX = 0,
        sepY = 0;

      for (let dy = -1; dy <= 1; dy++) {
        const by = by0 + dy;
        if (by < 0 || by >= binsY) continue;
        const rowB = by * binsX;
        for (let dx = -1; dx <= 1; dx++) {
          const bx = bx0 + dx;
          if (bx < 0 || bx >= binsX) continue;
          let j = binHead[rowB + bx];
          while (j >= 0) {
            if (j !== i) {
              const rx = ax - px[j],
                ry = ay - py[j];
              const minSep = rI + radius[j];
              const d2 = rx * rx + ry * ry;
              if (d2 < minSep * minSep && d2 > 1e-8) {
                const d = Math.sqrt(d2);
                const pen = (minSep - d) / minSep;
                sepX += (rx / d) * pen;
                sepY += (ry / d) * pen;
              } else if (d2 <= 1e-8) {
                sepX += (i > j ? 1 : -1) * 0.5;
              }
            }
            j = binNext[j];
          }
        }
      }

      let tvx = desX + sepX * sepWeight * maxSpd;
      let tvy = desY + sepY * sepWeight * maxSpd;
      const tSpd = Math.hypot(tvx, tvy);
      if (tSpd > maxSpd) {
        tvx = (tvx / tSpd) * maxSpd;
        tvy = (tvy / tSpd) * maxSpd;
      }

      let nvx = vx[i] + (tvx - vx[i]) * accelRate;
      let nvy = vy[i] + (tvy - vy[i]) * accelRate;
      let nx = clamp(ax + nvx * dt, rI, W - rI);
      let ny = clamp(ay + nvy * dt, rI, H - rI);

      // 3. O(1) Wall SDF collision pushout & tangential wall sliding
      const gIdx = (ny | 0) * W + (nx | 0);
      const wd = wallDist[gIdx];
      const minWall = rI + this.wallMargin;
      if (wd < minWall) {
        let gx = wallGX[gIdx],
          gy = wallGY[gIdx];
        if (gx === 0 && gy === 0) {
          // If inside a blocked cell, push back toward previous walkable position
          const bdx = ax - nx,
            bdy = ay - ny,
            bl = Math.hypot(bdx, bdy);
          if (bl > 1e-5) {
            gx = bdx / bl;
            gy = bdy / bl;
          }
        }
        const push = minWall - wd;
        nx = clamp(nx + gx * push, rI, W - rI);
        ny = clamp(ny + gy * push, rI, H - rI);
        const vn = nvx * gx + nvy * gy;
        if (vn < 0) {
          nvx -= gx * vn;
          nvy -= gy * vn;
        }
      }

      // Final walkability guard
      if (grid.costs[(ny | 0) * W + (nx | 0)] >= COST_BLOCK) {
        if (grid.costs[(ay | 0) * W + (nx | 0)] < COST_BLOCK) {
          ny = ay;
          nvy = 0;
        } else if (grid.costs[(ny | 0) * W + (ax | 0)] < COST_BLOCK) {
          nx = ax;
          nvx = 0;
        } else {
          nx = ax;
          ny = ay;
          nvx = 0;
          nvy = 0;
        }
      }

      px[i] = nx;
      py[i] = ny;
      vx[i] = nvx;
      vy[i] = nvy;
    }

    if (this.syncCongestion) {
      pf.setCongestionFromAgents(px, py, count, 1.0, vx, vy);
    }
  }
}
//#endregion
/**
 * @module flowfield-pathfinder/core
 * Standalone 2D Hierarchical Flow-Field & Portal-Graph Pathfinding Kernel (v2.0).
 *
 * Includes:
 * - [1.1] Surgical Chunk-Local Portal & CSR Patching (Incremental Topology Rebuild)
 * - [1.2] Undirected Fast-Path & Lazy Tarjan SCC Reachability
 * - [1.3] Incremental Swarm Field Subtree Repair (`updateSwarmField`)
 * - [2.1] 4-Pass Gauss-Seidel Raster-Sweep Eikonal Solver (FIM) in `ChunkIntegrator`
 * - [2.2] Exact Meijster 1D Parabolic Envelope Euclidean Distance Transform (EDT) for Wall SDF
 * - [2.3] Funnel-Projected Continuous Portal Entry Seeding
 * - [3.1] Any-Angle World-LOS Waypoint Polyline Extractor (`extractWaypoints` & `hasWorldLOS`)
 * - [3.2] ALT (A*, Landmarks, Triangle Inequality) Heuristic on `PortalGraph`
 * - [3.3] 3-Point Affine Portal Entry Slope Bias (`edgeEntryBias`)
 * - [4.1] Reference-Counted Dynamic Obstacle Stamp API (`stampObstacle`, `unstampObstacle`, `moveObstacle`)
 * - [4.2] Continuum Crowd Density & Velocity Flux Splatting (`updateCrowdCongestion`)
 * - [4.3] Wall-Safe Local Micro-Avoidance Tangent-Sliding Kernel (`applyLocalAvoidance`)
 * - [5.1] Quantized `Int8Array` / `Uint16Array` Flow Field Cache Support (`quantizeFields`)
 * - [5.2] Wait-Free `Atomics` Per-Chunk Seqlock (`chunkSeq`) for SharedArrayBuffer Workers
 */

//#region 1: Grid & Cost Constants
/** @constant {number} Chunk width/height in cells (16x16) */
export const CS = 16;

/** @constant {number} Total cells per chunk (256) */
export const CC = CS * CS;

/** @constant {number} Super-chunk width/height in chunks (4x4 chunks = 64x64 cells) */
export const SUPER_STEP = 4;

/** @constant {number} Default open terrain traversal cost */
export const COST_OPEN = 2;

/** @constant {number} Impassable obstacle cost threshold (255) */
export const COST_BLOCK = 255;

/** @constant {number} Maximum portal width before splitting */
export const MAX_PORTAL_LEN = 6;

/** @constant {number} Maximum tracked brushfire clearance radius in cells */
export const MAX_CLEARANCE = 16;

/** @constant {number} Default number of ALT heuristic landmarks */
export const NUM_LANDMARKS = 8;

/** @constant {number} Fixed-point scale for Uint16 quantized integration fields */
export const QUANT_I_SCALE = 32;
export const QUANT_I_INF = 0xFFFF;
//#endregion

//#region 2: Directional Lookup Tables & Math Utilities
export const SQ2 = Math.SQRT2;
export const OCT = SQ2 - 1;

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

/** Fast 32-bit FNV-1a mixer for composite cache keys */
export const hashMix = (h, v) => Math.imul((h ^ (v | 0)) >>> 0, 16777619) >>> 0;

/** Deterministic 32-bit hash & equality helper for rectangular vehicle footprints */
export const hashFootprint = (fp) => {
  if (!fp) return 0;
  let h = hashMix(2166136261, ((fp.width ?? 1) * 100) | 0);
  h = hashMix(h, ((fp.length ?? 1) * 100) | 0);
  return hashMix(h, ((fp.turnPenalty ?? 2) * 100) | 0);
};

export const sameFootprint = (a, b) => {
  if (a === b) return true;
  if (!a || !b) return false;
  return (a.width ?? 1) === (b.width ?? 1) &&
    (a.length ?? 1) === (b.length ?? 1) &&
    (a.turnPenalty ?? 2) === (b.turnPenalty ?? 2);
};
//#endregion

//#region 3: 4-Ary Cache-Friendly Min-Heap
/**
 * TypedArray-backed 4-ary min-heap for Eikonal, Dijkstra, and A* searches.
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
//#endregion

//#region 4: Zero-Allocation Slab Buffer Pool (Float32 & Quantized Int8/Uint16)
/**
 * Zero-allocation slab pool for recycling chunk-sized TypedArrays on cache eviction.
 * Supports both full Float32 fields and [5.1] Quantized Int8/Uint16 fields.
 */
export class FieldBufferPool {
  constructor(maxSlabs = 512) {
    this.maxSlabs = maxSlabs;
    this.f32Pool = [];
    this.i32Pool = [];
    this.i8Pool = [];
    this.u16Pool = [];
  }
  allocF32() {
    return this.f32Pool.length > 0 ? this.f32Pool.pop() : new Float32Array(CC);
  }
  allocI32() {
    return this.i32Pool.length > 0 ? this.i32Pool.pop() : new Int32Array(CC);
  }
  allocI8() {
    return this.i8Pool.length > 0 ? this.i8Pool.pop() : new Int8Array(CC);
  }
  allocU16() {
    return this.u16Pool.length > 0 ? this.u16Pool.pop() : new Uint16Array(CC);
  }
  releaseField(f) {
    if (!f || f._pooled) return;
    f._pooled = true;
    if (f.quantized) {
      if (f.integration && this.u16Pool.length < this.maxSlabs) this.u16Pool.push(f.integration);
      if (f.flowX && this.i8Pool.length < this.maxSlabs * 2) this.i8Pool.push(f.flowX, f.flowY);
    } else if (f.integration && this.f32Pool.length < this.maxSlabs * 3) {
      this.f32Pool.push(f.integration, f.flowX, f.flowY);
    }
    f.integration = f.flowX = f.flowY = null;
    if (f.sourceId && this.i32Pool.length < this.maxSlabs) {
      this.i32Pool.push(f.sourceId);
      f.sourceId = null;
    }
  }
}
//#endregion

//#region 5: Grid Core State, Seqlocks, EDT SDF & Crowd/Obstacle Layers
/**
 * 2D chunked cost grid supporting:
 * - Anisotropic directional clearance (`clearanceX`, `clearanceY`) for rectangular vehicles
 * - [5.2] `SharedArrayBuffer` backing with per-chunk `Atomics` seqlock (`chunkSeq`)
 * - [2.2] Exact Meijster Parabolic Envelope Euclidean Distance Transform (EDT) for Wall SDF
 * - [4.1] Reference-counted Dynamic Obstacle Stamps (`stampObstacle` / `unstampObstacle`)
 * - [4.2] Continuum Crowd Density & Velocity Flux Splatting (`updateCrowdCongestion`)
 */
export class Grid {
  constructor(cx, cy, existingCosts = null, opts = {}) {
    this.cx = cx;
    this.cy = cy;
    this.width = cx * CS;
    this.height = cy * CS;
    this.chunkCount = cx * cy;
    this.cellCount = this.width * this.height;
    const len = this.cellCount;

    // [5.2] SharedArrayBuffer backing & per-chunk Atomics seqlock
    this.useSharedMemory = !!(opts.shared && typeof SharedArrayBuffer !== 'undefined');
    this._allocBuf = (byteSize) => this.useSharedMemory ? new SharedArrayBuffer(byteSize) : new ArrayBuffer(byteSize);
    this.chunkSeq = new Int32Array(this._allocBuf(this.chunkCount * 4));

    if (existingCosts) {
      this.costs = existingCosts.subarray(0, len);
    } else {
      this.costs = new Uint8Array(this._allocBuf(len)).fill(COST_OPEN);
    }

    // Two-Tier Dirty Tracking (Topology vs. Weights)
    this.dirtyChunks = new Set();
    this.topologyDirtyChunks = new Set();
    this.weightsDirtyChunks = new Set();

    // Solver & feature configuration
    this.useEikonal = true;
    this.losSmoothing = true;

    // Wall Repulsion / Signed Distance Field (SDF)
    this.wallPenaltyRadius = 2.5;
    this.wallPenaltyWeight = 0.0;
    this.wallRepulsionWeight = 0.0;
    this.wallDist = null;
    this.wallGradX = null;
    this.wallGradY = null;
    this._sdfDirty = true;
    this._sdfFullDirty = true;
    this._sdfDirtyChunks = new Set();
    const maxDim = Math.max(this.width, this.height) + 1;
    this._edtG = new Float32Array(len);
    this._edtV = new Int32Array(maxDim);
    this._edtZ = new Float32Array(maxDim + 1);

    // Isotropic & Directional Clearance (X/Y Corridor Spans)
    this.clearance = null;
    this.clearanceX = null;
    this.clearanceY = null;
    this._clearanceDirty = true;
    this._clearanceFullDirty = true;
    this._clearanceDirtyChunks = new Set();

    // Lazy TypedArray Layer Pointers
    this.tacticalCosts = null;
    this.hasTactical = false;

    this.congestion = null;
    this.congestionFluxX = null;
    this.congestionFluxY = null;
    this.congestionWeight = 1.0;
    this.congestionLaneWeight = 0.65;
    this.hasCongestion = false;
    this.hasCongestionFlux = false;
    this._chunkCongSnap = new Float32Array(this.chunkCount);
    this._chunkCongNew = new Float32Array(this.chunkCount);

    this.dirMask = null;
    this.currentX = null;
    this.currentY = null;
    this.currentStrength = 0.5;
    this.hasAnisotropy = false;

    // [4.1] Reference-counted dynamic obstacles
    this.dynamicObstacles = null;
    this.hasDynamicObstacles = false;
    this._obstacleStamps = new Map();
    this._nextObstacleId = 1;

    // Terrain ID Layer & Scoped Unit Movement Cost Profiles
    this.terrainIds = null;
    this.profiles = new Map();
    this.profileIds = new Map();
    this._nextProfileId = 1;
  }

  /** [5.2] Begins a seqlock write on chunk `c` (increments sequence to odd) */
  beginChunkWrite(c) {
    if (this.useSharedMemory && typeof Atomics !== 'undefined') {
      Atomics.add(this.chunkSeq, c, 1);
    } else {
      this.chunkSeq[c] = (this.chunkSeq[c] + 1) | 0;
    }
  }

  /** [5.2] Ends a seqlock write on chunk `c` (increments sequence to even) */
  endChunkWrite(c) {
    if (this.useSharedMemory && typeof Atomics !== 'undefined') {
      Atomics.add(this.chunkSeq, c, 1);
    } else {
      this.chunkSeq[c] = (this.chunkSeq[c] + 1) | 0;
    }
  }

  /** Exports SharedArrayBuffer/ArrayBuffer views for hydrating a Worker thread */
  toSharedDescriptor() {
    return {
      cx: this.cx, cy: this.cy,
      useEikonal: this.useEikonal, losSmoothing: this.losSmoothing,
      wallPenaltyRadius: this.wallPenaltyRadius, wallPenaltyWeight: this.wallPenaltyWeight, wallRepulsionWeight: this.wallRepulsionWeight,
      congestionWeight: this.congestionWeight, congestionLaneWeight: this.congestionLaneWeight,
      currentStrength: this.currentStrength,
      costs: this.costs, chunkSeq: this.chunkSeq,
      clearance: this.clearance, clearanceX: this.clearanceX, clearanceY: this.clearanceY,
      wallDist: this.wallDist, wallGradX: this.wallGradX, wallGradY: this.wallGradY,
      tacticalCosts: this.tacticalCosts, hasTactical: this.hasTactical,
      congestion: this.congestion, congestionFluxX: this.congestionFluxX, congestionFluxY: this.congestionFluxY,
      hasCongestion: this.hasCongestion, hasCongestionFlux: this.hasCongestionFlux,
      dirMask: this.dirMask, currentX: this.currentX, currentY: this.currentY, hasAnisotropy: this.hasAnisotropy,
      dynamicObstacles: this.dynamicObstacles, hasDynamicObstacles: this.hasDynamicObstacles,
      terrainIds: this.terrainIds,
      profiles: Array.from(this.profiles.entries())
    };
  }

  /** Hydrates a `Grid` instance directly over shared buffers in a Worker */
  static fromSharedDescriptor(desc) {
    const g = new Grid(desc.cx, desc.cy, desc.costs, { shared: true });
    if (desc.chunkSeq) g.chunkSeq = desc.chunkSeq;
    g.useEikonal = desc.useEikonal; g.losSmoothing = desc.losSmoothing;
    g.wallPenaltyRadius = desc.wallPenaltyRadius; g.wallPenaltyWeight = desc.wallPenaltyWeight; g.wallRepulsionWeight = desc.wallRepulsionWeight;
    g.congestionWeight = desc.congestionWeight; g.congestionLaneWeight = desc.congestionLaneWeight;
    g.currentStrength = desc.currentStrength;
    if (desc.clearance) { g.clearance = desc.clearance; g.clearanceX = desc.clearanceX; g.clearanceY = desc.clearanceY; g._clearanceDirty = g._clearanceFullDirty = false; }
    if (desc.wallDist) { g.wallDist = desc.wallDist; g.wallGradX = desc.wallGradX; g.wallGradY = desc.wallGradY; g._sdfDirty = g._sdfFullDirty = false; }
    if (desc.tacticalCosts) { g.tacticalCosts = desc.tacticalCosts; g.hasTactical = desc.hasTactical; }
    if (desc.congestion) { g.congestion = desc.congestion; g.congestionFluxX = desc.congestionFluxX; g.congestionFluxY = desc.congestionFluxY; g.hasCongestion = desc.hasCongestion; g.hasCongestionFlux = desc.hasCongestionFlux; }
    if (desc.dirMask) { g.dirMask = desc.dirMask; g.currentX = desc.currentX; g.currentY = desc.currentY; g.hasAnisotropy = desc.hasAnisotropy; }
    if (desc.dynamicObstacles) { g.dynamicObstacles = desc.dynamicObstacles; g.hasDynamicObstacles = desc.hasDynamicObstacles; }
    if (desc.terrainIds) g.terrainIds = desc.terrainIds;
    if (desc.profiles) { for (const [k, v] of desc.profiles) g.registerProfile(k, v); }
    return g;
  }

  //#region 6: Indexing, Walkability & Two-Tier Dirty Tracking
  index(x, y) { return y * this.width + x; }
  inBounds(x, y) { return x >= 0 && y >= 0 && x < this.width && y < this.height; }

  getProfileTable(profile = null) {
    if (!profile) return null;
    return profile instanceof Uint8Array ? profile : (this.profiles.get(profile) || null);
  }

  getProfileId(profile = null) {
    if (!profile) return 0;
    let id = this.profileIds.get(profile);
    if (id === undefined) {
      id = this._nextProfileId++;
      this.profileIds.set(profile, id);
    }
    return id;
  }

  getEffectiveBaseCost(idx, profileTable = null) {
    if (this.hasDynamicObstacles && this.dynamicObstacles[idx] > 0) return COST_BLOCK;
    if (profileTable && this.terrainIds) return profileTable[this.terrainIds[idx]];
    return this.costs[idx];
  }

  isWalkable(x, y, minClearance = 1, profile = null, footprint = null) {
    if (!this.inBounds(x, y)) return false;
    const idx = this.index(x, y);
    const pTable = this.getProfileTable(profile);
    if (this.getEffectiveBaseCost(idx, pTable) >= COST_BLOCK) return false;
    const reqClr = footprint ? Math.min(footprint.width ?? 1, footprint.length ?? 1) : minClearance;
    const reqLong = footprint ? Math.max(footprint.width ?? 1, footprint.length ?? 1) : reqClr;
    if (reqClr > 1 || reqLong > 1) {
      if (this._clearanceDirty || !this.clearance) this.updateClearance();
      if (this.clearance[idx] < reqClr) return false;
      if (reqLong > reqClr && Math.max(this.clearanceX[idx], this.clearanceY[idx]) < reqLong) return false;
    }
    return true;
  }

  chunkIdAt(x, y) { return (y / CS | 0) * this.cx + (x / CS | 0); }
  chunkOriginX(c) { return (c % this.cx) * CS; }
  chunkOriginY(c) { return ((c / this.cx) | 0) * CS; }
  localIndex(cell) {
    return ((cell / this.width | 0) % CS) * CS + (cell % this.width % CS);
  }

  _addChunkAndBorderNeighbors(set, x, y) {
    const c = this.chunkIdAt(x, y);
    set.add(c);
    const lx = x % CS, ly = y % CS;
    if (lx === 0 && x > 0) set.add(c - 1);
    if (lx === CS - 1 && x < this.width - 1) set.add(c + 1);
    if (ly === 0 && y > 0) set.add(c - this.cx);
    if (ly === CS - 1 && y < this.height - 1) set.add(c + this.cx);
  }

  markCellDirty(x, y, isTopology = true) {
    this._addChunkAndBorderNeighbors(this.dirtyChunks, x, y);
    if (isTopology) {
      this._addChunkAndBorderNeighbors(this.topologyDirtyChunks, x, y);
      const c = this.chunkIdAt(x, y);
      this._sdfDirty = true;
      this._clearanceDirty = true;
      this._sdfDirtyChunks.add(c);
      this._clearanceDirtyChunks.add(c);
    } else {
      this._addChunkAndBorderNeighbors(this.weightsDirtyChunks, x, y);
    }
  }

  markChunkWeightDirty(c) {
    this.dirtyChunks.add(c);
    this.weightsDirtyChunks.add(c);
  }
  //#endregion

  //#region 7: Dirty Cluster Merging
  _buildDirtyClusters(dirtyChunkSet, halo, isFull) {
    const { width: W, height: H } = this;
    if (isFull || dirtyChunkSet.size === 0 || dirtyChunkSet.size > (this.chunkCount >> 1)) {
      return [{ x0: 0, y0: 0, x1: W - 1, y1: H - 1 }];
    }
    const rects = [];
    for (const c of dirtyChunkSet) {
      const ox = this.chunkOriginX(c), oy = this.chunkOriginY(c);
      rects.push({
        x0: Math.max(0, ox - halo),
        y0: Math.max(0, oy - halo),
        x1: Math.min(W - 1, ox + CS - 1 + halo),
        y1: Math.min(H - 1, oy + CS - 1 + halo)
      });
    }
    let merged = true;
    while (merged && rects.length > 1) {
      merged = false;
      for (let i = 0; i < rects.length; i++) {
        const a = rects[i];
        for (let j = i + 1; j < rects.length; j++) {
          const b = rects[j];
          if (a.x0 <= b.x1 + 2 && a.x1 + 2 >= b.x0 && a.y0 <= b.y1 + 2 && a.y1 + 2 >= b.y0) {
            a.x0 = Math.min(a.x0, b.x0);
            a.y0 = Math.min(a.y0, b.y0);
            a.x1 = Math.max(a.x1, b.x1);
            a.y1 = Math.max(a.y1, b.y1);
            rects.splice(j, 1);
            merged = true;
            break;
          }
        }
        if (merged) break;
      }
    }
    return rects;
  }
  //#endregion

  //#region 8: Cost Mutators, [4.1] Ref-Counted Dynamic Obstacles & [4.2] Crowd Congestion
  getCost(x, y) {
    return this.inBounds(x | 0, y | 0) ? this.costs[this.index(x | 0, y | 0)] : COST_BLOCK;
  }

  setCost(x, y, cost) {
    const ix = x | 0, iy = y | 0;
    if (!this.inBounds(ix, iy)) return false;
    const idx = this.index(ix, iy), prev = this.costs[idx];
    if (prev === cost) return false;
    const c = this.chunkIdAt(ix, iy);
    this.beginChunkWrite(c);
    this.costs[idx] = cost;
    this.endChunkWrite(c);
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
      this.beginChunkWrite(c);
      this.endChunkWrite(c);
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

  consumeDirtyState() {
    const topology = new Set(this.topologyDirtyChunks);
    const weights = new Set(this.weightsDirtyChunks);
    const all = new Set(this.dirtyChunks);
    this.dirtyChunks.clear();
    this.topologyDirtyChunks.clear();
    this.weightsDirtyChunks.clear();
    return { topology, weights, all };
  }

  ensureTerrainLayer() {
    if (!this.terrainIds) this.terrainIds = new Uint8Array(this._allocBuf(this.cellCount));
    return this.terrainIds;
  }

  ensureDynamicObstacleLayer() {
    if (!this.dynamicObstacles) this.dynamicObstacles = new Uint8Array(this._allocBuf(this.cellCount));
    this.hasDynamicObstacles = true;
    return this.dynamicObstacles;
  }

  ensureTacticalLayer() {
    if (!this.tacticalCosts) this.tacticalCosts = new Float32Array(this._allocBuf(this.cellCount * 4));
    this.hasTactical = true;
    return this.tacticalCosts;
  }

  ensureCongestionLayer(withFlux = false) {
    const len = this.cellCount;
    if (!this.congestion) this.congestion = new Float32Array(this._allocBuf(len * 4));
    if (withFlux && !this.congestionFluxX) {
      this.congestionFluxX = new Float32Array(this._allocBuf(len * 4));
      this.congestionFluxY = new Float32Array(this._allocBuf(len * 4));
    }
    this.hasCongestion = true;
    if (withFlux) this.hasCongestionFlux = true;
    return this.congestion;
  }

  ensureAnisotropicLayer() {
    const len = this.cellCount;
    if (!this.dirMask) this.dirMask = new Uint8Array(this._allocBuf(len)).fill(DIR_FLAGS.ALL);
    if (!this.currentX) {
      this.currentX = new Float32Array(this._allocBuf(len * 4));
      this.currentY = new Float32Array(this._allocBuf(len * 4));
    }
    this.hasAnisotropy = true;
  }

  /**
   * [4.1] Rasterizes a dynamic obstacle footprint (`{ x, y, w, h }` or `{ x, y, r }` or `{ cells }`)
   * into the reference-counted `dynamicObstacles` layer, only marking topology dirty on `0 <-> 1` transitions.
   */
  _rasterizeObstacleCells(shape) {
    const { width: W, height: H } = this;
    const cells = [];
    if (shape.cells) {
      for (let i = 0; i < shape.cells.length; i++) {
        const idx = shape.cells[i];
        if (idx >= 0 && idx < this.cellCount) cells.push(idx);
      }
    } else if (shape.r !== undefined) {
      const wx = shape.x, wy = shape.y, r = shape.r, r2 = r * r;
      const x0 = clamp(Math.floor(wx - r), 0, W - 1), x1 = clamp(Math.ceil(wx + r), 0, W - 1);
      const y0 = clamp(Math.floor(wy - r), 0, H - 1), y1 = clamp(Math.ceil(wy + r), 0, H - 1);
      for (let y = y0; y <= y1; y++) {
        for (let x = x0; x <= x1; x++) {
          if ((x + 0.5 - wx) ** 2 + (y + 0.5 - wy) ** 2 <= r2) cells.push(y * W + x);
        }
      }
    } else {
      const w = shape.w ?? 1, h = shape.h ?? 1;
      const x0 = clamp(Math.floor(shape.x), 0, W), x1 = clamp(Math.ceil(shape.x + w), 0, W);
      const y0 = clamp(Math.floor(shape.y), 0, H), y1 = clamp(Math.ceil(shape.y + h), 0, H);
      for (let y = y0; y < y1; y++) {
        for (let x = x0; x < x1; x++) cells.push(y * W + x);
      }
    }
    return new Int32Array(cells);
  }

  stampObstacle(shape) {
    const D = this.ensureDynamicObstacleLayer();
    const id = this._nextObstacleId++;
    const cells = this._rasterizeObstacleCells(shape);
    this._obstacleStamps.set(id, { shape: { ...shape }, cells });
    const W = this.width;
    for (let i = 0; i < cells.length; i++) {
      const idx = cells[i], prev = D[idx];
      if (prev < 255) D[idx] = prev + 1;
      if (prev === 0) {
        const x = idx % W, y = (idx / W) | 0;
        this.markCellDirty(x, y, true);
      }
    }
    return id;
  }

  unstampObstacle(id) {
    const entry = this._obstacleStamps.get(id);
    if (!entry || !this.dynamicObstacles) return false;
    const D = this.dynamicObstacles, { cells } = entry, W = this.width;
    for (let i = 0; i < cells.length; i++) {
      const idx = cells[i], prev = D[idx];
      if (prev > 0) {
        D[idx] = prev - 1;
        if (prev === 1) {
          const x = idx % W, y = (idx / W) | 0;
          this.markCellDirty(x, y, true);
        }
      }
    }
    this._obstacleStamps.delete(id);
    return true;
  }

  moveObstacle(id, newX, newY) {
    const entry = this._obstacleStamps.get(id);
    if (!entry) return false;
    if (entry.shape.x === newX && entry.shape.y === newY) return false;
    const D = this.ensureDynamicObstacleLayer(), W = this.width;
    const oldCells = entry.cells;
    entry.shape.x = newX;
    entry.shape.y = newY;
    const newCells = this._rasterizeObstacleCells(entry.shape);
    entry.cells = newCells;

    for (let i = 0; i < newCells.length; i++) {
      const idx = newCells[i], prev = D[idx];
      if (prev < 255) D[idx] = prev + 1;
      if (prev === 0) this.markCellDirty(idx % W, (idx / W) | 0, true);
    }
    for (let i = 0; i < oldCells.length; i++) {
      const idx = oldCells[i], prev = D[idx];
      if (prev > 0) {
        D[idx] = prev - 1;
        if (prev === 1) this.markCellDirty(idx % W, (idx / W) | 0, true);
      }
    }
    return true;
  }

  /**
   * [4.2] Splats agent positions and velocities into `congestion` & `congestionFluxX/Y`,
   * applies exponential decay, and marks chunks weight-dirty only when delta exceeds `dirtyThreshold`.
   */
  updateCrowdCongestion(agents, opts = {}) {
    const decay = opts.decay ?? 0.85;
    const splatWeight = opts.splatWeight ?? 2.0;
    const dirtyThreshold = opts.dirtyThreshold ?? 0.15;
    const withFlux = opts.withFlux !== false;
    const { width: W, height: H, cx, chunkCount } = this;

    const C = this.ensureCongestionLayer(withFlux);
    const FX = this.congestionFluxX, FY = this.congestionFluxY;
    const snap = this._chunkCongSnap, curSum = this._chunkCongNew;
    curSum.fill(0);

    // Decay pass
    for (let i = 0; i < this.cellCount; i++) {
      const v = C[i] * decay;
      C[i] = v < 0.01 ? 0 : v;
      if (withFlux) {
        FX[i] = C[i] > 0 ? FX[i] * decay : 0;
        FY[i] = C[i] > 0 ? FY[i] * decay : 0;
      }
    }

    // Bilinear agent splat pass
    for (let i = 0; i < agents.length; i++) {
      const a = agents[i];
      const lx = a.x - 0.5, ly = a.y - 0.5;
      const x0 = Math.floor(lx), y0 = Math.floor(ly);
      if (x0 < 0 || y0 < 0 || x0 >= W - 1 || y0 >= H - 1) continue;
      const tx = lx - x0, ty = ly - y0;
      const w = (a.weight ?? 1.0) * splatWeight;
      const vx = a.vx ?? 0, vy = a.vy ?? 0;

      const i00 = y0 * W + x0, i10 = i00 + 1, i01 = i00 + W, i11 = i01 + 1;
      const w00 = (1 - tx) * (1 - ty) * w, w10 = tx * (1 - ty) * w;
      const w01 = (1 - tx) * ty * w, w11 = tx * ty * w;

      C[i00] += w00; C[i10] += w10; C[i01] += w01; C[i11] += w11;
      if (withFlux && (vx !== 0 || vy !== 0)) {
        FX[i00] += vx * w00; FY[i00] += vy * w00;
        FX[i10] += vx * w10; FY[i10] += vy * w10;
        FX[i01] += vx * w01; FY[i01] += vy * w01;
        FX[i11] += vx * w11; FY[i11] += vy * w11;
      }
    }

    // Threshold-gated chunk weight dirtying
    for (let y = 0; y < H; y++) {
      const cyIdx = (y / CS | 0) * cx, row = y * W;
      for (let x = 0; x < W; x++) {
        curSum[cyIdx + (x / CS | 0)] += C[row + x];
      }
    }
    let marked = 0;
    const invCC = 1 / CC;
    for (let c = 0; c < chunkCount; c++) {
      const avg = curSum[c] * invCC;
      if (Math.abs(avg - snap[c]) >= dirtyThreshold || (avg === 0 && snap[c] > 0)) {
        snap[c] = avg;
        this.markChunkWeightDirty(c);
        marked++;
      }
    }
    return marked;
  }
  //#endregion

  //#region 9: Terrain ID Layer & Unit Movement Profiles
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
    if (!this.profileIds.has(name)) this.profileIds.set(name, this._nextProfileId++);
    this.profileIds.set(table, this.profileIds.get(name));
    return table;
  }

  setTerrain(x, y, terrainId) {
    const ix = x | 0, iy = y | 0;
    if (!this.inBounds(ix, iy)) return false;
    const T = this.ensureTerrainLayer(), idx = this.index(ix, iy);
    if (T[idx] === terrainId) return false;
    const c = this.chunkIdAt(ix, iy);
    this.beginChunkWrite(c);
    T[idx] = terrainId;
    this.endChunkWrite(c);
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

  //#region 10: [2.2] Exact Meijster Parabolic Envelope EDT for Wall Repulsion SDF
  enableWallRepulsion({ penaltyWeight = 1.5, repulsionWeight = 0.35, radius = 2.5 } = {}) {
    this.wallPenaltyWeight = penaltyWeight;
    this.wallRepulsionWeight = repulsionWeight;
    this.wallPenaltyRadius = radius;
    this._sdfFullDirty = true;
    this.updateWallSDF(true);
  }

  /**
   * [2.2] Exact O(N) 2-pass separable Meijster / Felzenszwalb Parabolic Envelope
   * Euclidean Distance Transform (EDT) & smooth central-difference gradient extraction.
   */
  updateWallSDF(forceFull = false) {
    const { width: W, height: H, cellCount: len, costs, _edtG: G, _edtV: V, _edtZ: Z } = this;
    if (!this.wallDist) {
      this.wallDist = new Float32Array(this._allocBuf(len * 4));
      this.wallGradX = new Float32Array(this._allocBuf(len * 4));
      this.wallGradY = new Float32Array(this._allocBuf(len * 4));
      this._sdfFullDirty = true;
    }
    const D = this.wallDist, GX = this.wallGradX, GY = this.wallGradY;
    const maxD = Math.ceil(this.wallPenaltyRadius) + 4;
    const maxD2 = maxD * maxD;
    const hasDyn = this.hasDynamicObstacles && this.dynamicObstacles;

    const clusters = this._buildDirtyClusters(this._sdfDirtyChunks, maxD, forceFull || this._sdfFullDirty);
    for (let c = 0; c < clusters.length; c++) {
      const { x0, y0, x1, y1 } = clusters[c];

      // Pass 1: 1D vertical squared distance transform along columns
      for (let x = x0; x <= x1; x++) {
        let d = maxD;
        for (let y = y0; y <= y1; y++) {
          const i = y * W + x;
          const blocked = costs[i] >= COST_BLOCK || (hasDyn && this.dynamicObstacles[i] > 0);
          d = blocked ? 0 : Math.min(maxD, d + 1);
          G[i] = d * d;
        }
        d = maxD;
        for (let y = y1; y >= y0; y--) {
          const i = y * W + x;
          if (G[i] === 0) {
            d = 0;
          } else {
            d = Math.min(maxD, d + 1);
            const d2 = d * d;
            if (d2 < G[i]) G[i] = d2;
          }
        }
      }

      // Pass 2: 1D horizontal lower parabolic envelope along rows
      for (let y = y0; y <= y1; y++) {
        const row = y * W;
        let k = 0;
        V[0] = x0;
        Z[0] = -1e12;
        Z[1] = 1e12;
        for (let q = x0 + 1; q <= x1; q++) {
          const gq = G[row + q] + q * q;
          let s;
          while (true) {
            const vk = V[k];
            s = (gq - (G[row + vk] + vk * vk)) / (2 * (q - vk));
            if (s > Z[k]) break;
            if (--k < 0) { k = 0; break; }
          }
          if (s <= Z[k]) {
            V[0] = q;
            Z[0] = -1e12;
            Z[1] = 1e12;
          } else {
            k++;
            V[k] = q;
            Z[k] = s;
            Z[k + 1] = 1e12;
          }
        }
        k = 0;
        for (let q = x0; q <= x1; q++) {
          while (Z[k + 1] < q) k++;
          const dx = q - V[k];
          const d2 = dx * dx + G[row + V[k]];
          D[row + q] = d2 >= maxD2 ? maxD : Math.sqrt(d2);
        }
      }

      // Pass 3: Smooth Euclidean gradient extraction
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
    }
    this._sdfDirty = this._sdfFullDirty = false;
    this._sdfDirtyChunks.clear();
  }
  //#endregion

  //#region 11: Isotropic & Directional Clearance Sweeps
  updateClearance(forceFull = false) {
    const { width: W, height: H, cellCount: len, costs } = this;
    if (!this.clearance) {
      this.clearance = new Uint8Array(this._allocBuf(len));
      this.clearanceX = new Uint8Array(this._allocBuf(len));
      this.clearanceY = new Uint8Array(this._allocBuf(len));
      this._clearanceFullDirty = true;
    }
    const C = this.clearance, CX = this.clearanceX, CY = this.clearanceY;
    const halo = MAX_CLEARANCE + 2;
    const hasDyn = this.hasDynamicObstacles && this.dynamicObstacles;

    const clusters = this._buildDirtyClusters(this._clearanceDirtyChunks, halo, forceFull || this._clearanceFullDirty);
    for (let c = 0; c < clusters.length; c++) {
      const { x0, y0, x1, y1 } = clusters[c];
      for (let y = y0; y <= y1; y++) {
        const row = y * W;
        for (let x = x0; x <= x1; x++) {
          const i = row + x;
          if (costs[i] >= COST_BLOCK || (hasDyn && this.dynamicObstacles[i] > 0)) {
            C[i] = CX[i] = CY[i] = 0;
          } else if (x === 0 || y === 0 || x === W - 1 || y === H - 1) {
            C[i] = CX[i] = CY[i] = 1;
          } else {
            C[i] = CX[i] = CY[i] = MAX_CLEARANCE;
          }
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
          if (CX[i - 1] + 1 < CX[i]) CX[i] = CX[i - 1] + 1;
          if (CY[i - W] + 1 < CY[i]) CY[i] = CY[i - W] + 1;
        }
      }
      for (let y = fy1; y >= fy0; y--) {
        const row = y * W;
        for (let x = fx1; x >= fx0; x--) {
          const i = row + x;
          if (!C[i]) continue;
          const m = Math.min(C[i + 1], C[i + W], C[i + W + 1], C[i + W - 1]);
          if (m + 1 < C[i]) C[i] = m + 1;
          if (CX[i + 1] + 1 < CX[i]) CX[i] = CX[i + 1] + 1;
          if (CY[i + W] + 1 < CY[i]) CY[i] = CY[i + W] + 1;
        }
      }
    }
    this._clearanceDirty = this._clearanceFullDirty = false;
    this._clearanceDirtyChunks.clear();
  }
  //#endregion

  //#region 12: Anisotropic Passability, Currents & Nearest Walkable
  setDirectionMask(x, y, mask = DIR_FLAGS.ALL) {
    const ix = x | 0, iy = y | 0;
    if (!this.inBounds(ix, iy)) return;
    this.ensureAnisotropicLayer();
    this.dirMask[this.index(ix, iy)] = mask & 0xFF;
    this.markCellDirty(ix, iy, true);
  }

  setCurrent(x, y, vx, vy) {
    const ix = x | 0, iy = y | 0;
    if (!this.inBounds(ix, iy)) return;
    this.ensureAnisotropicLayer();
    const idx = this.index(ix, iy);
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
        const idx = this.index(xx, yy);
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

  findNearestWalkable(x, y, maxR = 24, minClearance = 1, profile = null, footprint = null) {
    const { width: W, height: H } = this;
    const sx = clamp(x | 0, 0, W - 1), sy = clamp(y | 0, 0, H - 1);
    if (this.isWalkable(sx, sy, minClearance, profile, footprint)) return this.index(sx, sy);
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
            const idx = this.index(nx, ny), d = dx * dx + dy * dy;
            if (this.isWalkable(nx, ny, minClearance, profile, footprint) && d < bestD) { bestD = d; bestI = idx; }
          }
        }
      }
      if (bestI >= 0) return bestI;
    }
    return -1;
  }
  //#endregion
}
//#endregion

//#region 13: [1.1] Surgical Border Portal Detection & Crossing Cost Helpers
/**
 * Detects portals along a single chunk border `(isH, x, y, bIdx)`.
 * Used by [1.1] Surgical Chunk-Local Portal Patching to avoid full-map scans.
 */
export function detectBorderPortals(grid, isH, x, y, bIdx, minClearance = 1, maxPortalLen = MAX_PORTAL_LEN, profile = null, footprint = null) {
  const { cx, width } = grid;
  const hasDir = grid.hasAnisotropy && grid.dirMask;
  const border = ((isH ? y : x) + 1) * CS, origin = (isH ? x : y) * CS;
  const cA = y * cx + x, cB = cA + (isH ? cx : 1);
  const at = (u, v) => isH ? v * width + u : u * width + v;
  const dirAB = isH ? DIR_FLAGS.S : DIR_FLAGS.E;
  const dirBA = isH ? DIR_FLAGS.N : DIR_FLAGS.W;
  const W = (u, v) => grid.isWalkable(isH ? u : v, isH ? v : u, minClearance, profile, footprint);
  const out = [];

  const pushSegment = (segRun, segLen) => {
    const start = origin + segRun, mid = (segLen - 1) >> 1, cMid = start + mid;
    const cellsA = new Int32Array(segLen), cellsB = new Int32Array(segLen);
    let canAB = false, canBA = false;
    for (let k = 0; k < segLen; k++) {
      const ca = at(start + k, border - 1), cb = at(start + k, border);
      cellsA[k] = ca; cellsB[k] = cb;
      if (!hasDir || (grid.dirMask[ca] & dirAB)) canAB = true;
      if (!hasDir || (grid.dirMask[cb] & dirBA)) canBA = true;
    }
    if (!canAB && !canBA) return;
    out.push({
      id: 0,
      uid: (bIdx * CS + segRun) * (CS + 1) + segLen,
      bIdx, isH,
      chunkA: cA, chunkB: cB, border, start, end: start + segLen - 1,
      cellsA, cellsB, centerA: cellsA[mid], centerB: cellsB[mid],
      canAB, canBA,
      anchorX: isH ? cMid + 0.5 : border, anchorY: isH ? border : cMid + 0.5
    });
  };

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
        pushSegment(cur, segLen);
        cur += segLen;
      }
      run = -1;
    }
  }
  return out;
}

/**
 * Detects chunk boundary portals across the 2D grid, reusing clean border segments
 * when `prevBorderPortals` and `dirtyChunks` are provided ([1.1]).
 */
export function detectPortals(grid, minClearance = 1, maxPortalLen = MAX_PORTAL_LEN, profile = null, footprint = null, prevBorderPortals = null, dirtyChunks = null) {
  const { cx, cy } = grid;
  const totalBorders = cy * (cx - 1) + (cy - 1) * cx;
  const borderPortals = new Array(totalBorders);
  const portals = [];

  for (const isH of [false, true]) {
    const countX = cx - !isH, countY = cy - isH, bOffset = isH ? cy * (cx - 1) : 0;
    for (let y = 0; y < countY; y++) {
      for (let x = 0; x < countX; x++) {
        const cA = y * cx + x, cB = cA + (isH ? cx : 1);
        const bIdx = bOffset + y * countX + x;
        let segs;
        if (prevBorderPortals && dirtyChunks && !dirtyChunks.has(cA) && !dirtyChunks.has(cB)) {
          segs = prevBorderPortals[bIdx];
        } else {
          segs = detectBorderPortals(grid, isH, x, y, bIdx, minClearance, maxPortalLen, profile, footprint);
        }
        borderPortals[bIdx] = segs;
        for (let i = 0; i < segs.length; i++) {
          const p = segs[i];
          p.id = portals.length;
          portals.push(p);
        }
      }
    }
  }
  portals._borderPortals = borderPortals;
  return portals;
}

export const portalSideCells = (p, c) => c === p.chunkA ? p.cellsA : p.cellsB;
export const portalCenterCell = (p, c) => c === p.chunkA ? p.centerA : p.centerB;
export const portalExitX = (p, c) => p.isH ? 0 : c === p.chunkA ? 1 : -1;
export const portalExitY = (p, c) => p.isH ? c === p.chunkA ? 1 : -1 : 0;
export const pLocal = (grid, p, c) => grid.localIndex(portalCenterCell(p, c));

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

//#region 14: [2.1] Multi-Mode Eikonal Chunk Integrator (4-Pass Gauss-Seidel Raster FIM + Heap)
/**
 * Multi-mode chunk integrator supporting:
 * - [2.1] 4-Pass Gauss-Seidel Alternating Raster-Sweep Eikonal Solver (FIM) for isotropic chunks
 * - [5.2] Wait-free `Atomics` seqlock spin-read protection on `SharedArrayBuffer` chunks
 * - Anisotropic Rectangular Footprint (`opts.footprint = { width, length, turnPenalty }`)
 */
export class ChunkIntegrator {
  heap = new MinHeap(CC * 2);
  costs = new Float32Array(CC);
  blocked = new Uint8Array(CC);
  dirMasks = new Uint8Array(CC);
  turnPenaltyCell = new Float32Array(CC);
  curX = new Float32Array(CC);
  curY = new Float32Array(CC);
  fluxX = new Float32Array(CC);
  fluxY = new Float32Array(CC);
  dist = new Float32Array(CC);
  fleeBuf = new Float32Array(CC);
  blockedCount = 0;

  prepareChunk(grid, chunkId, minClearance = 1, profile = null, footprint = null) {
    const { costs, blocked, dirMasks, turnPenaltyCell, curX, curY, fluxX, fluxY } = this;
    const ox = grid.chunkOriginX(chunkId), oy = grid.chunkOriginY(chunkId);
    const W = grid.width;

    const fpW = footprint ? Math.min(footprint.width ?? 1, footprint.length ?? 1) : minClearance;
    const fpL = footprint ? Math.max(footprint.width ?? 1, footprint.length ?? 1) : fpW;
    const fpTurn = footprint ? (footprint.turnPenalty ?? 2.0) : 0;
    const useClearance = fpW > 1 || fpL > 1;
    if (useClearance && (grid._clearanceDirty || !grid.clearance)) grid.updateClearance();

    const useWall = grid.wallPenaltyWeight > 0;
    if (useWall && (grid._sdfDirty || !grid.wallDist)) grid.updateWallSDF();

    const pTable = grid.getProfileTable(profile);
    const hasTac = grid.hasTactical && grid.tacticalCosts;
    const hasCong = grid.hasCongestion && grid.congestion;
    const hasFlux = grid.hasCongestionFlux && grid.congestionFluxX;
    const hasAniso = grid.hasAnisotropy && grid.dirMask;
    const wallR = grid.wallPenaltyRadius, wallW = grid.wallPenaltyWeight;
    const useAtomicSeq = grid.useSharedMemory && typeof Atomics !== 'undefined';

    for (let attempt = 0; attempt < 4; attempt++) {
      const seq0 = useAtomicSeq ? Atomics.load(grid.chunkSeq, chunkId) : grid.chunkSeq[chunkId];
      if ((seq0 & 1) !== 0 && attempt < 3) continue;

      let blkCnt = 0;
      for (let ly = 0; ly < CS; ly++) {
        const gy = oy + ly, row = gy * W + ox, lRow = ly * CS;
        for (let lx = 0; lx < CS; lx++) {
          const gIdx = row + lx, lIdx = lRow + lx;
          const baseC = grid.getEffectiveBaseCost(gIdx, pTable);
          if (baseC >= COST_BLOCK) {
            blocked[lIdx] = 1; costs[lIdx] = COST_BLOCK; blkCnt++; continue;
          }
          let orientMask = 0xFF;
          turnPenaltyCell[lIdx] = 0;
          if (useClearance) {
            const isoClr = grid.clearance[gIdx];
            if (isoClr < fpW) { blocked[lIdx] = 1; costs[lIdx] = COST_BLOCK; blkCnt++; continue; }
            if (fpL > fpW) {
              const cxSpan = grid.clearanceX[gIdx], cySpan = grid.clearanceY[gIdx];
              if (cxSpan < fpL && cySpan < fpL) { blocked[lIdx] = 1; costs[lIdx] = COST_BLOCK; blkCnt++; continue; }
              if (isoClr < fpL) {
                turnPenaltyCell[lIdx] = fpTurn * (1 - isoClr / fpL);
                if (cxSpan < fpL) orientMask &= (DIR_FLAGS.N | DIR_FLAGS.S);
                else if (cySpan < fpL) orientMask &= (DIR_FLAGS.E | DIR_FLAGS.W);
              }
            }
          }
          blocked[lIdx] = 0;
          let eff = baseC + turnPenaltyCell[lIdx];
          if (hasTac) eff += grid.tacticalCosts[gIdx];
          if (hasCong) eff += grid.congestion[gIdx] * grid.congestionWeight;
          if (useWall) {
            const wd = grid.wallDist[gIdx];
            if (wd < wallR) eff += wallW * (1 - wd / wallR);
          }
          costs[lIdx] = Math.max(0.25, eff);
          if (hasAniso) {
            dirMasks[lIdx] = grid.dirMask[gIdx] & orientMask;
            curX[lIdx] = grid.currentX[gIdx];
            curY[lIdx] = grid.currentY[gIdx];
          } else {
            dirMasks[lIdx] = orientMask;
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
      this.blockedCount = blkCnt;
      const seq1 = useAtomicSeq ? Atomics.load(grid.chunkSeq, chunkId) : grid.chunkSeq[chunkId];
      if (seq0 === seq1) break;
    }
  }

  /**
   * [2.1] Fast 4-Pass Gauss-Seidel Alternating Raster Eikonal Solver for isotropic chunks.
   */
  _integrateRasterIsotropic(useEikonal, sourceOut) {
    const { costs, blocked, dist } = this;
    const maxCycles = this.blockedCount === 0 ? 1 : 3;

    for (let cycle = 0; cycle < maxCycles; cycle++) {
      let maxDelta = 0;
      for (let pass = 0; pass < 4; pass++) {
        const yStart = (pass & 1) ? CS - 1 : 0, yEnd = (pass & 1) ? -1 : CS, yStep = (pass & 1) ? -1 : 1;
        const xStart = (pass & 2) ? CS - 1 : 0, xEnd = (pass & 2) ? -1 : CS, xStep = (pass & 2) ? -1 : 1;

        for (let y = yStart; y !== yEnd; y += yStep) {
          const row = y * CS;
          for (let x = xStart; x !== xEnd; x += xStep) {
            const v = row + x;
            if (blocked[v]) continue;
            let bestD = dist[v], bestSrc = sourceOut ? sourceOut[v] : -1;
            const cv = costs[v];

            for (let d = 0; d < 8; d++) {
              const ux = x - DX[d], uy = y - DY[d];
              if (ux >>> 0 >= CS || uy >>> 0 >= CS) continue;
              const u = uy * CS + ux;
              const du = dist[u];
              if (du === Infinity) continue;
              if (d >= 4 && (blocked[uy * CS + x] || blocked[y * CS + ux])) continue;
              const stepScale = 0.25 * (costs[u] + cv);
              const nd = du + DL[d] * stepScale;
              if (nd < bestD) {
                bestD = nd;
                if (sourceOut) bestSrc = sourceOut[u];
              }
            }

            if (useEikonal) {
              const stepScale = 0.5 * cv;
              const tx = Math.min(x > 0 ? dist[v - 1] : Infinity, x < CS - 1 ? dist[v + 1] : Infinity);
              const ty = Math.min(y > 0 ? dist[v - CS] : Infinity, y < CS - 1 ? dist[v + CS] : Infinity);
              if (tx < Infinity || ty < Infinity) {
                const diff = Math.abs(tx - ty);
                let eik;
                if (diff < stepScale) {
                  eik = 0.5 * (tx + ty + Math.sqrt(2 * stepScale * stepScale - diff * diff));
                  if (eik < Math.max(tx, ty)) eik = Math.min(tx, ty) + stepScale;
                } else {
                  eik = Math.min(tx, ty) + stepScale;
                }
                if (eik < bestD) bestD = eik;
              }
            }

            const delta = dist[v] - bestD;
            if (delta > 0) {
              dist[v] = bestD;
              if (sourceOut) sourceOut[v] = bestSrc;
              if (delta > maxDelta) maxDelta = delta;
            }
          }
        }
      }
      if (maxDelta < 1e-4) return true;
    }
    return this.blockedCount === 0;
  }

  integrate(grid, chunkId, sources, out, opts = {}) {
    const minClearance = opts.minClearance ?? 1;
    const useEikonal = opts.eikonal ?? grid.useEikonal;
    const forward = !!opts.forward;
    const sourceOut = opts.sourceOut || null;
    const sourceWeights = opts.sourceWeights || null;
    const footprint = opts.footprint || null;

    this.prepareChunk(grid, chunkId, minClearance, opts.profile, footprint);
    const { costs, blocked, dirMasks, curX, curY, fluxX, fluxY, dist, heap } = this;
    const hasAniso = grid.hasAnisotropy || !!(footprint && (footprint.length !== footprint.width));
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
        dist[cell] = w;
        if (sourceOut) sourceOut[cell] = sid;
        if (hasAniso || hasFlux) heap.push(cell, w);
      }
    }

    // [2.1] Fast Gauss-Seidel Raster Eikonal solver for isotropic chunks
    if (!hasAniso && !hasFlux) {
      if (this._integrateRasterIsotropic(useEikonal, sourceOut)) {
        out.set(dist);
        return;
      }
      // If a complex maze inside the chunk didn't fully converge in 3 cycles, seed heap for final polish
      for (let i = 0; i < CC; i++) {
        if (dist[i] < Infinity) heap.push(i, dist[i]);
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

//#region 15: PortalGraph (Surgical CSR, Lazy SCC, ALT Landmarks & Entry Slope Bias)
/**
 * High-level abstract graph connecting chunk boundary portals in CSR format.
 * Includes:
 * - [1.1] Surgical per-border & per-chunk incremental CSR patching
 * - [1.2] Undirected fast-path bypassing Tarjan SCC when graph is symmetric
 * - [3.2] ALT Landmark Potentials (`landmarkDistTo` / `landmarkDistFrom`)
 * - [3.3] 3-Point Affine Portal Entry Slope Bias (`edgeEntryBias`)
 */
export class PortalGraph {
  constructor(grid, prevGraph = null, dirtyChunks = null, minClearance = 1, profile = null, footprint = null) {
    this.grid = grid;
    this.minClearance = minClearance;
    this.profile = profile;
    this.footprint = footprint;
    this.weightVersion = prevGraph ? prevGraph.weightVersion + 1 : 1;

    const sameFp = prevGraph && sameFootprint(prevGraph.footprint, footprint);
    const canReuse = !!(prevGraph && prevGraph.minClearance === minClearance && prevGraph.profile === profile && sameFp && dirtyChunks);

    // [1.1] Surgical border portal detection (only re-scans borders of dirtyChunks)
    const portals = this.portals = detectPortals(
      grid, minClearance, MAX_PORTAL_LEN, profile, footprint,
      canReuse ? prevGraph.borderPortals : null,
      canReuse ? dirtyChunks : null
    );
    this.borderPortals = portals._borderPortals;
    const P = portals.length;

    const lists = Array.from({ length: grid.chunkCount }, () => []);
    for (let i = 0; i < P; i++) {
      const p = portals[i];
      lists[p.chunkA].push(p.id);
      lists[p.chunkB].push(p.id);
    }
    const chunkPortals = this.chunkPortals = lists.map(l => new Int32Array(l));

    this.chunkDistCache = Array.from({ length: grid.chunkCount }, (_, c) =>
      canReuse && !dirtyChunks.has(c) ? prevGraph.chunkDistCache[c] : new Map()
    );
    this.chunkEdgeCache = new Array(grid.chunkCount);

    this.portalTraffic = new Float32Array(P);
    this.trafficWeight = 1.25;

    const adj = Array.from({ length: P }, () => []);
    const it = this._it = prevGraph ? prevGraph._it : new ChunkIntegrator();
    const field = this._field = prevGraph ? prevGraph._field : new Float32Array(CC);
    const itOpts = { minClearance, profile, footprint };

    for (let c = 0; c < grid.chunkCount; c++) {
      const ids = chunkPortals[c];
      if (ids.length < 2) {
        this.chunkEdgeCache[c] = [];
        continue;
      }
      // [1.1] Reuse cached intra-chunk edges if chunk and its border portals are untouched
      if (canReuse && !dirtyChunks.has(c) && prevGraph.chunkEdgeCache && prevGraph.chunkEdgeCache[c]) {
        const prevEdges = prevGraph.chunkEdgeCache[c];
        const remapped = [];
        for (let k = 0; k < prevEdges.length; k++) {
          const pe = prevEdges[k];
          const fromId = ids[pe.ib], toId = ids[pe.ia];
          const edgeObj = { ia: pe.ia, ib: pe.ib, from: fromId, to: toId, cost: pe.cost, bias: pe.bias, chunk: c };
          remapped.push(edgeObj);
          adj[fromId].push(edgeObj);
        }
        this.chunkEdgeCache[c] = remapped;
        continue;
      }

      const cCache = this.chunkDistCache[c];
      const cEdges = [];
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
            const bias = this._samplePortalEntryBias(row, pB, c);
            const edgeObj = { ia, ib, from: idB, to: idA, cost: d + crossA, bias, chunk: c };
            cEdges.push(edgeObj);
            adj[idB].push(edgeObj);
          }
        }
      }
      this.chunkEdgeCache[c] = cEdges;
    }

    const E = adj.reduce((acc, a) => acc + a.length, 0);
    this.edgeOffsets = new Int32Array(P + 1);
    this.edgeTargets = new Int32Array(E);
    this.baseEdgeCosts = new Float32Array(E);
    this.edgeCosts = new Float32Array(E);
    this.edgeEntryBias = new Float32Array(E);
    this.edgeChunks = new Int32Array(E);
    for (let p = 0, e = 0; p < P; this.edgeOffsets[++p] = e) {
      for (let k = 0; k < adj[p].length; k++) {
        const edge = adj[p][k];
        this.edgeTargets[e] = edge.to;
        this.baseEdgeCosts[e] = this.edgeCosts[e] = edge.cost;
        this.edgeEntryBias[e] = edge.bias;
        this.edgeChunks[e++] = edge.chunk;
      }
    }

    this._buildReverseAndComponents();
    this._buildSuperChunkHierarchy();
    this._buildLandmarks(NUM_LANDMARKS);
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

  /**
   * [3.3] Computes the affine entry slope bias along portal `pB` (`d(end) - d(start)`),
   * allowing A* to account for whether an agent enters near `start` (-1) or `end` (+1).
   */
  _samplePortalEntryBias(row, pB, chunk) {
    const cells = portalSideCells(pB, chunk);
    if (cells.length < 2) return 0;
    const { grid } = this;
    const d0 = row[grid.localIndex(cells[0])];
    const d1 = row[grid.localIndex(cells[cells.length - 1])];
    if (d0 === Infinity || d1 === Infinity) return 0;
    return clamp((d1 - d0) * 0.25, -4.0, 4.0);
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

    const comp = this.portalComp = new Int32Array(P).fill(-1);
    const queue = new Int32Array(P);
    let compId = 0;
    let hasOneWay = this.grid.hasAnisotropy;
    for (let p = 0; p < P; p++) {
      if (this.portals[p].canAB !== this.portals[p].canBA) hasOneWay = true;
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
    this.isDirected = hasOneWay;

    // [1.2] Undirected Fast-Path: Skip Tarjan SCC when graph has no one-way/anisotropic constraints
    if (!this.isDirected) {
      this.portalScc = comp;
      this.sccCount = compId;
      this.sccWords = 0;
      this.sccReachWords = null;
      this.sccReachLo = null;
      this.sccReachHi = null;
      return;
    }

    // Tarjan's Strongly Connected Components (SCC) for directed/anisotropic graphs
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
    const wordsPerScc = this.sccWords = Math.max(1, Math.ceil(sccCnt / 32));
    const reachWords = this.sccReachWords = new Uint32Array(sccCnt * wordsPerScc);
    const rLo = this.sccReachLo = new Uint32Array(sccCnt);
    const rHi = this.sccReachHi = new Uint32Array(sccCnt);

    const sccCounts = new Int32Array(sccCnt);
    for (let u = 0; u < P; u++) sccCounts[scc[u]]++;
    const sccOffsets = new Int32Array(sccCnt + 1);
    for (let s = 0; s < sccCnt; s++) sccOffsets[s + 1] = sccOffsets[s] + sccCounts[s];
    const sccMembers = new Int32Array(P);
    const sccCursor = new Int32Array(sccOffsets.subarray(0, sccCnt));
    for (let u = 0; u < P; u++) sccMembers[sccCursor[scc[u]]++] = u;

    for (let s = 0; s < sccCnt; s++) {
      const base = s * wordsPerScc;
      reachWords[base + (s >>> 5)] |= (1 << (s & 31));
      const bit64 = s & 63;
      if (bit64 < 32) rLo[s] |= (1 << bit64);
      else rHi[s] |= (1 << (bit64 - 32));

      for (let idx = sccOffsets[s]; idx < sccOffsets[s + 1]; idx++) {
        const u = sccMembers[idx];
        for (let e = this.edgeOffsets[u]; e < this.edgeOffsets[u + 1]; e++) {
          const sv = scc[this.edgeTargets[e]];
          if (sv === s) continue;
          const vBase = sv * wordsPerScc;
          for (let w = 0; w < wordsPerScc; w++) {
            reachWords[base + w] |= reachWords[vBase + w];
          }
          rLo[s] |= rLo[sv];
          rHi[s] |= rHi[sv];
        }
      }
    }
  }

  /** [1.2] Checks if `fromScc` can reach any SCC in `goalSccWords` (with O(1) undirected fast-path) */
  canSccReach(fromScc, goalSccWords, sccMaskLo = 0, sccMaskHi = 0) {
    if (!this.isDirected) return true;
    if (((this.sccReachLo[fromScc] & sccMaskLo) === 0) && ((this.sccReachHi[fromScc] & sccMaskHi) === 0)) {
      return false;
    }
    if (!goalSccWords) return true;
    const words = this.sccWords, base = fromScc * words;
    for (let w = 0; w < words; w++) {
      if ((this.sccReachWords[base + w] & goalSccWords[w]) !== 0) return true;
    }
    return false;
  }

  _buildSuperChunkHierarchy() {
    const { grid, portals } = this;
    const scx = this.scx = Math.ceil(grid.cx / SUPER_STEP);
    const scy = this.scy = Math.ceil(grid.cy / SUPER_STEP);
    const S = this.superChunkCount = scx * scy;
    const chunkToSuper = this.chunkToSuper = new Int32Array(grid.chunkCount);

    for (let c = 0; c < grid.chunkCount; c++) {
      const cx = c % grid.cx, cy = (c / grid.cx) | 0;
      chunkToSuper[c] = ((cy / SUPER_STEP) | 0) * scx + ((cx / SUPER_STEP) | 0);
    }

    const sAdj = Array.from({ length: S }, () => new Map());
    for (let i = 0; i < portals.length; i++) {
      const p = portals[i];
      const sA = chunkToSuper[p.chunkA], sB = chunkToSuper[p.chunkB];
      if (sA === sB) continue;
      const w = (SUPER_STEP * CS) * 0.5;
      if (p.canAB !== false) {
        const prev = sAdj[sA].get(sB);
        if (prev === undefined || w < prev) sAdj[sA].set(sB, w);
      }
      if (p.canBA !== false) {
        const prev = sAdj[sB].get(sA);
        if (prev === undefined || w < prev) sAdj[sB].set(sA, w);
      }
    }
    this.superAdj = sAdj;
  }

  /**
   * [3.2] Builds ALT (A*, Landmarks, Triangle Inequality) heuristic tables using
   * farthest-point landmark selection over `baseEdgeCosts`.
   */
  _buildLandmarks(maxLandmarks = NUM_LANDMARKS) {
    const P = this.portals.length;
    if (P < 16 || maxLandmarks <= 0) {
      this.numLandmarks = 0;
      this.landmarkDistTo = null;
      this.landmarkDistFrom = null;
      return;
    }
    const K = this.numLandmarks = Math.min(maxLandmarks, P);
    const LTo = this.landmarkDistTo = new Float32Array(K * P).fill(Infinity);
    const LFrom = this.landmarkDistFrom = new Float32Array(K * P).fill(Infinity);
    const minDist = new Float32Array(P).fill(Infinity);
    const heap = this._it.heap;
    const landmarks = new Int32Array(K);

    let nextL = 0;
    for (let k = 0; k < K; k++) {
      landmarks[k] = nextL;
      const off = k * P;

      // Reverse Dijkstra: shortest distance from every portal u TO landmark k
      LTo[off + nextL] = 0;
      heap.clear();
      heap.push(nextL, 0);
      while (!heap.isEmpty) {
        const v = heap.pop(), dv = heap.lastKey;
        if (dv > LTo[off + v]) continue;
        for (let r = this.revOffsets[v]; r < this.revOffsets[v + 1]; r++) {
          const u = this.revSources[r], e = this.revEdgeIdx[r];
          const nd = dv + this.baseEdgeCosts[e];
          if (nd < LTo[off + u]) {
            LTo[off + u] = nd;
            heap.push(u, nd);
          }
        }
      }

      // Forward Dijkstra: shortest distance FROM landmark k to every portal v
      if (!this.isDirected) {
        LFrom.subarray(off, off + P).set(LTo.subarray(off, off + P));
      } else {
        LFrom[off + nextL] = 0;
        heap.clear();
        heap.push(nextL, 0);
        while (!heap.isEmpty) {
          const u = heap.pop(), du = heap.lastKey;
          if (du > LFrom[off + u]) continue;
          for (let e = this.edgeOffsets[u]; e < this.edgeOffsets[u + 1]; e++) {
            const v = this.edgeTargets[e];
            const nd = du + this.baseEdgeCosts[e];
            if (nd < LFrom[off + v]) {
              LFrom[off + v] = nd;
              heap.push(v, nd);
            }
          }
        }
      }

      // Pick farthest reachable portal as next landmark
      let bestP = (nextL + Math.floor(P / K)) % P, bestD = -1;
      for (let p = 0; p < P; p++) {
        const d = LTo[off + p];
        if (d < minDist[p]) minDist[p] = d;
        if (minDist[p] < Infinity && minDist[p] > bestD) {
          bestD = minDist[p];
          bestP = p;
        }
      }
      nextL = bestP;
    }
    this.landmarks = landmarks;
  }

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
    const { grid, portals, chunkPortals, minClearance, profile, footprint, _it: it, _field: field } = this;
    const itOpts = { minClearance, profile, footprint };
    let weightDecreased = false;

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
          const newBase = d + crossA;
          if (newBase < this.baseEdgeCosts[e] - 1e-3) weightDecreased = true;
          this.baseEdgeCosts[e] = newBase;
          this.edgeCosts[e] = newBase + this.portalTraffic[idA] * this.trafficWeight;
          this.edgeEntryBias[e] = row ? this._samplePortalEntryBias(row, pB, c) : 0;
        }
      }
    }
    if (weightDecreased && this.numLandmarks > 0) this._buildLandmarks(this.numLandmarks);
    this.weightVersion++;
  }
}
//#endregion

//#region 16: MacroPathfinder (ALT A*, Affine Entry Bias, Waypoint Extractor & Swarm Repair)
/**
 * Hierarchical A*, ALT Landmark Heuristic, Funnel String-Puller, Any-Angle Waypoint Extractor
 * & Incremental Swarm Broadcast Pathfinder operating over `PortalGraph`.
 */
export class MacroPathfinder {
  constructor(grid, graph, goalCacheCap = 256) {
    this.grid = grid; this.graph = graph;
    this.goalCacheCap = goalCacheCap;
    this.goalCache = new Map();
    const N = graph.portals.length + 2;
    this.g = new Float64Array(N); this.parent = new Int32Array(N);
    this.parentChunk = new Int32Array(N); this.seen = new Uint32Array(N);
    this.closed = new Uint32Array(N);
    this.entryPos = new Float32Array(N);
    this.it = new ChunkIntegrator();
    this.startField = new Float32Array(CC); this.heap = new MinHeap(256); this.stamp = 0;
    this._swarmIdCounter = 1;
    this._superMask = new Uint8Array(graph.superChunkCount);
  }

  invalidateChunks(dirtyChunks) {
    for (const [cell, goal] of this.goalCache) {
      if (dirtyChunks.has(goal.chunk)) this.goalCache.delete(cell);
    }
  }

  _computeGoalLandmarks(portalCost) {
    const { graph } = this;
    const K = graph.numLandmarks, P = graph.portals.length;
    if (!K) return { lTo: null, lFrom: null };
    const lTo = new Float32Array(K).fill(Infinity);
    const lFrom = new Float32Array(K).fill(Infinity);
    const LTo = graph.landmarkDistTo, LFrom = graph.landmarkDistFrom;
    for (let i = 0; i < P; i++) {
      const pc = portalCost[i];
      if (pc === Infinity) continue;
      for (let k = 0; k < K; k++) {
        const off = k * P + i;
        if (LTo[off] + pc < lTo[k]) lTo[k] = LTo[off] + pc;
        if (LFrom[off] + pc < lFrom[k]) lFrom[k] = LFrom[off] + pc;
      }
    }
    return { lTo, lFrom };
  }

  createGoal(cell) {
    const { grid, graph } = this;
    const x = cell % grid.width, y = (cell / grid.width) | 0;
    const chunk = grid.chunkIdAt(x, y), field = new Float32Array(CC);
    this.it.integrate(grid, chunk, [grid.localIndex(cell)], field, {
      minClearance: graph.minClearance,
      profile: graph.profile,
      footprint: graph.footprint
    });
    const portalCost = new Float32Array(graph.portals.length).fill(Infinity);
    const cPortals = graph.chunkPortals[chunk];
    const compSet = new Set();
    const sccWords = graph.isDirected ? new Uint32Array(graph.sccWords) : null;
    let sccMaskLo = 0, sccMaskHi = 0;
    for (let i = 0; i < cPortals.length; i++) {
      const pid = cPortals[i], p = graph.portals[pid];
      const d = graph._samplePortalRowDist(field, p, chunk);
      if (d < Infinity) {
        portalCost[pid] = d;
        compSet.add(graph.portalComp[pid]);
        if (graph.isDirected) {
          const s = graph.portalScc[pid];
          sccWords[s >>> 5] |= (1 << (s & 31));
          const bit = s & 63;
          if (bit < 32) sccMaskLo |= (1 << bit);
          else sccMaskHi |= (1 << (bit - 32));
        }
      }
    }
    const { lTo, lFrom } = this._computeGoalLandmarks(portalCost);
    return { isMulti: false, cell, x, y, chunk, field, portalCost, compSet, sccWords, sccMaskLo, sccMaskHi, lTo, lFrom, graph };
  }

  createMultiGoal(targets) {
    const { grid, graph } = this;
    const portalCost = new Float32Array(graph.portals.length).fill(Infinity);
    const portalGoalCell = new Int32Array(graph.portals.length).fill(-1);
    const chunkGroups = new Map();
    const goalList = [];
    const compSet = new Set();
    const sccWords = graph.isDirected ? new Uint32Array(graph.sccWords) : null;
    let sccMaskLo = 0, sccMaskHi = 0;

    for (let i = 0; i < targets.length; i++) {
      const t = targets[i];
      const cell = typeof t === 'object' ? (t.cell ?? grid.index(t.x | 0, t.y | 0)) : t;
      const weight = typeof t === 'object' ? (t.weight ?? 0) : 0;
      if (cell < 0 || cell >= grid.cellCount) continue;
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
        footprint: graph.footprint,
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
          if (graph.isDirected) {
            const s = graph.portalScc[pid];
            sccWords[s >>> 5] |= (1 << (s & 31));
            const bit = s & 63;
            if (bit < 32) sccMaskLo |= (1 << bit);
            else sccMaskHi |= (1 << (bit - 32));
          }
        }
      }
    }

    const primary = goalList[0] || { cell: 0, x: 0, y: 0, chunk: 0 };
    const { lTo, lFrom } = this._computeGoalLandmarks(portalCost);
    return {
      isMulti: true,
      cell: primary.cell, x: primary.x, y: primary.y, chunk: primary.chunk,
      goals: goalList, chunkFields, portalCost, portalGoalCell,
      compSet, sccWords, sccMaskLo, sccMaskHi, lTo, lFrom, graph
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

  _computeSuperCorridorMask(startSuper, goalSuper) {
    const { graph, _superMask: mask } = this;
    const S = graph.superChunkCount, scx = graph.scx, scy = graph.scy;
    mask.fill(0);
    const dist = new Float32Array(S).fill(Infinity);
    const parent = new Int32Array(S).fill(-1);
    const heap = this.heap;
    heap.clear();
    dist[startSuper] = 0;
    heap.push(startSuper, 0);

    const gx = goalSuper % scx, gy = (goalSuper / scx) | 0;
    while (!heap.isEmpty) {
      const u = heap.pop();
      if (u === goalSuper) break;
      const du = dist[u];
      for (const [v, w] of graph.superAdj[u]) {
        const nd = du + w;
        if (nd < dist[v]) {
          dist[v] = nd;
          parent[v] = u;
          const vx = v % scx, vy = (v / scx) | 0;
          heap.push(v, nd + octile(vx - gx, vy - gy) * (SUPER_STEP * CS * 0.5));
        }
      }
    }
    if (dist[goalSuper] === Infinity) return null;

    for (let cur = goalSuper; cur !== -1; cur = parent[cur]) {
      const x = cur % scx, y = (cur / scx) | 0;
      for (let dy = -1; dy <= 1; dy++) {
        for (let dx = -1; dx <= 1; dx++) {
          const nx = x + dx, ny = y + dy;
          if (nx >= 0 && nx < scx && ny >= 0 && ny < scy) {
            mask[ny * scx + nx] = 1;
          }
        }
      }
    }
    return mask;
  }

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

  /**
   * [3.1] Extracts a minimal, any-angle world-LOS string-pulled waypoint polyline
   * `[{ x, y }, ...]` from a `path` with zero flow-field allocations.
   */
  extractWaypoints(path, sx, sy) {
    if (!path) return null;
    const { grid } = this;
    const clr = path.minClearance ?? this.graph.minClearance;
    const prof = path.profile ?? this.graph.profile;
    const fp = path.footprint ?? this.graph.footprint;

    const raw = [{ x: sx, y: sy }];
    const fx = path.funnelX, fy = path.funnelY;
    for (let i = 0; i < fx.length; i++) {
      const px = fx[i], py = fy[i], prev = raw[raw.length - 1];
      if (Math.hypot(px - prev.x, py - prev.y) > 0.15) {
        raw.push({ x: px, y: py });
      }
    }
    const last = raw[raw.length - 1];
    if (Math.hypot(path.goalX - last.x, path.goalY - last.y) > 0.05) {
      raw.push({ x: path.goalX, y: path.goalY });
    }
    if (raw.length <= 2) return raw;

    // Greedy any-angle supercover world-LOS string pulling
    const out = [raw[0]];
    let cur = 0;
    while (cur < raw.length - 1) {
      let next = cur + 1;
      for (let probe = raw.length - 1; probe > cur + 1; probe--) {
        if (hasWorldLOS(grid, raw[cur].x, raw[cur].y, raw[probe].x, raw[probe].y, clr, prof, fp)) {
          next = probe;
          break;
        }
      }
      out.push(raw[next]);
      cur = next;
    }
    return out;
  }

  findPath(startCell, goalOrCell, useSuperCorridor = true) {
    const goal = typeof goalOrCell === 'number'
      ? this.getOrCreateGoal(goalOrCell)
      : (Array.isArray(goalOrCell) ? this.createMultiGoal(goalOrCell) : goalOrCell);
    if (!goal) return null;
    const { grid, graph, g, parent, parentChunk, seen, closed, entryPos, heap } = this;
    const P = graph.portals.length, START = P, GOAL = P + 1;
    const sx = startCell % grid.width, sy = (startCell / grid.width) | 0, sc = grid.chunkIdAt(sx, sy);
    const gx = goal.x + 0.5, gy = goal.y + 0.5;

    this.it.integrate(grid, sc, [grid.localIndex(startCell)], this.startField, {
      minClearance: graph.minClearance,
      profile: graph.profile,
      footprint: graph.footprint,
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
      for (let i = 0; i < cPorts.length; i++) {
        const pid = cPorts[i], p = graph.portals[pid];
        if (this.startField[pLocal(grid, p, sc)] < Infinity && crossingCost(grid, p, sc, graph.profile) < Infinity) {
          if (!goal.compSet.has(graph.portalComp[pid])) continue;
          if (graph.canSccReach(graph.portalScc[pid], goal.sccWords, goal.sccMaskLo, goal.sccMaskHi)) {
            reachableMatch = true;
            break;
          }
        }
      }
      if (!reachableMatch) return null;
    }

    let superMask = null;
    if (useSuperCorridor && !sameChunkDirect && !goal.isMulti && graph.superChunkCount >= 9 && !graph.numLandmarks) {
      const sSuper = graph.chunkToSuper[sc], gSuper = graph.chunkToSuper[goal.chunk];
      const sDist = Math.abs((sSuper % graph.scx) - (gSuper % graph.scx)) + Math.abs(((sSuper / graph.scx) | 0) - ((gSuper / graph.scx) | 0));
      if (sDist >= 3) {
        superMask = this._computeSuperCorridorMask(sSuper, gSuper);
      }
    }

    const K = graph.numLandmarks;
    const LTo = graph.landmarkDistTo, LFrom = graph.landmarkDistFrom;
    const gLTo = goal.lTo, gLFrom = goal.lFrom;

    const runSearch = (corridorMask) => {
      if (++this.stamp >= 0xffffffff) { seen.fill(0); closed.fill(0); this.stamp = 1; }
      const s = this.stamp;

      const heuristic = (u, px, py) => {
        let h;
        if (!goal.isMulti) {
          h = Math.max(0, octile(px - gx, py - gy) - 0.5);
        } else {
          h = Infinity;
          for (let i = 0; i < goal.goals.length; i++) {
            const tg = goal.goals[i];
            const d = Math.max(0, octile(px - (tg.x + 0.5), py - (tg.y + 0.5)) - 0.5) + tg.weight;
            if (d < h) h = d;
          }
        }
        // [3.2] ALT Landmark Triangle Inequality Boost
        if (u < P && K > 0 && gLTo) {
          for (let k = 0; k < K; k++) {
            const off = k * P + u;
            const d1 = LTo[off] - gLTo[k];
            const d2 = gLFrom[k] - LFrom[off];
            if (d1 > h && d1 < 1e9) h = d1;
            if (d2 > h && d2 < 1e9) h = d2;
          }
        }
        return h;
      };

      heap.clear(); g[START] = 0; seen[START] = s; parent[START] = -1; parentChunk[START] = sc; entryPos[START] = 0;
      heap.push(START, heuristic(START, sx + 0.5, sy + 0.5));

      let chosenGoalCell = goal.cell;
      const relax = (v, cost, from, chunk, biasPos = 0, resolvedCell = -1) => {
        if (corridorMask && v < P && !corridorMask[graph.chunkToSuper[chunk]]) return;
        if (closed[v] === s || (seen[v] === s && cost >= g[v])) return;
        seen[v] = s; g[v] = cost; parent[v] = from; parentChunk[v] = chunk; entryPos[v] = biasPos;
        if (v === GOAL && resolvedCell >= 0) chosenGoalCell = resolvedCell;
        const h = v < P ? heuristic(v, graph.portals[v].anchorX, graph.portals[v].anchorY) : 0;
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
            portals: pArr, chunks: cArr, funnelX, funnelY,
            cost: g[GOAL], goalCell: chosenGoalCell, goalX: fgx, goalY: fgy,
            minClearance: graph.minClearance, profile: graph.profile, footprint: graph.footprint, graph
          };
        }
        const gu = g[u];
        if (u === START) {
          for (let i = 0; i < cPorts.length; i++) {
            const pid = cPorts[i], p = graph.portals[pid];
            const d = graph._samplePortalRowDist(this.startField, p, sc);
            const cross = crossingCost(grid, p, sc, graph.profile);
            if (d < Infinity && cross < Infinity) {
              const bPos = Math.sign(graph._samplePortalEntryBias(this.startField, p, sc));
              relax(pid, gu + d + cross, START, sc, bPos);
            }
          }
          if (!goal.isMulti && sc === goal.chunk) {
            const d = goal.field[grid.localIndex(startCell)];
            if (d < Infinity) relax(GOAL, gu + d, START, sc, 0, goal.cell);
          } else if (goal.isMulti && goal.chunkFields.has(sc)) {
            const cf = goal.chunkFields.get(sc), loc = grid.localIndex(startCell);
            const d = cf.field[loc];
            if (d < Infinity) relax(GOAL, gu + d, START, sc, 0, cf.sourceOut[loc]);
          }
        } else {
          const uPos = entryPos[u];
          for (let e = graph.edgeOffsets[u]; e < graph.edgeOffsets[u + 1]; e++) {
            const v = graph.edgeTargets[e];
            const eBias = graph.edgeEntryBias[e];
            // [3.3] Affine portal entry bias adjustment
            const biasAdj = uPos !== 0 && eBias !== 0 ? clamp(uPos * eBias * 0.25, -1.5, 1.5) : 0;
            const stepCost = Math.max(0.5, graph.edgeCosts[e] + biasAdj);
            relax(v, gu + stepCost, u, graph.edgeChunks[e], Math.sign(eBias));
          }
          const gc = goal.portalCost[u];
          if (gc < Infinity) {
            const rCell = goal.isMulti ? goal.portalGoalCell[u] : goal.cell;
            const rChunk = grid.chunkIdAt(rCell % grid.width, (rCell / grid.width) | 0);
            relax(GOAL, gu + gc, u, rChunk, 0, rCell);
          }
        }
      }
      return null;
    };

    const res = runSearch(superMask);
    return (res || !superMask) ? res : runSearch(null);
  }

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
      targets,
      goal,
      goalX: goal.x + 0.5,
      goalY: goal.y + 0.5,
      portalDist,
      portalNext,
      goalChunks,
      minClearance: graph.minClearance,
      profile: graph.profile,
      footprint: graph.footprint,
      weightVersion: graph.weightVersion,
      graph
    };
  }

  /**
   * [1.3] Incrementally repairs an existing `SwarmField` after weight or topology changes
   * by resetting only the downstream subtree of affected portals and re-running Dijkstra on the repair cone.
   */
  updateSwarmField(swarm, dirtyChunks = null) {
    if (!swarm) return null;
    const { graph, grid, heap } = this;

    // If topology changed (new graph instance or portal count changed), re-bind and repair
    if (swarm.graph !== graph || !dirtyChunks || dirtyChunks.size === 0) {
      const fresh = this.createSwarmField(swarm.targets ?? swarm.goal.cell);
      Object.assign(swarm, fresh, { uid: swarm.uid });
      return swarm;
    }

    const P = graph.portals.length;
    const { portalDist, portalNext, goal } = swarm;
    const affected = new Uint8Array(P);
    const queue = [];

    // Re-integrate goal chunks if any were dirty
    if (!goal.isMulti && dirtyChunks.has(goal.chunk)) {
      const updatedGoal = this.createGoal(goal.cell);
      swarm.goal = updatedGoal;
      for (const pid of graph.chunkPortals[goal.chunk]) {
        affected[pid] = 1;
        queue.push(pid);
      }
    } else if (goal.isMulti) {
      for (const c of dirtyChunks) {
        if (goal.chunkFields.has(c)) {
          const updatedGoal = this.createMultiGoal(swarm.targets);
          swarm.goal = updatedGoal;
          break;
        }
      }
    }

    // Mark all portals touching dirtyChunks
    for (const c of dirtyChunks) {
      const cPorts = graph.chunkPortals[c];
      for (let i = 0; i < cPorts.length; i++) {
        const pid = cPorts[i];
        if (!affected[pid]) {
          affected[pid] = 1;
          queue.push(pid);
        }
      }
    }

    // Propagate invalidation upstream along `portalNext` subtree
    let head = 0;
    while (head < queue.length) {
      const v = queue[head++];
      portalDist[v] = Infinity;
      portalNext[v] = -1;
      for (let r = graph.revOffsets[v]; r < graph.revOffsets[v + 1]; r++) {
        const u = graph.revSources[r];
        if (portalNext[u] === v && !affected[u]) {
          affected[u] = 1;
          queue.push(u);
        }
      }
    }

    // Re-seed affected portals from goal costs and unaffected neighbors
    heap.clear();
    const gCost = swarm.goal.portalCost;
    for (let i = 0; i < queue.length; i++) {
      const u = queue[i];
      let bestD = gCost[u], bestNext = bestD < Infinity ? -2 : -1;
      for (let e = graph.edgeOffsets[u]; e < graph.edgeOffsets[u + 1]; e++) {
        const v = graph.edgeTargets[e];
        if (!affected[v] && portalDist[v] < Infinity) {
          const nd = portalDist[v] + graph.edgeCosts[e];
          if (nd < bestD) { bestD = nd; bestNext = v; }
        }
      }
      if (bestD < Infinity) {
        portalDist[u] = bestD;
        portalNext[u] = bestNext;
        heap.push(u, bestD);
      }
    }

    // Also allow unaffected portals bordering dirty chunks to improve if weights decreased
    for (const c of dirtyChunks) {
      const cPorts = graph.chunkPortals[c];
      for (let i = 0; i < cPorts.length; i++) {
        const v = cPorts[i];
        if (portalDist[v] < Infinity) heap.push(v, portalDist[v]);
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

    swarm.weightVersion = graph.weightVersion;
    return swarm;
  }
}
//#endregion

//#region 17: Chunk/World Supercover LOS & Flow Vector Gradient Extraction
/**
 * Fast chunk-local supercover Line-of-Sight check.
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
 * [3.1] World-space supercover Line-of-Sight check respecting `minClearance`,
 * terrain `profile`, `footprint`, and uniform terrain traversal cost.
 */
export function hasWorldLOS(grid, x0, y0, x1, y1, minClearance = 1, profile = null, footprint = null) {
  let cx = clamp(x0 | 0, 0, grid.width - 1), cy = clamp(y0 | 0, 0, grid.height - 1);
  const ex = clamp(x1 | 0, 0, grid.width - 1), ey = clamp(y1 | 0, 0, grid.height - 1);
  const dx = Math.abs(ex - cx), dy = Math.abs(ey - cy);
  const sx = cx < ex ? 1 : -1, sy = cy < ey ? 1 : -1;
  const pTable = grid.getProfileTable(profile);
  const refCost = Math.max(
    grid.getEffectiveBaseCost(grid.index(cx, cy), pTable),
    grid.getEffectiveBaseCost(grid.index(ex, ey), pTable)
  );
  if (refCost >= COST_BLOCK) return false;

  let err = dx - dy;
  while (true) {
    if (!grid.isWalkable(cx, cy, minClearance, profile, footprint)) return false;
    if (grid.getEffectiveBaseCost(grid.index(cx, cy), pTable) > refCost) return false;
    if (cx === ex && cy === ey) return true;
    const e2 = 2 * err;
    if (e2 > -dy && e2 < dx) {
      if (!grid.isWalkable(cx + sx, cy, minClearance, profile, footprint) ||
          !grid.isWalkable(cx, cy + sy, minClearance, profile, footprint)) return false;
    }
    if (e2 > -dy) { err -= dy; cx += sx; }
    if (e2 < dx) { err += dx; cy += sy; }
  }
}

export function computeFlowVectors(f, exX, exY, opts = {}) {
  const { flowX: FX, flowY: FY, quantized } = f;
  const rawI = opts.rawIntegration || f.integration;
  const at = (x, y) => x >>> 0 >= CS || y >>> 0 >= CS ? Infinity : rawI[y * CS + x];
  const grid = opts.grid || null;
  const useWallRep = grid && grid.wallRepulsionWeight > 0 && grid.wallDist;
  const vortex = opts.vortex || f.vortex || null;
  const hasLOS = opts.losSmoothing && opts.chunkCosts && opts.targetLocalX !== undefined;
  const txL = opts.targetLocalX, tyL = opts.targetLocalY, cCosts = opts.chunkCosts;
  const refCost = hasLOS ? cCosts[clamp(tyL | 0, 0, CS - 1) * CS + clamp(txL | 0, 0, CS - 1)] : COST_OPEN;
  const borderExitMask = opts.borderExitMask || null;
  const borderExX = opts.borderExX || null;
  const borderExY = opts.borderExY || null;
  const qScale = quantized ? 127 : 1;

  for (let y = 0; y < CS; y++) {
    for (let x = 0; x < CS; x++) {
      const id = y * CS + x, cur = rawI[id];
      if (cur === Infinity) { FX[id] = FY[id] = 0; continue; }

      const cellExX = borderExX && borderExitMask && borderExitMask[id] ? borderExX[id] : exX;
      const cellExY = borderExY && borderExitMask && borderExitMask[id] ? borderExY[id] : exY;

      if ((!cur || (borderExitMask && borderExitMask[id])) && !vortex && (cellExX || cellExY)) {
        const E = at(x + 1, y), W = at(x - 1, y), S = at(x, y + 1), N = at(x, y - 1);
        const gx = cellExX !== 0 ? 0 : (E < W ? (E < cur ? E - cur : 0) : W < cur ? cur - W : 0);
        const gy = cellExY !== 0 ? 0 : (S < N ? (S < cur ? S - cur : 0) : N < cur ? cur - N : 0);
        const vx = cellExX - gx * 0.45, vy = cellExY - gy * 0.45, vl = Math.hypot(vx, vy) || 1;
        FX[id] = quantized ? Math.round((vx / vl) * qScale) : (vx / vl);
        FY[id] = quantized ? Math.round((vy / vl) * qScale) : (vy / vl);
        continue;
      }
      if (!cur && !vortex) {
        FX[id] = quantized ? Math.round(cellExX * qScale) : cellExX;
        FY[id] = quantized ? Math.round(cellExY * qScale) : cellExY;
        continue;
      }

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

      FX[id] = quantized ? Math.round(dx * qScale) : dx;
      FY[id] = quantized ? Math.round(dy * qScale) : dy;
    }
  }
}
//#endregion

//#region 18: FlowFieldCache ([2.3] Funnel Portal Seeding & [5.1] Quantized Field Storage)
/**
 * Collision-proof LRU cache of per-chunk flow fields backed by `FieldBufferPool`.
 * Supports:
 * - [2.3] Funnel-Projected Continuous Portal Entry Seeding in `getPortalField`
 * - [5.1] Optional Quantized `Int8Array`/`Uint16Array` field buffers (`opts.quantizeFields`)
 */
export class FlowFieldCache {
  constructor(grid, cap = 2048, opts = {}) {
    this.grid = grid;
    this.cap = cap;
    this.quantizeFields = !!opts.quantizeFields;
    this.entries = new Map();
    this.it = new ChunkIntegrator();
    this.pool = new FieldBufferPool(Math.min(cap, 1024));
    this._borderMask = new Uint8Array(CC);
    this._borderExX = new Float32Array(CC);
    this._borderExY = new Float32Array(CC);
    this._rawBuf = new Float32Array(CC);
    this._funnelDists = new Float32Array(CS);
  }

  /** Helper to read integration distance from either Float32 or Quantized Uint16 field */
  readIntegration(f, localIdx) {
    const v = f.integration[localIdx];
    if (!f.quantized) return v;
    return v === QUANT_I_INF ? Infinity : v / QUANT_I_SCALE;
  }

  key(c, kind, tid, minClearance = 1, extra = 0, profile = null, footprint = null) {
    const profId = this.grid.getProfileId(profile);
    let h = hashMix(2166136261, c);
    h = hashMix(h, kind.charCodeAt(0));
    h = hashMix(h, tid);
    h = hashMix(h, minClearance);
    h = hashMix(h, extra);
    h = hashMix(h, profId);
    return footprint ? hashMix(h, hashFootprint(footprint)) : h;
  }

  _match(hit, c, kind, tid, minClearance, extra, profile, footprint = null) {
    return !!(hit && hit.chunkId === c && hit.kind === kind && hit.tid === tid &&
      hit.minClearance === minClearance && hit.extra === extra &&
      hit.profile === profile && sameFootprint(hit.footprint, footprint));
  }

  _tag(f, c, kind, tid, minClearance, extra, profile, footprint = null) {
    f.chunkId = c; f.kind = kind; f.tid = tid;
    f.minClearance = minClearance; f.extra = extra;
    f.profile = profile; f.footprint = footprint;
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
    const { grid, pool, quantizeFields } = this;
    return {
      chunkId: cid,
      kind,
      quantized: quantizeFields,
      originX: grid.chunkOriginX(cid),
      originY: grid.chunkOriginY(cid),
      integration: quantizeFields ? pool.allocU16() : pool.allocF32(),
      flowX: quantizeFields ? pool.allocI8() : pool.allocF32(),
      flowY: quantizeFields ? pool.allocI8() : pool.allocF32(),
      sourceId: trackSources ? pool.allocI32() : null,
      _pooled: false
    };
  }

  getOrCompute(c, kind, tid, minClearance, extra, profile, computeFn, opts = {}) {
    const footprint = opts.footprint ?? null;
    const k = this.key(c, kind, tid, minClearance, extra, profile, footprint);
    const hit = this.entries.get(k);
    if (this._match(hit, c, kind, tid, minClearance, extra, profile, footprint)) {
      return this._promote(k, hit);
    }
    const f = this._allocField(c, kind, !!opts.trackSources);
    computeFn(f, this.it, this.grid, this.pool);
    this._tag(f, c, kind, tid, minClearance, extra, profile, footprint);
    return this._store(k, f);
  }

  getPortalField(c, p, minClearance = 1, nextField = null, nextTid = 0, profile = null, funnelX = undefined, funnelY = undefined, footprint = null) {
    const hasFunnelTarget = funnelX !== undefined && funnelY !== undefined;
    const funnelHash = hasFunnelTarget ? hashMix((funnelX * 4) | 0, (funnelY * 4) | 0) : 0;
    const extraKey = hashMix(nextTid, funnelHash);
    const k = this.key(c, 'p', p.uid, minClearance, extraKey, profile, footprint);
    const hit = this.entries.get(k);
    if (this._match(hit, c, 'p', p.uid, minClearance, extraKey, profile, footprint)) return this._promote(k, hit);

    const sideA = portalSideCells(p, c);
    const nextChunk = c === p.chunkA ? p.chunkB : p.chunkA;
    const sideB = portalSideCells(p, nextChunk);
    const { grid, _borderMask, _funnelDists } = this;
    _borderMask.fill(0);

    let minNext = Infinity;
    if (nextField) {
      for (let i = 0; i < sideB.length; i++) {
        const d = this.readIntegration(nextField, grid.localIndex(sideB[i]));
        if (d < minNext) minNext = d;
      }
    }

    // [2.3] Funnel-Projected Continuous Portal Entry Seeding
    let minFunnelD = Infinity;
    if (hasFunnelTarget && sideA.length > 1) {
      for (let i = 0; i < sideA.length; i++) {
        const cell = sideA[i];
        const wx = (cell % grid.width) + 0.5, wy = ((cell / grid.width) | 0) + 0.5;
        const fd = Math.hypot(wx - funnelX, wy - funnelY);
        _funnelDists[i] = fd;
        if (fd < minFunnelD) minFunnelD = fd;
      }
    }

    const sources = new Array(sideA.length);
    for (let i = 0; i < sideA.length; i++) {
      const loc = grid.localIndex(sideA[i]);
      _borderMask[loc] = 1;
      let w = 0;
      if (nextField && minNext < Infinity) {
        const nd = this.readIntegration(nextField, grid.localIndex(sideB[i]));
        w = nd < Infinity ? (nd - minNext) : (CS * COST_OPEN);
      }
      if (hasFunnelTarget && sideA.length > 1) {
        const fBias = (_funnelDists[i] - minFunnelD) * ( COST_OPEN * 0.35 );
        w = nextField && minNext < Infinity ? (0.7 * w + 0.3 * fBias) : fBias;
      }
      sources[i] = { cell: loc, weight: w, id: i };
    }

    const ox = grid.chunkOriginX(c), oy = grid.chunkOriginY(c);
    const f = this.build(k, c, 'p', sources, portalExitX(p, c), portalExitY(p, c), {
      minClearance,
      profile,
      footprint,
      borderExitMask: (nextField && minNext < Infinity) || hasFunnelTarget ? _borderMask : null,
      losSmoothing: hasFunnelTarget && grid.losSmoothing,
      targetLocalX: hasFunnelTarget ? clamp(funnelX - ox, 0.5, CS - 0.5) : undefined,
      targetLocalY: hasFunnelTarget ? clamp(funnelY - oy, 0.5, CS - 0.5) : undefined
    });
    return this._tag(f, c, 'p', p.uid, minClearance, extraKey, profile, footprint);
  }

  getGoalField(c, cell, minClearance = 1, profile = null, footprint = null) {
    const k = this.key(c, 'g', cell, minClearance, 0, profile, footprint);
    const hit = this.entries.get(k);
    if (this._match(hit, c, 'g', cell, minClearance, 0, profile, footprint)) return this._promote(k, hit);
    const loc = this.grid.localIndex(cell);
    const f = this.build(k, c, 'g', [loc], 0, 0, {
      minClearance,
      profile,
      footprint,
      losSmoothing: this.grid.losSmoothing,
      targetLocalX: (loc % CS) + 0.5,
      targetLocalY: ((loc / CS) | 0) + 0.5
    });
    return this._tag(f, c, 'g', cell, minClearance, 0, profile, footprint);
  }

  getSwarmChunkField(c, swarm) {
    const { minClearance, weightVersion, profile, footprint } = swarm;
    const k = this.key(c, 's', swarm.uid, minClearance, weightVersion, profile, footprint);
    const hit = this.entries.get(k);
    if (this._match(hit, c, 's', swarm.uid, minClearance, weightVersion, profile, footprint)) {
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

      const nk = this.key(nextChunk, 's', swarm.uid, minClearance, weightVersion, profile, footprint);
      const nField = this.entries.get(nk);
      let nMin = Infinity;
      if (nField && nField.chunkId === nextChunk) {
        for (let j = 0; j < nextCells.length; j++) {
          const nd = this.readIntegration(nField, grid.localIndex(nextCells[j]));
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
          const nd = this.readIntegration(nField, grid.localIndex(nextCells[j]));
          if (nd < Infinity) cellOffset = (nd - nMin) * 0.5;
        }
        sources.push({ cell: loc, weight: relW + cellOffset, id: pid });
      }
    }

    const singleGoalLoc = inChunkGoals && inChunkGoals.length === 1 && bestPD > 0 ? inChunkGoals[0].cell : -1;
    const f = this.build(k, c, 's', sources, inChunkGoals ? 0 : domExX, inChunkGoals ? 0 : domExY, {
      minClearance,
      profile,
      footprint,
      borderExitMask: !inChunkGoals ? _borderMask : null,
      borderExX: !inChunkGoals ? _borderExX : null,
      borderExY: !inChunkGoals ? _borderExY : null,
      losSmoothing: singleGoalLoc >= 0 && grid.losSmoothing,
      targetLocalX: singleGoalLoc >= 0 ? (singleGoalLoc % CS) + 0.5 : undefined,
      targetLocalY: singleGoalLoc >= 0 ? ((singleGoalLoc / CS) | 0) + 0.5 : undefined
    });
    return this._tag(f, c, 's', swarm.uid, minClearance, weightVersion, profile, footprint);
  }

  getMultiGoalField(c, sources, minClearance = 1, profile = null, footprint = null) {
    let h = sources.length;
    for (let i = 0; i < sources.length; i++) {
      const s = sources[i];
      h = hashMix(h, typeof s === 'object' ? s.cell : s);
    }
    const k = this.key(c, 'm', h, minClearance, 0, profile, footprint);
    const hit = this.entries.get(k);
    if (this._match(hit, c, 'm', h, minClearance, 0, profile, footprint)) return this._promote(k, hit);

    const f = this.build(k, c, 'm', sources, 0, 0, { minClearance, profile, footprint, trackSources: true });
    return this._tag(f, c, 'm', h, minClearance, 0, profile, footprint);
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
    const { grid, _rawBuf } = this;
    const f = this._allocField(cid, kind, !!opts.trackSources);
    const rawTarget = f.quantized ? _rawBuf : f.integration;

    this.it.integrate(grid, cid, src, rawTarget, {
      minClearance: opts.minClearance ?? 1,
      profile: opts.profile,
      footprint: opts.footprint,
      sourceOut: f.sourceId
    });

    if (f.quantized) {
      const qI = f.integration;
      for (let i = 0; i < CC; i++) {
        const d = rawTarget[i];
        qI[i] = d === Infinity ? QUANT_I_INF : Math.min(0xFFFE, Math.round(d * QUANT_I_SCALE));
      }
    }

    computeFlowVectors(f, exX, exY, {
      grid,
      rawIntegration: rawTarget,
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

//#region 19: Bilinear Flow Sampler & [4.3] Wall-Safe Local Micro-Avoidance Kernel
/**
 * Bilinearly interpolates the flow direction vector `(out.x, out.y)` at world position `(px, py)`.
 * Works transparently on both `Float32Array` and [5.1] Quantized `Int8Array`/`Uint16Array` fields.
 */
export function sampleFlowBilinear(f, px, py, out = { x: 0, y: 0 }, neighborResolver = null) {
  const lx = px - f.originX - 0.5, ly = py - f.originY - 0.5;
  const x0 = Math.floor(lx), y0 = Math.floor(ly), x1 = x0 + 1, y1 = y0 + 1;
  const tx = lx - x0, ty = ly - y0;
  const { integration: I, flowX: FX, flowY: FY, quantized } = f;
  const infVal = quantized ? QUANT_I_INF : Infinity;
  const fScale = quantized ? (1 / 127) : 1;
  const w00 = (1 - tx) * (1 - ty), w10 = tx * (1 - ty), w01 = (1 - tx) * ty, w11 = tx * ty;

  let sx = 0, sy = 0, ws = 0;
  const accum = (cx, cy, w) => {
    if (w <= 0) return;
    if (cx >>> 0 < CS && cy >>> 0 < CS) {
      const id = cy * CS + cx;
      if (I[id] < infVal) { sx += FX[id] * fScale * w; sy += FY[id] * fScale * w; ws += w; }
    } else if (neighborResolver) {
      const wx = f.originX + cx, wy = f.originY + cy;
      const nf = neighborResolver(wx, wy);
      if (nf) {
        const nid = (wy - nf.originY) * CS + (wx - nf.originX);
        const nInf = nf.quantized ? QUANT_I_INF : Infinity;
        const nScale = nf.quantized ? (1 / 127) : 1;
        if (nf.integration[nid] < nInf) {
          sx += nf.flowX[nid] * nScale * w; sy += nf.flowY[nid] * nScale * w; ws += w;
          return;
        }
      }
      const ccx = clamp(cx, 0, CS - 1), ccy = clamp(cy, 0, CS - 1), id = ccy * CS + ccx;
      if (I[id] < infVal) { sx += FX[id] * fScale * w; sy += FY[id] * fScale * w; ws += w; }
    } else {
      const ccx = clamp(cx, 0, CS - 1), ccy = clamp(cy, 0, CS - 1), id = ccy * CS + ccx;
      if (I[id] < infVal) { sx += FX[id] * fScale * w; sy += FY[id] * fScale * w; ws += w; }
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
    if (I[c] < infVal && (FX[c] || FY[c])) {
      const cl = Math.hypot(FX[c], FY[c]) || 1;
      out.x = FX[c] / cl; out.y = FY[c] / cl;
      return true;
    }
    if (len < 1e-6) return false;
  }
  out.x = sx / len; out.y = sy / len;
  return true;
}

/**
 * [4.3] Blends a sampled flow vector `out` with radial repulsion and tangent-sliding
 * around nearby dynamic `neighbors` (`[{ x, y, radius, vx, vy }]`), verifying against
 * `grid` walkability so agents are never pushed into static walls.
 */
export function applyLocalAvoidance(grid, wx, wy, out, neighbors, opts = {}) {
  if (!neighbors || !neighbors.length) return out;
  const selfRadius = opts.selfRadius ?? 0.4;
  const avoidStrength = opts.avoidStrength ?? 1.35;
  const minClearance = opts.minClearance ?? 1;
  const fx = out.x, fy = out.y;
  let rx = 0, ry = 0;

  for (let i = 0; i < neighbors.length; i++) {
    const nb = neighbors[i];
    const dx = wx - nb.x, dy = wy - nb.y;
    const d2 = dx * dx + dy * dy;
    if (d2 < 1e-8) continue;
    const radSum = selfRadius + (nb.radius ?? 0.4) + 0.65;
    if (d2 >= radSum * radSum) continue;

    const d = Math.sqrt(d2);
    const nx = dx / d, ny = dy / d;
    const pen = (radSum - d) / radSum;
    // Check if we are moving toward this neighbor
    const approach = -(fx * nx + fy * ny);
    const weight = pen * pen * (approach > 0 ? (1 + approach * 1.5) : 0.5);

    // Choose tangent direction aligned with desired flow
    const t1x = -ny, t1y = nx;
    const dotT = fx * t1x + fy * t1y;
    const tx = dotT >= 0 ? t1x : -t1x;
    const ty = dotT >= 0 ? t1y : -t1y;

    rx += (nx * 0.65 + tx * 0.75) * weight;
    ry += (ny * 0.65 + ty * 0.75) * weight;
  }

  if (rx === 0 && ry === 0) return out;
  let bx = fx + rx * avoidStrength, by = fy + ry * avoidStrength;
  const bl = Math.hypot(bx, by);
  if (bl < 1e-6) return out;
  bx /= bl; by /= bl;

  // Wall-safety probe: only accept the deflected vector if it doesn't point into a blocked cell
  const probeX = Math.floor(wx + bx * 0.65), probeY = Math.floor(wy + by * 0.65);
  if (grid.isWalkable(probeX, probeY, minClearance)) {
    out.x = bx; out.y = by;
  }
  return out;
}
//#endregion

//#region 20: FlowFieldCore (Standalone 2D Hierarchical Pathfinder Orchestrator)
/**
 * Standalone 2D Hierarchical Flow Field Pathfinding Kernel (`FlowFieldCore`).
 */
export class FlowFieldCore {
  constructor(opts = {}) {
    const cx = opts.width ? Math.ceil(opts.width / CS) : (opts.chunksX ?? 10);
    const cy = opts.height ? Math.ceil(opts.height / CS) : (opts.chunksY ?? 10);
    this.cacheCap = opts.cacheCap ?? 2048;
    this.minClearance = opts.minClearance ?? 1;
    this.portalLookahead = opts.portalLookahead ?? true;
    this.seamInterpolation = opts.seamInterpolation ?? true;
    this.useSuperChunks = opts.useSuperChunks ?? true;
    this.quantizeFields = opts.quantizeFields ?? false;

    this.grid = new Grid(cx, cy, opts.costs || null, {
      shared: opts.shared ?? false
    });
    if (opts.useEikonal !== undefined) this.grid.useEikonal = opts.useEikonal;
    if (opts.losSmoothing !== undefined) this.grid.losSmoothing = opts.losSmoothing;
    if (opts.wallRepulsion) this.grid.enableWallRepulsion(opts.wallRepulsion);

    this.graph = new PortalGraph(this.grid, null, null, this.minClearance, null, null);
    this.macro = new MacroPathfinder(this.grid, this.graph);
    this.cache = new FlowFieldCache(this.grid, this.cacheCap, { quantizeFields: this.quantizeFields });
    this.clearanceGraphs = new Map([[this._hierarchyKey(this.minClearance, null, null), { graph: this.graph, macro: this.macro }]]);
    this._activeSwarms = new Set();
    this._workQueue = [];
    this._tmpVec = { x: 0, y: 0 };
  }

  get width() { return this.grid.width; }
  get height() { return this.grid.height; }

  _hierarchyKey(minClearance, profile = null, footprint = null) {
    if (!profile && !footprint) return minClearance;
    const fpKey = footprint ? `${footprint.width ?? 1}x${footprint.length ?? 1}:${footprint.turnPenalty ?? 2}` : '';
    return `${minClearance}:${this.grid.getProfileId(profile)}:${fpKey}`;
  }

  // World, Terrain, Dynamic Obstacle & Crowd Mutator Delegates
  getCost(x, y) { return this.grid.getCost(x, y); }
  setCost(x, y, cost) { return this.grid.setCost(x, y, cost); }
  setRect(x, y, w, h, cost) { return this.grid.setRect(x, y, w, h, cost); }
  setDisc(x, y, r, cost) { return this.grid.setDisc(x, y, r, cost); }
  loadCosts(costs) { this.grid.loadCosts(costs); }

  registerProfile(name, costMapOrArray) { return this.grid.registerProfile(name, costMapOrArray); }
  setTerrain(x, y, terrainId) { return this.grid.setTerrain(x, y, terrainId); }
  setRectTerrain(x, y, w, h, terrainId) { return this.grid.setRectTerrain(x, y, w, h, terrainId); }

  /** [4.1] Reference-counted dynamic obstacle stamp delegates */
  stampObstacle(shape) { return this.grid.stampObstacle(shape); }
  unstampObstacle(id) { return this.grid.unstampObstacle(id); }
  moveObstacle(id, newX, newY) { return this.grid.moveObstacle(id, newX, newY); }

  /** [4.2] Continuum crowd density & flux splatting delegate */
  updateCrowdCongestion(agents, opts = {}) { return this.grid.updateCrowdCongestion(agents, opts); }

  decayPortalTraffic(decayFactor = 0.85) {
    for (const entry of this.clearanceGraphs.values()) {
      entry.graph.decayPortalTraffic(decayFactor);
    }
  }

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
        const nextGraph = new PortalGraph(this.grid, entry.graph, all, entry.graph.minClearance, entry.graph.profile, entry.graph.footprint);
        const nextMacro = new MacroPathfinder(this.grid, nextGraph);
        this.clearanceGraphs.set(hKey, { graph: nextGraph, macro: nextMacro });
        if (hKey === this.minClearance) {
          this.graph = nextGraph;
          this.macro = nextMacro;
        }
      } else if (weights.size > 0) {
        entry.graph.refreshWeights(weights);
        entry.macro.invalidateChunks(weights);
      }
    }

    // [1.3] Incrementally repair any registered SwarmFields
    for (const swarm of this._activeSwarms) {
      const { macro } = this._getHierarchy(swarm.minClearance, swarm.profile, swarm.footprint);
      macro.updateSwarmField(swarm, all);
    }

    return true;
  }

  _getHierarchy(minClearance = this.minClearance, profile = null, footprint = null) {
    if (this.grid.dirtyChunks.size > 0) this.rebuild();
    const hKey = this._hierarchyKey(minClearance, profile, footprint);
    let entry = this.clearanceGraphs.get(hKey);
    if (!entry) {
      const graph = new PortalGraph(this.grid, null, null, minClearance, profile, footprint);
      const macro = new MacroPathfinder(this.grid, graph);
      entry = { graph, macro };
      this.clearanceGraphs.set(hKey, entry);
    }
    return entry;
  }

  findPath(sx, sy, gx, gy, opts = {}) {
    const clr = opts.minClearance ?? this.minClearance;
    const prof = opts.profile ?? null;
    const fp = opts.footprint ?? null;
    const { macro } = this._getHierarchy(clr, prof, fp);
    const { grid } = this;
    const scx = clamp(sx | 0, 0, grid.width - 1), scy = clamp(sy | 0, 0, grid.height - 1);
    const gcx = clamp(gx | 0, 0, grid.width - 1), gcy = clamp(gy | 0, 0, grid.height - 1);
    const startCell = grid.isWalkable(scx, scy, clr, prof, fp) ? grid.index(scx, scy) : grid.findNearestWalkable(scx, scy, 16, clr, prof, fp);
    const goalCell = grid.isWalkable(gcx, gcy, clr, prof, fp) ? grid.index(gcx, gcy) : grid.findNearestWalkable(gcx, gcy, 16, clr, prof, fp);
    if (startCell < 0 || goalCell < 0) return null;
    return macro.findPath(startCell, goalCell, opts.useSuperChunks ?? this.useSuperChunks);
  }

  /**
   * [3.1] Computes or extracts an any-angle, world-LOS string-pulled waypoint array `[{ x, y }, ...]`
   * with zero flow-field allocations. Accepts either `(sx, sy, path)` or `(sx, sy, gx, gy, opts)`.
   */
  extractWaypoints(sx, sy, pathOrGx, gy = undefined, opts = {}) {
    const path = typeof pathOrGx === 'object' && pathOrGx !== null
      ? pathOrGx
      : this.findPath(sx, sy, pathOrGx, gy, opts);
    if (!path) return null;
    const { macro } = this._getHierarchy(path.minClearance, path.profile, path.footprint);
    return macro.extractWaypoints(path, sx, sy);
  }

  findMultiGoalPath(sx, sy, goals, opts = {}) {
    const clr = opts.minClearance ?? this.minClearance;
    const prof = opts.profile ?? null;
    const fp = opts.footprint ?? null;
    const { macro } = this._getHierarchy(clr, prof, fp);
    const { grid } = this;
    const scx = clamp(sx | 0, 0, grid.width - 1), scy = clamp(sy | 0, 0, grid.height - 1);
    const startCell = grid.isWalkable(scx, scy, clr, prof, fp) ? grid.index(scx, scy) : grid.findNearestWalkable(scx, scy, 16, clr, prof, fp);
    if (startCell < 0) return null;
    const validTargets = [];
    for (const g of goals) {
      const gx = clamp(g.x | 0, 0, grid.width - 1), gy = clamp(g.y | 0, 0, grid.height - 1);
      const cell = grid.isWalkable(gx, gy, clr, prof, fp) ? grid.index(gx, gy) : grid.findNearestWalkable(gx, gy, 16, clr, prof, fp);
      if (cell >= 0) validTargets.push({ cell, weight: g.weight ?? 0 });
    }
    if (!validTargets.length) return null;
    return macro.findPath(startCell, validTargets, false);
  }

  enqueuePath(sx, sy, gx, gy, opts = {}, callback = null) {
    const priority = opts.priority ?? 0;
    return new Promise(resolve => {
      this._workQueue.push({
        priority,
        run: () => {
          const res = Array.isArray(gx) ? this.findMultiGoalPath(sx, sy, gx, gy || {}) : this.findPath(sx, sy, gx, gy, opts);
          if (callback) callback(res);
          resolve(res);
        }
      });
      this._workQueue.sort((a, b) => b.priority - a.priority);
    });
  }

  processQueue(maxBudgetMs = 2.0) {
    const now = typeof performance !== 'undefined' ? () => performance.now() : () => Date.now();
    const t0 = now();
    if (this.grid.dirtyChunks.size > 0) this.rebuild();
    let processed = 0;
    while (this._workQueue.length > 0 && (now() - t0 < maxBudgetMs || processed === 0)) {
      const job = this._workQueue.shift();
      job.run();
      processed++;
    }
    return { processed, remaining: this._workQueue.length };
  }

  createSwarmField(goals, opts = {}) {
    const clr = opts.minClearance ?? this.minClearance;
    const prof = opts.profile ?? null;
    const fp = opts.footprint ?? null;
    const { macro } = this._getHierarchy(clr, prof, fp);
    const { grid } = this;
    const list = Array.isArray(goals) ? goals : [goals];
    const valid = [];
    for (const g of list) {
      const gx = clamp(g.x | 0, 0, grid.width - 1), gy = clamp(g.y | 0, 0, grid.height - 1);
      const cell = grid.isWalkable(gx, gy, clr, prof, fp) ? grid.index(gx, gy) : grid.findNearestWalkable(gx, gy, 16, clr, prof, fp);
      if (cell >= 0) valid.push({ cell, weight: g.weight ?? 0 });
    }
    if (!valid.length) return null;
    const swarm = macro.createSwarmField(valid.length === 1 && valid[0].weight === 0 ? valid[0].cell : valid);
    if (opts.autoUpdate !== false) this._activeSwarms.add(swarm);
    return swarm;
  }

  /** [1.3] Manually triggers incremental subtree repair on a `SwarmField` or unregisters it */
  updateSwarmField(swarm, dirtyChunks = null) {
    if (!swarm) return null;
    const { macro } = this._getHierarchy(swarm.minClearance, swarm.profile, swarm.footprint);
    return macro.updateSwarmField(swarm, dirtyChunks);
  }

  disposeSwarmField(swarm) {
    this._activeSwarms.delete(swarm);
  }

  getFieldForSwarm(swarm, wx, wy) {
    if (!swarm) return null;
    const { graph, macro } = this._getHierarchy(swarm.minClearance, swarm.profile, swarm.footprint);
    if (swarm.graph !== graph) macro.updateSwarmField(swarm, null);
    const { grid } = this;
    const cx = clamp(wx | 0, 0, grid.width - 1), cy = clamp(wy | 0, 0, grid.height - 1);
    const chunk = grid.chunkIdAt(cx, cy);
    return this.cache.getSwarmChunkField(chunk, swarm);
  }

  sampleSwarmFlow(swarm, wx, wy, out = { x: 0, y: 0 }, opts = null) {
    const field = this.getFieldForSwarm(swarm, wx, wy);
    if (!field) { out.x = 0; out.y = 0; return false; }
    const resolver = this.seamInterpolation ? (nx, ny) => {
      if (!this.grid.inBounds(nx, ny)) return null;
      return this.cache.getSwarmChunkField(this.grid.chunkIdAt(nx, ny), swarm);
    } : null;
    const ok = sampleFlowBilinear(field, wx, wy, out, resolver);
    if (ok && opts && opts.avoidAgents) {
      applyLocalAvoidance(this.grid, wx, wy, out, opts.avoidAgents, opts);
    }
    return ok;
  }

  _resolveStepField(path, graph, stepIdx) {
    const chunk = path.chunks[stepIdx];
    const clr = path.minClearance ?? this.minClearance;
    const prof = path.profile ?? null;
    const fp = path.footprint ?? null;
    if (stepIdx >= path.portals.length) {
      return this.cache.getGoalField(chunk, path.goalCell, clr, prof, fp);
    }
    const p = graph.portals[path.portals[stepIdx]];
    let nextField = null, nextTid = 0;
    if (this.portalLookahead && stepIdx + 1 < path.chunks.length) {
      const nextChunk = path.chunks[stepIdx + 1];
      if (stepIdx + 1 < path.portals.length) {
        const nextP = graph.portals[path.portals[stepIdx + 1]];
        nextTid = nextP.uid;
        const nfx = path.funnelX ? path.funnelX[stepIdx + 1] : undefined;
        const nfy = path.funnelY ? path.funnelY[stepIdx + 1] : undefined;
        nextField = this.cache.getPortalField(nextChunk, nextP, clr, null, 0, prof, nfx, nfy, fp);
      } else {
        nextTid = path.goalCell + 1;
        nextField = this.cache.getGoalField(nextChunk, path.goalCell, clr, prof, fp);
      }
    }
    const fx = path.funnelX ? path.funnelX[stepIdx] : undefined;
    const fy = path.funnelY ? path.funnelY[stepIdx] : undefined;
    return this.cache.getPortalField(chunk, p, clr, nextField, nextTid, prof, fx, fy, fp);
  }

  getFieldForPath(path, wx, wy, hintIdx = 0) {
    if (!path) return null;
    const clr = path.minClearance ?? this.minClearance;
    const prof = path.profile ?? null;
    const fp = path.footprint ?? null;
    const { graph } = this._getHierarchy(clr, prof, fp);
    if (path.graph !== graph) return null;
    const { grid } = this;
    const cx = clamp(wx | 0, 0, grid.width - 1), cy = clamp(wy | 0, 0, grid.height - 1);
    const localIdx = (cy % CS) * CS + (cx % CS);
    const chunk = grid.chunkIdAt(cx, cy);

    if (hintIdx >= 0 && hintIdx < path.chunks.length && path.chunks[hintIdx] === chunk) {
      const f = this._resolveStepField(path, graph, hintIdx);
      if (this.cache.readIntegration(f, localIdx) < Infinity) return { field: f, chunkIndex: hintIdx };
    }

    for (let i = Math.max(0, hintIdx); i < path.chunks.length; i++) {
      if (path.chunks[i] === chunk) {
        const f = this._resolveStepField(path, graph, i);
        if (this.cache.readIntegration(f, localIdx) < Infinity) return { field: f, chunkIndex: i };
      }
    }
    for (let i = Math.min(path.chunks.length - 1, hintIdx - 1); i >= 0; i--) {
      if (path.chunks[i] === chunk) {
        const f = this._resolveStepField(path, graph, i);
        if (this.cache.readIntegration(f, localIdx) < Infinity) return { field: f, chunkIndex: i };
      }
    }
    return null;
  }

  sampleFlow(path, wx, wy, out = { x: 0, y: 0 }, hintIdx = 0, opts = null) {
    const res = this.getFieldForPath(path, wx, wy, hintIdx);
    if (!res) { out.x = 0; out.y = 0; return false; }
    const resolver = this.seamInterpolation ? (nx, ny) => {
      if (!this.grid.inBounds(nx, ny)) return null;
      const nRes = this.getFieldForPath(path, nx, ny, res.chunkIndex);
      return nRes ? nRes.field : null;
    } : null;
    const ok = sampleFlowBilinear(res.field, wx, wy, out, resolver);
    if (ok && opts && opts.avoidAgents) {
      applyLocalAvoidance(this.grid, wx, wy, out, opts.avoidAgents, opts);
    }
    return ok;
  }
}

export { FlowFieldCore as FlowFieldPathfinder };
export default FlowFieldCore;
//#endregion
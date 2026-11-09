//#region 1: Grid & Cost Constants
/** @constant {number} Chunk width/height in cells (16x16) */
export const CS = 16;

/** @constant {number} Total cells per chunk (256) */
export const CC = CS * CS;

/** @constant {number} Super-chunk width/height in chunks (4x4 chunks = 64x64 cells) (#I4) */
export const SUPER_STEP = 4;

/** @constant {number} Default open terrain traversal cost */
export const COST_OPEN = 2;

/** @constant {number} Impassable obstacle cost threshold (255) */
export const COST_BLOCK = 255;

/** @constant {number} Maximum portal width before splitting (#A2) */
export const MAX_PORTAL_LEN = 6;

/** @constant {number} Maximum tracked brushfire clearance radius in cells (#B2) */
export const MAX_CLEARANCE = 16;
//#endregion

//#region 2: Directional Lookup Tables & Math Utilities
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

//#region 3: 4-Ary Cache-Friendly Min-Heap (#B5)
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
//#endregion

//#region 4: Zero-Allocation Slab Buffer Pool (#B4)
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

/**
 * 2D / 2.5D Multi-Floor chunked cost grid supporting:
 * - #H3: Stacked vertical layers (bridges, overpasses, multi-story buildings) & stair/ramp portals
 * - #H4: Anisotropic directional clearance (`clearanceX`, `clearanceY`) for rectangular vehicles
 * - #I3: `SharedArrayBuffer` backing (`opts.shared`) for lock-free Web Worker pipelines
 * - #B1, #B2, #F3: Two-tier dirty tracking & clustered multi-rect SDF/Clearance sweeps
 */
export class GridWorld {
  //#region 5: GridWorld Core State & SharedArrayBuffer Hydration (#H3, #H4, #I3)
  constructor(cx, cy, existingCosts = null, opts = {}) {
    this.cx = cx;
    this.cy = cy;
    this.layers = Math.max(1, opts.layers ?? 1);
    this.width = cx * CS;
    this.height = cy * CS;
    this.layerChunks = cx * cy;
    this.layerCells = this.width * this.height;
    this.chunkCount = this.layerChunks * this.layers;
    const len = this.layerCells * this.layers;

    // #I3: Optional SharedArrayBuffer backing for lock-free Worker concurrency
    this.useSharedMemory = !!(opts.shared && typeof SharedArrayBuffer !== 'undefined');
    this._allocBuf = (ByteSize) => this.useSharedMemory ? new SharedArrayBuffer(ByteSize) : new ArrayBuffer(ByteSize);

    if (existingCosts) {
      this.costs = existingCosts.subarray(0, len);
    } else {
      this.costs = new Uint8Array(this._allocBuf(len)).fill(COST_OPEN);
    }

    // #B1: Two-Tier Dirty Tracking (Topology vs. Weights)
    this.dirtyChunks = new Set();
    this.topologyDirtyChunks = new Set();
    this.weightsDirtyChunks = new Set();

    // Solver & feature configuration
    this.useEikonal = true;
    this.losSmoothing = true;

    // #4, #B2 & #F3: Wall Repulsion / Signed Distance Field (SDF)
    this.wallPenaltyRadius = 2.5;
    this.wallPenaltyWeight = 0.0;
    this.wallRepulsionWeight = 0.0;
    this.wallDist = null;
    this.wallGradX = null;
    this.wallGradY = null;
    this._sdfDirty = true;
    this._sdfFullDirty = true;
    this._sdfDirtyChunks = new Set();

    // #5, #B2, #F3 & #H4: Isotropic & Directional Clearance (X/Y Corridor Spans)
    this.clearance = null;
    this.clearanceX = null;
    this.clearanceY = null;
    this._clearanceDirty = true;
    this._clearanceFullDirty = true;
    this._clearanceDirtyChunks = new Set();

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

    // #D4 & #F1: Terrain ID Layer & Scoped Unit Movement Cost Profiles
    this.terrainIds = null;
    this.profiles = new Map();
    this.profileIds = new Map();
    this._nextProfileId = 1;

    // #H3: Multi-Floor Vertical Stair / Ramp / Overpass Portal Definitions
    this.verticalPortals = [];
  }

  /** #I3: Exports SharedArrayBuffer/ArrayBuffer views for hydrating a Worker thread */
  toSharedDescriptor() {
    return {
      cx: this.cx, cy: this.cy, layers: this.layers,
      useEikonal: this.useEikonal, losSmoothing: this.losSmoothing,
      wallPenaltyRadius: this.wallPenaltyRadius, wallPenaltyWeight: this.wallPenaltyWeight, wallRepulsionWeight: this.wallRepulsionWeight,
      congestionWeight: this.congestionWeight, congestionLaneWeight: this.congestionLaneWeight,
      currentStrength: this.currentStrength,
      costs: this.costs,
      clearance: this.clearance, clearanceX: this.clearanceX, clearanceY: this.clearanceY,
      wallDist: this.wallDist, wallGradX: this.wallGradX, wallGradY: this.wallGradY,
      tacticalCosts: this.tacticalCosts, hasTactical: this.hasTactical,
      congestion: this.congestion, congestionFluxX: this.congestionFluxX, congestionFluxY: this.congestionFluxY,
      hasCongestion: this.hasCongestion, hasCongestionFlux: this.hasCongestionFlux,
      dirMask: this.dirMask, currentX: this.currentX, currentY: this.currentY, hasAnisotropy: this.hasAnisotropy,
      dynamicObstacles: this.dynamicObstacles, hasDynamicObstacles: this.hasDynamicObstacles,
      terrainIds: this.terrainIds,
      profiles: Array.from(this.profiles.entries()),
      verticalPortals: this.verticalPortals
    };
  }

  /** #I3: Hydrates a `GridWorld` instance directly over shared buffers in a Worker */
  static fromSharedDescriptor(desc) {
    const g = new GridWorld(desc.cx, desc.cy, desc.costs, { layers: desc.layers, shared: true });
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
    if (desc.verticalPortals) g.verticalPortals = desc.verticalPortals.slice();
    return g;
  }
  //#endregion

  //#region 6: GridWorld Indexing, Walkability & Two-Tier Dirty Tracking (#B1, #H4)
  index(x, y, z = 0) { return z * this.layerCells + y * this.width + x; }
  inBounds(x, y, z = 0) { return x >= 0 && y >= 0 && x < this.width && y < this.height && z >= 0 && z < this.layers; }

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
    if (this.hasDynamicObstacles && this.dynamicObstacles[idx]) return COST_BLOCK;
    if (profileTable && this.terrainIds) return profileTable[this.terrainIds[idx]];
    return this.costs[idx];
  }

  /**
   * Checks walkability at `(x, y, z)`, supporting both isotropic `minClearance` and
   * #H4 anisotropic rectangular vehicle footprints `{ width, length }`.
   */
  isWalkable(x, y, minClearance = 1, profile = null, z = 0, footprint = null) {
    if (!this.inBounds(x, y, z)) return false;
    const idx = this.index(x, y, z);
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

  chunkIdAt(x, y, z = 0) { return z * this.layerChunks + (y / CS | 0) * this.cx + (x / CS | 0); }
  chunkLayer(c) { return (c / this.layerChunks) | 0; }
  chunkOriginX(c) { return ((c % this.layerChunks) % this.cx) * CS; }
  chunkOriginY(c) { return (((c % this.layerChunks) / this.cx) | 0) * CS; }
  localIndex(cell) {
    const rem = cell % this.layerCells;
    return ((rem / this.width | 0) % CS) * CS + (rem % this.width % CS);
  }

  _addChunkAndBorderNeighbors(set, x, y, z = 0) {
    const c = this.chunkIdAt(x, y, z);
    set.add(c);
    const lx = x % CS, ly = y % CS;
    if (lx === 0 && x > 0) set.add(c - 1);
    if (lx === CS - 1 && x < this.width - 1) set.add(c + 1);
    if (ly === 0 && y > 0) set.add(c - this.cx);
    if (ly === CS - 1 && y < this.height - 1) set.add(c + this.cx);
  }

  markCellDirty(x, y, isTopology = true, z = 0) {
    this._addChunkAndBorderNeighbors(this.dirtyChunks, x, y, z);
    if (isTopology) {
      this._addChunkAndBorderNeighbors(this.topologyDirtyChunks, x, y, z);
      const c = this.chunkIdAt(x, y, z);
      this._sdfDirty = true;
      this._clearanceDirty = true;
      this._sdfDirtyChunks.add(c);
      this._clearanceDirtyChunks.add(c);
    } else {
      this._addChunkAndBorderNeighbors(this.weightsDirtyChunks, x, y, z);
    }
  }

  markChunkWeightDirty(c) {
    this.dirtyChunks.add(c);
    this.weightsDirtyChunks.add(c);
  }
  //#endregion

  //#region 7: GridWorld Vertical Stair Portals & Dirty Cluster Merging (#F3, #H3)
  /**
   * #H3: Registers a vertical stair, ramp, or overpass portal connecting `(x, y)` on `fromLayer`
   * to `(x, y)` (or `(toX, toY)`) on `toLayer`.
   */
  addStairPortal(x, y, w = 2, h = 2, fromLayer = 0, toLayer = 1, opts = {}) {
    const toX = opts.toX ?? x, toY = opts.toY ?? y;
    const cxA = clamp(x | 0, 0, this.width - 1), cyA = clamp(y | 0, 0, this.height - 1);
    const cxB = clamp(toX | 0, 0, this.width - 1), cyB = clamp(toY | 0, 0, this.height - 1);
    const zA = clamp(fromLayer | 0, 0, this.layers - 1), zB = clamp(toLayer | 0, 0, this.layers - 1);
    if (zA === zB && cxA === cxB && cyA === cyB) return null;

    const cA = this.chunkIdAt(cxA, cyA, zA), cB = this.chunkIdAt(cxB, cyB, zB);
    const oxA = this.chunkOriginX(cA), oyA = this.chunkOriginY(cA);
    const oxB = this.chunkOriginX(cB), oyB = this.chunkOriginY(cB);
    const cellsA = [], cellsB = [];
    const pw = Math.max(1, Math.min(CS, w | 0)), ph = Math.max(1, Math.min(CS, h | 0));

    for (let dy = 0; dy < ph; dy++) {
      for (let dx = 0; dx < pw; dx++) {
        const ax = clamp(cxA + dx, oxA, oxA + CS - 1), ay = clamp(cyA + dy, oyA, oyA + CS - 1);
        const bx = clamp(cxB + dx, oxB, oxB + CS - 1), by = clamp(cyB + dy, oyB, oyB + CS - 1);
        cellsA.push(this.index(ax, ay, zA));
        cellsB.push(this.index(bx, by, zB));
      }
    }

    const vp = {
      chunkA: cA, chunkB: cB,
      layerA: zA, layerB: zB,
      cellsA: new Int32Array(cellsA),
      cellsB: new Int32Array(cellsB),
      centerA: cellsA[cellsA.length >> 1],
      centerB: cellsB[cellsB.length >> 1],
      canAB: true,
      canBA: opts.bidirectional !== false,
      cost: opts.cost ?? COST_OPEN,
      exitX: opts.exitX ?? 0,
      exitY: opts.exitY ?? 0,
      anchorX: cxA + pw * 0.5,
      anchorY: cyA + ph * 0.5
    };
    this.verticalPortals.push(vp);
    this.dirtyChunks.add(cA); this.dirtyChunks.add(cB);
    this.topologyDirtyChunks.add(cA); this.topologyDirtyChunks.add(cB);
    return vp;
  }

  _buildDirtyClusters(dirtyChunkSet, halo, isFull, layer = 0) {
    const { width: W, height: H } = this;
    if (isFull || dirtyChunkSet.size === 0 || dirtyChunkSet.size > (this.chunkCount >> 1)) {
      return [{ x0: 0, y0: 0, x1: W - 1, y1: H - 1 }];
    }
    const rects = [];
    for (const c of dirtyChunkSet) {
      if (this.chunkLayer(c) !== layer) continue;
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

  //#region 8: GridWorld Cost Mutators & Bulk Loaders
  getCost(x, y, z = 0) {
    return this.inBounds(x | 0, y | 0, z | 0) ? this.costs[this.index(x | 0, y | 0, z | 0)] : COST_BLOCK;
  }

  setCost(x, y, cost, z = 0) {
    const ix = x | 0, iy = y | 0, iz = z | 0;
    if (!this.inBounds(ix, iy, iz)) return false;
    const idx = this.index(ix, iy, iz), prev = this.costs[idx];
    if (prev === cost) return false;
    this.costs[idx] = cost;
    const topoChanged = (prev >= COST_BLOCK) !== (cost >= COST_BLOCK);
    this.markCellDirty(ix, iy, topoChanged, iz);
    return true;
  }

  setRect(x, y, w, h, cost, z = 0) {
    const x0 = clamp(Math.floor(x), 0, this.width), x1 = clamp(Math.ceil(x + w), 0, this.width);
    const y0 = clamp(Math.floor(y), 0, this.height), y1 = clamp(Math.ceil(y + h), 0, this.height);
    let changed = 0;
    for (let yy = y0; yy < y1; yy++) {
      for (let xx = x0; xx < x1; xx++) {
        if (this.setCost(xx, yy, cost, z)) changed++;
      }
    }
    return changed;
  }

  setDisc(wx, wy, r, cost, z = 0) {
    const r2 = r * r;
    const x0 = clamp(Math.floor(wx - r), 0, this.width - 1), x1 = clamp(Math.ceil(wx + r), 0, this.width - 1);
    const y0 = clamp(Math.floor(wy - r), 0, this.height - 1), y1 = clamp(Math.ceil(wy + r), 0, this.height - 1);
    let changed = 0;
    for (let y = y0; y <= y1; y++) {
      for (let x = x0; x <= x1; x++) {
        if ((x + 0.5 - wx) ** 2 + (y + 0.5 - wy) ** 2 <= r2) {
          if (this.setCost(x, y, cost, z)) changed++;
        }
      }
    }
    return changed;
  }

  loadCosts(srcCosts, layer = null) {
    if (layer !== null) {
      const off = layer * this.layerCells, len = Math.min(this.layerCells, srcCosts.length);
      for (let i = 0; i < len; i++) this.costs[off + i] = srcCosts[i];
      for (let c = layer * this.layerChunks; c < (layer + 1) * this.layerChunks; c++) {
        this.dirtyChunks.add(c); this.topologyDirtyChunks.add(c);
      }
    } else {
      const len = Math.min(this.costs.length, srcCosts.length);
      for (let i = 0; i < len; i++) this.costs[i] = srcCosts[i];
      for (let c = 0; c < this.chunkCount; c++) {
        this.dirtyChunks.add(c); this.topologyDirtyChunks.add(c);
      }
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
  //#endregion

  //#region 9: GridWorld Terrain ID Layer & Unit Movement Profiles (#D4, #F1)
  ensureTerrainLayer() {
    if (!this.terrainIds) this.terrainIds = new Uint8Array(this._allocBuf(this.layerCells * this.layers));
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
    if (!this.profileIds.has(name)) this.profileIds.set(name, this._nextProfileId++);
    this.profileIds.set(table, this.profileIds.get(name));
    return table;
  }

  setTerrain(x, y, terrainId, z = 0) {
    const ix = x | 0, iy = y | 0, iz = z | 0;
    if (!this.inBounds(ix, iy, iz)) return false;
    const T = this.ensureTerrainLayer(), idx = this.index(ix, iy, iz);
    if (T[idx] === terrainId) return false;
    T[idx] = terrainId;
    this.markCellDirty(ix, iy, true, iz);
    return true;
  }

  setRectTerrain(x, y, w, h, terrainId, z = 0) {
    const x0 = clamp(Math.floor(x), 0, this.width), x1 = clamp(Math.ceil(x + w), 0, this.width);
    const y0 = clamp(Math.floor(y), 0, this.height), y1 = clamp(Math.ceil(y + h), 0, this.height);
    for (let yy = y0; yy < y1; yy++) {
      for (let xx = x0; xx < x1; xx++) this.setTerrain(xx, yy, terrainId, z);
    }
  }
  //#endregion

  //#region 10: GridWorld Dynamic Moving Obstacle Stamp Layer (#C3)
  ensureDynamicObstacleLayer() {
    if (!this.dynamicObstacles) this.dynamicObstacles = new Uint8Array(this._allocBuf(this.layerCells * this.layers));
    return this.dynamicObstacles;
  }

  stampDynamicObstacle(wx, wy, r = 0.8, blocked = 1, z = 0) {
    const D = this.ensureDynamicObstacleLayer(), r2 = r * r, iz = clamp(z | 0, 0, this.layers - 1);
    const x0 = clamp(Math.floor(wx - r), 0, this.width - 1), x1 = clamp(Math.ceil(wx + r), 0, this.width - 1);
    const y0 = clamp(Math.floor(wy - r), 0, this.height - 1), y1 = clamp(Math.ceil(wy + r), 0, this.height - 1);
    for (let y = y0; y <= y1; y++) {
      for (let x = x0; x <= x1; x++) {
        if ((x + 0.5 - wx) ** 2 + (y + 0.5 - wy) ** 2 <= r2) {
          const idx = this.index(x, y, iz);
          if (D[idx] !== blocked) {
            D[idx] = blocked;
            this._addChunkAndBorderNeighbors(this.weightsDirtyChunks, x, y, iz);
            this._addChunkAndBorderNeighbors(this.dirtyChunks, x, y, iz);
          }
        }
      }
    }
    this.hasDynamicObstacles = true;
  }

  clearDynamicObstacles() {
    if (!this.dynamicObstacles || !this.hasDynamicObstacles) return;
    const D = this.dynamicObstacles, W = this.width, LC = this.layerCells;
    for (let i = 0; i < D.length; i++) {
      if (D[i]) {
        D[i] = 0;
        const z = (i / LC) | 0, rem = i % LC;
        const x = rem % W, y = (rem / W) | 0;
        this._addChunkAndBorderNeighbors(this.weightsDirtyChunks, x, y, z);
        this._addChunkAndBorderNeighbors(this.dirtyChunks, x, y, z);
      }
    }
    this.hasDynamicObstacles = false;
  }
  //#endregion

  //#region 11: GridWorld Multi-Layer Wall Repulsion & SDF (#4, #B2, #F3, #H3)
  enableWallRepulsion({ penaltyWeight = 1.5, repulsionWeight = 0.35, radius = 2.5 } = {}) {
    this.wallPenaltyWeight = penaltyWeight;
    this.wallRepulsionWeight = repulsionWeight;
    this.wallPenaltyRadius = radius;
    this._sdfFullDirty = true;
    this.updateWallSDF(true);
  }

  updateWallSDF(forceFull = false) {
    const { width: W, height: H, layers: L, layerCells: LC, costs } = this, len = LC * L;
    if (!this.wallDist) {
      this.wallDist = new Float32Array(this._allocBuf(len * 4));
      this.wallGradX = new Float32Array(this._allocBuf(len * 4));
      this.wallGradY = new Float32Array(this._allocBuf(len * 4));
      this._sdfFullDirty = true;
    }
    const D = this.wallDist, GX = this.wallGradX, GY = this.wallGradY;
    const maxD = Math.ceil(this.wallPenaltyRadius) + 4;

    for (let z = 0; z < L; z++) {
      const zOff = z * LC;
      const clusters = this._buildDirtyClusters(this._sdfDirtyChunks, maxD, forceFull || this._sdfFullDirty, z);
      for (let c = 0; c < clusters.length; c++) {
        const { x0, y0, x1, y1 } = clusters[c];
        for (let y = y0; y <= y1; y++) {
          const row = zOff + y * W;
          for (let x = x0; x <= x1; x++) {
            const i = row + x;
            D[i] = costs[i] >= COST_BLOCK ? 0 : maxD;
          }
        }
        for (let y = y0; y <= y1; y++) {
          const row = zOff + y * W;
          for (let x = x0; x <= x1; x++) {
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
        for (let y = y1; y >= y0; y--) {
          const row = zOff + y * W;
          for (let x = x1; x >= x0; x--) {
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
        for (let y = y0; y <= y1; y++) {
          const row = zOff + y * W;
          for (let x = x0; x <= x1; x++) {
            const i = row + x;
            const dE = D[row + Math.min(W - 1, x + 1)], dW = D[row + Math.max(0, x - 1)];
            const dS = D[zOff + Math.min(H - 1, y + 1) * W + x], dN = D[zOff + Math.max(0, y - 1) * W + x];
            const gx = dE - dW, gy = dS - dN, l = Math.hypot(gx, gy);
            GX[i] = l > 1e-5 ? gx / l : 0;
            GY[i] = l > 1e-5 ? gy / l : 0;
          }
        }
      }
    }
    this._sdfDirty = this._sdfFullDirty = false;
    this._sdfDirtyChunks.clear();
  }
  //#endregion

  //#region 12: GridWorld Isotropic & Directional Clearance Sweeps (#5, #B2, #F3, #H4)
  /**
   * #5, #B2, #F3 & #H4: Updates isotropic Chebyshev `clearance` AND directional corridor widths
   * `clearanceX` (E-W span) and `clearanceY` (N-S span) for rectangular vehicle routing.
   */
  updateClearance(forceFull = false) {
    const { width: W, height: H, layers: L, layerCells: LC, costs } = this, len = LC * L;
    if (!this.clearance) {
      this.clearance = new Uint8Array(this._allocBuf(len));
      this.clearanceX = new Uint8Array(this._allocBuf(len));
      this.clearanceY = new Uint8Array(this._allocBuf(len));
      this._clearanceFullDirty = true;
    }
    const C = this.clearance, CX = this.clearanceX, CY = this.clearanceY;
    const halo = MAX_CLEARANCE + 2;

    for (let z = 0; z < L; z++) {
      const zOff = z * LC;
      const clusters = this._buildDirtyClusters(this._clearanceDirtyChunks, halo, forceFull || this._clearanceFullDirty, z);
      for (let c = 0; c < clusters.length; c++) {
        const { x0, y0, x1, y1 } = clusters[c];
        for (let y = y0; y <= y1; y++) {
          const row = zOff + y * W;
          for (let x = x0; x <= x1; x++) {
            const i = row + x;
            if (costs[i] >= COST_BLOCK) {
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
          const row = zOff + y * W;
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
          const row = zOff + y * W;
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
    }
    this._clearanceDirty = this._clearanceFullDirty = false;
    this._clearanceDirtyChunks.clear();
  }
  //#endregion

  //#region 13: GridWorld Tactical Overlay, FOV Shadowcasting & Raycast (#6, #E1)
  ensureTacticalLayer() {
    if (!this.tacticalCosts) this.tacticalCosts = new Float32Array(this._allocBuf(this.layerCells * this.layers * 4));
    this.hasTactical = true;
    return this.tacticalCosts;
  }

  setTacticalCost(x, y, extraCost, z = 0) {
    const ix = x | 0, iy = y | 0, iz = z | 0;
    if (!this.inBounds(ix, iy, iz)) return;
    this.ensureTacticalLayer()[this.index(ix, iy, iz)] = Math.max(0, extraCost);
    this.markChunkWeightDirty(this.chunkIdAt(ix, iy, iz));
  }

  addTacticalDisc(wx, wy, r, peakCost, linearFalloff = true, z = 0) {
    const T = this.ensureTacticalLayer(), r2 = r * r, iz = clamp(z | 0, 0, this.layers - 1);
    const x0 = clamp(Math.floor(wx - r), 0, this.width - 1), x1 = clamp(Math.ceil(wx + r), 0, this.width - 1);
    const y0 = clamp(Math.floor(wy - r), 0, this.height - 1), y1 = clamp(Math.ceil(wy + r), 0, this.height - 1);
    for (let y = y0; y <= y1; y++) {
      for (let x = x0; x <= x1; x++) {
        const d2 = (x + 0.5 - wx) ** 2 + (y + 0.5 - wy) ** 2;
        if (d2 > r2) continue;
        const w = linearFalloff ? 1 - Math.sqrt(d2) / r : 1;
        T[this.index(x, y, iz)] += peakCost * w;
        this.markChunkWeightDirty(this.chunkIdAt(x, y, iz));
      }
    }
  }

  addTacticalFOV(wx, wy, r, peakCost, linearFalloff = true, onVisibleCell = null, z = 0) {
    const ox = wx | 0, oy = wy | 0, iz = clamp(z | 0, 0, this.layers - 1);
    if (!this.inBounds(ox, oy, iz)) return;
    const T = peakCost > 0 ? this.ensureTacticalLayer() : null;
    const { width: W, costs } = this, zOff = iz * this.layerCells, r2 = r * r, maxRow = Math.ceil(r);
    const startIdx = zOff + oy * W + ox;
    if (T) {
      T[startIdx] += peakCost;
      this.markChunkWeightDirty(this.chunkIdAt(ox, oy, iz));
    }
    if (onVisibleCell) onVisibleCell(ox, oy, startIdx, 0);

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
          if (!this.inBounds(mx, my, iz)) continue;
          const d2 = dx * dx + dy * dy, idx = zOff + my * W + mx;
          if (d2 <= r2 && costs[idx] < COST_BLOCK) {
            const dist = Math.sqrt(d2);
            if (T) {
              const w = linearFalloff ? 1 - dist / r : 1;
              T[idx] += peakCost * w;
              this.markChunkWeightDirty(this.chunkIdAt(mx, my, iz));
            }
            if (onVisibleCell) onVisibleCell(mx, my, idx, dist);
          }
          const isWall = !this.inBounds(mx, my, iz) || costs[idx] >= COST_BLOCK;
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
    const T = this.tacticalCosts, W = this.width, LC = this.layerCells;
    for (let i = 0; i < T.length; i++) {
      if (T[i] > 0) {
        T[i] = 0;
        const z = (i / LC) | 0, rem = i % LC;
        this.markChunkWeightDirty(this.chunkIdAt(rem % W, (rem / W) | 0, z));
      }
    }
    this.hasTactical = false;
  }

  raycast(x0, y0, x1, y1, opts = {}) {
    const minClearance = opts.minClearance ?? 1;
    const maxCost = opts.maxCost ?? (COST_BLOCK - 1);
    const iz = clamp((opts.layer ?? 0) | 0, 0, this.layers - 1);
    const pTable = this.getProfileTable(opts.profile);
    if (minClearance > 1 && (this._clearanceDirty || !this.clearance)) this.updateClearance();

    let cx = Math.floor(x0), cy = Math.floor(y0);
    const ex = Math.floor(x1), ey = Math.floor(y1);
    const dx = Math.abs(ex - cx), dy = Math.abs(ey - cy);
    const sx = cx < ex ? 1 : -1, sy = cy < ey ? 1 : -1;
    let err = dx - dy;

    const checkCell = (x, y) => {
      if (!this.inBounds(x, y, iz)) return false;
      const idx = this.index(x, y, iz);
      if (this.getEffectiveBaseCost(idx, pTable) > maxCost) return false;
      if (minClearance > 1 && this.clearance[idx] < minClearance) return false;
      return true;
    };

    while (true) {
      if (!checkCell(cx, cy)) {
        const hx = clamp(cx + 0.5, 0, this.width), hy = clamp(cy + 0.5, 0, this.height);
        return { hit: true, x: hx, y: hy, layer: iz, cell: this.inBounds(cx, cy, iz) ? this.index(cx, cy, iz) : -1, distance: Math.hypot(hx - x0, hy - y0) };
      }
      if (cx === ex && cy === ey) {
        return { hit: false, x: x1, y: y1, layer: iz, cell: this.index(ex, ey, iz), distance: Math.hypot(x1 - x0, y1 - y0) };
      }
      const e2 = 2 * err;
      if (e2 > -dy && e2 < dx) {
        if (!checkCell(cx + sx, cy) || !checkCell(cx, cy + sy)) {
          const hx = cx + 0.5, hy = cy + 0.5;
          return { hit: true, x: hx, y: hy, layer: iz, cell: this.index(cx, cy, iz), distance: Math.hypot(hx - x0, hy - y0) };
        }
      }
      if (e2 > -dy) { err -= dy; cx += sx; }
      if (e2 < dx) { err += dx; cy += sy; }
    }
  }
  //#endregion

  //#region 14: GridWorld Continuum Crowd Congestion & Velocity-Flux Layer (#9, #C1)
  ensureCongestionLayer(withFlux = false) {
    const len = this.layerCells * this.layers;
    if (!this.congestion) this.congestion = new Float32Array(this._allocBuf(len * 4));
    if (withFlux && !this.congestionFluxX) {
      this.congestionFluxX = new Float32Array(this._allocBuf(len * 4));
      this.congestionFluxY = new Float32Array(this._allocBuf(len * 4));
    }
    this.hasCongestion = true;
    if (withFlux) this.hasCongestionFlux = true;
    return this.congestion;
  }

  addCongestion(wx, wy, amount = 1.0, vx = 0, vy = 0, z = 0) {
    const ix = wx | 0, iy = wy | 0, iz = z | 0;
    if (!this.inBounds(ix, iy, iz)) return;
    const hasVel = vx !== 0 || vy !== 0;
    const C = this.ensureCongestionLayer(hasVel), idx = this.index(ix, iy, iz);
    C[idx] += amount;
    if (hasVel) {
      this.congestionFluxX[idx] += vx * amount;
      this.congestionFluxY[idx] += vy * amount;
    }
    this.markChunkWeightDirty(this.chunkIdAt(ix, iy, iz));
  }

  addAgentFlux(wx, wy, vx, vy, amount = 1.0, z = 0) {
    this.addCongestion(wx, wy, amount, vx, vy, z);
  }

  setCongestionFromAgents(px, py, count, unitWeight = 1.0, vx = null, vy = null, pz = null) {
    const withFlux = !!(vx && vy);
    const C = this.ensureCongestionLayer(withFlux), { width: W, height: H, layers: L } = this;
    C.fill(0);
    if (withFlux) {
      this.congestionFluxX.fill(0);
      this.congestionFluxY.fill(0);
    }
    for (let i = 0; i < count; i++) {
      const x = px[i] | 0, y = py[i] | 0, z = pz ? (pz[i] | 0) : 0;
      if (x >= 0 && x < W && y >= 0 && y < H && z >= 0 && z < L) {
        const idx = this.index(x, y, z);
        C[idx] += unitWeight;
        if (withFlux) {
          this.congestionFluxX[idx] += vx[i] * unitWeight;
          this.congestionFluxY[idx] += vy[i] * unitWeight;
        }
        this.markChunkWeightDirty(this.chunkIdAt(x, y, z));
      }
    }
  }

  decayCongestion(decayFactor = 0.85, eps = 0.05) {
    if (!this.congestion || !this.hasCongestion) return;
    const C = this.congestion, FX = this.congestionFluxX, FY = this.congestionFluxY;
    const W = this.width, LC = this.layerCells;
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
        const z = (i / LC) | 0, rem = i % LC;
        this.markChunkWeightDirty(this.chunkIdAt(rem % W, (rem / W) | 0, z));
      }
    }
    this.hasCongestion = active;
    if (!active) this.hasCongestionFlux = false;
  }
  //#endregion

  //#region 15: GridWorld Anisotropic Passability, Currents & Nearest Walkable (#11, #A5)
  ensureAnisotropicLayer() {
    const len = this.layerCells * this.layers;
    if (!this.dirMask) this.dirMask = new Uint8Array(this._allocBuf(len)).fill(DIR_FLAGS.ALL);
    if (!this.currentX) {
      this.currentX = new Float32Array(this._allocBuf(len * 4));
      this.currentY = new Float32Array(this._allocBuf(len * 4));
    }
    this.hasAnisotropy = true;
  }

  setDirectionMask(x, y, mask = DIR_FLAGS.ALL, z = 0) {
    const ix = x | 0, iy = y | 0, iz = z | 0;
    if (!this.inBounds(ix, iy, iz)) return;
    this.ensureAnisotropicLayer();
    this.dirMask[this.index(ix, iy, iz)] = mask & 0xFF;
    this.markCellDirty(ix, iy, true, iz);
  }

  setCurrent(x, y, vx, vy, z = 0) {
    const ix = x | 0, iy = y | 0, iz = z | 0;
    if (!this.inBounds(ix, iy, iz)) return;
    this.ensureAnisotropicLayer();
    const idx = this.index(ix, iy, iz);
    this.currentX[idx] = vx;
    this.currentY[idx] = vy;
    this.markCellDirty(ix, iy, false, iz);
  }

  setRectCurrent(x, y, w, h, vx, vy, z = 0) {
    this.ensureAnisotropicLayer();
    const x0 = clamp(Math.floor(x), 0, this.width), x1 = clamp(Math.ceil(x + w), 0, this.width);
    const y0 = clamp(Math.floor(y), 0, this.height), y1 = clamp(Math.ceil(y + h), 0, this.height);
    for (let yy = y0; yy < y1; yy++) {
      for (let xx = x0; xx < x1; xx++) {
        const idx = this.index(xx, yy, z);
        this.currentX[idx] = vx;
        this.currentY[idx] = vy;
        this.markCellDirty(xx, yy, false, z);
      }
    }
  }

  setSlopeFromHeightmap(heights, slopeScale = 1.0, z = 0) {
    this.ensureAnisotropicLayer();
    const { width: W, height: H, currentX: CX, currentY: CY } = this, zOff = z * this.layerCells;
    for (let y = 0; y < H; y++) {
      for (let x = 0; x < W; x++) {
        const i = y * W + x;
        const hE = heights[y * W + Math.min(W - 1, x + 1)], hW = heights[y * W + Math.max(0, x - 1)];
        const hS = heights[Math.min(H - 1, y + 1) * W + x], hN = heights[Math.max(0, y - 1) * W + x];
        CX[zOff + i] = -(hE - hW) * 0.5 * slopeScale;
        CY[zOff + i] = -(hS - hN) * 0.5 * slopeScale;
      }
    }
    for (let c = z * this.layerChunks; c < (z + 1) * this.layerChunks; c++) this.markChunkWeightDirty(c);
  }

  findNearestWalkable(x, y, maxR = 24, minClearance = 1, profile = null, z = 0, footprint = null) {
    const { width: W, height: H } = this;
    const sx = clamp(x | 0, 0, W - 1), sy = clamp(y | 0, 0, H - 1), sz = clamp(z | 0, 0, this.layers - 1);
    if (this.isWalkable(sx, sy, minClearance, profile, sz, footprint)) return this.index(sx, sy, sz);
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
            const idx = this.index(nx, ny, sz), d = dx * dx + dy * dy;
            if (this.isWalkable(nx, ny, minClearance, profile, sz, footprint) && d < bestD) { bestD = d; bestI = idx; }
          }
        }
      }
      if (bestI >= 0) return bestI;
    }
    return -1;
  }
  //#endregion
}

//#region 16: Multi-Layer Portal Detection & Crossing Cost Helpers (#A2, #A5, #D4, #H3, #H4)
/**
 * #A2, #A5, #H3 & #H4: Detects chunk boundary portals across all 2.5D vertical layers,
 * splitting wide runs (> `maxPortalLen`), checking directional masks and rectangular footprints,
 * and appending vertical stair/ramp portals between floors.
 */
export function detectPortals(grid, minClearance = 1, maxPortalLen = MAX_PORTAL_LEN, profile = null, footprint = null) {
  const portals = [], { cx, cy, width, layers, layerChunks, layerCells } = grid;
  const hasDir = grid.hasAnisotropy && grid.dirMask;

  for (let z = 0; z < layers; z++) {
    const zChunkOff = z * layerChunks, zCellOff = z * layerCells;
    const bLayerOff = z * (2 * cy * cx);

    const pushSegment = (isH, cA, cB, border, bIdx, origin, segRun, segLen) => {
      const start = origin + segRun, mid = (segLen - 1) >> 1, cMid = start + mid;
      const cellsA = new Int32Array(segLen), cellsB = new Int32Array(segLen);
      const at = (u, v) => zCellOff + (isH ? v * width + u : u * width + v);
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
        uid: ((bLayerOff + bIdx) * CS + segRun) * (CS + 1) + segLen,
        isH, isVertical: false, layer: z,
        chunkA: cA, chunkB: cB, border, start, end: start + segLen - 1,
        cellsA, cellsB, centerA: cellsA[mid], centerB: cellsB[mid],
        canAB, canBA,
        anchorX: isH ? cMid + 0.5 : border, anchorY: isH ? border : cMid + 0.5
      });
    };

    for (const isH of [false, true]) {
      const countX = cx - !isH, countY = cy - isH, bOffset = isH ? cy * (cx - 1) : 0;
      const W = (u, v) => grid.isWalkable(isH ? u : v, isH ? v : u, minClearance, profile, z, footprint);
      const at = (u, v) => zCellOff + (isH ? v * width + u : u * width + v);
      const dirAB = isH ? DIR_FLAGS.S : DIR_FLAGS.E;
      const dirBA = isH ? DIR_FLAGS.N : DIR_FLAGS.W;
      for (let y = 0; y < countY; y++) {
        for (let x = 0; x < countX; x++) {
          const border = ((isH ? y : x) + 1) * CS, origin = (isH ? x : y) * CS;
          const cA = zChunkOff + y * cx + x, cB = cA + (isH ? cx : 1), bIdx = bOffset + y * countX + x;
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
  }

  // #H3: Append Vertical Stair / Ramp / Overpass Portals connecting 2.5D layers
  for (let v = 0; v < grid.verticalPortals.length; v++) {
    const vp = grid.verticalPortals[v];
    portals.push({
      id: portals.length,
      uid: 0x10000000 + v,
      isH: false, isVertical: true,
      layerA: vp.layerA, layerB: vp.layerB,
      chunkA: vp.chunkA, chunkB: vp.chunkB,
      border: vp.anchorX | 0, start: vp.anchorY | 0, end: vp.anchorY | 0,
      cellsA: vp.cellsA, cellsB: vp.cellsB,
      centerA: vp.centerA, centerB: vp.centerB,
      canAB: vp.canAB, canBA: vp.canBA,
      verticalCost: vp.cost,
      exitX: vp.exitX, exitY: vp.exitY,
      anchorX: vp.anchorX, anchorY: vp.anchorY
    });
  }
  return portals;
}

export const portalSideCells = (p, c) => c === p.chunkA ? p.cellsA : p.cellsB;
export const portalCenterCell = (p, c) => c === p.chunkA ? p.centerA : p.centerB;
export const portalExitX = (p, c) => p.isVertical ? (p.exitX || 0) : (p.isH ? 0 : c === p.chunkA ? 1 : -1);
export const portalExitY = (p, c) => p.isVertical ? (p.exitY || 0) : (p.isH ? c === p.chunkA ? 1 : -1 : 0);
export const pLocal = (grid, p, c) => grid.localIndex(portalCenterCell(p, c));

/** #A5, #D4 & #H3: Anisotropic, profile-aware, and 2.5D vertical portal crossing cost */
export const crossingCost = (grid, p, fromChunk = p.chunkA, profile = null) => {
  if (fromChunk === p.chunkA && p.canAB === false) return Infinity;
  if (fromChunk === p.chunkB && p.canBA === false) return Infinity;
  const pTable = grid.getProfileTable(profile);
  const cA = grid.getEffectiveBaseCost(p.centerA, pTable);
  const cB = grid.getEffectiveBaseCost(p.centerB, pTable);
  if (cA >= COST_BLOCK || cB >= COST_BLOCK) return Infinity;
  if (p.isVertical) return p.verticalCost ?? COST_OPEN;
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

//#region 17: Multi-Mode Eikonal & Flee Chunk Integrator (#1, #A5, #H3, #H4, #I1)
/**
 * Multi-mode chunk integrator supporting:
 * - #1, #A5 & #I1: Float32 per-cell Eikonal quadratic upwind solver
 * - #H3: 2.5D Multi-layer chunk addressing
 * - #H4: Anisotropic Rectangular Footprint (`opts.footprint = { width, length, turnPenalty }`)
 *        restricting heading directions in narrow corridors and penalizing tight turns
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

  prepareChunk(grid, chunkId, minClearance = 1, profile = null, footprint = null) {
    const { costs, blocked, dirMasks, turnPenaltyCell, curX, curY, fluxX, fluxY } = this;
    const ox = grid.chunkOriginX(chunkId), oy = grid.chunkOriginY(chunkId);
    const zOff = grid.chunkLayer(chunkId) * grid.layerCells, W = grid.width;

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

    for (let ly = 0; ly < CS; ly++) {
      const gy = oy + ly, row = zOff + gy * W + ox, lRow = ly * CS;
      for (let lx = 0; lx < CS; lx++) {
        const gIdx = row + lx, lIdx = lRow + lx;
        const baseC = grid.getEffectiveBaseCost(gIdx, pTable);
        if (baseC >= COST_BLOCK) {
          blocked[lIdx] = 1; costs[lIdx] = COST_BLOCK; continue;
        }
        let orientMask = 0xFF;
        turnPenaltyCell[lIdx] = 0;
        if (useClearance) {
          const isoClr = grid.clearance[gIdx];
          if (isoClr < fpW) { blocked[lIdx] = 1; costs[lIdx] = COST_BLOCK; continue; }
          if (fpL > fpW) {
            const cxSpan = grid.clearanceX[gIdx], cySpan = grid.clearanceY[gIdx];
            if (cxSpan < fpL && cySpan < fpL) { blocked[lIdx] = 1; costs[lIdx] = COST_BLOCK; continue; }
            // #H4: If corridor is narrower than vehicle length, restrict movement to aligned corridor axis
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

/**
 * High-level abstract graph connecting chunk boundary portals in CSR format.
 * Supports:
 * - #H3: 2.5D Multi-Floor chunks & vertical stair/ramp portals
 * - #H4: Rectangular vehicle footprint filtering
 * - #I4: Two-Level Hierarchical Super-Chunks (64x64 Macro Tier) for long-range search pruning
 */
export class PortalGraph {
  //#region 18: PortalGraph Construction & Intra-Chunk Distance Tables
  constructor(grid, prevGraph = null, dirtyChunks = null, minClearance = 1, profile = null, footprint = null) {
    this.grid = grid;
    this.minClearance = minClearance;
    this.profile = profile;
    this.footprint = footprint;
    this.weightVersion = prevGraph ? prevGraph.weightVersion + 1 : 1;

    const portals = this.portals = detectPortals(grid, minClearance, MAX_PORTAL_LEN, profile, footprint), P = portals.length;
    const lists = Array.from({ length: grid.chunkCount }, () => []);
    for (let i = 0; i < P; i++) {
      const p = portals[i];
      lists[p.chunkA].push(p.id); lists[p.chunkB].push(p.id);
    }
    const chunkPortals = this.chunkPortals = lists.map(l => new Int32Array(l));
    const sameFp = prevGraph && JSON.stringify(prevGraph.footprint) === JSON.stringify(footprint);
    const canReuse = prevGraph && prevGraph.minClearance === minClearance && prevGraph.profile === profile && sameFp && dirtyChunks;

    this.chunkDistCache = Array.from({ length: grid.chunkCount }, (_, c) =>
      canReuse && !dirtyChunks.has(c) ? prevGraph.chunkDistCache[c] : new Map()
    );

    this.portalTraffic = new Float32Array(P);
    this.trafficWeight = 1.25;

    const adj = Array.from({ length: P }, () => []);
    const it = this._it = new ChunkIntegrator();
    const field = this._field = new Float32Array(CC);
    const itOpts = { minClearance, profile, footprint };

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
    this._buildSuperChunkHierarchy();
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
  //#endregion

  //#region 19: PortalGraph Reverse CSR, Connected Components & Tarjan SCC (#H2)
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
  //#endregion

  //#region 20: PortalGraph Super-Chunk Hierarchy & Dynamic Traffic Weights (#B1, #I4)
  /**
   * #I4: Builds the Two-Level 64x64 Super-Chunk Hierarchy (`SUPER_STEP = 4` chunks per axis).
   * Records super-chunk membership for every chunk and builds the coarse super-chunk adjacency graph.
   */
  _buildSuperChunkHierarchy() {
    const { grid, portals } = this;
    const scx = this.scx = Math.ceil(grid.cx / SUPER_STEP);
    const scy = this.scy = Math.ceil(grid.cy / SUPER_STEP);
    this.superLayerCount = scx * scy;
    const S = this.superChunkCount = this.superLayerCount * grid.layers;
    const chunkToSuper = this.chunkToSuper = new Int32Array(grid.chunkCount);

    for (let c = 0; c < grid.chunkCount; c++) {
      const z = grid.chunkLayer(c), rem = c % grid.layerChunks;
      const cx = rem % grid.cx, cy = (rem / grid.cx) | 0;
      chunkToSuper[c] = z * this.superLayerCount + ((cy / SUPER_STEP) | 0) * scx + ((cx / SUPER_STEP) | 0);
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
  //#endregion
}

/**
 * Hierarchical A*, Two-Level Super-Chunk Corridor Pruner (#I4), Funnel String-Puller (#F5)
 * & Swarm Broadcast Pathfinder (#B3) operating over `PortalGraph`.
 */
export class MacroPathfinder {
  //#region 21: MacroPathfinder Goal Creation & Multi-Goal Seeding
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
    this._superMask = new Uint8Array(graph.superChunkCount);
  }

  invalidateChunks(dirtyChunks) {
    for (const [cell, goal] of this.goalCache) {
      if (dirtyChunks.has(goal.chunk)) this.goalCache.delete(cell);
    }
  }

  createGoal(cell) {
    const { grid, graph } = this;
    const z = (cell / grid.layerCells) | 0, rem = cell % grid.layerCells;
    const x = rem % grid.width, y = (rem / grid.width) | 0;
    const chunk = grid.chunkIdAt(x, y, z), field = new Float32Array(CC);
    this.it.integrate(grid, chunk, [grid.localIndex(cell)], field, {
      minClearance: graph.minClearance,
      profile: graph.profile,
      footprint: graph.footprint
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
    return { isMulti: false, cell, x, y, layer: z, chunk, field, portalCost, compSet, sccMaskLo, sccMaskHi, graph };
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
      const cell = typeof t === 'object' ? (t.cell ?? grid.index(t.x | 0, t.y | 0, t.layer ?? 0)) : t;
      const weight = typeof t === 'object' ? (t.weight ?? 0) : 0;
      if (cell < 0 || cell >= grid.layerCells * grid.layers) continue;
      const z = (cell / grid.layerCells) | 0, rem = cell % grid.layerCells;
      const x = rem % grid.width, y = (rem / grid.width) | 0, chunk = grid.chunkIdAt(x, y, z);
      goalList.push({ cell, x, y, layer: z, chunk, weight });
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
          const bit = graph.portalScc[pid] & 63;
          if (bit < 32) sccMaskLo |= (1 << bit);
          else sccMaskHi |= (1 << (bit - 32));
        }
      }
    }

    const primary = goalList[0] || { cell: 0, x: 0, y: 0, layer: 0, chunk: 0 };
    return {
      isMulti: true,
      cell: primary.cell,
      x: primary.x,
      y: primary.y,
      layer: primary.layer,
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
  //#endregion

  //#region 22: MacroPathfinder Super-Chunk Corridor Pruner & Portal Funnel (#F5, #I4)
  /**
   * #I4: Computes a dilated 64x64 Super-Chunk corridor mask between `startSuper` and `goalSuper`
   * to prune long-range A* portal expansions on large maps.
   */
  _computeSuperCorridorMask(startSuper, goalSuper) {
    const { graph, _superMask: mask } = this;
    const S = graph.superChunkCount, scx = graph.scx, scy = graph.scy, sLC = graph.superLayerCount;
    mask.fill(0);
    const dist = new Float32Array(S).fill(Infinity);
    const parent = new Int32Array(S).fill(-1);
    const heap = this.heap;
    heap.clear();
    dist[startSuper] = 0;
    heap.push(startSuper, 0);

    const gRem = goalSuper % sLC, gx = gRem % scx, gy = (gRem / scx) | 0;
    while (!heap.isEmpty) {
      const u = heap.pop();
      if (u === goalSuper) break;
      const du = dist[u];
      for (const [v, w] of graph.superAdj[u]) {
        const nd = du + w;
        if (nd < dist[v]) {
          dist[v] = nd;
          parent[v] = u;
          const vRem = v % sLC, vx = vRem % scx, vy = (vRem / scx) | 0;
          heap.push(v, nd + octile(vx - gx, vy - gy) * (SUPER_STEP * CS * 0.5));
        }
      }
    }
    if (dist[goalSuper] === Infinity) return null;

    // Mark optimal super-chunk chain + 1-super-chunk halo dilation for smooth cornering
    for (let cur = goalSuper; cur !== -1; cur = parent[cur]) {
      const z = (cur / sLC) | 0, rem = cur % sLC, x = rem % scx, y = (rem / scx) | 0;
      for (let dy = -1; dy <= 1; dy++) {
        for (let dx = -1; dx <= 1; dx++) {
          const nx = x + dx, ny = y + dy;
          if (nx >= 0 && nx < scx && ny >= 0 && ny < scy) {
            mask[z * sLC + ny * scx + nx] = 1;
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
      if (p.isVertical) {
        lx[i] = rx[i] = p.anchorX; ly[i] = ry[i] = p.anchorY;
        continue;
      }
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
      funnelX[stepWrite] = p.isVertical ? p.anchorX : clamp(gx, p.isH ? p.start + 0.5 : p.border, p.isH ? p.end + 0.5 : p.border);
      funnelY[stepWrite] = p.isVertical ? p.anchorY : clamp(gy, p.isH ? p.border : p.start + 0.5, p.isH ? p.border : p.end + 0.5);
      stepWrite++;
    }
    return { funnelX, funnelY };
  }
  //#endregion

  //#region 23: MacroPathfinder Hierarchical A* & Swarm Broadcast Solver (#B3, #I4)
  findPath(startCell, goalOrCell, useSuperCorridor = true) {
    const goal = typeof goalOrCell === 'number'
      ? this.getOrCreateGoal(goalOrCell)
      : (Array.isArray(goalOrCell) ? this.createMultiGoal(goalOrCell) : goalOrCell);
    if (!goal) return null;
    const { grid, graph, g, parent, parentChunk, seen, closed, heap } = this;
    const P = graph.portals.length, START = P, GOAL = P + 1;
    const sz = (startCell / grid.layerCells) | 0, sRem = startCell % grid.layerCells;
    const sx = sRem % grid.width, sy = (sRem / grid.width) | 0, sc = grid.chunkIdAt(sx, sy, sz);
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
      const gLo = goal.sccMaskLo, gHi = goal.sccMaskHi;
      for (let i = 0; i < cPorts.length; i++) {
        const pid = cPorts[i], p = graph.portals[pid];
        if (this.startField[pLocal(grid, p, sc)] < Infinity && crossingCost(grid, p, sc, graph.profile) < Infinity) {
          if (!goal.compSet.has(graph.portalComp[pid])) continue;
          const uScc = graph.portalScc[pid];
          if (((graph.sccReachLo[uScc] & gLo) !== 0) || ((graph.sccReachHi[uScc] & gHi) !== 0)) {
            reachableMatch = true;
            break;
          }
        }
      }
      if (!reachableMatch) return null;
    }

    // #I4: Use Super-Chunk corridor pruning when start and single goal are far apart (> 2 super-chunks)
    let superMask = null;
    if (useSuperCorridor && !sameChunkDirect && !goal.isMulti && graph.superChunkCount >= 9) {
      const sSuper = graph.chunkToSuper[sc], gSuper = graph.chunkToSuper[goal.chunk];
      const sDist = Math.abs((sSuper % graph.scx) - (gSuper % graph.scx)) + Math.abs(((sSuper / graph.scx) | 0) - ((gSuper / graph.scx) | 0));
      if (sDist >= 3) {
        superMask = this._computeSuperCorridorMask(sSuper, gSuper);
      }
    }

    const runSearch = (corridorMask) => {
      if (++this.stamp >= 0xffffffff) { seen.fill(0); closed.fill(0); this.stamp = 1; }
      const s = this.stamp;

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
        if (corridorMask && v < P && !corridorMask[graph.chunkToSuper[chunk]]) return;
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
          const fz = (chosenGoalCell / grid.layerCells) | 0, fRem = chosenGoalCell % grid.layerCells;
          const fgx = (fRem % grid.width) + 0.5, fgy = ((fRem / grid.width) | 0) + 0.5;
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
            goalLayer: fz,
            minClearance: graph.minClearance,
            profile: graph.profile,
            footprint: graph.footprint,
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
            const rz = (rCell / grid.layerCells) | 0, rRem = rCell % grid.layerCells;
            const rChunk = grid.chunkIdAt(rRem % grid.width, (rRem / grid.width) | 0, rz);
            relax(GOAL, gu + gc, u, rChunk, rCell);
          }
        }
      }
      return null;
    };

    const res = runSearch(superMask);
    // Fallback to unconstrained search if dilated super-corridor was blocked by an internal maze
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
      goal,
      goalX: goal.x + 0.5,
      goalY: goal.y + 0.5,
      goalLayer: goal.layer ?? 0,
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
  //#endregion
}

//#region 24: Chunk Supercover LOS & Flow Vector Gradient Extraction (#3, #A1, #F2)
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

//#region 25: Multi-Chunk Geodesic Corridor & Spline Flow Field Builder (#10, #D2, #F4)
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

/**
 * #A1, #B4, #F1, #F2, #F4, #F5: Collision-proof LRU cache of per-chunk flow fields backed by `FieldBufferPool`.
 */
export class FlowFieldCache {
  //#region 26: FlowFieldCache Core LRU & Collision-Proof Hashing (#B4, #F1)
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
  //#endregion

  //#region 27: FlowFieldCache Portal, Goal, Swarm & Multi-Goal Builders (#A1, #B3, #F2, #F5)
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
  //#endregion

  //#region 28: FlowFieldCache Vortex, Flee, Corridor & Invalidation Builders (#7, #8, #B4, #F4)
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
  //#endregion
}

//#region 29: Bilinear Flow Field Sampler (#A3)
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

//#region 30: Agent State Constants (#G2)
/** Agent state constants for `CrowdSystem` (#G2) */
export const AGENT_STATE = Object.freeze({
  IDLE: 0,
  MOVING: 1,
  ARRIVING: 2,
  SETTLED: 3
});
//#endregion

/**
 * #C2, #G1, #G2, #G4: Structure-of-Arrays (SoA) Crowd Steering Simulator featuring:
 * - #G1: Predictive RVO-Lite (Time-to-Collision $\tau$ horizon avoidance) + Continuous Wall-SDF Repulsion
 * - #G2: Contagious Goal-Arrival Settling (stops destination mosh-pitting) & Stuck-Agent Auto-Repathing
 * - #G4: Automatic portal crossing traffic recording for Wardrop load-balancing
 */
export class CrowdSystem {
  //#region 31: CrowdSystem SoA Allocation & Agent Lifecycle (#C2)
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
  //#endregion

  //#region 32: CrowdSystem RVO-Lite, Contagious Settling & Step Simulation (#G1, #G2, #G4)
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
  //#endregion
}

/**
 * #G3: Hierarchical Squad & Formation Controller (`SquadSystem`).
 * Manages virtual squad leaders, rubber-banded leader speed throttling when squad members lag in bottlenecks,
 * and formation slot presets (`'line'`, `'column'`, `'wedge'`, `'box'`, `'circle'`).
 */
export class SquadSystem {
  //#region 33: SquadSystem Formation Slot Generator & Squad Management (#G3)
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
  //#endregion

  //#region 34: SquadSystem Rubber-Banded Leader & Formation Step (#C4, #G3)
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
  //#endregion
}

//#region 35: Async Web Worker & SharedArrayBuffer Pipeline (#I3)
/**
 * #I3: Web Worker + `SharedArrayBuffer` Lock-Free Async Pipeline.
 * Offloads graph rebuilds, pathfinding, and swarm field generation to a background Web Worker
 * operating directly over shared memory (with automatic inline fallback if Workers are unavailable).
 */
export class AsyncPathfinderPipeline {
  constructor(pathfinder, workerUrl = null) {
    this.pf = pathfinder;
    this.worker = null;
    this._reqId = 1;
    this._pending = new Map();

    if (typeof Worker !== 'undefined' && (workerUrl || typeof import.meta !== 'undefined')) {
      try {
        const modUrl = workerUrl || import.meta.url;
        const src = `
          import { GridWorld, PortalGraph, MacroPathfinder } from ${JSON.stringify(modUrl)};
          let grid = null, graphs = new Map();
          const getHier = (clr = 1, prof = null, fp = null) => {
            const key = JSON.stringify([clr, prof, fp]);
            let h = graphs.get(key);
            if (!h) {
              const graph = new PortalGraph(grid, null, null, clr, prof, fp);
              const macro = new MacroPathfinder(grid, graph);
              h = { graph, macro };
              graphs.set(key, h);
            }
            return h;
          };
          self.onmessage = (e) => {
            const { id, cmd, payload } = e.data;
            if (cmd === 'init') {
              grid = GridWorld.fromSharedDescriptor(payload);
              self.postMessage({ id, ok: true });
            } else if (cmd === 'rebuild') {
              if (payload.dirty) {
                for (const c of payload.dirty) grid.dirtyChunks.add(c);
              }
              const all = grid.consumeDirtyChunks();
              for (const [k, h] of graphs) {
                const nextG = new PortalGraph(grid, h.graph, all, h.graph.minClearance, h.graph.profile, h.graph.footprint);
                graphs.set(k, { graph: nextG, macro: new MacroPathfinder(grid, nextG) });
              }
              self.postMessage({ id, ok: true });
            } else if (cmd === 'findPath') {
              const { startCell, goalCell, clr, prof, fp } = payload;
              const { macro } = getHier(clr, prof, fp);
              const p = macro.findPath(startCell, goalCell);
              self.postMessage({
                id,
                path: p ? {
                  portals: p.portals, chunks: p.chunks, funnelX: p.funnelX, funnelY: p.funnelY,
                  cost: p.cost, goalCell: p.goalCell, goalX: p.goalX, goalY: p.goalY,
                  goalLayer: p.goalLayer, minClearance: p.minClearance, profile: p.profile
                } : null
              });
            }
          };
        `;
        const blob = new Blob([src], { type: 'application/javascript' });
        this.worker = new Worker(URL.createObjectURL(blob), { type: 'module' });
        this.worker.onmessage = (e) => {
          const { id, ...data } = e.data;
          const res = this._pending.get(id);
          if (res) { this._pending.delete(id); res(data); }
        };
        this._send('init', this.pf.grid.toSharedDescriptor());
      } catch (_) {
        this.worker = null;
      }
    }
  }

  _send(cmd, payload) {
    if (!this.worker) return Promise.resolve(null);
    const id = this._reqId++;
    return new Promise(resolve => {
      this._pending.set(id, resolve);
      this.worker.postMessage({ id, cmd, payload });
    });
  }

  async rebuildAsync() {
    const dirty = Array.from(this.pf.grid.dirtyChunks);
    this.pf.rebuild();
    if (this.worker) await this._send('rebuild', { dirty });
    return true;
  }

  async findPathAsync(sx, sy, gx, gy, opts = {}) {
    if (!this.worker) return this.pf.findPath(sx, sy, gx, gy, opts);
    const { grid } = this.pf;
    const clr = opts.minClearance ?? this.pf.minClearance;
    const prof = opts.profile ?? null;
    const fp = opts.footprint ?? null;
    const sz = opts.startLayer ?? opts.layer ?? 0, gz = opts.goalLayer ?? opts.layer ?? 0;
    const scx = clamp(sx | 0, 0, grid.width - 1), scy = clamp(sy | 0, 0, grid.height - 1);
    const gcx = clamp(gx | 0, 0, grid.width - 1), gcy = clamp(gy | 0, 0, grid.height - 1);
    const startCell = grid.isWalkable(scx, scy, clr, prof, sz, fp) ? grid.index(scx, scy, sz) : grid.findNearestWalkable(scx, scy, 16, clr, prof, sz, fp);
    const goalCell = grid.isWalkable(gcx, gcy, clr, prof, gz, fp) ? grid.index(gcx, gcy, gz) : grid.findNearestWalkable(gcx, gcy, 16, clr, prof, gz, fp);
    if (startCell < 0 || goalCell < 0) return null;

    const { graph } = this.pf._getHierarchy(clr, prof, fp);
    const res = await this._send('findPath', { startCell, goalCell, clr, prof, fp });
    if (res && res.path) {
      res.path.graph = graph;
      res.path.footprint = fp;
      return res.path;
    }
    return null;
  }

  terminate() {
    if (this.worker) {
      this.worker.terminate();
      this.worker = null;
    }
    this._pending.clear();
  }
}
//#endregion

/**
 * Unified Hierarchical Flow Field Pathfinding API (v3.0) exposing all flow field types,
 * Multi-Floor 2.5D Stacked Grids (#H3), Anisotropic Rectangular Vehicle Footprints (#H4),
 * Web Worker + SharedArrayBuffer Async Pipeline (#I3), and Two-Level 64x64 Super-Chunks (#I4).
 */
export class FlowFieldPathfinder {
  //#region 36: FlowFieldPathfinder Initialization & World Mutator Delegates (#C3, #D4, #E1, #H3)
  constructor(opts = {}) {
    const cx = opts.width ? Math.ceil(opts.width / CS) : (opts.chunksX ?? 10);
    const cy = opts.height ? Math.ceil(opts.height / CS) : (opts.chunksY ?? 10);
    this.cacheCap = opts.cacheCap ?? 2048;
    this.minClearance = opts.minClearance ?? 1;
    this.portalLookahead = opts.portalLookahead ?? true;
    this.seamInterpolation = opts.seamInterpolation ?? true;
    this.useSuperChunks = opts.useSuperChunks ?? true;

    this.grid = new GridWorld(cx, cy, opts.costs || null, {
      layers: opts.layers ?? 1,
      shared: opts.shared ?? false
    });
    if (opts.useEikonal !== undefined) this.grid.useEikonal = opts.useEikonal;
    if (opts.losSmoothing !== undefined) this.grid.losSmoothing = opts.losSmoothing;
    if (opts.wallRepulsion) this.grid.enableWallRepulsion(opts.wallRepulsion);

    this.graph = new PortalGraph(this.grid, null, null, this.minClearance, null, null);
    this.macro = new MacroPathfinder(this.grid, this.graph);
    this.cache = new FlowFieldCache(this.grid, this.cacheCap);
    this.clearanceGraphs = new Map([[this._hierarchyKey(this.minClearance, null, null), { graph: this.graph, macro: this.macro }]]);
    this._workQueue = [];
    this._tmpVec = { x: 0, y: 0 };
  }

  get width() { return this.grid.width; }
  get height() { return this.grid.height; }
  get layers() { return this.grid.layers; }

  _hierarchyKey(minClearance, profile = null, footprint = null) {
    if (!profile && !footprint) return minClearance;
    const fpKey = footprint ? `${footprint.width ?? 1}x${footprint.length ?? 1}:${footprint.turnPenalty ?? 2}` : '';
    return `${minClearance}:${this.grid.getProfileId(profile)}:${fpKey}`;
  }

  getCost(x, y, z = 0) { return this.grid.getCost(x, y, z); }
  setCost(x, y, cost, z = 0) { return this.grid.setCost(x, y, cost, z); }
  setRect(x, y, w, h, cost, z = 0) { return this.grid.setRect(x, y, w, h, cost, z); }
  setDisc(x, y, r, cost, z = 0) { return this.grid.setDisc(x, y, r, cost, z); }
  loadCosts(costs, layer = null) { this.grid.loadCosts(costs, layer); }

  // #H3: 2.5D Stair / Ramp / Overpass Portal registration
  addStairPortal(x, y, w = 2, h = 2, fromLayer = 0, toLayer = 1, opts = {}) {
    return this.grid.addStairPortal(x, y, w, h, fromLayer, toLayer, opts);
  }

  // #D4: Terrain & Profile API delegates
  registerProfile(name, costMapOrArray) { return this.grid.registerProfile(name, costMapOrArray); }
  setTerrain(x, y, terrainId, z = 0) { return this.grid.setTerrain(x, y, terrainId, z); }
  setRectTerrain(x, y, w, h, terrainId, z = 0) { return this.grid.setRectTerrain(x, y, w, h, terrainId, z); }

  // #C3: Dynamic Moving Obstacle API delegates
  stampDynamicObstacle(wx, wy, r, blocked = 1, z = 0) { this.grid.stampDynamicObstacle(wx, wy, r, blocked, z); }
  clearDynamicObstacles() { this.grid.clearDynamicObstacles(); }

  // #E1: World Raycast & Tactical FOV Shadowcasting delegates
  raycast(x0, y0, x1, y1, opts = {}) { return this.grid.raycast(x0, y0, x1, y1, opts); }
  addTacticalFOV(wx, wy, r, peakCost, linearFalloff = true, z = 0) { this.grid.addTacticalFOV(wx, wy, r, peakCost, linearFalloff, null, z); }

  // #C2, #G3 & #I3: Factory helpers for CrowdSystem, SquadSystem, and Async Worker Pipeline
  createCrowd(capacity = 4096, opts = {}) { return new CrowdSystem(this, capacity, opts); }
  createSquadSystem(crowd = null) { return new SquadSystem(this, crowd); }
  createAsyncPipeline(workerUrl = null) { return new AsyncPathfinderPipeline(this, workerUrl); }
  //#endregion

  //#region 37: FlowFieldPathfinder Incremental Hierarchy Rebuild (#B1)
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
  //#endregion

  //#region 38: FlowFieldPathfinder Macro Pathfinding & Time-Sliced Work Queue (#H3, #H4, #I4)
  /**
   * Computes a single-goal hierarchical macro path (supports `opts.minClearance`, `opts.profile`,
   * `opts.footprint = { width, length, turnPenalty }` (#H4), and `opts.startLayer` / `opts.goalLayer` (#H3)).
   */
  findPath(sx, sy, gx, gy, opts = {}) {
    const clr = opts.minClearance ?? this.minClearance;
    const prof = opts.profile ?? null;
    const fp = opts.footprint ?? null;
    const sz = clamp((opts.startLayer ?? opts.layer ?? 0) | 0, 0, this.grid.layers - 1);
    const gz = clamp((opts.goalLayer ?? opts.layer ?? 0) | 0, 0, this.grid.layers - 1);
    const { macro } = this._getHierarchy(clr, prof, fp);
    const { grid } = this;
    const scx = clamp(sx | 0, 0, grid.width - 1), scy = clamp(sy | 0, 0, grid.height - 1);
    const gcx = clamp(gx | 0, 0, grid.width - 1), gcy = clamp(gy | 0, 0, grid.height - 1);
    const startCell = grid.isWalkable(scx, scy, clr, prof, sz, fp) ? grid.index(scx, scy, sz) : grid.findNearestWalkable(scx, scy, 16, clr, prof, sz, fp);
    const goalCell = grid.isWalkable(gcx, gcy, clr, prof, gz, fp) ? grid.index(gcx, gcy, gz) : grid.findNearestWalkable(gcx, gcy, 16, clr, prof, gz, fp);
    if (startCell < 0 || goalCell < 0) return null;
    return macro.findPath(startCell, goalCell, opts.useSuperChunks ?? this.useSuperChunks);
  }

  findMultiGoalPath(sx, sy, goals, opts = {}) {
    const clr = opts.minClearance ?? this.minClearance;
    const prof = opts.profile ?? null;
    const fp = opts.footprint ?? null;
    const sz = clamp((opts.startLayer ?? opts.layer ?? 0) | 0, 0, this.grid.layers - 1);
    const { macro } = this._getHierarchy(clr, prof, fp);
    const { grid } = this;
    const scx = clamp(sx | 0, 0, grid.width - 1), scy = clamp(sy | 0, 0, grid.height - 1);
    const startCell = grid.isWalkable(scx, scy, clr, prof, sz, fp) ? grid.index(scx, scy, sz) : grid.findNearestWalkable(scx, scy, 16, clr, prof, sz, fp);
    if (startCell < 0) return null;
    const validTargets = [];
    for (const g of goals) {
      const gx = clamp(g.x | 0, 0, grid.width - 1), gy = clamp(g.y | 0, 0, grid.height - 1);
      const gz = clamp((g.layer ?? opts.goalLayer ?? opts.layer ?? 0) | 0, 0, grid.layers - 1);
      const cell = grid.isWalkable(gx, gy, clr, prof, gz, fp) ? grid.index(gx, gy, gz) : grid.findNearestWalkable(gx, gy, 16, clr, prof, gz, fp);
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
  //#endregion

  //#region 39: FlowFieldPathfinder Swarm, Vortex, Flee & Corridor Queries (#7, #8, #10, #B3)
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
      const gz = clamp((g.layer ?? opts.layer ?? 0) | 0, 0, grid.layers - 1);
      const cell = grid.isWalkable(gx, gy, clr, prof, gz, fp) ? grid.index(gx, gy, gz) : grid.findNearestWalkable(gx, gy, 16, clr, prof, gz, fp);
      if (cell >= 0) valid.push({ cell, weight: g.weight ?? 0 });
    }
    if (!valid.length) return null;
    return macro.createSwarmField(valid.length === 1 && valid[0].weight === 0 ? valid[0].cell : valid);
  }

  getFieldForSwarm(swarm, wx, wy, layer = 0) {
    if (!swarm) return null;
    const { graph } = this._getHierarchy(swarm.minClearance, swarm.profile, swarm.footprint);
    if (swarm.graph !== graph) return null;
    const { grid } = this;
    const cx = clamp(wx | 0, 0, grid.width - 1), cy = clamp(wy | 0, 0, grid.height - 1);
    const cz = clamp(layer | 0, 0, grid.layers - 1);
    const chunk = grid.chunkIdAt(cx, cy, cz);
    return this.cache.getSwarmChunkField(chunk, swarm);
  }

  sampleSwarmFlow(swarm, wx, wy, out = { x: 0, y: 0 }, layer = 0) {
    const field = this.getFieldForSwarm(swarm, wx, wy, layer);
    if (!field) { out.x = 0; out.y = 0; return false; }
    const resolver = this.seamInterpolation ? (nx, ny) => {
      if (!this.grid.inBounds(nx, ny, layer)) return null;
      return this.cache.getSwarmChunkField(this.grid.chunkIdAt(nx, ny, layer), swarm);
    } : null;
    return sampleFlowBilinear(field, wx, wy, out, resolver);
  }

  getVortexField(wx, wy, opts = {}) {
    const { grid } = this;
    const clr = opts.minClearance ?? this.minClearance;
    const layer = clamp((opts.layer ?? 0) | 0, 0, grid.layers - 1);
    const { graph, macro } = this._getHierarchy(clr, opts.profile, opts.footprint);
    const qx = clamp(wx | 0, 0, grid.width - 1), qy = clamp(wy | 0, 0, grid.height - 1);
    const chunk = grid.chunkIdAt(qx, qy, layer);
    const cx = clamp((opts.centerX ?? wx) | 0, 0, grid.width - 1);
    const cy = clamp((opts.centerY ?? wy) | 0, 0, grid.height - 1);
    return this.cache.getVortexField(chunk, grid.index(cx, cy, layer), { ...opts, graph, macro });
  }

  getFleeField(wx, wy, threatPoints, opts = {}) {
    const { grid } = this;
    const clr = opts.minClearance ?? this.minClearance;
    const layer = clamp((opts.layer ?? 0) | 0, 0, grid.layers - 1);
    const { graph } = this._getHierarchy(clr, opts.profile, opts.footprint);
    const cx = clamp(wx | 0, 0, grid.width - 1), cy = clamp(wy | 0, 0, grid.height - 1);
    return this.cache.getFleeField(grid.chunkIdAt(cx, cy, layer), threatPoints, { ...opts, graph });
  }

  getCorridorField(wx, wy, polyline, opts = {}) {
    const { grid } = this;
    const clr = opts.minClearance ?? this.minClearance;
    const layer = clamp((opts.layer ?? 0) | 0, 0, grid.layers - 1);
    const { graph } = this._getHierarchy(clr, opts.profile, opts.footprint);
    const cx = clamp(wx | 0, 0, grid.width - 1), cy = clamp(wy | 0, 0, grid.height - 1);
    return this.cache.getCorridorField(grid.chunkIdAt(cx, cy, layer), polyline, { ...opts, graph });
  }
  //#endregion

  //#region 40: FlowFieldPathfinder Tactical Cover/Vantage & Reachable Area Queries (#E2, #E3)
  findTacticalPosition(wx, wy, threatsOrTargets, opts = {}) {
    const mode = opts.mode ?? 'cover';
    const searchR = opts.searchRadius ?? 12;
    const minClearance = opts.minClearance ?? this.minClearance;
    const profile = opts.profile ?? null;
    const layer = clamp((opts.layer ?? 0) | 0, 0, this.grid.layers - 1);
    const { grid } = this, W = grid.width, H = grid.height, zOff = layer * grid.layerCells;
    const threats = Array.isArray(threatsOrTargets) ? threatsOrTargets : [threatsOrTargets];
    if (!threats.length) return null;

    const seenByEnemy = new Uint8Array(grid.layerCells);
    for (let i = 0; i < threats.length; i++) {
      const t = threats[i];
      grid.addTacticalFOV(t.x, t.y, searchR + 16, 0, false, (mx, my, idx) => {
        seenByEnemy[idx - zOff] = 1;
      }, layer);
    }

    const sx = clamp(wx | 0, 0, W - 1), sy = clamp(wy | 0, 0, H - 1);
    const x0 = Math.max(0, Math.floor(sx - searchR)), x1 = Math.min(W - 1, Math.ceil(sx + searchR));
    const y0 = Math.max(0, Math.floor(sy - searchR)), y1 = Math.min(H - 1, Math.ceil(sy + searchR));
    const r2 = searchR * searchR;
    const minRange = opts.minRange ?? 3.0, maxRange = opts.maxRange ?? searchR;
    let bestScore = Infinity, bestX = -1, bestY = -1, bestCell = -1;

    for (let y = y0; y <= y1; y++) {
      for (let x = x0; x <= x1; x++) {
        const dAgent2 = (x + 0.5 - wx) ** 2 + (y + 0.5 - wy) ** 2;
        if (dAgent2 > r2 || !grid.isWalkable(x, y, minClearance, profile, layer)) continue;
        const lIdx = y * W + x, gIdx = zOff + lIdx;
        const dAgent = Math.sqrt(dAgent2);
        const tacPenalty = grid.hasTactical && grid.tacticalCosts ? grid.tacticalCosts[gIdx] : 0;

        if (mode === 'cover') {
          if (seenByEnemy[lIdx]) continue;
          let adjWallBonus = 0;
          for (let d = 0; d < 4; d++) {
            const nx = x + DX[d], ny = y + DY[d];
            if (!grid.inBounds(nx, ny, layer) || grid.costs[zOff + ny * W + nx] >= COST_BLOCK) adjWallBonus += 1.5;
          }
          const score = dAgent + tacPenalty - adjWallBonus;
          if (score < bestScore) {
            bestScore = score; bestX = x + 0.5; bestY = y + 0.5; bestCell = gIdx;
          }
        } else {
          const t0 = threats[0];
          const dt = Math.hypot((x + 0.5) - t0.x, (y + 0.5) - t0.y);
          if (dt < minRange || dt > maxRange || !seenByEnemy[lIdx]) continue;
          const score = dAgent + tacPenalty * 2.0 + Math.abs(dt - (minRange + maxRange) * 0.5) * 0.25;
          if (score < bestScore) {
            bestScore = score; bestX = x + 0.5; bestY = y + 0.5; bestCell = gIdx;
          }
        }
      }
    }

    if (bestCell < 0) return null;
    return { x: bestX, y: bestY, layer, cell: bestCell, score: bestScore, path: this.findPath(wx, wy, bestX, bestY, opts) };
  }

  queryReachableArea(sx, sy, maxCostBudget, opts = {}) {
    const minClearance = opts.minClearance ?? this.minClearance;
    const profile = opts.profile ?? null;
    const layer = clamp((opts.layer ?? 0) | 0, 0, this.grid.layers - 1);
    const { grid } = this, W = grid.width, zOff = layer * grid.layerCells;
    const pTable = grid.getProfileTable(profile);
    if (minClearance > 1 && (grid._clearanceDirty || !grid.clearance)) grid.updateClearance();

    const scx = clamp(sx | 0, 0, W - 1), scy = clamp(sy | 0, 0, grid.height - 1);
    const startCell = grid.isWalkable(scx, scy, minClearance, profile, layer)
      ? grid.index(scx, scy, layer)
      : grid.findNearestWalkable(scx, scy, 8, minClearance, profile, layer);
    if (startCell < 0) return { cells: new Int32Array(0), costs: new Float32Array(0), borderCells: new Int32Array(0), getPathTo: () => null };

    const distMap = new Map([[startCell, 0]]);
    const parentMap = new Map([[startCell, -1]]);
    const heap = new MinHeap(256);
    heap.push(startCell, 0);

    const reachedCells = [];
    const reachedCosts = [];
    const borderCells = [];

    while (!heap.isEmpty) {
      const u = heap.pop(), du = heap.lastKey;
      if (du > distMap.get(u)) continue;
      reachedCells.push(u);
      reachedCosts.push(du);

      const rem = u - zOff, ux = rem % W, uy = (rem / W) | 0;
      const cu = grid.getEffectiveBaseCost(u, pTable);
      let isBorder = false;

      for (let d = 0; d < 8; d++) {
        const nx = ux + DX[d], ny = uy + DY[d];
        if (!grid.inBounds(nx, ny, layer)) { isBorder = true; continue; }
        const v = zOff + ny * W + nx;
        const cv = grid.getEffectiveBaseCost(v, pTable);
        if (cv >= COST_BLOCK || (minClearance > 1 && grid.clearance[v] < minClearance)) {
          isBorder = true; continue;
        }
        if (d >= 4 && (grid.getEffectiveBaseCost(zOff + uy * W + nx, pTable) >= COST_BLOCK || grid.getEffectiveBaseCost(zOff + ny * W + ux, pTable) >= COST_BLOCK)) {
          continue;
        }
        const step = DL[d] * ((cu + cv) * 0.5 / COST_OPEN);
        const nd = du + step;
        if (nd > maxCostBudget) {
          isBorder = true;
          continue;
        }
        const prev = distMap.get(v);
        if (prev === undefined || nd < prev) {
          distMap.set(v, nd);
          parentMap.set(v, u);
          heap.push(v, nd);
        }
      }
      if (isBorder) borderCells.push(u);
    }

    return {
      cells: new Int32Array(reachedCells),
      costs: new Float32Array(reachedCosts),
      borderCells: new Int32Array(borderCells),
      costMap: distMap,
      getPathTo: (tx, ty) => {
        const tCell = zOff + (ty | 0) * W + (tx | 0);
        if (!parentMap.has(tCell)) return null;
        const trace = [];
        for (let cur = tCell; cur !== -1; cur = parentMap.get(cur) ?? -1) {
          const cRem = cur - zOff;
          trace.push({ x: (cRem % W) + 0.5, y: ((cRem / W) | 0) + 0.5, layer, cell: cur });
        }
        return trace.reverse();
      }
    };
  }
  //#endregion

  //#region 41: FlowFieldPathfinder Path Flow & Adaptive Formation Sampling (#A3, #C4, #F5)
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
    const fx = path.funnelX ? path.funnelX[stepIdx] : undefined;
    const fy = path.funnelY ? path.funnelY[stepIdx] : undefined;
    return this.cache.getPortalField(chunk, p, clr, nextField, nextTid, prof, fx, fy);
  }

  getFieldForPath(path, wx, wy, hintIdx = 0, layer = null) {
    if (!path) return null;
    const clr = path.minClearance ?? this.minClearance;
    const prof = path.profile ?? null;
    const fp = path.footprint ?? null;
    const { graph } = this._getHierarchy(clr, prof, fp);
    if (path.graph !== graph) return null;
    const { grid } = this;
    const cx = clamp(wx | 0, 0, grid.width - 1), cy = clamp(wy | 0, 0, grid.height - 1);
    const localIdx = (cy % CS) * CS + (cx % CS);

    // If layer is not explicitly passed, infer from hintIdx chunk's layer if valid
    const inferredZ = layer !== null ? clamp(layer | 0, 0, grid.layers - 1)
      : (hintIdx >= 0 && hintIdx < path.chunks.length ? grid.chunkLayer(path.chunks[hintIdx]) : 0);
    const chunk = grid.chunkIdAt(cx, cy, inferredZ);

    if (hintIdx >= 0 && hintIdx < path.chunks.length && path.chunks[hintIdx] === chunk) {
      const f = this._resolveStepField(path, graph, hintIdx);
      if (f.integration[localIdx] < Infinity) return { field: f, chunkIndex: hintIdx, layer: inferredZ };
    }

    for (let i = Math.max(0, hintIdx); i < path.chunks.length; i++) {
      const c = path.chunks[i];
      if (c === chunk || (layer === null && (c % grid.layerChunks) === (chunk % grid.layerChunks))) {
        const f = this._resolveStepField(path, graph, i);
        if (f.integration[localIdx] < Infinity) return { field: f, chunkIndex: i, layer: grid.chunkLayer(c) };
      }
    }
    for (let i = Math.min(path.chunks.length - 1, hintIdx - 1); i >= 0; i--) {
      const c = path.chunks[i];
      if (c === chunk || (layer === null && (c % grid.layerChunks) === (chunk % grid.layerChunks))) {
        const f = this._resolveStepField(path, graph, i);
        if (f.integration[localIdx] < Infinity) return { field: f, chunkIndex: i, layer: grid.chunkLayer(c) };
      }
    }
    return null;
  }

  sampleFlow(path, wx, wy, out = { x: 0, y: 0 }, hintIdx = 0, layer = null) {
    const res = this.getFieldForPath(path, wx, wy, hintIdx, layer);
    if (!res) { out.x = 0; out.y = 0; return false; }
    const activeLayer = res.layer;
    const resolver = this.seamInterpolation ? (nx, ny) => {
      if (!this.grid.inBounds(nx, ny, activeLayer)) return null;
      const nRes = this.getFieldForPath(path, nx, ny, res.chunkIndex, activeLayer);
      return nRes ? nRes.field : null;
    } : null;
    return sampleFlowBilinear(res.field, wx, wy, out, resolver);
  }

  sampleFormationFlow(pathOrSwarm, leaderX, leaderY, slotForward, slotRight, agentX, agentY, out = { x: 0, y: 0 }, opts = {}) {
    const { grid, _tmpVec } = this;
    const layer = clamp((opts.layer ?? 0) | 0, 0, grid.layers - 1);
    const isSwarm = pathOrSwarm && pathOrSwarm.isSwarm;
    const okLeader = isSwarm
      ? this.sampleSwarmFlow(pathOrSwarm, leaderX, leaderY, _tmpVec, layer)
      : this.sampleFlow(pathOrSwarm, leaderX, leaderY, _tmpVec, opts.leaderHintIdx ?? 0, layer);
    const lfx = okLeader ? _tmpVec.x : 1, lfy = okLeader ? _tmpVec.y : 0;

    if (grid._clearanceDirty || !grid.clearance) grid.updateClearance();
    const lCell = grid.index(clamp(leaderX | 0, 0, grid.width - 1), clamp(leaderY | 0, 0, grid.height - 1), layer);
    const localClr = grid.clearance[lCell];
    const formRadius = opts.formationRadius ?? Math.max(1.5, Math.abs(slotRight));
    const scale = clamp(localClr / formRadius, opts.minCompression ?? 0.2, 1.0);

    const rightX = -lfy, rightY = lfx;
    const targetX = leaderX + lfx * slotForward + rightX * (slotRight * scale);
    const targetY = leaderY + lfy * slotForward + rightY * (slotRight * scale);

    const okAgent = isSwarm
      ? this.sampleSwarmFlow(pathOrSwarm, agentX, agentY, out, layer)
      : this.sampleFlow(pathOrSwarm, agentX, agentY, out, opts.agentHintIdx ?? 0, layer);
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
  //#endregion
}

export default FlowFieldPathfinder;
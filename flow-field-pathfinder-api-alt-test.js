<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>Single-Agent Multi-Stage Obstacle Course — Flow-Field Pathfinder API</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace;
      background: #f6f7f9;
      color: #111827;
      font-size: 12px;
      line-height: 1.4;
      padding: 12px;
    }
    header {
      display: flex;
      flex-wrap: wrap;
      align-items: center;
      justify-content: space-between;
      gap: 8px;
      padding-bottom: 10px;
      margin-bottom: 10px;
      border-bottom: 1px solid #d1d5db;
    }
    h1 { font-size: 14px; font-weight: 700; }
    .bar { display: flex; flex-wrap: wrap; gap: 4px; align-items: center; }
    button, select, input[type="range"] {
      font-family: inherit;
      font-size: 11px;
      background: #ffffff;
      color: #111827;
      border: 1px solid #9ca3af;
      padding: 4px 8px;
      cursor: pointer;
    }
    button:hover { background: #f3f4f6; }
    button.active {
      background: #111827;
      color: #ffffff;
      border-color: #111827;
    }
    .layout {
      display: grid;
      grid-template-columns: auto 340px;
      gap: 12px;
      align-items: start;
    }
    @media (max-width: 1180px) {
      .layout { grid-template-columns: 1fr; }
    }
    .canvas-wrap {
      background: #ffffff;
      border: 1px solid #d1d5db;
      padding: 8px;
      display: inline-block;
      position: relative;
    }
    canvas {
      display: block;
      border: 1px solid #111827;
      cursor: crosshair;
      max-width: 100%;
      height: auto;
    }
    .banner {
      position: absolute;
      top: 18px;
      left: 50%;
      transform: translateX(-50%);
      background: rgba(17, 24, 39, 0.92);
      color: #fff;
      padding: 6px 14px;
      font-weight: 700;
      pointer-events: none;
      display: none;
      border: 1px solid #000;
    }
    .sidebar {
      display: flex;
      flex-direction: column;
      gap: 10px;
    }
    .panel {
      background: #ffffff;
      border: 1px solid #d1d5db;
      padding: 10px;
    }
    .panel h2 {
      font-size: 12px;
      font-weight: 700;
      text-transform: uppercase;
      border-bottom: 1px solid #e5e7eb;
      padding-bottom: 4px;
      margin-bottom: 8px;
    }
    .kv {
      display: grid;
      grid-template-columns: 135px 1fr;
      row-gap: 3px;
      font-size: 11px;
    }
    .kv span:first-child { color: #4b5563; }
    .kv span:last-child { font-weight: 600; text-align: right; }
    .controls-list {
      display: flex;
      flex-direction: column;
      gap: 6px;
    }
    .control-row {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 6px;
    }
    .tag-list {
      display: flex;
      flex-wrap: wrap;
      gap: 4px;
      margin-top: 6px;
    }
    .tag {
      background: #f3f4f6;
      border: 1px solid #d1d5db;
      padding: 1px 5px;
      font-size: 10px;
    }
    .legend {
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
      margin-top: 6px;
      font-size: 11px;
      color: #374151;
    }
    .swatch {
      display: inline-block;
      width: 10px;
      height: 10px;
      vertical-align: middle;
      margin-right: 4px;
      border: 1px solid #374151;
    }
  </style>
</head>
<body>
  <header>
    <div>
      <h1>AGENT-01 OBSTACLE COURSE // FLOW-FIELD-PATHFINDER-API-ALT.JS</h1>
      <div id="api-status" style="font-size:11px;color:#4b5563;">Loading flow-field-pathfinder-api-alt.js...</div>
    </div>
    <div class="bar" id="stage-buttons"></div>
    <div class="bar">
      <button id="btn-play" class="active">PAUSE</button>
      <button id="btn-step">STEP</button>
      <button id="btn-reset">RESET STAGE</button>
      <button id="btn-auto" class="active">AUTO-ADVANCE: ON</button>
      <select id="sel-speed" title="Simulation Speed">
        <option value="1">1x SPEED</option>
        <option value="1.75">1.75x SPEED</option>
        <option value="3">3x SPEED</option>
      </select>
    </div>
  </header>

  <div class="layout">
    <div class="canvas-wrap">
      <div id="stage-banner" class="banner">STAGE CLEARED</div>
      <canvas id="sim-canvas" width="864" height="576"></canvas>
      <div class="legend">
        <span><i class="swatch" style="background:#10b981;"></i>Start Pad</span>
        <span><i class="swatch" style="background:#ef4444;"></i>Goal Beacon</span>
        <span><i class="swatch" style="background:#2563eb;"></i>Agent-01 &amp; Hull</span>
        <span><i class="swatch" style="background:#f59e0b;"></i>Chunk Portals / Stairs</span>
        <span><i class="swatch" style="background:#dc2626;"></i>Tactical FOV / Dynamic Block</span>
        <span style="margin-left:auto;">Click Canvas: <strong id="click-mode-label">Toggle Wall Cell</strong></span>
      </div>
    </div>

    <div class="sidebar">
      <div class="panel">
        <h2 id="stage-title">Stage 1</h2>
        <p id="stage-desc" style="font-size:11px;color:#374151;margin-bottom:6px;"></p>
        <div class="tag-list" id="stage-tags"></div>
      </div>

      <div class="panel">
        <h2>Stage Feature Controls</h2>
        <div id="stage-controls" class="controls-list"></div>
      </div>

      <div class="panel">
        <h2>Render &amp; Interaction Overlays</h2>
        <div class="controls-list">
          <div class="control-row">
            <label><input type="checkbox" id="chk-flow" checked /> Flow Vectors</label>
            <label><input type="checkbox" id="chk-dist" checked /> Integration Heat</label>
          </div>
          <div class="control-row">
            <label><input type="checkbox" id="chk-portals" checked /> Chunk Portals</label>
            <label><input type="checkbox" id="chk-rays" checked /> Agent LOS Rays</label>
          </div>
          <div class="control-row">
            <label><input type="checkbox" id="chk-special" checked /> Stage Field (SDF/Clearance/FOV)</label>
          </div>
          <div class="control-row">
            <span>Canvas Click Action:</span>
            <select id="sel-click-mode">
              <option value="wall">Toggle Wall (setCost)</option>
              <option value="goal">Move Goal Beacon</option>
              <option value="teleport">Teleport Agent-01</option>
            </select>
          </div>
        </div>
      </div>

      <div class="panel">
        <h2>Agent-01 &amp; Grid Telemetry</h2>
        <div class="kv">
          <span>Stage Progress:</span><span id="tel-stage">1 / 6</span>
          <span>Agent Position (x,y,z):</span><span id="tel-pos">0.0, 0.0, L0</span>
          <span>Agent Velocity / Speed:</span><span id="tel-vel">0.00 cells/s</span>
          <span>Goal Distance (Eikonal):</span><span id="tel-dist">0.00</span>
          <span>Cell Base / Eff. Cost:</span><span id="tel-cost">2 / 2.0</span>
          <span>Clearance (Iso / X / Y):</span><span id="tel-clr">1 / 1 / 1</span>
          <span>Wall SDF Distance:</span><span id="tel-sdf">N/A</span>
          <span>Goal Raycast LOS:</span><span id="tel-los">Blocked</span>
          <span>Detected Portals:</span><span id="tel-portals">0</span>
          <span>Course Completed:</span><span id="tel-laps">0 runs</span>
        </div>
      </div>
    </div>
  </div>

  <script type="module">
    const API_URL = 'https://raw.githubusercontent.com/438885/flow-field-pathfinder-api/refs/heads/main/flow-field-pathfinder-api-alt.js';

    async function loadPathfinderAPI() {
      try {
        return await import('https://raw.githubusercontent.com/438885/flow-field-pathfinder-api/refs/heads/main/flow-field-pathfinder-api-alt.js');
      } catch (_) {
        // Fallback for browsers that enforce strict MIME checks against raw.githubusercontent.com text/plain responses
        const response = await fetch(API_URL);
        if (!response.ok) throw new Error(`HTTP ${response.status} loading ${API_URL}`);
        const source = await response.text();
        const blobUrl = URL.createObjectURL(new Blob([source], { type: 'application/javascript' }));
        try {
          return await import(blobUrl);
        } finally {
          URL.revokeObjectURL(blobUrl);
        }
      }
    }

    const FF = await loadPathfinderAPI();
    const {
      CS, CC, COST_OPEN, COST_BLOCK, MAX_PORTAL_LEN,
      DX, DY, DL, DIR_FLAGS, clamp,
      MinHeap, FieldBufferPool, GridWorld, detectPortals, ChunkIntegrator
    } = FF;

    document.getElementById('api-status').textContent =
      `Loaded ${Object.keys(FF).length} exports from flow-field-pathfinder-api-alt.js (CS=${CS}, COST_OPEN=${COST_OPEN}, COST_BLOCK=${COST_BLOCK})`;

    // Grid dimensions: 3x2 chunks = 48x32 cells
    const CX = 3, CY = 2;
    const GW = CX * CS; // 48
    const GH = CY * CS; // 32
    const SCALE = 18;   // 864x576px canvas

    const canvas = document.getElementById('sim-canvas');
    const ctx = canvas.getContext('2d');
    const bufferPool = new FieldBufferPool(128);
    const chunkIntegrator = new ChunkIntegrator();
    const solverHeap = new MinHeap(GW * GH * 4);

    // Simulation state
    let currentStageIdx = 0;
    let running = true;
    let autoAdvance = true;
    let simSpeed = 1.0;
    let stageTime = 0;
    let completedCourses = 0;
    let bannerTimer = 0;

    let grid = null;
    let portals = [];
    let distField = null;
    let flowX = null;
    let flowY = null;
    let portalTargetLayer = null; // Per-cell vertical transition target layer (-1 if none)
    let visibleFOVCells = new Set();

    // Prepared per-cell cache populated via ChunkIntegrator.prepareChunk()
    let cellCosts = null;
    let cellBlocked = null;
    let cellDirMasks = null;
    let cellTurnPen = null;
    let cellCurX = null;
    let cellCurY = null;
    let cellFluxX = null;
    let cellFluxY = null;

    // Single Protagonist Agent
    const agent = {
      x: 4.5,
      y: 16.5,
      z: 0,
      vx: 0,
      vy: 0,
      heading: 0,
      speed: 7.2,
      minClearance: 1,
      profile: null,
      footprint: null,
      layerCooldown: 0,
      stuckTimer: 0,
      trail: [],
      fanRays: [],
      goalRay: null
    };

    const goal = { x: 43.5, y: 16.5, z: 0 };
    const startPad = { x: 4.5, y: 16.5, z: 0 };

    // Stage-specific configurable parameters
    const stageParams = {
      s1_wallRepulsion: true,
      s1_penaltyWeight: 2.6,
      s1_repulsionWeight: 0.65,
      s1_radius: 3.5,

      s2_profile: 'amphibious',

      s3_hullMode: 'rect2x4', // 'scout1', 'heavy2', 'rect2x4'

      s4_currentStrength: 0.7,
      s4_oneWayDiodes: true,

      s5_turretSpeed: 0.85,
      s5_peakThreat: 28,

      s6_viewLayer: 'auto', // 'auto', '0', '1'
      s6_pistonSpeed: 1.0
    };

    function addBorderWalls(g) {
      for (let z = 0; z < g.layers; z++) {
        g.setRect(0, 0, GW, 1, COST_BLOCK, z);
        g.setRect(0, GH - 1, GW, 1, COST_BLOCK, z);
        g.setRect(0, 0, 1, GH, COST_BLOCK, z);
        g.setRect(GW - 1, 0, 1, GH, COST_BLOCK, z);
      }
    }

    // Define the 6-Stage Obstacle Course
    const STAGES = [
      {
        name: 'Stage 1: Wall Repulsion SDF & Chunk Portals',
        shortTitle: '1. Wall SDF & Portals',
        desc: 'Agent-01 navigates a tight masonry slalom. GridWorld.enableWallRepulsion() & updateWallSDF() compute Signed Distance Field gradients (wallDist, wallGradX, wallGradY) that push trajectories away from sharp corners while detectPortals() extracts chunk border crossings.',
        tags: ['GridWorld', 'setRect / setDisc', 'enableWallRepulsion', 'updateWallSDF', 'detectPortals', 'MinHeap'],
        setup() {
          grid = new GridWorld(CX, CY, null, { layers: 1 });
          addBorderWalls(grid);

          // Masonry slalom baffles & pillars
          grid.setRect(11, 1, 2, 20, COST_BLOCK, 0);
          grid.setRect(22, 11, 2, 20, COST_BLOCK, 0);
          grid.setRect(33, 1, 2, 20, COST_BLOCK, 0);

          // Pillars in corridors
          grid.setDisc(17.5, 24.5, 2.2, COST_BLOCK, 0);
          grid.setDisc(28.5, 7.5, 2.2, COST_BLOCK, 0);
          grid.setDisc(39.5, 21.5, 2.2, COST_BLOCK, 0);

          if (stageParams.s1_wallRepulsion) {
            grid.enableWallRepulsion({
              penaltyWeight: stageParams.s1_penaltyWeight,
              repulsionWeight: stageParams.s1_repulsionWeight,
              radius: stageParams.s1_radius
            });
          } else {
            grid.wallPenaltyWeight = 0;
            grid.wallRepulsionWeight = 0;
            grid.updateWallSDF(true);
          }

          startPad.x = 5.5; startPad.y = 6.5; startPad.z = 0;
          goal.x = 43.5; goal.y = 26.5; goal.z = 0;
          agent.minClearance = 1;
          agent.profile = null;
          agent.footprint = null;
        },
        updateDynamic(dt) {},
        renderControls(container) {
          container.innerHTML = `
            <div class="control-row">
              <label><input type="checkbox" id="ctl-s1-sdf" ${stageParams.s1_wallRepulsion ? 'checked' : ''}/> Enable Wall SDF Repulsion</label>
            </div>
            <div class="control-row">
              <span>SDF Penalty Weight (${stageParams.s1_penaltyWeight.toFixed(1)}):</span>
              <input type="range" id="ctl-s1-pen" min="0" max="5" step="0.2" value="${stageParams.s1_penaltyWeight}" />
            </div>
            <div class="control-row">
              <span>SDF Radius (${stageParams.s1_radius.toFixed(1)}):</span>
              <input type="range" id="ctl-s1-rad" min="1.5" max="6" step="0.5" value="${stageParams.s1_radius}" />
            </div>
          `;
          container.querySelector('#ctl-s1-sdf').onchange = (e) => {
            stageParams.s1_wallRepulsion = e.target.checked;
            applyStage1SDF();
          };
          container.querySelector('#ctl-s1-pen').oninput = (e) => {
            stageParams.s1_penaltyWeight = parseFloat(e.target.value);
            applyStage1SDF();
            STAGES[0].renderControls(container);
          };
          container.querySelector('#ctl-s1-rad').oninput = (e) => {
            stageParams.s1_radius = parseFloat(e.target.value);
            applyStage1SDF();
            STAGES[0].renderControls(container);
          };
        }
      },
      {
        name: 'Stage 2: Terrain ID Layer & Scoped Unit Profiles',
        shortTitle: '2. Terrain Profiles',
        desc: 'Three parallel routes cross different biomes stored via ensureTerrainLayer() and setRectTerrain(): North Swamp (Terrain 2), Center Deep Water Channel (Terrain 1), and South Magma Scree (Terrain 3). Switching Agent-01’s movement profile dynamically alters traversal costs via registerProfile().',
        tags: ['ensureTerrainLayer', 'registerProfile', 'setRectTerrain', 'getEffectiveBaseCost'],
        setup() {
          grid = new GridWorld(CX, CY, null, { layers: 1 });
          addBorderWalls(grid);
          grid.enableWallRepulsion({ penaltyWeight: 1.2, repulsionWeight: 0.35, radius: 2.5 });

          // Divider islands separating the 3 biome lanes
          grid.setRect(13, 10, 22, 2, COST_BLOCK, 0);
          grid.setRect(13, 20, 22, 2, COST_BLOCK, 0);

          // Terrain IDs: 0=Open Road, 1=Deep Water, 2=Swamp Marsh, 3=Magma Scree
          grid.ensureTerrainLayer();
          grid.setRectTerrain(13, 1, 22, 9, 2, 0);   // North Lane: Swamp Marsh
          grid.setRectTerrain(13, 12, 22, 8, 1, 0);  // Center Lane: Deep Water
          grid.setRectTerrain(13, 22, 22, 9, 3, 0);  // South Lane: Magma Scree

          // Register unit movement profiles
          grid.registerProfile('standard',   { 0: 2, 1: COST_BLOCK, 2: 7,  3: 28 }); // Takes North Swamp
          grid.registerProfile('amphibious', { 0: 3, 1: 2,          2: 14, 3: COST_BLOCK }); // Cuts straight through Center Water
          grid.registerProfile('crawler',    { 0: 3, 1: COST_BLOCK, 2: 24, 3: 2 });  // Takes South Magma Scree

          startPad.x = 4.5; startPad.y = 16.0; startPad.z = 0;
          goal.x = 43.5; goal.y = 16.0; goal.z = 0;
          agent.minClearance = 1;
          agent.profile = stageParams.s2_profile;
          agent.footprint = null;
        },
        updateDynamic(dt) {},
        renderControls(container) {
          container.innerHTML = `
            <div class="control-row">
              <span>Active Unit Profile:</span>
              <select id="ctl-s2-prof">
                <option value="amphibious" ${stageParams.s2_profile === 'amphibious' ? 'selected' : ''}>Amphibious (Water=2, Swamp=14)</option>
                <option value="standard" ${stageParams.s2_profile === 'standard' ? 'selected' : ''}>Standard Walker (Swamp=7, Water=BLOCK)</option>
                <option value="crawler" ${stageParams.s2_profile === 'crawler' ? 'selected' : ''}>Thermal Crawler (Magma=2, Water=BLOCK)</option>
              </select>
            </div>
          `;
          container.querySelector('#ctl-s2-prof').onchange = (e) => {
            stageParams.s2_profile = e.target.value;
            agent.profile = stageParams.s2_profile;
            solveFlowField();
          };
        }
      },
      {
        name: 'Stage 3: Brushfire Clearance & Rectangular Footprints',
        shortTitle: '3. Clearance & Footprint',
        desc: 'GridWorld.updateClearance() sweeps both isotropic Chebyshev clearance and directional corridor spans (clearanceX, clearanceY). A 1-cell shortcut is topologically open, but larger or rectangular vehicle footprints ({ width, length, turnPenalty }) automatically prune narrow passages.',
        tags: ['updateClearance', 'clearance / clearanceX / clearanceY', 'isWalkable', 'footprint'],
        setup() {
          grid = new GridWorld(CX, CY, null, { layers: 1 });
          addBorderWalls(grid);
          grid.enableWallRepulsion({ penaltyWeight: 1.4, repulsionWeight: 0.4, radius: 2.5 });

          // Construct 3 horizontal passages across a central barrier (x = 16..31)
          grid.setRect(16, 1, 16, GH - 2, COST_BLOCK, 0);

          // Top Passage: 1-cell narrow squeeze gap at y = 5 (only clearance=1 fits)
          grid.setRect(16, 5, 16, 1, COST_OPEN, 0);

          // Middle Passage: 2-cell corridor with a sharp 90-degree chicane (blocks long 2x4 rectangular hull turn, allows 2x2)
          grid.setRect(16, 14, 9, 2, COST_OPEN, 0);
          grid.setRect(23, 14, 2, 5, COST_OPEN, 0);
          grid.setRect(23, 17, 9, 2, COST_OPEN, 0);

          // Bottom Passage: Wide 6-cell high-clearance highway at y = 23..28 (fits all hulls including 2x4 Hauler)
          grid.setRect(16, 23, 16, 6, COST_OPEN, 0);

          grid.updateClearance(true);

          startPad.x = 5.5; startPad.y = 15.0; startPad.z = 0;
          goal.x = 42.5; goal.y = 15.0; goal.z = 0;
          agent.profile = null;
          applyStage3Hull();
        },
        updateDynamic(dt) {},
        renderControls(container) {
          container.innerHTML = `
            <div class="control-row">
              <span>Agent Chassis Footprint:</span>
              <select id="ctl-s3-hull">
                <option value="scout1" ${stageParams.s3_hullMode === 'scout1' ? 'selected' : ''}>1x1 Micro Scout (Uses Top 1-Cell Gap)</option>
                <option value="heavy2" ${stageParams.s3_hullMode === 'heavy2' ? 'selected' : ''}>2x2 Isotropic Rover (Uses Mid Chicane)</option>
                <option value="rect2x4" ${stageParams.s3_hullMode === 'rect2x4' ? 'selected' : ''}>2x4 Rectangular Hauler (Uses South Highway)</option>
              </select>
            </div>
          `;
          container.querySelector('#ctl-s3-hull').onchange = (e) => {
            stageParams.s3_hullMode = e.target.value;
            applyStage3Hull();
            solveFlowField();
          };
        }
      },
      {
        name: 'Stage 4: Anisotropic One-Way Gates, Currents & Slopes',
        shortTitle: '4. Anisotropy & Currents',
        desc: 'Demonstrates ensureAnisotropicLayer(): one-way directional bitmasks (setDirectionMask with DIR_FLAGS), high-velocity vector current tunnels (setRectCurrent), and a central elevation mound computed via setSlopeFromHeightmap().',
        tags: ['DIR_FLAGS', 'setDirectionMask', 'setRectCurrent', 'setSlopeFromHeightmap', 'crossingCost'],
        setup() {
          grid = new GridWorld(CX, CY, null, { layers: 1 });
          addBorderWalls(grid);
          grid.enableWallRepulsion({ penaltyWeight: 1.2, repulsionWeight: 0.35, radius: 2.0 });
          grid.ensureAnisotropicLayer();
          grid.currentStrength = stageParams.s4_currentStrength;

          // Build a central heightmap hill (x=18..30, y=10..22) first
          const heights = new Float32Array(GW * GH);
          for (let y = 0; y < GH; y++) {
            for (let x = 0; x < GW; x++) {
              const dx = (x - 24) / 5.5, dy = (y - 16) / 5.5;
              const r2 = dx * dx + dy * dy;
              if (r2 < 1.8) heights[y * GW + x] = 6.0 * Math.exp(-r2 * 1.4);
            }
          }
          grid.setSlopeFromHeightmap(heights, 1.4, 0);

          // Channel walls creating North and South tunnels around the central hill
          grid.setRect(14, 8, 20, 2, COST_BLOCK, 0);
          grid.setRect(14, 22, 20, 2, COST_BLOCK, 0);

          // North Tunnel: Strong opposing Westbound headwind current (-2.5, 0)
          grid.setRectCurrent(14, 1, 20, 7, -2.2, 0.0, 0);

          // South Tunnel: Strong favorable Eastbound tailwind current (+2.2, 0)
          grid.setRectCurrent(14, 24, 20, 7, 2.2, 0.0, 0);

          // One-way gates at x = 14 (Center entry blocked by West-only diode if enabled)
          if (stageParams.s4_oneWayDiodes) {
            for (let y = 10; y < 22; y++) {
              grid.setDirectionMask(14, y, DIR_FLAGS.W, 0); // Westbound only -> blocks Eastbound shortcut!
              grid.setDirectionMask(15, y, DIR_FLAGS.W, 0);
            }
            // South tunnel has Eastbound diode gate
            for (let y = 24; y < 31; y++) {
              grid.setDirectionMask(24, y, DIR_FLAGS.E | DIR_FLAGS.NE | DIR_FLAGS.SE, 0);
            }
          }

          startPad.x = 5.5; startPad.y = 16.0; startPad.z = 0;
          goal.x = 42.5; goal.y = 16.0; goal.z = 0;
          agent.minClearance = 1;
          agent.profile = null;
          agent.footprint = null;
        },
        updateDynamic(dt) {},
        renderControls(container) {
          container.innerHTML = `
            <div class="control-row">
              <label><input type="checkbox" id="ctl-s4-diode" ${stageParams.s4_oneWayDiodes ? 'checked' : ''}/> Center West-Only Diode Gate (DIR_FLAGS.W)</label>
            </div>
            <div class="control-row">
              <span>Current Strength (${stageParams.s4_currentStrength.toFixed(2)}):</span>
              <input type="range" id="ctl-s4-cur" min="0" max="1.2" step="0.1" value="${stageParams.s4_currentStrength}" />
            </div>
          `;
          container.querySelector('#ctl-s4-diode').onchange = (e) => {
            stageParams.s4_oneWayDiodes = e.target.checked;
            STAGES[3].setup();
            solveFlowField();
          };
          container.querySelector('#ctl-s4-cur').oninput = (e) => {
            stageParams.s4_currentStrength = parseFloat(e.target.value);
            grid.currentStrength = stageParams.s4_currentStrength;
            solveFlowField();
            STAGES[3].renderControls(container);
          };
        }
      },
      {
        name: 'Stage 5: Tactical Threat FOV Shadowcasting & Raycasting',
        shortTitle: '5. Tactical FOV & Raycast',
        desc: 'Two security emitters sweep the arena. Every frame, clearTacticalCosts() and addTacticalFOV() cast 8-octant shadowcast vision cones that terminate against cover blocks. Agent-01 routes through low-threat shadow corridors while casting grid.raycast() sensor beams.',
        tags: ['ensureTacticalLayer', 'addTacticalFOV', 'addTacticalDisc', 'clearTacticalCosts', 'raycast'],
        setup() {
          grid = new GridWorld(CX, CY, null, { layers: 1 });
          addBorderWalls(grid);
          grid.enableWallRepulsion({ penaltyWeight: 1.5, repulsionWeight: 0.4, radius: 2.2 });

          // Cover bunkers that cast protective shadows against the security emitters
          grid.setRect(13, 6, 3, 6, COST_BLOCK, 0);
          grid.setRect(13, 20, 3, 6, COST_BLOCK, 0);
          grid.setRect(23, 13, 3, 6, COST_BLOCK, 0);
          grid.setRect(33, 6, 3, 6, COST_BLOCK, 0);
          grid.setRect(33, 20, 3, 6, COST_BLOCK, 0);

          startPad.x = 4.5; startPad.y = 16.0; startPad.z = 0;
          goal.x = 43.5; goal.y = 16.0; goal.z = 0;
          agent.minClearance = 1;
          agent.profile = null;
          agent.footprint = null;
          this.updateDynamic(0);
        },
        updateDynamic(dt) {
          grid.clearTacticalCosts();
          visibleFOVCells.clear();

          // Moving scanner origins orbiting in the upper and lower arena halves
          const t = stageTime * stageParams.s5_turretSpeed;
          const s1x = 20 + Math.cos(t) * 5.0;
          const s1y = 9 + Math.sin(t * 1.3) * 3.5;
          const s2x = 29 + Math.cos(-t * 0.9) * 5.0;
          const s2y = 23 + Math.sin(-t * 1.1) * 3.5;

          grid.addTacticalFOV(s1x, s1y, 11.5, stageParams.s5_peakThreat, true, (mx, my, idx) => {
            visibleFOVCells.add(idx);
          }, 0);
          grid.addTacticalFOV(s2x, s2y, 11.5, stageParams.s5_peakThreat, true, (mx, my, idx) => {
            visibleFOVCells.add(idx);
          }, 0);

          // Static central hazard disc
          grid.addTacticalDisc(24.5, 16.0, 4.5, stageParams.s5_peakThreat * 0.6, true, 0);
          solveFlowField();
        },
        renderControls(container) {
          container.innerHTML = `
            <div class="control-row">
              <span>Scanner Orbit Speed (${stageParams.s5_turretSpeed.toFixed(2)}):</span>
              <input type="range" id="ctl-s5-spd" min="0" max="2.0" step="0.1" value="${stageParams.s5_turretSpeed}" />
            </div>
            <div class="control-row">
              <span>FOV Threat Weight (${stageParams.s5_peakThreat}):</span>
              <input type="range" id="ctl-s5-thr" min="0" max="50" step="2" value="${stageParams.s5_peakThreat}" />
            </div>
          `;
          container.querySelector('#ctl-s5-spd').oninput = (e) => {
            stageParams.s5_turretSpeed = parseFloat(e.target.value);
            STAGES[4].renderControls(container);
          };
          container.querySelector('#ctl-s5-thr').oninput = (e) => {
            stageParams.s5_peakThreat = parseFloat(e.target.value);
            STAGES[4].updateDynamic(0);
            STAGES[4].renderControls(container);
          };
        }
      },
      {
        name: 'Stage 6: Dynamic Obstacles, Flux & 2.5D Multi-Layer Overpass',
        shortTitle: '6. 2.5D Layers & Dynamic',
        desc: 'A 2-layer 2.5D world (layers: 2). Layer 0 is split by a central chasm and patrolled by moving pistons (stampDynamicObstacle) and congestion velocity-flux (addAgentFlux / decayCongestion). Agent-01 climbs a vertical Stair Portal (addStairPortal) to the Layer 1 Skybridge Overpass and descends on the far side.',
        tags: ['layers: 2', 'addStairPortal', 'stampDynamicObstacle', 'addAgentFlux', 'decayCongestion'],
        setup() {
          grid = new GridWorld(CX, CY, null, { layers: 2 });
          addBorderWalls(grid);

          // Layer 0 (Ground): Impassable central chasm at x = 22..25
          grid.setRect(22, 1, 4, GH - 2, COST_BLOCK, 0);

          // Layer 1 (Elevated Overpass Deck): Only the skybridge corridor (x = 14..34, y = 11..21) is walkable
          grid.setRect(0, 0, GW, GH, COST_BLOCK, 1);
          grid.setRect(14, 11, 20, 10, COST_OPEN, 1);
          // Baffle on the skybridge
          grid.setRect(23, 11, 2, 6, COST_BLOCK, 1);

          grid.enableWallRepulsion({ penaltyWeight: 1.4, repulsionWeight: 0.4, radius: 2.2 });

          // Register vertical stair portals connecting Layer 0 and Layer 1 across the chasm
          grid.addStairPortal(15, 15, 2, 2, 0, 1, { cost: 2, exitX: 1, exitY: 0 });
          grid.addStairPortal(31, 15, 2, 2, 1, 0, { cost: 2, exitX: 1, exitY: 0 });

          startPad.x = 4.5; startPad.y = 16.0; startPad.z = 0;
          goal.x = 43.5; goal.y = 16.0; goal.z = 0;
          agent.minClearance = 1;
          agent.profile = null;
          agent.footprint = null;
          this.updateDynamic(0);
        },
        updateDynamic(dt) {
          grid.clearDynamicObstacles();
          grid.decayCongestion(0.75, 0.05);

          const t = stageTime * stageParams.s6_pistonSpeed;
          // Moving dynamic piston obstacles on Layer 0 (West approach & East exit)
          const p1y = 16 + Math.sin(t * 1.8) * 9.0;
          const p2y = 16 + Math.cos(t * 1.5) * 9.0;
          const p3y = 16 + Math.sin(-t * 1.7) * 9.0;

          grid.stampDynamicObstacle(10.0, p1y, 1.8, 1, 0);
          grid.stampDynamicObstacle(37.0, p2y, 1.8, 1, 0);
          grid.stampDynamicObstacle(40.5, p3y, 1.5, 1, 0);

          // Inject moving velocity-flux stream on Layer 1 Skybridge (south half)
          for (let x = 18; x <= 29; x++) {
            grid.addAgentFlux(x, 18, -3.5, 0, 2.2, 1);
            grid.addAgentFlux(x, 19, -3.5, 0, 2.2, 1);
          }

          solveFlowField();
        },
        renderControls(container) {
          container.innerHTML = `
            <div class="control-row">
              <span>2.5D Layer View:</span>
              <select id="ctl-s6-layer">
                <option value="auto" ${stageParams.s6_viewLayer === 'auto' ? 'selected' : ''}>Auto-Follow Agent Layer</option>
                <option value="0" ${stageParams.s6_viewLayer === '0' ? 'selected' : ''}>Layer 0 (Ground Floor)</option>
                <option value="1" ${stageParams.s6_viewLayer === '1' ? 'selected' : ''}>Layer 1 (Skybridge Overpass)</option>
              </select>
            </div>
            <div class="control-row">
              <span>Dynamic Piston Speed (${stageParams.s6_pistonSpeed.toFixed(1)}):</span>
              <input type="range" id="ctl-s6-spd" min="0" max="2.5" step="0.25" value="${stageParams.s6_pistonSpeed}" />
            </div>
          `;
          container.querySelector('#ctl-s6-layer').onchange = (e) => {
            stageParams.s6_viewLayer = e.target.value;
          };
          container.querySelector('#ctl-s6-spd').oninput = (e) => {
            stageParams.s6_pistonSpeed = parseFloat(e.target.value);
            STAGES[5].renderControls(container);
          };
        }
      }
    ];

    function applyStage1SDF() {
      if (stageParams.s1_wallRepulsion) {
        grid.enableWallRepulsion({
          penaltyWeight: stageParams.s1_penaltyWeight,
          repulsionWeight: stageParams.s1_repulsionWeight,
          radius: stageParams.s1_radius
        });
      } else {
        grid.wallPenaltyWeight = 0;
        grid.wallRepulsionWeight = 0;
      }
      solveFlowField();
    }

    function applyStage3Hull() {
      if (stageParams.s3_hullMode === 'scout1') {
        agent.minClearance = 1;
        agent.footprint = null;
      } else if (stageParams.s3_hullMode === 'heavy2') {
        agent.minClearance = 2;
        agent.footprint = null;
      } else {
        agent.minClearance = 2;
        agent.footprint = { width: 2, length: 4, turnPenalty: 4.5 };
      }
    }

    // Core Eikonal / Anisotropic Dijkstra solver powered by ChunkIntegrator, detectPortals, and MinHeap
    function solveFlowField() {
      if (!grid) return;

      portals = detectPortals(grid, agent.minClearance, MAX_PORTAL_LEN, agent.profile, agent.footprint);

      const totalCells = grid.layerCells * grid.layers;
      if (!distField || distField.length !== totalCells) {
        distField = new Float32Array(totalCells);
        flowX = new Float32Array(totalCells);
        flowY = new Float32Array(totalCells);
        portalTargetLayer = new Int8Array(totalCells);
        cellCosts = new Float32Array(totalCells);
        cellBlocked = new Uint8Array(totalCells);
        cellDirMasks = new Uint8Array(totalCells);
        cellTurnPen = new Float32Array(totalCells);
        cellCurX = new Float32Array(totalCells);
        cellCurY = new Float32Array(totalCells);
        cellFluxX = new Float32Array(totalCells);
        cellFluxY = new Float32Array(totalCells);
      }

      distField.fill(Infinity);
      flowX.fill(0);
      flowY.fill(0);
      portalTargetLayer.fill(-1);

      // Prepare each chunk using flow-field-pathfinder-api-alt.js ChunkIntegrator.prepareChunk()
      for (let c = 0; c < grid.chunkCount; c++) {
        chunkIntegrator.prepareChunk(grid, c, agent.minClearance, agent.profile, agent.footprint);
        const ox = grid.chunkOriginX(c);
        const oy = grid.chunkOriginY(c);
        const z = grid.chunkLayer(c);
        for (let ly = 0; ly < CS; ly++) {
          const gy = oy + ly;
          for (let lx = 0; lx < CS; lx++) {
            const gx = ox + lx;
            const li = ly * CS + lx;
            const gi = grid.index(gx, gy, z);
            cellCosts[gi] = chunkIntegrator.costs[li];
            cellBlocked[gi] = chunkIntegrator.blocked[li];
            cellDirMasks[gi] = chunkIntegrator.dirMasks[li];
            cellTurnPen[gi] = chunkIntegrator.turnPenaltyCell[li];
            cellCurX[gi] = chunkIntegrator.curX[li];
            cellCurY[gi] = chunkIntegrator.curY[li];
            cellFluxX[gi] = chunkIntegrator.fluxX[li];
            cellFluxY[gi] = chunkIntegrator.fluxY[li];
          }
        }
      }

      // Build vertical stair portal adjacency lookup
      const verticalLinks = new Map();
      for (const vp of grid.verticalPortals) {
        const len = Math.min(vp.cellsA.length, vp.cellsB.length);
        for (let k = 0; k < len; k++) {
          const ca = vp.cellsA[k], cb = vp.cellsB[k];
          if (vp.canAB) {
            if (!verticalLinks.has(ca)) verticalLinks.set(ca, []);
            verticalLinks.get(ca).push({ toCell: cb, toLayer: vp.layerB, cost: (vp.cost ?? COST_OPEN) / COST_OPEN });
          }
          if (vp.canBA) {
            if (!verticalLinks.has(cb)) verticalLinks.set(cb, []);
            verticalLinks.get(cb).push({ toCell: ca, toLayer: vp.layerA, cost: (vp.cost ?? COST_OPEN) / COST_OPEN });
          }
        }
      }

      const goalCell = grid.findNearestWalkable(goal.x, goal.y, 12, agent.minClearance, agent.profile, goal.z, agent.footprint);
      if (goalCell < 0) return;

      solverHeap.clear();
      distField[goalCell] = 0;
      solverHeap.push(goalCell, 0);

      const LC = grid.layerCells;
      while (!solverHeap.isEmpty) {
        const v = solverHeap.pop();
        const curD = solverHeap.lastKey;
        if (curD > distField[v]) continue;

        const z = (v / LC) | 0;
        const rem = v % LC;
        const vx = rem % GW;
        const vy = (rem / GW) | 0;

        // Backward search: relax predecessors u -> v
        for (let d = 0; d < 8; d++) {
          // Moving from predecessor u to v along direction d means u = v - D[d]
          const ux = vx - DX[d];
          const uy = vy - DY[d];
          if (ux < 0 || ux >= GW || uy < 0 || uy >= GH) continue;
          const u = z * LC + uy * GW + ux;
          if (cellBlocked[u]) continue;

          // Check if outgoing direction d is allowed at u
          if ((cellDirMasks[u] & (1 << d)) === 0) continue;

          // Prevent diagonal corner cutting
          if (d >= 4) {
            const c1 = z * LC + uy * GW + vx;
            const c2 = z * LC + vy * GW + ux;
            if (cellBlocked[c1] || cellBlocked[c2]) continue;
          }

          let stepCost = 0.5 * (cellCosts[u] + cellCosts[v]) / COST_OPEN;
          if (cellTurnPen[u] > 0) stepCost += cellTurnPen[u];

          // Anisotropic vector current modulation along u -> v
          const dirX = DX[d] / DL[d], dirY = DY[d] / DL[d];
          const dotCur = dirX * cellCurX[u] + dirY * cellCurY[u];
          if (dotCur !== 0) {
            stepCost *= clamp(1 - dotCur * grid.currentStrength, 0.2, 4.0);
          }

          // Velocity-flux counter-flow penalty
          const fx = cellFluxX[v], fy = cellFluxY[v];
          if (fx !== 0 || fy !== 0) {
            const dotFlux = dirX * fx + dirY * fy;
            if (dotFlux < 0) stepCost += (-dotFlux) * grid.congestionLaneWeight;
          }

          const nd = curD + DL[d] * stepCost;
          if (nd < distField[u]) {
            distField[u] = nd;
            solverHeap.push(u, nd);
          }
        }

        // Relax vertical stair portal predecessors u -> v
        for (const [uCell, links] of verticalLinks.entries()) {
          for (const lk of links) {
            if (lk.toCell === v && !cellBlocked[uCell]) {
              const nd = curD + lk.cost;
              if (nd < distField[uCell]) {
                distField[uCell] = nd;
                portalTargetLayer[uCell] = z;
                solverHeap.push(uCell, nd);
              }
            }
          }
        }
      }

      // Compute normalized flow vectors + Wall SDF repulsion blend
      for (let z = 0; z < grid.layers; z++) {
        const zOff = z * LC;
        for (let y = 0; y < GH; y++) {
          for (let x = 0; x < GW; x++) {
            const i = zOff + y * GW + x;
            if (!Number.isFinite(distField[i])) continue;

            let bestDx = 0, bestDy = 0;
            const d0 = distField[i];
            for (let d = 0; d < 8; d++) {
              if ((cellDirMasks[i] & (1 << d)) === 0) continue;
              const nx = x + DX[d], ny = y + DY[d];
              if (nx < 0 || nx >= GW || ny < 0 || ny >= GH) continue;
              const ni = zOff + ny * GW + nx;
              if (!Number.isFinite(distField[ni])) continue;
              if (d >= 4) {
                if (cellBlocked[zOff + y * GW + nx] || cellBlocked[zOff + ny * GW + x]) continue;
              }
              const drop = (d0 - distField[ni]) / DL[d];
              if (drop > 0) {
                bestDx += (DX[d] / DL[d]) * drop;
                bestDy += (DY[d] / DL[d]) * drop;
              }
            }

            // Blend Wall SDF gradient near obstacles
            if (grid.wallRepulsionWeight > 0 && grid.wallDist && grid.wallDist[i] < grid.wallPenaltyRadius) {
              const w = (1 - grid.wallDist[i] / grid.wallPenaltyRadius) * grid.wallRepulsionWeight;
              bestDx += grid.wallGradX[i] * w;
              bestDy += grid.wallGradY[i] * w;
            }

            const mag = Math.hypot(bestDx, bestDy);
            if (mag > 1e-5) {
              flowX[i] = bestDx / mag;
              flowY[i] = bestDy / mag;
            }
          }
        }
      }

      grid.consumeDirtyState();
    }

    // Sample bilinear flow vector at continuous coordinates (wx, wy, z)
    function sampleFlowAt(wx, wy, z) {
      const zOff = clamp(z | 0, 0, grid.layers - 1) * grid.layerCells;
      const cx = clamp(wx | 0, 0, GW - 1);
      const cy = clamp(wy | 0, 0, GH - 1);
      const baseIdx = zOff + cy * GW + cx;

      let sx = flowX[baseIdx], sy = flowY[baseIdx];
      // Average with immediate walkable neighbors for smooth steering
      for (let d = 0; d < 4; d++) {
        const nx = clamp(cx + DX[d], 0, GW - 1);
        const ny = clamp(cy + DY[d], 0, GH - 1);
        const ni = zOff + ny * GW + nx;
        if (!cellBlocked[ni]) {
          sx += flowX[ni] * 0.35;
          sy += flowY[ni] * 0.35;
        }
      }
      const m = Math.hypot(sx, sy);
      return m > 1e-5 ? { fx: sx / m, fy: sy / m } : { fx: 0, fy: 0 };
    }

    function loadStage(idx) {
      currentStageIdx = (idx + STAGES.length) % STAGES.length;
      stageTime = 0;

      const st = STAGES[currentStageIdx];
      st.setup();
      solveFlowField();

      // Reset Agent-01 at stage start pad
      agent.x = startPad.x;
      agent.y = startPad.y;
      agent.z = startPad.z;
      agent.vx = 0;
      agent.vy = 0;
      agent.heading = 0;
      agent.layerCooldown = 0;
      agent.stuckTimer = 0;
      agent.trail = [{ x: agent.x, y: agent.y, z: agent.z }];

      // Update UI metadata & buttons
      document.getElementById('stage-title').textContent = st.name;
      document.getElementById('stage-desc').textContent = st.desc;
      document.getElementById('stage-tags').innerHTML = st.tags.map(t => `<span class="tag">${t}</span>`).join('');
      st.renderControls(document.getElementById('stage-controls'));

      document.querySelectorAll('#stage-buttons button').forEach((b, i) => {
        b.classList.toggle('active', i === currentStageIdx);
      });
    }

    function stepSimulation(dt) {
      stageTime += dt;
      if (bannerTimer > 0) {
        bannerTimer -= dt;
        if (bannerTimer <= 0) document.getElementById('stage-banner').style.display = 'none';
      }

      STAGES[currentStageIdx].updateDynamic(dt);

      if (agent.layerCooldown > 0) agent.layerCooldown -= dt;

      const cellIdx = grid.index(agent.x | 0, agent.y | 0, agent.z);

      // Check 2.5D vertical stair portal transition
      if (agent.layerCooldown <= 0 && portalTargetLayer[cellIdx] >= 0 && portalTargetLayer[cellIdx] !== agent.z) {
        agent.z = portalTargetLayer[cellIdx];
        agent.layerCooldown = 0.6;
      }

      // If agent is ever inside a dynamically stamped obstacle, push to nearest walkable cell
      if (cellBlocked[grid.index(agent.x | 0, agent.y | 0, agent.z)]) {
        const nearest = grid.findNearestWalkable(agent.x, agent.y, 8, agent.minClearance, agent.profile, agent.z, agent.footprint);
        if (nearest >= 0) {
          const rem = nearest % grid.layerCells;
          agent.x = (rem % GW) + 0.5;
          agent.y = ((rem / GW) | 0) + 0.5;
        }
      }

      const { fx, fy } = sampleFlowAt(agent.x, agent.y, agent.z);
      // Environmental current drift
      const driftX = (cellCurX[cellIdx] || 0) * 0.25;
      const driftY = (cellCurY[cellIdx] || 0) * 0.25;

      const desiredVx = (fx + driftX) * agent.speed;
      const desiredVy = (fy + driftY) * agent.speed;

      const steerRate = Math.min(1, dt * 10);
      agent.vx += (desiredVx - agent.vx) * steerRate;
      agent.vy += (desiredVy - agent.vy) * steerRate;

      // Sub-step position integration with slide collision
      const nx = agent.x + agent.vx * dt;
      const ny = agent.y + agent.vy * dt;

      const canMoveXY = !cellBlocked[grid.index(nx | 0, ny | 0, agent.z)];
      const canMoveX = !cellBlocked[grid.index(nx | 0, agent.y | 0, agent.z)];
      const canMoveY = !cellBlocked[grid.index(agent.x | 0, ny | 0, agent.z)];

      if (canMoveXY) {
        agent.x = nx;
        agent.y = ny;
      } else if (canMoveX) {
        agent.x = nx;
        agent.vy *= 0.2;
      } else if (canMoveY) {
        agent.y = ny;
        agent.vx *= 0.2;
      } else {
        agent.vx *= -0.3;
        agent.vy *= -0.3;
      }

      agent.x = clamp(agent.x, 1.2, GW - 1.2);
      agent.y = clamp(agent.y, 1.2, GH - 1.2);

      if (Math.hypot(agent.vx, agent.vy) > 0.2) {
        agent.heading = Math.atan2(agent.vy, agent.vx);
      }

      // Record trail
      const lastPt = agent.trail[agent.trail.length - 1];
      if (!lastPt || Math.hypot(agent.x - lastPt.x, agent.y - lastPt.y) > 0.35 || lastPt.z !== agent.z) {
        agent.trail.push({ x: agent.x, y: agent.y, z: agent.z });
        if (agent.trail.length > 260) agent.trail.shift();
      }

      // Cast forward fan rays + Goal LOS ray using GridWorld.raycast()
      agent.goalRay = grid.raycast(agent.x, agent.y, goal.x, goal.y, {
        minClearance: 1,
        layer: agent.z,
        profile: agent.profile
      });

      agent.fanRays = [];
      const fanAngles = [-0.6, -0.3, 0, 0.3, 0.6];
      for (const aOff of fanAngles) {
        const ang = agent.heading + aOff;
        const tx = clamp(agent.x + Math.cos(ang) * 9.0, 0.5, GW - 0.5);
        const ty = clamp(agent.y + Math.sin(ang) * 9.0, 0.5, GH - 0.5);
        agent.fanRays.push(grid.raycast(agent.x, agent.y, tx, ty, {
          minClearance: 1,
          layer: agent.z,
          profile: agent.profile
        }));
      }

      // Check stage completion when Agent-01 reaches the Goal Beacon
      if (agent.z === goal.z && Math.hypot(agent.x - goal.x, agent.y - goal.y) < 1.35) {
        const banner = document.getElementById('stage-banner');
        if (currentStageIdx === STAGES.length - 1) completedCourses++;
        banner.textContent = `STAGE ${currentStageIdx + 1} CLEARED — ${autoAdvance ? 'ADVANCING TO NEXT STAGE' : 'RESETTING STAGE'}`;
        banner.style.display = 'block';
        bannerTimer = 1.6;
        if (autoAdvance) {
          loadStage((currentStageIdx + 1) % STAGES.length);
        } else {
          loadStage(currentStageIdx);
        }
      }
    }

    function getActiveRenderLayer() {
      if (grid.layers === 1) return 0;
      if (stageParams.s6_viewLayer === '0') return 0;
      if (stageParams.s6_viewLayer === '1') return 1;
      return agent.z;
    }

    function render() {
      const z = getActiveRenderLayer();
      const LC = grid.layerCells;
      const zOff = z * LC;

      const showFlow = document.getElementById('chk-flow').checked;
      const showDist = document.getElementById('chk-dist').checked;
      const showPortals = document.getElementById('chk-portals').checked;
      const showRays = document.getElementById('chk-rays').checked;
      const showSpecial = document.getElementById('chk-special').checked;

      ctx.clearRect(0, 0, canvas.width, canvas.height);

      // 1. Render Grid Cells
      for (let y = 0; y < GH; y++) {
        for (let x = 0; x < GW; x++) {
          const idx = zOff + y * GW + x;
          const px = x * SCALE, py = y * SCALE;

          const baseCost = grid.costs[idx];
          const isDynBlock = grid.hasDynamicObstacles && grid.dynamicObstacles[idx] > 0;

          if (baseCost >= COST_BLOCK) {
            ctx.fillStyle = z === 1 ? '#374151' : '#1f2937';
            ctx.fillRect(px, py, SCALE, SCALE);
            continue;
          }

          // Base walkable cell background
          ctx.fillStyle = z === 1 ? '#eef2ff' : '#ffffff';

          // Terrain biome tinting (Stage 2)
          if (grid.terrainIds) {
            const tid = grid.terrainIds[idx];
            if (tid === 1) ctx.fillStyle = '#bae6fd';      // Deep Water
            else if (tid === 2) ctx.fillStyle = '#d9f99d'; // Swamp Marsh
            else if (tid === 3) ctx.fillStyle = '#fed7aa'; // Magma Scree
          }

          ctx.fillRect(px, py, SCALE, SCALE);

          // Integration field distance heatmap
          if (showDist && Number.isFinite(distField[idx])) {
            const norm = clamp(distField[idx] / 75, 0, 1);
            ctx.fillStyle = `rgba(59, 130, 246, ${(1 - norm) * 0.16})`;
            ctx.fillRect(px, py, SCALE, SCALE);
          }

          // Stage-specific field overlays
          if (showSpecial) {
            // Stage 1: Wall SDF proximity shading
            if (currentStageIdx === 0 && grid.wallDist && grid.wallPenaltyWeight > 0) {
              const d = grid.wallDist[idx];
              if (d > 0 && d < grid.wallPenaltyRadius) {
                const alpha = (1 - d / grid.wallPenaltyRadius) * 0.28;
                ctx.fillStyle = `rgba(245, 158, 11, ${alpha})`;
                ctx.fillRect(px, py, SCALE, SCALE);
              }
            }
            // Stage 3: Pruned clearance cells shaded light red
            if (currentStageIdx === 2 && cellBlocked[idx] && baseCost < COST_BLOCK) {
              ctx.fillStyle = 'rgba(239, 68, 68, 0.22)';
              ctx.fillRect(px, py, SCALE, SCALE);
            }
            // Stage 5: Tactical threat overlay & FOV cone
            if (currentStageIdx === 4 && grid.tacticalCosts && grid.tacticalCosts[idx] > 0) {
              const alpha = clamp(grid.tacticalCosts[idx] / 35, 0.08, 0.55);
              ctx.fillStyle = `rgba(220, 38, 38, ${alpha})`;
              ctx.fillRect(px, py, SCALE, SCALE);
            }
            // Stage 6: Congestion flux shading
            if (currentStageIdx === 5 && grid.congestion && grid.congestion[idx] > 0.05) {
              const alpha = clamp(grid.congestion[idx] / 3.0, 0.1, 0.45);
              ctx.fillStyle = `rgba(168, 85, 247, ${alpha})`;
              ctx.fillRect(px, py, SCALE, SCALE);
            }
          }

          // Dynamic obstacle stamp (Stage 6)
          if (isDynBlock) {
            ctx.fillStyle = '#dc2626';
            ctx.fillRect(px + 1, py + 1, SCALE - 2, SCALE - 2);
          }

          // Subtle cell border
          ctx.strokeStyle = '#e5e7eb';
          ctx.lineWidth = 0.5;
          ctx.strokeRect(px, py, SCALE, SCALE);

          // Stage 3: Print isotropic clearance numbers in passages
          if (showSpecial && currentStageIdx === 2 && grid.clearance && (x % 2 === 0 && y % 2 === 0)) {
            ctx.fillStyle = '#6b7280';
            ctx.font = '8px monospace';
            ctx.fillText(String(grid.clearance[idx]), px + 4, py + 11);
          }

          // Stage 4: Render one-way diode masks & current vectors
          if (showSpecial && currentStageIdx === 3) {
            const mask = cellDirMasks[idx];
            if (mask !== 0xFF && mask !== 0) {
              ctx.fillStyle = 'rgba(220, 38, 38, 0.25)';
              ctx.fillRect(px, py, SCALE, SCALE);
              ctx.fillStyle = '#991b1b';
              ctx.font = 'bold 10px monospace';
              ctx.fillText((mask & DIR_FLAGS.E) ? '→' : '←', px + 4, py + 12);
            }
          }

          // Flow vector arrow
          if (showFlow && !cellBlocked[idx]) {
            const fx = flowX[idx], fy = flowY[idx];
            if (fx !== 0 || fy !== 0) {
              const cx = px + SCALE * 0.5, cy = py + SCALE * 0.5;
              ctx.strokeStyle = 'rgba(17, 24, 39, 0.42)';
              ctx.lineWidth = 1;
              ctx.beginPath();
              ctx.moveTo(cx, cy);
              ctx.lineTo(cx + fx * 6.5, cy + fy * 6.5);
              ctx.stroke();
            }
          }
        }
      }

      // 2. Render 16x16 Chunk Grid Lines & Detected Portals
      ctx.strokeStyle = '#9ca3af';
      ctx.lineWidth = 1;
      ctx.setLineDash([4, 4]);
      for (let cx = 1; cx < CX; cx++) {
        ctx.beginPath();
        ctx.moveTo(cx * CS * SCALE, 0);
        ctx.lineTo(cx * CS * SCALE, canvas.height);
        ctx.stroke();
      }
      for (let cy = 1; cy < CY; cy++) {
        ctx.beginPath();
        ctx.moveTo(0, cy * CS * SCALE);
        ctx.lineTo(canvas.width, cy * CS * SCALE);
        ctx.stroke();
      }
      ctx.setLineDash([]);

      if (showPortals) {
        for (const p of portals) {
          if (p.isVertical) {
            // Render vertical stair portal pad
            const cells = z === p.layerA ? p.cellsA : (z === p.layerB ? p.cellsB : null);
            if (!cells) continue;
            ctx.fillStyle = 'rgba(245, 158, 11, 0.45)';
            ctx.strokeStyle = '#d97706';
            ctx.lineWidth = 2;
            for (const cIdx of cells) {
              const rem
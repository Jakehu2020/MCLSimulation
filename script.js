const canvas = document.querySelector("#canvas");
const ctx = canvas.getContext("2d");

canvas.width  = 144 * 4;
canvas.height = 144 * 4;

// ── UI State ──────────────────────────────────────────────────────────────────
const ui = {
    showSensors:   true,
    showPath:      true,
    showWaypoints: true,
    showLookahead: true,
    showMCL:       false,
    showGhost:     false,
    driveMode:     'manual',   // 'manual' | 'auton'
    clickMode:     null,       // null | 'target' | 'waypoint'
    controller:    'MTP',
};

const keys = {};

// ── Field drawing ─────────────────────────────────────────────────────────────
function drawField() {
    ctx.fillStyle = '#a4a4a4';
    ctx.fillRect(0, 0, 9999, 9999);
    ctx.fillStyle = "#000";
    for (let i = 0; i < 6; i++) {
        ctx.fillRect(val(i * 24), 0, 0.5, 999);
        ctx.fillRect(0, val(i * 24), 999, 0.5);
    }
    ctx.fillStyle = "#a4952d55";
    const longGoalSize = 16, longGoalHeight = 58;
    ctx.fillRect(val(24 - longGoalSize/2), val(96 - longGoalHeight), val(longGoalSize), val(longGoalHeight * 2 - 48));
    ctx.fillRect(val(120 - longGoalSize/2), val(96 - longGoalHeight), val(longGoalSize), val(longGoalHeight * 2 - 48));
    ctx.rotate(Math.PI / 4);
    ctx.fillRect(val(96), val(-17), val(14), val(34));
    ctx.fillRect(val(85), val(-7), val(35), val(14));
    ctx.resetTransform();
}

// ── Robot & Sensors Setup ─────────────────────────────────────────────────────
const r = new Robot(40, 40, 90 * Math.PI / 180);
r.addSensor(  7.5,  3.5,  0           );
r.addSensor(  7.5, -3.5,  0           );
r.addSensor(  0,    7.5,  Math.PI / 2 );
r.addSensor(  0,   -7.5, -Math.PI / 2 );
r.addSensor( -7.5,  0,    Math.PI     );

const ghostRobot = new Robot(40, 40, Math.PI / 2);
robots.splice(robots.indexOf(ghostRobot), 1); // keep it off the main list

let lastPos   = [...r.pos];
let lastTheta = r.theta;

const mcl = new MCL(r, ghostRobot, {
    numParticles:  300,
    spreadX:       7,
    spreadY:       7,
    spreadTheta:   Math.PI / 4,
    motionNoise:   { x: 1.6, y: 1.6, theta: 0.06 },
    sensorSigma:   2,
    injectionRate: 0.03,
    adaptive:      false,
});

// ── Path / Waypoints ──────────────────────────────────────────────────────────
let userWaypoints = [];
let currentPath   = [];
r.state = { index: 0 };

function rebuildPath() {
    if (userWaypoints.length >= 2) {
        currentPath = bezierPath(userWaypoints, { resolution: 24, tension: 0.4 });
    } else {
        currentPath = [];
    }
    r.state = { index: 0 };
    updateWptCount();
}

function updateWptCount() {
    document.getElementById('wptCount').textContent =
        userWaypoints.length + ' waypoint' + (userWaypoints.length === 1 ? '' : 's');
}

// Controller map
const controllerMap = { MTP, purePursuit, ramsete, stanley };

// ── Manual Drive ──────────────────────────────────────────────────────────────
const DRIVE_POWER = 5;
const TURN_POWER  = 3;

function manualDrive() {
    if (ui.driveMode !== 'manual') return;

    // Q/E: pure point-turn — spin in place, ignores W/S
    if (keys['q'] || keys['e']) {
        const spin = keys['e'] ? TURN_POWER : -TURN_POWER;
        r.move(spin, -spin);
        updateMotorBars(spin, -spin);
        return;
    }

    // W/S + A/D: arcade mix — A/D steers, curves when combined with W/S
    const fwd = (keys['w'] || keys['arrowup']    ? DRIVE_POWER : 0)
              - (keys['s'] || keys['arrowdown']  ? DRIVE_POWER : 0);
    const arc = (keys['d'] || keys['arrowright'] ? TURN_POWER  : 0)
              - (keys['a'] || keys['arrowleft']  ? TURN_POWER  : 0);

    const L = fwd + arc;
    const R = fwd - arc;
    if (L !== 0 || R !== 0) r.move(L, R);
    updateMotorBars(L, R);
}

// ── FPS ───────────────────────────────────────────────────────────────────────
let frameCount = 0, lastFPSTime = performance.now(), fps = 0;
function tickFPS() {
    frameCount++;
    const now = performance.now();
    if (now - lastFPSTime >= 500) {
        fps = Math.round(frameCount * 1000 / (now - lastFPSTime));
        frameCount = 0;
        lastFPSTime = now;
        document.getElementById('fps-counter').textContent = fps + ' fps';
    }
}

// ── Telemetry update ─────────────────────────────────────────────────────────
function updateTelemetry(L, R) {
    document.getElementById('tX').textContent     = r.pos[0].toFixed(1);
    document.getElementById('tY').textContent     = r.pos[1].toFixed(1);
    document.getElementById('tTheta').textContent = (r.theta * 180 / Math.PI).toFixed(1);
    const speed = Math.abs(((L ?? 0) + (R ?? 0)) / 2);
    document.getElementById('tSpeed').textContent = speed.toFixed(2);
}

function updateMotorBars(L, R) {
    const maxPow = r.maxSpeed;
    setBar('barL', L / maxPow);
    setBar('barR', R / maxPow);
}

function setBar(id, v) {
    const bar = document.getElementById(id);
    const frac = Math.max(-1, Math.min(1, v));
    if (frac >= 0) {
        bar.style.left  = '50%';
        bar.style.width = (frac * 50) + '%';
    } else {
        bar.style.left  = (50 + frac * 50) + '%';
        bar.style.width = (-frac * 50) + '%';
    }
    bar.style.background = frac >= 0 ? 'var(--accent2)' : 'var(--warn)';
}

// ── Main Loop ─────────────────────────────────────────────────────────────────
setInterval(() => {
    drawField();

    // Auton tick
    let L = 0, R = 0;
    // Path-following controllers vs point controllers
    const PATH_CONTROLLERS = new Set(['purePursuit']);
    const isPathCtrl = PATH_CONTROLLERS.has(ui.controller);

    if (ui.driveMode === 'auton') {
        r.castSensors();
        if (isPathCtrl && currentPath.length > 1) {
            // Pure Pursuit: needs a full path array
            [L, R] = purePursuit(currentPath, r, r.state);
            r.move(L, R);
        } else if (!isPathCtrl && r.target) {
            // MTP / ramsete / stanley: need a single [x, y, theta] point
            // Auto-append heading angle so ramsete/stanley don't get undefined[2]
            const tgt = r.target;
            const tx = Array.isArray(tgt) ? tgt : [tgt[0], tgt[1]];
            const dx = tx[0] - r.pos[0], dy = tx[1] - r.pos[1];
            const targetWithTheta = [tx[0], tx[1], Math.atan2(dy, dx)];
            [L, R] = controllerMap[ui.controller](targetWithTheta, r, r.state);
            r.move(L, R);
        }
    } else {
        r.castSensors();
        manualDrive();
    }

    // Draw robot
    if (ui.showSensors) {
        r.draw();
    } else {
        // Draw robot body only, skip sensor rays
        const savedDraw = Distance.prototype.draw;
        Distance.prototype.draw = function(){};
        r.draw();
        Distance.prototype.draw = savedDraw;
    }

    // Draw path
    if (currentPath.length > 1) {
        drawPath(currentPath, userWaypoints, r.state, {
            robot:          r,
            lookahead:      15,
            showSpline:     ui.showPath,
            showWaypoints:  ui.showWaypoints,
            showProgress:   true,
            showLookahead:  ui.showLookahead,
            showDirection:  ui.showPath,
        });
    }

    // Draw single target dot if set
    if (r.target && !Array.isArray(r.target)) {
        const tx = val(r.target[0]), ty = val(r.target[1]);
        ctx.save();
        // Outer pulsing ring
        ctx.strokeStyle = '#00e5ff';
        ctx.lineWidth = 1.5;
        ctx.setLineDash([4, 3]);
        ctx.beginPath();
        ctx.arc(tx, ty, 12, 0, Math.PI * 2);
        ctx.stroke();
        // Cross
        ctx.setLineDash([]);
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.moveTo(tx - 14, ty); ctx.lineTo(tx + 14, ty);
        ctx.moveTo(tx, ty - 14); ctx.lineTo(tx, ty + 14);
        ctx.stroke();
        // Center dot
        ctx.fillStyle = '#00e5ff';
        ctx.beginPath();
        ctx.arc(tx, ty, 3, 0, Math.PI * 2);
        ctx.fill();
        // Label
        ctx.fillStyle = '#00e5ff';
        ctx.font = '10px monospace';
        ctx.textAlign = 'left';
        ctx.fillText('(' + r.target[0].toFixed(1) + ', ' + r.target[1].toFixed(1) + ')', tx + 16, ty + 4);
        ctx.restore();
    }

    // MCL
    if (ui.showMCL || ui.showGhost) {
        const globalDx = r.pos[0] - lastPos[0];
        const globalDy = r.pos[1] - lastPos[1];
        const dTheta   = wrapAngle(r.theta - lastTheta);
        const cosA = Math.cos(lastTheta), sinA = Math.sin(lastTheta);
        const localDx = cosA * globalDx + sinA * globalDy;
        const localDy = -sinA * globalDx + cosA * globalDy;
        lastPos   = [...r.pos];
        lastTheta = r.theta;

        mcl.step(localDx, localDy, dTheta);
        if (ui.showMCL)    mcl.draw();
        if (ui.showGhost)  drawGhostRobot(ghostRobot);
    } else {
        lastPos   = [...r.pos];
        lastTheta = r.theta;
    }

    // Telemetry
    updateTelemetry(L, R);
    updateMotorBars(L, R);
    tickFPS();
}, 80);

// ── Canvas Click ──────────────────────────────────────────────────────────────
canvas.addEventListener('click', (e) => {
    const rect = canvas.getBoundingClientRect();
    // Canvas CSS width = 576px, internal = 576px (4×144). getBoundingClientRect gives CSS px.
    const scaleX = canvas.width  / rect.width;
    const scaleY = canvas.height / rect.height;
    const cx = (e.clientX - rect.left) * scaleX;
    const cy = (e.clientY - rect.top)  * scaleY;
    const fx = reverseVal(cx);
    const fy = reverseVal(cy);

    if (ui.clickMode === 'target') {
        r.target = [fx, fy];
        currentPath = [];
        r.state = { index: 0 };
        updateWptCount();
        // Auto-switch to auton
        setDriveMode('auton');
        setClickMode(null);
    } else if (ui.clickMode === 'waypoint') {
        userWaypoints.push([fx, fy]);
        rebuildPath();
        // keep mode active for multiple adds
    }
});

// ── Keyboard ──────────────────────────────────────────────────────────────────
document.addEventListener('keydown', e => {
    keys[e.key.toLowerCase()] = true;
    if (e.key === 'Escape') setClickMode(null);
});
document.addEventListener('keyup',  e => {
    keys[e.key.toLowerCase()] = false;
});

// ── Click Mode Helper ─────────────────────────────────────────────────────────
function setClickMode(mode) {
    ui.clickMode = mode;
    const badge = document.getElementById('mode-badge');
    canvas.className = mode ? 'mode-' + mode : '';
    if (mode === 'target') {
        badge.textContent = '⊕  CLICK TO SET TARGET';
        badge.classList.remove('hidden');
    } else if (mode === 'waypoint') {
        badge.textContent = '◉  CLICK TO ADD WAYPOINTS  ·  ESC TO FINISH';
        badge.classList.remove('hidden');
    } else {
        badge.classList.add('hidden');
    }
    // Update button states
    document.getElementById('btnSetTarget').classList.toggle('active', mode === 'target');
    document.getElementById('btnAddWaypoint').classList.toggle('active', mode === 'waypoint');
}

function setDriveMode(mode) {
    ui.driveMode = mode;
    document.getElementById('btnManual').classList.toggle('active', mode === 'manual');
    document.getElementById('btnAuton').classList.toggle('active', mode === 'auton');
    document.getElementById('keyHint').style.display = mode === 'manual' ? '' : 'none';
    if (mode === 'manual') r.target = null;
}

// ── UI Wiring ─────────────────────────────────────────────────────────────────

// Controller buttons
const PATH_CTRL_NAMES = { purePursuit: 'RUN PATH' };
function updateCtrlHint(ctrl) {
    const need = PATH_CTRL_NAMES[ctrl] || 'SET TARGET';
    document.getElementById('ctrlNeed').textContent = need;
}
document.querySelectorAll('.ctrl-btn').forEach(btn => {
    btn.addEventListener('click', () => {
        document.querySelectorAll('.ctrl-btn').forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        ui.controller = btn.dataset.ctrl;
        r.controller = controllerMap[ui.controller] || MTP;
        updateCtrlHint(ui.controller);
    });
});

// Drive mode
document.getElementById('btnManual').addEventListener('click', () => setDriveMode('manual'));
document.getElementById('btnAuton').addEventListener('click',  () => setDriveMode('auton'));

// Set Target / Add Waypoint
document.getElementById('btnSetTarget').addEventListener('click', () => {
    setClickMode(ui.clickMode === 'target' ? null : 'target');
});
document.getElementById('btnAddWaypoint').addEventListener('click', () => {
    setClickMode(ui.clickMode === 'waypoint' ? null : 'waypoint');
});

// Run Path
document.getElementById('btnRunPath').addEventListener('click', () => {
    if (currentPath.length < 2) {
        alert('Add at least 2 waypoints first.');
        return;
    }
    r.target = null;
    r.state  = { index: 0 };
    setDriveMode('auton');
});

// Clear
document.getElementById('btnClearPath').addEventListener('click', () => {
    userWaypoints = [];
    currentPath   = [];
    r.target = null;
    r.state  = { index: 0 };
    updateWptCount();
    setClickMode(null);
});

// Toggles
const toggleBindings = {
    togSensors:   v => ui.showSensors   = v,
    togPath:      v => ui.showPath      = v,
    togWaypoints: v => ui.showWaypoints = v,
    togLookahead: v => ui.showLookahead = v,
    togMCL:       v => {
        ui.showMCL = v;
        document.getElementById('mclParams').style.display = v ? '' : 'none';
    },
    togGhost:     v => ui.showGhost = v,
};
Object.entries(toggleBindings).forEach(([id, fn]) => {
    const el = document.getElementById(id);
    if (el) el.addEventListener('change', () => fn(el.checked));
});

// MCL params — wire live inputs + reset
['mclSmoothing', 'mclJump', 'mclSigma'].forEach(id => {
    const el = document.getElementById(id);
    if (el) el.addEventListener('input', () => applyMCLParams());
});
function applyMCLParams() {
    mcl.sensorSigma   = parseFloat(document.getElementById('mclSigma').value)     || 2;
    mcl.smoothingAlpha = parseFloat(document.getElementById('mclSmoothing').value) ?? 0.25;
    mcl.jumpThreshold  = parseFloat(document.getElementById('mclJump').value)      || 12;
}
document.getElementById('btnResetMCL').addEventListener('click', () => {
    mcl.numParticles  = parseInt(document.getElementById('mclParticles').value) || 300;
    applyMCLParams();
    mcl.init();
});

// Noise slider
const noiseSlider = document.getElementById('noiseSlider');
const noiseVal    = document.getElementById('noiseVal');
noiseSlider.addEventListener('input', () => {
    r.noise = parseFloat(noiseSlider.value);
    noiseVal.textContent = r.noise.toFixed(2);
});

// Max speed
document.getElementById('maxSpeedInput').addEventListener('change', e => {
    r.maxSpeed = Math.max(1, parseFloat(e.target.value) || 8);
});

// Reset Robot
document.getElementById('btnResetRobot').addEventListener('click', () => {
    r.pos   = [40, 40];
    r.theta = 90 * Math.PI / 180;
    r.target = null;
    userWaypoints = [];
    currentPath   = [];
    r.state = { index: 0 };
    setDriveMode('manual');
    setClickMode(null);
    updateWptCount();
    mcl.init();
});
// ─────────────────────────────────────────────────────────────────────────────
//  mcl.js  –  Monte Carlo Localization
//  Depends on: walls[], raySegmentIntersect(), val(), ctx   (from your main file)
// ─────────────────────────────────────────────────────────────────────────────

// ── Helpers ──────────────────────────────────────────────────────────────────

/** Box-Muller: standard normal sample. */
function randn() {
    let u = 0, v = 0;
    while (u === 0) u = Math.random();
    while (v === 0) v = Math.random();
    return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

function wrapAngle(a) {
    while (a >  Math.PI) a -= 2 * Math.PI;
    while (a < -Math.PI) a += 2 * Math.PI;
    return a;
}

/** Linearly interpolate between two [r,g,b] colours at t ∈ [0,1]. */
function lerpColor(c0, c1, t) {
    return [
        Math.round(c0[0] + (c1[0] - c0[0]) * t),
        Math.round(c0[1] + (c1[1] - c0[1]) * t),
        Math.round(c0[2] + (c1[2] - c0[2]) * t),
    ];
}

// ── Ghost robot draw override ─────────────────────────────────────────────────
//  Call this instead of robot.draw() for the predicted-pose robot.
//  Draws a semi-transparent teal silhouette with a heading arrow.
function drawGhostRobot(robot) {
    const cx = val(robot.pos[0]);
    const cy = val(robot.pos[1]);
    const pw = val(robot.pos[0] + robot.size[0]) - cx;
    const ph = val(robot.pos[1] + robot.size[1]) - cy;

    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(robot.theta);

    // Body outline only
    ctx.strokeStyle = "rgba(0, 220, 200, 0.90)";
    ctx.lineWidth   = 2;
    ctx.setLineDash([4, 3]);
    ctx.strokeRect(-pw / 2, -ph / 2, pw, ph);

    // Heading arrow
    ctx.setLineDash([]);
    ctx.strokeStyle = "rgba(0, 220, 200, 1)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(pw / 2 + 4, 0);
    ctx.stroke();

    // Arrow head
    ctx.fillStyle = "rgba(0, 220, 200, 1)";
    ctx.beginPath();
    ctx.moveTo(pw / 2 + 4,  0);
    ctx.lineTo(pw / 2 - 2,  3);
    ctx.lineTo(pw / 2 - 2, -3);
    ctx.closePath();
    ctx.fill();

    ctx.restore();
}

// ─────────────────────────────────────────────────────────────────────────────
//  MCL class
// ─────────────────────────────────────────────────────────────────────────────
class MCL {
    /**
     * @param {Robot}  robot      - the real, moving robot (has distancers)
     * @param {Robot}  ghost      - stationary display robot for the estimate
     * @param {object} opts       - tunable parameters (all optional)
     */
    constructor(robot, ghost, {
        numParticles   = 300,       // starting particle count
        minParticles   = 50,        // floor when adaptive
        maxParticles   = 800,       // ceiling when adaptive
        adaptive       = false,     // grow/shrink count based on Neff
        spreadX        = 12,        // initial spread around start pose (inches)
        spreadY        = 12,
        spreadTheta    = Math.PI / 3,
        motionNoise    = { x: 0.8, y: 0.8, theta: 0.04 },  // per-tick std-dev
        sensorSigma    = 10,        // Gaussian sensor model std-dev (inches)
        injectionRate  = 0.03,      // fraction of random particles injected each cycle
        injectionSpreadX     = 10,  // tight spread around odometry (inches)
        injectionSpreadY     = 10,
        injectionSpreadTheta = 0.3, // ~17° tight heading spread
        injectionWideFrac    = 0.2, // fraction of injected particles that go wide (recovery)
        injectionWideScale   = 6,   // multiplier for wide injection spread
        smoothingAlpha       = 0.25, // EMA smoothing on estimate (0=instant, 1=frozen)
        jumpThreshold        = 12,  // inches — jumps larger than this get extra damping
        particleArrowLen = 6,       // pixels, heading arrow on each particle
        showArrows     = true,
        showDots       = true,
        showEstimate   = true,
        correctRobot      = false,  // enable feedback into robot.pos / theta
        correctionStrength = 0.08,  // lerp factor per tick (0 = no correction, 1 = hard snap)
        correctionThreshold = 0.25, // min (Neff / N) ratio before we trust the estimate
    } = {}) {
        this.robot   = robot;
        this.ghost   = ghost;

        // Config (all public so the UI can mutate them live)
        this.numParticles   = numParticles;
        this.minParticles   = minParticles;
        this.maxParticles   = maxParticles;
        this.adaptive       = adaptive;
        this.spread         = { x: spreadX, y: spreadY, theta: spreadTheta };
        this.motionNoise    = motionNoise;
        this.sensorSigma    = sensorSigma;
        this.injectionRate  = injectionRate;
        this.injectionSpread = {
            x: injectionSpreadX, y: injectionSpreadY, theta: injectionSpreadTheta
        };
        this.injectionWideFrac  = injectionWideFrac;
        this.injectionWideScale = injectionWideScale;
        this.smoothingAlpha     = smoothingAlpha;
        this.jumpThreshold      = jumpThreshold;
        this.particleArrowLen = particleArrowLen;
        this.showArrows   = showArrows;
        this.showDots     = showDots;
        this.showEstimate = showEstimate;

        this.correctRobot          = correctRobot;
        this.correctionStrength    = correctionStrength;
        this.correctionThreshold   = correctionThreshold;

        this.particles  = [];
        this.estimate_  = { x: robot.pos[0], y: robot.pos[1], theta: robot.theta };
        this._maxWeight = 1;    // tracked for colour normalisation

        this.init();
    }

    // ── Initialisation ───────────────────────────────────────────────────────

    /** Scatter particles in a Gaussian cloud around the robot's current pose. */
    init() {
        this.particles = [];
        for (let i = 0; i < this.numParticles; i++) {
            this.particles.push(this._randomParticleNear(
                this.robot.pos[0], this.robot.pos[1], this.robot.theta,
                this.spread.x, this.spread.y, this.spread.theta
            ));
        }
        this._normaliseWeights();
    }

    _randomParticleNear(x, y, theta, sx, sy, st) {
        return {
            x:      x     + randn() * sx,
            y:      y     + randn() * sy,
            theta:  wrapAngle(theta + randn() * st),
            weight: 1 / this.numParticles
        };
    }

    // ── Predict ──────────────────────────────────────────────────────────────

    /**
     * Odometry update.  Call this every tick with the robot's motion delta
     * in its OWN frame (forward dx, lateral dy, rotation dTheta).
     *
     * Example — you compute these from your odometry:
     *   mcl.predict(odom.dx, odom.dy, odom.dTheta);
     */
    predict(dx, dy, dTheta) {
        for (const p of this.particles) {
            const cos = Math.cos(p.theta), sin = Math.sin(p.theta);
            p.x     += cos * dx - sin * dy + randn() * this.motionNoise.x;
            p.y     += sin * dx + cos * dy + randn() * this.motionNoise.y;
            p.theta  = wrapAngle(p.theta + dTheta + randn() * this.motionNoise.theta);
        }
    }

    // ── Update (sensor weighting) ─────────────────────────────────────────────

    /**
     * Weight each particle by how well its simulated sensor readings match
     * the robot's actual readings.  Uses a Gaussian likelihood model.
     */
    update() {
        const sensors = this.robot.distancers;
        if (sensors.length === 0) return;

        const actual = sensors.map(s => s.reading);
        const sigma2 = 2 * this.sensorSigma ** 2;
        let total = 0;

        for (const p of this.particles) {
            let w = 1;
            for (let i = 0; i < sensors.length; i++) {
                const expected = this._simulateReading(p, sensors[i]);
                const diff     = expected - actual[i];
                w *= Math.exp(-(diff * diff) / sigma2);
            }
            p.weight = w;
            total   += w;
        }

        if (total > 0) {
            this._normaliseWeights(total);
        } else {
            // Total weight collapse → kidnapped robot, reinitialize
            console.warn("MCL: weight collapse – reinitialising particles.");
            this.init();
        }
    }

    /** Fast single-ray cast for a hypothetical particle pose + sensor config. */
    _simulateReading(pose, sensor) {
        const cos = Math.cos(pose.theta), sin = Math.sin(pose.theta);
        const ox  = sensor.offset[0],    oy  = sensor.offset[1];
        const wx  = pose.x + cos * ox - sin * oy;
        const wy  = pose.y + sin * ox + cos * oy;
        const angle = pose.theta + sensor.oTheta;
        const rdx = Math.cos(angle), rdy = Math.sin(angle);

        let minT = sensor.maxRange;
        for (const [ax, ay, bx, by] of walls) {
            const t = raySegmentIntersect(wx, wy, rdx, rdy, ax, ay, bx, by);
            if (t !== null && t < minT) minT = t;
        }
        return minT;
    }

    // ── Resample ──────────────────────────────────────────────────────────────

    /**
     * Low-variance systematic resampling + random injection.
     * Injection keeps diversity alive so the filter recovers from bad guesses.
     */
    resample() {
        // ── Adaptive particle count via Neff ─────────────────────────────────
        if (this.adaptive) {
            const neff = this._neff();
            const ratio = neff / this.numParticles;
            // Grow when uncertain, shrink when confident
            this.numParticles = Math.round(
                Math.max(this.minParticles,
                Math.min(this.maxParticles,
                this.numParticles * (ratio < 0.3 ? 1.15 : ratio > 0.7 ? 0.90 : 1)))
            );
        }

        const N       = this.numParticles;
        const inject  = Math.floor(N * this.injectionRate);
        const keep    = N - inject;

        // Low-variance wheel
        const newParticles = [];
        const r = Math.random() / keep;
        let c = this.particles[0].weight, i = 0;
        for (let m = 0; m < keep; m++) {
            const U = r + m / keep;
            while (U > c && i < this.particles.length - 1) {
                i++;
                c += this.particles[i].weight;
            }
            newParticles.push({ ...this.particles[i] });
        }

        // Injection split: most particles near odometry (tracking), a few wide (recovery)
        const cx  = this.robot.pos[0], cy  = this.robot.pos[1], ct = this.robot.theta;
        const wideCount = Math.floor(inject * this.injectionWideFrac);
        const nearCount = inject - wideCount;
        for (let j = 0; j < nearCount; j++) {
            newParticles.push(this._randomParticleNear(
                cx, cy, ct,
                this.injectionSpread.x,      // tight: ~10 in
                this.injectionSpread.y,
                this.injectionSpread.theta
            ));
        }
        for (let j = 0; j < wideCount; j++) {
            // Wide particles: scattered across the full field for kidnap recovery
            newParticles.push(this._randomParticleNear(
                cx, cy, ct,
                this.injectionSpread.x * this.injectionWideScale,
                this.injectionSpread.y * this.injectionWideScale,
                Math.PI
            ));
        }

        this.particles = newParticles;
        this._normaliseWeights();
    }

    // ── Estimate ──────────────────────────────────────────────────────────────

    /**
     * Weighted mean of all particles → pose estimate.
     * Angular mean uses the circular mean (sin/cos decomposition).
     * Sets ghost robot pose so it tracks the estimate live.
     */
    computeEstimate() {
        let wx = 0, wy = 0, sinSum = 0, cosSum = 0;
        for (const p of this.particles) {
            wx      += p.x     * p.weight;
            wy      += p.y     * p.weight;
            sinSum  += Math.sin(p.theta) * p.weight;
            cosSum  += Math.cos(p.theta) * p.weight;
        }
        const rawTheta = Math.atan2(sinSum, cosSum);

        // Jump filter: if the raw estimate leaps more than jumpThreshold,
        // increase smoothing so extreme outlier frames don't snap the ghost.
        const jump  = Math.hypot(wx - this.estimate_.x, wy - this.estimate_.y);
        const tJump = Math.abs(wrapAngle(rawTheta - this.estimate_.theta));

        let alpha = this.smoothingAlpha;   // base EMA: 0 = instant, 1 = frozen
        if (jump > this.jumpThreshold) {
            // Scale alpha up toward 0.98 the bigger the jump
            alpha = Math.min(0.98, alpha + (1 - alpha) * (jump / this.jumpThreshold - 1) * 0.5);
        }

        const newX     = this.estimate_.x * alpha + wx     * (1 - alpha);
        const newY     = this.estimate_.y * alpha + wy     * (1 - alpha);
        const newTheta = wrapAngle(this.estimate_.theta * alpha + rawTheta * (1 - alpha));

        this.estimate_ = { x: newX, y: newY, theta: newTheta };

        if (this.showEstimate) {
            this.ghost.pos[0] = newX;
            this.ghost.pos[1] = newY;
            this.ghost.theta  = newTheta;
        }
    }

    // ── Convenience: full cycle ───────────────────────────────────────────────

    /**
     * Call once per tick.
     *   mcl.step(odom.dx, odom.dy, odom.dTheta);
     */
    step(dx = 0, dy = 0, dTheta = 0) {
        this.predict(dx, dy, dTheta);
        this.update();
        this.resample();
        this.computeEstimate();
    }

    // ── Draw ──────────────────────────────────────────────────────────────────

    draw() {
        // Colour ramp: cold blue → yellow → hot red
        const COLD = [30,  100, 255];
        const MID  = [255, 220,   0];
        const HOT  = [255,  30,  30];

        const maxW = this._maxWeight || 1;
        const arrowPx = this.particleArrowLen;

        for (const p of this.particles) {
            const t = p.weight / maxW;             // 0 = lowest, 1 = highest
            const color = t < 0.5
                ? lerpColor(COLD, MID, t * 2)
                : lerpColor(MID,  HOT, (t - 0.5) * 2);
            const alpha = 0.35 + 0.65 * t;        // faint when low-weight
            const rgba  = `rgba(${color[0]},${color[1]},${color[2]},${alpha})`;

            ctx.save();
            ctx.translate(val(p.x), val(p.y));
            ctx.rotate(p.theta);

            if (this.showArrows) {
                ctx.strokeStyle = rgba;
                ctx.lineWidth = 1;
                ctx.beginPath();
                ctx.moveTo(0, 0);
                ctx.lineTo(arrowPx, 0);
                ctx.stroke();
            }

            if (this.showDots) {
                const r = 1.5 + 2 * t;             // bigger = more confident
                ctx.fillStyle = rgba;
                ctx.beginPath();
                ctx.arc(0, 0, r, 0, Math.PI * 2);
                ctx.fill();
            }

            ctx.restore();
        }

        // Ghost robot drawn last so it sits on top
        if (this.showEstimate) drawGhostRobot(this.ghost);
    }

    // ── Internal utils ────────────────────────────────────────────────────────

    _normaliseWeights(total) {
        if (total === undefined) {
            total = this.particles.reduce((s, p) => s + p.weight, 0);
        }
        const inv = total > 0 ? 1 / total : 1 / this.particles.length;
        let maxW = 0;
        for (const p of this.particles) {
            p.weight *= inv;
            if (p.weight > maxW) maxW = p.weight;
        }
        this._maxWeight = maxW;
    }

    /** Effective sample size — lower means more weight concentration. */
    _neff() {
        const sumSq = this.particles.reduce((s, p) => s + p.weight ** 2, 0);
        return sumSq > 0 ? 1 / sumSq : this.particles.length;
    }

    // ── Debug / UI helpers ────────────────────────────────────────────────────

    /** Human-readable snapshot, useful for an on-screen HUD. */
    stats() {
        return {
            particles: this.particles.length,
            neff:      Math.round(this._neff()),
            estimate:  {
                x:     +this.estimate_.x.toFixed(2),
                y:     +this.estimate_.y.toFixed(2),
                theta: +this.estimate_.theta.toFixed(3)
            },
            maxWeight: +this._maxWeight.toFixed(6)
        };
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Example wiring  (copy into your main file and adjust as needed)
// ─────────────────────────────────────────────────────────────────────────────
/*

// 1. Create the real robot and its ghost
const robot = new Robot(72, 72, 0);
robot.addSensor( 7.5,  3.5,  0           );   // front-right
robot.addSensor( 7.5, -3.5,  0           );   // front-left
robot.addSensor( 0,    7.5,  Math.PI/2   );   // left
robot.addSensor( 0,   -7.5, -Math.PI/2   );   // right
robot.addSensor(-7.5,  0,    Math.PI     );   // rear

const ghostRobot = new Robot(72, 72, 0);      // same start, never moves on its own

// 2. Instantiate MCL
const mcl = new MCL(robot, ghostRobot, {
    numParticles:  300,
    spreadX:       10,
    spreadY:       10,
    spreadTheta:   Math.PI / 4,
    motionNoise:   { x: 0.6, y: 0.6, theta: 0.03 },
    sensorSigma:   8,
    injectionRate: 0.03,
    adaptive:      false,
});

// 3. Game loop — you supply odom.dx / dy / dTheta from your odometry code
let lastPos   = [...robot.pos];
let lastTheta = robot.theta;

function loop() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // ── Odometry delta (replace with your real odometry) ──
    const globalDx    = robot.pos[0] - lastPos[0];
    const globalDy    = robot.pos[1] - lastPos[1];
    const dTheta      = wrapAngle(robot.theta - lastTheta);
    const cos = Math.cos(lastTheta), sin = Math.sin(lastTheta);
    const localDx     =  cos * globalDx + sin * globalDy;   // into robot frame
    const localDy     = -sin * globalDx + cos * globalDy;
    lastPos   = [...robot.pos];
    lastTheta = robot.theta;
    // ──────────────────────────────────────────────────────

    robot.castSensors();
    robot.tick();
    mcl.step(localDx, localDy, dTheta);   // full MCL cycle

    // Draw order: particles behind everything
    mcl.draw();
    robot.draw();

    // Optional HUD
    const s = mcl.stats();
    ctx.fillStyle = "#fff";
    ctx.font = "12px monospace";
    ctx.fillText(`particles: ${s.particles}  Neff: ${s.neff}  maxW: ${s.maxWeight}`, 8, 16);
    ctx.fillText(`est  x:${s.estimate.x}  y:${s.estimate.y}  θ:${s.estimate.theta}`, 8, 32);

    requestAnimationFrame(loop);
}
loop();

*/
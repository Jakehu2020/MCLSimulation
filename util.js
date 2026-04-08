const cos = Math.cos;
const sin = Math.sin;
function pointDist(pos1, pos2){
    return Math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2);
}
function lateralError(pos, target, theta) {
    const dx = target[0] - pos[0];
    const dy = target[1] - pos[1];
    return dx * Math.cos(theta) + dy * Math.sin(theta);
}
function angularError(pos, target, theta) {
    const dx = target[0] - pos[0];
    const dy = target[1] - pos[1];

    const hx = Math.cos(theta);
    const hy = Math.sin(theta);

    // perpendicular to heading
    return -dx * hy + dy * hx;
}
function val(x){
    if(typeof(x) == 'number'){
        return 4 * x;
    } else if (typeof(x) == 'object'){
        return x.map(y => 4 * y);
    }
    return x;
};
function reverseVal(x){
    if(typeof(x) == 'number'){
        return x/4;
    } else if (typeof(x) == 'object'){
        return x.map(y => y/4);
    }
    return x;
};
function normalizeAngle(angle){
    return angle - 2 * Math.PI * Math.floor((angle)/(2*Math.PI) + 1/2);
}
function calcPID(pid, err) {
    const now = performance.now();
    const dT  = Math.max((now - pid.lastTime) / 1000, 1e-6); // seconds, guard /0
    pid.lastTime = now;

    // Leaky integrator (kAW as decay) + anti-windup
    pid.integral = pid.kAW * pid.integral + pid.kI * err * dT;

    // Derivative on error (could swap to derivative-on-measurement to reduce kick)
    const derivative = pid.kD * (err - pid.prevErr) / dT;
    pid.prevErr = err;

    return pid.kP * err + pid.integral + derivative;
}
function wrapAngle(a) {
    while (a >  Math.PI) a -= 2 * Math.PI;
    while (a < -Math.PI) a += 2 * Math.PI;
    return a;
}

// DISTANCE SENSORS
const walls = [];

/**
 * Ray–segment intersection.
 * Ray:     P(t) = [rx,ry] + t·[rdx,rdy]        t ≥ 0
 * Segment: Q(u) = [ax,ay] + u·([bx,by]−[ax,ay]) u ∈ [0,1]
 * Returns t (distance along ray) or null if no hit.
 */
function raySegmentIntersect(rx, ry, rdx, rdy, ax, ay, bx, by) {
    const dx = bx - ax, dy = by - ay;
    const denom = rdx * dy - rdy * dx;
    if (Math.abs(denom) < 1e-10) return null;       // parallel / collinear
    const tx = ax - rx, ty = ay - ry;
    const t  = (tx * dy - ty * dx) / denom;
    const u  = (tx * rdy - ty * rdx) / denom;
    return (t >= 0 && u >= 0 && u <= 1) ? t : null;
}
function pushRect(x, y, w, h) {
    walls.push(
        [x,     y,     x + w, y    ],   // top
        [x + w, y,     x + w, y + h],   // right
        [x + w, y + h, x,     y + h],   // bottom
        [x,     y + h, x,     y    ]    // left
    );
}

function _circleSegmentIntersect(cx, cy, r, ax, ay, bx, by) {
    const dx = bx - ax, dy = by - ay;
    const fx = ax - cx, fy = ay - cy;

    const a = dx*dx + dy*dy;
    const b = 2 * (fx*dx + fy*dy);
    const c = fx*fx + fy*fy - r*r;
    let disc = b*b - 4*a*c;

    if (disc < 0 || a < 1e-10) return null;
    disc = Math.sqrt(disc);

    const t1 = (-b - disc) / (2*a);
    const t2 = (-b + disc) / (2*a);

    // Prefer the far intersection (t2) — it's further along the path
    for (const t of [t2, t1]) {
        if (t >= 0 && t <= 1) {
            return [ax + t * dx, ay + t * dy];
        }
    }
    return null;
}
// ─────────────────────────────────────────────────────────────────────────────
//  drawPath(path, rawWaypoints, state, opts)
//
//  path          — the sampled [x,y] array from bezierPath()
//  rawWaypoints  — the original control points you passed into bezierPath()
//  state         — the same {index} object used by purePursuit()
// ─────────────────────────────────────────────────────────────────────────────
function drawPath(path, rawWaypoints, state, {
    showSpline      = true,
    showWaypoints   = true,
    showProgress    = true,    // recolors the consumed portion of the path
    showDirection   = true,    // tick marks showing forward direction
    showIndex       = true,    // label each raw waypoint

    // Spline colours
    colorAhead      = "#00e5ff",
    colorDone       = "rgba(255,255,255,0.18)",
    lineWidth       = 2,

    // Direction ticks (every N sampled points)
    tickEvery       = 12,
    tickLen         = 5,       // pixels
    tickColor       = "rgba(0,229,255,0.55)",

    // Waypoint dots
    waypointRadius  = 5,
    waypointFill    = "#fff",
    waypointStroke  = "#00e5ff",

    // Lookahead ring (drawn at robot.pos — pass robot in opts if desired)
    robot           = null,
    lookahead       = 15,
    showLookahead   = true,
} = {}) {
    if (!path || path.length < 2) return;

    ctx.save();

    // ── 1. Spline line ───────────────────────────────────────────────────────
    if (showSpline) {
        // Split into done / ahead segments so each can have its own colour
        const split = showProgress ? Math.min(state.index, path.length - 1) : 0;

        // Done portion
        if (split > 0) {
            ctx.strokeStyle = colorDone;
            ctx.lineWidth   = lineWidth;
            ctx.setLineDash([4, 5]);
            ctx.beginPath();
            ctx.moveTo(val(path[0][0]), val(path[0][1]));
            for (let i = 1; i <= split; i++) {
                ctx.lineTo(val(path[i][0]), val(path[i][1]));
            }
            ctx.stroke();
        }

        // Ahead portion
        ctx.strokeStyle = colorAhead;
        ctx.lineWidth   = lineWidth;
        ctx.setLineDash([]);
        ctx.beginPath();
        ctx.moveTo(val(path[split][0]), val(path[split][1]));
        for (let i = split + 1; i < path.length; i++) {
            ctx.lineTo(val(path[i][0]), val(path[i][1]));
        }
        ctx.stroke();
    }

    // ── 2. Direction ticks ───────────────────────────────────────────────────
    if (showDirection) {
        ctx.strokeStyle = tickColor;
        ctx.lineWidth   = 1;
        ctx.setLineDash([]);

        for (let i = tickEvery; i < path.length - 1; i += tickEvery) {
            const [x0, y0] = path[i];
            const [x1, y1] = path[i + 1];
            const angle    = Math.atan2(y1 - y0, x1 - x0);
            const px = val(x0), py = val(y0);

            // Perpendicular tick
            const tx = Math.cos(angle + Math.PI / 2) * tickLen;
            const ty = Math.sin(angle + Math.PI / 2) * tickLen;
            ctx.beginPath();
            ctx.moveTo(px - tx, py - ty);
            ctx.lineTo(px + tx, py + ty);
            ctx.stroke();

            // Small arrowhead along the path
            ctx.beginPath();
            ctx.moveTo(px, py);
            ctx.lineTo(
                px + Math.cos(angle) * tickLen * 0.8,
                py + Math.sin(angle) * tickLen * 0.8
            );
            ctx.stroke();
        }
    }

    // ── 3. Raw waypoint dots ─────────────────────────────────────────────────
    if (showWaypoints) {
        for (let i = 0; i < rawWaypoints.length; i++) {
            const px = val(rawWaypoints[i][0]);
            const py = val(rawWaypoints[i][1]);

            // Outer glow ring
            ctx.strokeStyle = waypointStroke;
            ctx.lineWidth   = 1.5;
            ctx.beginPath();
            ctx.arc(px, py, waypointRadius + 3, 0, Math.PI * 2);
            ctx.stroke();

            // Fill dot
            ctx.fillStyle = waypointFill;
            ctx.beginPath();
            ctx.arc(px, py, waypointRadius, 0, Math.PI * 2);
            ctx.fill();

            // Index label
            if (showIndex) {
                ctx.fillStyle   = "#000";
                ctx.font        = "bold 9px monospace";
                ctx.textAlign   = "center";
                ctx.textBaseline = "middle";
                ctx.fillText(i, px, py);
            }
        }
    }

    // ── 4. Lookahead circle ──────────────────────────────────────────────────
    if (showLookahead && robot) {
        const px = val(robot.pos[0]);
        const py = val(robot.pos[1]);
        // Convert lookahead radius from field-inches to pixels via val()
        const rPx = val(robot.pos[0] + lookahead) - px;

        ctx.strokeStyle = "rgba(0,229,255,0.25)";
        ctx.lineWidth   = 1;
        ctx.setLineDash([3, 4]);
        ctx.beginPath();
        ctx.arc(px, py, rPx, 0, Math.PI * 2);
        ctx.stroke();
    }

    ctx.restore();
}
function pushRotatedRect(x, y, w, h, angle, px = 0, py = 0) {
    const cos = Math.cos(angle);
    const sin = Math.sin(angle);

    const rotate = (cx, cy) => {
        const dx = cx - px, dy = cy - py;
        return [
            px + dx * cos - dy * sin,
            py + dx * sin + dy * cos
        ];
    };

    const corners = [
        rotate(x,     y    ),
        rotate(x + w, y    ),
        rotate(x + w, y + h),
        rotate(x,     y + h)
    ];

    for (let i = 0; i < 4; i++) {
        const [ax, ay] = corners[i];
        const [bx, by] = corners[(i + 1) % 4];
        walls.push([ax, ay, bx, by]);
    }
}
const longGoalSize   = 16;
const longGoalHeight = 58;
const gx = longGoalSize / 2;                   //  8
const gy = 96 - longGoalHeight;                // 38
const gw = longGoalSize;                       // 16
const gh = longGoalHeight * 2 - 48;            // 68

pushRect(24 - gx, gy, gw, gh);

// Right long goal (centered at x=120)
pushRect(120 - gx, gy, gw, gh);

// Diagonal cross — same coords as your fillRect calls, rotated π/4 around
// the canvas origin. Adjust (px, py) if val() has a translate offset.
pushRotatedRect( 96, -17, 14, 34, Math.PI / 4);
pushRotatedRect( 85,  -7, 35, 14, Math.PI / 4);
walls.push(
    [0,   0,   144, 0  ],   // bottom wall
    [144, 0,   144, 144],   // right wall
    [144, 144, 0,   144],   // top wall
    [0,   144, 0,   0  ]    // left wall
);
function bezierPath(waypoints, {
    resolution = 20,    // samples per segment (more = smoother, heavier)
    tension    = 0.4,   // 0 = tight corners, 1 = very loose/round
} = {}) {
    if (waypoints.length < 2) return [...waypoints];

    // Phantom endpoints: reflect first/last so the spline starts and ends
    // tangent to the actual first and last segments.
    const pts = [
        [2*waypoints[0][0] - waypoints[1][0],
         2*waypoints[0][1] - waypoints[1][1]],
        ...waypoints,
        [2*waypoints.at(-1)[0] - waypoints.at(-2)[0],
         2*waypoints.at(-1)[1] - waypoints.at(-2)[1]],
    ];

    const path = [];

    // Iterate over real segments (indices 1..N-2 in extended pts)
    for (let i = 1; i < pts.length - 2; i++) {
        const [p0x, p0y] = pts[i - 1];
        const [p1x, p1y] = pts[i    ];
        const [p2x, p2y] = pts[i + 1];
        const [p3x, p3y] = pts[i + 2];

        // Catmull-Rom → Cubic Bezier control points
        const cp1x = p1x + (p2x - p0x) * tension / 2;
        const cp1y = p1y + (p2y - p0y) * tension / 2;
        const cp2x = p2x - (p3x - p1x) * tension / 2;
        const cp2y = p2y - (p3y - p1y) * tension / 2;

        // Sample the segment (skip t=0 on all but the first to avoid dupes)
        const start = (i === 1) ? 0 : 1;
        for (let s = start; s <= resolution; s++) {
            const t  = s / resolution;
            const mt = 1 - t;
            const x  = mt**3 * p1x
                      + 3 * mt**2 * t  * cp1x
                      + 3 * mt   * t**2 * cp2x
                      +            t**3 * p2x;
            const y  = mt**3 * p1y
                      + 3 * mt**2 * t  * cp1y
                      + 3 * mt   * t**2 * cp2y
                      +            t**3 * p2y;
            path.push([x, y]);
        }
    }

    return path;
}

function MTP(target, robot, state = {}, {
    settleDist = 2,
    scalar = 0.9,
    stopTurningError = 5,
}={}){
    const dx = target[0] - robot.pos[0];
    const dy = target[1] - robot.pos[1];
    let lateralErr = pointDist(robot.pos, target);

    if (lateralErr < settleDist) return [0, 0]; // settled

    const angleToTarget = Math.atan2(dy, dx);
    let angularErr = wrapAngle(angleToTarget - robot.theta);

    let direction = 1;
    if (Math.abs(angularErr) > Math.PI / 2) {
        direction  = -1;
        lateralErr  =0;
        // angularErr = wrapAngle(angularErr + Math.PI);
    } else if (lateralErr < stopTurningError){
        angularErr *= 0.2;
    }

    const cosScale = cos(angularErr);

    // PID outputs
    const lateralOut = (1 - scalar) * calcPID(robot.lateralPID, lateralErr) * cosScale * direction;
    const angularOut = scalar * calcPID(robot.angularPID, angularErr);

    // --- Differential mixing ---------------------------------------------
    let leftPow  = lateralOut + angularOut;
    let rightPow = lateralOut - angularOut;

    return [leftPow, rightPow];
}

function drivePID(target, robot, state, {
    settleDist = 5,
}={}){
    if(!state.target_point){
        state.target_point = [robot.pos[0] + cos(robot.theta) * target, robot.pos[1] + sin(robot.theta) * target];
    }
    let lateralErr = lateralError(robot.pos, state.target_point, robot.theta);
    console.log(lateralErr)

    if (lateralErr < settleDist) { state.target_point = null; return [0, 0] }; // settled

    const lateralOut = 0.05*calcPID(robot.lateralPID, lateralErr);

    return [lateralOut, lateralOut];
}

function purePursuit(path, robot, state={
    index=0
}={}, {
    scalar       = 4,     // angular scalar
    lookahead    = 1.5,      // lookahead radius (inches) — main tuning knob
    kCurvature   = 1.2,     // scales how hard ω responds to curvature
    settleDist   = 5,       // stop when within this distance of last point
    minLookahead = 6,       // clamp so tight corners don't collapse
    maxLookahead = 30,      // clamp so straights don't overshoot wildly
} = {}) {
    // ── 1. Settled? ─────────────────────────────────────────────────────────
    const last = path.at(-1);
    
    if (pointDist(robot.pos, last) < settleDist) {
        state.index = 0;
        return [0, 0];
    }

    // ── 2. Adaptive lookahead (speed-scaled) ────────────────────────────────
    //  Faster robot = larger lookahead = smoother; clamped to [min, max].
    const speed = Math.abs(
        (robot.lastLeftPow ?? 1) + (robot.lastRightPow ?? 1)
    ) / 2;
    const L = Math.min(maxLookahead, Math.max(minLookahead, lookahead * (0.5 + speed)));

    // ── 3. Find the lookahead ("carrot") point ───────────────────────────────
    //  Scan forward from state.index; for each segment test the lookahead
    //  circle against it. Take the furthest valid intersection so we never
    //  chase a point behind us.
    let carrot = null;

    for (let i = state.index; i < path.length - 1; i++) {
        const [ax, ay] = path[i];
        const [bx, by] = path[i + 1];
        const hit = _circleSegmentIntersect(robot.pos[0], robot.pos[1], L, ax, ay, bx, by);
        if (hit) {
            carrot = hit;
            state.index = i;   // don't go backwards
        }
    }

    // Fallback: if no intersection (robot lagging badly), aim at the nearest
    // point ahead of state.index.
    if (!carrot) {
        carrot = path[Math.min(state.index + 1, path.length - 1)];
    }

    // ── 4. Transform carrot into the robot's local frame ────────────────────
    const dx  = carrot[0] - robot.pos[0];
    const dy  = carrot[1] - robot.pos[1];
    const cosC = cos(robot.theta);
    const sinC = sin(robot.theta);
    const localX =  cosC * dx + sinC * dy;   // forward component
    const localY = -sinC * dx + cosC * dy;   // lateral component (left = +)

    // ── 5. Pure Pursuit curvature κ = 2·localY / L² ─────────────────────────
    //  Derivation: chord length ≈ L; lateral offset = localY → arc curvature.
    const dist = Math.hypot(localX, localY);
    if (dist < 1e-6) return [0, 0];

    // ── 6. Heading error (same reverse-detection as your MTP) ───────────────
    const angleToCarrot = Math.atan2(dy, dx);
    let angularErr = wrapAngle(angleToCarrot - robot.theta);

    let direction = 1;
    if (Math.abs(angularErr) > Math.PI / 2) {
        // direction  = -1;
        angularErr = wrapAngle(angularErr + Math.PI);
    }

    const cosScale  = cos(angularErr);
    const curvature = 2 * localY / (dist * dist);  // signed: + = left, − = right

    // ── 7. Mix lateral + angular, same pattern as your MTP ──────────────────
    const lateralOut = cosScale * direction;
    const angularOut = kCurvature * curvature;

    let leftPow  = lateralOut - angularOut * scalar;
    let rightPow = lateralOut + angularOut * scalar;

    const maxOut = Math.max(Math.abs(leftPow), Math.abs(rightPow), 1);
    leftPow  *= robot.maxSpeed/maxOut;
    rightPow *= robot.maxSpeed/maxOut;

    robot.lastLeftPow  =  leftPow;
    robot.lastRightPow =  rightPow;

    return [rightPow, leftPow];
}

function ramsete(target, robot, state={}, {
    settleDist   = 3,      // stop when within this distance of last point
    b            = 2.0,    // proportional
    zeta         = 0.5,    // dampening
    profile      = [null, null], // motion profile
    smallScalar  = 0.001,
    scalar = 0.03,
}={}){
    if(!target) return [0,0];
    let lateralErr = pointDist(robot.pos, target);
    if (lateralErr < settleDist) return [0, 0]; // settled
    
    // target {x, y, theta}
    error = [
        target[0] - robot.pos[0],
        target[1] - robot.pos[1],
        target[2] - robot.theta
    ];
    error = [
        cos(robot.theta) * error[0] + sin(robot.theta) * error[1],
        - sin(robot.theta) * error[0] + cos(robot.theta) * error[1],
        error[2]
    ];
    // small scalar motion profiling
    if(profile[0] === null || profile[1] === null){
        profile = [smallScalar * error[0], smallScalar * error[2]];
    }
    // calculate gains
    k = 2 * zeta * Math.sqrt(profile[1]**2 + b * profile[0]**2);
    const sinc = Math.abs(error[2]) < 1e-6 ? 1 : Math.sin(error[2]) / error[2];
    // debugger;
    // convert to outputs
    v = (Math.abs(error[0])**(1/3)) * (profile[0] * cos(error[2]) + k * error[0]);
    omega = scalar * (profile[1] + k * error[2] + (b * profile[0] * sin(error[2]) * error[1])/sinc) * robot.trackWidth/2;
    if(Math.abs(v+omega) < 0.5 && Math.abs(v-omega) < 0.5) omega *= 10;
    let leftPow = v + omega;
    let rightPow = v - omega;
    // console.log(omega);
    return [leftPow, rightPow];
}
function ramsete_(path, robot, state={}, {
    b            = 2.0,    // proportional
    zeta         = 0.5,    // dampening
    profile      = [null, null], // motion profile
    smallScalar  = 0.001,
    scalar = 0.03,

    lookahead    = 1.5,      // lookahead radius (inches) — main tuning knob
    kCurvature   = 1.2,     // scales how hard ω responds to curvature
    settleDist   = 5,       // stop when within this distance of last point
    minLookahead = 25,       // clamp so tight corners don't collapse
    maxLookahead = 50,      // clamp so straights don't overshoot wildly
}={}){
    const last = path.at(-1);
    
    if (pointDist(robot.pos, last) < settleDist) {
        state.index = 0;
        return [0, 0];
    }

    // ── 2. Adaptive lookahead (speed-scaled) ────────────────────────────────
    //  Faster robot = larger lookahead = smoother; clamped to [min, max].
    const speed = Math.abs(
        (robot.lastLeftPow ?? 1) + (robot.lastRightPow ?? 1)
    ) / 2;
    const L = Math.min(maxLookahead, Math.max(minLookahead, lookahead * (0.5 + speed)));

    // ── 3. Find the lookahead ("carrot") point ───────────────────────────────
    //  Scan forward from state.index; for each segment test the lookahead
    //  circle against it. Take the furthest valid intersection so we never
    //  chase a point behind us.
    let carrot = null;

    for (let i = state.index; i < path.length - 1; i++) {
        const [ax, ay] = path[i];
        const [bx, by] = path[i + 1];
        const hit = _circleSegmentIntersect(robot.pos[0], robot.pos[1], L, ax, ay, bx, by);
        if (hit) {
            carrot = hit;
            state.index = i;   // don't go backwards
        }
    }

    // Fallback: if no intersection (robot lagging badly), aim at the nearest
    // point ahead of state.index.
    if (!carrot) {
        carrot = path[Math.min(state.index + 1, path.length - 1)];
    }
    const angleToTarget = Math.atan2(carrot[1]-robot.pos[1], carrot[0]-robot.pos[0]);
    let angularErr = wrapAngle(angleToTarget - robot.theta);
    let target = [carrot[0], carrot[1], angularErr];
    // debugger;
    
    // target {x, y, theta}
    error = [
        target[0] - robot.pos[0],
        target[1] - robot.pos[1],
        target[2] - robot.theta
    ];
    error = [
        cos(robot.theta) * error[0] + sin(robot.theta) * error[1],
        - sin(robot.theta) * error[0] + cos(robot.theta) * error[1],
        error[2]
    ];
    // small scalar motion profiling
    if(profile[0] === null || profile[1] === null){
        profile = [smallScalar * error[0], smallScalar * error[2]];
    }
    // calculate gains
    k = 2 * zeta * Math.sqrt(profile[1]**2 + b * profile[0]**2);
    const sinc = Math.abs(error[2]) < 1e-6 ? 1 : Math.sin(error[2]) / error[2];
    // debugger;
    // convert to outputs
    v = (Math.abs(error[0])**(1/3)) * (profile[0] * cos(error[2]) + k * error[0]);
    omega = scalar * (profile[1] + k * error[2] + (b * profile[0] * sin(error[2]) * error[1])/sinc) * robot.trackWidth/2;

    let leftPow = v + omega;
    let rightPow = v - omega;
    // debugger;
    return [leftPow, rightPow];
}

function stanley(target, robot, state, {
    settleDist   = 3,      // stop when within this distance of last point
    k            = 0.2,
    velocity = 4,
    scalar = 0.8
}={}){
    if(!target) return [0,0];
    let lateralErr = pointDist(robot.pos, target);
    if (lateralErr < settleDist) return [0, 0]; // settled
    
    if(!target) return [0,0];
    const dx = target[0] - robot.pos[0];
    const dy = target[1] - robot.pos[1];
    lateralErr = pointDist(robot.pos, target);
    if (lateralErr < settleDist) return [0, 0]; // settled

    error = [
        target[0] - robot.pos[0],
        target[1] - robot.pos[1],
        target[2] - robot.theta
    ];
    error = [
        cos(robot.theta) * error[0] + sin(robot.theta) * error[1],
        - sin(robot.theta) * error[0] + cos(robot.theta) * error[1],
        error[2]
    ];

    const angleToTarget = Math.atan2(dy, dx);
    let angularErr = wrapAngle(angleToTarget - robot.theta);
    
    // delta = angleError * Math.atan(k * lateralErr / velocity);
    
    omega = angularErr * 5 * Math.sin(angularErr);
    // omega = velocity * Math.tan(delta) * (robot.trackWidth/2) * scalar / lateralErr;
    console.log(omega);

    let leftPow = velocity - omega;
    let rightPow = velocity + omega;
    // console.log(omega);
    return [leftPow, rightPow];
}
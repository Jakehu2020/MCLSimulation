class Distance {
    /**
     * @param {Robot}  robot    - parent robot
     * @param {number} ox       - local X offset from robot center (inches, +X = forward)
     * @param {number} oy       - local Y offset from robot center (inches, +Y = left)
     * @param {number} oTheta   - angle offset from robot heading (radians)
     * @param {number} maxRange - max sensing distance in inches (default 144″)
     */
    constructor(robot, ox, oy, oTheta, maxRange = 144) {
        this.robot    = robot;
        this.offset   = [ox, oy];
        this.oTheta   = oTheta;
        this.maxRange = maxRange;
        this.reading  = maxRange;   // last measured distance
        this.hitPt    = null;       // [x, y] of last hit in field-inches, or null
    }

    /** Sensor origin in world (field) coordinates. */
    get worldPos() {
        const { pos, theta } = this.robot;
        const cos = Math.cos(theta), sin = Math.sin(theta);
        return [
            pos[0] + cos * this.offset[0] - sin * this.offset[1],
            pos[1] + sin * this.offset[0] + cos * this.offset[1]
        ];
    }

    /** Absolute ray angle in world space. */
    get worldAngle() {
        return this.robot.theta + this.oTheta;
    }

    /**
     * Cast the ray against all segments in `walls`.
     * Updates this.reading and this.hitPt.
     * Returns the measured distance in inches.
     */
    cast() {
        const [wx, wy] = this.worldPos;
        const angle    = this.worldAngle;
        const rdx = Math.cos(angle);
        const rdy = Math.sin(angle);

        let minT = this.maxRange;

        for (const [ax, ay, bx, by] of walls) {
            const t = raySegmentIntersect(wx, wy, rdx, rdy, ax, ay, bx, by);
            if (t !== null && t < minT) minT = t;
        }

        this.reading = minT;
        this.hitPt   = minT < this.maxRange
            ? [wx + rdx * minT, wy + rdy * minT]
            : null;

        return minT;
    }

    /** Draw the ray beam and hit point onto the canvas. */
    draw() {
        const [wx, wy] = this.worldPos;
        const angle    = this.worldAngle;
        const rdx = Math.cos(angle);
        const rdy = Math.sin(angle);
        const hit = this.reading;

        const x1 = val(wx);
        const y1 = val(wy);
        const x2 = val(wx + rdx * hit);
        const y2 = val(wy + rdy * hit);
        const didHit = this.hitPt !== null;

        ctx.save();

        // Ray beam — solid if hit, faint if maxed out
        ctx.strokeStyle = didHit ? "rgba(255, 80, 80, 0.75)" : "rgba(255, 80, 80, 0.2)";
        ctx.lineWidth   = 1;
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        ctx.moveTo(x1, y1);
        ctx.lineTo(x2, y2);
        ctx.stroke();

        // Hit dot
        if (didHit) {
            ctx.fillStyle = "#ff3030";
            ctx.setLineDash([]);
            ctx.beginPath();
            ctx.arc(x2, y2, 3, 0, Math.PI * 2);
            ctx.fill();
        }

        ctx.restore();
    }
}
let robots = [];
class Robot {
    constructor(x, y, theta){
        robots.push(this);
        this.pos = [x, y];
        this.theta = theta;
        this.size = [15, 15];

        this.target = null;        // [x, y]
        this.controller = MTP;
        this.state = [];
        this.inauton = false;

        this.trackWidth = this.size[1];      // wheel-to-wheel distance (tune to your units)
        this.maxSpeed   = 8;       // max motor output (same units as pos/tick)
        this.noise = 0.05;
        
        this.distancers = [];

        this.lateralPID = {
            kP: 2.0, kD: 0.10, kI: 0.00, kAW: 0.90,
            integral: 0, prevErr: 0, lastTime: performance.now()
        };
        this.angularPID = {
            kP: 3.0, kD: 0.05, kI: 0.00, kAW: 0.90,
            integral: 0, prevErr: 0, lastTime: performance.now()
        };

        this.lastPos = [x, y];
    }
    move(left, right){
        const maxOut = Math.max(Math.abs(left), Math.abs(right), this.maxSpeed);
        let leftPow  = (left  / maxOut) * this.maxSpeed;
        let rightPow = (right / maxOut) * this.maxSpeed;
        
        const v     = (leftPow + rightPow) / 2;
        const omega = (leftPow - rightPow) / this.trackWidth;

        this.theta  = wrapAngle(this.theta + omega);
        this.pos[0] += v * Math.cos(this.theta) * (1 + this.noise * (2 * Math.random() - 1));
        this.pos[1] += v * Math.sin(this.theta) * (1 + this.noise * (2 * Math.random() - 1));
    }
    tick() {
        if(this.inauton) return;
        this.castSensors();

        if (!this.target) return;
        
        let [leftPow, rightPow] = this.controller(this.target, this, this.state);
        this.move(leftPow, rightPow);
    }
    draw() {
        const cx = val(this.pos[0]);
        const cy = val(this.pos[1]);

        // Derive pixel dimensions from val() so any scale/offset in val is respected
        const pw = val(this.pos[0] + this.size[0]) - cx;
        const ph = val(this.pos[1] + this.size[1]) - cy;
        const dotR = pw * 0.13; // front-dot radius, proportional to body width

        ctx.save();
        ctx.translate(cx, cy);
        ctx.rotate(this.theta);   // theta=0 faces +X, which is now "forward" after translate

        // Body
        ctx.fillStyle = "#222";
        ctx.fillRect(-pw / 2, -ph / 2, pw, ph);

        // Front-edge indicator dots (right side of robot = +X = forward)
        ctx.fillStyle = "#e74c3c";
        for (const side of [-1, 1]) {
            ctx.beginPath();
            ctx.arc(pw / 2, side * ph / 4, dotR, 0, Math.PI * 2);
            ctx.fill();
        }

        ctx.restore();

        for (const s of this.distancers) s.draw();
    }

    // distance
    addSensor(ox, oy, oTheta, maxRange = 144) {
        const sensor = new Distance(this, ox, oy, oTheta, maxRange);
        this.distancers.push(sensor);
        return sensor;   // return it so you can store a reference if needed
    }

    castSensors() {
        for (const s of this.distancers) s.cast();
    }

    async autonomous(controller, target){
        this.inauton = true;
        do {
            let [leftPow, rightPow] = controller(target, this, this.state);
            this.draw();
            await new Promise(res=>setTimeout(res, 3));
            this.move(leftPow, rightPow);
            if(leftPow == 0 && rightPow == 0) return;
        } while (3);
    }
}
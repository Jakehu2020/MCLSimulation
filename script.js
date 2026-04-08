const canvas = document.querySelector("#canvas");
const ctx = canvas.getContext("2d");

canvas.width = 144 * 4;
canvas.height = 144 * 4;

function drawField(){
    ctx.fillStyle='#a4a4a4';
    ctx.fillRect(0, 0, 9999, 9999);
    // draw the field
    ctx.fillStyle="#000";
    for(var i=0;i<6;i++){
        ctx.fillRect(val(i*24), 0, 0.5, 999);
        ctx.fillRect(0, val(i*24), 999, 0.5);
    }


    // field elements
    ctx.fillStyle="#a4952d55";
    let longGoalSize = 16;
    let longGoalHeight = 58;
    ctx.fillRect(val(24 - longGoalSize/2), val(96 - longGoalHeight), val(longGoalSize), val(longGoalHeight * 2 - 48));
    ctx.fillRect(val(120-longGoalSize/2), val(96-longGoalHeight), val(longGoalSize), val(longGoalHeight * 2 - 48));

    ctx.rotate(Math.PI/4);
    ctx.fillRect(val(96), val(-17), val(14), val(34));
    ctx.fillRect(val(85), val(-7), val(35), val(14));

    ctx.resetTransform();

}

drawField();

const r = new Robot(40, 40, 90*Math.PI/180);
r.addSensor(   7.5,   3.5,    0            );  // front-right, forward
r.addSensor(   7.5,  -3.5,    0            );  // front-left,  forward
r.addSensor(   0,     7.5,    Math.PI / 2  );  // left side,   90° left
r.addSensor(   0,    -7.5,   -Math.PI / 2  );  // right side,  90° right
r.addSensor(  -7.5,   0,      Math.PI      );  // rear,        backward

const ghostRobot = new Robot(40, 40, Math.PI/2);
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
let p = [Math.random() * 144,Math.random() * 144];
const waypoints = Array(12).fill``.map(x=>[Math.random() * 144,Math.random() * 144]);
const path      = bezierPath(waypoints, { resolution: 24, tension: 0.4 });
r.state   = { index: 0 };
// r.target = path;

setInterval(()=>{
    drawField();
    r.draw();
    r.tick();
    // ghostRobot.draw();

    drawPath(path, waypoints, r.state, {
        robot:        r,
        lookahead:    15,
        showProgress: true,
        showLookahead: true,
    });

    // r.controller = ramsete;//(path, r, r.state, { lookahead: 15 });


    // MCL
    // const globalDx    = r.pos[0] - lastPos[0];
    // const globalDy    = r.pos[1] - lastPos[1];
    // const dTheta      = wrapAngle(r.theta - lastTheta);
    // const cos = Math.cos(lastTheta), sin = Math.sin(lastTheta);
    // const localDx     =  cos * globalDx + sin * globalDy;   // into robot frame
    // const localDy     = -sin * globalDx + cos * globalDy;
    // lastPos   = [...r.pos];
    // lastTheta = r.theta;

    // mcl.step(localDx, localDy, dTheta);
    // mcl.draw();

    // const s = mcl.stats();
    // ctx.fillStyle = "#fff";
    // ctx.font = "12px monospace";
    // ctx.fillText(`particles: ${s.particles}  Neff: ${s.neff}  maxW: ${s.maxWeight}`, 8, 16);
    // ctx.fillText(`est  x:${s.estimate.x}  y:${s.estimate.y}  θ:${s.estimate.theta}`, 8, 32);

    ctx.fillStyle = "#fff";
    ctx.font = "12px monospace";
    ctx.fillText(`X: ${r.pos[0]}   Y: ${r.pos[1]}  Theta: ${r.theta}`, 8, 16);
}, 80);
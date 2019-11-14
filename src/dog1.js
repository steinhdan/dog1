var SerialPort = require("serialport");
var serialPort = new SerialPort('/dev/ttyACM0', { baudRate: 115200 });
var joystick = new (require('joystick'))(0, 2000, 350);
const rs2 = require('./index.js'); // The Intel Realsense SDK
let Controller = require('node-pid-controller');
/*
const mqtt = require('mqtt');

const client  = mqtt.connect('mqtt://mosquito.com');
client.subscribe('dog1');
client.on('message', function (topic, messageBuffer) {
    try {
        var message = JSON.parse(messageBuffer.toString());
        if (message.command === 'scan object') {
            startNavigation();
        }
    } catch (e) { }
});
*/

var config = new rs2.Config();
//config.enableAllStreams();
config.enableStream('pose', -1, 0, 0, '6dof', 0);
const pipeline = new rs2.Pipeline();
const profile = pipeline.start(config);

var count = 0;
var poseCount = 0;
var posFwd = 0;
var posSide = 0;

let ctrSide = new Controller({
    k_p: 0.25,
    k_i: 0.000,
    k_d: 0.006
  });
ctrSide.setTarget(0);
let ctrFwd = new Controller({
  k_p: 0.25,
  k_i: 0.000,
  k_d: 0.006
});
ctrFwd.setTarget(0);
  

// State / config
var speeds = {
    right:   { current: 0, wanted: 0, accel: 2, decel: 2 },
    forward: { current: 0, wanted: 0, accel: 2, decel: 2 },
    rotate:  { current: 0, wanted: 0, accel: 2, decel: 2 },
    up:      { current: 0, wanted: 0, accel: 10, decel: 10 }
};
var speedsArray = [ speeds.right, speeds.forward, speeds.rotate, speeds.up ];

// Upate current speeds based on wanted speed from joystick and accelleration limits
var accelInterval = 20;
setInterval(function () {
    speedsArray.forEach(speed => {
        var accel = (Math.abs(speed.current) > Math.abs(speed.wanted) ? speed.decel : speed.accel) * accelInterval / 1000;
        if (Math.abs(speed.current - speed.wanted) < accel) {
            speed.current = speed.wanted;
        } else {
            speed.current += speed.current > speed.wanted ? -accel : accel;
        }
    });
}, accelInterval);


var PI = Math.PI;
var path = [
    { x:  0.0,  z:  0.0,  yaw:  0.5  * PI }, // Turn left
    { x: -0.30, z:  0.0,  yaw:  0.5  * PI }, // Walk left
    { x: -0.30, z: -0.2,  yaw: -0.25  * PI }, // Turn right and walk a little towards object
    { delay: 1000 },
    { x: -0.22, z: -0.14, yaw: -0.17 * PI },
    { delay: 1000 },
    { x: -0.15, z: -0.1,  yaw: -0.1  * PI }, 
    { delay: 1000 },
    { x: -0.07, z: -0.07, yaw: -0.05 * PI }, 
    { delay: 1000 },
    { x:  0.0,  z: -0.05, yaw:  0.0  * PI }, 
    { delay: 1000 },
    { x:  0.07, z: -0.07, yaw:  0.05 * PI }, 
    { delay: 1000 },
    { x:  0.15, z: -0.1,  yaw:  0.1  * PI }, 
    { delay: 1000 },
    { x:  0.22, z: -0.14, yaw:  0.17 * PI },
    { delay: 1000 },
    { x:  0.30, z: -0.2,  yaw:  0.25  * PI }, // Final scan position
    { delay: 1000 },
    { x:  0.30, z:  0.0,  yaw:  0.5  * PI }, // Walk left
    { x:  0.0,  z:  0.0,  yaw:  0.5  * PI },
    { x:  0.0,  z:  0.0,  yaw:  0.0  * PI }
];
var pathIndex = 0;
var isAutonomous = false;

function startNavigation() {
    if (!isAutonomous) {
        isAutonomous = true;
        trot = true;
        pathIndex = 0;
        initialPose = currentPose;
    }    
}

function stopNavigation() {
    if (isAutonomous) {
        isAutonomous = false;
        speeds.forward.wanted = 0;
        speeds.right.wanted = 0;
        speeds.rotate.wanted = 0;
    }
}

var resumeTime = 0;
function navigate() {
    // Check if we are at the wanted position
    var targetPose = path[pathIndex];
    var initialYaw = initialPose.yaw;
    var x = currentPose.x - initialPose.x;
    var z = currentPose.z - initialPose.z;
    var currentPoseAdj = {
        yaw: currentPose.yaw - initialYaw,
        x: Math.cos(initialYaw) * x + Math.sin(initialYaw) * z,
        z: -Math.sin(initialYaw) * x + Math.cos(initialYaw) * z
    };

    console.log('Navi:', pathIndex, initialPose, currentPose, currentPoseAdj, targetPose);

    var xErr, zErr, yawErr;

    if (targetPose.delay) {
        if (Date.now() > resumeTime) {
            resumeTime = 0;
            pathIndex ++;
        }
    } else {
        xErr = targetPose.x - currentPoseAdj.x;
        zErr = targetPose.z - currentPoseAdj.z;
        yawErr = targetPose.yaw - currentPoseAdj.yaw;
    
        if ((Math.abs(xErr) < 0.02) &&
            (Math.abs(zErr) < 0.02) &&
            (Math.abs(yawErr) < 0.05 * PI) ) {
    
            // If yes, advance to next step
            pathIndex ++;
        }
    }

    // If no more steps, stop navigation
    if (pathIndex >= path.length) {
        stopNavigation();
    } else {
        targetPose = path[pathIndex];
        if (targetPose.delay) {
            if (resumeTime === 0) {
                resumeTime = Date.now() + targetPose.delay;
            }
            speeds.right.wanted = 0;
            speeds.forward.wanted = 0;
            speeds.rotate.wanted = 0;
        } else {
            xErr = targetPose.x - currentPoseAdj.x;
            zErr = targetPose.z - currentPoseAdj.z;
            yawErr = targetPose.yaw - currentPoseAdj.yaw;
        
            // Calculate speeds:
            // forwardSpeed = Math.sign(Err) * 0.5;
            var xSpeed = Math.min(1, Math.max(-1, xErr * 10)); 
            var zSpeed = Math.min(1, Math.max(-1, zErr * 10)); 
            var currentYaw = currentPoseAdj.yaw;
            speeds.right.wanted = ( Math.cos(-currentYaw) * xSpeed + Math.sin(-currentYaw) * zSpeed ) * 0.5;
            speeds.forward.wanted = -( -Math.sin(-currentYaw) * xSpeed + Math.cos(-currentYaw) *zSpeed ) * 0.8;
            speeds.rotate.wanted = -Math.min(1, Math.max(-1, yawErr * 5)) * 0.4;
        }
        console.log({ forwards: speeds.forward.wanted, right: speeds.right.wanted, rotate: speeds.rotate.wanted });
    }
}


var initialPose = { x: 0, z: 0, yaw: 0 };
var currentPose = { x: 0, z: 0, yaw: 0 };



var frameSet;
setInterval(() => {
  frameSet = pipeline.pollForFrames();
  if (frameSet) {
    frameSet.forEach(frame => {
      count++;
      if (frame && frame.isValid && frame instanceof rs2.PoseFrame) {
        // console.log(frame.poseData);
        poseCount++;
        // pos = Math.max(-0.9, Math.min(0.9, 3.0 * frame.poseData.rotation.z));
        let output = frame.poseData.rotation.x; // The error
        let input  = ctrFwd.update(output);
        posFwd += input;
        posFwd = Math.max(posFwd, -1);
        posFwd = Math.min(posFwd, 1);
        fwdBal = posFwd * 10;

        output = frame.poseData.rotation.z; // The error
        input  = ctrSide.update(output);
        posSide += input;
        posSide = Math.max(posSide, -1);
        posSide = Math.min(posSide, 1);
        sideBal = -posSide * 10;

        var angles = toEulerAngles(frame.poseData.rotation);
        currentPose = { x: frame.poseData.translation.x, z: frame.poseData.translation.z, yaw: angles.yaw };
        // console.log(currentPose);
         
//         console.log({ output, input, pos });
      }
      frame.destroy();
    });
    //frameSet.destroy();
  }
}, 5);

function toEulerAngles(qIn)
{
    q = {
        x: qIn.x,
        y: qIn.z,
        z: qIn.y,
        w: qIn.w
    }
    var angles = {};

    // roll (x-axis rotation)
    var sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
    var cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    angles.roll = Math.atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    var sinp = +2.0 * (q.w * q.y - q.z * q.x);
    if (Math.abs(sinp) >= 1)
        angles.pitch = copysign(Math.PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = Math.asin(sinp);

    // yaw (z-axis rotation)
    var siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
    var cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);  
    angles.yaw = Math.atan2(siny_cosp, cosy_cosp);

    return angles;
}

function copysign(x, y) {
    return Math.abs(x) * Math.sign(y);
}


var legWidth = 100;
var legDist = 160;
var thighLength = 64;
var shinLength = 75 - 8;
var legLength = thighLength + shinLength;
var initialHeight = legLength * 0.66;
var groundAngle = { x: 0, y: 0, z: 0 };


// Initial leg pose
var legs = [
    {
        servos: { knee: 0, thigh: 1, shoulder: 2 },
        pos : { x: legDist / 2, y: legWidth / 2, z: 0, kneeAngle: 0, thighAngle: 0 },
        target : { x: legDist / 2, y: legWidth / 2, z: -initialHeight, kneeAngle: 0, thighAngle: 0 }
    }, {
        servos: { knee: 3, thigh: 4, shoulder: 5 },
        pos : { x: legDist / 2, y: -legWidth / 2, z: 0, kneeAngle: 0, thighAngle: 0 },
        target : { x: legDist / 2, y: -legWidth / 2, z: -initialHeight, kneeAngle: 0, thighAngle: 0 }
    }, {
        servos: { knee: 6, thigh: 7, shoulder: 8 },
        pos : { x: -legDist / 2, y: legWidth / 2, z: 0, kneeAngle: 0, thighAngle: 0 },
        target : { x: -legDist / 2, y: legWidth / 2, z: -initialHeight, kneeAngle: 0, thighAngle: 0 }
    }, {
        servos: { knee: 9, thigh: 10, shoulder: 11 },
        pos: { x: -legDist / 2, y: -legWidth / 2, z: 0, kneeAngle: 0, thighAngle: 0},
        target: { x: -legDist / 2, y: -legWidth / 2, z: -initialHeight, kneeAngle: 0, thighAngle: 0 }
    }
];

legs.forEach(leg => {
    leg.tangent = Math.atan2(leg.pos.x, leg.pos.y);
});

function getKneeAngle(a, b, c) {
    return Math.acos((a*a + b*b - c*c) / ( 2 * a * b ));
}

function getThighAngle(a, b, c, legAngle) {
    return Math.acos((c * c + a * a - b * b) / ( 2 * c * a)) - legAngle;
}

function calculateIK(leg) {
    var xD = leg.pos.x - leg.target.x;
    var yD = leg.pos.y - leg.target.y;
    var zD = leg.pos.z - leg.target.z;
    var legDist = Math.sqrt(xD * xD + yD * yD + zD * zD);
    var legAngle = Math.atan(xD / zD);
    leg.target.shoulderAngle = Math.atan(yD / zD) * 180 / Math.PI;
    leg.target.kneeAngle = 180 - getKneeAngle(thighLength, shinLength, legDist) * 180 / Math.PI;
    leg.target.thighAngle = getThighAngle(thighLength, shinLength, legDist, legAngle) * 180 / Math.PI;
    // console.log({ kneeAngle: leg.target.kneeAngle, thighAngle: leg.target.thighAngle, legDist, legAngle });
}


var servos = [
//    { channel: 0, lowPos: { deg: 0, val: 2208 }, hiPos: { deg: 90, val: 1221 } },
{ channel: 0, lowPos: { deg: 0, val: 2210 }, hiPos: { deg: 90, val: 1221 } },
//    { channel: 1, lowPos: { deg: 0, val: 1239 }, hiPos: { deg: 90, val: 2202 } },
    { channel: 1, lowPos: { deg: 0, val: 1200 }, hiPos: { deg: 90, val: 2202 } },
    { channel: 2, lowPos: { deg: -35, val: 891 }, hiPos: { deg: 35, val: 1667 } },

//    { channel: 3, lowPos: { deg: 0, val: 736 }, hiPos: { deg: 90, val: 1725 } },
    { channel: 3, lowPos: { deg: 0, val: 736 }, hiPos: { deg: 90, val: 1725 } },
//    { channel: 4, lowPos: { deg: 0, val: 1897 }, hiPos: { deg: 90, val: 890 } },
{ channel: 4, lowPos: { deg: 0, val: 1990 }, hiPos: { deg: 90, val: 890 } },
 //   { channel: 5, lowPos: { deg: -35, val: 1191 }, hiPos: { deg: 35, val: 2022} },
    { channel: 5, lowPos: { deg: -35, val: 1221 }, hiPos: { deg: 35, val: 2052} },

//    { channel: 9, lowPos: { deg: 0, val: 2258 }, hiPos: { deg: 90, val: 1269 } },
    { channel: 9, lowPos: { deg: 0, val: 2230 }, hiPos: { deg: 90, val: 1269 } },
    { channel: 10, lowPos: { deg: 0, val: 1226 }, hiPos: { deg: 90, val: 2188 } },
    { channel: 11, lowPos: { deg: -35, val: 1018 }, hiPos: { deg: 35, val: 1811 } },

//    { channel: 12, lowPos: { deg: 0, val: 530 }, hiPos: { deg: 90, val: 1521 } },
    { channel: 12, lowPos: { deg: 0, val: 570 }, hiPos: { deg: 90, val: 1521 } },
//    { channel: 13, lowPos: { deg: 0, val: 1884 }, hiPos: { deg: 90, val: 923 } },
    { channel: 13, lowPos: { deg: 0, val: 1844 }, hiPos: { deg: 90, val: 923 } },
    { channel: 14, lowPos: { deg: -35, val: 998 }, hiPos: { deg: 35, val: 1797} }
];

servos.forEach((servo, i) => {
    servo.target = 1500;
    // servo.degSteps = (servo.hiPos.val - servo.lowPos.val) / (servo.hiPos.deg - servo.lowPos.deg);
    servo.degSteps = Math.sign((servo.hiPos.val - servo.lowPos.val) / (servo.hiPos.deg - servo.lowPos.deg)) * 11;
    servo.offset = servo.lowPos.val - servo.lowPos.deg * servo.degSteps;
    console.log({i, degSteps: servo.degSteps});
});



function setPos(servo, deg) {
    servo.target = deg * servo.degSteps + servo.offset;
}



setInterval(() => {
    // console.log({ travelDirection, speed, speeds.forward.wanted, speeds.right.wanted, rotateSpeed, height, stepOffset, stepLength })
    // console.log('Frames: ' + count + ', poses: ' + poseCount);
    count = 0;
    poseCount = 0;
    // gc();
  
}, 500);


var speed = 0;
var lastSpeed = 0;
var travelDirection = 0;
//var stepOffset = 6;
var stepOffset = 5.5;
var sideOffset = 0;
var stepLength;
var height = 50;
var stepOffsetSpeed = 0;
var sideOffsetSpeed = 0;

var stepCycle2 = [
    { x:  9, z: 3 },
    { x:  8, z: 1 },
    { x:  7, z: 0 },
    { x:  6, z: 0 },
    { x:  5, z: 0 },
    { x:  4, z: 0 },
    { x:  3, z: 0 },
    { x:  2, z: 0 },
    { x:  1, z: 0 },
    { x:  0, z: 0 },
    { x: -1, z: 0 },
    { x: -2, z: 0 },
    { x: -3, z: 0 },
    { x: -4, z: 0 },
    { x: -5, z: 0 },
    { x: -6, z: 0 },
    { x: -7, z: 0 },
    { x: -8, z: 0 },
    { x: -9, z: 2 }
];
var stepCycle3 = [
    { x:  6, z: 3 },
    { x:  5, z: 1 },
    { x:  4, z: 0 },
    { x:  3, z: 0 },
    { x:  2, z: 0 },
    { x:  1, z: 0 },
    { x:  0, z: 0 },
    { x: -1, z: 0 },
    { x: -2, z: 0 },
    { x: -3, z: 0 },
    { x: -4, z: 0 },
    { x: -5, z: 0 },
    { x: -6, z: 2 }
];
var stepCycle = [
    { x:  5, z: 3 },
    { x:  4, z: 1 },
    { x:  3, z: 0 },
    { x:  2, z: 0 },
    { x:  1, z: 0 },
    { x:  0, z: 0 },
    { x: -1, z: 0 },
    { x: -2, z: 0 },
    { x: -3, z: 0 },
    { x: -4, z: 0 },
    { x: -5, z: 2 }
];

function getStepPos(t) {
  var tStep = 1 / stepCycle.length;

  var fractionalIndex = t / tStep;
  var startIndex = Math.floor(fractionalIndex);
  var tween = fractionalIndex - startIndex;
  var endIndex = (startIndex + 1) % stepCycle.length;
  var startPos = stepCycle[startIndex];
  var endPos = stepCycle[endIndex];
  // console.log({ startIndex, endIndex, t, tween });
  return {
      x: startPos.x * (1 - tween) + endPos.x * tween,
      z: startPos.z * (1 - tween) + endPos.z * tween
  };
}

var lastTime = new Date().getTime();

var isStopped = true;
var isStarting = false;
var isWalking = false;
var isStopping = false;
var startWalkTime = 0;
var stopWalkTime = 0;

var trot = true;

var sideBal = -1.5;
var fwdBal = 0;

setInterval(function() {

    if (isAutonomous) {
        navigate();
    }

    var now = new Date().getTime();
    var tDiff = now - lastTime;
    height += speeds.up.current * tDiff / 15;
    height = Math.max(2, height);
    height = Math.min(75, height);
    stepOffset += stepOffsetSpeed * tDiff / 12;
    stepOffset = Math.max(-40, stepOffset);
    stepOffset = Math.min(40, stepOffset);
    sideOffset += sideOffsetSpeed * tDiff / 6;
    sideOffset = Math.max(-20, sideOffset);
    sideOffset = Math.min(20, sideOffset);

    lastTime = now;

    legs[0].target.z = -(legLength - height + sideBal + fwdBal);
    legs[1].target.z = -(legLength - height - sideBal + fwdBal);
    legs[2].target.z = -(legLength - height + sideBal - fwdBal);
    legs[3].target.z = -(legLength - height - sideBal - fwdBal);
    legs[0].target.x = legs[0].pos.x + stepOffset;
    legs[1].target.x = legs[1].pos.x + stepOffset;
    legs[2].target.x = legs[2].pos.x + stepOffset;
    legs[3].target.x = legs[3].pos.x + stepOffset;

    var travelSpeed = Math.max(Math.abs(speeds.forward.current), Math.abs(speeds.right.current));
    var rotateSpeedAbs = Math.abs(speeds.rotate.current * 0.7);
    speed = Math.max(travelSpeed, rotateSpeedAbs);
    travelDirection = speed === 0 ? 0 : Math.atan2(speeds.right.current, speeds.forward.current);

    // trot = speed > 0.3;

    if (speed > 0 && lastSpeed === 0 ) {
        startWalkTime = now;
    }
    if (speed === 0 && lastSpeed > 0 ) {
        stopWalkTime = now;
    }
    lastSpeed = speed;

    var walkT = (now - startWalkTime) / (trot ? 400 : 2000) + 0.25;

    var startScale = 1;

    if (trot || startWalkTime === 0) {
        sideOffset = 0;
    } else {
        if (stopWalkTime > startWalkTime) {
            startScale = Math.max(0, (1 - (now - stopWalkTime) / 500));
            sideOffset = -Math.cos((walkT * 2) % 1 * Math.PI * 2) * 20 * startScale;
        } else if (walkT < 0.75) {
            startScale = (walkT - 0.25) * 2;
            sideOffset = -Math.cos((walkT * 2) % 1 * Math.PI * 2) * 20 * startScale;
        } else {
            sideOffset = -Math.cos((walkT * 2) % 1 * Math.PI * 2) * 20;
        }
    }


    legs[0].target.y = legs[0].pos.y + sideOffset;
    legs[1].target.y = legs[1].pos.y + sideOffset;
    legs[2].target.y = legs[2].pos.y + sideOffset;
    legs[3].target.y = legs[3].pos.y + sideOffset;

    if (Math.abs(speed) > 0) {
        stepHeight = 6 * startScale;
        stepLength = 3 * speed * ( trot ? 2 : 1) * startScale;
    } else {
        stepHeight = 0;
        stepLength = 0;
    }

    var tDelta = trot ? [0.5, 0.0, 0.0, 0.5] : [0.25, 0.5, 0.75, 0];

    legs.forEach((leg, i) => {
        var legTangent = leg.tangent + (speeds.rotate.current > 0 ? 0 : Math.PI );
        if (legTangent > Math.PI) { legTangent -= Math.PI * 2; } 
        var stepDirection = travelDirection;
        if (rotateSpeedAbs > 0) {
            var dirDiff = legTangent - travelDirection;
                while (dirDiff > Math.PI) { dirDiff -= Math.PI * 2; }
            while (dirDiff < -Math.PI) { dirDiff += Math.PI * 2; }
            var diffWeight = rotateSpeedAbs / (rotateSpeedAbs + travelSpeed);
            stepDirection = travelDirection + dirDiff * diffWeight;
        }

        var stepPos = getStepPos((walkT + tDelta[i]) % 1 );
        var x = stepPos.x * Math.cos(stepDirection);
        var y = stepPos.x * Math.sin(stepDirection);

        leg.target.x += -x * stepLength;
        leg.target.y += -y * stepLength;
        leg.target.z += stepPos.z * stepHeight;

        // Update positions based on ground angle
        // leg.target.z += leg.target.x * Math.sin(groundAngle.y) + leg.target.y * Math.sin(groundAngle.x);
        // leg.target.x *= Math.cos(groundAngle.y);
        // leg.target.y *= Math.cos(groundAngle.x);

        calculateIK(leg);
        setPos(servos[leg.servos.knee], leg.target.kneeAngle);
        setPos(servos[leg.servos.thigh], leg.target.thighAngle);
        setPos(servos[leg.servos.shoulder], leg.target.shoulderAngle);
    });


}, 20);

setInterval(() => {
    servos.forEach((servo, i) => {
        var target = Math.round(servo.target * 4);
        serialPort.write([0x84, servo.channel, target & 0x7F, target >> 7 & 0x7F]);
    });

}, 20);

joystick.on('button', function(event) {
    if (event.init) return;

    if (event.value === 0 && event.number === 2) {
        trot = !trot;
        console.log('Walk mode: ' + (trot ? 'trot' : 'walk'));
    }
    if (event.value === 0 && event.number === 1) {
        console.log('Start navigation');
        startNavigation();
    }
    if (event.value === 0 && event.number === 3) {
        console.log('Stop navigation');
        stopNavigation();
    }
    // console.log(event);
});


joystick.on('axis', function(event) {
    if (event.init) return;

    // Typical Event: { time: 22283520, value: 32636, number: 3, type: 'axis', id: 0 }
    var value = event.value / 32768; // Normalize to the range -1.0 - 0 - 1.0

    // Pad1 left/right
    if(event.number === 3) {
        // sideOffsetSpeed = value;
        speeds.right.wanted = value;
    }

    // Pad1 up/down
    if(event.number === 4) {
        speeds.forward.wanted = value === 0 ? value : -value;
    }

    // Pad2 up/down
    if(event.number === 1) {
    }

    // Pad2 left/right
    if(event.number === 0) {
        // stepOffsetSpeed = value;
        speeds.rotate.wanted = value;
    }

    // Set height by shoulder buttons
    if(event.number === 5) { // Right
        speeds.up.wanted = value > 0 ? -value : 0;;
    }
    if(event.number === 2) { // Left
        speeds.up.wanted = value > 0 ? value : 0;;
    }

    if(event.number === 6 && value !== 0) { // Pad left/right
        sideBal += Math.sign(value);
    }    

    if(event.number === 7 && value !== 0) { // Pad up/down
        fwdBal += Math.sign(value);
    }    

    // console.log(event);
});

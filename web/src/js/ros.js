var errorYaw = document.getElementById("errorYaw")
var pitchYaw = document.getElementById("pitchYaw")
var rollYaw = document.getElementById("rollYaw")

var setpointThrottle = document.getElementById("setpointThrottle")
var setpointYaw = document.getElementById("setpointYaw")
var setpointPitch = document.getElementById("setpointPitch")
var setpointRoll = document.getElementById("setpointRoll")

var rcThrottle = document.getElementById("rcThrottle")
var rcYaw = document.getElementById("rcYaw")
var rcPitch = document.getElementById("rcPitch")
var rcRoll = document.getElementById("rcRoll")

var motorFrontLeft = document.getElementById("motorFrontLeft")
var motorFrontRight = document.getElementById("motorFrontRight")
var motorBackLeft = document.getElementById("motorBackLeft")
var motorBackRight = document.getElementById("motorBackRight")

var motorSwitch = document.getElementById("stopMotors")
var rebootSwitch = document.getElementById("rebootDrone")

var ros = new ROSLIB.Ros({
    url: 'ws://127.0.0.1:9090'
})

var imu = new ROSLIB.Topic({
    ros: ros,
    name: '/imu_measures',
    messageType: 'drone/ImuMeasures'
})

var setpoints = new ROSLIB.Topic({
    ros: ros,
    name: '/setpoints',
    messageType: 'drone/Setpoints'
})

var errors = new ROSLIB.Topic({
    ros: ros,
    name: '/imu_errors',
    messageType: 'drone/Errors'
})

var rcValues = new ROSLIB.Topic({
    ros: ros,
    name: '/rc_values',
    messageType: 'drone/RCValues'
})

var motors = new ROSLIB.Topic({
    ros: ros,
    name: '/motors',
    messageType: 'drone/Motors'
})

var reboot = new ROSLIB.Topic({
    ros: ros,
    name: '/reboot',
    messageType: 'std_msgs/Empty'
})

var updateParams = new ROSLIB.Topic({
    ros: ros,
    name: '/update_params',
    messageType: 'std_msgs/Empty'
})

var motorEnabler = new ROSLIB.Topic({
    ros: ros,
    name: '/enable_motors',
    messageType: 'std_msgs/Bool'
})

function drawIMU(message) {
    if (roll.length > hertz * graphSeconds) {
        roll.shift()
        pitch.shift()
        yaw.shift()
    }
    yaw.push({ x: graphTimer, y: message.yaw })
    pitch.push({ x: graphTimer, y: message.pitch })
    roll.push({ x: graphTimer, y: message.roll })
    graphTimer += 1 / hertz
}

function drawSetpoints(message) {
    if (yawSetpoint.length > hertz * graphSeconds) {
        yawSetpoint.shift()
        pitchSetpoint.shift()
        rollSetpoint.shift()
    }
    yawSetpoint.push({ x: graphTimer, y: message.set_yaw })
    pitchSetpoint.push({ x: graphTimer, y: message.set_pitch })
    rollSetpoint.push({ x: graphTimer, y: message.set_roll })

    setpointThrottle.innerHTML = map(message.set_throttle, 1000, 2000, 0, 100).toFixed(0)
    setpointYaw.innerHTML = message.set_yaw.toFixed(1)
    setpointPitch.innerHTML = message.set_pitch.toFixed(1)
    setpointRoll.innerHTML = message.set_roll.toFixed(1)
}

function drawErrors(message) {
    errorYaw.innerHTML = message.yaw_error.toFixed(1)
    pitchYaw.innerHTML = message.pitch_error.toFixed(1)
    rollYaw.innerHTML = message.roll_error.toFixed(1)
}

function drawRC(message) {
    rcThrottle.innerHTML = message.rc_throttle
    rcYaw.innerHTML = message.rc_yaw
    rcPitch.innerHTML = message.rc_pitch
    rcRoll.innerHTML = message.rc_roll
}

function drawMotors(message) {
    motorFrontLeft.innerHTML = map(message.front_left, 1000, 2000, 0, 100).toFixed(0)
    motorFrontRight.innerHTML = map(message.front_right, 1000, 2000, 0, 100).toFixed(0)
    motorBackLeft.innerHTML = map(message.back_left, 1000, 2000, 0, 100).toFixed(0)
    motorBackRight.innerHTML = map(message.back_right, 1000, 2000, 0, 100).toFixed(0)
}


function rebootDrone() {
    setTimeout(function () { rebootSwitch.checked = true; }, 350);
    reboot.publish()

}

function motorSwitchFunction() {
    var boolean = motorSwitch.checked
    console.log(boolean)
    motorEnabler.publish({ data: boolean })
}

window.onload = function () {
    drawPID()
    imu.subscribe(drawIMU);
    setpoints.subscribe(drawSetpoints)
    errors.subscribe(drawErrors)
    rcValues.subscribe(drawRC)
    motors.subscribe(drawMotors)

    setInterval(function () {
        for (graph of graphs) {
            graph.update()
        }
    }, 1 / refreshRateHz * 1000)

    setInterval(drawPID(), 5000)
}

function map(n, start1, stop1, start2, stop2) {
    return ((n - start1) / (stop1 - start1)) * (stop2 - start2) + start2;
};

var ros = new ROSLIB.Ros({
    url: 'ws://192.168.1.100:9090'
});

var imu = new ROSLIB.Topic({
    ros: ros,
    name: '/imu',
    messageType: 'geometry_msgs/Vector3'
});

var setpoints = new ROSLIB.Topic({
    ros: ros,
    name: '/setpoints',
    messageType: 'geometry_msgs/Vector3'
});

function drawIMU(message) {
    if (roll.length > hertz * graphSeconds) {
        roll.shift()
        pitch.shift()
        yaw.shift()
    }

    roll.push({ x: graphTimer, y: message.x })
    pitch.push({ x: graphTimer, y: message.y })
    yaw.push({ x: graphTimer, y: message.z })
    graphTimer += 1 / hertz
}

function drawSetpoints() {
    if (yawSetpoint.length > hertz * graphSeconds) {
        yawSetpoint.shift()
        pitchSetpoint.shift()
        rollSetpoint.shift()
    }

    roll.push({ x: graphTimer, y: message.x })
    pitch.push({ x: graphTimer, y: message.y })
    yaw.push({ x: graphTimer, y: message.z })
}

imu.subscribe(drawIMU);
setpoints.subscribe(drawSetpoints)

setInterval(function () {
    for (var graph of graphs) {
        graph.update()
    }
}, 1 / refreshRateHz * 1000)
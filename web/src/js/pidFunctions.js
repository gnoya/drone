var kpYaw = document.getElementById("kpYaw")
var kpPitch = document.getElementById("kpPitch")
var kpRoll = document.getElementById("kpRoll")

var kdYaw = document.getElementById("kdYaw")
var kdPitch = document.getElementById("kdPitch")
var kdRoll = document.getElementById("kdRoll")

var kiYaw = document.getElementById("kiYaw")
var kiPitch = document.getElementById("kiPitch")
var kiRoll = document.getElementById("kiRoll")


var kpConstants = new ROSLIB.Param({
    ros: ros,
    name: 'kp'
});
var kdConstants = new ROSLIB.Param({
    ros: ros,
    name: 'kd'
});
var kiConstants = new ROSLIB.Param({
    ros: ros,
    name: 'ki'
});

function drawPID() {
    kpConstants.get(function (value) {
        kpYaw.innerHTML = value[0].toFixed(2)
        kpPitch.innerHTML = value[1].toFixed(2)
        kpRoll.innerHTML = value[2].toFixed(2)
    });

    kdConstants.get(function (value) {
        kdYaw.innerHTML = value[0].toFixed(2)
        kdPitch.innerHTML = value[1].toFixed(2)
        kdRoll.innerHTML = value[2].toFixed(2)
    });

    kiConstants.get(function (value) {
        kiYaw.innerHTML = value[0].toFixed(2)
        kiPitch.innerHTML = value[1].toFixed(2)
        kiRoll.innerHTML = value[2].toFixed(2)
    });
}



function updatekpYaw() {
    var constant = parseFloat(prompt("Kp Yaw"));
    if (!isNaN(constant) && constant >= 0) {
        kpYaw.innerHTML = constant.toFixed(2)
        updatePIDParams("kp")
    }
}

function updatekpPitch() {
    var constant = parseFloat(prompt("Kp Pitch"));
    if (!isNaN(constant) && constant >= 0) {
        kpPitch.innerHTML = constant.toFixed(2)
        updatePIDParams("kp")
    }
}

function updatekpRoll() {
    var constant = parseFloat(prompt("Kp Roll"));
    if (!isNaN(constant) && constant >= 0) {
        kpRoll.innerHTML = constant.toFixed(2)
        updatePIDParams("kp")
    }
}

function updatekdYaw() {
    var constant = parseFloat(prompt("Kd Yaw"));
    if (!isNaN(constant) && constant >= 0) {
        kdYaw.innerHTML = constant.toFixed(2)
        updatePIDParams("kd")
    }
}

function updatekdPitch() {
    var constant = parseFloat(prompt("Kd Pitch"));
    if (!isNaN(constant) && constant >= 0) {
        kdPitch.innerHTML = constant.toFixed(2)
        updatePIDParams("kd")
    }
}

function updatekdRoll() {
    var constant = parseFloat(prompt("Kd Roll"));
    if (!isNaN(constant) && constant >= 0) {
        kdRoll.innerHTML = constant.toFixed(2)
        updatePIDParams("kd")
    }
}

function updatekiYaw() {
    var constant = parseFloat(prompt("Ki Yaw"));
    if (!isNaN(constant) && constant >= 0) {
        kiYaw.innerHTML = constant.toFixed(2)
        updatePIDParams("ki")
    }
}

function updatekiPitch() {
    var constant = parseFloat(prompt("Ki Pitch"));
    if (!isNaN(constant) && constant >= 0) {
        kiPitch.innerHTML = constant.toFixed(2)
        updatePIDParams("ki")
    }
}

function updatekiRoll() {
    var constant = parseFloat(prompt("Ki Roll"));
    if (!isNaN(constant) && constant >= 0) {
        kiRoll.innerHTML = constant.toFixed(2)
        updatePIDParams("ki")
    }
}


function updatePIDParams(type) {
    var newParameters, newYaw, newPitch, newRoll
    if (type == "kp") {
        newYaw = parseFloat(kpYaw.innerHTML)
        newPitch = parseFloat(kpPitch.innerHTML)
        newRoll = parseFloat(kpRoll.innerHTML)
        if (Number.isInteger(newYaw)) newYaw += 1e-15
        if (Number.isInteger(newPitch)) newPitch += 1e-15
        if (Number.isInteger(newRoll)) newRoll += 1e-15

        newParameters = [newYaw, newPitch, newRoll]
        kpConstants.set(newParameters)
    }
    else if (type == "kd") {
        newYaw = parseFloat(kdYaw.innerHTML)
        newPitch = parseFloat(kdPitch.innerHTML)
        newRoll = parseFloat(kdRoll.innerHTML)
        if (Number.isInteger(newYaw)) newYaw += 1e-15
        if (Number.isInteger(newPitch)) newPitch += 1e-15
        if (Number.isInteger(newRoll)) newRoll += 1e-15

        newParameters = [newYaw, newPitch, newRoll]
        kdConstants.set(newParameters)
    }
    else if (type == "ki") {
        newYaw = parseFloat(kiYaw.innerHTML)
        newPitch = parseFloat(kiPitch.innerHTML)
        newRoll = parseFloat(kiRoll.innerHTML)
        if (Number.isInteger(newYaw)) newYaw += 1e-15
        if (Number.isInteger(newPitch)) newPitch += 1e-15
        if (Number.isInteger(newRoll)) newRoll += 1e-15
        newParameters = [newYaw, newPitch, newRoll]
        kiConstants.set(newParameters)
    }
    updateParams.publish()
}
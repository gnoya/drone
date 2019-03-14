/*
TODO:
Cambiar el eje X, no se ve bien agarrar directamente los ms (creo yo).
Ajustar x,y,z a roll, pitch, yaw, respectivamente.
Mejorar el aspecto de la interfaz.
*/


var ros = new ROSLIB.Ros({
    url: 'ws://192.168.1.102:9090'
});

var listener = new ROSLIB.Topic({
    ros: ros,
    name: '/listener',
    messageType: 'geometry_msgs/Vector3'
});

timer = 0

function drawIMU(message) {
    var d = new Date()
    var n = d.getMilliseconds()
    timer += n / 100
    console.log(timer)
    console.log(message)
    // roll.shift()
    // pitch.shift()
    // yaw.shift()
    roll.push({ x: timer, y: message.x * timer / 10 })
    pitch.push({ x: timer, y: message.y * timer / 10 })
    yaw.push({ x: timer, y: message.z * timer / 10 })

    for (var graph of graphs) {
        graph.update()
    }
}

listener.subscribe(drawIMU);

i = 10

// setInterval(function () {
//     roll.push({ x: ++i / 12, y: i / 3 })
//     for (var graph of graphs) {
//         graph.update()
//     }
// }, 500)
const hertz = 100
const graphSeconds = 15
const refreshRateHz = 10

var offsetHeight = document.getElementById('chart_container').offsetHeight;
var offsetWidth = document.getElementById('chart_container').offsetWidth;


var size = {
    width: window.innerWidth || document.body.clientWidth,
    height: window.innerHeight || document.body.clientHeight
}

var yaw = []
var pitch = []
var roll = []

var yawSetpoint = []
var pitchSetpoint = []
var rollSetpoint = []

var graphTimer = 0

for (var i = 0; i < hertz * graphSeconds; i++) {
    yaw.push({ x: graphTimer, y: 0 })
    pitch.push({ x: graphTimer, y: 0 })
    roll.push({ x: graphTimer, y: 0 })
    yawSetpoint.push({ x: graphTimer, y: 0 })
    pitchSetpoint.push({ x: graphTimer, y: 0 })
    rollSetpoint.push({ x: graphTimer, y: 0 })
    graphTimer += 1 / hertz
}

var graphs = []
var angles = [yaw, pitch, roll]
var angle_setpoint = [yawSetpoint, pitchSetpoint, rollSetpoint]

var y_axiss = ['y_axis', 'y_axis2', 'y_axis3']
var charts = ['chart', 'chart2', 'chart3']
var legends = ['legend', 'legend2', 'legend3']
var names = ['Yaw', 'Pitch', 'Roll']
var tickValues = [[-20, -15, -10, 0, 10, 15, 20], [-25, -15, -5, 0, 5, 15, 25], [-25, -15, -5, 0, 5, 15, 25]]

for (var i = 0; i < 3; i++) {
    var graph = new Rickshaw.Graph({
        element: document.getElementById(charts[i]),
        width: offsetWidth * 0.85,
        height: offsetHeight * 1.0,
        renderer: 'line',
        interpolation: 'basis',
        min: tickValues[i][0],
        max: tickValues[i][tickValues[i].length - 1],
        padding: { top: 0.06, bottom: 0.06 },
        series: [{
            name: names[i],
            color: 'black',
            data: angles[i]
        },
        {
            name: 'Setpoint',
            color: 'steelblue',
            data: angle_setpoint[i]
        }
        ]
    });

    var x_axis = new Rickshaw.Graph.Axis.Time({ graph: graph });
    x_axis.render();
    var y_axis = new Rickshaw.Graph.Axis.Y({
        graph: graph,
        orientation: 'left',
        tickValues: tickValues[i],
        tickFormat: Rickshaw.Fixtures.Number.formatKMBT,
        element: document.getElementById(y_axiss[i]),

    });
    y_axis.render();

    graphs.push(graph)
    graph.render()
}
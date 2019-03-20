const hertz = 200
const graphSeconds = 15
const refreshRateHz = 10

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
var y_axiss = ['y_axis', 'y_axis2', 'y_axis3']
var charts = ['chart', 'chart2', 'chart3']

for (var i = 0; i < 3; i++) {
    let tickValues = [-45, 0, 45]
    min = tickValues[0]
    max = tickValues[tickValues.length - 1]
    if (i == 0) {
        data_ = yawSetpoint
        tickValues = [-90, 0, 90]
        min = tickValues[0]
        max = tickValues[tickValues.length - 1]
    } else if (i == 1) {
        data_ = pitchSetpoint
    } else if (i == 2) {
        data_ = rollSetpoint
    }

    var graph = new Rickshaw.Graph({
        element: document.getElementById(charts[i]),
        width: 800,
        height: 200,
        renderer: 'line',
        interpolation: 'basis',
        min: min,
        max: max,
        series: [{
            color: 'black',
            data: angles[i]
        },
        {
            color: 'red',
            data: data_
        }
        ]
    });
    var x_axis = new Rickshaw.Graph.Axis.Time({ graph: graph });
    var y_axis = new Rickshaw.Graph.Axis.Y({
        graph: graph,
        orientation: 'left',
        tickValues: tickValues,
        tickFormat: Rickshaw.Fixtures.Number.formatKMBT,
        element: document.getElementById(y_axiss[i]),
    });

    graphs.push(graph)
    graph.render()
}
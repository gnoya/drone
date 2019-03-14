var yaw = []
var pitch = []
var roll = []

var graphs = []
var angles = [yaw, pitch, roll]
var y_axiss = ['y_axis', 'y_axis2', 'y_axis3']
var charts = ['chart', 'chart2', 'chart3']

for (var i = 0; i < 3; i++) {
    var graph = new Rickshaw.Graph({
        element: document.getElementById(charts[i]),
        width: 500,
        height: 200,
        renderer: 'line',
        interpolation: 'basis',
        min: 'auto',
        series: [{
            color: 'black',
            data: angles[i]
        }]
    });
    var x_axis = new Rickshaw.Graph.Axis.Time({ graph: graph });
    var y_axis = new Rickshaw.Graph.Axis.Y({
        graph: graph,
        orientation: 'left',
        tickFormat: Rickshaw.Fixtures.Number.formatKMBT,
        element: document.getElementById(y_axiss[i]),
    });
    graphs.push(graph)
    graph.render()
}
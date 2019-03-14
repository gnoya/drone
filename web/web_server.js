const express = require('express');
const socket = require('socket.io');

var app = express();
var server = app.listen(3000);
var io = socket(server);
app.use(express.static('public'));
io.sockets.on('connection', newConnection);

function newConnection(socket) {
}
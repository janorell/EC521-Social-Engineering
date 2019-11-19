// Required module
var dgram = require('dgram');
var express = require('express');
var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);

// Port and IP
var PORT = 8082;
var HOST = '192.168.1.124';

// Create socket
var server = dgram.createSocket('udp4');
// Create server
server.on('listening', function () {
    var address = server.address();
    console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

var isStart;
var isStop;
// On connection


server.on('message', function (message, remote) {
        // Send Ok acknowledgement
        server.send(`${isStop}`,remote.port,remote.address,function(error){
          if(error){
            console.log('MEH!');
          }
          else{
            console.log(`Sent: ${isStop}`);
          }
          isStop = '';
        });
        server.send(`${isStart}`,remote.port,remote.address,function(error){
          if(error){
            console.log('MEH!');
          }
          else{
            console.log(`Sent: ${isStart}`);
          }
          isStart = '';
        });
});

// Points to index.html to serve webpage
app.get('/', function(req, res){
  res.sendFile(__dirname + '/client.html');
});

// on to get
	//when the server receives clicked message, do this
    io.on('stop', function(data) {
    	  console.log("stop received")
        isStop = data;
		  //send a message to ALL connected clients
    });

    io.on('start', function(data) {
    	  console.log("start received")
        isStart = data;
		  //send a message to ALL connected clients
    });

http.listen(3001, function() {
  console.log('running on :3001');
});
// Bind server to port and IP

io.on('connection', function(socket){

    socket.on('disconnect', function(){
    });
});
server.bind(PORT, HOST);

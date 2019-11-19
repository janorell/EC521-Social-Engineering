// Required module
var dgram = require('dgram');
var express = require('express');
var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);

// Port and IP
var PORT = 8082;
var HOST = '192.168.1.125';

// Create socket
var server = dgram.createSocket('udp4');
// Create server
server.on('listening', function () {
    var address = server.address();
    console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

var action = 0;
// On connection


// server.on('message', function (message, remote) {
//         // Send Ok acknowledgement
//         server.send(`${action}`,remote.port,remote.address,function(error){
//           if(error){
//             console.log('MEH!');
//           }
//           else{
//             console.log(`Sent: ${action}`);
//           }
//           action = 0;
//         });
// });
// On connection, print out received message
server.on('message', function (message, remote) {
    console.log(remote.address + ':' + remote.port +' - ' + message);
    //console.log("check sent");
    // Send Ok acknowledgement
    server.send(`${action}`,remote.port,remote.address,function(error){
      if(error){
        console.log('MEH!');
      }
      else{
        console.log(`Sent: ${action}`);
      }
      action = 0;
    });

});

// Points to index.html to serve webpage
app.get('/', function(req, res){
  res.sendFile(__dirname + '/client.html');
});

// on to get
	//when the server receives clicked message, do this
    io.on('stop', function(data) {
      //  data.on('stop', function(data){
    	  console.log("stop received")
        action = -1;
          io.emit('stopupdate', action);
      //});
		  //send a message to ALL connected clients
    });

    io.on('start', function(data) {
      //  data.on('start', function(data){
    	  console.log("start received")
        action = 1;
          io.emit('startupdate', action);
      //  });
		  //send a message to ALL connected clients
    });

http.listen(3333, function() {
  console.log('running on :3333');
});
// Bind server to port and IP

io.on('connection', function(client){

  client.on('start', function(data) {
    console.log("start clicked")
    action = 1;
  //send a message to ALL connected clients
    //io.emit('buttonUpdate', clickcheck);
  });
  client.on('stop', function(data) {
    console.log("stop clicked")
    action = -1;

  });

});
server.bind(PORT, HOST);

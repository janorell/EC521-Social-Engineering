var http = require('http');
var express = require('express');
var app = require('express')();


app.get('/', function(req, res){
  res.sendFile(__dirname + '/coinbase.html');
  console.log("running on 3000");
});
app.listen(3000);

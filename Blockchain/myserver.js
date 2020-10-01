var express = require('express');
var bodyParser = require('body-parser');
var app     = express();
var http = require('http');
var url = require('url');
var fs = require('fs');

app.use(bodyParser.urlencoded({ extended: true })); 

app.post('/myaction', function(req, res) {
  res.send("Thank you :)");
  var username = req.body.id;
  var passwd = req.body.pass;
  console.log("username: "+ username+", password: "+passwd+"\n")
  res.end();
});

app.listen(8081, function() {
  console.log('Server running at port 8081');
});

http.createServer(function (req, res) {
    var q = url.parse(req.url, true);
    var filename = "." + q.pathname;
    if(q.pathname=="/"){
      fs.readFile("./index.html", function(err, data) {
        if (err) {
          res.writeHead(404, {'Content-Type': 'text/html'});
          return res.end("404 Not Found");
        }
        res.writeHead(200, {'Content-Type': 'text/html'});
        res.write(data);
        return res.end();
      });
    } else {
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
    
  }).listen(8080);
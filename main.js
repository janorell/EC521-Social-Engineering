var http = require('http');

http.createServer(function (req, res) {
  res.writeHead(200, {'Content-Type': 'text/plain'});
  res.end('Site loading...');
  console.log("running on port 5000");
}).listen(5000);

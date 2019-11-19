# Code Readme

The code folder consists of the three files used to run our Autonomous Crawler:
- quest4.c
- udp_server_node.js
- client.html

'quest4.c' utilizes many libraries and drivers, particularly i2c (LIDARLite, Alphanumeric display), gpio, timer (PID control), mcpwm (steering and ESC control), uart (microLIDAR sensors), and pcnt (optical detector). 'quest4.c' uses UDP sockets to  communicate with the node server over port 8082. 'udp_server_node.js' utilizes socket.io to communicate with the client side html.

How to run locally:

1. Start node server:
node udp_server_node.js

2. Build ESP .c code using espressif build and make files, and then flash the code to the ESP32.

3. Open localhost:3333 for the client.html view

The publicly available site with dynamic DNS when the node server is running on our team's router is vschuweh.hopto.org:3001

Finally, testing code we used during our development process is included in folder "other_test_code".

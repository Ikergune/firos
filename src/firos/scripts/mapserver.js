// MIT License
//
// Copyright (c) <2015> <Ikergune, Etxetar>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files
// (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge,
// publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so,
// subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
// FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

var ROSLIB = require("roslib-socketio");
var HTTP_PORT = 10102;
var ROSBRIDGE_PORT = 9090;

var util = require('util');
util.debug = function(){};

if(process.argv.length > 2){
    HTTP_PORT = parseInt(process.argv[2], 10);
}

if(process.argv.length > 3){
    ROSBRIDGE_PORT = parseInt(process.argv[3], 10);
}

var app = require('http').createServer(function(){});
var io = require('socket.io')(app);
app.listen(HTTP_PORT);


console.log("Launched Map server at port " + HTTP_PORT);
RosBridge = {
    ros: null,
    connect: function(http){
        RosBridge.ros = new ROSLIB.Ros({
          url : 'ws://localhost:' + ROSBRIDGE_PORT,
          socketio: io
        });
    }
};

var exit = function(){
    console.log("\nClosing Map Server\n");
};


var connectionError = false;
process.on('uncaughtException', function(error){
    if((error.code === "ECONNREFUSED") || (error.code === "ECONNREFUSED") || (error.code === "ECONNRESET")){
        if(!connectionError){
            connectionError = true;
            console.error("Could not connect to Rosbridge, is it up?");
        }
    }else{
        console.error("System Error: ");
        console.error(error);
    }
});

process.on('exit', exit);

process.on('SIGTERM', exit);

process.on('SIGINT', function(){
    process.exit();
});

RosBridge.connect();
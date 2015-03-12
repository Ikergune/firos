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


console.log("Launched Map server");
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
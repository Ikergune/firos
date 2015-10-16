'use strict';

var zerorpc = require('zerorpc'),
lwm2mClient = require('lwm2m-node-lib').client;


function start(){
    var config;

    if (process.argv.length === 3) {
        config = require('./' + process.argv[2]);
    } else {
        config = require('./config');
    }

    var zerorpcServer = new zerorpc.Server({
        create: function(value, reply) {
            reply(null, 'OK');
        },
        set: function(value, reply) {
            reply(null, 'OK');
        },
        get: function(value, reply) {
            reply(null, 'OK');
        },
        remove: function(value, reply) {
            reply(null, 'OK');
        }
    });

    console.log('RUNNING SERVER on port ' + config.client.port);
    console.log('tcp://0.0.0.0:' + config.client.port);
    zerorpcServer.bind('tcp://0.0.0.0:' + config.client.port);

    lwm2mClient.register(
        config.lwm2m.host,
        config.lwm2m.port,
        config.lwm2m.url,
        config.lwm2m.endpointName,
        function() {
            // zerorpcServer.bind('tcp://0.0.0.0:' + config.client.port);
            console.log('COAP CLIENT STARTED');
        });
}


start();
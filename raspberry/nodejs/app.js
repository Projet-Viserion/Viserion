#!/usr/bin/node
let express = require('express');
let app = express();
let http = require('http').Server(app);
let io = require('socket.io')(http);
let exec = require('child_process').exec;


let can = require('socketcan');
let _ = require('c-struct');
let canStruct = require('./can-struct.js');
canStruct.init();

//initialiser la commnuincation avec le CAN via can0
let channel = can.createRawChannel("can0", true);

app.use(express.static(__dirname + '/dashboard'));


/*Servir Web*/
app.get('/', function(req, res) {
    res.sendFile('index.html');
});

http.listen(3000); // Lancement du serveur web
console.log("server is now running on port 3000");

let connections = [];

io.on('connection', handleConnection);

//traiter les commandes du serveur web
function handleConnection(client) {
    connections.push(client);
    console.log("Connected");
	client.on('MOVEFORWARD', moveForward);
    client.on('MOVEBACK', moveBack);
    client.on('MOVERIGHT', moveRight);
    client.on('MOVELEFT', moveLeft);
	client.on('MOVESTOP', stopMove);
	client.on('FRONTSTOP', frontStop);
    client.on('disconnect', (client) => {
        console.log("Disconned");
        let index = connections.indexOf(client);
        connections.splice(index,1);
    });
}


/*Gestion CAN
	- Recevoir messages via can0
	- Structurer ces messages en package en fonction de leur id
	- Appeller les fonctions correspondantes pour envoyer les informations au navigateur.
*/
function parseCanMsg(canMsg){
	if(canMsg.id===272){ // message us avant
		let obj = _.unpackSync('US_AV', canMsg.data);
		ultrason("AVG",(parseInt(obj.gauche * 0.02621)).toString());
    	ultrason("AVD",(parseInt(obj.droit * 0.02621)).toString());
    	ultrason("AVC",(parseInt(obj.centre * 0.02621)).toString());
	}	
	if(canMsg.id===256){ // message us arriere
		let obj = _.unpackSync('US_AR', canMsg.data);
		ultrason("ARG",(parseInt(obj.gauche * 0.02621)).toString());
    	ultrason("ARD",(parseInt(obj.droit * 0.02621)).toString());
    	ultrason("ARC",(parseInt(obj.centre * 0.02621)).toString());
	}
	if(canMsg.id===32){ // message distance
		let obj = _.unpackSync('DIST', canMsg.data);
		distanceR((obj.roue_droite/100).toString() + "cm");
		distanceL((obj.roue_gauche/100).toString() + "cm");
	}

	if(canMsg.id===48){ // message vitesse
		let obj = _.unpackSync('SPEED', canMsg.data);
		speedR(obj.roue_droite/100);	
		speedL(obj.roue_gauche/100);	
	}
	if(canMsg.id===80){ // message courant 
		let obj = _.unpackSync('CURR', canMsg.data);
		currentR(parseInt(obj.motor_current_r));
		currentL(parseInt(obj.motor_current_l));
		currentF(parseInt(obj.motor_current_f));
	}
	if(canMsg.id===16){ // message autre (battery volant capteur roue)
		let obj = _.unpackSync('OTHER', canMsg.data);
		battery(obj.battery);
		volant(144 - parseInt(obj.volant));
	}
}

//structure du message de CAN correspondante à sa fonction
_.register('US_AV', canStruct.paquet_us);
_.register('US_AR', canStruct.paquet_us);
_.register('DIST', canStruct.paquet_distance);
_.register('SPEED', canStruct.paquet_speed);
_.register('CURR', canStruct.paquet_current);
_.register('OTHER', canStruct.paquet_divers);



channel.addListener("onMessage", (msg)=> { 
	parseCanMsg(msg);
});


setInterval(() => { 
	exec("/opt/vc/bin/vcgencmd measure_temp", (result)=>{
		io.emit('tcpu', result);	
	});
}, 3000);


function ultrason(sensor, distance){
    io.emit(sensor,distance + ' cm');
}

function distance_parcouru(distance){
    io.emit("distance",distance + ' cm');
}

function battery(battery_percent){
    io.emit('battery', battery_percent);
}

function volant(volant_value){
    io.emit('volant', volant_value);
}

function currentL(current_value){
    io.emit('currentL', current_value);
}

function currentR(current_value){
    io.emit('currentR', current_value);
}

function currentF(current_value){
    io.emit('currentF', current_value);
}

function speedR(speed_value){
	io.emit('speedR', speed_value);
}

function speedL(speed_value){
	io.emit('speedL', speed_value);
}

function distanceR(distance){
	io.emit('distanceR', distance);
}

function distanceL(distance){
	io.emit('distanceL', distance);
}



moveForward = function(){
	channel.send(canStruct.av);
}

moveBack = function(){
	channel.send(canStruct.ar);
}

moveRight = function(){
	channel.send(canStruct.turnright);
}

moveLeft = function(){
	channel.send(canStruct.turnleft);
}

stopMove = function(){
	channel.send(canStruct.stop);	
	
}
frontStop = function(){
	channel.send(canStruct.turnstop);
}

channel.start();



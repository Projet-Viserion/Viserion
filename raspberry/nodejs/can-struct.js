let _ = require('c-struct');
const exec = require('child_process').execSync;


exports.init = function(){
	exec('sudo ip link set can0 down type can');

	result = exec('sudo ip link set can0 up type can bitrate 360000');
    if (result.toString()) {
        throw Error('CAN bus can\'t be initialized');
    }
}


exports.paquet_us = new _.Schema({
	gauche: _.type.uint16,
	centre: _.type.uint16,
	droit: _.type.uint16,

});

exports.paquet_distance = new _.Schema({
	roue_droite: _.type.uint16,
	roue_gauche: _.type.uint16,
});

exports.paquet_speed = new _.Schema({
	roue_droite: _.type.uint16,
	roue_gauche: _.type.uint16

});

exports.paquet_current = new _.Schema({
	motor_current_l: _.type.uint16,
	motor_current_r: _.type.uint16,
	motor_current_f: _.type.uint16,
});

exports.paquet_divers = new _.Schema({
	capteur_roue_d: _.type.uint8,
	capteur_roue_g: _.type.uint8,
	volant: _.type.uint8,
	battery:_.type.uint8,
});

exports.av = { id: 3,
     		ext: false,
            data: new Buffer([ 255,0]) 
};

exports.ar = { id: 3,
     		ext: false,
            data: new Buffer([0,255 ]) 
};

exports.stop = { id: 3,
     		ext: false,
            data: new Buffer([ 0,0 ]) 
};

exports.turnleft = { id: 1,
     		ext: false,
            data: new Buffer([ 1 ]) 
};

exports.turnright = { id: 1,
     		ext: false,
            data: new Buffer([255 ]) 
};

exports.turnstop = { id: 1,
     		ext: false,
            data: new Buffer([ 0 ]) 
};

exports.motorenable = { id: 2,
     		ext: false,
            data: new Buffer([ 1 ]) 
};

exports.motordisable = { id: 2,
     		ext: false,
            data: new Buffer([ 0 ]) 
};

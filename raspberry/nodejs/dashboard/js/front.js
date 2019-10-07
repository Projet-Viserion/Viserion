
$(document).ready(function() {
    let socket = io();
	// screen size 785 px * 480	
	let keysdown = {};
    $( "#button-enable" ).click(function() {
        socket.emit('EnableMotor', $('#motorenable').is(':checked'));
    });


    $( "#button-motor" ).click(function() {
        socket.emit('SetMotor', $('#motorprop').val());
    });


    $( "#button-volant" ).click(function() {
        socket.emit('SetMotor', $('#frontmotor').val());
    });


    socket.on("AVG",(msg)=>{
        $('#AVG').text(msg);
    });

    socket.on("AVC",(msg)=>{
        $('#AVC').text(msg);
    });

    socket.on("AVD",(msg)=>{
        $('#AVD').text(msg);
    });

    socket.on("ARG",(msg)=>{
        $('#ARG').text(msg);
    });

    socket.on("ARC",(msg)=>{
        $('#ARC').text(msg);
    });

    socket.on("ARD",(msg)=>{
        $('#ARD').text(msg);
    });


    socket.on("distance_parcouru",(msg)=>{
        $('distance').text("Distance : " + msg);
    });

    socket.on("battery",(msg)=>{
        jauge_bat.refresh(parseInt(msg));
    });

    socket.on("volant",(msg)=>{
        jauge_volant.refresh( parseInt(msg));
    });
    socket.on("currentR",(msg)=>{
        jauge_r.refresh( parseInt(msg));
    });

    socket.on("currentL",(msg)=>{
        jauge_l.refresh( parseInt(msg));
    });

    socket.on("currentF",(msg)=>{
        jauge_f.refresh(parseInt(msg));
    });
	
    socket.on("speedR",(msg)=>{
        jauge_speedD.refresh( parseInt(msg));
    });

	socket.on("speedL",(msg)=>{
        jauge_speedG.refresh( parseInt(msg));
    });

    socket.on("distanceR",(msg)=>{
        $('#distanceR').text(msg);
    });

    socket.on("distanceL",(msg)=>{
        $('#distanceL').text(msg);
    });

   socket.on("tcpu",(msg)=>{
        $('#tcpu').text(msg);
    });

    $(window).keydown(function(e) {
            if(keysdown[e.keyCode]) {
                return;
            }

            keysdown[e.keyCode] = true;



            switch(e.keyCode) {
                case 37: // left
                    socket.emit("MOVELEFT");                        // MOVELEFT MOVEFORWARD MOVEBACK MOVERIGHT STOP : Sortie
                    break;

                case 38: // up
                    socket.emit("MOVEFORWARD");
                    break;

                case 39: // right
                    socket.emit("MOVERIGHT");
                    break;

                case 40: // down
                    socket.emit("MOVEBACK");
                    break;

                default: return; // exit this handler for other keys
            }
            e.preventDefault(); // prevent the default action (scroll / move caret)

    });


    $(window).keyup(function(e) {

            switch(e.keyCode) {
                case 37: // left
					console.log("test1");
                    socket.emit("FRONTSTOP");
                    delete keysdown[e.keyCode];
                    break;

                case 38: // up
					console.log("test2");
                    socket.emit("MOVESTOP");
                    delete keysdown[e.keyCode];
                    break;

                case 39: // right
					console.log("test3");
                    socket.emit("FRONTSTOP");
                    delete keysdown[e.keyCode];
                    break;

                case 40: // down
					console.log("test4");
                    socket.emit("MOVESTOP");
                    delete keysdown[e.keyCode];
                    break;

                default:
                    return; // exit this handler for other keys
            }
        e.preventDefault(); // prevent the default action (scroll / move caret)
    });
});


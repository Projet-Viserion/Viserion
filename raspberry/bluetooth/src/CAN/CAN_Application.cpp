#include <signal.h>
#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#include "CAN/CAN_Application.h"

#include "Interface/CanInterface.h"

periode_t my_periode;

volatile uint32_t msTicks;
int control_can =0;

//Alarm wakes up with a alarm signal
 void alarmWakeup(int sig_num){
	 if(sig_num== SIGALRM){
		 msTicks=msTicks+1;
		 control_can=control_can+1;
		 CanCallback(msTicks);
	 }
 }

int getControlCan(){
	return control_can;
 }

 void init_my_can()
{
	subperiode_t sub1;
	{
		variable_paquet_t paquet1;
		paquet1.id = 16;
		paquet1.data = linkMotorsOrder();
		variable_paquet_t paquet2;
		paquet2.id = 17;
		paquet2.data = linkSteeringWheelOrder();

		sub1.variables[0] = paquet1;
		sub1.variables[1] = paquet2;
		sub1.nb_variables = 2;
	}

	subperiode_t sub2;
	{
		variable_paquet_t paquet1;
		paquet1.id = 16;
		paquet1.data = linkMotorsOrder();
		variable_paquet_t paquet2;
		paquet2.id = 17;
		paquet2.data = linkSteeringWheelOrder();

		sub2.variables[0] = paquet1;
		sub2.variables[1] = paquet2;
		sub2.nb_variables = 2;
	}

	my_periode.subperiodes[0] = sub1;
	my_periode.subperiodes[1] = sub2;
	my_periode.nb_subperiodes = 2;
	initCanPeriodic(50, &my_periode);
}

void launchCANServices (void)  {

	//Set value of variables contained in subperiod;
	//Default value are 0
	linkMotorsOrder()->intMessage[0]=0;
	linkMotorsOrder()->intMessage[1]=0;

	linkSteeringWheelOrder()->byteMessage[0]=0;

	init_my_can();

	signal(SIGALRM, alarmWakeup);
	ualarm(1000,1000);//First offset, Second period, in µs

	canSubscribe(0,linkUSFrontBack());
	canSubscribe(1, linkUSLeft());
	canSubscribe(2, linkUSRight());
	canSubscribe(0x102, linkSpeedWheelsLR());
	canSubscribe(257, linkPosSteeringWheel());
	canSubscribe(256, linkPosWheelsLR());

	canSubscribe(0x200, linkBattery());

	canInit();

  while (1) {
	  usleep(1000);
	runCanPeriodic();

	if(receiveMessage()==0){
		control_can=0;
	}

  }
}

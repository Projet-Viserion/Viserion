#include "Interface/CanInterface.h"
#include "Interface/ImageInterface.h"
#include <iostream>

//Information concerning the car
data_paquet_t posSteeringWheel;
data_paquet_t posWheelsLR;
data_paquet_t speedWheelsLR;
data_paquet_t USFrontBack;
data_paquet_t USLeft;
data_paquet_t USRight;
data_paquet_t motorsOrder;
data_paquet_t steeringWheelOrder;
data_paquet_t battery;

void initCarValue(){
	posSteeringWheel.floatMessage[0]=0.0;
	posSteeringWheel.floatMessage[1]=0.0;
	posWheelsLR.floatMessage[0]=0.0;
	posWheelsLR.floatMessage[1]=0.0;
	USFrontBack.floatMessage[0]=1000.0;
	USFrontBack.floatMessage[1]=1000.0;
	USLeft.floatMessage[0]=1000.0;
	USLeft.floatMessage[1]=1000.0;
	USRight.floatMessage[0]=1000.0;
	USRight.floatMessage[1]=1000.0;
}

data_paquet_t* linkPosSteeringWheel(){
	return &posSteeringWheel;
}

data_paquet_t* linkPosWheelsLR(){
	return &posWheelsLR;
}

data_paquet_t* linkSpeedWheelsLR(){
	return &speedWheelsLR;
}

data_paquet_t* linkUSFrontBack(){
	return &USFrontBack;
}

data_paquet_t* linkUSLeft(){
	return &USLeft;
}

data_paquet_t* linkUSRight(){
	return &USRight;
}

data_paquet_t* linkMotorsOrder(){

	//std::cout << motorsOrder.intMessage[0] << std::endl;
	//if ((*(linkCameraSpeedLimit()) * 10) < motorsOrder.intMessage[0]){
	//	motorsOrder.intMessage[0] = *(linkCameraSpeedLimit()) * 10;
	//	motorsOrder.intMessage[1] = *(linkCameraSpeedLimit()) * 10 ;
	//}
	return &motorsOrder;
}

data_paquet_t* linkSteeringWheelOrder(){
	return &steeringWheelOrder;
}
data_paquet_t* linkBattery(){
	return &battery;
}

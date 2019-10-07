#ifndef INTERFACES_RASP_H
#define INTERFACES_RASP_H

#include "CAN/CAN_Abstraction.h"

void initCarValue();

data_paquet_t* linkPosSteeringWheel();

data_paquet_t* linkPosWheelsLR();

data_paquet_t* linkSpeedWheelsLR();

data_paquet_t* linkUSFrontBack();

data_paquet_t* linkUSLeft();

data_paquet_t* linkUSRight();

data_paquet_t* linkMotorsOrder();

data_paquet_t* linkSteeringWheelOrder();

data_paquet_t* linkBattery();

#endif

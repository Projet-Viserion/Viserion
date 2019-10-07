#ifndef CAN_PERIODIC_H
#define CAN_PERIODIC_H

#include "CAN/CAN_Abstraction.h"

/*
Second abstraction concerning CAN bus
This abstraction gives some functions to manipulate periodic operating mode on CAN bus
*/

//Similar to a CAN frame, but the data is given thanks to a pointer
typedef struct
{
	data_paquet_t* data;
	int16_t id;
} variable_paquet_t;

//Represents a subperiode with variables which must be sent during this subperiode
typedef struct
{
	variable_paquet_t variables[10];
	uint16_t nb_variables;
}subperiode_t;

//Represents the global periode with its subperiode which must be sent during the global periode
typedef struct
{
	subperiode_t subperiodes[10];
	uint16_t nb_subperiodes;
}periode_t;

//Activates the periodicity, the duration of the subperiode (in ms) is given in argument
//Can not replace a CAN_Init
void initCanPeriodic(uint16_t periode_ms, periode_t* periodes);

//Interruption, must be called into the timer callback
void CanCallback(uint64_t time_ms);

//Must be called in the main loop of the program
void runCanPeriodic(void);

#endif

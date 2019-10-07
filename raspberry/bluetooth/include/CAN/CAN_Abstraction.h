#ifndef __CAN_ABSTRACTION_H
#define __CAN_ABSTRACTION_H

#include <stdint.h>

/*
First abstraction concerning CAN API
This abstraction gives some functions to easily manipulate CAN bus
*/

//Represents data under different formats
typedef union 
{
	char stringMessage[8];
	int8_t byteMessage[8];
	int16_t intMessage[4];
	float floatMessage[2];
}data_paquet_t;

//Represents a CAN frame with data and its id
typedef struct{
	data_paquet_t data;
	int16_t id;
}can_paquet_t;

//Subscribes to an id and set the memory place for the data
void canSubscribe(int id, data_paquet_t* data);

//Inits CAN bus
//Subscriptions must be done before this function
void canInit(void);

//Generic functions for sending and receiving messages 
int sendMessage(int16_t id, data_paquet_t data);
int receiveMessage(void);

#endif
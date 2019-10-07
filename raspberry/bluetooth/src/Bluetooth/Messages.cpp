#include "Bluetooth/Messages.h"
#include <iostream>

Messages::Messages(char l, char cid, char main_id, int32_t val, int32_t val2){
	level=l;
	complementaryID=cid;
	id=main_id;
	value=val;
	value2=val2;
}

Messages::~Messages(){}

//display message to console
void Messages::display(){
	switch(level){
		case 0x00 :
			std::cout<<"cmd"<<std::endl;
			break;
		default :
			std::cout<<"other"<<std::endl;
			break;
	}

	switch(complementaryID){
		case 0x00 :
			std::cout<<"UserCmd"<<std::endl;
			break;
		default :
			std::cout<<"other"<<std::endl;
			break;
	}

	switch(id)
	{
		case 0x08 :
			std::cout<<"front"<<std::endl;
			break;
		case 0x04 :
			std::cout<<"back"<<std::endl;
			break;
		case 0x02 :
			std::cout<<"left"<<std::endl;
			break;
		case 0x01 :
			std::cout<<"right"<<std::endl;
			break;
	}

	std::cout<<value<<std::endl;
}

//STATIC FUNCTIONS
//From a string, create a Messages object
Messages Messages::decode(std::string receivedBytes)
{
	char level = (receivedBytes[0]&0xC0) >>6;

	char complementaryId = (receivedBytes[0]&0x30)>>4;

	char id = (receivedBytes[0]&0x0F);

	int32_t value = (int)(receivedBytes[1]);

	Messages msg = Messages(level,complementaryId,id,value,0);
	return msg;
}

//From a Messages object, create a string
std::string Messages::encode(Messages msg)
{
	std::string sentBytes;

	char level_offset = ((msg.getLevel()&0x03)<<6);
	char complementaryId_offset = ((msg.getComplementaryID()&0x03)<<4);
	char id_offset = msg.getId()&0x0F;
	char begin = level_offset|complementaryId_offset|id_offset;


	std::string sentMessage;

	if(msg.getLevel()==0x00)
	{
		if(msg.getComplementaryID() == 0x00)
		{
			switch(msg.getId())
			{
				case 7 :
				sentMessage+=begin;
				sentMessage+=(char)(msg.getValue()&0x000000FF);
				sentMessage+='\n';
				break;

				case 6 :
				sentMessage.resize(3);
				sentMessage[0]=begin;
				sentMessage[1]=(char)(msg.getValue()&0x000000FF);
				sentMessage[2]='\n';
				break;

				case 1 :
				case 2 :
				case 3 :
				case 4 :
				case 5 :
				sentMessage.resize(10);
				sentMessage[0]=begin;
				sentMessage[1]=(char)(msg.getValue()&0x000000FF);
				sentMessage[2]=(char)((msg.getValue()&0x0000FF00)>>8);
				sentMessage[3]=(char)((msg.getValue()&0x00FF0000)>>16);
				sentMessage[4]=(char)((msg.getValue()&0xFF000000)>>24);
				sentMessage[5]=(char)(msg.getValue2()&0x000000FF);
				sentMessage[6]=(char)((msg.getValue2()&0x0000FF00)>>8);
				sentMessage[7]=(char)((msg.getValue2()&0x00FF0000)>>16);
				sentMessage[8]=(char)((msg.getValue2()&0xFF000000)>>24);
				sentMessage[9]='\n';
				break;

				default :
				sentMessage.resize(2);
				sentMessage[0]=begin;
				sentMessage[1]='\n';
				break;
			}
		}else if(msg.getComplementaryID()==0x01){
			sentMessage.resize(4);
			sentMessage[0]=begin;
			sentMessage[1]=(char)(msg.getValue()&0x000000FF);
			sentMessage[2]=(char)(msg.getValue2()&0x000000FF);
			sentMessage[3]='\n';
		}
		else if(msg.getComplementaryID() == 0x02)
		{
			switch(msg.getId())
			{
				case 0 :
				sentMessage.resize(2);
				sentMessage[0]=begin;
				sentMessage[1]='\n';
				break;

				case 1 :
				case 2 :
				sentMessage.resize(10);
				sentMessage[0]=begin;
				sentMessage[1]=(char)(msg.getValue()&0x000000FF);
				sentMessage[2]=(char)((msg.getValue()&0x0000FF00)>>8);
				sentMessage[3]=(char)((msg.getValue()&0x00FF0000)>>16);
				sentMessage[4]=(char)((msg.getValue()&0xFF000000)>>24);
				sentMessage[5]=(char)(msg.getValue2()&0x000000FF);
				sentMessage[6]=(char)((msg.getValue2()&0x0000FF00)>>8);
				sentMessage[7]=(char)((msg.getValue2()&0x00FF0000)>>16);
				sentMessage[8]=(char)((msg.getValue2()&0xFF000000)>>24);
				sentMessage[9]='\n';
				break;

				case 3:
				sentMessage.resize(3);
				sentMessage[0]=begin;
				sentMessage[1]=(char)(msg.getValue()&0x000000FF);
				sentMessage[2]='\n';
				break;

				case 4:
				sentMessage.resize(3);
				sentMessage[0]=begin;
				sentMessage[1]=(char)(msg.getValue()&0x000000FF);
				sentMessage[2]='\n';
				break;

				default :
				sentMessage.resize(2);
				sentMessage[0]=begin;
				sentMessage[1]='\n';
				break;
			}
		}
	}

	if(msg.getLevel()==0x03)
	{
		sentMessage.resize(2);
		sentMessage[0]=begin;
		sentMessage[1]='\n';
	}

	return sentMessage;
}

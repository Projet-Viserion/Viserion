#include <iostream>
#include <thread>
#include <string>
#include <stdlib.h>

#include "Interface/CanInterface.h"
#include "CAN/CAN_Application.h"
#include "CAN/CAN_Abstraction.h"
#include "Bluetooth/BluetoothCommunication.h"
#include "Bluetooth/Messages.h"

int can_Ok;

//Receives messages from the computer application and puts them into a list
void receptionBluetooth(BluetoothServer *bt)
{
	while (bt->isConnected())
	{
		usleep(1000);
		std::string receivedMsg;
		receivedMsg = bt->receiveMsg();
		//Added at the end of the list
		bt->receptionBuffer.push_back(receivedMsg);
	}
}

//Sends information to the computer application
void sendBluetooth(BluetoothServer *bt)
{
	while (bt->isConnected())
	{
		usleep(400000);
		if (can_Ok == 1)
		{
			//About the sterring wheel
			int32_t value = (int32_t) linkPosSteeringWheel()->byteMessage[0];
			Messages msg_steering_wheel = Messages(0x00, 0x00, 0x07, value, 0);
			bt->sendMsg(Messages::encode(msg_steering_wheel));

			//About the Front and Back US
			int32_t USB_offset = (int32_t) ((linkUSFrontBack()->intMessage[1]
					<< 16) & 0xFFFF0000)
					| ((int32_t) linkUSFrontBack()->intMessage[0] & 0x0000FFFF);
			int32_t USF_offset = (int32_t) ((linkUSFrontBack()->intMessage[3]
					<< 16) & 0xFFFF0000)
					| ((int32_t) linkUSFrontBack()->intMessage[2] & 0x0000FFFF);

			Messages msg_us_front_back = Messages(0x00, 0x00, 0x03, USF_offset,
					USB_offset);
			bt->sendMsg(Messages::encode(msg_us_front_back));

			//About the Right US
			int32_t USR1_offset =
					(int32_t) ((linkUSRight()->intMessage[1] << 16) & 0xFFFF0000)
							| ((int32_t) linkUSRight()->intMessage[0]
									& 0x0000FFFF);
			int32_t USR2_offset =
					(int32_t) ((linkUSRight()->intMessage[3] << 16) & 0xFFFF0000)
							| ((int32_t) linkUSRight()->intMessage[2]
									& 0x0000FFFF);

			Messages msg_us_right = Messages(0x00, 0x00, 0x05, USR2_offset,
					USR1_offset);
			bt->sendMsg(Messages::encode(msg_us_right));

			//About the Left US
			int32_t USL1_offset = (int32_t) ((linkUSLeft()->intMessage[1] << 16)
					& 0xFFFF0000)
					| ((int32_t) linkUSLeft()->intMessage[0] & 0x0000FFFF);
			int32_t USL2_offset = (int32_t) ((linkUSLeft()->intMessage[3] << 16)
					& 0xFFFF0000)
					| ((int32_t) linkUSLeft()->intMessage[2] & 0x0000FFFF);

			Messages msg_us_left = Messages(0x00, 0x00, 0x04, USL2_offset,
					USL1_offset);
			bt->sendMsg(Messages::encode(msg_us_left));

			//About the position of the wheels
			int32_t posWheelL = (int32_t) ((linkPosWheelsLR()->intMessage[1]
					<< 16) & 0xFFFF0000)
					| ((int32_t) linkPosWheelsLR()->intMessage[0] & 0x0000FFFF);
			int32_t posWheelR = (int32_t) ((linkPosWheelsLR()->intMessage[3]
					<< 16) & 0xFFFF0000)
					| ((int32_t) linkPosWheelsLR()->intMessage[2] & 0x0000FFFF);

			Messages msg_pos_wheels = Messages(0x00, 0x00, 0x01, posWheelL,
					posWheelR);
			bt->sendMsg(Messages::encode(msg_pos_wheels));

			//About the speed of the car
			int32_t speedValueL =
					(int32_t) ((linkSpeedWheelsLR()->intMessage[1] << 16)
							& 0xFFFF0000)
							| ((int32_t) linkSpeedWheelsLR()->intMessage[0]
									& 0x0000FFFF);
			int32_t speedValueR =
					(int32_t) ((linkSpeedWheelsLR()->intMessage[3] << 16)
							& 0xFFFF0000)
							| ((int32_t) linkSpeedWheelsLR()->intMessage[2]
									& 0x0000FFFF);
			Messages msg_speed = Messages(0x00, 0x00, 0x02, speedValueL,
					speedValueR);
			bt->sendMsg(Messages::encode(msg_speed));

			//About the battery level
			int32_t valueBat = (int32_t) linkBattery()->byteMessage[0];
			Messages msg_battery = Messages(0x00, 0x00, 0x06, valueBat, 0);
			bt->sendMsg(Messages::encode(msg_battery));
		}
	}
}

//Detects if the CAN is not alive
void controlCan(BluetoothServer *bt)
{
	int errorCanAlreadySent = 0;
	int canRdyAlreadySent = 1;
	while (1)
	{
		usleep(50000);
		if (getControlCan() > 100) //If we do not receive something for 100ms, the CAN bus is considered as lost
		{
			if (errorCanAlreadySent == 0)
			{
				Messages msg_Can_error = Messages(0x03, 0x00, 0x01, 0, 0);
				bt->sendMsg(Messages::encode(msg_Can_error));
				can_Ok = 0;
				errorCanAlreadySent = 1;
				canRdyAlreadySent = 0;
			}
		}
		else
		{
			if (canRdyAlreadySent == 0)
			{
				can_Ok = 1;
				errorCanAlreadySent = 0;
				canRdyAlreadySent = 1;
			}
		}
	}
}

//Treats received messages thanks to Bluetooth from the list
void handler(BluetoothServer *bt)
{
	while (bt->isConnected())
	{
		if (!bt->receptionBuffer.empty())
		{
			std::string msg = bt->receptionBuffer.front();
			bt->receptionBuffer.pop_front();
			Messages decoded_msg = Messages::decode(msg);

			if (decoded_msg.getLevel() == 0x00
					&& decoded_msg.getComplementaryID() == 0x00)
			{
				char id = decoded_msg.getId();
				if (id == 0x08)
				{
					int32_t speedF = decoded_msg.getValue();
					linkMotorsOrder()->intMessage[0] = (int16_t) speedF * 10;
					linkMotorsOrder()->intMessage[1] = (int16_t) speedF * 10;
				}
				else if (id == 0x04)
				{
					int32_t speedB = decoded_msg.getValue();
					linkMotorsOrder()->intMessage[0] = (int16_t) speedB * -10;
					linkMotorsOrder()->intMessage[1] = (int16_t) speedB * -10;
				}
				else if (id == 0x02)
				{
					int32_t angleL = decoded_msg.getValue();
					linkSteeringWheelOrder()->byteMessage[0] = (int8_t) angleL;
				}
				else if (id == 0x01)
				{
					int32_t angleR = decoded_msg.getValue();
					linkSteeringWheelOrder()->byteMessage[0] = -1
							* (int8_t) angleR;
				}
				msg.pop_back();
			}
		}
		else
			usleep(50000);
	}
	//If the Bluetooth server is disconnect, the Bluetooth communication is considered as lost
	std::cout << "Loss Bluetooth" << std::endl;
	linkMotorsOrder()->intMessage[0] = 0;
	linkMotorsOrder()->intMessage[1] = 0;
	linkSteeringWheelOrder()->byteMessage[0] = 0;
}

int main()
{
	//Start Bluetooth server and acceptance of the first connection
	BluetoothServer btServer;
	std::cout << "The bluetooth server has been declared" << std::endl;
	btServer.acceptConnection();
	std::cout << "The connection has been accepted" << std::endl;

	//Start a pool of threads

	//Bluetooth threads
	std::thread bluetoothReception(receptionBluetooth, &btServer);
	std::thread bluetoothSending(sendBluetooth, &btServer);
	std::thread messagesTreatment(handler, &btServer);
	std::cout << "Bluetooth threads are running\n";

	initCarValue();

	//Can threads
	can_Ok = 1;
	std::thread canWatchDog(controlCan, &btServer);
	std::thread canStart(launchCANServices);
	std::cout << "CAN services are running\n";

	//Synchronize threads:
	bluetoothReception.join();
	bluetoothSending.join();
	messagesTreatment.join();
	canWatchDog.join();
	canStart.join();
	return 0;
}

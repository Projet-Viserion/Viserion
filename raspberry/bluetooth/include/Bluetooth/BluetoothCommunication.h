#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <string>
#include <list>

#ifndef __BLUETOOTH_COMM__H
#define __BLUETOOTH_COMM__H

class BluetoothServer{
	public:
		BluetoothServer();
		~BluetoothServer();
		int acceptConnection();
		void sendMsg(const std::string & msg);
		std::string receiveMsg();
		int isConnected();

		// Tests
		std::list<std::string> receptionBuffer;
		std::list<std::string> tranmissionBuffer;
		
	private:
		int mainSocket;
		int clientSocket;
		int connected = 0;
		struct sockaddr_rc loc_addr = { 0 }, rem_addr = { 0 };
		socklen_t opt = sizeof(rem_addr);
};

#endif

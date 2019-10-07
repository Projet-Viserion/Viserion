#include <iostream>
#include "Bluetooth/BluetoothCommunication.h"

BluetoothServer::BluetoothServer(void)
{
	clientSocket = -1;
	mainSocket = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
	loc_addr.rc_family = AF_BLUETOOTH;
	loc_addr.rc_bdaddr = {0, 0, 0, 0, 0, 0};
	loc_addr.rc_channel = (uint8_t) 1;
	bind(mainSocket, (struct sockaddr *)&loc_addr, sizeof(loc_addr));
	//Maybe the listen should be placed into the acceptConnection()
	listen(mainSocket, 1);
}

BluetoothServer::~BluetoothServer(void){
  close(clientSocket);
  close(mainSocket);
}

int BluetoothServer::acceptConnection()
{
  clientSocket=accept(mainSocket,(struct sockaddr *)&rem_addr, &opt );
  connected = 1 ;
  std::cout << "Connection accepted" << std::endl;
  return connected;
}

void BluetoothServer::sendMsg(const std::string & msg){
  int bytes_send;
  bytes_send = write(clientSocket, msg.c_str() , msg.size());
  if (bytes_send < 0)
    connected = 0;

  //std::cout << bytes_send << " bytes have been sent"<< std::endl;
}

std::string BluetoothServer::receiveMsg(){
  int bytes_read;
  char buf[1024] = { 0 };

  bytes_read = read(clientSocket, buf, sizeof(buf));
  return buf;
}

int BluetoothServer::isConnected(){
  return connected;
}
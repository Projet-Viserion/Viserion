CXX = g++
CXXFLAGS = -Wall -g
PTHREAD = -pthread
INC = ../include

main : main.cpp CAN/CAN_Abstraction.cpp CAN/CAN_Application.cpp CAN/CAN_Periodic.cpp Bluetooth/BluetoothCommunication.cpp Bluetooth/Messages.cpp Interface/CanInterface.cpp Interface/ImageInterface.cpp
	$(CXX) $(CXXFLAGS) $(PTHREAD) -I$(INC) -o main main.cpp CAN/CAN_Abstraction.cpp CAN/CAN_Application.cpp CAN/CAN_Periodic.cpp Bluetooth/BluetoothCommunication.cpp Bluetooth/Messages.cpp Interface/CanInterface.cpp Interface/ImageInterface.cpp 

clean:
	rm -f *.o

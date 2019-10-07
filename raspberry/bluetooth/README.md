# Bluetooth and CAN communication on RasPi 3.0

:warning: **This code was produced for an older version of the CAN messages. The project is not functional with the current messages..**

# Prerequisites

## Install Bluetooth library

```
sudo apt-get install libbluetooth-dev
```

## Initialize CAN interface
Initialize CAN interface by following the following instruction from the 
[PICAN site].(https://copperhilltech.com/pican2-controller-area-network-can-interface-for-raspberry-pi/)


# Usage
To establish Bluetooth communication:

```
cd ~/bluetooth/src/
make
./main
```
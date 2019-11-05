# Overview of the software architecture


> [!IMPORTANT]
> When launching the DEBUG or LOAD of the STM32 card (Nucleo), it is necessary to manually press and hold the START button (ST-LINK imposes a reset that cuts off the latch circuit).

## Embedded code
Each ECU has its own embedded code. Different tools were used to produced code and to compile. 

### Nucleo

The code for the Nucleo board is produced with **CubeMx** and **Keil**. The Keil project can be found in [geicar/nucleo/MDK-ARM/](../../../geicar/nucleo/MDK-ARM/).

### Discovery
The code for Discovery board is produced for the IDE [Atollic TrueSTUDIO](https://atollic.com/truestudio/) for STM32. This IDE was choosen to avoid the Keil limitation of 32Kb.

### Raspberry
The code on the Raspberry is based on the distribution **Raspbian**.

## Networks
Communications between ECUs are provided through a **CAN bus**. Communication to external systems can be done via **Bluetooth** or **WiFi**. A Bluetooth library is available as well as a server based on **nodesjs**.
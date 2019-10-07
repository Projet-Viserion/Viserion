#ifndef CAN_APPLICATION_H
#define CAN_APPLICATION_H

#include "CAN/CAN_Abstraction.h" /*Our Abstration*/
#include "CAN/CAN_Periodic.h"

//Contains functions with the highest level about the CAN

//Launch the CAN communication
void launchCANServices (void);

//Getter
int getControlCan(void);

#endif

#ifndef _STERRING_H_
#define _STERRING_H_

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"


/**
*	Set the max and min value of the steering wheels sensor
**/
void steering_Init(void);

/**
* Set the speed of the steering motor. Speed value has to be between 0 and 100
**/
void steering_set_speed(GPIO_PinState en_steering, int speed);

/**
* Return the steering angle.
**/
int steering_get_angle(void);

/**
* Command the front wheel position
**/
void steering_set_position (GPIO_PinState en_steering, int msg_CAN);

/**
 * Cmd wheels with buttons
 */
void steering_move_with_button(void);

/**
* Return 1 if a steering control button is pressed
*/
int steering_is_a_button_pressed(void);


#endif


/* Includes ------------------------------------------------------------------*/

#include "steering.h"
#include "stdlib.h"
#include "math.h"


/* Private define ------------------------------------------------------------*/

#define LEFT_MAX_SPEED_STEERING 10 // Left steering max speed
#define RIGHT_MAX_SPEED_STEERING 90 // Right steering max speed
#define LEFT_MED_SPEED_STEERING 65 // Left steering medium speed
#define RIGHT_MED_SPEED_STEERING 35 // Right steering medium speed
#define NO_STEERING 50

#define centre_volant (gauche_volant + droite_volant)/2 // valeur initiale = 2110

#define LEFT_CMD_STEERING 60 // Left steering command
#define RIGHT_CMD_STEERING 40 // Right steering command
#define CENTER_CMD_STEERING (LEFT_CMD_STEERING + RIGHT_CMD_STEERING)/2

#define SPEED_STEP_1 30
#define SPEED_STEP_2 150

#define pin_bt_right_front_wheel GPIO_PIN_14
#define pin_bt_left_front_wheel GPIO_PIN_15

/* Private variables ---------------------------------------------------------*/

extern uint32_t ADCBUF[5];
int droite_volant, gauche_volant;



/* Programs ------------------------------------------------------------------*/

void steering_Init (void){
    steering_set_speed(GPIO_PIN_SET, RIGHT_MAX_SPEED_STEERING);
    HAL_Delay(2000);
    droite_volant = steering_get_angle();
    
    steering_set_speed(GPIO_PIN_SET, LEFT_MAX_SPEED_STEERING);
    HAL_Delay(2000);
    gauche_volant = steering_get_angle();
    
    steering_set_position(GPIO_PIN_SET, centre_volant);
}

void steering_set_speed(GPIO_PinState en_steering, int speed){
    
    /* Threshold rotating speed of steering wheels*/
    if (speed > RIGHT_MAX_SPEED_STEERING){
        speed = RIGHT_MAX_SPEED_STEERING;
    } else if (speed < LEFT_MAX_SPEED_STEERING){
        speed  = LEFT_MAX_SPEED_STEERING;
    }
    
    speed = 3200 * ( speed/ 100.0 );
    TIM1->CCR3 = speed;
    
    HAL_GPIO_WritePin( GPIOC, GPIO_PIN_12, en_steering);  //PC12  AV
}

int steering_get_angle(void){
    return ADCBUF[1];
}

void steering_set_position (GPIO_PinState en_steering, int msg_CAN){
    
    int cpt_pos = steering_get_angle();
    int cpt_centre, msg_corr, angle_diff;
    
    // Correction messages
    if (msg_CAN > 100){
        msg_CAN = 100;
        
    } // limit the CAN msg to [0,100]
    msg_corr = 6*(msg_CAN - CENTER_CMD_STEERING);
    cpt_centre = cpt_pos - centre_volant;
    angle_diff = msg_corr - cpt_centre;
    
    // Discrete command - steady/turning slow/turning
    if (((abs(cpt_pos - gauche_volant) <SPEED_STEP_1) && (msg_corr > cpt_pos)) | ((abs(cpt_pos - droite_volant) <SPEED_STEP_1)) && (msg_corr > cpt_pos))
    {
        steering_set_speed(GPIO_PIN_RESET, NO_STEERING);
    }
    else
    {
        if (abs(angle_diff)<SPEED_STEP_1){steering_set_speed(GPIO_PIN_RESET, NO_STEERING);} // steering wheels close to objective
        else if (abs(angle_diff)<SPEED_STEP_2) // steering wheels approaching objective
        {
            if (angle_diff > 0){steering_set_speed(en_steering, LEFT_MED_SPEED_STEERING);}
            else {steering_set_speed(en_steering, RIGHT_MED_SPEED_STEERING);}
        }
        else if (angle_diff > 0){steering_set_speed(en_steering, LEFT_MAX_SPEED_STEERING);} // steering wheels far to the left
        else {steering_set_speed(en_steering, RIGHT_MAX_SPEED_STEERING);}	// steering wheels far to the right
    }
}

int steering_is_a_button_pressed(){
	return ((!HAL_GPIO_ReadPin(GPIOB, pin_bt_right_front_wheel)) || (!HAL_GPIO_ReadPin(GPIOB, pin_bt_left_front_wheel)));
}	

void steering_move_with_button(void){
    static int previous_value_right = GPIO_PIN_RESET;
    static int previous_value_left = GPIO_PIN_RESET;
    int current_value_right = !HAL_GPIO_ReadPin(GPIOB, pin_bt_right_front_wheel);
    int current_value_left = !HAL_GPIO_ReadPin(GPIOB, pin_bt_left_front_wheel);
    
    if (
				((current_value_right == GPIO_PIN_SET) && (current_value_left == GPIO_PIN_SET))
        || ((current_value_right == GPIO_PIN_RESET) && (previous_value_right == GPIO_PIN_SET))
        || ((current_value_left == GPIO_PIN_RESET) && (previous_value_left == GPIO_PIN_SET))
				){
        steering_set_speed(GPIO_PIN_RESET, NO_STEERING);
        previous_value_right = GPIO_PIN_RESET;
        previous_value_left = GPIO_PIN_RESET;
    } else if ((current_value_right == GPIO_PIN_SET) && (previous_value_right == GPIO_PIN_RESET)){
        steering_set_speed(GPIO_PIN_SET, RIGHT_MAX_SPEED_STEERING);
        previous_value_right = GPIO_PIN_SET;
    } else if ((current_value_left == GPIO_PIN_SET) && (previous_value_left == GPIO_PIN_RESET)){
           steering_set_speed(GPIO_PIN_SET, LEFT_MAX_SPEED_STEERING);
           previous_value_left = GPIO_PIN_SET;
    }
}

/*
 * servo.h
 *
 *  Created on: Nov 19, 2023
 *      Author: valen
 *
 *
 */

#ifndef SRC_SERVO_H_
#define SRC_SERVO_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include "main.h"


//Upper and lower Pulse limit for 50Hz/20ms with Counterperiod at 2000
#define LOWER_PULSE_LIMIT 50
#define UPPER_PULSE_LIMIT 250

/* Speed of Servo using HAL_Delay
 * Slow
 * Normal
 * Fast
 * */
typedef enum{
	SERVO_SPEED_SLOW = 5,
	SERVO_SPEED_NORMAL = 10,
	SERVO_SPEED_FAST = 15
}SERVO_Speed;


//Similar to "HAL_TIM_PWM_Start"
HAL_StatusTypeDef servo_start_init(TIM_HandleTypeDef *htim, uint32_t Channel);


void servo_reset();


void servo_demo();


#endif /* SRC_SERVO_H_ */

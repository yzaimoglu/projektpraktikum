/*
 * servo.c
 *
 *  Created on: Nov 19, 2023
 *      Author: valen
 */

#include "servo.h"


static TIM_HandleTypeDef *timhandle;
static uint32_t channel;
static uint32_t servospeed;




static void set_Pulse(uint32_t pulse)
{
	if(pulse <= UPPER_PULSE_LIMIT && pulse >= LOWER_PULSE_LIMIT)
	{
		__HAL_TIM_SET_COMPARE(timhandle,channel,pulse);
	}
}

static void set_Servospeed(SERVO_Speed speed)
{
	servospeed = speed;
}


HAL_StatusTypeDef servo_start_init(TIM_HandleTypeDef *htim, uint32_t Channel)
{
	timhandle = htim;
	channel = Channel;
	set_Servospeed(SERVO_SPEED_NORMAL);
	return HAL_TIM_PWM_Start(htim, Channel);
}


void servo_reset()
{
	set_Pulse(LOWER_PULSE_LIMIT);
	HAL_Delay(200);
}


void servo_demo()
{
	set_Servospeed(SERVO_SPEED_FAST);
	for(uint32_t pulse = 50; pulse < 250; pulse ++){
		set_Pulse(pulse);
		HAL_Delay(servospeed);

	}
	set_Servospeed(SERVO_SPEED_SLOW);
	for(uint32_t pulse = 250; pulse > 50; pulse --){
		set_Pulse(pulse);
		HAL_Delay(servospeed);

	}
}


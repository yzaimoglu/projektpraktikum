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
static uint32_t currentpulse = LOWER_PULSE_LIMIT;



static void set_Pulse(uint32_t pulse)
{
	if(pulse <= UPPER_PULSE_LIMIT && pulse >= LOWER_PULSE_LIMIT)
	{
		currentpulse = pulse;
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
	set_Pulse(150);
	HAL_Delay(200);
}


uint16_t servo_rot(int16_t sensorval)
{
	uint32_t abssens = abs(sensorval);
	if(abssens <= CHANGE_THRESHOLD) return currentpulse;

	int8_t i = (sensorval > 0) ? 1: -1;

	// Checks if pulse is in allowed range (0.5-2.5ms)
	if(currentpulse + i <= UPPER_PULSE_LIMIT && currentpulse + i >= LOWER_PULSE_LIMIT){
		__HAL_TIM_SET_COMPARE(timhandle,channel,currentpulse);
	}

	// HAL_Delay((4095/abssens)+4);
    HAL_Delay(10);
	return currentpulse;
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

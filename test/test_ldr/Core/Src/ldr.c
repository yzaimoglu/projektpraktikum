#include "ldr.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

void ldr_init(ADC_HandleTypeDef type) {
	HAL_ADC_Start(&type);
}
void ldr_stop(ADC_HandleTypeDef type) {
	HAL_ADC_Stop(&type);
}

uint32_t ldr_get_value(ADC_HandleTypeDef type, uint32_t delay_ms) {
	uint32_t aValue;
	ldr_init(type);
	if (HAL_ADC_PollForConversion(&type, 1000) == HAL_OK) {
		aValue = HAL_ADC_GetValue(&type);
	} else {
		return ERROR_NUM;
	}
	ldr_stop(type);
	HAL_Delay(delay_ms);
	return aValue;
}

uint8_t ldr_get_percent(ADC_HandleTypeDef type, uint32_t delay_ms) {
	// 12bit
	// 0 = 100%
	// 4095 = 0%
	uint32_t aValue = ldr_get_value(type, delay_ms);
	if(aValue == ERROR_NUM) {
		return ERROR_PERC;
	}
	uint8_t reverse_perc = ((float) (aValue / 4095.0)) * 100;
	uint8_t perc = 100 - reverse_perc;
	return perc;
}

void ldr_uart_transmit(ADC_HandleTypeDef type, UART_HandleTypeDef uart_type) {
	char buf[50];
	uint32_t value = ldr_get_value(type, 1000);
	sprintf(buf, "ADC Value: %lu\r\n", value);
	HAL_UART_Transmit(&uart_type, (uint8_t*)buf, strlen(buf), 1000);
}

void ldr_uart_transmit_perc(ADC_HandleTypeDef type, UART_HandleTypeDef uart_type) {
	char buf[50];
	uint8_t value = ldr_get_percent(type, 1000);
	sprintf(buf, "Percent Value: %lu\r\n", value);
	HAL_UART_Transmit(&uart_type, (uint8_t*)buf, strlen(buf), 1000);
}

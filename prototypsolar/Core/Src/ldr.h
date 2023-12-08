
#ifndef SRC_LDR_H_
#define SRC_LDR_H_

#include "stm32f4xx_hal.h"

#define ERROR_NUM 5000
#define ERROR_PERC 101

void ldr_init(ADC_HandleTypeDef type);
void ldr_stop(ADC_HandleTypeDef type);
uint32_t ldr_get_value(ADC_HandleTypeDef type, uint32_t delay_ms);
uint8_t ldr_get_percent(ADC_HandleTypeDef type, uint32_t delay_ms);
void ldr_uart_transmit(ADC_HandleTypeDef type, UART_HandleTypeDef uart_type);
void ldr_uart_transmit_perc(ADC_HandleTypeDef type, UART_HandleTypeDef uart_type);

#endif /* SRC_LDR_H_ */

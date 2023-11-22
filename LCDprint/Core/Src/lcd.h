/*
 * lcd.h
 *
 *  Created on: Oct 14, 2022
 *      Author: pfeifer
 *
 * Usage:
 *
 * 1. create a LCD_HandleTypeDef and fill it
 * 2. call LCD_Begin()
 * 3. call LCD_SetCursor()
 * 4. call LCD_Print() or LCD_Printf()
 * 5. repeat from Step 3 whenever you want
 */

#ifndef SRC_LCD_LCD_H_
#define SRC_LCD_LCD_H_

#include "stm32f4xx_hal.h"
#include<stdbool.h>
#include<stdarg.h>

#define LCD_DEFAULT_ADDR (0x27 << 1)

typedef struct  {
	uint8_t i2c_addr;
	I2C_HandleTypeDef *i2c;
	bool backlight_enable;
} LCD_HandleTypeDef;

/**
 * Initialize the display. Must be called before any other function can be used.
 *
 * The LCD_HandleTypeDef must already be populated.
 * The i2c_addr is usually LCD_DEFAULT_ADDR.
 */
HAL_StatusTypeDef LCD_Begin(LCD_HandleTypeDef *dev);

/**
 * Position the LCD cursor; that is, set the location at which subsequent text written to the LCD will be displayed.
 */
HAL_StatusTypeDef LCD_SetCursor(LCD_HandleTypeDef *dev, uint8_t row, uint8_t col);


/**
 * Send text to display, without formatting.
 */
HAL_StatusTypeDef LCD_Print(LCD_HandleTypeDef *dev, const char* str);

/**
 * Send text to the display. See printf for formatting details.
 */
HAL_StatusTypeDef LCD_Printf(LCD_HandleTypeDef *dev, const char *format, ...);
/**
 * Clear the display
 */
HAL_StatusTypeDef LCD_Clear(LCD_HandleTypeDef *dev);
/**
 * Control the backlight.
 */
HAL_StatusTypeDef LCD_SetBacklight(LCD_HandleTypeDef *dev, bool backlight_on);


/**
 * Test the display. Will set an example text.
 * LCD_Begin() must have been called beforehand.
 */
HAL_StatusTypeDef LCD_Test(LCD_HandleTypeDef *dev);

#endif /* SRC_LCD_LCD_H_ */

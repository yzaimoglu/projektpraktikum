/*
 * lcd.c
 *
 *  Created on: Oct 14, 2022
 *      Author: pfeifer
 *
 *  Original by Aleksander Alekseev and @sstaub
 *  MIT License
 */
#include "lcd.h"
#include<stdio.h>

#define PIN_RS    (1 << 0)
#define PIN_EN    (1 << 2)
#define BACKLIGHT (1 << 3)

#define LCD_DELAY_MS 5
#define LCD_I2C_TIMEOUT 200

#define COLUMNS 16
#define ROWS 2

HAL_StatusTypeDef _LCD_SendInternal(LCD_HandleTypeDef *dev, uint8_t data, uint8_t flags) {
    HAL_StatusTypeDef res;
    for(int i=0;i<5;i++) {
        res = HAL_I2C_IsDeviceReady(dev->i2c, dev->i2c_addr, 1, LCD_I2C_TIMEOUT);
        if(res == HAL_OK)
            break;
    }
    if(res!=HAL_OK) {
    	return res;
    }


    uint8_t up = data & 0xF0;
    uint8_t lo = (data << 4) & 0xF0;
    uint8_t backlight = dev->backlight_enable?BACKLIGHT:0;

    uint8_t data_arr[4];
    data_arr[0] = up|flags|backlight|PIN_EN;
    data_arr[1] = up|flags|backlight;
    data_arr[2] = lo|flags|backlight|PIN_EN;
    data_arr[3] = lo|flags|backlight;

    //TODO send data_arr to the device
    res = HAL_I2C_Master_Transmit(dev->i2c, dev->i2c_addr, data_arr, sizeof(data_arr), LCD_I2C_TIMEOUT);
    if(res != HAL_OK) {
        return res;
    }


    HAL_Delay(LCD_DELAY_MS); //delay afterwards to not overwhelm display
    return res;
}

HAL_StatusTypeDef _LCD_SendCommand(LCD_HandleTypeDef *dev, uint8_t cmd) {
    return _LCD_SendInternal(dev, cmd, 0);
}

HAL_StatusTypeDef _LCD_SendData(LCD_HandleTypeDef *dev, uint8_t data) {
    return _LCD_SendInternal(dev, data, PIN_RS);
}

#define RETURN_IF_NOT_OK if(status!=HAL_OK)return status;

HAL_StatusTypeDef LCD_Print(LCD_HandleTypeDef *dev, const char *str) {
	HAL_StatusTypeDef status;
    while(*str) {
    	status = _LCD_SendData(dev, (uint8_t)(*str));
        RETURN_IF_NOT_OK;
        str++;
    }
    return HAL_OK;
}

HAL_StatusTypeDef LCD_Test(LCD_HandleTypeDef *dev) {
	HAL_StatusTypeDef status;
    // set address to 0x00
    status = _LCD_SendCommand(dev, 0b10000000);
    RETURN_IF_NOT_OK;
    status = LCD_Print(dev, "1602 LCD ready");
    RETURN_IF_NOT_OK;

    // set address to 0x40
    status = _LCD_SendCommand(dev, 0b11000000);
    RETURN_IF_NOT_OK;
    status = LCD_Print(dev, " !!! ");
    return status;
}

HAL_StatusTypeDef LCD_Begin(LCD_HandleTypeDef *dev) {
	dev->backlight_enable=true;
	HAL_StatusTypeDef status;

    // 4-bit mode, 2 lines, 5x7 format
    status = _LCD_SendCommand(dev, 0b00110000);
    RETURN_IF_NOT_OK;

    // display & cursor home (keep this!)
    status = _LCD_SendCommand(dev, 0b00000010);
    RETURN_IF_NOT_OK;

    // display on, right shift, underline off, blink off
    status = _LCD_SendCommand(dev, 0b00001100);
    RETURN_IF_NOT_OK;

    // clear display (optional here)
    status = _LCD_SendCommand(dev, 0b00000001);
    return status;
}


HAL_StatusTypeDef LCD_SetCursor(LCD_HandleTypeDef *dev, uint8_t row, uint8_t col) {
	if(row<0 || row>=ROWS || col<0 || col>=COLUMNS) {
		return HAL_ERROR;
	}
	uint8_t cmd = row==0?0b10000000:0b11000000;
	cmd+=col;
	return _LCD_SendCommand(dev, cmd);
}

HAL_StatusTypeDef LCD_Printf(LCD_HandleTypeDef *dev, const char *format, ...) {
	char lcd_buffer[COLUMNS + 1];
	va_list args;
	va_start(args, format);
	vsnprintf(lcd_buffer, COLUMNS + 1, format, args);
	va_end(args);
	return LCD_Print(dev, lcd_buffer);
}
HAL_StatusTypeDef LCD_Print_Solar_Voltage(LCD_HandleTypeDef lcd_handle, uint16_t solar_value){
	char voltageMsg[20];
	float voltage = (float)solar_value * 3.3f / 4095.0f * 1000.0f;
	int voltageInt = (int)(voltage);
	int voltageFrac = (int)((voltage - voltageInt) * 10);
	sprintf(voltageMsg, "Voltage:%d.%01dmV", voltageInt, voltageFrac);
	LCD_SetCursor(&lcd_handle, 0, 0);
	return LCD_Printf(&lcd_handle, voltageMsg);

}

HAL_StatusTypeDef LCD_Clear(LCD_HandleTypeDef *dev) {
	return _LCD_SendCommand(dev, 0b00000001);
}

HAL_StatusTypeDef LCD_SetBacklight(LCD_HandleTypeDef *dev, bool backlight_on) {
	dev->backlight_enable=backlight_on;
	return _LCD_SendCommand(dev, 0);
}

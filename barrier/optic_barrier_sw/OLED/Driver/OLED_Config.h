/*****************************************************************************
* | File      	:	OLED_Config.h
* | Author      :   Waveshare team
* | Function    :	Hardware interface
* | Info        :	
*----------------
* |	This version:   V1.0
* | Date        :   2019-07-31
* | Info        :   Basic version
******************************************************************************/
#ifndef _OLED_CONFIG_H_
#define _OLED_CONFIG_H_

#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <stdint.h>

/*------------------------------------------------------------------------------------------------------*/
char System_Init(void);
char System_Exit(void);

void I2C_Write_Byte(uint8_t value, uint8_t Cmd);

void Driver_Delay_ms(uint32_t xms);
void Driver_Delay_us(uint32_t xus);

#endif
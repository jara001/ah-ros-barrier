/*****************************************************************************
* | File      	:	OLED_Config.c
* | Author      :   Waveshare team
* | Function    :	Hardware interface
* | Info        :	
*----------------
* |	This version:   V1.0
* | Date        :   2019-07-31
* | Info        :   Basic version
******************************************************************************/
#include "OLED_Config.h"
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <wiringPiI2C.h>

#include <stdio.h>
#include "string.h"

int fd;

/********************************************************************************
function:	System Init and exit
note:
	Initialize the communication method
********************************************************************************/
char System_Init(void)
{
   //1.wiringPiSetupGpio
    //if(wiringPiSetup() < 0)//use wiringpi Pin number table
    if(wiringPiSetupGpio() < 0) { //use BCM2835 Pin number table
        fprintf(stderr, "set wiringPi lib failed	!!! \r\n");
        return 1;
    }
	
	fd = wiringPiI2CSetup(0x3c);

    return 0;
}

char System_Exit(void)
{
    return 0;
}

/********************************************************************************
function:	Hardware interface
note:
	I2C_Write_Byte(value, cmd):
		hardware I2C
********************************************************************************/
void I2C_Write_Byte(uint8_t value, uint8_t Cmd)
{
	int ref;
    ref = wiringPiI2CWriteReg8(fd, (int)Cmd, (int)value);
    while(ref != 0) {
        ref = wiringPiI2CWriteReg8 (fd, (int)Cmd, (int)value);
        if(ref == 0)
            break;
    }
}

/********************************************************************************
function:	Delay function
note:
	Driver_Delay_ms(xms) : Delay x ms
	Driver_Delay_us(xus) : Delay x us
********************************************************************************/
void Driver_Delay_ms(uint32_t xms)
{
    delay(xms);
}

void Driver_Delay_us(uint32_t xus)
{
    int j;
    for(j=xus; j > 0; j--);
}

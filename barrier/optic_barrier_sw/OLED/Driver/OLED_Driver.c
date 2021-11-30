/*****************************************************************************
* | File      	:	OLED_Driver.c
* | Author      :   Waveshare team
* | Function    :	2.23inch OLED using SSD1306 control
* | Info        :
*----------------
* |	This version:   V1.0
* | Date        :   2019-07-31
* | Info        :   Basic version
******************************************************************************/
#include "OLED_Driver.h"
#include <stdio.h>
#include "Show_Lib.h"
#include "string.h"

/*******************************************************************************
function:
		Common register initialization
*******************************************************************************/
static void OLED_InitReg()
{
	OLED_WriteReg(0xAE);//--turn off oled panel
	OLED_WriteReg(0x04);//--turn off oled panel	
	OLED_WriteReg(0x10);//--turn off oled panel
	OLED_WriteReg(0x40);//---set low column address
	OLED_WriteReg(0x81);//---set high column address
	OLED_WriteReg(0x80);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
	OLED_WriteReg(0xA1);//--set contrast control register
	OLED_WriteReg(0xA6); // Set SEG Output Current Brightness
	OLED_WriteReg(0xA8);//--Set SEG/Column Mapping     0xa0×óÓÒ·´ÖÃ 0xa1Õý³£
	OLED_WriteReg(0x1F);//Set COM/Row Scan Direction   0xc0ÉÏÏÂ·´ÖÃ 0xc8Õý³£
	OLED_WriteReg(0xC8);//--set normal display
	OLED_WriteReg(0xD3);//--set multiplex ratio(1 to 64)
	OLED_WriteReg(0x00);//--1/64 duty
	OLED_WriteReg(0xD5);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
	OLED_WriteReg(0xF0);//-not offset
	OLED_WriteReg(0xd8);//--set display clock divide ratio/oscillator frequency
	OLED_WriteReg(0x05);//--set divide ratio, Set Clock as 100 Frames/Sec
	OLED_WriteReg(0xD9);//--set pre-charge period
	OLED_WriteReg(0xC2);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
	OLED_WriteReg(0xDA);//--set com pins hardware configuration
	OLED_WriteReg(0x12);
	OLED_WriteReg(0xDB);//--set vcomh
	OLED_WriteReg(0x08);//Set VCOM Deselect Level
	OLED_WriteReg(0xAF);//-Set Page Addressing Mode (0x00/0x01/0x02)
}

void OLED_Clear(void)
{
    char Column,Page;
    for(Page = 0; Page < OLED_Page; Page++) {
        OLED_WriteReg(0xb0 + Page);    //Set page address
        OLED_WriteReg(0x04);           //Set display position - column low address
        OLED_WriteReg(0x10);           //Set display position - column high address
        for(Column = 0; Column < OLED_Column; Column++)
            OLED_WriteData(0xff);
    }
}

void OLED_Init(void)
{
    OLED_InitReg();
	
	OLED_Clear();
}

void OLED_SendBuf(UBYTE *Buf)
{
    char Column,Page;
    UBYTE *ptr = Buf;
    for(Page = 0; Page < OLED_Page; Page++) {
        OLED_WriteReg(0xb0 + Page);
        OLED_WriteReg(0x04);
        OLED_WriteReg(0x10);
        for(Column = 0; Column < OLED_Column; Column++) {
            OLED_WriteData(*ptr);
            ptr++;
        }
    } 
}

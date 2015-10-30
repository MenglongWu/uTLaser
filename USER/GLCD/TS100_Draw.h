/****************************************Copyright (c)**************************************************                         
**
**                                 http://www.powermcu.com
**
**--------------File Info-------------------------------------------------------------------------------
** File name:			GLCD.h
** Descriptions:		SSD1963 ²Ù×÷º¯Êý¿â
**						
**------------------------------------------------------------------------------------------------------
** Created by:			AVRman
** Created date:		2011-2-23
** Version:				1.0
** Descriptions:		The original version
**
**------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:	
** Version:
** Descriptions:		
********************************************************************************************************/

#ifndef __TS100_DRAW_H 
#define __TS100_DRAW_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "SSD1963_CMD.h"
#include <math.h>

/* Private define ------------------------------------------------------------*/
#define USE_16BIT_PMP

/*********************************************************************
* Overview: Horizontal and vertical display resolution
*                  (from the glass datasheet).
*********************************************************************/
#define DISP_HOR_RESOLUTION				320
#define DISP_VER_RESOLUTION				240

/*********************************************************************
* Overview: Horizontal synchronization timing in pixels
*                  (from the glass datasheet).
*********************************************************************/
#define DISP_HOR_PULSE_WIDTH		20    /* 20 */
#define DISP_HOR_BACK_PORCH			51	  /* 48	*/
#define DISP_HOR_FRONT_PORCH		20	  /* 20 */

/*********************************************************************
* Overview: Vertical synchronization timing in lines
*                  (from the glass datasheet).
*********************************************************************/
#define DISP_VER_PULSE_WIDTH		2	  /* 2 */
#define DISP_VER_BACK_PORCH			12	  /* 16 */
#define DISP_VER_FRONT_PORCH		4	  /* 4 */

/*********************************************************************
* Definition for SPI interface for HIMAX 8238-A relevant to hardware 
* layout; Hardware dependent!
*********************************************************************/
#define GPIO3 3
#define GPIO2 2
#define GPIO1 1
#define GPIO0 0
#define LCD_RESET (1<<GPIO3)	   /* LCD Reset signal (Reset for display panel, NOT ssd1963) */
#define LCD_SPENA (1<<GPIO0)	   /* SPI EN signal */
#define LCD_SPCLK (1<<GPIO1)	   /* SPI CLK */
#define LCD_SPDAT (1<<GPIO2)	   /* SPI DATA */

/* LCD color */
#define White          0xFFFF
//#define Black          0x0000
#define Black           0x22f2
//#define Grey           0xF7DE
#define Grey           0x22f2
#define Blue           0x001F
#define Blue2          0x051F
#define Red            0xF800
#define Magenta        0xF81F
#define Green          0x07E0
#define Cyan           0x7FFF
#define Yellow         0xFFE0

#define RGB565CONVERT(red, green, blue) (int) (((red >> 3) << 11) | ((green >> 2) << 5) | (blue >> 3))

/* Private function prototypes -----------------------------------------------*/
void LCD_Initializtion(void);
void LCD_Clear(uint16_t Color);	
void LCD_SetBacklight(uint8_t intensity);
uint16_t LCD_GetPoint(uint16_t Xpos,uint16_t Ypos);
void LCD_SetPoint(uint16_t Xpos,uint16_t Ypos,uint16_t point);
void PutChar(uint16_t Xpos,uint16_t Ypos,uint8_t c,uint16_t charColor,uint16_t bkColor);
void LCD_DrawLine( uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1 , uint16_t color );
void PutChinese(uint16_t Xpos,uint16_t Ypos,uint8_t *str,uint16_t Color,uint16_t bkColor); 
void GUI_Text(uint16_t Xpos, uint16_t Ypos, uint8_t *str,uint16_t Color, uint16_t bkColor);
void GUI_Chinese(uint16_t Xpos, uint16_t Ypos, uint8_t *str,uint16_t Color, uint16_t bkColor);	
//by Yu Jingxiong
void LCD_EightPoint(uint16_t x0,uint16_t y0,uint16_t x,uint16_t y,uint16_t color);
void LCD_MidpointCircle( uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);
void LCD_Rectangle( uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1 , uint16_t line_color, uint16_t fill_color );
void LCD_Triangle_Up(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1 , uint16_t line_color, uint16_t fill_color );
void LCD_Triangle_Down(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1 ,  uint16_t line_color, uint16_t fill_color );
void PutChar_Amplifier4( uint16_t Xpos, uint16_t Ypos, uint8_t ASCI, uint16_t charColor, uint16_t bkColor );
void PutChar_Amplifier16( uint16_t Xpos, uint16_t Ypos, uint8_t ASCI, uint16_t charColor, uint16_t bkColor );
void GUI_Text_Amplifier4(uint16_t Xpos, uint16_t Ypos, uint8_t *str,uint16_t Color, uint16_t bkColor);
void GUI_Text_Amplifier16(uint16_t Xpos, uint16_t Ypos, uint8_t *str,uint16_t Color, uint16_t bkColor);
void PutChinese_Amplifier4(uint16_t Xpos,uint16_t Ypos,uint8_t *str,uint16_t Color,uint16_t bkColor);
void Show_Matrix_zimo(uint16_t Xpos, uint16_t Ypos, uint8_t *Buffer, uint16_t Wide_char, uint16_t High, uint16_t charColor, uint16_t bkColor);
void LCD_Batter_Show( uint16_t x0, uint16_t y0 ,uint16_t rank );
void LCD_Wavelength_Selection( uint16_t x0, uint16_t y0 ,uint16_t wave ,uint16_t Color, uint16_t BkColor );
void LCD_OperatMode_Selection( uint16_t x0, uint16_t y0 ,uint16_t mode ,uint16_t Color, uint16_t BkColor );
void LCD_Power_Control_Selection( uint16_t x0, uint16_t y0 ,int16_t Current_Power ,uint16_t CharColor, uint16_t BkColor );
void LCD_Menu_Select( uint16_t menu );
void LCD_Timing_Display( uint16_t x0, uint16_t y0 ,uint16_t on_off );
void LCD_RunStop_Select(uint16_t x0,uint16_t y0,uint16_t RunStop );	   //x 140 y 36
void GUI_Text_Show_Number(uint16_t x, uint16_t y, uint8_t number, uint16_t Color, uint16_t bkColor);
void LCD_OperatMode_Selection( uint16_t x0, uint16_t y0 ,uint16_t mode ,uint16_t Color, uint16_t BkColor );
void LCD_Wavelength_Selection_Ex( uint16_t x0, uint16_t y0 ,uint16_t wave ,uint16_t Color, uint16_t BkColor );
void LCD_Power_Control_Selection_Ex( uint16_t x0, uint16_t y0 ,int16_t Current_Power ,uint16_t CharColor, uint16_t BkColor );
uint32_t ReadReg(uint16_t cmd,uint16_t *data);
#endif 

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/


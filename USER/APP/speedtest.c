#include "stm32f10x.h" 
#include "project.h"					//本工程的宏、函数、全局变量、结构体、版本信息、定义
#include "GUI.h"

int g_lcd_test = 0;

extern const unsigned char acOFF[512];
extern GUI_CONST_STORAGE unsigned char acpwm[1024];

#include "../uCGUI/GUI/Core/GUI_Protected.h"
void TestTickReset()
{
	g_lcd_test = 0;
}

int TestGetTick()
{
	return g_lcd_test;
}

int TestTick()
{
	g_lcd_test++;
}
void Test_LCD_L0_DrawBitmap_1BPP()
{
	int i;
	short index[2];

	g_lcd_test = 0;
	printf("----Test_LCD_L0_DrawBitmap_1BPP----");
	printf("start ...\n");

	index[0] = 0xff0000;
	index[1] = 0x00ff00;
	GUI_Context.DrawMode |= LCD_DRAWMODE_TRANS;
	for (i = 0; i < 1000; i++) {
		LCD_L0_DrawBitmap(0,0,
			64,64,
			1,
			8, 
			&acOFF[0],
			0,
			index);
	}
	printf("end %d\n", g_lcd_test);
}
extern GUI_CONST_STORAGE unsigned char acname[10000];

void Test_LCD_L0_DrawBitmap_2BPP()
{
	int i,k;
	short index[4];
	// return ;
	g_lcd_test = 0;
	printf("----Test_LCD_L0_DrawBitmap_2BPP----");
	printf("start ...\n");


	index[0] = 0xff0000;
	index[1] = RGB16(255,0,0);
	index[2] = RGB16(0,255,0);
	index[3] = RGB16(0,0,255);
	// GUI_Context.DrawMode |= LCD_DRAWMODE_TRANS;
	// for (i = 0; i < 500; i++) 
	{
		// index[2] = 0x0000ff + i;
		// index[3] = 0xf0f000 + i;
		LCD_L0_DrawBitmap(200,0,
			64,64,
			2,
			16, 

			&acpwm[0],
			1,
			index);
	}
	
			// LCD_L0_DrawBitmap(
			// 160,10,
			// 124,20,
			// 2,
			// 31,
			// &acname[0],
			// 1,
			// &index[0]);
			// while(1);
	for (k = 0; k < 0; k++) {
		for (i = 0; i < 60; i++) {
			index[2] = RGB16(0,255-i*2,0);
			index[3] = RGB16(0,0,255-i*2);
			LCD_L0_DrawBitmap(200,0,
				64,64,
				2,
				16, 
				&acpwm[0],
				1,
				index);
			Delay_ms(40);
		}
		for (i = 0; i < 60; i++) {
			index[2] = RGB16(0,255-60*2+i*2,0);
			index[3] = RGB16(0,0,255-60*2+i*2);
			LCD_L0_DrawBitmap(200,0,
				64,64,
				2,
				16, 
				&acpwm[0],
				1,
				index);
			Delay_ms(40);
		}
	}

	GUI_SetColor(RGB16(255, 0, 0));
	for (k = 0; k < 100;k++) {
		// LCD_L0_DrawHLine(k, 0, 200);
		LCD_L0_FillRect(0, 0, 320, 240);
	}
	Delay_ms(1000);
	printf("end %d\n", g_lcd_test);
}

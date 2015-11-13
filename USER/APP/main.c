/**
* @file    main.c
* @author  MenglongWu
* @version V1.0-bate
* @date    
* @brief   作者：吴梦龙\n


uTlaser 公司内部使用的光器件测试平台，包括测试连续激光器、脉冲激光器、APD、PIN。
	1. 连续激光器有3种功率输出方式 LV,MV,HV
	2. 脉冲激光器有2个PWM可选输出，两激光器同频同相，参数10us脉宽1ms周期的脉冲，其余脉宽为可选
	3. APD有20V、40V可选供电电压
	4. PIN没有软件控制，直接插入电路板，输入一定的光强，如果PIN正常电路板上的PIN检测LED灯亮
	5. 电源开关，关闭除STM32和LCD意外的外设电源，供插拔光器件，防止烧坏器件
	

*/


/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h" 
#include "stm32f10x_dma.h" 


#include "lcd\\gl_ui.h"
#include "lcd\\zimo_st9.h"
#include "lcd\\zimo_st20.h"

#include "prj_type.h"
#include "flash.h"
#include "project.h"					//本工程的宏、函数、全局变量、结构体、版本信息、定义
// #include "gl_key.h"						//异步按键驱动，待消息机制，本工程里没有使用该套接口
#include "key.h"						//按键扫描，本工程里使用该套接口获取按键状态（按下、抬起、长按）
#include "usart.h"						//串口


// Menglong Woo
#include "GUI.h"



/*****************************************************************************
我的定义
*****************************************************************************/
#ifndef _DEBUG_
#define debugtxt
#else 
#define debugtxt gl_text				//在LCD上显示调试信息
#endif

#ifndef _DEBUG_
#define dprintf
#else 
#define dprintf printf					///<通过串口显示调试信息
#endif


int g_lcd_test = 0;


struct wm_glide glide;
extern volatile uint32_t g_en ,g_tickgui ;
extern WM_HWIN hdlg;
extern const unsigned char acOFF[512];
extern GUI_CONST_STORAGE unsigned char acpwm[1024];
#include "../uCGUI/GUI/Core/GUI_Protected.h"
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



// *****************************************************************************
// 裸机程序任务分配
// 1. 主程序只负责 WP 桌面滑动
// 2. TIM5定时器做1ms定会
// 3. TIM6定时1ms，每10ms按键扫描、uCGUI更新界面、根据外部中断设置的定时时间到则检测触屏按压
// 4. EXTI0 检测触屏按下事件

int main(void)
{
	int index_move;
	GUI_RECT rect;
	int fpress = 0;
	int press = 0;
	int x0, y0;
	int x1, y1;
	int x, y;
	struct point pt;
	int b;
	int touch_x, touch_y;
	int tick = 0;
	WM_MESSAGE msg;
	WM_HWIN hMain, hleft,hright;
	USART_Configuration();
	Init_LED();
	
	printf("\n\n----------------------------------------------------------------------------\n");
	printf("        GLink uTLaser Runing\n");
	printf("%s\n%s\n", TARGET_NAME, RELEASE_DATE);

#define SYSCLK_FREQ_72MHz
	SystemInit();//SetSysClock();
	
	//液晶屏GUI配置
	gl_ui_setlib(p_zm_ascii_st9, 8, 12, p_zm_step_st9);
	gl_ui_setbkmode(BACKFILL);//背景填充模式
	gl_ui_setfontcolor(RGB16(255, 0, 0));
	gl_ui_setbkcolor(RGB16(0, 255, 0));
	gl_ui_setpencolor(RGB16(235, 34, 209));

	delay_init();
	/*****************按键中断初始化部分*********************** */
	// Function_IO_config(); 
	LCD_Configuration();
	// uTLaser 2015-8-31
	Init_PWM();
	// TIM5_Init(0);
	TIM5_Init();
	TIM6_Init();//1MS定时
	// TIM2_Configuration();
	TIM3_Init();
	{
		struct ctrl_pwm val;
		val.timer  = TIM3;
		val.ch     = PWM_CH1;
		val.high   = 100;
		val.enable = 1;
		Ctrl_PWM(&val);
		val.enable = 1;
		val.ch     = PWM_CH2;
		Ctrl_PWM(&val);
		// Ctrl_PWM(PWM_CH3, 8400, 900);
	}

	Init_Key();
	Init_APD();
	

	NVIC_Configuration();

	LCD_Initializtion();
	LCD_SetBacklight(0x80);
	//Delay(2000);// 稍作一段延时，非常重要，保证LCD完全上电，否则下面的Check1963会对LCD重启
	LCD_Clear(RGB(0, 0, 255));

	
	

	Init_PeripheralPower();
	Ctrl_PeripheralPower(CTRL_PP_ON);
	Init_LaserPower();
	// while (1) 
	{
		Ctrl_LaserPower(CTRL_LASER_HV);
		// Delay_ms(200);
		Ctrl_LaserPower(CTRL_LASER_MV);
		// Delay_ms(200);
		Ctrl_LaserPower(CTRL_LASER_LV);
		// Delay_ms(200);

		Ctrl_APD(CTRL_APD_20V);
		// Delay_ms(1000);
		Ctrl_APD(CTRL_APD_40V);
		// Delay_ms(1000);
	}
#ifdef _DEBUG_
	{
		int i;
		LCD_Clear(RGB(255, 0, 0));
		g_lcd_test = 0;
		printf("start ...\n");
		for (i  = 0; i < 30;i++) {
			LCD_L0_FillRect(0, 0, 320, 240);
		}
		printf("end %d\n", g_lcd_test);
		// for (i  = 0; i < 3000;i++) {
		// 	LCD_L0_DrawVLine(0, 0,240);
		// }

		for (i  = 0; i < 3000;i++) {
			LCD_L0_DrawHLine(0, 0,240);
		}
		// Test_LCD_L0_DrawBitmap_1BPP();
		// Test_LCD_L0_DrawBitmap_2BPP();
		LCD_Clear(RGB(0, 255, 0));
		gl_fill_rect(0,0,10,10);
		LCD_L0_FillRect(0, 0, 3, 3);
		
		
	}
#endif
	

	GUI_Init();
	ShowLogo();
	
	g_en = 1;
	TP_Init();
	TP_EINT();
	TP_GetADC(&pt);
	// TC_Adj();
	// TC_Adj();
	// Delay_ms(1000);
	FLASH_Configuration();


	// 配置 uCGUI 默认界面风格
	TEXT_SetDefaultTextColor(COL_ENABLE);
	BUTTON_GetDefaultTextColor(COL_ENABLE);
	BUTTON_SetDefaultBkColor(COL_BUTTON_BK, 0);
	BUTTON_SetDefaultBkColor(COL_BUTTON_BK, 1);
	BUTTON_SetDefaultTextColor(COL_ENABLE, 0);
	BUTTON_SetDefaultTextColor(COL_FOCUS, 1);
	
	// 1. 创建防 WP 滑动桌面
	// 2. 创建 WP 左面下方的左右滑动按钮（因为驱动问题不能便于实现界面跟随触屏滑动）
	hMain = MainTask();
	hleft = DestopBtn_Create(WM_HBKWIN, hMain);

	// 主程序只做一件事情，滑动 WP 主界面，滑动的多少在 DestopBtn 窗口里设置
	// 滑动过程实现两级速度滑动
	while(1) {
		if (glide.en == 1) {
			GUI_RECT rect;
			
			// while(glide.s_x != glide.e_x) {
			// 	glide.s_x += glide.d_x;
			// 	glide.s_y += glide.d_y;
				// WM_MoveTo(hMain, glide.s_x,0);
			while(glide.d1_loop > 0) {
				WM_MoveWindow(hMain, glide.d1_x, glide.d1_y);
				// printf("%d \n", glide.d1_loop);
				glide.d1_loop--;
				Delay_ms(30);

			}
			while(glide.d2_loop > 0) {
				WM_MoveWindow(hMain, glide.d2_x, glide.d2_y);
				// printf("%d \n", glide.d2_loop);
				glide.d2_loop--;
				Delay_ms(10);
			}
 			printf("after %d\n", rect.x0);
			// GUI_Exec();
			glide.en = 0;

			// WM_MoveTo(hMain, 200,0);
			// glide.d1_x = -10;
			// glide.d1_y = 0;
			// glide.d1_loop = 8;
			// glide.d2_x = -2;
			// glide.d2_y = 0;
			// glide.d2_loop = 10;
			// glide.en = 0;
		}
		// return 0;
	}
	
	// DemoRedraw();
	while(1)
	{
		TP_GetADC(&pt);
		Delay_ms(100);
		GUI_Exec();
		

		if (getlogxy(&pt)) 
		{

			// GUI_TOUCH_StoreUnstable(pt.x, pt.y);
			//GUI_Exec();
			sprintf(strout, "sec %d touch ADC %d %d  ", tick++, pt.x, pt.y);
			GUI_DispStringAt((const char*)strout, 0, 200);	
		}
	}//while结尾
}

void Debug_ucgui()
{
	GUI_SetBkColor(RGB(0, 0, 0)); //设置背景颜色 
	GUI_SetColor(RGB(255, 0, 255)); //设置前景颜色，及字体和绘图的颜色
	GUI_Clear(); //按指定颜色清屏
	GUI_DispStringAt("Hello World ..", 0, 100); //显示字符
	// GUI_CURSOR_Show();//显示鼠标, 测试触摸屏必须打开窗口功能 GUI_WINSUPPORT

	LCD_L0_FillRect(10, 10, 40, 40);
	//LCD_L0_SetPixelIndex(10, 100, RGB16(255, 0, 0));
	
	
	LCD_L0_SetPixelIndex(10, 190, RGB16(255, 0, 0));
	LCD_L0_SetPixelIndex(10, 191, RGB16(255, 0, 0));
	LCD_L0_SetPixelIndex(10, 192, RGB16(255, 0, 0));
	LCD_L0_SetPixelIndex(10, 193, RGB16(255, 0, 0));
	// LCD_SetPoint(100, 240, RGB16(255, 0, 0));
	gl_setarea(10, 200, 320, 240);
	gl_setpoint(10, 200, RGB16(255, 0, 0));
	gl_setpoint(10, 201, RGB16(0, 255, 0));
	gl_setpoint(10, 202, RGB16(255, 0, 0));
	gl_setpoint(10, 203, RGB16(0, 0, 255));
	gl_horizon_line(200, 270, 150);
}
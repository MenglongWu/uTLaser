#ifndef _PROJECT_H_
#define _PROJECT_H_
#include "stm32f10x_tim.h" 
#include "USER/TouchPanel/ads7843.h"
#include "WM.h"
//屏蔽警告
#pragma diag_suppress 167
#pragma diag_suppress 174
#pragma diag_suppress 1295//函数参数是空必须声明成void
#pragma diag_suppress 68//强制转换有符号和无符号


#define RGB8(r,g,b)	 // todo
#define RGB565(r,g,b)	 ((unsigned short) ( ((r) >> 3) << 11 ) | ( ((g) >> 2) << 5) | ( ((b) >> 3) << 0) )
#define RGB24(r,g,b)		((unsigned long) ( (r) << 16 )		| ( (g) << 8)		| ( (b) << 0) )

#define CONFIG_LCD_BPP	(24)
// Define RGB macro
#if CONFIG_LCD_BPP == 24
#define RGB(r,g,b)		RGB24((r),(g),(b))

#elif CONFIG_LCD_BPP == 16
#define RGB(r,g,b)		RGB565((r),(g),(b))

#elif CONFIG_LCD_BPP == 8
#define RGB(r,g,b)		RGB8((r),(g),(b))

#else
#warning "CONFIG_LCD_BPP only with 8/16/24 check out lcdconf.h"
#define RGB(r,g,b) RGB24((r),(g),(b))
#endif

#define RGB16(r,g,b) RGB565((r),(g),(b))


#define COL_FOCUS (RGB(0,0,0))
#define COL_ENABLE (RGB(255,255,255))
#define COL_DISABLE (RGB(120,120,120))
#define COL_BUTTON_BK (RGB(0,128,255))
#define COL_DIALOG_BK (RGB(30,30,30))

/*按键*/
#define KEY_A GPIO_Pin_5
#define KEY_B GPIO_Pin_3
#define KEY_C GPIO_Pin_2
#define KEY_X GPIO_Pin_1
#define KEY_Y GPIO_Pin_7
#define KEY_Z GPIO_Pin_6


struct wm_glide
{
	WM_HWIN hWin;
	int en;
	int d1_x;
	int d1_y;
	int d1_loop;
	int d2_x;
	int d2_y;
	int d2_loop;
};
// ***************************************************************************
// 激光器测试平台
#define CTRL_LASER_LV 0
#define CTRL_LASER_MV 1
#define CTRL_LASER_HV 2
void Ctrl_LaserPower(int val);

#define CTRL_APD_20V 0
#define CTRL_APD_40V 1
void Ctrl_APD(int val);
#define CTRL_PP_OFF 0
#define CTRL_PP_ON 1
void Ctrl_PeripheralPower(int val);

struct ctrl_pwm
{
	TIM_TypeDef *timer;// 定时器
	int ch;		// 通道
	int high;	// 高电平计数
	int cycle;
	int reversal;
	int enable;// 1 on; 0 off
};

#define PWM_TICK_MAX (36000)

#define PWM_CH1 0
#define PWM_CH2 1
#define PWM_CH3 2
#define PWM_CH4 3
void Ctrl_PWM(struct ctrl_pwm *val);
void Get_PWM(struct ctrl_pwm *val);
// ***************************************************************************


void USART_Configuration();
void delayMs(uint16_t ms);
void ADC_Configuration(void);
uint16_t GetAD(uint8_t chx);
void DAC_Configuration(void);
void TIM2_Init(void);
void TIM3_Init(uint16_t);
void TIM4_Init(uint16_t);
void TIM5_Init(uint16_t);
void TIM6_Init(void);
void NVIC_Configuration(void);
void FLASH_Configuration(void);
float DbmToScale(float dbm);
void Ctrl_RedLight(u8 v);
void Ctrl_Wavelength(u8 Wavelength_Selection_state);
void Ctrl_Operating_Mode(u8 Operating_Mode);
void Ctrl_Power(struct ctrl_param *v);
void AutoCtrlPower(void);
void Ctrl_Timing_Device(u8 Timer);
void Function_IO_config(void);
void RedLightIOConfig(void);
void External_Interrupt_Config(void);
void External_Interrupt_EXIT_Init(void);
void External_Interrupt_InterruptConfig(void);
void LCD_DrawMain(void);
void TurnOnPower(void);
void TurnOffPower(void);
void UI_ProWavelength(void);
void UI_ProPower(void);
void UI_ProMode(void);
void UI_ProRedLight_ShutdownTimer(void);
void ProChargerMonitor(void);
float GetBattery();
void ProTimerShutdown(void);
int main(void);
void DebugLookAD(void);
void UI_ProductionAdjust(void);
void UI_ProductConfig(void);
void DrawFocus(int16_t x,int16_t y,uint32_t color);
void IsHacker(void);
int ProTick1963IsLive(void);
void ProGet1963State(void);

void LCD_Batter_Show( uint16_t x0, uint16_t y0 ,uint16_t rank )	;		
void LCD_RedLight_Show( uint16_t x0, uint16_t y0 ,uint8_t flag);
void LCD_Wavelength_Selection( uint16_t x0, uint16_t y0 ,uint16_t wave ,uint16_t Color, uint16_t BkColor );
void LCD_Wavelength_Selection_Ex( uint16_t x0, uint16_t y0 ,uint16_t wave ,uint16_t Color, uint16_t BkColor );
void LCD_OperatMode_Selection( uint16_t x0, uint16_t y0 ,uint16_t mode ,uint16_t Color, uint16_t BkColor );

void LCD_Timing_Display( uint16_t x0, uint16_t y0 ,uint16_t on_off );
void Show_Matrix_zimo(uint16_t Xpos, uint16_t Ypos, uint8_t *Buffer, uint16_t Wide_char, uint16_t High, uint16_t charColor, uint16_t bkColor);
void LCD_Power_Control_Selection_Ex( uint16_t x0, uint16_t y0 ,int16_t Current_Power ,uint16_t CharColor, uint16_t BkColor );
void GUI_Text_Show_Number(uint16_t x, uint16_t y, uint8_t number, uint16_t Color, uint16_t bkColor );
void LCD_FLSAH_DrawPicture(uint16_t StartX,uint16_t StartY,uint16_t EndX,uint16_t EndY, uint8_t * pic);


//uint32_t InputPanel(int8_t *str,uint32_t len);
uint32_t InputPanel(int8_t *str,uint32_t len,uint32_t *outLen);


//启动版本信息
#define TARGET_NAME		"G-LINK TS100 VEX1.3.4"    		//目标板名称
#define RELEASE_DATE	"Release Date 2015.08.20"				//修改发布时间

#endif


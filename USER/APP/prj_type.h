#ifndef _PROJ_TYPE_H_
#define _PROJ_TYPE_H_
#include "stdint.h"
#include "stm32f10x.h"
#include "key.h"

//重定义别名

#define Key_Port_Set GPIOA
#define Key_Pin_Set KEY_A

#define Key_Port_Enter GPIOA
#define Key_Pin_Enter KEY_A

#define Key_Port_Up GPIOA
#define Key_Pin_Up KEY_A

#define Key_Port_Down GPIOA
#define Key_Pin_Down KEY_A

#define Key_Port_Left GPIOA
#define Key_Pin_Left KEY_A

#define Key_Port_Right GPIOA
#define Key_Pin_Right KEY_A

//保存当前波长、各个模式下的adc、dac值，提升用户体验
struct fast_switch
{
	int32_t enable;
	int32_t set;

	//CH1
	uint16_t ch1_adc_off;///<快速校准ch1
	uint16_t ch1_adc_cw;///<
	uint16_t ch1_adc_270;///<
	uint16_t ch1_adc_1k;///<
	uint16_t ch1_adc_2k;///<

	uint16_t ch1_dac_off;
	uint16_t ch1_dac_cw;
	uint16_t ch1_dac_270;
	uint16_t ch1_dac_1k;
	uint16_t ch1_dac_2k;

	//CH2
	uint16_t ch2_adc_off;
	uint16_t ch2_adc_cw;
	uint16_t ch2_adc_270;
	uint16_t ch2_adc_1k;
	uint16_t ch2_adc_2k;

	uint16_t ch2_dac_off;
	uint16_t ch2_dac_cw;
	uint16_t ch2_dac_270;
	uint16_t ch2_dac_1k;
	uint16_t ch2_dac_2k;

	//CH3
	uint16_t ch3_adc_off;
	uint16_t ch3_adc_cw;
	uint16_t ch3_adc_270;
	uint16_t ch3_adc_1k;
	uint16_t ch3_adc_2k;

	uint16_t ch3_dac_off;
	uint16_t ch3_dac_cw;
	uint16_t ch3_dac_270;
	uint16_t ch3_dac_1k;
	uint16_t ch3_dac_2k;
};
struct ctrl_param//需要控制的参数
{
	int32_t lastset;
	int32_t set; ///<设置值参数扩大1000倍
	int32_t cur; ///<实际值
	uint16_t adc;///<dac
	uint16_t dac;
	int8_t  ld;
	uint16_t vol;
	uint8_t level;//能控制的级别
	int32_t temp;
};

#include "user/touchpanel/touch.h"
struct project_env
{
	unsigned long flag;
	struct adj_tp adj_tp;
};
struct adj_power_flash
{
	uint32_t flag;///<标志，当其为0xAABBCCDD时候表示下面的数据有效，否则下面数据清零

	float _adc;///<校准ADC值，记录在输出10dBm时的ADC值，@see Ctrl_Power

	//float _1310_270;//DAC
	float _dac;///<校准DAC值，记录在输出10dBm时的DAC值，提高自动调节速度，@see AutoCtrlPower

	//////无用区域
	float _1310_1k;
	float _1310_2k;
	
	float _1550cw;
	float _1550_270;
	float _1550_1k;
	float _1550_2k;
	//////////
	uint8_t sn[28];
	uint8_t _650_en;///<650使能
	uint8_t _1310_en;///<ch1（1310）使能
	uint8_t _1490_en;///<ch2（1490）使能
	uint8_t _1550_en;///<ch3（1550）使能
	
	
	//贴牌
	uint32_t _logo_addr;///<logo地址，保留
	uint16_t _logo_backcolor;///<logo背景色，保留
	uint16_t _logo_w;///<logo宽度，宽高有最大限制，保留
	uint16_t _logo_h;///<logo高度，保留
	
	uint16_t _ch1wave;///<ch1波长，默认1310
	uint16_t _ch2wave;///<ch2波长，默认1490
	uint16_t _ch3wave;///<ch3波长，默认1550
	
};
/*struct point
{
	int16_t x;
	int16_t y;
};*/
// typedef struct _RECT
// {
// 	int32_t left;
// 	int32_t top;
// 	int32_t right;
// 	int32_t bottom;
// }RECT;

// typedef struct win_class
// {
// 	uint32_t id;
// 	RECT rc;
// }win_class;

extern struct ctrl_param g_power;//功率设置
extern struct adj_power_flash g_adj_power;//校准参数
extern volatile u16 powerDownDelayCnt;//关机长按延时
extern uint32_t hackval ;//进入后台输入的密码
extern uint32_t hackflag ;//是否输入密码标志
extern volatile u8 g_red_onoff ;//
extern volatile int8_t g_red_mode ;//红灯显示模式，常亮、闪烁、关闭
extern volatile u8 g_red_delay_100ms ;//红灯闪烁0.5s延时
extern volatile u8 g_onoff_en ;//使能关机，防止开机后再关机
extern volatile u8 g_autoctrlpower_en;

extern volatile u8 g_key_timer_100ms ;//按键定时器，用于判断长按下和单击，复用红灯和定时关机按键
extern volatile uint16_t g_batter_delay ;//电池刷新显示延时
/////////////////////
extern volatile uint16_t g_power_down;
extern volatile uint16_t g_ad_ms ,g_adjust_ms ,g_lcdbug_ms ,g_usart_ms ,g_lcdlisten_ms ,g_debug_ms ,g_redbug_ms;//ad采样间隔
extern volatile uint16_t g_adc[200];//ad采样数
extern volatile uint16_t ADCConvertedValue[2000];//AD采样DMA缓存

extern int8_t g_recvflag ;//串口接收标志
extern volatile uint8_t strout[256];

/*****************************************************************************
原来的定义
*****************************************************************************/
extern volatile u16 SysTickCounter;
extern volatile u16 Timer_State ;	    //定时器状态指示,  OFF,  5min  ，10min  ，15min  ，30min  ，60min 状态
extern volatile u16 Timer_Counter ;
extern volatile u16 Wavelength_Selection_state ;	 //0 = 关闭状态 1 = 1310nm 2 = 1495nm 3 = 1550nm 	4 = 红光
extern volatile u16 Operating_Mode ;	//0 = CW、 1 = PW270Hz、2 = 1KHz、3 = 2KHz  连续光/脉冲光选择
extern volatile u8  Batter_Lightning;
extern volatile u8  LCD_GetPoint_EN ;   //防止触摸屏重复操作，相当于按键去抖动,可以进行触摸屏采集。
extern volatile u8  LCD_GetPoint_Counter ; //触摸屏使能计数值。。
#endif


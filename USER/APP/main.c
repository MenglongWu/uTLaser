/**
* @file    main.c
* @author  MenglongWu
* @version VEx1.2.3
* @date    
* @brief   作者：吴梦龙\n
TS100——手持稳定光源，主程序接口
* 
**--------------File Info---------------------------------------------------------------------------------
** File name:               main.c
** Descriptions:            The TouchPanel application function
**--------------------------------------------------------------------------------------------------------
** Created by:              G-LINK GROP
** Created date:            2011-12-07	flash OK版本   
** Version:                 v1.0
** Descriptions:            The original version
**--------------------------------------------------------------------------------------------------------
** Modified by:			Yu Jingxiong   2011.11.10
V1.0    yujignxiong   2011.11.27
** Modified date:		重新阅读并标注加深理解。。。。keep moving！！          
** Version:                 
** Descriptions:     TIM_CtrlPWMOutputs(TIM3, ENABLE);       
**




2012.11.02 -- 2012.11.14 更新   吴梦龙
-1、完成功率自动控制
-2、生产校准后台
-3、AD采样改成DMA方式
-4、取消触摸屏触控
-5、完善电源检测

-文件结构
-main.c 				主程序
-PictureData.c 		16位彩色图片资源
-gl_ui.c 			图形接口
-zimo_st9.c 			部分ascii码宋体9号字体
-key.c   			按键扫描
-flash.c  			内部flash读写
-usart.c   			外部

2013.03.15 吴梦龙
1、重新划分编译器rom空间，0x08000000 - 0x0803f7ff（0x3f800大小），
余下0x800（2K）空间用于存储校准信息，flash.h文件里开始写入开始地址是0x0803F800
********************************************************************************************************
*/


/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h" 
//#include "stm32_eval.h" 
#include "..\\TouchPanel\\TouchPanel.h"
#include "..\\SysTick\\systick.h"
#include "..\\GLCD\\GLCD.h"
#include "stm32f10x_adc.h" 
#include "stm32f10x_dac.h"
#include "stm32f10x_dma.h"   
#include "stm32f10x_tim.h" 
#include "stm32f10x_usart.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_flash.h"


#include "lcd\\gl_ui.h"
#include "lcd\\zimo_st9.h"
#include "lcd\\zimo_st20.h"
#include <string.h>
#include <stdio.h>
#include "prj_type.h"
#include "flash.h"

#include "misc.h" 
#include <math.h>
#include <string.h>
#include <stdio.h>
//#include <PictureData.h>	
#include "USER\\GLCD\\PictureData.h"	//字模相关
#include "project.h"					//本工程的宏、函数、全局变量、结构体、版本信息、定义
#include "gl_key.h"						//异步按键驱动，待消息机制，本工程里没有使用该套接口
#include "lib\\base.h"
#include "LCD\\gl_type.h"				//图形库定义的类型
#include "key.h"						//按键扫描，本工程里使用该套接口获取按键状态（按下、抬起、长按）
#include "usart.h"						//串口




// Menglong Woo
#include "GUI.h"

#include "FRAMEWIN.h" 
#include "BUTTON.h" 
#include "LISTBOX.h"
#include "text.h" 
FRAMEWIN_Handle hFrame; 
BUTTON_Handle hButton1; 
BUTTON_Handle hButton2; 
LISTBOX_Handle hListbox;

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

int32_t g_pwm_high = 1;

uint16_t g_sm_mm_color = Yellow;
struct fast_switch fastsw;				//快速校准功率、缩短AutoCtrlPower校准时间
struct ctrl_param g_power;				/*< 功率设置 */
struct adj_power_flash g_adj_power;		/*< 校准参数 */
struct project_env g_env;
volatile u8 g_autoctrlpower_en = 1;

volatile u16 powerDownDelayCnt=0;		///<在定时中断里检查开机键是否被长按计数

uint32_t hackval = 0;					///<进入后台输入的密码
uint32_t hackflag = 0;					///<是否输入密码标志

volatile u8     g_red_onoff = 0;		///<红光开关状态
volatile int8_t g_red_mode = 0;			///<红灯显示模式，常亮、闪烁、关闭
volatile u8     g_red_delay_100ms = 0;	///<红灯闪烁0.5s延时
volatile u8     g_onoff_en = 0;			///<使能关机，防止开机后再关机

volatile u8 g_key_timer_100ms = 0;		///<按键定时器，用于判断长按下和单击，复用红灯和定时关机按键
volatile uint16_t g_batter_delay = 0;	///<电池刷新显示延时
volatile float g_battery_vol = 12;		///<电池电压，初始化默认12V，具体多少需要采集AD数据决定

volatile uint16_t g_power_down = 0;		///<是个标志变量，表示是否正常开机，LCD是否被SSD1963点亮
///<///////////////////
//以下g_xxx_ms表示1ms中断定时器计数，为了便于模块化管理，也为了以后软件维护方便，计数用分别独立的全局变量，
//相关处理函数自己给计数器清零，至于是向上计数还是向下计数由应用决定。
//便于模块化管理：多变量计数防止两两应用使用相同的计数器，某一应用复位计数器影响另一应用。
//软件维护方便：维护是如果需要添加计数器只需添加一个变量即可，没必要查看原先写的代码

volatile uint16_t	g_ad_ms = 0, 		///<<AD采样间隔
					g_adjust_ms = 0, 	///<自动校准功率间隔
					g_lcdbug_ms = 0, 	///<已经没有任何作用了，调试代码也被删除了
					g_usart_ms = 0, 		///<串口不间断输出调试信息延时
					g_lcdlisten_ms = 0, 	///<如果IC（SSD1963）没有正常启动、LCD被拔下IC断路、IC受静电复位
					///<则需要对IC重新启动初始化，启动间隔由g_lcdlisten_ms决定
					g_debug_ms = 0, 		///<Debugxxx函数里为减缓输出频率使用的延时
					g_redbug_ms = 0;	///<ad采样间隔
volatile uint16_t ADCConvertedValue[2000];///<AD采样DMA缓存，实际上只使用了400个空间
volatile uint16_t g_adc[200];			///<得到ADCConvertedValue里某通道的连续采样AD值

int8_t g_recvflag = 0;					/*<串口接收标志，调试所用，可以废除 */
volatile uint8_t strout[256];			///<偷懒用的，为了不想再每个调试函数里多次定义，但是这样做一定要
//记住strout长度是50字节，sprintf向里面输入内容越界的话后果很严重
//如果是函数内部里越界sprintf会影响strout后面的内存的值，如：
//char strout[3]
//char a = 0;
//char b = 2;
//sprintf(strout, "over!");
//执行以上代码后a、b的值会变成r、!，相对来说函数内部内存泄露还好查些，
//同理全局变量越界了会影响后面的全局变量，BUG就不好查找了，因为任何函数（A）都能调用它，但是可能
//当其他函数（B）读取被越界修改的值才执行错误的信息，调试时很容易把嫌疑落在函数（B）上

/*****************************************************************************
原来的定义
*****************************************************************************/
volatile u16 SysTickCounter=0;					///<系统Tick计数，没用
volatile u16 Timer_State = 0;					///<定时器状态指示,  OFF,  5min  ，10min  ，15min  ，30min  ，60min 状态
volatile u16 Timer_Counter = 0;					///<定时关机计数
volatile u16 Wavelength_Selection_state = 0;	///<0 = 关闭状态 1 = 1310nm 2 = 1495nm 3 = 1550nm 	4 = 红光
volatile u16 Operating_Mode = 0;				///<0 = CW、 1 = PW270Hz、2 = 1KHz、3 = 2KHz  连续光/脉冲光选择


volatile u8 FLAG_1310 = 0;		  ///<定时器中断中脉冲翻转标志。
volatile u8 FLAG_1490 = 0;
volatile u8 FLAG_1550 = 0;

uint16_t  TIM_Period[3] = {132, 35, 17};			///<分频值133、36、18分别对应541Hz、2KHz、4KHz
												///<分频系数统一为1000分频, 0代表关闭光源，1代表连续光源

volatile uint16_t  TIM_Period1310 = 132;		///<初始化激光器分频
volatile uint16_t  TIM_Period1490 = 132;
volatile uint16_t  TIM_Period1550 = 35;

///<触屏使用
uint16_t X[3]={889, 960, 3320};
uint16_t Y[3]={487, 3352, 1982};
/**************************************************************************************/

// #define DAC_DHR12RD_Address   ((uint32_t)0x40007420)
// #define ADC2_DR_Address ((uint32_t)0x40013C4C)

/*********************************功能控制端口*****************************************/
#define RCC_GPIO_CTRL_B 			RCC_APB2Periph_GPIOB
#define GPIO_CTRL_PORT_B 			GPIOB
#define GPIO_SYSPWR_ONOFF 			GPIO_Pin_6    ///<开关机控制（CHECK2） 开机置1，关机清0 

#define RCC_GPIO_CTRL_C 			RCC_APB2Periph_GPIOC	 ///<多路选择我开关是能段控制口，定时器中断响应，波长选择+频率控制
#define GPIO_CTRL_PORT_C 			GPIOC

#define GPIO_PORT_POWER_CHK 		GPIOA
#define GPIO_ONOFF_CHK 			    GPIO_Pin_1  ///<手动开关机, 启停监测CHECK1, PD2, 上升沿中断

#define GPIO_KEY_RED_CON 			GPIO_Pin_0
#define GPIO_KEY_1310_CON 			GPIO_Pin_3	 ///<对应管脚错误，2011.11.25测试修改
#define GPIO_KEY_1490_CON 			GPIO_Pin_2
#define GPIO_KEY_1550_CON 			GPIO_Pin_1
#define GPIO_CHARG_CHK 				GPIO_Pin_8


#define GPIO_PORT_LCD 				GPIOD
#define GPIO_LCD_OFF				GPIO_Pin_7    ///<关LCD屏控制（ON/OFF）  LCD使能端口，置低片选 




/*****************************************************************************
工程模块配置
*****************************************************************************/
/*****************一般功能IO口初始化******************************************/
/**
 * @brief	配置IO端口
 * @param\n
 * @retval\n	NULL
 * @remarks	
 */


void Function_IO_config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	//电源检测端口CHECK1(KEY_X)、CHECK2、CHARGER
	//CHECK1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin		= GPIO_ONOFF_CHK;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//CHECK2
	GPIO_InitStructure.GPIO_Pin		= GPIO_SYSPWR_ONOFF;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB, GPIO_SYSPWR_ONOFF);
	//CHARGER悬浮或下拉



	//按键KEY_A, KEY_b, KEY_B, KEY_Y, KEY_Z
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin		= KEY_A | KEY_B | KEY_C | KEY_Y | KEY_Z;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	//激光器：1550、1310、1490、650
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin		= 
									  GPIO_KEY_RED_CON | GPIO_KEY_1310_CON | 
									  GPIO_KEY_1490_CON | GPIO_KEY_1550_CON;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_Out_PP;

	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC, GPIO_KEY_1310_CON);
	GPIO_ResetBits(GPIOC, GPIO_KEY_1490_CON);
	GPIO_ResetBits(GPIOC, GPIO_KEY_1550_CON);
	GPIO_ResetBits(GPIOC, GPIO_KEY_RED_CON);

	// 	//LCD使能
	// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	// 	GPIO_InitStructure.GPIO_Pin = GPIO_LCD_OFF;
	// 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	// 	GPIO_Init(GPIOD, &GPIO_InitStructure);
	// 	GPIO_SetBits(GPIOD, GPIO_LCD_OFF);

	//1963:TS
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin		= GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	LCD_Configuration();
}
/**
 * @brief	配置红光IO
 * @param\n	
 * @retval\n	NULL
 * @remarks	
 */

void RedLightIOConfig()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz ;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin		= GPIO_Pin_0;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}
/**
 * @brief	配置串口
 * @param	
 * @retval\n	NULL
 * @remarks	波特率115200，用于输出调试信息
 */

void USART_Configuration()
{
	USART_InitTypeDef USART_InitStructure;
	struct com_dev comdev;
	//u8 KeyNum = 0;
	//uint32_t IntDeviceSeriNum[3];	

	USART_InitStructure.USART_BaudRate				= 115200;
	USART_InitStructure.USART_WordLength			= USART_WordLength_8b;
	USART_InitStructure.USART_StopBits				= USART_StopBits_1;
	USART_InitStructure.USART_Parity				= USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl	= USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode					= USART_Mode_Rx | USART_Mode_Tx;

	//comdev.usart = COM_B6B7;
	//comdev.usart		= COM_B10B11;
// 	comdev.dma_tch		= DMA1_Channel4;
// 	comdev.dma_rch		= DMA1_Channel5;
// 	comdev.dma_f_tch	= DMA1_FLAG_TC4;
// 	comdev.dma_f_rch	= DMA1_FLAG_TC5;
	
	
	comdev.usart		= COM_C10C11;
	comdev.dma_tch		= DMA1_Channel4;
	comdev.dma_rch		= DMA1_Channel5;
	comdev.dma_f_tch	= DMA1_FLAG_TC4;
	comdev.dma_f_rch	= DMA1_FLAG_TC5;
	CommInit(&comdev, &USART_InitStructure);
	CommDMAMode(1);	
	
	
	
}


/**
 * @brief	电池电压测量通道，显示电池电量，低电压关机
 * @param\n	
 * @retval\n	NULL
 * @remarks	
 */

void ADC_Configuration()
{
	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	//ADC1 采样wave 等其他共5个通道，用DMA通道1
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	// Enable ADC1 and GPIOC clock 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 , ENABLE);	

	// DMA1 channel1 configuration ----------------------------------------------
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr	= (uint32_t)&(ADC1->DR);//ADC1_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr		= (uint32_t)&ADCConvertedValue;
	DMA_InitStructure.DMA_DIR					= DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize			= 400;//ADC_BUF;//two adc channel, cycle sampling 11 times
	DMA_InitStructure.DMA_PeripheralInc			= DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc				= DMA_MemoryInc_Enable;//这样可以寻址ADCConvertedValue
	DMA_InitStructure.DMA_PeripheralDataSize	= DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize		= DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode					= DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority				= DMA_Priority_High;
	DMA_InitStructure.DMA_M2M					= DMA_M2M_Disable;

	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel1, ENABLE);// Enable DMA1 channel1 


	ADC_InitStructure.ADC_Mode					= ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode			= ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode	= ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv		= ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign				= ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel			= 2;

	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 1, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 2, ADC_SampleTime_55Cycles5);
	//ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 1, ADC_SampleTime_55Cycles5);


	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);
	ADC_ResetCalibration(ADC1);						// Enable ADC1 reset calibration register    
	while(ADC_GetResetCalibrationStatus(ADC1));		// Check the end of ADC1 reset calibration register 
	ADC_StartCalibration(ADC1);						// Start ADC1 calibration 
	while(ADC_GetCalibrationStatus(ADC1));			// Check the end of ADC1 calibration 	
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);			// Start ADC1 Software Conversion  


}

void ADC2Configuration(void)  
{  
}

/**
 * @brief	配置DAC，用于功率输出
 * @param\n	
 * @retval\n	NULL
 * @remarks	
 */

void DAC_Configuration(void)
{  
	DAC_InitTypeDef DAC_InitStructure ;
	/* GPIOA Periph clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	/* DAC Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

	DAC_InitStructure.DAC_Trigger		 =DAC_Trigger_Software ;
	//设置为软件触发
	DAC_InitStructure.DAC_WaveGeneration =DAC_WaveGeneration_None ;
	DAC_InitStructure.DAC_OutputBuffer	 =DAC_OutputBuffer_Enable ;

	/* DAC channel1 Configuration */
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);

	/* Enable DAC Channel1: Once the DAC channel2 is enabled, PA.04 is 
	automatically connected to the DAC converter. */
	DAC_Cmd(DAC_Channel_1, ENABLE);

	//实际对应神州IV开发板的DAC1引脚--PA.4 (PIN29)
}



void TIM2_Configuration(void)
{
		//TIM设置
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
	TIM_OCInitTypeDef TIM_OCStruct;

	GPIO_InitTypeDef gpio_init;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	//不分频。PWM频率=72000000/900=80Khz
	/* ---------------------------------------------------------------
	TIM2CLK 即PCLK1=36MHz
	TIM2 Configuration: generate 1 PWM signals :
	    TIM2CLK = 36 MHz, Prescaler = 0x0, TIM2 counter clock = 36 MHz
	    TIM2 ARR Register = 900 => TIM2 Frequency = TIM2 counter clock/(ARR + 1)
	    TIM2 Frequency = 36 KHz.
	    TIM2 Channel2 duty cycle = (TIM2_CCR2/ TIM2_ARR)* 100 
	TIM2CLK = 36 MHz, Prescaler = 0, TIM2 counter clock = 36MHz
	--------------------------------------------------------------- */
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 36000; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值  80K
	TIM_TimeBaseStructure.TIM_Prescaler = (2-1); //设置用来作为TIMx时钟频率除数的预分频值  不分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

	/* Output Compare Active Mode configuration: Channel3 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 100;                   //设置待装入捕获比较寄存器的脉冲值，初始的占空比
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);  //使能TIMx在CCR2上的预装载寄存器

	TIM_OCInitStructure.TIM_Pulse = 50;
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);  //使能TIMx在CCR2上的预装载寄存器
	//上面两句中的OC2确定了是channle几，要是OC3则是channel 3  

	TIM_ARRPreloadConfig(TIM2, ENABLE); //使能TIMx在ARR上的预装载寄存器 

	/* TIM3 enable counter */
	TIM_Cmd(TIM2, ENABLE);  //使能TIMx外设

	return ;
}

void TIM3_Configuration(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	//不分频。PWM频率=72000000/900=80Khz
	/* ---------------------------------------------------------------
	TIM3CLK 即PCLK1=36MHz
	TIM3 Configuration: generate 1 PWM signals :
	    TIM3CLK = 36 MHz, Prescaler = 0x0, TIM3 counter clock = 36 MHz
	    TIM3 ARR Register = 900 => TIM3 Frequency = TIM3 counter clock/(ARR + 1)
	    TIM3 Frequency = 36 KHz.
	    TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR)* 100 
	TIM3CLK = 36 MHz, Prescaler = 0, TIM3 counter clock = 36MHz
	--------------------------------------------------------------- */
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 36000; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值  80K
	TIM_TimeBaseStructure.TIM_Prescaler = (2 - 1); //设置用来作为TIMx时钟频率除数的预分频值  不分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

	/* Output Compare Active Mode configuration: Channel3 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 100;                   //设置待装入捕获比较寄存器的脉冲值，初始的占空比
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIMx在CCR2上的预装载寄存器
	//上面两句中的OC2确定了是channle几，要是OC3则是channel 3  

	TIM_ARRPreloadConfig(TIM3, ENABLE); //使能TIMx在ARR上的预装载寄存器 

	/* TIM3 enable counter */
	TIM_Cmd(TIM3, ENABLE);  //使能TIMx外设
}


/**
 * @brief	用于定时计数，1ms定时中断
 * @param\n	
 * @retval\n	NULL
 * @remarks	
 */

void TIM2_Init(void)
{   
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 
	/* ---------------------------------------------------------------
	TIM4 Configuration: Output Compare Timing Mode:
	TIM2CLK = 36 MHz, Prescaler = 7200, TIM2 counter clock = 7.2 MHz
	--------------------------------------------------------------- */
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period		= 7200;	//固定定时时长，100ms ，10Hz     注： 10ms时候72000	100Hz
	TIM_TimeBaseStructure.TIM_Prescaler		= (1000 - 1);
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode	= TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	/* Clear TIM3 update pending flag[清除TIM3溢出中断标志] */
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	/* TIM IT enable */
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	/* TIM3 enable counter */
	TIM_Cmd(TIM2, ENABLE);
	//TIM_Cmd(TIM3, DISABLE);  	//初始化时候先关闭TIM3。激光器输出CW波形
}

/*******************************************************************************
* Function Name  : NVIC_Configuration       	  对应1310通道，管脚PC1, 初始频率270hz
* Description    : Configures the used IRQ Channels and sets their priority.
*******************************************************************************/
/**
 * @brief	用于定时，ch1（默认1310）输出脉冲光
 * @param\n	
 * @retval\n	NULL
 * @remarks	
 */

void TIM3_Init(uint16_t  TIM_Period1310)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 
	/* ---------------------------------------------------------------
	TIM4 Configuration: Output Compare Timing Mode:
	TIM2CLK = 36 MHz, Prescaler = 7200, TIM2 counter clock = 7.2 MHz
	--------------------------------------------------------------- */
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period		= TIM_Period1310;
	TIM_TimeBaseStructure.TIM_Prescaler		= (1000 - 1);
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode	= TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	/* Clear TIM3 update pending flag[清除TIM3溢出中断标志] */
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	/* TIM IT enable */
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	/* TIM3 enable counter */
	TIM_Cmd(TIM3, ENABLE);
	TIM_Cmd(TIM3, DISABLE);  	//初始化时候先关闭TIM3.。激光器输出CW波形
}


/*******************************************************************************
* Function Name  : NVIC_Configuration						   对应1490通道，管脚PC2, 初始频率270hz
* Description    : Configures the used IRQ Channels and sets their priority.
*******************************************************************************/
/**
 * @brief	用于定时，ch2（默认1490）输出脉冲光
 * @param\n	
 * @retval\n	NULL
 * @remarks	
 */
void TIM4_Init(uint16_t  TIM_Period1490)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	/* TIM5 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); 
	/* ---------------------------------------------------------------
	TIM4 Configuration: Output Compare Timing Mode:
	TIM2CLK = 36 MHz, Prescaler = 7200, TIM2 counter clock = 7.2 MHz
	--------------------------------------------------------------- */
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period		= TIM_Period1490;
	TIM_TimeBaseStructure.TIM_Prescaler		= (1000 - 1);
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode	= TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	/* Clear TIM5 update pending flag[清除TIM5溢出中断标志] */
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	/* TIM IT enable */
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	/* TIM5 enable counter */
	TIM_Cmd(TIM4, ENABLE);
	TIM_Cmd(TIM4, DISABLE);  	//初始化时候先关闭TIM4.。激光器输出CW波形
}


/*******************************************************************************
* Function Name  : NVIC_Configuration					   对应1550通道，管脚PC3, 初始频率270hz
* Description    : Configures the used IRQ Channels and sets their priority.
*******************************************************************************/
/**
 * @brief	用于定时，ch3（默认1550）输出脉冲光
 * @param\n	
 * @retval\n	NULL
 * @remarks	
 */
void TIM5_Init(uint16_t  TIM_Period1550)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	/* TIM5 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); 

	/* ---------------------------------------------------------------
	TIM4 Configuration: Output Compare Timing Mode:
	TIM2CLK = 36 MHz, Prescaler = 7200, TIM2 counter clock = 7.2 MHz
	--------------------------------------------------------------- */
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period		= TIM_Period1550;	//自动加载的计数值。。。
	TIM_TimeBaseStructure.TIM_Prescaler		= (1000 - 1);
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode	= TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	/* Clear TIM5 update pending flag[清除TIM5溢出中断标志] */
	TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	/* TIM IT enable */
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
	/* TIM5 enable counter */
	TIM_Cmd(TIM5, ENABLE);
	TIM_Cmd(TIM5, DISABLE);
}

/**
 * @brief	用于定时，定时1ms
 * @param\n	
 * @retval\n	NULL
 * @remarks	
 */
void TIM6_Init()
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); 
	//时钟72M    1ms中断  向上计数
	TIM_TimeBaseStructure.TIM_Period		= 10-1;
	TIM_TimeBaseStructure.TIM_Prescaler		= 7200-1;
	TIM_TimeBaseStructure.TIM_CounterMode	= TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

	TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE); //开定时器中断
	TIM_Cmd(TIM6, ENABLE);
}

/**
 * @brief	配置中断优先级
 * @param\n	
 * @retval\n	NULL
 * @remarks	注：一共16个优先级，分为抢占式和响应式。两种优先级所占的数量由此代码确定，
NVIC_PriorityGroup_x可以是0、1、2、3、4，分别代表抢占优先级有1、2、4、8、16个和响应优先级有16、8、4、2、1个。
规定两种优先级的数量后，所有的中断级别必须在其中选择，抢占级别高的会打断其他中断优先执行，
而响应级别高的会在其他中断执行完优先执行。
 */
void NVIC_Configuration(void)
{ 
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Set the Vector Table base address at 0x08000000 */
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);

	/* Configure the Priority Group to 2 bits */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	/* Enable the TIM5 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel					 = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	 //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority		 = 0;		 //响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd				 = ENABLE;
	// NVIC_Init(&NVIC_InitStructure);

	/* Enable the TIM5 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel					 = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	 //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority		 = 0;		 //响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd				 = ENABLE;
	// NVIC_Init(&NVIC_InitStructure);

	/* Enable the TIM5 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel					 = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd				 = ENABLE;
	// NVIC_Init(&NVIC_InitStructure);

	/* Enable the TIM5 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel					 = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd				 = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel					 = TIM6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority		 = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd				 = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel					 = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority		 = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd				 = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief	从内部FLASH读取校准信息，以及配置信息
 * @param\n	
 * @retval\n	NULL
 * @remarks	
 */

void FLASH_Configuration()
{
	uint32_t i;
	ReadFlash(FLASH_PAGE_START, 
		(uint32_t*)&(g_env), 
		sizeof(struct project_env));
	if(g_env.flag != 0xaabbccdd) 
	{
		TC_Adj();
		tp_getadj(&g_env.adj_tp);

		g_env.flag = 0xaabbccdd;
		WriteFlash(FLASH_PAGE_START, 
			(uint32_t*)&(g_env), 
			sizeof(struct project_env));
	}
	else {
		tp_setadj(&g_env.adj_tp);
	}
}



/****************************************************************************
数模转换部分
*****************************************************************************/

/**
 * @brief	制定AD通道获取AD数据
 * @param	
 	chx = 0 电池，chx = 1 功率控制
 * @retval\n	NULL
 * @remarks	
 */

uint16_t GetAD(uint8_t chx)
{
	uint32_t i, k, start = 0;
	uint32_t tmp;
	float ave;

	if(chx == 0)
		start = 0;
	else if(chx == 1)
		start = 1;
	for(k = 0, i = start;i < 400;/*ADC_BUF;*/i+=2) {
		g_adc[k] = ADCConvertedValue[i];
		k++;
	}

	tmp = 0;
	for(i = 0;i < 200;i++) {
		tmp += g_adc[i];
	}
	ave = (float)tmp / 200;

	return (uint16_t)ave;
}


//计算10的x次方需要的数组，x = 浮点型，小数部分(0 - 0.9999)，有效范围-99.99dBm - +99.99dBm
volatile const float c10_10_0[10] = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000};
volatile const float c10_0_0001[10] = {1.000000 , 1.000230 , 1.000461 , 1.000691 , 1.000921 , 1.001152 , 1.001383 , 1.001613 , 1.001844 , 1.002074 };
volatile const float c10_0_001[10]  = {1.000000 , 1.002305 , 1.004616 , 1.006932 , 1.009253 , 1.011579 , 1.013911 , 1.016249 , 1.018591 , 1.020939 };
volatile const float c10_0_01[10]   = {1.000000 , 1.023293 , 1.047129 , 1.071519 , 1.096478 , 1.122018 , 1.148154 , 1.174898 , 1.202264 , 1.230269 };
volatile const float c10_0_1[10]    = {1.000000 , 1.258925 , 1.584893 , 1.995262 , 2.511886 , 3.162278 , 3.981072 , 5.011872 , 6.309573 , 7.943282};
/**
 * @brief	计算dbm所对应的比例系数
 * @param	dbm 0-10.00
 * @retval\n	比例系数
 * @remarks	
 */
float DbmToScale(float dbm)
{
	int a0, a1, a2;
	int b0 = 0;
	float tmp;
	int flag =0;

	if(dbm < 0) {
		dbm = -dbm;
		flag = 1;
	}
	if(dbm >= 10) {
		//dbm =10;
		b0 = (int)(dbm / 10);
		dbm = dbm - b0*10;
	}
	a0 = (int)dbm;
	dbm *=10;
	a1 = (int)dbm;
	dbm *=10;
	a2 = (int)dbm;
	dbm *=10;

	a2 = a2 - a1 * 10;
	a1 = a1 - a0 * 10;

	tmp = (float)(c10_0_1[a0]*c10_0_01[a1]*c10_0_001[a2]*c10_10_0[b0]);
	if(flag)
		tmp = 1/tmp;
	return tmp;
}

/****************************************************************************
外围器件控制部分
*****************************************************************************/
/**
* @brief	红光控制
* @param	1：点亮、0：关闭
* @retval	0:abc
* @retval	1:23
*/
void Ctrl_RedLight(u8 v)
{
	if(v == 1) {
		GPIO_SetBits(GPIO_CTRL_PORT_C, GPIO_KEY_RED_CON);
		//debugtxt(100, 0, "1", -1);
	}
	else {
		GPIO_ResetBits(GPIO_CTRL_PORT_C, GPIO_KEY_RED_CON);
		// debugtxt(100, 0, "0", -1);
	}
}

/**
* @brief	选择波长，同时使能相应波长的定时器和端口
* @param	Wavelength_Selection_state 对应下列WL_xxx波长宏
* @retval\n	
* @remarks	其中该函数不对WL_RED做处理，红光处理在Ctrl_RedLight内
*/
#define WL_OFF  0
#define WL_1310 1
#define WL_1490 2
#define WL_1550 3
#define WL_RED  4
void Ctrl_Wavelength(u8 Wavelength_Selection_state)	  //Operating_Mode  0 = CW、 1 = 270Hz、2 = 1KHz、3 = 2KHz  连续光/脉冲光选择
{
	//Wavelength_Selection_state   0 = 关闭状态 1 = 1310nm 2 = 1495nm 3 = 1550nm 4 = 红光
	switch(Wavelength_Selection_state)				
	{
	case   WL_OFF:	  // 0 = 关闭光源
		TIM_Cmd(TIM3, DISABLE);	   //1310nm
		TIM_Cmd(TIM4, DISABLE);	   //1490nm
		TIM_Cmd(TIM5, DISABLE);	   //1550nm
		GPIO_ResetBits(GPIO_CTRL_PORT_C, GPIO_KEY_1310_CON);
		GPIO_ResetBits(GPIO_CTRL_PORT_C, GPIO_KEY_1490_CON);
		GPIO_ResetBits(GPIO_CTRL_PORT_C, GPIO_KEY_1550_CON);

		break;

	case	WL_1310:	 // 1 = 1310nm	PC1			
		TIM_Cmd(TIM4, DISABLE);
		TIM_Cmd(TIM5, DISABLE);
		TIM_Cmd(TIM3, ENABLE);
		GPIO_ResetBits(GPIO_CTRL_PORT_C, GPIO_KEY_1490_CON);
		GPIO_ResetBits(GPIO_CTRL_PORT_C, GPIO_KEY_1550_CON);

		break;		

	case	WL_1490:	 // 2 = 1490nm	PC2
		TIM_Cmd(TIM3, DISABLE);			
		TIM_Cmd(TIM5, DISABLE);
		TIM_Cmd(TIM4, ENABLE);
		GPIO_ResetBits(GPIO_CTRL_PORT_C, GPIO_KEY_1310_CON);
		GPIO_ResetBits(GPIO_CTRL_PORT_C, GPIO_KEY_1550_CON);

		break;

	case	WL_1550:	 // 3 = 1550nm	PC3
		TIM_Cmd(TIM3, DISABLE);
		TIM_Cmd(TIM4, DISABLE);
		TIM_Cmd(TIM5, ENABLE);
		GPIO_ResetBits(GPIO_CTRL_PORT_C, GPIO_KEY_1310_CON);
		GPIO_ResetBits(GPIO_CTRL_PORT_C, GPIO_KEY_1490_CON);

		break; 
	case	WL_RED:	 // 4 = 红光
		break;
	default:
		break;     		 
	}
}

/*****************************模式选择模拟开关控制端口********************************
* Function Name  : Ctrl_Operating_Mode	  BY Yu Jingxiong  2011.11.15
* Description    : 在选定了波长的前提下Wavelength_Selection_state为1、2、3，控制激光器输出模式Operating_Mode
对定时器的PWM脉冲频率进行控制 ，即对各个定时器的计数值进行设置。。
**************************************************************************************/
/**
* @brief	设定非可见光输出模式
* @param	Operating_Mode指定输出模式，OPM_xxx
* @retval\n	
* @remarks	
*/
#define OPM_CW 0		//连续以及其他脉冲输出模式
#define OPM_270 1
#define OPM_1K 2
#define OPM_2K 3		
#define OPM_OFF 0x80	//Bit7被置位表示暂时关闭光源输出
void Ctrl_Operating_Mode(u8 Operating_Mode)	 //Operating_Mode  0 = CW、 1 = 270Hz、2 = 1KHz、3 = 2KHz  连续光/脉冲光选择
{										     //Wavelength_Selection_state   0 = 关闭状态 1 = 1310nm 2 = 1495nm 3 = 1550nm 4 = 红光

	switch(Operating_Mode)				
	{
	case   0:	  // 0 = CW、
		if(( Wavelength_Selection_state == 0 )||( Wavelength_Selection_state == 4 ))
			;   //添加报错显示函数，关闭状态和红光 不可选模式
		else if( Wavelength_Selection_state == 1 )	  //1310波长
		{
			TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM3, DISABLE);	
			TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM4, DISABLE);	
			TIM_ITConfig(TIM5, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM5, DISABLE);				
			GPIO_SetBits(GPIO_CTRL_PORT_C, GPIO_KEY_1310_CON);
			GPIO_ResetBits(GPIO_CTRL_PORT_C, GPIO_KEY_1490_CON);
			GPIO_ResetBits(GPIO_CTRL_PORT_C, GPIO_KEY_1550_CON);
		}

		else if( Wavelength_Selection_state == 2 )	  //1490波长
		{
			TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM3, DISABLE);	
			TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM4, DISABLE);	
			TIM_ITConfig(TIM5, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM5, DISABLE);				
			GPIO_SetBits(GPIO_CTRL_PORT_C, GPIO_KEY_1490_CON);
			GPIO_ResetBits(GPIO_CTRL_PORT_C, GPIO_KEY_1310_CON);
			GPIO_ResetBits(GPIO_CTRL_PORT_C, GPIO_KEY_1550_CON);
		}

		else if( Wavelength_Selection_state == 3 )	  //1550波长
		{
			TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM3, DISABLE);	
			TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM4, DISABLE);	
			TIM_ITConfig(TIM5, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM5, DISABLE);				
			GPIO_SetBits(GPIO_CTRL_PORT_C, GPIO_KEY_1550_CON);
			GPIO_ResetBits(GPIO_CTRL_PORT_C, GPIO_KEY_1490_CON);
			GPIO_ResetBits(GPIO_CTRL_PORT_C, GPIO_KEY_1310_CON);
		}

		break;

	case	1:	 // 1 = PL 270Hz、
		if(( Wavelength_Selection_state == 0 )||( Wavelength_Selection_state == 4 ))
			;   //添加报错显示函数，关闭状态和红光 不可选模式
		else if( Wavelength_Selection_state == 1 )	  //1310波长
		{
			TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM3, DISABLE);
			TIM_Period1310 = TIM_Period[0];
			TIM3_Init( TIM_Period1310);
			TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
			TIM_Cmd(TIM3, ENABLE);	
		}

		else if( Wavelength_Selection_state == 2 )	  //1490波长
		{
			TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM4, DISABLE);
			TIM_Period1490 = TIM_Period[0];
			TIM4_Init( TIM_Period1490);
			TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
			TIM_Cmd(TIM4, ENABLE);
		}

		else if( Wavelength_Selection_state == 3 )	  //1550波长
		{
			TIM_ITConfig(TIM5, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM5, DISABLE);
			TIM_Period1550 = TIM_Period[0];
			TIM5_Init( TIM_Period1550);
			TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
			TIM_Cmd(TIM5, ENABLE);
		}
		break;		

	case	2:	 // 2 = PL 1KHz、
		if(( Wavelength_Selection_state == 0 )||( Wavelength_Selection_state == 4 ))
			;   //添加报错显示函数，关闭状态和红光 不可选模式
		else if( Wavelength_Selection_state == 1 )	  //1310波长
		{
			TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM3, DISABLE);
			TIM_Period1310 = TIM_Period[1];
			TIM3_Init( TIM_Period1310);
			TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
			TIM_Cmd(TIM3, ENABLE);
		}

		else if( Wavelength_Selection_state == 2 )	  //1490波长
		{
			TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM4, DISABLE);
			TIM_Period1490 = TIM_Period[1];
			TIM4_Init( TIM_Period1490);
			TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
			TIM_Cmd(TIM4, ENABLE);
		}

		else if( Wavelength_Selection_state == 3 )	  //1550波长
		{
			TIM_ITConfig(TIM5, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM5, DISABLE);
			TIM_Period1550 = TIM_Period[1];
			TIM5_Init( TIM_Period1550);
			TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
			TIM_Cmd(TIM5, ENABLE);
		}     	
		break;

	case	3:	 // 3 = PL 2KHz
		if(( Wavelength_Selection_state == 0 )||( Wavelength_Selection_state == 4 ))
			;   //添加报错显示函数，关闭状态和红光 不可选模式
		else if( Wavelength_Selection_state == 1 )	  //1310波长
		{
			TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM3, DISABLE);
			TIM_Period1310 = TIM_Period[2];
			TIM3_Init( TIM_Period1310);
			TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
			TIM_Cmd(TIM3, ENABLE);
		}

		else if( Wavelength_Selection_state == 2 )	  //1490波长
		{
			TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM4, DISABLE);
			TIM_Period1490 = TIM_Period[2];
			TIM4_Init( TIM_Period1490);
			TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
			TIM_Cmd(TIM4, ENABLE);
		}

		else if( Wavelength_Selection_state == 3 )	  //1550波长
		{
			TIM_ITConfig(TIM5, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM5, DISABLE);
			TIM_Period1550 = TIM_Period[2];
			TIM5_Init( TIM_Period1550);
			TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
			TIM_Cmd(TIM5, ENABLE);
		} 
		break; 

	default:
		break;     		 
	}

	//关闭光源
	if(Operating_Mode & 0x80) {
		TIM_Cmd(TIM3, DISABLE);	
		TIM_Cmd(TIM4, DISABLE);	
		TIM_Cmd(TIM5, DISABLE);	

		GPIO_ResetBits(GPIO_CTRL_PORT_C, GPIO_KEY_1310_CON);
		GPIO_ResetBits(GPIO_CTRL_PORT_C, GPIO_KEY_1490_CON);
		GPIO_ResetBits(GPIO_CTRL_PORT_C, GPIO_KEY_1550_CON);
	}
}
void ClearFastSwitch()
{
	
}
/**
 * @brief	功率控制
 * @param  	传递参数结构
 * @retval\n	NULL
 * @remarks	
 */

void Ctrl_Power(struct ctrl_param *v)
{
	float tmpdbm;
	char strout[256];
	// sprintf(strout, "fast %d set %d", fastsw.set, g_power.set);
	// gl_text(0, 150, strout, -1);
	if(fastsw.set == g_power.set) {
		//提高调节速度
		//static uint16_t lastset = -1000;
		uint16_t waveIndex;
		uint16_t modeIndex;
		static uint16_t lastWaveIndex = 0;
		static uint16_t lastModeIndex = 1;

		uint16_t *(adc_dac_code)[2][3][5] = {
			//寻址索引 *adc_dac_code[adc_or_dac][waveIndex][modeIndex & 0x80];
			//adc adc_or_dac = 0
			&fastsw.ch1_adc_cw, &fastsw.ch1_adc_270, &fastsw.ch1_adc_1k, &fastsw.ch1_adc_2k, &fastsw.ch1_adc_off, 
			&fastsw.ch2_adc_cw, &fastsw.ch2_adc_270, &fastsw.ch2_adc_1k, &fastsw.ch2_adc_2k, &fastsw.ch2_adc_off, 
			&fastsw.ch3_adc_cw, &fastsw.ch3_adc_270, &fastsw.ch3_adc_1k, &fastsw.ch3_adc_2k, &fastsw.ch3_adc_off, 

			//dac adc_or_dac = 1
			&fastsw.ch1_dac_cw, &fastsw.ch1_dac_270, &fastsw.ch1_dac_1k, &fastsw.ch1_dac_2k, &fastsw.ch1_dac_off, 
			&fastsw.ch2_dac_cw, &fastsw.ch2_dac_270, &fastsw.ch2_dac_1k, &fastsw.ch2_dac_2k, &fastsw.ch2_dac_off, 
			&fastsw.ch3_dac_cw, &fastsw.ch3_dac_270, &fastsw.ch3_dac_1k, &fastsw.ch3_dac_2k, &fastsw.ch3_dac_off
		};
		
		
		waveIndex   = Wavelength_Selection_state - 1;
		if(Operating_Mode & 0x80) {
			modeIndex = 4;
		}
		else {
			modeIndex = Operating_Mode & 0x7f;
		}
		

		
		*adc_dac_code[0][lastWaveIndex][lastModeIndex] = g_power.adc;
		*adc_dac_code[1][lastWaveIndex][lastModeIndex] = g_power.dac;
		lastWaveIndex = waveIndex;
		lastModeIndex = modeIndex;
		
		// sprintf(strout, "%d %d %d %d", modeIndex, waveIndex, *adc_dac_code[0][waveIndex][modeIndex], *adc_dac_code[1][waveIndex][modeIndex]);
		// gl_text(90, 0, strout, -1);
		

		//if(*adc_dac_code[0][waveIndex][modeIndex] != 0) {
		if(fastsw.enable == 1) {
			g_power.adc = *adc_dac_code[0][waveIndex][modeIndex];
			g_power.dac = *adc_dac_code[1][waveIndex][modeIndex];
			
			goto _OutPut;
		}
	}
	
	else {
		fastsw.set = g_power.set;
		fastsw.enable = 1;
	}
		
	
	/*3.3V分成4096份，每份0.805mV
	Vad = AD * 0.805mV
	Vad = P * R * K * A/B = AD * 0.805
	P:功率大小，dbm->power, 用函数DbmToScale转换dbm到power
	R:电阻大小，1000欧
	K:光响应度，(1550nm)约0.986，(1310nm)约0.895其他波长待定
	其实这个值不用太在意，它也不能说是响应度，只是个校正偏差而已，有了自动校准以后，K的值意义不大
	A/B:光环形器分光比，这里是50:50所以可以舍去
	AD = P * R * K / 0.805 = P * 1224.84472 = DbmToScale(v->set) * 1224.84472
	*/
	//scale = DbmToScale((float)(v->set/1000);
	/*if(Wavelength_Selection_state == 1)
// 	v->set -= 220;//后期通过响应度来修改*/

// 	if(Wavelength_Selection_state == WL_1310) {
// 		if(Operating_Mode == OPM_CW) {
// 			tmpdbm = (float)(v->set/1000.0) + g_adj_power._adc;
// 		}
// 		else if(Operating_Mode == OPM_270) {
// 			tmpdbm = (float)(v->set/1000.0) + g_adj_power._dac;
// 		}
// 		else if(Operating_Mode == OPM_1K) {
// 			tmpdbm = (float)(v->set/1000.0) + g_adj_power._1310_1k;
// 		}
// 		else if(Operating_Mode == OPM_2K) {
// 			tmpdbm = (float)(v->set/1000.0) + g_adj_power._1310_2k;
// 		}
// 	}
// 	else if(Wavelength_Selection_state == WL_1490 ) {
// 	}
// 	else if(Wavelength_Selection_state == WL_1550) {
// 		if(Operating_Mode == OPM_CW) {
// 			tmpdbm = (float)(v->set/1000.0) + g_adj_power._1550cw;
// 		}
// 		else if(Operating_Mode == OPM_270) {
// 			tmpdbm = (float)(v->set/1000.0) + g_adj_power._1550_270;
// 		}
// 		else if(Operating_Mode == OPM_1K) {
// 			tmpdbm = (float)(v->set/1000.0) + g_adj_power._1550_1k;
// 		}
// 		else if(Operating_Mode == OPM_2K) {
// 			tmpdbm = (float)(v->set/1000.0) + g_adj_power._1550_2k;
// 		}
// 	}
// 	else {
// 		//tmp.set = v->set;
// 		tmpdbm = (float)(v->set/1000.0);
// 	}
	tmpdbm = (float)(v->set/1000.0);// + g_adj_power._1550_1k;
	
// 	//v->dac = (uint16_t)(DbmToScale((float)(v->set/1000.0)) * 2460);//2460只是个大概的数，方便快速调节
// 	
// 	if(Wavelength_Selection_state == WL_1550) {
// 		//v->adc = (uint16_t)(DbmToScale((float)(v->set/1000.0)) * 1224.84472);
// 		//v->adc = (uint16_t)(DbmToScale(tmpdbm) * 1224.84472);
// 		v->adc = (uint16_t)((pow(10, (float)tmpdbm/10))*g_adj_power._1550cw*10);
// 	}
// 	else if(Wavelength_Selection_state == WL_1310) {
// 		//v->adc = (uint16_t)(DbmToScale((float)(v->set/1000.0)) * (1000*1.040/0.805));
// 		//v->adc = (uint16_t)(DbmToScale(tmpdbm) * (1000*1.040/0.805));
// 		v->adc = (uint16_t)((pow(10, (float)tmpdbm/10))*g_adj_power._adc*10);
// 	}
// 	else {
// 		//v->adc = (uint16_t)(DbmToScale(tmpdbm) * 1224.84472);
// 		v->adc = (uint16_t)((pow(10, (float)tmpdbm/10))*0);
// 	}
	
// 	
	//以上计算v->adc全部忽略
	v->adc = (uint16_t)((pow(10, (float)tmpdbm/10))*g_adj_power._adc*10);
	v->dac = (uint16_t)((pow(10, (float)tmpdbm/10))*g_adj_power._dac*10);


	
	sprintf(strout, "v->dac %4.4d", v->dac);
	debugtxt(0, 12, strout, -1);
	sprintf(strout, "v->adc %4.4d", v->adc);
	debugtxt(0, 24, strout, -1);
	sprintf(strout, "v->set dbm %3.3f", (float)(v->set / 1000.0));
	debugtxt(0, 36, strout, -1);

	if(v->adc > 4095) {
		v->adc = 4095;
	}
	else if(v->adc < 0){
		v->adc = 0;
	}
	if(v->dac > 4095) {
		v->dac = 4095;
	}
	else if(v->dac < 0){
		v->dac = 0;
	}
_OutPut:
	DAC_SoftwareTriggerCmd(DAC_Channel_1, DISABLE);  
	DAC_SetChannel1Data(DAC_Align_12b_R, v->dac);
	DAC_SoftwareTriggerCmd(DAC_Channel_1, ENABLE);	
	
	// sprintf(strout, "%d %d", v->adc, v->dac);
	// gl_text(90, 10, strout, -1);
#if 0
	LCD_Power_Control_Selection_Ex(55, 90, ((uint16_t)((int32_t)v->set / 1000)), White, Grey);	
#endif
	g_autoctrlpower_en = 1;


	// sprintf(strout, "ad %d da %d", g_power.adc, g_power.dac);
	// gl_text(0, 170, strout, -1);
	// 	g_red_mode++;
	// 	if(g_red_mode >=3)
	// 		g_red_mode = 0;
	// 	g_red_onoff = ~g_red_onoff;
	// 	LCD_RedLight_Show(9, 15, g_red_onoff);
}

/**
 * @brief	功率自动校正
 * @param\n	
 * @retval\n	NULL
 * @remarks	调节时间间隔根据实际输出值与设置值的偏差大小而定，
 偏差越大调节时间越小，最小间隔200ms，最大间隔1000ms。当功率与
 设置值相差无几时，AutoCtrlPower较少干预功率值，有利于功率稳定
 */

void AutoCtrlPower()
{
	uint8_t strout[256];
	uint16_t ad, flag = 0, fadj = 0;
	static uint16_t adjust_time  = 800;
	static uint16_t balance_times = 0;

	//根据偏差的多少，设定下次校正时间和校正幅度
	if(g_adjust_ms >= adjust_time ) {
		g_adjust_ms = 0;
		if(g_autoctrlpower_en == 0 && Operating_Mode & 0x80) {
			return ;
		}
		ad = GetAD(1);//读取ad值，200次取平均
		sprintf(strout, "GetAD %4.4d", ad);
		debugtxt(0, 48, strout, -1);

		if(ad - g_power.adc > 50) {
			g_power.dac -= 50;//校正幅度-50
			fadj = 1;
			adjust_time  = 300;//300ms后再次校正
			balance_times = 0;
		}
		else if(g_power.adc - ad > 50) {
			g_power.dac += 50;
			fadj = 1;
			adjust_time  = 300;
			balance_times = 0;
		}
		else if(ad - g_power.adc > 6) {
			g_power.dac -= 5;
			fadj = 1;
			adjust_time  = 200;
			balance_times = 0;


		}
		else if(g_power.adc - ad> 6) {
			g_power.dac += 5;
			fadj = 1;
			adjust_time  = 200;
			balance_times = 0;


		}
		else if(ad - g_power.adc > 2) {
			g_power.dac -= 1;
			fadj = 1;
			adjust_time  = 1000;
			balance_times = 0;
			if(Operating_Mode ==OPM_270) {
				g_autoctrlpower_en = 0;
			}

		}
		else if(g_power.adc - ad> 2) {
			g_power.dac += 1;
			fadj = 1;
			adjust_time  = 1000;
			balance_times = 0;
			if(Operating_Mode ==OPM_270 ) {
				g_autoctrlpower_en = 0;
			}
		}
		else {
			if(balance_times++ > 2) {
				g_autoctrlpower_en = 0;
			}
		}

		if(fadj) {
			if(g_power.dac > 4095) {
				g_power.dac = 4095;
			}
			DAC_SoftwareTriggerCmd(DAC_Channel_1, DISABLE);  
			DAC_SetChannel1Data(DAC_Align_12b_R, g_power.dac);
			DAC_SoftwareTriggerCmd(DAC_Channel_1, ENABLE);

		}
		//sprintf(strout, "adc %4.4d, dac %4.4d mode %d wave %d", g_power.adc, g_power.dac, Operating_Mode, Wavelength_Selection_state);
		//debugtxt(0, 60, strout, -1);
		//gl_text(0, 60, strout, -1);
	}
}

/*****************************模式选择模拟开关控制端口********************************
* Function Name  : Ctrl_Timing_Device	  BY Yu Jingxiong  2011.11.25
* Description    : //Timer_State  对应定时器关机设置为5min  ，10min  ，15min  ，30min  ，60min 状态
TIM2设置为100ms周期，3000 ， 6000  ， 9000  ， 18000 ，  36000
Timer_State：0 关闭，1       ，2     ，3       ，4      ，5  
**************************************************************************************/
#define TM_OFF   0
#define TM_5MIN  1
#define TM_10MIN 2
#define TM_15MIN 3
/**
 * @brief	定时关机
 * @param\n	
 * @retval\n	NULL
 * @remarks	
 */

void Ctrl_Timing_Device(u8 Timer)	  
{
	switch(Timer)				
	{
	case   0:	  // 0 = 定时器关闭
		Timer_Counter = 0;	  //调整状态之后保证每次计数从零开始
		Timer_State  = TM_OFF;
		break;
	case   2:	  // 1 = 定时器打开
		Timer_Counter = 0;
		Timer_State  = TM_10MIN;
		break;
	default:
		break;
	}
}



/**
 * @brief	绘制TS100主界面
 * @param\n	
 * @retval\n	NULL
 * @remarks	在该函数里进行一些全局变量的初始化
 */

void LCD_DrawMain(void)
{
	uint16_t x, y;

	//显示LOGO
	// 	for( x=0; x < 320; x++ )		//上边界蓝条宽34  下边界蓝条宽49
	// 		for( y=0; y < 240; y++ )
	// 			gl_setpoint(x, y, 0x22f2);	//屏幕主色调gray
	//LCD_Clear(Black);
	LCD_Clear(RGB16(255, 0, 0));
#if 0
	LCD_FLSAH_DrawPicture(38, 91, 38+243-1, 91+57-1, (uint8_t*)gImage_logo);
#endif
	//Delay_ms(2000);
	// 	gl_setarea(0, 0, 319, 239);
	// 	for( x=0; x < 320; x++ )		//上边界蓝条宽34  下边界蓝条宽49
	// 		for( y=0; y < 240; y++ )
	// 			gl_setpoint(x, y, 0x22f2);	//屏幕主色调gray
	gl_clear(0, 0, 320, 240, 0x22f2);

	
	//初始化波长
	Wavelength_Selection_state = WL_1310;
	if(g_adj_power._1310_en == 0) {
		if(g_adj_power._1490_en) {
			Wavelength_Selection_state = WL_1490;
		}
		else {
			Wavelength_Selection_state = WL_1550;
		}
	}
	Ctrl_Wavelength( Wavelength_Selection_state); //Wavelength_Selection_state   0 = 关闭状态 1 = 1310nm 2 = 1495nm 3 = 1550nm 4 = 红光
#if 0
	LCD_Wavelength_Selection_Ex( 170, 200 , Wavelength_Selection_state , g_sm_mm_color, Grey );
#endif

	//初始化输出频率
	Operating_Mode = OPM_CW;
	Ctrl_Operating_Mode( Operating_Mode);	     //Operating_Mode  0 = CW、 1 = 270Hz、2 = 1KHz、3 = 2KHz  连续光/脉冲光选择
	// LCD_OperatMode_Selection( 15, 200, Operating_Mode, g_sm_mm_color, Grey );

	
	//初始化功率
	// fastsw.set 和 g_power.set 初始化要避开
	memset((void*)&fastsw.set, 0, sizeof(struct fast_switch) - sizeof(int32_t));
	fastsw.set = (int32_t)(-10000);
	fastsw.enable = 0;
	g_power.lastset = (int32_t)(-10000);
	g_power.set = (int32_t)(-10000);
	Ctrl_Power(&g_power);
	
	
	// 1.3.1~1.3.2 版本 由于加上波长快速切换，Ctrl_Power 内部有缺陷，第一次执行Ctrl_Power(&g_power);
	// 功率值一直在跳动，
	// 当切换成其他功率值再切换回-10dBm功率输出稳定
	// Ctrl_Power 波长快速切换并不能实现光源off - CW/270/1K/2K之间的快速切换，以后想办法修订
	g_power.set = (int32_t)(-9000);
	Ctrl_Power(&g_power);
	g_power.set = (int32_t)(-10000);
	Ctrl_Power(&g_power);
	/////////////////////////////////////////
	
	
	//初始化定时关机
	Timer_State = TM_10MIN;//TM_OFF;
	Ctrl_Timing_Device( Timer_State );
	// LCD_Timing_Display( 120, 12 , Timer_State);

	//初始化红光
	g_red_onoff = 0;
	// if(g_adj_power._650_en == 1)
	// 	LCD_RedLight_Show(9, 15, g_red_onoff);

	//初始化电池电量
	g_batter_delay = -1;//立即检测，而不是100ms后
	ProChargerMonitor();

	// n();	//校准触摸屏
	// LCD_Batter_Show(0, 0, 6/*LEVEL_4*/);//为了解决初始化时候电池判别次数不够，不显示电池量	
}
void Delay(uint32_t time)
{
	for (; time!= 0; time--);
}

/**
 * @brief	接通一键开机电源
 * @param\n	
 * @retval\n	NULL
 * @remarks	
 */

void TurnOnPower()
{
	int i = 0, isDown = 0;

	// 	//总电源使能，打开
	GPIO_SetBits(GPIO_CTRL_PORT_B, GPIO_SYSPWR_ONOFF);
#if 0
	Delay_ms(10); 
	//开机按键是否按下
	i = 0;
	while(i++ < 5) {//用户长按800ms，每200ms去抖动
		Delay_ms(200);
		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == 1) {
			isDown = 1;
		}
		else {
 			isDown = 0;
 			break;
		}
	}

	//if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == 1) 
	if(isDown)
	{ 		
	}
	else {
		GPIO_ResetBits(GPIO_CTRL_PORT_B, GPIO_SYSPWR_ONOFF);//关闭总电源
		powerDownDelayCnt = 0;
		i = 0;
		while(i++<10) {
			printf("power off\n");
			Delay_ms(1000);
		}
	}
#endif
}

/**
 * @brief	关闭一键开机电源
 * @param\n	
 * @retval\n	NULL
 * @remarks	
 */

void TurnOffPower()
{
	//GPIO_ResetBits(GPIO_CTRL_PORT_B, GPIO_SYSPWR_ONOFF);
	//关闭液晶屏，提示用户关机成
	if(powerDownDelayCnt >= 6) {
		g_power_down = 1;
		g_red_mode = 0;
		Ctrl_RedLight(0);
		LCD_Clear(Black);
		Delay_ms(1000);
		LCD_SetBacklight(0x03);		
		//TODO :关闭LCD背光和电源
		GPIO_SetBits(GPIO_PORT_LCD, GPIO_LCD_OFF);//关显示屏操作 
		// 		delayMs(10);//延迟一小段时间，等待操作者松开按键
		// 		Delay_ms(10
		GPIO_ResetBits(GPIO_CTRL_PORT_B, GPIO_SYSPWR_ONOFF); //关机 CHECK2置0 清标志位

	}	
}


/*****************************************************************************

*****************************************************************************/
/**
 * @brief	响应波长设置
 * @param\n	
 * @retval\n	NULL
 * @remarks	
响应用户操作部分\n
6个按键，按键布局\n
X  Y  Z\n
A  B  C\n
 */

void UI_ProWavelength()
{
	int tmpWave;

	if(KeyDown(GPIOA, KEY_B)) {
		int wave[3] = {WL_1310, WL_1490, WL_1550};
		int i, find;
		static int index = 0;


		if(g_adj_power._1310_en == 0) {
			wave[0] = 0;
		}
		if(g_adj_power._1490_en == 0) {
			wave[1] = 0;
		}
		if(g_adj_power._1550_en == 0) {
			wave[2] = 0;
		}

		find = 0;
		for(i = index+1;i < 3;++i) {
			if(wave[i] != 0 && wave[i] != Wavelength_Selection_state) {
				Wavelength_Selection_state = wave[i];
				index = i;
				find = 1;
				break;
			}
		}
		if(find == 0) {
			for(i = 0;i < index;++i) {
				if(wave[i] != 0 && wave[i] != Wavelength_Selection_state) {
					Wavelength_Selection_state = wave[i];
					index = i;
					find = 1;
					break;
				}
			}
		}

		if(find == 0 || index >= 3)
			index = -1;		

		Ctrl_Operating_Mode( Operating_Mode);
		Ctrl_Wavelength( Wavelength_Selection_state); //Wavelength_Selection_state   0 = 关闭状态 1 = 1310nm 2 = 1495nm 3 = 1550nm 4 = 红光
		// LCD_Wavelength_Selection_Ex( 170, 200 , Wavelength_Selection_state , g_sm_mm_color, Grey );	//波长显示切换
		Ctrl_Power(&g_power);
	}
}

/**
 * @brief	响应波长设置
 * @param\n	
 * @retval\n	NULL
 * @remarks	
响应用户操作部分\n
6个按键，按键布局\n
X  Y  Z\n
A  B  C\n
 */

void MoveTest()
{
	WM_MoveTo(hFrame, 200, 50);

	while(WM_GetWindowOrgX(hFrame) > 10) {
		if(WM_GetWindowOrgX(hFrame) > 50 ) {
			WM_MoveWindow(hFrame, -30, 0);
		}
		else {
			WM_MoveWindow(hFrame, -5, 0);
		}
		GUI_Exec();
	}
	while(WM_GetWindowOrgX(hFrame) < 30) {
		WM_MoveWindow(hFrame, 5, 0);
		GUI_Exec();
	}
	
}
void UI_ProUp()
{
	int tmpWave;

	if(KeyDown_Ex(GPIOA, KEY_B)) {

	//if (GPIO_ReadInputDataBit(GPIOA, KEY_B) == 0) {
		g_pwm_high += 500;
		if (g_pwm_high < 0) {
			g_pwm_high = 0;
		}
		else if (g_pwm_high > 65536) {
			g_pwm_high = 65536;
		}
		Ctrl_RedLight(1);	
		// Ctrl_PWM(PWM_CH3, g_pwm_high, 3600);

		
		WM_MoveWindow(hFrame, 30, 0);
		MoveTest();
		// WM_InvalidateWindow(WM_GetClientWindow(hFrame));
	}
	
}
void UI_ProDown()
{
	int tmpWave;

	if(KeyDown_Ex(GPIOA, KEY_C)) {
		g_pwm_high -= 500;
		if (g_pwm_high < 0) {
			g_pwm_high = 0;
		}
		else if (g_pwm_high > 65536) {
			g_pwm_high = 65536;

		}
		Ctrl_RedLight(0);	
		// Ctrl_PWM(PWM_CH3, g_pwm_high, 3600);


		
		WM_MoveWindow(hFrame, -30, 0);
		// WM_InvalidateWindow(WM_GetClientWindow(hFrame));
	}
}
void UI_ProAdj()
{
	if(KeyDown(GPIOA, KEY_Z)) {
		TC_Adj();
		tp_getadj(&g_env.adj_tp);

		g_env.flag = 0xaabbccdd;
		WriteFlash(FLASH_PAGE_START, 
			(uint32_t*)&(g_env), 
			sizeof(struct project_env));
	}
}
/**
 * @brief	响应功率控制
 * @param\n	
 * @retval\n	NULL
 * @remarks	
 */

void UI_ProPower()
{
	uint8_t flag = 0;
	if(KeyPress(GPIOA, KEY_C)) {
		flag = 1;
		g_power.lastset = g_power.set;
		g_power.set -= 1000;
	}
	//调整功率大小++
	else if(KeyPress(GPIOA, KEY_Z)) {
		flag = 1;
		g_power.lastset = g_power.set;
		g_power.set += 1000;
	}	
	if(flag) {
		if((int32_t)g_power.set > 0) {
			g_power.set = 0;
		}
		else if((int32_t)g_power.set < -19000) {
			g_power.set = (uint32_t)(-19000);
		}
		//memset((void*)&fastsw, 0, sizeof(struct fast_switch));
		memset((void*)&fastsw.set, 0, sizeof(struct fast_switch) - sizeof(int32_t));
		fastsw.enable = 0;
		Ctrl_Power(&g_power);
	}
}


/**
 * @brief	响应模式设置
 * @param\n	
 * @retval\n	NULL
 * @remarks	
 */

void UI_ProMode()
{
	// 	if(KeyDown(GPIOA, KEY_Y)) {
	// 		Operating_Mode++;
	// 		if(Operating_Mode > 3)
	// 			Operating_Mode = 0;
	// 		Ctrl_Operating_Mode( Operating_Mode);	     //Operating_Mode  0 = CW、 1 = 270Hz、2 = 1KHz、3 = 2KHz  连续光/脉冲光选择
			//LCD_OperatMode_Selection( 15, 200, Operating_Mode, g_sm_mm_color, Grey );	//工作模式显示切换
	// 		switch(Operating_Mode) {
	// 		case 0:
	// 			debugtxt(0, 100-24, "cw ", -1);
	// 			break;
	// 		case 1:
	// 			debugtxt(0, 100-24, "270", -1);
	// 			break;
	// 		case 2:
	// 			debugtxt(0, 100-24, "1k ", -1);
	// 			break;
	// 		case 3:
	// 			debugtxt(0, 100-24, "2k ", -1);
	// 			break;
	// 		}
	// 		Ctrl_Power(&g_power);
	// 		if(Operating_Mode == 0 && Timer_State == TM_OFF) {
	// 			hackflag = 1;
	// 			hackval = g_power.set;
	// 		}
	// 	}	
	int i = KeyDown_Ex(GPIOA, KEY_Y);
	int8_t flag = 0;
	//长按mode键1s以上，关闭非可见光源
	//if(i >= 500)
	if (i > 0 && i < 400) {
		Operating_Mode ^= 0x80;
		flag = 1;
		goto _draw;
	}
	//短按mode键60ms以下，打开非可见光源
	//else if (i > 0 && i < 400){
	else if (i >= 500) {
		if(Operating_Mode & 0x80) {//光源已经关闭，恢复打开前模式
			Operating_Mode &= ~0x80;
		}
		else {//非可见光已经打开某模式
			Operating_Mode++;
			if(Operating_Mode > 3)
				Operating_Mode = 0;
		}
_draw:;
		Ctrl_Operating_Mode( Operating_Mode);	     //Operating_Mode  0 = CW、 1 = 270Hz、2 = 1KHz、3 = 2KHz  连续光/脉冲光选择
		// 		if(Operating_Mode & 0x80) {
					// LCD_OperatMode_Selection( 15, 200, Operating_Mode, RGB16(255, 51, 51)/*Red*/, Grey );	//工作模式显示切换
		// 		}
		// 		else {
		// LCD_OperatMode_Selection( 15, 200, Operating_Mode, g_sm_mm_color, Grey );	//工作模式显示切换
		//		}

		switch(Operating_Mode) {
		case 0:
			debugtxt(0, 100-24, "cw ", -1);
			break;
		case 1:
			debugtxt(0, 100-24, "270", -1);
			break;
		case 2:
			debugtxt(0, 100-24, "1k ", -1);
			break;
		case 3:
			debugtxt(0, 100-24, "2k ", -1);
			break;
		}
		Ctrl_Power(&g_power);
		if(Operating_Mode == 0 && Timer_State == TM_OFF) {
			hackflag = 1;
			hackval = g_power.set;
		}
		while(GPIO_ReadInputDataBit(GPIOA, KEY_Y) == 0);
	}
}



/**
 * @brief	响应红光输出和定时关机设置
 * @param\n	
 * @retval\n	NULL
 * @remarks	
 */

void UI_ProRedLight_ShutdownTimer()
{
	int i;
	i = KeyDown_Ex(GPIOA, KEY_A);


	//if(KeyDown_Ex(GPIOA, KEY_A, 1000)){//g_key_timer_100ms > 10) {
	if(i >= 500)
	{
		g_key_timer_100ms = 0;
		if(Timer_State == TM_10MIN)
			Timer_State = TM_OFF;
		else
			Timer_State = TM_10MIN;
		Ctrl_Timing_Device(Timer_State);
		// LCD_Timing_Display( 120, 12 , Timer_State);
		while(GPIO_ReadInputDataBit(GPIOA, KEY_A) == 0);
	}
	else if(i >0 && i < 400 && g_adj_power._650_en == 1)
	{
		g_red_mode++;
		if(g_red_mode >= 3)
			g_red_mode = 0;
		if(g_red_mode == 0)
			g_red_onoff = 0;
		else if(g_red_mode == 1)
			g_red_onoff = 1;
		else if(g_red_mode == 2)
			g_red_onoff = 0;
		Ctrl_RedLight(g_red_onoff);
		// LCD_RedLight_Show(9, 15, g_red_onoff);
	}
}
// double atof_(char* s)
// {
// 	double v=0, k = 0, j = 1;
// 	int sign=1;
// 	while ( *s == ' '  ||  (unsigned int)(*s - 9) < 5u) s++;
// 	switch (*s)
// 	{
// 	case '-':
// 		sign=-1;
// 	case '+':
// 		++s;
// 	}
// 	while ((unsigned int) (*s - '0') < 10u)
// 	{
// 		v=v*10+*s-'0';
// 		++s;
// 	}
// 	if(*s == '.') {
// 		s++;
// 		while ((unsigned int) (*s - '0') < 10u)
// 		{
// 			k=k*10+*s-'0';
// 			++s;
// 			j = j * 10;
// 		}
// 		k /= j;
// 	}
// 	
// 	v = v + k;
// 	return sign==-1?-v:v;
// }

/**
 * @brief	响应后门按键过程
 * @param\n	
 * @retval\n	NULL
 * @remarks	
 */

void UI_ProductionAdjust()
{
	int x, y;
	uint8_t flag = 0;
	uint8_t light = 0xff;
	int32_t strlen;
	uint32_t btnClk = 0;
	struct point loc[10] = {
		(4)*8, (56), (4+10)*8, (56), (4+10+10)*8, (56), (4+10+10+10)*8, (56), 
		(4)*8, (56+24), (4+10)*8, (56+24), (4+10+10)*8, (56+24), (4+10+10+10)*8, (56+24), 
		20, 56+24+24, 
		20, 56+24+24+24, 
	};
	float adjval[8];
	int8_t index = 0, last_index;
	last_index = index;



	adjval[0] = (float)g_adj_power._adc;
	adjval[1] = (float)g_adj_power._dac;
	adjval[2] = (float)g_adj_power._1310_1k;
	adjval[3] = (float)g_adj_power._1310_2k;

	adjval[4] = (float)g_adj_power._1550cw;
	adjval[5] = (float)g_adj_power._1550_270;
	adjval[6] = (float)g_adj_power._1550_1k;
	adjval[7] = (float)g_adj_power._1550_2k;

_Redraw:;
	for( x=0; x < 320; x++ )		//上边界蓝条宽34  下边界蓝条宽49
		for( y=0; y < 240; y++ )
			;//gl_setpoint(x, y, 0x22f2);	//屏幕主色调gray

	//LCD_Clear(0x22f2);
	gl_clear(0, 0, 320, 240, COL_White);
	gl_text((4)*8, (56-24), "ADC", -1);
	gl_text((4+10)*8, (56-24), "DAC", -1);
	gl_text((4+10+10)*8, (56-24), "--", -1);
	gl_text((4+10+10+10)*8, (56-24), "--", -1);

	gl_text(0, (56), "-10", -1);
	sprintf(strout, "%6.2f", (float)(adjval[0]));
	gl_text((4)*8, (56), strout, -1);
	sprintf(strout, "%6.2f", (float)adjval[1]);
	gl_text((4+10)*8, (56), strout, -1);
	sprintf(strout, "%6.2f", (float)adjval[2]);
	gl_text((4+10+10)*8, (56), strout, -1);
	sprintf(strout, "%6.2f", (float)adjval[3]);
	gl_text((4+10+10+10)*8, (56), strout, -1);

	gl_text(0, (56+24), "---", -1);
	sprintf(strout, "%6.2f", (float)adjval[4]);
	gl_text((4)*8, (56+24), strout, -1);
	sprintf(strout, "%6.2f", (float)adjval[5]);
	gl_text((4+10)*8, (56+24), strout, -1);
	sprintf(strout, "%6.2f", (float)adjval[6]);
	gl_text((4+10+10)*8, (56+24), strout, -1);
	sprintf(strout, "%6.2f", (float)adjval[7]);
	gl_text((4+10+10+10)*8, (56+24), strout, -1);

	gl_text(20, 56+24+24, "save", -1);
	gl_text(20, 56+24+24+24, "exit", -1);

	DrawFocus(loc[index].x, loc[index].y, COL_Black);//RGB16(255, 255, 0));
	while(1) {

		if(ProTick1963IsLive())
			;
		ProGet1963State();
		TurnOffPower();
		//if(ProTick1963IsLive()) 
		{
			;//goto _Redraw;
		}
		if(KeyPress(GPIOA, KEY_A)) {
			flag = 1;
			index--;
		}
		else if(KeyPress(GPIOA, KEY_C)) {
			flag = 1;
			index++;
		}
		else if(KeyPress(GPIOA, KEY_C)) {
			flag = 1;
			//adjval[index] -= 0.01;
		}
		// 		else if(KeyPress(GPIOA, KEY_Z)) {
		// 			flag = 1;
		// 			btnClk = 1;
		// 			//while(KeyPress(GPIOA, KEY_Z));
		// 			//adjval[index] += 0.01;
		// 			
		// 		}
		else if(KeyPress(GPIOA, KEY_Z)) {
			flag = 1;
			btnClk = 1;
			if(index < 8) {
				//adjval[index] = 0;	

			}
			else if(index == 8) {
				g_adj_power.flag = 0xaabbccdd;

				g_adj_power._adc   = adjval[0];
				g_adj_power._dac = adjval[1];
				g_adj_power._1310_1k  = adjval[2];
				g_adj_power._1310_2k  = adjval[3];

				g_adj_power._1550cw   = adjval[4];
				g_adj_power._1550_270 = adjval[5];
				g_adj_power._1550_1k  = adjval[6];
				g_adj_power._1550_2k  = adjval[7];

				WriteFlash(FLASH_PAGE_START, 
					(uint32_t*)&(g_adj_power), 
					sizeof(struct adj_power_flash));
				//gl_text(0, 0, "save", -1);
				gl_text(20, 56+24+24, "save...", -1);
				Delay_ms(1000);
				gl_text(20, 56+24+24, "       ", -1);
				gl_text(20, 56+24+24, "save", -1);
			}
			else if(index == 9) {
				break;
			}


		}
		if(flag) {
			//ProTick1963IsLive();

			if(index >= 10)
				index = 0;
			else if(index < 0)
				index = 9;
			if(btnClk && index < 8) {

				//sprintf(strout, "%6.2f", adjval[index]);
				*strout = '\0';
				// if(InputPanel(strout, 6, &strlen)) {
				// 	adjval[index] = atof_(strout);
				// }

				gl_text(loc[index].x, loc[index].y, strout, -1);
				btnClk = 0;
				goto _Redraw;

			}
			else if(btnClk && index == 9) {
				break;
			}
			// 			gl_fill_rect(loc[last_index].x, loc[last_index].y - 5, 5, 5);
			// 			gl_fill_rect(loc[index].x, loc[index].y - 5, 5, 5);
			DrawFocus(loc[last_index].x, loc[last_index].y, COL_White);//RGB16(38, 93, 150));
			DrawFocus(loc[index].x, loc[index].y, COL_Black);//RGB16(255, 255, 0));
			last_index = index;


			flag = 0;
			btnClk = 0;
		}
		/*

		}*/

	}
}


/**
 * @brief	重新绘制用户界面
 * @param\n	
 * @retval\n	NULL
 * @remarks	主要是为处理LCD重启后的绘制，理想状态下不会使用到它
 */

void UI_ProRedraw()
{
	LCD_Clear(Black);	
	// LCD_Timing_Display( 120, 12 , Timer_State);
	// LCD_RedLight_Show(9, 15, g_red_onoff);
	// LCD_Batter_Show(0, 0, 4);
#if 0
	LCD_Power_Control_Selection_Ex(55, 90, ((uint16_t)((int32_t)g_power.set / 1000)), White, Grey);	
#endif
	// LCD_Wavelength_Selection_Ex( 170, 200 , Wavelength_Selection_state , g_sm_mm_color, Grey );	//波长显示切换
	// LCD_OperatMode_Selection( 15, 200, Operating_Mode, g_sm_mm_color, Grey );	//工作模式显示切换
}
/**
 * @brief	SSD1963检测复位后重启过程
 * @param\n	
 * @retval\n	NULL
 * @remarks	
 */

int ProTick1963IsLive()
{
	if(g_lcdlisten_ms >= 100 && g_power_down == 0) {
		g_lcdlisten_ms = 0;
		//LCD_ReadReg(CMD_RD_MEMSTART, &data1);
		//if(data1 != 0x22f2 && data1 != 0xffe0) {
		if(SSD1963_IsRestart()) {


			printf("Restart LCD\n"); 
			LCD_Initializtion();	
			LCD_SetBacklight(0x80);
			//UI_ProRedraw();
			return 1;
		}

	}
	return 0;
}
#define RES1 0x0c
#define RES2 0x0f
#define RES3 0x3a
#define RES4 0xa8
#define RES5 0xe8
#define RES6 0xe9
#define RES7 0xff
/**
 * @brief	获取SSD1963状态
 * @param\n	
 * @retval\n	NULL
 * @remarks	调试时用
 */

void ProGet1963State()
{
	static uint8_t strout[256];
	uint16_t data1, data2, data3, data4;
	
	LCD_WriteCommand(0x00);
	
	if(g_usart_ms >= 1000) {
		g_usart_ms = 0;
		dprintf("\n*******************************************\n");
		
		LCD_WriteCommand(0xb9);
		
		data1 = LCD_ReadData();
		
		data2 = LCD_ReadData();
		dprintf("GPIO conf %x %x\n", data1, data2);
		//LCD_ReadReg(0xb9, &data1);
		//printf("GPIO conf %x\n", data1);

		LCD_ReadReg(RES1, &data1);
		LCD_ReadReg(RES2, &data2);
		LCD_ReadReg(RES3, &data3);
		LCD_ReadReg(RES4, &data4);
		dprintf("Reserved %x %x %x %x   ", data1, data2, data3, data4);
		LCD_ReadReg(RES5, &data1);
		LCD_ReadReg(RES6, &data2);
		LCD_ReadReg(RES7, &data3);
		dprintf("%x %x %x\n", data1, data2, data3);



		LCD_ReadReg(0X0A, &data1);
		LCD_ReadReg(0X0B, &data2);
		LCD_ReadReg(0X0D, &data2);
		LCD_ReadReg(0X0E, &data2);
		sprintf(strout, "Power Mode %x\tAddress Mode %x\tDisplay Mode %x\tEffect status %x\n", data1, data2, data3, data4);
		dprintf(strout);

		LCD_ReadReg(CMD_RD_MEMSTART, &data1);
		LCD_ReadReg(CMD_RD_MEM_AUTO, &data2);
		LCD_ReadReg(CMD_RD_DDB_START, &data3);
		LCD_ReadReg(CMD_GET_PANEL_MODE, &data4);
		sprintf(strout, "Mem start %x\tMem auto %x\tDDB start %x\tPanel mode %x\n", data1, data2, data3, data4);
		dprintf(strout);
		//gl_text(0, 8, strout, -1);

		LCD_ReadReg(CMD_GET_HOR_PERIOD, &data1);
		LCD_ReadReg(CMD_GET_VER_PERIOD, &data2);
		sprintf(strout, "Hor %x\tVer %x\n", data1, data2);
		dprintf(strout);
		//gl_text(0, 20, strout, -1);


		LCD_ReadReg(CMD_GET_GPIO_CONF, &data1);
		LCD_ReadReg(CMD_GET_GPIO_STATUS, &data2);
		sprintf(strout, "GPIO Conf %x\tStatus %x\n", data1, data2);
		dprintf(strout);
		//gl_text(0, 32, strout+8, -1);

		LCD_ReadReg(CMD_GET_POST_PROC, &data1);
		LCD_ReadReg(CMD_GET_PWM_CONF, &data2);
		sprintf(strout, "Post %x\tPWM %x\n", data1, data2);
		dprintf(strout);
		//gl_text(0, 44, strout, -1);


		LCD_ReadReg(CMD_GET_LCD_GEN0, &data1);
		LCD_ReadReg(CMD_GET_LCD_GEN1, &data2);
		LCD_ReadReg(CMD_GET_LCD_GEN2, &data3);
		LCD_ReadReg(CMD_GET_LCD_GEN3, &data4);
		sprintf(strout, "LCD GEN[0-3] %x %x %x %x\n", data1, data2, data3, data4);
		dprintf(strout);
		//gl_text(0, 56, strout, -1);


		LCD_ReadReg(CMD_GET_GPIO0_ROP, &data1);
		LCD_ReadReg(CMD_GET_GPIO1_ROP, &data2);
		LCD_ReadReg(CMD_GET_GPIO2_ROP, &data3);
		LCD_ReadReg(CMD_GET_GPIO3_ROP, &data4);
		sprintf(strout, "GPIO[0-3] ROP %x %x %x %x\n", data1, data2, data3, data4);
		dprintf(strout);
		//gl_text(0, 68, strout, -1);

		LCD_ReadReg(CMD_GET_ABC_DBC_CONF, &data1);
		LCD_ReadReg(CMD_GET_DBC_HISTO_PTR, &data2);
		LCD_ReadReg(CMD_GET_DBC_THRES, &data3);
		sprintf(strout, "DBC conf Histo ptr Thres %x %x %x\n", data1, data2, data3);
		dprintf(strout);
		//gl_text(0, 80, strout, -1);

		LCD_ReadReg(CMD_GET_SIGNAL_MODE, &data1);
		sprintf(strout, "Tearing state %x\n", data1);
		dprintf(strout);
		//gl_text(0, 92, strout, -1);

		dprintf("\n");
	}
}
/*
* Function: 绘制界面焦点
* Parameters:
x, y坐标，color：RGB16(R, G, B)颜色
*/
void DrawFocus(int16_t x, int16_t y, uint32_t color)
{
	uint32_t brush, pen;

	brush = gl_ui_setbrushcolor(color);
	pen = gl_ui_setpencolor(color);
	//gl_fill_rect(x, y - 5, 30, 5);	
	gl_fill_rect(x-3, y, 3, 12);
	gl_ui_setbrushcolor(brush);
	gl_ui_setpencolor(pen);
}


/**
 * @brief	成功进入产品调试后面，提供校准界面和产品型号定制
 * @param\n
 * @retval\n	NULL
 * @remarks	后面密码是-9dBm、-8dBm、-7dBm、-4dBm。\n
 选择密码，然后选择off或2KHz切换到CW模式，表示输入一个密码，完成所有\n
 密码后进入后台
 */

void IsHacker()
{
	int8_t tmpredmode;
	static uint8_t state = 0;

	//UI_ProductionAdjust();//debug;
	if(hackflag == 1) {
		hackflag = 0;
		switch(state) {
case 0:
	if((int32_t)hackval == -9000) {
		state++;
		//UI_DebugMain();
	}
	else
		state = 0;
	break;
case 1:
	if((int32_t)hackval == -8000)
		state++;
	else
		state = 0;
	break;
case 2:
	if((int32_t)hackval == -7000)
		state++;
	else
		state = 0;
	break;
case 3:
	if((int32_t)hackval == -4000) {
		state = 0;
		
		tmpredmode = g_red_mode;//保存红光当前状态，防止在校准界面弹出红光闪烁
		g_red_mode = 0;			
		Ctrl_RedLight(g_red_mode);//关闭红光
		//UI_DebugMain();
		g_red_mode = tmpredmode;//恢复之前红光状态
		LCD_DrawMain();
		//UI_ProductionAdjust();

	}
	else
		state = 0;
	break;
default:
	state = 0;
	break;
		}
	}
}




/*****************************************************************************
检测模块
*****************************************************************************/

//电池等级
#define LEVEL_POWER    0  //外部供电
#define LEVEL_FULL     2  //充满电
#define LEVEL_CHARGE   4  //正在充电
#define LEVEL_4        6  //电池4格
#define LEVEL_3        7  //电池3格
#define LEVEL_2        8  //电池2格
#define LEVEL_1        9  //电池1格
#define LEVEL_0        10  //电池0格
#define LEVEL_SHUTDOWN 11 //电池自动关机
/*int8_t disp_battery_index[12] = {
//0  1  2  3  4  5  6  7  8  9  10 11其中1、3无用
4, 4, 4, 4, 4, 5, 4, 3, 2, 1, 0, 0
};*/
/**
 * @brief	电源检测
 * @param\n	
 * @retval\n	NULL
 * @remarks	划分电量等级，且低电量自动关机，关机前刷红色屏幕提示\n
只有当前等级与上一次等级的插的绝对值>=2时才更新，电量检测不会再两个相邻等级里来回变动，\n
另外也保证在LEVEL_POWER、LEVEL_FULL、LEVEL_CHARGE三个等级可以快速响应切换\n
电池电量检测浮动在0.030V左右，当快没电的时候浮动可达0.100V
 */

void ProChargerMonitor()
{
	uint8_t strout[256];
	static uint8_t times = 100;
	static uint16_t last_level = LEVEL_4, level_show = LEVEL_4;
	uint16_t level = LEVEL_4; 
	float vol, tvol;
	//static float last_vol = 100;
	int x, y;
	int ad;
	//return ;
	/*************************充电指示部分*******************************************************/
	if(g_batter_delay > 100) {
		g_batter_delay = 0;
		ad = GetAD(0);
		//vol = (float)GetAD(0)*0.00349+0.7;//Ref = 3.3

		vol = (float)ad*0.002643333+0.77;//Ref = 2.5, M7二极管压降0.77
		g_battery_vol = vol;
		sprintf(strout, "better:%0.3f %d %f", vol, ad, (float)ad * 0.00061);
		//debugtxt(0, 12*6, strout, 30);
		//gl_text(0, 65, strout, 30);

		//debug
		if((GPIO_ReadInputDataBit(GPIO_PORT_POWER_CHK, GPIO_CHARG_CHK)==0)) {//充满、只挂电池、只挂外加电源
			if(vol > 12.0) {//外加电源远远大于
				level = LEVEL_POWER;
				//debugtxt(0, 100+12, "out side", 20);
				//gl_text(0, 100+12, "out side", 20);
			}
			else if(vol > 11.4) {
				level = LEVEL_FULL;
				//debugtxt(0, 100+12, "full    ", 20);	
				//gl_text(0, 100+12, "out side", 20);
			}
			else if(vol > 7.6) {
				level = LEVEL_4;
				//debugtxt(0, 100+12, "only batt", 20);
				//gl_text(0, 100+12, "out side", 20);
			}
			//else if(7.60 < vol && vol < 7.97) {
			else if(vol > 7.4) {
				level = LEVEL_3;

				//debugtxt(0, 100+12, "full    ", 20);	
			}
			else if(vol > 7.1) {
				level = LEVEL_2;
			}
			//else if(6.80 <= vol && vol < 7.17) {
			else if(vol > 6.9) {
				level = LEVEL_1;
			}
			else if(vol > 6.7) {
				level = LEVEL_0;	
			}
			else if(vol < 6.7) {
				level = LEVEL_SHUTDOWN;

				// LCD_Batter_Show(0, 0, LEVEL_0);
				for( x=0; x < 320; x++ )		//屏幕刷红色，表示电量过低自动关机
					for( y=0; y < 240; y++ )
						gl_setpoint(x, y, RGB16(255, 0, 0));
				Delay_ms(2000);
				powerDownDelayCnt = 1100;//表示要关机了
				TurnOffPower();

			}
		}
		else if((GPIO_ReadInputDataBit(GPIO_PORT_POWER_CHK, GPIO_CHARG_CHK)==1)) {//正在充电
			level = LEVEL_CHARGE;
			//debugtxt(0, 100+12, "charge...", 20);	
		}


		//为了处理处理器内部AD转换不稳定（0.030V的浮动）而进行以下处理
		if(
			//只有连续多次电量检测级别相等
			//要么电量越来越低，要么是连接充电器
			(last_level == level &&
			times++ >= 2 && 
			(level > level_show || level - level_show <= -2 || level == level_show) )||
			//或者上一次电量检测是插入外电源的，这一次需要立即更新
			//BUG:LEVEL_POWER和LEVEL_FULL可能回来回转换，但是用户界面里这两个状态的图标是一样的，
			//所以不回被用户发现异常
			(level_show == LEVEL_CHARGE || level_show == LEVEL_POWER ||level_show == LEVEL_FULL)
			) {

				level_show = level;//上次显示的等级
				times = 0;
				// LCD_Batter_Show(0, 0, level_show);
				sprintf(strout, "better:%0.3f", vol);
				debugtxt(0, 12*6, strout, 30);			
		}
		if(last_level != level)
			times = 0;
		sprintf(strout, "state %d %d %d-", times, level_show, level);
		debugtxt(0, 100+24, strout, -1);
		last_level = level;
	}
	return ;
}
/**
 * @brief	获取电池电量，并换算成电压值，电压值存在0.2~0.4的偏差
 * @param\n	
 * @retval\n	NULL
 * @remarks	
 */

float GetBattery()
{
	uint32_t ad;
	if(g_batter_delay > 100) {
		g_batter_delay = 0;
		ad = GetAD(0);
		//vol = (float)GetAD(0)*0.00349+0.7;//Ref = 3.3

		g_battery_vol = (float)ad*0.002643333+0.77;//Ref = 2.5, M7二极管压降0.77
	}
	return g_battery_vol;
}



/**
 * @brief	响应定时关机
 * @param\n	
 * @retval\n	NULL
 * @remarks	
 */

void ProTimerShutdown()
{
	/***************************定时关机部分*************************************************************/
	//一般定时用，定时关机，可以设置为5min  ，10min  ，15min  ，30min  ，60min 状态
	//TIM2设置为100ms周期，3000 ， 6000  ， 9000  ， 18000 ，  36000		   

	switch(Timer_State)				
	{
	case   0:	  // 0 = 定时器关闭

		break;   
	case   2:	  // 1 = 1Omin
		if( Timer_Counter >=6000 )
		{ 
			GPIO_ResetBits(GPIO_PORT_LCD, GPIO_LCD_OFF);//关显示屏操作 
			//delayMs(10);//延迟一小段时间，等待操作者松开按键
			Delay_ms(10);
			GPIO_ResetBits(GPIO_CTRL_PORT_B, GPIO_SYSPWR_ONOFF); //关机 CHECK2置0 清标志位
			Timer_State = 0;
			// LCD_Timing_Display( 255, 12 , Timer_State );	  //定时器显示切换
		}
		break;
	default:
		break;
	}	
}

/**
 * @brief	串口输出启动过程，检测1963
 * @param\n	
 * @retval\n	NULL
 * @remarks	
 */
int restarttimes = 100;
void Check1963()
{
	uint8_t strout[256];
	uint16_t data;
	int times = 0;	

	printf("Checking chip...");
	LCD_ReadReg(CMD_RD_MEMSTART, &data);
	if(data != 0x22f2) {
		while(times++ < 3) {
			LCD_ReadReg(CMD_RD_MEMSTART, &data);
			if(data != 0x22f2) {
				printf(".");
				LCD_Initializtion();	
				Delay_ms(100);
			}
		}
		g_usart_ms = 2000;
		printf("\n");
		//多次启动LCD失败，关闭系统
		LCD_ReadReg(CMD_RD_MEMSTART, &data);
		if(data != 0x22f2) {
			printf("Startup chip error!\n\n");
			ProGet1963State();
			// 			powerDownDelayCnt = 1000;
			// 			while(1) {
			// 				printf("Shut down!!!\n");
			// 				TurnOffPower();
			// 				LCD_Initializtion();
			// // 				LCD_ReadReg(CMD_RD_MEMSTART, &data);
			// // 				if(data == 0x22f2) {	
			// // 					break;
			// // 				}
			// 				Delay_ms(30);
			// 			}
		}
	}
	restarttimes = times;
	printf("\nStartup chip success!\n");
	LCD_SetBacklight(0x80);
}
/**
 * @brief	串口输出启动过程，检测LCD
 * @param\n	
 * @retval\n	NULL
 * @remarks	
 */

void CheckLCD()
{
	uint16_t data;
	printf("Checking LCD...\n");
	LCD_SetPoint(15, 15, 0xaabb);
	Delay_ms(1);
	data = LCD_GetPoint(15, 15);
	if(data != 0xaabb) {
		printf("Open LCD error!Please check whether the LCD  connect!\n");
	}
	else {
		printf("Open LCD success\n");
	}

}


	
static void _cbBkWindow(WM_MESSAGE* pMsg) {
	int Id;
  switch (pMsg->MsgId) {
  	 case WM_NOTIFY_PARENT:
  	 	Id    = WM_GetId(pMsg->hWinSrc);      /* Id of widget */
  	 	BUTTON_SetText(hButton1, "dfwer");
  	 	// hItem  = WM_GetDialogItem(hDlg, Id);
  	 	break;
		// case WM_PAINT:
		//   GUI_SetBkColor(GUI_RED);
		//   GUI_Clear();
		//   GUI_SetColor(GUI_WHITE);
		//   GUI_SetFont(&GUI_Font24_ASCII);
		//   GUI_DispStringHCenterAt("WIDGET_ListBoxOwnerDraw", 160, 5);
// break;
  default:
    WM_DefaultProc(pMsg);
  }
}

void TestWin()
{

	static const GUI_ConstString _ListBox[] = {
	"English", "Deutsch", "1", "2", "231233", "sdfsf", NULL
	};
	hFrame = FRAMEWIN_Create("test", 0, WM_CF_SHOW, 50, 0, 150, 100); 
	WM_Paint(hFrame); 
	hButton1 = BUTTON_CreateAsChild(20, 20, 60, 30, hFrame, 1, WM_CF_SHOW); 
	BUTTON_SetText(hButton1, "OK"); 
	WM_Paint(hButton1);
	hListbox = LISTBOX_CreateAsChild(_ListBox, hFrame, 20, 60, 100, 80, WM_CF_SHOW);
	WM_Paint(hListbox); 

	FRAMEWIN_SetBarColor(hFrame, 0, RGB(0, 255, 0));
	FRAMEWIN_SetBarColor(hFrame, 1, RGB(0, 0, 255));
	// FRAMEWIN_SetTitleVis(hFrame, 0);
	hFrame = FRAMEWIN_Create("test", 0, WM_CF_SHOW, 0, 50, 200, 150); 
	WM_Paint(hFrame); 
	hButton1 = BUTTON_CreateAsChild(40, 20, 100, 50, hFrame, 1, WM_CF_SHOW); 
	BUTTON_SetText(hButton1, "OK1112318278917092jasuoriuwoemrlaiuseoriw"); 
	
	
	FRAMEWIN_SetBarColor(hFrame, 0, RGB(0, 255, 0));
	FRAMEWIN_SetBarColor(hFrame, 1, RGB(0, 0, 255));
	// FRAMEWIN_SetTitleVis(hFrame, 0);
	WM_Paint(hFrame); 
	WM_Paint(hButton1);
	WM_SetDesktopColor(RGB(255, 0, 0));
	WM_SetCallback(hFrame, &_cbBkWindow);

}


void TP_EINT()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; //选择引脚2 3 5
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//GPIO_Mode_IPU; //选择输入模式为浮空输入 GPIO_Mode_IN_FLOATING
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;          //输出频率最大50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);                                 //设置PC.2/PC.3/PC.5
	
	//清空中断标志
	EXTI_ClearITPendingBit(EXTI_Line0);

	//选择中断管脚PC.2 PC.3 PC.5
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);

	EXTI_InitStructure.EXTI_Line = EXTI_Line0; //选择中断线路2 3 5
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; //设置为中断请求，非事件请求
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //设置中断触发方式为上下降沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;                                          //外部中断使能
	EXTI_Init(&EXTI_InitStructure);
}


// ***************************************************************************
// LED 显示

#define RCC_LASTER     RCC_APB2Periph_GPIOC
#define GROUP_LASTER   GPIOC
#define PIN_LASTER     (GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9)
// LED-连续激光器
// LED-APD
#define RCC_LED_APD   RCC_APB2Periph_GPIOD
#define GROUP_LED_APD GPIOD
#define PIN_LED_APD_20V   GPIO_Pin_12
#define PIN_LED_APD_40V   GPIO_Pin_13
void Init_LED()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	

	// 脉冲激光器 LED 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin		= GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA, GPIO_Pin_6 | GPIO_Pin_7);

	// 连续激光器 LED
	RCC_APB2PeriphClockCmd(RCC_LASTER, ENABLE);
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin		= PIN_LASTER;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_Out_PP;
	GPIO_Init(GROUP_LASTER, &GPIO_InitStructure);
	GPIO_SetBits(GROUP_LASTER, PIN_LASTER);


	// APD 电压选择 LED
	RCC_APB2PeriphClockCmd(RCC_LED_APD, ENABLE);
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin		= PIN_LED_APD_20V | PIN_LED_APD_40V;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_Out_PP;
	GPIO_Init(GROUP_LED_APD, &GPIO_InitStructure);
	GPIO_SetBits(GROUP_LED_APD, PIN_LED_APD_20V | PIN_LED_APD_40V);


	// 硬件有错误，没有用的脚，但是必须改成上拉输入
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin		= GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}


#define CTRL_LED_LASER_LV 0
#define CTRL_LED_LASER_MV 1
#define CTRL_LED_LASER_HV 2

#define CTRL_LED_APD_20V 3
#define CTRL_LED_APD_40V 4

#define CTRL_LED_PWM1    5
#define CTRL_LED_PWM2    6
void Ctrl_LED(int index, int val)
{
	GPIO_TypeDef *group;
	uint16_t io;

	
	switch(index) {
	case CTRL_LED_LASER_LV:
		group = GPIOC;
		io = GPIO_Pin_7;
		break;
	case CTRL_LED_LASER_MV:
		group = GPIOC;
		io = GPIO_Pin_8;
		break;
	case CTRL_LED_LASER_HV:
		group = GPIOC;
		io = GPIO_Pin_9;
		break;


	case CTRL_LED_APD_20V:
		group = GPIOD;
		io = GPIO_Pin_12;
		break;
	case CTRL_LED_APD_40V:
		group = GPIOD;
		io = GPIO_Pin_13;
		break;


	// 暂时废除
	// case CTRL_LED_PWM1:
	// 	group = ;
	// 	io = ;
	// 	break;
	// case CTRL_LED_PWM2:
	// 	group = ;
	// 	io = ;
	// 	break;
	}
	if (val == 0) {
		GPIO_ResetBits(group, io);
	}
	else {
		GPIO_SetBits(group, io);
	}
	
}


// ***************************************************************************
// 连续激光器控制
#define GROUP_LD1 GPIOC
#define GROUP_LD2 GPIOB
#define GROUP_LD3 GPIOB

#define RCC_LD1 RCC_APB2Periph_GPIOC
#define RCC_LD2 RCC_APB2Periph_GPIOB
#define RCC_LD3 RCC_APB2Periph_GPIOB

#define PIN_LD_OUTPUT_1 GPIO_Pin_4
#define PIN_LD_OUTPUT_2 GPIO_Pin_10
#define PIN_LD_OUTPUT_3 GPIO_Pin_11
void Init_LaserPower()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_LD1, ENABLE);
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin		= PIN_LD_OUTPUT_1;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_Out_PP;
	GPIO_Init(GROUP_LD1, &GPIO_InitStructure);
	GPIO_SetBits(GROUP_LD1, PIN_LD_OUTPUT_1);

	RCC_APB2PeriphClockCmd(RCC_LD2, ENABLE);
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin		= PIN_LD_OUTPUT_2;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_Out_PP;
	GPIO_Init(GROUP_LD2, &GPIO_InitStructure);
	GPIO_SetBits(GROUP_LD2, PIN_LD_OUTPUT_2);

	RCC_APB2PeriphClockCmd(RCC_LD3, ENABLE);
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin		= PIN_LD_OUTPUT_3;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_Out_PP;
	GPIO_Init(GROUP_LD3, &GPIO_InitStructure);
	GPIO_SetBits(GROUP_LD3, PIN_LD_OUTPUT_3);
}


// #define CTRL_LASER_LV 0
// #define CTRL_LASER_MV 1
// #define CTRL_LASER_HV 2
void Ctrl_LaserPower(int val)
{

	GPIO_ResetBits(GROUP_LD1, PIN_LD_OUTPUT_1);
	GPIO_ResetBits(GROUP_LD2, PIN_LD_OUTPUT_2);
	GPIO_ResetBits(GROUP_LD3, PIN_LD_OUTPUT_3);
	Ctrl_LED(CTRL_LED_LASER_LV, 0);
	Ctrl_LED(CTRL_LED_LASER_MV, 0);
	Ctrl_LED(CTRL_LED_LASER_HV, 0);
	switch(val) {
	case CTRL_LASER_LV:
		GPIO_SetBits(GROUP_LD1, PIN_LD_OUTPUT_1);
		Ctrl_LED(CTRL_LED_LASER_LV, 1);
		// TODO send message to LED
		break;
	case CTRL_LASER_MV:
		GPIO_SetBits(GROUP_LD2, PIN_LD_OUTPUT_2);
		Ctrl_LED(CTRL_LED_LASER_MV, 1);
		// TODO send message to LED
		break;
	case CTRL_LASER_HV:
		GPIO_SetBits(GROUP_LD3, PIN_LD_OUTPUT_3);
		Ctrl_LED(CTRL_LED_LASER_HV, 1);
		// TODO send message to LED
		break;
	}
}



#define USE_TIM1_PWM 0
#define USE_TIM2_PWM 0
#define USE_TIM3_PWM 1
#define USE_TIM4_PWM 0

// 开启 CH3
#define RCC_TIM2_PWM   RCC_APB2Periph_GPIOA
#define GROUP_TIM2_PWM GPIOA
#define PIN_TIM2_PWM   (GPIO_Pin_2)

// 开启 CH1 CH2
#define RCC_TIM3_PWM   RCC_APB2Periph_GPIOA
#define GROUP_TIM3_PWM GPIOA
#define PIN_TIM3_PWM   (GPIO_Pin_6 | GPIO_Pin_7)

void Init_PWM(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;


	if (USE_TIM2_PWM) {
		RCC_APB2PeriphClockCmd(RCC_TIM2_PWM | RCC_APB2Periph_AFIO , ENABLE);
		GPIO_InitStructure.GPIO_Pin   = PIN_TIM2_PWM; //TIM_CH2
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP | GPIO_Mode_AF_PP;  //复用推挽输出
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GROUP_TIM2_PWM, &GPIO_InitStructure);
	}

	if (USE_TIM3_PWM) {
		RCC_APB2PeriphClockCmd(RCC_TIM3_PWM | RCC_APB2Periph_AFIO , ENABLE);//
		GPIO_InitStructure.GPIO_Pin   = PIN_TIM3_PWM; //TIM_CH2
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;  //复用推挽输出
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GROUP_TIM3_PWM, &GPIO_InitStructure);	
	}
	

}


// #define PWM_CH1 0
// #define PWM_CH2 1
// #define PWM_CH3 2
// #define PWM_CH4 3
static struct ctrl_pwm pwm_dev1, pwm_dev2;
void Ctrl_PWM(struct ctrl_pwm *val)
{
	//TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	switch (val->ch) {
	case PWM_CH1:
		pwm_dev1.timer  = val->timer;
		pwm_dev1.ch     = val->ch;
		pwm_dev1.high   = val->high;
		pwm_dev1.cycle  = val->cycle;
		pwm_dev1.reversal  = val->reversal;
		pwm_dev1.enable = val->enable;

		break;
	case PWM_CH2:
		pwm_dev2.timer  = val->timer;
		pwm_dev2.ch     = val->ch;
		pwm_dev2.high   = val->high;
		pwm_dev2.cycle  = val->cycle;
		pwm_dev2.reversal  = val->reversal;
		pwm_dev2.enable = val->enable;
		break;
	default:
		break;
	}
	


	TIM_Cmd(val->timer, DISABLE);  //使能TIMx外设
	TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState = val->enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse       = val->high;                   //设置待装入捕获比较寄存器的脉冲值，初始的占空比
	TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高

	switch (val->ch) {
	case PWM_CH1:
		TIM_OC1Init(val->timer, &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(val->timer, TIM_OCPreload_Enable);
		break;
	case PWM_CH2:
		TIM_OC2Init(val->timer, &TIM_OCInitStructure);
		TIM_OC2PreloadConfig(val->timer, TIM_OCPreload_Enable);
		break;
	case PWM_CH3:
		TIM_OC3Init(val->timer, &TIM_OCInitStructure);
		TIM_OC3PreloadConfig(val->timer, TIM_OCPreload_Enable);
		break;
	case PWM_CH4:
		TIM_OC4Init(val->timer, &TIM_OCInitStructure);
		TIM_OC4PreloadConfig(val->timer, TIM_OCPreload_Enable);
		break;
	default:
		break;
	}

	TIM_ARRPreloadConfig(val->timer, ENABLE); //使能TIMx在ARR上的预装载寄存器 
	TIM_Cmd(val->timer, ENABLE);  //使能TIMx外设
	// 保证相位相同，必须重新设置计数器
	TIM_SetCounter(val->timer, 0);
}

void Get_PWM(struct ctrl_pwm *val)
{
	switch (val->ch) {
	case PWM_CH1:
		val->timer  = pwm_dev1.timer;
		val->ch     = pwm_dev1.ch;
		val->high   = pwm_dev1.high;
		val->cycle  = pwm_dev1.cycle;
		val->reversal  = pwm_dev1.reversal;
		val->enable = pwm_dev1.enable;
		break;
	case PWM_CH2:
		val->timer  = pwm_dev2.timer;
		val->ch     = pwm_dev2.ch;
		val->high   = pwm_dev2.high;
		val->cycle  = pwm_dev2.cycle;
		val->reversal  = pwm_dev2.reversal;
		val->enable = pwm_dev2.enable;
		break;
	default:
		val->ch = -1;
		printf("no ch\r\n");
		break;
	}
}
// ***************************************************************************
// APD电源控制
#define RCC_APD       RCC_APB2Periph_GPIOA
#define GROUP_APD 	  GPIOA
#define PIN_APD_40V   GPIO_Pin_8
void Init_APD()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APD, ENABLE);
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin		= PIN_APD_40V;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_Out_PP;
	GPIO_Init(GROUP_APD, &GPIO_InitStructure);
	// 输出40V
	GPIO_SetBits(GROUP_APD, PIN_APD_40V);
}

// #define CTRL_APD_20V 0
// #define CTRL_APD_40V 1
void Ctrl_APD(int val)
{
	Ctrl_LED(CTRL_LED_APD_20V, 0);
	Ctrl_LED(CTRL_LED_APD_40V, 0);

	switch(val) {
	case CTRL_APD_20V:
		GPIO_ResetBits(GPIOA,GPIO_Pin_8);
		Ctrl_LED(CTRL_LED_APD_20V, 1);
		// TODO send message to lcd
		break;
	case CTRL_APD_40V:
		GPIO_SetBits(GPIOA,GPIO_Pin_8);
		Ctrl_LED(CTRL_LED_APD_40V, 1);
		// TODO send message to lcd
		break;
	}
}

// ***************************************************************************
// 外设电源开关

#define RCC_PP   RCC_APB2Periph_GPIOA
#define GROUP_PP GPIOA
#define PIN_PP   GPIO_Pin_12

void Init_PeripheralPower()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_PP, ENABLE);
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin		= PIN_PP;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_Out_PP;
	GPIO_Init(GROUP_PP, &GPIO_InitStructure);
	// 输出40V
	GPIO_SetBits(GROUP_PP, PIN_APD_40V);
}

// #define CTRL_PP_OFF 0
// #define CTRL_PP_ON 1
void Ctrl_PeripheralPower(int val)
{
	switch(val){
	case CTRL_PP_OFF:
		GPIO_ResetBits(GROUP_PP, PIN_PP);
		break;
	case CTRL_PP_ON:
		GPIO_SetBits(GROUP_PP, PIN_PP);
		break;
	}
}


// ***************************************************************************
// 按键IO全部设置成上拉输入
#define KEY_LT GPIO_Pin_2
#define KEY_RT GPIO_Pin_6
#define KEY_M  GPIO_Pin_4
#define KEY_LB GPIO_Pin_3
#define KEY_RB GPIO_Pin_5
#define KEY_GPIO_X GPIOE
void Init_Key()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	// 上拉输入按键
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin		= KEY_LT | KEY_RT | KEY_M | KEY_LB | KEY_RB;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_IPU;
	// GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_Out_PP;
	GPIO_Init(KEY_GPIO_X, &GPIO_InitStructure);
	// GPIO_SetBits(KEY_GPIO_X, KEY_LT | KEY_RT | KEY_M | KEY_LB | KEY_RB);
}

int32_t ScanKey()
{
	if(KeyDown(KEY_GPIO_X, KEY_LT)) {
		printf("KEY_LT\r\n");
		// GUI_StoreKeyMsg(GUI_KEY_LEFT,1);
		return 0;
	}
	else if(KeyPress(KEY_GPIO_X, KEY_RT)) {

		printf("KEY_RT\r\n");
		// GUI_StoreKeyMsg(GUI_KEY_RIGHT,1);
		return 0;
	}
	else if(KeyDown(KEY_GPIO_X, KEY_M)) {

		printf("KEY_M\r\n");
		GUI_StoreKeyMsg(GUI_KEY_ENTER,1);
		return 0;
	}
	else if(KeyDown_Ex(KEY_GPIO_X, KEY_LB)) {
		printf("KEY_LB\r\n");
		GUI_StoreKeyMsg(GUI_KEY_TAB,1);
		return 0;
	}
	else if(KeyDown(KEY_GPIO_X, KEY_RB)) {
		printf("KEY_RB\r\n");
		return 0;
	}
}

extern WM_HWIN hdlg ;
/*******************************************************************************
* Function Name  : main
* Description    : Main program
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
//main(
extern WM_HWIN hdlg ;
int main(void)
{
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
	USART_Configuration();
	Init_PWM();
	Init_LED();
	
	
	USART_SendData(USART3, (uint8_t) 'd');
	USART_SendData(USART3, (uint8_t) 'd');
	USART_SendData(USART3, (uint8_t) 'd');
	USART_SendData(USART3, (uint8_t) 'd');
	USART_SendData(USART3, (uint8_t) 'd');
	printf("\n\n----------------------------------------------------------------------------\n");
	printf("        GLink TS100 Runing\n");
#define SYSCLK_FREQ_72MHz
	SystemInit();//SetSysClock();
	
	USART_SendData(USART3, (uint8_t) 'k');
	USART_SendData(USART3, (uint8_t) 'k');
	USART_SendData(USART3, (uint8_t) 'k');
	USART_SendData(USART3, (uint8_t) 'k');
	USART_SendData(USART3, (uint8_t) 'k');

	//液晶屏GUI配置
	gl_ui_setlib(p_zm_ascii_st9, 8, 12, p_zm_step_st9);
	gl_ui_setbkmode(BACKFILL);//背景填充模式
	gl_ui_setfontcolor(RGB16(255, 0, 0));
	gl_ui_setbkcolor(RGB16(0, 255, 0));
	gl_ui_setpencolor(RGB16(235, 34, 209));

	delay_init();
	/*****************按键中断初始化部分*********************** */
	Function_IO_config(); 
	// uTLaser 2015-8-31
	Init_PWM();
	TIM2_Configuration();
	TIM3_Configuration();
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
	TIM6_Init();//1MS定时

	NVIC_Configuration();

	LCD_Initializtion();
	LCD_SetBacklight(0x80);
	//Delay(2000);// 稍作一段延时，非常重要，保证LCD完全上电，否则下面的Check1963会对LCD重启
	LCD_Clear(RGB16(255, 0, 0));
	
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
	
	printf("Draw UI\n");
	//LCD_DrawMain();
	powerDownDelayCnt=0;
	g_batter_delay = 10001;	

	
	//while(1)
	 //	UI_DebugMain();
	LCD_Clear(RGB16(255, 0, 0));
	// Init_LED();
	// while(1) {
	// 	ScanKey();
	// 	Delay(1000);

	// }
	//InputPanel(strout, 50, 0);
	sprintf(strout, "res times %d", restarttimes);
	//printf("%s\n", strout);

	
	//Delay(100000);
	GUI_Init();
	
	
	GUI_SetBkColor(RGB(0, 0, 0)); //设置背景颜色 
	GUI_SetColor(RGB(255, 255, 255)); //设置前景颜色，及字体和绘图的颜色
	GUI_Clear(); //按指定颜色清屏
	GUI_DispStringAt("Hello World ..", 0, 100); //显示字符
	// GUI_CURSOR_Show();//显示鼠标, 测试触摸屏必须打开窗口功能 GUI_WINSUPPORT

	LCD_L0_FillRect(10, 10, 40, 40);
	//LCD_L0_SetPixelIndex(10, 100, RGB16(255, 0, 0));
	
	
	LCD_L0_SetPixelIndex(10, 190, RGB16(255, 0, 0));
	LCD_L0_SetPixelIndex(10, 191, RGB16(255, 0, 0));
	LCD_L0_SetPixelIndex(10, 192, RGB16(255, 0, 0));
	LCD_L0_SetPixelIndex(10, 193, RGB16(255, 0, 0));
	LCD_SetPoint(100, 240, RGB16(255, 0, 0));
	gl_setarea(10, 200, 320, 240);
	gl_setpoint(10, 200, RGB16(255, 0, 0));
	gl_setpoint(10, 201, RGB16(0, 255, 0));
	gl_setpoint(10, 202, RGB16(255, 0, 0));
	gl_setpoint(10, 203, RGB16(0, 0, 255));
	gl_horizon_line(200, 270, 150);
	//Delay(1000000);//必须稍加延时, 否则会白屏  
	//gl_text(0, 200-12, "abcdefg", -1);
	// TestWin();
	
	TP_Init();
	TP_EINT();
	TP_GetADC(&pt);
	// TC_Adj();
	FLASH_Configuration();
	TC_Test();
	
	//while(1);
	
	MainTask();

	while(1) {
		ScanKey();
		GUI_Exec();	
	}
	
	// DemoRedraw();
	while(1)
	{
		TP_GetADC(&pt);
		// GUI_TOUCH_GetUnstable(&x, &y);
		// if (y != -1) {
		// 	press = 1;
		// }
		// else {
		// 	press = 0;
		// }
		
		// // press = getlogxy(&pt);
		// if (press == 1) {
			
		// 	if (fpress == 0) {
		// 		fpress = 1;
		// 		y0 = y;
		// 		y1 = y;
		// 		WM_GetWindowRectEx(hdlg, &rect);

		// 	}
		// 	else {
		// 		y1 = y;
		// 		WM_MoveWindow(hdlg, 0, rect.y0 + y1 - y0);

		// 	}
		// }
		// else {
		// 	fpress = 0;
		// 	y1 = pt.y;
		// }
		
		// sprintf(strout, "p %d fp %d ADC %d %d %d %d[%d] ", press, fpress, y0, y1, rect.x0, rect.y0, rect.y0 + y1 - y0);
		// GUI_DispStringAt((const char*)strout, 0, 10);
		Delay_ms(100);
		GUI_Exec();
		
		
		
		// b = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0);
		// sprintf(strout, "%d %d", b);
		

		if (getlogxy(&pt)) 
		{

			// GUI_TOUCH_StoreUnstable(pt.x, pt.y);
			//GUI_Exec();
			sprintf(strout, "sec %d touch ADC %d %d  ", tick++, pt.x, pt.y);
			GUI_DispStringAt((const char*)strout, 0, 200);	
		}
		// TP_GetADC(&pt);

			
		
		
		Ctrl_RedLight(0);
		

		ProGet1963State();
		if(ProTick1963IsLive())
				UI_ProRedraw();

		//UI_ProRedLight_ShutdownTimer();//
		//UI_ProWavelength();
		//UI_ProPower();
		//UI_ProMode();
		// UI_ProUp();
		// UI_ProDown();
		TurnOffPower();
		//ProChargerMonitor();
		//ProTimerShutdown();//定时关机
		//if(Operating_Mode != OPM_270) {
		//AutoCtrlPower();

		IsHacker();		
	}//while结尾
}

/*
* Function: 调试查看，正式版没有
*/
extern uint8_t msgindex;
void DebugLookAD()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	int16_t ad = 200;
	char strout[256];//, times = 0;
	static uint16_t flag = 0;
	float vol;//scale;

	gl_key_init();
	g_power.set = (uint32_t)(-10000);
	Ctrl_Power(&g_power);
	flag = 1;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	while(1) {
		TurnOffPower();
		// 		if(gl_key_down(6)) {
		// 			//sprintf(strout, "key %d", times++);
		// 			sprintf(strout, "key %d", msgindex);
		// 			gl_text(125, 0, strout, -1);
		// 		}
		// 		continue;
		// 		UI_ProMode();
		// 		UI_ProWavelength();

		if(KeyPress(GPIOA, KEY_C)) {
			ad-= 20;
			g_power.set += 100;
			flag = 1;
		}
		else if(KeyPress(GPIOA, KEY_Z)) {
			ad+= 20;
			g_power.set -= 100;
			flag = 1;
		}
		// 		else if(KeyPress(GPIOA, KEY_A)) {
		// 			ad-= 100;
		// 			flag = 1;
		// 		}
		// 		else if(KeyPress(GPIOA, KEY_X)) {
		// 			ad+= 100;
		// 			flag = 1;
		// 		}
		if(flag) {
			flag = 0;
			if(ad > 4095)
				ad = 0;
			else if(ad < 0)
				ad = 4095;
			//DAC_SoftwareTriggerCmd(DAC_Channel_1, DISABLE);  
			DAC_SetChannel1Data(DAC_Align_12b_R, ad/*ad*/);
			DAC_SoftwareTriggerCmd(DAC_Channel_1, ENABLE);  			

			//DAC_SoftwareTriggerCmd(DAC_Channel_2, DISABLE);  
			DAC_SetChannel1Data(DAC_Align_12b_R, ad);
			DAC_SoftwareTriggerCmd(DAC_Channel_2, ENABLE);  			
			sprintf(strout, "debug set adc:%4.4d", ad);
			//Ctrl_Power(&g_power);

			gl_text(0, 100-12, strout, 20);
		}
		continue;
		//AutoCtrlPower();
		UI_ProRedLight_ShutdownTimer();
		if(g_ad_ms > 2000) {

			/* Enable GPIOD clock */

			// 			if(flag) {
			// 				flag = 0;
			// 				GPIO_SetBits(GPIOC, GPIO_Pin_0);
			// 			}
			// 			else {
			// 				flag = 1;
			// 				GPIO_ResetBits(GPIOC, GPIO_Pin_0);
			// 			}

			g_ad_ms = 0;
			//sprintf(strout, "adc %4.4d", GetAD(1));

			vol = (float)GetAD(0)*0.00349+0.7;
			sprintf(strout, "ad0 %0.3f", vol);
			gl_text(0, 200, (uint8_t*)strout, 20);

			vol = (float)GetAD(1)*0.000805;
			//vol = (float)GetAD(1)*0.00349+0.7;
			sprintf(strout, "ad1 %3.3f", vol);
			gl_text(0, 200+12, (uint8_t*)strout, 20);
			ProChargerMonitor();
		}

	}
}
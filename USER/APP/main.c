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
#include "USER/GLCD/GLCD.h"
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

#include "prj_type.h"
#include "flash.h"
#include "project.h"					//本工程的宏、函数、全局变量、结构体、版本信息、定义
#include "gl_key.h"						//异步按键驱动，待消息机制，本工程里没有使用该套接口
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

struct project_env g_env;
int g_lcd_test = 0;

/*****************************************************************************
工程模块配置
*****************************************************************************/
/*****************一般功能IO口初始化******************************************/

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



void TIM3_Init(void)
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

	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIMx在CCR2上的预装载寄存器
	//上面两句中的OC2确定了是channle几，要是OC3则是channel 3  

	TIM_ARRPreloadConfig(TIM3, ENABLE); //使能TIMx在ARR上的预装载寄存器 

	/* TIM3 enable counter */
	TIM_Cmd(TIM3, ENABLE);  //使能TIMx外设
}



void TIM5_Init()
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); 
	//时钟72M    1ms中断  向上计数
	TIM_TimeBaseStructure.TIM_Period		= 10-1;
	TIM_TimeBaseStructure.TIM_Prescaler		= 7200-1;
	TIM_TimeBaseStructure.TIM_CounterMode	= TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE); //开定时器中断
	TIM_Cmd(TIM5, ENABLE);
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
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd				 = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel					 = TIM6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority		 = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd				 = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel					 = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority		 = 0;
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
	struct point pt[] = {
		{60,                 60},
		{LCD_XSIZE_TFT - 60,                 60},
		{60,                 LCD_YSIZE_TFT - 60},
		{LCD_XSIZE_TFT - 60 ,LCD_YSIZE_TFT - 60},
		{LCD_XSIZE_TFT / 2, LCD_YSIZE_TFT / 2},
		{0, 0},
	};
	struct point adj[6];
	
	// return ;
	ReadFlash(FLASH_PAGE_START, 
		(uint32_t*)&(g_env), 
		sizeof(struct project_env));

	// 
	// 
	// 
	// 
	// 
	if(g_env.flag != 0xaabbccdd) 
	{
		// TC_Adj();
		adj[0].x = 2712; adj[0].y = 992;
		adj[1].x = 2899; adj[1].y = 3304;
		adj[2].x = 1149; adj[2].y = 868;
		adj[3].x = 1134; adj[3].y = 3309;
		adj[4].x = 2056; adj[4].y = 2089;
		tp_adj(pt, adj, &g_env.adj_tp);

		tp_setadj(&g_env.adj_tp);
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
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin		= GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA, GPIO_Pin_6 | GPIO_Pin_7);

	// 连续激光器 LED
	RCC_APB2PeriphClockCmd(RCC_LASTER, ENABLE);
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin		= PIN_LASTER;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_Out_PP;
	GPIO_Init(GROUP_LASTER, &GPIO_InitStructure);
	GPIO_SetBits(GROUP_LASTER, PIN_LASTER);


	// APD 电压选择 LED
	RCC_APB2PeriphClockCmd(RCC_LED_APD, ENABLE);
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin		= PIN_LED_APD_20V | PIN_LED_APD_40V;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_Out_PP;
	GPIO_Init(GROUP_LED_APD, &GPIO_InitStructure);
	GPIO_SetBits(GROUP_LED_APD, PIN_LED_APD_20V | PIN_LED_APD_40V);


	// 硬件有错误，没有用的脚，但是必须改成上拉输入
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_2MHz;
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
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin		= PIN_LD_OUTPUT_1;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_Out_PP;
	GPIO_Init(GROUP_LD1, &GPIO_InitStructure);
	GPIO_SetBits(GROUP_LD1, PIN_LD_OUTPUT_1);

	RCC_APB2PeriphClockCmd(RCC_LD2, ENABLE);
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin		= PIN_LD_OUTPUT_2;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_Out_PP;
	GPIO_Init(GROUP_LD2, &GPIO_InitStructure);
	GPIO_SetBits(GROUP_LD2, PIN_LD_OUTPUT_2);

	RCC_APB2PeriphClockCmd(RCC_LD3, ENABLE);
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_2MHz;
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
	TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
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
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_2MHz;
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
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_2MHz;
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
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin		= KEY_LT | KEY_RT | KEY_M | KEY_LB | KEY_RB;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_IPU;
	// GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_Out_PP;
	GPIO_Init(KEY_GPIO_X, &GPIO_InitStructure);
	// GPIO_SetBits(KEY_GPIO_X, KEY_LT | KEY_RT | KEY_M | KEY_LB | KEY_RB);
}

int32_t ScanKey()
{
	if(KeyPress(KEY_GPIO_X, KEY_LT)) {
		printf("KEY_LT\r\n");
		GUI_StoreKeyMsg(GUI_KEY_SHIFT_TAG,1);
		return 0;
	}
	else if(KeyPress(KEY_GPIO_X, KEY_RT)) {

		printf("KEY_RT\r\n");
		// GUI_StoreKeyMsg(GUI_KEY_RIGHT,1);
		return 0;
	}
	else if(KeyPress(KEY_GPIO_X, KEY_M)) {

		printf("KEY_M\r\n");
		GUI_StoreKeyMsg(GUI_KEY_ENTER,1);
		return 0;
	}
	else if(KeyPress(KEY_GPIO_X, KEY_LB)) {
		printf("KEY_LB\r\n");
		GUI_StoreKeyMsg(GUI_KEY_TAB,1);
		return 0;
	}
	else if(KeyPress(KEY_GPIO_X, KEY_RB)) {
		printf("KEY_RB\r\n");
		return 0;
	}
}

struct wm_glide glide;
extern volatile uint32_t g_en ,g_tickgui ;
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

void Test_LCD_L0_DrawBitmap_2BPP()
{
	int i;
	short index[4];
	// return ;
	g_lcd_test = 0;
	printf("----Test_LCD_L0_DrawBitmap_2BPP----");
	printf("start ...\n");


	index[0] = 0xff0000;
	index[1] = 0x00ff00;
	index[2] = 0x0000ff;
	index[3] = 0xf0f000;
	// GUI_Context.DrawMode |= LCD_DRAWMODE_TRANS;
	for (i = 0; i < 500; i++) {
		LCD_L0_DrawBitmap(0,0,
			64,64,
			2,
			16, 
			&acpwm[0],
			1,
			index);
	}
	printf("end %d\n", g_lcd_test);
}
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
		Delay_ms(200);
		Ctrl_LaserPower(CTRL_LASER_MV);
		Delay_ms(200);
		Ctrl_LaserPower(CTRL_LASER_LV);
		// Delay_ms(200);

		Ctrl_APD(CTRL_APD_20V);
		// Delay_ms(1000);
		Ctrl_APD(CTRL_APD_40V);
		// Delay_ms(1000);
	}
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
	
	
	printf("Draw UI\n");
	//LCD_DrawMain();
	// powerDownDelayCnt=0;
	// g_batter_delay = 10001;	

	

	GUI_Init();

	g_en = 1;
	TP_Init();
	TP_EINT();
	TP_GetADC(&pt);
	// TC_Adj();
	// TC_Adj();
	// Delay_ms(1000);
	FLASH_Configuration();
	TC_Test();
	
	//while(1);
	
	TEXT_SetDefaultTextColor(COL_ENABLE);
	BUTTON_GetDefaultTextColor(COL_ENABLE);
	BUTTON_SetDefaultBkColor(COL_BUTTON_BK, 0);
	BUTTON_SetDefaultBkColor(COL_BUTTON_BK, 1);
	BUTTON_SetDefaultTextColor(COL_ENABLE, 0);
	BUTTON_SetDefaultTextColor(COL_FOCUS, 1);
	hMain = MainTask();
	hleft = DestopBtn_Create(WM_HBKWIN, hMain);

	// MESSAGEBOX_Create("fds","df",GUI_MB_OK);
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
/**
******************************************************************************
* @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
* @author  MCD Application Team
* @version V3.4.0
* @date    10/15/2010
* @brief   Main Interrupt Service Routines.
*          This file provides template for all exceptions handler and 
*          peripherals interrupt service routine.
******************************************************************************
* @copy
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
*/ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "stm32f10x.h" 
#include "stm32f10x_dac.h" 
#include "stm32f10x_adc.h" 
#include "stm32f10x_tim.h" 
#include "stm32f10x_dma.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "key.h"
#include "gl_key.h"
#include "prj_type.h"
/** @addtogroup STM32F10x_StdPeriph_Template
* @{
*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

#define GPIO_CTRL_PORT_C 			GPIOC
#define GPIO_KEY_1310_CON 					   GPIO_Pin_3
#define GPIO_KEY_1490_CON 					   GPIO_Pin_2
#define GPIO_KEY_1550_CON 					   GPIO_Pin_1
extern volatile u8 g_red_onoff;
extern volatile int8_t g_red_mode ;
extern volatile u8 g_red_delay_100ms ;
extern volatile u8 g_key_timer_100ms ;
extern volatile uint16_t g_batter_delay ;
/* Private variables ---------------------------------------------------------*/
extern volatile u16 SysTickCounter;
extern volatile u16 powerDownDelayCnt;
//uint16_t ADCConvertedValue;	
extern volatile u8 FLAG_1310 ;		  //��ʱ���ж������巭ת��־��
extern volatile u8 FLAG_1490 ;
extern volatile u8 FLAG_1550 ;

extern volatile u16 Timer_State ;	    //��ʱ��״ָ̬ʾ��0 = OFF, 1 = ON
extern volatile u16 Timer_Counter ;

extern volatile u16 Wavelength_Selection_state ;	 //0 = �ر�״̬ 1 = 1310nm 2 = 1495nm 3 = 1550nm 	4 = ���
extern volatile u16 Operating_Mode ;	//0 = CW�� 1 = PW270Hz��2 = 1KHz��3 = 2KHz  ������/�����ѡ��
extern volatile int16_t Current_Power ;		//�ɵ�����״ָ̬ʾ��־λ��0-34��35���ȼ�����ʾ-30��5��dbmֵ��
extern volatile u8  ON_OFF_MARK ;		//���ػ���־λ����������1�����������ж�����0
extern volatile u8  Run_Stop ;          //���У�ֹͣʹ�ܣ���ʼʱ��ʹ�ܡ�
extern volatile u8  Keys_Shake ;		//�����ж�ȥ���������жϺ���1.��
extern volatile u8  Keys_Shake_Counter ;//����ȥ����SaysTick����ֵ����ʱ300ms������Ӧ������ѭ����ʱ����жϲ���ʱ���Ĺ�ϵ
//extern volatile u8  LCD_GetPoint_EN ;   //��ֹ�������ظ��������൱�ڰ���ȥ����,���Խ��д������ɼ���
extern volatile u8  LCD_GetPoint_Counter ;

/********************FLASH����****************************/
extern volatile uint16_t  Calibration_YES  ;	  //���ӽ����л�ʹ�ܶ˿ڣ��Ⱥ��¡���Դ ��Դ ����  UP  ��ʱ ģʽ down  ������7��ʹ��
extern volatile uint16_t  Calibrated_Number  ; 	  //��У׼�ĸ�����У׼֮�������ʱ���� DAC_Data[104] ��104��֮�����дFLASH����										  
extern volatile uint16_t  Save_EN  ;			  //DAC_Data����洢��ǰDAֵ�������ж�����1��ִ����֮����0
extern volatile u16 dacData ;
extern uint16_t DAC_Data[104];
/*********************************************************/

extern volatile u8 g_onoff_en;
/*
��ص�������ͨ��
*/
void ADC1_2_IRQHandler(void)
{		
}
#include "key.h"


//1ms��ʱ
extern volatile uint16_t g_delay_ms,g_adjust_ms,g_lcdbug_ms;
extern volatile uint8_t g_keya_flag;
extern volatile uint16_t g_ad_ms;
/**
* @brief  This function handles TIM5 global interrupt request.
* @param  None
* һ�㶨ʱ�ã���ʱ�ػ�����������Ϊ5min  ��10min  ��15min  ��30min  ��60min ״̬
TIM2����Ϊ100ms���ڣ�3000 �� 6000  �� 9000  �� 18000 ��  36000
*/
void TIM2_IRQHandler(void)	   //1310nm����
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		//SysTickCounter++;
		/*
		if(g_onoff_en == 0 ) {
			if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1)==0)
				g_onoff_en = 1;
		}
		else {
			if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1) == 1)
				powerDownDelayCnt++;
			else
				powerDownDelayCnt = 0;
		}*/		
		if(g_red_mode == 2) {
			g_red_delay_100ms++;
			if(g_red_delay_100ms >= 5) {
				g_red_onoff = !g_red_onoff;
				Ctrl_RedLight(g_red_onoff);
				//LCD_RedLight_Show(9,15,g_red_onoff);
				g_red_delay_100ms = 0;
			}
		}
		if(GPIO_ReadInputDataBit(GPIOA,KEY_A) == 0)
			g_key_timer_100ms++;
		else
			g_key_timer_100ms = 0;
			
		if(Timer_State != 0)	 //����ʱ����ʱ����ʼ��ʱ������
			Timer_Counter++ ; 
		
// 		if(LCD_GetPoint_EN == 0)	 
// 			LCD_GetPoint_Counter ++;

// 		if(LCD_GetPoint_Counter == 2)
// 		{ 
// 			LCD_GetPoint_EN = 1;
// 			LCD_GetPoint_Counter =0 ;
// 		}
		
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}


/**
* @brief  This function handles TIM5 global interrupt request.
* @param  None
* @retval None
*/
float g_sin[40] = {0,0.1564,0.3090,0.4540,0.5878,0.7071,0.8090,0.8910,0.9511,0.9877,
	1.0000,0.9877,0.9511,0.8910,0.8090,0.7071,0.5878,0.4540,0.3090,0.1564,0.0000,
-0.1564,-0.3090,-0.4540,-0.5878,-0.7071,-0.8090,-0.8910,-0.9511,-0.9877,-1.0000,
-0.9877,-0.9511,-0.8910,-0.8090,-0.7071,-0.5878,-0.4540,-0.3090,-0.1564};
void AdjustSin()
{
	static int i = 0;
	if(i++ >= 40) {
		i = 0;
	}
	DAC_SetChannel1Data(DAC_Align_12b_R, (uint16_t)(g_sin[i] * 1500 +1500 +300));    //  ��dacData/4096��*3.3== OUT
	DAC_SoftwareTriggerCmd(DAC_Channel_1,ENABLE); 
}

void TIM3_IRQHandler(void)	   //1310nm����
{
	
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{
		
		if(FLAG_1310)
		{
			GPIO_ResetBits(GPIO_CTRL_PORT_C,GPIO_KEY_1310_CON);
			FLAG_1310 = 0;
			//printf("\n\r 1310nm PC1 =%d\n",GPIO_ReadOutputDataBit(GPIO_CTRL_PORT_C,  GPIO_KEY_1310_CON));
		}
		else
		{
			GPIO_SetBits(GPIO_CTRL_PORT_C,GPIO_KEY_1310_CON);
			FLAG_1310 = 1;
			//printf("\n\r 1310nm PC1 =%d\n",GPIO_ReadOutputDataBit(GPIO_CTRL_PORT_C,  GPIO_KEY_1310_CON));
		}
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}

/**
* @brief  This function handles TIM5 global interrupt request.
* @param  None
* @retval None
*/
void TIM4_IRQHandler(void)	   //1490nm����
{

	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
	{
		if(FLAG_1490)
		{
			GPIO_ResetBits(GPIO_CTRL_PORT_C,GPIO_KEY_1490_CON);
			//printf("\n\r 1490nm PC1 =%d\n",GPIO_ReadOutputDataBit(GPIO_CTRL_PORT_C,  GPIO_KEY_1490_CON));
			FLAG_1490 = 0;
		}
		else
		{
			GPIO_SetBits(GPIO_CTRL_PORT_C,GPIO_KEY_1490_CON);
			//printf("\n\r 1490nm PC1 =%d\n",GPIO_ReadOutputDataBit(GPIO_CTRL_PORT_C,  GPIO_KEY_1490_CON));
			FLAG_1490 = 1;
		}
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
}

/**
* @brief  This function handles TIM5 global interrupt request.
* @param  None
* @retval None
*/
void TIM5_IRQHandler(void)	   //1550nm����
{

	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
	{
		if(FLAG_1550)
		{
			GPIO_ResetBits(GPIO_CTRL_PORT_C,GPIO_KEY_1550_CON);
			//printf("\n\r 1550nm PC1 =%d\n",GPIO_ReadOutputDataBit(GPIO_CTRL_PORT_C,  GPIO_KEY_1550_CON));
			FLAG_1550 = 0;
		}
		else
		{
			GPIO_SetBits(GPIO_CTRL_PORT_C,GPIO_KEY_1550_CON);
			//printf("\n\r 1550nm PC1 =%d\n",GPIO_ReadOutputDataBit(GPIO_CTRL_PORT_C,  GPIO_KEY_1550_CON));
			FLAG_1550 = 1;
		}
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	}
}

//1ms��ʱ
extern volatile uint16_t g_delay_ms,g_adjust_ms,g_lcdbug_ms,g_usart_ms,g_lcdlisten_ms,g_debug_ms;
extern volatile uint8_t g_keya_flag;
extern volatile uint16_t g_ad_ms;
extern volatile uint16_t g_touch_ms;
void TIM6_IRQHandler(void)
{
	#define RED_CW_HIGH 5
	#define RED_CW_LOW  3
	
	static u32 counter = 0,times = 0,redPlusTimer = 10;
	static uint8_t noKeyCounter = 0;
	
	/* www.armjishu.com ARM������̳ */
	if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
		
		
		if(g_onoff_en == 0 ) {
			if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1)==0)
				g_onoff_en = 1;
		}
		else {
			if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1) == 1)
				powerDownDelayCnt++;
			else
				powerDownDelayCnt = 0;
		}	
		g_ad_ms++;
		g_adjust_ms++;
		if(g_delay_ms)
			g_delay_ms--;
		g_batter_delay++;
		g_lcdbug_ms++;
		g_usart_ms++;
		g_lcdlisten_ms++;
		g_debug_ms++;

		if (g_touch_ms) {
			g_touch_ms--;
			if (g_touch_ms == 0) {
				TP_StartADC();
			}
		}
		//gl_key_scan();


		//�����⿪��һ��ʱ����������������200Hz���ҵ�����
		g_redbug_ms++;
		if(g_redbug_ms >= redPlusTimer && g_red_mode == 1) {
			g_redbug_ms = 0;
			//g_red_onoff = !g_red_onoff;
			if(redPlusTimer == RED_CW_HIGH) {
				redPlusTimer = RED_CW_LOW;
				g_red_onoff = 0;
			}
			else {
				redPlusTimer = RED_CW_HIGH;
				g_red_onoff = 1;
			}
			Ctrl_RedLight(g_red_onoff);
			//LCD_RedLight_Show(9,15,g_red_onoff);
			g_red_delay_100ms = 0;
			
		}
		
		
		//��ʱ���ް�����⣬ÿ100ms���һ��
		if(noKeyCounter++ >= 100) {
			noKeyCounter = 0;
			
			if( (GPIO_ReadInputData(GPIOA) & (KEY_A | KEY_B | KEY_C | KEY_Y | KEY_Z) ) !=
				(KEY_A | KEY_B | KEY_C | KEY_Y | KEY_Z)) {
					
				Timer_Counter = 0;
			}
			
		}
		
	}
}

/*CHECK1��CHECK3��CHECK4�жϺ�������*****************************************/
/* CHECK1�ֶ����ػ��жϺ����������ж���Ӧ0 */
/*void EXTI0_IRQHandler(void) 
{
	if(EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		
		Ctrl_RedLight(1);
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}*/

void EXTI1_IRQHandler(void) 
{
	if(EXTI_GetITStatus(EXTI_Line1) != RESET)
	{
		
		/* Clear the EXTI Line 0 */
		gl_key_exti(1);
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}
/* CHECK4ģʽѡ�񰴼�����  */
/*���ʵ��������ж� */
void EXTI2_IRQHandler(void) 
{
	if(EXTI_GetITStatus(EXTI_Line2) != RESET)
	{
		gl_key_exti(2);
		/* Clear the EXTI Line 2 */
		EXTI_ClearITPendingBit(EXTI_Line2);
	}
}

/* ��ʱѡ�񰴼�����  */
/* Timer_State 0 1 2 3 4 5 ��Ӧ�� OFF,  5min  ��10min  ��15min  ��30min  ��60min    */
void EXTI3_IRQHandler(void) 
{
	if(EXTI_GetITStatus(EXTI_Line3) != RESET)
	{
		gl_key_exti(3);
		EXTI_ClearITPendingBit(EXTI_Line3);
	}
}

/* CHECK3����ѡ�񰴼����ƣ�	*/
/* 0 = �ر�״̬  1 = 1310nm  2 = 1495nm  3 = 1550nm  4 = ���   Wavelength_Selection_state  */
void EXTI9_5_IRQHandler(void) 
{
	//����ѡ���ж�
	if(EXTI_GetITStatus(EXTI_Line5) != RESET)
	{
		//�������ƺ�����
		gl_key_exti(5);
		/* Clear the EXTI Line 7  */
		EXTI_ClearITPendingBit(EXTI_Line5);
	}
	//ģʽ�����ж�
	if(EXTI_GetITStatus(EXTI_Line7) != RESET)
	{
		gl_key_exti(7);
		/* Clear the EXTI Line 5  */
		EXTI_ClearITPendingBit(EXTI_Line7);
	}
	//���ʵݼ������жϿ���
	if(EXTI_GetITStatus(EXTI_Line6) != RESET)
	{
		gl_key_exti(6);
		/* Clear the EXTI Line 6  */
		EXTI_ClearITPendingBit(EXTI_Line6);
	}
}


/**
* @brief   This function handles NMI exception.
* @param  None
* @retval None
*/
void NMI_Handler(void)
{
}

/**
* @brief  This function handles Hard Fault exception.
* @param  None
* @retval None
*/
void HardFault_Handler(void)
{
	/* Go to infinite loop when Hard Fault exception occurs */
	while (1)
	{
	}
}

/**
* @brief  This function handles Memory Manage exception.
* @param  None
* @retval None
*/
void MemManage_Handler(void)
{
	/* Go to infinite loop when Memory Manage exception occurs */
	while (1)
	{
	}
}

/**
* @brief  This function handles Bus Fault exception.
* @param  None
* @retval None
*/
void BusFault_Handler(void)
{
	/* Go to infinite loop when Bus Fault exception occurs */
	while (1)
	{
	}
}

/**
* @brief  This function handles Usage Fault exception.
* @param  None
* @retval None
*/
void UsageFault_Handler(void)
{
	/* Go to infinite loop when Usage Fault exception occurs */
	while (1)
	{
	}
}

/**
* @brief  This function handles SVCall exception.
* @param  None
* @retval None
*/
void SVC_Handler(void)
{
}

/**
* @brief  This function handles Debug Monitor exception.
* @param  None
* @retval None
*/
void DebugMon_Handler(void)
{
}

/**
* @brief  This function handles PendSVC exception.
* @param  None
* @retval None
*/
void PendSV_Handler(void)
{
}




#include "usart.h"
extern int8_t g_recvflag;

void DMA1_Channel4_IRQHandler()
{
	if(DMA_GetITStatus(DMA1_IT_TC4) != RESET) {
		DMA_ClearITPendingBit(DMA1_IT_TC4);
	}
}

void DMA1_Channel5_IRQHandler()
{
	if(DMA_GetITStatus(DMA1_IT_TC5) != RESET) {
		printf("12");
		g_recvflag ++;
		/*if(rbufIndex == 1) {
			CommRecv(RxBuffer2,1024);
			rbufIndex = 2;
		}
		else if(rbufIndex == 2) {
			CommRecv(RxBuffer1,1024);
			rbufIndex = 1;
		}*/
		CommTurnBuffer();
		DMA_ClearITPendingBit(DMA1_IT_TC5);
	}
}

void DMA1_Channel6_IRQHandler()
{
	if(DMA_GetITStatus(DMA1_IT_TC6) != RESET) {
		printf("12");
		g_recvflag ++;
		/*if(rbufIndex == 1) {
			CommRecv(RxBuffer2,1024);
			rbufIndex = 2;
		}
		else if(rbufIndex == 2) {
			CommRecv(RxBuffer1,1024);
			rbufIndex = 1;
		}*/
		CommTurnBuffer();
		DMA_ClearITPendingBit(DMA1_IT_TC6);
	}
}
void DMA1_Channel7_IRQHandler()
{
	if(DMA_GetITStatus(DMA1_IT_TC7) != RESET) {
		DMA_ClearITPendingBit(DMA1_IT_TC7);
	}
}
void DMA1_Channel2_IRQHandler()
{
	if(DMA_GetITStatus(DMA1_IT_TC2) != RESET) {
		DMA_ClearITPendingBit(DMA1_IT_TC2);
	}
}
void DMA1_Channel3_IRQHandler()
{
	if(DMA_GetITStatus(DMA1_IT_TC3) != RESET) {
		printf("12");
		g_recvflag ++;
		/*if(rbufIndex == 1) {
			CommRecv(RxBuffer2,1024);
			rbufIndex = 2;
		}
		else if(rbufIndex == 2) {
			CommRecv(RxBuffer1,1024);
			rbufIndex = 1;
		}*/
		CommTurnBuffer();
		DMA_ClearITPendingBit(DMA1_IT_TC3);
	}
}


/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
* @brief  This function handles PPP interrupt request.
* @param  None
* @retval None
*/
/*void PPP_IRQHandler(void)
{
}*/

/**
* @}
*/ 


/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/

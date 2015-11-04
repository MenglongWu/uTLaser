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

// extern volatile u8 g_red_onoff;
// extern volatile int8_t g_red_mode ;
// extern volatile u8 g_red_delay_100ms ;
// extern volatile u8 g_key_timer_100ms ;
// extern volatile uint16_t g_batter_delay ;
/* Private variables ---------------------------------------------------------*/
// extern volatile u16 SysTickCounter;
// extern volatile u16 powerDownDelayCnt;


/*********************************************************/

extern volatile u8 g_onoff_en;
/*
电池电量采样通道
*/
void ADC1_2_IRQHandler(void)
{		
}



//1ms定时
extern volatile uint16_t g_delay_ms;

void TIM2_IRQHandler(void)	   //1310nm控制
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{	
		// TODO
		// if(g_red_mode == 2) {
		// 	g_red_delay_100ms++;
		// 	if(g_red_delay_100ms >= 5) {
		// 		g_red_onoff = !g_red_onoff;
		// 		// Ctrl_RedLight(g_red_onoff);
		// 		//LCD_RedLight_Show(9,15,g_red_onoff);
		// 		g_red_delay_100ms = 0;
		// 	}
		// }		
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}


void TIM3_IRQHandler(void)	   //1310nm控制
{
	
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{
		// TODO
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}

/**
* @brief  This function handles TIM5 global interrupt request.
* @param  None
* @retval None
*/
void TIM4_IRQHandler(void)	   //1490nm控制
{

	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
	{
		// TODO
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
}

/**
* @brief  This function handles TIM5 global interrupt request.
* @param  None
* @retval None
*/
void TIM5_IRQHandler(void)	   //1550nm控制
{
#ifdef _DEBUG_
	static int t = 0;
#endif
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
	{
		if(g_delay_ms)
			g_delay_ms--;
#ifdef _DEBUG_
		if (t++ >= 1000) {
			printf("TIM5 IRQ\n");
			t = 0;
		}
#endif
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	}
}

extern volatile uint16_t g_touch_ms;
volatile uint32_t g_en = 0,g_tickgui = 0;
void TIM6_IRQHandler(void)
{
#ifdef _DEBUG_
	static int t = 0;
#endif
	if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
	{
		
#ifdef _DEBUG_
		if (t++ >= 1000) {
			printf("TIM5 IRQ\n");
			t = 0;
		}
#endif
		if (g_en && g_tickgui++ > 20) {
			ScanKey();
			GUI_Exec();		
			g_tickgui = 0;
		}

		if (g_touch_ms) {
			g_touch_ms--;
			if (g_touch_ms == 0) {
				TP_StartADC();
			}
		}
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
		
	}
}

/*CHECK1、CHECK3、CHECK4中断函数部分*****************************************/
/* CHECK1手动开关机中断函数，利用中断响应0 */
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
/* CHECK4模式选择按键控制  */
/*功率递增控制中断 */
void EXTI2_IRQHandler(void) 
{
	if(EXTI_GetITStatus(EXTI_Line2) != RESET)
	{
		gl_key_exti(2);
		/* Clear the EXTI Line 2 */
		EXTI_ClearITPendingBit(EXTI_Line2);
	}
}

/* 定时选择按键控制  */
/* Timer_State 0 1 2 3 4 5 对应： OFF,  5min  ，10min  ，15min  ，30min  ，60min    */
void EXTI3_IRQHandler(void) 
{
	if(EXTI_GetITStatus(EXTI_Line3) != RESET)
	{
		gl_key_exti(3);
		EXTI_ClearITPendingBit(EXTI_Line3);
	}
}

/* CHECK3波长选择按键控制，	*/
/* 0 = 关闭状态  1 = 1310nm  2 = 1495nm  3 = 1550nm  4 = 红光   Wavelength_Selection_state  */
void EXTI9_5_IRQHandler(void) 
{
	//波长选择中断
	if(EXTI_GetITStatus(EXTI_Line5) != RESET)
	{
		//按键控制红光输出
		gl_key_exti(5);
		/* Clear the EXTI Line 7  */
		EXTI_ClearITPendingBit(EXTI_Line5);
	}
	//模式调节中断
	if(EXTI_GetITStatus(EXTI_Line7) != RESET)
	{
		gl_key_exti(7);
		/* Clear the EXTI Line 5  */
		EXTI_ClearITPendingBit(EXTI_Line7);
	}
	//功率递减控制中断控制
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
// extern int8_t g_recvflag;

void DMA1_Channel4_IRQHandler()
{
	if(DMA_GetITStatus(DMA1_IT_TC4) != RESET) {
		DMA_ClearITPendingBit(DMA1_IT_TC4);
	}
}

void DMA1_Channel5_IRQHandler()
{
	if(DMA_GetITStatus(DMA1_IT_TC5) != RESET) {
		// printf("12");
		// g_recvflag ++;
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
		// printf("12");
		// g_recvflag ++;
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
		// printf("12");
		// g_recvflag ++;
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

/****************************************Copyright (c)****************************************************
**                                      
**                                 http://www.powermcu.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               TouchPanel.h
** Descriptions:            The TouchPanel application function
**
**--------------------------------------------------------------------------------------------------------
** Created by:              AVRman
** Created date:            2010-11-7
** Version:                 v1.0
** Descriptions:            The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
** Version:                 
** Descriptions:            
**
*********************************************************************************************************/

/**
 ******************************************************************************
 * @file	ADS7843.h
 * @brief	ADS7843 触屏芯片基于STM32F103驱动

TODO:introduce 
 *
 @section Platform
	-# 
 @section Library
	-# 
- 2015-9-7,MenglongWoo,MenglongWoo@163.com
 	- brief

*/

#ifndef _ADS7843_H_
#define _ADS7843_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/
/* AD通道选择命令字和工作寄存器 */
#define	CHX 	(0x90)	/* 通道Y+的选择控制字 */	
#define	CHY 	(0xd0) 	/* 通道X+的选择控制字 */



#define LCD_XSIZE_TFT 320
#define LCD_YSIZE_TFT 240
 	
#define TP_CS(x)	x ? GPIO_SetBits(GPIOB,GPIO_Pin_5): GPIO_ResetBits(GPIOB,GPIO_Pin_5)

#define TP_INT_IN   GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)

/* Private function prototypes -----------------------------------------------*/			
void TP_Init(void);	

#endif

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/



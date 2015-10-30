/**
 ******************************************************************************
 * @file	
 * @author	MenglongWu
 * @version	V1.0
 * @date	2012.10.14
 * @brief	作者：吴梦龙\n
 移植配置文件，只需要修改两个地方\n
	1、使用到的字库\n
	2、相关设备的画点函数用GL_SETPOINT宏替换\n
	3、手持式稳定光源移植
 ******************************************************************************
 * @attention
 *
 * ATTENTION
*
* <h2><center>&copy; COPYRIGHT </center></h2>
******************************************************************************
*/

#ifndef _GL_UI_CONFIG_H_
#define _GL_UI_CONFIG_H_

//选择需要的字库
//#include "lcd\\zimo_yh16.h"//微软雅黑16
#include "lcd\\zimo_st9.h"//宋体9
#include "lcd\\zimo_st20.h"//宋体20
//#include "lcd\\led19264.h"//底层硬件接口
//使用到的显示设备
//#include "glcd.h"
#include "..\\USER\\GLCD\\glcd.h"
//#include "..\\

//显示设备接口
#define GL_UI_OLD_DEV//为了兼容以前编写的老设备，接口不一样

//#define GL_SETPOINT(x,y,color) LCD_SetPoint(x,y,color)//LED_Set_Point(x,y)//手持稳定光源显示器画点函数
#define GL_SETPOINT(x,y,color) LCD_WriteData(color)//LED_Set_Point(x,y)//手持稳定光源显示器画点函数

////////////////////////////////////////////////////

#endif


/**
 ******************************************************************************
 * @file	
 * @author	MenglongWu
 * @version	V1.0
 * @date	2012.10.14
 * @brief	���ߣ�������\n
 ��ֲ�����ļ���ֻ��Ҫ�޸������ط�\n
	1��ʹ�õ����ֿ�\n
	2������豸�Ļ��㺯����GL_SETPOINT���滻\n
	3���ֳ�ʽ�ȶ���Դ��ֲ
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

//ѡ����Ҫ���ֿ�
//#include "lcd\\zimo_yh16.h"//΢���ź�16
#include "lcd\\zimo_st9.h"//����9
#include "lcd\\zimo_st20.h"//����20
//#include "lcd\\led19264.h"//�ײ�Ӳ���ӿ�
//ʹ�õ�����ʾ�豸
//#include "glcd.h"
#include "..\\USER\\GLCD\\glcd.h"
//#include "..\\

//��ʾ�豸�ӿ�
#define GL_UI_OLD_DEV//Ϊ�˼�����ǰ��д�����豸���ӿڲ�һ��

//#define GL_SETPOINT(x,y,color) LCD_SetPoint(x,y,color)//LED_Set_Point(x,y)//�ֳ��ȶ���Դ��ʾ�����㺯��
#define GL_SETPOINT(x,y,color) LCD_WriteData(color)//LED_Set_Point(x,y)//�ֳ��ȶ���Դ��ʾ�����㺯��

////////////////////////////////////////////////////

#endif


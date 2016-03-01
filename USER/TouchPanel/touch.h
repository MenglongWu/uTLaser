/**
 ******************************************************************************
 * @file	touch.h
 * @brief	声明触屏公共接口，以及驱动层相关的接口

TODO:introduce 
 *
 @section Platform
	-# 
 @section Library
	-# 
- 20xx-xx-xx,author,email
 	- brief

 * @attention
 *
 * ATTENTION
 *
 * <h2><center>&copy; COPYRIGHT </center></h2>
*/

#ifndef _TOUCH_H_
#define _TOUCH_H_

// 坐标值以及触屏AD值
struct point  
{
   unsigned short  x;
   unsigned short  y;
};

// 校准算法
struct adj_tp
{
	float K;
	float A1;
	float B1;
	float C1;
	float A2;
	float B2;
	float C2;
};


// 
// 与硬件相关
// 

/**
 * @brief	获取触屏AD值
 * @param	pt x，y两线AD值
 * @retval	1 有触屏
 * @retval	0 无触屏
 */
int gettouch(struct point *pt);
void tp_adj(struct point *lcd, struct point *tp, struct adj_tp *retadj);
void TC_Test();
void tp_getadj(struct adj_tp *adj);
void tp_setadj(struct adj_tp *adj);	
void TC_Adj();

#endif

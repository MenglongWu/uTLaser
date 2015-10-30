/**
 ******************************************************************************
 * @file	touch.c
 * @brief	触屏公共接口

TODO:introduce 
 *
 @section Platform
	-# 
 @section Library
	-# 
- 20xx-xx-xx,author,email
 	- brief
*/

#include "touch.h"



/**
 * @brief	检查触屏点是否有效
 * @param	null
 * @retval	null
 * @remarks	
 * @see	
 */

void tp_availabili()
{
	
}
/**
 * @brief	3点校准算法
 * @param	lcd lcd坐标
 * @param	tp 触屏坐标
 * @retval	retadj
 * @remarks	
 * @see	
 */
void tp_adj(struct point *lcd, struct point *tp, struct adj_tp *retadj)
{
#define _Xd(c) ( (lcd+c)->x)
#define _Yd(c) ( (lcd+c)->y)
#define _X(c) ( (tp+c)->x)
#define _Y(c) ( (tp+c)->y)

#define K	(retadj->K)
#define A1	(retadj->A1)
#define B1	(retadj->B1)
#define C1	(retadj->C1)
#define A2	(retadj->A2)
#define B2	(retadj->B2)
#define C2	(retadj->C2)


	// 3 点校准做算法
	K = (_X(0) - _X(2)) * (_Y(1) - _Y(2)) - (_X(1) - _X(2)) * (_Y(0) - _Y(2));

	A1 = ( (_Xd(0) - _Xd(2)) * (_Y(1) - _Y(2)) - (_Xd(1) - _Xd(2)) * (_Y(0) - _Y(2)) ) / K;
	B1 = ( (_X(0) - _X(2)) * (_Xd(1) - _Xd(2)) - (_Xd(0) - _Xd(2)) * (_X(1) - _X(2)) ) / K;
	C1 = ( _Y(0) * (_X(2) * _Xd(1) - _X(1) * _Xd(2)) +
	    _Y(1) * (_X(0) * _Xd(2) - _X(2) * _Xd(0)) +
	    _Y(2) * (_X(1) * _Xd(0) - _X(0) * _Xd(1))) / K;

	A2 = ( (_Yd(0) - _Yd(2)) * (_Y(1) - _Y(2)) - (_Yd(1) - _Yd(2)) * (_Y(0) - _Y(2)) ) / K;
	B2 = ( (_X(0) - _X(2)) * (_Yd(1) - _Yd(2)) - (_Yd(0) - _Yd(2)) * (_X(1) - _X(2)) ) / K;
	C2 = ( _Y(0) * (_X(2) * _Yd(1) - _X(1) * _Yd(2)) +
	    _Y(1) * (_X(0) * _Yd(2) - _X(2) * _Yd(0)) +
	    _Y(2) * (_X(1) * _Yd(0) - _X(0) * _Yd(1))) / K;

	// printf("\n[%d %d %d %d %d %d %d ]\n", (
	// 	(int)(A1*1000), (int)(B1*1000), (int)(C1*1000), 
	// 	(int)(A2*1000), (int)(B2*1000), (int)(C2*1000), 
	// 	(int)(K*1000)));
}



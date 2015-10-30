#include "key.h"
//Function define
//移植需要设定按键按下时端口的电平Key_Signal
//#define Key_Signal 1//下拉电阻输入
#define Key_Signal 0//上拉电阻输入
volatile uint16_t g_delay_ms = 0;

/**
* @brief	阻塞方式延时
* @param	ms 延时单位为ms，最长延时65536
* @remark 延时计数g_delay_ms需要放在1KHz中断里面，每次中断里对g_delay_ms减1
*			直至为0时Delay_ms返回\n
*			必须在调用Delay_ms前初始化相应中断
*/
void Delay_ms(uint16_t ms)
{
	g_delay_ms = ms;
	while(g_delay_ms);
}
/**
* @brief	获取按键是长按
* @param	GPIOx 按键端口组
* @param	kpin  端口引脚
* @retval\n 1 表示按下\n0 表示没按下
* @remark	如果按键一直长按\n
*			第1次调用KeyPress会在480ms后返回\n
*			第n次调用会在180ms后返回
*/
int8_t KeyPress(GPIO_TypeDef* GPIOx,uint16_t kpin)
{
	static GPIO_TypeDef *oldGPIOx = 0;//上次处理的端口组
	static uint16_t oldkpin = 0;//上一次处理的端口引脚
	static uint8_t secend = 0;  //第二个字符显示标志

	//if((GPIOx->IDR & kpin) == 0) {
	if(GPIO_ReadInputDataBit(GPIOx,kpin) == Key_Signal) {
		Delay_ms(60);
		//if((GPIOx->IDR & kpin) == 0) {
		if(GPIO_ReadInputDataBit(GPIOx,kpin) == Key_Signal) {
			//dprintf("Port = %x Pin = %x\n",oldGPIOx,oldkpin);
			if(oldGPIOx == GPIOx && oldkpin == kpin) {
				if(secend == 1) 
					Delay_ms(480);//重复延时，第一次keydown事件在按下410ms后
				else
					//Delay_ms(220);//重复速度，第二次开始每秒10次keydown事件
					Delay_ms(180);//重复速度，第二次开始每秒10次keydown事件
				secend = 2;
				//if((GPIOx->IDR & kpin) != 0) {
				if(GPIO_ReadInputDataBit(GPIOx,kpin) != Key_Signal) {
					oldGPIOx = 0;oldkpin = 0;
					secend = 0;
					return 0;
				}
				//dprintf("sec\n");
				return 2;
			}
			else {
				oldGPIOx = GPIOx;
				oldkpin = kpin;
				secend = 1;
				//printf("first\n");
				return 1;
			}
		}
	}
	//dprintf("byebye\n");
	//oldGPIOx = 0;oldkpin = 0;
	return 0;
}


/**
* @brief	判断按键是否按下
* @param	GPIOx 按键端口组
* @param	kpin  端口引脚
* @retval 0 表示没按下
* @retval 1 表示按下
*/

int8_t KeyDown(GPIO_TypeDef* GPIOx,uint16_t kpin)
{
	static GPIO_TypeDef* oldGPIOx = 0;
	static uint16_t oldkpin = 0;
	
	if(kpin == 0 && GPIOx == 0) {
		oldGPIOx = 0;
		oldkpin = 0;
		return 1;
	}
	if(oldGPIOx == GPIOx && oldkpin == kpin) {
		if(GPIO_ReadInputDataBit(GPIOx,kpin) == Key_Signal) {
			return 0;
		}
		else {
			oldGPIOx = 0;
			oldkpin = 0;
		}
	}
	//if((GPIOx->IDR & kpin) == 0) {
	if(GPIO_ReadInputDataBit(GPIOx,kpin) == Key_Signal) {
		Delay_ms(40);
		//if((GPIOx->IDR & kpin) == 0) {
		if(GPIO_ReadInputDataBit(GPIOx,kpin) == Key_Signal) {
			oldGPIOx = GPIOx ;
			oldkpin = kpin;
			//while(GPIO_ReadInputDataBit(GPIOx,kpin) == Key_Signal);
			
			return 1;
		}
	}
	return 0;
}

/**
* @brief	获取按键是否按下
* @param	GPIOx 按键端口组
* @param	kpin  端口引脚
* @retval\n	0表示没有按下否则返回按下的时间（ms）
* @remarks 是KeyDown的扩展功能，能返回该按键长按的时间（ms），
*		最长时间为500ms。当一个按键有按下、长按若干S时用它实现
*/
int16_t KeyDown_Ex(GPIO_TypeDef* GPIOx,uint16_t kpin)
{
	static GPIO_TypeDef* oldGPIOx = 0;
	static uint16_t oldkpin = 0;
	uint16_t i;
	
// 	if(kpin == 0 && GPIOx == 0) {
// 		oldGPIOx = 0;
// 		oldkpin = 0;
// 		return 1;
// 	}
// 	if(oldGPIOx == GPIOx && oldkpin == kpin) {
// 		if(GPIO_ReadInputDataBit(GPIOx,kpin) == Key_Signal) {
// 			return 0;
// 		}
// 		else {
// 			oldGPIOx = 0;
// 			oldkpin = 0;
// 		}
// 	}
	//if((GPIOx->IDR & kpin) == 0) {

	i = 0;
	while(i < 500 && GPIO_ReadInputDataBit(GPIOx,kpin) == Key_Signal) {
		Delay_ms(10);
		i+= 10;
	}
	return i;
}
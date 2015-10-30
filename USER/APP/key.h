#ifndef _KEY_H_
#define _KEY_H_
#include "stm32f10x.h"
/*°´¼ü*/
#define KEY_A GPIO_Pin_5
#define KEY_B GPIO_Pin_3
#define KEY_C GPIO_Pin_2
#define KEY_X GPIO_Pin_1
#define KEY_Y GPIO_Pin_7
#define KEY_Z GPIO_Pin_6

#define KEY_PORT_A GPIOA
#define KEY_PORT_B GPIOA
#define KEY_PORT_C GPIOA
#define KEY_PORT_X GPIOA
#define KEY_PORT_Y GPIOA
#define KEY_PORT_Z GPIOA

void Delay_ms(uint16_t ms);
int8_t KeyPress(GPIO_TypeDef* GPIOx,uint16_t kpin);
int8_t KeyDown(GPIO_TypeDef* GPIOx,uint16_t kpin);
int16_t KeyDown_Ex(GPIO_TypeDef* GPIOx,uint16_t kpin);
#endif


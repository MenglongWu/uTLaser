#ifndef _GL_KEY_H_
#define _GL_KEY_H_
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"


//定义最多按键数，这里的按键数表示外部中断数
//如果有更多的按键，需要多中断组成扫描矩阵
#define MAXKEY 10

extern struct gl_key _key[MAXKEY];
#define GL_KS_NO     0
#define GL_KS_DOWN   1
#define GL_KS_UP     2
#define GL_KS_PRESS  3

#define KEY_1_PORT GPIOA
#define KEY_2_PORT GPIOA
#define KEY_3_PORT GPIOA
#define KEY_4_PORT GPIOA
#define KEY_5_PORT GPIOA
#define KEY_6_PORT GPIOA
#define KEY_7_PORT GPIOA
#define KEY_8_PORT GPIOA
#define KEY_9_PORT GPIOA

#define KEY_1_PIN GPIO_Pin_1
#define KEY_2_PIN GPIO_Pin_2
#define KEY_3_PIN GPIO_Pin_3
#define KEY_4_PIN GPIO_Pin_4
#define KEY_5_PIN GPIO_Pin_5
#define KEY_6_PIN GPIO_Pin_6
#define KEY_7_PIN GPIO_Pin_7
#define KEY_8_PIN GPIO_Pin_8
#define KEY_9_PIN GPIO_Pin_9

#define KEY_1_PINSOURCE GPIO_PinSource1
#define KEY_2_PINSOURCE GPIO_PinSource1
#define KEY_3_PINSOURCE GPIO_PinSource1
#define KEY_4_PINSOURCE GPIO_PinSource1
#define KEY_5_PINSOURCE GPIO_PinSource1
#define KEY_6_PINSOURCE GPIO_PinSource1
#define KEY_7_PINSOURCE GPIO_PinSource1
#define KEY_8_PINSOURCE GPIO_PinSource1
#define KEY_9_PINSOURCE GPIO_PinSource1


#define KEY_IS_DOWN(port,pin) (GPIO_ReadInputDataBit((GPIO_TypeDef*)port,pin) == 0)
#define KEY_IS_UP(port,pin) (GPIO_ReadInputDataBit((GPIO_TypeDef*)port,pin) == 1)
#define KD_FLAG 0x1000
#define KU_FLAG 0x7fff

extern uint32_t _key_ms;
extern uint32_t _key_ms_out;
struct gl_key
{
	uint8_t ks;//按键状态
	uint16_t exticode;//中断
	uint16_t vk;//虚拟键码
	
	uint16_t sharke;
	uint32_t port;
	uint32_t pin;
};

void gl_key_init(void);
uint8_t getkeymsg(void);
void gl_key_scan(void);
uint8_t gl_key_down(uint8_t vk);
void gl_key_exti(uint8_t line);
#endif


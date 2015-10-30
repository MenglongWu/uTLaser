/**
 ******************************************************************************
 * @file	gl_key.c
 * @author	MenglongWu
 * @version	V1.0
 * @date	2013.10.22
 * @brief	作者：吴梦龙\n
 按键消息队列。用中断方式记录按键历史，具备按键按下、按键抬起、双击、单击多种
 消息，在gl_key.h里配置按键按键映射。
 ******************************************************************************
 * @attention
 *
 * ATTENTION
*
* <h2><center>&copy; COPYRIGHT </center></h2>
******************************************************************************
*/
#include "gl_key.h"

uint32_t _key_ms = 0;
uint32_t _key_ms_out = 0;

struct gl_key _key[10] = {0};
uint8_t msg[100];
uint8_t msgindex = 0;
uint8_t msgread = 0;

void gl_key_init()
{
	uint8_t i;
	uint32_t rcc;
	EXTI_InitTypeDef   EXTI_InitStructure;
	GPIO_InitTypeDef   GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	
	//移植：这里定义需要扫描的端口和端口组，此端口必须是外部中断端口
	rcc = RCC_APB2Periph_GPIOA;
	_key[1].port = (uint32_t)GPIOA;
	_key[1].pin  = GPIO_Pin_1;
	_key[2].port = (uint32_t)GPIOA;
	_key[2].pin  = GPIO_Pin_2;
	_key[3].port = (uint32_t)GPIOA;
	_key[3].pin  = GPIO_Pin_3;
	_key[5].port = (uint32_t)GPIOA;
	_key[5].pin  = GPIO_Pin_5;
	_key[6].port = (uint32_t)GPIOA;
	_key[6].pin  = GPIO_Pin_6;
	_key[7].port = (uint32_t)GPIOA;
	_key[7].pin  = GPIO_Pin_7;
	
	
	//端口方向
	RCC_APB2PeriphClockCmd(rcc, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	for(i = 0;i < MAXKEY;++i) {
		_key[i].sharke = 100;
		if(_key[i].pin != 0) {//没有在上面定义的端口，默认初始化都是0
			GPIO_InitStructure.GPIO_Pin = _key[i].pin;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
			GPIO_Init((GPIO_TypeDef*)_key[i].port, &GPIO_InitStructure);
			
			EXTI_InitStructure.EXTI_Line = _key[i].pin;//EXTI_Linex 和 GPIO_Pin_x相同
			EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
			EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; 
			EXTI_InitStructure.EXTI_LineCmd = ENABLE;
			EXTI_Init(&EXTI_InitStructure);
		}
	}
	
	//外部中断属性
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//移植：添加使用到的中断
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_Init(&NVIC_InitStructure);
}
uint8_t getkeymsg()
{
}
void gl_key_scan()
{
	char strout[30];
	static uint32_t lastdown = 0,lastup = 0,times = 0;
	uint8_t i;
	_key_ms_out++;
	
	if(_key_ms++ >= 10) {
		_key_ms = 0;
		for(i = 0;i < MAXKEY;++i) {
			if(_key[i].ks == GL_KS_DOWN && //外部中断检测到按下状态
				KEY_IS_DOWN(_key[i].port,_key[i].pin) &&//10ms去抖动后再次检测按键	按下状态
				_key_ms_out - lastup > _key[i].sharke) {//保证按下状态为每秒10次
				
				lastup = _key_ms_out;
				_key[i].vk = i;// | KD_FLAG);
				//_key[i].ks = GL_KS_UP;
				msgindex++;
				if(times == 1)
					_key[i].sharke  = 500;
				else
					_key[i].sharke  = 100;
				times = 2;
				printf("%d     down\n",i);
			}
			else if(_key[i].ks == GL_KS_UP   && KEY_IS_UP(_key[i].port,_key[i].pin)) {
 				//_key[i].vk = i;
				_key[i].ks = GL_KS_NO;
 				printf("%d    up\n",i);
				_key[i].sharke = 100;
				times = 1;
 			}			
		}
	}
}

uint8_t gl_key_down(uint8_t vk)
{
	if(_key[vk].vk == vk){//(vk | KD_FLAG)) {
		_key[vk].vk = 0;
		return 1;
	}
	return 0;
}

void gl_key_exti(uint8_t line)
{
	static uint32_t lastdown = 0,lastup = 0;	
	if(/*_key_ms_out - lastup > 5 &&*/ KEY_IS_UP(_key[line].port,_key[line].pin)) {
		lastup = _key_ms_out;
		_key[line].ks = GL_KS_UP;
	}
	else if(/*_key_ms_out - lastdown > 5 && */KEY_IS_DOWN(_key[line].port,_key[line].pin)) {
		lastdown = _key_ms_out;
		_key[line].ks = GL_KS_DOWN;
	}
}
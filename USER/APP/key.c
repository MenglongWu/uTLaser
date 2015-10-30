#include "key.h"
//Function define
//��ֲ��Ҫ�趨��������ʱ�˿ڵĵ�ƽKey_Signal
//#define Key_Signal 1//������������
#define Key_Signal 0//������������
volatile uint16_t g_delay_ms = 0;

/**
* @brief	������ʽ��ʱ
* @param	ms ��ʱ��λΪms�����ʱ65536
* @remark ��ʱ����g_delay_ms��Ҫ����1KHz�ж����棬ÿ���ж����g_delay_ms��1
*			ֱ��Ϊ0ʱDelay_ms����\n
*			�����ڵ���Delay_msǰ��ʼ����Ӧ�ж�
*/
void Delay_ms(uint16_t ms)
{
	g_delay_ms = ms;
	while(g_delay_ms);
}
/**
* @brief	��ȡ�����ǳ���
* @param	GPIOx �����˿���
* @param	kpin  �˿�����
* @retval\n 1 ��ʾ����\n0 ��ʾû����
* @remark	�������һֱ����\n
*			��1�ε���KeyPress����480ms�󷵻�\n
*			��n�ε��û���180ms�󷵻�
*/
int8_t KeyPress(GPIO_TypeDef* GPIOx,uint16_t kpin)
{
	static GPIO_TypeDef *oldGPIOx = 0;//�ϴδ���Ķ˿���
	static uint16_t oldkpin = 0;//��һ�δ���Ķ˿�����
	static uint8_t secend = 0;  //�ڶ����ַ���ʾ��־

	//if((GPIOx->IDR & kpin) == 0) {
	if(GPIO_ReadInputDataBit(GPIOx,kpin) == Key_Signal) {
		Delay_ms(60);
		//if((GPIOx->IDR & kpin) == 0) {
		if(GPIO_ReadInputDataBit(GPIOx,kpin) == Key_Signal) {
			//dprintf("Port = %x Pin = %x\n",oldGPIOx,oldkpin);
			if(oldGPIOx == GPIOx && oldkpin == kpin) {
				if(secend == 1) 
					Delay_ms(480);//�ظ���ʱ����һ��keydown�¼��ڰ���410ms��
				else
					//Delay_ms(220);//�ظ��ٶȣ��ڶ��ο�ʼÿ��10��keydown�¼�
					Delay_ms(180);//�ظ��ٶȣ��ڶ��ο�ʼÿ��10��keydown�¼�
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
* @brief	�жϰ����Ƿ���
* @param	GPIOx �����˿���
* @param	kpin  �˿�����
* @retval 0 ��ʾû����
* @retval 1 ��ʾ����
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
* @brief	��ȡ�����Ƿ���
* @param	GPIOx �����˿���
* @param	kpin  �˿�����
* @retval\n	0��ʾû�а��·��򷵻ذ��µ�ʱ�䣨ms��
* @remarks ��KeyDown����չ���ܣ��ܷ��ظð���������ʱ�䣨ms����
*		�ʱ��Ϊ500ms����һ�������а��¡���������Sʱ����ʵ��
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
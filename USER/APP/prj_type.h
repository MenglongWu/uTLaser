#ifndef _PROJ_TYPE_H_
#define _PROJ_TYPE_H_
#include "stdint.h"
#include "stm32f10x.h"
#include "key.h"

//�ض������

#define Key_Port_Set GPIOA
#define Key_Pin_Set KEY_A

#define Key_Port_Enter GPIOA
#define Key_Pin_Enter KEY_A

#define Key_Port_Up GPIOA
#define Key_Pin_Up KEY_A

#define Key_Port_Down GPIOA
#define Key_Pin_Down KEY_A

#define Key_Port_Left GPIOA
#define Key_Pin_Left KEY_A

#define Key_Port_Right GPIOA
#define Key_Pin_Right KEY_A

//���浱ǰ����������ģʽ�µ�adc��dacֵ�������û�����
struct fast_switch
{
	int32_t enable;
	int32_t set;

	//CH1
	uint16_t ch1_adc_off;///<����У׼ch1
	uint16_t ch1_adc_cw;///<
	uint16_t ch1_adc_270;///<
	uint16_t ch1_adc_1k;///<
	uint16_t ch1_adc_2k;///<

	uint16_t ch1_dac_off;
	uint16_t ch1_dac_cw;
	uint16_t ch1_dac_270;
	uint16_t ch1_dac_1k;
	uint16_t ch1_dac_2k;

	//CH2
	uint16_t ch2_adc_off;
	uint16_t ch2_adc_cw;
	uint16_t ch2_adc_270;
	uint16_t ch2_adc_1k;
	uint16_t ch2_adc_2k;

	uint16_t ch2_dac_off;
	uint16_t ch2_dac_cw;
	uint16_t ch2_dac_270;
	uint16_t ch2_dac_1k;
	uint16_t ch2_dac_2k;

	//CH3
	uint16_t ch3_adc_off;
	uint16_t ch3_adc_cw;
	uint16_t ch3_adc_270;
	uint16_t ch3_adc_1k;
	uint16_t ch3_adc_2k;

	uint16_t ch3_dac_off;
	uint16_t ch3_dac_cw;
	uint16_t ch3_dac_270;
	uint16_t ch3_dac_1k;
	uint16_t ch3_dac_2k;
};
struct ctrl_param//��Ҫ���ƵĲ���
{
	int32_t lastset;
	int32_t set; ///<����ֵ��������1000��
	int32_t cur; ///<ʵ��ֵ
	uint16_t adc;///<dac
	uint16_t dac;
	int8_t  ld;
	uint16_t vol;
	uint8_t level;//�ܿ��Ƶļ���
	int32_t temp;
};

#include "user/touchpanel/touch.h"
struct project_env
{
	unsigned long flag;
	struct adj_tp adj_tp;
};
struct adj_power_flash
{
	uint32_t flag;///<��־������Ϊ0xAABBCCDDʱ���ʾ�����������Ч������������������

	float _adc;///<У׼ADCֵ����¼�����10dBmʱ��ADCֵ��@see Ctrl_Power

	//float _1310_270;//DAC
	float _dac;///<У׼DACֵ����¼�����10dBmʱ��DACֵ������Զ������ٶȣ�@see AutoCtrlPower

	//////��������
	float _1310_1k;
	float _1310_2k;
	
	float _1550cw;
	float _1550_270;
	float _1550_1k;
	float _1550_2k;
	//////////
	uint8_t sn[28];
	uint8_t _650_en;///<650ʹ��
	uint8_t _1310_en;///<ch1��1310��ʹ��
	uint8_t _1490_en;///<ch2��1490��ʹ��
	uint8_t _1550_en;///<ch3��1550��ʹ��
	
	
	//����
	uint32_t _logo_addr;///<logo��ַ������
	uint16_t _logo_backcolor;///<logo����ɫ������
	uint16_t _logo_w;///<logo��ȣ������������ƣ�����
	uint16_t _logo_h;///<logo�߶ȣ�����
	
	uint16_t _ch1wave;///<ch1������Ĭ��1310
	uint16_t _ch2wave;///<ch2������Ĭ��1490
	uint16_t _ch3wave;///<ch3������Ĭ��1550
	
};
/*struct point
{
	int16_t x;
	int16_t y;
};*/
// typedef struct _RECT
// {
// 	int32_t left;
// 	int32_t top;
// 	int32_t right;
// 	int32_t bottom;
// }RECT;

// typedef struct win_class
// {
// 	uint32_t id;
// 	RECT rc;
// }win_class;

extern struct ctrl_param g_power;//��������
extern struct adj_power_flash g_adj_power;//У׼����
extern volatile u16 powerDownDelayCnt;//�ػ�������ʱ
extern uint32_t hackval ;//�����̨���������
extern uint32_t hackflag ;//�Ƿ����������־
extern volatile u8 g_red_onoff ;//
extern volatile int8_t g_red_mode ;//�����ʾģʽ����������˸���ر�
extern volatile u8 g_red_delay_100ms ;//�����˸0.5s��ʱ
extern volatile u8 g_onoff_en ;//ʹ�ܹػ�����ֹ�������ٹػ�
extern volatile u8 g_autoctrlpower_en;

extern volatile u8 g_key_timer_100ms ;//������ʱ���������жϳ����º͵��������ú�ƺͶ�ʱ�ػ�����
extern volatile uint16_t g_batter_delay ;//���ˢ����ʾ��ʱ
/////////////////////
extern volatile uint16_t g_power_down;
extern volatile uint16_t g_ad_ms ,g_adjust_ms ,g_lcdbug_ms ,g_usart_ms ,g_lcdlisten_ms ,g_debug_ms ,g_redbug_ms;//ad�������
extern volatile uint16_t g_adc[200];//ad������
extern volatile uint16_t ADCConvertedValue[2000];//AD����DMA����

extern int8_t g_recvflag ;//���ڽ��ձ�־
extern volatile uint8_t strout[256];

/*****************************************************************************
ԭ���Ķ���
*****************************************************************************/
extern volatile u16 SysTickCounter;
extern volatile u16 Timer_State ;	    //��ʱ��״ָ̬ʾ,  OFF,  5min  ��10min  ��15min  ��30min  ��60min ״̬
extern volatile u16 Timer_Counter ;
extern volatile u16 Wavelength_Selection_state ;	 //0 = �ر�״̬ 1 = 1310nm 2 = 1495nm 3 = 1550nm 	4 = ���
extern volatile u16 Operating_Mode ;	//0 = CW�� 1 = PW270Hz��2 = 1KHz��3 = 2KHz  ������/�����ѡ��
extern volatile u8  Batter_Lightning;
extern volatile u8  LCD_GetPoint_EN ;   //��ֹ�������ظ��������൱�ڰ���ȥ����,���Խ��д������ɼ���
extern volatile u8  LCD_GetPoint_Counter ; //������ʹ�ܼ���ֵ����
#endif


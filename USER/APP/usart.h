#ifndef _USART_H_
#define _USART_H_

#include "stm32f10x_dma.h"

#pragma diag_suppress 1295//函数参数是空必须声明成void



#define FASTMIN(a, b) ( (a) + ( ((b)-(a)) & -((b)<(a)) ) )
#define FASTMAX(a, b) ( (a) + ( ((b)-(a)) & -((b)>(a)) ) )
#define CHARUCASE(c) ( (c) ^ (0x20 & -((c)>='a' && (c)<='z') ) )
#define CHARLCASE(c) ( (c) ^ (0x20 & -((c)>='A' && (c)<='Z') ) )

#define COM_WIN (256)

#define COM_BUSY 1

#define COM_CRBUF 0x01
#define COM_CTBUF 0x02

struct dma_buf
{
	uint32_t *buf;
	uint16_t len;
	uint16_t num;
};
struct com_dev
{
	uint8_t usart;

#define COM_A2A3 0
#define COM_A9A10 1
#define COM_B6B7 2
#define COM_D5D6 3
#define COM_B10B11 4
#define COM_C10C11 5
#define COM_D8D9 6	
	DMA_Channel_TypeDef* dma_tch;
	DMA_Channel_TypeDef* dma_rch;
	uint32_t dma_f_tch;//1-9	
	uint32_t dma_f_rch;//1-9
};


struct com_buf
{
#define CB_OR 0x01//only read
#define CB_OW 0X02//only write
	uint8_t flag;
	uint8_t buf[COM_WIN];
	uint8_t *str_addr;
	uint16_t len;
};
struct com_multi_buf
{
	uint8_t index;
	//uint8_t num_buf;
	struct com_buf buf1;
	struct com_buf buf2;
};

//void CommInit(uint8_t COM, USART_InitTypeDef* USART_InitStructure);
void CommInit(struct com_dev *v, USART_InitTypeDef* USART_InitStructure);
void CommDMAMode(uint32_t mask);
uint8_t CommSend(uint8_t *buf,uint16_t len);
uint8_t CommDMAStop();
uint8_t CommRecv(uint8_t *buf,uint16_t len);
uint16_t CommCopyToUser(struct com_buf *v,uint8_t *buf,uint16_t len);
uint16_t CommCopyToBuf(struct com_buf *v,uint8_t *buf,uint16_t len);
void CommClear(uint32_t flag);
uint16_t CommRead(uint8_t *buf,uint16_t len);
void CommTurnBuffer(void);
#endif




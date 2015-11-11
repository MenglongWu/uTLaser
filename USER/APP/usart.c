/**
  ******************************************************************************
  * @file	usart.c
  * @author	MenglongWu
  * @version 
  * @date	
  * @brief	作者：吴梦龙\n
  STM32串口配置，具备双缓冲功能，具备DMA和非DMA模式
  ******************************************************************************
  * @attention
  *
  * ATTENTION
 *
 * <h2><center>&copy; COPYRIGHT </center></h2>
 ******************************************************************************
 */

#include "stm32f10x.h"
#include "stm32f10x_dma.h"
#include "usart.h"
#include <stdio.h>

static USART_TypeDef* comSelect = USART1;
struct com_multi_buf com_2buf;
static struct com_dev comdev;
static struct com_buf comtbuf;

//void CommInit(uint8_t COM, USART_InitTypeDef* USART_InitStructure)
void CommInit(struct com_dev *v, USART_InitTypeDef* USART_InitStructure)
{
	uint8_t COM = v->usart;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_TypeDef* com_usart[] = {USART2,USART1,USART1,USART2,USART3,USART3,USART3};
	GPIO_TypeDef* com_tx_port[] = {GPIOA,GPIOA,GPIOB,GPIOD,GPIOB,GPIOC,GPIOD};
	GPIO_TypeDef* com_rx_port[] = {GPIOA,GPIOA,GPIOB,GPIOD,GPIOB,GPIOC,GPIOD};
	uint16_t com_tx_pin[] = {GPIO_Pin_2,GPIO_Pin_9,GPIO_Pin_6,GPIO_Pin_5,GPIO_Pin_10,GPIO_Pin_10,GPIO_Pin_8};
	uint16_t com_rx_pin[] = {GPIO_Pin_3,GPIO_Pin_10,GPIO_Pin_7,GPIO_Pin_6,GPIO_Pin_11,GPIO_Pin_11,GPIO_Pin_9};

	comdev = *v;
	if(COM == 0) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA	| RCC_APB2Periph_AFIO,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
		comSelect = USART2;
	}
	else if(COM == 1) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA	| RCC_APB2Periph_AFIO,ENABLE);
		comSelect = USART1;
	}
	else if(COM == 2) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOB	| RCC_APB2Periph_AFIO,ENABLE);
		GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);
		comSelect = USART1;
	}
	else if(COM == 3) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD	| RCC_APB2Periph_AFIO,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
		GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
		comSelect = USART2;
	}
	else if(COM == 4) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB	| RCC_APB2Periph_AFIO,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
		comSelect = USART3;
	}
	else if(COM == 5) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC	| RCC_APB2Periph_AFIO,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
		GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);
		comSelect = USART3;
	}
	else if(COM == 6) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD	| RCC_APB2Periph_AFIO,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
		GPIO_PinRemapConfig(GPIO_FullRemap_USART3, ENABLE);
		comSelect = USART3;
	}
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = com_tx_pin[COM];
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(com_tx_port[COM], &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = com_rx_pin[COM];
	GPIO_Init(com_rx_port[COM], &GPIO_InitStructure);

	USART_Init(com_usart[COM], USART_InitStructure);
	USART_Cmd(com_usart[COM], ENABLE);
	USART_DMACmd(com_usart[COM],USART_DMAReq_Tx | USART_DMAReq_Rx,ENABLE);
	
	
}
void CommDMAMode(uint32_t mask)
{
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure_t,NVIC_InitStructure_r;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
		
	if(comSelect == USART1) {
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
	}
	else if(comSelect == USART2) {
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
	}
	else {//usart3
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
	}
	DMA_DeInit(comdev.dma_tch);
	DMA_DeInit(comdev.dma_rch);
	
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)0;
	
	DMA_InitStructure.DMA_BufferSize = 0;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

	
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_Init(comdev.dma_tch, &DMA_InitStructure);
	DMA_ITConfig(comdev.dma_tch, DMA_IT_TC | DMA_IT_TE, ENABLE);
	DMA_ClearFlag(comdev.dma_f_tch);        //清除标志位	
	
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_Init(comdev.dma_rch, &DMA_InitStructure);
	DMA_ITConfig(comdev.dma_rch, DMA_IT_TC | DMA_IT_TE, ENABLE);
	DMA_ClearFlag(comdev.dma_f_rch);        //清除标志位	
	
	
	// Set the Vector Table base address at 0x08000000 
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	if(comSelect == USART1) {
		NVIC_InitStructure_t.NVIC_IRQChannel = DMA1_Channel4_IRQn;
		NVIC_InitStructure_r.NVIC_IRQChannel = DMA1_Channel5_IRQn;
	}
	else if(comSelect == USART2) {
		NVIC_InitStructure_t.NVIC_IRQChannel = DMA1_Channel7_IRQn;
		NVIC_InitStructure_r.NVIC_IRQChannel = DMA1_Channel6_IRQn;
	}
	else  {//comSelect == USART3
		NVIC_InitStructure_t.NVIC_IRQChannel = DMA1_Channel2_IRQn;
		NVIC_InitStructure_r.NVIC_IRQChannel = DMA1_Channel3_IRQn;
	}
	
	NVIC_InitStructure_t.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure_t.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure_t.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure_t);
	NVIC_InitStructure_r.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure_r.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure_r.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure_r);
	
	com_2buf.index = 1;
	com_2buf.buf1.len = com_2buf.buf2.len = 0;
	com_2buf.buf1.str_addr = (uint8_t*)com_2buf.buf1.buf;
	com_2buf.buf2.str_addr = (uint8_t*)com_2buf.buf2.buf;
	CommRecv(com_2buf.buf1.buf,COM_WIN);
}
uint8_t CommSend(uint8_t *buf,uint16_t len)
{
	if(DMA_GetFlagStatus(comdev.dma_f_tch) != RESET)
		goto _Next;
	if(  len == 0 || 
		(comdev.dma_tch->CNDTR/*CMAR */ > 0 && DMA_GetFlagStatus(comdev.dma_f_tch) == RESET))
		return 1;
_Next:;
	DMA_ClearFlag(comdev.dma_f_tch);        //清除标志位
	DMA_Cmd(comdev.dma_tch, DISABLE);
	CommCopyToBuf(&comtbuf,buf,len);
	comdev.dma_tch->CMAR = (uint32_t)&comtbuf.buf;
	comdev.dma_tch->CNDTR = len;
	DMA_Cmd(comdev.dma_tch, ENABLE);
	return 0;
}
uint8_t CommRecv(uint8_t *buf,uint16_t len)
{
	if(  len == 0 || 
		(comdev.dma_rch->CNDTR  > 0 && DMA_GetFlagStatus(comdev.dma_f_rch) == RESET))
		return 1;
	DMA_ClearFlag(comdev.dma_f_rch);        //清除标志位
	DMA_Cmd(comdev.dma_rch, DISABLE);
	comdev.dma_rch->CMAR = (uint32_t)buf;
	comdev.dma_rch->CNDTR = len;
	DMA_Cmd(comdev.dma_rch, ENABLE);
	return 0;	
}
uint8_t CommDMAStop()
{
	DMA_Cmd(comdev.dma_tch, DISABLE);
	comdev.dma_tch->CMAR  = 0;
	DMA_ClearFlag(comdev.dma_f_tch);
	DMA_Cmd(comdev.dma_rch, DISABLE);
	comdev.dma_rch->CMAR  = 0;
	DMA_ClearFlag(comdev.dma_f_rch);
}
//uint8_t CommRecvCopy(uint8_t *buf,uint16_t len)
uint16_t CommCopyToUser(struct com_buf *v,uint8_t *buf,uint16_t len)
{
	/*uint16_t i,vlen;
	uint8_t *dst,*str;
	//uint32_t *dst;uint32_t *str;
	
	vlen = len < v->len? len: v->len;

	dst = (uint8_t*)buf;
	str = (uint8_t*)v->str_addr;
	for(i = 0;i < vlen;i++) {
		*dst = *str;
		dst++;
		str++;
	}
	
	v->len = v->len - (vlen);
	v->str_addr += (vlen);
	return (vlen);*/
	
	uint16_t i,count,tcount;
	uint32_t *dst32,*src32;
	uint8_t *dst8,*src8;
	
	count = FASTMIN((len),(v->len));
	
	dst32 = (uint32_t*)buf;src32 = (uint32_t*)v->str_addr;
	tcount = count >> 2;
	for(i = 0;i < tcount;i++) {
		*(dst32 +i)= *(src32+i);
	}
	if(tcount = count & 3) {
		//dst8 = (uint8_t*)buf;src8 = (uint8_t*)v->str_addr;	
		dst8 = (uint8_t*)(dst32 +i);src8 = (uint8_t*)(src32+i);	
		for(i = 0;i < tcount;i++) {
			*(dst8 +i)= *(src8+i);
		}
	}
	v->len = v->len - count;
	v->str_addr += count;
	return (count);
}
uint16_t CommCopyToBuf(struct com_buf *v,uint8_t *buf,uint16_t len)
{
	uint16_t i,count,tcount;
	uint32_t *dst32,*src32;
	uint8_t *dst8,*src8;
	
	count = FASTMIN((len),(COM_WIN));
	
	dst32 = (uint32_t*)&v->buf;src32 = (uint32_t*)buf;
	tcount = count >> 2;
	for(i = 0;i < tcount;i++) {
		*(dst32 +i)= *(src32+i);
	}
	if(tcount = count & 3) {
		//dst8 = (uint8_t*)buf;src8 = (uint8_t*)v->str_addr;	
		dst8 = (uint8_t*)(dst32 +i);src8 = (uint8_t*)(src32+i);	
		for(i = 0;i < tcount;i++) {
			*(dst8 +i)= *(src8+i);
		}
	}
	//v->len = v->len - count;
	//v->str_addr += count;
	return (count);	
}
void CommClear(uint32_t flag)
{
// 	if(flag & COM_CTBUF) {
// 		DMA_Cmd(comdev.dma_tch, DISABLE);
// 		comdev.dma_tch->CNDTR  = 0;
// 		DMA_ClearFlag(comdev.dma_f_tch);
// 		DMA_Cmd(comdev.dma_tch, ENABLE);
// 	}
// 	if(flag & COM_CRBUF) {
// 		DMA_Cmd(comdev.dma_rch, DISABLE);
// 		comdev.dma_tch->CNDTR  = 0;
// 		DMA_ClearFlag(comdev.dma_f_rch);
// 		DMA_Cmd(comdev.dma_rch, ENABLE);	
// 		com_2buf.buf1.len = 0;com_2buf.buf2.len = 0;
// 		com_2buf.buf1.str_addr = (uint8_t*)com_2buf.buf1.buf;
// 		com_2buf.buf2.str_addr = (uint8_t*)com_2buf.buf2.buf;
// 	}
}

void CommTurnBuffer()
{
	if(com_2buf.index == 1) {
		com_2buf.buf1.str_addr = (uint8_t*)com_2buf.buf1.buf;
		com_2buf.buf1.len = COM_WIN;
		CommRecv(com_2buf.buf2.buf,COM_WIN);
		com_2buf.index  = 2;
	}
	else if(com_2buf.index  == 2) {
		com_2buf.buf2.str_addr = (uint8_t*)com_2buf.buf2.buf;
		com_2buf.buf2.len = COM_WIN;
		CommRecv(com_2buf.buf1.buf,COM_WIN);
		com_2buf.index  = 1;
	}
}
uint16_t CommRead(uint8_t *buf,uint16_t len)
{
	uint16_t ret;
	int i;
	uint8_t *dst = buf;
	uint8_t *str;
	
	if(com_2buf.buf1.len) {
		ret = CommCopyToUser(&com_2buf.buf1,buf,len);
		//printf("from last buf1\n");
		return ret;
	}
	else if(com_2buf.buf2.len) {
		ret = CommCopyToUser(&com_2buf.buf2,buf,len);
		//printf("from last buf2\n");
		return ret;
	}
	
	ret = ((uint16_t)COM_WIN-(comdev.dma_rch->CNDTR));
	if(ret) {
		if(com_2buf.index == 1) {
			com_2buf.buf1.str_addr = (uint8_t*)com_2buf.buf1.buf;
			com_2buf.buf1.len = ret;
			comdev.dma_rch->CMAR = (uint32_t)com_2buf.buf2.buf;
			com_2buf.index = 2;
		}
		else  {
			com_2buf.buf2.str_addr = (uint8_t*)com_2buf.buf2.buf;
			com_2buf.buf2.len = ret;
			comdev.dma_rch->CMAR = (uint32_t)com_2buf.buf1.buf;
			com_2buf.index = 1;
		}
		comdev.dma_rch->CCR &= ((uint32_t)0xFFFFFFFE);//DMA_Cmd(DMA1_Channel5, DISABLE);
		DMA_ClearFlag(comdev.dma_f_rch);        //清除标志位
		comdev.dma_rch->CNDTR = COM_WIN;
		comdev.dma_rch->CCR |= ((uint32_t)0x00000001);//DMA_Cmd(DMA1_Channel5, ENABLE);
		
		if(com_2buf.buf1.len) {
			ret = CommCopyToUser(&com_2buf.buf1,buf,len);
			//printf("new from buf1\n");
			return ret;
		}
		else if(com_2buf.buf2.len) {
			ret = CommCopyToUser(&com_2buf.buf2,buf,len);
			//printf("new from buf2\n");
			return ret;
		}
	}
	return ret;
}

/* Private variables ---------------------------------------------------------*/
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* Private function prototypes -----------------------------------------------*/
PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART */
	if (ch == '\n') {
		USART_SendData(comSelect, (uint8_t) '\r');	
		while (USART_GetFlagStatus(comSelect, USART_FLAG_TC) == RESET);
	}
	USART_SendData(comSelect, (uint8_t) ch);

	/* Loop until the end of transmission */
	while (USART_GetFlagStatus(comSelect, USART_FLAG_TC) == RESET);

	return ch;
}

#ifdef  USE_FULL_ASSERT

/**
* @brief  Reports the name of the source file and the source line number
*   where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{ 
	/* User can add his own implementation to report the file name and line number,
	ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif
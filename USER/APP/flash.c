#include "stm32f10x_flash.h"
#include "flash.h"
/*
#if defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || (STM32F10X_CL) || defined (STM32F10X_XL)
	#define FLASH_PAGE_SIZE    ((uint16_t)0x800)
	#define FLASH_PAGE_START ((u32)0x08002000)
#else
	#define FLASH_PAGE_SIZE    ((uint16_t)0x400)
	#define FLASH_PAGE_START ((u32)0x08002000)
#endif
*/
uint32_t WriteFlash(uint32_t addr,uint32_t *data,uint32_t len)
{
	uint32_t numPage,EraseCounter;
	volatile FLASH_Status FLASHStatus;
	uint32_t startAddr,endAddr;
	
	FLASH_Unlock();								//Unlock the Flash Program Erase controller
	FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);  // Clear All pending flags	
	
	FLASHStatus = FLASH_COMPLETE;
	numPage = (len / FLASH_PAGE_SIZE) + (len % FLASH_PAGE_SIZE) + 1;
	numPage = 1;

	for(EraseCounter = 0; (EraseCounter < numPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
	{
		FLASHStatus = FLASH_ErasePage(addr + (FLASH_PAGE_SIZE * EraseCounter));
	}
	
	startAddr = addr;
	endAddr = startAddr + len;
	FLASHStatus = FLASH_COMPLETE;
	while((startAddr < endAddr) )//&& (FLASHStatus == FLASH_COMPLETE))
	{
		FLASHStatus = FLASH_ProgramWord(startAddr, *data++);
		startAddr = startAddr + 4;
	}
	FLASH_Lock();
}

uint32_t ReadFlash(uint32_t addr,uint32_t *data,uint32_t len)
{
	uint32_t numPage,EraseCounter;
	volatile FLASH_Status FLASHStatus;
	uint32_t startAddr,endAddr;
	uint32_t *p;
	
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);  // Clear All pending flags	
	FLASHStatus = FLASH_COMPLETE;
	startAddr = addr;
	endAddr = startAddr + len;
	
	while(startAddr < endAddr) {
		p = (uint32_t*)startAddr;
		*data++ = *p;
		startAddr += 4;
	}
	FLASH_Lock();
}

uint8_t pagetmp[FLASH_PAGE_SIZE];
uint32_t CoverFlash(uint32_t addr,uint32_t addr2,uint32_t *data,uint32_t len)
{
	uint32_t start,i;
	uint32_t *pstart32;
	uint8_t *pstart8;
	
	start = addr2 - addr;
	pstart32 = (uint32_t*)start;
	pstart8 = (uint8_t*)data;
	ReadFlash(addr,(uint32_t*)pagetmp,FLASH_PAGE_SIZE);
	for(i = 0;i < len;++i) {
		pagetmp[start+i] = *(pstart8+i);
	}
	WriteFlash(addr,(uint32_t*)pagetmp,FLASH_PAGE_SIZE);
	
}
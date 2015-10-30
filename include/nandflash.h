/**
 ******************************************************************************
 * @file	nandflash.h
 * @brief	
 *			nand flash通用接口，屏蔽底层实现__low_nand_xxx()
 *
 * @section NAND_COM NAND API 公共接口
 	- nand_init\n
	- nand_readid\n
 * @section NAND_HP NAND API 物理地址接口
	- nand_erase_block\n
	- nand_write_page\n
	- nand_read_page\n
 * @section NAND_LOG NAND API 逻辑地址接口
  	- nand_erase\n
  	- nand_write\n
  	- nand_read\n
*/
/*
 *--------------------------------------------------
 * version    |    author    |    date    |    content
 * V1.0			Menglong Wu		2014-10-16		1.xxxxx
 ******************************************************************************
*/
#ifndef _NAND_FLASH_H_
#define _NAND_FLASH_H_
#include "board_config.h"
#include "board.h"
//#define RAMFS
#ifdef RAMFS
	#define RAM_BYTE_PER_SPACE 16
	#define RAM_BYTE_PER_CHUNK 512
	#define RAM_CHUNK_PER_PAGE 4
	#define RAM_PAGE_PER_BLOCK 8
	#define RAM_BLOCK_COUNT    256
#else
	#define RAM_BYTE_PER_SPACE 16
	#define RAM_BYTE_PER_CHUNK 512
	#define RAM_CHUNK_PER_PAGE 4
	#define RAM_PAGE_PER_BLOCK 64
	#define RAM_BLOCK_COUNT    256
#endif

#define RAM_SIZE ((RAM_BYTE_PER_SPACE + RAM_BYTE_PER_CHUNK) * \
					RAM_CHUNK_PER_PAGE* \
					RAM_PAGE_PER_BLOCK*\
					RAM_BLOCK_COUNT)


#define RAM_SECTOR_ADDR (RAM_BYTE_PER_CHUNK)
#define RAM_PAGE_ADDR	(RAM_CHUNK_PER_PAGE * \
	(RAM_SECTOR_ADDR + RAM_BYTE_PER_SPACE))

#define RAM_BLOCK_ADDR (RAM_PAGE_PER_BLOCK*RAM_PAGE_ADDR)



//*****************************************************************************
//硬件驱动层

//根据具体MCU、NandFlash而实现具体接口内容
extern char __low_nand_write_page(unsigned int page_addr,unsigned char *from,unsigned int size);
/**
 * @brief	向Nand写入1页数据
 * @param	page_addr 写入页的起始地址
 * @param	form 待写入页的内容
 * @param	size 写入的大小
 * @retval	待完善
 * @remarks	调用底层驱动nand_read_page
 */
extern void __low_nand_read_page(unsigned int page_addr,unsigned char *to,unsigned int size);
/**
 * @brief	擦除Nand 1块
 * @param	block_addr 块地址
 * @retval	待完善
 * @remarks\n NULL
 * @see nand_erase_block
 */
extern char __low_nand_erase_block(unsigned int block_addr);
/**
 * @brief	初始化Nand硬件接口
 * @retval\n 待完善
 * @remarks\n NULL
 * @see low_nand_init
 */
extern void __low_nand_init();
/**
 * @brief	读取Nand芯片设备号
 * @retval\n	NULL
 * @see nand_readid
 */
extern char __low_nand_readid(unsigned int *part1,unsigned *part2);




//*****************************************************************************
//Bootloader拷贝，拷贝范围从0x30000000到bss_start段地址，具体段划分参看boot.lds
/**
 * @brief	拷贝SDRAM数据到nand
 * @param\n NULL
 * @retval\n	NULL
 * @remarks	拷贝范围从0x30000000到bss_start段地址，具体段划分参看boot.lds
 * @see nand_write_page nand_erase_block
 */
extern void copy_sdram2nand();
/**
 * @brief	拷贝SDRAM数据到nand
 * @param\n NULL
 * @retval\n	NULL
 * @remarks	拷贝范围从0x30000000到bss_start段地址，具体段划分参看boot.lds
 * @see nand_write_page nand_erase_block
 */
extern void copy_nand2sdram();
/**
 * @brief	功能同copy_nand2sdram，添加LED显示功能，非标准接口
 * @see copy_nand2sdram
 */
extern void copy_nand2sdram_ex();



//*****************************************************************************
//Nand坏块管理，采用内联机制

static inline void nand_init();
static inline char nand_readid(unsigned int *part1,unsigned *part2);

static inline char nand_write_page(unsigned int page_addr,unsigned char *from,unsigned int size);
static inline void nand_read_page(unsigned int page_addr,unsigned char *to,unsigned int size);
static inline char nand_erase_block(unsigned int block_addr);



static inline int nand_erase(unsigned int block);
static inline int nand_write(
		unsigned int   block,unsigned int page,unsigned int off,
		unsigned char *data,
		unsigned int   nlen);
static inline void nand_read(
		unsigned int   block,unsigned int page,unsigned int off,
		unsigned char *data,
		unsigned int   nlen);



//硬件无关层

/**
 * @brief	擦除Nand 1块，输入物理地址
 * @param	block_addr 块地址
 * @retval	待完善
 * @remarks	调用底层驱动__low_nand_erase_block
 * @see __low_nand_erase_block
 */
static inline char nand_erase_block(unsigned int block_addr)
{
	return __low_nand_erase_block(block_addr);
}


/**
 * @brief	向Nand写入1页数据，输入物理地址
 * @param	page_addr 写入页的起始地址
 * @param	form 待写入页的内容
 * @param	size 写入的大小
 * @retval	待完善
 * @remarks	调用底层驱动__low_nand_write_page
 * @see __low_nand_write_page
 */
static inline char nand_write_page(unsigned int page_addr,unsigned char *from,unsigned int size)
{
	return __low_nand_write_page(page_addr,from,size);
}
 

/**
 * @brief	向Nand读出1页数据，输入物理地址
 * @param	page_addr 写入页的起始地址
 * @param	to 待读出页的内容
 * @param	size 读出的大小
 * @retval	待完善
 * @see	__low_nand_read_page
 */
static inline  void nand_read_page(unsigned int page_addr,unsigned char *to,unsigned int size)
{
	__low_nand_read_page(page_addr,to,size);
}





/**
 * @brief	初始化Nand硬件接口
 * @retval\n	NULL
 * @remarks	调用底层驱动__low_nand_init
 * @see __low_nand_init
 */
static inline void nand_init()
{
	__low_nand_init();
}

/**
 * @brief	读取Nand芯片设备号
 * @retval\n	NULL
 * @remarks	调用底层驱动__low_nand_readid，建议调试Nand驱动时首先测试该函数，
 该函数接口目的在于测试驱动时序是否正确
 * @see __low_nand_readid
 */
static inline char nand_readid(unsigned int *part1,unsigned *part2)
{
	return __low_nand_readid(part1,part2);
}






/**
 * @brief	向NandFlash写入数据，输入逻辑号
 * @param	block block号
 * @param	page 相对所在block偏移的页号
 * @param	off 相对所在page偏移的字节
 * @param	data 待写入数据内容
 * @param	nlen 待写入数据长度
 * @retval	0:表示成功；-1表示失败
 * @remarks	该函数调用底层驱动__low_nand_erase_block
 * @see __low_nand_erase_block
 */

static inline int nand_erase(unsigned int block)
{
	return __low_nand_erase_block(block * NAND_BLOCK_ADDR);
}

/**
 * @brief	向NandFlash写入数据，输入逻辑号
 * @param	block block号
 * @param	page 相对所在block偏移的页号
 * @param	off 相对所在page偏移的字节
 * @param	data 待写入数据内容
 * @param	nlen 待写入数据长度
 * @retval	0:表示成功；-1表示失败
 * @remarks	该函数调用底层驱动__low_nand_write_page
 * @see __low_nand_write_page
 */
void PrintByte(unsigned char *data,unsigned int len);
static inline int nand_write(
		unsigned int   block,unsigned int page,unsigned int off,
		unsigned char *data,
		unsigned int   nlen)
{
	// printk("nand_write()\n");
	// printk("\twrite Addr %8.8x\n",(block * NAND_BLOCK_ADDR) + (page * NAND_PAGE_ADDR) + off);
	// PrintByte(data,nlen);
	return __low_nand_write_page(
			(block * NAND_BLOCK_ADDR) + (page * NAND_PAGE_ADDR) + off,
			data,
			nlen);
}

/**
 * @brief	向NandFlash写入数据，输入逻辑号
 * @param	block block号
 * @param	page 相对所在block偏移的页号
 * @param	off 相对所在page偏移的字节
 * @param	data 待写入数据内容
 * @param	nlen 待写入数据长度
 * @retval	0:表示成功；-1表示失败
 * @remarks	该函数调用底层驱动__low_nand_read_page
 * @see __low_nand_read_page
 */
static inline void nand_read(
		unsigned int   block,unsigned int page,unsigned int off,
		unsigned char *data,
		unsigned int   nlen)
{
	// printk("nand_read()\n");
	// printk("\tread Addr %8.8x\n",(block * NAND_BLOCK_ADDR) + (page * NAND_PAGE_ADDR) + off);
	__low_nand_read_page(
			(block * NAND_BLOCK_ADDR) + (page * NAND_PAGE_ADDR) + off,
			data,
			nlen);
}
#endif


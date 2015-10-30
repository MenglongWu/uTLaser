/**
 ******************************************************************************
 * @file	board_config.h
 * @brief	
 *			电路板配置，用于TQ2440
 *
 *--------------------------------------------------
 * version    |    author    |    date    |    content
 * V1.0			Menglong Wu		2014-10-17	
 * V1.0			Menglong Wu		2014-11-15	1.V1.0里对S34ML02G1中Plane的认识有歧义，
 *		现将Plane选择码A18看成Block最低位
 ******************************************************************************
*/

#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_ 



// Target information
#define TARGET_BOARD 	"TQ2440"
#define TARGET_VERSION 	"V1.0.20141114"

// Nand Flash

// 芯片名：S34ML02G1
// 1. 1 Page = (2k + 64) bytes
// 2. 1 Block = (2k + 64) bytesx 64 pages = (128k + 4k) bytes
// 3. 1 Plane = (128k + 4k) bytes x 1024 Blocks
// 4. For 1 Gb and 2 Gbdevicesthere are 1024 Blocksper Plane
//    For 4 Gbdevice there are 2048Blocksper Plane

// Note:
//    2 Gb and 4 Gbdeviceshave two Planes

// 地址格式
// A0 - A11: column address in the page
// A12 - A17: page address in the block
// A18: plane address (for multiplane operations) / block address (for normal operations)
// A19 - A28: block address

// 如下定义属于应用层使用的，关于底层的定义查看Nand_S34ML02G1.H
#define NAND_PAGE_ADDR_SHIFT (12)					//A12
#define NAND_PAGE_ADDR (1 << NAND_PAGE_ADDR_SHIFT)	//0x1000为页起始地址
#define NAND_PAGE_SIZE 0x800						//2K
#define NAND_PAGE_MASK (NAND_PAGE_SIZE-1)			//页掩码

#define NAND_SPACE_SIZE 0x40						//额外空间

#define NAND_BLOCK_ADDR_SHIFT	(18)						//块起始地址 A18
#define NAND_BLOCK_ADDR 	(1 << NAND_BLOCK_ADDR_SHIFT)//0x40000	
#define NAND_BLOCK_SIZE 	0x20000						//1 Block = 2 Plane x 64 Page x 2048 Byte(与上文的3不冲突)
#define NAND_BLOCK_MASK 	(NAND_BLOCK_SIZE-1)			//块掩码
 
#define NAND_BLOCK_COUNT 	2048						//Block总块数，Device拥有2个Plaen，每个Plane有1024个Block

#define NAND_DIVECE_SIZE 	(NAND_BLOCK_SIZE*NAND_BLOCK_COUNT) 		//设备总大小

#define NAND_SECTOR_SIZE 	0x200						// 扇区大小512Byte
#define NAND_SECTOR_COUNT 	(NAND_DIVECE_SIZE / NAND_SECTOR_SIZE)		// 坏块描述表长度

#define SECTOR_PER_PAGE 4									//定义每个Page有多少个Sector

#define PAGE_PER_BLOCK 			(NAND_BLOCK_SIZE / NAND_PAGE_SIZE)	//每Block有多少Page
#define PAGE_PER_BLOCK_MASK 	(~(PAGE_PER_BLOCK - 1))

#define SECT_PER_BLOCK  		(NAND_BLOCK_SIZE / NAND_SECTOR_SIZE)	//每Block有多少Sector
#define SECT_PER_BLOCK_MASK 	(~(SECT_PER_BLOCK - 1))

#define BLOCK_FIRST_SECT(sect) 		((sect & SECT_PER_BLOCK_MASK))			//输入任意Sector，输出该Sector所述Block第一个Sector号
#define BLOCK_LAST_SECT(sect)			(BLOCK_FIRST_SECT(sect) + SECT_PER_BLOCK)
// 扇区号和物理地址的转化
#if (SECTOR_PER_PAGE == 4)
	#define SECT_STROE_ADDR(sect) \
 		(\
 			(NAND_PAGE_ADDR   * (sect >> 2)) + \
 			(NAND_SECTOR_SIZE * (sect & 3)) \
 		)
 	#define SECT_SPACE_ADDR(sect) \
 		(\
 			(NAND_PAGE_ADDR   * (sect >> 2)) + \
 			(NAND_PAGE_SIZE + (sect & 3)) \
 		)
#elif (SECTOR_PER_PAGE == 2)
	#define SECT_STROE_ADDR(sect) \
 		( \
 			(NAND_PAGE_ADDR   * (sect >> 1)) + \
 			(NAND_SECTOR_SIZE * (sect & 1)) \
 		)
#elif (SECTOR_PER_PAGE == 1)
 		#define SECT_STROE_ADDR(sect) \
 		( \
 			(NAND_PAGE_ADDR *    sect)\
 		)
#else
 		#error "SECTOR_PER_PAGE unuse check in board_config.h"
#endif


#endif



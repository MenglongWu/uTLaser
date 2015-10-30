/****************************************Copyright (c)**************************************************                         
**
**                                 http://www.powermcu.com
**
**--------------File Info-------------------------------------------------------------------------------
** File name:			GLCD.c
** Descriptions:		SSD1963 操作函数库
**						
**------------------------------------------------------------------------------------------------------
** Created by:			AVRman
** Created date:		2011-2-23
** Version:				1.0
** Descriptions:		The original version
**
**------------------------------------------------------------------------------------------------------
** Modified by:			Yu Jingxiong   2011.11.10
** Modified date:		重新阅读并标注加深理解。。。。keep moving！！	
** Version:
** Descriptions:		
********************************************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "GLCD.h" 
#include "HzLib.h"
#include "AsciiLib.h"
//#include "stm32f10x_conf.h"
// #include "stm32f10x_spi.h"
// #include "stm32f10x_fsmc.h"
//#include <PictureData.h>
#include "PictureData.h"
void LCD_FLSAH_DrawPicture(uint16_t StartX,uint16_t StartY,uint16_t EndX,uint16_t EndY, uint8_t * pic);
/* Private define ------------------------------------------------------------*/
/* 使用总线方式时定义地址 */
/* 挂在不同的BANK,使用不同地址线时请自行换算地址 */

//#define LCD_REG              (*((volatile unsigned short *) 0x6C000000)) // RS = 0 
//#define LCD_RAM              (*((volatile unsigned short *) 0x6C000002)) // RS = 1





static void delay_ms(uint16_t ms);

/*******************************************************************************
* Function Name  : LCD_CtrlLinesConfig
* Description    : Configures LCD Control lines (FSMC Pins) in alternate function
Push-Pull mode.
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
#define RCC_TS		RCC_APB2Periph_GPIOB
#define PORT_TS		GPIOB
#define PIN_TS		GPIO_Pin_12

#define RCC_LCD_REST 	RCC_APB2Periph_GPIOC
#define PORT_LCD_RESET  GPIOC
#define PIN_LCD_RESET	GPIO_Pin_6

#define RCC_LCD_CS	RCC_APB2Periph_GPIOD
#define PORT_LCD_CS GPIOD
#define PIN_LCD_CS	GPIO_Pin_7


void LCD_CtrlLinesConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* Enable FSMC, GPIOD, GPIOE, GPIOF, GPIOG and AFIO clocks */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOF | 
	                       RCC_APB2Periph_GPIOG | RCC_APB2Periph_AFIO, ENABLE);
	
	/* Set PD.00(D2), PD.01(D3), PD.04(NOE), PD.05(NWE), PD.08(D13), PD.09(D14),
	 PD.10(D15), PD.14(D0), PD.15(D1) as alternate function push pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 |
	                            GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_14 | 
	                            GPIO_Pin_15/*YU*/| GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	/* Set PE.07(D4), PE.08(D5), PE.09(D6), PE.10(D7), PE.11(D8), PE.12(D9), PE.13(D10),
	 PE.14(D11), PE.15(D12) as alternate function push pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | 
	                            GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | 
	                            GPIO_Pin_15;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	
	//1963:TS
	RCC_APB2PeriphClockCmd(RCC_TS, ENABLE);
	GPIO_InitStructure.GPIO_Pin = PIN_TS;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(PORT_TS, &GPIO_InitStructure);
	
	
	RCC_APB2PeriphClockCmd(RCC_LCD_REST, ENABLE);
	GPIO_InitStructure.GPIO_Pin = PIN_LCD_RESET;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(PORT_LCD_RESET, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_LCD_CS, ENABLE);
	GPIO_InitStructure.GPIO_Pin = PIN_LCD_CS;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(PORT_LCD_CS, &GPIO_InitStructure);
	GPIO_SetBits(PORT_LCD_CS,PIN_LCD_CS);
}

/*******************************************************************************
* Function Name  : LCD_FSMCConfig
* Description    : Configures the Parallel interface (FSMC) for LCD(Parallel mode)
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
static void LCD_FSMCConfig(void)
{
	FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
	FSMC_NORSRAMTimingInitTypeDef FSMC_NORSRAMTimingInitStructure;
	/* FSMC读速度设置 */
	FSMC_NORSRAMTimingInitStructure.FSMC_AddressSetupTime = 5;  /* 地址建立时间  */
	FSMC_NORSRAMTimingInitStructure.FSMC_AddressHoldTime = 2;	   
	FSMC_NORSRAMTimingInitStructure.FSMC_DataSetupTime = 5;	   /* 数据建立时间  */
	FSMC_NORSRAMTimingInitStructure.FSMC_BusTurnAroundDuration = 0x00;
	FSMC_NORSRAMTimingInitStructure.FSMC_CLKDivision = 0x02;
	FSMC_NORSRAMTimingInitStructure.FSMC_DataLatency = 0x00;
	FSMC_NORSRAMTimingInitStructure.FSMC_AccessMode = FSMC_AccessMode_A;	/* FSMC 访问模式 */
	
	FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM4;
	FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
	FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM;
	FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
	FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
	FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
	FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
	FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;
	FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
	FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &FSMC_NORSRAMTimingInitStructure;
	FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure); 
	/* FSMC写速度设置 */
	FSMC_NORSRAMTimingInitStructure.FSMC_AddressSetupTime = 5;   /* 地址建立时间  */
	FSMC_NORSRAMTimingInitStructure.FSMC_AddressHoldTime = 2;	   
	FSMC_NORSRAMTimingInitStructure.FSMC_DataSetupTime = 5;	   /* 数据建立时间  */
	FSMC_NORSRAMTimingInitStructure.FSMC_BusTurnAroundDuration = 0x00;
	FSMC_NORSRAMTimingInitStructure.FSMC_CLKDivision = 0x02;
	FSMC_NORSRAMTimingInitStructure.FSMC_DataLatency = 0x00;
	FSMC_NORSRAMTimingInitStructure.FSMC_AccessMode = FSMC_AccessMode_A;	/* FSMC 访问模式 */
	FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &FSMC_NORSRAMTimingInitStructure;	  
	
	FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure); 
	
	/* Enable FSMC Bank4_SRAM Bank */
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM4, ENABLE);  
}
uint32_t LCD_ReadReg(uint16_t cmd,uint16_t *data)
{
	LCD_REG = cmd;
	*data = LCD_RAM;
	return 1;
}
void LCD_WriteReg(uint16_t cmd)
{
	LCD_REG = cmd;
}
void LCD_WriteRegData(uint16_t cmd,uint16_t data)
{
	LCD_REG = cmd;
	LCD_RAM = data;
}
/*******************************************************************************
* Function Name  : LCD_Configuration
* Description    : Configure the LCD Control pins and FSMC Parallel interface
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void LCD_Configuration(void)
{
	/* Configure the LCD Control pins --------------------------------------------*/
	LCD_CtrlLinesConfig();

	/* Configure the FSMC Parallel interface -------------------------------------*/
	LCD_FSMCConfig();
}

/*******************************************************************************
* Function Name  : LCD_WriteReg
* Description    : controller command
* Input          : - cmd: writes command.
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/

//static __inline void LCD_WriteCommand(uint16_t cmd)
//  void LCD_WriteCommand(uint16_t cmd)
// {
// 	/* Write cmd */
// 	LCD_REG = cmd;
// }

/*******************************************************************************
* Function Name  : LCD_WriteRAM
* Description    : Writes to the LCD RAM.
* Input          : - data: the pixel color in RGB mode (5-6-5).
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
// static __inline void LCD_WriteData(uint16_t data)					 
// {
// 	/* Write 16-bit data */
// 	LCD_RAM = data;
// }

/*******************************************************************************
* Function Name  : LCD_ReadRAM
* Description    : Reads the LCD RAM.
* Input          : None
* Output         : None
* Return         : LCD RAM Value.
* Attention		 : None
*******************************************************************************/
// static __inline uint16_t LCD_ReadData(void)
// {  
// 	/* Read 16-bit data */
// 	return LCD_RAM;
// }

//SSD1963上电过程，
void SSD1963_PowerUp()
{
	//delay_ms(100)大约10ms
	GPIO_SetBits(PORT_LCD_CS,PIN_LCD_CS);
	delay_ms(2000);
	GPIO_ResetBits(PORT_LCD_CS,PIN_LCD_CS);
	delay_ms(2000);
	
	GPIO_SetBits(PORT_LCD_RESET,PIN_LCD_RESET);
	delay_ms(1000);
	GPIO_ResetBits(PORT_LCD_RESET,PIN_LCD_RESET);
	delay_ms(1000);
	GPIO_SetBits(PORT_LCD_RESET,PIN_LCD_RESET);
	delay_ms(1000);
}

/*******************************************************************************
* Function Name  : SSD1963_SSD1963_GPIO_WR
* Description    : Set a GPIO pin to state high(1) or low(0)
* Input          : - pin: LCD_RESET or LCD_SPENA or LCD_SPCLK  or LCD_SPDAT
*                  - state: 0 for low and 1 for high
* Output         : None
* Return         : None
* Attention		 : Set the GPIO pin an output prior using this function
*******************************************************************************/
static void SSD1963_GPIO_WR(uint8_t pin, uint8_t state)
{
	static uint8_t _gpioStatus = 0;  /* ssd1963 specific 静态变量 */

	if( state )
	{
		_gpioStatus = _gpioStatus | pin;
	}
	else
	{
		_gpioStatus = _gpioStatus & ( ~pin );
	}

	LCD_WriteCommand(0xBA);		     /* Set GPIO value */
	LCD_WriteData(_gpioStatus);
}

/*******************************************************************************
* Function Name  : SSD1963_SPI_Write
* Description    : SPI Write
* Input          : - byte: data
* Output         : None
* Return         : None
* Attention		 : GPIO pins for the SPI port set all output prior to
*                  using this function
*******************************************************************************/
static void SSD1963_SPI_Write(uint8_t byte)
{
	uint8_t bit_ctr;

	for( bit_ctr = 0; bit_ctr < 8; bit_ctr++ )
	{
		if( byte & 0x80 )
		{
			SSD1963_GPIO_WR(LCD_SPDAT,1);
		}
		else
		{
			SSD1963_GPIO_WR(LCD_SPDAT,0);
		}
		SSD1963_GPIO_WR(LCD_SPCLK,0);
		SSD1963_GPIO_WR(LCD_SPCLK,1);
		byte = ( byte << 1 );
	}
}

/*******************************************************************************
* Function Name  : SSD1963_SPI_WriteReg
* Description    : write reg
* Input          : - reg: 
*                  - cmd: 
* Output         : None
* Return         : None
* Attention		 : GPIO pins for the SPI port set all output prior to
*                  using this function
*******************************************************************************/
static void SSD1963_SPI_WriteReg(uint8_t reg, uint16_t cmd)
{
	SSD1963_GPIO_WR(LCD_SPENA,0);
	cmd = ( (reg << 10) | (1 << 9) | cmd );
	SSD1963_SPI_Write( ( (uint8_t) (cmd)>>8) );
	SSD1963_SPI_Write( ( (uint8_t) cmd) );
	SSD1963_GPIO_WR(LCD_SPENA,1);
}

//利用LCD的gpio寄存器配置信息，反正这个GPIO也没用，干脆用来检测复位
void SSD1963_SetCheckFlag()
{
	LCD_WriteCommand(0xB8);
	LCD_WriteData(0x0f);
	LCD_WriteData(0x01);
}

uint8_t SSD1963_IsRestart()
{
	uint8_t data1,data2;
	LCD_WriteCommand(0xb9);
	data1 = LCD_ReadData();
	data2 = LCD_ReadData();
	if(data1 != 0x0f)
		return 1;
	return 0;
}
/*******************************************************************************
* Function Name  : delay_ms
* Description    : Delay Time
* Input          : - cnt: Delay Time
* Output         : None
* Return         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
static void delay_ms(uint16_t ms)    
{ 
	uint16_t i,j; 
	for( i = 0; i < ms; i++ )
	{ 
		for( j = 0; j < 1141; j++ );
	}
}

/*******************************************************************************
* Function Name  : LCD_Initializtion
* Description    : SSD1963 Resets
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void LCD_Initializtion(void)
{
	//LCD_Configuration();
	SSD1963_PowerUp();
	LCD_SetBacklight(0xff);//优先进行亮度调节。。最暗状态

	
	/* Set MN(multipliers) of PLL, VCO = crystal freq * (N+1) */
	/* PLL freq = VCO/M with 250MHz < VCO < 800MHz */
	/* The max PLL freq is around 120MHz. To obtain 120MHz as the PLL freq */
	LCD_WriteCommand(0xE2);			    /* Set PLL with OSC = 10MHz (hardware) */
	/* Multiplier N = 35, VCO (>250MHz)= OSC*(N+1), VCO = 360MHz */
	LCD_WriteData(0x23);
	LCD_WriteData(0x02);				/* Divider M = 2, PLL = 360/(M+1) = 120MHz */
	LCD_WriteData(0x54);				/* Validate M and N values */	
	
	LCD_WriteCommand(0xE0);			    /* Start PLL command */
	LCD_WriteData(0x01);				/* enable PLL */	
	delay_ms(10);						/* wait stablize */
	delay_ms(50);
	
	LCD_WriteCommand(0xE0);			    /* Start PLL command again */
	LCD_WriteData(0x03);				/* now, use PLL output as system clock */		
	delay_ms(10);
	
	/* once PLL locked (at 120MHz), the data hold time set shortest */
 	LCD_WriteCommand(0x01);			    /* Soft reset */
 	delay_ms(10);
	delay_ms(50);
	
	/* Set LSHIFT freq, i.e. the DCLK with PLL freq 120MHz set previously */
	/* Typical DCLK for TYX350TFT320240 is 6.5MHz in 24 bit format */
	/* 6.5MHz = 120MHz*(LCDC_FPR+1)/2^20 */
	/* LCDC_FPR = 56796 (0x00DDDC) */
	LCD_WriteCommand(0xE6);
	LCD_WriteData(0x00);
	LCD_WriteData(0xDD);
	LCD_WriteData(0xDC);
	
	/* Set panel mode, varies from individual manufacturer */
	LCD_WriteCommand(0xB0);
	LCD_WriteData(0x20);				/* set 24-bit 3.5" TFT Panel */
	LCD_WriteData(0x00);				/* set Hsync+Vsync mode */
	LCD_WriteData((DISP_HOR_RESOLUTION-1)>>8&0x07); /* Set panel size */
	LCD_WriteData((DISP_HOR_RESOLUTION-1)&0xff);
	LCD_WriteData((DISP_VER_RESOLUTION-1)>>8&0x07);
	LCD_WriteData((DISP_VER_RESOLUTION-1)&0xff);
	LCD_WriteData(0x00);				/* RGB sequence */
	
	/* Set horizontal period */
	LCD_WriteCommand(0xB4);
	#define HT ( DISP_HOR_RESOLUTION + DISP_HOR_PULSE_WIDTH + DISP_HOR_BACK_PORCH + DISP_HOR_FRONT_PORCH )
	LCD_WriteData((HT-1)>>8);	
	LCD_WriteData(HT-1);
	#define HPS ( DISP_HOR_PULSE_WIDTH + DISP_HOR_BACK_PORCH )
	LCD_WriteData((HPS-1)>>8);
	LCD_WriteData(HPS-1);
	LCD_WriteData(DISP_HOR_PULSE_WIDTH-1);
	LCD_WriteData(0x00);
	LCD_WriteData(0x00);
	LCD_WriteData(0x00);
	
	/* Set vertical period */
	LCD_WriteCommand(0xB6);
	
	#define VT ( DISP_VER_PULSE_WIDTH + DISP_VER_BACK_PORCH + DISP_VER_FRONT_PORCH + DISP_VER_RESOLUTION )
	LCD_WriteData((VT-1)>>8);
	LCD_WriteData(VT-1);
	#define VSP ( DISP_VER_PULSE_WIDTH + DISP_VER_BACK_PORCH )
	LCD_WriteData((VSP-1)>>8);
	LCD_WriteData(VSP-1);
	LCD_WriteData(DISP_VER_PULSE_WIDTH-1);
	LCD_WriteData(0x00);
	LCD_WriteData(0x00);
	
	/* Set pixel data interface */
	LCD_WriteCommand(0xF0);
	#ifdef USE_16BIT_PMP
	LCD_WriteData(0x03);		/* 16-bit(565 format) data */
	#else
	LCD_WriteData(0x00);		/* 8-bit data for 16bpp */
	#endif
	delay_ms(5);
	delay_ms(50);
	
	LCD_WriteCommand(0xB8);	    /* Set all GPIOs to output, controlled by host */
	LCD_WriteData(0x0f);		/* Set GPIO0 as output */
	LCD_WriteData(0x01);		/* GPIO[3:0] used as normal GPIOs */
	
	/* LL Reset to LCD */
	SSD1963_GPIO_WR(LCD_SPENA, 1);
	SSD1963_GPIO_WR(LCD_SPCLK, 1);
	SSD1963_GPIO_WR(LCD_SPDAT,1);
	SSD1963_GPIO_WR(LCD_RESET,1);
	SSD1963_GPIO_WR(LCD_RESET,0);
	delay_ms(1);
	SSD1963_GPIO_WR(LCD_RESET,1);
	
	SSD1963_SPI_WriteReg(0x00,0x07);
	SSD1963_SPI_WriteReg(0x01,0x00);
	SSD1963_SPI_WriteReg(0x02,0x03);
	SSD1963_SPI_WriteReg(0x03,0xcc);
	SSD1963_SPI_WriteReg(0x04,0x46);
	SSD1963_SPI_WriteReg(0x05,0x0d);
	SSD1963_SPI_WriteReg(0x06,0x00);
	SSD1963_SPI_WriteReg(0x07,0x00);
	SSD1963_SPI_WriteReg(0x08,0x08);
	SSD1963_SPI_WriteReg(0x09,0x40);
	SSD1963_SPI_WriteReg(0x0a,0x88);
	SSD1963_SPI_WriteReg(0x0b,0x88);
	SSD1963_SPI_WriteReg(0x0c,0x30);
	SSD1963_SPI_WriteReg(0x0d,0x20);
	SSD1963_SPI_WriteReg(0x0e,0x6a);
	SSD1963_SPI_WriteReg(0x0f,0xa4);
	SSD1963_SPI_WriteReg(0x10,0x04);
	SSD1963_SPI_WriteReg(0x11,0x24);
	SSD1963_SPI_WriteReg(0x12,0x24);
	SSD1963_SPI_WriteReg(0x1e,0x00);
	SSD1963_SPI_WriteReg(0x20,0x00);				
	
	
	
	LCD_WriteCommand(0x29);	    /* Turn on display; show the image on display */	
	LCD_SetBacklight(0x00);	
	SSD1963_SetCheckFlag();
}

/******************************************************************************
* Function Name  : LCD_SetArea
* Description    : Sets Area.
* Input          : - start_x: start column
*                  - start_y: start row 
*				   - end_x: end column 
*				   - end_y: end row
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
//static __inline void LCD_SetArea(uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y)
// __inline void LCD_SetArea(uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y)
// {
// 	LCD_WriteCommand(CMD_SET_COLUMN);
// 	LCD_WriteData(start_x>>8);
// 	LCD_WriteData(start_x);
// 	LCD_WriteData(end_x>>8);
// 	LCD_WriteData(end_x);

// 	LCD_WriteCommand(CMD_SET_PAGE);
// 	LCD_WriteData(start_y>>8);
// 	LCD_WriteData(start_y);
// 	LCD_WriteData(end_y>>8);
// 	LCD_WriteData(end_y);
// }

/*******************************************************************************
* Function Name  : LCD_SetBacklight
* Description    : This function makes use of PWM feature of ssd1963 to adjust
*				   the backlight intensity.
* Input          : - intensity: intensity from
*                               0x00 (total backlight shutdown, PWM pin pull-down to VSS)
0xff (99% pull-up, 255/256 pull-up to VDD)
* Output         : None
* Return         : None
* Attention		 : The base frequency of PWM set to around 300Hz with PLL set to 120MHz.
*                  This parameter is hardware dependent.
*                  Backlight circuit with shutdown pin connected to PWM output of ssd1963.
*******************************************************************************/
void LCD_SetBacklight(uint8_t intensity)
{
	LCD_WriteCommand(0xBE);		    /* Set PWM configuration for backlight control */
	LCD_WriteData(0x0E);			/* PWMF[7:0] = 14, PWM base freq = PLL/(256*(1+14))/256 = */ 
	/* 122Hz for a PLL freq = 120MHz */
	LCD_WriteData(intensity);		/* Set duty cycle, from 0x00 (total pull-down) to 0xFF */ 
	/* (99% pull-up , 255/256) */
	LCD_WriteData(0x01);			/* PWM enabled and controlled by host (mcu) */
	LCD_WriteData(0x00);
	LCD_WriteData(0x00);
	LCD_WriteData(0x00);
}

/*******************************************************************************
* Function Name  : LCD_Clear
* Description    : 将屏幕填充成指定的颜色，如清屏，则填充 0xffff
* Input          : - Color: Screen Color
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void LCD_Clear(uint16_t Color)
{
	uint32_t index;

	LCD_SetArea(0,0,DISP_HOR_RESOLUTION -1 ,DISP_VER_RESOLUTION -1 );
	LCD_WriteCommand(CMD_WR_MEMSTART);
	for( index=0; index < DISP_HOR_RESOLUTION * DISP_VER_RESOLUTION; index++ )
	{
		LCD_WriteData(Color);
	}		 
}

/******************************************************************************
* Function Name  : LCD_GetPoint
* Description    : 获取指定座标的颜色值
* Input          : - Xpos: Row Coordinate
*                  - Xpos: Line Coordinate 
* Output         : None
* Return         : Screen Color
* Attention		 : None
*******************************************************************************/
uint16_t LCD_GetPoint(uint16_t Xpos,uint16_t Ypos)
{
	LCD_SetArea(Xpos,Ypos,Xpos,Ypos);
	LCD_WriteCommand(CMD_RD_MEMSTART);
	return LCD_ReadData();
}

/******************************************************************************
* Function Name  : LCD_SetPoint
* Description    : 在指定座标画点
* Input          : - Xpos: Row Coordinate
*                  - Ypos: Line Coordinate 
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void LCD_SetPoint(uint16_t Xpos,uint16_t Ypos,uint16_t point)	 //point为颜色值？？
{
	if( Xpos > DISP_HOR_RESOLUTION || Ypos > DISP_VER_RESOLUTION ) 
	{
		return;
	}
	LCD_SetArea(Xpos,Ypos,Xpos,Ypos);
	LCD_WriteCommand(CMD_WR_MEMSTART);
	LCD_WriteData(point);
}

/******************************************************************************
* Function Name  : LCD_DrawLine
* Description    : Bresenham's line algorithm
* Input          : - x1: A点行座标
*                  - y1: A点列座标 
*				   - x2: B点行座标
*				   - y2: B点列座标 
*				   - color: 线颜色
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/	 
void LCD_DrawLine( uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1 , uint16_t color )
{
	short dx,dy;      /* 定义X Y轴上增加的变量值 */
	short temp;       /* 起点 终点大小比较 交换数据时的中间变量 */

	if( x0 > x1 )     /* X轴上起点大于终点 交换数据 */
	{
		temp = x1;
		x1 = x0;
		x0 = temp;   
	}
	if( y0 > y1 )     /* Y轴上起点大于终点 交换数据 */
	{
		temp = y1;
		y1 = y0;
		y0 = temp;   
	}

	dx = x1-x0;       /* X轴方向上的增量 */
	dy = y1-y0;       /* Y轴方向上的增量 */

	if( dx == 0 )     /* X轴上没有增量 画垂直线 */ 
	{
		do
		{ 
			LCD_SetPoint(x0, y0, color);   /* 逐点显示 描垂直线 */
			y0++;
		}
		while( y1 >= y0 ); 
		return; 
	}
	if( dy == 0 )     /* Y轴上没有增量 画水平直线 */ 
	{
		do
		{
			LCD_SetPoint(x0, y0, color);   /* 逐点显示 描水平线 */
			x0++;
		}
		while( x1 >= x0 ); 
		return;
	}
	/* 布兰森汉姆(Bresenham)算法画线 */
	if( dx > dy )                         /* 靠近X轴 */
	{
		temp = 2 * dy - dx;               /* 计算下个点的位置 */         
		while( x0 != x1 )
		{
			LCD_SetPoint(x0,y0,color);    /* 画起点 */ 
			x0++;                         /* X轴上加1 */
			if( temp > 0 )                /* 判断下下个点的位置 */
			{
				y0++;                     /* 为右上相邻点，即（x0+1,y0+1） */ 
				temp += 2 * dy - 2 * dx; 
			}
			else         
			{
				temp += 2 * dy;           /* 判断下下个点的位置 */  
			}       
		}
		LCD_SetPoint(x0,y0,color);
	}  
	else
	{
		temp = 2 * dx - dy;                      /* 靠近Y轴 */       
		while( y0 != y1 )
		{
			LCD_SetPoint(x0,y0,color);     
			y0++;                 
			if( temp > 0 )           
			{
				x0++;               
				temp+=2*dy-2*dx; 
			}
			else
			{
				temp += 2 * dy;
			}
		} 
		LCD_SetPoint(x0,y0,color);
	}
}

/******************************************************************************
* Function Name  : LCD_EightPoint	  BY Yu Jingxiong  2011.11.13
* Description    : 根据在区间x=0到y=x之间的确定点运用八分法画点
* Input          : - x0: 圆心X座标
*                  - y0: 圆心Y座标 
*				   - x:  八分之一园上X坐标
*				   - y:  八分之一园上y坐标
*				   - color: 线颜色
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void LCD_EightPoint(uint16_t x0,uint16_t y0,uint16_t x,uint16_t y,uint16_t color)
{
	LCD_SetPoint(x0+x,y0+y,color);
	LCD_SetPoint(x0+x,y0-y,color);
	LCD_SetPoint(x0-x,y0+y,color);
	LCD_SetPoint(x0-x,y0-y,color);
	LCD_SetPoint(x0+y,y0+x,color);
	LCD_SetPoint(x0+y,y0-x,color);
	LCD_SetPoint(x0-y,y0+x,color);
	LCD_SetPoint(x0-y,y0-x,color);
}
/******************************************************************************
* Function Name  : LCD_MidpointCircle	  BY Yu Jingxiong  2011.11.13
* Description    : 中点法画园程序
* Input          : - x0: 圆心X坐标
*                  - y0: 圆心Y坐标 
*				   - r : 圆半径
*				   - color: 线颜色
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void LCD_MidpointCircle( uint16_t x0, uint16_t y0, uint16_t r, uint16_t color)
{
	uint16_t x,y;
	float d;

	if( x0+r > DISP_HOR_RESOLUTION || y0+r > DISP_VER_RESOLUTION ) 
	{
		return;
	}
	x=0;
	y=r;
	d=5.0/4-r;

	while(x<=y)
	{
		LCD_EightPoint(x0,y0,x,y,color);
		if(d<0)
			d+=x*2.0+3;
		else
		{
			d+=2.0*(x-y)+5;
			y--;
		} 
		x++;
	}
}
/******************************************************************************
* Function Name  : LCD_Rectangle	  BY Yu Jingxiong  2011.11.13
* Description    : Draw A Rectangle 
* Input          : - x0: 矩形左上角X坐标   注意保证输入的右下角值大于左上角值
*                  - y0: 矩形左上角y坐标
*				   - x1: 矩形右下角X坐标
*				   - y1: 矩形右下角y坐标 
*				   - line_color: 线颜色
- fill_color: 填充颜色
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void LCD_Rectangle( uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1 , uint16_t line_color, uint16_t fill_color )
{
	uint16_t x,y;

	if( x0 > DISP_HOR_RESOLUTION || y0 > DISP_VER_RESOLUTION || x1 > DISP_HOR_RESOLUTION || y1 > DISP_VER_RESOLUTION ) 
	{
		return;
	} 

	LCD_DrawLine(x0, y0, x1, y0, line_color);
	LCD_DrawLine(x0, y0, x0, y1, line_color);
	LCD_DrawLine(x1, y1, x1, y0, line_color);
	LCD_DrawLine(x1, y1, x0, y1, line_color);

	for( x=x0+1; x < x1-1; x++ )
		for( y=y0+1; y < y1-1; y++ )
			LCD_SetPoint(x,y,fill_color);
}

/******************************************************************************
* Function Name  : LCD_Triangle_Up	  BY Yu Jingxiong  2011.11.13
* Description    : Draw A Triangle Up
* Input          : - x0: 三角形左下角X坐标
*                  - y0: 三角形左下角y坐标
*				   - x1: 三角形右下角X坐标
*				   - y1: 三角形右下角y坐标 
*                  - hight: 三角形高度
*				   - line_color: 线颜色
- fill_color: 填充颜色
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void LCD_Triangle_Up(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1 , uint16_t line_color, uint16_t fill_color )
{								         /*  正三角形hight一般取0.707*(x1-x0)  */
	uint16_t x,y;				         /*  顶点坐标((x0+x1)/2 , (y1-hight) )*/
	uint16_t hight= 0.707*(x1-x0)/2;		 /*  一般 y0==y1   */    
	if( x0>DISP_HOR_RESOLUTION-1 || y0>DISP_VER_RESOLUTION-1 || x1>DISP_HOR_RESOLUTION-1 || y1>DISP_VER_RESOLUTION-1 || (y1-hight)<0) 
	{
		return;
	} 

	LCD_DrawLine(x0, y0, x1, y1, line_color);
	LCD_DrawLine(x0, y0, (x0+x1)/2 , y1-hight , line_color);
	LCD_DrawLine(x1, y1, (x0+x1)/2 , y1-hight , line_color);

	for( y=(y1-hight); y < y1; y++ )
		for( x=(x0+x1)/2-((hight+y-y1)*1.732)/3; x < (x0+x1)/2+((hight+y-y1)*1.732)/3; x++ )     
			LCD_SetPoint( x,y,fill_color );

}

/******************************************************************************
* Function Name  : LCD_Triangle_Down	  BY Yu Jingxiong  2011.11.13
* Description    : Draw A Triangle Down
* Input          : - x0: 三角形左上角X坐标
*                  - y0: 三角形左上角y坐标
*				   - x1: 三角形右上角X坐标
*				   - y1: 三角形右上角y坐标 
*                  - hight: 三角形高度
*				   - line_color: 线颜色
- fill_color: 填充颜色
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void LCD_Triangle_Down(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1 ,  uint16_t line_color, uint16_t fill_color )
{								            /*  正三角形hight一般取0.707*(x1-x0)/2  */
	uint16_t x,y;			            	/*  顶点坐标((x0+x1)/2 , (y1-hight) )   */
	uint16_t hight= 0.707*(x1-x0)/2;			/*  一般 y0==y1  */    
	if( x0>DISP_HOR_RESOLUTION-1 || y0>DISP_VER_RESOLUTION-1 || x1>DISP_HOR_RESOLUTION-1 || y1>DISP_VER_RESOLUTION-1 || (y1+hight)>DISP_VER_RESOLUTION-1) 
	{
		return;
	} 

	LCD_DrawLine(x0, y0, x1, y1, line_color);
	LCD_DrawLine(x0, y0, (x0+x1)/2 , y1+hight , line_color);
	LCD_DrawLine(x1, y1, (x0+x1)/2 , y1+hight , line_color);

	for( y=y1 ; y < (y1+hight); y++ )
		for( x=(x0+x1)/2-((hight+y1-y)*1.732)/3; x < (x0+x1)/2+((hight+y1-y)*1.732)/3; x++ )     
			LCD_SetPoint( x,y,fill_color );
}


/******************************************************************************
* Function Name  : PutChar
* Description    : 将Lcd屏上任意位置显示一个字符
* Input          : - Xpos: 水平坐标 
*                  - Ypos: 垂直坐标  
*				   - ASCI: 显示的字符
*				   - charColor: 字符颜色   
*				   - bkColor: 背景颜色 
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void PutChar( uint16_t Xpos, uint16_t Ypos, uint8_t ASCI, uint16_t charColor, uint16_t bkColor )
{
	uint16_t i, j;
	uint8_t buffer[16], tmp_char;
	//GetASCIICode(buffer,ASCI);  /* 取字模数据 */
	for( i=0; i<16; i++ )
	{
		tmp_char = buffer[i];
		for( j=0; j<8; j++ )
		{
			if( (tmp_char >> 7 - j) & 0x01 == 0x01 )
			{
				LCD_SetPoint( Xpos + j, Ypos + i, charColor );  /* 字符颜色 */
			}
			else
			{
				LCD_SetPoint( Xpos + j, Ypos + i, bkColor );  /* 背景颜色 */
			}
		}
	}
}

/******************************************************************************
* Function Name  : PutChar_Amplifier4	 字符放大两倍函数	  By Yu Jingxiong 2011.11.14
* Description    : 将Lcd屏上任意位置显示一个字符
* Input          : - Xpos: 水平坐标 
*                  - Ypos: 垂直坐标  
*				   - ASCI: 显示的字符
*				   - charColor: 字符颜色   LCD上字符显示为16行，8列的像素点
*				   - bkColor: 背景颜色	 ，由于LCD显示原因，字符汉字显示只能扩大4倍，16倍，像素点整体扩倍-->32行，8列
*******************************************************************************/
void PutChar_Amplifier4( uint16_t Xpos, uint16_t Ypos, uint8_t ASCI, uint16_t charColor, uint16_t bkColor )
{
	uint16_t  i, j;
	uint8_t   buffer[16], tmp_char;
	uint16_t  buffer_twice[32], tmp_char_twice;
	//GetASCIICode(buffer,ASCI);  /* 取字模数据 */
	for( i=0; i<16; i++ )
	{
		tmp_char = buffer[i];
		tmp_char_twice = 0;
		for( j=0; j<8; j++ )
		{
			tmp_char_twice = tmp_char_twice << 2 ;	 //注意此处右移函数放在for循环开始、结尾处的差异
			// if( tmp_char & 0x80 == 0x80 )			 //注意此处采取最高位、最低位与	tmp_char_twice 左移右移的关系
			if((tmp_char >> 7 - j) & 0x01 == 0x01	)
				tmp_char_twice = tmp_char_twice | 0x0003 ; 
			else
				tmp_char_twice = tmp_char_twice | 0x0000 ;
			//tmp_char = tmp_char << 1;				  
		}
		buffer_twice[i*2]  = tmp_char_twice ;
		buffer_twice[i*2+1] = tmp_char_twice;
	}

	for( i=0; i<32; i++ )
	{
		tmp_char_twice = buffer_twice[i];
		for( j=0; j<16; j++ )
		{
			if( (tmp_char_twice >> 15 - j) & 0x0001 == 0x0001 )
			{
				LCD_SetPoint( Xpos + j, Ypos + i, charColor );  /* 字符颜色 */
			}
			else
			{
				LCD_SetPoint( Xpos + j, Ypos + i, bkColor );  /* 背景颜色 */
			}
		}
	}
}


/******************************************************************************
* Function Name  : PutChar_Amplifier16	 字符放大两倍函数	  By Yu Jingxiong 2011.11.14
* Description    : 将Lcd屏上任意位置显示一个字符
* Input          : - Xpos: 水平坐标 
*                  - Ypos: 垂直坐标  
*				   - ASCI: 显示的字符
*				   - charColor: 字符颜色   LCD上字符显示为16行，8列的像素点
*				   - bkColor: 背景颜色	 ，由于LCD显示原因，字符汉字显示只能扩大4倍，16倍，像素点整体扩倍-->32行，8列
*******************************************************************************/
void PutChar_Amplifier16( uint16_t Xpos, uint16_t Ypos, uint8_t ASCI, uint16_t charColor, uint16_t bkColor )
{
	uint16_t  i, j;
	uint8_t   buffer[16], tmp_char;
	uint16_t  buffer_twice[32], tmp_char_twice;
	uint32_t  buffer_four[64], tmp_char_four;
	//GetASCIICode(buffer,ASCI);  /* 取字模数据 */
	for( i=0; i<16; i++ )
	{
		tmp_char = buffer[i];
		tmp_char_twice = 0;
		for( j=0; j<8; j++ )
		{
			tmp_char_twice = tmp_char_twice << 2 ;	 //注意此处右移函数放在for循环开始、结尾处的差异
			// if( tmp_char & 0x80 == 0x80 )			 //注意此处采取最高位、最低位与	tmp_char_twice 左移右移的关系
			if((tmp_char >> 7 - j) & 0x01 == 0x01	)
				tmp_char_twice = tmp_char_twice | 0x0003 ; 
			else
				tmp_char_twice = tmp_char_twice | 0x0000 ;
			//tmp_char = tmp_char << 1;				  
		}
		buffer_twice[i*2]  = tmp_char_twice ;
		buffer_twice[i*2+1] = tmp_char_twice;
	}

	for( i=0; i<32; i++ )
	{
		tmp_char_twice = buffer_twice[i];
		tmp_char_four = 0;
		for( j=0; j<16; j++ )
		{		  	 		   		
			tmp_char_four = tmp_char_four << 2 ;                   	//注意此处采取最高位、最低位与	tmp_char_twice 左移右移的关系		         
			if((tmp_char_twice >> 15 - j) & 0x0001 == 0x0001	)		//注意此处右移函数放在for循环开始、结尾处的差异
				tmp_char_four = tmp_char_four | 0x00000003 ; 
			else
				tmp_char_four = tmp_char_four | 0x00000000 ;				  
		}
		buffer_four[i*2]  = tmp_char_four ;
		buffer_four[i*2+1] = tmp_char_four;
	}

	for( i=0; i<64; i++ )
	{
		tmp_char_twice = buffer_twice[i];
		for( j=0; j<32; j++ )
		{
			if( (tmp_char_twice >> 31 - j) & 0x00000001 == 0x00000001 )
			{
				LCD_SetPoint( Xpos + j, Ypos + i, charColor );  /* 字符颜色 */
			}
			else
			{
				LCD_SetPoint( Xpos + j, Ypos + i, bkColor );  /* 背景颜色 */
			}
		}
	}
}

/******************************************************************************
* Function Name  : GUI_Text
* Description    : 在指定座标显示字符串
* Input          : - Xpos: 行座标
*                  - Ypos: 列座标 
*				   - str: 字符串
*				   - charColor: 字符颜色   
*				   - bkColor: 背景颜色 
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void GUI_Text(uint16_t Xpos, uint16_t Ypos, uint8_t *str, uint16_t Color, uint16_t bkColor)
{
	uint8_t TempChar;	//GUI_Text(44,10,"Touch crosshair to calibrate",0xffff,Black);
	do					//示例用法
	{
		TempChar = *str++;  
		PutChar( Xpos, Ypos, TempChar, Color, bkColor );    
		if( Xpos < DISP_HOR_RESOLUTION - 8 )
		{
			Xpos += 8;
		} 
		else if ( Ypos < DISP_VER_RESOLUTION - 16 )
		{
			Xpos = 0;
			Ypos += 16;
		}   
		else
		{
			Xpos = 0;
			Ypos = 0;
		}    
	}
	while ( *str != 0 );
}

/******************************************************************************
* Function Name  : GUI_Text_Amplifier4		 By Yu Jingxiong 2011.11.14
* Description    : 在指定座标显示字符串		 显示扩大四倍的字符串
* Input          : - Xpos: 行座标
*                  - Ypos: 列座标 
*				   - str: 字符串
*				   - charColor: 字符颜色   
*				   - bkColor: 背景颜色 
*******************************************************************************/
void GUI_Text_Amplifier4(uint16_t Xpos, uint16_t Ypos, uint8_t *str,uint16_t Color, uint16_t bkColor)
{
	uint8_t TempChar;	//GUI_Text(44,10,"Touch crosshair to calibrate",0xffff,Black);
	do					//示例用法
	{
		TempChar = *str++;  
		PutChar_Amplifier4( Xpos, Ypos, TempChar, Color, bkColor );  //占用32行，16列  
		if( Xpos < DISP_HOR_RESOLUTION - 16 )
		{
			Xpos += 16;
		} 
		else if ( Ypos < DISP_VER_RESOLUTION - 32 )
		{
			Xpos = 0;
			Ypos += 32;
		}   
		else
		{
			Xpos = 0;
			Ypos = 0;
		}    
	}
	while ( *str != 0 );
}

/******************************************************************************
* Function Name  : GUI_Text_Amplifier16		 By Yu Jingxiong 2011.11.18
* Description    : 在指定座标显示字符串		 显示扩大16倍的字符串
* Input          : - Xpos: 行座标
*                  - Ypos: 列座标 
*				   - str: 字符串
*				   - charColor: 字符颜色   
*				   - bkColor: 背景颜色 
*******************************************************************************/
void GUI_Text_Amplifier16(uint16_t Xpos, uint16_t Ypos, uint8_t *str,uint16_t Color, uint16_t bkColor)
{
	uint8_t TempChar;	//GUI_Text(44,10,"Touch crosshair to calibrate",0xffff,Black);
	do					//示例用法
	{
		TempChar = *str++;  
		PutChar_Amplifier4( Xpos, Ypos, TempChar, Color, bkColor );  //占用64行，32列  
		if( Xpos < DISP_HOR_RESOLUTION - 32 )
		{
			Xpos += 32;
		} 
		else if ( Ypos < DISP_VER_RESOLUTION - 64 )
		{
			Xpos = 0;
			Ypos += 64;
		}   
		else
		{
			Xpos = 0;
			Ypos = 0;
		}    
	}
	while ( *str != 0 );
}
/******************************************************************************
* Function Name  : PutChinese
* Description    : 将Lcd屏上任意位置显示一个中文字
* Input          : - Xpos: 水平坐标 
*                  - Ypos: 垂直坐标  
*				   - str: 显示的中文字
*				   - Color: 字符颜色   
*				   - bkColor: 背景颜色 
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void PutChinese(uint16_t Xpos,uint16_t Ypos,uint8_t *str,uint16_t Color,uint16_t bkColor)
{
	uint8_t i,j;
	uint8_t buffer[32];
	uint16_t tmp_char=0;

	//GetGBKCode(buffer,str);  /* 取字模数据 */

	for ( i = 0; i < 16; i++ )
	{
		tmp_char = buffer[i*2];
		tmp_char = ( tmp_char << 8 );
		tmp_char |= buffer[2*i+1];
		for (j = 0; j < 16; j++ )
		{
			if ( (tmp_char >> 15-j ) & 0x01 == 0x01 )
			{
				LCD_SetPoint(Xpos+j,Ypos+i,Color);  /* 字符颜色 */
			}
			else
			{
				LCD_SetPoint(Xpos+j,Ypos+i,bkColor);  /* 背景颜色 */
			}
		}
	}
}

/******************************************************************************
* Function Name  : PutChinese_Amplifier4						  By Yu Jingxiong 2011.11.14
* Description    : 将Lcd屏上任意位置显示一个中文字
* Input          : - Xpos: 水平坐标 
*                  - Ypos: 垂直坐标  
*				   - str: 显示的中文字
*				   - Color: 字符颜色   
*				   - bkColor: 背景颜色 
*******************************************************************************/
void PutChinese_Amplifier4(uint16_t Xpos,uint16_t Ypos,uint8_t *str,uint16_t Color,uint16_t bkColor)
{
	uint8_t i,j;
	uint8_t buffer[32];
	uint16_t tmp_char=0;
	//GetGBKCode(buffer,str);  /* 取字模数据 */

	for ( i = 0; i < 16; i++ )
	{
		tmp_char = buffer[i*2];
		tmp_char = ( tmp_char << 8 );
		tmp_char |= buffer[2*i+1];
		for (j = 0; j < 16; j++ )
		{
			if ( (tmp_char >> 15-j ) & 0x01 == 0x01 )
			{
				LCD_SetPoint(Xpos+j,Ypos+i,Color);  /* 字符颜色 */
			}
			else
			{
				LCD_SetPoint(Xpos+j,Ypos+i,bkColor);  /* 背景颜色 */
			}
		}
	}
}
/******************************************************************************
* Function Name  : GUI_Chinese
* Description    : 在指定座标显示字符串
* Input          : - Xpos: 行座标
*                  - Ypos: 列座标 
*				   - str: 字符串
*				   - charColor: 字符颜色   
*				   - bkColor: 背景颜色 
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void GUI_Chinese(uint16_t Xpos, uint16_t Ypos, uint8_t *str,uint16_t Color, uint16_t bkColor)
{
	do
	{
		PutChinese(Xpos,Ypos,str++,Color,bkColor);
		str++;
		if ( Xpos < DISP_HOR_RESOLUTION - 16 )
		{
			Xpos += 16;
		}
		else if ( Ypos < DISP_VER_RESOLUTION - 16 )
		{
			Xpos = 0;
			Ypos += 16;
		}
		else
		{
			Xpos = 0;
			Ypos = 0;
		}       
	}
	while(*str!=0);
} 













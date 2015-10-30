/****************************************Copyright (c)**************************************************                         
**
**                                 http://www.powermcu.com
**
**--------------File Info-------------------------------------------------------------------------------
** File name:			GLCD.c
** Descriptions:		SSD1963 ����������
**						
**------------------------------------------------------------------------------------------------------
** Created by:			AVRman
** Created date:		2011-2-23
** Version:				1.0
** Descriptions:		The original version
**
**------------------------------------------------------------------------------------------------------
** Modified by:			Yu Jingxiong   2011.11.10
** Modified date:		�����Ķ�����ע������⡣������keep moving����	
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
/* ʹ�����߷�ʽʱ�����ַ */
/* ���ڲ�ͬ��BANK,ʹ�ò�ͬ��ַ��ʱ�����л����ַ */

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
	/* FSMC���ٶ����� */
	FSMC_NORSRAMTimingInitStructure.FSMC_AddressSetupTime = 5;  /* ��ַ����ʱ��  */
	FSMC_NORSRAMTimingInitStructure.FSMC_AddressHoldTime = 2;	   
	FSMC_NORSRAMTimingInitStructure.FSMC_DataSetupTime = 5;	   /* ���ݽ���ʱ��  */
	FSMC_NORSRAMTimingInitStructure.FSMC_BusTurnAroundDuration = 0x00;
	FSMC_NORSRAMTimingInitStructure.FSMC_CLKDivision = 0x02;
	FSMC_NORSRAMTimingInitStructure.FSMC_DataLatency = 0x00;
	FSMC_NORSRAMTimingInitStructure.FSMC_AccessMode = FSMC_AccessMode_A;	/* FSMC ����ģʽ */
	
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
	/* FSMCд�ٶ����� */
	FSMC_NORSRAMTimingInitStructure.FSMC_AddressSetupTime = 5;   /* ��ַ����ʱ��  */
	FSMC_NORSRAMTimingInitStructure.FSMC_AddressHoldTime = 2;	   
	FSMC_NORSRAMTimingInitStructure.FSMC_DataSetupTime = 5;	   /* ���ݽ���ʱ��  */
	FSMC_NORSRAMTimingInitStructure.FSMC_BusTurnAroundDuration = 0x00;
	FSMC_NORSRAMTimingInitStructure.FSMC_CLKDivision = 0x02;
	FSMC_NORSRAMTimingInitStructure.FSMC_DataLatency = 0x00;
	FSMC_NORSRAMTimingInitStructure.FSMC_AccessMode = FSMC_AccessMode_A;	/* FSMC ����ģʽ */
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

//SSD1963�ϵ���̣�
void SSD1963_PowerUp()
{
	//delay_ms(100)��Լ10ms
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
	static uint8_t _gpioStatus = 0;  /* ssd1963 specific ��̬���� */

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

//����LCD��gpio�Ĵ���������Ϣ���������GPIOҲû�ã��ɴ�������⸴λ
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
	LCD_SetBacklight(0xff);//���Ƚ������ȵ��ڡ����״̬

	
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
* Description    : ����Ļ����ָ������ɫ��������������� 0xffff
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
* Description    : ��ȡָ���������ɫֵ
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
* Description    : ��ָ�����껭��
* Input          : - Xpos: Row Coordinate
*                  - Ypos: Line Coordinate 
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void LCD_SetPoint(uint16_t Xpos,uint16_t Ypos,uint16_t point)	 //pointΪ��ɫֵ����
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
* Input          : - x1: A��������
*                  - y1: A�������� 
*				   - x2: B��������
*				   - y2: B�������� 
*				   - color: ����ɫ
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/	 
void LCD_DrawLine( uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1 , uint16_t color )
{
	short dx,dy;      /* ����X Y�������ӵı���ֵ */
	short temp;       /* ��� �յ��С�Ƚ� ��������ʱ���м���� */

	if( x0 > x1 )     /* X�����������յ� �������� */
	{
		temp = x1;
		x1 = x0;
		x0 = temp;   
	}
	if( y0 > y1 )     /* Y�����������յ� �������� */
	{
		temp = y1;
		y1 = y0;
		y0 = temp;   
	}

	dx = x1-x0;       /* X�᷽���ϵ����� */
	dy = y1-y0;       /* Y�᷽���ϵ����� */

	if( dx == 0 )     /* X����û������ ����ֱ�� */ 
	{
		do
		{ 
			LCD_SetPoint(x0, y0, color);   /* �����ʾ �费ֱ�� */
			y0++;
		}
		while( y1 >= y0 ); 
		return; 
	}
	if( dy == 0 )     /* Y����û������ ��ˮƽֱ�� */ 
	{
		do
		{
			LCD_SetPoint(x0, y0, color);   /* �����ʾ ��ˮƽ�� */
			x0++;
		}
		while( x1 >= x0 ); 
		return;
	}
	/* ����ɭ��ķ(Bresenham)�㷨���� */
	if( dx > dy )                         /* ����X�� */
	{
		temp = 2 * dy - dx;               /* �����¸����λ�� */         
		while( x0 != x1 )
		{
			LCD_SetPoint(x0,y0,color);    /* ����� */ 
			x0++;                         /* X���ϼ�1 */
			if( temp > 0 )                /* �ж����¸����λ�� */
			{
				y0++;                     /* Ϊ�������ڵ㣬����x0+1,y0+1�� */ 
				temp += 2 * dy - 2 * dx; 
			}
			else         
			{
				temp += 2 * dy;           /* �ж����¸����λ�� */  
			}       
		}
		LCD_SetPoint(x0,y0,color);
	}  
	else
	{
		temp = 2 * dx - dy;                      /* ����Y�� */       
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
* Description    : ����������x=0��y=x֮���ȷ�������ð˷ַ�����
* Input          : - x0: Բ��X����
*                  - y0: Բ��Y���� 
*				   - x:  �˷�֮һ԰��X����
*				   - y:  �˷�֮һ԰��y����
*				   - color: ����ɫ
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
* Description    : �е㷨��԰����
* Input          : - x0: Բ��X����
*                  - y0: Բ��Y���� 
*				   - r : Բ�뾶
*				   - color: ����ɫ
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
* Input          : - x0: �������Ͻ�X����   ע�Ᵽ֤��������½�ֵ�������Ͻ�ֵ
*                  - y0: �������Ͻ�y����
*				   - x1: �������½�X����
*				   - y1: �������½�y���� 
*				   - line_color: ����ɫ
- fill_color: �����ɫ
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
* Input          : - x0: ���������½�X����
*                  - y0: ���������½�y����
*				   - x1: ���������½�X����
*				   - y1: ���������½�y���� 
*                  - hight: �����θ߶�
*				   - line_color: ����ɫ
- fill_color: �����ɫ
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void LCD_Triangle_Up(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1 , uint16_t line_color, uint16_t fill_color )
{								         /*  ��������hightһ��ȡ0.707*(x1-x0)  */
	uint16_t x,y;				         /*  ��������((x0+x1)/2 , (y1-hight) )*/
	uint16_t hight= 0.707*(x1-x0)/2;		 /*  һ�� y0==y1   */    
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
* Input          : - x0: ���������Ͻ�X����
*                  - y0: ���������Ͻ�y����
*				   - x1: ���������Ͻ�X����
*				   - y1: ���������Ͻ�y���� 
*                  - hight: �����θ߶�
*				   - line_color: ����ɫ
- fill_color: �����ɫ
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void LCD_Triangle_Down(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1 ,  uint16_t line_color, uint16_t fill_color )
{								            /*  ��������hightһ��ȡ0.707*(x1-x0)/2  */
	uint16_t x,y;			            	/*  ��������((x0+x1)/2 , (y1-hight) )   */
	uint16_t hight= 0.707*(x1-x0)/2;			/*  һ�� y0==y1  */    
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
* Description    : ��Lcd��������λ����ʾһ���ַ�
* Input          : - Xpos: ˮƽ���� 
*                  - Ypos: ��ֱ����  
*				   - ASCI: ��ʾ���ַ�
*				   - charColor: �ַ���ɫ   
*				   - bkColor: ������ɫ 
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void PutChar( uint16_t Xpos, uint16_t Ypos, uint8_t ASCI, uint16_t charColor, uint16_t bkColor )
{
	uint16_t i, j;
	uint8_t buffer[16], tmp_char;
	//GetASCIICode(buffer,ASCI);  /* ȡ��ģ���� */
	for( i=0; i<16; i++ )
	{
		tmp_char = buffer[i];
		for( j=0; j<8; j++ )
		{
			if( (tmp_char >> 7 - j) & 0x01 == 0x01 )
			{
				LCD_SetPoint( Xpos + j, Ypos + i, charColor );  /* �ַ���ɫ */
			}
			else
			{
				LCD_SetPoint( Xpos + j, Ypos + i, bkColor );  /* ������ɫ */
			}
		}
	}
}

/******************************************************************************
* Function Name  : PutChar_Amplifier4	 �ַ��Ŵ���������	  By Yu Jingxiong 2011.11.14
* Description    : ��Lcd��������λ����ʾһ���ַ�
* Input          : - Xpos: ˮƽ���� 
*                  - Ypos: ��ֱ����  
*				   - ASCI: ��ʾ���ַ�
*				   - charColor: �ַ���ɫ   LCD���ַ���ʾΪ16�У�8�е����ص�
*				   - bkColor: ������ɫ	 ������LCD��ʾԭ���ַ�������ʾֻ������4����16�������ص���������-->32�У�8��
*******************************************************************************/
void PutChar_Amplifier4( uint16_t Xpos, uint16_t Ypos, uint8_t ASCI, uint16_t charColor, uint16_t bkColor )
{
	uint16_t  i, j;
	uint8_t   buffer[16], tmp_char;
	uint16_t  buffer_twice[32], tmp_char_twice;
	//GetASCIICode(buffer,ASCI);  /* ȡ��ģ���� */
	for( i=0; i<16; i++ )
	{
		tmp_char = buffer[i];
		tmp_char_twice = 0;
		for( j=0; j<8; j++ )
		{
			tmp_char_twice = tmp_char_twice << 2 ;	 //ע��˴����ƺ�������forѭ����ʼ����β���Ĳ���
			// if( tmp_char & 0x80 == 0x80 )			 //ע��˴���ȡ���λ�����λ��	tmp_char_twice �������ƵĹ�ϵ
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
				LCD_SetPoint( Xpos + j, Ypos + i, charColor );  /* �ַ���ɫ */
			}
			else
			{
				LCD_SetPoint( Xpos + j, Ypos + i, bkColor );  /* ������ɫ */
			}
		}
	}
}


/******************************************************************************
* Function Name  : PutChar_Amplifier16	 �ַ��Ŵ���������	  By Yu Jingxiong 2011.11.14
* Description    : ��Lcd��������λ����ʾһ���ַ�
* Input          : - Xpos: ˮƽ���� 
*                  - Ypos: ��ֱ����  
*				   - ASCI: ��ʾ���ַ�
*				   - charColor: �ַ���ɫ   LCD���ַ���ʾΪ16�У�8�е����ص�
*				   - bkColor: ������ɫ	 ������LCD��ʾԭ���ַ�������ʾֻ������4����16�������ص���������-->32�У�8��
*******************************************************************************/
void PutChar_Amplifier16( uint16_t Xpos, uint16_t Ypos, uint8_t ASCI, uint16_t charColor, uint16_t bkColor )
{
	uint16_t  i, j;
	uint8_t   buffer[16], tmp_char;
	uint16_t  buffer_twice[32], tmp_char_twice;
	uint32_t  buffer_four[64], tmp_char_four;
	//GetASCIICode(buffer,ASCI);  /* ȡ��ģ���� */
	for( i=0; i<16; i++ )
	{
		tmp_char = buffer[i];
		tmp_char_twice = 0;
		for( j=0; j<8; j++ )
		{
			tmp_char_twice = tmp_char_twice << 2 ;	 //ע��˴����ƺ�������forѭ����ʼ����β���Ĳ���
			// if( tmp_char & 0x80 == 0x80 )			 //ע��˴���ȡ���λ�����λ��	tmp_char_twice �������ƵĹ�ϵ
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
			tmp_char_four = tmp_char_four << 2 ;                   	//ע��˴���ȡ���λ�����λ��	tmp_char_twice �������ƵĹ�ϵ		         
			if((tmp_char_twice >> 15 - j) & 0x0001 == 0x0001	)		//ע��˴����ƺ�������forѭ����ʼ����β���Ĳ���
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
				LCD_SetPoint( Xpos + j, Ypos + i, charColor );  /* �ַ���ɫ */
			}
			else
			{
				LCD_SetPoint( Xpos + j, Ypos + i, bkColor );  /* ������ɫ */
			}
		}
	}
}

/******************************************************************************
* Function Name  : GUI_Text
* Description    : ��ָ��������ʾ�ַ���
* Input          : - Xpos: ������
*                  - Ypos: ������ 
*				   - str: �ַ���
*				   - charColor: �ַ���ɫ   
*				   - bkColor: ������ɫ 
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void GUI_Text(uint16_t Xpos, uint16_t Ypos, uint8_t *str, uint16_t Color, uint16_t bkColor)
{
	uint8_t TempChar;	//GUI_Text(44,10,"Touch crosshair to calibrate",0xffff,Black);
	do					//ʾ���÷�
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
* Description    : ��ָ��������ʾ�ַ���		 ��ʾ�����ı����ַ���
* Input          : - Xpos: ������
*                  - Ypos: ������ 
*				   - str: �ַ���
*				   - charColor: �ַ���ɫ   
*				   - bkColor: ������ɫ 
*******************************************************************************/
void GUI_Text_Amplifier4(uint16_t Xpos, uint16_t Ypos, uint8_t *str,uint16_t Color, uint16_t bkColor)
{
	uint8_t TempChar;	//GUI_Text(44,10,"Touch crosshair to calibrate",0xffff,Black);
	do					//ʾ���÷�
	{
		TempChar = *str++;  
		PutChar_Amplifier4( Xpos, Ypos, TempChar, Color, bkColor );  //ռ��32�У�16��  
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
* Description    : ��ָ��������ʾ�ַ���		 ��ʾ����16�����ַ���
* Input          : - Xpos: ������
*                  - Ypos: ������ 
*				   - str: �ַ���
*				   - charColor: �ַ���ɫ   
*				   - bkColor: ������ɫ 
*******************************************************************************/
void GUI_Text_Amplifier16(uint16_t Xpos, uint16_t Ypos, uint8_t *str,uint16_t Color, uint16_t bkColor)
{
	uint8_t TempChar;	//GUI_Text(44,10,"Touch crosshair to calibrate",0xffff,Black);
	do					//ʾ���÷�
	{
		TempChar = *str++;  
		PutChar_Amplifier4( Xpos, Ypos, TempChar, Color, bkColor );  //ռ��64�У�32��  
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
* Description    : ��Lcd��������λ����ʾһ��������
* Input          : - Xpos: ˮƽ���� 
*                  - Ypos: ��ֱ����  
*				   - str: ��ʾ��������
*				   - Color: �ַ���ɫ   
*				   - bkColor: ������ɫ 
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void PutChinese(uint16_t Xpos,uint16_t Ypos,uint8_t *str,uint16_t Color,uint16_t bkColor)
{
	uint8_t i,j;
	uint8_t buffer[32];
	uint16_t tmp_char=0;

	//GetGBKCode(buffer,str);  /* ȡ��ģ���� */

	for ( i = 0; i < 16; i++ )
	{
		tmp_char = buffer[i*2];
		tmp_char = ( tmp_char << 8 );
		tmp_char |= buffer[2*i+1];
		for (j = 0; j < 16; j++ )
		{
			if ( (tmp_char >> 15-j ) & 0x01 == 0x01 )
			{
				LCD_SetPoint(Xpos+j,Ypos+i,Color);  /* �ַ���ɫ */
			}
			else
			{
				LCD_SetPoint(Xpos+j,Ypos+i,bkColor);  /* ������ɫ */
			}
		}
	}
}

/******************************************************************************
* Function Name  : PutChinese_Amplifier4						  By Yu Jingxiong 2011.11.14
* Description    : ��Lcd��������λ����ʾһ��������
* Input          : - Xpos: ˮƽ���� 
*                  - Ypos: ��ֱ����  
*				   - str: ��ʾ��������
*				   - Color: �ַ���ɫ   
*				   - bkColor: ������ɫ 
*******************************************************************************/
void PutChinese_Amplifier4(uint16_t Xpos,uint16_t Ypos,uint8_t *str,uint16_t Color,uint16_t bkColor)
{
	uint8_t i,j;
	uint8_t buffer[32];
	uint16_t tmp_char=0;
	//GetGBKCode(buffer,str);  /* ȡ��ģ���� */

	for ( i = 0; i < 16; i++ )
	{
		tmp_char = buffer[i*2];
		tmp_char = ( tmp_char << 8 );
		tmp_char |= buffer[2*i+1];
		for (j = 0; j < 16; j++ )
		{
			if ( (tmp_char >> 15-j ) & 0x01 == 0x01 )
			{
				LCD_SetPoint(Xpos+j,Ypos+i,Color);  /* �ַ���ɫ */
			}
			else
			{
				LCD_SetPoint(Xpos+j,Ypos+i,bkColor);  /* ������ɫ */
			}
		}
	}
}
/******************************************************************************
* Function Name  : GUI_Chinese
* Description    : ��ָ��������ʾ�ַ���
* Input          : - Xpos: ������
*                  - Ypos: ������ 
*				   - str: �ַ���
*				   - charColor: �ַ���ɫ   
*				   - bkColor: ������ɫ 
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













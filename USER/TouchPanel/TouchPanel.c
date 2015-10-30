/****************************************Copyright (c)****************************************************
**                                      
**                                 http://www.powermcu.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               TouchPanel.c
** Descriptions:            The TouchPanel application function
**
**--------------------------------------------------------------------------------------------------------
** Created by:              AVRman
** Created date:            2010-11-7
** Version:                 v1.0
** Descriptions:            The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
** Version:                 
** Descriptions:            
**
*********************************************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "TouchPanel.h"
#include "..\\SysTick\\systick.h"
#include "..\\GLCD\\GLCD.h"
#include "stm32f10x_spi.h"
#include "lcd\\gl_ui.h"
/* Private variables ---------------------------------------------------------*/
Matrix matrix ;	  /* 矩阵typedef yu */
Coordinate  display ;	/* 坐标typedef yu */

/* DisplaySample LCD坐标上对应的ads7843采样AD值 如：LCD 坐标45,45 应该的X Y采样ADC分别为3388,920 */	
Coordinate ScreenSample[3];
/* LCD上的坐标 */
Coordinate DisplaySample[3] =   {
                                            { 30,  45 },
											{ 290, 45 },
                                            { 160,210 }
	                            } ;

/* Private define ------------------------------------------------------------*/
#define THRESHOLD 2   /* 差值门限 */

extern uint16_t X[3];
extern uint16_t Y[3];

int xdata, ydata;
static int x[10],y[10];
static int count;
static char dbgout[256];
static int event = 0;

#define GATE_TC (40)

#define LCD_XSIZE_TFT 320
#define LCD_YSIZE_TFT 240
struct point  //typedef 坐标点
{
   unsigned short  x;
   unsigned short  y;
};

int gettouch(struct point *pt)
{
	
	if (event) {
		event = 0;
		pt->x = xdata;
		pt->y = ydata;
		return 1;
	}
	return 0;
	
}

void TC_Adj()
{
	int i,k = 0;
	struct point adj[5];
	struct point pt[5] = {
		{60,                 60},
		{LCD_XSIZE_TFT - 60, 60},
		{60,                 LCD_YSIZE_TFT - 60},
		{LCD_XSIZE_TFT -60 , LCD_YSIZE_TFT - 60},
		{LCD_XSIZE_TFT / 2, LCD_YSIZE_TFT / 2},
	};
	gl_ui_setpencolor(RGB16(0,255,0));
	//gl_fill_rect(pt[0].x,  pt[0].y , LCD_XSIZE_TFT, LCD_YSIZE_TFT);
	gl_rect(0,0,LCD_XSIZE_TFT,LCD_YSIZE_TFT);
	gl_ui_setpencolor(RGB16(255,0,0));
	gl_fill_rect(pt[0].x,  pt[0].y ,4,4); 
	
	
	while(k < 6) {
		if (gettouch(&adj[k-1])) {
			gl_ui_setpencolor(RGB16(0,0,0));
			for (i = 0; i < sizeof(pt)/ sizeof(struct point); i++) {
				gl_fill_rect(pt[i].x,  pt[i].y ,4,4);
			}
			gl_ui_setpencolor(RGB16(255,0,0));
			gl_fill_rect(pt[k].x,  pt[k].y ,4,4); 		
			k++;
		}
	}
}


int fliter()
{
	int avx,avy;
	int det;
	avx = (x[0] + x[1] + x[2] + x[3]) >> 2;
	avy = (y[0] + y[1] + y[2] + y[3]) >> 2;

	det = x[0] - x[1] > 0 ? x[0] - x[1] : x[1] - x[0];
	/*
	if (det > GATE_TC) {
		return 0;
	}
	det = x[2] - x[3] > 0 ? x[2] - x[3] : x[3] - x[2];
	if (det > GATE_TC) {
		return 0;
	}
	det = y[0] - y[1] > 0 ? y[0] - y[1] : y[1] - y[0];
	if (det > GATE_TC) {
		return 0;
	}
	det = y[2] - y[3] > 0 ? y[2] - y[3] : y[3] - y[2];
	if (det > GATE_TC) {
		return 0;
	}*/
	xdata = avx;
	ydata = avy;
	event = 1;
	return 1;

}

void TP_GetAdXY(int *x,int *y);
void EXTI0_IRQHandler(void) 
{
	int a,b;
	if(EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		Ctrl_RedLight(1);
//		TP_GetAdXY(&x[count],&y[count]);
		
		a = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0);
/*
		x[0] = Read_X();
		x[1] = Read_X();
		x[2] = Read_X();
		x[3] = Read_X();
		y[0] = Read_Y();
		y[1] = Read_Y();
		y[2] = Read_Y();
		y[3] = Read_Y();*/
		
	
		b = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0);
		sprintf(dbgout,"%d %d", a, b);
		GUI_DispStringAt(dbgout,0,80);
/*
		count++;

		
		if (1||count >= 8) {
			count = 0;
			if (fliter()) {
				sprintf(dbgout,"%d %d\n\n%d %d\n%d %d \n%d %d\n%d %d\n"
					"%d %d\n%d %d \n%d %d\n%d %d",xdata,ydata,x[0], y[0],
					x[1], y[1],
					x[2], y[2],
					x[3], y[3],
					x[4], y[4],
					x[5], y[5],
					x[6], y[6],
					x[7], y[7]);
					GUI_DispStringAt(dbgout,0,80);
			}
		}		*/
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}

/*******************************************************************************
* Function Name  : ADS7843_SPI_Init
* Description    : ADS7843 SPI 初始化
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
static void ADS7843_SPI_Init(void) 
{ 
  SPI_InitTypeDef  SPI_InitStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
  /* DISABLE SPI2 */ 
  SPI_Cmd(SPI2, DISABLE); 
  /* SPI2 Config -------------------------------------------------------------*/ 
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //双工模式
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;    //主端口
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; 
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; 	//选择SPI模式
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; 
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; 
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32; 
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;  //大端模式
  SPI_InitStructure.SPI_CRCPolynomial = 7; 
  SPI_Init(SPI2, &SPI_InitStructure); 
  /* Enable SPI2 */ 
  SPI_Cmd(SPI2, ENABLE); 
} 

/*******************************************************************************
* Function Name  : TP_Init
* Description    : ADS7843端口初始化，触摸屏通信
* Input          : None
* Output         : None
* Return         : None	 
* Attention		 : None
*******************************************************************************/
void TP_Init(void) 
{ 
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB , ENABLE);
	/* Configure SPI2 pins: SCK, MISO and MOSI ---------------------------------*/ 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13  | GPIO_Pin_14 | GPIO_Pin_15; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;       
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	/* TP_CS */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/* TP_IRQ */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	TP_CS(1); 	//x ? GPIO_SetBits(GPIOB,GPIO_Pin_5): GPIO_ResetBits(GPIOB,GPIO_Pin_5)片选端的初始化
	ADS7843_SPI_Init(); 
	//TP_CS(0);
} 


/*******************************************************************************
* Function Name  : DelayUS
* Description    : 延时1us
* Input          : - cnt: 延时值
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
static void DelayUS(vu32 cnt)
{
  uint16_t i;
  for(i = 0;i<cnt;i++)
  {
     uint8_t us = 12; /* 设置值为12，大约延1微秒 */    
     while (us--)     /* 延1微秒	*/
     {
       ;   
     }
  }
}


/*******************************************************************************
* Function Name  : WR_CMD
* Description    : 向 ADS7843 写数据
* Input          : - cmd: 传输的数据
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
static void WR_CMD (uint8_t cmd)  	 //SPI方式进行指令读写
{ 
  /* Wait for SPI2 Tx buffer empty */ 
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET); 
  /* Send SPI2 data */ 
  SPI_I2S_SendData(SPI2,cmd); 
  /* Wait for SPI2 data reception */ 
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET); 
  /* Read SPI2 received data */ 
  SPI_I2S_ReceiveData(SPI2); 
} 


/*******************************************************************************
* Function Name  : RD_AD
* Description    : 读取ADC值 ，12位精度AD？？？
* Input          : None
* Output         : None
* Return         : ADS7843返回二字节数据
* Attention		 : None
*******************************************************************************/
static int RD_AD(void)  //static。。静态函数，保持RD的值？
{ 
  unsigned short buf,temp; 			 //short一般占用两个字节
  /* Wait for SPI2 Tx buffer empty */ 
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET); 
  /* Send SPI2 data */ 
  SPI_I2S_SendData(SPI2,0x0000); 
  /* Wait for SPI2 data reception */ 
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET); 
  /* Read SPI2 received data */ 
  temp=SPI_I2S_ReceiveData(SPI2); 
  buf=temp<<8; 
  DelayUS(1); 
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET); 
  /* Send SPI2 data */ 
  SPI_I2S_SendData(SPI2,0x0000); 
  /* Wait for SPI2 data reception */ 
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET); 
  /* Read SPI2 received data */ 
  temp=SPI_I2S_ReceiveData(SPI2); 
  buf |= temp; 
  buf>>=3; 	//舍弃低三位，只取[14:3位]，12位精度，
  buf&=0xfff;  
  return buf; 
} 


/*******************************************************************************
* Function Name  : Read_X
* Description    : 读取ADS7843通道X+的ADC值 
* Input          : None
* Output         : None
* Return         : ADS7843返回通道X+的ADC值
* Attention		 : None
*******************************************************************************/
int Read_X(void)  
{  
  int i; 
  TP_CS(0); 	//CS作用有待考虑。。。。写指令读数据时候需将CS拉低?
  DelayUS(1); 
  WR_CMD(CHX); 	 /* 通道Y+的选择控制字 */
  DelayUS(1); 
  i=RD_AD(); 
  TP_CS(1); 
  return i;    
} 

/*******************************************************************************
* Function Name  : Read_Y
* Description    : 读取ADS7843通道Y+的ADC值
* Input          : None
* Output         : None
* Return         : ADS7843返回通道Y+的ADC值
* Attention		 : None
*******************************************************************************/
int Read_Y(void)  
{  
  int i; 
  TP_CS(0); 
  DelayUS(1); 
  WR_CMD(CHY); 
  DelayUS(1); 
  i=RD_AD(); 
  TP_CS(1); 
  return i;     
} 

/**
 * @brief	屏蔽和打开触屏中断
 * @param	null
 * @retval	null
 * @remarks	
 * @see	
 */
void TP_EnableIRQ(int v)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	

	if (v == 1) {
		// Step 2 不能放在 Step 1 之上，否则会被硬件重新赋值
		//清空中断标志
		EXTI_ClearITPendingBit(EXTI_Line0);

		// Step 1
		NVIC_InitStructure.NVIC_IRQChannel					 = EXTI0_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority		 = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd				 = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		
		
		
		
		// Step 3
		//选择中断管脚PC.2 PC.3 PC.5
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);

		EXTI_InitStructure.EXTI_Line = EXTI_Line0; //选择中断线路2 3 5
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; //设置为中断请求，非事件请求
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //设置中断触发方式为上下降沿触发
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;                                          //外部中断使能
		EXTI_Init(&EXTI_InitStructure);
	}
	else {
		// Step 1
		NVIC_InitStructure.NVIC_IRQChannel					 = EXTI0_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority		 = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd				 = DISABLE;
		NVIC_Init(&NVIC_InitStructure);
		
		
		// Step 2 不能放在 Step 1 之上，否则会被硬件重新赋值
		//清空中断标志
		EXTI_ClearITPendingBit(EXTI_Line0);
		
		// Step 3
		//选择中断管脚PC.2 PC.3 PC.5
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);

		EXTI_InitStructure.EXTI_Line = EXTI_Line0; //选择中断线路2 3 5
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; //设置为中断请求，非事件请求
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //设置中断触发方式为上下降沿触发
		EXTI_InitStructure.EXTI_LineCmd = DISABLE;                                          //外部中断使能
		EXTI_Init(&EXTI_InitStructure);
	}
}
/*******************************************************************************
* Function Name  : TP_GetAdXY
* Description    : 读取ADS7843 通道X+ 通道Y+的ADC值
* Input          : None
* Output         : None
* Return         : ADS7843返回 通道X+ 通道Y+的ADC值 
* Attention		 : None
*******************************************************************************/
void TP_GetAdXY(int *x,int *y)  
{ 
	
  int adx,ady; 
  adx=Read_X(); 
  DelayUS(1); 
  ady=Read_Y(); 
  *x=adx; 
  *y=ady; 
} 

/*******************************************************************************
* Function Name  : TP_DrawPoint
* Description    : 在指定座标画点
* Input          : - Xpos: Row Coordinate
*                  - Ypos: Line Coordinate 
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void TP_DrawPoint(uint16_t Xpos,uint16_t Ypos)
{
  LCD_SetPoint(Xpos,Ypos,0xf800);     /* 中心点 */
  LCD_SetPoint(Xpos+1,Ypos,0xf800);	  //0xf800 red红色
  LCD_SetPoint(Xpos,Ypos+1,0xf800);
  LCD_SetPoint(Xpos+1,Ypos+1,0xf800);	
}	

/*******************************************************************************
* Function Name  : DrawCross
* Description    : 在指定座标画十字准星
* Input          : - Xpos: Row Coordinate
*                  - Ypos: Line Coordinate 
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void DrawCross(uint16_t Xpos,uint16_t Ypos)
{
  LCD_DrawLine(Xpos-15,Ypos,Xpos-2,Ypos,0xffff);
  LCD_DrawLine(Xpos+2,Ypos,Xpos+15,Ypos,0xffff);
  LCD_DrawLine(Xpos,Ypos-15,Xpos,Ypos-2,0xffff);
  LCD_DrawLine(Xpos,Ypos+2,Xpos,Ypos+15,0xffff);
  
  LCD_DrawLine(Xpos-15,Ypos+15,Xpos-7,Ypos+15,RGB565CONVERT(184,158,131));	  //???????/
  LCD_DrawLine(Xpos-15,Ypos+7,Xpos-15,Ypos+15,RGB565CONVERT(184,158,131));

  LCD_DrawLine(Xpos-15,Ypos-15,Xpos-7,Ypos-15,RGB565CONVERT(184,158,131));
  LCD_DrawLine(Xpos-15,Ypos-15,Xpos-15,Ypos-7,RGB565CONVERT(184,158,131));

  LCD_DrawLine(Xpos+7,Ypos+15,Xpos+15,Ypos+15,RGB565CONVERT(184,158,131));
  LCD_DrawLine(Xpos+15,Ypos+7,Xpos+15,Ypos+15,RGB565CONVERT(184,158,131));

  LCD_DrawLine(Xpos+7,Ypos-15,Xpos+15,Ypos-15,RGB565CONVERT(184,158,131));
  LCD_DrawLine(Xpos+15,Ypos-15,Xpos+15,Ypos-7,RGB565CONVERT(184,158,131));	  	
}	
	
/*******************************************************************************
* Function Name  : Read_Ads7846
* Description    : 得到滤波之后的X Y
* Input          : None
* Output         : None
* Return         : Coordinate结构体地址
* Attention		 : None
*******************************************************************************/
Coordinate *Read_Ads7846(void)
{
  static Coordinate  screen;
  int m0,m1,m2,TP_X[1],TP_Y[1],temp[3];
  uint8_t count=0;
  int buffer[2][9]={{0},{0}};  /* 坐标X和Y进行多次采样 */
  
  do					       /* 循环采样9次 */
  {		   
    TP_GetAdXY(TP_X,TP_Y);  
	buffer[0][count]=TP_X[0];  
	buffer[1][count]=TP_Y[0];
	count++;  
  }								/* 未被按下时，TP_INT_IN==1，循环不执行，当被按下时且采集没到达9次时进行AD采集直至到达9次*/
  while(!TP_INT_IN&& count<9);  /* TP_INT_IN为触摸屏中断引脚,当用户点击触摸屏时TP_INT_IN会被置低 */
  if(count==9)   /* 成功采样9次,进行滤波 */ 
  {  
    /* 为减少运算量,分别分3组取平均值 */
    temp[0]=(buffer[0][0]+buffer[0][1]+buffer[0][2])/3;
	temp[1]=(buffer[0][3]+buffer[0][4]+buffer[0][5])/3;
	temp[2]=(buffer[0][6]+buffer[0][7]+buffer[0][8])/3;
	/* 计算3组数据的差值 */
	m0=temp[0]-temp[1];
	m1=temp[1]-temp[2];
	m2=temp[2]-temp[0];
	/* 对上述差值取绝对值 */
	m0=m0>0?m0:(-m0);
    m1=m1>0?m1:(-m1);
	m2=m2>0?m2:(-m2);
	/* 判断绝对差值是否都超过差值门限，如果这3个绝对差值都超过门限值，则判定这次采样点为野点,抛弃采样点，差值门限取为2 */
	if( m0>THRESHOLD  &&  m1>THRESHOLD  &&  m2>THRESHOLD ) return 0;   //THRESHOLD 差值门限2  ，return 函数结束
	/* 计算它们的平均值，同时赋值给screen */ 
	if(m0<m1)
	{
	  if(m2<m0)   //m2<m0<m1
	    screen.x=(temp[0]+temp[2])/2;	  //screen包含两个量，X与Y
	  else 		  //m0<m1   m0<m2
	    screen.x=(temp[0]+temp[1])/2;	
	}
	else if(m2<m1) // m2<m1<m0
	  screen.x=(temp[0]+temp[2])/2;
	else   //m1<m0   m1<m2
	  screen.x=(temp[1]+temp[2])/2;

	/* 同上 计算Y的平均值 */
    temp[0]=(buffer[1][0]+buffer[1][1]+buffer[1][2])/3;
	temp[1]=(buffer[1][3]+buffer[1][4]+buffer[1][5])/3;
	temp[2]=(buffer[1][6]+buffer[1][7]+buffer[1][8])/3;
	m0=temp[0]-temp[1];
	m1=temp[1]-temp[2];
	m2=temp[2]-temp[0];
	m0=m0>0?m0:(-m0);
	m1=m1>0?m1:(-m1);
	m2=m2>0?m2:(-m2);
	if(m0>THRESHOLD&&m1>THRESHOLD&&m2>THRESHOLD) return 0;

	if(m0<m1)
	{
	  if(m2<m0) 
	    screen.y=(temp[0]+temp[2])/2;
	  else 
	    screen.y=(temp[0]+temp[1])/2;	
    }
	else if(m2<m1) 
	   screen.y=(temp[0]+temp[2])/2;
	else
	   screen.y=(temp[1]+temp[2])/2;

	return &screen;	  //返回的是坐标点指针
  }  
  return 0; 
}
	 
/* 下面是触摸屏到液晶屏坐标变换的转换函数 */
/* 只有在LCD和触摸屏间的误差角度非常小时,才能运用下面公式 */
/*******************************************************************************
* Function Name  : setCalibrationMatrix
* Description    : 计算出 K A B C D E F
* Input          : None
* Output         : None
* Return         : 返回1表示成功 0失败
* Attention		 : None
*******************************************************************************/
FunctionalState setCalibrationMatrix( Coordinate * displayPtr,	  //注意形参，两组坐标，一组矩阵值
                          Coordinate * screenPtr,
                          Matrix * matrixPtr)
{

  FunctionalState retTHRESHOLD = ENABLE ;
  /* K＝(X0－X2) (Y1－Y2)－(X1－X2) (Y0－Y2) */
   matrixPtr->Divider = ((screenPtr[0].x - screenPtr[2].x) * (screenPtr[1].y - screenPtr[2].y)) - 
                       ((screenPtr[1].x - screenPtr[2].x) * (screenPtr[0].y - screenPtr[2].y)) ;

/*	matrixPtr->Divider =-5489152;	 */			   	
  if( matrixPtr->Divider == 0 )
  {
    retTHRESHOLD = DISABLE;
  }
  else				 // ABCDEF == 45110	-547430	  100964140	   -429825	 32835	  137375580
  {
    /* A＝((XD0－XD2) (Y1－Y2)－(XD1－XD2) (Y0－Y2))／K	*/
   matrixPtr->An = ((displayPtr[0].x - displayPtr[2].x) * (screenPtr[1].y - screenPtr[2].y)) - 
                    ((displayPtr[1].x - displayPtr[2].x) * (screenPtr[0].y - screenPtr[2].y)) ;	
/*	  matrixPtr->An =15340;		*/
	/* B＝((X0－X2) (XD1－XD2)－(XD0－XD2) (X1－X2))／K	*/
     matrixPtr->Bn =((screenPtr[0].x - screenPtr[2].x) * (displayPtr[1].x - displayPtr[2].x)) - 
                    ((displayPtr[0].x - displayPtr[2].x) * (screenPtr[1].x - screenPtr[2].x)) ;	  
/*	 matrixPtr->Bn =-573300;	*/
    /* C＝(Y0(X2XD1－X1XD2)+Y1(X0XD2－X2XD0)+Y2(X1XD0－X0XD1))／K */
    matrixPtr->Cn =(screenPtr[2].x * displayPtr[1].x - screenPtr[1].x * displayPtr[2].x) * screenPtr[0].y +
                    (screenPtr[0].x * displayPtr[2].x - screenPtr[2].x * displayPtr[0].x) * screenPtr[1].y +
                    (screenPtr[1].x * displayPtr[0].x - screenPtr[0].x * displayPtr[1].x) * screenPtr[2].y ; 
/*	matrixPtr->Cn = 242160440;	*/			 
    /* D＝((YD0－YD2) (Y1－Y2)－(YD1－YD2) (Y0－Y2))／K	*/
     matrixPtr->Dn = ((displayPtr[0].y - displayPtr[2].y) * (screenPtr[1].y - screenPtr[2].y)) - 
                    ((displayPtr[1].y - displayPtr[2].y) * (screenPtr[0].y - screenPtr[2].y)) ;	 	
/*	matrixPtr->Dn =-410850;	   */
    /* E＝((X0－X2) (YD1－YD2)－(YD0－YD2) (X1－X2))／K	*/
   matrixPtr->En = ((screenPtr[0].x - screenPtr[2].x) * (displayPtr[1].y - displayPtr[2].y)) - 
                    ((displayPtr[0].y - displayPtr[2].y) * (screenPtr[1].x - screenPtr[2].x)) ;	  
/*	matrixPtr->En =	3630;	 */
    /* F＝(Y0(X2YD1－X1YD2)+Y1(X0YD2－X2YD0)+Y2(X1YD0－X0YD1))／K */
    matrixPtr->Fn = (screenPtr[2].x * displayPtr[1].y - screenPtr[1].x * displayPtr[2].y) * screenPtr[0].y +
                    (screenPtr[0].x * displayPtr[2].y - screenPtr[2].x * displayPtr[0].y) * screenPtr[1].y +
                    (screenPtr[1].x * displayPtr[0].y - screenPtr[0].x * displayPtr[1].y) * screenPtr[2].y ;   
/*	matrixPtr->Fn =-5489152;	*/
  }
  return( retTHRESHOLD ) ;
}

/*******************************************************************************
* Function Name  : getDisplayPoint
* Description    : 通过 K A B C D E F 把通道X Y的值转换为液晶屏坐标
* Input          : None
* Output         : None
* Return         : 返回1表示成功 0失败
* Attention		 : None
*******************************************************************************/
FunctionalState getDisplayPoint(Coordinate * displayPtr,
                     Coordinate * screenPtr,
                     Matrix * matrixPtr )
{
  FunctionalState retTHRESHOLD =ENABLE ;

  if( matrixPtr->Divider != 0 )
  {
    /* XD = AX+BY+C */        //触摸屏与显示屏之间的转换公式，待研究。。
    displayPtr->x = ( (matrixPtr->An * screenPtr->x) + 
                      (matrixPtr->Bn * screenPtr->y) + 
                       matrixPtr->Cn 
                    ) / matrixPtr->Divider ;
	/* YD = DX+EY+F */        
    displayPtr->y = ( (matrixPtr->Dn * screenPtr->x) + 
                      (matrixPtr->En * screenPtr->y) + 
                       matrixPtr->Fn 
                    ) / matrixPtr->Divider ;
  }
  else
  {
    retTHRESHOLD = DISABLE;
  }
  return(retTHRESHOLD);
} 

/*******************************************************************************
* Function Name  : TouchPanel_Calibrate
* Description    : 校准触摸屏
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void TouchPanel_Calibrate(void)
{
  /* */ uint8_t i;
  //Coordinate * Ptr;

  for(i=0;i<3;i++)
  {
  
 /*取消开机触摸屏校准2011.11.23yujignxiong 	   
   LCD_Clear(Black);
   //GUI_Text(44,10,"Touch crosshair to calibrate",0xffff,Black);
   delay_ms(500);
   DrawCross(DisplaySample[i].x,DisplaySample[i].y);
   do
   {
     Ptr=Read_Ads7846();   
   }
   while( Ptr == (void*)0 );  //得到AD的XY 坐标，即触摸按点的坐标值，返回值为0
   X[i] = ScreenSample[i].x= Ptr->x; 
   Y[i] = ScreenSample[i].y= Ptr->y;	 */

  /* */  
     ScreenSample[i].x = X[i] ;
     ScreenSample[i].y = Y[i]  ;   
 
  }	

							 //根据显示屏的十字坐标，与触摸屏的获取坐标计算得出参数，【 K A B C D E F】
  setCalibrationMatrix( &DisplaySample[0],&ScreenSample[0],&matrix ) ;  /* 送入值得到参数 */	   

/*
  LCD_Clear(Yellow);
  GUI_Chinese(100,104,"聚联科技",White,Red);
  GUI_Text(90,140,"I LIKE G-LINK",White,Red);
  delay_ms(10000);
  LCD_Clear(Black);
*/

} 

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/

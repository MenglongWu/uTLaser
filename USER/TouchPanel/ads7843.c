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
#include "ads7843.h"
#include "..\\SysTick\\systick.h"
#include "..\\GLCD\\GLCD.h"
#include "stm32f10x_spi.h"
#include "lcd\\gl_ui.h"
#include "touch.h"



static void tp_env();
/* Private define ------------------------------------------------------------*/
#define THRESHOLD 2   /* 差值门限 */

extern uint16_t X[3];
extern uint16_t Y[3];

int xdata, ydata;
static int x[10],y[10];
static int count;
static char dbgout[2560];
static int event = 0;


volatile uint16_t g_touch_ms = 0;
#define GATE_TC (15)


struct adj_tp tpadj;



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

void TC_Test()
{
	int i,j;
	struct point pt[] = {
		{50,50},		{100,50},		{150,50},		{200,50},	{250,50},		{350,50},
		{50,100},		{100,100},		{150,100},		{200,100},	{250,100},		{350,100},
		{50,150},		{100,150},		{150,150},		{200,150},	{250,150},		{350,150},
		{50,200},		{100,200},		{150,200},		{200,200},	{250,200},		{350,200},
		{50,250},		{100,250},		{150,250},		{200,250},	{250,250},		{350,250},
	};

	gl_ui_setpencolor(RGB16(255,255,255));
	for (i = 0; i < sizeof(pt)/ sizeof(struct point); i++) {
		if (pt[i].x < LCD_XSIZE_TFT && pt[i].y < LCD_YSIZE_TFT ) {
			gl_fill_rect(pt[i].x,  pt[i].y ,4,4);	
		}
		
	}

	
}

void tp_getadj(struct adj_tp *adj)
{
	adj->K = tpadj.K;
	adj->A1 = tpadj.A1;
	adj->B1 = tpadj.B1;
	adj->C1 = tpadj.C1;
	adj->A2 = tpadj.A2;
	adj->B2 = tpadj.B2;
	adj->C2 = tpadj.C2;
}

void tp_setadj(struct adj_tp *adj)
{
	tpadj.K = adj->K;
	tpadj.A1 = adj->A1;
	tpadj.B1 = adj->B1;
	tpadj.C1 = adj->C1;
	tpadj.A2 = adj->A2;
	tpadj.B2 = adj->B2;
	tpadj.C2 = adj->C2;
}

void TC_Adj()
{
	int i,k = 0;
	struct point adj[6];
	struct point pt[] = {
		{60,                 60},
		{LCD_XSIZE_TFT - 60,                 60},
		{60,                 LCD_YSIZE_TFT - 60},
		{LCD_XSIZE_TFT - 60 ,LCD_YSIZE_TFT - 60},
		{LCD_XSIZE_TFT / 2, LCD_YSIZE_TFT / 2},
		{0, 0},
	};
	gl_ui_setpencolor(RGB16(255,0,0));
	gl_ui_setbrushcolor(RGB16(255,0,0));
	gl_fill_rect(0, 0 , LCD_XSIZE_TFT-1, LCD_YSIZE_TFT-1);
	// gl_rect(0,0,LCD_XSIZE_TFT,LCD_YSIZE_TFT);
	gl_ui_setpencolor(RGB16(255,255,255));
	gl_fill_rect(pt[0].x,  pt[0].y ,4,4); 
	
	sprintf(dbgout, "%d %d %d ", pt[4].x,pt[4].y, sizeof(pt)/ sizeof(struct point));
	GUI_DispStringAt(dbgout,0,110);

	k = 1;
	while(k < sizeof(pt)/ sizeof(struct point)) {
		if (TP_IsPress()) {
			// gettouch(&adj[k-1]);
			// g_touch_ms = 1000;
			Delay_ms(1000);
			// while (gettouch(&adj[k-1]) == 0);
			gettouch(&adj[k-1]);
			gl_ui_setpencolor(RGB16(255,0,0));
			for (i = 0; i < sizeof(pt)/ sizeof(struct point); i++) {
				gl_fill_rect(pt[i].x,  pt[i].y ,4,4);
			}
			gl_ui_setpencolor(RGB16(255,255,255));
			gl_fill_rect(pt[k].x,  pt[k].y ,4,4); 		

			sprintf(dbgout, "%d %d %d ", pt[k].x,pt[k].y, k);
			GUI_DispStringAt(dbgout,0,110);
			printf("%d %d\r\n", adj[k-1].x, adj[k-1].y);
			k++;
			Delay_ms(300);
			// while (gettouch(&adj[k-1]) == 1);
			// while(TP_IsPress());
			// gettouch(&adj[k-1]);
		}
	}


	gl_ui_setpencolor(RGB16(255,0,0));
	for (i = 0; i < sizeof(pt)/ sizeof(struct point); i++) {
		gl_fill_rect(pt[i].x,  pt[i].y ,4,4);
	}


	tp_adj(pt, adj, &tpadj);
	// sprintf(dbgout, "\n[%f %f %f %f %f %f %f ]\n", (
	// 	tpadj.A1, tpadj.B1, tpadj.C1, 
	// 	tpadj.A2, tpadj.B2, tpadj.C2, 
	// 	tpadj.K));
	// GUI_DispStringAt(dbgout,0,70);
	sprintf(dbgout, "\n[%d %d\n%d %d\n%d %d\n%d %d\n%d %d\n%d %d]\n", 
		adj[0].x,adj[0].y,
		adj[1].x,adj[1].y,
		adj[2].x,adj[2].y,
		adj[3].x,adj[3].y,
		adj[4].x,adj[4].y,
		adj[5].x,adj[5].y);
	GUI_DispStringAt(dbgout,0,10);
}

int getlogxy(struct point *pt)
{
	struct point ad;

	if (gettouch(&ad)) {
		pt->x = tpadj.A1 * ad.x + tpadj.B1 * ad.y + tpadj.C1;
		pt->y = tpadj.A2 * ad.x + tpadj.B2 * ad.y + tpadj.C2;	

		
		return 1;
	}
	return 0;
	
}
int fliter()
{
	int avx,avy;
	int det;
	avx = (x[0] + x[1] + x[2] + x[3]) >> 2;
	avy = (y[0] + y[1] + y[2] + y[3]) >> 2;

	det = x[0] - x[1] > 0 ? x[0] - x[1] : x[1] - x[0];
	
	if (det > GATE_TC) {
		return 0;
	}
	det = x[2] - x[3] > 0 ? x[2] - x[3] : x[3] - x[2];
	if (det > GATE_TC) {
		return 0;
	}
	// det = y[0] - y[1] > 0 ? y[0] - y[1] : y[1] - y[0];
	// if (det > GATE_TC) {
	// 	return 0;
	// }
	xdata = avx;
	ydata = avy;
	event = 1;
	return 1;

}

static void Delay(vu32 cnt)
{
  int j, i;
  for(i = 0;i<cnt;i++)
  {
    for (j = 0;j < 1000;j++);
  }
}

int TP_IsPress()
{
	return !GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0);
}
void TP_GetAdXY(int *x,int *y);
void TP_StartADC()
{
	struct point ad, pt;
	TP_GetAdXY(&x[0], &y[0]);
	TP_GetAdXY(&x[1], &y[1]);
	TP_GetAdXY(&x[2], &y[2]);
	TP_GetAdXY(&x[3], &y[3]);
		
	if (fliter() ) {
		sprintf(dbgout,"%d %d\n\n%d %d\n%d %d \n%d %d\n%d %d\n",xdata,ydata,x[0], y[0],
			x[1], y[1],
			x[2], y[2],
			x[3], y[3]);
		if (TP_IsPress()) {
	 		g_touch_ms = 100;
	 	}
	 	

 		pt.x = tpadj.A1 * xdata + tpadj.B1 * ydata + tpadj.C1;
 		pt.y = tpadj.A2 * xdata + tpadj.B2 * ydata + tpadj.C2;	
	 	GUI_TOUCH_StoreUnstable(pt.x, pt.y);
		//GUI_Exec();

	}
	else {
		if (TP_IsPress()) {
	 		g_touch_ms = 10;
	 	}
	}		
	
	
}
void EXTI0_IRQHandler(void) 
{
	int a,b;
	if(EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		// Ctrl_RedLight(1);
	 	if (TP_IsPress()) {
	 		g_touch_ms = 40;
	 	}
	 	else {
	 		GUI_TOUCH_StoreUnstable(-1, -1);
	 	}
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


	tp_env();
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
static NVIC_InitTypeDef NVIC_InitStructure;
static EXTI_InitTypeDef EXTI_InitStructure;
static void tp_env()
{
	NVIC_InitStructure.NVIC_IRQChannel					 = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority		 = 1;

	EXTI_InitStructure.EXTI_Line    = EXTI_Line0; //选择中断线路2 3 5
	EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt; //设置为中断请求，非事件请求
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //设置中断触发方式为上下降沿触发
}
void TP_EnableIRQ(int v)
{
	
	// 注意下面代码里，屏蔽和打开中断的顺序，操作寄存器的顺序不能改变
	switch (v) {
	case 1:
		// Step 1 清空中断标志
		EXTI->PR = EXTI_Line0;     							// EXTI_ClearITPendingBit(EXTI_Line0);
		// Step 2
		NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		EXTI_InitStructure.EXTI_LineCmd                      = ENABLE;
		break;
	case 0:
		// Step 1
		NVIC_InitStructure.NVIC_IRQChannelCmd                = DISABLE;
		NVIC_Init(&NVIC_InitStructure);
		
		// Step 2 清空中断标志，不能放在 Step 1 之上，否则会被硬件重新赋值
		EXTI->PR = EXTI_Line0;      						// EXTI_ClearITPendingBit(EXTI_Line0);
		
		EXTI_InitStructure.EXTI_LineCmd                      = DISABLE;
		break;
	}
	// Step 3
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);
	// Step 4
	EXTI_Init(&EXTI_InitStructure);
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

  TP_EnableIRQ(0);
  adx=Read_X(); 
  DelayUS(1); 
  ady=Read_Y(); 
  TP_EnableIRQ(1);
  *x=adx; 
  *y=ady; 
} 

void TP_GetADC(struct point *pt)
{ 
  TP_EnableIRQ(0);
  pt->x =Read_X(); 
  DelayUS(1); 
  pt->y =Read_Y(); 
  TP_EnableIRQ(1);
} 


/*******************************************************************************
* Function Name  : Read_Ads7846
* Description    : 得到滤波之后的X Y
* Input          : None
* Output         : None
* Return         : struct point结构体地址
* Attention		 : None
*******************************************************************************/
struct point *Read_Ads7846(void)
{
  static struct point  screen;
  int m0,m1,m2,TP_X[1],TP_Y[1],temp[3];
  uint8_t count=0;
  int buffer[2][9]={{0},{0}};  /* 坐标X和Y进行多次采样 */
  
  do					       /* 循环采样9次 */
  {		   
    // TP_GetAdXY(TP_X,TP_Y);  
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



void GUI_TOUCH_X_ActivateX()
{
	// struct point pt;
	// TP_GetADC(&pt);
}
void GUI_TOUCH_X_ActivateY()
{
	// struct point pt;
	// TP_GetADC(&pt);
}
int  GUI_TOUCH_X_MeasureX(void)
{
	
	return tpadj.A1 * xdata + tpadj.B1 * ydata + tpadj.C1;

}
int  GUI_TOUCH_X_MeasureY(void)
{

	return tpadj.A2 * xdata + tpadj.B2 * ydata + tpadj.C2;	
}
/*********************************************************************************************************
      END FILE
*********************************************************************************************************/

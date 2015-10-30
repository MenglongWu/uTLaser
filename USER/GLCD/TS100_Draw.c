/******************************************************************************
*File:TS100_Draw
液晶屏绘制内容
*******************************************************************************/


#include "GLCD.h" 
#include "HzLib.h"
#include "AsciiLib.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_fsmc.h"
#include "stm32f10x_dac.h"
#include "prj_type.h"
#include "LCD\\gl_ui.h"
#include "LCD\\gl_type.h"
#include "project.h"
#include "key.h"
#include "lib\\base.h"
//#include <PictureData.h>
#include "..\\GLCD\\PictureData.h"
#include "flash.h"
void LCD_FLSAH_DrawPicture(uint16_t StartX,uint16_t StartY,uint16_t EndX,uint16_t EndY, uint8_t * pic);

/*********************************功能函数********************************************/
/******************************************************************************
* Function Name  : LCD_Batter_Show	  BY Yu Jingxiong  2011.11.15		  ok
* Description    : Draw A battery on LCD 
* Input          : - x0: 电池左上角X坐标   注意保证输入的右下角值大于左上角值
*                  - y0: 电池左上角y坐标
- rank ：电池等级状态 0红 1黄 2绿 3绿  4充电状态
*******************************************************************************/
#define LEVEL_POWER    0  //外部供电
#define LEVEL_FULL     2  //充满电
#define LEVEL_CHARGE   4  //正在充电
#define LEVEL_4        6  //电池4格
#define LEVEL_3        7  //电池3格
#define LEVEL_2        8  //电池2格
#define LEVEL_1        9  //电池1格
#define LEVEL_0        10  //电池0格
#define LEVEL_SHUTDOWN 11 //电池自动关机
void LCD_Batter_Show( uint16_t x0, uint16_t y0 ,uint16_t rank )			
{
// 	if( x0 > DISP_HOR_RESOLUTION || y0 > DISP_VER_RESOLUTION || x1 > DISP_HOR_RESOLUTION || y1 > DISP_VER_RESOLUTION  ) 
// 	{
// 		return;
// 	} 
#ifdef _DEBUG_
	switch(rank)
	{					//White   Black   Grey   Blue  Blue2    Red    Magenta   Cyan   Yellow 
	case LEVEL_0: 
		gl_text(250,15,"better 0",-1);
		break;
	case LEVEL_1: 
		gl_text(250,15,"better 1",-1);
		break;
	case LEVEL_2: 
		gl_text(250,15,"better 2",-1);
		break;
	case LEVEL_3: 
		gl_text(250,15,"better 3",-1);
		break;
	case LEVEL_4: 
		gl_text(250,15,"better 4",-1);
		break;
	case LEVEL_CHARGE: 
		gl_text(250,15,"charge...",-1);
		break;
	case LEVEL_FULL: 
		gl_text(250,15,"full",-1);
		break;
	case LEVEL_POWER: 
		gl_text(250,15,"outside",-1);
		break;
	default: 
		break;
	
	}
#else
	switch(rank)
	{					//White   Black   Grey   Blue  Blue2    Red    Magenta   Cyan   Yellow 
	case LEVEL_0: 
		//LCD_Rectangle(  x0+1,  y0+1,  x0+8,  y0+16 ,  Red,  Red );
		LCD_FLSAH_DrawPicture(250,15,250+56-1,15+44-1,(uint8_t*)gImage_battery_0_4);
		break ; 	
	case LEVEL_1: 
		//LCD_Rectangle(  x0+1,  y0+1,  x0+16,  y0+16 ,  Yellow,  Yellow );
		LCD_FLSAH_DrawPicture(250,15,250+56-1,15+44-1,(uint8_t*)gImage_battery_1_4);
		break ; 
	case LEVEL_2: 
		//LCD_Rectangle(  x0+1,  y0+1,  x0+24,  y0+16 ,  Green,  Green );
		LCD_FLSAH_DrawPicture(250,15,250+56-1,15+44-1,(uint8_t*)gImage_battery_2_4);
		break ; 
	case LEVEL_3: 
		//LCD_Rectangle(  x0+1,  y0+1,  x0+24,  y0+16 ,  Green,  Green );
		LCD_FLSAH_DrawPicture(250,15,250+56-1,15+44-1,(uint8_t*)gImage_battery_3_4);
		break ; 
	case LEVEL_4: 
	case LEVEL_FULL: 
	case LEVEL_POWER: 
		//LCD_Rectangle(  x0+1,  y0+1,  x0+24,  y0+16 ,  Green,  Green );
		LCD_FLSAH_DrawPicture(250,15,250+56-1,15+44-1,(uint8_t*)gImage_battery_4_4);
		break ; 
	case LEVEL_CHARGE: 
		//LCD_Rectangle(  x0+1,  y0+1,  x0+32,  y0+16 ,  Green,  Green );
		LCD_FLSAH_DrawPicture(250,15,250+56-1,15+44-1,(uint8_t*)gImage_battery_ac);
		break ;

	default :			    
		break ;
	} 
#endif
}
void LCD_RedLight_Show( uint16_t x0, uint16_t y0 ,uint8_t flag)
{
#ifdef _DEBUG_
	if(flag == 0)
		gl_text(0,0,"Red Off",20);
	else 
		gl_text(0,0,"Red On ",20);
#else
	if(flag == 0)
		LCD_FLSAH_DrawPicture(9,15,9+62-1,15+44-1,(uint8_t*)gImage_redlight_off);
	else 
		LCD_FLSAH_DrawPicture(9,15,9+62-1,15+44-1,(uint8_t*)gImage_redlight_on);
#endif
	
}
/******************************************************************************
* Function Name  : LCD_Wavelength_Selection	  BY Yu Jingxiong  2011.11.15	  ok
* Description    : Draw Wavelength Selection on LCD 
* Input          : - x0: 显示条左上角X坐标   注意保证输入的右下角值大于左上角值
*                  - y0: 显示条左上角y坐标
- wave ：波长选择，0 = 1310nm 1 = 1495nm 2 = 1550nm 	3 = 红光
*******************************************************************************/
void LCD_Wavelength_Selection( uint16_t x0, uint16_t y0 ,uint16_t wave ,uint16_t Color, uint16_t BkColor )
{
// 	uint16_t x,y;
// 	switch(wave)
// 	{					//White   Black   BkColor   Color  Color2    Red    Magenta   Cyan   Yellow 
// 	case 0: 
// 		LCD_RunStop_Select( 130, 5, 1 );	   //x 130 y 5
// 		break ;
// 	case 1: 
// 		Show_Matrix_zimo(x0, y0, Num_1,  4, 56, Color , BkColor);
// 		Show_Matrix_zimo(x0+31, y0, Num_3,  4, 56, Color , BkColor);
// 		Show_Matrix_zimo(x0+63, y0, Num_1,  4, 56, Color , BkColor);
// 		Show_Matrix_zimo(x0+95, y0, Num_0,  4, 56, Color , BkColor);
// 		Show_Matrix_zimo(x0+127, y0, Num_0n,  4, 56, Color , BkColor);
// 		Show_Matrix_zimo(x0+159, y0, Num_0m,  4, 56, Color , BkColor);
// 		break ; 	
// 	case 2: 
// 		Show_Matrix_zimo(x0, y0, Num_1,  4, 56, Color , BkColor);
// 		Show_Matrix_zimo(x0+31, y0, Num_4,  4, 56, Color , BkColor);
// 		Show_Matrix_zimo(x0+63, y0, Num_9,  4, 56, Color , BkColor);
// 		//  Show_Matrix_zimo(x0+95, y0, Num_0,  4, 56, Color , BkColor);
// 		//  Show_Matrix_zimo(x0+127, y0, Num_n,  4, 56, Color , BkColor);
// 		//  Show_Matrix_zimo(x0+159, y0, Num_m,  4, 56, Color , BkColor);
// 		break ; 
// 	case 3: 
// 		Show_Matrix_zimo(x0, y0, Num_1,  4, 56, Color , BkColor);
// 		Show_Matrix_zimo(x0+31, y0, Num_5,  4, 56, Color , BkColor);
// 		Show_Matrix_zimo(x0+63, y0, Num_5,  4, 56, Color , BkColor);
// 		Show_Matrix_zimo(x0+95, y0, Num_0,  4, 56, Color , BkColor);
// 		//Show_Matrix_zimo(x0+127, y0, Num_n,  4, 56, Color , BkColor);
// 		//Show_Matrix_zimo(x0+159, y0, Num_m,  4, 56, Color , BkColor);
// 		break ; 
// // 	case 4: 
// // 		for( x=x0; x < x0+31; x++ )	
// // 			for( y=y0; y < y0+56; y++ )
// // 				LCD_SetPoint(x,y,BkColor);
// // 		Show_Matrix_zimo(x0+31, y0, Num_V,  4, 56, Color , BkColor);		
// // 		Show_Matrix_zimo(x0+63, y0, Num_L,  4, 56, Color , BkColor);
// // 		Show_Matrix_zimo(x0+95, y0, Num_S,  4, 56, Color , BkColor);
// // 		for( x=x0+127; x < x0+191; x++ )	
// // 			for( y=y0; y < y0+56; y++ )
// // 				LCD_SetPoint(x,y,BkColor);
// // 		break ;

// 	default :			    
// 		break ;			                          
// 	} 
}
void LCD_Wavelength_Selection_Ex( uint16_t x0, uint16_t y0 ,uint16_t wave ,uint16_t Color, uint16_t BkColor )
{

	uint8_t *waveCode[10] = {Num_00,Num_01,Num_02,Num_03,Num_04,Num_05,Num_06,Num_07,Num_08,Num_09};
	uint8_t i;
	//配置的三个波长具体数值
	uint16_t *cfgwave[4] = {0,&g_adj_power._ch1wave,&g_adj_power._ch2wave,&g_adj_power._ch3wave};
	uint16_t tmpwave;
	uint8_t codeIndex;

	//为了兼容其他部分函数，wave只能是1、2、3
	if(wave) {
		tmpwave = *cfgwave[wave];
	}
	else {
		return;
	}	
	for(i = 0;i < 4;++i) {					//显示固定4个波长字母，每个字母大小16x29
		codeIndex = tmpwave % 10;
		if(tmpwave) {						//从字母最低位开始显示
			Show_Matrix_zimo(x0+48-16*i, y0, waveCode[codeIndex],  2, 29, Color , BkColor);
		}
		else {
			gl_clear(	x0+48-16*i,		y0,	//字母不足4位，前面用背景色擦除
						x0+48-16*i+16,y0+29,Black);	
		}
		tmpwave /= 10;
	}
	Show_Matrix_zimo(x0+64, y0, Num_0n,  2, 29, Color , BkColor);	//显示单位nm
	Show_Matrix_zimo(x0+80, y0, Num_0m,  2, 29, Color , BkColor);
}


/******************************************************************************
* Function Name  : LCD_OperatMode_Selection	  BY Yu Jingxiong  2011.11.20	  ok
* Description    : Draw OperatMode Selection on LCD 
* Input          : - x0: 显示条左上角X坐标   注意保证输入的右下角值大于左上角值
*                  - y0: 显示条左上角y坐标
- mode ：波长选择，0 = CW、 1 = 270Hz、2 = 1KHz、3 = 2KHz  连续光/脉冲光选择
*******************************************************************************/
void LCD_OperatMode_Selection( uint16_t x0, uint16_t y0 ,uint16_t mode ,uint16_t Color, uint16_t BkColor )
{ 
	uint16_t x,y;
	//小号字体显示 16* 29
	switch(mode)
	{					//White   Black   BkColor   Blue  Color    Red    Magenta   Cyan   Yellow 
	case 0: 
		for( x=x0; x < x0+47; x++ )	//右侧触摸按键条宽
			for( y=y0; y < y0+29; y++ )
				LCD_SetPoint(x,y,BkColor);
		Show_Matrix_zimo(x0+47, y0, Num_0C,  2, 29, Color , BkColor);
		Show_Matrix_zimo(x0+63, y0, Num_0W,  2, 29, Color , BkColor);
		for( x=x0+79; x < x0+111; x++ )	//右侧触摸按键条宽
			for( y=y0; y < y0+29; y++ )
				LCD_SetPoint(x,y,BkColor);
		break ; 	
	case 1: 
		Show_Matrix_zimo(x0, y0, Num_00,  2, 29, Color , BkColor);
		Show_Matrix_zimo(x0+15, y0, Num_0dian,  2, 29, Color , BkColor);
		Show_Matrix_zimo(x0+31, y0, Num_02,  2, 29, Color , BkColor);
		Show_Matrix_zimo(x0+47, y0, Num_07,  2, 29, Color , BkColor);
		Show_Matrix_zimo(x0+63, y0, Num_0K,  2, 29, Color , BkColor);
		Show_Matrix_zimo(x0+79, y0, Num_0H,  2, 29, Color , BkColor);
		Show_Matrix_zimo(x0+95, y0, Num_0z,  2, 29, Color , BkColor);
		break ; 
	case 2: 
		for( x=x0; x < x0+47; x++ )	
			for( y=y0; y < y0+29; y++ )
				LCD_SetPoint(x,y,BkColor);
		Show_Matrix_zimo(x0+47, y0, Num_01,  2, 29, Color , BkColor);	//确保KHz位置不变
		Show_Matrix_zimo(x0+63, y0, Num_0K,  2, 29, Color , BkColor);
		Show_Matrix_zimo(x0+79, y0, Num_0H,  2, 29, Color , BkColor);
		Show_Matrix_zimo(x0+95, y0, Num_0z,  2, 29, Color , BkColor);
		break ; 
	case 3: 
		Show_Matrix_zimo(x0+47, y0, Num_02,  2, 29, Color , BkColor);
		Show_Matrix_zimo(x0+63, y0, Num_0K,  2, 29, Color , BkColor);
		Show_Matrix_zimo(x0+79, y0, Num_0H,  2, 29, Color , BkColor);
		Show_Matrix_zimo(x0+95, y0, Num_0z,  2, 29, Color , BkColor);
		break ;

	default :			    
		break ;			                          
	} 


	if(Operating_Mode & 0x80) {
		//添加OFF光源图标
 		for( x=x0; x < x0+130; x++ )	//右侧触摸按键条宽
 			for( y=y0; y < y0+29; y++ )
 				LCD_SetPoint(x,y,BkColor);
		//gl_fill_rect(x0,y0,130,29);
		Show_Matrix_zimo(x0+47, y0, Num_0O,  2, 29, Color , BkColor);
		Show_Matrix_zimo(x0+63, y0, Num_0f,  2, 29, Color , BkColor);
		Show_Matrix_zimo(x0+79, y0, Num_0f,  2, 29, Color , BkColor);
	}

}



/******************************************************************************
* Function Name  : LCD_Timing_Display	  BY Yu Jingxiong  2011.11.15	  ok
* Description    : Draw Wavelength Selection on LCD 
* Input          : - x0: 显示条左上角X坐标   注意保证输入的右下角值大于左上角值
*                  - y0: 显示条左上角y坐标
- on_off ：定时器状态，0 倒计时关闭状态 OFF,  5min  ，10min  ，15min  ，30min  ，60min
*******************************************************************************/
void LCD_Timing_Display( uint16_t x0, uint16_t y0 ,uint16_t on_off )
{

	if( x0 > DISP_HOR_RESOLUTION || y0 > DISP_VER_RESOLUTION  ) 
	{
		return;
	} 
#ifdef _DEBUG_
	switch(on_off)
	{
	case 0:
		gl_text(125,15,"tim off  ",-1);
		break;
	case 1:
		break;
	case 2:
		gl_text(125,15,"tim 10min",-1);
		break;
	case 3:
		break;
	case 4:
		break;
	case 5:
		break;
	default:
		break;
	}
#else
	switch(on_off)
	{					//White   Black   Grey   Blue  Blue2    Red    Magenta   Cyan   Yellow 
	case 0:   //OFF
		LCD_FLSAH_DrawPicture(125,15,125+77-1,15+44-1,(uint8_t*)gImage_t_off);
		break ; 	
	case 1: 
		GUI_Text( x0,  y0,  "T-5Min ", Green,  Black);
		break ;
	case 2:
		LCD_FLSAH_DrawPicture(125,15,125+77-1,15+44-1,(uint8_t*)gImage_t_10min);
		break ; 
	case 3:
		//LCD_DrawPicture( x0-21, y0, x0+8, y0+35, Nao_Zhong_30_36_GREEN ,  1080);    
		GUI_Text( x0,  y0,  "T-15Min", Green,  Black);
		break ;
	case 4: 
		//LCD_DrawPicture( x0-21, y0, x0+8, y0+35, Nao_Zhong_30_36_GREEN ,  1080);   
		GUI_Text( x0,  y0,  "T-30Min", Green,  Black);
		break ;
	case 5: 
		//LCD_DrawPicture( x0-21, y0, x0+8, y0+35, Nao_Zhong_30_36_GREEN ,  1080);   
		GUI_Text( x0,  y0,  "T-60Min", Green,  Black);
		break ;

	default :			    
		break ;			                          
	} 
#endif
}

/***************************************************************************************************/
/******************************************************************************
* Function Name  : Show_Matrix_zimo
* Description    : 在指定座标显示字模提取软件提取的字模数据		  OK
* Input          : - Xpos: 起始行座标
*                  - Ypos: 起始列座标 
*				   - Buffer: 保存字模数据的数组地址。。。
*				   - Wide_char,  字模宽度，以字节计算，像素宽度 = Wide_char * 8
*				   - High     ， 字模高度，以像素点计算
*				   - charColor: 字符颜色   
*				   - bkColor: 背景颜色 
*******************************************************************************/
void Show_Matrix_zimo(uint16_t Xpos, uint16_t Ypos, uint8_t *Buffer, uint16_t Wide_char, uint16_t High, uint16_t charColor, uint16_t bkColor)
{
	uint16_t  x_char ,x;
	uint16_t  Temp_y;  //临时 Y坐标
	uint8_t  /* Temp_Buffer[1000], */tmp_char;	  //注意：最大显示字体的字节数目为1000

	//memcpy( Temp_Buffer, Buffer, Wide_char*High);

	for( Temp_y = Ypos; Temp_y< Ypos+High-1 ; Temp_y++)		//y坐标递增
	{
		for( x_char=0; x_char<Wide_char; x_char++ )	 //x坐标字符计数增量
		{
			tmp_char = Buffer[x_char + (Temp_y - Ypos)*Wide_char];
			for( x=0; x<8; x++ )
			{
				if( (tmp_char >> 7 - x) & 0x01 == 0x01 )
				{
					LCD_SetPoint( ( Xpos + x_char * 8 + x ), Temp_y, charColor );  /* 字符颜色 */
				}
				else
				{
					LCD_SetPoint( ( Xpos + x_char * 8 + x ), Temp_y, bkColor );  /* 背景颜色 */
				}
			}
		}
	}
}


/******************************************************************************
* Function Name  : LCD_Power_Control_Selection	  BY Yu Jingxiong  2011.11.21	  ok
* Description    : Draw Power Control on LCD 
* Input          : - x0: 显示条左上角X坐标   注意保证输入的右下角值大于左上角值
*                  - y0: 显示条左上角y坐标
*                  - Current_Power: 显示功率值

change ：波长选择，0 = decrease、 1 = 	increase  
操作方式：以 1dbm 递增或递减，变化范围是：-30dbm到3dbm，共34个等级
*******************************************************************************/
// void LCD_Power_Control_Selection( uint16_t x0, uint16_t y0 ,int16_t Current_Power ,uint16_t CharColor, uint16_t BkColor )
// { 
// }
void LCD_Power_Control_Selection_Ex( uint16_t x0, uint16_t y0 ,int16_t Current_Power ,uint16_t CharColor, uint16_t BkColor )
{ 
	//uint8_t (*code[])[224] = {&Num_0,&Num_1,&Num_2,&Num_3,&Num_4,&Num_5,&Num_6,&Num_7,&Num_8,&Num_9};
	uint8_t *code[] = {(uint8_t*)Num_0,(uint8_t*)Num_1,(uint8_t*)Num_2,(uint8_t*)Num_3,(uint8_t*)Num_4,
						(uint8_t*)Num_5,(uint8_t*)Num_6,(uint8_t*)Num_7,(uint8_t*)Num_8,(uint8_t*)Num_9};
	int16_t tdisplay = Current_Power;
	uint8_t digit;
	if(tdisplay < 0) {
		Show_Matrix_zimo(x0, y0, Num_fuhao,  4,56, CharColor , BkColor);
		tdisplay = -tdisplay;
	}
	tdisplay %= 100;
	digit = tdisplay / 10;
	Show_Matrix_zimo(x0+32, y0, code[digit],  4,56, CharColor , BkColor);
	digit = tdisplay % 10;
	Show_Matrix_zimo(x0+62, y0, code[digit],  4,56, CharColor , BkColor);

	LCD_FLSAH_DrawPicture(x0+98,y0+13,x0+98+63-1,y0+13+33-1,(uint8_t*)gImage_dbm);	


	//Show_Matrix_zimo(x0+98, y0, Num_big_d,  4,56, CharColor , BkColor);
 	//Show_Matrix_zimo(x0+130, y0, Num_big_b,  4,56, CharColor , BkColor);
 	//Show_Matrix_zimo(x0+162, y0, Num_big_m,  4,56, CharColor , BkColor);
}

/******************************************************************************
* Function Name  : LCD_Menu_Select	  BY Yu Jingxiong  2011.11.21	  ok
* Description    : Draw menu selection on LCD 
* Input          : - x0: 显示条左上角X坐标   注意保证输入的右下角值大于左上角值
*                  - y0: 显示条左上角y坐标
*                  - menu: 当前菜单选项

《menu》	 0 menu  1波长   2模式   3功率   4定时器   
*******************************************************************************/
void LCD_Menu_Select( uint16_t menu )	   //x 279 y 36
{ 
}	

/******************************************************************************
* Function Name  : LCD_Menu_Select	  BY Yu Jingxiong  2011.11.24	  ok
* Description    : Draw menu selection on LCD 
* Input          : - x0: 显示条左上角X坐标   注意保证输入的右下角值大于左上角值
*                  - y0: 显示条左上角y坐标 
*******************************************************************************/
// void LCD_RunStop_Select(uint16_t x0,uint16_t y0,uint16_t RunStop )	   //x 140 y 36
// { 
// }


/*****************数字显示函数封装 2011.12.05*****************************/
/*************************************************************************/
void GUI_Text_Show_Number(uint16_t x, uint16_t y, uint8_t number, uint16_t Color, uint16_t bkColor )
{
}

/*********************************************************************************************************
END FILE
*********************************************************************************************************/


//片上FLASH图片显示函数


//基本函数
/*********************************************************************
* Function Name  : LCD_FLSAH_DrawPicture
* Description    : 读取FLASH中的图片数据进行显示操纵
* Input          : - StartX: X起点
*                  - StartY: Y起点 
*				   - EndX: X终点
*				   - EndY: Y终点   
*				   - pic: 数据组指针强制转换结果
* By yujingxiong    2012.04.26 
example  ：LCD_FLSAH_DrawPicture( 0, 0, 31, 59, (uint8_t *)&Glink32_60 );
const unsigned char Glink32_60[3840]
*********************************************************************/
void LCD_FLSAH_DrawPicture(uint16_t StartX,uint16_t StartY,uint16_t EndX,uint16_t EndY, uint8_t * pic)
{
	uint16_t i;
	uint16_t x,y;
	uint16_t Once_Number;		
	uint8_t Half_Pixel_Color[640] = {0};
	uint16_t Once_Pixel_Color[320] = {0};  

	Once_Number = (EndX - StartX + 1);	 

	for( y=StartY; y<=EndY ; y++ )  
	{
		memcpy(Half_Pixel_Color, ( pic + 2*Once_Number*(y-StartY)) , 2*Once_Number);	
		for( i=0; i < Once_Number; i++ )
		{
			Once_Pixel_Color[i] = (Half_Pixel_Color[2*i])*256 + Half_Pixel_Color[2*i+1];	   	   	   
		}
		for( x=StartX; x<=EndX ; x++ )
			LCD_SetPoint(x,y,Once_Pixel_Color[x - StartX]);
	}
}

// double atof_(char* s)
// {
// 	double v=0,k = 0,j = 1;
// 	int sign=1;
// 	while ( *s == ' '  ||  (unsigned int)(*s - 9) < 5u) s++;
// 	switch (*s)
// 	{
// 	case '-':
// 		sign=-1;
// 	case '+':
// 		++s;
// 	}
// 	while ((unsigned int) (*s - '0') < 10u)
// 	{
// 		v=v*10+*s-'0';
// 		++s;
// 	}
// 	if(*s == '.') {
// 		s++;
// 		while ((unsigned int) (*s - '0') < 10u)
// 		{
// 			k=k*10+*s-'0';
// 			++s;
// 			j = j * 10;
// 		}
// 		k /= j;
// 	}
// 	
// 	v = v + k;
// 	return sign==-1?-v:v;
// }

void UI_DebugMain()
{
	POINT stloc[] = {
		35,0, 72,0,   115,0,  158,0,
		25,25, 72,25,   115,25, 158,25 ,
								158,38};
	POINT loc[] = {
		25,12,   67,12,   105,12, 158,0,
		25,38,   67,38,   105,38, 158,25,
								  158,38};
	int8_t *caption[] = {"ADC","DAC","dBm","Adjust","Mode","Laser","Auto","Produce","Exit"};
	
	int8_t enableAuto = 0;
	int16_t oldpen,oldbrush,oldbk,oldfont;	
	int32_t index = 0,oldindex = 0,i;
	int32_t strlen;
	uint8_t flag = 0;
	uint8_t btnClk = 0;
	char strout[50];
	uint32_t retInt;
	float dbm,battery;
	uint8_t mode = 0,laser = 1;
	int val[5] = {0};
	uint16_t ad;


	oldfont = gl_ui_setfontcolor(COL_Black);
	oldpen = gl_ui_setpencolor(COL_Black);
	oldbrush =  gl_ui_setbrushcolor(COL_White);
	oldbk = gl_ui_setbkcolor(COL_White);
	
	
	Wavelength_Selection_state = 1;
_Redraw:;
	gl_clear(0,0,320,240,COL_White);
	gl_text(0,224-12,RELEASE_DATE,-1);
	gl_text(0,224,TARGET_NAME,-1);
	
	for(i = 0;i < 9;++i) {
		gl_text(stloc[i].x,stloc[i].y,caption[i],-1);
	}
	
	for(i = 0;i < 2;++i) {
		sprintf(strout,"%d",val[i]);
		gl_text(loc[i].x+2,loc[i].y,strout,-1);
	}
	sprintf(strout,"%4.4d",g_power.adc);
	gl_text(loc[0].x+2,loc[0].y,strout,-1);
	
	sprintf(strout,"%4.4d",g_power.dac);
	gl_text(loc[1].x+2,loc[1].y,strout,-1);
	
	dbm = g_power.set/1000.0;
	sprintf(strout,"%0.2f",dbm);
	gl_text(loc[2].x+2,loc[2].y,strout,-1);
	mode = Operating_Mode;
	switch(mode) {
	case 1:
		sprintf(strout,"270  ");
		break;
	case 2:
		sprintf(strout,"1K  ");
		break;
	case 3:
		sprintf(strout,"2K  ");
		break;
	default:
		sprintf(strout,"CW    ");
		mode = 0;
		break;
	}
	gl_text(loc[4].x+2,loc[4].y,strout,-1);
	Ctrl_Operating_Mode(mode);

	laser = Wavelength_Selection_state;
	switch(laser) {
	case 1:
		sprintf(strout,"1310");
		break;
	case 2:
		sprintf(strout,"1490");
		break;
	case 3:
		sprintf(strout,"1550");
		break;
	case 4:
		sprintf(strout,"650 ");
		break;
	default://1310
		//laser = 4;
		laser = 0;
		sprintf(strout,"no  ");
		break;
	}
	gl_text(loc[5].x+2,loc[5].y,strout,-1);
	Ctrl_Wavelength(laser);
	
	switch(enableAuto) {
	case 1:
		sprintf(strout,"on ");
		break;
	case 0:
		sprintf(strout,"off");
	}
	gl_text(loc[6].x+2,loc[6].y,strout,-1);
	//Ctrl_Power(&g_power);
	
	//draw_arrow(10,10,1);
	flag = 1;
	g_debug_ms = 1000;
	while(1) {
		if(ProTick1963IsLive())
 			goto _Redraw;
		if(enableAuto)
			AutoCtrlPower();
		
		if(g_debug_ms >= 1000) { 
			g_debug_ms = 0;
			ad = GetAD(1);
			sprintf(strout,"ADC:%d  ",ad);
			gl_text(0,64,strout,-1);
			sprintf(strout,"DAC:%d  ",g_power.dac);
			gl_text(0,76,strout,-1);
			battery = GetBattery();
			sprintf(strout,"Battery:%0.3f",battery);
			gl_text(0,88,strout,30);
		}
		
		if(KeyPress(KEY_PORT_B,KEY_B)) {
			if(KeyPress(KEY_PORT_Z,KEY_Z)) {
				flag =  1;
				val[index] += 1;
				
				 
				
				gl_ui_setbkmode(BACKFILL);
				sprintf(strout,"%4.4d",val[index]);
				gl_text(loc[index].x+2,loc[index].y,strout,-1);
				if(index == 0) {
					
					g_power.adc = val[index];
				}
				else if(index == 1) {
					DAC_SetChannel1Data(DAC_Align_12b_R, (uint16_t)(val[index]));    //  （dacData/4096）*3.3== OUT
					DAC_SoftwareTriggerCmd(DAC_Channel_1,ENABLE); 
					g_power.dac = val[index];
				}
				
				//gl_ui_setbkmode(TRANSPARENT);
			}
			if(KeyPress(KEY_PORT_C,KEY_C)) {
				flag =  1;
				val[index] -= 1;
				
				
				
				gl_ui_setbkmode(BACKFILL);
				sprintf(strout,"%4.4d",val[index]);
				gl_text(loc[index].x+2,loc[index].y,strout,-1);
				if(index == 0) {
					g_power.adc = val[index];
				}
				else if(index == 1) {
					DAC_SetChannel1Data(DAC_Align_12b_R, (uint16_t)(val[index]));    //  （dacData/4096）*3.3== OUT
					DAC_SoftwareTriggerCmd(DAC_Channel_1,ENABLE);  
					g_power.dac = val[index];
				}
			}
		}
// 		else if(KeyPress(KEY_PORT_Z,KEY_Z)) {
// 			flag =  1;
// 			if(index > 3) {
// 				oldindex = index;
// 				index-= 1;
// 			}
// 		}
		else if(KeyPress(KEY_PORT_Z,KEY_Z)) {
			flag =  1;
			btnClk = 1;
		}
		
		else if(KeyPress(KEY_PORT_C,KEY_C)) {
			flag =  1;
			oldindex = index;
			index++;
			if(index > 8)
				index = 0;			
			DrawFocus(loc[oldindex].x,loc[oldindex].y,COL_White);
			DrawFocus(loc[index].x,loc[index].y,COL_Black);
			//goto _Redraw;
		}
		else if(KeyPress(KEY_PORT_A,KEY_A)) {
			flag =  1;
			oldindex = index;
			index--;
			if(index < 0)
				index = 8;
			DrawFocus(loc[oldindex].x,loc[oldindex].y,COL_White);
			DrawFocus(loc[index].x,loc[index].y,COL_Black);
			//goto _Redraw;
		}
		
		if(flag) {
			
			
			
			
			if(btnClk) {
				if(index == 0) {//设置ADC
					*strout = '\0';
					if(InputPanel(strout,5,&strlen)) {
						val[index] = atof_(strout);
					}
					g_power.adc = val[index];
					
				}
				else if(index == 1) {//设置DAC
					*strout = '\0';
					if(InputPanel(strout,5,&strlen)) {
						val[index] = atoi(strout);
					}
					g_power.dac = val[index];
					DAC_SetChannel1Data(DAC_Align_12b_R, (uint16_t)(val[index]));    //  （dacData/4096）*3.3== OUT
					DAC_SoftwareTriggerCmd(DAC_Channel_1,ENABLE);  
				}
				else if(index == 2) {//后台
					*strout = '\0';
					if(InputPanel(strout,7,&strlen)) {
						dbm = atof_(strout);
					}
					g_power.lastset = g_power.set;
					g_power.set = (int32_t)(dbm*1000);
					Ctrl_Power(&g_power);
				}
				else if(index == 3) {//
					UI_ProductionAdjust();
					
				}
				else if(index == 4) {//
					*strout = '\0';
// 					if(InputPanel(strout,7,&strlen)) {
// 						mode = atoi(strout);
// 					}
					mode++;
					if(mode > 3) {
						mode = 0;
					}
					Operating_Mode = mode ;
				}
				else if(index == 5) {//
					*strout = '\0';
// 					if(InputPanel(strout,7,&strlen)) {
// 						laser = atoi(strout);
// 					}
					laser++;
					if(laser > 4) {
						laser = 0;
					}
					Wavelength_Selection_state = laser;
				}
				else if(index == 6) {//
					enableAuto = !enableAuto;
				}
				else if(index == 7) {//
					UI_ProductConfig();
				}
				else if(index == 8) {//exit
					goto _End;
				}
				flag = 0;
				btnClk = 0;
				goto _Redraw;
			}
			
			
			else if(btnClk && index == 9) {
				break;
			}
			else if(btnClk && index == 8) {
				//todo:写入FLASH
				btnClk = 0;
			}
			
			DrawFocus(loc[oldindex].x,loc[oldindex].y,COL_White);
			//RedrawRect(loc[oldindex].x-2,loc[oldindex].y,2,8);
			DrawFocus(loc[index].x,loc[index].y,COL_Black);
			//RedrawRect(loc[index].x-2,loc[index].y,2,8);
			oldindex = index;
			
			flag = 0;
			btnClk = 0;
		
		}
	}
_End:;
	gl_ui_setfontcolor(oldfont);
	gl_ui_setpencolor(oldpen);
	gl_ui_setbrushcolor(oldbrush);
	gl_ui_setbkcolor(oldbk);
}

//产品配置
void UI_ProductConfig()
{
	POINT stloc[] = {//静态文本位置
		4,2,     4,14,    4,26,//   4,38,  4,50,    4,62,     
		66,14,   66,26,   66,38,  66,50,    66,62,
		156,50,  156,62};
	POINT loc[] = {//游标位置
		34,2,    
		4,38,    4,50,    4,62,
		34,26,   34,38,  34,50,    34,62,     //Logo /*66 14*/
		104,26,   104,38,  104,50,    104,62,
		156,50,  156,62};
	int8_t *caption[] = {//静态文本内容
		"SN","Laser","650",//"1310","1490","1550",
		"LOGO","Addr","Color","High","Width",
		"Save","Exit"};
	struct adj_power_flash tmpFlash;
	
	
	int8_t enableAuto = 0;
	int16_t oldpen,oldbrush,oldbk,oldfont;	
	int32_t index = 0,oldindex = 0,i;
	int32_t strlen,loclen;
	uint8_t flag = 0;
	uint8_t btnClk = 0;
	char strout[50];
	uint32_t retInt;
	float dbm,battery;
	uint8_t mode = 0,laser = 1;
	int val[5] = {0};
	uint16_t ad;

	loclen = sizeof(loc)/sizeof(POINT);
	oldfont = gl_ui_setfontcolor(COL_Black);
	oldpen = gl_ui_setpencolor(COL_Black);
	oldbrush =  gl_ui_setbrushcolor(COL_White);
	oldbk = gl_ui_setbkcolor(COL_White);
	
	tmpFlash = g_adj_power;
_Redraw:;
	gl_clear(0,0,320,240,COL_White);
	
	//静态文本
 	for(i = 0;i < 10;++i) {
 		gl_text(stloc[i].x,stloc[i].y,caption[i],-1);
 	}
// 	for(i = 0;i < 3;++i) {
// 		gl_text(stloc[i].x,stloc[i].y,caption[i],-1);
// 	}

	//sn
	gl_text(loc[0].x,loc[0].y,tmpFlash.sn,-1);
	
	//Laser
	sprintf(strout,"%d",tmpFlash._ch1wave);
	gl_text(loc[1].x,loc[1].y,strout,-1);
	sprintf(strout,"%d",tmpFlash._ch2wave);
	gl_text(loc[2].x,loc[2].y,strout,-1);
	sprintf(strout,"%d",tmpFlash._ch3wave);
	gl_text(loc[3].x,loc[3].y,strout,-1);
	
	
	
	if(!tmpFlash._650_en) {
		gl_text(loc[4].x,loc[4].y,"off",-1);
	}
	else {
		gl_text(loc[4].x,loc[4].y,"on ",-1);
	}
	
	
	if(!tmpFlash._1310_en) {
		gl_text(loc[5].x,loc[5].y,"off",-1);
	}
	else {
		gl_text(loc[5].x,loc[5].y,"on ",-1);
	}
	if(!tmpFlash._1490_en) {
		gl_text(loc[6].x,loc[6].y,"off",-1);
	}
	else {
		gl_text(loc[6].x,loc[6].y,"on ",-1);
	}
	if(!tmpFlash._1550_en) {
		gl_text(loc[7].x,loc[7].y,"off",-1);
	}
	else {
		gl_text(loc[7].x,loc[7].y,"on ",-1);
	}
	
	//Logo
	gl_text(66,14,"LOGO",-1);
	sprintf(strout,"0x%X",tmpFlash._logo_addr);
	gl_text(loc[8].x,loc[8].y,strout,-1);

	sprintf(strout,"0x%X",tmpFlash._logo_backcolor);
	gl_text(loc[9].x,loc[9].y,strout,-1);
	
	sprintf(strout,"%d",tmpFlash._logo_h);
	gl_text(loc[10].x,loc[10].y,strout,-1);

	sprintf(strout,"%d",tmpFlash._logo_w);
	gl_text(loc[11].x,loc[11].y,strout,-1);

	flag = 1;
	while(1) {
		//移动游标
		if(KeyPress(KEY_PORT_A,KEY_A)) {
			oldindex = index;
			index--;
			if(index < 0) {
				index = loclen-1;
			}
			DrawFocus(loc[oldindex].x,loc[oldindex].y,COL_White);
			DrawFocus(loc[index].x,loc[index].y,COL_Black);
			//goto _Redraw;
		}
		else if(KeyPress(KEY_PORT_C,KEY_C)) {
			oldindex = index;
			index++;
			if(index >= loclen) {
				index = 0;
			}
			DrawFocus(loc[oldindex].x,loc[oldindex].y,COL_White);
			DrawFocus(loc[index].x,loc[index].y,COL_Black);
			//goto _Redraw;
		}
		
		
		//按下确定键
		else if(KeyPress(KEY_PORT_Z,KEY_Z)) {
			flag = 1;
			btnClk = 1;
		}
		
		//
		if(flag) {
			

			if(btnClk) {
				switch(index) {
				case 0://sn
					strout[0] = '\0';					
					if(InputPanel(strout,25,&strlen)) {
						for(i = 0;i < 25;++i) {
							tmpFlash.sn[i] = strout[i];
						}
					}
					gl_text(loc[0].x,loc[0].y,"                             ",-1);
					gl_text(loc[0].x,loc[0].y,tmpFlash.sn,-1);
					break;
				case 1://_ch1wave
					strout[0] = '\0';					
					if(InputPanel(strout,25,&strlen)) {
						tmpFlash._ch1wave = atoi(strout);
					}
					gl_text(loc[1].x,loc[1].y,"    ",-1);
					gl_text(loc[1].x,loc[1].y,strout,-1);
					break;
				case 2://_ch2wave
					strout[0] = '\0';					
					if(InputPanel(strout,25,&strlen)) {
						tmpFlash._ch2wave = atoi(strout);
					}
					gl_text(loc[2].x,loc[2].y,"    ",-1);
					gl_text(loc[2].x,loc[2].y,strout,-1);
					break;
				case 3://_ch3wave
					strout[0] = '\0';					
					if(InputPanel(strout,25,&strlen)) {
						tmpFlash._ch3wave = atoi(strout);
					}
					gl_text(loc[3].x,loc[3].y,"    ",-1);
					gl_text(loc[3].x,loc[3].y,strout,-1);
					break;
					
					
				case 4://650
					if(tmpFlash._650_en) {
						gl_text(loc[4].x,loc[4].y,"off",-1);
					}
					else {
						gl_text(loc[4].x,loc[4].y,"on ",-1);
					}
					tmpFlash._650_en = !tmpFlash._650_en;
					break;
				case 5://1310
					if(tmpFlash._1310_en) {
						gl_text(loc[5].x,loc[5].y,"off",-1);
					}
					else {
						gl_text(loc[5].x,loc[5].y,"on ",-1);
					}
					tmpFlash._1310_en = !tmpFlash._1310_en;
					break;
				case 6://1490
					if(tmpFlash._1490_en) {
						gl_text(loc[6].x,loc[6].y,"off",-1);
					}
					else {
						gl_text(loc[6].x,loc[6].y,"on ",-1);
					}
					tmpFlash._1490_en = !tmpFlash._1490_en;
					break;
				case 7://1550
					if(tmpFlash._1550_en) {
						gl_text(loc[7].x,loc[7].y,"off",-1);
					}
					else {
						gl_text(loc[7].x,loc[7].y,"on ",-1);
					}
					tmpFlash._1550_en = !tmpFlash._1550_en;
					break;
					
					
					
					
				case 8://Logo Addr
					strout[0] = '\0';					
					if(InputPanel(strout,9,&strlen)) {
						tmpFlash._logo_addr = atof_(strout);
					}
					gl_text(loc[8].x,loc[8].y,"         ",-1);
					sprintf(strout,"0x%X",tmpFlash._logo_addr);
					gl_text(loc[8].x,loc[8].y,strout,-1);
					break;
				case 9://Logo black color
					strout[0] = '\0';					
					if(InputPanel(strout,8,&strlen)) {
						tmpFlash._logo_backcolor = atof_(strout);
					}
					gl_text(loc[9].x,loc[9].y,"      ",-1);
					sprintf(strout,"0x%X",tmpFlash._logo_backcolor);
					gl_text(loc[9].x,loc[9].y,strout,-1);
					break;
				case 10://Logo high
					strout[0] = '\0';					
					if(InputPanel(strout,4,&strlen)) {
						tmpFlash._logo_h = atof_(strout);
					}
					gl_text(loc[10].x,loc[10].y,"    ",-1);
					sprintf(strout,"%d",tmpFlash._logo_h);
					gl_text(loc[10].x,loc[10].y,strout,-1);
					break;
				case 11://Logo width
						strout[0] = '\0';					
					if(InputPanel(strout,4,&strlen)) {
						tmpFlash._logo_w = atof_(strout);
					}
					gl_text(loc[11].x,loc[11].y,"    ",-1);
					sprintf(strout,"%d",tmpFlash._logo_w);
					gl_text(loc[11].x,loc[11].y,strout,-1);
					break;
				
				
				case 12://Save
					g_adj_power = tmpFlash;
					g_adj_power.flag = 0xaabbccdd;
					WriteFlash(FLASH_PAGE_START,
						(uint32_t*)&(g_adj_power),
						sizeof(struct adj_power_flash));
					gl_text(loc[12].x,loc[12].y,"Save...",-1);
					Delay_ms(1000);
					gl_text(loc[12].x,loc[12].y,"       ",-1);
					gl_text(loc[12].x,loc[12].y,"Save",-1);
					break;
				case 13://Exit
					goto _End;
					break;
				}
				flag = 0;
				btnClk = 0;
				goto _Redraw;
			}
			
			
			DrawFocus(loc[oldindex].x,loc[oldindex].y,COL_White);
			//RedrawRect(loc[oldindex].x-2,loc[oldindex].y,2,8);
			DrawFocus(loc[index].x,loc[index].y,COL_Black);
			//RedrawRect(loc[index].x-2,loc[index].y,2,8);
			oldindex = index;
			
			flag = 0;
			btnClk = 0;
		
		}
	}
_End:;
	gl_ui_setfontcolor(oldfont);
	gl_ui_setpencolor(oldpen);
	gl_ui_setbrushcolor(oldbrush);
	gl_ui_setbkcolor(oldbk);
}
//注意其对应声明头文件包含在main.c文件中
//main文件中包含图像数据头文件 TEMP.h 
//LCD_FLSAH_DrawPicture( 0, 0, 319, 239, (uint8_t *)&PIC_TEMP );

uint32_t InputPanel(int8_t *str,uint32_t len,uint32_t *outLen)
{
	//char tmpstr[30] = "";
	uint32_t ret;
	RECT rc[] = {
		{127,158+55,34,26,},	//	 "0",
		{93,132+55,34,26,},	//	 "1",
		{127,132+55,34,26,},	//	 "2",
		{161,132+55,34+3,26,},	//	 "3",
		{93,106+55,34,26,},	//	 "4",
		{127,106+55,34,26,},	//	 "5",
		{161,106+55,34+3,26,},	//	 "6",
		{93,80+55,34,26,},	//	 "7",
		{127,80+55,34,26,},	//	 "8",
		{161,80+55,34+3,26,},	//	 "9",

		{71-2,80+55,22+2,26,},	//	 "+",        10
		{71-2,106+55,22+2,26,},	//	 "-",
		{71-2,132+55,22+2,26,},	//	 "*",
		{71-2,158+55,22+2,26,},	//	 "/",

		{195+3,80+55,23+18,26},	//	 "back", 14
		
		{195+3,106+55,23+18,26,},	//	 ".",    15
		{195+3,132+55,23+18,26,},	//	 "Esc",    16
		{195+3,158+55,23+18,26,},	//	 "enter",17
		{161,158+55,34+3,26,},	//	 "space",18
		{93,158+55,34,26,},	//	 "tab",  19
		};
 	uint32_t idMatrix[] = {
 		10,7,8,9,  14,
		11,4,5,6,  15,
		12,1,2,3,  16,
		13,19,0,18,17,
 	};
	int8_t *caption[] = {
		"nop","  7","  8","  9"," Back",
		" -","  4","  5","  6"," . ",
		"nop","  1","  2","  3"," Esc",
		"nop","Turn","  0","Space","Enter"};
	int8_t *addStr[] = {
		"","7","8","9","",
		"-","4","5","6",".",
		"","1","2","3","",
		"","","0","",""};
	RECT rcShow = {71-2,135-15,142+28,15};
	int i;
	int16_t oldpen,oldbrush,oldbk,oldfont;
	int32_t index = 14,oldindex = 14;
	uint8_t flag = 0;
	uint8_t btnClk = 0;
	int32_t strIndex = 0;
	int8_t *ptmp;
		
	oldfont = gl_ui_setfontcolor(RGB16(65,92,252));
	oldpen = gl_ui_setpencolor(RGB16(0xff,0,0));
	oldbrush =  gl_ui_setbrushcolor(RGB16(215,215,215));
	oldbk = gl_ui_setbkcolor(RGB16(215,215,215));
	for(i = 0;i < 20;++i) {
		gl_fill_rect(
			rc[idMatrix[i]].left,rc[idMatrix[i]].top,
			rc[idMatrix[i]].right,rc[idMatrix[i]].bottom);
		gl_text(rc[idMatrix[i]].left+2,rc[idMatrix[i]].top+6,caption[i],-1);
	}
	gl_fill_rect(
		rcShow.left,rcShow.top,rcShow.right,rcShow.bottom);
	
	while(KeyPress(GPIOA,KEY_Z));
	flag = 1;
	while(1) {
		if(ProTick1963IsLive())
			;
		TurnOffPower();
		
		if(KeyPress(GPIOA,KEY_Y)) {
			flag =  1;
			if(index > 4) {
				oldindex = index;
				index-= 5;
			}
		}
		else if(KeyPress(GPIOA,KEY_Z)) {
			flag =  1;
			btnClk = 1;
		}
		else if(KeyPress(GPIOA,KEY_A)) {
			flag =  1;
			oldindex = index;
			index-= 1;
		}
		else if(KeyPress(GPIOA,KEY_B)) {
			flag =  1;
			if(index < 15) {
				oldindex = index;
				index+= 5;
			}
		}
		if(KeyPress(GPIOA,KEY_C)) {
			flag =  1;
			oldindex = index;
			index+= 1;
		}
		if(flag) {
			
			if(index <  0) {
				index = 0;
			}
			else if(index > 19) {
				index = 19;
			}

			gl_ui_setfontcolor(RGB16(65,92,252));
			gl_text(rc[idMatrix[oldindex]].left+2,rc[idMatrix[oldindex]].top+6,caption[oldindex],-1);			
				
			gl_ui_setfontcolor(RGB16(0xff,0,0));
			gl_text(rc[idMatrix[index]].left+2,rc[idMatrix[index]].top+6,caption[index],-1);			
			
			
			if(btnClk) {
				if(strIndex+1 < len &&//缓存容量必须够大
					idMatrix[index] <= 9 || idMatrix[index] == 11 || idMatrix[index] == 15) {
					//*(str+strIndex) = *addStr[index];
					//*(str+strIndex) = addStr[index];
					ptmp = addStr[index];
					
					//tmpstr[strIndex] =  *addStr[index];//*ptmp;//idMatrix[index] + '0';
					//*(tmpstr+strIndex+1) = '\0';
					str[strIndex] = *ptmp;
					str[strIndex+1] = '\0';
					
					strIndex++;
				}
				else if(idMatrix[index] == 14) {
					*(str+strIndex-1) = '\0';
					//*(tmpstr+strIndex-1) = '\0';
					strIndex--;
					if(strIndex < 0)
						strIndex = 0;
				}
				else if(idMatrix[index] == 16) {
					ret = 0;
					break;
				}
				else if(idMatrix[index] == 17) {
					ret = 1;
					break;
				}
				//gl_text(0,0,"abc",-1);
// 				sprintf(str,"index = %d",strIndex);
// 				gl_text(10,0,str,-1);
// 				sprintf(str,"id = %d",idMatrix[index]);
// 				gl_text(10,10,str,-1);
				gl_fill_rect(
					rcShow.left,rcShow.top,rcShow.right,rcShow.bottom);
				if(strIndex >= 25) {
					//gl_text(rcShow.left,rcShow.top,str-strIndex,10);
					//gl_text(rcShow.left+1,rcShow.top+1,tmpstr,10);
					gl_text(rcShow.left+1,rcShow.top+1,str+(strIndex-24),24);
				}
				else {
					//gl_text(rcShow.left,rcShow.top,str,10);
					//gl_text(rcShow.left+1,rcShow.top+1,tmpstr,-1);
					gl_text(rcShow.left+1,rcShow.top+1,str,-1);
				}
				//gl_text(0,0,tmpstr,-1);
				//gl_text(0,0,addStr[index],-1);
			}
				
			
			
			flag = 0;
			btnClk  = 0;
		}
	}
	while(KeyPress(GPIOA,KEY_Z));
	gl_fill_rect(71-2,135-15,142+28,120);
	
	gl_ui_setfontcolor(oldfont);
	gl_ui_setpencolor(oldpen);
	gl_ui_setbrushcolor(oldbrush);
	gl_ui_setbkcolor(oldbk);
	*outLen = strIndex;
	return ret;
}

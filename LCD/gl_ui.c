
/**
 ******************************************************************************
* @file    gl_ui.c
* @author  MenglongWu
* @version V1.0
* @date    2012-10-17
* @brief   作者：吴梦龙\n
底层GUI接口具备可移植性，实现最基本的绘点功能，不带缓存，更多功能有待完善
 ******************************************************************************
 * @attention
 *
 * ATTENTION
*
* <h2><center>&copy; COPYRIGHT </center></h2>
******************************************************************************
*/

#include "gl_type.h"
#include "gl_ui_config.h"
#include "gl_ui.h"


#pragma diag_suppress 550
//内部全局变量
static void (*_setpoint)(unsigned short x, unsigned short y,uint32_t  color);
static unsigned short _fcolor = 0x0000;		//字体颜色
static unsigned short _bkcolor = 0xffff;	//背景颜色
//static unsigned short _alpha = 100;		//透明度
static unsigned char  _bkmode = TRANSPARENT;//背景填充模式
static unsigned short _pencolor = 0x0000;	//笔颜色
static unsigned short _brushcolor = 0x0000;		//画刷颜色
static unsigned char *_zm_ascii[128] = {0};		//字库
static struct gl_ui_arraylib _zm_lib;		//作废了

/**
* @brief	设置绘点函
* @param	setpoint 回调函数
*		回调函数，需要底层硬件接口符合回调函数的参数传递规则。
*		由于以前写的函数接口不一定是按照此规则编写的，
*		当使用宏GL_UI_OLD_DEV的时候，就用GL_SETPOINT代替回调函数。
* @retval\n	NULL
* @remarks
*/
void gl_ui_hook(
	void (*setpoint)(unsigned short x, unsigned short y,uint32_t  color))
{
	_setpoint = setpoint;
}

/**
* @brief	设置背景填充模式
* @param	bkmode 可以是TRANSPARENT或BACKFILL
* @retval\n	上一次的填充模式
* @remarks
*/
uint32_t gl_ui_setbkmode(uint32_t bkmode)
{
	unsigned short old = bkmode;
	_bkmode = bkmode;
	return old;
}


/**
* @brief	设置文字颜色
* @param	color 根据设备选择RGB()或RGB16()
* @retval\n	上一次的设置值
* @remarks
*/
uint32_t gl_ui_setfontcolor(uint32_t  color)
{
	unsigned short old = _fcolor;
	_fcolor = color;
	return old;
}

/**
* @brief	设置背景色
* @param	color 根据设备选择RGB()或RGB16()
* @retval\n	上一次的设置值
* @remarks
*/
uint32_t gl_ui_setbkcolor(uint32_t  color)
{
	unsigned short old = _bkcolor;
	_bkcolor = color;
	return old;
}


/**
* @brief	设置画笔颜色
* @param	color 根据设备选择RGB()或RGB16()
* @retval\n	上一次的设置值
* @remarks
*/
uint32_t gl_ui_setpencolor(uint32_t  color)
{
	unsigned short old = _pencolor;
	_pencolor = color;
	return old;
}

/**
* @brief	设置画刷颜色
* @param	color 根据设备选择RGB()或RGB16()
* @retval\n	上一次的设置值
* @remarks
*/
uint32_t gl_ui_setbrushcolor(uint32_t  color)
{
	unsigned short old = _bkcolor;
	_brushcolor = color;
	return old;
}


/**
* @brief	设置字库
* @param	lib 查阅相应的p_zm_ascii_xxx
* @param	w 查阅相应的ZM_xxx_W
* @param	h 查阅相应的ZM_xxx_H
* @param	lib 查阅相应的p_zm_step_xxx
* @retval\n	NULL
* @remarks
*/
unsigned char gl_ui_setlib(unsigned char *lib[128],uint16_t w,uint16_t h,uint8_t *step)
//struct gl_ui_arraylib gl_ui_setlib(struct gl_ui_arraylib *lib)
{
	uint16_t i;
// 	unsigned char old,*news;// = p_zm_ascii
// 	unsigned short i;
 	for(i = 0;i < 128;++i)
 		//_zm_ascii[i] = p_zm_ascii[i];
 		_zm_ascii[i] = lib[i];
// 	return old;
	//struct gl_ui_arraylib old;
	//struct gl_ui_arraylib old;

	for(i = 0;i < 128;++i) {
		_zm_lib.lib[i] = lib[i];
		_zm_lib.step[i] = *step++;
	}
	
	_zm_lib.w = w;
	_zm_lib.h = h;
	
	
	return 0;
	
}

/**
* @brief	基本绘点函数
* @param	x 绘制的位置x
* @param	y 绘制的位置y
* @param	color 绘点颜色,对于只有黑白的显示器，颜色只能是-1（白色）和0（黑色）
			相应的gl_ui_setfontcol,gl_ui_setbkcolor等函数的color也是这样的设定
* @retval\n	NULL
* @remarks\n 应用层永远不会直接访问此函数，底层调用
*/
void gl_setpoint(unsigned short x, unsigned short y,uint32_t  color)
{
#ifdef GL_UI_OLD_DEV
	GL_SETPOINT(x,y,color);
#else
	_setpoint(x,y,color);
#endif
}

/**
* @brief	绘制水平线
* @param	x1 水平线起点x坐标
* @param	x2 水平线终点x坐标
* @param	y 水平线y坐标
*/
void gl_horizon_line(uint16_t x1,uint16_t x2,uint16_t y)
{
	uint16_t i;
	//for(i=x1; i<=x2; i++)
		//gl_setpoint(i,y,_pencolor);
	gl_setarea(x1,y,x2,y);
	for(i=x1; i<=x2; i++)
		gl_setpoint(i,y,_pencolor);
}

/*
* Function: 
* Parameters:
* Return:
* Remarks:
*/
/**
* @brief	绘制垂直线
* @param	x 水平线x坐标
* @param	y1 水平线起点y坐标
* @param	y2 水平线终点y坐标
* @retval\n	NULL
* @remarks
*/
void gl_vertical_line(uint16_t x,uint16_t y1,uint16_t y2)
{
	uint16_t i;
// 	for(i=y1; i<=y2; i++)
// 		gl_setpoint(x,i,_pencolor);
	gl_setarea(x,y1,x,y2);
 	for(i=y1; i<=y2; i++)
 		gl_setpoint(x,i,_pencolor);
}


/**
* @brief	绘制点阵
* @param	buf 点阵缓存
* @param	px 点阵需要绘制的起点坐标x
* @param	py 点阵需要绘制的起点坐标y
* @param	w 点阵宽度
* @param	h 点阵高度
* @retval\n	NULL
* @remarks\n 矩阵点只能是0和1，1处使用_pencolor绘制，背景色用根据透明度情况不绘制
			或使用背景画刷填充，背景模式通过调用gl_ui_setbkmode设置
*/
void gl_picture(uint8_t *buf,uint16_t px,uint16_t py,uint16_t w,uint16_t h)
{
	uint16_t x,y;
	uint8_t *tbuf = buf;
	w = w + px;
	h = h + py;
	
	
	if(_bkmode == TRANSPARENT) {
		for(y = py;y < h;++y) {
			//gl_setarea(px,y,px+w-1,y);
			for(x = px;x < w;) {	
				gl_setarea(x,y,x,y);
				if(*tbuf & 0x80) gl_setpoint(x,y, _pencolor);
				x++;
				
				gl_setarea(x,y,x,y);
				if(*tbuf & 0x40) gl_setpoint(x,y, _pencolor);
				x++;
				
				gl_setarea(x,y,x,y);
				if(*tbuf & 0x20) gl_setpoint(x,y, _pencolor);
				x++;
				
				gl_setarea(x,y,x,y);
				if(*tbuf & 0x10) gl_setpoint(x,y, _pencolor);
				x++;
				
				gl_setarea(x,y,x,y);
				if(*tbuf & 0x08) gl_setpoint(x,y, _pencolor);
				x++;
				
				gl_setarea(x,y,x,y);
				if(*tbuf & 0x04) gl_setpoint(x,y, _pencolor);
				x++;
				
				gl_setarea(x,y,x,y);
				if(*tbuf & 0x02) gl_setpoint(x,y, _pencolor);
				x++;
				
				gl_setarea(x,y,x,y);
				if(*tbuf & 0x01) gl_setpoint(x,y, _pencolor);
				x++;
				tbuf++;
			}
		}
	}
	else {
		gl_setarea(px,py,w-1,h-1);
		for(y = py;y < h;++y) {
			//gl_setarea(px,y,px+w-1,y);
			for(x = px;x < w;) {	
				gl_setpoint(x++,y,*tbuf & 0x80 ? _pencolor : _bkcolor);
				gl_setpoint(x++,y,*tbuf & 0x40 ? _pencolor : _bkcolor);
				gl_setpoint(x++,y,*tbuf & 0x20 ? _pencolor : _bkcolor);
				gl_setpoint(x++,y,*tbuf & 0x10 ? _pencolor : _bkcolor);
				gl_setpoint(x++,y,*tbuf & 0x08 ? _pencolor : _bkcolor);
				gl_setpoint(x++,y,*tbuf & 0x04 ? _pencolor : _bkcolor);
				gl_setpoint(x++,y,*tbuf & 0x02 ? _pencolor : _bkcolor);
				gl_setpoint(x++,y,*tbuf & 0x01 ? _pencolor : _bkcolor);
				tbuf++;
			}
		}
	}
}


/**
* @brief	输出ascii文字
* @param	x 文字在屏幕输出坐标x
* @param	y 文字在屏幕输出坐标y
* @param	str 文字内容
* @param	num 从str里显示多少个byte，如果num等于-1，将在检测到str里字符为'\0'时
*			停止，所以str字符串必须以'\0'结束，否则将导致不可预见后果
* @retval\n	NULL
* @remarks\n 在掉用此函数前必须先调用gl_ui_setlib设置字库。
	为了精简代码存储量，ascii字库也不是每个都写在zimo_xxx.c里的，
	根据情况自己删减zimo_xxx.c的内容，具体方法看提供的手册
*/
void gl_text(uint16_t x,uint16_t y,uint8_t *str,uint16_t num)
{
	uint16_t i = 0;
	uint8_t *tstr = str;
	uint32_t old;
	
	i = 0;
	old = gl_ui_setpencolor(_fcolor);
	while(*tstr != '\0' && i++ < num) {
		//gl_picture((uint8_t*)p_zm_ascii[*tstr++],x,y,ZM_WIDTH,ZM_HIGH);
		gl_picture(_zm_ascii[*tstr],x,y,_zm_lib.w,_zm_lib.h);
		//gl_picture(_zm_lib.lib[*tstr],x,y,_zm_lib.w,_zm_lib.h);
		x += _zm_lib.step[*tstr];
		tstr++;
	}
	gl_ui_setpencolor(old );
}


/**
* @brief	绘制一个有填充色的矩形
* @param	x 矩形坐标x
* @param	y 矩形坐标y
* @param	w 矩形宽度
* @param	h 矩形高度
* @retval\n	NULL
* @remarks\n 线条色是_pencolor，用gl_ui_setpencolor设置，
*			填充色是_brushcolor，用gl_ui_setbrushcolor设置
*/
void gl_fill_rect(
	uint16_t x,uint16_t y,
	uint16_t w,uint16_t h)
{
	int tx,ty;
	
	gl_horizon_line (x    ,x+w-1,y    );
	gl_horizon_line (x    ,x+w-1,y+h-1);
	gl_vertical_line(x    ,y+1  ,y+h-2);
	gl_vertical_line(x+w-1,y+1  ,y+h-2);
	
	
// 	for(ty = y+1;ty < y+h-1;ty++) {
// 		for(tx = x+1;tx < x+w-1;tx++) {
// 			gl_setpoint(tx    ,ty,_brushcolor);		
// 		}
// 	}
	//gl_setarea(x+1,y+1,x+w-1,y+h-1);
	gl_setarea(x+1,y+1,x+w-2,y+h-2);
	for(ty = y+1;ty < y+h-1;ty++) {
		//gl_setarea(x+1,ty,x+w-2,ty);
		for(tx = x+1;tx < x+w-1;tx++) {
			gl_setpoint(tx    ,ty,_brushcolor);		
		}
	}
}

/**
* @brief	绘制一个空心矩形
* @param	x 矩形坐标x
* @param	y 矩形坐标y
* @param	w 矩形宽度
* @param	h 矩形高度
*/
void gl_rect(
	uint16_t x,uint16_t y,
	uint16_t w,uint16_t h)
{
	gl_horizon_line (x    ,x+w-1,y    );
	gl_horizon_line (x    ,x+w-1,y+h-1);
	gl_vertical_line(x    ,y+1  ,y+h-2);
	gl_vertical_line(x+w-1,y+1  ,y+h-2);
}


/**
* @brief	用特定颜色清除某区域
* @param	x1 清除起点坐标x
* @param	y1 清除起点坐标y
* @param	x2 清除终点坐标x
* @param	y2 清除终点坐标y
* @param	color 清除后用指定颜色填充
*/
void gl_clear(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2,uint32_t color)
{
	uint32_t i,t;
	LCD_SetArea(x1,y1,x2,y2);
	LCD_WriteCommand(CMD_WR_MEMSTART);
	
	t = (x2-x1)*(y2-y1);
	for(i = 0;i < t;i++)
		LCD_WriteData(color);
}


/**
* @brief	设置液晶屏内存自加区域
* @param	x1 内存自加区域起点坐标x
* @param	y1 内存自加区域起点坐标y
* @param	x2 内存自加区域终点坐标x
* @param	y2内存自加区域终点坐标y
* @retval\n	NULL
* @remarks\n 该函数提供给具备内存自加功能的LCD屏，需要有针对的移植
*/
void gl_setarea(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
{
	LCD_SetArea(x1,y1,x2,y2);
	LCD_WriteCommand(CMD_WR_MEMSTART);
}

/**
* @brief	显示一副图片
* @param	StartX 图片起点坐标x
* @param	StartY 图片起点坐标y
* @param	EndX   图片终点坐标x
* @param	EndY   图片终点坐标y
* @param	pic    图片内容
* remark\n 该函数只支持640x320一下分辨率的图片显示，并且图片每像素点占用位16bit
*/
void gl_bmp(uint16_t StartX,uint16_t StartY,uint16_t EndX,uint16_t EndY, uint8_t * pic)
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
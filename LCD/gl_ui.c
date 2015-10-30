
/**
 ******************************************************************************
* @file    gl_ui.c
* @author  MenglongWu
* @version V1.0
* @date    2012-10-17
* @brief   ���ߣ�������\n
�ײ�GUI�ӿھ߱�����ֲ�ԣ�ʵ��������Ļ�㹦�ܣ��������棬���๦���д�����
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
//�ڲ�ȫ�ֱ���
static void (*_setpoint)(unsigned short x, unsigned short y,uint32_t  color);
static unsigned short _fcolor = 0x0000;		//������ɫ
static unsigned short _bkcolor = 0xffff;	//������ɫ
//static unsigned short _alpha = 100;		//͸����
static unsigned char  _bkmode = TRANSPARENT;//�������ģʽ
static unsigned short _pencolor = 0x0000;	//����ɫ
static unsigned short _brushcolor = 0x0000;		//��ˢ��ɫ
static unsigned char *_zm_ascii[128] = {0};		//�ֿ�
static struct gl_ui_arraylib _zm_lib;		//������

/**
* @brief	���û�㺯
* @param	setpoint �ص�����
*		�ص���������Ҫ�ײ�Ӳ���ӿڷ��ϻص������Ĳ������ݹ���
*		������ǰд�ĺ����ӿڲ�һ���ǰ��մ˹����д�ģ�
*		��ʹ�ú�GL_UI_OLD_DEV��ʱ�򣬾���GL_SETPOINT����ص�������
* @retval\n	NULL
* @remarks
*/
void gl_ui_hook(
	void (*setpoint)(unsigned short x, unsigned short y,uint32_t  color))
{
	_setpoint = setpoint;
}

/**
* @brief	���ñ������ģʽ
* @param	bkmode ������TRANSPARENT��BACKFILL
* @retval\n	��һ�ε����ģʽ
* @remarks
*/
uint32_t gl_ui_setbkmode(uint32_t bkmode)
{
	unsigned short old = bkmode;
	_bkmode = bkmode;
	return old;
}


/**
* @brief	����������ɫ
* @param	color �����豸ѡ��RGB()��RGB16()
* @retval\n	��һ�ε�����ֵ
* @remarks
*/
uint32_t gl_ui_setfontcolor(uint32_t  color)
{
	unsigned short old = _fcolor;
	_fcolor = color;
	return old;
}

/**
* @brief	���ñ���ɫ
* @param	color �����豸ѡ��RGB()��RGB16()
* @retval\n	��һ�ε�����ֵ
* @remarks
*/
uint32_t gl_ui_setbkcolor(uint32_t  color)
{
	unsigned short old = _bkcolor;
	_bkcolor = color;
	return old;
}


/**
* @brief	���û�����ɫ
* @param	color �����豸ѡ��RGB()��RGB16()
* @retval\n	��һ�ε�����ֵ
* @remarks
*/
uint32_t gl_ui_setpencolor(uint32_t  color)
{
	unsigned short old = _pencolor;
	_pencolor = color;
	return old;
}

/**
* @brief	���û�ˢ��ɫ
* @param	color �����豸ѡ��RGB()��RGB16()
* @retval\n	��һ�ε�����ֵ
* @remarks
*/
uint32_t gl_ui_setbrushcolor(uint32_t  color)
{
	unsigned short old = _bkcolor;
	_brushcolor = color;
	return old;
}


/**
* @brief	�����ֿ�
* @param	lib ������Ӧ��p_zm_ascii_xxx
* @param	w ������Ӧ��ZM_xxx_W
* @param	h ������Ӧ��ZM_xxx_H
* @param	lib ������Ӧ��p_zm_step_xxx
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
* @brief	������㺯��
* @param	x ���Ƶ�λ��x
* @param	y ���Ƶ�λ��y
* @param	color �����ɫ,����ֻ�кڰ׵���ʾ������ɫֻ����-1����ɫ����0����ɫ��
			��Ӧ��gl_ui_setfontcol,gl_ui_setbkcolor�Ⱥ�����colorҲ���������趨
* @retval\n	NULL
* @remarks\n Ӧ�ò���Զ����ֱ�ӷ��ʴ˺������ײ����
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
* @brief	����ˮƽ��
* @param	x1 ˮƽ�����x����
* @param	x2 ˮƽ���յ�x����
* @param	y ˮƽ��y����
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
* @brief	���ƴ�ֱ��
* @param	x ˮƽ��x����
* @param	y1 ˮƽ�����y����
* @param	y2 ˮƽ���յ�y����
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
* @brief	���Ƶ���
* @param	buf ���󻺴�
* @param	px ������Ҫ���Ƶ��������x
* @param	py ������Ҫ���Ƶ��������y
* @param	w ������
* @param	h ����߶�
* @retval\n	NULL
* @remarks\n �����ֻ����0��1��1��ʹ��_pencolor���ƣ�����ɫ�ø���͸�������������
			��ʹ�ñ�����ˢ��䣬����ģʽͨ������gl_ui_setbkmode����
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
* @brief	���ascii����
* @param	x ��������Ļ�������x
* @param	y ��������Ļ�������y
* @param	str ��������
* @param	num ��str����ʾ���ٸ�byte�����num����-1�����ڼ�⵽str���ַ�Ϊ'\0'ʱ
*			ֹͣ������str�ַ���������'\0'���������򽫵��²���Ԥ�����
* @retval\n	NULL
* @remarks\n �ڵ��ô˺���ǰ�����ȵ���gl_ui_setlib�����ֿ⡣
	Ϊ�˾������洢����ascii�ֿ�Ҳ����ÿ����д��zimo_xxx.c��ģ�
	��������Լ�ɾ��zimo_xxx.c�����ݣ����巽�����ṩ���ֲ�
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
* @brief	����һ�������ɫ�ľ���
* @param	x ��������x
* @param	y ��������y
* @param	w ���ο��
* @param	h ���θ߶�
* @retval\n	NULL
* @remarks\n ����ɫ��_pencolor����gl_ui_setpencolor���ã�
*			���ɫ��_brushcolor����gl_ui_setbrushcolor����
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
* @brief	����һ�����ľ���
* @param	x ��������x
* @param	y ��������y
* @param	w ���ο��
* @param	h ���θ߶�
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
* @brief	���ض���ɫ���ĳ����
* @param	x1 ����������x
* @param	y1 ����������y
* @param	x2 ����յ�����x
* @param	y2 ����յ�����y
* @param	color �������ָ����ɫ���
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
* @brief	����Һ�����ڴ��Լ�����
* @param	x1 �ڴ��Լ������������x
* @param	y1 �ڴ��Լ������������y
* @param	x2 �ڴ��Լ������յ�����x
* @param	y2�ڴ��Լ������յ�����y
* @retval\n	NULL
* @remarks\n �ú����ṩ���߱��ڴ��Լӹ��ܵ�LCD������Ҫ����Ե���ֲ
*/
void gl_setarea(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
{
	LCD_SetArea(x1,y1,x2,y2);
	LCD_WriteCommand(CMD_WR_MEMSTART);
}

/**
* @brief	��ʾһ��ͼƬ
* @param	StartX ͼƬ�������x
* @param	StartY ͼƬ�������y
* @param	EndX   ͼƬ�յ�����x
* @param	EndY   ͼƬ�յ�����y
* @param	pic    ͼƬ����
* remark\n �ú���ֻ֧��640x320һ�·ֱ��ʵ�ͼƬ��ʾ������ͼƬÿ���ص�ռ��λ16bit
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
#ifndef _GL_UI_H_
#define _GL_UI_H_


#define TRANSPARENT 0	//±³¾°Í¸Ã÷
#define BACKFILL 1		//ÓÃ±³¾°Ìî³ä
/*
#define RGB(r,g,b) ((uint32_t)(r << 16 | g << 8 | b))
#define RGB16(r,g,b) ((uint16_t)((r & 0xF8)<< 8 | (g & 0xFC)<< 3 | (b & 0xF8) >> 3))
*/
#define RGB8(r,g,b)		// todo
#define RGB565(r,g,b)		((unsigned short) ( ((r) >> 3) << 11 ) | ( ((g) >> 2) << 5) | ( ((b) >> 3) << 0) )
#define RGB24(r,g,b)		 	((unsigned long) ( (r) << 16 )        | ( (g) << 8)        | ( (b) << 0) )

#define CONFIG_LCD_BPP  (24)
// Define RGB macro
#if CONFIG_LCD_BPP == 24
#define RGB(r,g,b)			RGB24((r),(g),(b))

#elif CONFIG_LCD_BPP == 16
#define RGB(r,g,b)			RGB565((r),(g),(b))

#elif CONFIG_LCD_BPP == 8
#define RGB(r,g,b)			RGB8((r),(g),(b))

#else
#warning "CONFIG_LCD_BPP only with 8/16/24 check out lcdconf.h"
#define RGB(r,g,b) RGB24((r),(g),(b))
#endif

#define RGB16(r,g,b) RGB565((r),(g),(b))

struct gl_ui_arraylib
{
	uint16_t w;
	uint16_t h;
	uint16_t step[128];
	uint8_t *lib[128];
	
	//uint8_t *(*lib[128]);
};
void gl_ui_hook(
	void (*setpoint)(unsigned short x, unsigned short y,uint32_t color));
void gl_setpoint(unsigned short x, unsigned short y,uint32_t color);
uint32_t gl_ui_setbkmode(uint32_t bkmode);
uint32_t gl_ui_setfontcolor(uint32_t color);
uint32_t gl_ui_setbkcolor(uint32_t color);
uint32_t gl_ui_setpencolor(uint32_t  color);
uint32_t gl_ui_setbrushcolor(uint32_t  color);
unsigned char gl_ui_setlib(unsigned char *lib[128],uint16_t w,uint16_t h,uint8_t *step);
//unsigned char gl_ui_setlib(unsigned char *lib[128],uint16_t w,uint16_t h,uint16_t step);
//unsigned char gl_ui_setlib(unsigned char *(*lib[128]),uint16_t w,uint16_t h);
//unsigned char gl_ui_setlib(unsigned char *lib[128]);
//struct gl_ui_arraylib gl_ui_setlib(struct gl_ui_arraylib *lib);

void gl_horizon_line(uint16_t x1,uint16_t x2,uint16_t y);//,uint16_t color);
void gl_vertical_line(uint16_t x,uint16_t y1,uint16_t y2);//,uint16_t color);
void gl_picture(uint8_t *buf,uint16_t px,uint16_t py,uint16_t w,uint16_t h);
void gl_text(uint16_t x,uint16_t y,uint8_t *str,uint16_t num);//,uint16_t color,uint16_t bkcolor);
void gl_rect(
	uint16_t x,uint16_t y,
	uint16_t w,uint16_t h);//,
	//uint16_t linecolor);
void gl_fill_rect(
	uint16_t x,uint16_t y,
	uint16_t w,uint16_t h);//,
	//uint16_t linecolor,uint16_t fillcolor);
	
void gl_clear(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2,uint32_t color);
void gl_setarea(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);
void gl_bmp(uint16_t StartX,uint16_t StartY,uint16_t EndX,uint16_t EndY, uint8_t * pic);
#endif


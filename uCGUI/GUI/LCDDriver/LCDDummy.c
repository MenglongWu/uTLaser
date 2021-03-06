/*
*********************************************************************************************************
*                                                uC/GUI
*                        Universal graphic software for embedded applications
*
*                       (c) Copyright 2002, Micrium Inc., Weston, FL
*                       (c) Copyright 2002, SEGGER Microcontroller Systeme GmbH
*
*              µC/GUI is protected by international copyright laws. Knowledge of the
*              source code may not be used to write a similar product. This file may
*              only be used in accordance with a license and should not be redistributed
*              in any way. We appreciate your understanding and fairness.
*
----------------------------------------------------------------------
File        : LCDDummy.C
Purpose     : Empty driver
              This driver does no perform any function, but it can be
              used for 2 purposes:
              a) Satisfy all externals so an application can be
                compiled and linked in target hardware even if the
                driver is not already available
              b) Template for a starting point for a new driver.
----------------------------------------------------------------------   
Adapting to a new system (creating a new driver):
  In this case, the first step is to fill the routines 
  LCD_L0_GetPixelIndex, LCD_L0_SetPixelIndex and LCD_L0_Init with
  functionality, which is sufficient to make the hardware work.
  A second (optional) step would be to optimize higher level routines. 
----------------------------------------------------------------------   
Version-Date---Author-Explanation                                        
----------------------------------------------------------------------   
1.00.00 020417 JE     a) Changed to have only to adapt _GetPixelIndex
                         and _SetPixelIndex
0.90.00 020214 JE     a) First release






- 2015-11-05,Menglong Woo, MenglongWoo@aliyun.com
  - SSD1963 图像处理器特质驱动优化部分包括
      - LCD_Area
      - _DrawBitLine1BPP
        - 普通模式
        - 透明模式
        - 不包括异或模式LCD_DRAWMODE_XOR
      - 
      - _DrawBitLine2BPP
        - 普通模式
        - 透明模式
      - LCD_L0_SetPixelIndex
      - LCD_L0_GetPixelIndex
      - LCD_L0_XorPixel
      - LCD_L0_DrawHLine
      - LCD_L0_DrawVLine
      - LCD_L0_FillRect
---------------------------END-OF-HEADER------------------------------
*/

#include "LCD_Private.h"      /* private modul definitions & config */
#include "GUI_Private.h"
#include "GUIDebug.h"

// Menglong Woo
#include "./USER/GLCD/glcd.h"
#include "LCD/gl_ui.h"
#if (LCD_CONTROLLER == -1) \
    && (!defined(WIN32) | defined(LCD_SIMCONTROLLER))

/*********************************************************************
*
*       Defines
*
**********************************************************************
*/

#ifndef LCD_INIT_CONTROLLER
  #define LCD_INIT_CONTROLLER()
#endif

/*********************************************************************
*
*       Macros for MIRROR_, SWAP_ and LUT_
*/
#if (!defined (LCD_LUT_COM) && !defined(LCD_LUT_SEG))
  #if   (!LCD_MIRROR_X && !LCD_MIRROR_Y && !LCD_SWAP_XY) 
    #define LOG2PHYS_X(x, y) x
    #define LOG2PHYS_Y(x, y) y
  #elif (!LCD_MIRROR_X && !LCD_MIRROR_Y &&  LCD_SWAP_XY) 
    #define LOG2PHYS_X(x, y) y
    #define LOG2PHYS_Y(x, y) x
  #elif (!LCD_MIRROR_X &&  LCD_MIRROR_Y && !LCD_SWAP_XY) 
    #define LOG2PHYS_X(x, y) x
    #define LOG2PHYS_Y(x, y) LCD_YSIZE - 1 - (y)
  #elif (!LCD_MIRROR_X &&  LCD_MIRROR_Y &&  LCD_SWAP_XY) 
    #define LOG2PHYS_X(x, y) y
    #define LOG2PHYS_Y(x, y) LCD_XSIZE - 1 - (x)
  #elif ( LCD_MIRROR_X && !LCD_MIRROR_Y && !LCD_SWAP_XY) 
    #define LOG2PHYS_X(x, y) LCD_XSIZE - 1 - (x)
    #define LOG2PHYS_Y(x, y) y
  #elif ( LCD_MIRROR_X && !LCD_MIRROR_Y &&  LCD_SWAP_XY) 
    #define LOG2PHYS_X(x, y) LCD_YSIZE - 1 - (y)
    #define LOG2PHYS_Y(x, y) x
  #elif ( LCD_MIRROR_X &&  LCD_MIRROR_Y && !LCD_SWAP_XY) 
    #define LOG2PHYS_X(x, y) LCD_XSIZE - 1 - (x)
    #define LOG2PHYS_Y(x, y) LCD_YSIZE - 1 - (y)
  #elif ( LCD_MIRROR_X &&  LCD_MIRROR_Y &&  LCD_SWAP_XY) 
    #define LOG2PHYS_X(x, y) LCD_YSIZE - 1 - (y)
    #define LOG2PHYS_Y(x, y) LCD_XSIZE - 1 - (x)
  #endif
#else
  #if   ( defined (LCD_LUT_COM) && !defined(LCD_LUT_SEG))
    #define LOG2PHYS_X(x, y) x
    #define LOG2PHYS_Y(x, y) LCD__aLine2Com0[y]
  #elif (!defined (LCD_LUT_COM) &&  defined(LCD_LUT_SEG))
    #define LOG2PHYS_X(x, y) LCD__aCol2Seg0[x]
    #define LOG2PHYS_Y(x, y) y
  #elif ( defined (LCD_LUT_COM) &&  defined(LCD_LUT_SEG))
    #define LOG2PHYS_X(x, y) LCD__aCol2Seg0[x]
    #define LOG2PHYS_Y(x, y) LCD__aLine2Com0[y]
  #endif
#endif

/*********************************************************************
*
*       Static functions
*
**********************************************************************
*/
static __inline void LCD_Area(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1)
{
  LCD_SetArea(x0,y0,x1,y1);
  LCD_WriteCommand(CMD_WR_MEMSTART);
}
/*********************************************************************
*
*       Draw Bitmap 1 BPP
*/
static void  _DrawBitLine1BPP(int x, int y, U8 const GUI_UNI_PTR *p, int Diff, int xsize, const LCD_PIXELINDEX*pTrans) {
  LCD_PIXELINDEX Index0 = *(pTrans+0);
  LCD_PIXELINDEX Index1 = *(pTrans+1);
  x += Diff;
  
  // printf("start %d %d %d %d\n", x, y, Diff,xsize);
  switch (GUI_Context.DrawMode & (LCD_DRAWMODE_TRANS | LCD_DRAWMODE_XOR)) {
    case 0:
      LCD_Area(x,y,xsize,y);
      do {
        LCD_WriteData((*p & (0x80 >> Diff)) ? Index1 : Index0);
        if (++Diff == 8) {
          Diff = 0;
          p++;
        }
      } while (--xsize);
      break;
    case LCD_DRAWMODE_TRANS:
      {
        int k = 1;
        do {
          
          if (*p & (0x80 >> Diff)) {
            if (k) {
              LCD_Area(x,y,x + xsize,y);
              k = 0;
            }
            LCD_WriteData(Index1);
          }
          else {
            k++;
          }
          x++;
          if (++Diff == 8) {
            Diff = 0;
            p++;
          }
        } while (--xsize);
      }

      // do {
      //   if (*p & (0x80 >> Diff)) {
      //     LCD_L0_SetPixelIndex(x, y, Index1);
      //   }
      //   x++;
      //   if (++Diff == 8) {
      //     Diff = 0;
      //     p++;
      //   }
      // } while (--xsize);
      break;
    case LCD_DRAWMODE_XOR:;
      do {
        if (*p & (0x80 >> Diff)) {
          int Pixel = LCD_L0_GetPixelIndex(x, y);
          LCD_L0_SetPixelIndex(x, y, LCD_NUM_COLORS - 1 - Pixel);
        }
        x++;
        if (++Diff == 8) {
          Diff = 0;
          p++;
        }
      } while (--xsize);
      break;
  }
  // printf("end %d %d %d %d\n", x, y, Diff,xsize);
}

/*********************************************************************
*
*       Draw Bitmap 2 BPP
*/
#if (LCD_MAX_LOG_COLORS > 2)
static void  _DrawBitLine2BPP(int x, int y, U8 const GUI_UNI_PTR * p, int Diff, int xsize, const LCD_PIXELINDEX * pTrans) {
  LCD_PIXELINDEX Pixels = *p;
  int CurrentPixel = Diff;
  x += Diff;
  switch (GUI_Context.DrawMode & (LCD_DRAWMODE_TRANS | LCD_DRAWMODE_XOR)) {
    case 0:
      if (pTrans) {
        LCD_Area(x,y,xsize*4,y);
        do {
          int Shift = (3 - CurrentPixel) << 1;
          int Index = (Pixels & (0xC0 >> (6 - Shift))) >> Shift;
          LCD_PIXELINDEX PixelIndex = *(pTrans + Index);
          // LCD_L0_SetPixelIndex(x++, y, PixelIndex);
          LCD_WriteData(PixelIndex);
          if (++CurrentPixel == 4) {
            CurrentPixel = 0;
            Pixels = *(++p);
          }
        } while (--xsize);
      } else {
        LCD_Area(x,y,xsize*4,y);
        do {
          int Shift = (3 - CurrentPixel) << 1;
          int Index = (Pixels & (0xC0 >> (6 - Shift))) >> Shift;
          // LCD_L0_SetPixelIndex(x++, y, Index);
          LCD_WriteData(Index);
          if (++CurrentPixel == 4) {
            CurrentPixel = 0;
            Pixels = *(++p);
          }
        } while (--xsize);
      }
      break;
    case LCD_DRAWMODE_TRANS:
      if (pTrans) {
        int k = 1;
        do {
          int Shift = (3 - CurrentPixel) << 1;
          int Index = (Pixels & (0xC0 >> (6 - Shift))) >> Shift;
          if (Index) {
            LCD_PIXELINDEX PixelIndex = *(pTrans + Index);
            // LCD_L0_SetPixelIndex(x, y, PixelIndex);
            if (k) {
              LCD_Area(x,y,(x + xsize)*2,y);
              k = 0;
            }
            LCD_WriteData(PixelIndex);
          }
           else {
              k++;
          }
          x++;
          if (++CurrentPixel == 4) {
            CurrentPixel = 0;
            Pixels = *(++p);
          }
        } while (--xsize);
      } else {
        int k = 1;
        do {
          int Shift = (3 - CurrentPixel) << 1;
          int Index = (Pixels & (0xC0 >> (6 - Shift))) >> Shift;
          if (Index) {
            // LCD_L0_SetPixelIndex(x, y, Index);
            if (k) {
              LCD_Area(x,y,(x + xsize)*2,y);
              k = 0;
            }
            LCD_WriteData(Index);
          }
          else {
              k++;
          }
          x++;
          if (++CurrentPixel == 4) {
            CurrentPixel = 0;
            Pixels = *(++p);
          }
        } while (--xsize);
      }
        break;
  }
}

#endif

/*********************************************************************
*
*       Draw Bitmap 4 BPP
*/
#if (LCD_MAX_LOG_COLORS > 4)
static void  _DrawBitLine4BPP(int x, int y, U8 const GUI_UNI_PTR * p, int Diff, int xsize, const LCD_PIXELINDEX * pTrans) {
  LCD_PIXELINDEX Pixels = *p;
  int CurrentPixel = Diff;
  x += Diff;
  switch (GUI_Context.DrawMode & (LCD_DRAWMODE_TRANS | LCD_DRAWMODE_XOR)) {
    case 0:
      if (pTrans) {
        do {
          int Shift = (1 - CurrentPixel) << 2;
          int Index = (Pixels & (0xF0 >> (4 - Shift))) >> Shift;
          LCD_PIXELINDEX PixelIndex = *(pTrans + Index);
          LCD_L0_SetPixelIndex(x++, y, PixelIndex);
          if (++CurrentPixel == 2) {
            CurrentPixel = 0;
            Pixels = *(++p);
          }
		    } while (--xsize);
      } else {
        do {
          int Shift = (1 - CurrentPixel) << 2;
          int Index = (Pixels & (0xF0 >> (4 - Shift))) >> Shift;
          LCD_L0_SetPixelIndex(x++, y, Index);
          if (++CurrentPixel == 2) {
            CurrentPixel = 0;
            Pixels = *(++p);
          }
		    } while (--xsize);
      }
      break;
    case LCD_DRAWMODE_TRANS:
      if (pTrans) {
        do {
          int Shift = (1 - CurrentPixel) << 2;
          int Index = (Pixels & (0xF0 >> (4 - Shift))) >> Shift;
          if (Index) {
            LCD_PIXELINDEX PixelIndex = *(pTrans + Index);
            LCD_L0_SetPixelIndex(x, y, PixelIndex);
          }
          x++;
          if (++CurrentPixel == 2) {
            CurrentPixel = 0;
            Pixels = *(++p);
          }
		    } while (--xsize);
      } else {
        do {
          int Shift = (1 - CurrentPixel) << 2;
          int Index = (Pixels & (0xF0 >> (4 - Shift))) >> Shift;
          if (Index) {
            LCD_L0_SetPixelIndex(x, y, Index);
          }
          x++;
          if (++CurrentPixel == 2) {
            CurrentPixel = 0;
            Pixels = *(++p);
          }
		    } while (--xsize);
      }
      break;
  }
}
#endif

/*********************************************************************
*
*       Draw Bitmap 8 BPP
*/
#if (LCD_MAX_LOG_COLORS > 16)
static void  _DrawBitLine8BPP(int x, int y, U8 const GUI_UNI_PTR * p, int xsize, const LCD_PIXELINDEX * pTrans) {
  LCD_PIXELINDEX Pixel;
  switch (GUI_Context.DrawMode & (LCD_DRAWMODE_TRANS | LCD_DRAWMODE_XOR)) {
    case 0:
      if (pTrans) {
        for (; xsize > 0; xsize--, x++, p++) {
          Pixel = *p;
          LCD_L0_SetPixelIndex(x, y, *(pTrans + Pixel));
        }
      } else {
        for (; xsize > 0; xsize--, x++, p++) {
          LCD_L0_SetPixelIndex(x, y, *p);
        }
      }
      break;
    case LCD_DRAWMODE_TRANS:
      if (pTrans) {
        for (; xsize > 0; xsize--, x++, p++) {
          Pixel = *p;
          if (Pixel) {
            LCD_L0_SetPixelIndex(x, y, *(pTrans + Pixel));
          }
        }
      } else {
        for (; xsize > 0; xsize--, x++, p++) {
          Pixel = *p;
          if (Pixel) {
            LCD_L0_SetPixelIndex(x, y, Pixel);
          }
        }
      }
      break;
  }
}
#endif

/*********************************************************************
*
*       Draw Bitmap 16 BPP
*/
#if (LCD_BITSPERPIXEL > 8)
static void  DrawBitLine16BPP(int x, int y, U16 const GUI_UNI_PTR * p, int xsize, const LCD_PIXELINDEX * pTrans) {
  LCD_PIXELINDEX pixel;
  if ((GUI_Context.DrawMode & LCD_DRAWMODE_TRANS) == 0) {
    if (pTrans) {
      for (; xsize > 0; xsize--, x++, p++) {
        pixel = *p;
        LCD_L0_SetPixelIndex(x, y, *(pTrans + pixel));
      }
    } else {
      for (;xsize > 0; xsize--, x++, p++) {
        LCD_L0_SetPixelIndex(x, y, *p);
      }
    }
  } else {
    if (pTrans) {
      for (; xsize > 0; xsize--, x++, p++) {
        pixel = *p;
        if (pixel) {
          LCD_L0_SetPixelIndex(x, y, *(pTrans + pixel));
        }
      }
    } else {
      for (; xsize > 0; xsize--, x++, p++) {
        pixel = *p;
        if (pixel) {
          LCD_L0_SetPixelIndex(x, y, pixel);
        }
      }
    }
  }
}
#endif

/*********************************************************************
*
*       Exported functions
*
**********************************************************************
*/

/*********************************************************************
*
*       LCD_L0_SetPixelIndex
*
* Purpose:
*   Sets the index of the given pixel. The upper layers
*   calling this routine make sure that the coordinates are in range, so
*   that no check on the parameters needs to be performed.
*/
void LCD_L0_SetPixelIndex(int x, int y, int PixelIndex) {
  /* Convert logical into physical coordinates (Dep. on LCDConf.h) */
  #if LCD_SWAP_XY | LCD_MIRROR_X| LCD_MIRROR_Y
    int xPhys = LOG2PHYS_X(x, y);
    int yPhys = LOG2PHYS_Y(x, y);
  #else
    #define xPhys x
    #define yPhys y
  #endif
  /* Write into hardware ... Adapt to your system */
  {
    /* ... */
  // gl_setpoint(x,y,PixelIndex);
  	LCD_Area(x,y,x+1,y+1);  
    LCD_WriteData(PixelIndex);
  }
}

/*********************************************************************
*
*       LCD_L0_GetPixelIndex
*
* Purpose:
*   Returns the index of the given pixel. The upper layers
*   calling this routine make sure that the coordinates are in range, so
*   that no check on the parameters needs to be performed.
*/
unsigned int LCD_L0_GetPixelIndex(int x, int y) {
  LCD_PIXELINDEX PixelIndex;
  /* Convert logical into physical coordinates (Dep. on LCDConf.h) */
  #if LCD_SWAP_XY | LCD_MIRROR_X| LCD_MIRROR_Y
    int xPhys = LOG2PHYS_X(x, y);
    int yPhys = LOG2PHYS_Y(x, y);
  #else
    #define xPhys x
    #define yPhys y
  #endif
  /* Read from hardware ... Adapt to your system */
  {
    // PixelIndex = 0;/* ... */
    PixelIndex = LCD_GetPoint(x,y);
    // PixelIndex = 0;
  }
  return PixelIndex;
}

/*********************************************************************
*
*       LCD_L0_XorPixel
*/
void LCD_L0_XorPixel(int x, int y) {
  LCD_PIXELINDEX PixelIndex = LCD_L0_GetPixelIndex(x, y);
  LCD_L0_SetPixelIndex(x, y, LCD_NUM_COLORS - PixelIndex - 1);
}

/*********************************************************************
*
*       LCD_L0_DrawHLine
*/
void LCD_L0_DrawHLine  (int x0, int y,  int x1) {
  if (GUI_Context.DrawMode & LCD_DRAWMODE_XOR) {
    for (; x0 <= x1; x0++) {
      LCD_L0_XorPixel(x0, y);
    }
  } else {
    // for (; x0 <= x1; x0++) {
    //   LCD_L0_SetPixelIndex(x0, y, LCD_COLORINDEX);
    // }

    LCD_Area(x0, y, x1, y);
    
    
    for (; x0 <= x1; x0++) {
        LCD_WriteData(LCD_COLORINDEX);
    }
  }
}

/*********************************************************************
*
*       LCD_L0_DrawVLine
*/
void LCD_L0_DrawVLine  (int x, int y0,  int y1) {
  if (GUI_Context.DrawMode & LCD_DRAWMODE_XOR) {
    for (; y0 <= y1; y0++) {
      LCD_L0_XorPixel(x, y0);
    }
  } else {
    // for (; y0 <= y1; y0++) {
    //   LCD_L0_SetPixelIndex(x, y0, LCD_COLORINDEX);
    // }
    LCD_Area(x, y0, x, y1);
    
    
    for (; y0 <= y1; y0++) {
        LCD_WriteData(LCD_COLORINDEX);
    }
  }
}

/*********************************************************************
*
*       LCD_L0_FillRect
*/
void LCD_L0_FillRect(int x0, int y0, int x1, int y1) {
  // ver 1
  // for (; y0 <= y1; y0++) {
  //   LCD_L0_DrawHLine(x0, y0, x1);
  // }
  // return ;

  // ver 5
  int t;
  LCD_Area(x0, y0, x1, y1);
  
  
  t = ((x1 - x0) +1 ) * ((y1 -y0) +1);
  for ( ; t--; ) {
      LCD_WriteData(LCD_COLORINDEX);
  }
}

/*********************************************************************
*
*       LCD_L0_DrawBitmap
*/
void LCD_L0_DrawBitmap(int x0, int y0,
                       int xsize, int ysize,
                       int BitsPerPixel, 
                       int BytesPerLine,
                       const U8 GUI_UNI_PTR * pData, int Diff,
                       const LCD_PIXELINDEX* pTrans)
{
  int i;
//   printf("x0 %d\ny0 %d\nxsize %d\nysize %d\nBitsPerPixel %d\nBytesPerLine %d\nDiff %d \n",
// x0,
// y0,
// xsize,
// ysize,
// BitsPerPixel,
// BytesPerLine,
// Diff);
  /* Use _DrawBitLineXBPP */
  for (i=0; i<ysize; i++) {
    switch (BitsPerPixel) {
    case 1:
      _DrawBitLine1BPP(x0, i + y0, pData, Diff, xsize, pTrans);
      break;
    #if (LCD_MAX_LOG_COLORS > 2)
      case 2:
        _DrawBitLine2BPP(x0, i + y0, pData, Diff, xsize, pTrans);
        break;
    #endif
    #if (LCD_MAX_LOG_COLORS > 4)
      case 4:
        _DrawBitLine4BPP(x0, i + y0, pData, Diff, xsize, pTrans);
        break;
    #endif
    #if (LCD_MAX_LOG_COLORS > 16)
      case 8:
        _DrawBitLine8BPP(x0, i + y0, pData, xsize, pTrans);
        break;
    #endif
    #if (LCD_BITSPERPIXEL > 8)
      case 16:
        DrawBitLine16BPP(x0, i + y0, (const U16 *)pData, xsize, pTrans);
        break;
    #endif
    }
    pData += BytesPerLine;
  }
}

/*********************************************************************
*
*       LCD_L0_SetOrg
*/
void LCD_L0_SetOrg(int x, int y) {
  GUI_USE_PARA(x);
  GUI_USE_PARA(y);
}

/*********************************************************************
*
*       LCD_On / LCD_Off
*/
void LCD_On (void) {
#ifdef LCD_ON
  LCD_ON();
#endif
}

void LCD_Off (void) {
#ifdef LCD_OFF
  LCD_OFF();
#endif
}

/*********************************************************************
*
*       LCD_L0_Init
*
* Purpose:
*   Initialises the LCD-controller.
*/
int  LCD_L0_Init(void) {
  //LCD_INIT_CONTROLLER();
  return 0;
}

/*********************************************************************
*
*       LCD_L0_SetLUTEntry
*/
void LCD_L0_SetLUTEntry(U8 Pos, LCD_COLOR Color) {
  GUI_USE_PARA(Pos);
  GUI_USE_PARA(Color);
}

#else

void LCDDummy_c(void);
void LCDDummy_c(void) { } /* avoid empty object files */

#endif /* (LCD_CONTROLLER undefined) */


void GUI_X_Init(void)
{

}

/**** ExecIdle - called if nothing else is left to do ****/
void GUI_X_ExecIdle(void)
{

}

/**** Timing routines - required for blinking ****/
unsigned long g_ucgui_time_ms;
int  GUI_X_GetTime(void)
{
	return g_ucgui_time_ms;
}
void GUI_X_Delay(int Period)
{

}

/**** Multitask routines - required only if multitasking is used (#define GUI_OS 1) ****/
static int lock = 0;
void GUI_X_Unlock(void)
{
  lock = 0;
}
void GUI_X_Lock(void)
{
  while(lock == 1);
  lock = 1;
}
U32  GUI_X_GetTaskId(void)
{
	return 0;
}
void GUI_X_InitOS(void)
{

}

/****      Event driving (optional with multitasking)  ****/
void GUI_X_WaitEvent(void)
{

}
void GUI_X_SignalEvent(void)
{

}
/**** Recording (logs/warnings and errors) - required only for higher levels ****/
void GUI_X_Log(const char *s)
{

}
void GUI_X_Warn(const char *s)
{

}
void GUI_X_ErrorOut(const char *s)
{

}

void exit(int i)
{

}

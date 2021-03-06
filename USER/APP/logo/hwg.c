/*********************************************************************
*                SEGGER MICROCONTROLLER SYSTEME GmbH                 *
*        Solutions for real time microcontroller applications        *
*                           www.segger.com                           *
**********************************************************************
*
* C-file generated by
*
*        �C/GUI-BitmapConvert V3.90.
*        Compiled Aug 19 2004, 09:07:56
*          (c) 2002  Micrium, Inc.
  www.micrium.com

  (c) 1998-2002  Segger
  Microcontroller Systeme GmbH
  www.segger.com
*
**********************************************************************
*
* Source file: hwg
* Dimensions:  55 * 18
* NumColors:   2
*
**********************************************************************
*/

#include "stdlib.h"

#include "GUI.h"

#ifndef GUI_CONST_STORAGE
  #define GUI_CONST_STORAGE const
#endif

/*   Palette
The following are the entries of the palette table.
Every entry is a 32-bit value (of which 24 bits are actually used)
the lower   8 bits represent the Red component,
the middle  8 bits represent the Green component,
the highest 8 bits (of the 24 bits used) represent the Blue component
as follows:   0xBBGGRR
*/

static GUI_CONST_STORAGE GUI_COLOR Colorshwg[] = {
     0x000000,0xFFFFFF
};

static GUI_CONST_STORAGE GUI_LOGPALETTE Palhwg = {
  2,	/* number of entries */
  0, 	/* No transparency */
  &Colorshwg[0]
};

 GUI_CONST_STORAGE unsigned char achwg[] = {
  ________, ________, ________, ________, ________, ________, ________,
  _______X, XXXXXXXX, ________, __XXXXXX, XX____XX, ___X__XX, ________,
  _XXXX__X, X__X_X_X, ____XXXX, __X___X_, _X_____X, X_XX__X_, ________,
  _XX_X__X, XX_X_XXX, ____X__X, __X___X_, _X______, XXX__XXX, XXXXXX__,
  _XX_X__X, _XXX_X_X, ____X__X, __XXXXXX, XX______, XX___X__, ____XX__,
  _XX_X__X, _X_XX__X, ____X__X, __X___X_, _X_____X, XXX_X___, ____XX__,
  _XX_X__X, XXXXXXXX, ____X__X, __X___X_, _X____X_, __XX__XX, XX__X___,
  _XX_X___, ___X____, ____X__X, __XXXXXX, XX______, __X___X_, _X__X___,
  _XX_X___, ___X____, ____X__X, ________, ________, _XX___X_, _X__X___,
  _XX_X__X, XXXXXXXX, ____X__X, ________, ________, XXX___X_, _X__X___,
  _XXXX___, ___X____, ____X__X, XXXXXXXX, XXX_____, X_X___X_, _X__X___,
  _XX___XX, XXXXXXXX, X___XXXX, __X__XX_, _______X, __X___XX, XX__X___,
  _______X, ____X__X, ____X__X, __X___X_, _X____X_, __X___X_, _X__X___,
  ______XX, _X__X__X, ________, __X___XX, X_______, __X_____, ____X___,
  ______X_, _XX__X__, X_______, __XXXX_X, X_______, __X_____, ____X___,
  _____XX_, __X__X__, X_______, __XXX___, XX_____X, XXX_____, _XXX____,
  _____X__, __X__X__, ________, __X_____, __XX____, ________, ________,
  ________, ________, ________, ________, ________, ________, ________
};

GUI_CONST_STORAGE GUI_BITMAP bmhwg = {
  55, /* XSize */
  18, /* YSize */
  7, /* BytesPerLine */
  1, /* BitsPerPixel */
  achwg,  /* Pointer to picture data (indices) */
  &Palhwg  /* Pointer to palette */
};

/* *** End of file *** */

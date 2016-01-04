/*********************************************************************
*       SEGGER MICROCONTROLLER SYSTEME GmbH        *
*   Solutions for real time microcontroller applications    *
*                                 *
*         emWin GSC sample code              *
*                                 *
**********************************************************************

----------------------------------------------------------------------
File    : WIDGET_ListBoxOwnerDraw.c
Purpose  : Demonstrates a owner drawn list box
----------------------------------------------------------------------
*/

#include <stddef.h>
#include "GUI.h"
#include "DIALOG.h"
#include "wm.h"
#include "project.h"
#include "USER/TouchPanel/touch.h"
#include "prj_type.h"
#include "flash.h"


static FRAMEWIN_Handle hFrame = 0;
static BUTTON_Handle hButton1;
static BUTTON_Handle hButton2;
static WM_HWIN this = 0;
static WM_HWIN hMain = 0;

#define GUI_ID_RAW (GUI_ID_BUTTON0+0)
#define GUI_ID_LOGIC (GUI_ID_BUTTON0+1)
#define GUI_ID_LEFT (GUI_ID_BUTTON0+2)
#define GUI_ID_RIGHT (GUI_ID_BUTTON0+3)
// #define GUI_ID_PWM_REVERSAL (GUI_ID_BUTTON0+3)
// #define GUI_ID_PWM_WIDTH (GUI_ID_BUTTON0+4)
// #define GUI_ID_LASTER (GUI_ID_BUTTON0+5)
// #define GUI_ID_SETTING (GUI_ID_BUTTON0+6)


static const GUI_WIDGET_CREATE_INFO _aDialogCreate[] = {
  { FRAMEWIN_CreateIndirect,  "", 0,          0,  220, 320, 40 , FRAMEWIN_CF_MOVEABLE },
     // { LISTBOX_CreateIndirect,  0,            GUI_ID_MULTIEDIT0, 10, 10, 100, 100, 0, 100 },
  // /* Check box for multi select mode */
  //   { CHECKBOX_CreateIndirect, 0,             GUI_ID_CHECK0,  120, 10,  0,  0 },
  //   { TEXT_CreateIndirect,   "Multi select",     0,         140, 10, 80, 15, TEXT_CF_LEFT },
  // /* Check box for owner drawn list box */
  //   { CHECKBOX_CreateIndirect, 0,             GUI_ID_CHECK1,  120, 35,  0,  0 },
  //   { TEXT_CreateIndirect,   "Owner drawn",        0,        140,  35, 80, 15, TEXT_CF_LEFT },
  // /* Buttons */
  //   { BUTTON_CreateIndirect, "OK",           GUI_ID_OK,     120, 65, 80, 20 },
  //   { BUTTON_CreateIndirect, "Cancel",         GUI_ID_CANCEL,   120, 90, 80, 20 },
  
  {BUTTON_CreateIndirect,  "<",           GUI_ID_LEFT,  0,0,40,20},
  {BUTTON_CreateIndirect,  ">",           GUI_ID_RIGHT,  320-40,0,40,20},

  // {BUTTON_CreateIndirect,   "PWM\r\nNormal",     GUI_ID_PWM_REVERSAL,   162,85,66,66},
  // {BUTTON_CreateIndirect,   "1us",           GUI_ID_PWM_WIDTH,  232,85,66,66},
  // {BUTTON_CreateIndirect,   "Laster",            GUI_ID_LASTER,   22,155,66,66},
  // {BUTTON_CreateIndirect,   "Set",           GUI_ID_SETTING,  95,155,66,66},
  // {BUTTON_CreateIndirect,   "2",           GUI_ID_BUTTON0+7,  162,155,136,66},
};


static int move = 0;
extern struct wm_glide glide;
static void OnLeftClick(WM_MESSAGE * pMsg)
{
  GUI_RECT rect;

  WM_GetWindowRectEx(hMain, &rect);

  if (glide.en == 1) {
    return ;
  }
  if (rect.x1 <= 200) {
    return ;
  }
  
  // if (move < -100) {
  //   return ;
  // }
  // move -= 100;
  glide.s_x = rect.x0;
  glide.s_y = rect.y0;
  glide.d1_x = -10;
  glide.d1_y = 0;
  glide.d1_loop = 8;
  glide.d2_x = -2;
  glide.d2_y = 0;
  glide.d2_loop = 10;

  glide.e_x = rect.x0 + glide.d1_x*glide.d1_loop + glide.d2_x * glide.d2_loop;
  glide.e_y = rect.y0 + glide.d1_y*glide.d1_loop + glide.d2_y * glide.d2_loop;
  // glide.d1_x = -100;
  // glide.d1_loop = 1;
  // glide.d2_loop = 0;
  
  glide.hWin = hMain;
  glide.en = 1;
  // WM_MoveTo(hMain, move,0);
  // printf("kdjflsjldjfsf");

}

static void OnRightClick(WM_MESSAGE * pMsg)
{
  GUI_RECT rect;

  WM_GetWindowRectEx(hMain, &rect);
  if (glide.en == 1) {
    return ;
  }
  printf("rect.x0 %d\n", rect.x0);
  if (rect.x0 > 0) {
    return ;
  }
  // if (move >= 0) {
  //   return ;
  // }
  // move += 100;
  glide.s_x = rect.x0;
  glide.s_y = rect.y0;

  glide.d1_x = 10;
  glide.d1_y = 0;
  glide.d1_loop = 8;
  glide.d2_x = 2;
  glide.d2_y = 0;
  glide.d2_loop = 10;


  glide.e_x = rect.x0 + glide.d1_x*glide.d1_loop + glide.d2_x * glide.d2_loop;
  glide.e_y = rect.y0 + glide.d1_y*glide.d1_loop + glide.d2_y * glide.d2_loop;
  // glide.d1_x = 100;
  // glide.d1_loop = 1;
  // glide.d2_loop = 0;

  glide.hWin = hMain;
  glide.en = 1;
  // WM_MoveTo(hMain, move,0);

}
static void Init_Ctrl(WM_MESSAGE * pMsg)
{
  WM_HWIN hDlg,hButton;
  WM_MESSAGE msg;

  hDlg = pMsg->hWin;
  hButton = WM_GetDialogItem(hDlg, GUI_ID_LEFT);
  BUTTON_SetTextColor(hButton, 0, COL_DISABLE);
  BUTTON_SetBkColor(hButton, 1, COL_DIALOG_BK);
  BUTTON_SetBkColor(hButton, 0, COL_DIALOG_BK);

  hButton = WM_GetDialogItem(hDlg, GUI_ID_RIGHT);
  BUTTON_SetBkColor(hButton, 1, COL_DIALOG_BK);
  BUTTON_SetBkColor(hButton, 0, COL_DIALOG_BK);
  BUTTON_SetTextColor(hButton, 0, COL_DISABLE);
}
/*********************************************************************
*
*    _cbCallback
*/
static void _cbCallback(WM_MESSAGE * pMsg) {
  static char strout[40];
  struct point touch;
  int i;
  int NCode, Id;
  WM_HWIN hWin;


  // printf("%x %x %x\n", pMsg->MsgId, pMsg->hWin,pMsg->hWinSrc);
  switch (pMsg->MsgId) {
  case WM_INIT_DIALOG:

    FRAMEWIN_AddCloseButton(pMsg->hWin, FRAMEWIN_BUTTON_RIGHT, 0);
    FRAMEWIN_SetTitleVis(pMsg->hWin, 0);
    FRAMEWIN_SetClientColor(pMsg->hWin, COL_DIALOG_BK);
    FRAMEWIN_SetBorderSize(pMsg->hWin, 0);
    Init_Ctrl(pMsg);
  case WM_TOUCH:
      DockDrop(pMsg);
  //   if (getlogxy(&touch)) {
      
  //     sprintf(strout, "logic %d %d", touch.x, touch.y);
  //     hWin = WM_GetDialogItem(pMsg->hWin, GUI_ID_LOGIC);;
  //     TEXT_SetText(hWin, strout);

  //     TP_GetADC(&touch);
  //     sprintf(strout, "raw %d %d", touch.x, touch.y);
  //     hWin = WM_GetDialogItem(pMsg->hWin, GUI_ID_RAW);;
  //     TEXT_SetText(hWin, strout);
  //   }
    
    break;
  // case WM_PAINT:
  //   for (i = 0; i < sizeof(pt)/ sizeof(struct point); i++) {
  //     GUI_FillRect(pt[i].x, pt[i].y, pt[i].x + 2, pt[i].y + 2);

  //   }
    
  //   break;
  // case WM_KEY:
  //   Id = WM_GetId(pMsg->hWinSrc);

  //   switch (Id) {
  //   case GUI_ID_LEFT:
  //     OnLeftClick(pMsg);
  //     break;
  //   }
  //   break;
  case WM_NOTIFY_PARENT:
    NCode = pMsg->Data.v;   
    switch(NCode) {
    case WM_NOTIFICATION_CLICKED:
      Id = WM_GetId(pMsg->hWinSrc);
      WM_SetFocus(pMsg->hWinSrc);
      switch (Id) {
      case GUI_ID_LEFT:
        OnLeftClick(pMsg);
        break;
      case GUI_ID_RIGHT:
        OnRightClick(pMsg);
        break;
        
      }
      break;
    }
    break;
  default:
    WM_DefaultProc(pMsg);
  }
}


WM_HWIN DestopBtn_Create(WM_HWIN hParent, WM_HWIN main)
{
  // FRAMEWIN_SetTitleVis(hFrame, 0);
  // WM_SetCallback(WM_HBKWIN, &_cbBkWindow);
  this = GUI_CreateDialogBox(_aDialogCreate, GUI_COUNTOF(_aDialogCreate), &_cbCallback, (WM_HWIN)hParent, 0, 0); 
  hMain = main;
  return this;
}


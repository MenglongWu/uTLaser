/*********************************************************************
*                SEGGER MICROCONTROLLER SYSTEME GmbH                 *
*        Solutions for real time microcontroller applications        *
*                                                                    *
*                    emWin GSC sample code                           *
*                                                                    *
**********************************************************************

----------------------------------------------------------------------
File        : WIDGET_ListBoxOwnerDraw.c
Purpose     : Demonstrates a owner drawn list box
----------------------------------------------------------------------
*/

#include <stddef.h>
#include "GUI.h"
#include "DIALOG.h"
#include "wm.h"

/*********************************************************************
*
*       Static data
*
**********************************************************************
*/

static int _MultiSel;
static int _OwnerDrawn;
static int _VarY = 1;
static int _PrevTime;

/*********************************************************************
*
*       Bitmap data for user drawn list box
*/
const GUI_COLOR ColorsSmilie0[] = {
     0xFFFFFF,0x000000,0x0000FF
};

const GUI_COLOR ColorsSmilie1[] = {
     0xFFFFFF,0x000000,0x00FFFF
};

const GUI_LOGPALETTE PalSmilie0 = {
  3,	/* number of entries */
  1, 	/* Has transparency */
  &ColorsSmilie0[0]
};

const GUI_LOGPALETTE PalSmilie1 = {
  3,	/* number of entries */
  1, 	/* Has transparency */
  &ColorsSmilie1[0]
};

const unsigned char acSmilie0[] = {
  0x00, 0x55, 0x40, 0x00,
  0x01, 0xAA, 0x90, 0x00,
  0x06, 0xAA, 0xA4, 0x00,
  0x19, 0x6A, 0x59, 0x00,
  0x69, 0x6A, 0x5A, 0x40,
  0x6A, 0xA6, 0xAA, 0x40,
  0x6A, 0xA6, 0xAA, 0x40,
  0x6A, 0xA6, 0xAA, 0x40,
  0x6A, 0xAA, 0xAA, 0x40,
  0x1A, 0x95, 0xA9, 0x00,
  0x06, 0x6A, 0x64, 0x00,
  0x01, 0xAA, 0x90, 0x00,
  0x00, 0x55, 0x40, 0x00
};

const unsigned char acSmilie1[] = {
  0x00, 0x55, 0x40, 0x00,
  0x01, 0xAA, 0x90, 0x00,
  0x06, 0xAA, 0xA4, 0x00,
  0x19, 0x6A, 0x59, 0x00,
  0x69, 0x6A, 0x5A, 0x40,
  0x6A, 0xA6, 0xAA, 0x40,
  0x6A, 0xA6, 0xAA, 0x40,
  0x6A, 0xA6, 0xAA, 0x40,
  0x6A, 0xAA, 0xAA, 0x40,
  0x1A, 0x6A, 0x69, 0x00,
  0x06, 0x95, 0xA4, 0x00,
  0x01, 0xAA, 0x90, 0x00,
  0x00, 0x55, 0x40, 0x00
};

const GUI_BITMAP bmSmilie0 = {
 13, /* XSize */
 13, /* YSize */
 4,  /* BytesPerLine */
 2,  /* BitsPerPixel */
 acSmilie0,   /* Pointer to picture data (indices) */
 &PalSmilie0  /* Pointer to palette */
};

const GUI_BITMAP bmSmilie1 = {
 13, /* XSize */
 13, /* YSize */
 4,  /* BytesPerLine */
 2,  /* BitsPerPixel */
 acSmilie1,   /* Pointer to picture data (indices) */
 &PalSmilie1  /* Pointer to palette */
};

/*********************************************************************
*
*       Default contents of list box
*/
static const GUI_ConstString _ListBox[] = {
  "English", "Deutsch", NULL
};

/*********************************************************************
*
*       Dialog resource
*
* This table conatins the info required to create the dialog.
* It has been created manually, but could also be created by a GUI-builder.
*/
static const GUI_WIDGET_CREATE_INFO _aDialogCreate[] = {
  { FRAMEWIN_CreateIndirect,  "Owner drawn list box",    0,                  50,  50, 220, 140, FRAMEWIN_CF_MOVEABLE },
  { LISTBOX_CreateIndirect,   0,                         GUI_ID_MULTIEDIT0,  10,  10, 100, 100, 0, 100 },
/* Check box for multi select mode */
  { CHECKBOX_CreateIndirect,  0,                         GUI_ID_CHECK0,     120,  10,   0,   0 },
  { TEXT_CreateIndirect,      "Multi select",            0,                 140,  10,  80,  15, TEXT_CF_LEFT },
/* Check box for owner drawn list box */
  { CHECKBOX_CreateIndirect,  0,                         GUI_ID_CHECK1,     120,  35,   0,   0 },
  { TEXT_CreateIndirect,      "Owner drawn",              0,                140,  35,  80,  15, TEXT_CF_LEFT },
/* Buttons */
  { BUTTON_CreateIndirect,    "OK",                      GUI_ID_OK,         120,  65,  80,  20 },
  { BUTTON_CreateIndirect,    "Cancel",                  GUI_ID_CANCEL,     120,  90,  80,  20 },
};

/*********************************************************************
*
*       Static code
*
**********************************************************************
*/
/*********************************************************************
*
*       _cbBkWindow
*/
static void _cbBkWindow(WM_MESSAGE* pMsg) {
  switch (pMsg->MsgId) {
  case WM_PAINT:
    GUI_SetBkColor(GUI_RED);
    GUI_Clear();
    GUI_SetColor(GUI_WHITE);
    GUI_SetFont(&GUI_Font24_ASCII);
    GUI_DispStringHCenterAt("WIDGET_ListBoxOwnerDraw", 160, 5);
    break;
  default:
    WM_DefaultProc(pMsg);
  }
}

/*********************************************************************
*
*       _GetItemSizeX
*/
static int _GetItemSizeX(WM_HWIN hWin, int ItemIndex) {
  char acBuffer[100];
  int  DistX;
  LISTBOX_GetItemText(hWin, ItemIndex, acBuffer, sizeof(acBuffer));
  DistX = GUI_GetStringDistX(acBuffer);
  return DistX + bmSmilie0.XSize + 16;
}

/*********************************************************************
*
*       _GetItemSizeY
*/
static int _GetItemSizeY(WM_HWIN hWin, int ItemIndex) {
  int DistY;
  DistY = GUI_GetFontDistY() + 1;
  if (LISTBOX_GetMulti(hWin)) {
    if (LISTBOX_GetItemSel(hWin, ItemIndex)) {
      DistY += 8;
    }
  } else if (LISTBOX_GetSel(hWin) == ItemIndex) {
    DistY += 8;
  }
  return DistY;
}

/*********************************************************************
*
*       _OwnerDraw
*
* Purpose:
*   This is the owner draw function.
*   It allows complete customization of how the items in the listbox are
*   drawn. A command specifies what the function should do;
*   The minimum is to react to the draw command (WIDGET_ITEM_DRAW);
*   If the item x-size differs from the default, then this information
*   needs to be returned in reaction to WIDGET_ITEM_GET_XSIZE.
*   To insure compatibility with future version, all unhandled commands
*   must call the default routine LISTBOX_OwnerDraw.
*/
static int _OwnerDraw(const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo) {
  WM_HWIN hWin;
  int Index;
  hWin     = pDrawItemInfo->hWin;
  Index    = pDrawItemInfo->ItemIndex;
  switch (pDrawItemInfo->Cmd) {
  case WIDGET_ITEM_GET_XSIZE:
    return _GetItemSizeX(hWin, Index);
  case WIDGET_ITEM_GET_YSIZE:
    return _GetItemSizeY(hWin, Index);
  case WIDGET_ITEM_DRAW:
    {
      int MultiSel, Sel, YSize, FontDistY;
      int IsDisabled, IsSelected;
      int ColorIndex = 0;
      char acBuffer[100];
      const GUI_BITMAP * pBm;
      const GUI_FONT* pOldFont = 0;
      GUI_COLOR aColor[4] = {GUI_BLACK, GUI_WHITE, GUI_WHITE, GUI_GRAY};
      GUI_COLOR aBkColor[4] = {GUI_WHITE, GUI_GRAY, GUI_BLUE, 0xC0C0C0};
      IsDisabled = LISTBOX_GetItemDisabled(pDrawItemInfo->hWin, pDrawItemInfo->ItemIndex);
      IsSelected = LISTBOX_GetItemSel(hWin, Index);
      MultiSel   = LISTBOX_GetMulti(hWin);
      Sel        = LISTBOX_GetSel(hWin);
      YSize      = _GetItemSizeY(hWin, Index);
      /* Calculate color index */
      if (MultiSel) {
        if (IsDisabled) {
          ColorIndex = 3;
        } else {
          ColorIndex = (IsSelected) ? 2 : 0;
        }
      } else {
        if (IsDisabled) {
          ColorIndex = 3;
        } else {
          if (pDrawItemInfo->ItemIndex == Sel) {
            ColorIndex = WM_HasFocus(pDrawItemInfo->hWin) ? 2 : 1;
          } else {
            ColorIndex = 0;
          }
        }
      }
      /* Draw item */
      GUI_SetBkColor(aBkColor[ColorIndex]);
      GUI_SetColor  (aColor[ColorIndex]);
      LISTBOX_GetItemText(pDrawItemInfo->hWin, pDrawItemInfo->ItemIndex, acBuffer, sizeof(acBuffer));
      GUI_Clear();
      if ((ColorIndex == 1) || (ColorIndex == 2)) {
        pOldFont = GUI_SetFont(&GUI_Font13B_1);
      }
      FontDistY  = GUI_GetFontDistY();
      GUI_DispStringAt(acBuffer, pDrawItemInfo->x0 + bmSmilie0.XSize + 16, pDrawItemInfo->y0 + (YSize - FontDistY) / 2);
      if (pOldFont) {
        GUI_SetFont(pOldFont);
      }
      GUI_DispCEOL();
      /* Draw bitmap */
      pBm = MultiSel ? IsSelected ? &bmSmilie1 : &bmSmilie0 : (pDrawItemInfo->ItemIndex == Sel) ? &bmSmilie1 : &bmSmilie0;
      GUI_DrawBitmap(pBm, pDrawItemInfo->x0 + 7, pDrawItemInfo->y0 + (YSize - pBm->YSize) / 2);
      /* Draw focus rectangle */
      if (MultiSel && (pDrawItemInfo->ItemIndex == Sel)) {
        GUI_RECT rFocus;
        GUI_RECT rInside;
        WM_GetInsideRectEx(pDrawItemInfo->hWin, &rInside);
        rFocus.x0 = pDrawItemInfo->x0;
        rFocus.y0 = pDrawItemInfo->y0;
        rFocus.x1 = rInside.x1;
        rFocus.y1 = pDrawItemInfo->y0 + YSize - 1;
        GUI_SetColor(GUI_WHITE - aBkColor[ColorIndex]);
        GUI_DrawFocusRect(&rFocus, 0);
      }
    }
    break;
  default:
    return LISTBOX_OwnerDraw(pDrawItemInfo);
  }
  return 0;
}

/*********************************************************************
*
*       _cbCallback
*/
static void _cbCallback(WM_MESSAGE * pMsg) {
  int NCode, Id;
  WM_HWIN hDlg, hListBox, hItem;
  hDlg = pMsg->hWin;
  hListBox = WM_GetDialogItem(hDlg, GUI_ID_MULTIEDIT0);
  switch (pMsg->MsgId) {
    case WM_INIT_DIALOG:
      LISTBOX_SetText(hListBox, _ListBox);
      LISTBOX_AddString(hListBox, "Français");
      LISTBOX_AddString(hListBox, "Japanese");
      LISTBOX_AddString(hListBox, "Italiano");
      LISTBOX_AddString(hListBox, "Español");
      LISTBOX_AddString(hListBox, "Greek");
      LISTBOX_AddString(hListBox, "Hebrew");
      LISTBOX_AddString(hListBox, "Dutch");
      LISTBOX_AddString(hListBox, "Other language ...");
      LISTBOX_SetScrollStepH(hListBox, 6);
      LISTBOX_SetAutoScrollH(hListBox, 1);
      LISTBOX_SetAutoScrollV(hListBox, 1);
      LISTBOX_SetOwnerDraw(hListBox, _OwnerDraw);
      hItem  = WM_GetDialogItem(hDlg, GUI_ID_CHECK1);
      CHECKBOX_Check(hItem);
      break;
    case WM_KEY:
      switch (((WM_KEY_INFO*)(pMsg->Data.p))->Key) {
        case GUI_KEY_ESCAPE:
          GUI_EndDialog(hDlg, 1);
          break;
        case GUI_KEY_ENTER:
          GUI_EndDialog(hDlg, 0);
          break;
      }
      break;
    case WM_TOUCH_CHILD:
      WM_SetFocus(hListBox);
      break;
    case WM_NOTIFY_PARENT:
      Id    = WM_GetId(pMsg->hWinSrc);      /* Id of widget */
      NCode = pMsg->Data.v;                 /* Notification code */
      hItem  = WM_GetDialogItem(hDlg, Id);
      switch (NCode) {
        case WM_NOTIFICATION_SEL_CHANGED:
          LISTBOX_InvalidateItem(hListBox, LISTBOX_ALL_ITEMS);
          break;
        case WM_NOTIFICATION_RELEASED:      /* React only if released */
          switch (Id) {
            case GUI_ID_OK:
              GUI_EndDialog(hDlg, 0);
              break;
            case GUI_ID_CANCEL:
              GUI_EndDialog(hDlg, 1);
              break;
            case GUI_ID_CHECK0:
              _MultiSel ^= 1;
              LISTBOX_SetMulti(hListBox, _MultiSel);
              WM_SetFocus(hListBox);
              LISTBOX_InvalidateItem(hListBox, LISTBOX_ALL_ITEMS);
              break;
            case GUI_ID_CHECK1:
              _OwnerDrawn ^= 1;
              if (_OwnerDrawn) {
                LISTBOX_SetOwnerDraw(hListBox, _OwnerDraw);
              } else {
                LISTBOX_SetOwnerDraw(hListBox, NULL);
              }
              LISTBOX_InvalidateItem(hListBox, LISTBOX_ALL_ITEMS);
              break;
          }
          break;
      }
      break;
    default:
      WM_DefaultProc(pMsg);
  }
}

/*********************************************************************
*
*       MainTask
*
*       Demonstrates a owner drawn list box
*
**********************************************************************
*/

void MainTask(void) {
  // GUI_Init();
  WM_SetCallback(WM_HBKWIN, &_cbBkWindow);
  // WM_SetCreateFlags(WM_CF_MEMDEV);  /* Use memory devices on all windows to avoid flicker */
  // GUI_ExecDialogBox(_aDialogCreate, GUI_COUNTOF(_aDialogCreate), &_cbCallback, 0, 0, 0);
  GUI_CreateDialogBox(_aDialogCreate, GUI_COUNTOF(_aDialogCreate), NULL/*&_cbCallback*/, WM_HBKWIN, 0, 0); 
  return ;
  while (1) {
    _MultiSel   = 0;
    _OwnerDrawn = 1;
    GUI_ExecDialogBox(_aDialogCreate, GUI_COUNTOF(_aDialogCreate), &_cbCallback, 0, 0, 0);
    GUI_Delay(1000);
  }
}




// 程序清单 4.1 建立一个自动更新窗口的回调函数
void WinHandler (WM_MESSAGE * pMsg)
{
	switch (pMsg->MsgId)
	{ 
	case WM_PAINT:
		GUI_SetBkColor(0xff00);
		GUI_Clear();
		GUI_DispStringAt("hello world",0,0);
		break;
	}
}
// 使用回调函数，更为详细的示例如程序清单4.2所示：
// 程序清单4.2  回调函数
#include "GUI.H"

/* 背景窗的回调函数 */
static void cbBackgroundWin(WM_MESSAGE* pMsg) 
{
switch (pMsg->MsgId) 
{
case WM_PAINT:    GUI_Clear();
default:          WM_DefaultProc(pMsg);
}
}
/* 前景窗的回调函数 */
static void cbForegroundWin(WM_MESSAGE* pMsg) 
{
switch (pMsg->MsgId) 
{
case WM_PAINT:    GUI_SetBkColor(GUI_GREEN);
GUI_Clear();
GUI_DispString("Foreground window");
default:          WM_DefaultProc(pMsg);
}
}
/*  回调机制 */
void DemoRedraw(void) 
{
GUI_HWIN hWnd;
// while(1) 
{
/*创建一个前景窗 */
hWnd = WM_CreateWindow(10, 10, 100, 100, WM_CF_SHOW, cbForegroundWin, 0);
/*显示前景窗*/
GUI_Delay(1000);
/* 删除前景窗 */
WM_DeleteWindow(hWnd);
GUI_DispStringAt("Background of window has not been redrawn", 10, 10);
/* 等待，显示并不重绘 */
GUI_Delay(1000);
GUI_Clear();
/* 设置背景窗的回调功能 */
WM_SetCallback(WM_HBKWIN, cbBackgroundWin);
/*创建一个前景窗 */
hWnd = WM_CreateWindow(10, 10, 100, 100,WM_CF_SHOW, cbForegroundWin, 0);
/* 显示前景窗*/
GUI_Delay(1000);
/*删除前景窗 */
WM_DeleteWindow(hWnd);
/* 等待，显示将重绘*/
GUI_Delay(1000);
/* 删除回调函数 */
WM_SetCallback(WM_HBKWIN, 0);
}
}


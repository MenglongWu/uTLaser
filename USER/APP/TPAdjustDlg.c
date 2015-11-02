/*********************************************************************
*				SEGGER MICROCONTROLLER SYSTEME GmbH				 *
*		Solutions for real time microcontroller applications		*
*																	*
*					emWin GSC sample code							 *
*																	*
**********************************************************************

----------------------------------------------------------------------
File		: WIDGET_ListBoxOwnerDraw.c
Purpose	 : Demonstrates a owner drawn list box
----------------------------------------------------------------------
*/

#include <stddef.h>
#include "GUI.h"
#include "DIALOG.h"
#include "wm.h"
#include "project.h"
#include "USER/TouchPanel/touch.h"



static FRAMEWIN_Handle hFrame = 0;
static BUTTON_Handle hButton1;
static BUTTON_Handle hButton2;
static WM_HWIN this = 0;

#define GUI_ID_PP (GUI_ID_BUTTON0+0)
#define GUI_ID_APD (GUI_ID_BUTTON0+1)
#define GUI_ID_PWM (GUI_ID_BUTTON0+2)
#define GUI_ID_PWM_REVERSAL (GUI_ID_BUTTON0+3)
#define GUI_ID_PWM_WIDTH (GUI_ID_BUTTON0+4)
#define GUI_ID_LASTER (GUI_ID_BUTTON0+5)
#define GUI_ID_SETTING (GUI_ID_BUTTON0+6)


static const GUI_WIDGET_CREATE_INFO _aDialogCreate[] = {
	{ FRAMEWIN_CreateIndirect,	"Owner drawn list box",	0,					0,	0, 320, 240 , FRAMEWIN_CF_MOVEABLE },
		 // { LISTBOX_CreateIndirect,	 0,						 GUI_ID_MULTIEDIT0,	10,	10, 100, 100, 0, 100 },
	// /* Check box for multi select mode */
	//	 { CHECKBOX_CreateIndirect,	0,						 GUI_ID_CHECK0,	 120,	10,	 0,	 0 },
	//	 { TEXT_CreateIndirect,		"Multi select",			0,				 140,	10,	80,	15, TEXT_CF_LEFT },
	// /* Check box for owner drawn list box */
	//	 { CHECKBOX_CreateIndirect,	0,						 GUI_ID_CHECK1,	 120,	35,	 0,	 0 },
	//	 { TEXT_CreateIndirect,		"Owner drawn",				0,				140,	35,	80,	15, TEXT_CF_LEFT },
	// /* Buttons */
	//	 { BUTTON_CreateIndirect,	"OK",						GUI_ID_OK,		 120,	65,	80,	20 },
	//	 { BUTTON_CreateIndirect,	"Cancel",					GUI_ID_CANCEL,	 120,	90,	80,	20 },
	// {BUTTON_CreateIndirect,	 "ON/OFF",					GUI_ID_PP,	 22,15,136,136},
	// {BUTTON_CreateIndirect,	 "APD",						GUI_ID_APD,	 162,15,66,66},
	// {BUTTON_CreateIndirect,	 "PWM",						GUI_ID_PWM,	 232,15,66,66},
	// {BUTTON_CreateIndirect,	 "PWM\r\nNormal",			GUI_ID_PWM_REVERSAL,	 162,85,66,66},
	// {BUTTON_CreateIndirect,	 "1us",						GUI_ID_PWM_WIDTH,	 232,85,66,66},
	// {BUTTON_CreateIndirect,	 "Laster",						GUI_ID_LASTER,	 22,155,66,66},
	// {BUTTON_CreateIndirect,	 "Set",						GUI_ID_SETTING,	 95,155,66,66},
	// {BUTTON_CreateIndirect,	 "2",						GUI_ID_BUTTON0+7,	 162,155,136,66},
};

// void TurnBack(WM_HWIN	hWin)
// {
// 	while(WM_GetWindowOrgY(hWin) > 50) {
// 		while(WM_GetWindowOrgY(hWin) > 30) {
// 			WM_MoveWindow(hWin,0,-20);
// 			// Sleep(20);
// 			GUI_Exec();
// 		}
// 		while(WM_GetWindowOrgY(hWin) > 10) {
// 			WM_MoveWindow(hWin,0,-2);
// 			// Sleep(20);
// 			GUI_Exec();
// 		}
// 	}
// 	WM_MoveTo(hWin,0,0);
// }

// static void _cbBkWindow(WM_MESSAGE* pMsg) 
// {
// 	int x1,y1;
// 	static int x0,y0;


// 	switch (pMsg->MsgId) {
// 	case WM_PAINT:
// 		// GUI_SetBkColor(GUI_RED);
// 		// GUI_Clear();
// 		TC_Adj();
// 		// GUI_SetColor(GUI_WHITE);
// 		// GUI_SetFont(&GUI_Font24_ASCII);
// 		// GUI_DispStringHCenterAt("WIDGET_ListBoxOwnerDraw", 160, 5);
// 		break;
// 	default:
// 		WM_DefaultProc(pMsg);
// 	}
// }

// ***************************************************************************

GUI_POINT poly[6] = {
	{0, -10},
	{0,10},
	{0,0},
	{-10,0},
	{10,0},
	{0,0},
};
/*********************************************************************
*
*		 _cbCallback
*/
static void _cbCallback(WM_MESSAGE * pMsg) {
	char strout[222];
	static GUI_RECT rect;
	int NCode, Id;
	static int x0,y0 = 0;
	int x1,y1, i;
	static int fpress = 0;
	int press = 0;
	static int index = 0;
	GUI_PID_STATE state;
	GUI_POINT polya[3];
	WM_MESSAGE smsg;
	WM_HWIN hDlg, hListBox, hItem;
	GUI_POINT pt[] = {
		{60,                 60},
		{LCD_XSIZE_TFT - 60,                 60},
		{60,                 LCD_YSIZE_TFT - 60},
		{LCD_XSIZE_TFT - 60 ,LCD_YSIZE_TFT - 60},
		{LCD_XSIZE_TFT / 2, LCD_YSIZE_TFT / 2},
		{0, 0},
	};

	hDlg = pMsg->hWin;
	hListBox = WM_GetDialogItem(hDlg, GUI_ID_MULTIEDIT0);


	// printf("%x %x %x\n", pMsg->MsgId, pMsg->hWin,pMsg->hWinSrc);
	switch (pMsg->MsgId) {
	case WM_INIT_DIALOG:
		// hButton1 = WM_GetDialogItem(pMsg->hWin, GUI_ID_BUTTON0);
		// BUTTON_SetText(hButton1, "dfwer");
		// GUI_SetFont(&GUI_Font8x10_ASCII);
		// FRAMEWIN_AddCloseButton(pMsg->hWin, FRAMEWIN_BUTTON_RIGHT, 0);
		// hFrame = WM_GetDialogItem(pMsg->hWin, 0);
		FRAMEWIN_SetTitleVis(pMsg->hWin, 0);
		// FRAMEWIN_SetClientColor(pMsg->hWin, RGB(255,0,0));
		FRAMEWIN_SetBorderSize(pMsg->hWin, 0);
		// Init_Ctrl(pMsg);
	case WM_TOUCH:
		{
			struct point pt;

			if (TP_IsPress()) {
				gettouch(&pt);
				Delay_ms(100);
				if (TP_IsPress()) {
					gettouch(&pt);	
					printf("touch %d %d\r\n", pt.x, pt.y);	

					GUI_FillPolygon(poly,6, 22,22);
					// WM_InvalidateWindow(this);
					index++;
					WM_HideWindow(this);
					WM_ShowWindow(this);
					// WM_Paint(this);
				}
				
			}
		}
		
		break;
	case WM_PAINT:
		GUI_SetColor(RGB(255,0,0));
		for (i = 0; i < 5; i++) {
			GUI_FillPolygon(poly,6, pt[i].x,pt[i].y);
		}
		GUI_SetColor(RGB(0,255,0));
		GUI_FillPolygon(poly,6, pt[index].x,pt[index].y);
		printf("index = %d\r\n", index);
		
		if (index >= 5) {
			index = 0;
			// WM_DeleteWindow(this);
			// GUI_EndDialog(this, 1);
			WM_HideWindow(this);
		}
	
		break;
	
	default:
		WM_DefaultProc(pMsg);
	}
}


WM_HWIN TPAdjustDlg_Create(WM_HWIN hParent)
{
	// FRAMEWIN_SetTitleVis(hFrame, 0);
	// WM_SetCallback(WM_HBKWIN, &_cbBkWindow);
	this = GUI_CreateDialogBox(_aDialogCreate, GUI_COUNTOF(_aDialogCreate), &_cbCallback, (WM_HWIN)hParent, 0, 0); 
	return this;
}


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
#include "prj_type.h"
#include "flash.h"


static FRAMEWIN_Handle hFrame = 0;
static BUTTON_Handle hButton1;
static BUTTON_Handle hButton2;
static WM_HWIN this = 0;

#define GUI_ID_RAW (GUI_ID_BUTTON0+0)
#define GUI_ID_LOGIC (GUI_ID_BUTTON0+1)
#define GUI_ID_CLOSE (GUI_ID_BUTTON0+2)
// #define GUI_ID_PWM_REVERSAL (GUI_ID_BUTTON0+3)
// #define GUI_ID_PWM_WIDTH (GUI_ID_BUTTON0+4)
// #define GUI_ID_LASTER (GUI_ID_BUTTON0+5)
// #define GUI_ID_SETTING (GUI_ID_BUTTON0+6)


static const GUI_WIDGET_CREATE_INFO _aDialogCreate[] = {
	{ FRAMEWIN_CreateIndirect,	"",	0,					0,	0, 320, 240 , FRAMEWIN_CF_MOVEABLE },
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
	{TEXT_CreateIndirect,	 "raw",					GUI_ID_RAW,	 100,10,100,20},
	{TEXT_CreateIndirect,	 "logic",						GUI_ID_LOGIC,	 100,30,100,20},
	{BUTTON_CreateIndirect,	 "CLOSE",						GUI_ID_CLOSE,	 0,0,50,20},
	// {BUTTON_CreateIndirect,	 "PWM\r\nNormal",			GUI_ID_PWM_REVERSAL,	 162,85,66,66},
	// {BUTTON_CreateIndirect,	 "1us",						GUI_ID_PWM_WIDTH,	 232,85,66,66},
	// {BUTTON_CreateIndirect,	 "Laster",						GUI_ID_LASTER,	 22,155,66,66},
	// {BUTTON_CreateIndirect,	 "Set",						GUI_ID_SETTING,	 95,155,66,66},
	// {BUTTON_CreateIndirect,	 "2",						GUI_ID_BUTTON0+7,	 162,155,136,66},
};



static void OnHideClick(WM_MESSAGE * pMsg)
{
	// WM_HWIN hWin;
	// hWin = WM_GetDialogItem(pMsg->hWin, GUI_ID_CLOSE);;
	// BUTTON_SetText(hWin, "dddf");
	WM_HideWindow(this);
	// printf("kdjflsjldjfsf");

}
/*********************************************************************
*
*		 _cbCallback
*/
static void _cbCallback(WM_MESSAGE * pMsg) {
	static char strout[40];
	struct point touch;
	int i;
	int NCode, Id;
	WM_HWIN hWin;
	struct point pt[] = {
		{50,50},		{100,50},		{150,50},		{200,50},	{250,50},		{350,50},
		{50,100},		{100,100},		{150,100},		{200,100},	{250,100},		{350,100},
		{50,150},		{100,150},		{150,150},		{200,150},	{250,150},		{350,150},
		{50,200},		{100,200},		{150,200},		{200,200},	{250,200},		{350,200},
		{50,250},		{100,250},		{150,250},		{200,250},	{250,250},		{350,250},
	};
	
	

	// printf("%x %x %x\n", pMsg->MsgId, pMsg->hWin,pMsg->hWinSrc);
	switch (pMsg->MsgId) {
	case WM_INIT_DIALOG:
		// hButton1 = WM_GetDialogItem(pMsg->hWin, GUI_ID_BUTTON0);
		// BUTTON_SetText(hButton1, "dfwer");
		// GUI_SetFont(&GUI_Font8x10_ASCII);
		FRAMEWIN_AddCloseButton(pMsg->hWin, FRAMEWIN_BUTTON_RIGHT, 0);
		// hFrame = WM_GetDialogItem(pMsg->hWin, 0);
		FRAMEWIN_SetTitleVis(pMsg->hWin, 0);
		FRAMEWIN_SetClientColor(pMsg->hWin, RGB(255,0,0));
		FRAMEWIN_SetBorderSize(pMsg->hWin, 0);
		// Init_Ctrl(pMsg);
	case WM_TOUCH:
		
		if (getlogxy(&touch)) {
			
			sprintf(strout, "logic %d %d", touch.x, touch.y);
			hWin = WM_GetDialogItem(pMsg->hWin, GUI_ID_LOGIC);;
			TEXT_SetText(hWin, strout);

			TP_GetADC(&touch);
			sprintf(strout, "raw %d %d", touch.x, touch.y);
			hWin = WM_GetDialogItem(pMsg->hWin, GUI_ID_RAW);;
			TEXT_SetText(hWin, strout);
		}
		
		break;
	case WM_PAINT:
		for (i = 0; i < sizeof(pt)/ sizeof(struct point); i++) {
			GUI_FillRect(pt[i].x, pt[i].y, pt[i].x + 2, pt[i].y + 2);
		}
		
		break;
	case WM_KEY:
		Id = WM_GetId(pMsg->hWinSrc);

		switch (Id) {
		case GUI_ID_CLOSE:
			OnHideClick(pMsg);
			break;
		}
		break;
	case WM_NOTIFY_PARENT:
		NCode = pMsg->Data.v;		
		switch(NCode) {
		case WM_NOTIFICATION_CLICKED:
			Id = WM_GetId(pMsg->hWinSrc);
			WM_SetFocus(pMsg->hWinSrc);
			switch (Id) {
			case GUI_ID_CLOSE:
				OnHideClick(pMsg);
				break;
			}
			break;
		}
		break;
	default:
		WM_DefaultProc(pMsg);
	}
}


WM_HWIN TPTestDlg_Create(WM_HWIN hParent)
{
	// FRAMEWIN_SetTitleVis(hFrame, 0);
	// WM_SetCallback(WM_HBKWIN, &_cbBkWindow);
	this = GUI_CreateDialogBox(_aDialogCreate, GUI_COUNTOF(_aDialogCreate), &_cbCallback, (WM_HWIN)hParent, 0, 0); 
	return this;
}


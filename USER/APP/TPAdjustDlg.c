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
static WM_HWIN parent = 0;

#define GUI_ID_PP (GUI_ID_BUTTON0+0)
#define GUI_ID_APD (GUI_ID_BUTTON0+1)
#define GUI_ID_PWM (GUI_ID_BUTTON0+2)
#define GUI_ID_PWM_REVERSAL (GUI_ID_BUTTON0+3)
#define GUI_ID_PWM_WIDTH (GUI_ID_BUTTON0+4)
#define GUI_ID_LASTER (GUI_ID_BUTTON0+5)
#define GUI_ID_SETTING (GUI_ID_BUTTON0+6)
#define GUI_ID_CLOSE (GUI_ID_BUTTON0+2)

static const GUI_WIDGET_CREATE_INFO _aDialogCreate[] = {
	{ FRAMEWIN_CreateIndirect,	"Owner drawn list box",	0,					0,	0, 320, 240 , FRAMEWIN_CF_MOVEABLE },
	{BUTTON_CreateIndirect,	 "CLOSE",						GUI_ID_CLOSE,	 0,0,50,20},
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
static void OnHideClick(WM_MESSAGE * pMsg)
{
	// WM_HWIN hWin;
	// hWin = WM_GetDialogItem(pMsg->hWin, GUI_ID_CLOSE);;
	// BUTTON_SetText(hWin, "dddf");
	// WM_HideWindow(this);
	WM_ShowWindow(parent);
	GUI_EndDialog(this,0);
	// printf("kdjflsjldjfsf");

}

int IsGood(int a, int b, int dt)
{
	printf("a - b dt %d %d %d\r\n", a, b, dt);
	if ( (a - b) < dt &&
		(a - b) > -dt) {
		printf("is good \r\n");
		return 1;
	}
	printf("is bad\r\n");
	return 0;
}
int CheckAdj(struct point *adj)
{
	printf("%d %d\r\n",adj[0].x, adj[0].y);
	printf("%d %d\r\n",adj[1].x, adj[1].y);
	printf("%d %d\r\n",adj[2].x, adj[2].y);
	printf("%d %d\r\n",adj[3].x, adj[3].y);
	printf("%d %d\r\n",adj[4].x, adj[4].y);
	if (
		IsGood(adj[0].x, adj[1].x,200) &&
		IsGood(adj[2].x, adj[3].x,200) &&
		IsGood(adj[0].y, adj[2].y,200) &&
		IsGood(adj[1].y, adj[3].y,200) ) {
		printf("all good\r\n");
	}

		// isGood((adj[0].x + adj[1].x)/2, adj[3].x,60) &&
}

// ***************************************************************************
#define FLIT_TP (100)
GUI_POINT poly[6] = {
	{0, -10},
	{0,10},
	{0,0},
	{-10,0},
	{10,0},
	{0,0},
};
extern struct project_env g_env;
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
	static struct point adj[5],adjprev={3333,1111};
	int dt;
	WM_HWIN hDlg, hListBox, hItem;
	GUI_POINT pt[] = {
		{60,                 60},
		{LCD_XSIZE_TFT - 60,                 60},
		{60,                 LCD_YSIZE_TFT - 60},
		{LCD_XSIZE_TFT - 60 ,LCD_YSIZE_TFT - 60},
		{LCD_XSIZE_TFT / 2, LCD_YSIZE_TFT / 2},
		{0, 0},
	};
	struct adj_tp tpadj;
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
		FRAMEWIN_SetClientColor(pMsg->hWin, COL_DIALOG_BK);
		FRAMEWIN_SetBorderSize(pMsg->hWin, 0);
		// Init_Ctrl(pMsg);
	case WM_TOUCH:
		{
			// struct point pt;

			if (TP_IsPress()) {
				gettouch(&adj[index]);
				Delay_ms(100);
				if (TP_IsPress()) {
					gettouch(&adj[index]);

					dt = ((adj[index].x + adj[index].y) - (adjprev.x + adjprev.y) );
					printf("dt %d %d %d\r\n", dt, adj[index].x , adj[index].y);
					if ( dt < FLIT_TP && dt > -FLIT_TP) {
						break;
					}
					adjprev.x = adj[index].x;
					adjprev.y = adj[index].y;

					printf("touch %d %d\r\n", adj[index].x, adj[index].y);	

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
			if (CheckAdj(adj) ) {
				GUI_DispStringAt("----OK----\nTouch adjust success", LCD_XSIZE_TFT / 2-40, LCD_YSIZE_TFT / 2);
				tp_adj(pt, adj, &g_env.adj_tp);
				
				g_env.flag = 0xaabbccdd;
				WriteFlash(FLASH_PAGE_START, 
					(uint32_t*)&(g_env), 
					sizeof(struct project_env));
				tp_setadj(&g_env.adj_tp);

				
			}
			else {
				GUI_DispStringAt("----ERROR----\nPlease adjust again!!!",LCD_XSIZE_TFT / 2 - 40, LCD_YSIZE_TFT / 2);
			}
			Delay_ms(2000);
			GUI_EndDialog(this, 0);
			// WM_HideWindow(this);
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


WM_HWIN TPAdjustDlg_Create(WM_HWIN hParent)
{
	// FRAMEWIN_SetTitleVis(hFrame, 0);
	// WM_SetCallback(WM_HBKWIN, &_cbBkWindow);
	// this = GUI_CreateDialogBox(_aDialogCreate, GUI_COUNTOF(_aDialogCreate), &_cbCallback, (WM_HWIN)hParent, 0, 0); 
	parent = hParent;
	this = GUI_CreateDialogBox(_aDialogCreate, GUI_COUNTOF(_aDialogCreate), &_cbCallback, (WM_HWIN)0, 0, 0); 
	return this;
}


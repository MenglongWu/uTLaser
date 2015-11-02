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
WM_HWIN hdlg = 0;

#define GUI_ID_PP (GUI_ID_BUTTON0+0)
#define GUI_ID_APD (GUI_ID_BUTTON0+1)
#define GUI_ID_PWM (GUI_ID_BUTTON0+2)
#define GUI_ID_PWM_REVERSAL (GUI_ID_BUTTON0+3)
#define GUI_ID_PWM_WIDTH (GUI_ID_BUTTON0+4)
#define GUI_ID_LASTER (GUI_ID_BUTTON0+5)
#define GUI_ID_SETTING (GUI_ID_BUTTON0+6)
#define GUI_ID_DELETE (GUI_ID_BUTTON0+8)


static const GUI_WIDGET_CREATE_INFO _aDialogCreate[] = {
	{ FRAMEWIN_CreateIndirect,	"Owner drawn list box",	0,					0,	0, 320, 240 , FRAMEWIN_CF_MOVEABLE },
	//	 { LISTBOX_CreateIndirect,	 0,						 GUI_ID_MULTIEDIT0,	10,	10, 100, 100, 0, 100 },
	// /* Check box for multi select mode */
	//	 { CHECKBOX_CreateIndirect,	0,						 GUI_ID_CHECK0,	 120,	10,	 0,	 0 },
	//	 { TEXT_CreateIndirect,		"Multi select",			0,				 140,	10,	80,	15, TEXT_CF_LEFT },
	// /* Check box for owner drawn list box */
	//	 { CHECKBOX_CreateIndirect,	0,						 GUI_ID_CHECK1,	 120,	35,	 0,	 0 },
	//	 { TEXT_CreateIndirect,		"Owner drawn",				0,				140,	35,	80,	15, TEXT_CF_LEFT },
	// /* Buttons */
	//	 { BUTTON_CreateIndirect,	"OK",						GUI_ID_OK,		 120,	65,	80,	20 },
	//	 { BUTTON_CreateIndirect,	"Cancel",					GUI_ID_CANCEL,	 120,	90,	80,	20 },
	{BUTTON_CreateIndirect,	 "ON/OFF",					GUI_ID_PP,	 22,15,136,136},
	{BUTTON_CreateIndirect,	 "APD",						GUI_ID_APD,	 162,15,66,66},
	{BUTTON_CreateIndirect,	 "PWM",						GUI_ID_PWM,	 232,15,66,66},
	{BUTTON_CreateIndirect,	 "PWM\r\nNormal",			GUI_ID_PWM_REVERSAL,	 162,85,66,66},
	{BUTTON_CreateIndirect,	 "1us",						GUI_ID_PWM_WIDTH,	 232,85,66,66},
	{BUTTON_CreateIndirect,	 "Laster",						GUI_ID_LASTER,	 22,155,66,66},
	{BUTTON_CreateIndirect,	 "Set",						GUI_ID_SETTING,	 95,155,66,66},
	{BUTTON_CreateIndirect,	 "Test",						GUI_ID_DELETE,	 162,155,136,66},
};

void TurnBack(WM_HWIN	hWin)
{
	while(WM_GetWindowOrgY(hWin) > 50) {
		while(WM_GetWindowOrgY(hWin) > 30) {
			WM_MoveWindow(hWin,0,-20);
			// Sleep(20);
			GUI_Exec();
		}
		while(WM_GetWindowOrgY(hWin) > 10) {
			WM_MoveWindow(hWin,0,-2);
			// Sleep(20);
			GUI_Exec();
		}
	}
	WM_MoveTo(hWin,0,0);
}

static void _cbBkWindow(WM_MESSAGE* pMsg) 
{
	int x1,y1;
	static int x0,y0;


	switch (pMsg->MsgId) {
	case WM_PAINT:
		GUI_SetBkColor(GUI_RED);
		GUI_Clear();
		// GUI_SetColor(GUI_WHITE);
		// GUI_SetFont(&GUI_Font24_ASCII);
		// GUI_DispStringHCenterAt("WIDGET_ListBoxOwnerDraw", 160, 5);
		break;
	default:
		WM_DefaultProc(pMsg);
	}
}

// ***************************************************************************
// uLaster 测试代码按键消息
static int sg_bppdown = 0;//标识外部电源是否关闭
void OnPerPowerClick(WM_MESSAGE * pMsg)
{
	char strout[200];
	BUTTON_Handle hdc;
	WM_HWIN hDlg;
	int color;

	hdc = pMsg->hWinSrc;
	BUTTON_GetText(hdc,strout,200);
	if (strout[1] == 'n') {
		BUTTON_SetText(hdc, "Off");
		Ctrl_PeripheralPower(CTRL_PP_OFF);
		color = RGB(255,255,255);
		sg_bppdown = 1;
	}
	else {
		BUTTON_SetText(hdc, "On");
		Ctrl_PeripheralPower(CTRL_PP_ON);
		color = RGB(0,0,0);
		sg_bppdown = 0;
	}
	

	hDlg = pMsg->hWin;

	hdc = WM_GetDialogItem(hDlg, GUI_ID_APD);
	BUTTON_SetTextColor(hdc, 0, color);
	hdc = WM_GetDialogItem(hDlg, GUI_ID_PWM);
	BUTTON_SetTextColor(hdc, 0, color);
	hdc = WM_GetDialogItem(hDlg, GUI_ID_PWM_REVERSAL);
	BUTTON_SetTextColor(hdc, 0, color);
	hdc = WM_GetDialogItem(hDlg, GUI_ID_PWM_WIDTH);
	BUTTON_SetTextColor(hdc, 0, color);
	hdc = WM_GetDialogItem(hDlg, GUI_ID_LASTER);
	BUTTON_SetTextColor(hdc, 0, color);

}
void OnAPDClick(WM_MESSAGE * pMsg)
{
	char strout[200];
	BUTTON_Handle hdc;

	hdc = pMsg->hWinSrc;
	BUTTON_GetText(hdc,strout,200);

	switch(strout[4]) {
	case '4':
		BUTTON_SetText(hdc, "APD 20V");
		Ctrl_APD(CTRL_APD_20V);
		break;
	default:// 'H':
		BUTTON_SetText(hdc, "APD 40V");
		Ctrl_APD(CTRL_APD_40V);
		break;
	}
}




void OnPWMClick(WM_MESSAGE * pMsg)
{
	static int pwm_together = 2222;
	struct ctrl_pwm pwm1,pwm2;
	BUTTON_Handle hdc;

	hdc = pMsg->hWinSrc;

	pwm1.ch = PWM_CH1;
	pwm2.ch = PWM_CH2;
	Get_PWM(&pwm1);
	Get_PWM(&pwm2);
	printf("ch %d %d\r\n", pwm1.ch, pwm2.ch);
	printf("high %d %d\r\n", pwm1.high, pwm2.high);
	printf("cycle %d %d\r\n", pwm1.cycle, pwm2.cycle);
	printf("reversal %d %d\r\n", pwm1.reversal, pwm2.reversal);
	printf("enable %d %d\r\n", pwm1.enable, pwm2.enable);
	switch(pwm_together) {
	case 1:
		BUTTON_SetText(hdc, "PWMoff\r\nPWM2");
		pwm1.enable = 0;
		pwm2.enable = 1;
		pwm_together = 2;
		break;
	case 2:
		BUTTON_SetText(hdc, "PWM1\r\nPWM2");
		pwm1.enable = 1;
		pwm2.enable = 1;
		pwm_together = 3;
		break;
	case 3:	
		BUTTON_SetText(hdc, "PWMoff\r\nPWMoff");
		pwm1.enable = 0;
		pwm2.enable = 0;
		pwm_together = 4;
		break;
	default:
		BUTTON_SetText(hdc, "PWM1\r\nPWMoff");
		pwm1.enable = 1;
		pwm2.enable = 0;
		pwm_together = 1;
		break;
	}
	Ctrl_PWM(&pwm1);
	Ctrl_PWM(&pwm2);
}
void OnReversalClick(WM_MESSAGE * pMsg)
{
	static int pwm_reversal = 3333;
	struct ctrl_pwm pwm1,pwm2;
	BUTTON_Handle hdc;
	
	pwm1.ch = PWM_CH1;
	pwm2.ch = PWM_CH2;
	Get_PWM(&pwm1);
	Get_PWM(&pwm2);

	hdc = pMsg->hWinSrc;
	switch(pwm_reversal) {
	case 1:
		BUTTON_SetText(hdc, "PWM\r\nReversal");
		pwm1.reversal = 1;
		pwm2.reversal = 1;
		pwm_reversal = 2;
		break;
	default:
		BUTTON_SetText(hdc, "PWM\r\nNormal");
		pwm1.reversal = 0;
		pwm2.reversal = 0;
		pwm_reversal = 1;
		break;
	}

	pwm1.high = PWM_TICK_MAX - pwm1.high;
	pwm2.high = PWM_TICK_MAX - pwm2.high;
	// 如果为负数，则将PWM反转
	if (pwm1.high < 0) {
		pwm1.high = PWM_TICK_MAX >> 2;
	}
	if (pwm2.high < 0) {
		pwm2.high = PWM_TICK_MAX >> 2;
	}
	Ctrl_PWM(&pwm1);
	Ctrl_PWM(&pwm2);
}

struct pwm_array
{
	int tick;
	char *describe;
};
void OnWidthClick(WM_MESSAGE * pMsg)
{
	static int pwm_width = 1000;
	struct ctrl_pwm pwm1,pwm2;
	struct pwm_array array[] = {
		{1,"27ns"},
		{9,"250ns"},
		{18,"500ns"},
		{36,"1us"},
		{360,"10us"},
		{720,"20us"},

		// only for debug
		// {10000,"27ns"},
		// {900,"250ns"},
		// {1800,"500ns"},
		// {3600,"1us"},
		// {10000,"10us"},
		// {20000,"20us"},
	};
	BUTTON_Handle hdc;

	hdc = pMsg->hWinSrc;
	pwm1.ch = PWM_CH1;
	pwm2.ch = PWM_CH2;
	Get_PWM(&pwm1);
	Get_PWM(&pwm2);


	pwm_width++;
	if (pwm_width >= sizeof(array) / sizeof(struct pwm_array)) {
		pwm_width = 0;
	}

	if (pwm1.reversal == 1) {
		pwm1.high = PWM_TICK_MAX - array[pwm_width].tick;
		pwm2.high = PWM_TICK_MAX -  array[pwm_width].tick;
	}
	else {
		pwm1.high = array[pwm_width].tick;
		pwm2.high = array[pwm_width].tick;
	}

	BUTTON_SetText(hdc, array[pwm_width].describe);
	// 如果为负数，则将PWM反转
	if (pwm1.high < 0) {
		pwm1.high = PWM_TICK_MAX >> 2;
	}
	if (pwm2.high < 0) {
		pwm2.high = PWM_TICK_MAX >> 2;
	}
	Ctrl_PWM(&pwm1);
	Ctrl_PWM(&pwm2);
}

void OnLaserClick(WM_MESSAGE * pMsg)
{
	char strout[200];
	BUTTON_Handle hdc;

	hdc = pMsg->hWinSrc;
	BUTTON_GetText(hdc,strout,200);

	switch(strout[3]) {
	case 'L':
		BUTTON_SetText(hdc, "LD MV");
		Ctrl_LaserPower(CTRL_LASER_MV);
		break;
	case 'M':
		BUTTON_SetText(hdc, "LD HV");
		Ctrl_LaserPower(CTRL_LASER_HV);
		break;
	default:// 'H':
		BUTTON_SetText(hdc, "LD LV");
		Ctrl_LaserPower(CTRL_LASER_LV);
		break;
	}
}

static const GUI_WIDGET_CREATE_INFO _aDialogCreatemsg[] = {
	{ FRAMEWIN_CreateIndirect,	"Owner drawn list box",	0,					0,	0, 120, 120 , FRAMEWIN_CF_MOVEABLE },
	{BUTTON_CreateIndirect,	 "ON/OFsss",					GUI_ID_PP,	 22,15,36,36},
};

static void _cbCallback2(WM_MESSAGE * pMsg) {
	char strout[222];
	static GUI_RECT rect;
	int NCode, Id;
	static int x0,y0 = 0;
	int x1,y1;
	static int fpress = 0;
	int press = 0;
	GUI_PID_STATE state;

	WM_HWIN hDlg, hListBox, hItem;
	hDlg = pMsg->hWin;
	hListBox = WM_GetDialogItem(hDlg, GUI_ID_MULTIEDIT0);


	switch (pMsg->MsgId) {

	case WM_INIT_DIALOG:
		// // hButton1 = WM_GetDialogItem(pMsg->hWin, GUI_ID_BUTTON0);
		// // BUTTON_SetText(hButton1, "dfwer");
		// // GUI_SetFont(&GUI_Font8x10_ASCII);
		// hFrame = WM_GetDialogItem(pMsg->hWin, 0);
		// FRAMEWIN_SetTitleVis(pMsg->hWin, 0);
		// FRAMEWIN_SetClientColor(pMsg->hWin, RGB(255,0,0));
		// FRAMEWIN_SetBorderSize(pMsg->hWin, 0);
		// Init_Ctrl(pMsg);

		break;
	default:
		WM_DefaultProc(pMsg);
	}
}
WM_HWIN hWin = 0;
WM_HWIN hWinTest = 0;
void OnSettingClick(WM_MESSAGE * pMsg)
{
	// int i;
	// char strout[220];
	// struct point pt;
	if (hWin != 0) {
		WM_ShowWindow(hWin);
		return ;
	}

	Delay_ms(300);
	hWin = TPAdjustDlg_Create(pMsg->hWin);
	WM_HideWindow(pMsg->hWin);
	// // TC_Adj();
	// TC_Test();
	
	WM_ShowWindow(pMsg->hWin);
	// WM_InvalidateWindow(pMsg->hWin);
	// WM_SendMessage(WM_PAINT, pMsg);
	
	// WM_DeleteWindow(hWin);
	printf("are you look me ? %x\r\n",hWin);
	


}
void OnDeleteClick(WM_MESSAGE * pMsg)
{
	if (hWinTest != 0) {
		WM_ShowWindow(hWinTest);
		return ;
	}
	hWinTest = TPTestDlg_Create(pMsg->hWin);	
}
void Init_Ctrl(WM_MESSAGE * pMsg)
{
	WM_HWIN hDlg;
	WM_MESSAGE msg;

	hDlg = pMsg->hWin;

	msg.hWinSrc = WM_GetDialogItem(hDlg, GUI_ID_PP);
	OnPerPowerClick(&msg);
	msg.hWinSrc = WM_GetDialogItem(hDlg, GUI_ID_APD);
	OnAPDClick(&msg);
	msg.hWinSrc = WM_GetDialogItem(hDlg, GUI_ID_PWM);
	OnPWMClick(&msg);
	msg.hWinSrc = WM_GetDialogItem(hDlg, GUI_ID_PWM_REVERSAL);
	OnReversalClick(&msg);
	msg.hWinSrc = WM_GetDialogItem(hDlg, GUI_ID_PWM_WIDTH);
	OnWidthClick(&msg);
	msg.hWinSrc = WM_GetDialogItem(hDlg, GUI_ID_LASTER);
	OnLaserClick(&msg);
}
// ***************************************************************************

/*********************************************************************
*
*		 _cbCallback
*/
static void _cbCallback(WM_MESSAGE * pMsg) {
	char strout[222];
	static GUI_RECT rect;
	int NCode, Id;
	static int x0,y0 = 0;
	int x1,y1;
	static int fpress = 0;
	int press = 0;
	GUI_PID_STATE state;

	WM_HWIN hDlg, hListBox, hItem;
	hDlg = pMsg->hWin;
	hListBox = WM_GetDialogItem(hDlg, GUI_ID_MULTIEDIT0);


	switch (pMsg->MsgId) {
	case WM_KEY:
		Id = WM_GetId(pMsg->hWinSrc);

		switch (Id) {
		case GUI_ID_PP:
			OnPerPowerClick(pMsg);
			break;
		case GUI_ID_APD:
			OnAPDClick(pMsg);
			break;
		case GUI_ID_PWM:
			OnPWMClick(pMsg);
			break;
		case GUI_ID_PWM_REVERSAL:
			OnReversalClick(pMsg);
			break;
		case GUI_ID_PWM_WIDTH:
			OnWidthClick(pMsg);
			break;
		case GUI_ID_LASTER:
			OnLaserClick(pMsg);
			break;
		case GUI_ID_SETTING:
			OnSettingClick(pMsg);
		default:
			break;
		}
		
		// hButton1 = WM_GetDialogItem(pMsg->hWin, GUI_ID_BUTTON0);
		// hButton1 = pMsg->hWinSrc;
		// // BUTTON_SetText(hButton1, "sdfffr");
		// BUTTON_GetText(hButton1,strout,222);
		// if (strout[0] == 'a') {
		// 	BUTTON_SetText(hButton1, "bbbb");
		// }
		// else {
		// 	BUTTON_SetText(hButton1, "aaaa");
		// }
		// printf(".............................\r\n");
		break;
	case WM_INIT_DIALOG:
		// hButton1 = WM_GetDialogItem(pMsg->hWin, GUI_ID_BUTTON0);
		// BUTTON_SetText(hButton1, "dfwer");
		// GUI_SetFont(&GUI_Font8x10_ASCII);
		hFrame = WM_GetDialogItem(pMsg->hWin, 0);
		FRAMEWIN_SetTitleVis(pMsg->hWin, 0);
		FRAMEWIN_SetClientColor(pMsg->hWin, RGB(255,0,0));
		FRAMEWIN_SetBorderSize(pMsg->hWin, 0);
		Init_Ctrl(pMsg);
	case WM_TOUCH:
		break;

	case WM_NOTIFY_PARENT:
		Id = WM_GetId(pMsg->hWinSrc);
		NCode = pMsg->Data.v;		
		switch(NCode) {
		case WM_NOTIFICATION_CLICKED:

			Id = WM_GetId(pMsg->hWinSrc);
			WM_SetFocus(pMsg->hWinSrc);
			switch (Id) {
			case GUI_ID_PP:
				OnPerPowerClick(pMsg);
				break;
			case GUI_ID_APD:
				OnAPDClick(pMsg);
				break;
			case GUI_ID_PWM:
				OnPWMClick(pMsg);
				break;
			case GUI_ID_PWM_REVERSAL:
				OnReversalClick(pMsg);
				break;
			case GUI_ID_PWM_WIDTH:
				OnWidthClick(pMsg);
				break;
			case GUI_ID_LASTER:
				OnLaserClick(pMsg);
				break;
			case GUI_ID_SETTING:
				OnSettingClick(pMsg);
				break;
			case GUI_ID_DELETE:
				OnDeleteClick(pMsg);
				break;
			default:
				break;
			}
			break;
			hButton1 = WM_GetDialogItem(pMsg->hWin, Id);
			BUTTON_SetText(hButton1, "Click");	

			break;
		case WM_NOTIFICATION_GOT_FOCUS:
			hButton1 = WM_GetDialogItem(pMsg->hWin, Id);
			BUTTON_SetTextColor(hButton1, 0, RGB(255,0,0));
			break;
		case WM_NOTIFICATION_LOST_FOCUS:
			hButton1 = WM_GetDialogItem(pMsg->hWin, Id);
			if (sg_bppdown == 0) {
				
				BUTTON_SetTextColor(hButton1, 0, RGB(0,0,0));	
			}
			else {
				BUTTON_SetTextColor(hButton1, 0, RGB(255,255,255));		
			}
			
			break;
		}
		break;




		GUI_PID_GetState(&state);

		if (state.Pressed == 1) {
			if (fpress == 0) {
				fpress = 1;
				y0 = state.y;
				WM_GetWindowRectEx(pMsg->hWin, &rect);
				break;
			}
			else {
				WM_MoveTo(pMsg->hWin, 0, rect.y0 + state.y - y0 );
			}
			sprintf(strout,"%d %d[%d] [%d] %d\n%d %d %d",state.x,state.y,state.Pressed, fpress,
				rect.y0 + state.y - y0 ,
				rect.y0, state.y , y0);
			GUI_DispStringAt(strout, 0, 10); // 显示文本
		}
		else {
			fpress = 0;
			TurnBack(pMsg->hWin);
		}



		break;
	default:
		WM_DefaultProc(pMsg);
	}
}

// static void _cbCallback(WM_MESSAGE* pMsg) {
//	 char strout[256];
//	 int Id,NCode;
//	 static int x0,y0;
//	 int x1,y1;
//	 switch (pMsg->MsgId) {
//	 case WM_PAINT:
//		 GUI_SetBkColor(GUI_RED);
//		 break;
//	 case WM_INIT_DIALOG:
//		 hButton1 = WM_GetDialogItem(pMsg->hWin, GUI_ID_BUTTON0);
//		 BUTTON_SetText(hButton1, "dfwer");
//		 hFrame = WM_GetDialogItem(pMsg->hWin, 0);
//		 FRAMEWIN_SetTitleVis(pMsg->hWin, 0);
//		 FRAMEWIN_SetClientColor(pMsg->hWin, RGB(255,0,0));
//		 FRAMEWIN_SetBorderSize(pMsg->hWin, 0);
//		 break;
//	 case WM_NOTIFY_PARENT:
//		 Id = WM_GetId(pMsg->hWinSrc);
//		 NCode = pMsg->Data.v;		
//		 switch(NCode) {
//		 case WM_NOTIFICATION_CLICKED:
//			 hButton1 = WM_GetDialogItem(pMsg->hWin, Id);
//			 BUTTON_SetText(hButton1, "Click");	
//			 break;
//		 case WM_NOTIFICATION_GOT_FOCUS:
//			 hButton1 = WM_GetDialogItem(pMsg->hWin, Id);
//			 // GUI_TOUCH_GetUnstable(&x0,&y0);
//			 BUTTON_SetText(hButton1, "get focus");	
//			 break;
//		 case WM_NOTIFICATION_LOST_FOCUS:
//			 hButton1 = WM_GetDialogItem(pMsg->hWin, Id);
//			 BUTTON_SetText(hButton1, "lose focus"); 
//			 break;
//		 }
//	 case WM_TOUCH_CHILD:
//		 Id = WM_GetId(pMsg->hWinSrc);
//		 GUI_TOUCH_GetUnstable(&x1,&y1);
//		 // WM_MoveWindow(pMsg->hWin, 0,y1 - y0);
//		 sprintf(strout, "y0 %d y1 %d y1-y0 %d", y0, y1, y1 - y0);
//		 hButton1 = WM_GetDialogItem(pMsg->hWin, GUI_ID_BUTTON0);
//		 BUTTON_SetText(hButton1, strout); 
//		 break;
//	 case WM_TOUCH:
//		 GUI_TOUCH_GetUnstable(&x1,&y1);
//		 if (x1 != -1) {
//			 // WM_MoveTo(pMsg->hWin, 0,0 + y1 - 20);
//		 }
//		 else {
//			 // TurnBack(pMsg->hWin);
//		 }
//		 break;
//		 // Id = WM_GetId(pMsg->hWinSrc);
//		 // GUI_TOUCH_GetUnstable(&x1,&y1);
//		 // WM_MoveTo(pMsg->hWin, 0,y1-5);
//		 // sprintf(strout, "%4.4d %4.4d", x1,y1);
//		 // hButton1 = WM_GetDialogItem(pMsg->hWin, GUI_ID_BUTTON0);
//		 // BUTTON_SetText(hButton1, strout); 
//	 case WM_CREATE:

//	 break;
//	 default:
//		 WM_DefaultProc(pMsg);
//	 }
//	 GUI_TOUCH_GetUnstable(&x1,&y1);

// }

void MainTask()
{

	// static const GUI_ConstString _ListBox[] = {
	// "English", "Deutsch", "1", "2", "231233", "sdfsf",NULL
	// };
	// hFrame = FRAMEWIN_Create("test",0,WM_CF_SHOW,50,0,150,100); 
	// WM_Paint(hFrame); 
	// hButton1 = BUTTON_CreateAsChild(20,20,60,30,hFrame,1,WM_CF_SHOW); 
	// BUTTON_SetText(hButton1,"OK"); 
	// WM_Paint(hButton1);
	// hListbox = LISTBOX_CreateAsChild(_ListBox, hFrame, 20, 60, 100, 80, WM_CF_SHOW);
	// WM_Paint(hListbox); 

	// FRAMEWIN_SetBarColor(hFrame,0,RGB(0,255,0));
	// FRAMEWIN_SetBarColor(hFrame,1,RGB(0,0,255));
	// // FRAMEWIN_SetTitleVis(hFrame, 0);
	// hFrame = FRAMEWIN_Create("test",0,WM_CF_SHOW,0,50,200,150); 
	// WM_Paint(hFrame); 
	// hButton1 = BUTTON_CreateAsChild(40,20,100,50,hFrame,1,WM_CF_SHOW); 
	// BUTTON_SetText(hButton1,"OK1112318278917092jasuoriuwoemrlaiuseoriw"); 


	// FRAMEWIN_SetBarColor(hFrame,0,RGB(0,255,0));
	// FRAMEWIN_SetBarColor(hFrame,1,RGB(0,0,255));
	// // FRAMEWIN_SetTitleVis(hFrame, 0);
	// WM_Paint(hFrame); 
	// WM_Paint(hButton1);
	// WM_SetDesktopColor(RGB(255,0,0));
	// WM_SetCallback(hFrame, &_cbBkWindow);

	// WM_SetDesktopColor(RGB(255,0,0));
	// hFrame = FRAMEWIN_Create("test",0,WM_CF_SHOW,0,0,320,240); 
	FRAMEWIN_SetTitleVis(hFrame, 0);
	WM_SetCallback(WM_HBKWIN, &_cbBkWindow);
	hdlg = GUI_CreateDialogBox(_aDialogCreate, GUI_COUNTOF(_aDialogCreate), &_cbCallback, (WM_HWIN)WM_HBKWIN, 0, 0); 
}


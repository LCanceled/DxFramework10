
#ifndef DXUTD3DAPP_H
#define DXUTD3DAPP_H

#include "DxUtInclude.h"
#include "DxUtEffectPool.h"

namespace DxUt {

class CD3DApp {
private:
	HINSTANCE m_hInst;
	TCHAR * m_szWndClassName;
	TCHAR * m_szWndTitleText;
	WNDPROC m_WndProc;
	WORD m_wWndPosX, m_wWndPosY;
	BOOL m_bFullscreen;
	__int64 m_liCountNum;
	__int64 m_liLastCountNum; 
	BOOL m_bPaused;

	LPDIRECTINPUT8 m_pDinput;
	LPDIRECTINPUTDEVICE8 m_pKeyboard;
	LPDIRECTINPUTDEVICE8 m_pMouse;

	void (*m_fpnOnResizeWindow)();

	/* Pools initialization */
	CEffectPool * m_pEffectPool;

	/* Consle initialization */
	HANDLE m_hConsleOutput;

	void WindowResize();
	void WindowFullScreen();
	void CalculateTime();
	void PollKeyboard();
	void PollMouse();
	friend LRESULT CALLBACK WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);
public:
	CD3DApp(HINSTANCE hInst, TCHAR * szClassName, TCHAR * szTitleText,
		WORD wWndPosX, WORD wWndPosY, WORD wWndWidth, WORD wWndHeight, void (*onResizeWindowFunction)());
	CD3DApp(CD3DApp & cpy); /* Copy constructor should never be called */
	~CD3DApp() {}

	void Loop(void(*loopFunction)());

	BOOL GetPaused() {return m_bPaused; }
	void SetPaused(BOOL pause) {m_bPaused = pause; }

	BOOL IsFullscreen() {return m_bFullscreen; }
	void ToggleFullscreen(BOOL f) {m_bFullscreen = f; }

	void Print(float flt) {
		char buf[256]; 
		sprintf(buf, "%f\n", flt); Print(buf); 
	}
	void Print(int iNum) {
		char buf[256]; 
		sprintf(buf, "%i\n", iNum); Print(buf); 
	}
	void Print(DWORD dwNum) {
		char buf[256]; 
		sprintf(buf, "%u\n", (int)dwNum); Print(buf); 
	}
	void Print(char * szStr) {
		DWORD written; 
		WriteConsoleA(m_hConsleOutput, szStr, strlen(szStr), &written, 0); 
	}

	void DestroyD3DApp();
};


};

#endif
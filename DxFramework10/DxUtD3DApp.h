
#ifndef DXUTD3DAPP_H
#define DXUTD3DAPP_H

#include "DxUtApp.h"

namespace DxUt {

class CEffectPool;
class CMeshPool;
class CCollisionGraphics;

class CD3DApp : public CApp {
private:
	HINSTANCE m_hInst;
	TCHAR * m_szWndClassName;
	TCHAR * m_szWndTitleText;
	WNDPROC m_WndProc;
	WORD m_wWndPosX, m_wWndPosY;
	BOOL m_bFullscreen; 

	LPDIRECTINPUT8 m_pDinput;
	LPDIRECTINPUTDEVICE8 m_pKeyboard;
	LPDIRECTINPUTDEVICE8 m_pMouse;

	/* Pools initialization */
	CEffectPool * m_pEffectPool;
	CMeshPool * m_pMeshPool;

	CCollisionGraphics * m_pCollisionGraphics;

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
		WORD wWndPosX, WORD wWndPosY, WORD wWnuiIdth, WORD wWndHeight, void (*onResizeWindowFunction)());
	CD3DApp(CD3DApp & cpy); /* Copy constructor should never be called */
	~CD3DApp() {}

	void Loop(void(*loopFunction)());

	BOOL GetPaused() {return m_bPaused; }
	void SetPaused(BOOL pause) {m_bPaused = pause; }

	void Print(float val, char * str="\n") {
		char buf[256]; 
		sprintf(buf, "%f%s\0", val, str); Print(buf); 
	}
	void Print(double val, char * str="\n") {
		char buf[256]; sprintf(buf, "%f%s", val, str); Print(buf); 
	}
	void Print(int val, char * str="\n") {
		char buf[256]; 
		sprintf(buf, "%i%s\0", val, str); Print(buf); 
	}
	void Print(UINT val, char * str="\n") {
		char buf[256]; 
		sprintf(buf, "%u%s\0", val, str); Print(buf); 
	}
	void Print(char * str) {
		DWORD dwWritten; 
		WriteConsoleA(m_hConsleOutput, str, strlen(str), &dwWritten, 0); 
	}
	
	/* Misc utility functions */

	//Reset the window render target and depth stencil to the global ones
	void ResetRenderTargetAndView();

	void ExtractPixelsFromImageFile(CHAR * szImageFile, void ** ppData,
		UINT uiStride, UINT * puiImageWidth=0, UINT * puiImageHeight=0);

	void Destroy();
};


};

#endif
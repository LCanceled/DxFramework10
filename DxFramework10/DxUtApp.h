
#ifndef DXUTAPP_H
#define DXUTAPP_H

#include "DxUtInclude.h"

namespace DxUt {

class CApp {
protected:
	friend LRESULT CALLBACK WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

	void (*m_fpnOnResizeWindow)();

	virtual void WindowResize()=0;
	virtual void WindowFullScreen()=0;
	
	bool m_bPaused;
	bool m_bFullscreen;
public:
	CApp():m_bPaused(0),m_bFullscreen(0) {}
	virtual ~CApp() {};

	virtual void Loop(void(*loopFunction)())=0;

	bool GetPaused() {return m_bPaused; }
	void SetPaused(BOOL pause) {m_bPaused = pause; }

	bool IsFullscreen() {return m_bFullscreen; }
	void ToggleFullscreen(bool f) {m_bFullscreen = f; }

	virtual void Print(float flt, char * str="\n")=0;
	virtual void Print(double dbl, char * str="\n")=0;
	virtual void Print(int iNum, char * str="\n")=0;
	virtual void Print(UINT uiNum, char * str="\n")=0;
	virtual void Print(char * szStr)=0;

	virtual void Destroy()=0;
};


};


#endif


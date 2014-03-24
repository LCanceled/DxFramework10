
#ifndef DXUTAPP_H
#define DXUTAPP_H

#include "DxUtInclude.h"

namespace DxUt {

class CApp {
protected:
	//CTimer m_Timer;
	void (*m_fpnOnResizeWindow)();

	bool m_bPaused;
public:
	CApp() {}
	virtual ~CApp()=0;

	virtual void Loop(void(*loopFunction)())=0;

	bool GetPaused() {return m_bPaused; }
	void SetPaused(BOOL pause) {m_bPaused = pause; }

	virtual void Print(float flt)=0;
	virtual void Print(int iNum)=0;
	virtual void Print(UINT uiNum)=0;
	virtual void Print(char * szStr)=0;

	virtual void Destroy()=0;
};


};


#endif



#ifndef DXUTGLAPP_H
#define DXUTGLAPP_H

#include "DxUtApp.h"

namespace DxUt {

class CGLApp : public CApp {
protected:
	
public:
	CGLApp() {}
	~CGLApp() {}

	void Loop(void(*loopFunction)()) {}

	void Print(float flt) {};
	void Print(int iNum) {};
	void Print(UINT uiNum) {};
	void Print(char * szStr) {};

	void Destroy() {};
};


};


#endif
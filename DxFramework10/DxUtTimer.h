
#ifndef DXUTTIMER_H
#define DXUTTIMER_H

#include "DxUtInclude.h"
#include <iostream>
#include <fstream>

#define MAX_TIMERS 64

namespace DxUt {

class CTimer {
private:
	struct STimeInfo {
		STimeInfo():elapsedTime(0), totalTime(0), dwCount(0) {str[0] = '\0'; }
		
		__int64 liCountNum;
		DWORD dwCount;
		double elapsedTime;
		double totalTime;
		char str[256];
	};

	static STimeInfo m_rgTimer[MAX_TIMERS];
	static DWORD m_dwTimerCount;
	__int64 m_liCountsPerSecond;
	static DWORD m_dwCount;
public:
	CTimer(DWORD dwCount) {
		QueryPerformanceFrequency((LARGE_INTEGER*)&m_liCountsPerSecond);
	}
	//~CTimer() {}

	void StartTimer(DWORD dwTimer) {
		QueryPerformanceCounter((LARGE_INTEGER*)&m_rgTimer[dwTimer].liCountNum);
	}
	void EndTimer(DWORD dwTimer) {
		__int64 liCountNum; 
		QueryPerformanceCounter((LARGE_INTEGER*)&liCountNum);
		__int64 deltaCount = liCountNum - m_rgTimer[dwTimer].liCountNum;
		m_rgTimer[dwTimer].elapsedTime += (double)deltaCount;
		m_rgTimer[dwTimer].totalTime += (double)deltaCount;
		m_rgTimer[dwTimer].dwCount++;
	}
	void DisplayTimer(DWORD dwTimer, const char * str);

	void PresentTimer(DWORD dwPresentationInterval);
};


};


#endif


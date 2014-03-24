
#ifndef DXUTTIMER_H
#define DXUTTIMER_H

#include <Windows.h>
#include <assert.h>

#define MAX_TIMERS 64

namespace DxUt {

class CTimer {
private:
	struct STimeInfo {
		STimeInfo():elapsedTime(0), totalTime(0), dwTotalCount(0), dwCountSinceLastPresentation(0), str(0) {}
		
		__int64 liCountNum;
		DWORD dwTotalCount;
		DWORD dwCountSinceLastPresentation;
		double elapsedTime;
		double totalTime;
		char * str;
	};

	static STimeInfo m_Timers[MAX_TIMERS];
	__int64 m_liCountsPerSecond;
	
	CTimer() {
		QueryPerformanceFrequency((LARGE_INTEGER*)&m_liCountsPerSecond);
	}
	//~CTimer() {}

public:

	/* Do not nest start and end calls */
	void StartTimer(DWORD dwTimer) {
		assert(dwTimer < MAX_TIMERS);

		QueryPerformanceCounter((LARGE_INTEGER*)&m_Timers[dwTimer].liCountNum);
		m_Timers[dwTimer].elapsedTime = 0;
	}
	void EndTimer(DWORD dwTimer, char * str) {
		assert(dwTimer < MAX_TIMERS);

		__int64 liCountNum; 
		QueryPerformanceCounter((LARGE_INTEGER*)&liCountNum);
		__int64 deltaCount = liCountNum - m_Timers[dwTimer].liCountNum;
		m_Timers[dwTimer].elapsedTime += (double)deltaCount;
		m_Timers[dwTimer].totalTime += (double)deltaCount;
		m_Timers[dwTimer].dwTotalCount++;
		m_Timers[dwTimer].dwCountSinceLastPresentation++;
		m_Timers[dwTimer].str = str;
	}

	void PresentTimers(DWORD dwPresentationInterval, bool bPresentAvg);

	static CTimer Get() {static CTimer c; return c; }
};


};


#endif



#ifndef DXUTTIMER_H
#define DXUTTIMER_H

#include <Windows.h>
#include <assert.h>

#define MAX_TIMERS 64

namespace DxUt {

class CTimer {
private:
	struct STimeInfo {
		STimeInfo():bTimerInUse(0), elapsedTime(0), totalTime(0), dwTotalCount(0), dwCountSinceLastPresentation(0), str(0) {}
		
		bool bTimerInUse;
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
		memset(m_Timers, 0, sizeof(STimeInfo)*MAX_TIMERS);
	}
	//~CTimer() {}

public:

	/* Do not nest start and end calls */
	void StartTimer(DWORD dwTimer) {
		assert(dwTimer < MAX_TIMERS);
		assert(!m_Timers[dwTimer].bTimerInUse);

		QueryPerformanceCounter((LARGE_INTEGER*)&m_Timers[dwTimer].liCountNum);
		m_Timers[dwTimer].elapsedTime = 0;
		m_Timers[dwTimer].bTimerInUse = 1;
	}
	void EndTimer(DWORD dwTimer, char * str) {
		assert(dwTimer < MAX_TIMERS);
		assert(m_Timers[dwTimer].bTimerInUse);

		__int64 liCountNum; 
		QueryPerformanceCounter((LARGE_INTEGER*)&liCountNum);
		__int64 deltaCount = liCountNum - m_Timers[dwTimer].liCountNum;
		m_Timers[dwTimer].elapsedTime += (double)deltaCount;
		m_Timers[dwTimer].totalTime += (double)deltaCount;
		m_Timers[dwTimer].dwTotalCount++;
		m_Timers[dwTimer].dwCountSinceLastPresentation++;
		m_Timers[dwTimer].str = str;
		m_Timers[dwTimer].bTimerInUse = 0;
	}

	void PresentTimers(DWORD dwPresentationInterval, bool bPresentAvg);

	double GetElapsedTime(DWORD dwTimer) {
		assert(dwTimer < MAX_TIMERS);
		assert(!m_Timers[dwTimer].bTimerInUse);
		return m_Timers[dwTimer].elapsedTime/m_liCountsPerSecond; 
	}
	double GetTotalTime(DWORD dwTimer) {
		assert(dwTimer < MAX_TIMERS);
		assert(!m_Timers[dwTimer].bTimerInUse);
		return m_Timers[dwTimer].totalTime/m_liCountsPerSecond;
	}

	static CTimer Get() {static CTimer c; return c; }
};


};


#endif


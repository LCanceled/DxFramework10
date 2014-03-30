
#ifndef DXUTTIMER_H
#define DXUTTIMER_H

#include <Windows.h>
#include <assert.h>

#define MAX_TIMERS 64

namespace DxUt {

class CTimer {
private:
	struct STimeInfo {
		STimeInfo():bTimerInUse(0), elapsedTime(0), totalTime(0), uiTotalCount(0), uiCountSinceLastPresentation(0), str(0) {}
		
		bool bTimerInUse;
		__int64 liCountNum;
		UINT uiTotalCount;
		UINT uiCountSinceLastPresentation;
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
	void StartTimer(UINT uiTimer) {
		assert(uiTimer < MAX_TIMERS);
		assert(!m_Timers[uiTimer].bTimerInUse);

		QueryPerformanceCounter((LARGE_INTEGER*)&m_Timers[uiTimer].liCountNum);
		m_Timers[uiTimer].elapsedTime = 0;
		m_Timers[uiTimer].bTimerInUse = 1;
	}
	void EndTimer(UINT uiTimer, char * str) {
		assert(uiTimer < MAX_TIMERS);
		assert(m_Timers[uiTimer].bTimerInUse);

		__int64 liCountNum; 
		QueryPerformanceCounter((LARGE_INTEGER*)&liCountNum);
		__int64 deltaCount = liCountNum - m_Timers[uiTimer].liCountNum;
		m_Timers[uiTimer].elapsedTime += (double)deltaCount;
		m_Timers[uiTimer].totalTime += (double)deltaCount;
		m_Timers[uiTimer].uiTotalCount++;
		m_Timers[uiTimer].uiCountSinceLastPresentation++;
		m_Timers[uiTimer].str = str;
		m_Timers[uiTimer].bTimerInUse = 0;
	}

	void PresentTimers(UINT uiPresentationInterval, bool bPresentAvg);

	double GetElapsedTime(UINT uiTimer) {
		assert(uiTimer < MAX_TIMERS);
		assert(!m_Timers[uiTimer].bTimerInUse);
		return m_Timers[uiTimer].elapsedTime/m_liCountsPerSecond; 
	}
	double GetTotalTime(UINT uiTimer) {
		assert(uiTimer < MAX_TIMERS);
		assert(!m_Timers[uiTimer].bTimerInUse);
		return m_Timers[uiTimer].totalTime/m_liCountsPerSecond;
	}

	static CTimer Get() {static CTimer c; return c; }
};


};


#endif


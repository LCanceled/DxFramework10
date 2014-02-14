
#include "DxUtTimer.h"
#include "DxUtD3DApp.h"

namespace DxUt {

DWORD CTimer::m_dwTimerCount = 0;
DWORD CTimer::m_dwCount = 0;
CTimer::STimeInfo CTimer::m_rgTimer[MAX_TIMERS];

void CTimer::DisplayTimer(DWORD dwTimer, const char * str)
 {
	if (1) {//m_rgTimer[dwTimer].dwCount > m_dwCount) {
		char buf[256];
		//itoa((int)(m_rgTimer[dwTimer].elapsedTime), buf, 10);
		//itoa((int)(m_rgTimer[dwTimer].totalTime/m_rgTimer[dwTimer].dwCount), buf, 10);
		sprintf(buf, "%f.4", (m_rgTimer[dwTimer].totalTime/m_rgTimer[dwTimer].dwCount)/(double)m_liCountsPerSecond);
		int len = strlen(buf);
		buf[len] = '\n';
		buf[len+1] = ' ';
		buf[len+2] = '\0';
		if (str != NULL) {
			buf[len] = ' ';
			buf[len+1] = '\0';
			strcat(buf, str);
			len = strlen(buf);
			buf[len] = '\n';
			buf[len+1] = '\0';
		}
		strcpy(m_rgTimer[dwTimer].str, buf);

		m_rgTimer[dwTimer].elapsedTime = 0;
	} else {
		//m_rgTimer[dwTimer].dwCount++;
	}
}

void CTimer::PresentTimer(DWORD dwPresentationInterval)
{
	if (m_dwCount++ >= dwPresentationInterval) {
		for (DWORD i=0; i<MAX_TIMERS; i++) {
			if (m_rgTimer[i].str[0] != '\0') g_D3DApp->Print(m_rgTimer[i].str);
		}
		m_dwCount = 0;
	}
}







};
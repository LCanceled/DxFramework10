
#include "DxUtTimer.h"
#include "DxUtD3DApp.h"

namespace DxUt {

CTimer::STimeInfo CTimer::m_Timers[MAX_TIMERS];

void CTimer::PresentTimers(UINT uiPresentationInterval, bool bPresentAvg)
{
	int bPresented = 0;
	for (UINT i=0; i<MAX_TIMERS; i++) {
		if (m_Timers[i].str != NULL && m_Timers[i].uiCountSinceLastPresentation >= uiPresentationInterval) {
			char buf[256];
			if (bPresentAvg) {
				sprintf(buf, "\n%s: %f.4", m_Timers[i].str, (m_Timers[i].totalTime/m_Timers[i].uiTotalCount)/(double)m_liCountsPerSecond);
			} else {
				sprintf(buf, "\n%s: %f.4", m_Timers[i].str, (m_Timers[i].elapsedTime)/(double)m_liCountsPerSecond);
			}
			g_App->Print(buf);
			m_Timers[i].uiCountSinceLastPresentation = 0;
			bPresented++;
		}
	}
	if (bPresented) {
		g_App->Print("\n\n");
	}
}







};
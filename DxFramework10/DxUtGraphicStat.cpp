	
#include "DxUtGraphicStat.h"
#include "DxUtInclude.h"

namespace DxUt {

GraphicStat::GraphicStat():m_NumVertices(0), m_NumTriangles(0),
	m_FrameRate(0), m_TimeElapsed(0), m_FrameCount(0) 
{
}

void GraphicStat::CreateGraphics(float size, D3DXCOLOR * color)
{
	m_TextColor = *color;
	m_Text.CreateText(size, 400, 0, "Times New Roman");
}

void GraphicStat::UpdateStatus()
{
	m_TimeElapsed += g_SPFrame;
	m_FrameCount++;

	if (m_TimeElapsed >= 1.f) {
		m_FrameRate = m_FrameCount;
		m_TimeElapsed = 0;
		m_FrameCount = 0;
	}
}

void GraphicStat::DrawFrameRate(float perWndWidth, float perWndHeight)
{
	CHAR text[20];
	sprintf_s(text, 20, "FrameRate: %d", m_FrameRate);

	FltRect rect(perWndWidth, perWndHeight, 0, 0);
	m_Text.DrawText(text, rect, DT_NOCLIP, m_TextColor);
}

void GraphicStat::DrawAllStatus(float perWndWidth, float perWndHeight)
{
	CHAR text[100];
	sprintf_s(text, 100, "Vertices: %d\nTriangles: %d\nSPerFrame: %.4f\nFrameRate: %d",
		m_NumVertices, m_NumTriangles, g_SPFrame, m_FrameRate);

	FltRect rect(perWndWidth, perWndHeight, 0, 0);
	m_Text.DrawText(text, rect, DT_NOCLIP, m_TextColor);
}

void GraphicStat::DestroyGraphics()
{	
	m_NumVertices = 0;
	m_NumTriangles = 0;

	m_TimeElapsed = 0;
	m_FrameCount = 0;
	m_FrameRate = 0;

	m_Text.DestroyText();
	m_TextColor = D3DXCOLOR(0xff000000);
}


};



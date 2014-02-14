
#ifndef GRAPHICSTAT_H
#define GRAPHICSTAT_H

#include "DxUtInclude.h"
#include "DxUtText.h"

namespace DxUt {

class GraphicStat {
private:
	DWORD m_NumVertices;
	DWORD m_NumTriangles;

	FLOAT m_TimeElapsed;
	WORD m_FrameCount; 
	DWORD m_FrameRate;

	Text m_Text;
	D3DXCOLOR m_TextColor;
public:
	GraphicStat();
	~GraphicStat() {}

	void CreateGraphics(float size, D3DXCOLOR * color=&D3DXCOLOR(0xff00ccaa));
	void AddVertices(DWORD numVert=1) {m_NumVertices += numVert;}
	void RemoveVertices(DWORD numVert=1) {m_NumVertices -= numVert;}
	void AddTriangles(DWORD numTri=1) {m_NumTriangles += numTri;}
	void RemoveTriangles(DWORD numTri=1) {m_NumTriangles -= numTri;}

	void UpdateStatus();

	void DrawFrameRate(float perWndWidth, float perWndHeight);
	void DrawAllStatus(float perWndWidth, float perWndHeight);

	void OnLostDevice() {m_Text.OnLostDevice();}
	void OnResetDevice() {m_Text.OnResetDevice();}

	DWORD & GetNumTriangles() {return m_NumTriangles;}
	DWORD & GetNumVertices() {return m_NumVertices;}
	DWORD & GetFramerate() {return m_FrameRate;}

	void DestroyGraphics();
};


};


#endif
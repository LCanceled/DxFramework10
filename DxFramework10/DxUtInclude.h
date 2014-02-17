   
#ifndef DXUTINCLUDE_H
#define DXUTINCLUDE_H

#if defined(DEBUG) | defined(_DEBUG)
#ifndef D3D_DEBUG_INFO
#define D3D_DEBUG_INFO
#endif

#pragma comment(lib, "d3d10.lib")
#pragma comment(lib, "d3dx10d.lib")
#pragma comment(lib, "dxgi.lib")
#pragma comment(lib, "dinput8.lib")
#pragma comment(lib, "dxguid.lib")
#pragma comment(lib, "WinMM.lib")
#else
#pragma comment(lib, "d3d10.lib")
#pragma comment(lib, "d3dx9.lib")
#pragma comment(lib, "d3dx10.lib")
#pragma comment(lib, "d3dxof.lib")
#pragma comment(lib, "dxgi.lib")
#pragma comment(lib, "dinput8.lib")
#pragma comment(lib, "dxguid.lib")
#pragma comment(lib, "WinMM.lib")
#endif

#include <d3d10.h>
#include <d3dx10.h>
#include <dxgi.h>
#define DIRECTINPUT_VERSION  0x0800
#include <dinput.h>
#include <crtdbg.h>
#include <stdio.h>
#include <time.h>
#include <wchar.h>
#include "DxUtError.h"
#include "DxUtVector.h"
#include "DxUtMatrix.h"

#define Abs(x) (((x) < 0) ? (-x) : (x))
#define Min(x, y) (((x) < (y)) ? (x) : (y))
#define Max(x, y) (((x) > (y)) ? (x) : (y))
template<typename T>
inline void Swap(T & x, T & y)
{
	T tmp(x);
	x = y;
	y = tmp;
}

#ifndef ReleaseX
#define ReleaseX(resource)				\
{										\
	if (resource) resource->Release();	\
	resource = NULL;					\
}
#endif

namespace DxUt {

//It is a requirement that g_D3DApp be declared and initialized
//as it will initialize and maintain the global variables below
extern class CD3DApp * g_D3DApp;

//The file directory for the program
extern CHAR				g_szFileDir[MAX_PATH];

//The window
extern HWND				g_hWnd;
extern	UINT			g_uiWndWidth;
extern	UINT			g_uiWndHeight;

//The time in s since the start of the program and the fps
extern	double			g_TimeElapsed;
extern	float			g_SPFrame;

//The state of the keyboard and the mouse
extern CHAR				g_KeysState[256];
extern DIMOUSESTATE		g_MouseState;

//Direct3D10 resources
extern IDXGISwapChain *				g_pSwapChain;		
extern ID3D10Device *				g_pD3DDevice;
extern ID3D10RenderTargetView *		g_pRenderTargetView;
extern ID3D10DepthStencilView *		g_pDepthStencilView;

//szDir and szFile in this order will be concatenated into szPath of size MAX_PATH
//void InsertDirectory(CHAR * szDir, CHAR * szFile, CHAR * szPath);

//szDir and szFile in this order will be concatenated into szPath of size MAX_PATH if
//the first character of szFile is a '/' otherwise szPath will be szFile
//void InsertDirectoryEx(CHAR * szDir, CHAR * szFile, CHAR * szPath);

void ChangeExtension(char * szStr, char * szExt, char * szNewStr);

/* 
 * Fast Rounding of Floating Point Numbers in C/C++ on Wintel Platform, Laurent de Soras,
 * <http://ldesoras.free.fr/doc/articles/rounding_en.pdf>
*/
inline int floorf_ASM(double x) 
{
	const float round_towards_m_i = -0.5f;
	int	i;
	__asm {
		fld   x
		fadd st, st (0)
		fadd round_towards_m_i
		fistp i
		sar   i, 1
	}
	return (i);
}

inline int ceilf_ASM(double x)
{
	const float round_towards_p_i = -0.5f;
	int i;
	__asm {
		fld   x
		fadd st, st (0)
		fsubr round_towards_p_i
		fistp i
		sar   i, 1
	}
	return (-i);
}

inline float fracf(const float a)
{
	return a - floorf_ASM(a);
}

inline float RoundToNearest(const float a)
{
	return (float)ceilf_ASM(a - .5f);
}



};


#endif

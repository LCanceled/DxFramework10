   
#ifndef DXUTINCLUDE_H
#define DXUTINCLUDE_H

#if defined(WIN32) | defined(_WINDOWS_) | defined(_WINDOWS)

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
#include <wchar.h>


#ifndef ReleaseX
#define ReleaseX(resource)				\
{										\
	if (resource) resource->Release();	\
	resource = NULL;					\
}
#endif


#else 
typedef unsigned int UINT;

struct DIMOUSESTATE {
    long    lX;
    long    lY;
    long    lZ;
    unsigned char rgbButtons[4];
};

#endif

#include <time.h>
#include "DxUtError.h"
#include "DxUtVector.h"
#include "DxUtMatrix.h"
#include "DxUtTimer.h"

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

namespace DxUt {

//It is a requirement that g_D3DApp be declared and initialized
//as it will initialize and maintain the global variables below
extern class CApp * g_App;

//The file directory for the program
extern CHAR				g_szFileDir[MAX_PATH];

//The window
extern HWND				g_hWnd;
extern	UINT			g_uiWnuiIdth;
extern	UINT			g_uiWndHeight;

//The time in s since the start of the program and the fps
extern	double			g_TimeElapsed;
extern	float			g_SPFrame;

//The state of the keyboard and the mouse
extern CHAR				g_KeysState[256];
extern DIMOUSESTATE		g_MouseState;

#if defined(WIN32) | defined(_WINDOWS_) | defined(_WINDOWS)

//Direct3D10 resources
extern IDXGISwapChain *				g_pSwapChain;		
extern ID3D10Device *				g_pD3DDevice;
extern ID3D10RenderTargetView *		g_pRenderTargetView;
extern ID3D10DepthStencilView *		g_pDepthStencilView;

#endif

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

inline float fastSqrt(const float & x)
{
	__m128 r0; 
	r0 = _mm_load_ss(&x);
	r0 = _mm_sqrt_ss(r0);

	float flt;
	_mm_store_ss(&flt, r0);

	return flt;
}




};


#endif

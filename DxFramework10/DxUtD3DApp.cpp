
#include "DxUtD3DApp.h"
#include "DxUtEffectPool.h"

namespace DxUt {

	/*
//Sets the global file directory from the current directory
void InitializeDxFramework()
{
}

//Sets the global file directory from globalFileDir
void InitializeDxFramework(CHAR * szGlobalFileDir)
{
	sprintf_s(g_szFileDir, "%s/", szGlobalFileDir);
}*/
	
LRESULT CALLBACK WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	RECT rect;
	switch(msg)
	{
	case WM_ACTIVATE:
		if (wParam == WA_ACTIVE || wParam == WA_CLICKACTIVE) //For some reason, wParam will become WA_ACTIVE
			g_D3DApp->SetPaused(FALSE);						 //when the window is minized and LOWORD(wParam) == WA_ACTIVE.
		else if (LOWORD(wParam) == WA_INACTIVE)				 //The problem is fixed by wParam == WA_ACTIVE.
			g_D3DApp->SetPaused(TRUE);

		return 0;

	case WM_SIZE:
		GetClientRect(hWnd, &rect);
		g_uiWndWidth = rect.right;
		g_uiWndHeight = rect.bottom;

		g_D3DApp->WindowResize();
		g_D3DApp->m_fpnOnResizeWindow();

		g_D3DApp->SetPaused(FALSE);

		return 0;

	case WM_COMMAND:
		return 0;

	case WM_KEYUP:
		if (wParam == VK_F2 && !g_D3DApp->IsFullscreen()) 
			g_D3DApp->WindowFullScreen();
		else if (wParam == VK_ESCAPE)
			PostQuitMessage(0);

		return 0;

	case WM_CLOSE:
	case WM_DESTROY:
		PostQuitMessage(0);
		return 0;

	default:
		return DefWindowProc(hWnd, msg, wParam, lParam);
	}
}

CD3DApp::CD3DApp(HINSTANCE hInst, TCHAR * szClassName, TCHAR * szTitleText,
		WORD wWndPosX, WORD wWndPosY, WORD wWndWidth, WORD wWndHeight, void (*onResizeWindowFunction)()):
	m_hInst(hInst), m_wWndPosX(wWndPosX), m_wWndPosY(wWndPosY), 	
	m_bFullscreen(0), m_bPaused(0), m_pDinput(0), m_pKeyboard(0), m_pMouse(0), m_fpnOnResizeWindow(onResizeWindowFunction)
{
	GetCurrentDirectoryA(MAX_PATH, g_szFileDir);

	g_uiWndWidth = wWndWidth;
	g_uiWndHeight = wWndHeight;

	//////////////////////////////
	//Window initialization
	DWORD len = wcslen(szClassName);
	m_szWndClassName = new TCHAR[len+1];
	wcscpy(m_szWndClassName, szClassName);

	len = wcslen(szTitleText);
	m_szWndTitleText = new TCHAR[len+1];
	wcscpy(m_szWndTitleText, szTitleText);

	WNDCLASSW wc;
	wc.hInstance = m_hInst;
	wc.lpfnWndProc = WndProc;
	wc.lpszClassName = m_szWndClassName;
	wc.hIcon = LoadIcon(NULL, IDI_APPLICATION);
	wc.hCursor = LoadCursor(NULL, IDC_ARROW);
	wc.hbrBackground = (HBRUSH)(BLACK_BRUSH);
	wc.style = CS_CLASSDC;
	wc.lpszMenuName = NULL;
	wc.cbClsExtra = 0;
	wc.cbWndExtra = 0;

	RegisterClassW(&wc);

	RECT rect = {0, 0, g_uiWndWidth, g_uiWndHeight};
	AdjustWindowRect(&rect, WS_CAPTION | WS_SYSMENU | WS_SIZEBOX | WS_MINIMIZEBOX, TRUE);
	g_hWnd = CreateWindow(m_szWndClassName, m_szWndTitleText, WS_CAPTION | WS_SYSMENU | WS_SIZEBOX | WS_MINIMIZEBOX,
		m_wWndPosX, m_wWndPosY, (rect.right - rect.left), (rect.bottom - rect.top), NULL, NULL, m_hInst, NULL);

	if (!g_hWnd) {
		DxUtSendError("Window could not be created."); }

	UpdateWindow(g_hWnd);

	//////////////////////////////
	//Direct3D10 initialization
		DXGI_SWAP_CHAIN_DESC desc;
	desc.BufferCount = 1;
	desc.BufferDesc.Width = g_uiWndWidth;
	desc.BufferDesc.Height = g_uiWndHeight;
	desc.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	desc.BufferDesc.RefreshRate.Denominator = 60;
	desc.BufferDesc.RefreshRate.Numerator = 1;
	desc.BufferDesc.ScanlineOrdering = DXGI_MODE_SCANLINE_ORDER_UNSPECIFIED;
	desc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
	desc.Flags = NULL;
	desc.OutputWindow = g_hWnd;
	desc.SampleDesc.Count = 1;
	desc.SampleDesc.Quality = 0;
	desc.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;
	desc.Windowed = TRUE;

	UINT flags = 0;
#if defined(DEBUG) || defined(_DEBUG)  
    flags |= D3D10_CREATE_DEVICE_DEBUG;
#endif

	if (FAILED(D3D10CreateDeviceAndSwapChain(NULL, D3D10_DRIVER_TYPE_HARDWARE, NULL, flags, D3D10_SDK_VERSION, &desc, &g_pSwapChain, &g_pD3DDevice))) {
		DxUtSendError("Direct3D device could not be created.");
	}

	WindowResize();

	if (FAILED(DirectInput8Create(m_hInst, DIRECTINPUT_VERSION, IID_IDirectInput8, (void**)&m_pDinput, NULL))) {
		DxUtSendError("Dinput8 could not be created."); }

	if (FAILED(m_pDinput->CreateDevice(GUID_SysKeyboard, &m_pKeyboard, NULL))) {
		DxUtSendError("Dinput8 could not be create keyboard."); }
	if (FAILED(m_pKeyboard->SetDataFormat(&c_dfDIKeyboard))) {
		DxUtSendError("Keyboard could not set format."); }
	if (FAILED(m_pKeyboard->SetCooperativeLevel(g_hWnd, DISCL_FOREGROUND | DISCL_NONEXCLUSIVE))) {
		DxUtSendError("Keyboard could not set cooperative level."); }
	m_pKeyboard->Acquire();

	if (FAILED(m_pDinput->CreateDevice(GUID_SysMouse, &m_pMouse, NULL))) {
		DxUtSendError("Dinput8 could not be create mouse."); }
	if (FAILED(m_pMouse->SetDataFormat(&c_dfDIMouse))) {
		DxUtSendError("Mouse could not set format."); }
	if (FAILED(m_pMouse->SetCooperativeLevel(g_hWnd, DISCL_FOREGROUND | DISCL_NONEXCLUSIVE))) {
		DxUtSendError("Mouse could not set cooperative level."); }
	m_pMouse->Acquire();

	m_pEffectPool = new CEffectPool;

	QueryPerformanceCounter((LARGE_INTEGER*)&m_liCountNum);

	AllocConsole();
	m_hConsleOutput = GetStdHandle(STD_OUTPUT_HANDLE);

	ShowWindow(g_hWnd, SW_SHOWDEFAULT);
}

CD3DApp::CD3DApp(CD3DApp & cpy)
{
	DxUtSendError("There can exist only one instance of CD3DApp.");
}

void CD3DApp::Loop(void (*loopFunction)())
{
	//m_liLastCountNum = 0;
	MSG msg;
	ZeroMemory(&msg, sizeof(msg));
	while (msg.message != WM_QUIT) {
		if (PeekMessage(&msg, 0, 0, 0, PM_REMOVE)) {
			TranslateMessage(&msg);
			DispatchMessage(&msg); }
		else {
			if (m_bPaused)
				Sleep(80);
			else {
				static int frameCount = 0; frameCount++;
				double time = g_TimeElapsed;
				__int64 oldCountNum = m_liLastCountNum;
				CalculateTime(); 
				PollKeyboard();
				PollMouse();

				loopFunction();

				double delta = 1e-6*(m_liCountNum - oldCountNum);
				double sPFrame = .03;
				if (sPFrame - delta > 0) Sleep(1000*(sPFrame - delta));
			}
		}
	}
}

void CD3DApp::WindowResize()
{
	ID3D10RenderTargetView * pRTV[] = {NULL};
	g_pD3DDevice->OMSetRenderTargets(1, pRTV, NULL);

	ReleaseX(g_pRenderTargetView);
	ReleaseX(g_pDepthStencilView);
	//ReleaseX(g_pDepthSetncilBuffer);

	if (FAILED(g_pSwapChain->ResizeBuffers(1, g_uiWndWidth, g_uiWndHeight, DXGI_FORMAT_R8G8B8A8_UNORM, 0))) {
		DxUtSendError("Window could not be resized.");
	}
	ID3D10Texture2D* pBackBuffer;
	if (FAILED(g_pSwapChain->GetBuffer(0, __uuidof(ID3D10Texture2D), (void**)(&pBackBuffer)))) {
		DxUtSendError("Window could not be resized."); }
	if (FAILED(g_pD3DDevice->CreateRenderTargetView(pBackBuffer, 0, &g_pRenderTargetView))) {
		DxUtSendError("Window could not be resized."); }
	ReleaseX(pBackBuffer);

	D3D10_TEXTURE2D_DESC depthStencilDesc;
	depthStencilDesc.Width				= g_uiWndWidth;
	depthStencilDesc.Height				= g_uiWndHeight;
	depthStencilDesc.MipLevels			= 1;
	depthStencilDesc.ArraySize			= 1;
	depthStencilDesc.Format				= DXGI_FORMAT_D24_UNORM_S8_UINT;
	depthStencilDesc.SampleDesc.Count   = 1; 
	depthStencilDesc.SampleDesc.Quality = 0; 
	depthStencilDesc.Usage				= D3D10_USAGE_DEFAULT;
	depthStencilDesc.BindFlags			= D3D10_BIND_DEPTH_STENCIL;
	depthStencilDesc.CPUAccessFlags		= 0; 
	depthStencilDesc.MiscFlags			= 0;

	ID3D10Texture2D * depthStencilBuffer;
	if (FAILED(g_pD3DDevice->CreateTexture2D(&depthStencilDesc, 0, &depthStencilBuffer))) {
		DxUtSendError("Window could not be resized."); }
	if (FAILED(g_pD3DDevice->CreateDepthStencilView(depthStencilBuffer, 0, &g_pDepthStencilView))) {
		DxUtSendError("Window could not be resized."); }
	ReleaseX(depthStencilBuffer);

	g_pD3DDevice->OMSetRenderTargets(1, &g_pRenderTargetView, g_pDepthStencilView);

	D3D10_VIEWPORT vp;
	vp.TopLeftX = 0;
	vp.TopLeftY = 0;
	vp.Width    = g_uiWndWidth;
	vp.Height   = g_uiWndHeight;
	vp.MinDepth = 0.0f;
	vp.MaxDepth = 1.0f;

	g_pD3DDevice->RSSetViewports(1, &vp);
}

void CD3DApp::WindowFullScreen()
{
	/*if (!m_bFullscreen) {
		INT width = GetSystemMetrics(SM_CXSCREEN);
		INT height = GetSystemMetrics(SM_CYSCREEN);
		g_uiWndWidth = width;
		g_uiWndHeight = height;
		m_bFullscreen = TRUE;

		SetWindowLongPtrW(g_hWnd, GWL_STYLE, WS_POPUP);
		SetWindowPos(g_hWnd, HWND_NOTOPMOST, 0, 0, width, height, SWP_NOZORDER | SWP_SHOWWINDOW);

		//g_pSwapChain->SetFullscreenState(TRUE, NULL);
		WindowResize();
	}
	else {
		RECT rect = {0, 0, g_uiWndWidth, g_uiWndHeight};
		AdjustWindowRect(&rect, WS_CAPTION | WS_SYSMENU | WS_SIZEBOX | WS_MINIMIZEBOX, 0);
		rect.right -= rect.left; rect.bottom -= rect.top;
		//g_PreParam.BackBufferWidth = g_uiWndWidth;
		//g_PreParam.BackBufferHeight = g_uiWndHeight;
		//g_PreParam.Windowed = true;
		m_bFullscreen = TRUE;

		SetWindowLongPtrW(g_hWnd, GWL_STYLE, WS_CAPTION | WS_SYSMENU | WS_SIZEBOX | WS_MINIMIZEBOX);
		SetWindowPos(g_hWnd, HWND_NOTOPMOST, m_wWndPosX, m_wWndPosY,
			rect.right, rect.bottom, SWP_SHOWWINDOW);  

		WindowResize();
	}*/
}

void CD3DApp::CalculateTime()
{
	__int64 countsPer_s;
	QueryPerformanceFrequency((LARGE_INTEGER*)&countsPer_s);
	QueryPerformanceCounter((LARGE_INTEGER*)&m_liCountNum);
	__int64 deltaCount = m_liCountNum-m_liLastCountNum;
	float sPerFrame = (float)(deltaCount)/(float)countsPer_s;
	if (sPerFrame > 1.f) sPerFrame = .001f;
	m_liLastCountNum = m_liCountNum;

	g_TimeElapsed += sPerFrame;
	g_SPFrame = sPerFrame;
}

void CD3DApp::PollKeyboard()
{
	if (FAILED(m_pKeyboard->GetDeviceState(sizeof(g_KeysState), (void**)&g_KeysState))) {
		ZeroMemory(g_KeysState, sizeof(*g_KeysState));
		m_pKeyboard->Acquire();
	}
}

void CD3DApp::PollMouse()
{
	if (FAILED(m_pMouse->GetDeviceState(sizeof(g_MouseState), &g_MouseState))) {
		ZeroMemory(&g_MouseState, sizeof(g_MouseState));
		m_pMouse->Acquire();
	}
}

void CD3DApp::DestroyD3DApp()
{
	g_uiWndWidth = 0;
	g_uiWndHeight = 0;
	m_wWndPosX = 0;
	m_wWndPosY = 0;
	m_bFullscreen = 0;
	m_liCountNum = 0;
	m_liLastCountNum = 0;
	m_bPaused = 1;

	m_hInst = 0; 
	g_hWnd = 0;
	g_uiWndWidth = 0;
	g_uiWndHeight = 0;

	/* Free Effect pools */
	m_pEffectPool->DestroyEffectPool();
	delete m_pEffectPool;
	m_pEffectPool = NULL;

	ReleaseX(g_pSwapChain);
	ReleaseX(g_pD3DDevice);
	ReleaseX(m_pDinput);
	m_pKeyboard->Unacquire();
	ReleaseX(m_pKeyboard);
	m_pMouse->Unacquire();
	ReleaseX(m_pMouse);

	UnregisterClassW(m_szWndClassName, m_hInst);
	delete[] m_szWndClassName;
	m_szWndClassName = NULL;
	delete[] m_szWndTitleText;
	m_szWndTitleText = NULL;
}


};






















/*
void CD3DApp::InitDirectX()
{
	DXGI_SWAP_CHAIN_DESC desc;
	desc.BufferCount = 1;
	desc.BufferDesc.Width = g_uiWndWidth;
	desc.BufferDesc.Height = g_uiWndHeight;
	desc.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	desc.BufferDesc.RefreshRate.Denominator = 60;
	desc.BufferDesc.RefreshRate.Numerator = 1;
	desc.BufferDesc.ScanlineOrdering = DXGI_MODE_SCANLINE_ORDER_UNSPECIFIED;
	desc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
	desc.Flags = NULL;
	desc.OutputWindow = g_hWnd;
	desc.SampleDesc.Count = 1;
	desc.SampleDesc.Quality = 0;
	desc.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;
	desc.Windowed = TRUE;

	UINT flags = 0;
#if defined(DEBUG) || defined(_DEBUG)  
    flags |= D3D10_CREATE_DEVICE_DEBUG;
#endif

	if (FAILED(D3D10CreateDeviceAndSwapChain(NULL, D3D10_DRIVER_TYPE_HARDWARE, NULL, flags, D3D10_SDK_VERSION, &desc, &g_pSwapChain, &g_pD3DDevice))) {
		DxUtSendError("Direct3D device could not be created.");
	}

	WindowResize();

	if (FAILED(DirectInput8Create(m_hInst, DIRECTINPUT_VERSION, IID_IDirectInput8, (void**)&m_pDinput, NULL))) {
		DxUtSendError("Dinput8 could not be created."); }

	if (FAILED(m_pDinput->CreateDevice(GUID_SysKeyboard, &m_pKeyboard, NULL))) {
		DxUtSendError("Dinput8 could not be create keyboard."); }
	if (FAILED(m_pKeyboard->SetDataFormat(&c_dfDIKeyboard))) {
		DxUtSendError("Keyboard could not set format."); }
	if (FAILED(m_pKeyboard->SetCooperativeLevel(g_hWnd, DISCL_FOREGROUND | DISCL_NONEXCLUSIVE))) {
		DxUtSendError("Keyboard could not set cooperative level."); }
	m_pKeyboard->Acquire();

	if (FAILED(m_pDinput->CreateDevice(GUID_SysMouse, &m_pMouse, NULL))) {
		DxUtSendError("Dinput8 could not be create mouse."); }
	if (FAILED(m_pMouse->SetDataFormat(&c_dfDIMouse))) {
		DxUtSendError("Mouse could not set format."); }
	if (FAILED(m_pMouse->SetCooperativeLevel(g_hWnd, DISCL_FOREGROUND | DISCL_NONEXCLUSIVE))) {
		DxUtSendError("Mouse could not set cooperative level."); }
	m_pMouse->Acquire();

	QueryPerformanceCounter((LARGE_INTEGER*)&m_liCountNum);

	ShowWindow(g_hWnd, SW_SHOWDEFAULT);
} */

/*
void CD3DApp::InitCallbacks(WNDPROC wndProc, void (*onResizeWindowFunction)())
{
	m_WndProc = wndProc;
	m_fpnOnResizeWindow = onResizeWindowFunction;
}

void CD3DApp::InitWindow(HINSTANCE hInst, TCHAR * szClassName, TCHAR * szTitleText, HMODULE hMod, 
	LPCWSTR iconResource, WORD wWndPosX, WORD wWndPosY, WORD wWndWidth, WORD wWndHeight)
{
	WORD len = (WORD)wcslen(szClassName);
	m_szWndClassName = new TCHAR[len+1];
	wmemcpy(m_szWndClassName, szClassName, len);
	m_szWndClassName[len] = '\0';

	len = (WORD)wcslen(szTitleText);
	m_szWndTitleText = new TCHAR[len+1];
	wmemcpy(m_szWndTitleText, szTitleText, len);
	m_szWndTitleText[len] = '\0';

	m_hInst = hInst;
	m_wWndPosX = wWndPosX;
	m_wWndPosY = wWndPosY;
	g_uiWndWidth = wWndWidth;
	g_uiWndHeight = wWndHeight;
	m_bFullscreen = 0;

	WNDCLASSW wc;
	wc.hInstance = m_hInst;
	wc.lpfnWndProc = m_WndProc;
	wc.lpszClassName = m_szWndClassName;
	wc.hIcon = LoadIcon(hMod, iconResource);
	wc.hCursor = LoadCursor(NULL, IDC_ARROW);
	wc.hbrBackground = (HBRUSH)(BLACK_BRUSH);
	wc.style = CS_CLASSDC;
	wc.lpszMenuName = NULL;
	wc.cbClsExtra = 0;
	wc.cbWndExtra = 0;

	RegisterClassW(&wc);

	RECT rect = {0, 0, g_uiWndWidth, g_uiWndHeight};
	AdjustWindowRect(&rect, WS_CAPTION | WS_SYSMENU | WS_SIZEBOX | WS_MINIMIZEBOX, TRUE);
	g_hWnd = CreateWindow(m_szWndClassName, m_szWndTitleText, WS_CAPTION | WS_SYSMENU | WS_SIZEBOX | WS_MINIMIZEBOX,
		m_wWndPosX, m_wWndPosY, (rect.right - rect.left), (rect.bottom - rect.top), NULL, NULL, m_hInst, NULL);

	if (!g_hWnd) {
		DxUtSendError("Window could not be created."); }

	UpdateWindow(g_hWnd);
}*/
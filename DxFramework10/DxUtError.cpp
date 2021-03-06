
#include "DxUtError.h"
#include <stdio.h>
#include "DxUtInclude.h"

void OutputDebugString(int i, char * str) 
{
	char buf[256];
	itoa(i, buf, 10);
	OutputDebugStringA(buf);
	OutputDebugStringA(str);
}

void OutputDebugString(float flt, char * str)
{
	char buf[256];
	itoa((int)flt, buf, 10);
	if (flt < 0 && flt > -1.f)
		OutputDebugStringA("-");
	OutputDebugStringA(buf);
	OutputDebugStringA(".");
	flt = abs(flt);
	float frac = abs((flt-floorf(flt))*1e6);
	itoa((int)frac, buf, 10);
	OutputDebugStringA(buf);
	OutputDebugStringA(str);
}

void _DestroyProcess()
{
	DWORD exitCode = 0;					
	GetExitCodeProcess(NULL, &exitCode);
	ExitProcess(exitCode);	
}

void _SendError(CHAR * szFunction, CHAR * szError, CHAR * szFile, UINT uiLine)
{
	UINT uiLen = strlen(szFunction)+strlen(szError)+strlen(szFile)+100;
	char * text = new char[uiLen];

	sprintf_s(text, uiLen, "%s:\nError: %s\nFile: %s\nLine: %d", szFunction, szError, szFile, uiLine);
	MessageBoxA(0, text, 0, 0);	

#if defined (DEBUG) | defined (_DEBUG)
	if (TERMINATE_ON_DEBUG_ERROR) _DestroyProcess();	
#else
	if (TERMINATE_ON_RELEASE_ERROR) _DestroyProcess();	
#endif

	delete text;
}

void _SendErrorEx(CHAR * szFunction, CHAR * szError1, CHAR * szError2, CHAR * szFile, UINT uiLine)
{
	WORD uiLen = strlen(szFunction)+strlen(szError1)+strlen(szError2)+strlen(szFile)+100;
	char * text = new char[uiLen];

	sprintf_s(text, uiLen, "%s:\nError: %s,\n       %s\nFile: %s\nLine: %d",
		szFunction, szError1, szError2, szFile, uiLine);
	MessageBoxA(0, text, 0, 0);	

#if defined (DEBUG) | defined (_DEBUG)
	if (TERMINATE_ON_DEBUG_ERROR) _DestroyProcess();	
#else
	if (TERMINATE_ON_RELEASE_ERROR) _DestroyProcess();	
#endif

	delete text;
}


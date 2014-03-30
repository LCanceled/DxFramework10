
#ifndef DXUTERROR_H
#define DXUTERROR_H

#include <windows.h>

#define TERMINATE_ON_DEBUG_ERROR 0
#define TERMINATE_ON_RELEASE_ERROR 0

void OutputDebugString(int i, char * str);
void OutputDebugString(float flt, char * str);

void _DestroyProcess();
void _SendError(CHAR * szFunction, CHAR * szError, CHAR * szFile, UINT uiLine);
void _SendErrorEx(CHAR * szFunction, CHAR * szError1, CHAR * szError2, CHAR * szFile, UINT uiLine);

#ifndef DxUtSendError
#define DxUtSendError(szEr)									\
{															\
	_SendError(__FUNCTION__, szEr, __FILE__, __LINE__);		\
	DebugBreak();											\
}
#endif

#ifndef DxUtSendErrorEx
#define DxUtSendErrorEx(szEr1, szEr2)								\
{																	\
	_SendErrorEx(__FUNCTION__, szEr1, szEr2, __FILE__, __LINE__);	\
	DebugBreak();													\
}
#endif

#if defined (DEBUG) | defined (_DEBUG)
#define Assert(x, szFailMsg)		\
	if (!(x)) {						\
		DxUtSendError((szFailMsg));	\
	}								
#else
#define Assert(x, szFailMsg)	\
	;
#endif

#if defined (DEBUG) | defined (_DEBUG)
#define AssertEx(x, szFailMsg1, szFailMsg2)				\
	if (!(x)) {											\
		DxUtSendErrorEx((szFailMsg1), (szFailMsg2));	\
	}																	
#else
#define AssertEx(x, szFailMsg1, szFailMsg2)				\
	;					
#endif



#endif
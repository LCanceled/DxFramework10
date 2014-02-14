
#ifndef DXUTTOKENIZER_H
#define DXUTTOKENIZER_H

#include "DxUtInclude.h"

namespace DxUt {

enum TOKENTYPE {
	TOKENTEXT,
	TOKENINT,
	TOKENFLOAT,
	TOKENPUNCTUATION,	//,.;()[]{}
	TOKENMATH,			//=+-
	TOKENQUOTE,			//"..."
};

class TokenInfo {
public:
	union {
		INT i;
		FLOAT flt;
		CHAR * str;
	};
	DWORD pos;
	DWORD len;
	TOKENTYPE type;
};

class Token {
private:
	HANDLE m_File;
	CHAR * m_FileBuf;
	DWORD m_FileLength;
	TokenInfo m_Token;

	bool ValidText(int i);
public:
	Token();
	~Token() {CloseFile();}

	bool OpenFile(char * file);
	void CloseFile();

	TokenInfo & GCurrTok() {
		return m_Token; }
	bool GNextTok(TokenInfo & tok);
	//pos is position to start search at
	bool FindTok(char * str, DWORD pos);
	//pos is position to start search at
	bool FindTok(TOKENTYPE type, DWORD pos);
};


};


#endif

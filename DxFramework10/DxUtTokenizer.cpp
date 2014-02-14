
#include "DxUtTokenizer.h"

namespace DxUt {
/*
Token::Token():m_File(INVALID_HANDLE_VALUE), m_FileBuf(0), m_FileLength(0)
{
	ZeroMemory(&m_Token, sizeof(m_Token));
}

bool Token::OpenFile(char * file)
{
	char _file[MAX_PATH];
	InsertDirectoryEx(_file, MAX_PATH, g_szFileDir, file);

	m_File = CreateFileA(_file, GENERIC_READ, FILE_SHARE_READ | 
		FILE_SHARE_WRITE, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
	if (m_File == INVALID_HANDLE_VALUE) return 0;

	DWORD numBytesRead = 0;
	m_FileLength = GetFileSize(m_File, 0);
	m_FileBuf = new char[m_FileLength+1];
	if (!ReadFile(m_File, m_FileBuf, m_FileLength, &numBytesRead, 0))
		return 0;
	m_FileBuf[m_FileLength] = '\0';
	if (m_FileBuf[0] == '\0') return 0;

	return 1;
}

void Token::CloseFile()
{
	CloseHandle(m_File);
	m_File = INVALID_HANDLE_VALUE;

	if (m_FileBuf) delete[] m_FileBuf;
	m_FileBuf = NULL;

	m_FileLength = 0;
	ZeroMemory(&m_Token, sizeof(m_Token));

}

bool Token::GNextTok(TokenInfo & tok)
{
	for (DWORD i=(m_Token.pos+m_Token.len); ; i++) {
		char ch = m_FileBuf[i];
		if (ch == '\0') return 0;

		char ch1 = m_FileBuf[i+1];
		bool n1 = ch1 >= '0' && ch1 <= '9';

		if (ch == ' ') {}
		else if (ch == '\t') {}
		else if (ch == '\n') {}
		else if (ch == '#' || (ch == '/' && ch1 == '/')) {
			char * c = &m_FileBuf[i]; 
			for (; *c != '\n' && *c != '\0'; i++) {} i--;
		}
		else if ((ch >= '0' && ch <= '9') || (ch == '.' && n1) || ((ch == '+' || ch == '-') && (ch1 == '.' || n1)) ) {
			m_Token.pos = i;
			if ((ch == '+' || ch == '-')) i++;

			char * c = &m_FileBuf[i];
			for (; *c >= '0' && *c <= '9'; c++, i++) {}

			if (*c == '.') {c++, i++;
				for (; *c >= '0' && *c <= '9'; c++, i++) {}
				if (*c == 'e' || *c == 'E') {c++, i++;
					for (; *c >= '0' && *c <= '9'; c++, i++) {}
				}
				if (*c != 'f') {c--, i--;}

				m_Token.flt = (float)atof(m_FileBuf + m_Token.pos);
				m_Token.len = i - m_Token.pos + 1;
				m_Token.type = TOKENFLOAT;
			}
			else {c--, i--;
				m_Token.i = atoi(m_FileBuf + m_Token.pos);
				m_Token.len = i - m_Token.pos + 1;
				m_Token.type = TOKENINT;
			}
		} 
		else if (ch == ',' || ch == '.' || ch == ';' || ch == '(' ||
			ch == ')' || ch == '[' || ch == ']' || ch == '{' || ch == '}') {

			m_Token.str = &m_FileBuf[i];
			m_Token.pos = i;
			m_Token.len = 1;
			m_Token.type = TOKENPUNCTUATION;
		}
		else if (ch == '=' || ch == '+' || ch == '-' || ch == '*' || ch == '/') {
			m_Token.str = &m_FileBuf[i];
			m_Token.pos = i;
			m_Token.len = 1;
			m_Token.type = TOKENMATH;
		}
		else if (ch == '"') {
			m_Token.pos = ++i;
			m_Token.str = &m_FileBuf[i];

			char * c = &m_FileBuf[i];
			for (; *c != '"' && *c != '\0'; i++) {}
			if (*c == '\0') return 0;

			m_Token.len = i - m_Token.pos;
			m_Token.type = TOKENQUOTE;
		}
		else if ((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') || ch == '_') {
			m_Token.str = &m_FileBuf[i];
			m_Token.pos = i;
			m_Token.type = TOKENTEXT;
			for (; ((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') || ch == '_') && ch != '\0'; i++) {} i--;
			m_Token.len = i - m_Token.pos + 1;
		}
	}

	return 1;
}

//pos is position to start search at
bool Token::FindTok(TOKENTYPE type, DWORD pos)
{
	char ch_1 = m_FileBuf[pos-1];
	if (ch_1 == ' ' || ch_1 == '\t' || ch_1 == '\n') {
		m_Token.pos = pos;
		m_Token.len = 0;
	} else {
		char ch = m_FileBuf[pos];
		for (; ch != '\0' || ch != ' ' || ch != '\t' || ch != '\n'; ch = m_FileBuf[++pos]) {}
		m_Token.pos = pos;
		m_Token.len = 0;
	}

	//TokenInfo tok;
	//for (; 
	//GNextTok(tok);
	return 1;
}

//pos is position to start search at
//bool FindTok(char * str, DWORD pos


};

 
/*
bool Token::ValidText(int i)
{
	char ch = m_FileBuf[i];
	if ((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') || ch == '_')
		return 1;
	if (ch >= '0' && ch <= '9') {
		while (ch >= '0' && ch <= '9') {
			ch = m_FileBuf[--i];
		}
		if ((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') || ch == '_') 
			return 1;
	}

	return 0;
}
*/

};
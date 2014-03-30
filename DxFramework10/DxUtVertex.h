
#ifndef DXUTVERTEX_H
#define DXUTVERTEX_H

#include "DxUtInclude.h"

namespace DxUt {

struct SVertexP {
	SVertexP() {}
	SVertexP(float x, float y, float z): pos(Vector3F(x, y, z)) {}

	Vector3F pos;
};

struct SVertexPN  {
	SVertexPN() {}
	SVertexPN(float x, float y, float z, float nx, float ny, float nz):
		pos(Vector3F(x, y, z)), nor(Vector3F(nx, ny, nz)) {}

	Vector3F pos;
	Vector3F nor;
};

struct SVertexPNC  {
	SVertexPNC() {}
	SVertexPNC(float x, float y, float z, float nx, float ny, float nz, UINT _color):
		pos(Vector3F(x, y, z)), nor(Vector3F(nx, ny, nz)), color(_color) {}

	Vector3F pos;
	Vector3F nor;
	D3DXCOLOR color;
};

struct SVertexPT {
	SVertexPT() {}
	SVertexPT(float x, float y, float z, float tx, float ty):
		pos(Vector3F(x, y, z)), tex(Vector2F(tx, ty)) {}

	Vector3F pos;
	Vector2F tex;
};

struct SVertexPNT  {
	SVertexPNT() {}
	SVertexPNT(float x, float y, float z, float nx, float ny, float nz, float tx, float ty):
		pos(Vector3F(x, y, z)), nor(Vector3F(nx, ny, nz)), tex(Vector2F(tx, ty)) {}


	Vector3F pos;
	Vector3F nor;
	Vector2F tex;
};

inline static const D3D10_INPUT_ELEMENT_DESC * GetVertexElementDescP() 
{
	static D3D10_INPUT_ELEMENT_DESC el[] = {
		{"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D10_INPUT_PER_VERTEX_DATA, 0}
	};

	return el;
}

inline static const D3D10_INPUT_ELEMENT_DESC * GetVertexElementDescPN() 
{
	static D3D10_INPUT_ELEMENT_DESC el[] = {
		{"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D10_INPUT_PER_VERTEX_DATA, 0},
		{"NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D10_INPUT_PER_VERTEX_DATA, 0}
	};

	return el;
}

inline static const D3D10_INPUT_ELEMENT_DESC * GetVertexElementDescPNC() 
{
	static D3D10_INPUT_ELEMENT_DESC el[] = {
		{"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D10_INPUT_PER_VERTEX_DATA, 0},
		{"NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D10_INPUT_PER_VERTEX_DATA, 0},
		{"COLOR", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 24, D3D10_INPUT_PER_VERTEX_DATA, 0}
	};

	return el;
}

inline static const D3D10_INPUT_ELEMENT_DESC * GetVertexElementDescPT() 
{
	static D3D10_INPUT_ELEMENT_DESC el[] = {
		{"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D10_INPUT_PER_VERTEX_DATA, 0},
		{"TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, 12, D3D10_INPUT_PER_VERTEX_DATA, 0}
	};

	return el;
}

inline static const D3D10_INPUT_ELEMENT_DESC * GetVertexElementDescPNT() 
{
	static D3D10_INPUT_ELEMENT_DESC el[] = {
		{"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D10_INPUT_PER_VERTEX_DATA, 0},
		{"NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D10_INPUT_PER_VERTEX_DATA, 0},
		{"TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, 24, D3D10_INPUT_PER_VERTEX_DATA, 0},
	};

	return el;
}


};

#endif
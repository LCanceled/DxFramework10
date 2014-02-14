
#ifndef DXUTSHAPES_H
#define DXUTSHAPES_H

#include "DxUtInclude.h"

namespace DxUt {

void CreateSphere(Vector3F & pos, FLOAT radius, WORD segments, WORD rings,
	CONST D3D10_INPUT_ELEMENT_DESC * el, UINT nEl, UINT op, ID3DX10Mesh *& mesh);
void CreateBox(float width, float height, float depth, Matrix4x4F & trans,
	CONST D3D10_INPUT_ELEMENT_DESC * el, UINT nEl, UINT op, ID3DX10Mesh  *& mesh);
void CreateGrid(DWORD rows, DWORD columns, float width, float depth, Matrix4x4F &
	trans, CONST D3D10_INPUT_ELEMENT_DESC * el, UINT nEl, UINT op, ID3DX10Mesh  *& mesh);
void CreateScreenQuad(ID3DX10Mesh *& mesh);


};

#endif
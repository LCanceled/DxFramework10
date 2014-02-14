
#ifndef DXUTTRIOCTREE_H
#define DXUTTRIOCTREE_H

#include "DxUtInclude.h"
#include "DxUtArray.h"
#include "DxUtAABBox.h"
#include "DxUtTriangle.h"

namespace DxUt {

class CTriOctree {
private:

	struct SNode {
		CAABBox box;
		/* 
			0: 0,0,0 
			1: x,0,0
			2: 0,y,0
			3: x,y,0
			4: 0,0,z
			5: x,0,z
			6: 0,y,z
			7: x,y,z
		*/
		SNode * rgChild[8];
	};
	struct SLeaf {
		CArray<Vector3F> rgVert;
		CArray<Vector3F> rgEdge;
		CArray<STriangleF> rgTri;
	};
	SNode * m_pTree;
	DWORD * m_rgAdj;
	Vector3F * m_rgTmpVert[8];
	DWORD m_nTmpVert[8];
	DWORD m_nTrisPerOct;

	void BuildTree(SNode * pTree, Vector3F * rgVert, DWORD nVert, SNode ** pPreNode);
	void SubdivideTriangles(Vector3F * rgVert, DWORD nVert, CAABBox & box);
	void SortAlongAxis(Vector3F * rgVert, DWORD nVert, DWORD dwAxis, Vector3F & center, Vector3F * rgRemainingVert, DWORD & nRemainingVert);
public:
	CTriOctree();
	//~Octree() {}

	void CreateTriOctree(ID3DX10Mesh * pMesh, DWORD dwStride, DWORD nTrisPerOct);
	void CreateTriOctree(STriangleF * rgTri, DWORD nTri, DWORD * pAdj, DWORD nTrisPerOct);
};



};


#endif

#include "DxUtTriOctree.h"
#include "DxUtMesh.h"

namespace DxUt {

CTriOctree::CTriOctree():m_pTree(0)
{
}

void CTriOctree::CreateTriOctree(ID3DX10Mesh * pMesh, DWORD dwStride, DWORD nTrisPerOct)
{
	Assert(!m_pTree, "Octree::CreateOctree octree must be destroyed before creating a new one.");

	DWORD nTri = pMesh->GetFaceCount();
	DWORD nVert = 3 * pMesh->GetFaceCount();
	Vector3F * rgVert = new Vector3F[nVert];
	ExtractVertexTriangleListFromMesh(pMesh, rgVert, dwStride);
	Vector3F * pVertMem = new Vector3F[8*nVert];
	for (DWORD i=0; i<8; i++) m_rgTmpVert[i] = pVertMem + i*nVert;
	ZeroMemory(m_nTmpVert, sizeof(m_nTmpVert));
	m_rgAdj = new DWORD[3 * nVert];
	ExtractAdjanceyFromMesh(pMesh, m_rgAdj);
	m_nTrisPerOct = nTrisPerOct;
	
	m_pTree = new SNode;
	m_pTree->box.ComputeAABBox(rgVert, nVert);
	SNode ** pPreNode = &m_pTree;
	BuildTree(m_pTree, rgVert, nVert, pPreNode);

	delete[] pVertMem;
}

void CTriOctree::BuildTree(SNode * pTree, Vector3F * rgVert, DWORD nVert, SNode ** pPreNode)
{
	SNode ** rgChild = pTree->rgChild;
	Vector3F & parentMinPW = pTree->box.MinPW();
	Vector3F & parentMaxPW = pTree->box.MaxPW();
	Vector3F & parentCenter =  pTree->box.CenterPointW();
	Vector3F hW(.5f*(parentMaxPW - parentMinPW));
	if (nVert/3 > m_nTrisPerOct) {
		SubdivideTriangles(rgVert, nVert, pTree->box);
		for (DWORD i=0; i<2; i++) {
			for (DWORD j=0; j<2; j++) {
				for (DWORD k=0; k<2; k++) {
					DWORD idx = k+j*2+i*4;
					rgChild[idx] = new SNode;
					rgChild[idx]->box.SetMinP(Vector3F(parentMinPW.x + (float)k*hW.x, parentMinPW.y + (float)j*hW.y, parentMinPW.z + (float)i*hW.z));
					rgChild[idx]->box.SetMaxP(Vector3F(parentCenter.x + (float)k*hW.x, parentCenter.y + (float)j*hW.y, parentCenter.z + (float)i*hW.z));
					BuildTree(rgChild[idx], m_rgTmpVert[idx], m_nTmpVert[idx], &rgChild[idx]);
				}
			}
		}
	} else {
		delete *pPreNode;
		*pPreNode = (SNode*)(new SLeaf);
		SLeaf * pLeaf = (SLeaf*)*pPreNode;

		
		//CArray<Vector3F> rgVert;
		//CArray<Vector3F> rgEdge;
		//CArray<STriangleF> rgTri;
	}
}

void CTriOctree::SubdivideTriangles(Vector3F * rgVert, DWORD nVert, CAABBox & box)
{
	Vector3F & center = box.CenterPointW();

	memcpy(m_rgTmpVert[0], rgVert, sizeof(Vector3F)*nVert);
	m_nTmpVert[0] = nVert;

	// Sort along z
	SortAlongAxis(m_rgTmpVert[0], m_nTmpVert[0], 2, center, m_rgTmpVert[4], m_nTmpVert[4]);

	// Sort along y
	SortAlongAxis(m_rgTmpVert[0], m_nTmpVert[0], 1, center, m_rgTmpVert[2], m_nTmpVert[2]);
	SortAlongAxis(m_rgTmpVert[4], m_nTmpVert[4], 1, center, m_rgTmpVert[6], m_nTmpVert[6]);

	// Sort along x
	SortAlongAxis(m_rgTmpVert[0], m_nTmpVert[0], 0, center, m_rgTmpVert[1], m_nTmpVert[1]);
	SortAlongAxis(m_rgTmpVert[2], m_nTmpVert[2], 0, center, m_rgTmpVert[3], m_nTmpVert[3]);
	SortAlongAxis(m_rgTmpVert[4], m_nTmpVert[4], 0, center, m_rgTmpVert[5], m_nTmpVert[5]);
	SortAlongAxis(m_rgTmpVert[6], m_nTmpVert[6], 0, center, m_rgTmpVert[7], m_nTmpVert[7]);
}

void CTriOctree::SortAlongAxis(Vector3F * rgVert, DWORD nVert, DWORD dwAxis, Vector3F & center, Vector3F * rgRemainingVert, DWORD & nRemainingVert)
{
	nRemainingVert = 0;
	DWORD nNewVert = 0;
	for (DWORD i=0; i<nVert; i+=3) {
		if (rgVert[i][dwAxis] > center[dwAxis] && rgVert[i+1][dwAxis] > center[dwAxis] && rgVert[i+2][dwAxis] > center[dwAxis]) {
			rgRemainingVert[nRemainingVert++] = rgVert[i];
		} else if (rgVert[i][dwAxis] < center[dwAxis] && rgVert[i+1][dwAxis] < center[dwAxis] && rgVert[i+2][dwAxis] < center[dwAxis]) {
			rgRemainingVert[nNewVert++] = rgVert[i];
		} else {
			rgRemainingVert[nRemainingVert++] = rgVert[i];
			rgRemainingVert[nNewVert++] = rgVert[i];
		}
	}
}


};




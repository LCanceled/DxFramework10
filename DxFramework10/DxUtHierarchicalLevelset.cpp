
#include "DxUtHierarchicalLevelset.h"
#include "DxUtD3DApp.h"
#include <fstream>

namespace DxUt {

#define IsLeaf(pNode) ((!pNode->pChildB))

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////   Class CSDFBVTree   ///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

COctreeLevelSet::COctreeLevelSet():m_nOcts(0), m_pLastOct(0)
{
}

/* Kd Tree 
void COctreeLevelSet::CBVTreeEx::CreateBVTree(SNode * pKdTree, DWORD dwMaxDepth)
{
	Assert(!m_pTree, "CBVTreeEx::CreateBVTree tree must be destroyed before creating a new one.");

	m_pTree = new BranchNode;
	m_BVs = new COBBox[2*(1 << dwMaxDepth)];
	m_nBV = 0;

	m_dwMaxDepth = dwMaxDepth;
	BuildBVTree(pOctree, m_pTree, 0);

	Matrix4x4F rot;
	m_BVs[0].RotationW(rot); //why???
	m_BVs[0].GetRotVecW(0) = rot.GetColumnVec3F(0);
	m_BVs[0].GetRotVecW(1) = rot.GetColumnVec3F(1);
	m_BVs[0].GetRotVecW(2) = rot.GetColumnVec3F(2);

	m_TopLevelOBBoxW = *m_pTree->pOBB;
}

void COctreeLevelSet::CSDFBVTree::BuildBVTree(COctreeLevelSet::SNode * pOctree, BranchNode * pNode, DWORD dwDepth)
{
	// or null
	//if (dwDepth >= m_dwMaxDepth || null) return;

	m_nBranchNodes++;

	// Compute OBB
	COBBox & oBB = m_BVs[m_nBV];
	oBB = pOctree->box;
	pNode->pOBB = &m_BVs[m_nBV++];

	pNode->pChildA = new BranchNode;
	BuildBVTree(, pNode->pChildA, dwDepth+1);

	pNode->pChildB = new BranchNode;
	BuildBVTree(pOctree, pNode->pChildB, dwDepth+1);

	Matrix4x4F rotT, cRot;
	pNode->pOBB->RotationW(rotT);
	rotT.MTranspose();

	pNode->pChildA->pOBB->RotationW(cRot);
	cRot = rotT*cRot;

	Vector3F rotVec0[3] = {cRot.GetColumnVec3F(0), cRot.GetColumnVec3F(1), cRot.GetColumnVec3F(2) };
	pNode->pChildA->pOBB->SetOBB(rotT*(pNode->pChildA->pOBB->GetCenterW() - pNode->pOBB->GetCenterW()), pNode->pChildA->pOBB->GetHalfWidthsW(), rotVec0);

	pNode->pChildB->pOBB->RotationW(cRot);
	cRot = rotT*cRot;

	Vector3F rotVec1[3] = {cRot.GetColumnVec3F(0), cRot.GetColumnVec3F(1), cRot.GetColumnVec3F(2) };
	pNode->pChildB->pOBB->SetOBB(rotT*(pNode->pChildB->pOBB->GetCenterW() - pNode->pOBB->GetCenterW()), pNode->pChildB->pOBB->GetHalfWidthsW(), rotVec1);
}*/

void COctreeLevelSet::CBVTreeEx::CreateBVTree(Vector3F * verts, DWORD nVert, DWORD * pAdj, DWORD dwTriPerBV, DWORD nVertexParticles, DWORD nEdges, DWORD nCellsX,
	DWORD nCellsY, DWORD nCellsZ, float fCellSize, Vector3F gridMinP, CFinitePointGrid3F<DWORD> & vertToIdx, CFinitePointGrid3F<DWORD> & edgeToIdx, bool bKdTree)
{
	Assert(!m_pTree, "CSDFBVTree::CreateBVTree tree must be destroyed before creating a new one.");

	if (nVert % 3) {DxUtSendError("The nVert must be divisible by 3."); }
	DWORD nTri = nVert/3;

	m_pAdj = new DWORD[3*nTri];
	memcpy(m_pAdj, pAdj, sizeof(DWORD)*3*nTri);

	DWORD * rgFaceIndex = new DWORD[nVert/3];
	for (DWORD i=0; i<nTri; i++) rgFaceIndex[i] = i;

	m_pTree = new BranchNode;
	m_BVs = new COBBox[2*nTri]; // Worst case there are this many bv's
	m_nBV = 0;
	//m_Tris = new STriangleF[nTri];
	//m_TransformedTris = new STriangleF[nTri];
	//m_nTri = nTri;

	//m_dwMaxDepth = dwMaxDepth;
	m_dwTriPerBV = dwTriPerBV;
	m_pVertexPointGrid = &vertToIdx;
	m_pEdgePointGrid = &edgeToIdx;
	m_rgVertexParticleDuplicate.Resize(nVertexParticles);
	for (DWORD i=0, end=m_rgVertexParticleDuplicate.GetSize(); i<end; i++) m_rgVertexParticleDuplicate[i] = 0;
	m_rgEdgeDuplicate.Resize(nEdges);
	for (DWORD i=0, end=m_rgEdgeDuplicate.GetSize(); i<end; i++) m_rgEdgeDuplicate[i] = 0;

	BuildBVTree(m_pTree, verts, rgFaceIndex, nVert, 0, &m_pTree);

	if (m_nBV >= 2*nTri)
		DebugBreak();

	Matrix4x4F rot;
	m_BVs[0].RotationW(rot);
	m_BVs[0].GetRotVecW(0) = rot.GetColumnVec3F(0);
	m_BVs[0].GetRotVecW(1) = rot.GetColumnVec3F(1);
	m_BVs[0].GetRotVecW(2) = rot.GetColumnVec3F(2); 

	delete[] rgFaceIndex;
	rgFaceIndex = NULL;

	m_TopLevelOBBoxW = *m_pTree->pOBB;
}

void COctreeLevelSet::CBVTreeEx::BuildBVTree(BranchNode * pNode, Vector3F * verts, DWORD * rgFaceIndex, DWORD nVert, DWORD dwDepth, BranchNode ** pPreNode)
{
	if (nVert > 3 && nVert/3 > m_dwTriPerBV) {
		m_nBranchNodes++;

		float mean = 0;
		Vector3F axis;
		ComputeOBB(verts, nVert, m_BVs[m_nBV], mean, axis, dwDepth);

		DWORD dwSplitIndex = 0;
		PartitionVert(verts, rgFaceIndex, nVert, axis, mean, dwSplitIndex);
		pNode->pOBB = &m_BVs[m_nBV++];

		pNode->pChildA = new BranchNode;
		BuildBVTree(pNode->pChildA, verts, rgFaceIndex, dwSplitIndex, dwDepth+1, &pNode->pChildA);

		pNode->pChildB = new BranchNode;
		BuildBVTree(pNode->pChildB, &verts[dwSplitIndex], &rgFaceIndex[dwSplitIndex/3], nVert-dwSplitIndex, dwDepth+1, &pNode->pChildB);

		Matrix4x4F rotT, cRot;
		pNode->pOBB->RotationW(rotT);
		rotT.MTranspose();

		// ChildA
		pNode->pChildA->pOBB->RotationW(cRot);
		cRot = rotT*cRot;

		Vector3F rotVec0[3] = {cRot.GetColumnVec3F(0), cRot.GetColumnVec3F(1), cRot.GetColumnVec3F(2) };
		pNode->pChildA->pOBB->SetOBB(rotT*(pNode->pChildA->pOBB->GetCenterW() - pNode->pOBB->GetCenterW()), pNode->pChildA->pOBB->GetHalfWidthsW(), rotVec0);
		
		// ChildB
		pNode->pChildB->pOBB->RotationW(cRot);
		cRot = rotT*cRot;

		Vector3F rotVec1[3] = {cRot.GetColumnVec3F(0), cRot.GetColumnVec3F(1), cRot.GetColumnVec3F(2) };
		pNode->pChildB->pOBB->SetOBB(rotT*(pNode->pChildB->pOBB->GetCenterW() - pNode->pOBB->GetCenterW()), pNode->pChildB->pOBB->GetHalfWidthsW(), rotVec1);
	}
	else {
		m_nLeafNodes++;
 
		delete *pPreNode;
		*pPreNode = (BranchNode*)(new SLeafNodeEx);
		SLeafNodeEx * pLeaf = (SLeafNodeEx*)*pPreNode;

		//delete *pPreNode;
		//SLeafNodeEx * pLeaf = new SLeafNodeEx;
		//(SLeafNodeEx*)*pPreNode = pLeaf;

		float mean;
		Vector3F axis;
		ComputeOBB(verts, nVert, m_BVs[m_nBV], mean, axis, dwDepth);
		pLeaf->pOBB = &m_BVs[m_nBV++];
	
		/* Compute the vertex particles */
		DWORD dwVal = 0, dwEntry = 0;
		pLeaf->rgVertexParticleIndex.Reserve(nVert);
		for (DWORD i=0; i<nVert; i++) {
			if (m_pVertexPointGrid->PointInGrid(verts[i], &dwVal)) {
				if (dwVal != -1 && !pLeaf->rgVertexParticleIndex.FindIfEquals(dwVal, dwEntry)) {
					//g_D3DApp->Print(verts[i].x); g_D3DApp->Print(verts[i].y); g_D3DApp->Print(verts[i].z); g_D3DApp->Print("\n");
					pLeaf->rgVertexParticleIndex.PushBack(dwVal);
				}
			}
		}
		
		/* Compute the edge particles */
		DWORD nEdges = (pLeaf->rgVertexParticleIndex.GetSize() + nVert/3);
		pLeaf->rgEdgeIndex.Reserve(nEdges);
		for (DWORD i=0; i<nVert; i+=3) {
			SVisitedEdge e1(verts[i+0], verts[i+1]);
			SVisitedEdge e2(verts[i+1], verts[i+2]);
			SVisitedEdge e3(verts[i+2], verts[i+0]);

			//DWORD dwTmp = 0;
			//if (dwVal != -1) {
				if (!m_pEdgePointGrid->PointInGrid(e1.vAvg, &dwVal)) {
					DebugBreak();
				}
				if (dwVal != -1 && !pLeaf->rgEdgeIndex.FindIfEquals(dwVal, dwEntry))
					pLeaf->rgEdgeIndex.PushBack(dwVal);
			//}
			//if (dwVal != -1) {
				if (!m_pEdgePointGrid->PointInGrid(e2.vAvg, &dwVal)) {
					DebugBreak();
				}
				if (dwVal != -1 && !pLeaf->rgEdgeIndex.FindIfEquals(dwVal, dwEntry))
					pLeaf->rgEdgeIndex.PushBack(dwVal);
			//}
			//if (dwVal != -1) {
				if (!m_pEdgePointGrid->PointInGrid(e3.vAvg, &dwVal)) {
					DebugBreak();
				}
				if (dwVal != -1 && !pLeaf->rgEdgeIndex.FindIfEquals(dwVal, dwEntry))
					pLeaf->rgEdgeIndex.PushBack(dwVal);
			//}
		}//*/
	}
}

DWORD COctreeLevelSet::CBVTreeEx::BVCollision(CBVTreeEx & collideBVTree, CArray<DWORD> & rgVertexParticleIndex, CArray<DWORD> & rgEdgeIndex)
{
	m_pCollideBVTree = &collideBVTree;
	m_pVertexParticleIndex = &rgVertexParticleIndex;
	m_pEdgeIndex = &rgEdgeIndex;
	m_nBVBVTests = m_nTriTriTests = 0;

	Matrix4x4F rot;
	m_BVs[0].RotationW(rot);
	rot = m_Rot*rot;
	Vector3F pos(m_Scale*(m_Rot*m_BVs[0].GetCenterW()) + m_Trans);

	Matrix4x4F cRot;
	collideBVTree.m_BVs[0].RotationW(cRot);
	cRot = collideBVTree.m_Rot*cRot;
	Vector3F cPos(collideBVTree.m_Scale*(collideBVTree.m_Rot*
		collideBVTree.m_BVs[0].GetCenterW()) + collideBVTree.m_Trans);

	//Compute the pos, rot, scl relative to the this BVTree
	Matrix4x4F rotR(rot.Transpose());
	Vector3F posR((rotR*(cPos - pos))/m_Scale);
	rotR = rotR*cRot;
	//m_RelativeScale = collideBVTree.m_Scale/m_Scale;
	float fSclR = collideBVTree.m_Scale/m_Scale;

	DWORD oldSize = rgVertexParticleIndex.GetSize();
	FindBVCollisions(m_pTree, collideBVTree.m_pTree, rotR, posR, fSclR);

	return rgVertexParticleIndex.GetSize() - oldSize;	
}

bool COctreeLevelSet::CBVTreeEx::FindBVCollisions(BranchNode * pNode, BranchNode * pCollideNode, Matrix4x4F & rot, Vector3F & trans, float scl)
{
	m_nBVBVTests++;

	if (pNode->pOBB->OBBoxIntersectW(*pCollideNode->pOBB, rot, trans, scl)) {
		if (!IsLeaf(pNode) || !IsLeaf(pCollideNode)) {
			if (IsLeaf(pCollideNode) || (!IsLeaf(pNode) && pNode->pOBB->GetHalfWidthsW().x >= pCollideNode->pOBB->GetHalfWidthsW().x)) {
				Matrix4x4F rotR;
				pNode->pChildA->pOBB->RotationWT(rotR);

				Vector3F transR(rotR*(trans - pNode->pChildA->pOBB->GetCenterW()));
				rotR *= rot;

				if (FindBVCollisions(pNode->pChildA, pCollideNode, rotR, transR, scl)) return 1;

				pNode->pChildB->pOBB->RotationWT(rotR);

				transR = Vector3F(rotR*(trans - pNode->pChildB->pOBB->GetCenterW()));
				rotR *= rot;

				if (FindBVCollisions(pNode->pChildB, pCollideNode, rotR, transR, scl)) return 1; 
			}
			else {
				Matrix4x4F rotR;
				pCollideNode->pChildA->pOBB->RotationW(rotR);

				rotR = rot*rotR;
				Vector3F transR(scl*(rot*pCollideNode->pChildA->pOBB->GetCenterW()) + trans);

				if (FindBVCollisions(pNode, pCollideNode->pChildA, rotR, transR, scl)) return 1;

				pCollideNode->pChildB->pOBB->RotationW(rotR);

				rotR = rot*rotR;
				transR = scl*(rot*pCollideNode->pChildB->pOBB->GetCenterW()) + trans;

				if (FindBVCollisions(pNode, pCollideNode->pChildB, rotR, transR, scl)) return 1;
			}
		}
		else {
			m_nTriTriTests++;

			SLeafNodeEx * pLeaf = (SLeafNodeEx*)pNode;

			for (DWORD i=0, end=pLeaf->rgVertexParticleIndex.GetSize(); i<end; i++) {
				DWORD dwVP = pLeaf->rgVertexParticleIndex[i];
				if (!m_rgVertexParticleDuplicate[dwVP]) {
					m_pVertexParticleIndex->PushBack(dwVP);
					m_rgVertexParticleDuplicate[dwVP] = 1;
				} 
			}
			for (DWORD i=0, end=pLeaf->rgEdgeIndex.GetSize(); i<end; i++) {
				DWORD dwE = pLeaf->rgEdgeIndex[i];
				if (!m_rgEdgeDuplicate[dwE]) {
					m_pEdgeIndex->PushBack(dwE);
					m_rgEdgeDuplicate[dwE] = 1;
				}
			}

			return 0;
		}
	}

	return 0;
}















void COctreeLevelSet::CBVTreeEx::DestroyBVHierarchy(BranchNode * pNode)
{
	BranchNode * p1 = pNode->pChildA;
	BranchNode * p2 = pNode->pChildB;
	if (!IsLeaf(pNode)) {
		DestroyBVHierarchy(p1);
		DestroyBVHierarchy(p2);
		delete pNode;
		pNode = NULL;
	} else {
		SLeafNodeEx * pLeaf = (SLeafNodeEx*)pNode;
		pLeaf->rgVertexParticleIndex.Clear();
		pLeaf->rgEdgeIndex.Clear();
		delete pLeaf;
		pLeaf = NULL;
	}
}

void COctreeLevelSet::CBVTreeEx::DestroyBVTree()
{
	if (m_pTree) {
		DestroyBVHierarchy(m_pTree);
		m_pTree = NULL;
		//delete m_pTree;
		//m_pTree = NULL;

		m_nBranchNodes = 0;
		m_nLeafNodes = 0;

		delete[] m_pAdj;
		m_pAdj = NULL;

		delete[] m_BVs;
		m_nBV = NULL;
		//delete[] m_Tris;
		//m_nTri = NULL;
		//delete[] m_TransformedTris;
		//m_TransformedTris = NULL;

		m_Trans.x = 0;
		m_Trans.y = 0;
		m_Trans.z = 0;
		m_Scale = 0;

		m_pCollideBVTree = NULL;
		m_CPs = NULL;
		m_nBVBVTests = 0;
		m_nTriTriTests = 0;

		m_rgVertexParticleDuplicate.Clear();
		m_rgEdgeDuplicate.Clear();
	}
}











//http://www.tor.com/blogs/2011/11/ten-anime-series-you-should-see-before-you-die
//http://prog21.dadgum.com/23.html
// SET TRANSFORMS!!!!!!!!!!!

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////   Class COctreeLevelSet   ///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void COctreeLevelSet::ComputeSDF(ID3DX10Mesh * pMesh, DWORD dwStride, char * szLevelSetFile, float fMinCellSize, float fInterpolationError, Vector3F & expansion)
{
	DWORD nVert = 3*pMesh->GetFaceCount();
	Vector3F * verts = new Vector3F[nVert];
	ExtractVertexTriangleListFromMesh(pMesh, verts, dwStride);

	DWORD * pAdj = new DWORD[3*nVert];
	ExtractAdjanceyFromMesh(pMesh, pAdj);

	ComputeSDF(verts, nVert, pAdj, szLevelSetFile, fMinCellSize, fInterpolationError, expansion);
}

void COctreeLevelSet::ComputeSDF(STriangleF * tris, DWORD nTri, DWORD * pAdj, char * szLevelSetFile, float fMinCellSize, float fInterpolationError, Vector3F & expansion)
{
	Assert(0, "COctreeLevelSet::ComputeSDF function not implemented.");
}

void COctreeLevelSet::ComputeSDF(Vector3F * verts, DWORD nVert, DWORD * pAdj, char * szLevelSetFile, float fMinCellSize, float fInterpolationError, Vector3F & expansion)
{
	/* Construct HLevelSet */
	CAABBox aabb;
	static const float eps = 1e-3;
	m_GridBox.ComputeAABBox(verts, nVert);
	m_GridMin = m_GridBox.MinPW() - Vector3F(eps);
	m_GridMax = m_GridBox.MaxPW() + Vector3F(eps);
	m_GridBox.SetMinP(m_GridMin);
	m_GridBox.SetMaxP(m_GridMax);
	m_TransformObjToGrid = m_GridMin;

	m_pOctree = new SNode;
	m_pOctree->box = m_GridBox;
	m_pOctree->box.SetMinP(m_pOctree->box.MinPW() - expansion);
	m_pOctree->box.SetMaxP(m_pOctree->box.MaxPW() + expansion);
	m_nOcts = 0;
	m_dwMaxDepth = 0;

	STriangleFEx * tris = new STriangleFEx[nVert/3];
	for (DWORD i=0, end=nVert/3; i<end; i++) {
		float fDummy = 0;
		tris[i].vPosW[0] = verts[3*i + 0];
		tris[i].vPosW[1] = verts[3*i + 1];
		tris[i].vPosW[2] = verts[3*i + 2];
		tris[i].nor = tris[i].Normal();
		tris[i].bBadTriangle |= ComputeAdmissibleZone(i, 0, verts, pAdj, tris[i].vertNor[0], tris[i].edgeNor[0], fDummy) == -1;
		tris[i].bBadTriangle |= ComputeAdmissibleZone(i, 1, verts, pAdj, tris[i].vertNor[1], tris[i].edgeNor[1], fDummy) == -1;
		tris[i].bBadTriangle |= ComputeAdmissibleZone(i, 2, verts, pAdj, tris[i].vertNor[2], tris[i].edgeNor[2], fDummy) == -1;
		tris[i].bBadTriangle |= tris[i].Area() < 1e-4;
		if (tris[i].bBadTriangle)
			int a=0;
	}
	m_fMinCellSize =  fMinCellSize;
	m_fInterpolationEps = fInterpolationError;
	BuildOctree(m_pOctree, 0, verts, nVert, tris);
	WriteOctree(szLevelSetFile);
	delete[] tris;
}

void COctreeLevelSet::CreateParticleRepresentation(ID3DX10Mesh * pMesh, DWORD dwStride, DWORD dwMaxDepth, DWORD dwTriPerOct, char * szLevelSetFile)
{
	m_dwMaxDepth = dwMaxDepth;
	m_dwTriPerVolume = dwTriPerOct;
	CLevelSet::CreateParticleRepresentation(pMesh, dwStride, szLevelSetFile);
}

void COctreeLevelSet::CreateParticleRepresentation(STriangleF * tris, DWORD nTri, DWORD * pAdj, DWORD dwMaxDepth, DWORD dwTriPerOct, char * szLevelSetFile)
{
}

void COctreeLevelSet::CreateParticleRepresentation(Vector3F * verts, DWORD nVert, DWORD * pAdj, DWORD dwMaxDepth, DWORD dwTriPerOct, char * szLevelSetFile)
{
	m_dwMaxDepth = dwMaxDepth;
	m_dwTriPerVolume = dwTriPerOct;
	ComputeMeshParticles(nVert, nVert/3, verts, pAdj, szLevelSetFile);
}

float COctreeLevelSet::ComputeSignedDistance(SNode * pNode, Vector3F & pt)
{
	Vector3F & widths = 2.f*pNode->box.HalfWidthsW();
	
	Vector3F dxyz = (pt - pNode->box.MinPW())/widths;
	float t = (float)InterpolateTrilinear(dxyz.x, dxyz.y, dxyz.z,
			pNode->V[0], pNode->V[1], pNode->V[2], pNode->V[3], pNode->V[4], pNode->V[5], pNode->V[6], pNode->V[7]);
	return t;
}

bool COctreeLevelSet::ParticleInOct(SNode * pNode, Vector3F & pt, float & fDist)
{
	CAABBox tmpBox(pNode->box);
	const float eps = 1e-1;
	tmpBox.SetMinP(tmpBox.MinPW() - Vector3F(eps));
	tmpBox.SetMaxP(tmpBox.MaxPW() + Vector3F(eps));
	if (!tmpBox.PointInAABBoxW(pt)) {
		tmpBox.PointInAABBoxW(pt);
		DebugBreak();
	}

	float t = ComputeSignedDistance(pNode, pt);
	fDist = t;
	m_pLastOct = pNode;
	return t < -m_fValidParticleDistance;
}

bool COctreeLevelSet::IsOctRefined(SNode * pNode, Vector3F * rotVec, Vector3F & hW, Vector3F & center, DWORD dwDepth, Vector3F * verts, DWORD nVert, STriangleFEx * tris)
{
	if (pNode->box.CenterPointW().y > .85f && pNode->box.CenterPointW().x < -5.45f && abs(pNode->box.CenterPointW().z) < 6.f)
		int a=0;

	float fHalfCellSize = .5f*m_fMinCellSize;
	if (hW.x <= fHalfCellSize && hW.y <= fHalfCellSize && hW.z <= fHalfCellSize) {
		return 1;
	}

	/* Determine if the cell is empty */				
	float fExactDist = ComputeClosestDistance(pNode->box.CenterPointW(), verts, nVert, tris);
	float fMaxHW = Max(hW.x, hW.y); fMaxHW = Max(hW.y, hW.z);
	if (fExactDist > fMaxHW) {
		return 1;
	}
	
	Vector3F widths(2.f*Abs(hW.x), 2.f*Abs(hW.y), 2.f*Abs(hW.z));
	for (int i=-1, ai; i<=1; i++) {
		ai = !Abs(i);
		for (int j=-1, aj; j<=1; j++) {
			aj = !Abs(j);
			for (int k=-1, ak; k<=1; k++) {
				ak = !Abs(k);
				// Do not sample corners
				DWORD sum = Abs(i) + Abs(j) + Abs(k);
				if (sum == 3) 
					continue;

				Vector3F pt(center.x + (float)k*hW.x, center.y + (float)j*hW.y, center.z + (float)i*hW.z);
				float fExactDist = ComputeClosestDistance(pt, verts, nVert, tris);

				float fInterpDist;
				ParticleInOct(pNode, pt, fInterpDist);

				float fDist = abs(fInterpDist - fExactDist);
				if (ak && fDist * widths.x > m_fInterpolationEps && hW.x > fHalfCellSize) return 0;
				if (aj && fDist * widths.y > m_fInterpolationEps && hW.y > fHalfCellSize) return 0;
				if (ai && fDist * widths.z > m_fInterpolationEps && hW.z > fHalfCellSize) return 0;
			}
		}
	}
	return 1;
}

void COctreeLevelSet::ComputeOctDists(SNode * pNode, Vector3F * verts, DWORD nVert, STriangleFEx * tris)
{
	Vector3F & hW = pNode->box.HalfWidthsW();
	Vector3F & center = pNode->box.CenterPointW();
	for (int i=-1, idx=0; i<=1; i+=2) {
		for (int j=-1; j<=1; j+=2) {
			for (int k=-1; k<=1; k+=2, idx++) {
				pNode->V[idx] = FLT_MAX;
				pNode->V[idx] = ComputeClosestDistance(Vector3F(center.x + (float)k*hW.x, center.y + (float)j*hW.y, center.z + (float)i*hW.z), verts, nVert, tris);
				//CLevelSet::ParticleInLevelSet_Fast(Vector3F(center.x + (float)k*hW.x, center.y + (float)j*hW.y, center.z + (float)i*hW.z), pNode->V[idx]);
				if (pNode->V[idx] == FLT_MAX) {
					CLevelSet::ParticleInLevelSet_Fast(Vector3F(center.x + (float)k*hW.x, center.y + (float)j*hW.y, center.z + (float)i*hW.z), pNode->V[idx]);
					DebugBreak();
				}
			}
		}
	}
	/*bool inside = 0;
	for (DWORD i=0; i<8; i++) {
		inside |= pNode->V[i] < 0;
	}
	if (center.y > 3.f && inside) {
		//DebugBreak();
		for (int i=-1, idx=0; i<=1; i+=2) {
			for (int j=-1; j<=1; j+=2) {
				for (int k=-1; k<=1; k+=2, idx++) {
					if (idx == 5)
						int a=0;
					pNode->V[idx] = 4096;
					pNode->V[idx] = ComputeClosestDistance(Vector3F(center.x + (float)k*hW.x, center.y + (float)j*hW.y, center.z + (float)i*hW.z), verts, nVert, tris);
					//CLevelSet::ParticleInLevelSet_Fast(Vector3F(center.x + (float)k*hW.x, center.y + (float)j*hW.y, center.z + (float)i*hW.z), pNode->V[idx]);
					if (pNode->V[idx] == 4096) {
						CLevelSet::ParticleInLevelSet_Fast(Vector3F(center.x + (float)k*hW.x, center.y + (float)j*hW.y, center.z + (float)i*hW.z), pNode->V[idx]);
						DebugBreak();
					}
				}
			}
		}
	}*/
}

void COctreeLevelSet::BuildOctree(SNode * pTree, DWORD dwDepth, Vector3F * verts, DWORD nVert, STriangleFEx * tris)
{
	Vector3F rotVec[3] = {Vector3F(1.f, 0., 0.), Vector3F(0., 1.f, 0.), Vector3F(0., 0., 1.f) };
	Vector3F & hW = pTree->box.HalfWidthsW();
	Vector3F & center = pTree->box.CenterPointW();
	ComputeOctDists(pTree, verts, nVert, tris);
	m_nOcts++;

	m_dwMaxDepth = max(m_dwMaxDepth, dwDepth);

	if (IsOctRefined(pTree, rotVec, hW, center, dwDepth, verts, nVert, tris)) {
		pTree->bLeaf = 1;
		return;
	}

	DWORD idx = 0;
	Vector3F hHW(.5f*hW);
	for (int i=-1; i<=1; i+=2) {
		for (int j=-1; j<=1; j+=2) {
			for (int k=-1; k<=1; k+=2) {
				pTree->rgChild[idx] = new SNode;
				pTree->rgChild[idx]->box.SetAABB(Vector3F(center.x + (float)k*hHW.x, center.y + (float)j*hHW.y, center.z + (float)i*hHW.z), hHW);
				BuildOctree(pTree->rgChild[idx], dwDepth+1, verts, nVert, tris); idx++;
			}
		}
	}
}

void COctreeLevelSet::DoAdditionalProcessing(char * szLevelSet, Vector3F * verts, DWORD nVert, DWORD * pAdj, 
	CFinitePointGrid3F<DWORD> & vertToIdx, CFinitePointGrid3F<DWORD> & edgeToIdx) 
{
	if (!LoadOctree(szLevelSet)) {
		Assert(0, "COctreeLevelSet::ComputeMeshParticles could not load octree level set.");
	}
	m_fMinCellSize;

	/* Construct triangle hierarchies */
	Vector3F gridMinP = m_GridMin;
	m_TriBVTree.CreateBVTree(verts, nVert, pAdj, m_dwTriPerVolume, m_rgVertexParticle.GetSize(), m_rgEdge.GetSize(),
		m_nCellsX, m_nCellsY, m_nCellsZ, m_fMinCellSize, gridMinP, vertToIdx, edgeToIdx, 0);

	m_rgVertexParticleIndex.Reserve(m_rgVertexParticle.GetSize());
	m_rgEdgeIndex.Reserve(m_rgEdge.GetSize());
}

bool COctreeLevelSet::LoadOctree(char * szFile)
{
	char file[MAX_PATH];
	ChangeExtension(szFile, "olvset", file);

	std::ifstream stream(file, std::ios::binary);
	if (!stream) {
		return 0; 
	}

	stream.read((char*)&m_dwMaxDepth, sizeof(DWORD));
	stream.read((char*)&m_nOcts, sizeof(DWORD));
	
	m_pOctree = new SNode;
	m_fMinCellSize = FLT_MAX;

	LoadOctreeLevel(stream, &m_pOctree);

	stream.close();

	return 1;
}
	
void COctreeLevelSet::WriteOctree(char * szFile)
{
	char file[MAX_PATH];
	ChangeExtension(szFile, "olvset", file);

	std::ofstream stream(file, std::ios::out | std::ios::binary); stream.close();
	stream.open(file, std::ios::out | std::ios::binary);

	stream.write((char*)&m_dwMaxDepth, sizeof(DWORD));
	stream.write((char*)&m_nOcts, sizeof(DWORD));

	WriteOctreeLevel(stream, m_pOctree);

	stream.close();
}

void COctreeLevelSet::LoadOctreeLevel(std::ifstream & stream, SNode ** pNode)
{
	stream.read((char*)(*pNode), sizeof(SNode) - 8*sizeof(SNode*));
	float h = (*pNode)->box.HalfWidthsW().x;
	m_fMinCellSize = Min(m_fMinCellSize, h);
	if (!(*pNode)->bLeaf) {
		for (DWORD i=0; i<8; i++) {
			(*pNode)->rgChild[i] = new SNode;
			LoadOctreeLevel(stream, &(*pNode)->rgChild[i]);
		}
	}
}

void COctreeLevelSet::WriteOctreeLevel(std::ofstream & stream, SNode * pNode)
{
	stream.write((char*)pNode, sizeof(SNode) - 8*sizeof(SNode*));
	if (!pNode->bLeaf) {
		for (DWORD i=0; i<8; i++) {
			WriteOctreeLevel(stream, pNode->rgChild[i]);
		}
	}
}

DWORD COctreeLevelSet::LevelSetCollision(CLevelSet & _collideLevelSet, CArray<SContactPoint> * CPs)
{
	COctreeLevelSet & collideLevelSet = *((COctreeLevelSet*)&_collideLevelSet);
	/* Compute collision indicies */
	bool bUseHierarchy = 0;
	if (bUseHierarchy) {
		m_TriBVTree.BVCollision(collideLevelSet.m_TriBVTree, m_rgVertexParticleIndex, m_rgEdgeIndex);
		SetCollisionIndicies(&m_rgVertexParticleIndex, &m_rgEdgeIndex);
		//g_D3DApp->Print(m_TriBVTree.GetNumTriTriTests());

		collideLevelSet.m_TriBVTree.BVCollision(m_TriBVTree, collideLevelSet.m_rgVertexParticleIndex, collideLevelSet.m_rgEdgeIndex);
		collideLevelSet.SetCollisionIndicies(&collideLevelSet.m_rgVertexParticleIndex, &collideLevelSet.m_rgEdgeIndex);
		//g_D3DApp->Print(collideLevelSet.m_TriBVTree.GetNumTriTriTests());
	} else {
		SetCollisionIndicies(NULL, NULL);
		collideLevelSet.SetCollisionIndicies(NULL, NULL);
	}

	m_pLastOct = NULL;
	DWORD n = CLevelSet::LevelSetCollision(collideLevelSet, CPs); 
	m_rgVertexParticleIndex.SetSize(0);
	m_rgEdgeIndex.SetSize(0);
	m_TriBVTree.ResetDuplicateArrays();
	collideLevelSet.m_rgVertexParticleIndex.SetSize(0);
	collideLevelSet.m_rgEdgeIndex.SetSize(0);
	collideLevelSet.m_TriBVTree.ResetDuplicateArrays();
	//g_D3DApp->Print(n);

	return n;
}

bool COctreeLevelSet::ParticleInOctreeLevel(SNode * pNode, Vector3F & pt, float & fDist)
{
	if (pNode->bLeaf)
		return ParticleInOct(pNode, pt, fDist); 

	Vector3F dxyz((pt - pNode->box.MinPW())/(2.f*pNode->box.HalfWidthsW()));
	DWORD idx = (dxyz.x > .5f) + (dxyz.y > .5f)*2 + (dxyz.z > .5f)*4;
	return ParticleInOctreeLevel(pNode->rgChild[idx], pt, fDist);

	/*for (DWORD i=0; i<8; i++) {
		if (pNode->rgChild[i]->box.PointInAABBoxW(pt))
			return ParticleInOctreeLevel(pNode->rgChild[i], pt, fDist);
	}*/
	return 0;
}

bool COctreeLevelSet::ParticleInLevelSet(Vector3F & particle, float & fDist)
{
	Vector3F gridPt(m_TransformWorldToObj*particle), cpt;
	fDist = sqrt(m_GridBox.ClosestPointSq(gridPt, cpt));
	m_pLastOct = NULL;
	/* Check if the particle is in the grid */
	if (gridPt.x < m_GridMin.x || gridPt.y < m_GridMin.y || gridPt.z < m_GridMin.z || 
		gridPt.x > m_GridMax.x || gridPt.y > m_GridMax.y || gridPt.z > m_GridMax.z) return 0; 
	//gridPt -= m_TransformObjToGrid;

	bool r = ParticleInOctreeLevel(m_pOctree, gridPt, fDist);
	//if (!r)
	//	DebugBreak();
	return r;
}

bool COctreeLevelSet::ParticleInLevelSet_Fast(Vector3F & particle, float & fDist)
{
	Vector3F gridPt(particle), cpt;
	fDist = sqrt(m_GridBox.ClosestPointSq(gridPt, cpt));
	m_pLastOct = NULL;
	/* Check if the particle is in the grid */
	if (gridPt.x < m_GridMin.x || gridPt.y < m_GridMin.y || gridPt.z < m_GridMin.z || 
		gridPt.x > m_GridMax.x || gridPt.y > m_GridMax.y || gridPt.z > m_GridMax.z) return 0; 
	//gridPt -= m_TransformObjToGrid;

	bool r = ParticleInOctreeLevel(m_pOctree, gridPt, fDist);
	//if (!r)
	//	DebugBreak();
	return r;
}

void COctreeLevelSet::ComputeGradientLevel(SNode * pNode, Vector3F & pt, Vector3F & grad)
{
	if (pNode->bLeaf) {
		//Vector3F & dxyz = pt;
		//Vector3F & minP = pNode->box.MinPW();
		//Vector3F & maxP = pNode->box.MaxPW();
		Vector3F dxyz = (pt - pNode->box.MinPW())/(2.f*pNode->box.HalfWidthsW());

		grad = Vector3F(
			InterpolateTrilinear(1.f, dxyz.y, dxyz.z, pNode->V[0], pNode->V[1], pNode->V[2], pNode->V[3], pNode->V[4], pNode->V[5], pNode->V[6], pNode->V[7]) -
			InterpolateTrilinear(0.f, dxyz.y, dxyz.z, pNode->V[0], pNode->V[1], pNode->V[2], pNode->V[3], pNode->V[4], pNode->V[5], pNode->V[6], pNode->V[7]),

			InterpolateTrilinear(dxyz.x, 1.f, dxyz.z, pNode->V[0], pNode->V[1], pNode->V[2], pNode->V[3], pNode->V[4], pNode->V[5], pNode->V[6], pNode->V[7]) -
			InterpolateTrilinear(dxyz.x, 0.f, dxyz.z, pNode->V[0], pNode->V[1], pNode->V[2], pNode->V[3], pNode->V[4], pNode->V[5], pNode->V[6], pNode->V[7]),

			InterpolateTrilinear(dxyz.x, dxyz.y, 1.f, pNode->V[0], pNode->V[1], pNode->V[2], pNode->V[3], pNode->V[4], pNode->V[5], pNode->V[6], pNode->V[7]) -
			InterpolateTrilinear(dxyz.x, dxyz.y, 0.f, pNode->V[0], pNode->V[1], pNode->V[2], pNode->V[3], pNode->V[4], pNode->V[5], pNode->V[6], pNode->V[7]));
		return;
	}

	
	Vector3F dxyz((pt - pNode->box.MinPW())/(2.f*pNode->box.HalfWidthsW()));
	//if (dxyz.x < -1e-3 || dxyz.y < -1e-3 || dxyz.z < -1e-3)
	//	DebugBreak();
	DWORD idx = (dxyz.x > .5f) + (dxyz.y > .5f)*2 + (dxyz.z > .5f)*4;
	return ComputeGradientLevel(pNode->rgChild[idx], pt, grad);
}

Vector3F COctreeLevelSet::ComputeGradient(Vector3F & pt)
{
	Vector3F gridPt(m_TransformWorldToObj*pt), grad;
	if (gridPt.x < m_GridMin.x || gridPt.y < m_GridMin.y || gridPt.z < m_GridMin.z || 
		gridPt.x > m_GridMax.x || gridPt.y > m_GridMax.y || gridPt.z > m_GridMax.z) {

			CAABBox box(m_GridMin, m_GridMax);
			Vector3F out;
			box.ClosestPointSq(gridPt, out);
			return (m_TransformNormalGridToWorld * (out - gridPt)).Normalize();
	}

	ComputeGradientLevel(m_pOctree, gridPt, grad);
	return m_TransformNormalGridToWorld * grad.Normalize();
}

Vector3F COctreeLevelSet::ComputeGradient_Fast(Vector3F & pt)
{
	//Vector3F perfectNor = CLevelSet::ComputeGradient_Fast(pt).Normalize();
	Vector3F gridPt(pt), grad;
	ComputeGradientLevel(m_pOctree, gridPt, grad);
	return m_TransformNormalGridToWorld * grad.Normalize();
}

bool COctreeLevelSet::HandleEdgeEdgeCollision(SEdge & edge, Matrix4x4F & T, CArray<SContactPoint> * CPs, 
	DWORD dwType, float norFlip, CLevelSet & collideLevelSet)
{
	/* If both vertices are admissible, then there is no reason to consider the rest of the edge */
	if ((edge.pVP1->closestDist >= 0 && edge.pVP2->closestDist >= 0 && (edge.pVP1->closestDist + edge.pVP2->closestDist) > edge.edgeLength) ||
		(edge.pVP1->bAdmissible + edge.pVP2->bAdmissible == 2) ) 
		return 0;

	Vector3F v1(m_TransformObjToWorld*edge.pVP1->v);
	Vector3F v2(m_TransformObjToWorld*edge.pVP2->v);
	Vector3F coneDir = m_TransformNormalGridToWorld*edge.coneDir;
	Vector3F edgeDir(v2 - v1);
	float edgeLen = edgeDir.Length();
	int nVertPerEdge = floorf_ASM(edgeLen/m_fMinEdgeLen)-1;
	if (nVertPerEdge <= 0) return 0;
	float t = 1.f/(nVertPerEdge+1);

	/* Find the first interior point starting from v1 */
	Vector3F & edgeDirV1 = edgeDir;
	float fClosestDistV1 = 0;
	SRay ray, _ray;
	SRayIntersectData intersect0, intersect1;
	ray.p = collideLevelSet.m_TransformWorldToObj*v1;
	ray.d = edgeDirV1.MulNormal(edgeDirV1, collideLevelSet.m_TransformWorldToObj);
	if (!collideLevelSet.m_OBBox.OBBoxIntersectRelativeW(ray, 1, &intersect0, &intersect1))
		return 0;
	intersect0.pos = collideLevelSet.m_TransformObjToWorld*intersect0.pos;
	intersect1.pos = collideLevelSet.m_TransformObjToWorld*intersect1.pos;
	//intersect0.t = 0;
	//intersect1.t = 1.f;

	if (m_bUseDumbScheme) {
		m_rgEdgePiece.Resize(0);
		BisectionSearch(collideLevelSet, intersect0.pos, intersect1.pos, coneDir, edge.coneAngle, norFlip, CPs);
		ProcessEdgePieces(collideLevelSet, coneDir, edge.coneAngle, norFlip, CPs);
	}

	Vector3F v1Initial(v1);//intersect0.pos;
	float fAdvance1 = intersect0.t * edgeLen;
	((COctreeLevelSet*)&collideLevelSet)->m_pLastOct = NULL; fClosestDistV1 = 0;
	_ray.p = ray.p;// + fAdvance1 * (ray.d / edgeLen);
	_ray.d = ray.d / edgeLen;
	if (!edge.pVP1->bIntersected)
		if (!MarchExteriorEdgeVertex(collideLevelSet, edge.edgeLength, v1Initial, edgeDirV1, _ray, fClosestDistV1, v1, fAdvance1))
			return 0;
	Vector3F v1Interior;fClosestDistV1 = 0; 
	float fAdvanceOld1 = fAdvance1;
	MarchInteriorEdgeVertex(collideLevelSet, edge.edgeLength, v1Initial, edgeDirV1, _ray, fClosestDistV1, v1Interior, fAdvanceOld1);
	
	/* Find the first interior point starting from v2 */
	float fAdvance2 = edgeLen - intersect1.t * edgeLen;
	Vector3F edgeDirV2(-edgeDir);
	float fClosestDistV2 = 0;
	Vector3F v2Initial(v2);//intersect1.pos;
	((COctreeLevelSet*)&collideLevelSet)->m_pLastOct = NULL;
	_ray.d = -ray.d / edgeLen;
	_ray.p = ray.p + (edgeLen) * (ray.d / edgeLen);
	if (!edge.pVP2->bIntersected)
		if (!MarchExteriorEdgeVertex(collideLevelSet, edge.edgeLength, v2Initial, edgeDirV2, _ray, fClosestDistV2, v2, fAdvance2))
			return 0;
	Vector3F dir((v1Interior - v2));
	float ddd = DotXYZ(dir, edgeDirV2)/edgeLen;
	if (ddd < 3.f*((COctreeLevelSet*)&collideLevelSet)->m_fMinCellSize)
		return ComputeEdgeEdgeIntersection(collideLevelSet, nVertPerEdge, edge.pVP1->bAdmissible, edge.pVP2->bAdmissible, v1, v2, edgeDirV1, coneDir, edge.coneAngle, norFlip, CPs);
	else {
		Vector3F v2Interior; fClosestDistV2 = 0;
		float fAdvanceOld2 = fAdvance2;
		if (!MarchInteriorEdgeVertex(collideLevelSet, edge.edgeLength, v2Initial, edgeDirV2, _ray, fClosestDistV2, v2Interior, fAdvanceOld2))
			return 0;

		return ComputeEdgeEdgeIntersection(collideLevelSet, nVertPerEdge, edge.pVP1->bAdmissible, 0, v1, v1Interior, edgeDirV1, coneDir, edge.coneAngle, norFlip, CPs)
			&& ComputeEdgeEdgeIntersection(collideLevelSet, nVertPerEdge, edge.pVP2->bAdmissible, 0, v2, v2Interior, edgeDirV2, coneDir, edge.coneAngle, norFlip, CPs);
	}
}

bool COctreeLevelSet::MarchExteriorEdgeVertex(CLevelSet & collideLevelSet, float fEdgeLength, 
	Vector3F & v1, Vector3F & edgeDir, SRay & ray, float & fClosestDist, Vector3F & finalPt, float & fAdvance)
{
	SEdgeParticle wEP;
	Vector3F unitEdgeDir(edgeDir/fEdgeLength);
	wEP.v = v1 + fAdvance*unitEdgeDir;
	
	SRay tmpRay(ray);
	tmpRay.p = ray.p + fAdvance*ray.d;

	SRayIntersectData data;
	while (!collideLevelSet.ParticleInLevelSet(wEP.v, fClosestDist)) {
		float stepSize = ((COctreeLevelSet*)&collideLevelSet)->m_fMinCellSize;
		if (((COctreeLevelSet*)&collideLevelSet)->m_pLastOct) { //Should check that it is not null
			if (((COctreeLevelSet*)&collideLevelSet)->m_pLastOct->box.InteresectFromInside(tmpRay, data)) {
				stepSize = data.t;
			}
		}
		float fAClosestDist = Abs(fClosestDist);
		stepSize = Max(fAClosestDist, stepSize);
		fAdvance += stepSize + 1e-3;

		if (fAdvance > fEdgeLength) {
			return 0;
		}
		wEP.v = v1 + fAdvance*unitEdgeDir;
		tmpRay.p = ray.p + (fAdvance + 1e-3)*ray.d;
	}
	finalPt = wEP.v;
	return 1;
}

bool COctreeLevelSet::MarchInteriorEdgeVertex(CLevelSet & collideLevelSet, float fEdgeLength, 
	Vector3F & v1, Vector3F & edgeDir, SRay & ray, float & fClosestDist, Vector3F & finalPt, float & fAdvance)
{
	SEdgeParticle wEP;
	Vector3F unitEdgeDir(edgeDir/fEdgeLength);
	wEP.v = v1  + fAdvance*unitEdgeDir;
	
	SRay tmpRay(ray);
	tmpRay.p = ray.p + fAdvance*ray.d;

	SRayIntersectData data;
	while (collideLevelSet.ParticleInLevelSet(wEP.v, fClosestDist)) {//Make closest dist positive when below
		float stepSize = ((COctreeLevelSet*)&collideLevelSet)->m_fMinCellSize;
		if (((COctreeLevelSet*)&collideLevelSet)->m_pLastOct) {// Should check that it is not null
			if (((COctreeLevelSet*)&collideLevelSet)->m_pLastOct->box.InteresectFromInside(tmpRay, data)) {
				stepSize = data.t;
			}
		}
		float fAClosestDist = Abs(fClosestDist);
		stepSize = Max(fAClosestDist, stepSize);
		fAdvance += stepSize + 1e-3;

		if (fAdvance > fEdgeLength) {
			finalPt = wEP.v;
			return 1;
		}
		finalPt = wEP.v;
		wEP.v = v1 + fAdvance*unitEdgeDir;
		tmpRay.p = ray.p + fAdvance*ray.d;
	}
	return 1;
}

void COctreeLevelSet::DrawOctreeLevel(DWORD dwMaxLevel, SNode * pOctree, DWORD dwCurLevel)
{
	if (dwCurLevel == dwMaxLevel || pOctree->bLeaf) {
		Vector3F & pt = pOctree->box.CenterPointW();
		if (!pOctree)
			DebugBreak();
	
	//if (pOctree->box.CenterPointW().y > .0f && pOctree->box.CenterPointW().x < -0.f && abs(pOctree->box.CenterPointW().z) < 8.f) 
	//if (pOctree->box.CenterPointW().y > .85f && pOctree->box.CenterPointW().x < -5.45f && abs(pOctree->box.CenterPointW().z) < 6.f)
		float fDist;
		if (ParticleInOct(pOctree, pt, fDist)) // && (pOctree->box.CenterPointW().y > 1.0f && pOctree->box.CenterPointW().x > -0.f && pOctree->box.CenterPointW().z < 0.f)) {
		//	ParticleInOct(pOctree, pt, fDist);
			CCollisionGraphics::DrawBox(m_TransformObjToWorld*pt, pOctree->box.HalfWidthsW(), m_TransformNormalGridToWorld);
		//
		return;
	}

	for (DWORD i=0; i<8; i++) {
		if (!pOctree->rgChild[i])
			DebugBreak();
		DrawOctreeLevel(dwMaxLevel, pOctree->rgChild[i], dwCurLevel+1);
	}
}

void COctreeLevelSet::DrawOctLevelSet(DWORD dwLevel)
{
	DrawOctreeLevel(dwLevel, m_pOctree, 0);
}

void COctreeLevelSet::DestroyOctree(SNode * pNode)
{
	if (pNode->bLeaf) return;

	for (DWORD i=0; i<8; i++) {
		DestroyOctree(pNode->rgChild[i]);
		delete pNode->rgChild[i];
		pNode->rgChild[i];
	}
}

void COctreeLevelSet::DestroyLevelSet()
{
	if (m_pOctree) {
		CLevelSet::DestroyLevelSet();

		DestroyOctree(m_pOctree);
		delete m_pOctree;
		m_pOctree = NULL;

		m_TriBVTree.DestroyBVTree();
		m_KdTree.DestroyBVTree();

		m_rgVertexParticleIndex.Clear();
		m_rgEdgeIndex.Clear();
		//m_rgVertexParticleDuplicate.Clear();
		//m_rgEdgeDuplicate.Clear();
	}
}

void COctreeLevelSet::BisectionSearch(CLevelSet & collideLevelSet, Vector3F & v1, Vector3F & v2, Vector3F & coneDir, float coneAngle, float norFlip, CArray<SContactPoint> * CPs)
{
	float d1, d2, edgeLen;
	edgeLen = (v1 - v2).Length();
	collideLevelSet.ParticleInLevelSet(v1, d1);
	collideLevelSet.ParticleInLevelSet(v2, d2);
	if (d1 + d2 > edgeLen) return;
	float bigger = max(m_fMinCellSize, ((COctreeLevelSet*)&collideLevelSet)->m_fMinCellSize);
	if (edgeLen < bigger) {
		if (d1 < 0) {
			Vector3F computedNor = collideLevelSet.ComputeGradient(v1);
			if (IsNormalAdmissible(coneDir, coneAngle, computedNor, .2f)) {
				m_rgEdgePiece.PushBack(SEdgePiece(v1, v2, d1, d2));

				/*SContactPoint cp;
				cp.iNor = norFlip*computedNor;
				cp.iPos = v1;
				cp.dist = d1;
				CPs->PushBack(cp);*/
			}
		}
		else if (d2 < 0) {
			Vector3F computedNor = collideLevelSet.ComputeGradient(v2);
			if (IsNormalAdmissible(coneDir, coneAngle, computedNor, .2f)) {
				m_rgEdgePiece.PushBack(SEdgePiece(v1, v2, d1, d2));

				/*SContactPoint cp;
				cp.iNor = norFlip*computedNor;
				cp.iPos = v2;
				cp.dist = d2;
				CPs->PushBack(cp);*/
			}
		}
		return;
	}

	BisectionSearch(collideLevelSet, v1, .5f*(v1+v2), coneDir, coneAngle, norFlip, CPs);
	BisectionSearch(collideLevelSet, .5f*(v1+v2), v2, coneDir, coneAngle, norFlip, CPs);
}

void COctreeLevelSet::HandleFace(CLevelSet & collideLevelSet, STriangleF & face, float norFlip, CArray<SContactPoint> * CPs)
{
	float fClosestDist;
	Vector3F ctd(face.Centroid());
	//float fClosestDist2;
	/*bool result1 = collideLevelSet.ParticleInLevelSet(face.Centroid(), fClosestDist2);
	bool result2 = collideLevelSet.CLevelSet::ParticleInLevelSet(face.Centroid(), fClosestDist);
	if (result1 != result2) {
		collideLevelSet.ParticleInLevelSet(face.Centroid(), fClosestDist2);
	}*/
	/*if (!collideLevelSet.ParticleInLevelSet(face.Centroid(), fClosestDist)) {
		m_pCGraphics->DrawTriangle(face.vPosW[0], face.vPosW[1], face.vPosW[2], &Vector3F(1.f, 0, 0));
		return;
	}*/
	collideLevelSet.ParticleInLevelSet(face.Centroid(), fClosestDist);
	float fCentroidRadius = face.CentroidRadius();
	
	//Vector3F gradDir = Vector3F(0, -norFlip*1.f, 0);
	//Vector3F gradDir = Vector3F(-norFlip*1.f, 0, 0);
	Vector3F gradDir = collideLevelSet.ComputeGradient(ctd);
	Vector3F v1(face.vPosW[0] - ctd);
	float dp1 = DotXYZ(gradDir, v1);
	Vector3F v2(face.vPosW[1] - ctd);
	float dp2 = DotXYZ(gradDir, v2);
	Vector3F v3(face.vPosW[2] - ctd);
	float dp3 = DotXYZ(gradDir, v3);
	float maxDP = max(dp1, dp2);
	maxDP = max(maxDP, dp3);

	//float ctdDist = face.CentroidRadius();
	if (fClosestDist > 0 && fClosestDist < (maxDP +.1f)) {
		return;
	}
	
	if (fCentroidRadius < fClosestDist && fClosestDist > 0) {
		if (m_bDrawTris) m_pCGraphics->DrawTriangle(face.vPosW[0], face.vPosW[1], face.vPosW[2], &Vector3F(1.f, 0, 0));
		return;
	}

	if (fClosestDist < 0) {
		SContactPoint cp;
		Vector3F nor = face.Normal();
		cp.iNor = norFlip*face.Normal();

		cp.iPos = ctd;//face.vPosW[0];
		Vector3F computedNor = collideLevelSet.ComputeGradient(ctd);
		bool bFound = 0;
		if (collideLevelSet.ParticleInLevelSet(cp.iPos, fClosestDist) && IsNormalAdmissible(computedNor, .95f, nor)) {
			cp.iPos = face.vPosW[1];
			CPs->PushBack(cp); 
			cp.iPos = face.vPosW[2];
			CPs->PushBack(cp); 
			cp.iPos = face.vPosW[3];
			CPs->PushBack(cp); 
			bFound=1;
		}
		cp.iPos = ctd;//face.vPosW[1];

		/*if (collideLevelSet.ParticleInLevelSet(cp.iPos, fClosestDist) && IsNormalAdmissible(computedNor, .95f, nor)) {CPs->PushBack(cp); bFound=1; }
		cp.iPos = face.vPosW[2];

		if (collideLevelSet.ParticleInLevelSet(cp.iPos, fClosestDist) && IsNormalAdmissible(computedNor, .95f, nor)) {CPs->PushBack(cp); bFound=1; }*/

		if (bFound) {
			if (m_bDrawTris) m_pCGraphics->DrawTriangle(face.vPosW[0], face.vPosW[1], face.vPosW[2], &Vector3F(0, 1.f, 0));
			return;
		}
	}

	float fTriSize = face.MaxEdgeLength();
	if ((((COctreeLevelSet*)&collideLevelSet)->m_pLastOct != NULL && fTriSize <= 2.f*((COctreeLevelSet*)&collideLevelSet)->m_pLastOct->box.HalfWidthsW().z) ||
		fTriSize <= 2.f*((COctreeLevelSet*)&collideLevelSet)->m_fMinCellSize) {

		SContactPoint cp;
		Vector3F nor = face.Normal();
		cp.dist = fClosestDist;
		cp.iNor = norFlip*face.Normal();

		cp.iPos = face.vPosW[0];
		Vector3F computedNor = collideLevelSet.ComputeGradient(cp.iPos);
		bool bFound = 0;
		if (collideLevelSet.ParticleInLevelSet(cp.iPos, fClosestDist) && IsNormalAdmissible(computedNor, .95f, nor)) {
			CPs->PushBack(cp); bFound=1; }
		cp.iPos = face.vPosW[1];
		computedNor = collideLevelSet.ComputeGradient(cp.iPos);
		if (collideLevelSet.ParticleInLevelSet(cp.iPos, fClosestDist) && IsNormalAdmissible(computedNor, .95f, nor)) {
			CPs->PushBack(cp); bFound=1; }
		cp.iPos = face.vPosW[2];
		computedNor = collideLevelSet.ComputeGradient(cp.iPos);
		if (collideLevelSet.ParticleInLevelSet(cp.iPos, fClosestDist) && IsNormalAdmissible(computedNor, .95f, nor)) {
			this;
			CPs->PushBack(cp); bFound=1; }

		if (bFound) {
			if (m_bDrawTris) m_pCGraphics->DrawTriangle(face.vPosW[0], face.vPosW[1], face.vPosW[2], &Vector3F(0, 1.f, 0)); }
		else {
			if (m_bDrawTris) m_pCGraphics->DrawTriangle(face.vPosW[0], face.vPosW[1], face.vPosW[2], &Vector3F(1.f, 0, 0)); }
		return;
	}

	STriangleF rgSubdividedTri[4];
	SubdivideTriangle(face, rgSubdividedTri);

	HandleFace(collideLevelSet, rgSubdividedTri[0], norFlip, CPs);
	HandleFace(collideLevelSet, rgSubdividedTri[1], norFlip, CPs);
	HandleFace(collideLevelSet, rgSubdividedTri[2], norFlip, CPs);
	HandleFace(collideLevelSet, rgSubdividedTri[3], norFlip, CPs);
}

void COctreeLevelSet::ProcessEdgePieces(CLevelSet & collideLevelSet, Vector3F & coneDir, float coneAngle, float norFlip, CArray<SContactPoint> * CPs)
{
	DWORD i=0;
	//if (m_rgEdgePiece[0].d[0] > 0) i++;
	for (DWORD end=m_rgEdgePiece.GetSize(); i<end; i++) {
		/* March to find edge piece that is outside on the right */
		Vector3F & v1 = m_rgEdgePiece[i].v[0];
		float d1 = m_rgEdgePiece[i].d[0];
		while (m_rgEdgePiece[i].d[1] < 0) {if (i+1>=end) break; i++;}

		Vector3F & v2 = m_rgEdgePiece[i].v[1];
		float d2 = m_rgEdgePiece[i].d[1];

		ComputeEdgeEdgeIntersection(collideLevelSet, 1, 0, 0, v1, v2, v2-v1, coneDir, coneAngle, norFlip, CPs);
	}
}


};




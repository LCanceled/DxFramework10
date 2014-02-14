
#include "DxUtBVTree.h"
#include "DxUtMesh.h"
#include "TriTriTest.h"
#include "DxUtPolygon.h"
#include <map>

//For use in BVTree Intersect and
//find Bounding Volume Transforms
#define IsLeaf(pNode) ((!pNode->pChildB))

namespace DxUt {

BVTree::BVTree():m_pTree(0), m_nBranchNodes(0), m_nLeafNodes(0), m_rgBV(0), m_rgTri(0), m_rgTransformedTri(0), m_rgAdj(0), m_Trans(0.f, 0.f, 0.f),
	m_fScale(1.f), m_pCollideBVTree(0), m_rgCP(0), m_bConvexCollision(0), m_nBVBVTests(0), m_nTriTriTests(0) /*Fix*/
{
	dwCounter = 0;
	m_TriTransform.MIdenity();
	m_Rot.MIdenity();
	m_InvRot.MIdenity();
}

void AverageNormals(Vector3F & nextNor, Vector3F & preNor, Vector3F & avgNor, DWORD & nNor)
{
	/* Do not average normals that share the same normal */
	if (DotXYZ(nextNor, preNor) < .99f) {
		avgNor += nextNor;
		nNor++;
	}
}

Vector3F ComputeConeNormal(DWORD dwInitialTri, DWORD dwInitialEdge, STriangleF * rgTri, DWORD * rgAdj, float & fTheta)
{
	DWORD nNor = 0;
	Vector3F avgNor(0,0,0);
	DWORD dwNextEdge = dwInitialEdge;
	DWORD dwNextTri = dwInitialTri;
	Vector3F preNor(rgTri[dwInitialTri].Normal());
	Vector3F nextNor(rgTri[dwInitialTri].Normal());
	//STriangleF & preTri = rgTri[dwInitialTri];
	do {
		nextNor = rgTri[dwNextTri].Normal();
		AverageNormals(nextNor, preNor, avgNor, nNor);

		DWORD dwPreTri = dwNextTri;
		dwNextTri = rgAdj[3*dwNextTri + dwNextEdge];

		/* Check the three edges */
		dwNextEdge = 0;
		while (rgAdj[3*dwNextTri+dwNextEdge] != dwPreTri) dwNextEdge++;
		dwNextEdge = (dwNextEdge + 1) % 3;
		
		if (dwNextTri == dwInitialTri) {
			nextNor = nextNor = rgTri[dwNextTri].Normal();
			break;
		}
		preNor = nextNor;
	} while (dwNextTri !=dwInitialTri);
	AverageNormals(nextNor, preNor, avgNor, nNor);
	avgNor = avgNor.Normalize();

	/* Compute the maximum divergence (fTheta) among the avgNor and the polygon normals */
	fTheta = 1.;
	dwNextTri = dwInitialTri;
	dwNextEdge = dwInitialEdge;
	do {
		/* Find the maximum of theta */
		Vector3F & n = rgTri[dwNextTri].Normal();
		float d = DotXYZ(n, avgNor);
		fTheta = min(fTheta, d);

		DWORD dwPreTri = dwNextTri;
		dwNextTri = rgAdj[3*dwNextTri + dwNextEdge];

		/* Check the three edges */
		dwNextEdge = 0;
		while (rgAdj[3*dwNextTri+dwNextEdge] != dwPreTri) dwNextEdge++;
		dwNextEdge = (dwNextEdge + 1) % 3;

	} while (dwNextTri !=dwInitialTri);

	return avgNor;
}
/*
void ComputePolygons(CArray<CContactRegions::SPolygon3FEx> & rgPolygon, DWORD nTri, STriangleF * rgTri, DWORD * rgAdj)
{
	rgPolygon.Reserve(nTri);
	for (DWORD i=0; i<nTri; i++) {
		float fTheta0; Vector3F nor0(ComputeConeNormal(i, 0, rgTri, rgAdj, fTheta0));
		float fTheta1; Vector3F nor1(ComputeConeNormal(i, 1, rgTri, rgAdj, fTheta1));
		float fTheta2; Vector3F nor2(ComputeConeNormal(i, 2, rgTri, rgAdj, fTheta2));

		rgPolygon.PushBack();
		rgPolygon.GetBack().Init(
			rgTri[i + 0], rgTri[rgAdj[3*i + 0]], rgTri[rgAdj[3*i + 1]], rgTri[rgAdj[3*i + 2]], 
			rgAdj[3*i + 0], rgAdj[3*i + 1], rgAdj[3*i + 2],
			fTheta0, nor0, fTheta1, nor1, fTheta2, nor2
			);
	}
}*/

void BVTree::CreateBVTree(ID3DX10Mesh * pMesh, DWORD dwStride, bool bMakeKdTree)
{
	Assert(!m_pTree, "BVTree::CreateBVTree tree must be destroyed before creating a new one.");
	
	DWORD nTri = pMesh->GetFaceCount();
	DWORD nVert = 3*pMesh->GetFaceCount();
	Vector3F * rgVert = new Vector3F[nVert];
	ExtractVertexTriangleListFromMesh(pMesh, rgVert, dwStride);

	m_rgAdj = new DWORD[3*nVert];
	ExtractAdjanceyFromMesh(pMesh, m_rgAdj);
	
	m_pTree = new BranchNode;
	m_rgBV = new COBBox[2*nTri];
	m_nBV = 0;
	m_rgTri = new STriangleF[nTri];
	m_rgTransformedTri = new STriangleF[nTri];
	m_nTri = nTri;
	m_bKdTree = bMakeKdTree;

	DWORD * rgFaceIndex = new DWORD[nVert/3];
	for (DWORD i=0; i<nTri; i++) rgFaceIndex[i] = i;

	BuildBVTree(m_pTree, rgVert, rgFaceIndex, nVert, 0);

	Matrix4x4F rot;
	m_rgBV[0].RotationW(rot);
	m_rgBV[0].m_RotVecW[0] = rot.GetColumnVec3F(0);
	m_rgBV[0].m_RotVecW[1] = rot.GetColumnVec3F(1);
	m_rgBV[0].m_RotVecW[2] = rot.GetColumnVec3F(2);

	delete[] rgVert;
	rgVert = NULL;
	delete[] rgFaceIndex;
	rgFaceIndex = NULL;

	m_TopLevelOBBoxW = *m_pTree->pOBB;

	/*UINT nVert = 3*pMesh->GetFaceCount();
	UINT nTri = pMesh->GetFaceCount();

	Vector3F * rgVert = new Vector3F[nVert];
	m_rgAdj = new DWORD[3*nTri];

	ExtractVertexTriangleListFromMesh(pMesh, rgVert, dwStride);
	ExtractAdjanceyFromMesh(pMesh, m_rgAdj);

	DWORD * rgFaceIndex = new DWORD[nVert/3];
	for (DWORD i=0; i<nTri; i++) rgFaceIndex[i] = i;

	m_pTree = new BranchNode;
	m_rgBV = new COBBox[2*nTri];
	m_nBV = 0;
	m_rgTri = new STriangleF[nTri];
	m_rgTransformedTri = new STriangleF[nTri];
	m_nTri = nTri;

	BuildBVTree(m_pTree, rgVert, rgFaceIndex, nVert);

	Matrix4x4F rot;
	m_rgBV[0].RotationW(rot);
	m_rgBV[0].m_RotVecW[0] = rot.GetColumnVec3F(0);
	m_rgBV[0].m_RotVecW[1] = rot.GetColumnVec3F(1);
	m_rgBV[0].m_RotVecW[2] = rot.GetColumnVec3F(2);

	delete[] rgVert;
	rgVert = NULL;
	delete[] rgFaceIndex;
	rgFaceIndex = NULL;

	ComputePolygons(m_rgPolygon, m_nTri, m_rgTri, m_rgAdj);
	
	m_TopLevelOBBoxW = *m_pTree->pOBB;*/
}

void BVTree::CreateBVTree(STriangleF * rgTri, DWORD nTri, DWORD * adj, bool bMakeKdTree)
{
	Assert(!m_pTree, "BVTree::CreateBVTree tree must be destroyed before creating a new one.");

	DWORD nVert = 3*nTri;
	DWORD * rgFaceIndex = new DWORD[nVert/3];
	for (DWORD i=0; i<nTri; i++) rgFaceIndex[i] = i;

	m_pTree = new BranchNode;
	m_rgBV = new COBBox[2*nTri];
	m_nBV = 0;
	m_rgTri = new STriangleF[nTri];
	m_rgTransformedTri = new STriangleF[nTri];
	m_nTri = nTri;
	m_bKdTree = bMakeKdTree;

	__debugbreak();
	// This code does not work because rgTri != Vector3F array
	BuildBVTree(m_pTree, (Vector3F*)rgTri, rgFaceIndex, nVert, 0);

	Matrix4x4F rot;
	m_rgBV[0].RotationW(rot);
	m_rgBV[0].m_RotVecW[0] = rot.GetColumnVec3F(0);
	m_rgBV[0].m_RotVecW[1] = rot.GetColumnVec3F(1);
	m_rgBV[0].m_RotVecW[2] = rot.GetColumnVec3F(2);

	delete[] rgFaceIndex;
	rgFaceIndex = NULL;

	//ComputePolygons(m_rgPolygon, m_nTri, m_rgTri, m_rgAdj);
	
	m_TopLevelOBBoxW = *m_pTree->pOBB;
}

//The vertices must be in a triangle list order
//When the function returns, the vertices will
//not be in a triangle list order
void BVTree::CreateBVTree(Vector3F * rgVert, DWORD nVert, DWORD * adj, bool bMakeKdTree)
{
	Assert(!m_pTree, "BVTree::CreateBVTree tree must be destroyed before creating a new one.");

	if (nVert % 3) {DxUtSendError("The nVert must be divisible by 3."); }
	DWORD nTri = nVert/3;

	m_rgAdj = new DWORD[3*nTri];
	memcpy(m_rgAdj, adj, sizeof(DWORD)*3*nTri);

	DWORD * rgFaceIndex = new DWORD[nVert/3];
	for (DWORD i=0; i<nTri; i++) rgFaceIndex[i] = i;

	m_pTree = new BranchNode;
	m_rgBV = new COBBox[2*nTri];
	m_nBV = 0;
	m_rgTri = new STriangleF[nTri];
	m_rgTransformedTri = new STriangleF[nTri];
	m_nTri = nTri;
	m_bKdTree = bMakeKdTree;

	BuildBVTree(m_pTree, rgVert, rgFaceIndex, nVert, 0);

	Matrix4x4F rot;
	m_rgBV[0].RotationW(rot);
	m_rgBV[0].m_RotVecW[0] = rot.GetColumnVec3F(0);
	m_rgBV[0].m_RotVecW[1] = rot.GetColumnVec3F(1);
	m_rgBV[0].m_RotVecW[2] = rot.GetColumnVec3F(2);

	delete[] rgFaceIndex;
	rgFaceIndex = NULL;

	//ComputePolygons(m_rgPolygon, m_nTri, m_rgTri, m_rgAdj);
	
	m_TopLevelOBBoxW = *m_pTree->pOBB;
}

/*void BVTree::LoadBVFromFile(char * file)
{
}

void BVTree::SaveBVToFile(char * file)
{
}*/

//The transform is assumed be from the base coordinate system to the new system
//rT must be a matrix of a rotation followed by a rotation 
void BVTree::SetTransform(Matrix4x4F & rT, FLOAT fScl)
{
	m_Rot = rT;
	m_Rot.m[0][3] = 0.f;
	m_Rot.m[1][3] = 0.f;
	m_Rot.m[2][3] = 0.f;
	m_InvRot = rT.Transpose();

	m_Trans = Vector3F(rT.GetColumnVec3F(3));
	m_fScale = fScl;

	Matrix4x4F sclM; 
	sclM.MScaling(fScl,fScl,fScl);
	m_TriTransform = sclM*m_Rot;
	m_TriTransform.m[0][3] = m_Trans.x;
	m_TriTransform.m[1][3] = m_Trans.y;
	m_TriTransform.m[2][3] = m_Trans.z;

	m_TopLevelOBBoxW.TransformOBBW(m_Trans, m_Rot, m_fScale);
}

//The transform is assumed be from the base coordinate system to the new system
void BVTree::SetTransform(Matrix4x4F & rot, Vector3F & trans, FLOAT fScl)
{
	m_Rot = rot;
	m_InvRot = rot.Transpose();

	m_Trans = trans;
	m_fScale = fScl;

	Matrix4x4F sclM; 
	sclM.MScaling(fScl,fScl,fScl);
	m_TriTransform = sclM*m_Rot;
	m_TriTransform.m[0][3] = m_Trans.x;
	m_TriTransform.m[1][3] = m_Trans.y;
	m_TriTransform.m[2][3] = m_Trans.z;

	m_TopLevelOBBoxW.TransformOBBW(m_Trans, m_Rot, m_fScale);
}

void BVTree::TransformTriangles()
{
	for (DWORD i=0; i<m_nTri; i++) {
		m_rgTransformedTri[i] = m_rgTri[i];
		m_rgTransformedTri[i].Transform(m_TriTransform);
	}
}

//rgCPs should have some memory reserved as a guess at the total number of contact points
//Returns 0 when there are no collision; otherwise, it returns the number of collision pairs
DWORD BVTree::BVCollision(BVTree & collideBVTree, CArray<SContactPoint> * rgCP)
{
	m_pCollideBVTree = &collideBVTree;
	m_rgCP = rgCP;
	m_nBVBVTests = m_nTriTriTests = 0;

	Matrix4x4F rot;
	m_rgBV[0].RotationW(rot);
	rot = m_Rot*rot;
	Vector3F pos(m_fScale*(m_Rot*m_rgBV[0].m_CenterW) + m_Trans);

	Matrix4x4F cRot;
	collideBVTree.m_rgBV[0].RotationW(cRot);
	cRot = collideBVTree.m_Rot*cRot;
	Vector3F cPos(collideBVTree.m_fScale*(collideBVTree.m_Rot*
		collideBVTree.m_rgBV[0].m_CenterW) + collideBVTree.m_Trans);

	//Compute the pos, rot, scl relative to the this BVTree
	Matrix4x4F rotR(rot.Transpose());
	Vector3F posR((rotR*(cPos - pos))/m_fScale);
	rotR = rotR*cRot;
	//m_RelativeScale = collideBVTree.m_fScale/m_fScale;
	FLOAT fSclR = collideBVTree.m_fScale/m_fScale;

	DWORD oldSize = m_rgCP->GetSize();
	FindBVCollisions(m_pTree, collideBVTree.m_pTree, rotR, posR, fSclR);

	return m_rgCP->GetSize() - oldSize;
}

static int stillGo = 1;

DWORD BVTree::BVCollisionConvex(BVTree & collideBVTree, CArray<SContactPoint> * rgCP)
{
	m_pCollideBVTree = &collideBVTree;
	m_rgCP = rgCP;
	m_nBVBVTests = m_nTriTriTests = 0;

	Matrix4x4F rot;
	m_rgBV[0].RotationW(rot);
	rot = m_Rot*rot;
	Vector3F pos(m_fScale*(m_Rot*m_rgBV[0].m_CenterW) + m_Trans);

	Matrix4x4F cRot;
	collideBVTree.m_rgBV[0].RotationW(cRot);
	cRot = collideBVTree.m_Rot*cRot;
	Vector3F cPos(collideBVTree.m_fScale*(collideBVTree.m_Rot*
		collideBVTree.m_rgBV[0].m_CenterW) + collideBVTree.m_Trans);

	//Compute the pos, rot, scl relative to the this BVTree
	Matrix4x4F rotR(rot.Transpose());
	Vector3F posR((rotR*(cPos - pos))/m_fScale);
	rotR = rotR*cRot;
	//m_RelativeScale = collideBVTree.m_fScale/m_fScale;
	FLOAT fSclR = collideBVTree.m_fScale/m_fScale;

	DWORD oldSize = m_rgCP->GetSize();
	if (FindBVCollisionsConvex(m_pTree, collideBVTree.m_pTree, rotR, posR, fSclR)) {
		int a=0;
	} 
	//if (!stillGo) return -2;

	return m_rgCP->GetSize() - oldSize;
}

void BVTree::BuildBVTree(BranchNode * pNode, Vector3F * rgVert, DWORD * rgFaceIndex, DWORD nVert, DWORD dwLevel)
{
	if (nVert > 3) {
		m_nBranchNodes++;

		FLOAT fMean = 0;
		Vector3F axis;
		ComputeOBB(rgVert, nVert, m_rgBV[m_nBV], fMean, axis, dwLevel);

		DWORD dwSplitIndex = 0;
		PartitionVert(rgVert, rgFaceIndex, nVert, axis, fMean, dwSplitIndex);
		pNode->pOBB = &m_rgBV[m_nBV++];

		pNode->pChildA = new BranchNode;
		BuildBVTree(pNode->pChildA, rgVert, rgFaceIndex, dwSplitIndex, dwLevel+1);

		pNode->pChildB = new BranchNode;
		BuildBVTree(pNode->pChildB, &rgVert[dwSplitIndex], &rgFaceIndex[dwSplitIndex/3], nVert-dwSplitIndex, dwLevel);

		Matrix4x4F rotT, cRot;
		pNode->pOBB->RotationW(rotT);
		rotT.MTranspose();

		pNode->pChildA->pOBB->RotationW(cRot);
		cRot = rotT*cRot;

		pNode->pChildA->pOBB->m_RotVecW[0] = cRot.GetColumnVec3F(0);
		pNode->pChildA->pOBB->m_RotVecW[1] = cRot.GetColumnVec3F(1);
		pNode->pChildA->pOBB->m_RotVecW[2] = cRot.GetColumnVec3F(2);
		//pNode->pChildA->pOBB->m_RotVecL[0] = cRot.GetColumnVec3F(0);
		//pNode->pChildA->pOBB->m_RotVecL[1] = cRot.GetColumnVec3F(1);
		//pNode->pChildA->pOBB->m_RotVecL[2] = cRot.GetColumnVec3F(2);

		pNode->pChildA->pOBB->m_CenterW = 
			rotT*(pNode->pChildA->pOBB->m_CenterW - pNode->pOBB->m_CenterW);
		//pNode->pChildA->pOBB->m_CenterL = 
		//	rotT*(pNode->pChildA->pOBB->m_CenterW - pNode->pOBB->m_CenterW);

		pNode->pChildB->pOBB->RotationW(cRot);
		cRot = rotT*cRot;

		pNode->pChildB->pOBB->m_RotVecW[0] = cRot.GetColumnVec3F(0);
		pNode->pChildB->pOBB->m_RotVecW[1] = cRot.GetColumnVec3F(1);
		pNode->pChildB->pOBB->m_RotVecW[2] = cRot.GetColumnVec3F(2);
		//pNode->pChildB->pOBB->m_RotVecL[0] = cRot.GetColumnVec3F(0);
		//pNode->pChildB->pOBB->m_RotVecL[1] = cRot.GetColumnVec3F(1);
		//pNode->pChildB->pOBB->m_RotVecL[2] = cRot.GetColumnVec3F(2);

		pNode->pChildB->pOBB->m_CenterW = 
			rotT*(pNode->pChildB->pOBB->m_CenterW - pNode->pOBB->m_CenterW);
		//pNode->pChildB->pOBB->m_CenterL = 
		//	rotT*(pNode->pChildB->pOBB->m_CenterW - pNode->pOBB->m_CenterW);

	}
	else {
		m_nLeafNodes++;
 
		FLOAT fMean;
		Vector3F axis;
		ComputeOBB(rgVert, nVert, m_rgBV[m_nBV], fMean, axis, dwLevel);
		pNode->pOBB = &m_rgBV[m_nBV++];
		
		STriangleF & tri = m_rgTri[*rgFaceIndex];
		tri.vPosW[0] = rgVert[0];
		tri.vPosW[1] = rgVert[1];
		tri.vPosW[2] = rgVert[2];
		tri.dwTriangle = *rgFaceIndex;
		((LeafNode*)pNode)->pTri = &tri;
	}
}

void BVTree::ComputeOBB(Vector3F * rgVert, DWORD nVert, COBBox & oBB, FLOAT & fMean, Vector3F & axis, DWORD dwLevel)
{
	Vector3F rotVec[3];
	bool bNotTri = 1;
	if (m_bKdTree) {
		//Compute the center and axis vectors based upon the covariance
		Matrix4x4F cov;		//Covariance of the triangle mesh
		Vector3F ctd;		//Centroid of the triangle mesh
		CovarianceTriangles3x3F(rgVert, nVert, cov, ctd);
		
		static Vector3F axes[] = {Vector3F(1.f, 0, 0), Vector3F(0, 1.f, 0), Vector3F(0, 0, 1.f) };
		memcpy(rotVec, axes, 3*sizeof(Vector3F));
		axis = axes[dwLevel % 3];
		fMean = DotXYZ(ctd, axis);
	}
	else if (nVert != 3) {
		//Compute the center and axis vectors based upon the covariance
		Matrix4x4F cov;		//Covariance of the triangle mesh
		Vector3F ctd;		//Centroid of the triangle mesh
		CovarianceTriangles3x3F(rgVert, nVert, cov, ctd);

		Matrix4x4F A;		//Eigen vectors
		Vector3F lambda;	//Eigen values
		JacobiTransformation3x3F(cov, A, lambda);

		rotVec[2] = A.GetColumnVec3F(2);
		rotVec[1] = A.GetColumnVec3F(1);
		rotVec[0] = A.GetColumnVec3F(0);

		DWORD dwCol;
		if (lambda.x > lambda.y) {
			if (lambda.x > lambda.z)
				dwCol = 0;
			else dwCol = 2;
		} else if (lambda.y > lambda.z) {
			dwCol = 1;
		} else dwCol = 2;
		axis = A.GetColumnVec3F(dwCol);
		fMean = DotXYZ(ctd, axis);
	}
	else {
		//Compute the OBB for a triangle
		Vector3F e1, e2, e3, p1, p2, p3;
		e1 = rgVert[1] - rgVert[0];
		e2 = rgVert[2] - rgVert[0];
		e3 = rgVert[1] - rgVert[2];

		if (e1.LengthSq() > e2.LengthSq()) {
			if (e1.LengthSq() > e3.LengthSq()) {
				p1 = rgVert[0], p2 = rgVert[1], p3 = rgVert[2]; 
			} 
			else { 
				p1 = rgVert[2], p2 = rgVert[1], p3 = rgVert[0]; 
			}
		}
		else if (e2.LengthSq() > e3.LengthSq()) {
			p1 = rgVert[0], p2 = rgVert[2], p3 = rgVert[1]; 
		}
		else { 
			p1 = rgVert[2], p2 = rgVert[1], p3 = rgVert[0]; 
		}

		rotVec[0] = (p2 - p1).Normalize();

		Vector3F v1(rgVert[1] - rgVert[0]);
		Vector3F v2(rgVert[2] - rgVert[0]);
		rotVec[1] = CrossXYZ(v1, v2);
		rotVec[1] = rotVec[1].Normalize();

		rotVec[2] = CrossXYZ(rotVec[0], rotVec[1]);
		rotVec[2] = rotVec[2].Normalize();

		fMean = 0.f;
		bNotTri = 0;
	}

	float dot = 0;
	float dMin[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
	float dMax[3] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
	for (DWORD i=0; i<nVert; i++) {
		dot = DotXYZ(rotVec[0], rgVert[i]);
		if (dot < dMin[0]) 
			dMin[0] = dot;
		if (dot > dMax[0])
			dMax[0] = dot;

		dot = DotXYZ(rotVec[1], rgVert[i]);
		if (dot < dMin[1]) 
			dMin[1] = dot;
		if (dot > dMax[1])
			dMax[1] = dot;

		dot = DotXYZ(rotVec[2], rgVert[i]);
		if (dot < dMin[2]) 
			dMin[2] = dot;
		if (dot > dMax[2])
			dMax[2] = dot;
	}

	oBB.m_RotVecW[0] = rotVec[0];
	oBB.m_RotVecW[1] = rotVec[1];
	oBB.m_RotVecW[2] = rotVec[2];

	oBB.m_HalfWidthsW.x = .5f*(dMax[0] - dMin[0]);
	oBB.m_HalfWidthsW.y = .5f*(dMax[1] - dMin[1]);
	oBB.m_HalfWidthsW.z = .5f*(dMax[2] - dMin[2]);

	oBB.m_CenterW = 
		.5f*(dMax[0] + dMin[0])*rotVec[0] + 
		.5f*(dMax[1] + dMin[1])*rotVec[1] + 
		.5f*(dMax[2] + dMin[2])*rotVec[2];

	memcpy(oBB.m_RotVecL,  oBB.m_RotVecW, sizeof(Vector3F)*3);
	oBB.m_HalfWidthsL = oBB.m_HalfWidthsW;
	oBB.m_CenterL = oBB.m_CenterW;
}

void BVTree::PartitionVert(Vector3F * rgVert, DWORD * rgFaceIndex, DWORD nVert, Vector3F & axis, FLOAT fMean, DWORD & splitIndex)
{
	DWORD nTri = nVert/3;
	splitIndex=0;

	if (nTri == 2) {
		STriangleF tri(rgVert[0], rgVert[1], rgVert[2]);
		Vector3F ctd(tri.Centroid());

		if (DotXYZ(ctd, axis) < fMean) { 
			rgVert[0] = rgVert[3];
			rgVert[1] = rgVert[4];
			rgVert[2] = rgVert[5];

			rgVert[3] = tri.vPosW[0];
			rgVert[4] = tri.vPosW[1];
			rgVert[5] = tri.vPosW[2];

			DWORD d = rgFaceIndex[0];
			rgFaceIndex[0] = rgFaceIndex[1];
			rgFaceIndex[1] = d;
		}
		splitIndex = 3*(1);
		return;
	}

	for (DWORD i=0, m1=0, m2=0; i<nTri; i++, m1=i*3) {
		STriangleF tri(rgVert[m1], rgVert[m1+1], rgVert[m1+2]);
		Vector3F ctd(tri.Centroid());

		if (DotXYZ(ctd, axis) > fMean) { 
			m2 = splitIndex*3;

			rgVert[m1  ] = rgVert[m2  ];
			rgVert[m1+1] = rgVert[m2+1];
			rgVert[m1+2] = rgVert[m2+2];

			rgVert[m2  ] = tri.vPosW[0];
			rgVert[m2+1] = tri.vPosW[1];
			rgVert[m2+2] = tri.vPosW[2];

			DWORD a = m1/3, b = m2/3;
			DWORD d = rgFaceIndex[a];
			rgFaceIndex[a] = rgFaceIndex[b];
			rgFaceIndex[b] = d;

			splitIndex++;
		}
	}
	
	static int count=0;
	static int maxi = 0;
	static int numi = 0;
	if (splitIndex == 0 || splitIndex == nTri) {
		OutputDebugStringA("\nWarning: BVTree::PartitionVert could not find an optimal splitting axias\n");
		splitIndex = nTri/2;
	}
	splitIndex *= 3;
}

bool BVTree::FindTriTriCollision(STriangleF & t1, STriangleF & t2, STriTriIntersectData & triData)
{
	//The triangles are transformed to world space to test for intersection
	///TODO: transform them relative to each other for performance and accuracy reasons.
	m_nTriTriTests++;
	t1.Transform(m_TriTransform);
	t2.Transform(m_pCollideBVTree->m_TriTransform);

	return TriTriIntersect(t1, t2, triData);
}

bool BVTree::FindBVCollisions(BranchNode * pNode, 
	BranchNode * pCollideNode, Matrix4x4F & rot, Vector3F & trans, FLOAT fScl)
{
	m_nBVBVTests++;

	if (pNode->pOBB->OBBoxIntersectW(*pCollideNode->pOBB, rot, trans, fScl)) {
		if (!IsLeaf(pNode) || !IsLeaf(pCollideNode)) {
			if (IsLeaf(pCollideNode) || (!IsLeaf(pNode) && pNode->pOBB->m_HalfWidthsW.x >= pCollideNode->pOBB->m_HalfWidthsW.x)) {
				Matrix4x4F rotR;
				pNode->pChildA->pOBB->RotationWT(rotR);

				Vector3F transR(rotR*(trans - pNode->pChildA->pOBB->m_CenterW));
				rotR *= rot;

				if (FindBVCollisions(pNode->pChildA, pCollideNode, rotR, transR, fScl)) return 1;

				pNode->pChildB->pOBB->RotationWT(rotR);

				transR = Vector3F(rotR*(trans - pNode->pChildB->pOBB->m_CenterW));
				rotR *= rot;

				if (FindBVCollisions(pNode->pChildB, pCollideNode, rotR, transR, fScl)) return 1; 
			}
			else {
				Matrix4x4F rotR;
				pCollideNode->pChildA->pOBB->RotationW(rotR);

				rotR = rot*rotR;
				Vector3F transR(fScl*(rot*pCollideNode->pChildA->pOBB->m_CenterW) + trans);

				if (FindBVCollisions(pNode, pCollideNode->pChildA, rotR, transR, fScl)) return 1;

				pCollideNode->pChildB->pOBB->RotationW(rotR);

				rotR = rot*rotR;
				transR = fScl*(rot*pCollideNode->pChildB->pOBB->m_CenterW) + trans;

				if (FindBVCollisions(pNode, pCollideNode->pChildB, rotR, transR, fScl)) return 1;
			}
		}
		else {
			STriTriIntersectData data;
			STriangleF t1(*((LeafNode*)pNode)->pTri);
			STriangleF t2(*((LeafNode*)pCollideNode)->pTri);
			if (FindTriTriCollision(t1, t2, data)) {
				SContactPoint s;
				//Warning: local space triangles
				s.pTri[0] = ((LeafNode*)pNode)->pTri;
				s.pTri[1] = ((LeafNode*)pCollideNode)->pTri;
				s.iPos = data.iPos[0];
				s.iNor = Vector3F(0, 1.f, 0);//data.iNor.Normalize();
				m_rgCP->PushBack(s);
			}	

			return 0;
		}
	}

	return 0;
}


bool BVTree::FindBVCollisionsConvex(BranchNode * pNode, 
	BranchNode * pCollideNode, Matrix4x4F & rot, Vector3F & trans, FLOAT fScl)
{
	m_nBVBVTests++;

	if (pNode->pOBB->OBBoxIntersectW(*pCollideNode->pOBB, rot, trans, fScl)) {
		if (!IsLeaf(pNode) || !IsLeaf(pCollideNode)) {
			if (IsLeaf(pCollideNode) || (!IsLeaf(pNode) && pNode->pOBB->m_HalfWidthsW.x >= pCollideNode->pOBB->m_HalfWidthsW.x)) {
				Matrix4x4F rotR;
				pNode->pChildA->pOBB->RotationWT(rotR);

				Vector3F transR(rotR*(trans - pNode->pChildA->pOBB->m_CenterW));
				rotR *= rot;

				if (FindBVCollisionsConvex(pNode->pChildA, pCollideNode, rotR, transR, fScl)) return 1;

				pNode->pChildB->pOBB->RotationWT(rotR);

				transR = Vector3F(rotR*(trans - pNode->pChildB->pOBB->m_CenterW));
				rotR *= rot;

				if (FindBVCollisionsConvex(pNode->pChildB, pCollideNode, rotR, transR, fScl)) return 1; 
			}
			else {
				Matrix4x4F rotR;
				pCollideNode->pChildA->pOBB->RotationW(rotR);

				rotR = rot*rotR;
				Vector3F transR(fScl*(rot*pCollideNode->pChildA->pOBB->m_CenterW) + trans);

				if (FindBVCollisionsConvex(pNode, pCollideNode->pChildA, rotR, transR, fScl)) return 1;

				pCollideNode->pChildB->pOBB->RotationW(rotR);

				rotR = rot*rotR;
				transR = fScl*(rot*pCollideNode->pChildB->pOBB->m_CenterW) + trans;

				if (FindBVCollisionsConvex(pNode, pCollideNode->pChildB, rotR, transR, fScl)) return 1;
			}
		}
		else {
			STriTriIntersectDataEx data;
			STriangleF t1(*((LeafNode*)pNode)->pTri);
			STriangleF t2(*((LeafNode*)pCollideNode)->pTri);
			if (FindTriTriCollision(t1, t2, data)) {
				t1.dwTriangle = data.dwFaceIndex[0] = ((LeafNode*)pNode)->pTri->dwTriangle;
				t2.dwTriangle = data.dwFaceIndex[1] = ((LeafNode*)pCollideNode)->pTri->dwTriangle;
				data.rgTri[0] = t1;
				data.rgTri[1] = t2;

				//FindContactPoints(data);

				return 1;
			}

			return 0;
		}
	}

	return 0;
}
//
//bool BVTree::FindContactPoints(STriTriIntersectDataEx & triData)
//{			
//	DWORD dwNextBodyIndex = 0;
//
//	/* The adjancey information for both bodies */
//	DWORD * rgpAdj[2] = {m_rgAdj, m_pCollideBVTree->m_rgAdj};
//	
//	/* The body to which the last triangle was found adjacent to  */
//	DWORD dwLastBodyIndex = 0;
//
//	/* The vector formed form the intersection of the last two triangle planes */
//	Vector3F lastILine(triData.iLine.Normalize());
//	const float eps = .0001f;
//
//	/* The last, and current face indices for bodies 1 and 2, respectively */
//	DWORD rgIniFaceIndex[2] = {triData.dwFaceIndex[0], triData.dwFaceIndex[1]};
//	DWORD rgCurFaceIndex[2] = {triData.dwFaceIndex[0], triData.dwFaceIndex[1]};
//	DWORD rgNextFaceIndex[2] = {triData.dwFaceIndex[0], triData.dwFaceIndex[1]};
//
//	/* The (squared) normals of the triangle planes */
//	Vector3F rgLastTriNormals[2] = {triData.normals[0], triData.normals[1] };
//	DWORD rgLastTriNormalsFlags[2] = {0, 0};
//
//	/* The number of faces in the contact region for the bodies respectively */
//	DWORD nFacesInCTR[2] = {0, 0};
//
//	/* The number of possible canidates for being coplanar*/
//	DWORD nCoplanarFacesInCTR[2] = {0, 0};
//
//	/* Average normal for transforming the polygon surface */
//	Vector3F avgNor[2] = {Vector3F(0, 0, 0), Vector3F(0, 0, 0) };
//
//	/* Contact Points Computation */
//	STriTriIntersectDataEx data;
//	memcpy(&data, &triData, sizeof(STriTriIntersectDataEx));
//
//	unsigned char rgFaceFlag[2] = {0, 0};
//
//	/* Max number of iterations that this loop can go for */
//	DWORD dwMaxIter = 3*m_nTri;
//	DWORD dwIter = 0;
//
//	dwCounter++;
//	if (dwCounter == 7517)
//		int aasdf=0;
//
//	int count=0;
//	bool bNextBodySwap = 0;
//	DWORD dwEdgeIndex = 1;
//	if (data.dwType >> 24)
//		dwNextBodyIndex = 1;
//	do {
//		//fix this later
//		/* There is a collision, but they do not intersect deeply enough for there to be correct collision data. */  
//		if (dwIter > dwMaxIter) {
//			m_ContactRegion.Reset();
//			return 0;
//		}
//		dwIter++;
//
//		/* The body to which the next triangle will be found  */
//		if ((data.dwType & 0x3) == 2)
//			dwNextBodyIndex = !dwNextBodyIndex;
//		else if ((data.dwType & 0x3) == 1)
//			dwNextBodyIndex = !1;
//		else if ((data.dwType & 0x3) == 0)
//			dwNextBodyIndex = !0;
//
//		Vector3F dif(data.iPos[1] - data.iPos[0]);
//		//if (DotXYZ(dif, lastILine) < 0) {dwEdgeIndex = 1; }
//
//		DWORD e = (data.dwType >> (8 + 8*dwEdgeIndex)) & 0xf;
//		rgNextFaceIndex[dwNextBodyIndex] = rgpAdj[dwNextBodyIndex][3*rgCurFaceIndex[dwNextBodyIndex] + e];
//		rgNextFaceIndex[!dwNextBodyIndex] = rgCurFaceIndex[!dwNextBodyIndex];
//
//		Vector3F preIPos(data.iPos[dwEdgeIndex]);
//		STriTriIntersectDataEx nextData;
//		nextData.rgTri[0] = m_rgTri[rgNextFaceIndex[0]];
//		nextData.rgTri[1] = m_pCollideBVTree->m_rgTri[rgNextFaceIndex[1]]; 
//		if (!FindTriTriCollision(nextData.rgTri[0], nextData.rgTri[1], nextData)) {
//			m_ContactRegion.Reset();
//			return 0;
//		}
//
//		Vector3F iLine(nextData.iLine.Normalize());
//		double d = DotXYZ(iLine, lastILine);
//		if (d < (1.0-eps) || d > (1.0+eps)) {
//			/* Compute the face flags */
//			d = DotXYZ(rgLastTriNormals[0], nextData.normals[0]);
//			if (d < (1.0-eps) || d > (1.0+eps)) {
//				rgLastTriNormalsFlags[0] = !rgLastTriNormalsFlags[0];
//				nFacesInCTR[0]++;
//				avgNor[0] += rgLastTriNormals[0];
//			}
//
//			d = DotXYZ(rgLastTriNormals[1], nextData.normals[1]);
//			if (d < (1.0-eps) || d > (1.0+eps)) {
//				rgLastTriNormalsFlags[1] = !rgLastTriNormalsFlags[1];
//				nFacesInCTR[1]++;
//				avgNor[1] += rgLastTriNormals[1];
//			} 
//
//			lastILine = iLine;
//		}
//		
//		CContactRegions::SPolygon3FEx * pPolygon = &m_rgPolygon[rgNextFaceIndex[0]];
//		pPolygon->rgEdge[0].edge(nextData.rgTri[0].vPosW[0], nextData.rgTri[0].vPosW[1]);
//		pPolygon->rgEdge[1].edge(nextData.rgTri[0].vPosW[1], nextData.rgTri[0].vPosW[2]);
//		pPolygon->rgEdge[2].edge(nextData.rgTri[0].vPosW[2], nextData.rgTri[0].vPosW[0]);
//		pPolygon->normal = nextData.rgTri[0].Normal();
//		rgLastTriNormals[0] = nextData.normals[0];
//		rgFaceFlag[0] = (unsigned char)rgLastTriNormalsFlags[0];
//		rgFaceFlag[1] = (unsigned char)(dwNextBodyIndex == 0);
//		m_ContactRegion.PushVertex(preIPos, pPolygon, rgFaceFlag, 0);
//
//		pPolygon = &m_pCollideBVTree->m_rgPolygon[rgCurFaceIndex[1]];
//		pPolygon->rgEdge[0].edge(data.rgTri[1].vPosW[0], data.rgTri[1].vPosW[1]);
//		pPolygon->rgEdge[1].edge(data.rgTri[1].vPosW[1], data.rgTri[1].vPosW[2]);
//		pPolygon->rgEdge[2].edge(data.rgTri[1].vPosW[2], data.rgTri[1].vPosW[0]);
//		pPolygon->normal = data.rgTri[1].Normal();
//		
//		rgLastTriNormals[1] = nextData.normals[1];
//		rgFaceFlag[0] = (unsigned char)rgLastTriNormalsFlags[1];
//		rgFaceFlag[1] = (unsigned char)(dwNextBodyIndex != 0);
//		
//		m_ContactRegion.PushVertexForward(preIPos, pPolygon, rgFaceFlag, 1);
//
//		data = nextData;
//		rgCurFaceIndex[dwNextBodyIndex] = rgNextFaceIndex[dwNextBodyIndex];
//
//	} while (rgCurFaceIndex[0] != rgIniFaceIndex[0] || rgCurFaceIndex[1] != rgIniFaceIndex[1]);
//
//	//bool result = m_ContactRegion.FormContactRegion(nFacesInCTR, m_rgCP, &m_rgPolygon, &m_TriTransform);
//	bool result = m_ContactRegion.FormContactRegion(0, nFacesInCTR[0], &m_rgPolygon, m_TriTransform);
//	m_ContactRegion.FormContactPoints(0, m_rgCP);
//	
//	//bool result = m_ContactRegion.FormContactRegion(1, nFacesInCTR[1], &m_pCollideBVTree->m_rgPolygon, m_pCollideBVTree->m_TriTransform);
//	//m_ContactRegion.FormContactPoints(1, m_rgCP);
//
//	//bool result = m_ContactRegion.FormContactRegion(0, nFacesInCTR[0], &m_rgPolygon, m_TriTransform);
//	//result |= m_ContactRegion.FormContactRegion(1, nFacesInCTR[1], &m_pCollideBVTree->m_rgPolygon, m_pCollideBVTree->m_TriTransform);
//
//	//m_ContactRegion.FormContactPointsSD(m_rgCP);
//	//http://news.ycombinator.com/item?id=4401276
//	//http://programmingpraxis.com/
//
//	return result;
//}

DWORD BVTree::FindBVCollisions(BranchNode * pNode, SRay & ray, SRay & worldRay, SRayIntersectData & data)
{
	//m_nBVBVTests++;
	DWORD r = 0;

	if (pNode->pOBB->OBBoxIntersectRelativeW(ray)) {
		if (!IsLeaf(pNode)) {
			SRay rayR;

			Matrix4x4F rotR; 
			pNode->pChildA->pOBB->RotationWT(rotR);
			rayR.p = rotR*(ray.p - pNode->pChildA->pOBB->m_CenterW);
			rayR.d = rotR*ray.d;

			r += FindBVCollisions(pNode->pChildA, rayR, worldRay, data);
			//if (FindBVCollisions(pNode->pChildA, rayR)) return 1; 

			pNode->pChildB->pOBB->RotationWT(rotR);

			rayR.p = rotR*(ray.p - pNode->pChildB->pOBB->m_CenterW);
			rayR.d = rotR*ray.d;

			r += FindBVCollisions(pNode->pChildB, rayR, worldRay, data);
			//if (FindBVCollisions(pNode->pChildB, rayR)) return 1; 
		}
		else {
			//m_nTriTriTests++;

			STriangleF & t1 = m_rgTransformedTri[((LeafNode*)pNode)->pTri->dwTriangle];
			//STriangleF t1(*((LeafNode*)pNode)->pTri);
			//t1.Transform(m_TriTransform);
		
			SRayIntersectData tmp;
			if (TriRayIntersect(t1, worldRay, tmp)) {
				if (tmp.t < data.t) {
					data = tmp;
					data.dwTri = ((LeafNode*)pNode)->pTri->dwTriangle;
				}
				return 1;
			}
			STriangleF t2(t1.vPosW[0], t1.vPosW[2], t1.vPosW[1]);
			if (TriRayIntersect(t2, worldRay, tmp)) {
				if (tmp.t < data.t) {
					tmp.nor = -tmp.nor;
					Swap(tmp.v, tmp.w);
					data = tmp;
					data.dwTri = ((LeafNode*)pNode)->pTri->dwTriangle;
				}
				return 1;
			}
		}
	}
	return r; 
}

bool BVTree::IntersectRay(SRay & ray, SRayIntersectData & data)
{
	//m_nBVBVTests = m_nTriTriTests = 0;

	Matrix4x4F rot;
	m_rgBV[0].RotationW(rot);
	rot = m_Rot*rot;
	Vector3F pos((m_Rot*m_rgBV[0].m_CenterW) + m_Trans);

	Matrix4x4F rotR(rot.Transpose());
	SRay rayR;
	rayR.p = ((rotR*(ray.p - pos))/m_fScale);
	rayR.d = rotR*ray.d;

	data.t = FLT_MAX;
	if (FindBVCollisions(m_pTree, rayR, ray, data)) {
		return 1;
	}

	return 0;
}

void BVTree::DrawBVTreeLevel(BranchNode * pNode, Matrix4x4F & rot, Vector3F & trans, DWORD dwLevel)
{
	if (m_dwDrawLevel == dwLevel || IsLeaf(pNode)) {
		m_pInstance->DrawBox(trans, pNode->pOBB->GetHalfWidthsW(), rot);
		return;
	}

	Matrix4x4F rotR;
	pNode->pChildA->pOBB->RotationW(rotR);
	Matrix4x4F newRot(rot*rotR);

	Vector3F transR(pNode->pChildA->pOBB->m_CenterW);
	Vector3F newTrans(trans + rot*transR);

	Matrix4x4F id;
	id.MIdenity();
	DrawBVTreeLevel(pNode->pChildA, newRot, newTrans, dwLevel+1);

	pNode->pChildB->pOBB->RotationW(rotR);
	newRot = rot*rotR;

	transR = pNode->pChildB->pOBB->m_CenterW;
	newTrans = trans + rot*transR;

	DrawBVTreeLevel(pNode->pChildB, newRot, newTrans, dwLevel+1);
}

void BVTree::DrawBVTree(CCollisionGraphics * pInstance, DWORD dwLevel)
{
	m_nBVBVTests = m_nTriTriTests = 0;

	Matrix4x4F rot;
	m_rgBV[0].RotationW(rot);
	rot = m_Rot*rot;
	Vector3F trans((m_Rot*m_rgBV[0].m_CenterW) + m_Trans);

	m_pInstance = pInstance;
	m_dwDrawLevel = dwLevel;
	DrawBVTreeLevel(m_pTree, rot, trans, 0);
}

void BVTree::DestroyBVHierarchy(BranchNode * pNode)
{
	if (!IsLeaf(pNode)) {
		DestroyBVHierarchy(pNode->pChildA);
		DestroyBVHierarchy(pNode->pChildB);

		delete pNode->pChildA;
		delete pNode->pChildB;
		pNode->pChildA = NULL;
		pNode->pChildB = NULL;
	}
}

void BVTree::DestroyBVTree()
{
	if (m_pTree) 
	
	{
		DestroyBVHierarchy(m_pTree);
		delete m_pTree;
		m_pTree = NULL;
	}
	m_nBranchNodes = 0;
	m_nLeafNodes = 0;

	delete[] m_rgAdj;
	m_rgAdj = NULL;

	delete[] m_rgBV;
	m_nBV = NULL;
	delete[] m_rgTri;
	m_nTri = NULL;
	delete[] m_rgTransformedTri;
	m_rgTransformedTri = NULL;

	m_Trans.x = 0;
	m_Trans.y = 0;
	m_Trans.z = 0;
	m_fScale = 0;

	m_pCollideBVTree = NULL;
	m_rgCP = NULL;
	m_nBVBVTests = 0;
	m_nTriTriTests = 0;
}


};


//currentPoly.ClipFirstEdge(l);
		//currentPoly.ConvexClipAgainstLine(l, 1);
		//previousVertex = rgVertices[i].v;

	/*Wrap the initial vertex
	m_rgContactRegionVertex.push_back(m_rgContactRegionVertex[0]);
	rgVertices = m_rgContactRegionVertex.data();
	nVertices++;

	//Compute the newell normal
	Vector3F nor(0,0,0);
	for (DWORD i=1; i<nVertices; i++) {
		nor.x += (rgVertices[i].v.y - rgVertices[i-1].v.y)*(rgVertices[i].v.z + rgVertices[i-1].v.z);
		nor.y += (rgVertices[i].v.z - rgVertices[i-1].v.z)*(rgVertices[i].v.x + rgVertices[i-1].v.x);
		nor.z += (rgVertices[i].v.x - rgVertices[i-1].v.x)*(rgVertices[i].v.y + rgVertices[i-1].v.y);
	}

	m_rgContactRegionVertex.resize(0);
	nor = nor.Normalize();
	if (nor.LengthSq() < 0.5f) {
		m_rgCP->resize(0);
		return 0;
	}

	// Find the first cp where there is a change in the iLine vector 
	/*DWORD dwFirstCPIndex = 0;
	DWORD dwIniTriNormalFlags = rgVertices[dwFirstCPIndex].dwFaceFlags[dwBodyIndex];
	while (dwIniTriNormalFlags == rgVertices[dwFirstCPIndex].dwFaceFlags[dwBodyIndex]) {dwFirstCPIndex++; }

	Vector3F nor(0,0,0);
	Vector3F previousCP(rgVertices[dwFirstCPIndex].v);
	DWORD dwFaceFlag = rgVertices[dwFirstCPIndex].dwFaceFlags[dwBodyIndex];
	for (DWORD i=dwFirstCPIndex+1; i<nVertices; i++) {
		Vector3F v1(rgVertices[i].v  - previousCP);
		Vector3F v2(rgVertices[i-1].v - previousCP);
		Vector3F v(CrossXYZ(v1, v2));
		nor += v;
		if (rgVertices[i].dwFaceFlags[dwBodyIndex] != dwFaceFlag) {
			previousCP = rgVertices[i].v;
			dwFaceFlag = rgVertices[i].dwFaceFlags[dwBodyIndex];
		}
	}
	Vector3F v1(rgVertices[0].v - previousCP);	
	Vector3F v2(rgVertices[nVertices-1].v - previousCP);
	Vector3F v(CrossXYZ(v1, v2));
	nor += v;
	for (DWORD i=1; i<=dwFirstCPIndex; i++) {
		Vector3F v1(rgVertices[i].v  - previousCP);	
		Vector3F v2(rgVertices[i-1].v - previousCP);			
		Vector3F v(CrossXYZ(v1, v2));	
		nor += v;
	}

	m_rgContactRegionVertex.resize(0);
	nor = nor.Normalize();
	if (nor.LengthSq() < 0.5f) {
		m_rgCP->resize(0);
		return 0;
	}*/

/*

					DWORD dwPolyIndex = m_rgPolygon.size();
					if (rgVertices[i].faceFlags[2+dwBodyIndex]) {
						dwPolyIndex = dwCurrentPolygon;
					} else {
						//many things, complex
						//Last edge
						triIndex=0;//need to clip the actual triangle edge
						//Check this: STriangleF * pCurTriangle = &rgVertices[dwFirstCPIndex].tri[dwBodyIndex];
						while (rgAdj[pPreTriangle->dwIndx + triIndex] != pCurTriangle->dwIndx)
							triIndex++;
						SLine3F _l(rgVertices[i-1].v, rgVertices[i].v);
						currentPoly.ClipLastEdge(_l);

						//First edge
						triIndex=0;
						//Check this: STriangleF * pCurTriangle = &rgVertices[dwFirstCPIndex].tri[dwBodyIndex];
						while (rgAdj[pCurTriangle->dwIndx + triIndex] != pPreTriangle->dwIndx)
							triIndex++;
						m_rgPolygon.push_back(SPolygon3FEx(SLine3F(pCurTriangle->vPosW[triIndex], pCurTriangle->vPosW[(triIndex+1)%2]), 0));
						SLine3F _l(rgVertices[i].v, rgVertices[i+1].v);
						m_rgPolygon.back().ClipFirstEdge(_l);

						//need moar clipping, 3d edge intersect end points
					}
					visitedTable.insert(std::pair<int, int>(rgVertices[i].tri[dwBodyIndex].dwIndx, dwPolyIndex) );
					dwCurrentPolygon = dwPolyIndex;
				}
				else {//do thngs here also//-1 index thing// add indices to poly edges
					dwCurrentPolygon = it->second;
				}
				*/











/* Compute the rotation to rotate into the frame of the average normal 
	//avgNor[dwBodyIndex] = Vector3F(1,-1.f, 0);
	Vector3F nor(avgNor[dwBodyIndex].Normalize());
	Vector3F rotVec(CrossXYZ(nor, Vector3F(0, 1.f, 0)));
	float theta  = acosf(DotXYZ(nor, Vector3F(0, 1.f, 0)));
	Matrix4x4F rot; rot.MRotationAxisLH(rotVec, -theta);

	Vector3F testNormal(rot * nor);

	SPolygon poly;
	poly.lines.reserve(nVertices);
	for (int i=1; i<nVertices; i++) {
		poly.lines.push_back(SLine(rgVertices[i-1].v, rgVertices[i].v));
	}
	poly.lines.push_back(SLine(rgVertices[nVertices-1].v, rgVertices[0].v));

	//poly.Transform(rot);
	Vector3F areaVec;
	poly.ComputeArea(areaVec);
	if (DotXYZ(areaVec, Vector3F(0, 1.f, 0)) < 0) areaVec = -areaVec;

	nor = areaVec.Normalize();
	//rot.MTranspose();
	//nor = rot*nor;
	poly.lines.clear();
	m_rgContactRegionVertex.resize(0);
	if (dwBodyIndex == 1) nor = -nor;
	*/
	//nor = -nor;

	//nor = Vector3F(0, -1.f, 0);
	
	//if (nor.y > 0) nor = -nor;*/

/*
	///Contact Nortmal Calculation///
	DWORD c = !(nPl[0] < nPl[1]), idx = 0;			//c selects the polyhedron with the fewer planes
	while (idx < nVrt && (delF[idx] == c)) idx++;	//idx is the index of the first vertex in finding a CN 

	Vector3F nor(0, 0, 0);								//the CN
	Vector3F preV(cList[idx]);							//the first vertex in finding a CN
	for (DWORD i=(idx+1), j=1; i<=nVrt; i++, j++) {
		Vector3F v1(cList[i]   - preV);				
		Vector3F v2(cList[i-1] - preV);				
		Vector3F v(CrossXYZ(v1, v2));	
		nor += v;	
		if (!(delF[i] == c)) {
			preV = cList[i], j=0;
		}
	}
	for (DWORD i=1; i<=idx; i++) {
		Vector3F v1(cList[i]   - preV);				
		Vector3F v2(cList[i-1] - preV);				
		Vector3F v(CrossXYZ(v1, v2));	
		nor += v;
	}
	*m_rgContactNor = nor.Normalize();*/

	/*if (m_rgContactNor->LengthSq() < 0.5f) {	//this is done in case the normal is zero, which 
		return 0;						//can happen due to floating point arithmetic
	}*/



/*
DOUBLE eps = .0001;			//epsilon for line equal test
	Vector3F pIL(cp.iLine);	//previous intersect line vector

	DWORD nVrt   = 0;		//number of vertices in CR
	DWORD nPl[2] = {0, 0};	//number of planes in CR for A, B
	BOOL loop    = 0;		//if infinite loop, 1 otherwise 0
	DWORD iter   = 0;		//no. of iterations of loop; iter < nCts
*/



		/*	memcpy(&cPair, &g_CPair,
				sizeof(SCollisionPair));
			nVert = 0;
			preIL = cPair.iLine;
			nPl[0] = 0, nPl[1] = 0; 							
			FLOAT sign = 1.f;									
			preTriId[0] = cPair.fIdx[0], preTriId[1] = cPair.fIdx[1];
			initTri[0] = cPair.fIdx[0], initTri[1] = cPair.fIdx[1];
			iter = 0;

			t = cPair.iType;
			q = !(t & 0x01);

			p = &adj[q][cPair.fIdx[q]*3]; 
			v = cPair.iPos[0x01 & (t >> 1)];

			tIdx[0] = cPair.fIdx[0], tIdx[1] = cPair.fIdx[1];
			tIdx[q] = p[(t >> 2)];

			STriangleF & tri1 = m_rgTri[tIdx[0]];
			t1.vPosW[0] = m_TriTransform*tri1.vPosW[0];
			t1.vPosW[1] = m_TriTransform*tri1.vPosW[1];
			t1.vPosW[2] = m_TriTransform*tri1.vPosW[2];
			t1.fIdx= tri1.fIdx;

			STriangleF & cTri1 = m_pCollideBVTree->m_rgTri[tIdx[1]];
			t2.vPosW[0] = m_pCollideBVTree->m_TriTransform*cTri1.vPosW[0];
			t2.vPosW[1] = m_pCollideBVTree->m_TriTransform*cTri1.vPosW[1];
			t2.vPosW[2] = m_pCollideBVTree->m_TriTransform*cTri1.vPosW[2];
			t2.fIdx = cTri1.fIdx;

			if (!TriTriIntersect(t1, t2, ntCP)) { 
				int a=0;
			}*/



/*m_nTriTriTests++;
			STriangleF t1, t2;
			STriangleF & tri = *((LeafNode*)pNode)->tri;
			STriangleF & cTri = *((LeafNode*)pCollideNode)->tri;

			t1.vPosW[0] = m_TriTransform*tri.vPosW[0];
			t1.vPosW[1] = m_TriTransform*tri.vPosW[1];
			t1.vPosW[2] = m_TriTransform*tri.vPosW[2];
			t1.fIdx= tri.fIdx;

			t2.vPosW[0] = m_pCollideBVTree->m_TriTransform*cTri.vPosW[0];
			t2.vPosW[1] = m_pCollideBVTree->m_TriTransform*cTri.vPosW[1];
			t2.vPosW[2] = m_pCollideBVTree->m_TriTransform*cTri.vPosW[2];
			t2.fIdx = cTri.fIdx;

			SCollisionPair & cPair = m_rgCP[m_nCPs];
			if (TriTriIntersect(t1, t2, cPair)) { 
				//cPair.pTri[0] = tri->tri;
				//cPair.pTri[1] = cTri->tri;

				//m_rgAdj[cPair.fIdx[0]][3] += 0x00030000;
				//m_pCollideBVTree->m_rgAdj[cPair.fIdx[1]][3] += 0x00030000;
				if (++m_nCPs >= m_nMaxCPairs) return 1;
			}*/




		/*double c[3][3];
		for (WORD i=0; i<3; i++) {
			for (WORD j=0; j<3; j++) {
				c[i][j] = (double)cov(i,j);
			}
		}
		double e[3][3];
		eigen_and_sort1(e, c);

		for (WORD i=0; i<3; i++) {
			for (WORD j=0; j<3; j++) {
				eigenM(i,j) = (float)e[i][j];
			}
		}*/

			/*double p1[3] = {t1.vPosW[0].x, t1.vPosW[0].y, t1.vPosW[0].z};
			double p2[3] = {t1.vPosW[1].x, t1.vPosW[1].y, t1.vPosW[1].z};
			double p3[3] = {t1.vPosW[2].x, t1.vPosW[2].y, t1.vPosW[2].z};

			double q1[3] = {t2.vPosW[0].x, t2.vPosW[0].y, t2.vPosW[0].z};
			double q2[3] = {t2.vPosW[1].x, t2.vPosW[1].y, t2.vPosW[1].z};
			double q3[3] = {t2.vPosW[2].x, t2.vPosW[2].y, t2.vPosW[2].z};

			if (tri_contact(p1, p2, p3, q1, q2, q3)) { 
				m_rgCP[m_nCPs].pTri[0] = tri->tri;
				m_rgCP[m_nCPs].pTri[1] = cTri->tri;
				if (++m_nCPs >= m_nMaxCPairs) return 1;
			}*/

			/*
				if (!vertWelded) {
		D3DXWELDEPSILONS ep;
		ZeroMemory(&ep, sizeof(ep));
		ep.Position = .001f;

		DWORD * adjIn = new DWORD[3*pMesh->GetNumFaces()];
		pMesh->GenerateAdjacency(.001f, adjIn); 

		if (D3DXWeldVertices(pMesh, D3DXWELDEPSILONS_WELDALL, &ep, adjIn, 0, 0, 0) != D3D_OK)
			DxUtSendError("Could not weld vertices of pMesh for bounding volume tree.");

		delete[] adjIn;
	}*/

				/*float p1[3] = {t1.vPosW[0].x, t1.vPosW[0].y, t1.vPosW[0].z};
			float p2[3] = {t1.vPosW[1].x, t1.vPosW[1].y, t1.vPosW[1].z};
			float p3[3] = {t1.vPosW[2].x, t1.vPosW[2].y, t1.vPosW[2].z};

			float q1[3] = {t2.vPosW[0].x, t2.vPosW[0].y, t2.vPosW[0].z};
			float q2[3] = {t2.vPosW[1].x, t2.vPosW[1].y, t2.vPosW[1].z};
			float q3[3] = {t2.vPosW[2].x, t2.vPosW[2].y, t2.vPosW[2].z};

			int c = 0;
			float iPos1[3], iPos2[3];
			if (tri_tri_intersect_with_isectline(p1, p2, p3, q1, q2, q3, &c, iPos1, iPos2)) {
				//m_rgCP[m_nCPs].iPos[0] = 
				m_rgCP[m_nCPs].pTri[0] = tri->tri;
				m_rgCP[m_nCPs].pTri[1] = cTri->tri;
				if (++m_nCPs >= m_nMaxCPairs) return 1;
			}*/

				/*cPair.iI = 0;
				if (!m_CPairHash->InTable(&cPair.iPos[0])) {
					m_CPairHash->AddElement(&cPair.iPos[0]);
					cPair.iI += 1;
				}
				if (!m_CPairHash->InTable(&cPair.iPos[1])) {
					m_CPairHash->AddElement(&cPair.iPos[1]);
					cPair.iI += cPair.iI ? 2 : 1;
				}*/

				/*if (!m_CPairHash->InTable(&cPair.iPos[0])) {
					m_CPairHash->AddElement(&cPair.iPos[0]);
					cPair.iI += 1;
				}
				if (!m_CPairHash->InTable(&cPair.iPos[1])) {
					m_CPairHash->AddElement(&cPair.iPos[1]);
					cPair.iI += 2;//cPair.iI ? 2 : 1;
				}*/
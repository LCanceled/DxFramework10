
#include "DxUtBVTree.h"
#include "DxUtMesh.h"
#include "TriTriTest.h"
#include "DxUtPolygon.h"
#include <map>

//For use in BVTree Intersect and
//find Bounding Volume Transforms
#define IsLeaf(pNode) ((!pNode->pChildB))

namespace DxUt {

BVTree::BVTree():m_pTree(0), m_nBranchNodes(0), m_nLeafNodes(0), m_BVs(0), m_Tris(0), m_TransformedTris(0), m_pAdj(0), m_Trans(0.f, 0.f, 0.f),
	m_Scale(1.f), m_pCollideBVTree(0), m_CPs(0), m_nBVBVTests(0), m_nTriTriTests(0) /*Fix*/
{
	uiCounter = 0;
	m_TriTransform.MIdenity();
	m_Rot.MIdenity();
	m_InvRot.MIdenity();
}

void AverageNormals(Vector3F & nextNor, Vector3F & preNor, Vector3F & avgNor, UINT & nNor)
{
	/* Do not average normals that share the same normal */
	if (DotXYZ(nextNor, preNor) < .99f) {
		avgNor += nextNor;
		nNor++;
	}
}

Vector3F ComputeConeNormal(UINT uiInitialTri, UINT uiInitialEdge, STriangleF * tris, UINT * pAdj, float & fTheta)
{
	UINT nNor = 0;
	Vector3F avgNor(0,0,0);
	UINT uiNextEdge = uiInitialEdge;
	UINT uiNextTri = uiInitialTri;
	Vector3F preNor(tris[uiInitialTri].Normal());
	Vector3F nextNor(tris[uiInitialTri].Normal());
	//STriangleF & preTri = tris[uiInitialTri];
	do {
		nextNor = tris[uiNextTri].Normal();
		AverageNormals(nextNor, preNor, avgNor, nNor);

		UINT uiPreTri = uiNextTri;
		uiNextTri = pAdj[3*uiNextTri + uiNextEdge];

		/* Check the three edges */
		uiNextEdge = 0;
		while (pAdj[3*uiNextTri+uiNextEdge] != uiPreTri) uiNextEdge++;
		uiNextEdge = (uiNextEdge + 1) % 3;
		
		if (uiNextTri == uiInitialTri) {
			nextNor = nextNor = tris[uiNextTri].Normal();
			break;
		}
		preNor = nextNor;
	} while (uiNextTri !=uiInitialTri);
	AverageNormals(nextNor, preNor, avgNor, nNor);
	avgNor = avgNor.Normalize();

	/* Compute the maximum divergence (fTheta) among the avgNor and the polygon normals */
	fTheta = 1.;
	uiNextTri = uiInitialTri;
	uiNextEdge = uiInitialEdge;
	do {
		/* Find the maximum of theta */
		Vector3F & n = tris[uiNextTri].Normal();
		float d = DotXYZ(n, avgNor);
		fTheta = min(fTheta, d);

		UINT uiPreTri = uiNextTri;
		uiNextTri = pAdj[3*uiNextTri + uiNextEdge];

		/* Check the three edges */
		uiNextEdge = 0;
		while (pAdj[3*uiNextTri+uiNextEdge] != uiPreTri) uiNextEdge++;
		uiNextEdge = (uiNextEdge + 1) % 3;

	} while (uiNextTri !=uiInitialTri);

	return avgNor;
}
/*
void ComputePolygons(CArray<CContactRegions::SPolygon3FEx> & rgPolygon, UINT nTri, STriangleF * tris, UINT * pAdj)
{
	rgPolygon.Reserve(nTri);
	for (UINT i=0; i<nTri; i++) {
		float fTheta0; Vector3F nor0(ComputeConeNormal(i, 0, tris, pAdj, fTheta0));
		float fTheta1; Vector3F nor1(ComputeConeNormal(i, 1, tris, pAdj, fTheta1));
		float fTheta2; Vector3F nor2(ComputeConeNormal(i, 2, tris, pAdj, fTheta2));

		rgPolygon.PushBack();
		rgPolygon.GetBack().Init(
			tris[i + 0], tris[pAdj[3*i + 0]], tris[pAdj[3*i + 1]], tris[pAdj[3*i + 2]], 
			pAdj[3*i + 0], pAdj[3*i + 1], pAdj[3*i + 2],
			fTheta0, nor0, fTheta1, nor1, fTheta2, nor2
			);
	}
}*/

void BVTree::CreateBVTree(CMesh * pMesh)
{
	Assert(!m_pTree, "BVTree::CreateBVTree tree must be destroyed before creating a new one.");
	
	UINT nTri = pMesh->GetNumTriangles();
	UINT nVert = 3*pMesh->GetNumTriangles();
	Vector3F * verts = pMesh->GetNewVertexTriangleList();
	m_pAdj = new UINT[3*nTri];
	memcpy(m_pAdj, pMesh->GetAdjancey(), 3*nTri*sizeof(UINT));
	
	m_pTree = new BranchNode;
	m_BVs = new COBBox[2*nTri];
	m_nBV = 0;
	m_Tris = new STriangleF[nTri];
	m_TransformedTris = new STriangleF[nTri];
	m_nTri = nTri;

	UINT * uiFaceIndices = new UINT[nVert/3];
	for (UINT i=0; i<nTri; i++) uiFaceIndices[i] = i;

	BuildBVTree(m_pTree, verts, uiFaceIndices, nVert, 0);

	Matrix4x4F rot;
	m_BVs[0].RotationW(rot);
	m_BVs[0].m_RotVecW[0] = rot.GetColumnVec3F(0);
	m_BVs[0].m_RotVecW[1] = rot.GetColumnVec3F(1);
	m_BVs[0].m_RotVecW[2] = rot.GetColumnVec3F(2);

	delete[] verts;
	verts = NULL;
	delete[] uiFaceIndices;
	uiFaceIndices = NULL;

	m_TopLevelOBBoxW = *m_pTree->pOBB;
}

void BVTree::CreateBVTree(STriangleF * tris, UINT nTri, UINT * adj)
{
	Assert(!m_pTree, "BVTree::CreateBVTree tree must be destroyed before creating a new one.");

	UINT nVert = 3*nTri;
	Vector3F * verts = new Vector3F[nVert];
	for (UINT i=0; i<nVert; i++) verts[i] = tris[i/3].vPosW[i%3]; 
	UINT * uiFaceIndices = new UINT[nVert/3];
	for (UINT i=0; i<nTri; i++) uiFaceIndices[i] = i;

	m_pTree = new BranchNode;
	m_BVs = new COBBox[2*nTri];
	m_nBV = 0;
	m_Tris = new STriangleF[nTri];
	m_TransformedTris = new STriangleF[nTri];
	m_nTri = nTri;

	BuildBVTree(m_pTree, verts, uiFaceIndices, nVert, 0);

	Matrix4x4F rot;
	m_BVs[0].RotationW(rot);
	m_BVs[0].m_RotVecW[0] = rot.GetColumnVec3F(0);
	m_BVs[0].m_RotVecW[1] = rot.GetColumnVec3F(1);
	m_BVs[0].m_RotVecW[2] = rot.GetColumnVec3F(2);

	delete[] verts;
	verts = NULL;
	delete[] uiFaceIndices;
	uiFaceIndices = NULL;

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
void BVTree::SetTransform(Matrix4x4F & rT, float scl)
{
	m_Rot = rT;
	m_Rot.m[0][3] = 0.f;
	m_Rot.m[1][3] = 0.f;
	m_Rot.m[2][3] = 0.f;
	m_InvRot = rT.Transpose();

	m_Trans = Vector3F(rT.GetColumnVec3F(3));
	m_Scale = scl;

	Matrix4x4F sclM; 
	sclM.MScaling(scl,scl,scl);
	m_TriTransform = sclM*m_Rot;
	m_TriTransform.m[0][3] = m_Trans.x;
	m_TriTransform.m[1][3] = m_Trans.y;
	m_TriTransform.m[2][3] = m_Trans.z;

	m_TopLevelOBBoxW.TransformOBBW(m_Trans, m_Rot, m_Scale);
}

//The transform is assumed be from the base coordinate system to the new system
void BVTree::SetTransform(Matrix4x4F & rot, Vector3F & trans, float scl)
{
	m_Rot = rot;
	m_InvRot = rot.Transpose();

	m_Trans = trans;
	m_Scale = scl;

	Matrix4x4F sclM; 
	sclM.MScaling(scl,scl,scl);
	m_TriTransform = sclM*m_Rot;
	m_TriTransform.m[0][3] = m_Trans.x;
	m_TriTransform.m[1][3] = m_Trans.y;
	m_TriTransform.m[2][3] = m_Trans.z;

	m_TopLevelOBBoxW.TransformOBBW(m_Trans, m_Rot, m_Scale);
}

void BVTree::TransformTriangles()
{
	for (UINT i=0; i<m_nTri; i++) {
		m_TransformedTris[i] = m_Tris[i];
		m_TransformedTris[i].Transform(m_TriTransform);
	}
}

//rgCPs should have some memory reserved as a guess at the total number of contact points
//Returns 0 when there are no collision; otherwise, it returns the number of collision pairs
UINT BVTree::BVCollision(BVTree & collideBVTree, CArray<SContactPoint> * CPs)
{
	m_pCollideBVTree = &collideBVTree;
	m_CPs = CPs;
	m_nBVBVTests = m_nTriTriTests = 0;

	Matrix4x4F rot;
	m_BVs[0].RotationW(rot);
	rot = m_Rot*rot;
	Vector3F pos(m_Scale*(m_Rot*m_BVs[0].m_CenterW) + m_Trans);

	Matrix4x4F cRot;
	collideBVTree.m_BVs[0].RotationW(cRot);
	cRot = collideBVTree.m_Rot*cRot;
	Vector3F cPos(collideBVTree.m_Scale*(collideBVTree.m_Rot*
		collideBVTree.m_BVs[0].m_CenterW) + collideBVTree.m_Trans);

	//Compute the pos, rot, scl relative to the this BVTree
	Matrix4x4F rotR(rot.Transpose());
	Vector3F posR((rotR*(cPos - pos))/m_Scale);
	rotR = rotR*cRot;
	//m_RelativeScale = collideBVTree.m_Scale/m_Scale;
	float fSclR = collideBVTree.m_Scale/m_Scale;

	UINT oldSize = m_CPs->GetSize();
	FindBVCollisions(m_pTree, collideBVTree.m_pTree, rotR, posR, fSclR);

	return m_CPs->GetSize() - oldSize;
}

static int stillGo = 1;

UINT BVTree::BVCollisionConvex(BVTree & collideBVTree, CArray<SContactPoint> * CPs)
{
	m_pCollideBVTree = &collideBVTree;
	m_CPs = CPs;
	m_nBVBVTests = m_nTriTriTests = 0;

	Matrix4x4F rot;
	m_BVs[0].RotationW(rot);
	rot = m_Rot*rot;
	Vector3F pos(m_Scale*(m_Rot*m_BVs[0].m_CenterW) + m_Trans);

	Matrix4x4F cRot;
	collideBVTree.m_BVs[0].RotationW(cRot);
	cRot = collideBVTree.m_Rot*cRot;
	Vector3F cPos(collideBVTree.m_Scale*(collideBVTree.m_Rot*
		collideBVTree.m_BVs[0].m_CenterW) + collideBVTree.m_Trans);

	//Compute the pos, rot, scl relative to the this BVTree
	Matrix4x4F rotR(rot.Transpose());
	Vector3F posR((rotR*(cPos - pos))/m_Scale);
	rotR = rotR*cRot;
	//m_RelativeScale = collideBVTree.m_Scale/m_Scale;
	float fSclR = collideBVTree.m_Scale/m_Scale;

	UINT oldSize = m_CPs->GetSize();
	if (FindBVCollisionsConvex(m_pTree, collideBVTree.m_pTree, rotR, posR, fSclR)) {
		int a=0;
	} 
	//if (!stillGo) return -2;

	return m_CPs->GetSize() - oldSize;
}

void BVTree::BuildBVTree(BranchNode * pNode, Vector3F * verts, UINT * uiFaceIndices, UINT nVert, UINT uiLevel)
{
	if (nVert > 3) {
		m_nBranchNodes++;

		float mean = 0;
		Vector3F axis;
		ComputeOBB(verts, nVert, m_BVs[m_nBV], mean, axis, uiLevel);

		UINT uiuiSplitIndex = 0;
		PartitionVert(verts, uiFaceIndices, nVert, axis, mean, uiuiSplitIndex);
		pNode->pOBB = &m_BVs[m_nBV++];

		pNode->pChildA = new BranchNode;
		BuildBVTree(pNode->pChildA, verts, uiFaceIndices, uiuiSplitIndex, uiLevel+1);

		pNode->pChildB = new BranchNode;
		BuildBVTree(pNode->pChildB, &verts[uiuiSplitIndex], &uiFaceIndices[uiuiSplitIndex/3], nVert-uiuiSplitIndex, uiLevel);

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
 
		float mean;
		Vector3F axis;
		ComputeOBB(verts, nVert, m_BVs[m_nBV], mean, axis, uiLevel);
		pNode->pOBB = &m_BVs[m_nBV++];
		
		STriangleF & tri = m_Tris[*uiFaceIndices];
		tri.vPosW[0] = verts[0];
		tri.vPosW[1] = verts[1];
		tri.vPosW[2] = verts[2];
		tri.uiTriangle = *uiFaceIndices;
		((LeafNode*)pNode)->pTri = &tri;
	}
}

void BVTree::ComputeOBB(Vector3F * verts, UINT nVert, COBBox & oBB, float & mean, Vector3F & axis, UINT uiLevel)
{
	Vector3F rotVec[3];
	bool bNotTri = 1;
	if (nVert != 3) {
		//Compute the center and axis vectors based upon the covariance
		Matrix4x4F cov;		//Covariance of the triangle mesh
		Vector3F ctd;		//Centroid of the triangle mesh
		CovarianceTriangles3x3F(verts, nVert, cov, ctd);

		Matrix4x4F A;		//Eigen vectors
		Vector3F lambda;	//Eigen values
		JacobiTransformation3x3F(cov, A, lambda);

		rotVec[2] = A.GetColumnVec3F(2);
		rotVec[1] = A.GetColumnVec3F(1);
		rotVec[0] = A.GetColumnVec3F(0);

		UINT uiCol;
		if (lambda.x > lambda.y) {
			if (lambda.x > lambda.z)
				uiCol = 0;
			else uiCol = 2;
		} else if (lambda.y > lambda.z) {
			uiCol = 1;
		} else uiCol = 2;
		axis = A.GetColumnVec3F(uiCol);
		mean = DotXYZ(ctd, axis);
	}
	else {
		//Compute the OBB for a triangle
		Vector3F e1, e2, e3, p1, p2, p3;
		e1 = verts[1] - verts[0];
		e2 = verts[2] - verts[0];
		e3 = verts[1] - verts[2];

		if (e1.LengthSq() > e2.LengthSq()) {
			if (e1.LengthSq() > e3.LengthSq()) {
				p1 = verts[0], p2 = verts[1], p3 = verts[2]; 
			} 
			else { 
				p1 = verts[2], p2 = verts[1], p3 = verts[0]; 
			}
		}
		else if (e2.LengthSq() > e3.LengthSq()) {
			p1 = verts[0], p2 = verts[2], p3 = verts[1]; 
		}
		else { 
			p1 = verts[2], p2 = verts[1], p3 = verts[0]; 
		}

		rotVec[0] = (p2 - p1).Normalize();

		Vector3F v1(verts[1] - verts[0]);
		Vector3F v2(verts[2] - verts[0]);
		rotVec[1] = CrossXYZ(v1, v2);
		rotVec[1] = rotVec[1].Normalize();

		rotVec[2] = CrossXYZ(rotVec[0], rotVec[1]);
		rotVec[2] = rotVec[2].Normalize();

		mean = 0.f;
		bNotTri = 0;
	}

	float dot = 0;
	float dMin[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
	float dMax[3] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
	for (UINT i=0; i<nVert; i++) {
		dot = DotXYZ(rotVec[0], verts[i]);
		if (dot < dMin[0]) 
			dMin[0] = dot;
		if (dot > dMax[0])
			dMax[0] = dot;

		dot = DotXYZ(rotVec[1], verts[i]);
		if (dot < dMin[1]) 
			dMin[1] = dot;
		if (dot > dMax[1])
			dMax[1] = dot;

		dot = DotXYZ(rotVec[2], verts[i]);
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

void BVTree::PartitionVert(Vector3F * verts, UINT * uiFaceIndices, UINT nVert, Vector3F & axis, float mean, UINT & uiSplitIndex)
{
	UINT nTri = nVert/3;
	uiSplitIndex=0;

	if (nTri == 2) {
		STriangleF tri(verts[0], verts[1], verts[2]);
		Vector3F ctd(tri.Centroid());

		if (DotXYZ(ctd, axis) < mean) { 
			verts[0] = verts[3];
			verts[1] = verts[4];
			verts[2] = verts[5];

			verts[3] = tri.vPosW[0];
			verts[4] = tri.vPosW[1];
			verts[5] = tri.vPosW[2];

			UINT d = uiFaceIndices[0];
			uiFaceIndices[0] = uiFaceIndices[1];
			uiFaceIndices[1] = d;
		}
		uiSplitIndex = 3*(1);
		return;
	}

	for (UINT i=0, m1=0, m2=0; i<nTri; i++, m1=i*3) {
		STriangleF tri(verts[m1], verts[m1+1], verts[m1+2]);
		Vector3F ctd(tri.Centroid());

		if (DotXYZ(ctd, axis) > mean) { 
			m2 = uiSplitIndex*3;

			verts[m1  ] = verts[m2  ];
			verts[m1+1] = verts[m2+1];
			verts[m1+2] = verts[m2+2];

			verts[m2  ] = tri.vPosW[0];
			verts[m2+1] = tri.vPosW[1];
			verts[m2+2] = tri.vPosW[2];

			UINT a = m1/3, b = m2/3;
			UINT d = uiFaceIndices[a];
			uiFaceIndices[a] = uiFaceIndices[b];
			uiFaceIndices[b] = d;

			uiSplitIndex++;
		}
	}
	
	static int count=0;
	static int maxi = 0;
	static int numi = 0;
	if (uiSplitIndex == 0 || uiSplitIndex == nTri) {
		OutputDebugStringA("\nWarning: BVTree::PartitionVert could not find an optimal splitting axias\n");
		uiSplitIndex = nTri/2;
	}
	uiSplitIndex *= 3;
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
	BranchNode * pCollideNode, Matrix4x4F & rot, Vector3F & trans, float scl)
{
	m_nBVBVTests++;

	if (pNode->pOBB->OBBoxIntersectW(*pCollideNode->pOBB, rot, trans, scl)) {
		if (!IsLeaf(pNode) || !IsLeaf(pCollideNode)) {
			if (IsLeaf(pCollideNode) || (!IsLeaf(pNode) && pNode->pOBB->m_HalfWidthsW.x >= pCollideNode->pOBB->m_HalfWidthsW.x)) {
				Matrix4x4F rotR;
				pNode->pChildA->pOBB->RotationWT(rotR);

				Vector3F transR(rotR*(trans - pNode->pChildA->pOBB->m_CenterW));
				rotR *= rot;

				if (FindBVCollisions(pNode->pChildA, pCollideNode, rotR, transR, scl)) return 1;

				pNode->pChildB->pOBB->RotationWT(rotR);

				transR = Vector3F(rotR*(trans - pNode->pChildB->pOBB->m_CenterW));
				rotR *= rot;

				if (FindBVCollisions(pNode->pChildB, pCollideNode, rotR, transR, scl)) return 1; 
			}
			else {
				Matrix4x4F rotR;
				pCollideNode->pChildA->pOBB->RotationW(rotR);

				rotR = rot*rotR;
				Vector3F transR(scl*(rot*pCollideNode->pChildA->pOBB->m_CenterW) + trans);

				if (FindBVCollisions(pNode, pCollideNode->pChildA, rotR, transR, scl)) return 1;

				pCollideNode->pChildB->pOBB->RotationW(rotR);

				rotR = rot*rotR;
				transR = scl*(rot*pCollideNode->pChildB->pOBB->m_CenterW) + trans;

				if (FindBVCollisions(pNode, pCollideNode->pChildB, rotR, transR, scl)) return 1;
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
				m_CPs->PushBack(s);
			}	

			return 0;
		}
	}

	return 0;
}


bool BVTree::FindBVCollisionsConvex(BranchNode * pNode, 
	BranchNode * pCollideNode, Matrix4x4F & rot, Vector3F & trans, float scl)
{
	m_nBVBVTests++;

	if (pNode->pOBB->OBBoxIntersectW(*pCollideNode->pOBB, rot, trans, scl)) {
		if (!IsLeaf(pNode) || !IsLeaf(pCollideNode)) {
			if (IsLeaf(pCollideNode) || (!IsLeaf(pNode) && pNode->pOBB->m_HalfWidthsW.x >= pCollideNode->pOBB->m_HalfWidthsW.x)) {
				Matrix4x4F rotR;
				pNode->pChildA->pOBB->RotationWT(rotR);

				Vector3F transR(rotR*(trans - pNode->pChildA->pOBB->m_CenterW));
				rotR *= rot;

				if (FindBVCollisionsConvex(pNode->pChildA, pCollideNode, rotR, transR, scl)) return 1;

				pNode->pChildB->pOBB->RotationWT(rotR);

				transR = Vector3F(rotR*(trans - pNode->pChildB->pOBB->m_CenterW));
				rotR *= rot;

				if (FindBVCollisionsConvex(pNode->pChildB, pCollideNode, rotR, transR, scl)) return 1; 
			}
			else {
				Matrix4x4F rotR;
				pCollideNode->pChildA->pOBB->RotationW(rotR);

				rotR = rot*rotR;
				Vector3F transR(scl*(rot*pCollideNode->pChildA->pOBB->m_CenterW) + trans);

				if (FindBVCollisionsConvex(pNode, pCollideNode->pChildA, rotR, transR, scl)) return 1;

				pCollideNode->pChildB->pOBB->RotationW(rotR);

				rotR = rot*rotR;
				transR = scl*(rot*pCollideNode->pChildB->pOBB->m_CenterW) + trans;

				if (FindBVCollisionsConvex(pNode, pCollideNode->pChildB, rotR, transR, scl)) return 1;
			}
		}
		else {
			STriTriIntersectDataEx data;
			STriangleF t1(*((LeafNode*)pNode)->pTri);
			STriangleF t2(*((LeafNode*)pCollideNode)->pTri);
			if (FindTriTriCollision(t1, t2, data)) {
				t1.uiTriangle = data.uiFaceIndex[0] = ((LeafNode*)pNode)->pTri->uiTriangle;
				t2.uiTriangle = data.uiFaceIndex[1] = ((LeafNode*)pCollideNode)->pTri->uiTriangle;
				data.tris[0] = t1;
				data.tris[1] = t2;

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
//	UINT uiNextBodyIndex = 0;
//
//	/* The adjancey information for both bodies */
//	UINT * rgpAdj[2] = {m_pAdj, m_pCollideBVTree->m_pAdj};
//	
//	/* The body to which the last triangle was found adjacent to  */
//	UINT uiLastBodyIndex = 0;
//
//	/* The vector formed form the intersection of the last two triangle planes */
//	Vector3F lastILine(triData.iLine.Normalize());
//	const float eps = .0001f;
//
//	/* The last, and current face indices for bodies 1 and 2, respectively */
//	UINT rgIniFaceIndex[2] = {triData.uiFaceIndex[0], triData.uiFaceIndex[1]};
//	UINT rgCurFaceIndex[2] = {triData.uiFaceIndex[0], triData.uiFaceIndex[1]};
//	UINT rgNextFaceIndex[2] = {triData.uiFaceIndex[0], triData.uiFaceIndex[1]};
//
//	/* The (squared) normals of the triangle planes */
//	Vector3F rgLastTriNormals[2] = {triData.normals[0], triData.normals[1] };
//	UINT rgLastTriNormalsFlags[2] = {0, 0};
//
//	/* The number of faces in the contact region for the bodies respectively */
//	UINT nFacesInCTR[2] = {0, 0};
//
//	/* The number of possible canidates for being coplanar*/
//	UINT nCoplanarFacesInCTR[2] = {0, 0};
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
//	UINT uiMaxIter = 3*m_nTri;
//	UINT uiIter = 0;
//
//	uiCounter++;
//	if (uiCounter == 7517)
//		int aasdf=0;
//
//	int count=0;
//	bool bNextBodySwap = 0;
//	UINT uiEdgeIndex = 1;
//	if (data.uiType >> 24)
//		dwNextBodyIndex = 1;
//	do {
//		//fix this later
//		/* There is a collision, but they do not intersect deeply enough for there to be correct collision data. */  
//		if (uiIter > dwMaxIter) {
//			m_ContactRegion.Reset();
//			return 0;
//		}
//		uiIter++;
//
//		/* The body to which the next triangle will be found  */
//		if ((data.uiType & 0x3) == 2)
//			dwNextBodyIndex = !dwNextBodyIndex;
//		else if ((data.uiType & 0x3) == 1)
//			dwNextBodyIndex = !1;
//		else if ((data.uiType & 0x3) == 0)
//			dwNextBodyIndex = !0;
//
//		Vector3F dif(data.iPos[1] - data.iPos[0]);
//		//if (DotXYZ(dif, lastILine) < 0) {dwEdgeIndex = 1; }
//
//		UINT e = (data.uiType >> (8 + 8*dwEdgeIndex)) & 0xf;
//		rgNextFaceIndex[dwNextBodyIndex] = rgpAdj[dwNextBodyIndex][3*rgCurFaceIndex[dwNextBodyIndex] + e];
//		rgNextFaceIndex[!dwNextBodyIndex] = rgCurFaceIndex[!dwNextBodyIndex];
//
//		Vector3F preIPos(data.iPos[dwEdgeIndex]);
//		STriTriIntersectDataEx nextData;
//		nextData.tris[0] = m_Tris[rgNextFaceIndex[0]];
//		nextData.tris[1] = m_pCollideBVTree->m_Tris[rgNextFaceIndex[1]]; 
//		if (!FindTriTriCollision(nextData.tris[0], nextData.tris[1], nextData)) {
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
//		pPolygon->edges[0].edge(nextData.tris[0].vPosW[0], nextData.tris[0].vPosW[1]);
//		pPolygon->edges[1].edge(nextData.tris[0].vPosW[1], nextData.tris[0].vPosW[2]);
//		pPolygon->edges[2].edge(nextData.tris[0].vPosW[2], nextData.tris[0].vPosW[0]);
//		pPolygon->normal = nextData.tris[0].Normal();
//		rgLastTriNormals[0] = nextData.normals[0];
//		rgFaceFlag[0] = (unsigned char)rgLastTriNormalsFlags[0];
//		rgFaceFlag[1] = (unsigned char)(dwNextBodyIndex == 0);
//		m_ContactRegion.PushVertex(preIPos, pPolygon, rgFaceFlag, 0);
//
//		pPolygon = &m_pCollideBVTree->m_rgPolygon[rgCurFaceIndex[1]];
//		pPolygon->edges[0].edge(data.tris[1].vPosW[0], data.tris[1].vPosW[1]);
//		pPolygon->edges[1].edge(data.tris[1].vPosW[1], data.tris[1].vPosW[2]);
//		pPolygon->edges[2].edge(data.tris[1].vPosW[2], data.tris[1].vPosW[0]);
//		pPolygon->normal = data.tris[1].Normal();
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
//	//bool result = m_ContactRegion.FormContactRegion(nFacesInCTR, m_CPs, &m_rgPolygon, &m_TriTransform);
//	bool result = m_ContactRegion.FormContactRegion(0, nFacesInCTR[0], &m_rgPolygon, m_TriTransform);
//	m_ContactRegion.FormContactPoints(0, m_CPs);
//	
//	//bool result = m_ContactRegion.FormContactRegion(1, nFacesInCTR[1], &m_pCollideBVTree->m_rgPolygon, m_pCollideBVTree->m_TriTransform);
//	//m_ContactRegion.FormContactPoints(1, m_CPs);
//
//	//bool result = m_ContactRegion.FormContactRegion(0, nFacesInCTR[0], &m_rgPolygon, m_TriTransform);
//	//result |= m_ContactRegion.FormContactRegion(1, nFacesInCTR[1], &m_pCollideBVTree->m_rgPolygon, m_pCollideBVTree->m_TriTransform);
//
//	//m_ContactRegion.FormContactPointsSD(m_CPs);
//
//	return result;
//}

UINT BVTree::FindBVCollisions(BranchNode * pNode, SRay & ray, SRay & worldRay, SRayIntersectData & data)
{
	//m_nBVBVTests++;
	UINT r = 0;

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

			STriangleF & t1 = m_TransformedTris[((LeafNode*)pNode)->pTri->uiTriangle];
			//STriangleF t1(*((LeafNode*)pNode)->pTri);
			//t1.Transform(m_TriTransform);
		
			SRayIntersectData tmp;
			if (TriRayIntersect(t1, worldRay, tmp)) {
				if (tmp.t < data.t) {
					data = tmp;
					data.uiTri = ((LeafNode*)pNode)->pTri->uiTriangle;
				}
				return 1;
			}
			STriangleF t2(t1.vPosW[0], t1.vPosW[2], t1.vPosW[1]);
			if (TriRayIntersect(t2, worldRay, tmp)) {
				if (tmp.t < data.t) {
					tmp.nor = -tmp.nor;
					Swap(tmp.v, tmp.w);
					data = tmp;
					data.uiTri = ((LeafNode*)pNode)->pTri->uiTriangle;
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
	m_BVs[0].RotationW(rot);
	rot = m_Rot*rot;
	Vector3F pos((m_Rot*m_BVs[0].m_CenterW) + m_Trans);

	Matrix4x4F rotR(rot.Transpose());
	SRay rayR;
	rayR.p = ((rotR*(ray.p - pos))/m_Scale);
	rayR.d = rotR*ray.d;

	data.t = FLT_MAX;
	if (FindBVCollisions(m_pTree, rayR, ray, data)) {
		return 1;
	}

	return 0;
}

void BVTree::DrawBVTreeLevel(BranchNode * pNode, Matrix4x4F & rot, Vector3F & trans, UINT uiLevel)
{
	if (m_uiDrawLevel == uiLevel || IsLeaf(pNode)) {
		CCollisionGraphics::DrawBox(trans, pNode->pOBB->GetHalfWidthsW(), rot);
		return;
	}

	Matrix4x4F rotR;
	pNode->pChildA->pOBB->RotationW(rotR);
	Matrix4x4F newRot(rot*rotR);

	Vector3F transR(pNode->pChildA->pOBB->m_CenterW);
	Vector3F newTrans(trans + rot*transR);

	Matrix4x4F id;
	id.MIdenity();
	DrawBVTreeLevel(pNode->pChildA, newRot, newTrans, uiLevel+1);

	pNode->pChildB->pOBB->RotationW(rotR);
	newRot = rot*rotR;

	transR = pNode->pChildB->pOBB->m_CenterW;
	newTrans = trans + rot*transR;

	DrawBVTreeLevel(pNode->pChildB, newRot, newTrans, uiLevel+1);
}

void BVTree::DrawBVTree(UINT uiLevel)
{
	m_nBVBVTests = m_nTriTriTests = 0;

	Matrix4x4F rot;
	m_BVs[0].RotationW(rot);
	rot = m_Rot*rot;
	Vector3F trans((m_Rot*m_BVs[0].m_CenterW) + m_Trans);

	m_uiDrawLevel = uiLevel;
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

	delete[] m_pAdj;
	m_pAdj = NULL;

	delete[] m_BVs;
	m_nBV = NULL;
	delete[] m_Tris;
	m_nTri = NULL;
	delete[] m_TransformedTris;
	m_TransformedTris = NULL;

	m_Trans.x = 0;
	m_Trans.y = 0;
	m_Trans.z = 0;
	m_Scale = 0;

	m_pCollideBVTree = NULL;
	m_CPs = NULL;
	m_nBVBVTests = 0;
	m_nTriTriTests = 0;
}


};

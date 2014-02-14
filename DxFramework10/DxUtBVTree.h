
#ifndef DXUTBVTREE_H
#define DXUTBVTREE_H

#include "DxUtInclude.h"
#include "DxUtOBBox.h"
#include "DxUtTriangle.h"
//#include "DxUtContactRegions.h"
#include "DxUtCollisionGraphics.h"
#include "DxUtRay.h"
#include <vector>

namespace DxUt {

class BVTree {
protected:
	friend class CRigidBody;
	friend class CRigidBodyWorld;
	friend class CRayTracer;

	struct BranchNode {
		BranchNode():pChildA(0), pChildB(0) {}
		~BranchNode() {}

		COBBox * pOBB;
		
		BranchNode * pChildA;
		BranchNode * pChildB;
	};
	struct LeafNode {
		LeafNode() {}
		COBBox * pOBB;

		STriangleF * pTri;
		/* pChildB == 0 */
		/* Do not add any other variables to this struct! */
	};
	struct STriTriIntersectDataEx : public STriTriIntersectData {
		DWORD dwFaceIndex[2];
		STriangleF rgTri[2];
	};

	BranchNode * m_pTree;
	DWORD m_nBranchNodes;
	DWORD m_nLeafNodes;

	COBBox * m_rgBV; 
	DWORD m_nBV;
	STriangleF * m_rgTri;
	STriangleF * m_rgTransformedTri;
	DWORD m_nTri;
	DWORD * m_rgAdj;
	/* DWORD m_nAdj = 3*m_nTri; */

	Matrix4x4F m_TriTransform;
	Matrix4x4F m_Rot, m_InvRot;
	Vector3F m_Trans;
	FLOAT m_fScale;

	BVTree * m_pCollideBVTree;

	/* Keep track of if this tree is a kd-Tree */
	bool m_bKdTree;

	BOOL m_bConvexCollision;
	//CContactRegions m_ContactRegion;
	CArray<SContactPoint> * m_rgCP;
	//CArray<CContactRegions::SPolygon3FEx> m_rgPolygon;
	DWORD dwCounter;

	DWORD m_nBVBVTests, m_nTriTriTests;

	//The top level OBBox transformed in world space
	COBBox m_TopLevelOBBoxW;

	CCollisionGraphics * m_pInstance;
	DWORD m_dwDrawLevel;

	void BuildBVTree(BranchNode * pNode, Vector3F * rgVert, DWORD * rgFaceIndex, DWORD nVert, DWORD dwLevel);
	void ComputeOBB(Vector3F * rgVert, DWORD nVert, COBBox & oBB, FLOAT & fMean, Vector3F & axis, DWORD dwLevel);
	void PartitionVert(Vector3F * rgVert, DWORD * rgFaceIndex, DWORD nVert, Vector3F & axis, FLOAT fMean, DWORD & splitIndex);

	virtual bool FindBVCollisions(BranchNode * pNode, BranchNode * pCollideNode, Matrix4x4F & rot, Vector3F & trans, FLOAT fScl);
	bool FindBVCollisionsConvex(BranchNode * pNode, BranchNode * pCollideNode, Matrix4x4F & rot, Vector3F & trans, FLOAT fScl);
	bool FindContactPoints(STriTriIntersectDataEx & triData);
	bool FindTriTriCollision(STriangleF & t1, STriangleF & t2, STriTriIntersectData & triData);
	DWORD FindBVCollisions(BranchNode * pNode, SRay & rayR, SRay & worldRay, SRayIntersectData & data);

	void DrawBVTreeLevel(BranchNode * pNode, Matrix4x4F & rot, Vector3F & trans, DWORD dwLevel);

	virtual void DestroyBVHierarchy(BranchNode * pNode);
public:
	BVTree();
	~BVTree() {}

	//It is assumed that adjancey information is contained in the mesh
	void CreateBVTree(ID3DX10Mesh * pMesh, DWORD dwStride, bool bMakeKdTree=0);
	void CreateBVTree(STriangleF * rgTri, DWORD nTri, DWORD * pAdj, bool bMakeKdTree=0);

	//The vertices must be in a triangle list order
	void CreateBVTree(Vector3F * rgVert, DWORD nVert, DWORD * pAdj, bool bMakeKdTree=0);

	//void LoadBVFromFile(char * file);
	//void SaveBVToFile(char * file);

	//The transform is assumed be from the standard basis coordinate system to the new system
	//rT must be a matrix of a rotation followed by a rotation 
	void SetTransform(Matrix4x4F & rT, FLOAT fScl);

	//The transform is assumed be from the standard basis coordinate system to the new system
	void SetTransform(Matrix4x4F & rot, Vector3F & trans, FLOAT fScl);
	
	/* Transform all the triangles of the mesh for faster repeated calls to BVCollision */
	void TransformTriangles();

	//rgCPs should have some memory reserved as a guess at the total number of contact points
	//Returns 0 when there are no collision; otherwise, it returns the number of collision pairs
	virtual DWORD BVCollision(BVTree & collideBVTree, CArray<SContactPoint> * rgCP);

	//If the two bodies are convex, this method may be called.
	DWORD BVCollisionConvex(BVTree & collideBVTree, CArray<SContactPoint> * rgCP);

	bool IntersectRay(SRay & ray, SRayIntersectData & data);

	void DrawBVTree(CCollisionGraphics * pInstance, DWORD dwLevel);

	DWORD & GetNumBVBVTests() {return m_nBVBVTests; }
	DWORD & GetNumTriTriTests() {return m_nTriTriTests; }

	COBBox * GetTopLevelOBBox() {return &m_TopLevelOBBoxW; }

	virtual void DestroyBVTree();
};


};

#endif
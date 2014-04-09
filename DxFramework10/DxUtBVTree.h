
#ifndef DXUTBVTREE_H
#define DXUTBVTREE_H

#include "DxUtInclude.h"
#include "DxUtOBBox.h"
#include "DxUtTriangle.h"
//#include "DxUtContactRegions.h"
#include "DxUtCollisionGraphics.h"
#include "DxUtRay.h"
#include "DxUtMesh.h"

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
		UINT uiFaceIndex[2];
		STriangleF tris[2];
	};

	BranchNode * m_pTree;
	UINT m_nBranchNodes;
	UINT m_nLeafNodes;

	COBBox * m_BVs; 
	UINT m_nBV;
	STriangleF * m_Tris;
	STriangleF * m_TransformedTris;
	UINT m_nTri;
	UINT * m_pAdj;
	/* UINT m_nAdj = 3*m_nTri; */

	Matrix4x4F m_TriTransform;
	Matrix4x4F m_Rot, m_InvRot;
	Vector3F m_Trans;
	float m_Scale;

	BVTree * m_pCollideBVTree;

	//bool m_bConvexCollision;
	//CContactRegions m_ContactRegion;
	CArray<SContactPoint> * m_CPs;
	//CArray<CContactRegions::SPolygon3FEx> m_rgPolygon;
	UINT uiCounter;

	UINT m_nBVBVTests, m_nTriTriTests;

	//The top level OBBox transformed in world space
	COBBox m_TopLevelOBBoxW;

	UINT m_uiDrawLevel;

	//The vertices must be in a triangle list order
	void CreateBVTree(Vector3F * verts, UINT nVert, UINT * pAdj);
	void BuildBVTree(BranchNode * pNode, Vector3F * verts, UINT * rgFaceIndex, UINT nVert, UINT uiLevel);
	void ComputeOBB(Vector3F * verts, UINT nVert, COBBox & oBB, float & mean, Vector3F & axis, UINT uiLevel);
	void PartitionVert(Vector3F * verts, UINT * uiFaceIncides, UINT nVert, Vector3F & axis, float mean, UINT & uiSplitIndex);

	virtual bool FindBVCollisions(BranchNode * pNode, BranchNode * pCollideNode, Matrix4x4F & rot, Vector3F & trans, float scl);
	bool FindBVCollisionsConvex(BranchNode * pNode, BranchNode * pCollideNode, Matrix4x4F & rot, Vector3F & trans, float scl);
	bool FindContactPoints(STriTriIntersectDataEx & triData);
	bool FindTriTriCollision(STriangleF & t1, STriangleF & t2, STriTriIntersectData & triData);
	UINT FindBVCollisions(BranchNode * pNode, SRay & rayR, SRay & worldRay, SRayIntersectData & data);

	void DrawBVTreeLevel(BranchNode * pNode, Matrix4x4F & rot, Vector3F & trans, UINT uiLevel);

	virtual void DestroyBVHierarchy(BranchNode * pNode);
public:
	BVTree();
	~BVTree() {}

	//It is assumed that adjancey information is contained in the mesh
	void CreateBVTree(CMesh * pMesh);
	void CreateBVTree(STriangleF * tris, UINT nTri, UINT * pAdj);

	//void LoadBVFromFile(char * file);
	//void SaveBVToFile(char * file);

	//The transform is assumed be from the standard basis coordinate system to the new system
	//rT must be a matrix of a rotation followed by a rotation 
	void SetTransform(Matrix4x4F & rT, float scl);

	//The transform is assumed be from the standard basis coordinate system to the new system
	void SetTransform(Matrix4x4F & rot, Vector3F & trans, float scl);
	
	/* Transform all the triangles of the mesh for faster repeated calls to BVCollision */
	void TransformTriangles();

	//rgCPs should have some memory reserved as a guess at the total number of contact points
	//Returns 0 when there are no collision; otherwise, it returns the number of collision pairs
	virtual UINT BVCollision(BVTree & collideBVTree, CArray<SContactPoint> * CPs);

	//If the two bodies are convex, this method may be called.
	UINT BVCollisionConvex(BVTree & collideBVTree, CArray<SContactPoint> * CPs);

	bool IntersectRay(SRay & ray, SRayIntersectData & data);

	void DrawBVTree(UINT uiLevel);

	UINT & GetNumBVBVTests() {return m_nBVBVTests; }
	UINT & GetNumTriTriTests() {return m_nTriTriTests; }

	COBBox * GetTopLevelOBBox() {return &m_TopLevelOBBoxW; }

	virtual void DestroyBVTree();
};


};

#endif
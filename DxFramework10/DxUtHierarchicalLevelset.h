
#ifndef DXUTSDFOCTREE_H
#define DXUTSDFOCTREE_H

#include "DxUtAABBox.h"
#include "DxUtOBBox.h"
#include "DxUtBVTree.h"
#include "DxUtLevelSet.h"
#include "DxUtFinitePointGrid.h"

namespace DxUt {

class COctreeLevelSet : public CLevelSet {
private:
	struct SNode {
		SNode():bLeaf(0) {ZeroMemory(rgChild, sizeof(rgChild)); }

		CAABBox box;

		bool bLeaf;

		/* Signed distances */ 
		/*
			float V000;
			float V100;
			float V010;
			float V110;
			float V001;
			float V101;
			float V011;
			float V111;
		*/
		float V[8];
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

	class CBVTreeEx : public BVTree {
	public:
		struct SLeafNodeEx {
			SLeafNodeEx():dummy0(0), dummy1(0) {}
			COBBox * pOBB;

			void * dummy0;
			void * dummy1;
			CArray<DWORD> rgVertexParticleIndex;
			CArray<DWORD> rgEdgeIndex;
		};

		DWORD m_dwMaxDepth;
		DWORD m_dwTriPerBV;
		CFinitePointGrid3F<DWORD> * m_pVertexPointGrid;
		CFinitePointGrid3F<DWORD> * m_pEdgePointGrid;
		CArray<DWORD> * m_pVertexParticleIndex;
		CArray<DWORD> * m_pEdgeIndex;
		CArray<bool> m_rgVertexParticleDuplicate;
		CArray<bool> m_rgEdgeDuplicate;

		void BuildBVTree(BranchNode * pNode, Vector3F * rgVert, DWORD * rgFaceIndex, DWORD nVert, DWORD dwDepth, BranchNode ** pPreNode);
		bool FindBVCollisions(BranchNode * pNode, BranchNode * pCollideNode, Matrix4x4F & rot, Vector3F & trans, FLOAT fScl);
	public:
		CBVTreeEx() {}
		
		void CreateBVTree(Vector3F * rgVert, DWORD nVert, DWORD * pAdj, DWORD dwTriPerBV, DWORD nVertexParticles, DWORD nEdges, DWORD nCellsX, DWORD nCellsY,
			DWORD nCellsZ, float fCellSize, Vector3F gridMinP, CFinitePointGrid3F<DWORD> & vertToIdx, CFinitePointGrid3F<DWORD> & edgeToIdx, bool bKdTree=0);
	
		DWORD BVCollision(CBVTreeEx & collideBVTree, CArray<DWORD> & rgVertexParticleIndex, CArray<DWORD> & rgEdgeIndex);

		void ResetDuplicateArrays() {
			for (DWORD i=0, end=m_rgVertexParticleDuplicate.GetSize(); i<end; i++) m_rgVertexParticleDuplicate[i] = 0;
			for (DWORD i=0, end=m_rgEdgeDuplicate.GetSize(); i<end; i++) m_rgEdgeDuplicate[i] = 0;
		}

		void DestroyBVHierarchy(BranchNode * pNode);
		void DestroyBVTree();
	};

	SNode * m_pOctree;
	DWORD m_dwMaxDepth;
	DWORD m_nOcts;
	SNode * m_pLastOct;
	float m_fMinCellSize;
	float m_fInterpolationEps;

	DWORD m_dwTriPerVolume;
	CBVTreeEx m_TriBVTree;
	CBVTreeEx m_KdTree;

	/* Collision indicies */
	CArray<DWORD> m_rgVertexParticleIndex;
	CArray<DWORD> m_rgEdgeIndex;
	//CArray<DWORD> m_rgVertexParticleDuplicate;
	//CArray<DWORD> m_rgEdgeDuplicate;

	void DoAdditionalProcessing(char * szLevelSet, Vector3F * rgVert, DWORD nVert, DWORD * rgAdj, 
		CFinitePointGrid3F<DWORD> & vertToIdx, CFinitePointGrid3F<DWORD> & edgeToIdx);
	void BuildOctree(SNode * pTree, DWORD dwDepth, Vector3F * rgVert, DWORD nVert, STriangleFEx * rgTri);
	void ComputeOctDists(SNode * pNode, Vector3F * rgVert, DWORD nVert, STriangleFEx * rgTri);
	bool IsOctRefined(SNode * pNode, Vector3F * rotVec, Vector3F & hW, Vector3F & center, DWORD dwDepth, Vector3F * rgVert, DWORD nVert, STriangleFEx * rgTri);
	float ComputeSignedDistance(SNode * pNode, Vector3F & pt);
	bool ParticleInOct(SNode * pNode, Vector3F & pt, float & fDist);
	bool ParticleInOctreeLevel(SNode * pNode, Vector3F & pt, float & fDist);
	void ComputeGradientLevel(SNode * pNode, Vector3F & pt, Vector3F & grad);
	bool HandleEdgeEdgeCollision(SEdge & edge, Matrix4x4F & T,
		CArray<SContactPoint> * rgCP, DWORD dwType, float norFlip, CLevelSet & collideLevelSet);
	bool MarchExteriorEdgeVertex(CLevelSet & collideLevelSet, float fEdgeLength, 
		Vector3F & v1, Vector3F & edgeDir, SRay & ray, float & fClosestDist, Vector3F & finalPt, float & fAdvance);
	bool MarchInteriorEdgeVertex(CLevelSet & collideLevelSet, float fEdgeLength, 
		Vector3F & v1, Vector3F & edgeDir, SRay & ray, float & fClosestDist, Vector3F & finalPt, float & fAdvance);
	
	void BisectionSearch(CLevelSet & collideLevelSet, Vector3F & v1, Vector3F & v2, Vector3F & coneDir, float coneAngle, float norFlip, CArray<SContactPoint> * rgCP);
	void HandleFace(CLevelSet & collideLevelSet, STriangleF & face, float norFlip, CArray<SContactPoint> * rgCP);
	void ProcessEdgePieces(CLevelSet & collideLevelSet, Vector3F & coneDir, float coneAngle, float norFlip, CArray<SContactPoint> * rgCP);

	bool LoadOctree(char * szFile);
	void WriteOctree(char * szFile);
	void LoadOctreeLevel(std::ifstream & stream, SNode ** pNode);
	void WriteOctreeLevel(std::ofstream & stream, SNode * pNode);

	void DrawOctreeLevel(CCollisionGraphics * pInstance, DWORD dwLevel, SNode * pOctree, DWORD dwCurLevel);
	//void ComputeMeshParticles(DWORD nVert, DWORD nTri, Vector3F * rgVert, DWORD * rgAdj, char * szLevelSetFile, float fCellSize);

	void DestroyOctree(SNode * pNode);
public:
	COctreeLevelSet();
	//~COctreeLevelSet() {}

	bool LoadLevelSet(char * szFile) {return LoadOctree(szFile); }
	void WriteLevelSet(char * szFile) {WriteOctree(szFile); }

	/* Compute the ASDF and write it to a file */
	void ComputeSDF(ID3DX10Mesh * pMesh, DWORD dwStride, char * szLevelSetFile, float fMinCellSize, float fInterpolationError, Vector3F & expansion);
	void ComputeSDF(STriangleF * rgTri, DWORD nTri, DWORD * rgAdj, char * szLevelSetFile, float fMinCellSize, float fInterpolationError, Vector3F & expansion);
	void ComputeSDF(Vector3F * rgVert, DWORD nVert, DWORD * rgAdj, char * szLevelSetFile, float fMinCellSize, float fInterpolationError, Vector3F & expansion);

	void CreateParticleRepresentation(ID3DX10Mesh * pMesh, DWORD dwStride, DWORD dwMaxDepth, DWORD dwTriPerOct, char * szLevelSetFile);
	/* For the creation of face vertices a higher tessellated mesh needs to be passed in */ 
	void CreateParticleRepresentation(STriangleF * rgTri, DWORD nTri, DWORD * rgAdj, DWORD dwMaxDepth, DWORD dwTriPerOct, char * szLevelSetFile);
	void CreateParticleRepresentation(Vector3F * rgVert, DWORD nVert, DWORD * rgAdj, DWORD dwMaxDepth, DWORD dwTriPerOct, char * szLevelSetFile);

	void SetTransform(Matrix4x4F & rot, Vector3F & trans) {
		CLevelSet::SetTransform(rot, trans);
		m_TriBVTree.SetTransform(rot, trans, 1.f);
	}

	bool ParticleInLevelSet(Vector3F & particle, float & fDist);
	bool ParticleInLevelSet_Fast(Vector3F & particle, float & fDist);

	DWORD LevelSetCollision(CLevelSet & collideLevelSet, CArray<SContactPoint> * rgCP);
	
	Vector3F ComputeGradient(Vector3F & pt);
	Vector3F ComputeGradient_Fast(Vector3F & pt);

	void DrawBVTree(CCollisionGraphics * pInstance, DWORD dwLevel) {
		m_TriBVTree.DrawBVTree(pInstance, dwLevel);
	}
	void DrawOctLevelSet(CCollisionGraphics * pInstance, DWORD dwLevel);

	void DestroyLevelSet();
};




};

#endif

#ifndef DXUTLEVELSET_H
#define DXUTLEVELSET_H

#include "DxUtInclude.h"
#include "DxUtTriangle.h"
#include "DxUtArray.h"
#include "DxUtInterpolater.h"
#include "DxUtFinitePointGrid.h"
#include "DxUtOBBox.h"
#include "DxUtAABBox.h"
#include "DxUtConvexHull.h"
#include "DxUtPolygon.h"
#include "DxUtCollisionGraphics.h"

namespace DxUt {

class CCollisionGraphics;

class CLevelSet {
protected:
	CCollisionGraphics * m_pCGraphics;
	bool m_bDrawTris;

	friend class COctreeLevelSet;
	friend class CRigidBodyWorld;
	class SFace;

	struct SFaceVertex {
		SFaceVertex() {}
		SFaceVertex(Vector3F & _v, Vector3F & _nor, float _vertDist):v(_v),nor(_nor),vertDist(_vertDist) {}

		Vector3F v;
		Vector3F nor;
		float vertDist;
	};

	struct SVertexParticle {
		Vector3F v;
		
		/* Specify the admissible normal region */
		Vector3F	coneDir;
		float		coneAngle;
		//DWORD idx;
		bool bIntersected;
		bool bAdmissible;

		Vector3F tV;
		CArray<SFaceVertex> rgInsetV;
		CArray<SFace*> rgFace;
		float closestDist;

		bool operator==(SVertexParticle & vp) {
			return v == vp.v;
		}
	};
	struct SEdgeParticle {
		SEdgeParticle() {}
		SEdgeParticle(Vector3F & _v):v(_v) {}

		Vector3F	v;
	};
	struct SEdge {
		DWORD nEdgeParticles;
		SEdgeParticle * rgEdgeParticle;

		SVertexParticle * pVP1;
		SVertexParticle * pVP2;

		float edgeLength;

		Vector3F	coneDir;
		float		coneAngle;
	};
	struct SEdgePiece {
		SEdgePiece() {}
		SEdgePiece(Vector3F & v1, Vector3F & v2, float d1, float d2) {v[0] = v1, v[1] = v2; d[0] = d1, d[1] = d2; }
		SEdgePiece(Vector3F * _v, float * _d) {v[0] = _v[0], v[1] = _v[1]; d[0] = _d[0], d[1] = _d[1]; }

		Vector3F v[2];
		float d[2];
	};
	struct SFace {
		struct SFaceVertex : public SConvexHull2F {
			SFaceVertex():pVP(0) {}
			SFaceVertex(Vector2F & _pos2F, SVertexParticle * _pVP):pVP(_pVP) {pos2F = _pos2F; }
			SFaceVertex(Vector2F & _pos2F, Vector3F & _v, SVertexParticle * _pVP):v(_v), pVP(_pVP) {pos2F = _pos2F; }

			Vector3F v;
			SVertexParticle * pVP;
		};

		SFace():v(0) {}
		Vector3F v, nor;
		COBBox oBB;
		DWORD dwFace;
		CArray<SFaceVertex> rgInteriorVert;
		CArray<SFaceVertex> rgBoundaryVert;
	};
	struct SVisitedEdge {
		SVisitedEdge() {}
		SVisitedEdge(Vector3F & _v1, Vector3F & _v2):vAvg(.5f*(_v1+_v2)) {}

		Vector3F vAvg;
	};
	struct SVisitedTriangle {
		SVisitedTriangle():bVisited(0) {}
		STriangleF tri;
		bool bVisited;
	};
	struct STriangleFEx : public STriangleF {
		STriangleFEx():bBadTriangle(0) {}
		/* Angle weighted normals */
		bool bBadTriangle;
		Vector3F nor;
		Vector3F edgeNor[3];
		Vector3F vertNor[3];
	};
	
	CArray<SVertexParticle> m_rgVertexParticle;
	CArray<SEdgeParticle> m_rgEdgeParticle;
	CArray<SEdge> m_rgEdge;
	CArray<SFace> m_rgFace;
	CArray<SFace*> m_rgToProcessFace;
	CArray<bool> m_rgIsFaceProcessed;
	CArray<SEdgePiece> m_rgEdgePiece;

	struct SLevelSetVertex {
		float dist;
	};
	SLevelSetVertex * m_rgLSVertex;

	float m_fCellSize;
	int m_nCellsX, m_nLSVertsX;
	int m_nCellsY, m_nLSVertsY;
	int m_nCellsZ, m_nLSVertsZ;
	/* Min-max corners of grid */
	Vector3F m_GridMin, m_GridMax;
	CAABBox m_GridBox;
	float m_fValidParticleDistance;
	/* Local to grid transform */
	Vector3F m_TransformObjToGrid;
	/* World to local (obj origin) transform */
	Matrix4x4F m_TransformWorldToObj;
	/* Only a rotation matrix */
	Matrix4x4F m_TransformObjToWorld;
	Matrix4x4F m_TransformNormalGridToWorld;

	/* Used for levelsets collision detection */
	COBBox m_OBBox;
	float m_fBSphereRadius;
	friend class CRigidBody;
	CAABBox m_AABBox;

	/* Maximum penetration depth */
	Vector3F m_Gravity;
	float m_fTimeStepSize;
	float m_fMinEdgeLen;
	float m_fMaxFacePenetrationDepth;
	bool m_bUseDumbScheme;

	/* Collision indicies */
	CArray<DWORD> * m_pVertexParticleIndex;
	CArray<DWORD> * m_pEdgeIndex;
	CArray<DWORD> m_rgDefaultVertexParticleIndex;
	CArray<DWORD> m_rgDefaultEdgeIndex;

	Vector3F TransformWorldToGrid(Vector3F & pt) {
		return (m_TransformWorldToObj * pt) - m_TransformObjToGrid;
	}

	//void AverageNormals(Vector3F & nextNor, Vector3F & preNor, Vector3F & avgNor, DWORD & nNor);
	void ComputeFaceVertices(DWORD dwInitialTri, SVisitedTriangle * rgTri, DWORD * rgAdj,
		CFinitePointGrid3F<DWORD> & vertToIdx, CFinitePointGrid3F<DWORD> & bVisitedEdges, CArray<SLine3F> & rgEdge);
	int ComputeAdmissibleZone(DWORD dwInitialTri, DWORD dwInitialEdge, 
		Vector3F * rgVert, DWORD * rgAdj, Vector3F & angleWeightedNor, Vector3F & edgeNor, float & fConeAngle);

	void ComputeInsetVertices(Vector3F & nor, CFinitePointGrid3F<DWORD> & vertToIdx, CArray<SLine3F> & rgEdge);

	void ComputeNondegenerateVertices(DWORD nVert, DWORD nTri, Vector3F * rgVert, DWORD * rgAdj, CFinitePointGrid3F<DWORD> & vertToIdx);
	float ComputeClosestDistance(Vector3F & pt, Vector3F * rgVert, DWORD nVert, STriangleFEx * rgTri);
	void AddEdgeVertices(Vector3F & v1, Vector3F & v2, float t, DWORD nVertPerEdge,
		CFinitePointGrid3F<DWORD> & vertToIdx, Vector3F & normal, Vector3F * rgAdjTriVert, DWORD & dwVal);
	virtual void ComputeMeshParticles(DWORD nVert, DWORD nTri, Vector3F * rgVert, DWORD * rgAdj, char * szLevelSetFile);
	virtual void DoAdditionalProcessing(char * szLevelSetFile, Vector3F * rgVert, DWORD nVert, DWORD * rgAdj, 
		CFinitePointGrid3F<DWORD> & vertToIdx, CFinitePointGrid3F<DWORD> & edgeToIdx) {}

	bool _LoadLevelSet(char * szFile);
	void ComputeSDF(Vector3F * rgVert, DWORD nVert, DWORD * rgAdj);
	float ComputeSignedDistance(Vector3F & dxyz, int cellX, int cellY, int cellZ);

	bool IsNormalAdmissible(Vector3F & coneDir, float coneAngle, Vector3F & normal, float eps=1e-1) {
		float d = -DotXYZ(normal, coneDir) + eps;
		return d > coneAngle;
	}

	Vector3F m_FaceVertexScaling;
	float ParticleInLevelSet_Cheap(Vector3F & particle, float & fDist);
	float ParticleInLevelSet_Cheap_Fast(Vector3F & particle, float & fDist);

	virtual bool HandleEdgeEdgeCollision(SEdge & edge, Matrix4x4F & T, CArray<SContactPoint> * rgCP,
		DWORD dwType, float norFlip, CLevelSet & collideLevelSet);
	bool HandleEdgeEdgeCollision_Helper(SEdge & edge, CArray<SContactPoint> * rgCP,
		float norFlip, CLevelSet & collideLevelSet);

	virtual bool MarchExteriorEdgeVertex(CLevelSet & collideLevelSet, DWORD nVertPerEdge,
		float fEdgeLength, Vector3F & v1, Vector3F & edgeDir, float & fClosestDist, Vector3F & finalPt, bool bCmp);
	virtual bool MarchInteriorEdgeVertex(CLevelSet & collideLevelSet, DWORD nVertPerEdge,
		float fEdgeLength, Vector3F & v1, Vector3F & edgeDir, float & fClosestDist, Vector3F & finalPt, bool bCmp);

	bool MarchToAdmissibleVertex(CLevelSet & collideLevelSet, DWORD nVertPerEdge,
		float fEdgeLength, Vector3F & v1, Vector3F & edgeDir, Vector3F & coneDir, float coneAngle);
	bool ComputeEdgeEdgeIntersection(CLevelSet & collideLevelSet, DWORD nVertPerEdge, bool bAdmissible1, bool bAdmisslbe2, Vector3F & v1,
		Vector3F & v2, Vector3F & edgeDir, Vector3F & coneDir, float coneAngle, float norFlip, CArray<SContactPoint> * rgCP);

	virtual void HandleFace(CLevelSet & collideLevelSet, STriangleF & face, float norFlip, CArray<SContactPoint> * rgCP);
public:
	CLevelSet();
	~CLevelSet() {}

	virtual bool LoadLevelSet(char * szFile);
	virtual void WriteLevelSet(char * szFile);

	/* The magnitude of gravity is assumed to be constant throughout the simulation as well as the step size */
	void Initialize(Vector3F & gravity, float fTimeStepSize, float fMaxVelocity);

	/* Compute the level set and write it to a file */
	virtual void ComputeSDF(ID3DX10Mesh * pMesh, DWORD dwStride, char * szLevelSetFile, float fCellSize);
	virtual void ComputeSDF(STriangleF * rgTri, DWORD nTri, DWORD * rgAdj, char * szLevelSetFile, float fCellSize);
	virtual void ComputeSDF(Vector3F * rgVert, DWORD nVert, DWORD * rgAdj, char * szLevelSetFile, float fCellSize);

	virtual void RecomputeSDFFast();

	/* Create the particle representation that represents the mesh for collision querrys */
	virtual void CreateParticleRepresentation(ID3DX10Mesh * pMesh, DWORD dwStride, char * szLevelSetFile);
	/* For the creation of face vertices a higher tessellated mesh needs to be passed in */ 
	virtual void CreateParticleRepresentation(STriangleF * rgTri, DWORD nTri, DWORD * rgAdj, char * szLevelSetFile);
	virtual void CreateParticleRepresentation(Vector3F * rgVert, DWORD nVert, DWORD * rgAdj, char * szLevelSetFile);

	/*void SetTransform(Matrix4x4F & rT) {
		m_TransformObjToWorld = rT;
		m_TransformWorldToObj = m_TransformObjToWorld.Inverse();
	}*/
	virtual void SetTransform(Matrix4x4F & rot, Vector3F & trans) {
		m_TransformObjToWorld = rot;
		m_TransformObjToWorld.m[0][3] = trans.x;
		m_TransformObjToWorld.m[1][3] = trans.y;
		m_TransformObjToWorld.m[2][3] = trans.z;
		m_TransformWorldToObj = m_TransformObjToWorld.Inverse();
		m_TransformNormalGridToWorld = rot;
		m_OBBox.TransformOBBW(trans, rot, 1.f);
		m_AABBox.TransformAABBoxW(trans, rot, 1.03f);
	}
	virtual void SetCollisionIndicies(CArray<DWORD> * pVertexParticleIndex, CArray<DWORD> * pEdgeIndex) {
		if (pVertexParticleIndex != NULL) {
			m_pVertexParticleIndex = pVertexParticleIndex;
		} else m_pVertexParticleIndex = &m_rgDefaultVertexParticleIndex;

		/*if (pEdgeIndex != NULL) {
			m_pEdgeIndex = pEdgeIndex;
		} else m_pEdgeIndex = &m_rgDefaultEdgeIndex;*/
	}

	virtual bool ParticleInLevelSet(Vector3F & particle, float & fDist);
	virtual bool ParticleInLevelSet_Fast(Vector3F & particle, float & fDist);

	virtual DWORD LevelSetCollision(CLevelSet & collideLevelSet, CArray<SContactPoint> * rgCP);

	virtual Vector3F ComputeGradient(Vector3F & pt);
	virtual Vector3F ComputeGradient_Fast(Vector3F & pt);

	virtual void DrawLevelset(CCollisionGraphics * pInstance);
	void DrawAdmissibleZones(CCollisionGraphics * pInstance);

	void AssignCG(CCollisionGraphics * cg) {m_pCGraphics = cg; }

	virtual void DestroyLevelSet();
};


};


#endif
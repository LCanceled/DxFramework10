
#ifndef DXUTLEVELSET_H
#define DXUTLEVELSET_H

#include "DxUtInclude.h"
#include "DxUtTriangle.h"
#include "DxUtArray.h"
#include "DxUtInterpolater.h"
#include "DxUtFinitePointGrid.h"
#include "DxUtConvexHull.h"
#include "DxUtOBBox.h"
#include "DxUtAABBox.h"
#include "DxUtPolygon.h"
#include "DxUtMesh.h"

namespace DxUt {

class CLevelSet {
protected:
	bool m_bDrawTris;

	//friend class COctreeLevelSet;
	friend class CRigidBodyWorld;
	friend class CRigidBody;

	struct STriangleFEx : public STriangleF {
		STriangleFEx():bBadTriangle(0) {}
		/* Angle weighted normals */
		bool bBadTriangle;
		Vector3F nor;
		Vector3F edgeNor[3];
		Vector3F vertNor[3];
	};

	struct SFaceParticle {
		/*struct SFaceVertex : public SConvexHullVector2F {
			SFaceVertex():pVP(0) {}
			SFaceVertex(Vector2F & _pos2F, SVertexParticle * _pVP):pVP(_pVP) {pos2F = _pos2F; }
			SFaceVertex(Vector2F & _pos2F, Vector3F & _v, SVertexParticle * _pVP):v(_v), pVP(_pVP) {pos2F = _pos2F; }

			Vector3F v;
			SVertexParticle * pVP;
		};

		SFaceParticle():v(0) {}
		Vector3F v, nor;
		COBBox oBB;
		UINT uiFace;
		CArray<SFaceVertex> interiorVertices;
		CArray<SFaceVertex> boundaryVertices;*/

		Vector3F boundaryVertices[3];
	};
	struct SVertexParticle {
		Vector3F v;
		
		/* Specify the admissible normal region */
		Vector3F	coneDir;
		float		coneCosAngle;
		//UINT uiIndex;
		bool bIntersected;
		bool bAdmissible;

		float closestDist;

		CArray<Vector3F> adjFaceNormals;

		bool operator==(SVertexParticle & vp) {
			return v == vp.v;
		}
	};
	struct SEdgeParticle {
		SVertexParticle * pVP1;
		SVertexParticle * pVP2;

		float edgeLength;

		Vector3F	coneDir;
		float		coneCosAngle;

		bool bIntersected;
		bool bAdmissible;

		Vector3F adjTriangleVerts[2];
		Vector3F adjFaceNormals[2];
		Vector3F adjOrthoEdgeVecs[2];
	};
	
	CArray<SVertexParticle> m_VertexParticles;
	CArray<SEdgeParticle> m_EdgeParticles;

	CArray<SFaceParticle> m_FaceParticles;
	CArray<SFaceParticle*> m_ToProcessFaceParticles;
	CArray<bool> m_IsFaceProcessed;

	struct SLevelSetVertex {
		float dist;
	};
	SLevelSetVertex * m_LSVertices;

	float m_CellSize;
	int m_nCellsX, m_nLSVertsX;
	int m_nCellsY, m_nLSVertsY;
	int m_nCellsZ, m_nLSVertsZ;
	/* Min-max corners of grid */
	Vector3F m_GridMin, m_GridMax;
	CAABBox m_GridBox;
	float m_ValidParticleDistance;
	/* Local to grid transform */
	Vector3F m_TransformObjToGrid;
	/* World to local (obj origin) transform */
	Matrix4x4F m_TransformWorldToObj;
	/* Only a rotation matrix */
	Matrix4x4F m_TransformObjToWorld;
	Matrix4x4F m_TransformNormalGridToWorld;

	/* Used for levelsets collision detection */
	COBBox m_OBBox;
	CAABBox m_AABBox;

	/* Maximum penetration depth */
	/*Vector3F m_Gravity;
	float m_TimeStepSize;
	float m_fMinEdgeLen;
	Vector3F m_FaceVertexScaling;
	float m_fMaxFacePenetrationDepth;*/

	/* Collision indicies */
	CArray<UINT> * m_pVertexParticleIndex;
	CArray<UINT> * m_pEdgeIndex;
	CArray<UINT> m_DefaultVertexParticleIndices;
	CArray<UINT> m_DefaultEdgeIndices;

	// Whether to completely sample the faces
	bool m_bUseFaceSampling;

	Vector3F TransformWorldToGrid(Vector3F & pt) {
		return (m_TransformWorldToObj * pt) - m_TransformObjToGrid;
	}

	//void AverageNormals(Vector3F & nextNor, Vector3F & preNor, Vector3F & avgNor, UINT & nNor);
	int ComputeAdmissibleZone(UINT uiInitialTri, UINT uiInitialEdge, 
		Vector3F * verts, UINT * pAdj, Vector3F & angleWeightedNor, Vector3F & edgeNor, float & coneCosAngle, CArray<Vector3F> * adjFaceNormals);
	
	void ComputeNondegenerateVertices(UINT nVert, UINT nTri, Vector3F * verts, UINT * pAdj, CFinitePointGrid3F<UINT> & vertToIndex);
	void AddEdgeVertices(Vector3F & v1, Vector3F & v2, Vector3F & v3,
		CFinitePointGrid3F<UINT> & vertToIndex, Vector3F & normal, Vector3F * adjTriVerts, UINT & dwVal);
	//void ComputeFaceVertices(UINT uiInitialTri, SVisitedTriangle * tris, UINT * pAdj,
	//	CFinitePointGrid3F<UINT> & vertToIndex, CFinitePointGrid3F<UINT> & bVisitedEdges, CArray<SSegment3F> & edges);
	//void ComputeInsetVertices(Vector3F & nor, CFinitePointGrid3F<UINT> & vertToIndex, CArray<SSegment3F> & edges);

	virtual void ComputeMeshParticles(UINT nVert, UINT nTri, Vector3F * verts, UINT * pAdj, char * szLevelSetFile);
	//virtual void PostComputeMeshParticles(char * szLevelSetFile, Vector3F * verts, UINT nVert, UINT * pAdj, 
	//	CFinitePointGrid3F<UINT> & vertToIndex, CFinitePointGrid3F<UINT> & edgeToIndex) {}

	void ComputeSDF(Vector3F * verts, UINT nVert, UINT * pAdj);
	float ComputeSignedDistance(Vector3F & pt, Vector3F * verts, UINT nVert, STriangleFEx * tris);
	float ComputeSignedDistance(Vector3F & dxyz, int cellX, int cellY, int cellZ);

	virtual bool HandleEdgeEdgeCollision(SEdgeParticle & edge, Matrix4x4F & T, CArray<SContactPoint> * CPs,
		UINT uiType, float norFlip, CLevelSet & collideLevelSet);
	virtual bool MarchExteriorEdgeVertex(Vector3F & vStart, Vector3F & edgeDir, float edgeLen, float t, 
		Vector3F & coneDir, float cosConeAngle, CLevelSet & collideLevelSet, float & closestDist, Vector3F & finalPt, bool & bIntersected);		
	bool ComputeEdgeEdgeIntersection(CLevelSet & collideLevelSet, Vector3F & v1, Vector3F & v2, 
		Vector3F & edgeDir, Vector3F & coneDir, float coneCosAngle, float norFlip, CArray<SContactPoint> * CPs);

	virtual void HandleFace(CLevelSet & collideLevelSet, STriangleF & face, float norFlip, CArray<SContactPoint> * CPs);

	bool IsNormalAdmissible(Vector3F & coneDir, float coneCosAngle, Vector3F & normal, float eps=5e-2) {
		float d = -DotXYZ(normal, coneDir) + eps;
		return d > coneCosAngle;
	}

	Vector3F ClampNormalToAdmissibleZone(Vector3F & normal, CArray<Vector3F> & faceNormals) {
		float bestNormalDist = FLT_MAX;
		UINT uiBestIndex = 0;
		for (int i=0, end=faceNormals.GetSize(); i<end; i++) {
			float d = abs(DotXYZ(normal, faceNormals[i]));
			if (d < bestNormalDist) {
				bestNormalDist = d;
				uiBestIndex = i;
			}
		}
		return faceNormals[uiBestIndex];
	}

	virtual bool LoadLevelSet(char * szFile);
	virtual void WriteLevelSet(char * szFile);
public:
	CLevelSet();
	~CLevelSet() {}

	virtual void CreateLevelSet(CMesh * pMesh, char * szLevelSetFile, bool bLoadFromFile=1, float cellSize=.1f);
	virtual void CreateLevelSet(STriangleF * tris, UINT nTri, UINT * pAdj, char * szLevelSetFile, bool bLoadFromFile=1, float cellSize=.1f);

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
	virtual void SetCollisionIndicies(CArray<UINT> * pVertexParticleIndex, CArray<UINT> * pEdgeIndex) {
		if (pVertexParticleIndex != NULL) {
			m_pVertexParticleIndex = pVertexParticleIndex;
		} else m_pVertexParticleIndex = &m_DefaultVertexParticleIndices;

		/*if (pEdgeIndex != NULL) {
			m_pEdgeIndex = pEdgeIndex;
		} else m_pEdgeIndex = &m_DefaultEdgeIndices;*/
	}

	virtual bool ParticleInLevelSet(Vector3F & particle, float & dist);

	virtual UINT LevelSetCollision(CLevelSet & collideLevelSet, CArray<SContactPoint> * CPs);

	virtual Vector3F ComputeGradient(Vector3F & pt);

	virtual void DrawLevelset();
	void DrawAdmissibleZones();

	virtual void DestroyLevelSet();
};


};


#endif


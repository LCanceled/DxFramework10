
#ifndef RIGIDBODYWORLD_H
#define RIGIDBODYWORLD_H

#include "DxUtInclude.h"
#include "DxUtRigidBody.h"
#include "DxUtCollisionGraphics.h"
#include "DxUtCamera.h"
#include "DxUtMesh.h"
#include "DxUtRay.h"
#include "DxUtInfiniteGrid.h"

namespace DxUt {

class CRigidBodyWorld {
private:
	/* A contact point between two rigid bodies */
	struct SRBObject;
	struct SRBContact {
		SRBObject * rBOk;
		SRBObject * rBOl;
		Vector3F iPos;
		Vector3F iNor;
		float dist;
		float pushOutVel;
		DWORD dwLayer;
		DWORD dwIdx;
		
		/* Cached solver info */
		Vector3F t1, t2;
		float fEffectiveMass[3];
		Vector3F JNA[3];
		Vector3F JWA[3];
		Vector3F JNB[3];
		Vector3F JWB[3];
	};
	struct SRBObject {
		WORD bUsed;
		WORD bActive;
		int iHeight;
		DWORD dwLayer;
		CRigidBody rB;
		CArray<SRBContact*> rgRBCP;
		bool bConstrainCM;
		bool bDenter;
		Vector3F constraintCMPos;
	};
	struct SContactLayer {
		CArray<SRBContact> rgRBCP;
		CArray<SRBObject*> rgRBObj;
	};

	CArray<SRBObject> m_rgRBObject;
	/* This is the pointer to the head of the RBObject array */
	SRBObject * m_pRBObject;

	CArray<SContactLayer> m_rgContactLayer;
	CArray<SRBContact> m_rgRBContactPoint;
	CArray<SContactPoint> m_rgContactPoint;

	CCollisionGraphics m_CollisionGraphics;
	CRigidBody::GeometryType m_RBType;

	//CUniformGrid<SRBObject> m_Grid;

	Vector3F m_Gravity;
	float m_fTimeStepSize;
	float m_fMaxVelocity;
	bool m_bUseHierarchicalLevelSet;
	bool m_bUsePushOut;

	friend class CRayTracer;

	DWORD FindContacts();
	void SolveConstraints(DWORD nContacts);
	void SolveLayer(DWORD k, VectorNF & lambda, float dt);
	void PGSSolve(DWORD nContacts, float dt, bool bNeedSetup);
	void PGSSetup(DWORD nContacts);
	void ComputeContactGraph();
	void ApplyCMConstraint(SRBObject & rBO, float dt);
public:
	CRigidBodyWorld();
	//~CRigidBodyWorld() {}

	/* Give a nonzero hint as to how many RBs and contacts among them there will be */
	void CreateRigidBodyWorld(DWORD nHintRBs, DWORD nHintContacts, bool bUseHierarchicalLevelSet=0,
		Vector3F * gravity=&Vector3F(0, 3.8f, 0), float fStepSize=.03f, float fMaxVelocity=10.f);

	/* Note that once a rigidbody has been added there is no way to remove it. It can at */
	/* most be disabled from taking part further in the the rigidbody world. Because */
	/* of this one should reshape the rigidbody world if simulating a much smaller */
	/* collection of bodies than the total number that were added to it. */

	/* The funciton returns the id for this rigidbody */
	/* Any future changes to a body's characteristics can be changed by way of calling */
	/* GetRigidBody on that id to obtain the CRigidBody and using its getters and setters */
	DWORD AddRigidBody(CMesh * pMesh, DWORD dwStride, float fScale, float fMass, Vector3F & pos, Matrix4x4F & rot, Vector3F & linVel, Vector3F & angVel,
		float fElasticity, float fMu, Vector3F & force, Vector3F & torque, char * szLevelSet, DWORD dwTriPerOct,
		CRigidBody::GeometryType type=CRigidBody::GT_TRIANGLE_MESH, SMaterial * pOverrideMaterial=NULL);
	void DisableRigidBody(DWORD dwId);
	void EnableRigidBody(DWORD dwId);

	/* Constrains the center of mass to lie at a certain position */
	void AddCenterOfMassPositionConstraint(DWORD dwRigidBody, Vector3F & pos);

	void UpdateRigidBodies(FLOAT dt, Vector3F & gAcel);

	void DrawRigidBodies(CCamera * pCam, SLightDir & light, DWORD dwShaderPass);

	void SetCameraForCollisionGraphics(CCamera * pCam) {m_CollisionGraphics.SetCamera(pCam); }
	void DrawCollisionGraphics(CCamera * pCam);

	void Dent(Vector3F & cpt);

	/* Returns the world matrix for the body with dwId */
	Matrix4x4F GetRBWorld(DWORD dwId);
	CRigidBody * GetRigidBody(DWORD dwId);
	DWORD GetNumBodies() {return m_rgRBObject.GetSize(); }

	void DestroyRigidBodyWorld();
};

inline void CRigidBodyWorld::DrawRigidBodies(CCamera * pCam, SLightDir & light, DWORD dwShaderPass)
{
	for (DWORD i=0, end=m_rgRBObject.GetSize(); i<end; i++) {
		m_rgRBObject[i].rB.GetMesh()->SetupDraw(pCam, light);
		m_rgRBObject[i].rB.GetMesh()->DrawAllSubsets(pCam, m_pRBObject[i].rB.GetWorldMatrix(), dwShaderPass, m_rgRBObject[i].rB.GetOverrideMaterial());
	}
}

inline Matrix4x4F CRigidBodyWorld::GetRBWorld(DWORD dwId) 
{
	Assert(dwId < m_rgRBObject.GetSize(), "CRigidBodyWorld::GetRBWorld "
		"an id was specifed which does not exist for any body.");

	return m_pRBObject[dwId].rB.GetWorldMatrix();
}
inline CRigidBody * CRigidBodyWorld::GetRigidBody(DWORD dwId) 
{
	Assert(dwId < m_rgRBObject.GetSize(), "CRigidBodyWorld::GetRigidBody "
		"an id was specifed which does not exist for any body.");

	return &m_rgRBObject[dwId].rB;
}


};


#endif



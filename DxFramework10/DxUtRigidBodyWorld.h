
#ifndef RIGIDBODYWORLD_H
#define RIGIDBODYWORLD_H

#include "DxUtInclude.h"
#include "DxUtRigidBody.h"
#include "DxUtCollisionGraphics.h"
#include "DxUtCamera.h"
#include "DxUtMesh.h"
#include "DxUtRay.h"

namespace DxUt {

class CRigidBodyWorld {
private:
	friend class CRayTracer;

	/* A contact point between two rigid bodies */
	struct SRBObject;
	struct SRBContact {
		SRBObject * rBOk;
		SRBObject * rBOl;
		Vector3F iPos;
		Vector3F iNor;
		float dist;
		float pushOutVel;
		UINT uiLayer;
		UINT uiIndex;
		
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
		UINT uiLayer;
		CRigidBody rB;
		CArray<SRBContact*> RBCPs;
		bool bConstrainCM;
		bool bDenter;
		Vector3F constraintCMPos;
	};
	struct SContactLayer {
		CArray<SRBContact> RBCPs;
		CArray<SRBObject*> RBObjs;
	};

	CArray<SRBObject> m_RBObjects;
	/* This is the pointer to the head of the RBObject array */
	SRBObject * m_pRBObject;

	CArray<SContactLayer> m_ContactLayers;
	CArray<SRBContact> m_RBContactPoints;
	CArray<SContactPoint> m_ContactPoints;

	CRigidBody::GeometryType m_RBType;

	Vector3F m_Gravity;
	float m_TimeStepSize;
	float m_MaxVelocity;
	bool m_bUseHierarchicalLevelSet;
	bool m_bUsePushOut;

	UINT FindContacts();
	void SolveConstraints(UINT nContacts);
	void SolveLayer(UINT k, VectorNF & lambda, float dt);
	void PGSSolve(UINT nContacts, float dt, bool bNeedSetup);
	void PGSSetup(UINT nContacts);
	void ComputeContactGraph();
	void ApplyCMConstraint(SRBObject & rBO, float dt);
public:
	CRigidBodyWorld();
	//~CRigidBodyWorld() {}

	/* Give a nonzero hint as to how many RBs and contacts among them there will be */
	void CreateRigidBodyWorld(UINT nHintRBs, UINT nHintContacts, bool bUseHierarchicalLevelSet=0,
		Vector3F gravity=Vector3F(0, -4.8f, 0), float stepSize=.03f, float fMaxVelocity=10.f);

	/* The funciton returns the id for this rigidbody */
	/* Any future changes to a body's characteristics can be changed by way of calling */
	/* GetRigidBody on that id to obtain the CRigidBody and using its getters and setters */
	UINT AddRigidBody(CMesh * pMesh, float scale, float mass, const Vector3F & pos, const Matrix4x4F & rot, const Vector3F & linVel, const Vector3F & angVel,
		float elasticity, float mu, const Vector3F & force, const Vector3F & torque, char * szLevelSet, UINT uiTriPerOct,
		CRigidBody::GeometryType type=CRigidBody::GT_TRIANGLE_MESH, SMaterial * pOverrideMaterial=NULL);
	void DisableRigidBody(UINT uiId);
	void EnableRigidBody(UINT uiId);

	/* Constrains the center of mass to lie at a certain position */
	void AddCenterOfMassPositionConstraint(UINT uiRigidBody, Vector3F & pos);

	void UpdateRigidBodies(float dt, const Vector3F & gAcel);

	void DrawRigidBodies(CCamera * pCam, SLightDir & light, UINT uiShaderPass);

	void SetCameraForCollisionGraphics(CCamera * pCam) {CCollisionGraphics::SetCamera(pCam); }
	void DrawCollisionGraphics(CCamera * pCam);

	void Dent(Vector3F & cpt);

	/* Returns the world matrix for the body with uiId */
	Matrix4x4F GetRBWorld(UINT uiId);
	CRigidBody * GetRigidBody(UINT uiId);
	UINT GetNumBodies() {return m_RBObjects.GetSize(); }

	void DestroyRigidBodyWorld();
};

inline void CRigidBodyWorld::DrawRigidBodies(CCamera * pCam, SLightDir & light, UINT uiShaderPass)
{
	for (UINT i=0, end=m_RBObjects.GetSize(); i<end; i++) {
		m_RBObjects[i].rB.GetMesh()->SetupDraw(pCam, light);
		m_RBObjects[i].rB.GetMesh()->DrawAllSubsets(pCam, m_pRBObject[i].rB.GetWorldMatrix(), uiShaderPass, m_RBObjects[i].rB.GetOverrideMaterial());
	}
}

inline Matrix4x4F CRigidBodyWorld::GetRBWorld(UINT uiId) 
{
	Assert(uiId < m_RBObjects.GetSize(), "CRigidBodyWorld::GetRBWorld "
		"an id was specifed which does not exist for any body.");

	return m_pRBObject[uiId].rB.GetWorldMatrix();
}
inline CRigidBody * CRigidBodyWorld::GetRigidBody(UINT uiId) 
{
	Assert(uiId < m_RBObjects.GetSize(), "CRigidBodyWorld::GetRigidBody "
		"an id was specifed which does not exist for any body.");

	return &m_RBObjects[uiId].rB;
}


};


#endif



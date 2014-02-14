
#ifndef DXUTRIGIDBODY_H
#define DXUTRIGIDBODY_H

#include "DxUtInclude.h"
#include "DxUtBVTree.h"
#include "DxUtHierarchicalLevelset.h"
#include "DxUtMesh.h"
#include "DxUtRay.h"

namespace DxUt {

struct STriangleF;

enum RigidBodyType {
	RBT_Static	 = 0xfff1,
	RBT_Scripted = 0xfff2,
	RBT_Dynamic	 = 0xfff3
};

class CRigidBody {
public:
	enum GeometryType {
		GT_TRIANGLE_MESH,
		GT_LEVEL_SET,
		GT_OBBOX
	};
protected:
	friend class CRigidBodyWorld;

	Vector3F m_CenterOfMass;
	Vector3F m_Pos,		m_PrePos;
	Matrix4x4F m_Rot,	m_PreRot;
	Vector3F m_LinVel,	m_PreLinVel;
	Vector3F m_AngVel,	m_PreAngVel;

	Matrix4x4F m_IBody,	m_InvIBody;
	Matrix4x4F m_I,		m_InvI;

	FLOAT m_InvMass,		m_Density;
	FLOAT m_fElasticity,	m_fMu;

	Vector3F m_Force;
	Vector3F m_Torque;

	CLevelSet * m_pLevelSet;
	FLOAT m_fScale;

	GeometryType m_RBType;

	/* Drawable mesh */
	CMesh * m_Mesh;
	SMaterial * m_pOverrideMaterial;

	bool m_bNotStatic;

	/* The contact points stored for this body */
	//std::vector<SRBContact> m_rgCPs;

	Vector3F ComputeLinVel(Vector3F & impulse);
	Vector3F ComputeAngVel(Vector3F & impulse, Vector3F & iPos);
	Matrix4x4F GetPart1(Vector3F & iPos, Matrix4x4F & r);
	Matrix4x4F GetPart2(Vector3F & iPos);
	Vector3F GetPart3(Vector3F & gAcel, Vector3F & iPos, float dt);
	float GetbVector(float * J, Vector3F & gAcel, Vector3F iPos, float dt);

	Vector3F GetImpulseCoefficient(Vector3F & jthCt, Vector3F & ithCt, Vector3F & iNor);
	float GetVelocityAtContactPoint(Vector3F & jthCt, Vector3F & jthNor);

	friend class CRayTracer;
	friend class CRigidBodyWorld;
public:
	CRigidBody();
	//~CRigidBody() {}

	/* Mass < 0 represents an infinitely heavy object */
	void CreateRigidBody(CMesh * pMesh, DWORD dwStride, FLOAT fScale, FLOAT fMass, Vector3F & gravity, float fTimeStepSize, float fMaxVel,
		char * szLevelSet, DWORD dwTriPerOct, bool bUseHierarchicalLevelSet, GeometryType type=GT_TRIANGLE_MESH, SMaterial * pOverrideMaterial=NULL);
	void CreateRigidBody(STriangleF * rgTri, DWORD nTri, DWORD * rgAdj, FLOAT fScale, FLOAT fMass, Vector3F & gravity, float fTimeStepSize, float fMaxVel,
		char * szLevelSet, DWORD dwTriPerOct, bool bUseHierarchicalLevelSet, GeometryType type=GT_TRIANGLE_MESH, SMaterial * pOverrideMaterial=NULL);

	void IntegratePos(FLOAT dt);
	void IntegrateVel(FLOAT dt, Vector3F & gAcel);

	/* Returns -1 on no collision; otherwise, return is the number of contact points */
	DWORD DetermineCollision(CRigidBody * pRB, CArray<SContactPoint> * rgCP);
	void TransformLevelSet() {m_pLevelSet->SetTransform(m_Rot, m_Pos); }
	DWORD DetermineCollisionLevelSet(CRigidBody * pRB, CArray<SContactPoint> * rgCP);
	DWORD DetermineCollisionConvexPolyhedron(CRigidBody * pRB, CArray<SContactPoint> * rgCP);
	/* Returns the number of contact points held in the array of them rgCPs */
	//DWORD GetContactPoints(SRBContact * rgCPs) {rgCPs = m_rgCPs.data(); return m_rgCPs.size(); }
	/* Sets this body's array of contact points to contain none */
	//void ResetCollisions() {m_rgCPs.resize(0); }
	
	float ComputeFrictionlessImpulse(CRigidBody & rB, Vector3F & iPos, Vector3F & iNor);
	float ComputeFrictionImpulse(CRigidBody & rB, Vector3F & iPos, Vector3F & t);
	void ComputeFrictionlessImpulsePart(CRigidBody & rB, Vector3F & iPos, Vector3F & iNor,
		float & fEffectiveMass, Vector3F & JNA, Vector3F & JWA, Vector3F & JNB, Vector3F & JWB);
	void ComputeFrictionImpulsePart(CRigidBody & rB, Vector3F & iPos, Vector3F & t, 
		float & fEffectiveMass, Vector3F & JNA, Vector3F & JWA, Vector3F & JNB, Vector3F & JWB);
	void ApplyImpulse(Vector3F & impulse, Vector3F & iPos);
	void ApplyFrictionImpulse(Vector3F & impulse, Vector3F & iPos);
	void SavePosAndRot() {m_PrePos = m_Pos; m_PreRot = m_Rot;}
	void RestorePosAndRot() {m_Pos = m_PrePos; m_Rot = m_PreRot;}
	void SaveLinAndAngVel() {m_PreLinVel = m_LinVel; m_PreAngVel = m_AngVel; }
	void RestoreLinAndAngVel() {m_LinVel = m_PreLinVel; m_AngVel = m_PreAngVel; }
	void SetStatic(bool bStatic) {if (m_InvMass != 0) m_bNotStatic = !bStatic; }

	float MaxRadiusSq() {return m_pLevelSet->m_OBBox.GetMaxRadiusSq(); }

	Vector3F & GetPos() {return m_Pos;}
	Matrix4x4F & GetRot() {return m_Rot; }
	Matrix4x4F GetWorldMatrix() {
		Matrix4x4F m(m_Rot);
		m.m[0][3] = m_Pos.x, m.m[1][3] = m_Pos.y, m.m[2][3] = m_Pos.z;
		m.m[0][0] *= m_fScale, m.m[1][1] *= m_fScale, m.m[2][2] *= m_fScale;
		return m;
	}
	const Matrix4x4F & GetIBody() const {return m_IBody; }
	const void SetIBody(Matrix4x4F & iBody) {m_IBody = iBody; m_InvIBody = m_IBody.Inverse(); }
	Vector3F & GetLinVel() {return m_LinVel;}
	Vector3F & GetAngVel() {return m_AngVel;}
	Vector3F & GetForce() {return m_Force;}
	Vector3F & GetTorque() {return m_Torque;}

	FLOAT & GetMass() {return m_InvMass;}
	FLOAT & GetElasticity() {return m_fElasticity;}
	FLOAT & GetFriction() {return m_fMu;}
	FLOAT & GetScale() {return m_fScale;}

	CLevelSet * GetLevelSet() {return m_pLevelSet; }

	bool IsStatic() {return m_InvMass == 0; }

	CMesh * GetMesh() {return m_Mesh; }
	SMaterial * GetOverrideMaterial() {return m_pOverrideMaterial; }

	void DestroyRigidBody();
};
	
void ComputeVolume(STriangleF * rgTri, DWORD nTri, double & dVol);
//The vertices must be in a triangle list order
void ComputeVolume(Vector3F * rgVert, DWORD nVert, double & dVol);

void ComputeInertiaTensor(STriangleF * rgTri, DWORD nTri, double fMass, Matrix4x4F & I, Vector3F & cm, double & dDensity);
void ComputeInertiaTensor(Vector3F * rgVert, DWORD nVert, double fMass, Matrix4x4F & I, Vector3F & cm, double & dDensity);


};


#endif



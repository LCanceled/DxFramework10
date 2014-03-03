
#ifndef DXUTRIGIDBODY_H
#define DXUTRIGIDBODY_H

#include "DxUtInclude.h"
#include "DxUtBVTree.h"
#include "DxUtLevelSet.h"
//#include "DxUtHierarchicalLevelset.h"
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

	float m_InvMass,		m_Density;
	float m_Elasticity,	m_Mu;

	Vector3F m_Force;
	Vector3F m_Torque;

	CLevelSet * m_pLevelSet;
	float m_Scale;

	GeometryType m_RBType;

	/* Drawable mesh */
	CMesh * m_pMesh;
	SMaterial * m_pOverrideMaterial;

	bool m_bNotStatic;

	/* The contact points stored for this body */
	//std::vector<SRBContact> m_rgCPs;

	Vector3F ComputeLinVel(Vector3F & impulse);
	Vector3F ComputeAngVel(Vector3F & impulse, Vector3F & iPos);
	float GetbVector(float * J, Vector3F & gAcel, Vector3F iPos, float dt);

	Vector3F GetImpulseCoefficient(Vector3F & jthCt, Vector3F & ithCt, Vector3F & iNor);
	float GetVelocityAtContactPoint(Vector3F & jthCt, Vector3F & jthNor);

	friend class CRayTracer;
	friend class CRigidBodyWorld;
public:
	CRigidBody();
	//~CRigidBody() {}

	/* Mass < 0 represents an infinitely heavy object */
	void CreateRigidBody(CMesh * pMesh, DWORD dwStride, float scale, float mass, Vector3F & gravity, float timeStepSize, float maxVel,
		char * szLevelSet, DWORD dwTriPerOct, bool bUseHierarchicalLevelSet, GeometryType type=GT_TRIANGLE_MESH, SMaterial * pOverrideMaterial=NULL);

	void IntegratePos(float dt);
	void IntegrateVel(float dt, Vector3F & gAcel);

	/* Returns -1 on no collision; otherwise, return is the number of contact points */
	DWORD DetermineCollision(CRigidBody * pRB, CArray<SContactPoint> * CPs);
	/* Must be called before determing a collision */
	void TransformLevelSet() {m_pLevelSet->SetTransform(m_Rot, m_Pos); }
	DWORD DetermineCollisionLevelSet(CRigidBody * pRB, CArray<SContactPoint> * CPs);
	//DWORD DetermineCollisionConvexPolyhedron(CRigidBody * pRB, CArray<SContactPoint> * CPs);
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
		m.m[0][0] *= m_Scale, m.m[1][1] *= m_Scale, m.m[2][2] *= m_Scale;
		return m;
	}
	const Matrix4x4F & GetIBody() const {return m_IBody; }
	const void SetIBody(Matrix4x4F & iBody) {m_IBody = iBody; m_InvIBody = m_IBody.Inverse(); }
	Vector3F & GetLinVel() {return m_LinVel;}
	Vector3F & GetAngVel() {return m_AngVel;}
	Vector3F & GetForce() {return m_Force;}
	Vector3F & GetTorque() {return m_Torque;}

	float & GetMass() {return m_InvMass;}
	float & GetElasticity() {return m_Elasticity;}
	float & GetFriction() {return m_Mu;}
	float & GetScale() {return m_Scale;}

	CLevelSet * GetLevelSet() {return m_pLevelSet; }

	bool IsStatic() {return m_InvMass == 0; }

	CMesh * GetMesh() {return m_pMesh; }
	SMaterial * GetOverrideMaterial() {return m_pOverrideMaterial; }

	void DestroyRigidBody();
};
	
void ComputeVolume(STriangleF * tris, DWORD nTri, double & vol);
//The vertices must be in a triangle list order
void ComputeVolume(Vector3F * verts, DWORD nVert, double & vol);

void ComputeInertiaTensor(STriangleF * tris, DWORD nTri, double mass, Matrix4x4F & I, Vector3F & cm, double & density);
void ComputeInertiaTensor(Vector3F * verts, DWORD nVert, double mass, Matrix4x4F & I, Vector3F & cm, double & density);


};


#endif



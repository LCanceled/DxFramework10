
#include "DxUtRigidBody.h"
#include "DxUtTriangle.h"
#include "DxUtMesh.h"

namespace DxUt {

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////     CRigidBody    /////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

CRigidBody::CRigidBody():m_bNotStatic(1),m_pOverrideMaterial(0)
{
}

void CRigidBody::CreateRigidBody(CMesh * pMesh, float scale, float mass, Vector3F & gravity, float timeStepSize,
	float fMaxVelocity, char * szLevelSet, UINT uiTriPerOct, bool bUseHierarchicalLevelSet, GeometryType type, SMaterial * pOverideMaterial)
{
	m_pMesh = pMesh;
	
	UINT nVert = 3*pMesh->GetNumTriangles();
	Vector3F * verts = pMesh->GetNewVertexTriangleList();
	
	double density = 0;
	if (mass < 0) {m_IBody.MZero(); m_CenterOfMass = Vector3F(0); }
	else ComputeInertiaTensor(verts, nVert, mass, m_IBody, m_CenterOfMass, density);
	delete[] verts;

	/* Center the mass at the center of mass */
	/*for (UINT i=0; i<nVert; i++) {
		verts[i] -= m_CenterOfMass;
	}*/

	Assert(szLevelSet, "CRigidBody::CreateRigidBody szLevelSet file must be specified.");
	//if (bUseHierarchicalLevelSet)
	//	m_pLevelSet = new COctreeLevelSet;
	//else 
	m_pLevelSet = new CLevelSet;
	//if (bUseHierarchicalLevelSet)
	//	((COctreeLevelSet*)m_pLevelSet)->CreateParticleRepresentation(verts, nVert, pAdj, 1, uiTriPerOct, szLevelSet);
	//else 
	m_pLevelSet->CreateLevelSet(m_pMesh, szLevelSet);

	m_InvMass = 1.f/mass;
	m_Density = (float)density;
	if (mass < 0) {
		m_InvMass = 0;
		m_InvIBody.MZero();
		m_bNotStatic = 0;
	} else {
		m_InvMass = 1.f/mass;
		m_InvIBody = m_IBody.Inverse();
	}
	m_Scale = scale;
	
	m_RBType = type;

	if (pOverideMaterial) {
		m_pOverrideMaterial = new SMaterial;
		memcpy(m_pOverrideMaterial, pOverideMaterial, sizeof(SMaterial));
	}
}

void CRigidBody::IntegratePos(float dt)
{
	/* p(t+dt) = p(t) + v*dt */
	/* q(t+dt) = q(t) + w* * R */

	m_Pos += dt*m_LinVel;
	m_Rot += dt*m_AngVel.SkewMatrix3x3F()*m_Rot;

	/* Make m_Rot orthogonal */
	/* x */
	float div = (float)(1.0/sqrt(m_Rot.m[0][0]*m_Rot.m[0][0] + m_Rot.m[1][0]*m_Rot.m[1][0] + m_Rot.m[2][0]*m_Rot.m[2][0]));
	m_Rot.m[0][0] *= div, m_Rot.m[1][0] *= div, m_Rot.m[2][0] *= div;

	/* z */
	m_Rot.m[0][2] = m_Rot.m[1][0]*m_Rot.m[2][1] - m_Rot.m[2][0]*m_Rot.m[1][1];
	m_Rot.m[1][2] = m_Rot.m[2][0]*m_Rot.m[0][1] - m_Rot.m[0][0]*m_Rot.m[2][1];
	m_Rot.m[2][2] = m_Rot.m[0][0]*m_Rot.m[1][1] - m_Rot.m[1][0]*m_Rot.m[0][1];
	div = (float)(1.0/sqrt(m_Rot.m[0][2]*m_Rot.m[0][2] + m_Rot.m[1][2]*m_Rot.m[1][2] + m_Rot.m[2][2]*m_Rot.m[2][2]));
	m_Rot.m[0][2] *= div, m_Rot.m[1][2] *= div, m_Rot.m[2][2] *= div;

	/* y */
	m_Rot.m[0][1] = m_Rot.m[1][2]*m_Rot.m[2][0] - m_Rot.m[2][2]*m_Rot.m[1][0];
	m_Rot.m[1][1] = m_Rot.m[2][2]*m_Rot.m[0][0] - m_Rot.m[0][2]*m_Rot.m[2][0];
	m_Rot.m[2][1] = m_Rot.m[0][2]*m_Rot.m[1][0] - m_Rot.m[1][2]*m_Rot.m[0][0];

	Matrix4x4F m = m_Rot.Transpose();
	m_I = m_Rot*m_IBody*m;
	m_InvI = m_Rot*m_InvIBody*m;
}

void CRigidBody::IntegrateVel(float dt, const Vector3F & gAcel)
{
	if (m_InvMass != 0) 
		m_LinVel = m_LinVel + dt*(m_InvMass*m_Force + gAcel);

	m_AngVel = m_AngVel + m_InvI*(
		-dt*(m_AngVel.SkewMatrix3x3F()*m_I*m_AngVel)) + m_Torque;// + m_Torque);
}
/*
UINT CRigidBody::DetermineCollision(CRigidBody * pRB, CArray<SContactPoint> * CPs)
{
	//Code may need to consider m_CenterOfMass
	m_BVTree->SetTransform(m_Rot, m_Pos, m_Scale);
	pRB->m_BVTree->SetTransform(pRB->m_Rot, pRB->m_Pos, m_Scale);

	UINT n = m_BVTree->BVCollision(*pRB->m_BVTree, CPs);
	/*if (n) {
		SRBContact * ptr = CPs->data() + (CPs->size()-n);
		for (UINT i=0; i<n; i++, ptr++) { 
			/* This body i 
			ptr->rBi = this;
			ptr->rBj = pRB;
			
			/* The colliding body j 
			/*SRBContact c;
			c.iPos = ptr->iPos;
			c.iNor = ptr->iNor;
			c.rBi = pRB;
			c.rBi = this;
			pRB->CPs->push_back(c);
		}
	}

	return n;
}*/

UINT CRigidBody::DetermineCollisionLevelSet(CRigidBody * pRB, CArray<SContactPoint> * CPs)
{
	UINT n = m_pLevelSet->LevelSetCollision(*pRB->m_pLevelSet, CPs);

	return n;
}

Vector3F CRigidBody::ComputeLinVel(Vector3F & impulse)
{
	return m_InvMass*impulse;
}

Vector3F CRigidBody::ComputeAngVel(Vector3F & impulse, Vector3F & r)
{
	DebugBreak(); // r is wrong. need center of mass and posiiton
	return  m_InvI*CrossXYZ(r, impulse);
}

float CRigidBody::GetbVector(float * J, Vector3F & gAcel, Vector3F iPos, float dt) 
{
	DebugBreak();
	/*if (m_RigidBodyType == RBT_Static)
		return 0;*/

	float e = 0;
	Vector3F v(m_LinVel + dt*(gAcel) + e*m_LinVel); //m_InvMass*m_Force + 
	Vector3F w(m_AngVel + e*m_AngVel);// + dt*m_InvI*(m_Torque - (m_AngVel.SkewMatrix3x3F()*m_I*m_AngVel)));
	return v.x*J[0] + v.y*J[1] + v.z*J[2] + w.x*J[3] + w.y*J[4] + w.z*J[5]; 
}

Vector3F CRigidBody::GetImpulseCoefficient(Vector3F & jthCt, Vector3F & ithCt, Vector3F & iNor)
{
	DebugBreak();
	return Vector3F(0);
	//return  -m_InvMass*iNor - (m_InvI*((ithCt - (m_Pos)).SkewMatrix3x3F()*iNor)).SkewMatrix3x3F()*(jthCt - (m_Pos));
}

float CRigidBody::GetVelocityAtContactPoint(Vector3F & jthCt, Vector3F & jthNor)
{
	Vector3F vel(m_LinVel - (jthCt - (m_Pos)).SkewMatrix3x3F()*m_AngVel);
	return DotXYZ(vel, jthNor);
}

float CRigidBody::ComputeFrictionlessImpulse(CRigidBody & rB, Vector3F & iPos, Vector3F & iNor)
{
	Vector3F JNA(-iNor);
	Vector3F JWA(-(iPos - (m_Pos)).SkewMatrix3x3F()*iNor);
	Vector3F invMassA(m_InvMass, m_InvMass, m_InvMass);

	Vector3F JNB(iNor);
	Vector3F JWB((iPos - (rB.m_Pos+rB.m_CenterOfMass)).SkewMatrix3x3F()*iNor);
	Vector3F invMassB(rB.m_InvMass, rB.m_InvMass, rB.m_InvMass);

	Vector3F tmp1(JNA*invMassA);
	Vector3F tmp2(JWA*m_InvI);
	Vector3F tmp3(JNB*invMassB);
	Vector3F tmp4(JWB*rB.m_InvI);
	float fEffectiveMass = (DotXYZ(tmp1, JNA) + DotXYZ(tmp2, JWA))*m_bNotStatic + (DotXYZ(tmp3, JNB) + DotXYZ(tmp4, JWB))*rB.m_bNotStatic;
	fEffectiveMass = 1.f/fEffectiveMass;
	float fB = (DotXYZ(JNA, m_LinVel) + DotXYZ(JWA, m_AngVel)) + (DotXYZ(JNB, rB.m_LinVel) + DotXYZ(JWB, rB.m_AngVel));
	return fEffectiveMass*(fB);
}

float CRigidBody::ComputeFrictionImpulse(CRigidBody & rB, Vector3F & iPos, Vector3F & t)
{
	Vector3F JNA(-t);
	Vector3F JWA(-(iPos - (m_Pos)).SkewMatrix3x3F()*t);
	Vector3F invMassA(m_InvMass, m_InvMass, m_InvMass);

	Vector3F JNB(t);
	Vector3F JWB((iPos - (rB.m_Pos)).SkewMatrix3x3F()*t);
	Vector3F invMassB(rB.m_InvMass, rB.m_InvMass, rB.m_InvMass);

	Vector3F tmp1(JNA*invMassA);
	Vector3F tmp2(JWA*m_InvI);
	Vector3F tmp3(JNB*invMassB);
	Vector3F tmp4(JWB*rB.m_InvI);
	float fEffectiveMass = (DotXYZ(tmp1, JNA) + DotXYZ(tmp2, JWA))*m_bNotStatic + (DotXYZ(tmp3, JNB) + DotXYZ(tmp4, JWB))*rB.m_bNotStatic;
	fEffectiveMass = 1.f/fEffectiveMass;
	float fB = (DotXYZ(JNA, m_LinVel) + DotXYZ(JWA, m_AngVel)) + (DotXYZ(JNB, rB.m_LinVel) + DotXYZ(JWB, rB.m_AngVel));
	return fEffectiveMass*(fB);
}

void CRigidBody::ComputeFrictionlessImpulsePart(CRigidBody & rB, Vector3F & iPos, Vector3F & iNor, 
	float & fEffectiveMass, Vector3F & JNA, Vector3F & JWA, Vector3F & JNB, Vector3F & JWB)
{
	JNA = (-iNor);
	JWA = (-(iPos - (m_Pos)).SkewMatrix3x3F()*iNor);
	Vector3F invMassA(m_InvMass, m_InvMass, m_InvMass);

	JNB = (iNor);
	JWB = ((iPos - (rB.m_Pos)).SkewMatrix3x3F()*iNor);
	Vector3F invMassB(rB.m_InvMass, rB.m_InvMass, rB.m_InvMass);

	Vector3F tmp1(JNA*invMassA);
	Vector3F tmp2(JWA*m_InvI);
	Vector3F tmp3(JNB*invMassB);
	Vector3F tmp4(JWB*rB.m_InvI);
	fEffectiveMass = (DotXYZ(tmp1, JNA) + DotXYZ(tmp2, JWA)) + (DotXYZ(tmp3, JNB) + DotXYZ(tmp4, JWB));
	fEffectiveMass = 1.f/fEffectiveMass;
}

void CRigidBody::ComputeFrictionImpulsePart(CRigidBody & rB, Vector3F & iPos, Vector3F & t,
	float & fEffectiveMass, Vector3F & JNA, Vector3F & JWA, Vector3F & JNB, Vector3F & JWB)
{
	JNA = (-t);
	JWA = (-(iPos - (m_Pos)).SkewMatrix3x3F()*t);
	Vector3F invMassA(m_InvMass, m_InvMass, m_InvMass);

	JNB = (t);
	JWB = ((iPos - (rB.m_Pos)).SkewMatrix3x3F()*t);
	Vector3F invMassB(rB.m_InvMass, rB.m_InvMass, rB.m_InvMass);

	Vector3F tmp1(JNA*invMassA);
	Vector3F tmp2(JWA*m_InvI);
	Vector3F tmp3(JNB*invMassB);
	Vector3F tmp4(JWB*rB.m_InvI);
	fEffectiveMass = (DotXYZ(tmp1, JNA) + DotXYZ(tmp2, JWA))*m_bNotStatic + (DotXYZ(tmp3, JNB) + DotXYZ(tmp4, JWB))*rB.m_bNotStatic;
	fEffectiveMass = 1.f/fEffectiveMass;
}

void CRigidBody::ApplyImpulse(Vector3F & impulse, Vector3F & iPos)
{
	if (!m_bNotStatic) return;
	Vector3F r = iPos - (m_Pos);
	m_AngVel += m_InvI*CrossXYZ(r, impulse);
	m_LinVel += m_InvMass*impulse;
}

void CRigidBody::ApplyFrictionImpulse(Vector3F & impulse, Vector3F & iPos)
{
	if (!m_bNotStatic) return;
	Vector3F r = iPos - (m_Pos);
	m_AngVel -= m_InvI*CrossXYZ(r, impulse);
	m_LinVel -= m_InvMass*impulse;
}

void CRigidBody::DestroyRigidBody() 
{
	m_pLevelSet->DestroyLevelSet();
	delete m_pLevelSet;
	m_pLevelSet = NULL;
	if (m_pOverrideMaterial) {
		delete m_pOverrideMaterial;
		m_pOverrideMaterial = NULL;
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////    CRigidBody Functions    /////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ComputeVolume(STriangleF * tris, UINT nTri, double & vol)
{
	double f1x;
	double intg = 0;
	for (UINT i=0; i<nTri; i++) {
		double x0 = tris[i].vPosW[0].x, y0 = tris[i].vPosW[0].y, z0 = tris[i].vPosW[0].z;
		double x1 = tris[i].vPosW[1].x, y1 = tris[i].vPosW[1].y, z1 = tris[i].vPosW[1].z;
		double x2 = tris[i].vPosW[2].x, y2 = tris[i].vPosW[2].y, z2 = tris[i].vPosW[2].z;

		// get edges (1(a,b,c), 2(a,b,c)) and cross product of edges
		double b1 = y1-y0, c1 = z1-z0, b2 = y2-y0, c2 = z2-z0;
		double d0 = b1*c2-b2*c1;

		// compute integral term
		f1x = x0+x1+x2;							

		// update integral
		intg += d0*f1x;
	}

	// volume of polyhedron
	vol = intg/6.0;
}
/*
//The vertices must be in a triangle list order
void ComputeVolume(Vector3F * verts, UINT nVert, double & vol)
{
	if (nVert % 3) {DxUtSendError("ComputeVolume the verts must be divisible by 3.");}
	UINT nTri = nVert/3;

	ComputeVolume((STriangleF*)verts, nTri, vol);
}*/

#define Subexpressions(w0,w1,w2,f1,f2,f3,g0,g1,g2,temp0,temp1,temp2)		\
{																			\
	temp0 = w0+w1; f1 = temp0+w2; temp1 = w0*w0; temp2 = temp1+w1*temp0;	\
	f2 = temp2+w2*f1; f3 = w0*temp1+w1*temp2+w2*f2;							\
	g0 = f2+w0*(f1+w0); g1 = f2+w1*(f1+w1); g2 = f2+w2*(f1+w2);				\
}

void ComputeInertiaTensor(STriangleF * tris, UINT nTri, double mass, Matrix4x4F & I, Vector3F & cm, double & density)
{
	double temp0, temp1, temp2;
	double f1x, f2x, f3x, g0x, g1x, g2x;
	double f1y, f2y, f3y, g0y, g1y, g2y;
	double f1z, f2z, f3z, g0z, g1z, g2z;
	const double mult[10] = {1.f/6,1.f/24,1.f/24,1.f/24,1.f/60,1.f/60,1.f/60,1.f/120,1.f/120,1.f/120};
	double intg[10] = {0,0,0,0,0,0,0,0,0,0}; // order: 1, x, y, z, x^2, y^2, z^2, xy, yz, zx

	for (UINT i=0; i<nTri; i++) {
		double x0 = tris[i].vPosW[0].x, y0 = tris[i].vPosW[0].y, z0 = tris[i].vPosW[0].z;
		double x1 = tris[i].vPosW[1].x, y1 = tris[i].vPosW[1].y, z1 = tris[i].vPosW[1].z;
		double x2 = tris[i].vPosW[2].x, y2 = tris[i].vPosW[2].y, z2 = tris[i].vPosW[2].z;

		// get edges (1(a,b,c), 2(a,b,c)) and cross product of edges
		double a1 = x1-x0, b1 = y1-y0, c1 = z1-z0, a2 = x2-x0, b2 = y2-y0, c2 = z2-z0;
		double d0 = b1*c2-b2*c1,  d1 = a2*c1-a1*c2,  d2 = a1*b2-a2*b1;

		// compute integral terms
		Subexpressions(x0,x1,x2,f1x,f2x,f3x,g0x,g1x,g2x,temp0,temp1,temp2);
		Subexpressions(y0,y1,y2,f1y,f2y,f3y,g0y,g1y,g2y,temp0,temp1,temp2);
		Subexpressions(z0,z1,z2,f1z,f2z,f3z,g0z,g1z,g2z,temp0,temp1,temp2);

		// update integrals
		intg[0] += d0*f1x;
		intg[1] += d0*f2x; intg[2] += d1*f2y; intg[3] += d2*f2z;
		intg[4] += d0*f3x; intg[5] += d1*f3y; intg[6] += d2*f3z;
		intg[7] += d0*(y0*g0x+y1*g1x+y2*g2x);
		intg[8] += d1*(z0*g0y+z1*g1y+z2*g2y);
		intg[9] += d2*(x0*g0z+x1*g1z+x2*g2z);
	}
	for (WORD i=0; i<10; i++)
		intg[i] *= mult[i];

	// density of polyhedron
	density = mass/intg[0];

	// center of mass
	cm.x = (float)(intg[1]/intg[0]);
	cm.y = (float)(intg[2]/intg[0]);
	cm.z = (float)(intg[3]/intg[0]);

	// inertia tensor relative to center of mass
	I.m[0][0] = (float)(density*(intg[5]+intg[6]) - mass*(cm.y*cm.y+cm.z*cm.z));
	I.m[1][1] = (float)(density*(intg[4]+intg[6]) - mass*(cm.z*cm.z+cm.x*cm.x));
	I.m[2][2] = (float)(density*(intg[4]+intg[5]) - mass*(cm.x*cm.x+cm.y*cm.y));
	I.m[0][1] = I.m[1][0] = (float)(-density*intg[7] + mass*cm.x*cm.y);
	I.m[1][2] = I.m[2][1] = (float)(-density*intg[8] + mass*cm.y*cm.z);
	I.m[2][0] = I.m[0][2] = (float)(-density*intg[9] + mass*cm.z*cm.x);

	I.m[3][3] = 1.f;
	I.m[0][3] = I.m[1][3] = I.m[2][3] = I.m[3][0] = I.m[3][1] = I.m[3][2] = 0;
}

//The vertices must be in a triangle list order
void ComputeInertiaTensor(Vector3F * verts, UINT nVert, double mass,
	Matrix4x4F & I, Vector3F & cm, double & density)
{
	if (nVert % 3) {DxUtSendError("ComputeInertiaTensor the verts must be divisible by 3.");}
	UINT nTri = nVert/3;

	STriangleF * rgTris = new STriangleF[nTri];
	for (UINT i=0; i<nTri; i++) {
		rgTris[i] = STriangleF(verts[3*i+0],verts[3*i+1],verts[3*i+2]);
	}
	ComputeInertiaTensor(rgTris, nTri, mass,  I, cm, density);
	delete[] rgTris;
}


};



	/*D3DXQUATERNION w(-m_AngVel.x, -m_AngVel.y, -m_AngVel.z, 0);
	D3DXQuaternionMultiply(&w, &w, &m_QRot);
	m_QRot = m_QRot + .5f*w*dt;
	D3DXQuaternionNormalize(&m_QRot, &m_QRot);

	D3DXMATRIX rot;
	D3DXMatrixRotationQuaternion(&rot, &m_QRot);
	m_Rot.m[0][0] = rot._11, m_Rot.m[1][0] = rot._21, m_Rot.m[2][0] = rot._31;
	m_Rot.m[0][1] = rot._12, m_Rot.m[1][1] = rot._22, m_Rot.m[2][1] = rot._32;
	m_Rot.m[0][2] = rot._13, m_Rot.m[1][2] = rot._23, m_Rot.m[2][2] = rot._33;*/

#include "DxUtRigidBodyWorld.h"
#include "DxUtTriangle.h"
#include "DxUtTimer.h"
#include "DxUtD3DApp.h"

namespace DxUt {

CRigidBodyWorld::CRigidBodyWorld():m_pRBObject(0)
{
}

void CRigidBodyWorld::CreateRigidBodyWorld(UINT nHintRBs, UINT nHintContacts, bool bUseHierarchicalLevelSet, Vector3F gravity, float stepSize, float maxVel)
{
	Assert(!m_RBObjects.GetSize(), "CRigidBodyWorld::CreateRigidBodyWorld must destroy the world before creating a new one.");
	Assert(nHintRBs > 0 && nHintContacts > 0, "CRigidBodyWorld::CreateRigidBodyWorld must enter nonzero values for the hints.");

	m_RigidBodies.Reserve(nHintRBs);
	m_RBObjects.Reserve(nHintRBs);
	m_RBContactPoints.Reserve(nHintContacts);
	m_ContactPoints.Reserve(nHintContacts);
	
	m_Gravity = gravity;
	m_TimeStepSize = stepSize;
	m_MaxVelocity = maxVel;
	m_bUseHierarchicalLevelSet = bUseHierarchicalLevelSet;
}

UINT CRigidBodyWorld::AddRigidBody(CMesh * pMesh, float scale, float mass, const Vector3F & pos, const Matrix4x4F & rot, const Vector3F & linVel, const Vector3F & angVel,
	float elasticity, float mu, const Vector3F & force, const Vector3F & torque, char * szLevelSet, UINT uiTriPerOct, CRigidBody::GeometryType type, SMaterial * pOverrideMaterial)
{
	m_RBObjects.PushBack();
	SRBObject & rbo = m_RBObjects.GetBack();

	rbo.bUsed = 0;
	rbo.bActive = 1;
	rbo.bDenter = 0;
	m_RigidBodies.PushBack();
	CRigidBody & r = m_RigidBodies.GetBack();
	r.CreateRigidBody(pMesh, scale, mass, m_Gravity, m_TimeStepSize, m_MaxVelocity, szLevelSet, uiTriPerOct, m_bUseHierarchicalLevelSet, type, pOverrideMaterial);
	r.m_Pos = pos;
	r.m_Rot = rot;
	r.m_LinVel = linVel;
	r.m_AngVel = angVel;
	r.m_Elasticity = elasticity;
	r.m_Mu = mu;
	r.m_Force = force;
	r.m_Torque = torque;
	rbo.bConstrainCM = 0;
	rbo.rB.AddRigidBody(&r);

	m_pRBObject = m_RBObjects.GetData();

	m_RBType = type;

	return m_RBObjects.GetSize()-1;
}

UINT CRigidBodyWorld::AddRigidBodyToCluster(UINT uiCluster, CMesh * pMesh, float scale, float mass, const Vector3F & pos, const Matrix4x4F & rot, const Vector3F & linVel, const Vector3F & angVel,
	float elasticity, float mu, const Vector3F & force, const Vector3F & torque, char * szLevelSet, UINT uiTriPerOct, CRigidBody::GeometryType type, SMaterial * pOverrideMaterial)
{
	m_RBObjects.PushBack();
	SRBObject & rbo = m_RBObjects.GetBack();

	rbo.bUsed = 0;
	rbo.bActive = 1;
	rbo.bDenter = 0;
	m_RigidBodies.PushBack();
	CRigidBody & r = m_RigidBodies.GetBack();
	r.CreateRigidBody(pMesh, scale, mass, m_Gravity, m_TimeStepSize, m_MaxVelocity, szLevelSet, uiTriPerOct, m_bUseHierarchicalLevelSet, type, pOverrideMaterial);
	r.m_Pos = pos;
	r.m_Rot = rot;
	r.m_LinVel = linVel;
	r.m_AngVel = angVel;
	r.m_Elasticity = elasticity;
	r.m_Mu = mu;
	r.m_Force = force;
	r.m_Torque = torque;
	rbo.bConstrainCM = 0;
	rbo.rB.AddRigidBody(&r);

	m_pRBObject = m_RBObjects.GetData();

	m_RBType = type;

	return m_RBObjects.GetSize()-1;
}

/* Constrains the center of mass to lie at a certain position */
void CRigidBodyWorld::AddCenterOfMassPositionConstraint(UINT uiRigidBody, Vector3F & pos)
{
	m_RBObjects[uiRigidBody].bConstrainCM = 1;
	m_RBObjects[uiRigidBody].constraintCMPos = pos;
}


void CRigidBodyWorld::DisableRigidBody(UINT uiId) 
{
	Assert(uiId < m_RBObjects.GetSize(), "CRigidBodyWorld::DisableRigidBody "
		"an id was specifed which does not exist for any body.");

	m_pRBObject[uiId].bActive = 0;
}

void CRigidBodyWorld::EnableRigidBody(UINT uiId) 
{
	Assert(uiId < m_RBObjects.GetSize(), "CRigidBodyWorld::EnableRigidBody "
		"an id was specifed which does not exist for any body.");

	m_pRBObject[uiId].bActive = 1;
}

UINT CRigidBodyWorld::FindContacts()
{
	UINT nTotalContacts = 0;
	UINT s = m_RBObjects.GetSize();
	m_RBContactPoints.Resize(0);
	m_ContactPoints.Resize(0);
	for (UINT i=0, end=m_ContactLayers.GetSize(); i<end; i++) 
		m_ContactLayers.Resize(0);
	for (UINT i=0, end=m_RBObjects.GetSize(); i<end; i++)
		m_RBObjects[i].RBCPs.Resize(0);

	/* Transform all the bodies */
	for (UINT i=0; i<s; i++)
		m_pRBObject[i].rB.TransformLevelSet();
	
	UINT uiContact = 0;
	UINT uiPairs = 0;
	for (UINT i=0; i<s; i++) {/* FIX FOR NONACTIVE RIGID BODIES */
		for (UINT j=i+1, end=min(s, j+1+35); j<s; j++) {
			if (m_pRBObject[i].rB.IsStatic() && m_pRBObject[j].rB.IsStatic())
				continue;

			UINT nContactsij = m_pRBObject[i].rB.DetermineCollisionLevelSet(&m_pRBObject[j].rB, &m_ContactPoints);

			Vector3F nor(0, 0, 0);
			nTotalContacts += nContactsij;
			if (nContactsij) {
				m_RBContactPoints.Resize(m_RBContactPoints.GetSize() + nContactsij);
				SRBContact * RBCPs = m_RBContactPoints.GetData() + (m_RBContactPoints.GetSize() - nContactsij);
				SContactPoint * CPs = m_ContactPoints.GetData() + (m_ContactPoints.GetSize() - nContactsij);
				for (UINT k=0; k<nContactsij; k++) {
					RBCPs[k].iPos = CPs[k].iPos;
					RBCPs[k].iNor = CPs[k].iNor;
					RBCPs[k].dist = CPs[k].dist;
					RBCPs[k].rBOk = &m_pRBObject[i];
					RBCPs[k].rBOl = &m_pRBObject[j];
					RBCPs[k].uiIndex = uiContact++;
				}
			}
		}
	}
					
	SRBContact * RBCPs = m_RBContactPoints.GetData();
	for (UINT i=0, end=m_RBContactPoints.GetSize(); i<end; i++) {
		RBCPs[i].rBOk->RBCPs.PushBack(&RBCPs[i]);
		RBCPs[i].rBOl->RBCPs.PushBack(&RBCPs[i]);
	}

	return nTotalContacts;
}

void CRigidBodyWorld::ComputeContactGraph()
{
	CArray<SRBObject*> objQueues;
	objQueues.Reserve(m_RBObjects.GetSize());
	/* Push the static objects first onto the queue  */
	UINT uiInfinity = 1000000;
	SRBObject * pRBObjects = m_RBObjects.GetData();
	for (UINT i=0, end=m_RBObjects.GetSize(); i<end; i++) {
		pRBObjects[i].iHeight = uiInfinity;
		pRBObjects[i].bUsed = 0;

		if (pRBObjects[i].rB.IsStatic()) {
			objQueues.PushBack();
			objQueues.GetBack() = &m_RBObjects[i];
			pRBObjects[i].iHeight = 0;
			pRBObjects[i].bUsed = 1;
		}
	}

	UINT uiCounter = 0, nLayers = 0;
	while (objQueues.GetSize() > uiCounter) {
		SRBObject * pObjQueue = objQueues[uiCounter];

		for (UINT i=0, nCPs=pObjQueue->RBCPs.GetSize(); i<nCPs; i++) {
			SRBContact ** pCPs = pObjQueue->RBCPs.GetData();
			SRBObject * pNeighborObj;
			if (pCPs[i]->rBOk != pObjQueue) pNeighborObj = pCPs[i]->rBOk;
			else pNeighborObj = pCPs[i]->rBOl;

			if (!pNeighborObj->bUsed) {
				objQueues.PushBack(pNeighborObj);
				pNeighborObj->bUsed = 1;
			}
			pNeighborObj->iHeight = min(pNeighborObj->iHeight, pObjQueue->iHeight + 1);
			if (pNeighborObj->iHeight == pObjQueue->iHeight && pNeighborObj->iHeight != 0)
				pCPs[i]->uiLayer = pNeighborObj->iHeight - 1;
			else pCPs[i]->uiLayer = min(pNeighborObj->iHeight, pObjQueue->iHeight);

			nLayers = max(nLayers, pCPs[i]->uiLayer);
		}

		uiCounter++;
	}
	objQueues.Clear();

	/* All bodies not touching a static body */
	for (UINT i=0, end=m_RBObjects.GetSize(); i<end; i++) { 
		if (!pRBObjects[i].bUsed) {
			for (UINT j=0, nCPs=pRBObjects[i].RBCPs.GetSize(); j<nCPs; j++) {
				//SRBContact ** pRBCPs = pRBObjects[i].RBCPs.GetData();
				//pRBCPs[j]->uiLayer = nLayers+1;
				pRBObjects[i].RBCPs[j]->uiLayer = nLayers+1;
			}
		}
	}

	m_ContactLayers.Resize(nLayers+2);
	SContactLayer * pContactLayers = m_ContactLayers.GetData();
	SRBContact * pRBContactPoints = m_RBContactPoints.GetData();
	for (UINT i=0, end=m_ContactLayers.GetSize(); i<end; i++) {
		pContactLayers[i].RBCPs.Reserve(m_RBContactPoints.GetSize());
		pContactLayers[i].RBCPs.Resize(0);
		
		pContactLayers[i].RBObjs.Reserve(m_RBContactPoints.GetSize());
		pContactLayers[i].RBObjs.Resize(0);
	}
	/* Add the contact points */
	for (UINT i=0, end=m_RBContactPoints.GetSize(); i<end; i++) {
		UINT uiIndex = pRBContactPoints[i].uiLayer;
		pContactLayers[uiIndex].RBCPs.PushBack(m_RBContactPoints[i]);
	}
	/* Add the rigid body objects */
	bool cool=0;
	for (UINT i=0, end=m_RBObjects.GetSize(); i<end; i++) {
		if (pRBObjects[i].iHeight == uiInfinity) 
			cool = 1;
	}
	if (!cool)
		int asdf=0;
	for (UINT i=0, end=m_RBObjects.GetSize(); i<end; i++) {
		if (pRBObjects[i].iHeight == uiInfinity) 
			continue;

		bool bUpper = 0;
		bool bLower = 0;
		for (UINT j=0, end2=pRBObjects[i].RBCPs.GetSize(); j<end2; j++) {
			SRBContact ** pCPs =  pRBObjects[i].RBCPs.GetData();
			SRBObject * pNeighborObj;
			if (pCPs[j]->rBOk != &pRBObjects[i]) pNeighborObj = pCPs[j]->rBOk;
			else pNeighborObj = pCPs[j]->rBOl;

			if (pNeighborObj->iHeight > pRBObjects[i].iHeight)
				bUpper = 1;
			if (pNeighborObj->iHeight < pRBObjects[i].iHeight)
				bLower = 1;
			if (bUpper && bLower)
				break;
		}

		//if (bUpper)
			m_ContactLayers[pRBObjects[i].iHeight].RBObjs.PushBack(&pRBObjects[i]);
		//if (bLower)
		//	m_ContactLayers[pRBObjects[i].iHeight-1].RBObjs.PushBack(&pRBObjects[i]);
	}
}

void CRigidBodyWorld::PGSSetup(UINT nContacts)
{
	for (UINT k=0, end=m_ContactLayers.GetSize(); k<end; k++) {
		SRBContact * CPs = m_ContactLayers[k].RBCPs.GetData();
		for (UINT j=0, end2=m_ContactLayers[k].RBCPs.GetSize(); j<end2; j++) {
		//SRBContact * CPs = m_RBContactPoints.GetData();
		//for (UINT j=0, end2=m_RBContactPoints.GetSize(); j<end2; j++) {
			SRBContact & rbC = CPs[j];
			UINT uiIndex = CPs[j].uiIndex;
			Vector3F & nor = rbC.iNor;
			Vector3F t1(Vector3F(rbC.rBOl->rB.m_LinVel - rbC.rBOk->rB.m_LinVel + Vector3F(.1f)).Normalize());
			Vector3F & t2 = CrossXYZ(t1, nor).Normalize();
			t1 = CrossXYZ(t2, nor).Normalize();

			rbC.t1 = t1;
			rbC.t2 = t2;

			rbC.rBOk->rB.ComputeFrictionlessImpulsePart(rbC.rBOl->rB, rbC.iPos, nor,
				rbC.effectiveMass[0], rbC.JNA[0], rbC.JWA[0], rbC.JNB[0], rbC.JWB[0]);

			rbC.rBOk->rB.ComputeFrictionImpulsePart(rbC.rBOl->rB, rbC.iPos, t1,
				rbC.effectiveMass[1], rbC.JNA[1], rbC.JWA[1], rbC.JNB[1], rbC.JWB[1]);

			rbC.rBOk->rB.ComputeFrictionImpulsePart(rbC.rBOl->rB, rbC.iPos, t2,
				rbC.effectiveMass[2], rbC.JNA[2], rbC.JWA[2], rbC.JNB[2], rbC.JWB[2]);

			/* Compute velocities for push out */
			rbC.pushOutVel = (DotXYZ(rbC.JNA[0], rbC.rBOk->rB.m_LinVel) + DotXYZ(rbC.JWA[0], rbC.rBOk->rB.m_AngVel)) +
				(DotXYZ(rbC.JNB[0], rbC.rBOl->rB.m_LinVel) + DotXYZ(rbC.JWB[0], rbC.rBOl->rB.m_AngVel));
		}
	}
}

#define COMPUTE_IMPULSE(index, impulse, pushOut) \
	fB = (DotXYZ(rbC.JNA[index], rbk.m_LinVel) + DotXYZ(rbC.JWA[index], rbk.m_AngVel)) + \
			(DotXYZ(rbC.JNB[index], rbl.m_LinVel) + DotXYZ(rbC.JWB[index], rbl.m_AngVel)); \
	impulse =  rbC.effectiveMass[index]*(fB - pushOut);

void CRigidBodyWorld::SolveLayer(UINT k, VectorNF & lambda, float dt)
{
	SRBContact * CPs = m_ContactLayers[k].RBCPs.GetData();
	for (UINT j=0, end2=m_ContactLayers[k].RBCPs.GetSize(); j<end2; j++) {
	//SRBContact * CPs = m_RBContactPoints.GetData();
	//for (UINT j=0, end2=m_RBContactPoints.GetSize(); j<end2; j++) {
		SRBContact & rbC = CPs[j];
		UINT uiIndex = CPs[j].uiIndex;
		CRigidBody & rbk = rbC.rBOk->rB;
		CRigidBody & rbl = rbC.rBOl->rB;
		//float mu = 0.0;//1; // Friction is WRONG! box platform
		float mu = Min(rbC.rBOk->rB.m_Mu, rbC.rBOl->rB.m_Mu);
		mu = .1f;

		float fB, fImpulseN, fImpulseT1=0, fImpulseT2=0;
		float fPushOut = .1f*(rbC.pushOutVel*dt + rbC.dist)/dt;
		if (!m_bUsePushOut || rbC.dist > -.01f) fPushOut = 0;
		COMPUTE_IMPULSE(0, fImpulseN, fPushOut);
		//g_D3DApp->Print(fB + fPushOut/dt);
		COMPUTE_IMPULSE(1, fImpulseT1, 0);
		COMPUTE_IMPULSE(2, fImpulseT2, 0);

		float lambdaUpdateN = lambda.c[3*uiIndex+0] + fImpulseN;
		float lambdaUpdateT1= lambda.c[3*uiIndex+1] + fImpulseT1;
		float lambdaUpdateT2= lambda.c[3*uiIndex+2] + fImpulseT2;
		if (lambdaUpdateN < 0) lambdaUpdateN = 0;
		float fLimit = mu*lambdaUpdateN;
		if (lambdaUpdateT1 < -fLimit) 
			lambdaUpdateT1 = -fLimit;
		else if (lambdaUpdateT1 > fLimit) 
			lambdaUpdateT1 = fLimit;
		if (lambdaUpdateT2 < -fLimit)
			lambdaUpdateT2 = -fLimit; 
		else if (lambdaUpdateT2 > fLimit)
			lambdaUpdateT2 = fLimit;
		float difN = lambdaUpdateN -  lambda.c[3*uiIndex+0];
		float difT1= lambdaUpdateT1-  lambda.c[3*uiIndex+1];
		float difT2= lambdaUpdateT2-  lambda.c[3*uiIndex+2];

		Vector3F impulse = difN*rbC.iNor + difT1*rbC.t1 + difT2*rbC.t2;
		rbC.rBOk->rB.ApplyImpulse(impulse, rbC.iPos);
		rbC.rBOl->rB.ApplyImpulse(-impulse, rbC.iPos);

		lambda.c[3*uiIndex+0] = lambdaUpdateN;
		lambda.c[3*uiIndex+1] = lambdaUpdateT1;
		lambda.c[3*uiIndex+2] = lambdaUpdateT2;
	}
}

#define PRINT

void CRigidBodyWorld::PGSSolve(UINT nContacts, float dt, bool bNeedSetup)
{
	if (bNeedSetup) {
		ComputeContactGraph();

		PGSSetup(nContacts);
	}

	VectorNF lambda;
	lambda.CreateVector(3*nContacts);
	lambda.Set(0);

	//CArray<Vector3F> rgFrictionImpulses;
	//rgFrictionImpulses.Reserve(3*nContacts);

	UINT nIterations = 100;
	for (UINT i=0; i<nIterations; i++) {
		for (UINT k=0, end=m_ContactLayers.GetSize(); k<end; k++)
			SolveLayer(k, lambda, dt);

		/* Position constraints */
		for (UINT k=0, end=m_RBObjects.GetSize(); k<end; k++) {
			ApplyCMConstraint(m_RBObjects[k], dt);
		}
	}

	/* Shock propagation */
	lambda.Set(0);
	for (UINT i=0; i<nIterations/2 + 1; i++) {
		for (UINT k=0, end=m_ContactLayers.GetSize(); k<end; k++) {
			SolveLayer(k, lambda, dt);

			UINT level = min(k+1, end-1);
			SRBObject ** pRBObjects = m_ContactLayers[level].RBObjs.GetData();
			for (UINT j=0, end2=m_ContactLayers[level].RBObjs.GetSize(); j<end2; j++) {
				pRBObjects[j]->rB.SetStatic(1);
			}
		}

		// Reset everything 
		SRBObject * pRBObjects = m_RBObjects.GetData();
		for (UINT i=0, end=m_RBObjects.GetSize(); i<end; i++) { 
			pRBObjects[i].rB.SetStatic(0);
		}
		
		/* Position constraints */
		for (UINT k=0, end=m_RBObjects.GetSize(); k<end; k++) {
			ApplyCMConstraint(m_RBObjects[k], dt);
		}
	}

	lambda.DestroyVector();
	//rgFrictionImpulses.Clear();
}

void CRigidBodyWorld::UpdateRigidBodies(float dt, const Vector3F & gAcel)
{
	UINT s = m_RBObjects.GetSize();
	//g_D3DApp->Print("Step");
	//while (dtTotalStep < dt) {
		/* Calculate sub step restriction */
		/*float maxVel = 1e-4;
		float fMinThickness = .15f;		//TODO: Compute this automatically
		float dtSubStep = 0, dtTotalStep = 0;
		for (UINT i=0; i<s; i++) {
			if (!m_pRBObject[i].bActive) continue;

			m_pRBObject[i].rB.SavePosAndRot();
			Vector3F & angVel = m_pRBObject[i].rB.GetAngVel();
			float fMaxRadius = sqrtf(m_pRBObject[i].rB.MaxRadiusSq());
			float fVel = m_pRBObject[i].rB.GetLinVel().Length() + angVel.Length()*fMaxRadius;
			if (maxVel < fVel) maxVel = fVel;
		}
		float fRestriction = .5f*fMinThickness/maxVel;
		if (dtTotalStep + fRestriction > dt) dtSubStep = (dt - dtTotalStep) + 1e-4;
		else dtSubStep += fRestriction;
		dtTotalStep += dtSubStep;
		dtTotalStep = dt + 1e-4;
		dtSubStep = dt;
		//g_D3DApp->Print(dtSubStep);
		*/
		float dtSubStep = dt;

		for (UINT i=0; i<s; i++) {
			if (!m_pRBObject[i].bActive) continue;

			m_pRBObject[i].rB.SavePosAndRot();
			m_pRBObject[i].rB.SaveLinAndAngVel();
			m_pRBObject[i].rB.IntegratePos(dtSubStep);
		}

#ifdef PRINT
		CTimer::Get().StartTimer(0);
#endif
		UINT nContacts = FindContacts();
#ifdef PRINT
		CTimer::Get().EndTimer(0, "End FindContacts");
#endif
		if (nContacts) {
			//g_D3DApp->Print(nContacts, "");

			m_bUsePushOut = 0;
#ifdef PRINT
			CTimer::Get().StartTimer(1);
#endif
			PGSSolve(nContacts, dtSubStep, 1);
#ifdef PRINT
			CTimer::Get().EndTimer(1, "End PGSSolve");
#endif
		}
#ifdef PRINT
		CTimer::Get().PresentTimers(1, 1);
#endif

		for (UINT i=0; i<s; i++) {
			if (!m_pRBObject[i].bActive) continue;

			m_pRBObject[i].rB.RestorePosAndRot();
			m_pRBObject[i].rB.IntegratePos(dtSubStep);
			m_pRBObject[i].rB.RestoreLinAndAngVel();
				
			m_pRBObject[i].rB.IntegrateVel(dtSubStep, gAcel); 
			if (m_pRBObject[i].rB.m_LinVel.Length() > m_MaxVelocity) {
				m_pRBObject[i].rB.m_LinVel = m_MaxVelocity*m_pRBObject[i].rB.m_LinVel.Normalize();
				// Need to account for angular as well
			}
			m_pRBObject[i].bUsed = 0; 
		}

		/* Post stab stuff*/
		if (nContacts) {
			m_bUsePushOut = 0;
			m_pRBObject[1];
			PGSSolve(nContacts, dtSubStep, 0);
		}

		
		/*static int once = 1;
		if (once) {
			m_RBObjects[1].bDenter = 1;
			m_RBObjects[2].bDenter = 1;
			m_RBObjects[3].bDenter = 1;
			m_RBObjects[4].bDenter = 1;
			once = 0;
			//Dent(Vector3F(-.1f, 1.f, 0));
			//Dent(Vector3F(.1f, 1.f, 0));

		}

		// Denting
		if (nContacts && m_RBObjects[0].RBCPs.GetSize()) {
			m_RBObjects[0].RBCPs[0]->rBOk;
			for (int i=0; i<m_RBObjects[0].RBCPs.GetSize(); i++) {
				float v = m_RBObjects[1].rB.GetLinVel().Length();
				if (m_RBObjects[0].RBCPs[i]->rBOl->bDenter) {// == &m_RBObjects[1] && m_RBObjects[1].bDenter) {
					Dent(m_RBObjects[0].RBCPs[i]->iPos);
					m_RBObjects[0].RBCPs[i]->rBOl->bDenter = 0;
				}
			}
		}*/
		
	//}
}
/*
void CRigidBodyWorld::Dent(Vector3F & cpt)
{
	float sigma = 15.f;
	float A = .2f;

	ID3DX10MeshBuffer * vBuf;
	ID3DX10MeshBuffer * iBuf;
	ID3DX10Mesh * mesh = m_RBObjects[0].rB.m_pMesh->GetMesh();
	mesh->GetVertexBuffer(0, &vBuf);
	mesh->GetIndexBuffer(&iBuf);
	SIZE_T sizeV = vBuf->GetSize();
	SIZE_T sizeI = iBuf->GetSize();
	SVertexPNT * dataV;
	UINT * dataI;
	vBuf->Map((void**)&dataV, &sizeV);
	iBuf->Map((void**)&dataI, &sizeI);
	for (int i=0; i<mesh->GetVertexCount(); i++) {
		Vector3F & pos = dataV[i].pos;
		float distSq = (pos-cpt).LengthSq();
		pos.y -= A*exp(-sigma*distSq);
	}
	for (int i=0; i<mesh->GetFaceCount()*3; i+=3) {
		Vector3F & v0 = dataV[dataI[i]].pos;
		Vector3F & v1 = dataV[dataI[i+1]].pos;
		Vector3F & v2 = dataV[dataI[i+2]].pos;
		Vector3F e0(v1-v0);
		Vector3F e1(v2-v0);
		Vector3F n(CrossXYZ(e0, e1).Normalize());
				
		dataV[dataI[i]].nor = n;
		dataV[dataI[i+1]].nor = n;
		dataV[dataI[i+2]].nor = n;
	}
	vBuf->Unmap();
	iBuf->Unmap();
	mesh->CommitToDevice();

	CLevelSet * level = m_RBObjects[0].rB.GetLevelSet();

	for (int i=0; i<level->m_nLSVertsZ; i++) {
	//for (int i=0; i<0; i++) {
		for (int j=0; j<level->m_nLSVertsY; j++) {
			for (int k=0; k<level->m_nLSVertsX; k++) {
				Vector3F pos(k*level->m_CellSize, j*level->m_CellSize, i*level->m_CellSize);
				pos += level->m_TransformObjToGrid;
				pos = level->m_TransformObjToWorld*pos;
				float & lDist = level->m_LSVertices[i*level->m_nLSVertsY*level->m_nLSVertsX + j*level->m_nLSVertsX + k].dist;
				//if (lDist > 0) continue;

				float x = (pos-cpt).Length();
				if (x > A+.05) continue;

				float y = cpt.y-pos.y;
				float v = 2.f*sigma*A*exp(-sigma*x*x);
				float nor = sqrt(x*x + 1.f/(v*v));
				float alpha = (y - A*exp(-sigma*x*x)) * v * nor;

				if (-alpha > 0)
					lDist = -alpha;
				else
					lDist = min(lDist, -alpha);
			}
		}
	}
}
*/

void CRigidBodyWorld::ApplyCMConstraint(SRBObject & rBO, float dt)
{
	if (!rBO.bActive || !rBO.bConstrainCM || !rBO.rB.m_bNotStatic) return;

	Vector3F newPos(rBO.rB.m_PrePos + dt * rBO.rB.m_LinVel);
	Vector3F & target = rBO.constraintCMPos;
	Vector3F dir = (target - newPos);
	float dist = dir.Length();
	if (dist > 0) {
		dir = dir/dist;
		rBO.rB.m_LinVel += (dist/dt) * dir;
	}

}

void CRigidBodyWorld::DrawCollisionGraphics(CCamera * pCam)
{
	/* Transform all the bodies */
	for (UINT i=0; i<m_RBObjects.GetSize(); i++)
		m_pRBObject[i].rB.TransformLevelSet();

	CCollisionGraphics::SetCamera(pCam);
	CCollisionGraphics::DrawContactPoints((CArray<SContactPoint>*)&m_ContactPoints);
	/*for (UINT i=1, end=2; i<end; i++) {
		//((COctreeLevelSet*)m_RBObjects[i].rB.GetLevelSet())->AssignCG(&m_CollisionGraphics);
		//m_RBObjects.GetSize(); i<end; i++) {
		m_RBObjects[i].rB.GetLevelSet()->DrawLevelset();
		//m_RBObjects[i].rB.GetLevelSet().DrawAdmissibleZones(&m_CollisionGraphics);
		
		//m_RBObjects[i].rB.GetLevelSet().DrawBVTree(&m_CollisionGraphics, 8);
		//((COctreeLevelSet*)m_RBObjects[i].rB.GetLevelSet())->DrawOctLevelSet(&m_CollisionGraphics, 10);
	}*/
}

void CRigidBodyWorld::DestroyRigidBodyWorld() 
{
	Assert(m_RBObjects.GetCapacity(), "CRigidBodyWorld::DestroyRigidBodyWorld "
		"cannot destroy when the world was never created.");

	UINT s = m_RBObjects.GetSize();
	for (UINT i=0; i<s; i++) {
		m_pRBObject[i].rB.DestroyRigidBody();
	}
	m_RBObjects.Clear();
	m_RBContactPoints.Clear();
}


};
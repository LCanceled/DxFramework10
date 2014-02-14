
#include "DxUtRigidBodyWorld.h"
#include "DxUtTriangle.h"
#include "DxUtTimer.h"
#include "DxUtD3DApp.h"

namespace DxUt {

CRigidBodyWorld::CRigidBodyWorld():m_pRBObject(0)
{
}

void CRigidBodyWorld::CreateRigidBodyWorld(DWORD nHintRBs, DWORD nHintContacts, bool bUseHierarchicalLevelSet, Vector3F * gravity, float fStepSize, float fMaxVel)
{
	Assert(!m_rgRBObject.GetSize(), "CRigidBodyWorld::CreateRigidBodyWorld must destroy the world before creating a new one.");
	Assert(nHintRBs > 0 && nHintContacts > 0, "CRigidBodyWorld::CreateRigidBodyWorld must enter nonzero values for the hints.");

	m_rgRBObject.Reserve(nHintRBs);
	m_rgRBContactPoint.Reserve(nHintContacts);
	
	m_CollisionGraphics.CreateGraphics();

	m_Gravity = *gravity;
	m_fTimeStepSize = fStepSize;
	m_fMaxVelocity = fMaxVel;
	m_bUseHierarchicalLevelSet = bUseHierarchicalLevelSet;
}

DWORD CRigidBodyWorld::AddRigidBody(CMesh * pMesh, DWORD dwStride, float fScale, float fMass, Vector3F & pos,
	Matrix4x4F & rot, Vector3F & linVel, Vector3F & angVel, float fElasticity, float fMu, Vector3F & force,
	Vector3F & torque, char * szLevelSet, DWORD dwTriPerOct, CRigidBody::GeometryType type, SMaterial * pOverrideMaterial)
{
	SRBObject rbo;
	rbo.bUsed = 0;
	rbo.bActive = 1;
	rbo.bDenter = 0;
	CRigidBody & r = rbo.rB;
	r.CreateRigidBody(pMesh, dwStride, fScale, fMass, m_Gravity, m_fTimeStepSize, m_fMaxVelocity, szLevelSet, dwTriPerOct, m_bUseHierarchicalLevelSet, type, pOverrideMaterial);
	r.m_Pos = pos;
	r.m_Rot = rot;
	r.m_LinVel = linVel;
	r.m_AngVel = angVel;
	r.m_fElasticity = fElasticity;
	r.m_fMu = fMu;
	r.m_Force = force;
	r.m_Torque = torque;
	rbo.bConstrainCM = 0;

	m_rgRBObject.PushBack(rbo);
	m_pRBObject = m_rgRBObject.GetData();

	m_RBType = type;

	r.GetLevelSet()->AssignCG(&m_CollisionGraphics);

	return m_rgRBObject.GetSize()-1;
}

/* Constrains the center of mass to lie at a certain position */
void CRigidBodyWorld::AddCenterOfMassPositionConstraint(DWORD dwRigidBody, Vector3F & pos)
{
	m_rgRBObject[dwRigidBody].bConstrainCM = 1;
	m_rgRBObject[dwRigidBody].constraintCMPos = pos;
}

void CRigidBodyWorld::DisableRigidBody(DWORD dwId) 
{
	Assert(dwId < m_rgRBObject.GetSize(), "CRigidBodyWorld::DisableRigidBody "
		"an id was specifed which does not exist for any body.");

	m_pRBObject[dwId].bActive = 0;
}

void CRigidBodyWorld::EnableRigidBody(DWORD dwId) 
{
	Assert(dwId < m_rgRBObject.GetSize(), "CRigidBodyWorld::EnableRigidBody "
		"an id was specifed which does not exist for any body.");

	m_pRBObject[dwId].bActive = 1;
}

static int stillGo = 1;

DWORD CRigidBodyWorld::FindContacts()
{
	/*static bool once = 0; 
	if (!once) {
		for (DWORD i=0; i<m_rgRBObject.GetSize(); i++)
			m_Grid.AddObject(&m_rgRBObject[i], box);

	}*/

	DWORD nTotalContacts = 0;
	DWORD s = m_rgRBObject.GetSize();
	m_rgRBContactPoint.Resize(0);
	m_rgContactPoint.Resize(0);
	for (DWORD i=0, end=m_rgContactLayer.GetSize(); i<end; i++) 
		m_rgContactLayer.Resize(0);
	for (DWORD i=0, end=m_rgRBObject.GetSize(); i<end; i++)
		m_rgRBObject[i].rgRBCP.Resize(0);

	/* Transform all the bodies */
	for (DWORD i=0; i<s; i++)
		m_pRBObject[i].rB.TransformLevelSet();
	
	DWORD dwContact = 0;
	DWORD dwPairs = 0;
	for (DWORD i=0; i<s; i++) {/* FIX FOR NONACTIVE RIGID BODIES */
		for (DWORD j=i+1, end=min(s, j+1+35); j<s; j++) {
			if (m_pRBObject[i].rB.IsStatic() && m_pRBObject[j].rB.IsStatic())
				continue;

			DWORD						nContactsij = m_pRBObject[i].rB.DetermineCollisionLevelSet(
				&m_pRBObject[j].rB, &m_rgContactPoint);

			Vector3F nor(0, 0, 0);
			nTotalContacts += nContactsij;
			if (nContactsij) {
				m_rgRBContactPoint.Resize(m_rgRBContactPoint.GetSize() + nContactsij);
				SRBContact * rgRBCP = m_rgRBContactPoint.GetData() + m_rgRBContactPoint.GetSize() - nContactsij;
				SContactPoint * rgCP = m_rgContactPoint.GetData() + m_rgContactPoint.GetSize() - nContactsij;
				for (DWORD k=0; k<nContactsij; k++) {
					rgRBCP[k].iPos = rgCP[k].iPos;
					rgRBCP[k].iNor = rgCP[k].iNor;
					rgRBCP[k].dist = rgCP[k].dist;
					rgRBCP[k].rBOk = &m_pRBObject[i];
					rgRBCP[k].rBOl = &m_pRBObject[j];
					rgRBCP[k].dwIdx = dwContact++;
				}
			}
		}
	}
					
	SRBContact * rgRBCP = m_rgRBContactPoint.GetData();
	for (DWORD i=0, end=m_rgRBContactPoint.GetSize(); i<end; i++) {
		rgRBCP[i].rBOk->rgRBCP.PushBack(&rgRBCP[i]);
		rgRBCP[i].rBOl->rgRBCP.PushBack(&rgRBCP[i]);
	}

	return nTotalContacts;
}

bool noGo = 0;

void CRigidBodyWorld::ComputeContactGraph()
{
	CArray<SRBObject*> rgObjQueue;
	rgObjQueue.Reserve(m_rgRBObject.GetSize());
	/* Push the static objects first onto the queue  */
	DWORD dwInfinity = 100000;
	SRBObject * pRBObjects = m_rgRBObject.GetData();
	for (DWORD i=0, end=m_rgRBObject.GetSize(); i<end; i++) {
		pRBObjects[i].iHeight = dwInfinity;
		pRBObjects[i].bUsed = 0;

		if (pRBObjects[i].rB.IsStatic()) {
			rgObjQueue.PushBack();
			rgObjQueue.GetBack() = &m_rgRBObject[i];
			pRBObjects[i].iHeight = 0;
			pRBObjects[i].bUsed = 1;
		}
	}

	DWORD dwCounter = 0, nLayers = 0;
	while (rgObjQueue.GetSize() > dwCounter) {
		SRBObject * pObjQueue = rgObjQueue[dwCounter];

		for (DWORD i=0, nCPs=pObjQueue->rgRBCP.GetSize(); i<nCPs; i++) {
			SRBContact ** pCPs = pObjQueue->rgRBCP.GetData();
			SRBObject * pNeighborObj;
			if (pCPs[i]->rBOk != pObjQueue) pNeighborObj = pCPs[i]->rBOk;
			else pNeighborObj = pCPs[i]->rBOl;

			if (!pNeighborObj->bUsed) {
				rgObjQueue.PushBack(pNeighborObj);
				pNeighborObj->bUsed = 1;
			}
			pNeighborObj->iHeight = min(pNeighborObj->iHeight, pObjQueue->iHeight + 1);
			if (pNeighborObj->iHeight == pObjQueue->iHeight && pNeighborObj->iHeight != 0)
				pCPs[i]->dwLayer = pNeighborObj->iHeight - 1;
			else pCPs[i]->dwLayer = min(pNeighborObj->iHeight, pObjQueue->iHeight);

			nLayers = max(nLayers, pCPs[i]->dwLayer);
		}

		dwCounter++;
	}
	rgObjQueue.Clear();

	/* All bodies not touching a static body */
	for (DWORD i=0, end=m_rgRBObject.GetSize(); i<end; i++) { 
		if (!pRBObjects[i].bUsed) {
			for (DWORD j=0, nCPs=pRBObjects[i].rgRBCP.GetSize(); j<nCPs; j++) {
				//SRBContact ** pRBCPs = pRBObjects[i].rgRBCP.GetData();
				//pRBCPs[j]->dwLayer = nLayers+1;
				pRBObjects[i].rgRBCP[j]->dwLayer = nLayers+1;
			}
		}
	}

	m_rgContactLayer.Resize(nLayers+2);
	SContactLayer * pContactLayers = m_rgContactLayer.GetData();
	SRBContact * pRBContactPoints = m_rgRBContactPoint.GetData();
	for (DWORD i=0, end=m_rgContactLayer.GetSize(); i<end; i++) {
		pContactLayers[i].rgRBCP.Reserve(m_rgRBContactPoint.GetSize());
		pContactLayers[i].rgRBCP.Resize(0);
		
		pContactLayers[i].rgRBObj.Reserve(m_rgRBContactPoint.GetSize());
		pContactLayers[i].rgRBObj.Resize(0);
	}
	/* Add the contact points */
	for (DWORD i=0, end=m_rgRBContactPoint.GetSize(); i<end; i++) {
		DWORD idx = pRBContactPoints[i].dwLayer;
		pContactLayers[idx].rgRBCP.PushBack(m_rgRBContactPoint[i]);
	}
	/* Add the rigid body objects */
	bool cool=0;
	for (DWORD i=0, end=m_rgRBObject.GetSize(); i<end; i++) {
		if (pRBObjects[i].iHeight == dwInfinity) 
			cool = 1;
	}
	if (!cool)
		int asdf=0;
	for (DWORD i=0, end=m_rgRBObject.GetSize(); i<end; i++) {
		if (pRBObjects[i].iHeight == dwInfinity) 
			continue;

		bool bUpper = 0;
		bool bLower = 0;
		for (DWORD j=0, end2=pRBObjects[i].rgRBCP.GetSize(); j<end2; j++) {
			SRBContact ** pCPs =  pRBObjects[i].rgRBCP.GetData();
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
			m_rgContactLayer[pRBObjects[i].iHeight].rgRBObj.PushBack(&pRBObjects[i]);
		//if (bLower)
		//	m_rgContactLayer[pRBObjects[i].iHeight-1].rgRBObj.PushBack(&pRBObjects[i]);
	}
}

void CRigidBodyWorld::PGSSetup(DWORD nContacts)
{
	for (DWORD k=0, end=m_rgContactLayer.GetSize(); k<end; k++) {
		SRBContact * rgCP = m_rgContactLayer[k].rgRBCP.GetData();
		for (DWORD j=0, end2=m_rgContactLayer[k].rgRBCP.GetSize(); j<end2; j++) {
		//SRBContact * rgCP = m_rgRBContactPoint.GetData();
		//for (DWORD j=0, end2=m_rgRBContactPoint.GetSize(); j<end2; j++) {
			SRBContact & rbC = rgCP[j];
			DWORD idx = rgCP[j].dwIdx;
			Vector3F & nor = rbC.iNor;
			Vector3F t1(Vector3F(rbC.rBOl->rB.m_LinVel - rbC.rBOk->rB.m_LinVel + Vector3F(.1f)).Normalize());
			Vector3F & t2 = CrossXYZ(t1, nor).Normalize();
			t1 = CrossXYZ(t2, nor).Normalize();

			rbC.t1 = t1;
			rbC.t2 = t2;

			rbC.rBOk->rB.ComputeFrictionlessImpulsePart(rbC.rBOl->rB, rbC.iPos, nor,
				rbC.fEffectiveMass[0], rbC.JNA[0], rbC.JWA[0], rbC.JNB[0], rbC.JWB[0]);

			rbC.rBOk->rB.ComputeFrictionImpulsePart(rbC.rBOl->rB, rbC.iPos, t1,
				rbC.fEffectiveMass[1], rbC.JNA[1], rbC.JWA[1], rbC.JNB[1], rbC.JWB[1]);

			rbC.rBOk->rB.ComputeFrictionImpulsePart(rbC.rBOl->rB, rbC.iPos, t2,
				rbC.fEffectiveMass[2], rbC.JNA[2], rbC.JWA[2], rbC.JNB[2], rbC.JWB[2]);

			/* Compute velocities for push out */
			rbC.pushOutVel = (DotXYZ(rbC.JNA[0], rbC.rBOk->rB.m_LinVel) + DotXYZ(rbC.JWA[0], rbC.rBOk->rB.m_AngVel)) +
				(DotXYZ(rbC.JNB[0], rbC.rBOl->rB.m_LinVel) + DotXYZ(rbC.JWB[0], rbC.rBOl->rB.m_AngVel));
		}
	}
}

#define COMPUTE_IMPULSE(index, impulse, pushOut) \
	fB = (DotXYZ(rbC.JNA[index], rbk.m_LinVel) + DotXYZ(rbC.JWA[index], rbk.m_AngVel)) + \
			(DotXYZ(rbC.JNB[index], rbl.m_LinVel) + DotXYZ(rbC.JWB[index], rbl.m_AngVel)); \
	impulse =  rbC.fEffectiveMass[index]*(fB - pushOut);

void CRigidBodyWorld::SolveLayer(DWORD k, VectorNF & lambda, float dt)
{
	SRBContact * rgCP = m_rgContactLayer[k].rgRBCP.GetData();
	for (DWORD j=0, end2=m_rgContactLayer[k].rgRBCP.GetSize(); j<end2; j++) {
	//SRBContact * rgCP = m_rgRBContactPoint.GetData();
	//for (DWORD j=0, end2=m_rgRBContactPoint.GetSize(); j<end2; j++) {
		SRBContact & rbC = rgCP[j];
		DWORD idx = rgCP[j].dwIdx;
		CRigidBody & rbk = rbC.rBOk->rB;
		CRigidBody & rbl = rbC.rBOl->rB;
		float mu = 0.1;//1; // Friction is WRONG! box platform

		float fB, fImpulseN, fImpulseT1=0, fImpulseT2=0;
		float fPushOut = .5f*(rbC.pushOutVel*dt + rbC.dist)/dt;
		if (!m_bUsePushOut || rbC.dist > -.05f) fPushOut = 0;
		COMPUTE_IMPULSE(0, fImpulseN, fPushOut);
		//g_D3DApp->Print(fB + fPushOut/dt);
		COMPUTE_IMPULSE(1, fImpulseT1, 0);
		COMPUTE_IMPULSE(2, fImpulseT2, 0);

		float lambdaUpdateN = lambda.c[3*idx+0] + fImpulseN;
		float lambdaUpdateT1= lambda.c[3*idx+1] + fImpulseT1;
		float lambdaUpdateT2= lambda.c[3*idx+2] + fImpulseT2;
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
		float difN = lambdaUpdateN -  lambda.c[3*idx+0];
		float difT1= lambdaUpdateT1-  lambda.c[3*idx+1];
		float difT2= lambdaUpdateT2-  lambda.c[3*idx+2];

		Vector3F impulse = difN*rbC.iNor + difT1*rbC.t1 + difT2*rbC.t2;
		rbC.rBOk->rB.ApplyImpulse(impulse, rbC.iPos);
		rbC.rBOl->rB.ApplyImpulse(-impulse, rbC.iPos);

		lambda.c[3*idx+0] = lambdaUpdateN;
		lambda.c[3*idx+1] = lambdaUpdateT1;
		lambda.c[3*idx+2] = lambdaUpdateT2;
	}
}

//#define PRINT
CTimer g_Timer(1000);

void CRigidBodyWorld::PGSSolve(DWORD nContacts, float dt, bool bNeedSetup)
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

	DWORD nIterations = 20;
	for (DWORD i=0; i<nIterations; i++) {
		for (DWORD k=0, end=m_rgContactLayer.GetSize(); k<end; k++)
			SolveLayer(k, lambda, dt);

		/* Position constraints */
		for (DWORD k=0, end=m_rgRBObject.GetSize(); k<end; k++) {
			ApplyCMConstraint(m_rgRBObject[k], dt);
		}
	}

	/* Shock propagation */
	lambda.Set(0);
	for (DWORD i=0; i<nIterations/10 + 1; i++) {
		for (DWORD k=0, end=m_rgContactLayer.GetSize(); k<end; k++) {
			SolveLayer(k, lambda, dt);

			DWORD level = min(k+1, end-1);
			SRBObject ** pRBObjects = m_rgContactLayer[level].rgRBObj.GetData();
			for (DWORD j=0, end2=m_rgContactLayer[level].rgRBObj.GetSize(); j<end2; j++) {
				pRBObjects[j]->rB.SetStatic(1);
			}
		}

		// Reset everything 
		SRBObject * pRBObjects = m_rgRBObject.GetData();
		for (DWORD i=0, end=m_rgRBObject.GetSize(); i<end; i++) { 
			pRBObjects[i].rB.SetStatic(0);
		}
		
		/* Position constraints */
		for (DWORD k=0, end=m_rgRBObject.GetSize(); k<end; k++) {
			ApplyCMConstraint(m_rgRBObject[k], dt);
		}
	}

	lambda.DestroyVector();
	//rgFrictionImpulses.Clear();
}

void CRigidBodyWorld::UpdateRigidBodies(FLOAT dt, Vector3F & gAcel)
{
	if (noGo) return;
	float fMinThickness = .15f;

	DWORD s = m_rgRBObject.GetSize();
	float dtSubStep = 0, dtTotalStep = 0;
	//g_D3DApp->Print("Step");
	//while (dtTotalStep < dt) {
		/* Calculate sub step restriction */
		float fMaxVel = 1e-4;
		for (DWORD i=0; i<s; i++) {
			if (!m_pRBObject[i].bActive) continue;

			m_pRBObject[i].rB.SavePosAndRot();
			Vector3F & angVel = m_pRBObject[i].rB.GetAngVel();
			float fMaxRadius = sqrtf(m_pRBObject[i].rB.MaxRadiusSq());
			float fVel = m_pRBObject[i].rB.GetLinVel().Length() + angVel.Length()*fMaxRadius;
			if (fMaxVel < fVel) fMaxVel = fVel;
		}
		float fRestriction = .5f*fMinThickness/fMaxVel;
		if (dtTotalStep + fRestriction > dt) dtSubStep = (dt - dtTotalStep) + 1e-4;
		else dtSubStep += fRestriction;
		dtTotalStep += dtSubStep;
		dtTotalStep = dt + 1e-4;
		dtSubStep = dt;
		//g_D3DApp->Print(dtSubStep);

		for (DWORD i=0; i<s; i++) {
			if (!m_pRBObject[i].bActive) continue;

			m_pRBObject[i].rB.SavePosAndRot();
			m_pRBObject[i].rB.SaveLinAndAngVel();
			m_pRBObject[i].rB.IntegratePos(dtSubStep);
		}

	#ifdef PRINT
		g_Timer.StartTimer(0);
		g_Timer.StartTimer(1);
	#endif
		//timer.StartTimer(0);
		DWORD nContacts = FindContacts();
		//timer.EndTimer(0, "End finding intersection");
		if (nContacts) {
			//noGo = 1;
			//return;
	#ifdef PRINT
			g_Timer.EndTimer(1);
			g_Timer.DisplayTimer(1, "End FindContacts True");

			g_Timer.StartTimer(2);
			g_D3DApp->Print(nContacts);
	#endif
			m_bUsePushOut = 1;
			PGSSolve(nContacts, dtSubStep, 1);
			//timer.EndTimer(1, "End solving");
	#ifdef PRINT
			g_Timer.EndTimer(2);
			g_Timer.DisplayTimer(2, "End PGSSolve");
			g_Timer.PresentTimer(1);
	#endif
			if (noGo) return;
		} else {
	#ifdef PRINT
			g_Timer.EndTimer(0);
			g_Timer.DisplayTimer(0, "End FindContacts False");
			g_Timer.PresentTimer(1);
	#endif
		}

		for (DWORD i=0; i<s; i++) {
			if (!m_pRBObject[i].bActive) continue;

			m_pRBObject[i].rB.RestorePosAndRot();
			m_pRBObject[i].rB.IntegratePos(dtSubStep);
			m_pRBObject[i].rB.RestoreLinAndAngVel();
			if (noGo) {
				m_pRBObject[i].rB.IntegrateVel(dtSubStep, Vector3F(0,0,0)); 
				m_pRBObject[i].rB.m_LinVel = Vector3F(0,0,m_pRBObject[i].rB.m_LinVel.z);
			}
			else {
				m_pRBObject[i].rB.IntegrateVel(dtSubStep, gAcel); 
				if (m_pRBObject[i].rB.m_LinVel.Length() > m_fMaxVelocity) {
					m_pRBObject[i].rB.m_LinVel = m_fMaxVelocity*m_pRBObject[i].rB.m_LinVel.Normalize();
					// Need to account for angular as well
				}
			}
			m_pRBObject[i].bUsed = 0; 
		}

		/* Post stab stuff*/
		if (nContacts) {
			m_bUsePushOut = 0;
			m_pRBObject[1];
			PGSSolve(nContacts, dtSubStep, 0);
		}

		
		static int once = 1;
		if (once) {
			m_rgRBObject[1].bDenter = 1;
			m_rgRBObject[2].bDenter = 1;
			m_rgRBObject[3].bDenter = 1;
			m_rgRBObject[4].bDenter = 1;
			once = 0;
			//Dent(Vector3F(-.1f, 1.f, 0));
			//Dent(Vector3F(.1f, 1.f, 0));

		}

		// Denting
		if (nContacts && m_rgRBObject[0].rgRBCP.GetSize()) {
			m_rgRBObject[0].rgRBCP[0]->rBOk;
			for (int i=0; i<m_rgRBObject[0].rgRBCP.GetSize(); i++) {
				float v = m_rgRBObject[1].rB.GetLinVel().Length();
				if (m_rgRBObject[0].rgRBCP[i]->rBOl->bDenter) {// == &m_rgRBObject[1] && m_rgRBObject[1].bDenter) {
					Dent(m_rgRBObject[0].rgRBCP[i]->iPos);
					m_rgRBObject[0].rgRBCP[i]->rBOl->bDenter = 0;
				}
			}
		}
		
	//}
}

void CRigidBodyWorld::Dent(Vector3F & cpt)
{
	float sigma = 15.f;
	float A = .2f;

	ID3DX10MeshBuffer * vBuf;
	ID3DX10MeshBuffer * iBuf;
	ID3DX10Mesh * mesh = m_rgRBObject[0].rB.m_Mesh->GetMesh();
	mesh->GetVertexBuffer(0, &vBuf);
	mesh->GetIndexBuffer(&iBuf);
	SIZE_T sizeV = vBuf->GetSize();
	SIZE_T sizeI = iBuf->GetSize();
	SVertexPNT * dataV;
	DWORD * dataI;
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

	CLevelSet * level = m_rgRBObject[0].rB.GetLevelSet();

	for (int i=0; i<level->m_nLSVertsZ; i++) {
	//for (int i=0; i<0; i++) {
		for (int j=0; j<level->m_nLSVertsY; j++) {
			for (int k=0; k<level->m_nLSVertsX; k++) {
				Vector3F pos(k*level->m_fCellSize, j*level->m_fCellSize, i*level->m_fCellSize);
				pos += level->m_TransformObjToGrid;
				pos = level->m_TransformObjToWorld*pos;
				float & lDist = level->m_rgLSVertex[i*level->m_nLSVertsY*level->m_nLSVertsX + j*level->m_nLSVertsX + k].dist;
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
	for (DWORD i=0; i<m_rgRBObject.GetSize(); i++)
		m_pRBObject[i].rB.TransformLevelSet();

	m_CollisionGraphics.SetCamera(pCam);
	m_CollisionGraphics.DrawContactPoints((CArray<SContactPoint>*)&m_rgContactPoint);
	for (DWORD i=0, end=1; i<end; i++) {
		//((COctreeLevelSet*)m_rgRBObject[i].rB.GetLevelSet())->AssignCG(&m_CollisionGraphics);
		//m_rgRBObject.GetSize(); i<end; i++) {
		//m_rgRBObject[i].rB.GetLevelSet()->DrawLevelset(&m_CollisionGraphics);
		//m_rgRBObject[i].rB.GetLevelSet().DrawAdmissibleZones(&m_CollisionGraphics);
		
		//m_rgRBObject[i].rB.GetLevelSet().DrawBVTree(&m_CollisionGraphics, 8);
		//((COctreeLevelSet*)m_rgRBObject[i].rB.GetLevelSet())->DrawOctLevelSet(&m_CollisionGraphics, 10);
	}
}

void CRigidBodyWorld::DestroyRigidBodyWorld() 
{
	Assert(m_rgRBObject.GetCapacity(), "CRigidBodyWorld::DestroyRigidBodyWorld "
		"cannot destroy when the world was never created.");

	DWORD s = m_rgRBObject.GetSize();
	for (DWORD i=0; i<s; i++) {
		m_pRBObject[i].rB.DestroyRigidBody();
	}
	m_rgRBObject.Clear();
	m_rgRBContactPoint.Clear();
	m_CollisionGraphics.DestroyGraphics();
}


};









/*
void CRigidBodyWorld::SolveConstraints(DWORD nContacts)
{
	DWORD K = nContacts;

	MatrixNxNF A;
	A.CreateMatrix(K);
	VectorNF b;
	b.CreateVector(K);

	SRBContact * rgCP = m_rgRBContactPoint.GetData();
	for (DWORD j=0; j<K; j++) {
		SRBObject * rBOk = rgCP[j].rBOk;
		SRBObject * rBOl = rgCP[j].rBOl;
		Vector3F jthCtPoint(rgCP[j].iPos);
		Vector3F jthCtNormal(rgCP[j].iNor);

		b.c[j] = -rBOk->rB.GetVelocityAtContactPoint(jthCtPoint, jthCtNormal) +
			rBOl->rB.GetVelocityAtContactPoint(jthCtPoint, jthCtNormal);

		for (DWORD i=0, nCt=0, row=j*K; i<K; i++) {
			A.c[row+i] = 0;
			Vector3F sij(0,0,0);
			if (rgCP[i].rBOk == rBOk) {
				sij = rBOk->rB.GetImpulseCoefficient(jthCtPoint, rgCP[i].iPos, rgCP[i].iNor);

				if (rgCP[i].rBOl == rBOl)
					sij -= rBOl->rB.GetImpulseCoefficient(jthCtPoint, rgCP[i].iPos, -rgCP[i].iNor);
			}
			else if (rgCP[i].rBOk == rBOl) {
				sij = -rBOl->rB.GetImpulseCoefficient(jthCtPoint, rgCP[i].iPos, rgCP[i].iNor);

				if (rgCP[i].rBOl == rBOk)
					sij += rBOk->rB.GetImpulseCoefficient(jthCtPoint, rgCP[i].iPos, -rgCP[i].iNor);
			}
			else if (rgCP[i].rBOl == rBOk) {/* && rgCP[i].rBOk != rBOl
				sij = rBOk->rB.GetImpulseCoefficient(jthCtPoint, rgCP[i].iPos, -rgCP[i].iNor);
			}
			else if (rgCP[i].rBOl == rBOl) {/* && rgCP[i].rBOk != rBOk
				sij = -rBOl->rB.GetImpulseCoefficient(jthCtPoint, rgCP[i].iPos, -rgCP[i].iNor);
			}
			else {
				//DebugBreak();	
				int a=0;
			}

			A.c[row+i] = DotXYZ(sij, jthCtNormal);
		}
	}

	VectorNF x; x.CreateVector(K);
	memset(x.c, 0, sizeof(float)*K);
	ProjectedGaussSeidelMethod(A, x, b, 0.0001f, 500);

	SRBContact * rgCPs = m_rgRBContactPoint.GetData();
	for (DWORD t=0; t<K; t++) {
		if (x.c[t] > 1000.f)
			break;

		Vector3F nt(rgCPs[t].iNor);
		rgCPs[t].rBOk->rB.ApplyImpulse(x.c[t]*(-nt), rgCPs[t].iPos);
		rgCPs[t].rBOl->rB.ApplyImpulse(x.c[t]*(nt), rgCPs[t].iPos);
	}

	A.DestroyMatrix();
	b.DestroyVector();
	x.DestroyVector();
}

*/

/*

DWORD						nContactsij = m_pRBObject[i].rB.DetermineCollision(
				&m_pRBObject[j].rB, (std::vector<SContactPoint>*)&m_pRBObject[i].rgCP);
			SRBContact *				rgCPi=NULL;// = &m_pRBObject[i].rgCP.back();
			if (m_pRBObject[i].rgCP.GetSize()) rgCPi = m_pRBObject[i].rgCP.GetData();
			std::vector<SRBContact> &	rgCPj = m_pRBObject[j].rgCP;

			for (SRBContact * pCPi=rgCPi, * pEnd=rgCPi+nContactsij; pCPi<pEnd; pCPi++, nTotalContacts++) {
				/* body i
				pCPi->dwContactPointIndex = nTotalContacts;
				pCPi->rBOl = &m_pRBObject[j];

				/* body j
				SRBContact c = {nTotalContacts, 
					&m_pRBObject[i], pCPi->iPos, -pCPi->iNor};
				rgCPj.PushBack(c);

				/* all contact points 
				SRBContact cc = {0, &m_pRBObject[j], rgCPi->iPos, rgCPi->iNor};
				cc.rBOk =  &m_pRBObject[i];
				m_rgRBContactPoint.PushBack(cc);
			}
		}
	}
	return nTotalContacts;
}

void CRigidBodyWorld::SolveConstraints(DWORD nContacts)
{
	DWORD K = nContacts;

	MatrixNxNF A;
	A.CreateMatrix(K);
	VectorNF b;
	b.CreateVector(K);

	SRBObject * rgObj = m_rgRBObject.GetData();
	for (DWORD k=0, nObj=m_rgRBObject.GetSize(), t=0; k<nObj; k++) {

		SRBObject &		rBOk = rgObj[k];
		SRBContact *	rgCPk = rBOk.rgCP.GetData();
		DWORD			nCPsk = rBOk.rgCP.GetSize();

		for (DWORD j=0; j<nCPsk; j++, t++) {

			SRBObject &		rBOl = *rgCPk->rBOl;
			SRBContact *	rgCPl = rBOl.rgCP.GetData();
			Vector3F		jthCtPoint(rgCPk[j].iPos);
			Vector3F		jthCtNormal(rgCPk[j].iNor);

			b.c[t] = rBOk.rB.GetVelocityAtContactPoint(jthCtPoint, jthCtNormal) -
				rBOl.rB.GetVelocityAtContactPoint(jthCtPoint, -jthCtNormal);

			for (DWORD i=0, nCtk=0, nCtl=0, row=t*K; i<K; i++) {
				A.c[row+i] = 0;
				if (((DWORD*)&rgCPk[nCtk])[0] == i) {
					Vector3F s(rBOk.rB.GetImpulseCoefficient(jthCtPoint, rgCPk[nCtk].iPos, rgCPk[nCtk].iNor));nCtk++;

					A.c[row+i] = DotXYZ(s, jthCtNormal);
				}
				if (((DWORD*)&rgCPl[nCtl])[0] == i) {
					Vector3F s(rBOl.rB.GetImpulseCoefficient(jthCtPoint, rgCPl[nCtl].iPos, rgCPl[nCtl].iNor));nCtl++;

					A.c[row+i] -= DotXYZ(s, jthCtNormal);
				}
			}
		}
	}

	VectorNF x; x.CreateVector(K);
	memset(x.c, 0, sizeof(float)*K);
	ProjectedGaussSeidelMethod(A, x, b, 0.001f, 90);

	SRBContact * rgCPs = m_rgRBContactPoint.GetData();
	for (DWORD t=0; t<K; t++) {
		/* If too large of impulse, discard. 
		if (x.c[t] > 1000.f)
			break;

		Vector3F nt(rgCPs[t].iNor);
		rgCPs[t].rBOk->rB.ApplyImpulse(x.c[t]*(nt), rgCPs[t].iPos);
		rgCPs[t].rBOl->rB.ApplyImpulse(x.c[t]*(-nt), rgCPs[t].iPos);
	}

	A.DestroyMatrix();
	b.DestroyVector();
	x.DestroyVector();
}
*/




	/*


			Alk.MZero();
			Vector3F & iPosL = c[l].iPos;
			Vector3F & iPosK = c[k].iPos; 

			if (c[l].i == c[k].i) 
				Alk += c[l].rBi->GetPart1(iPosL, c[k].rBi->GetPart2(iPosK));
			if (c[l].i == c[k].j) 
				Alk -= c[l].rBi->GetPart1(iPosL, c[k].rBj->GetPart2(iPosK));
			if (c[l].j == c[k].i)
				Alk -= c[l].rBj->GetPart1(iPosL, c[k].rBi->GetPart2(iPosK));
			if (c[l].j == c[k].j) 
				Alk += c[l].rBj->GetPart1(iPosL, c[k].rBj->GetPart2(iPosK));

			Vector3F v((c[l].iNor*Alk));
			A.c[col + k] = DotXYZ(v, c[k].iNor);
		}
	}

	VectorNF b; 
	b.CreateVector(K); 
	for (DWORD k=0; k<K; k++) {
		Vector3F v(c[k].rBi->GetPart3(gAcel, c[k].iPos, dt) - c[k].rBj->GetPart3(gAcel, c[k].iPos, dt));
		b.c[k] = DotXYZ(c[k].iNor, v);
	}

	VectorNF x, xL, xH; x.CreateVector(K);
	GaussSeidelMethod(A, x, b, xL, xH, 0, 90);

	float dif = x.c[0] - x.c[1];
	for (DWORD k=0; k<K; k++) {
		if (x.c[k] > 1000.f)
			break;

		Vector3F nk(c[k].iNor);
		c[k].rBi->ApplyImpulse((x.c[k]*(-nk)), c[k].iPos, gAcel, dt);
		c[k].rBj->ApplyImpulse(x.c[k]*nk, c[k].iPos, gAcel, dt);
	}

	A.Destroy();
	b.Destroy();
	x.Destroy();*/




#include "DxUtContactRegions.h"

namespace DxUt {

void CContactRegions::ResetInteriorPolygons()
{
	SPolygon3FEx ** rgInteriorPolygons = m_pInteriorPolygon->GetData();
	for (DWORD i=0, end=m_pInteriorPolygon->GetSize(); i<end; i++) {
		rgInteriorPolygons[i]->bVisited = 0;
	}
}

bool CContactRegions::BisectEdgePolygon(SPolygon3FEx * pInnerPolygon, 
	SPolygon3FEx * pOuterPolygon, SLine3F * pVertexEdge, unsigned char & dwEdge, Vector3F & intersection)
{
	dwEdge = 0;
	while (pInnerPolygon->rgEdge[dwEdge].dwPoly != pOuterPolygon->dwPoly) dwEdge++;
	SLine3F & e = pInnerPolygon->rgEdge[dwEdge].edge;

	double t=0;
	if (!e.Intersect(*pVertexEdge, intersection, t)) return 0;

	return 1;
}

//bool CContactRegions::FormContactRegion(DWORD * rgNFaces, CArray<SContactPoint> * _rgCP, CArray<SPolygon3FEx> * _rgPolygon, Matrix4x4F * T)
bool CContactRegions::FormContactRegion(DWORD dwBody, DWORD nFaces, CArray<SPolygon3FEx> * _rgPolygon, Matrix4x4F & T)
{
	/* Offset the _rgPolygons in the reverse direction for 1 */ 
	if (dwBody == 1) {
		SContactRegionVertex * rgVert = m_rgContactRegionVertex[1].GetData();
		unsigned char cPreFaceFlag = 0; 
		for (DWORD i=1, end=m_rgContactRegionVertex[1].GetSize(); i<end; i++) {
			if (rgVert[i].rgFaceFlag[0] != rgVert[(i+1)%end].rgFaceFlag[0])
				cPreFaceFlag = !cPreFaceFlag;
		
			rgVert[i].rgFaceFlag[0] = cPreFaceFlag;
		} 
		rgVert[0].rgFaceFlag[0] = 0;
	}

	//OutputDebugStringA("Hello Start\n");
	dwCounter++;
	if (dwCounter == 1497)
		int a=0;

	m_pInteriorPolygon = &m_rgInteriorPolygon[dwBody];
	m_pCoplanarPolygon = &m_rgCoplanarPolygon[dwBody];

	m_pContactRegionVertex = &m_rgContactRegionVertex[dwBody];
	SContactRegionVertex * rgVertices = m_pContactRegionVertex->GetData();
	DWORD nVertices = m_pContactRegionVertex->GetSize();

	//m_rgCP = _rgCP; 
	m_pPolygon = m_rgPolygon[dwBody] = _rgPolygon;
	m_pTriTransform = &T;

	if (nFaces == 0) {

		/*Construct one coplanar polygon from rgVertex*/
		m_pCoplanarPolygon->PushBack();
		SCoplanarPolygon3F & p = m_pCoplanarPolygon->GetBack();

		p.bVisited = 0;
		p.dwPoly = 0;
		p.rgEdge.Reserve(nVertices);
		for (DWORD i=0; i<nVertices; i++) {
			SPolygonEdge edge;
			edge.edge(rgVertices[i].v, rgVertices[(i+1)%nVertices].v);
			edge.dwPoly = -1;
			p.rgEdge.PushBack(edge);
		}
		p.normal = rgVertices[0].pPolygon->normal;//p.ComputeNormal();
		p.avgNormal = p.normal;
		/*if (abs(p.avgNormal.x) < .1f && abs(p.avgNormal.y) < .1f && abs(p.avgNormal.z) < .1f) {
			int aasdf=0;
		}
		if (abs(p.avgNormal.y) > .71 || abs(p.avgNormal.y) < .70)
			int asdf=0;*/

		m_pContactRegionVertex->Resize(0);

		return 1;
	}
	
	/* The general case */
	/* Find the first cp where there is a change in the iLine vector  */
	DWORD dwFirstCP = 0;
	SPolygon3FEx * pPrePolygon = NULL;
	DWORD dwIniTriNormalFlags = rgVertices[dwFirstCP].rgFaceFlag[0];
	while (dwIniTriNormalFlags == rgVertices[dwFirstCP].rgFaceFlag[0]) {
		pPrePolygon = rgVertices[dwFirstCP].pPolygon; dwFirstCP++; }
	if (!pPrePolygon) {
		DebugBreak();
	}

	/* This is necessary in cases where nFaces is odd */
	if (ArePolygonsPlanar(rgVertices[0].pPolygon->normal, rgVertices[nVertices-1].pPolygon->normal)) {
		bool cIniFaceFlag = rgVertices[nVertices-1].rgFaceFlag[0] > 0;
		m_pContactRegionVertex->Reserve(dwFirstCP);
		for (DWORD i=0; i<dwFirstCP; i++) {
			(*m_pContactRegionVertex)[i].rgFaceFlag[0] = cIniFaceFlag;
			m_pContactRegionVertex->PushBack((*m_pContactRegionVertex)[i]);
		}
		m_pContactRegionVertex->PushBack((*m_pContactRegionVertex)[dwFirstCP]);
	} else {
		/* In this case, 0 is the first CP index */
		dwFirstCP = 0;
		pPrePolygon = rgVertices[nVertices-1].pPolygon;
		m_pContactRegionVertex->PushBack((*m_pContactRegionVertex)[0]);//nVertices may need to be expanded
	}
	rgVertices = m_pContactRegionVertex->GetData();
	nVertices = m_pContactRegionVertex->GetSize();

	/* Compute the weighted normal */
	m_pInteriorPolygon->PushBack(rgVertices[dwFirstCP].pPolygon);
	(*m_pInteriorPolygon)[0]->bVisited = 1;

	SPolygon3FEx * pCurPolygon = rgVertices[dwFirstCP].pPolygon;
	DWORD dwCurPolygon = rgVertices[dwFirstCP].pPolygon->dwPoly;

	Vector3F preVertex(rgVertices[dwFirstCP].v);
	DWORD dwPreFaceFlag = rgVertices[dwFirstCP].rgFaceFlag[0];

	/* Start the initial bisection */
	if (!BisectEdgePolygon(pCurPolygon, pPrePolygon, &SLine3F(rgVertices[dwFirstCP].v, rgVertices[dwFirstCP+1].v), 
		pCurPolygon->bisection[0].dwEdgeStart, pCurPolygon->bisection[0].intersectionStart)) {ResetAll(); return 0; }
	pPrePolygon = pCurPolygon;

	//OutputDebugStringA("Hello Bisection\n");

	int nPolygonEdges = 1;
	for (DWORD i=dwFirstCP+1; i<nVertices; i++) {
		pCurPolygon = rgVertices[i].pPolygon;

		/* If the new and old poygons are not coplanar, or if the new polygon is coplanar to the old one */
		if (rgVertices[i].rgFaceFlag[0] != dwPreFaceFlag || rgVertices[i].rgFaceFlag[1]) {
			if (pPrePolygon->nBisectors > 1) 
				DebugBreak();
			
			DWORD dwBisection = pPrePolygon->nBisectors;

			/* Push a boundry edge */
			SPolygonEdge edge;
			edge.edge(preVertex, rgVertices[i].v);
			edge.dwPoly = -nPolygonEdges++;
			pPrePolygon->rgBoundaryEdges.PushBack(edge);

			/* End the bisection */
			if (!BisectEdgePolygon(pPrePolygon, pCurPolygon, &edge.edge,
				pPrePolygon->bisection[dwBisection].dwEdgeEnd, pPrePolygon->bisection[dwBisection].intersectionEnd)) {ResetAll(); return 0; }
			
			//may need nBisectors > 0 condition
			if (pPrePolygon->bisection[0].dwEdgeStart == pPrePolygon->bisection[dwBisection].dwEdgeEnd) {
				pPrePolygon->rgBoundaryEdges.PushBack(SPolygonEdge(
					SLine3F( pPrePolygon->bisection[dwBisection].intersectionEnd, pPrePolygon->bisection[0].intersectionStart),
					pPrePolygon->rgEdge[pPrePolygon->bisection[0].dwEdgeStart]));
			} else if (dwBisection > 0) {
				SPolygonBisection * pBisection = &pPrePolygon->bisection[1];
				pPrePolygon->rgBoundaryEdges.PushBack(SPolygonEdge( 
					SLine3F(pBisection->intersectionEnd, pPrePolygon->rgEdge[pBisection->dwEdgeEnd].edge.e2),
					pPrePolygon->rgEdge[pBisection->dwEdgeEnd]));

				pBisection = &pPrePolygon->bisection[0];
				pPrePolygon->rgBoundaryEdges.PushBack(SPolygonEdge(
					SLine3F(pPrePolygon->rgEdge[pBisection->dwEdgeStart].edge.e1, pBisection->intersectionStart),
					pPrePolygon->rgEdge[pBisection->dwEdgeStart]));
			}
			pPrePolygon->nBisectors++;

			if (i == (nVertices-1)) break;//fix this (add more to nvertices)
			else if (!pCurPolygon->bVisited) {
				m_pInteriorPolygon->PushBack(rgVertices[i].pPolygon);
				pCurPolygon->bVisited = 1;
			}

			/* Start the next bisection */
			dwBisection = pCurPolygon->nBisectors;
			if (!BisectEdgePolygon(pCurPolygon, pPrePolygon, &edge.edge, 
				pCurPolygon->bisection[dwBisection].dwEdgeStart, pCurPolygon->bisection[dwBisection].intersectionStart)) {ResetAll(); return 0; }
			if (pCurPolygon->nBisectors > 0) {
				if (pCurPolygon->bisection[0].dwEdgeEnd == pCurPolygon->bisection[1].dwEdgeStart) {
					pCurPolygon->rgBoundaryEdges.PushBack(SPolygonEdge( 
						SLine3F(pCurPolygon->bisection[0].intersectionEnd, pCurPolygon->bisection[1].intersectionStart),
						pCurPolygon->rgEdge[pCurPolygon->bisection[0].dwEdgeEnd]));
				}
				else {
					SPolygonBisection * pBisection = &pCurPolygon->bisection[0];
					pCurPolygon->rgBoundaryEdges.PushBack(SPolygonEdge(
						SLine3F(pBisection->intersectionEnd, pCurPolygon->rgEdge[pBisection->dwEdgeEnd].edge.e2),
						pCurPolygon->rgEdge[pBisection->dwEdgeEnd]));

					pBisection = &pCurPolygon->bisection[1];
					pCurPolygon->rgBoundaryEdges.PushBack(SPolygonEdge(
						SLine3F(pCurPolygon->rgEdge[pBisection->dwEdgeStart].edge.e1, pBisection->intersectionStart),
						pCurPolygon->rgEdge[pBisection->dwEdgeStart]));
				}
			}

			dwPreFaceFlag = rgVertices[i].rgFaceFlag[0];
		}
		else {
			/* Push a boundry edge */
			SPolygonEdge edge;
			edge.edge(preVertex, rgVertices[i].v);
			edge.dwPoly = -nPolygonEdges++;
			pCurPolygon->rgBoundaryEdges.PushBack(edge);
		}
		preVertex = rgVertices[i].v;
		pPrePolygon = pCurPolygon;
	}
	(*m_pInteriorPolygon)[0]->bVisited = 1;

	//OutputDebugStringA("Hello Bisection End\n");

	/*m_rgCP->Resize(m_pContactRegionVertex->GetSize());
	SContactPoint * rgCP = m_rgCP->GetData();
	for (DWORD i=0; i<nVertices; i++)
		rgCP[i].iPos = rgVertices[i].v;*/

	for (DWORD i=0, end=m_pInteriorPolygon->GetSize(); i<end; i++) {
		SPolygon3FEx * p = (*m_pInteriorPolygon)[i];
		if (p->nBisectors == 1 && p->bisection[0].dwEdgeStart != p->bisection[0].dwEdgeEnd) {
		
			/* Add the edge p->bisection[0].dwEdgeStart */
			p->rgBoundaryEdges.PushBack(SPolygonEdge(
				SLine3F(p->bisection[0].intersectionEnd, p->rgEdge[p->bisection[0].dwEdgeEnd].edge.e2),
				p->rgEdge[p->bisection[0].dwEdgeEnd]) );
			
			/* Add the thrid edge */
			if ((p->bisection[0].dwEdgeEnd+2)%3 == p->bisection[0].dwEdgeStart) {
				DWORD dwEdgeThird = 3 - (p->bisection[0].dwEdgeStart + p->bisection[0].dwEdgeEnd);
				p->rgBoundaryEdges.PushBack(SPolygonEdge(
					SLine3F(p->rgEdge[dwEdgeThird].edge.e1, p->rgEdge[dwEdgeThird].edge.e2),
					p->rgEdge[dwEdgeThird]));
			}

			/* Add the edge p->bisection[0].dwEdgeEnd */
			p->rgBoundaryEdges.PushBack(SPolygonEdge(
				SLine3F(p->rgEdge[p->bisection[0].dwEdgeStart].edge.e1, p->bisection[0].intersectionStart),
				p->rgEdge[p->bisection[0].dwEdgeStart]));
		}
	}

	m_pContactRegionVertex->Resize(0);

	if (dwCounter == 19)
		int a=0;

	m_pCoplanarPolygon->Reserve(2*nFaces);//Not enough faces
	ComputeCoplanarPolygons();
	//ComputeNormals();

	//ResetAll();

	//OutputDebugStringA("Hello End\n");
	return 1;
}

bool CContactRegions::FormCoplanarPolygons(SPolygon3FEx * pPolygon, int iOuterBoundaryEdgeValue, int iIniBoundaryEdgeValue)
{
	if (pPolygon->dwPoly == 19)
		int a=0;

	pPolygon->bVisited = 1;
	pPolygon->dwCoplanarPolygon = m_pCoplanarPolygon->GetSize()-1;//value meaning dwPoly

	int iInnerBoundaryEdge = 0;
	SPolygonEdge * rgBoundaryEdge = pPolygon->rgBoundaryEdges.GetData();
	while (rgBoundaryEdge[iInnerBoundaryEdge].dwPoly != iOuterBoundaryEdgeValue) iInnerBoundaryEdge++;
	
	SCoplanarPolygon3F * pBackCPolygon = &m_pCoplanarPolygon->GetBack();
	for (DWORD i=iInnerBoundaryEdge+1, size=pPolygon->rgBoundaryEdges.GetSize(), end=2*size+1; ; i++) {
		if (rgBoundaryEdge[i%size].dwPoly == iIniBoundaryEdgeValue) return 0;

		if (i > end) 
			DebugBreak();

		if (rgBoundaryEdge[i%size].dwPoly < 0 || !rgBoundaryEdge[i%size].bCoplanar)
			pBackCPolygon->rgEdge.PushBack(rgBoundaryEdge[i%size]);
		else {
			FormCoplanarPolygons(&(*m_pPolygon)[rgBoundaryEdge[i%size].dwPoly], pPolygon->dwPoly, iIniBoundaryEdgeValue);
			break;
		}
	}

	return 1;
}

void CContactRegions::FormInteriorPolygons(SPolygon3FEx * pPolygon)
{
	if (pPolygon->bVisited) return;
	pPolygon->bVisited = 1;

	pPolygon->TransformPolygon(*m_pTriTransform);
	m_pInteriorPolygon->PushBack(pPolygon);

	for (DWORD i=0, end=pPolygon->rgBoundaryEdges.GetSize(); i<end; i++) {
		if (pPolygon->rgBoundaryEdges[i].dwPoly >= 0) {
			SPolygon3FEx * pNext = &(*m_pPolygon)[pPolygon->rgBoundaryEdges[i].dwPoly];
			if (!pNext->bVisited) FormInteriorPolygons(pNext);
		}
	}
}

void CContactRegions::ComputeCoplanarPolygons()
{
	if (dwCounter == 54)
		int a=0;
	/* Form the interior polygons */
	for (DWORD j=0, end2=m_pInteriorPolygon->GetSize(); j<end2; j++) {
		for (DWORD i=0, end=(*m_pInteriorPolygon)[j]->rgBoundaryEdges.GetSize(); i<end; i++) {
			if ((*m_pInteriorPolygon)[j]->rgBoundaryEdges[i].dwPoly >= 0)
				FormInteriorPolygons(&(*m_pPolygon)[(*m_pInteriorPolygon)[j]->rgBoundaryEdges[i].dwPoly]);
		}
	}
	ResetInteriorPolygons();

	/* Find the coplanar polygons */
	//Make sure that inter has all polys
	m_pCoplanarPolygon->Reserve(10);//Fix this
	if (dwCounter == 1497)
		int a=0;
	SPolygon3FEx ** rgInteriorPolygons = m_pInteriorPolygon->GetData();
	for (DWORD i=0, end=m_pInteriorPolygon->GetSize(); i<end; i++) {
		if (rgInteriorPolygons[i]->bVisited) continue;

		/* Find the boundary edge */
		DWORD dwBoundaryEdge = 0;//fix this for truly interior polys (add coplanar test)
		while (rgInteriorPolygons[i]->rgBoundaryEdges[dwBoundaryEdge].dwPoly >= 0 &&
			rgInteriorPolygons[i]->rgBoundaryEdges[dwBoundaryEdge].bCoplanar) dwBoundaryEdge++;

		m_pCoplanarPolygon->PushBack();//DWORD < 0 not good and problem when starting with coplanar tri
		FormCoplanarPolygons(
			rgInteriorPolygons[i], 
			rgInteriorPolygons[i]->rgBoundaryEdges[dwBoundaryEdge].dwPoly,
			rgInteriorPolygons[i]->rgBoundaryEdges[dwBoundaryEdge].dwPoly);
		//if (rgInteriorPolygons[i]->rgBoundaryEdges[dwBoundaryEdge].dwPoly >= 0) {
			m_pCoplanarPolygon->GetBack().rgEdge.PushBack(rgInteriorPolygons[i]->rgBoundaryEdges[dwBoundaryEdge]);
		//}

		m_pCoplanarPolygon->GetBack().normal = rgInteriorPolygons[i]->normal;
		m_pCoplanarPolygon->GetBack().areaNormal = m_pCoplanarPolygon->GetBack().ComputeArea() * rgInteriorPolygons[i]->normal;
	}

	/* Reset again */
	SPolygon3FEx ** rgIntPolygon = m_pInteriorPolygon->GetData();
	for (DWORD i=0, end=m_pInteriorPolygon->GetSize(); i<end; i++)
		rgIntPolygon[i]->bVisited = 0;

	/* Reset the coplanar polygon normals */
	//OutputDebugStringA("Hello0\n");
	for (DWORD i=0, end=m_pCoplanarPolygon->GetSize(); i<end; i++)
		(*m_pCoplanarPolygon)[i].avgNormal = (*m_pCoplanarPolygon)[i].areaNormal;
}

bool CContactRegions::FormContactPoints(DWORD dwBody, CArray<SContactPoint> * rgCP)
{
	m_pInteriorPolygon = &m_rgInteriorPolygon[dwBody];
	m_pCoplanarPolygon = &m_rgCoplanarPolygon[dwBody];

	m_rgCP = rgCP;
	m_pPolygon = m_rgPolygon[dwBody];


	//OutputDebugStringA("Hello1\n");
	for (DWORD i=0, end=m_pCoplanarPolygon->GetSize(); i<end; i++) {
		ComputeCPsInterior(&(*m_pCoplanarPolygon)[i]);
	}
	//OutputDebugStringA("Hello2\n");
	for (DWORD i=0, end=m_pCoplanarPolygon->GetSize(); i<end; i++) {
		ComputeCPsBoundary(&(*m_pCoplanarPolygon)[i]);
	}

	for (DWORD i=0, end=m_pCoplanarPolygon->GetSize(); i<end; i++)
		(*m_pCoplanarPolygon)[i].avgNormal = Vector3F(0);
	ResetAll();

	//m_rgCP->data()[0].iNor = -rgPolyhedron[0].areaNormal.Normalize();

	return 1;
}

void CContactRegions::AddConvexEdgeToContactRegion(SCoplanarPolygon3F * pPolygon, SPolygonEdge * pEdge)
{
	SCoplanarPolygon3F * pEdgePolygon = &(*m_pCoplanarPolygon)[(*m_pPolygon)[pEdge->dwPoly].dwCoplanarPolygon];
	if (pEdgePolygon->bVisited) return;

	pEdgePolygon->avgNormal += pPolygon->areaNormal;
	pPolygon->avgNormal += pEdgePolygon->areaNormal;

	/*Vector3F avgNormal(pPolygon->areaNormal + pEdgePolygon->areaNormal);
	Vector3F normal(avgNormal.Normalize());
	SContactPoint pt;
	pt.iNor = -normal;
	pt.iPos = pEdge->edge.e1;
	m_rgCP->PushBack(pt);
	pt.iPos = pEdge->edge.e2;
	m_rgCP->PushBack(pt);*/
}

void CContactRegions::AddConcaveEdgeToContactRegion(SCoplanarPolygon3F * pPolygon, SPolygonEdge * pEdge, bool bPreEdge, bool bNexEdge)
{
	SCoplanarPolygon3F * pEdgePolygon = &(*m_pCoplanarPolygon)[(*m_pPolygon)[pEdge->dwPoly].dwCoplanarPolygon];
	if (pEdgePolygon->bVisited) return;

	/*SContactPoint pt;
	pt.iNor = pPolygon->normal;
	//if (bPreEdge) {
		pt.iPos = pEdge->edge.e1;
		m_rgCP->PushBack(pt);
	//}
	//if (bNexEdge) {
		pt.iPos = pEdge->edge.e2;
		m_rgCP->PushBack(pt);
	//}*/
}

void CContactRegions::ComputeCPsInterior(SCoplanarPolygon3F * pPolygon)
{
	SPolygonEdge * rgEdge = pPolygon->rgEdge.GetData();

	bool bPreEdge = rgEdge[pPolygon->rgEdge.GetSize()-1].bConvex;
	for (DWORD i=0, end=pPolygon->rgEdge.GetSize(); i<end; i++) {
		if (rgEdge[i].dwPoly >= 0) {
			bool bCurEdge = rgEdge[i].bConvex;
			bool bNexEdge = rgEdge[(i+1)%end].bConvex;
		
			if (bCurEdge) {
				AddConvexEdgeToContactRegion(pPolygon, &rgEdge[i]);
			} else {
				AddConcaveEdgeToContactRegion(pPolygon, &rgEdge[i], bPreEdge, bNexEdge);
			}
		}
		bPreEdge = rgEdge[i].bConvex;
	}
	pPolygon->bVisited = 1;
}

void CContactRegions::ComputeCPsBoundary(SCoplanarPolygon3F * pPolygon)
{
	SContactPoint pt;
	pt.iNor = -pPolygon->avgNormal.Normalize();
	for (DWORD i=0, end=pPolygon->rgEdge.GetSize(); i<end; i++) {
		if (pPolygon->rgEdge[i].dwPoly > 0)
			continue;

		SLine3F & edge = pPolygon->rgEdge[i].edge;
		pt.iPos = edge.e1;
		m_rgCP->PushBack(pt);
		if (pPolygon->rgEdge[(i+1)%end].dwPoly > 0) {
			pt.iPos = edge.e2;
			m_rgCP->PushBack(pt);
		}
	}
}

bool IsAdmissibleNormal(Vector3F & nor, Vector3F & coneDir, float fConeTheta, bool bVertex, Vector3F & edgeDir) 
{
	if (bVertex) {
		float d = -DotXYZ(nor, coneDir);
		return d > fConeTheta;
	} /*else {

	}*/

	return 0;
}

void CContactRegions::AddValidContactPoint(Vector3F & pos, Vector3F & coneDir,
	float fConeTheta, bool bVertex, Vector3F & edgeDir, CArray<SCoplanarPolygon3F> * pOtherPolygons)
{
	SContactPoint pt;
	DWORD dwBestDist = 0;
	double bestDist = FLT_MAX;
	for (DWORD i=0, end=pOtherPolygons->GetSize(); i<end; i++) {
		//(*pOtherPolygons)[i].ClosestDist()
		double dist=0;
		if (dist < bestDist) {
			bestDist = dist;
			dwBestDist = i;
		}
	}
	if (bestDist < 1e-4) /* On the contact boundary */
		return;

	/* Determine if the normals are both in the admissible zone */
	Vector3F & nor = (*pOtherPolygons)[dwBestDist].normal;
	if (IsAdmissibleNormal(nor, coneDir, fConeTheta, bVertex, edgeDir)) {
		pt.iPos = pos;
		pt.iNor = -nor;
		m_rgCP->PushBack(pt);
	}
}

void CContactRegions::ComputeEdges(SCoplanarPolygon3F * pPolygon, CArray<SCoplanarPolygon3F> * pOtherPolygons)
{
	SPolygonEdge * rgEdge = pPolygon->rgEdge.GetData();

	for (DWORD i=0, end=pPolygon->rgEdge.GetSize(); i<end; i++) {
		if (rgEdge[i].dwPoly >= 0) {
			SCoplanarPolygon3F * pEdgePolygon = &(*m_pCoplanarPolygon)[(*m_pPolygon)[rgEdge[i].dwPoly].dwCoplanarPolygon];
			if (pEdgePolygon->bVisited) continue;

			Vector3F & e1 = rgEdge[i].edge.e1;
			Vector3F & e2 = rgEdge[i].edge.e2;
			Vector3F e1_2(.5f*(e1+e2));

			AddValidContactPoint(e1, rgEdge[i].coneDir, rgEdge[i].coneTheta, 0, Vector3F(0,0,0), pOtherPolygons);
			//AddValidContactPoint(e2, rgEdge[(i+1)%end].coneDir, rgEdge[(i+1)%end].coneTheta, 0, Vector3F(0,0,0));
			//AddValidContactPoint(e1_2, (rgEdge[i].coneDir + rgEdge[(i+1)%end].coneDir).Normalize(), rgEdge[i].coneTheta, 1, e2-e1);
		}
	}
	pPolygon->bVisited = 1;
}

void CContactRegions::FormContactPointsSD(CArray<SContactPoint> * rgCP)
{
	/* Body 1*/
	m_pCoplanarPolygon = &m_rgCoplanarPolygon[1];
	for (DWORD i=0, end=m_rgCoplanarPolygon[0].GetSize(); i<end; i++) {
		ComputeEdges(&(m_rgCoplanarPolygon[0])[i], &m_rgCoplanarPolygon[0]);
	}

	/* Body 2 */
	m_pCoplanarPolygon = &m_rgCoplanarPolygon[0];
	for (DWORD i=0, end=m_rgCoplanarPolygon[1].GetSize(); i<end; i++) {
		ComputeEdges(&(m_rgCoplanarPolygon[1])[i], &m_rgCoplanarPolygon[1]);
	}

	ResetAll();
}

void CContactRegions::ResetAll()
{
	SPolygon3FEx ** rgIntPolygon = m_pInteriorPolygon->GetData();
	for (DWORD i=0, end=m_pInteriorPolygon->GetSize(); i<end; i++) {
		rgIntPolygon[i]->bVisited = 0;
		rgIntPolygon[i]->rgBoundaryEdges.Clear();
		rgIntPolygon[i]->nBisectors = 0;
	}
	m_pInteriorPolygon->Resize(0);

	SCoplanarPolygon3F * rgCoPolygons = m_pCoplanarPolygon->GetData();
	for (DWORD i=0, end=m_pCoplanarPolygon->GetSize(); i<end; i++) {
		rgCoPolygons[i].rgEdge.Clear();
		rgCoPolygons[i].bVisited = 0;
	}
	m_pCoplanarPolygon->Resize(0);
	
	m_rgContactRegionVertex[0].Resize(0);
	m_rgContactRegionVertex[1].Resize(0);
}


};








	/*
bool BVTree::ClipPolygonBegin(DWORD dwVertex, DWORD dwBodyIndex, SPolygon3FEx * pPrePolygon)
{
	dwCounter;
	DWORD dwEdge=0;
	SPolygon3FEx * pCurPolygon = m_rgContactRegionVertex[dwVertex].pPolygon;

	if (pCurPolygon->nBisector && pCurPolygon->bisectorData[0].boundaryEdgeEnd == -1)
		DebugBreak();
	else if (pCurPolygon->nBisector == 2 && pCurPolygon->bisectorData[1].boundaryEdgeEnd == -1)
		DebugBreak();
	
	while (pCurPolygon->edges.data()[dwEdge].index != pPrePolygon->polyIndex)
		dwEdge++;

	SContactRegionVertex * rgVertex = m_rgContactRegionVertex.data();
	SLine3F l(rgVertex[dwVertex].v, rgVertex[dwVertex+1].v);

	SLine3F e;
	if (pCurPolygon->nBisector) {
		e = pCurPolygon->boundaryEdges.data()[pCurPolygon->bisectorData[0].boundaryEdgeEnd].edge;
		pCurPolygon->bisectorData[1].edgeBegin = dwEdge;
		pCurPolygon->bisectorData[1].boundaryEdgeBegin = pCurPolygon->bisectorData[0].boundaryEdgeEnd;
	} else {
		e = pCurPolygon->edges.data()[dwEdge].edge;
		pCurPolygon->bisectorData[0].edgeBegin = dwEdge;
		pCurPolygon->bisectorData[0].boundaryEdgeBegin = 0;
	}

	Vector3F intersection; double t=0;
	if (!e.Intersect(l, intersection, t)) return 0; 
	e.e2 = intersection;
	if (pCurPolygon->nBisector) {
		pCurPolygon->boundaryEdges.data()[pCurPolygon->bisectorData[0].boundaryEdgeEnd].edge = e;
	} else {
		pCurPolygon->PushBoundaryEdge(e, pPrePolygon->polyIndex);
	}

	return 1;
}

bool BVTree::ClipPolygonEnd(DWORD dwVertex, DWORD dwBodyIndex, SPolygon3FEx * pPrePolygon)
{
	DWORD dwEdge=0;
	SPolygon3FEx * pCurPolygon = m_rgContactRegionVertex[dwVertex].pPolygon;
	
	while (pPrePolygon->edges.data()[dwEdge].index != pCurPolygon->polyIndex)
		dwEdge++;

	SContactRegionVertex * rgVertex = m_rgContactRegionVertex.data();
	SLine3F l(rgVertex[dwVertex-1].v, rgVertex[dwVertex].v);
	
	SLine3F e;
	if (pPrePolygon->nBisector) {
		if (dwEdge == pPrePolygon->bisectorData[0].edgeBegin) {
			e = pPrePolygon->boundaryEdges.data()[pPrePolygon->bisectorData[0].boundaryEdgeBegin].edge;
			pPrePolygon->bisectorData[1].edgeEnd = dwEdge;
			pPrePolygon->bisectorData[1].boundaryEdgeEnd = -1;
		} else {
			e = pPrePolygon->edges.data()[dwEdge].edge;
			pPrePolygon->bisectorData[1].edgeEnd = dwEdge;
			pPrePolygon->bisectorData[1].boundaryEdgeEnd = pPrePolygon->boundaryEdges.size()-1;
		}

	} else if (dwEdge == pPrePolygon->bisectorData[0].edgeBegin) {
		e = pPrePolygon->boundaryEdges.data()[pPrePolygon->bisectorData[0].boundaryEdgeBegin].edge;
		pPrePolygon->bisectorData[0].edgeEnd = dwEdge;
		pPrePolygon->bisectorData[0].boundaryEdgeEnd = -1;
	} else {
		e = pPrePolygon->edges.data()[dwEdge].edge;
		pPrePolygon->bisectorData[0].edgeEnd = dwEdge;
		pPrePolygon->bisectorData[0].boundaryEdgeEnd = pPrePolygon->boundaryEdges.size();
	}

	Vector3F intersection; double t=0;
	if (!e.Intersect(l, intersection, t)) return 0;
	e.e1 = intersection;
	if (dwEdge == pPrePolygon->bisectorData[0].edgeBegin) {//something wrong here
		pPrePolygon->boundaryEdges.data()[pPrePolygon->bisectorData[0].boundaryEdgeBegin].edge = e;
	//} else if (pPrePolygon->nBisector && dwEdge == pPrePolygon->bisectorData[1].edgeBegin) {//check this
	//	pPrePolygon->boundaryEdges.data()[pPrePolygon->bisectorData[1].boundaryEdgeBegin].edge = e;
	} else {
		pPrePolygon->PushBoundaryEdge(e, pCurPolygon->polyIndex);
	}

	pPrePolygon->nBisector++;

	return 1;
}*/



	/* Find the polyhedrons 
	SPolygon3FEx * rgCoplanarPolygon = m_rgCoplanarPolygon.data();
	for (DWORD i=0, end=m_rgCoplanarPolygon.size(); i<end; i++) {
		if (m_rgCoplanarPolygon[i].visited) continue;

		m_rgPolyhedron.push_back(SPolyhedron3FEx());
		m_rgPolyhedron.back().polys.push_back(&rgCoplanarPolygon[i]);
		//m_rgPolyhedron.back().convex = FormPolyhedrons(&rgCoplanarPolygon[i]);
	}

	/* Finally, compute the normals and contact points 
	SPolyhedron3FEx * rgPolyhedron = m_rgPolyhedron.data();
	/*for (DWORD i=0, end=m_rgPolyhedron.size(); i<end; i++) {
		rgPolyhedron[i].areaNormal = rgPolyhedron[i].ComputeAreaNormal();
	}*/
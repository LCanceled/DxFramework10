
#include "DxUtPolygon.h"

namespace DxUt {

bool SLine2F::Intersect(SLine2F & l, Vector2F & intersection, double t)
{
	Vector2F & a0 = e1;
	Vector2F & a1 = e2;
	Vector2F & b0 = l.e1;
	Vector2F & b1 = l.e2;

	//Paul Bourke
	double mua,mub;
	double denom,numera,numerb;

	denom  = (b1.y-b0.y) * (a1.x-a0.x) - (b1.x-b0.x) * (a1.y-a0.y);
	numera = (b1.x-b0.x) * (a0.y-b0.y) - (b1.y-b0.y) * (a0.x-b0.x);
	numerb = (a1.x-a0.x) * (a0.y-b0.y) - (a1.y-a0.y) * (a0.x-b0.x);

	const double EPS = .0001;
	/* Are the line coincident? */
	if (Abs(numera) < EPS && Abs(numerb) < EPS && Abs(denom) < EPS) {
		intersection.x = (a0.x + a1.x) / 2;
		intersection.y = (a0.y + a1.y) / 2;
		return true;
	}

	/* Are the line parallel */
	if (Abs(denom) < EPS) {
		intersection.x = 0;
		intersection.y = 0;
		return false;
	}

	/* Is the intersection along the the segments */
	mua = numera / denom;
	mub = numerb / denom;
	if (mua < 0 || mua > 1 || mub < 0 || mub > 1) {
		intersection.x = 0;
		intersection.y = 0;
		return false;
	}
	intersection.x = a0.x + mua * (a1.x - a0.x);
	intersection.y = a0.y + mua * (a1.y - a0.y);
	return true;
}

bool SLine3F::Intersect(SLine3F & l, Vector3F & intersection, double t)
{
	struct SDoubleVec {
		double x,y,z;
	};
	SDoubleVec p1 = {e1.x, e1.y, e1.z};
	SDoubleVec p2 = {e2.x, e2.y, e2.z};
	SDoubleVec p3 = {l.e1.x, l.e1.y, l.e1.z};
	SDoubleVec p4 = {l.e2.x, l.e2.y, l.e2.z};

	//Paul Bourke
   SDoubleVec p13,p43,p21, pa, pb;
   double d1343,d4321,d1321,d4343,d2121;
   double numer,denom, mua, mub;
   const double EPS = 1e-8f;

   p13.x = p1.x - p3.x;
   p13.y = p1.y - p3.y;
   p13.z = p1.z - p3.z;
   p43.x = p4.x - p3.x;
   p43.y = p4.y - p3.y;
   p43.z = p4.z - p3.z;
   if (Abs(p43.x) < EPS && Abs(p43.y) < EPS && Abs(p43.z) < EPS)
      return 0;
   p21.x = p2.x - p1.x;
   p21.y = p2.y - p1.y;
   p21.z = p2.z - p1.z;
   if (Abs(p21.x) < EPS && Abs(p21.y) < EPS && Abs(p21.z) < EPS)
      return 0;

   d1343 = p13.x * p43.x + p13.y * p43.y + p13.z * p43.z;
   d4321 = p43.x * p21.x + p43.y * p21.y + p43.z * p21.z;
   d1321 = p13.x * p21.x + p13.y * p21.y + p13.z * p21.z;
   d4343 = p43.x * p43.x + p43.y * p43.y + p43.z * p43.z;
   d2121 = p21.x * p21.x + p21.y * p21.y + p21.z * p21.z;

   denom = d2121 * d4343 - d4321 * d4321;
   if (Abs(denom) < EPS)
      return 0;
   numer = d1343 * d4321 - d1321 * d4343;

   mua = numer / denom;
   mub = (d1343 + d4321 * (mua)) / d4343;

   pa.x = p1.x + mua * p21.x;
   pa.y = p1.y + mua * p21.y;
   pa.z = p1.z + mua * p21.z;
   pb.x = p3.x + mub * p43.x;
   pb.y = p3.y + mub * p43.y;
   pb.z = p3.z + mub * p43.z;

   //intersection = pa;
   intersection.x = pa.x;
   intersection.y = pa.y;
   intersection.z = pa.z;
   t = mua;

   return 1;
}
/*
SPolygon3F::SPolygon3F(STriangleF & tri, DWORD triIndex, DWORD e0Index, DWORD e1Index, DWORD e2Index) 
{
	edges.push_back(SPolygonEdge<SLine3F>(SLine3F(tri.vPosW[0], tri.vPosW[1]), e0Index));
	edges.push_back(SPolygonEdge<SLine3F>(SLine3F(tri.vPosW[1], tri.vPosW[2]), e1Index));
	edges.push_back(SPolygonEdge<SLine3F>(SLine3F(tri.vPosW[2], tri.vPosW[0]), e2Index));

	normal = tri.Normal();
	polyIndex = triIndex;
}*/
/*
bool SPolygon3F::ConvexClipAgainstLine(SLine3F & l, bool bCW)
{
	EdgeIterator iter = edges.begin();
	EdgeIterator firstEdge, secondEdge;
	DWORD nEdges = edges.size(), dwFirst=1;

	SLine3F localL(Vector3F(0,0,0), l.e2 - l.e1);
	PlaneF localPlane; localPlane.ComputePlane(localL.e2, localL.e1, normal+localL.e1);
	
	for (DWORD i=0; i<nEdges; i++, iter++) {
		Vector3F localE0(iter->edge.e1 - l.e1);
		Vector3F localE1(iter->edge.e2 - l.e1);

		float d;
		double pred0 = localPlane.PointUnderPlane(localE0, d);
		double pred1 = localPlane.PointUnderPlane(localE1, d);
		if ((pred0 == 0 && pred1 != 0) || (pred0 != 0 && pred1 == 0)) {
			if (dwFirst) firstEdge = iter;
			else { 
				secondEdge = iter;
				if (pred0 != 0) {
					EdgeIterator tmp = firstEdge;
					firstEdge = secondEdge;
					secondEdge = tmp;
				}
				dwFirst = 2;
				break;
			}
			dwFirst = 0;
		}
	}
	if (dwFirst != 2) return 0;

	SLine3F newL;
	double t=0;
	firstEdge->edge.Intersect(l, newL.e1, t);
	firstEdge->edge.e2 = newL.e1; 
	EdgeIterator lastEdge = edges.end(); firstEdge++;
	while (firstEdge != secondEdge && firstEdge != lastEdge) {
		EdgeIterator nextEdge = firstEdge;
		nextEdge++;

		edges.erase(firstEdge);
		firstEdge = nextEdge;
	}
	if (firstEdge != secondEdge) {
		firstEdge = edges.begin();
		while (firstEdge != secondEdge) {
			EdgeIterator nextEdge = firstEdge;
			nextEdge++;

			edges.erase(firstEdge);
			firstEdge = nextEdge;
		}
	}
	secondEdge->edge.Intersect(l, newL.e2, t);
	secondEdge->edge.e1 = newL.e2;
	//On the border of the contact region
	secondEdge->index = -1;
	edges.insert(firstEdge, newL);

	return 1;
}*/
/*
void SPolygon3F::Join(STriangleF & tri, DWORD e0Index, DWORD e1Index, DWORD e2Index)
{
	Vector3F e1(tri.vPosW[1] - tri.vPosW[0]);
	Vector3F e2(tri.vPosW[2] - tri.vPosW[1]);
	Vector3F e3(tri.vPosW[0] - tri.vPosW[2]);

	EdgeIterator iter = edges.begin();
	DWORD nEdges = edges.size();
	for (DWORD i=0; i<nEdges; i++) {

	}
}
*/


};













/*
{
		int N = edges.size();
		SLine<T> * l = edges.data();

		//Paul Bourke
		int i,j;
		double areaVec[3] = {0,0,0};

		for (i=0;i<N;i++) {
			j = (i + 1) % N;
			areaVec[2] += l[i].e1.x * l[j].e1.y;
			areaVec[2] -= l[i].e1.y * l[j].e1.x;
	
			areaVec[1] += l[i].e1.z * l[j].e1.x;
			areaVec[1] -= l[i].e1.x * l[j].e1.z;

			areaVec[0] += l[i].e1.y * l[j].e1.z;
			areaVec[0] -= l[i].e1.z * l[j].e1.y;
		}

		areaVec /= 2.f;
		return sqrt(areaVec[0]*areaVec[0] + areaVec[1]*areaVec[1] + areaVec[2]*areaVec[2]);
	}
*/
/*SPolygon3F::SPolygon3F(STriangleF & tri) 
{
	edges.push_back(SLine3F(tri.vPosW[0], tri.vPosW[1]));
	edges.push_back(SLine3F(tri.vPosW[1], tri.vPosW[2]));
	edges.push_back(SLine3F(tri.vPosW[2], tri.vPosW[0]));

	normal = tri.Normal();
	tangent = (tri.vPosW[1] - tri.vPosW[0]).Normalize();
	bitangent = CrossXYZ(normal, tangent);
	relativePosition = tri.vPosW[0];

	EdgeIterator iter = edges.begin();
	poly2D.edges.push_back(ProjectLineTo2D(*iter));
	poly2D.edges.push_back(ProjectLineTo2D(*++iter));
	poly2D.edges.push_back(ProjectLineTo2D(*++iter));
}

SLine2F SPolygon3F::ProjectLineTo2D(SLine3F & l)
{
	SLine3F relativeLine(l.e1 - relativePosition, l.e2 - relativePosition);

	return SLine2F(
		Vector2F(DotXYZ(tangent, relativeLine.e1), DotXYZ(bitangent, relativeLine.e1)),
		Vector2F(DotXYZ(tangent, relativeLine.e2), DotXYZ(bitangent, relativeLine.e2)) );
}
*/
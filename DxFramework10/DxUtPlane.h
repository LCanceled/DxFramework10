
#ifndef DXUTPLANE_H
#define DXUTPLANE_H

#include "DxUtVector.h"

namespace DxUt {

class PlaneF  {
public:
	Vector3F n;
	float d;
public:
	PlaneF() {}
	PlaneF(PlaneF & copy) {memcpy(this, &copy, sizeof(PlaneF));}
	PlaneF(Vector3F & nor, float dist):n(nor), d(dist) {}
	PlaneF(Vector3F & p1, Vector3F & p2, Vector3F & p3) {
		ComputePlane(p1, p2, p3);
	}
	//~PlaneF() {d=0;}

	void ComputePlane(Vector3F & pt, Vector3F & nor) {
		n = nor.Normalize();
		d = DotXYZ(pt, n);
	}

	//Normalized plane
	void ComputePlane(Vector3F & p1, Vector3F & p2, Vector3F & p3) {
		Vector3F v1(p2 - p1);
		Vector3F v2(p3 - p1);
		n = Vector3F(CrossXYZ(v1,v2)).Normalize();
		d = DotXYZ(p1, n);
	}
	//Denormalized plane
	void ComputePlaneSq(Vector3F & p1, Vector3F & p2, Vector3F & p3) {
		Vector3F v1(p2 - p1);
		Vector3F v2(p3 - p1);
		n = Vector3F(CrossXYZ(v1,v2));
		d = DotXYZ(p1, n);
	}

	/*void NormalizePlane() {
		n = n.Normalize();
		d = DotXYZ(p1, n);
	}*/

	bool PointUnderPlane(Vector3F & pt, float & dist);
	float DistanceToPlane(Vector3F & pt);
	//The plane must be normalized
	Vector3F ClosestPointOnPlane(Vector3F & pt);

	PlaneF operator+() {return *this;}
	PlaneF operator-() {return PlaneF(-n, -d);}

	friend PlaneF operator*(float flt, PlaneF & pl) {
		return PlaneF((flt*pl.n), flt*pl.d);
	}
	PlaneF operator/(float flt) {
		float div = 1.f/flt;
		return PlaneF((div*n), div*d); 
	}

	PlaneF & operator=(PlaneF & pl) {
		n = pl.n;
		d = pl.d;

		return *this;
	}
	PlaneF & operator*=(float flt) {
		n.x *= flt; 
		n.y *= flt; 
		n.z *= flt;
		d *= flt;

		return *this;
	}
	PlaneF & operator/=(float flt) {
		float div = 1.f/flt;
		n.x *= div; 
		n.y *= div; 
		n.z *= div;
		d *= div;

		return *this;
	}

	void operator()(float a, float b, float c, float _d) {n.x = a, n.y = b, n.z = c, d = _d;}
	void operator()(Vector3F & _n, float _d) {n = _n, d = _d;}
};

inline bool PlaneF::PointUnderPlane(Vector3F & pt, float & dist)
{
	dist = DotXYZ(pt, n);
	return dist > d;
}

/* Compute signed distance to plane. */
inline float PlaneF::DistanceToPlane(Vector3F & pt)
{
	return DotXYZ(pt, n) - d;
}

//The plane must be normalized
inline Vector3F PlaneF::ClosestPointOnPlane(Vector3F & pt)
{
	float dist = DotXYZ(pt, n);
	Vector3F c((dist-d)*n);
	return pt - c;
}


inline void Compute2DPlaneBasis(Vector3F & n, Vector3F & v1, Vector3F & v2)
{
	Vector3F up(0, 1, 0);
	v1 = CrossXYZ(up, n);
	if (v1.LengthSq() < 1e-4) v1 = Vector3F(0, 0, 1.f);
	v2 = CrossXYZ(n, v1);
}

inline void Compute2DPlaneBasis(PlaneF & pl, Vector3F & v1, Vector3F & v2)
{
	Compute2DPlaneBasis(pl.n, v1, v2);
}

inline Vector2F ProjectPointTo2DPlaneBasis(Vector3F & pt, Vector3F & o, Vector3F & v1, Vector3F & v2)
{
	Vector3F & p(pt - o);
	return Vector2F(DotXYZ(v1, p), DotXYZ(v2, p));
}


};


#endif
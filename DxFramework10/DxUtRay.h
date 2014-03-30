
#ifndef RAY_H
#define RAY_H

#include "DxUtVector.h"

namespace DxUt {

struct SRay {
	Vector3F d;
	Vector3F p;
public:
	SRay() {}
	//~SRay() {}

	/* The transformation must strictly be a rotation and translation */
	/*void Transform(Matrix4x4F & rT) {
		p = rT*p;
		d = rT^d;
	}*/
};

struct SRayIntersectData {
	Vector3F pos;
	Vector3F nor;
	UINT uiBody;
	UINT uiTri;
	float t;
	
	/* Barycentric coordinates */
	float u, v, w;
};


};


#endif
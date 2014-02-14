
#ifndef DXUTLIGHT_H
#define DXUTLIGHT_H

#include "DxUtInclude.h"

namespace DxUt {

struct SLightDir {
	D3DXCOLOR amb;
	D3DXCOLOR dif;
	D3DXCOLOR spe;
	Vector3F vec;
};

struct SLightPoint {
	D3DXCOLOR amb;
	D3DXCOLOR dif;
	D3DXCOLOR spe;
	Vector3F pos;
	Vector3F atten;
};

struct SLightSpot {
	D3DXCOLOR amb;
	D3DXCOLOR dif;
	D3DXCOLOR spe;
	Vector3F pos;
	Vector3F vec;
	FLOAT spotPow;
	Vector3F atten;
};


};


#endif


#ifndef DXUTMATERIAL_H
#define DXUTMATERIAL_H

#include "DxUtInclude.h"

namespace DxUt {

struct SMaterial {
	D3DXCOLOR amb;
	D3DXCOLOR dif;
	D3DXCOLOR spe;
	float pow;
};


};


#endif
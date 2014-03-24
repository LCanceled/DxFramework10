
#ifndef DXUTEFFECTPOOL_H
#define DXUTEFFECTPOOL_H

#include "DxUtResourcePool.h"
#include "DxUtShaders.h"

namespace DxUt {

class CEffectPool : public CResourcePool<CEffect> {
public:
	CEffectPool() {}
	//~CEffectPool() {}

	friend class CD3DApp;
};


};


#endif
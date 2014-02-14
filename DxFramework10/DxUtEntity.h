
#ifndef DXUTENTITY_H
#define DXUTENTITY_H

#include "DxUtAABBox.h"
#include "DxUtBVTree.h"

namespace DxUt {

struct SEntity {
	AABBox * pBox;
	BVTree * pBVTree;
};


};


#endif

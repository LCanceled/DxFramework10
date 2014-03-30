
#ifndef DXUTMESHPOOL_H
#define DXUTMESHPOOL_H

#include <d3dx10.h>
#include "DxUtMesh.h"
#include "DxUtResourcePool.h"

namespace DxUt {

class CMeshPool : public CResourcePool<CMesh> {
private:
	CMeshPool() {}
	//~CMeshPool() {}

	friend class CD3DApp;
public:

};


};

#endif
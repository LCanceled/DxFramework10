
#ifndef DXUTMESHPOOL_H
#define DXUTMESHPOOL_H

#include <d3dx10.h>
#include "DxUtMesh.h"
#include "DxUtResourcePool.h"

namespace DxUt {

struct ID3DX10MeshEx : public ID3DX10Mesh {
	void Destroy() {Release(); }
};

class CMeshPool : public CResourcePool<ID3DX10MeshEx> {
	CMeshPool() {}
	//~CMeshPool() {}

	void Destroy();

	friend class CD3DApp;
public:

};


};

#endif
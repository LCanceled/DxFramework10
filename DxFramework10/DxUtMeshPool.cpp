
#include "DxUtMeshPool.h"

namespace DxUt {

std::unordered_map<std::string, CResourcePool<ID3DX10MeshEx>::SResourceElement> CResourcePool<ID3DX10MeshEx>::m_Elements;


void CMeshPool::Destroy() {
	for (UM::iterator it=m_Elements.begin(); it!=m_Elements.end(); it++) {
		ReleaseX(it->second.pResource);
		delete it->second.pResource;
	}
	m_Elements.clear();
}

};

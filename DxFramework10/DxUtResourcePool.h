
#ifndef RESOURCEPOOL_H
#define RESOURCEPOOL_H

#include <string>
#include <unordered_map>

namespace DxUt {

/* Cannot make this class a singelton because of DirectX */
template<typename T>
class CResourcePool {
protected:
	struct SResourceElement {
		SResourceElement() {}
		~SResourceElement() {}

		std::string name;
		T * pResource;

		bool operator==(SResourceElement & e) {
			return name == e.name;
		}
	};
	typedef std::unordered_map<std::string, SResourceElement> UM; 
	static UM m_Elements;
		
	virtual void Destroy() {
		for (UM::iterator it=m_Elements.begin(); it!=m_Elements.end(); it++) {
			it->second.pResource->Destroy();
			delete it->second.pResource;
			it->second.pResource = NULL;
		}
		m_Elements.clear();
	}

	CResourcePool() {}
	//~CResourcePool() {}

	friend class CD3DApp;
public:
	static bool GetResource(char * szName, T ** pResource) {
		DWORD dwIndex = 0;
		SResourceElement el;
		UM::iterator got = m_Elements.find(szName);
		if (got != m_Elements.end()) {
			*pResource = got->second.pResource;
			return 1;
		}  else {
			return 0;
		}
	}

	static void PutResource(char * szName, T * pResource) {
		SResourceElement el;
		el.pResource = pResource;
		m_Elements.insert(std::pair<std::string, SResourceElement>(std::string(szName), el));
	}
	
};



};


#endif
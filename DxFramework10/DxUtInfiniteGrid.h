
#ifndef CUNIFORMGRID_H
#define CUNIFORMGRID_H

#include "DxUtInclude.h"
#include "DxUtAABBox.h"
#include <vector>

namespace DxUt {

/* The hash table should be of a size at least 2 times the number of elements in it */
#define HASHTABLE_SCALING 2

template <class T>
class CUniformGrid {
private:
	struct SObjectEntry {
		DWORD dwId;
		BOOL bUsed;

		/* The data that represents in */
		/* which cells the entry lies */
		int xCellL, yCellL, zCellL;
		int xCellR, yCellR, zCellR;

		T * pObj;
		CAABBox * pBox;
	};
	std::vector<SObjectEntry> m_rgObjEntry;
	
	struct SHashTableEntry {
		SObjectEntry * pObjEntry;
		SHashTableEntry * pNext;
	};
	std::vector<SHashTableEntry> m_rgHashTablelEntry;

	float m_fCellSize;

	int IntFloor(float flt) {/* http://stereopsis.com/FPU.html#convert */
        int a = (int)flt;
        return flt < 0.0f && flt != a ? a - 1 : a;
	}
	
	void HashObjToHashTableEntries(SObjectEntry & rObj, Vector3F & min, Vector3F & max);
	SHashTableEntry * GetFreeNodeOfHashTableEntry(DWORD dwHashIndex) {
		SHashTableEntry * pStart =  m_rgHashTablelEntry.data() + dwHashIndex;
		if (!pStart->pObjEntry) return pStart;

		while (pStart->pNext) {pStart = pStart->pNext;}
		pStart->pNext = new SHashTableEntry;
		pStart->pNext->pObjEntry = NULL;
		pStart->pNext->pNext = NULL;
		return pStart->pNext;
	}
	DWORD ComputeHashTableIndex(int i, int j, int k) {
		static const int h1 = 0x8da6b343;
		static const int h2 = 0xd8163841;
		static const int h3 = 0xcb1ab31f;

		int n = h1 * i + h2 * j + h3 * k;
		n = n % m_rgHashTablelEntry.size();
		if (n < 0) n += m_rgHashTablelEntry.size();
		return n;
	}
	void FindObjectsInSameCell(DWORD dwId, int i, int j, int k,  std::vector<T*> & rgObj);
	void RemoveObjFromHashTableEntry(DWORD dwObjId, DWORD dwHashIndex);
	void DestroyHashTableEntries();
public:
	CUniformGrid();
	//~CUniformGrid();

	/* Creates a grid of infinite extent centered at 0 */
	/* Give a nonzero hint as to how many Objects there will be */
	void CreateGrid(float fCellSize, DWORD nHintObj);

	/* The return is an id for that object */
	/* Any object entered must possess its own AABB */
	DWORD AddObject(T * pObj, CAABBox * pBox);

	/* It is assumed that the owner of each AABB updates it */
	void UpdateGrid();

	/* rgObj should have some base of memory reserved */
	void FindOverlappingObjects(DWORD dwId, std::vector<T*> & rgObj);

	/* Once FindOverlappingObjects finds an array of objs, any subsequent call */
	/* to the function will exclude those previously found from a new array of objs. */
	/* Simply put, it will not give duplicates. When it becomes necessary to find, */
	/* once again, previously acquired objs, ResetUsedObjs should be called. */
	void ResetUsedObjs();

	void DestroyGrid();
};

/////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////Private Member Functions//////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

template <typename T>
void CUniformGrid<T>::HashObjToHashTableEntries(SObjectEntry & rObj, Vector3F & min, Vector3F & max)
{
	float div = 1.f/m_fCellSize;
	int xCellL = floorf_ASM(min.x*div);
	int yCellL = floorf_ASM(min.y*div);
	int zCellL = floorf_ASM(min.z*div);
	int xCellR = ceilf_ASM(max.x*div);
	int yCellR = ceilf_ASM(max.y*div);
	int zCellR = ceilf_ASM(max.z*div);

	/* Remove the obj from cells which it is now not in */
	for (int i=rObj.xCellL; i<rObj.xCellR; i++) {
		for (int j=rObj.yCellL; j<rObj.yCellR; j++) {
			for (int k=rObj.zCellL; k<rObj.zCellR; k++) {
				if (xCellL <= i && i < xCellR &&
					yCellL <= j && j < yCellR &&
					zCellL <= k && k < zCellR) continue;

				RemoveObjFromHashTableEntry(rObj.dwId, ComputeHashTableIndex(i, j, k));
			}
		}
	}
	/* Add the obj to the new cells */
	for (int i=xCellL; i<xCellR; i++) {
		for (int j=yCellL; j<yCellR; j++) {
			for (int k=zCellL; k<zCellR; k++) {
				if (rObj.xCellL <= i && i < rObj.xCellR &&
					rObj.yCellL <= j && j < rObj.yCellR &&
					rObj.zCellL <= k && k < rObj.zCellR) continue;

				SHashTableEntry * pH = GetFreeNodeOfHashTableEntry(ComputeHashTableIndex(i, j, k));
				pH->pObjEntry = &rObj;
			}
		}
	}
	rObj.xCellL = xCellL;
	rObj.yCellL = yCellL;
	rObj.zCellL = zCellL;
	rObj.xCellR = xCellR;
	rObj.yCellR = yCellR;
	rObj.zCellR = zCellR;
}

template <typename T>
void CUniformGrid<T>::FindObjectsInSameCell(DWORD dwHashIndex, int i, int j, int k, std::vector<T*> & rgObj)
{
	SHashTableEntry * pH = &m_rgHashTablelEntry[dwHashIndex];
	do {
		SObjectEntry & rObj = *pH->pObjEntry;
		if (!rObj.bUsed && 
			rObj.xCellL <= i && i < rObj.xCellR &&
			rObj.yCellL <= j && j < rObj.yCellR &&
			rObj.zCellL <= k && k < rObj.zCellR) 
		{
			rgObj.push_back(pH->pObjEntry->pObj);
			rObj.bUsed = 1;
		}
		pH = pH->pNext;
	} while (pH);
}

template <typename T>
void CUniformGrid<T>::RemoveObjFromHashTableEntry(DWORD dwObjId, DWORD dwHashIndex) 
{
	SHashTableEntry * pStart = m_rgHashTablelEntry.data() + dwHashIndex;
	if (pStart->pObjEntry->dwId == dwObjId) {
		SHashTableEntry * pNext = pStart->pNext;
		if (pNext) {
			memcpy(pStart, pNext, sizeof(SHashTableEntry));
			delete pNext;
		} else {
			pStart->pObjEntry = NULL;
			pStart->pNext = NULL;
		}

		return;
	}

	/* It is guranteed that this is false at some point */
	while (pStart->pNext->pObjEntry->dwId != dwObjId) {
		pStart = pStart->pNext; }

	SHashTableEntry * pDelete = pStart->pNext;
	pStart->pNext = pStart->pNext->pNext;
	delete pDelete;
}

template <typename T>
void CUniformGrid<T>::DestroyHashTableEntries()
{
	DWORD n = m_rgHashTablelEntry.size();
	SHashTableEntry * pHash = m_rgHashTablelEntry.data();
	for (DWORD i=0; i<n; i++, pHash++) {
		SHashTableEntry * pLink = pHash->pNext;
		while (pLink) {
			SHashTableEntry * pNext = pLink->pNext;
			delete pLink;
			pLink = pNext;
		}
		memset(pHash, 0, sizeof(SHashTableEntry));
	}

}

/////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////Public Member Functions///////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

template <typename T>
CUniformGrid<T>::CUniformGrid()
{
}

template <typename T>
void CUniformGrid<T>::CreateGrid(float fCellSize, DWORD nHintObj)
{
	m_fCellSize = fCellSize;
	m_rgObjEntry.reserve(nHintObj);
	m_rgHashTablelEntry.resize(HASHTABLE_SCALING*nHintObj);
	memset(m_rgHashTablelEntry.data(), 0, HASHTABLE_SCALING*nHintObj*sizeof(SHashTableEntry));
}

template <typename T>
DWORD CUniformGrid<T>::AddObject(T * pObj, CAABBox * pBox)
{
	SObjectEntry o;
	o.dwId = m_rgObjEntry.size();
	o.bUsed = 0;
	o.xCellL = o.xCellR = 0;
	o.yCellL = o.yCellR = 0;
	o.zCellL = o.zCellR = 0;
	o.pObj = pObj;
	o.pBox = pBox;
	m_rgObjEntry.push_back(o);

	if (HASHTABLE_SCALING*m_rgObjEntry.size() > m_rgHashTablelEntry.size()) {
		DestroyHashTableEntries();
		m_rgHashTablelEntry.resize(HASHTABLE_SCALING*m_rgObjEntry.capacity());

		/* Reset the obj cells */
		DWORD n = m_rgObjEntry.size();
		SObjectEntry * pObj = m_rgObjEntry.data();
		for (DWORD i=0; i<n; i++, pObj++) {
			pObj->xCellL = pObj->xCellR = 0;
			pObj->yCellL = pObj->yCellR = 0;
			pObj->zCellL = pObj->zCellR = 0;
		}

		UpdateGrid();
	} else {
		HashObjToHashTableEntries(m_rgObjEntry.back(), pBox->m_MinPW, pBox->m_MaxPW); /* TODO!!! */
	}

	return m_rgObjEntry.size()-1;
}

template <typename T>
void CUniformGrid<T>::UpdateGrid()
{
	DWORD n = m_rgObjEntry.size();
	SObjectEntry * pObj = m_rgObjEntry.data();
	for (DWORD i=0; i<n; i++, pObj++) {
		HashObjToHashTableEntries(*pObj, pObj->pBox->m_MinPW, pObj->pBox->m_MaxPW);
	}
}

template <typename T>
void CUniformGrid<T>::FindOverlappingObjects(DWORD dwObjId, std::vector<T*> & rgObj)
{
	SObjectEntry & rObj = m_rgObjEntry.data()[dwObjId];
	rObj.bUsed = 1;

	for (int i=rObj.xCellL; i<rObj.xCellR; i++) {
		for (int j=rObj.yCellL; j<rObj.yCellR; j++) {
			for (int k=rObj.zCellL; k<rObj.zCellR; k++) {
				FindObjectsInSameCell(ComputeHashTableIndex(i, j, k), i, j, k, rgObj);
			}
		}
	}
}

template <typename T>
void CUniformGrid<T>::ResetUsedObjs()
{
	DWORD n = m_rgObjEntry.size();
	SObjectEntry * pObj = m_rgObjEntry.data();
	for (DWORD i=0; i<n; i++, pObj++)
		pObj->bUsed = 0;
}

template <typename T>
void CUniformGrid<T>::DestroyGrid()
{
	DestroyHashTableEntries();
	m_rgHashTablelEntry.clear();
	m_rgObjEntry.clear();
}


};


#endif
















	/*while (pStart->pObjEntry->dwId != dwObjId) {
		pPre = &pStart->pNext;
		pStart = pStart->pNext; }
	SHashTableEntry * pNext = pStart->pNext;

	if (!pPre) {
		if (!pNext) {
			pStart->pObjEntry = NULL;
			pStart->pNext = NULL;
		} else {
			pStart->pObjEntry = pNext->pObjEntry;
			pStart->pNext = pNext->pNext;
		}
	} else {
		delete pStart;
		*pPre = pNext;
	}*/

#ifndef DXUTUNIFORMGRID_H
#define DXUTUNIFORMGRID_H

#include "DxUtInclude.h"
#include "DxUtAABBox.h"
#include <vector>

namespace DxUt {

template <typename T>
class CUniformGrid {
private:
	struct SObjectEntry {
		DWORD dwId;
		BOOL bUsed;

		/* The data that represents in */
		/* which cells the entry lies */
		INT32 xCellL, yCellL, zCellL;
		INT32 xCellR, yCellR, zCellR;

		T * pObj;
		AABBox * pBox;
	};
	std::vector<SObjectEntry> m_rgObjEntry;
	SObjectEntry * m_pObjEntry;
	
	struct STableEntry {
		SObjectEntry * pObjEntry;
		STableEntry * pNext;
	};
	std::vector<STableEntry> m_rgTblEntry;
	STableEntry * m_pTblEntry;

	float m_fCellSize;

	void HashObjToCell(SObjectEntry & pObj, Vector3F min, Vector3F max);
	STableEntry * GetFreeNodeOfCell(STableEntry * pStart) {
		while (pStart->pObjEntry != 0) {
			if (pStart->pNext == NULL) pStart->pNext = new STableEntry = {0, 0};
			pStart = pStart->pNext; 
		}
		return pStart;
	}
	DWORD ComputeHashTableIndex(INT32 i, INT32 j, INT32 k) {
		static const INT32 h1 = 0x8da6b343;
		static const INT32 h2 = 0xd8163841;
		static const INT32 h3 = 0xcb1ab31f;

		INT32 n = h1 * i + h2 * j + h3 * k;
		n = n % m_rgTblEntry.size();
		if (n < 0) n += m_rgTblEntry.size();
		return n;
	}
public:
	CUniformGrid();
	//~CUniformGrid();

	/* Creates a grid of infinite extent centered at 0 */
	/* Give a nonzero hint as to how many Objects there will be */
	void CreateGrid(float fCellSize, DWORD nHintObj);

	/* The return is an id for that object */
	/* Any object entered must possess its own AABB */
	DWORD AddObject(T * pObj, AABBox * pBox);

	/* It is assumed that the owner of each AABB updates it */
	void UpdateGrid();

	/* rgObj should have some base of memory reserved */
	void FindObjectsInSameCell(DWORD dwId, std::vector<T*> & rgObj);

	void DestroyGrid();
};

/* The hash table should be of a size at least 2 times the number of elements in it */
#define HASHTABLE_SCALING 2

template <typename T>
CUniformGrid<T>::CUniformGrid()
{
}

template <typename T>
void CUniformGrid<T>::CreateGrid(float fCellSize, DWORD nHintObj)
{
	m_rgObjEntry.reserve(nHintObj);
	m_rgTblEntry.resize(HASHTABLE_SCALING*nHintObj);
	memset(m_rgTblEntry.data(), 0, HASHTABLE_SCALING*nHintObj, sizeof(STableEntry));
	
	m_fCellSize = fCellSize;
}
template <typename T>
DWORD CUniformGrid<T>::AddObject(T * pObj, AABBox * pBox)
{
	SObjectEntry o;
	o.dwId = m_rgObjEntry.size();
	o.pObj = pObj;
	o.pBox = pBox;
	m_rgObjEntry.push_back(o);
	m_pObjEntry = m_rgObjEntry.data();

	if (HASHTABLE_SCALING*m_rgObjEntry.size() > m_rgTblEntry.size()) {
		m_rgTblEntry.resize(HASHTABLE_SCALING*m_rgObjEntry.capacity());
		//ReHash();
		//	memset(m_rgTblEntry.data(), 0, HASHTABLE_SCALING*nHintObj, sizeof(STableEntry));
	}
	m_pTblEntry = m_rgTblEntry.data();

	return m_rgObjEntry.size()-1;
}

template <typename T>
void CUniformGrid<T>::HashObjToCell(T & pObj, Vector3F min, Vector3F max)
{
	float div = 1.f/m_fCellSize;
	INT32 xCellL = (INT32)floorf(min.x*div);
	INT32 yCellL = (INT32)floorf(min.y*div);
	INT32 zCellL = (INT32)floorf(min.z*div);
	INT32 xCellR = (INT32)ceilf(max.x*div);
	INT32 yCellR = (INT32)ceilf(max.y*div);
	INT32 zCellR = (INT32)ceilf(max.z*div);

	const INT32 h1 = 0x8da6b343;
	const INT32 h2 = 0xd8163841;
	const INT32 h3 = 0xcb1ab31f;
	/* Remove the objs from the old cells */
	for (INT32 i=pObj.xCellL; i<pObj.xCellR; i++) {
		for (INT32 j=pObj.yCellL; j<pObj.yCellR; j++) {
			for (INT32 k=pObj.zCellL; k<pObj.zCellR; k++) {
				if (xCellL <= i && i < xCellR &&
					yCellL <= j && j < yCellR &&
					zCellL <= k && k < zCellR) continue;
				RemoveNodeOfCell(pObj, );
			}
		}
	}
	for (INT32 i=xCellL; i<xCellR; i++) {
		for (INT32 j=yCellL; j<yCellR; j++) {
			for (INT32 k=zCellL; k<zCellR; k++) {
				if (pObj.xCellL <= i && i < pObj.xCellR &&
					pObj.yCellL <= j && j < pObj.yCellR &&
					pObj.zCellL <= k && k < pObj.zCellR) continue;

				INT32 n = h1 * i + h2 * j + h3 * k;
				n = n % m_rgTblEntry.size();
				if (n < 0) n += m_rgTblEntry.size();
				pObj.dwHashIndex = n;

				STableEntry * pC = GetFreeNodeOfCell(&m_pTblEntry[n]);
				pC->pObjEntry = pObj;
			}
		}
	}
	pObj.xCellL = xCellL;
	pObj.yCellL = yCellL;
	pObj.zCellL = zCellL;
	pObj.xCellR = xCellR;
	pObj.yCellR = yCellR;
	pObj.zCellR = zCellR;
}

template <typename T>
void CUniformGrid<T>::UpdateGrid()
{
	DWORD n = m_rgObjEntry.size();
	for (DWORD i=0; i<n; i++) {
		HashObjToCell(m_pObjEntry[n], m_pObjEntry[i].pBox.m_MinPW, m_pObjEntry[i].pBox.m_MaxPW);
	}

}

template <typename T>
void CUniformGrid<T>::FindObjectsInSameCell(DWORD dwId, std::vector<T*> & rgObj)/* Undo objs assigned bused = 1 */
{
	SObjectEntry & rO = m_pObjEntry[dwId];
	r0.bUsed = 1;

	INT32 nX = (r0.xCellR - r0.xCellL);
	INT32 nY = (r0.yCellR - r0.yCellL);
	INT32 nZ = (r0.zCellR - r0.zCellL);
	for (INT32 start=0, end=nX*nY*nZ; start < end; start++) {
		INT32 i = start/(nZ*nY);
		INT32 j = (start / nZ) % nY;
		INT32 k = start % nZ;

		DWORD n = ComputeHashTableIndex(i, j, k);
		STableEntry * p = m_rgTblEntry[n];
		do {
			SObjectEntry & r = *p->pObjEntry;
			if (!r.bUsed && 
				r.xCellL <= i && i < r.xCellR &&
				r.yCellL <= j && j < r.yCellR &&
				r.zCellL <= k && k < r.zCellR) {
					rgObj.push_back(p->pObjEntry);
					r.bUsed = 1;
			}
			p = p->pNext;
		} while (p);
	}
}

template <typename T>
void CUniformGrid<T>::DestroyGrid()
{
}



};


#endif


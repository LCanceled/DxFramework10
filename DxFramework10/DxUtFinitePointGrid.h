
#ifndef DXUTFINITEPOINTGRID_H
#define DXUTFINITEPOINTGRID_H

#include "DxUtInclude.h"
#include "DxUtArray.h"

namespace DxUt {

//TODO: make generic

template <typename T>
class CFinitePointGrid3F {
private:
	struct SEntry {
		SEntry() {}
		SEntry(Vector3F & _pt, T & _val):pt(_pt),val(_val) {}
		Vector3F pt;
		T val;
	};
	struct SCell {
		CArray<SEntry> points;
	};

	int m_iCellsX;
	int m_iCellsY;
	int m_iCellsZ;
	float m_CellSize;

	SCell * m_Cells;
	Vector3F m_CellOrigin;

	DWORD ComputeGridIndex(Vector3F & pt) {
		Vector3F gridPt(pt - m_CellOrigin);
		gridPt /= m_CellSize;

		int x = (int)floorf(gridPt.x);
		int y = (int)floorf(gridPt.y);
		int z = (int)floorf(gridPt.z);
		if (x < 0 || y < 0 || z < 0 || x >= m_iCellsX || y >= m_iCellsY || z >= m_iCellsZ) {
			DebugBreak();
		}
		return (DWORD)(z * m_iCellsY * m_iCellsX + y * m_iCellsX + x);
	}
public:
	CFinitePointGrid3F():m_Cells(0) {}
	//~CFinitePointGrid3F() {}

	void CreateGrid(DWORD iCellsX, DWORD iCellsY, DWORD iCellsZ, float fCellSize, Vector3F & minGridPt) {
		m_iCellsX = (int)iCellsX;
		m_iCellsY = (int)iCellsY;
		m_iCellsZ = (int)iCellsZ;
		m_CellSize = fCellSize;
		m_CellOrigin = minGridPt;
		m_Cells = new SCell[iCellsX*iCellsY*iCellsZ];
	}

	void AddPoint(Vector3F & pt, T & val) {
		m_Cells[ComputeGridIndex(pt)].points.PushBack(SEntry(pt, val));
	}

	bool PointInGrid(Vector3F & pt, T * val) {
		int gridId = ComputeGridIndex(pt);
		CArray<SEntry> & points = m_Cells[gridId].points;
		SEntry * pPoints = points.GetData();
		for (DWORD i=0, end=points.GetSize(); i<end; i++) {
			if (pPoints[i].pt == pt) {
				*val = pPoints[i].val;
				return 1;
			}
		}
		val = 0;
		return 0;
	}

	/* */
	void Reset() {
		for (DWORD i=0, end=(DWORD)(m_iCellsX*m_iCellsY*m_iCellsZ); i<end; i++) {
			m_Cells[i].points.Resize(0);
		}
	}

	void DestroyGrid() {
		if (m_Cells) {
			for (DWORD i=0, end=(DWORD)(m_iCellsX*m_iCellsY*m_iCellsZ); i<end; i++) {
				m_Cells[i].points.Clear();
			}
			delete[] m_Cells;
			m_Cells = 0;
		}
	}
};


};


#endif






/*
struct SCellKey {
		int x,y,z; 
	};
	struct SCellVal {
		CArray<Vector3F> verts;
	};
	struct SHashFunc {
		int Hash(SCellKey & pt) {
			static const int h1 = 0x8da6b343;
			static const int h2 = 0xd8163841;
			static const int h3 = 0xcb1ab31f;
			int n = h1 * pt.x + h2 * pt.y + h3 * pt.z;
		}
	};
	struct SCmpFunc {
		bool Cmp(SCellKey & c1, SCellKey & c2) {
			return c1.x == c2.x && c1.y == c2.y && c1.z == c2.z;
		}
	};
*/


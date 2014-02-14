
#ifndef CONVEXHULL_H
#define CONVEXHULL_H

#include "DxUtInclude.h"
#include "DxUtArray.h"

namespace DxUt {

struct SConvexHull2F {
	Vector2F pos2F;
};

/* Template class T must inherit from SConvexHull2F */ 
/* Returns a list of points on the convex hull in counter-clockwise order */
template <typename T>
void ComputeConvexHull2D(CArray<T> & rgPtIn, CArray<T> & rgPtOut)
{
	struct ConvexHullHelper {
		float Cross(const Vector2F & O, const Vector2F & A, const Vector2F & B) {
			return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
		}
		static int cmp(const void * a, const void * b) {
			Vector2F & p1 = ((T*)a)->pos2F;
			Vector2F & p2 = ((T*)b)->pos2F;
			return 1 - (p1.x < p2.x || (p1.x == p2.x && p1.y < p2.y))*(2);
		}
	};	
	static ConvexHullHelper helper;

	/* Allocations */
	int n = rgPtIn.GetSize(), k = 0;
	rgPtOut.Resize(2*n);
	T * pPtIn = rgPtIn.GetData();
	T * pPtOut = rgPtOut.GetData();

	/* Sort the points */
	qsort(rgPtIn.GetData(), rgPtIn.GetSize(), sizeof(T), helper.cmp);

	/* Lower convex hull */
	for (int i = 0; i < n; i++) {	
		while (k >= 2 && helper.Cross(pPtOut[k-2].pos2F, pPtOut[k-1].pos2F, pPtIn[i].pos2F) <= 0) k--;
		memcpy(&pPtOut[k++], &pPtIn[i], sizeof(T));
	}
	/* Upper convex hull */
	for (int i = n-2, t = k+1; i >= 0; i--) {
		while (k >= t && helper.Cross(pPtOut[k-2].pos2F, pPtOut[k-1].pos2F, pPtIn[i].pos2F) <= 0) k--;
		memcpy(&pPtOut[k++], &pPtIn[i], sizeof(T));
	}
 
	rgPtOut.Resize(k);
}


};


#endif



#include "DxUtInclude.h"

namespace DxUt {

/* Assume that a cell corner is at zero */
/* 0 < x < 1, 0 < y < 1, 0 < z < 1 */
__forceinline double InterpolateTrilinear(
	double x, double y, double z,
	double V000, double V100, double V010, double V110, double V001, double V101, double V011, double V111)
{
	return 
		V000 * (1. - x) * (1. - y) * (1. - z) +
		V100 * x * (1. - y) * (1. - z) + 
		V010 * (1. - x) * y * (1. - z) + 
		V001 * (1. - x) * (1. - y) * z +
		V101 * x * (1. - y) * z + 
		V011 * (1. - x) * y * z + 
		V110 * x * y * (1. - z) + 
		V111 * x * y * z;
}

__forceinline double InterpolateTrilinear(
	double x, double y, double z,
	double cellX, double cellY, double cellZ,
	double V000, double V100, double V010, double V011, double V001, double V101, double V110, double V111)
{
	x =	fracf(x/cellX);
	y = fracf(y/cellY);
	z = fracf(z/cellZ);

	return 
		V000 * (1. - x) * (1. - y) * (1. - z) +
		V100 * x * (1. - y) * (1. - z) + 
		V010 * (1. - x) * y * (1. - z) + 
		V001 * (1. - x) * (1. - y) * z +
		V101 * x * (1. - y) * z + 
		V011 * (1. - x) * y * z + 
		V110 * x * y * (1. - z) + 
		V111 * x * y * z;
}











};
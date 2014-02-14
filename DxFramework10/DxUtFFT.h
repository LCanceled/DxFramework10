
#ifndef DXUTFFT_H
#define DXUTFFT_H

/*-------------------------------------------------------------------------
   Perform a 2D FFT inplace given a complex 2D array
   The direction dir, 1 for forward, -1 for reverse
   The size of the array (nx,ny)
   Return false if there are memory problems or
      the dimensions are not powers of 2
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

namespace DxUt {

struct COMPLEX {
	double real;
	double imag;
};

/*-------------------------------------------------------------------------
	Calculate the closest but lower power of two of a number
	twopm = 2**m <= n
	Return TRUE if 2**m == n
*/
int Powerof2(int n,int *m,int *twopm);

/*-------------------------------------------------------------------------
   This computes an in-place complex-to-complex FFT
   x and y are the real and imaginary arrays of 2^m points.
   dir =  1 gives forward transform
   dir = -1 gives reverse transform

     Formula: forward
                  N-1
                  ---
              1   \          - j k 2 pi n / N
      X(n) = ---   >   x(k) e                    = forward transform
              N   /                                n=0..N-1
                  ---
                  k=0

      Formula: reverse
                  N-1
                  ---
                  \          j k 2 pi n / N
      X(n) =       >   x(k) e                    = forward transform
                  /                                n=0..N-1
                  ---
                  k=0
*/
int FFT(int dir,int m,double *x,double *y);

int FFT2D(COMPLEX **c,int nx,int ny,int dir);


}


#endif
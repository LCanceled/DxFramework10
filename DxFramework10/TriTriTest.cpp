
#include "TriTriTest.h"
#include <windows.h>

/* some 3D macros */

#define SET(v1, v2)				\
				v1[0] = v2[0];	\
				v1[1] = v2[1];	\
				v1[2] = v2[2];

#define CROSS(dest,v1,v2)                       \
               dest[0]=v1[1]*v2[2]-v1[2]*v2[1]; \
               dest[1]=v1[2]*v2[0]-v1[0]*v2[2]; \
               dest[2]=v1[0]*v2[1]-v1[1]*v2[0];

#define DOT(v1,v2) (v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2])
 


#define SUB(dest,v1,v2) dest[0]=v1[0]-v2[0]; \
                        dest[1]=v1[1]-v2[1]; \
                        dest[2]=v1[2]-v2[2]; 


#define SCALAR(dest,alpha,v) dest[0] = alpha * v[0]; \
                             dest[1] = alpha * v[1]; \
                             dest[2] = alpha * v[2];



#define CHECK_MIN_MAX(p1,q1,r1,p2,q2,r2) {\
  SUB(v1,p2,q1)\
  SUB(v2,p1,q1)\
  CROSS(N1,v1,v2)\
  SUB(v1,q2,q1)\
  if (DOT(v1,N1) > 0.0f) return 0;\
  SUB(v1,p2,p1)\
  SUB(v2,r1,p1)\
  CROSS(N1,v1,v2)\
  SUB(v1,r2,p1) \
  if (DOT(v1,N1) > 0.0f) return 0;\
  else return 1; }



/* Permutation in a canonical form of T2's vertices */

#define TRI_TRI_3D(p1,q1,r1,p2,q2,r2,dp2,dq2,dr2) { \
  if (dp2 > 0.0f) { \
     if (dq2 > 0.0f) CHECK_MIN_MAX(p1,r1,q1,r2,p2,q2) \
     else if (dr2 > 0.0f) CHECK_MIN_MAX(p1,r1,q1,q2,r2,p2)\
     else CHECK_MIN_MAX(p1,q1,r1,p2,q2,r2) }\
  else if (dp2 < 0.0f) { \
    if (dq2 < 0.0f) CHECK_MIN_MAX(p1,q1,r1,r2,p2,q2)\
    else if (dr2 < 0.0f) CHECK_MIN_MAX(p1,q1,r1,q2,r2,p2)\
    else CHECK_MIN_MAX(p1,r1,q1,p2,q2,r2)\
  } else { \
    if (dq2 < 0.0f) { \
      if (dr2 >= 0.0f)  CHECK_MIN_MAX(p1,r1,q1,q2,r2,p2)\
      else CHECK_MIN_MAX(p1,q1,r1,p2,q2,r2)\
    } \
    else if (dq2 > 0.0f) { \
      if (dr2 > 0.0f) CHECK_MIN_MAX(p1,r1,q1,p2,q2,r2)\
      else  CHECK_MIN_MAX(p1,q1,r1,q2,r2,p2)\
    } \
    else  { \
      if (dr2 > 0.0f) CHECK_MIN_MAX(p1,q1,r1,r2,p2,q2)\
      else if (dr2 < 0.0f) CHECK_MIN_MAX(p1,r1,q1,r2,p2,q2)\
      else return coplanar_tri_tri3d(p1,q1,r1,p2,q2,r2,N1,N2);\
     }}}
  


/*
*
*  Three-dimensional Triangle-Triangle Overlap Test
*
*/


int tri_tri_overlap_test_3d(double p1[3], double q1[3], double r1[3], 

			    double p2[3], double q2[3], double r2[3])
{
  double dp1, dq1, dr1, dp2, dq2, dr2;
  double v1[3], v2[3];
  double N1[3], N2[3]; 
  
  /* Compute distance signs  of p1, q1 and r1 to the plane of
     triangle(p2,q2,r2) */


  SUB(v1,p2,r2)
  SUB(v2,q2,r2)
  CROSS(N2,v1,v2)

  SUB(v1,p1,r2)
  dp1 = DOT(v1,N2);
  SUB(v1,q1,r2)
  dq1 = DOT(v1,N2);
  SUB(v1,r1,r2)
  dr1 = DOT(v1,N2);
  
  if (((dp1 * dq1) > 0.0f) && ((dp1 * dr1) > 0.0f))  return 0; 

  /* Compute distance signs  of p2, q2 and r2 to the plane of
     triangle(p1,q1,r1) */

  
  SUB(v1,q1,p1)
  SUB(v2,r1,p1)
  CROSS(N1,v1,v2)

  SUB(v1,p2,r1)
  dp2 = DOT(v1,N1);
  SUB(v1,q2,r1)
  dq2 = DOT(v1,N1);
  SUB(v1,r2,r1)
  dr2 = DOT(v1,N1);
  
  if (((dp2 * dq2) > 0.0f) && ((dp2 * dr2) > 0.0f)) return 0;

  /* Permutation in a canonical form of T1's vertices */


  if (dp1 > 0.0f) {
    if (dq1 > 0.0f) TRI_TRI_3D(r1,p1,q1,p2,r2,q2,dp2,dr2,dq2)
    else if (dr1 > 0.0f) TRI_TRI_3D(q1,r1,p1,p2,r2,q2,dp2,dr2,dq2)	
    else TRI_TRI_3D(p1,q1,r1,p2,q2,r2,dp2,dq2,dr2)
  } else if (dp1 < 0.0f) {
    if (dq1 < 0.0f) TRI_TRI_3D(r1,p1,q1,p2,q2,r2,dp2,dq2,dr2)
    else if (dr1 < 0.0f) TRI_TRI_3D(q1,r1,p1,p2,q2,r2,dp2,dq2,dr2)
    else TRI_TRI_3D(p1,q1,r1,p2,r2,q2,dp2,dr2,dq2)
  } else {
    if (dq1 < 0.0f) {
      if (dr1 >= 0.0f) TRI_TRI_3D(q1,r1,p1,p2,r2,q2,dp2,dr2,dq2)
      else TRI_TRI_3D(p1,q1,r1,p2,q2,r2,dp2,dq2,dr2)
    }
    else if (dq1 > 0.0f) {
      if (dr1 > 0.0f) TRI_TRI_3D(p1,q1,r1,p2,r2,q2,dp2,dr2,dq2)
      else TRI_TRI_3D(q1,r1,p1,p2,q2,r2,dp2,dq2,dr2)
    }
    else  {
      if (dr1 > 0.0f) TRI_TRI_3D(r1,p1,q1,p2,q2,r2,dp2,dq2,dr2)
      else if (dr1 < 0.0f) TRI_TRI_3D(r1,p1,q1,p2,r2,q2,dp2,dr2,dq2)
      else return coplanar_tri_tri3d(p1,q1,r1,p2,q2,r2,N1,N2);
    }
  }
};



int coplanar_tri_tri3d(double p1[3], double q1[3], double r1[3],
		       double p2[3], double q2[3], double r2[3],
		       double normal_1[3], double normal_2[3]){
  
  double P1[2],Q1[2],R1[2];
  double P2[2],Q2[2],R2[2];

  double n_x, n_y, n_z;

  n_x = ((normal_1[0]<0)?-normal_1[0]:normal_1[0]);
  n_y = ((normal_1[1]<0)?-normal_1[1]:normal_1[1]);
  n_z = ((normal_1[2]<0)?-normal_1[2]:normal_1[2]);


  /* Projection of the triangles in 3D onto 2D such that the area of
     the projection is maximized. */


  if (( n_x > n_z ) && ( n_x >= n_y )) {
    // Project onto plane YZ

      P1[0] = q1[2]; P1[1] = q1[1];
      Q1[0] = p1[2]; Q1[1] = p1[1];
      R1[0] = r1[2]; R1[1] = r1[1]; 
    
      P2[0] = q2[2]; P2[1] = q2[1];
      Q2[0] = p2[2]; Q2[1] = p2[1];
      R2[0] = r2[2]; R2[1] = r2[1]; 

  } else if (( n_y > n_z ) && ( n_y >= n_x )) {
    // Project onto plane XZ

    P1[0] = q1[0]; P1[1] = q1[2];
    Q1[0] = p1[0]; Q1[1] = p1[2];
    R1[0] = r1[0]; R1[1] = r1[2]; 
 
    P2[0] = q2[0]; P2[1] = q2[2];
    Q2[0] = p2[0]; Q2[1] = p2[2];
    R2[0] = r2[0]; R2[1] = r2[2]; 
    
  } else {
    // Project onto plane XY

    P1[0] = p1[0]; P1[1] = p1[1]; 
    Q1[0] = q1[0]; Q1[1] = q1[1]; 
    R1[0] = r1[0]; R1[1] = r1[1]; 
    
    P2[0] = p2[0]; P2[1] = p2[1]; 
    Q2[0] = q2[0]; Q2[1] = q2[1]; 
    R2[0] = r2[0]; R2[1] = r2[1]; 
  }

  return tri_tri_overlap_test_2d(P1,Q1,R1,P2,Q2,R2);
    
};



/*
*                                                                
*  Three-dimensional Triangle-Triangle Intersection              
*
*/

/*
   This macro is called when the triangles surely intersect
   It constructs the segment of intersection of the two triangles
   if they are not coplanar.
*/

#define M1(x) ((int)(*(#x)))
#define M2(x, y) (((x + y != 'p' + 'q')*(1 - (x < y)*2)*(x - y)))
#define M3(x, y) ((1 - (x + y == 'p' + 'r')*2)*(1 - (x < y)*2))

#define CONSTRUCT_INTERSECTION(p1,q1,r1,p2,q2,r2) { \
   SUB(v1,q1,p1) \
  SUB(v2,r2,p1) \
  CROSS(N,v1,v2) \
  SUB(v,p2,p1) \
  if (DOT(v,N) > 0.0f) {\
    SUB(v1,r1,p1) \
    CROSS(N,v1,v2) \
    if (DOT(v,N) <= 0.0f) { \
      SUB(v2,q2,p1) \
      CROSS(N,v1,v2) \
      if (DOT(v,N) > 0.0f) { \
	SUB(v1,p1,p2) \
	SUB(v2,p1,r1) \
	alpha = DOT(v1,N2) / DOT(v2,N2); \
	SCALAR(v1,alpha,v2) \
	a = M1(p1), b = M1(r1); \
	eIdx[0] = M2(a, b); \
	SUB(source,p1,v1) \
	SUB(v1,p2,p1) \
	SUB(v2,p2,r2) \
	alpha = DOT(v1,N1) / DOT(v2,N1); \
	SCALAR(v1,alpha,v2) \
	SUB(target,p2,v1) \
	a = M1(p2), b = M1(r2); \
	eIdx[1] = M2(a, b); \
	alpha = M3(a, b); \
	SCALAR(v1, alpha, v2) \
	CROSS(v2, v1, N2) \
	dwType = 2; \
	dwType |= (eIdx[0] << 8) | (eIdx[1] << 16); \
	return 1; \
      } else { \
	SUB(v1,p2,p1) \
	SUB(v2,p2,q2) \
	alpha = DOT(v1,N1) / DOT(v2,N1); \
	SCALAR(v1,alpha,v2) \
	SUB(source,p2,v1) \
	a = M1(p2), b = M1(q2); \
	eIdx[0] = M2(a, b); \
	SUB(v1,p2,p1) \
	SUB(v2,p2,r2) \
	alpha = DOT(v1,N1) / DOT(v2,N1); \
	SCALAR(v1,alpha,v2) \
	SUB(target,p2,v1) \
	a = M1(p2), b = M1(r2); \
	eIdx[1] = M2(a, b); \
	alpha = M3(a, b); \
	SCALAR(v1, alpha, v2) \
	CROSS(v2, v1, N2) \
	dwType = 0; \
	dwType |= (eIdx[0] << 8) | (eIdx[1] << 16); \
	return 1; \
      } \
    } else { \
      return 0; \
    } \
  } else { \
    SUB(v2,q2,p1) \
    CROSS(N,v1,v2) \
    if (DOT(v,N) < 0.0f) { \
      return 0; \
    } else { \
      SUB(v1,r1,p1) \
      CROSS(N,v1,v2) \
      if (DOT(v,N) >= 0.0f) { \
	SUB(v1,p1,p2) \
	SUB(v2,p1,r1) \
	alpha = DOT(v1,N2) / DOT(v2,N2); \
	SCALAR(v1,alpha,v2) \
	SUB(source,p1,v1) \
	a = M1(p1), b = M1(r1); \
	eIdx[0] = M2(a, b); \
	SUB(v1,p1,p2) \
	SUB(v2,p1,q1) \
	alpha = DOT(v1,N2) / DOT(v2,N2); \
	SCALAR(v1,alpha,v2) \
	SUB(target,p1,v1) \
	a = M1(p1), b = M1(q1); \
	eIdx[1] = M2(a, b); \
	alpha = M3(a, b); \
	SCALAR(v1, alpha, v2) \
	CROSS(v2, v1, N1) \
	dwType = 1; \
	dwType |= (eIdx[0] << 8) | (eIdx[1] << 16); \
	return 1; \
      } else { \
	SUB(v1,p1,p2) \
	SUB(v2,p1,q1) \
	alpha = DOT(v1,N2) / DOT(v2,N2); \
	SCALAR(v1,alpha,v2) \
	SUB(source,p1,v1) \
	a = M1(p1), b = M1(q1); \
	eIdx[0] = M2(a, b); \
	SUB(v1,p2,p1) \
	SUB(v2,p2,q2) \
	alpha = DOT(v1,N1) / DOT(v2,N1); \
	SCALAR(v1,alpha,v2) \
	SUB(target,p2,v1) \
	a = M1(p2), b = M1(q2); \
	eIdx[1] = M2(a, b); \
	alpha = M3(a, b); \
	SCALAR(v1, alpha, v2) \
	CROSS(v2, v1, N2) \
	dwType = 2; \
	dwType |= (eIdx[0] << 8) | (eIdx[1] << 16); \
	return 1; \
      }}}}

#define TRI_TRI_INTER_3D(p1,q1,r1,p2,q2,r2,dp2,dq2,dr2) { \
  if (dp2 > 0.0f) { \
     if (dq2 > 0.0f) CONSTRUCT_INTERSECTION(p1,r1,q1,r2,p2,q2) \
     else if (dr2 > 0.0f) CONSTRUCT_INTERSECTION(p1,r1,q1,q2,r2,p2)\
     else CONSTRUCT_INTERSECTION(p1,q1,r1,p2,q2,r2) }\
  else if (dp2 < 0.0f) { \
    if (dq2 < 0.0f) CONSTRUCT_INTERSECTION(p1,q1,r1,r2,p2,q2)\
    else if (dr2 < 0.0f) CONSTRUCT_INTERSECTION(p1,q1,r1,q2,r2,p2)\
    else CONSTRUCT_INTERSECTION(p1,r1,q1,p2,q2,r2)\
  } else { \
    if (dq2 < 0.0f) { \
      if (dr2 >= 0.0f)  CONSTRUCT_INTERSECTION(p1,r1,q1,q2,r2,p2)\
      else CONSTRUCT_INTERSECTION(p1,q1,r1,p2,q2,r2)\
    } \
    else if (dq2 > 0.0f) { \
      if (dr2 > 0.0f) CONSTRUCT_INTERSECTION(p1,r1,q1,p2,q2,r2)\
      else  CONSTRUCT_INTERSECTION(p1,q1,r1,q2,r2,p2)\
    } \
    else  { \
      if (dr2 > 0.0f) CONSTRUCT_INTERSECTION(p1,q1,r1,r2,p2,q2)\
      else if (dr2 < 0.0f) CONSTRUCT_INTERSECTION(p1,r1,q1,r2,p2,q2)\
      else { \
		dwType = 3; \
       	*coplanar = 1; \
	return coplanar_tri_tri3d(p1,q1,r1,p2,q2,r2,N1,N2);\
     } \
  }} }
  

/*
   The following version computes the segment of intersection of the
   two triangles if it exists. 
   coplanar returns whether the triangles are coplanar
   source and target are the endpoints of the line segment of intersection 
*/

#define NORMALIZE(N)								\
{													\
	alpha = 1.0/sqrt(N[0]*N[0]+N[1]*N[1]+N[2]*N[2]);\
	N[0] *= alpha, N[1] *= alpha, N[2] *= alpha;	\
}	

int tri_tri_intersection_test_3d(double p1[3], double q1[3], double r1[3], double p2[3], double q2[3], double r2[3],
	int * coplanar, double source[3], double target[3], double pl1[4], double pl2[4], unsigned long & dwType, double iLine[3])
				 
{
  double dp1, dq1, dr1, dp2, dq2, dr2;
  double v1[3], v2[3], v[3];
  double N1[3], N2[3], N[3];
  double alpha=0;

  // Compute distance signs  of p1, q1 and r1 
  // to the plane of triangle(p2,q2,r2)


  SUB(v1,p2,r2)
  SUB(v2,q2,r2)
  CROSS(N2,v1,v2)
  pl2[0] = N2[0];
  pl2[1] = N2[1];
  pl2[2] = N2[2];
  pl2[3] = -DOT(N2,r2);

  SUB(v1,p1,r2)
  dp1 = DOT(v1,N2);
  SUB(v1,q1,r2)
  dq1 = DOT(v1,N2);
  SUB(v1,r1,r2)
  dr1 = DOT(v1,N2);
  
  if (((dp1 * dq1) > 0.0f) && ((dp1 * dr1) > 0.0f))  return 0; 

  // Compute distance signs  of p2, q2 and r2 
  // to the plane of triangle(p1,q1,r1)

  
  SUB(v1,q1,p1)
  SUB(v2,r1,p1)
  CROSS(N1,v1,v2)
  pl1[0] = N1[0];
  pl1[1] = N1[1];
  pl1[2] = N1[2];
  pl1[3] = -DOT(N1,p1);

  SUB(v1,p2,r1)
  dp2 = DOT(v1,N1);
  SUB(v1,q2,r1)
  dq2 = DOT(v1,N1);
  SUB(v1,r2,r1)
  dr2 = DOT(v1,N1);
  
  if (((dp2 * dq2) > 0.0f) && ((dp2 * dr2) > 0.0f)) return 0;

  // Permutation in a canonical form of T1's vertices

  CROSS(iLine, N1, N2);
  DWORD eIdx[2] = {0};
  DWORD ix = 0;
  double coeps = 0;//1e-5;
  int a=0, b=0;
  if (dp1 > 0.0f) {
    if (dq1 > 0.0f) TRI_TRI_INTER_3D(r1,p1,q1,p2,r2,q2,dp2,dr2,dq2)
    else if (dr1 > 0.0f) TRI_TRI_INTER_3D(q1,r1,p1,p2,r2,q2,dp2,dr2,dq2)
    else TRI_TRI_INTER_3D(p1,q1,r1,p2,q2,r2,dp2,dq2,dr2)
  } else if (dp1 < 0.0f) {
    if (dq1 < 0.0f) TRI_TRI_INTER_3D(r1,p1,q1,p2,q2,r2,dp2,dq2,dr2)
    else if (dr1 < 0.0f) TRI_TRI_INTER_3D(q1,r1,p1,p2,q2,r2,dp2,dq2,dr2)
    else TRI_TRI_INTER_3D(p1,q1,r1,p2,r2,q2,dp2,dr2,dq2)
  } else {
    if (dq1 < 0.0f) {
      if (dr1 >= 0.0f) TRI_TRI_INTER_3D(q1,r1,p1,p2,r2,q2,dp2,dr2,dq2)
      else TRI_TRI_INTER_3D(p1,q1,r1,p2,q2,r2,dp2,dq2,dr2)
    }
    else if (dq1 > 0.0f) {
      if (dr1 > 0.0f) TRI_TRI_INTER_3D(p1,q1,r1,p2,r2,q2,dp2,dr2,dq2)
      else TRI_TRI_INTER_3D(q1,r1,p1,p2,q2,r2,dp2,dq2,dr2)
    }
    else  {
      if (dr1 > 0.0f) TRI_TRI_INTER_3D(r1,p1,q1,p2,q2,r2,dp2,dq2,dr2)
      else if (dr1 < 0.0f) TRI_TRI_INTER_3D(r1,p1,q1,p2,r2,q2,dp2,dr2,dq2)
      else {
	// triangles are co-planar

	*coplanar = 1;
	dwType = 3;
	return coplanar_tri_tri3d(p1,q1,r1,p2,q2,r2,N1,N2);
      }
    }
	}
};



/*
*
*  Two dimensional Triangle-Triangle Overlap Test    
*
*/


/* some 2D macros */

#define ORIENT_2D(a, b, c)  ((a[0]-c[0])*(b[1]-c[1])-(a[1]-c[1])*(b[0]-c[0]))


#define INTERSECTION_TEST_VERTEX(P1, Q1, R1, P2, Q2, R2) {\
  if (ORIENT_2D(R2,P2,Q1) >= 0.0f)\
    if (ORIENT_2D(R2,Q2,Q1) <= 0.0f)\
      if (ORIENT_2D(P1,P2,Q1) > 0.0f) {\
	if (ORIENT_2D(P1,Q2,Q1) <= 0.0f) return 1; \
	else return 0;} else {\
	if (ORIENT_2D(P1,P2,R1) >= 0.0f)\
	  if (ORIENT_2D(Q1,R1,P2) >= 0.0f) return 1; \
	  else return 0;\
	else return 0;}\
    else \
      if (ORIENT_2D(P1,Q2,Q1) <= 0.0f)\
	if (ORIENT_2D(R2,Q2,R1) <= 0.0f)\
	  if (ORIENT_2D(Q1,R1,Q2) >= 0.0f) return 1; \
	  else return 0;\
	else return 0;\
      else return 0;\
  else\
    if (ORIENT_2D(R2,P2,R1) >= 0.0f) \
      if (ORIENT_2D(Q1,R1,R2) >= 0.0f)\
	if (ORIENT_2D(P1,P2,R1) >= 0.0f) return 1;\
	else return 0;\
      else \
	if (ORIENT_2D(Q1,R1,Q2) >= 0.0f) {\
	  if (ORIENT_2D(R2,R1,Q2) >= 0.0f) return 1; \
	  else return 0; }\
	else return 0; \
    else  return 0; \
 };



#define INTERSECTION_TEST_EDGE(P1, Q1, R1, P2, Q2, R2) { \
  if (ORIENT_2D(R2,P2,Q1) >= 0.0f) {\
    if (ORIENT_2D(P1,P2,Q1) >= 0.0f) { \
        if (ORIENT_2D(P1,Q1,R2) >= 0.0f) return 1; \
        else return 0;} else { \
      if (ORIENT_2D(Q1,R1,P2) >= 0.0f){ \
	if (ORIENT_2D(R1,P1,P2) >= 0.0f) return 1; else return 0;} \
      else return 0; } \
  } else {\
    if (ORIENT_2D(R2,P2,R1) >= 0.0f) {\
      if (ORIENT_2D(P1,P2,R1) >= 0.0f) {\
	if (ORIENT_2D(P1,R1,R2) >= 0.0f) return 1;  \
	else {\
	  if (ORIENT_2D(Q1,R1,R2) >= 0.0f) return 1; else return 0;}}\
      else  return 0; }\
    else return 0; }}



int ccw_tri_tri_intersection_2d(double p1[2], double q1[2], double r1[2], 
				double p2[2], double q2[2], double r2[2]) {
  if ( ORIENT_2D(p2,q2,p1) >= 0.0f ) {
    if ( ORIENT_2D(q2,r2,p1) >= 0.0f ) {
      if ( ORIENT_2D(r2,p2,p1) >= 0.0f ) return 1;
      else INTERSECTION_TEST_EDGE(p1,q1,r1,p2,q2,r2)
    } else {  
      if ( ORIENT_2D(r2,p2,p1) >= 0.0f ) 
	INTERSECTION_TEST_EDGE(p1,q1,r1,r2,p2,q2)
      else INTERSECTION_TEST_VERTEX(p1,q1,r1,p2,q2,r2)}}
  else {
    if ( ORIENT_2D(q2,r2,p1) >= 0.0f ) {
      if ( ORIENT_2D(r2,p2,p1) >= 0.0f ) 
	INTERSECTION_TEST_EDGE(p1,q1,r1,q2,r2,p2)
      else  INTERSECTION_TEST_VERTEX(p1,q1,r1,q2,r2,p2)}
    else INTERSECTION_TEST_VERTEX(p1,q1,r1,r2,p2,q2)}
};


int tri_tri_overlap_test_2d(double p1[2], double q1[2], double r1[2], 
			    double p2[2], double q2[2], double r2[2]) {
  if ( ORIENT_2D(p1,q1,r1) < 0.0f )
    if ( ORIENT_2D(p2,q2,r2) < 0.0f )
      return ccw_tri_tri_intersection_2d(p1,r1,q1,p2,r2,q2);
    else
      return ccw_tri_tri_intersection_2d(p1,r1,q1,p2,q2,r2);
  else
    if ( ORIENT_2D(p2,q2,r2) < 0.0f )
      return ccw_tri_tri_intersection_2d(p1,q1,r1,p2,r2,q2);
    else
      return ccw_tri_tri_intersection_2d(p1,q1,r1,p2,q2,r2);

};

/*+++
   Copyright (c) Aerotech Inc., 1996 - 1999

   This header file is used with the Aerotech System libraries.
+++*/

#ifndef __AER_PROF_H__
#define __AER_PROF_H__

//typedef struct tagAER_PROFILE
//{
//   LONG  lPoint;
//   DWORD dwTime;
//   LONG  lVelStart;
//   LONG  lVelEnd;
//} AER_PROFILE;
//typedef AER_PROFILE  *PAER_PROFILE;
//
//typedef struct tagAER_PROFILE_PACKET
//{
//	 AXISMASK      mAxis;
//    AER_PROFILE   tProf[MAX_AXES];
//} AER_PROFILE_PACKET;
//typedef AER_PROFILE_PACKET *PAER_PROFILE_PACKET;

#ifdef __cplusplus
extern "C" {
#endif

//AERERR_CODE AER_DLLENTRY AerProfileLoadQueue( HAERCTRL hAerCtrl, AXISMASK mAxis,
//                                              PAER_PROFILE pProf );
//AERERR_CODE AER_DLLENTRY AerProfileLoadQueueEx( HAERCTRL hAerCtrl, AXISINDEX iAxis,
//                                                LONG lPos, DWORD dwTime,
//                                                LONG lVelStart, LONG lVelEnd );
//
//AERERR_CODE AER_DLLENTRY AerProfileLoadQueueMulti( HAERCTRL hAerCtrl, AXISMASK mAxis,
//                                                   PAER_PROFILE pProf );
//AERERR_CODE AER_DLLENTRY AerProfileLoadQueueMultiEx( HAERCTRL hAerCtrl,
//                                                     AXISMASK mAxis,
//                                                     PLONG plPosArray,
//                                                     PDWORD pdwTimeArray,
//                                                     PLONG plVelStartArray,
//                                                     PLONG plVelEndArray );
//
//AERERR_CODE AER_DLLENTRY AerProfileStartQueue( HAERCTRL hAerCtrl, AXISINDEX iAxis );
//AERERR_CODE AER_DLLENTRY AerProfileStartQueueMulti( HAERCTRL hAerCtrl, AXISMASK mAxis );
//
//AERERR_CODE AER_DLLENTRY AerProfileStopQueue( HAERCTRL hAerCtrl, AXISINDEX iAxis );
//AERERR_CODE AER_DLLENTRY AerProfileStopQueueMulti( HAERCTRL hAerCtrl, AXISMASK mAxis );
//
//AERERR_CODE AER_DLLENTRY AerProfileFlushQueue(HAERCTRL hAerCtrl, AXISINDEX iAxis );
//AERERR_CODE AER_DLLENTRY AerProfileFlushQueueMulti(HAERCTRL hAerCtrl, AXISMASK mAxis );
//
//// Backwards - compatable
//// AerProfileLoadGlobal is now AerProfileLoadQueueMulti
//AERERR_CODE AER_DLLENTRY AerProfileLoadGlobal( HAERCTRL hAerCtrl, AXISMASK mAxis,
//                                               PAER_PROFILE pProf );

AERERR_CODE AER_DLLENTRY AerProfileGetDepth( HAERCTRL hAerCtrl, PAOULONG pDepths);


/*********************** Ellipse defines and declarations ************/

#define NUM_APPROX_ITERATIONS  100

//	These are declared to keep track of the Quadrants

#define QUAD_ONE	1
#define QUAD_TWO	2
#define QUAD_THREE	3
#define QUAD_FOUR	4

//  These values are constants used in computing the Carlson Elliptical integral of the
//	second kind.  With the exception of SK_ERRTOL these values should not be changed
//	The SK_ERRTOL value can be changed to try and achieve a higher degree of precision
//	on the computation of the arc length.

#define SK_ERRTOL	0.0015 //0.05     // original
//#define SK_ERRTOL	0.000015 //0.05
#define SK_CONST_1	(3.0/14.0)
#define SK_CONST_2	(1.0/6.0)
#define SK_CONST_3	(9.0/22.0)
#define SK_CONST_4	(3.0/26.0)
#define SK_CONST_5	(0.25*SK_CONST_3)
#define SK_CONST_6	(1.5*SK_CONST_4)

//  These values are constants used in computing the Carlson Elliptical integral of the
//	first kind.  With the exception of FK_ERRTOL these values should not be changed
//	The FK_ERRTOL value can be changed to try and achieve a higher degree of precision
//	on the computation of the arc length.

#define FK_ERRTOL	0.0025 //0.08  // original
//#define FK_ERRTOL	0.000025 //0.08
#define FK_THIRD	(1.0/3.0)
#define FK_CONST_1	(1.0/24.0)
#define FK_CONST_2	0.1
#define FK_CONST_3	(3.0/44.0)
#define FK_CONST_4	(1.0/14.0)

//	This value is used to check if the coordinates
//	given are actually on the supposed ellipse

#define COORD_ERR_TOL 1.0e-9

DOUBLE AerGetCarlsonEllipseFirstKind(	DOUBLE x, DOUBLE y, DOUBLE z);

DOUBLE AerGetCarlsonEllipseSecondKind(	DOUBLE x, DOUBLE y, DOUBLE z);

DOUBLE AerGetEllipseCircumference(		DOUBLE semiMajorAxisNum,
										         DOUBLE semiMinorAxisNum);

DOUBLE AerGetBasicEllipseArcLength(		DOUBLE semiMajorAxisNum,
				         						DOUBLE semiMinorAxisNum,
							         			DOUBLE angleLength,
                                       WORD   a_greater_than_b);

AERERR_CODE AerGetEllipticalArcLength(	DOUBLE   dstart_x_coord,
										         DOUBLE   dstart_y_coord,
										         DOUBLE   dend_x,
										         DOUBLE   dend_y,
										         DOUBLE   dcenter_i,
										         DOUBLE   dcenter_j,
										         DOUBLE   dmajor_p,
										         DOUBLE   dminor_q,
										         WORD     wDirection,
										         WORD     wFull,
                                       PDOUBLE  pfdRadiusDistance);

WORD findQuad(DOUBLE dAngle);



#ifdef __cplusplus
}
#endif

#endif
// __AER_PROF_H__

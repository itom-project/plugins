/* ********************************************************************
Template for a camera / grabber plugin for the software itom
URL: http://lccv.ufal.br/
Copyright (C) 2016, Universidade Federal de Alagoas (UFAL), Brazil

You can use this template, use it in your plugins, modify it,
copy it and distribute it without any license restrictions.
*********************************************************************** */

#ifndef DICINTERPOLATION_H
#define DICINTERPOLATION_H

#include <stddef.h>
#ifdef NO_ITOM_BUILD
#include "retVal.h"
#else
#include "common/retVal.h"
#endif

#ifdef USEOPENMP
	#define USEOMP 1
#else
	#define USEOMP 0
#endif

struct MatT {
    int sizex;
    int sizey;
    float *data;

    MatT() : sizex(0), sizey(0), data(NULL) {}
};

// flag: Bit Dez   Func
//        0   1     calculate derivatives
//        1   2     keep image
//        2   4     reset image
//        3   8
//        4   16    interpolation type LoByte: Bi-Linear, Bi-Cubic, Bi-Quintic, Bi-Heptic
//        5   32    interpolation type HighByte
//        6   64    Reserved for other interpolation types
//        7   128   
//        8   256   Use Cuda if available
//        9   512   Column first, i.e. Matlab, memory ordering

template<typename _Tp> ito::RetVal doInterpolateLinear(const _Tp* inPtr, const int width, const int height, const int stepin, 
    const float* positions, const int numPos, float* outPtr, const int stepout, const int flags);

ito::RetVal doInterpolateCubicPts(const float *img, const int sizex, const int sizey, const int imgStep, const float *dx, const float *dy, const float *dxy, const float *positions, const int numPts, float *res, const int flags = 0);

ito::RetVal doCalcAMat(const float *img, const int sizex, const int sizey, const int imgStep, const int interpType, const int flags);
ito::RetVal doInterpolateCubic(const float *positions, const int numPts, float *res, const int flags);
ito::RetVal doInterpolateQuintic(const float *positions, const int numPts, float *res, const int flags);
ito::RetVal doInterpolateHeptic(const float *positions, const int numPos, float *res, const int flags);

#endif // DICINTERPOLATION_H
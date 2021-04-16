/* ********************************************************************
    Template for a camera / grabber plugin for the software itom
	URL: http://lccv.ufal.br/
    Copyright (C) 2016, Universidade Federal de Alagoas (UFAL), Brazil
    
    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#ifndef DICPRIVATE_H
#define DICPRIVATE_H

#ifdef NO_ITOM_BUILD
#else
#include "DataObject/dataobj.h"
#include "common/retVal.h"
#endif

struct cell {
    cell(double cx, double cy, double x0, double x1, double y0, double y1) :
        cx(cx), cy(cy), x0(x0), x1(x1), y0(y0), y1(y1) { memset(nodes, 0, 16 * sizeof(unsigned short)); }
    cell() : cx(0), cy(0), x0(0), x1(0), y0(0), y1(0) { memset(nodes, 0, 16 * sizeof(unsigned short)); }

    double cx;
    double cy;
    double x0;
    double x1;
    double y0;
    double y1;
    unsigned short nodes[16];
    ito::DataObject refInt;
};

// in dicInterpolation
ito::RetVal doInterpolate(ito::DataObject *inFieldPtr, ito::DataObject *positionsdPtr, ito::DataObject *outFieldPtr, const int algo, const int flag, const int useCuda=0);
void clearFilterCache(void);

// in dicInitialGuess
ito::RetVal doCalcInitialGuessFFTMScale(ito::DataObject *imgInRef, ito::DataObject *imgInDef, cell *thisCell, const int guessType, const float maxxy[2], const float minsize, ito::float64 estCenter[2], ito::float64 &correlCoeffRet);
ito::RetVal doCalcInitialGuessFFT(ito::DataObject *imgInRef, ito::DataObject *imgInDef, cell *thisCell, const int guessType, double estCenter[2], double &correlCoeffRet);
ito::RetVal doCalcInitialGuess(ito::DataObject *imgInRef, ito::DataObject *imgInDef, cell *thisCell, const int range[2], const int guessType, double estCenter[2], double &correlCoeffRet);

// in dicDisplacementSS
int doCalcCoord(cell *thisCell, cv::Mat *coeffs, ito::DataObject *pts);

// in dicFilterDef
ito::RetVal normalizeInt(ito::DataObject &dObjIn, ito::DataObject &dObjOut, double *intSqr = NULL);

#endif // DICPRIVATE_H
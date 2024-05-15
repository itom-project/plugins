/* ********************************************************************
Plugin "DIC" for itom software
URL: http://lccv.ufal.br/
Copyright (C) 2016, Universidade Federal de Alagoas (UFAL), Brazil

This file is part of a plugin for the measurement software itom.

This itom-plugin is free software; you can redistribute it and/or modify it
under the terms of the GNU Library General Public Licence as published by
the Free Software Foundation; either version 2 of the Licence, or (at
your option) any later version.

itom and its plugins are distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Library
General Public Licence for more details.

You should have received a copy of the GNU Library General Public License
along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */
#include "matlab/dicMatlab.h"
#include "dicInterpolation.h"
#include <thread>
#include "dicInterpolation.hu"

#ifdef USECUDA
    bool hasCuda = 1;
#else
    bool hasCuda = 0;
#endif
int NTHREADS = -1;

#ifdef WIN32
#include <Windows.h>

/*
BOOL WINAPI DllMain(
    _In_ HINSTANCE hinstDLL,
    _In_ DWORD     fdwReason,
    _In_ LPVOID    lpvReserved
)
{
}
*/
#endif

#if USECUDA
int cudaInitialized = 0;
std::vector<int> cudaDevices;
#endif

//----------------------------------------------------------------------------------------------------------------------------------
void determineThreadCount()
{
    // try to find out 'optimal' number of threads to use

    unsigned int nthreads = std::thread::hardware_concurrency();
    if (nthreads < 1)
        NTHREADS = 1;
    else
        NTHREADS = nthreads;

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
extern "C" DLLEXPORT int interpolateBiLi(const float *data, const int sizeX, const int sizeY, const float *pts, const int numPts, float *result, const int flags)
{
    int ret = 0;

    if (NTHREADS < 0)
    {
        determineThreadCount();
    }

    if (data == NULL || result == NULL)
        return -1;

    // interpolation type by flag does not agree with function call
    if ((flags & 112) != 0)
    {
        return -2;
    }

    // checking for cuda
    if (flags & 256)
    {
#ifdef USECUDA
        if (!cudaInitialized)
        {
            ito::RetVal retval = InitCudaDevice(cudaDevices);
            if (!retval.containsError())
                cudaInitialized = 1;
            else
                return -3;
        }

        ito::RetVal retval = h_interpolBiLi<float>(data, sizeX, sizeY, sizeY,
            pts, numPts, result, 3, flags | 512);
#else
        return -3;
#endif
    }
    else
    {
        ito::RetVal retval = doInterpolateLinear<float>(data, sizeX, sizeY, sizeY,
            pts, numPts, result, 3, flags | 512);
    }

    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------
extern "C" DLLEXPORT int interpolateBiCu(const float *data, const int sizeX, const int sizeY, const float *pts, const int numPts, float *result, const int flags)
{
    int ret = 0;

    if (NTHREADS < 0)
    {
        determineThreadCount();
    }

    if (data == NULL || result == NULL)
        return -1;

    // force bicubic interpolation
    int nflags = (flags & ~112) + 16;

    // checking for cuda
    if (nflags & 256)
    {
#ifdef USECUDA
        if (!cudaInitialized)
        {
            ito::RetVal retval = InitCudaDevice(cudaDevices);
            if (!retval.containsError())
                cudaInitialized = 1;
            else
                return -3;
        }

        ito::RetVal retval = h_interpolAMat(data, sizeX, sizeY, sizeY,
            pts, numPts, result, 1, nflags | 512);
#else
        return -3;
#endif
    }
    else
    {
        //ito::RetVal doInterpolateCubicPts(const float *img, const int sizex, const int sizey, const int imgStep, const float *positions, const int numPts, float *res, const int flags = 0);
        ito::RetVal retval = doCalcAMat(data, sizeX, sizeY, sizeY, 1, nflags | 512);
        retval += doInterpolateCubic(pts, numPts, result, nflags | 512);
    }

    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------
extern "C" DLLEXPORT int interpolate(const float *data, const int sizeX, const int sizeY, const float *pts, const int numPts, float *result, const int flags)
{
    int ret = 0;

    if (NTHREADS < 0)
    {
        determineThreadCount();
    }

    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------

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

#ifndef DICMATLAB
#define DICMATLAB

    #if defined (WIN32) && defined(BUILDLIBRARY)
        #define DLLEXPORT extern "C" __declspec(dllexport)
    #else
        #define DLLEXPORT extern "C"
    #endif

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

    DLLEXPORT int interpolateBiLi(const float *data, const int sizeX, const int sizeY, const float *pts, const int numPts, float *result, const int flags);
    DLLEXPORT int interpolateBiCu(const float *data, const int sizeX, const int sizeY, const float *pts, const int numPts, float *result, const int flags);
    DLLEXPORT int interpolate(const float *data, const int sizeX, const int sizeY, const float *pts, const int numPts, float *result, const int flags);

#endif // DICMATLAB

/* ********************************************************************
    Plugin "Ximea" for itom software
    URL: http://www.twip-os.com
    Copyright (C) 2013, twip optical solutions GmbH
    Copyright (C) 2018, Institut für Technische Optik, Universität Stuttgart

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

#ifndef XIMEAFUNCSIMPORT_H
#define XIMEAFUNCSIMPORT_H

#include "xiApi.h"
//#include "xiExt.h"

#if defined(USE_API_4_10)
    #include "m3Api.h"
#endif 


//#ifdef USE_API_3_16
//
//#define XI_RAW8 XI_MONO8
//#define XI_RAW16 XI_MONO16
//
//#endif

//pointer to camera functions
XI_RETURN (__cdecl *pxiGetNumberDevices)(OUT PDWORD pNumberDevices);
XI_RETURN (__cdecl *pxiOpenDevice)(IN DWORD DevId, OUT PHANDLE hDevice);
XI_RETURN (__cdecl *pxiCloseDevice)(IN HANDLE hDevice);
XI_RETURN (__cdecl *pxiStartAcquisition)(IN HANDLE hDevice);
XI_RETURN (__cdecl *pxiStopAcquisition)(IN HANDLE hDevice);
XI_RETURN (__cdecl *pxiGetImage)(IN HANDLE hDevice, IN DWORD timeout, OUT LPXI_IMG img);
XI_RETURN (__cdecl *pxiSetParam)(IN HANDLE hDevice, const char* prm, void* val, DWORD size, XI_PRM_TYPE type);
XI_RETURN (__cdecl *pxiGetParam)(IN HANDLE hDevice, const char* prm, void* val, DWORD * size, XI_PRM_TYPE * type);

#if defined(USE_API_4_10)
    MM40_RETURN (__cdecl *pUpdateFrameShading)(IN HANDLE hDevice, IN HANDLE hFieldBuffer, IN LPMMSHADING lpSahding);
    MM40_RETURN (__cdecl *pCalculateShading)(IN HANDLE hDevice, INOUT LPMMSHADING lpMMS, DWORD dwCX, DWORD dwCY, LPWORD pBlack, LPWORD pWhite );
    MM40_RETURN (__cdecl *pCalculateShadingRaw)(INOUT LPMMSHADING lpMMS, DWORD dwCX, DWORD dwCY, LPWORD pBlack, LPWORD pWhite ); //does not exist in new API
    MM40_RETURN (__cdecl *pInitializeShading)(IN HANDLE hDevice, INOUT LPMMSHADING lpMMS, DWORD dwCX, DWORD dwCY, WORD wOff, WORD wMul);
    MM40_RETURN (__cdecl *pProcessFrame)(IN HANDLE hDevice);
    //MM40_RETURN (__cdecl *pSetShadingRaw)(IN LPMMSHADING lpSahding); //does not exist in new API
#endif //!defined(USE_API_4_10) && !defined(USE_API_3_16)

#endif
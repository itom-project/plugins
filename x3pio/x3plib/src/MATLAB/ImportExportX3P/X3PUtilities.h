/***************************************************************************
 *   Copyright by Georg Wiora (NanoFocus AG) 2008                          *
 *                                                                         *
 *   This file is part of the openGPS (R)[TM] software library.            *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Lesser General Public License (LGPL)    *
 *   as published by the Free Software Foundation; either version 3 of     *
 *   the License, or (at your option) any later version.                   *
 *   for detail see the files "licence_LGPL-3.0.txt" and                   *
 *   "licence_GPL-3.0.txt".                                                *
 *                                                                         *
 *   openGPS is distributed in the hope that it will be useful,            *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        * 
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Lesser General Public License for more details.                   *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 *                                                                         *
 *   The name "openGPS" and the logo are registered as                     *
 *   European trade mark No. 006178354 for                                 *
 *   Physikalisch Technische Bundesanstalt (PTB)                           *
 *   http://www.ptb.de/                                                    *
 *                                                                         *
 *   More information about openGPS can be found at                        *
 *   http://www.opengps.eu/                                                *
 ***************************************************************************/

// Definitions for x3p utility functions
#ifndef _X3PUTILITIES_H
#define _X3PUTILITIES_H

#define SHARED_OPENGPS_LIBRARY
#define _UNICODE
#define UNICODE

#include <string>
#include <tchar.h>
#include "mex.h"
#include <opengps/cxx/opengps.hxx>
#include <opengps/iso5436_2.h>
#include <opengps/cxx/iso5436_2.hxx>
#include <opengps/cxx/iso5436_2_handle.hxx>
#include <opengps/cxx/iso5436_2_xsd.hxx>

#define OGPS_LICENSETEXT _T("openX3P is part of the openGPS project (see <a href=\"http://www.opengps.eu/\">www.opengps.eu</a> for more info)\n")

// Uri for vendor specific extension
#define OGPS_VEXT_URI _T("http://www.mathworks.com/x3p/version_1")

// convert wstring to matlab string.
mxArray *ConvertWtoMStr(const std::wstring &inp);
// Convert matlab string to wstring
std::wstring ConvertMtoWStr(const mxArray *inp);
// Get point info structure
mxArray *GetPointInfoStructure(OGPS_ISO5436_2Handle handle);
// Get the name and version from X3P dll
std::wstring GetX3P_Dll_ID(void);
// Get a properly formated timestamp for the current date
OpenGPS::String TimeStamp(void);

#endif

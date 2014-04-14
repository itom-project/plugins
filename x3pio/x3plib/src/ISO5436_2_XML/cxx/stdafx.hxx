/***************************************************************************
 *   Copyright by Johannes Herwig (NanoFocus AG) 2007                      *
 *   Copyright by Georg Wiora (NanoFocus AG) 2007                          *
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

/*! @file
 * Standard includes and defines.
 */

#ifndef _OPENGPS_STDAFX_HXX
#define _OPENGPS_STDAFX_HXX

#if !(defined linux | defined __APPLE__)
    #include <tchar.h>
#endif

#include "version.h"

using namespace OpenGPS;

#define _OPENGPS_BINFORMAT_INT16_SIZE 2
#define _OPENGPS_BINFORMAT_INT32_SIZE 4
#define _OPENGPS_BINFORMAT_FLOAT_SIZE 4
#define _OPENGPS_BINFORMAT_DOUBLE_SIZE 8

#define _OPENGPS_XSD_ISO5436_NAMESPACE _T("http://www.opengps.eu/2008/ISO5436_2")
#define _OPENGPS_XSD_ISO5436_LOCATION _T("http://www.opengps.eu/2008/ISO5436_2/ISO5436_2.xsd")

#define _OPENGPS_XSD_ISO5436_MAIN_PATH _T("main.xml")
#define _OPENGPS_XSD_ISO5436_MAIN_CHECKSUM_PATH _T("md5checksum.hex")

#define _OPENGPS_XSD_ISO5436_DATALINK_PATH _T("bindata/data.bin")
#define _OPENGPS_XSD_ISO5436_VALIDPOINTSLINK_PATH _T("bindata/valid.bin")

// TODO: Unix? / Mac?
#define _OPENGPS_ENV_OPENGPS_LOCATION _T("OPENGPS_LOCATION")
#define _OPENGPS_ISO5436_LOCATION _T("iso5436_2.xsd")

#ifdef _DEBUG

#include <assert.h>

#ifndef _ASSERT
#  define _ASSERT(x) assert(x)
#endif

#ifndef _VERIFY
#  define _VERIFY(func, retval) _ASSERT(func == retval)
#endif

#else /* _DEBUG */

#ifndef _ASSERT
#  define _ASSERT(x) ((void)0)
#endif

#ifndef _VERIFY
#  define _VERIFY(func, retval) func
#endif

#endif /* _DEBUG */

#define _OPENGPS_EXCEPTION_MESG(x) x

#if !_OPENGPS_CLEAN_BUILD_FLAG
  #define _OPENGPS_EXPERIMENTAL _T(" (Experimental Version!)")
#else
  #if !_OPENGPS_CLEAN_REVISION_FLAG
    #define _OPENGPS_EXPERIMENTAL _T(" (Experimental Version!)")
  #endif
#endif

#ifndef _OPENGPS_EXPERIMENTAL
  #define _OPENGPS_EXPERIMENTAL
#endif

/// Name of the program
#define _OPENGPS_NAME _T("openGPS ISO 5436-2 XML") _OPENGPS_EXPERIMENTAL
/// Short description of the library
#define _OPENGPS_DESCRIPTION _T("openGPS class library implementing an xml-version of ISO 5436-2 file format.")

#endif /* _OPENGPS_STDAFX_HXX */

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

/*! @addtogroup Cpp
   @{ */

/*! @file
 * Additional helper methods used with xsd/xml_schema types provided by
 * CodeSynthesis.
 */

#ifndef _OPENGPS_CXX_ISO5436_2_XSD_UTILS_HXX
#define _OPENGPS_CXX_ISO5436_2_XSD_UTILS_HXX

#include <xsd/cxx/version.hxx>

#if (XSD_INT_VERSION >= 3020000L)

// Explicitly instantiate char-based (not only wchar_t-based, which is done
// automatically) serialziation macros.
#include <xsd/cxx/tree/bits/literals.hxx>

#include <opengps/opengps.h>
#include <xsd/cxx/tree/date-time.hxx>
#include <ostream>

namespace xml_schema
{
   typedef ::xsd::cxx::tree::date_time< wchar_t, ::xsd::cxx::tree::simple_type < ::xsd::cxx::tree::type > > date_time;
}

/*!
 * Streams a date-time type to be used with std::cout.
 */
_OPENGPS_EXPORT std::basic_ostream<char>& operator<<(std::basic_ostream<char>& os, const xml_schema::date_time& datetime);

#ifdef _UNICODE
/*!
 * Streams a date-time type to an output string-stream.
 */
_OPENGPS_EXPORT std::basic_ostream<wchar_t>& operator<<(std::basic_ostream<wchar_t>& os, const xml_schema::date_time& datetime);
#endif

#endif /* XSD_INT_VERSION */

#endif /* _OPENGPS_CXX_ISO5436_2_XSD_UTILS_HXX */

/*! @} */

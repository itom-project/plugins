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

#include <opengps/cxx/exceptions.hxx>
#include <opengps/cxx/string.hxx>
#include "stdafx.hxx"

Exception::Exception(const OGPS_ExceptionId id,
                     const OGPS_ExceptionChar *text,
                     const OGPS_ExceptionChar *details,
                     const OGPS_ExceptionChar *method)
#if linux
   : std::exception()
#else
   : std::exception(text)
#endif
{
#if linux
   std::cerr << text << std::endl;
#endif

   _ASSERT(text && details);

   m_Id = id;
   m_Details.FromChar(details);
   m_Method.FromChar(method);
}

Exception::Exception(const Exception& rhs) throw()
   : std::exception(rhs)
{
   m_Id = rhs.m_Id;
   m_Details = rhs.m_Details;
   m_Method = rhs.m_Method;
}

Exception::~Exception() throw()
{
}

OGPS_ExceptionId Exception::id() const throw()
{
   return m_Id;
}

const OpenGPS::String& Exception::details() const throw()
{
   return m_Details;
}

const OpenGPS::String& Exception::method() const throw()
{
   return m_Method;
}

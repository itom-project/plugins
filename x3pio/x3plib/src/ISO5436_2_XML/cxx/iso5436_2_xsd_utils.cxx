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

#include <opengps/cxx/iso5436_2_xsd_utils.hxx>

#if (XSD_INT_VERSION >= 3020000L)

#include <opengps/cxx/string.hxx>
#include <iomanip>
#include <tchar.h>

std::basic_ostream<char>& operator<<(std::basic_ostream<char>& os, const xml_schema::date_time& datetime)
{
   os << std::setfill('0') << std::setw(4) << datetime.year() << '-' << std::setw(2) << datetime.month()
      << '-' << std::setw(2) << datetime.day() << 'T' << std::setw(2) << datetime.hours()
      << ':' << std::setw(2) << datetime.minutes() << ':' << std::setw(2) << datetime.seconds();

   if(datetime.zone_present())
   {
      int offset_h = datetime.zone_hours();
      OpenGPS::String tzoffsign;
      if(offset_h < 0)
      {
         offset_h *= -1;
         tzoffsign.assign(1, _T('-'));
      }
      else
      {
         tzoffsign.assign(1, _T('+'));
      }
      os << ".0" << tzoffsign.ToChar() << std::setw(2) << offset_h << ':' << std::setw(2) << datetime.zone_minutes();
   }

   return os;
}




#ifdef _UNICODE

#include <sstream>

std::basic_ostream<wchar_t>& operator<<(std::basic_ostream<wchar_t>& os, const xml_schema::date_time& datetime)
{
   // std::ostringstream uses char as a base type (not wchar_t)
   std::ostringstream char_stream;
   // Uses the char-operator from above
   char_stream << datetime;

   // Convert to wchar_t and write to wchar_t-stream
   OpenGPS::String buffer;
   buffer.FromChar(char_stream.str().c_str());
   os << buffer;

   return os;
}

#endif /* _UNICODE */

#endif /* XSD_INT_VERSION */

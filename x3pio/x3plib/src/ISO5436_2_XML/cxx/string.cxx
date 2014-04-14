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

#include <opengps/cxx/string.hxx>

#include <stdlib.h>
#include <sstream>
#include <iomanip>
#include <string.h>

#include "stdafx.hxx"

String::String() :
BaseType()
{
#ifdef _UNICODE
   m_Chars = NULL;
#endif
}

String::String(const BaseType& s) : BaseType(s)
{
#ifdef _UNICODE
   m_Chars = NULL;
#endif
}

String::String(const OGPS_Character* const s) : BaseType(s)
{
#ifdef _UNICODE
   m_Chars = NULL;
#endif
}

String::~String()
{
#ifdef _UNICODE
   _OPENGPS_DELETE_ARRAY(m_Chars);
#endif
}

const char* String::ToChar()
{

#ifdef _UNICODE
   _OPENGPS_DELETE_ARRAY(m_Chars);

   const size_t len = length();
   m_Chars = new char[len + 1];
   /// @bug: It would be safer to use wcstomb_s on windows systems but this function is not ANSI-standard.
#pragma warning(suppress: 4996)
   wcstombs(m_Chars, c_str(), len);
   m_Chars[len] = 0;

   return m_Chars;
#else
   return c_str();
#endif

}

void String::FromChar(const char* const s)
{
   if(s)
   {
#ifdef _UNICODE
      const size_t len = strlen(s);
      FromChar(s, len);
#else
      *this = s;
#endif
   }
   else
   {
      clear();
   }
}

void String::FromChar(const char* const s, const size_t length)
{
   if(s && length > 0)
   {
#ifdef _UNICODE
      wchar_t* chars = new wchar_t[length];
      /// @bug: It would be safer to use wcstomb_s on windows systems but this function is not ANSI-standard.
#pragma warning(suppress: 4996)
      mbstowcs(chars, s, length);
      this->assign(chars, length);
      _OPENGPS_DELETE_ARRAY(chars);
#else
      this->assign(s, length);
#endif
   }
   else
   {
      clear();
   }
}

size_t String::CopyTo(OGPS_Character *const target, const size_t size) const
{
   const size_t len = length();

   if(size < len + 1)
   {
      return (len + 1);
   }

   _ASSERT(target);

   if(len > 0)
   {
#ifdef _UNICODE
    #if linux
        wcsncpy(target, c_str(), size);
    #else
        wcscpy_s(target, size, c_str());
    #endif
#else
    #if linux
        strncpy(target, c_str(), size);
    #else
        strcpy_s(target, size, c_str());
    #endif
#endif
   }
   else
   {
      target[len] = 0;
   }

   return len;
}

OGPS_Boolean String::ConvertToMd5(OpenGPS::UnsignedByte md5[16]) const
{
   if(size() != 32)
   {
      return FALSE;
   }

   String str(*this);

   for(size_t n = 30; n > 0; n-=2)
   {
      str.insert(n, 1, _T(' '));
   }

#ifdef _UNICODE
   std::wistringstream buffer(str);
#else
   std::istringstream buffer(str);
#endif /* _UNICODE */

   int md5n[16];
   for(size_t k = 0; k < 16; ++k)
   {
      buffer >> std::hex >> md5n[k];

      _ASSERT(md5n[k] >= 0 && md5n[k] < 256);

      md5[k] = (OpenGPS::UnsignedByte)md5n[k];
   }

   return buffer.eof() && !buffer.fail();
}

OGPS_Boolean String::ConvertFromMd5(const OpenGPS::UnsignedByte md5[16])
{
#ifdef _UNICODE
   std::wostringstream buffer;
#else
   std::ostringstream buffer;
#endif /* _UNICODE */

   buffer.fill('0');

   for(size_t k = 0; k < 16; ++k)
   {
      buffer << std::hex << std::setw(2) << md5[k];
   }

   if(buffer.good())
   {
      assign(buffer.str());
      return TRUE;
   }

   return FALSE;
}

String& String::ReplaceAll(const String& old_str, const String& new_str)
{
   size_t pos = 0;
   do
   {
      pos = find(old_str, pos);
      if(pos != String::npos)
      {
         replace(pos, old_str.length(), new_str);
         pos += new_str.length();
      }
   } while(pos != String::npos);
   return *this;
}

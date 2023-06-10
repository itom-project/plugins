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

#ifdef _WIN32

#include <stdlib.h>
#include <time.h>
#include <windows.h>

#include "win32_environment.hxx"
#include "stdafx.hxx"

#include <opengps/cxx/string.hxx>

#include <sstream>

#ifdef _UNICODE

namespace OpenGPS
{
   typedef std::wostringstream OutStringStream;
}

#else /* _UNICODE */

namespace OpenGPS
{
   typedef std::ostringstream OutStringStream;
}

#endif /* _UNICODE */

Win32Environment::Win32Environment()
: Environment()
{
}

Win32Environment::~Win32Environment()
{
}

OGPS_Boolean Win32Environment::m_InitRandom = TRUE;

OGPS_Character Win32Environment::GetDirectorySeparator() const
{
   return _T('\\');
}

OGPS_Character Win32Environment::GetAltDirectorySeparator() const
{
   return _T('/');
}

OGPS_Boolean Win32Environment::GetPathName(const OpenGPS::String& path, OpenGPS::String& clean_path) const
{
   _ASSERT(path.length() > 0);
   _ASSERT(path.c_str() != clean_path.c_str());

   /* See for reference: http://msdn2.microsoft.com/en-us/library/aa365247(VS.85).aspx */

   /* TODO: This is currently not verified:
    * Do not use the following reserved device names for the name of a file: CON, PRN, AUX, NUL, COM1, COM2, COM3, COM4, COM5, COM6, COM7, COM8, COM9, LPT1, LPT2, LPT3, LPT4, LPT5, LPT6, LPT7, LPT8, and LPT9. Also avoid these names followed by an extension, for example, NUL.tx7.
    */

   const size_t length = path.size();

   clean_path.resize(length);
   clean_path.erase();

   const OGPS_Character separator = GetDirectorySeparator();

   OGPS_Boolean hasDroppedChars = FALSE;
   OGPS_Boolean hasPathSeparator = FALSE;

   /* Use any character in the current code page for a name, including Unicode characters, except characters in the range of zero (0) through 31, or any character that the file system does not allow. A name can contain characters in the extended character set (128–255). However, it cannot contain the following reserved characters: < > : " / \ | ? *
   */
   for(size_t index = 0; index < length; ++index)
   {
      const OGPS_Character c = path.c_str()[index];

      if(c > 31 &&
         c != _T('<') &&
         c != _T('>') &&
         /* c != _T(':') && TODO: This check is currently omitted, because ':' is allowed in volume names... */
         c != _T('\"') &&
         c != _T('|') &&
         /* c != _T('?') &&  && TODO: This check is currently omitted, because '?' is allowed in UNC names... */
         c != _T('*'))
      {
         if(c == separator || c == GetAltDirectorySeparator())
         {
            if(hasPathSeparator)
            {
               /* Support UNC names, namely: \\<server>\<share> */
               if(clean_path.size() == 0)
               {
                  clean_path.append(&separator, 1);
               }

               continue;
            }

            hasPathSeparator = TRUE;
            continue;
         }

         if(hasPathSeparator)
         {
            clean_path.append(&separator, 1);
            hasPathSeparator = FALSE;
         }

         clean_path.append(&c, 1);
         continue;
      }

      hasDroppedChars = TRUE;
   }

   /* Do not end a file or directory name with a trailing space or a period. Although the underlying file system may support such names, the operating system does not. You can start a name with a period (.).
    */
   const size_t length2 = clean_path.size();
   for(size_t index2 = length2 - 1; index2 > 0; --index2)
   {
      const OGPS_Character c2 = clean_path.c_str()[index2];

      if(c2 == _T(' ') || c2 == _T('.'))
      {
         clean_path.resize(index2);
         hasDroppedChars = TRUE;
         continue;
      }

      break;
   }

   return !hasDroppedChars;
}

OpenGPS::String Win32Environment::GetFileName(const OpenGPS::String& path) const
{
   OpenGPS::String pattern;
   pattern.append(1, GetDirectorySeparator());
   pattern.append(1, GetAltDirectorySeparator());

   const size_t found = path.find_last_of(pattern);

   if(found != OpenGPS::String::npos)
   {
      return path.substr(found + 1);
   }

   return path;
}

OpenGPS::String Win32Environment::ConcatPathes(const OpenGPS::String& path1, const OpenGPS::String& path2) const
{
   _ASSERT(path1.length() > 0 || path2.length() > 0);

   // TODO: see .NET implementation

   if(path1.length() == 0)
   {
      return path2;
   }

   if(path2.length() == 0)
   {
      return path1;
   }

   OGPS_Boolean path2StartsWithSeparator = (path2[0] == GetDirectorySeparator() || path2[0] == GetAltDirectorySeparator());

   OpenGPS::String path = path1;

   if(path1[path1.length() - 1] != GetDirectorySeparator() && path1[path1.length() - 1] != GetAltDirectorySeparator())
   {
      if(!path2StartsWithSeparator)
      {
         path += GetDirectorySeparator();
      }
   }
   else
   {
      if(path2StartsWithSeparator)
      {
         path += path2.substr(1, path2.length() - 1);
         return path;
      }
   }

   path += path2;

   return path;
}

#ifndef INVALID_FILE_ATTRIBUTES
#define INVALID_FILE_ATTRIBUTES          (DWORD)-1
#endif /* INVALID_FILE_ATTRIBUTES */

OGPS_Boolean Win32Environment::PathExists(const OpenGPS::String& file) const
{
   _ASSERT(file.length() > 0);

   ResetLastErrorCode();

   const DWORD dwAttr = GetFileAttributes(file.c_str());
   if (dwAttr == INVALID_FILE_ATTRIBUTES)
   {
      const DWORD dwError = GetLastError();
      if (dwError == ERROR_FILE_NOT_FOUND)
      {
         // file not found
         return FALSE;
      }
      else if (dwError == ERROR_PATH_NOT_FOUND)
      {
         // path not found
         return FALSE;
      }
      else if (dwError == ERROR_ACCESS_DENIED)
      {
         // file or directory exists, but access is denied
         return TRUE;
      }
      //else
      {
         // some other error has occured
         _ASSERT(FALSE);
         return FALSE;
      }
   }
   //else
   {
      /*
      if (dwAttr & FILE_ATTRIBUTE_DIRECTORY)
      {
      // this is a directory
      }
      else
      {
      // this is an ordinary file
      }
      */
      return TRUE;
   }
}

OGPS_Boolean Win32Environment::RemoveFile(const OpenGPS::String& file) const
{
   _ASSERT(file.length() > 0);

   ResetLastErrorCode();

   return (DeleteFile(file.c_str()) != 0);
}

#define GETRANDOM(min, max) ((rand()%(int)(((max) + 1)-(min)))+ (min))

OpenGPS::String Win32Environment::GetUniqueName() const
{
   if(m_InitRandom)
   {
      srand ( (unsigned int)time(NULL) );
      m_InitRandom = FALSE;
   }

   // random positive number with max. 8 digits
   int randNum = GETRANDOM(0, 10000000);

   OpenGPS::OutStringStream os;
   os << randNum;

   return os.str();
}

OGPS_Boolean Win32Environment::CreateDir(const OpenGPS::String& path) const
{
   _ASSERT(path.length() > 0);

   ResetLastErrorCode();

   return (CreateDirectory(path.c_str(), NULL) != 0);
}

OGPS_Boolean Win32Environment::RemoveDir(const OpenGPS::String& path) const
{
   _ASSERT(path.length() > 0);

   ResetLastErrorCode();

   const OpenGPS::String pattern = ConcatPathes(path, _T("*"));

   WIN32_FIND_DATA found;
   HANDLE handle = FindFirstFile(pattern.c_str(), &found);
   if(handle != INVALID_HANDLE_VALUE)
   {
      do
      {
         const OpenGPS::String fname = found.cFileName;
         if(fname != _T(".") && fname != _T(".."))
         {
            const OpenGPS::String fpath = ConcatPathes(path, fname);
            DeleteFile(fpath.c_str());
         }
      } while(FindNextFile(handle, &found));

      FindClose(handle);
   }

   return (RemoveDirectory(path.c_str()) != 0);
}

OpenGPS::String Win32Environment::GetTempDir() const
{
   ResetLastErrorCode();

   unsigned long size = GetTempPath(0, NULL);

   OGPS_Character* buffer = new OGPS_Character[size];
   if(GetTempPath(size, buffer) + 1 == size)
   {
      OpenGPS::String path(buffer);
      _OPENGPS_DELETE(buffer);
      return path;
   }

   _ASSERT(FALSE);

   _OPENGPS_DELETE(buffer);
   return _T("");
}

OGPS_Boolean Win32Environment::RenameFile(const OpenGPS::String& src, const OpenGPS::String& dst) const
{
   ResetLastErrorCode();

   return (MoveFileEx(src.c_str(), dst.c_str(), MOVEFILE_COPY_ALLOWED | MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) != 0);
}

OGPS_Boolean Win32Environment::GetVariable(const OpenGPS::String& varName, OpenGPS::String& value) const
{
   ResetLastErrorCode();

   const size_t bufferLength = GetEnvironmentVariable(varName.c_str(), NULL, 0);

   if(bufferLength == 0)
   {
      _ASSERT(GetLastError() == ERROR_ENVVAR_NOT_FOUND);

      return FALSE;
   }

   value.resize(bufferLength - 1);
   _ASSERT(value.size() == bufferLength - 1);

   /* This readonly->read/write cast should be safe here. GetEnvironmentVariable is not supposed
   to vary buffer size nor its location. Only new content should be copied in. */
   if(GetEnvironmentVariable(varName.c_str(), (LPTSTR)value.c_str(), (DWORD)bufferLength) != bufferLength - 1)
   {
      value.erase();
      return FALSE;
   }
   else
   {
       OpenGPS::String unescaped;
      unescaped.assign(value);

      const size_t bufferLength2 = ExpandEnvironmentStrings(unescaped.c_str(), NULL, 0);

      if(bufferLength2 == 0)
      {
         _ASSERT(GetLastError() == ERROR_ENVVAR_NOT_FOUND);

         return FALSE;
      }

      value.resize(bufferLength2 - 1);
      _ASSERT(value.size() == bufferLength2 - 1);

      /* Wired cast should be safe here. Same as above. */
      /* This assumes we use Unicode here. See API for ExpandEnvironmentStrings. */
      if(ExpandEnvironmentStrings(unescaped.c_str(), (LPTSTR)value.c_str(), (DWORD)bufferLength2) != bufferLength2)
      {
         value.erase();
         return FALSE;
      }
   }

   return TRUE;
}

OpenGPS::String Win32Environment::GetLastErrorMessage() const
{
   const DWORD error_code = GetLastError();
   LPTSTR lpMsgBuf = NULL;
   FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
      NULL,
      error_code,
      0,
      (LPTSTR)&lpMsgBuf,
      0,
      NULL);
   OpenGPS::String message;
   if(lpMsgBuf != NULL)
   {
      message.assign(lpMsgBuf);
      LocalFree(lpMsgBuf);
   }
   return message;
}

void Win32Environment::ResetLastErrorCode() const
{
   SetLastError(0);
}

Environment* Environment::CreateInstance()
{
   return new Win32Environment();
}

#endif /* _WIN32 */

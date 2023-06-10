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

#ifdef linux
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <assert.h>
#include <stdio.h>
#include <sys/stat.h>
#include <errno.h>
#include <dirent.h>
#include <string.h>

#include "linux_environment.hxx"

#include <opengps/cxx/string.hxx>

#include <sstream>

#ifdef _UNICODE

extern int errno;

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

using namespace OpenGPS;


int copyFile(const char *src, const char *dst)
{
    FILE *in_fd = NULL, *out_fd = NULL;
    int n_chars;
    const int BUFSIZE = 4096;
    char buf[BUFSIZE];


    // open files
    if( (in_fd = fopen(src, "r")) == NULL )
    {
        return -1;
    }


    if( (out_fd = fopen(dst, "w")) == NULL )
    {
		fclose(in_fd);
        return -1;
    }

    // copy files
    while( (n_chars = fread(buf, 1, BUFSIZE, in_fd)) > 0 )
    {
        if( fwrite(buf, 1, n_chars, out_fd) != n_chars )
        {
			fclose(in_fd);
			fclose(out_fd);
            // write error
            return -1;
        }


        if( n_chars == -1 )
        {
			fclose(in_fd);
			fclose(out_fd);
            // read error
            return -1;
        }
    }


    // close files
    if( fclose(in_fd) == -1 || fclose(out_fd) == -1 )
    {
        // error closing file
        return -1;
    }

    return 0;
}

LinuxEnvironment::LinuxEnvironment()
: Environment()
{
}

LinuxEnvironment::~LinuxEnvironment()
{
}

OGPS_Boolean LinuxEnvironment::m_InitRandom = TRUE;

OGPS_Character LinuxEnvironment::GetDirectorySeparator() const
{
    #if linux
        return _T('/');
    #else
        return _T('\\');
    #endif
}

OGPS_Character LinuxEnvironment::GetAltDirectorySeparator() const
{
   return _T('/');
}

OGPS_Boolean LinuxEnvironment::GetPathName(const OpenGPS::String& path, OpenGPS::String& clean_path) const
{
   assert(path.length() > 0);
   assert(path.c_str() != clean_path.c_str());

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

OpenGPS::String LinuxEnvironment::GetFileName(const OpenGPS::String& path) const
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

OpenGPS::String LinuxEnvironment::ConcatPathes(const OpenGPS::String& path1, const OpenGPS::String& path2) const
{
   assert(path1.length() > 0 || path2.length() > 0);

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
#define INVALID_FILE_ATTRIBUTES          (unsigned long)-1
#endif /* INVALID_FILE_ATTRIBUTES */

OGPS_Boolean LinuxEnvironment::PathExists(const OpenGPS::String& file) const
{
   assert(file.length() > 0);

   ResetLastErrorCode();

   struct stat dwAttr;
   OpenGPS::String tempFile(file.c_str());
   if (stat(tempFile.ToChar(), &dwAttr)) //GetFileAttributes(file.c_str());
//   if (dwAttr == INVALID_FILE_ATTRIBUTES)
   {
      const unsigned long dwError = errno;
      if (dwError == ENOENT)
      {
         // file not found
         return FALSE;
      }
/*
      else if (dwError == ERROR_PATH_NOT_FOUND)
      {
         // path not found
         return FALSE;
      }
*/
      else if (dwError == EACCES)
      {
         // file or directory exists, but access is denied
         return TRUE;
      }
      //else
      {
         // some other error has occured
         assert(FALSE);
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

OGPS_Boolean LinuxEnvironment::RemoveFile(const OpenGPS::String& file) const
{
   assert(file.length() > 0);

   ResetLastErrorCode();

//   return (DeleteFile(file.c_str()) != 0);
   OpenGPS::String tempFile(file.c_str());
   return (remove(tempFile.ToChar()) != 0);
}

#define GETRANDOM(min, max) ((rand()%(int)(((max) + 1)-(min)))+ (min))

OpenGPS::String LinuxEnvironment::GetUniqueName() const
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

OGPS_Boolean LinuxEnvironment::CreateDir(const OpenGPS::String& path) const
{
   assert(path.length() > 0);

   ResetLastErrorCode();

   OpenGPS::String tempPath(path.c_str());
//   umask(0);
//   return (mkdir(tempPath.ToChar(), S_IRWXU | S_IRWXG | S_IRWXO) == 0);
   return (mkdir(tempPath.ToChar(), 0755) == 0);
}

OGPS_Boolean LinuxEnvironment::RemoveDir(const OpenGPS::String& path) const
{
    DIR *dir;
    struct dirent *entry;
    assert(path.length() > 0);

    ResetLastErrorCode();

    OpenGPS::String tempPath(path.c_str());
    dir = opendir(tempPath.ToChar());
    if (dir == NULL)
        return 0;
//    while (entry == readdir(dir) != NULL)
    while ((entry = readdir(dir)) != NULL)
    {
        if ((strcmp(entry->d_name, ".") == 0) || (strcmp(entry->d_name, ".") == 0))
            continue;
        else if (entry->d_type == DT_DIR)
        {
            OpenGPS::String subdir;
            subdir.FromChar(entry->d_name);
            RemoveDir(subdir);
        }
        else
        {
            remove(entry->d_name);
        }
    }
    closedir(dir);

    remove(tempPath.ToChar());

   return (errno != 0);
}

OpenGPS::String LinuxEnvironment::GetTempDir() const
{
   ResetLastErrorCode();

   char *tmpPath = getenv("TMPDIR");

   if (tmpPath && strlen(tmpPath) != 0)
   {
       OpenGPS::String tempPath;
       tempPath.FromChar(tmpPath);
       return tempPath;
   }
   else
   {
       DIR *dir = opendir("/tmp");
       if (dir != NULL)
       {
           closedir(dir);
           return _T("/tmp/");
       }
   }

   assert(FALSE);

   return _T("");
}

OGPS_Boolean LinuxEnvironment::RenameFile(const OpenGPS::String& src, const OpenGPS::String& dst) const
{
   ResetLastErrorCode();
   int ret;

   OpenGPS::String tempSrc(src.c_str());
   OpenGPS::String tempDst(dst.c_str());
//   ret = copy(tempSrc.ToChar(), tempDst.ToChar());
   ret = copyFile(tempSrc.ToChar(), tempDst.ToChar());
   ret += unlink(tempSrc.ToChar());

   return !ret;
//   return (MoveFileEx(src.c_str(), dst.c_str(), MOVEFILE_COPY_ALLOWED | MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) != 0);
}

OGPS_Boolean LinuxEnvironment::GetVariable(const OpenGPS::String& varName, OpenGPS::String& value) const
{
   ResetLastErrorCode();

   OpenGPS::String tempVarName(varName.c_str());
   char *varbuf = getenv(tempVarName.ToChar());

   if (!varbuf)
   {
      assert(varbuf == NULL);

      return FALSE;
   }

   value.FromChar(varbuf);

   return TRUE;
}

OpenGPS::String LinuxEnvironment::GetLastErrorMessage() const
{
    OpenGPS::String message;
    message.FromChar(strerror(errno));

   return message;
}

void LinuxEnvironment::ResetLastErrorCode() const
{
    errno = 0;
}

Environment* Environment::CreateInstance()
{
   return new LinuxEnvironment();
}

#endif /* linux */

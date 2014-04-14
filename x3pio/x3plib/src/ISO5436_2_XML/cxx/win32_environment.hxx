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
 * The environment of Microsoft Windows operating systems.
 */

#ifndef _OPENGPS_WIN32_ENVIRONMENT_HXX
#define _OPENGPS_WIN32_ENVIRONMENT_HXX

#ifdef _WIN32

#ifndef _OPENGPS_ENVIRONMENT_HXX
#  include "environment.hxx"
#endif

namespace OpenGPS
{
   /*!
    * Implements access methods to various parts of the environment
    * on Microsoft Windows operating systems.
    */
   class Win32Environment : public Environment
   {
   public:
      /*! Creates a new instance. */
      Win32Environment();

      /*! Destroys this instance. */
      ~Win32Environment();

      virtual OGPS_Character GetDirectorySeparator() const;
      virtual OGPS_Character GetAltDirectorySeparator() const;
      virtual OGPS_Boolean GetPathName(const OpenGPS::String& path, OpenGPS::String& clean_path) const;
      virtual OpenGPS::String GetFileName(const OpenGPS::String& path) const;
      virtual OpenGPS::String ConcatPathes(const OpenGPS::String& path1, const OpenGPS::String& path2) const;
      virtual OGPS_Boolean PathExists(const OpenGPS::String& file) const;
      virtual OGPS_Boolean RemoveFile(const OpenGPS::String& file) const;
      virtual OpenGPS::String GetUniqueName() const;
      virtual OGPS_Boolean CreateDir(const OpenGPS::String& path) const;
      virtual OGPS_Boolean RemoveDir(const OpenGPS::String& path) const;
      virtual OpenGPS::String GetTempDir() const;
      virtual OGPS_Boolean RenameFile(const OpenGPS::String& src, const OpenGPS::String& dst) const;
      virtual OGPS_Boolean GetVariable(const OpenGPS::String& varName, OpenGPS::String& value) const;
      virtual OpenGPS::String GetLastErrorMessage() const;

   private:
      /*! Resets the last system error API code. */
      void ResetLastErrorCode() const;

      /*! Tracks whether the sofware randomization API still needs to be initialized before its usage. */
      static OGPS_Boolean m_InitRandom;
   };
}

#endif /* _WIN32 */

#endif /* _OPENGPS_WIN32_ENVIRONMENT_HXX */

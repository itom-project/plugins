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
 * Copyright and license information.
 */

#ifndef _OPENGPS_CXX_INFO_HXX
#define _OPENGPS_CXX_INFO_HXX

#ifndef _OPENGPS_CXX_OPENGPS_HXX
# include <opengps/cxx/opengps.hxx>
#endif

namespace OpenGPS
{
   class String;

   /*!
    * Publishes license text, ownership and similar information.
    */
   class _OPENGPS_EXPORT Info
   {
   public:
      /*!
       * Gets a short name identifying this software library.
       *
       * @param text Contains the resulting message.
       */
      static void GetName(OpenGPS::String* const text);

      /*!
       * Gets a short message describing the purpose of this software library.
       *
       * @param text Contains the resulting message.
       */
      static void GetAbout(OpenGPS::String* const text);

      /*!
       * Gets the version identifier of this software library.
       *
       * @param text Contains the resulting message.
       */
      static void GetVersion(OpenGPS::String* const text);

      /*!
       * Gets the copyright information of this software library.
       *
       * @param text Contains the resulting message.
       */
      static void GetCopyright(OpenGPS::String* const text);

      /*!
       * Gets the license information of this software library.
       *
       * @param text Contains the resulting message.
       */
      static void GetLicense(OpenGPS::String* const text);

      /*!
       * Prints the version identifier of this software library to standard output.
       */
      static void PrintVersion();

      /*!
       * Prints the copyright information of this software library to standard output.
       */
      static void PrintCopyright();

      /*!
       * Prints the license information of this software library to standard output.
       */
      static void PrintLicense();

   private:
      Info();
      ~Info();
   };

}

#endif    /* _OPENGPS_CXX_INFO_HXX */

/*! @} */

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
 * Exception types thrown within this software library.
 */


#ifndef _OPENGPS_CXX_EXCPTIONS_HXX
#define _OPENGPS_CXX_EXCPTIONS_HXX

#include <exception>

#ifndef _OPENGPS_CXX_OPENGPS_HXX
#  include <opengps/cxx/opengps.hxx>
#endif

#ifndef _OPENGPS_MESSAGES_H
#  include <opengps/messages.h>
#endif

#ifndef _OPENGPS_CXX_STRING_HXX
#  include <opengps/cxx/string.hxx>
#endif

#if defined(SHARED_OPENGPS_LIBRARY) || defined(BUILD_ISO5436_2_XML_DLL)
   class _OPENGPS_EXPORT std::exception;
#endif /* defined(SHARED_OPENGPS_LIBRARY) || defined(BUILD_ISO5436_2_XML_DLL) */

namespace OpenGPS
{
   /*!
    * Describes a general exception.
    */
   class _OPENGPS_EXPORT Exception : public std::exception
   {
   public:
      /*!
       * Creates a new instance.
       *
       * @param id Type of the exception object.
       * @param text Brief description of the exception.
       * @param details Detailed description of the exception. May be set to NULL.
       * @param method The name of the method wherein the failure condition occured. May be set to NULL.
       */
      Exception(const OGPS_ExceptionId id, const OGPS_ExceptionChar *text, const OGPS_ExceptionChar *details, const OGPS_ExceptionChar *method);
      /*!
       * Creates a new instance.
       *
       * @param rhs The instance to copy from.
       */
      Exception(const Exception& rhs) throw();

      /*! Destroys this instance. */
      virtual ~Exception() throw();

      /*! Gets an identifier for the current type of excpetion. */
      OGPS_ExceptionId id() const throw();

      /*! Gets a detailed description possible with hints to its avoidance. */
      const OpenGPS::String& details() const throw();

      /*! Gets the name of the method where the exception occured or NULL. */
      const OpenGPS::String& method() const throw();

   private:
      /*! Describes the type of failure. */
      OGPS_ExceptionId m_Id;

      /*! Detailed description of the failure. */
      OpenGPS::String m_Details;

      /*! Name of the method wherein the failure occured. */
      OpenGPS::String m_Method;
   };
}

#endif /* _OPENGPS_CXX_EXCPTIONS_HXX */

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
 * Error and warning messages during application.
 */

#ifndef _OPENGPS_C_MESSAGES_HXX
#define _OPENGPS_C_MESSAGES_HXX

#ifndef _OPENGPS_CXX_OPENGPS_HXX
#  include <opengps/cxx/opengps.hxx>
#endif

#ifndef _OPENGPS_CXX_EXCEPTIONS_HXX
#  include <opengps/cxx/exceptions.hxx>
#endif

#ifndef _OPENGPS_CXX_STRING_HXX
#  include <opengps/cxx/string.hxx>
#endif

namespace OpenGPS
{
   /*!
    * Maintains a history of exceptions.
    */
   class ExceptionHistory
   {
   public:
      /*!
       * Sets an exception object as the last one that occured during execution.
       * @param ex Add this exception object to the history of exceptions.
       */
      static void SetLastException(const OpenGPS::Exception& ex);
      /*!
       * Sets an exception object as the last one that occured during execution.
       * @param ex Add this exception object to the history of exceptions.
       */
      static void SetLastException(const std::exception& ex);
      /*!
       * Notify that an unknown type of exception occured.
       */
      static void SetLastException();

      /*!
       * Reset the history of exceptions.
       */
      static void Reset();

      /*!
       * Gets the brief failure description of the last known exception or NULL.
       */
      static const OGPS_Character* GetLastErrorMessage();

      /*!
       * Gets the detailed failure description of the last known exception or NULL.
       */
      static const OGPS_Character* GetLastErrorDescription();

      /*!
       * Gets the classifier of the last known exception or ::OGPS_ExNone if none
       * is maintained so far.
       */
      static OGPS_ExceptionId GetLastErrorId();

   private:
      /*! Creates a new instance. */
      ExceptionHistory();
      /*! Destroys this instance. */
      ~ExceptionHistory();

      /*! The brief description of the last known failure condition or empty. */
      static OpenGPS::String m_LastErrorMessage;

      /*! The detailed description of the last known failure condition or empty. */
      static OpenGPS::String m_LastErrorDescription;

      /*! The classifier of the last known failure condition or ::OGPS_ExNone. */
      static OGPS_ExceptionId m_LastErrorId;

      /*! Source of the last error condition. */
      static OpenGPS::String m_LastErrorSource;

      /*! Dumps a message to the error console. This works in _DEBUG only. */
      static void DumpIt();
   };
}

/*!
 * Helper for handling thrown exceptions.
 *
 * First resets the history of exceptions, see OpenGPS::ExceptionHistory::Reset. Then executes a statement
 * that is surrounded by a try-catch block. If an exception is thrown within the statement it is added to
 * the history of exceptions, see OpenGPS::ExceptionHistory::SetLastException.
 *
 * @param STATEMENT A statement with boolean return value to be executed.
 */
#define _OPENGPS_GENERIC_EXCEPTION_HANDLER_RETVALBOOL(STATEMENT) \
   OpenGPS::ExceptionHistory::Reset(); \
   try \
   { \
      return (STATEMENT); \
   } \
   catch(const OpenGPS::Exception& ex) \
   { \
      OpenGPS::ExceptionHistory::SetLastException(ex); \
   } \
   catch(const std::exception& sx) \
   { \
      OpenGPS::ExceptionHistory::SetLastException(sx); \
   } \
   catch(...) \
   { \
      OpenGPS::ExceptionHistory::SetLastException(); \
   } \
   return FALSE;

/*!
 * Helper for handling thrown exceptions.
 *
 * First resets the history of exceptions, see OpenGPS::ExceptionHistory::Reset. Then executes a statement
 * that is surrounded by a try-catch block. If an exception is thrown within the statement it is added to
 * the history of exceptions, see OpenGPS::ExceptionHistory::SetLastException.
 *
 * @param STATEMENT The statement to be executed within the try-block.
 * @param CLEANUP_STATEMENT The statement to be executed within the catch-block.
 */
#define _OPENGPS_GENERIC_EXCEPTION_HANDLER_CLEANUP(STATEMENT, CLEANUP_STATEMENT) \
   OpenGPS::ExceptionHistory::Reset(); \
   try \
   { \
      STATEMENT; \
   } \
   catch(const OpenGPS::Exception& ex) \
   { \
      CLEANUP_STATEMENT; \
      OpenGPS::ExceptionHistory::SetLastException(ex); \
   } \
   catch(const std::exception& sx) \
   { \
      CLEANUP_STATEMENT; \
      OpenGPS::ExceptionHistory::SetLastException(sx); \
   } \
   catch(...) \
   { \
      CLEANUP_STATEMENT; \
      OpenGPS::ExceptionHistory::SetLastException(); \
   }

/*!
 * Helper for handling thrown exceptions.
 *
 * First resets the history of exceptions, see OpenGPS::ExceptionHistory::Reset. Then executes a statement
 * that is surrounded by a try-catch block. If an exception is thrown within the statement it is added to
 * the history of exceptions, see OpenGPS::ExceptionHistory::SetLastException.
 *
 * See ::_OPENGPS_GENERIC_EXCEPTION_HANDLER_RETVALBOOL for statements of boolean return type.
 *
 * @param STATEMENT A statement to be executed.
 */
#define _OPENGPS_GENERIC_EXCEPTION_HANDLER(STATEMENT) \
   OpenGPS::ExceptionHistory::Reset(); \
   try \
   { \
      STATEMENT; \
   } \
   catch(const OpenGPS::Exception& ex) \
   { \
      OpenGPS::ExceptionHistory::SetLastException(ex); \
   } \
   catch(const std::exception& sx) \
   { \
      OpenGPS::ExceptionHistory::SetLastException(sx); \
   } \
   catch(...) \
   { \
      OpenGPS::ExceptionHistory::SetLastException(); \
   }

#endif /* _OPENGPS_C_MESSAGES_HXX */

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
 * Interface for writing typed point data to an underlying stream of point vectors.
 */

#ifndef _OPENGPS_POINT_VECTOR_WRITER_CONTEXT_HXX
#define _OPENGPS_POINT_VECTOR_WRITER_CONTEXT_HXX

#ifndef _OPENGPS_CXX_OPENGPS_HXX
#  include <opengps/cxx/opengps.hxx>
#endif

namespace OpenGPS
{
   /*!
    * Encapsulates an underlying stream of point vector data. Provides
    * methods for writing point data to that stream. Three values of
    * point data make up one point vector. When all three components
    * have been written (by using the PointVectorWriterContext::Write/
    * PointVectorWriterContext::Skip methods) PointVectorWriterContext::MoveNext
    * must be called to complete the transaction.
    */
   class PointVectorWriterContext
   {
   public:
      /*! Destroys this instance. */
      virtual ~PointVectorWriterContext();

      /*!
       * Writes a single point of type ::OGPS_Int16 to the underlying stream.
       *
       * A specific implementation may throw an OpenGPS::Exception if this operation
       * is not permitted due to the current state of the object instance.
       *
       * @param value Contains the point value on success.
       */
      virtual void Write(const OGPS_Int16* const value) = 0;

      /*!
       * Writes a single point of type ::OGPS_Int32 to the underlying stream.
       *
       * A specific implementation may throw an OpenGPS::Exception if this operation
       * is not permitted due to the current state of the object instance.
       *
       * @param value Contains the point value on success.
       */
      virtual void Write(const OGPS_Int32* const value) = 0;

      /*!
       * Writes a single point of type ::OGPS_Float to the underlying stream.
       *
       * A specific implementation may throw an OpenGPS::Exception if this operation
       * is not permitted due to the current state of the object instance.
       *
       * @param value Contains the point value on success.
       */
      virtual void Write(const OGPS_Float* const value) = 0;

      /*!
       * Writes a single point of type ::OGPS_Double to the underlying stream.
       *
       * A specific implementation may throw an OpenGPS::Exception if this operation
       * is not permitted due to the current state of the object instance.
       *
       * @param value Contains the point value on success.
       */
      virtual void Write(const OGPS_Double* const value) = 0;

      /*!
       * There is no point data to be written to the underlying stream for
       * the current axis. One point vector has three components, whereas
       * only the Z component needs to be set explicitly in major cases
       * because of implicit axis definitions of X or Y.
       * Use this skip method then (with the X or Y components)
       * to indicate that, because a successfully written point vector must
       * comprise all three component values.
       *
       * A specific implementation may throw an OpenGPS::Exception if this operation
       * is not permitted due to the current state of the object instance.
       *
       * @remarks This must be called for integrity if
       * there is no point data to be stored
       * because the corresponding axis is of incremental type.
       */
      virtual void Skip() = 0;

      /*!
       * Complete the transaction of the current point vector. One point
       * vector is made up of three components (data points). After
       * three subsequent PointVectorWriterContext::Write/
       * PointVectorWriterContext::Skip calls, call this method to
       * indicate that the whole point vector has been written.
       *
       * A specific implementation may throw an OpenGPS::Exception if this operation
       * is not permitted due to the current state of the object instance.
       */
      virtual void MoveNext() = 0;

   protected:
      /*! Creates a new instance. */
      PointVectorWriterContext();

      /*!
       * Asks if the underlying data stream is still valid.
       * @returns Returns TRUE if no previous access to the underlying
       * data stream was harmful. Returns FALSE if any damage occured.
       */
      virtual OGPS_Boolean IsGood() const = 0;
   };
}

#endif /* _OPENGPS_POINT_VECTOR_WRITER_CONTEXT_HXX */

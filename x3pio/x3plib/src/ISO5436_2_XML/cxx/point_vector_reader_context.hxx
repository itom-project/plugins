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
 * Interface for reading typed point data from an underlying stream of point vectors.
 */

#ifndef _OPENGPS_POINT_VECTOR_READER_CONTEXT_HXX
#define _OPENGPS_POINT_VECTOR_READER_CONTEXT_HXX

#ifndef _OPENGPS_CXX_OPENGPS_HXX
#  include <opengps/cxx/opengps.hxx>
#endif

namespace OpenGPS
{
   /*!
    * Encapsulates an underlying stream of point vector data. Provides
    * methods for reading point data from that stream. The exact data type
    * of point data stored at the current position of the imagined stream
    * pointer must be known a-priori though. This meta information must
    * be extracted from elsewhere before and is not accessible from here.
    * Three values of point data make up one point vector. To read one
    * point vector use PointVectorReaderContext::MoveNext to start reading.
    * Check if the current point vector is valid. Then make three subsequent
    * calls to the PointVectorReaderContext::Read/
    * PointVectorReaderContext::Skip methods (you must know the data type of
    * all values in the stream to be able to do that).
    */
   class PointVectorReaderContext
   {
   public:
      /*! Destroys this instance. */
      virtual ~PointVectorReaderContext();

      /*!
       * Reads the currently underlying data as ::OGPS_Int16.
       * Also moves the current reading position of the stream
       * to the next coordinate of the three-vector.
       *
       * A specific implementation may throw an OpenGPS::Exception if this operation
       * is not permitted due to the current state of the object instance.
       *
       * @param value Contains the point value on success.
       */
      virtual void Read(OGPS_Int16* const value) = 0;

      /*!
       * Reads the currently underlying data as ::OGPS_Int32.
       * Also moves the current reading position of the stream
       * to the next coordinate of the three-vector.
       *
       * A specific implementation may throw an OpenGPS::Exception if this operation
       * is not permitted due to the current state of the object instance.
       *
       * @param value Contains the point value on success.
       */
      virtual void Read(OGPS_Int32* const value) = 0;

      /*!
       * Reads the currently underlying data as ::OGPS_Float.
       * Also moves the current reading position of the stream
       * to the next coordinate of the three-vector.
       *
       * A specific implementation may throw an OpenGPS::Exception if this operation
       * is not permitted due to the current state of the object instance.
       *
       * @param value Contains the point value on success.
       */
      virtual void Read(OGPS_Float* const value) = 0;

      /*!
       * Reads the currently underlying data as ::OGPS_Double.
       * Also moves the current reading position of the stream
       * to the next coordinate of the three-vector.
       *
       * A specific implementation may throw an OpenGPS::Exception if this operation
       * is not permitted due to the current state of the object instance.
       *
       * @param value Contains the point value on success.
       */
      virtual void Read(OGPS_Double* const value) = 0;

      /*!
       * Skips reading of the currently underlying data.
       * Also moves the current reading position of the stream
       * to the next coordinate of the three-vector.
       *
       * A specific implementation may throw an OpenGPS::Exception if this operation
       * is not permitted due to the current state of the object instance.
       *
       * @remarks This must be called for integrity if
       * it is expected from the reading process that there
       * is no point data available for reading, i.e. for
       * the current coordinate no point data was saved
       * because the corresponding axis is of incremental type.
       */
      virtual void Skip() = 0;

      /*!
       * Move the current reading position of the stream
       * to the next three-vector. This is possible only if all
       * three coordinates of the current point vector data
       * had been fully read (see PointVectorReaderContext::Read
       * and PointVectorReaderContext::Skip member
       * functions). If PointVectorReaderContext::IsValid returns
       * FALSE, call this method directly to move to the
       * next point vector in storage.
       *
       * A specific implementation may throw an OpenGPS::Exception if this operation
       * is not permitted due to the current state of the object instance.
       *
       * @returns Returns TRUE when there is more data to be parsed, FALSE otherwise.
       */
      virtual OGPS_Boolean MoveNext() = 0;

      /*!
       * Asks if there is readable point vector stored
       * at the current position. A point vector may be
       * invalid if it had not been measured, but is needed
       * for topological consistency though.
       * @remarks This must be checked before one attempts to
       * PointVectorReaderContext::Read/
       * PointVectorReaderContext::Skip any of the point
       * data of one of the three coordinates.
       *
       * A specific implementation may throw an OpenGPS::Exception if this operation
       * is not permitted due to the current state of the object instance.
       *
       * @returns Returns TRUE if the current point is readable,
       * FALSE otherwise.
       */
      virtual OGPS_Boolean IsValid() const = 0;

   protected:
      /*! Creates a new instance. */
      PointVectorReaderContext();
   };
}

#endif /* _OPENGPS_POINT_VECTOR_READER_CONTEXT_HXX */

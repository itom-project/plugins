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
 * Generic parser of a point vector.
 */

#ifndef _OPENGPS_POINT_VECTOR_PARSER_HXX
#define _OPENGPS_POINT_VECTOR_PARSER_HXX

#ifndef _OPENGPS_CXX_OPENGPS_HXX
#  include <opengps/cxx/opengps.hxx>
#endif

#ifndef _OPENGPS_DATA_POINT_TYPE_H
#  include <opengps/data_point_type.h>
#endif

namespace OpenGPS
{
   class PointVectorReaderContext;
   class PointVectorWriterContext;
   class PointVectorBase;
   class DataPointParser;

   /*!
    * Generic parser of a point vector.
    *
    * Reads/Writes an instance of OpenGPS::PointVectorBase from/to any file
    * or resource described by an instance of OpenGPS::PointVectorReaderContext/
    * OpenGPS::PointVectorWriterContext.
    */
   class PointVectorParser
   {
   public:
      /*! Creates a new instance. */
      PointVectorParser();

      /*! Destroys this instance. */
      virtual ~PointVectorParser();

      /*!
       * Sets the interface object to be used for parsing point data of the X component.
       * @remarks The pointer passed will become managed. Its resources may become
       * inaccessible at any time.
       * @param value Typesafe point parser of the first vector component.
       * @see PointVectorParser::CreateDataPointParser
       */
      void SetX(DataPointParser* const value);

      /*!
       * Sets the interface object to be used for parsing point data of the Y component.
       * @remarks The pointer passed will become managed. Its resources may become
       * inaccessible at any time.
       * @param value Typesafe point parser of the second vector component.
       * @see PointVectorParser::CreateDataPointParser
       */
      void SetY(DataPointParser* const value);

      /*!
       * Sets the interface object to be used for parsing point data of the Z component.
       * @remarks The pointer passed will become managed. Its resources may become
       * inaccessible at any time.
       * @param value Typesafe point parser of the third vector component.
       * @see PointVectorParser::CreateDataPointParser
       */
      void SetZ(DataPointParser* const value);

      /*!
       * Reads point vector data from arbitrary media.
       *
       * Throws an OpenGPS::Exception if this operation
       * is not permitted due to the current state of the object instance.
       *
       * @param context Methods to read point data from the media.
       * @param value Holds the point data read assembled as a three-vector.
       */
      virtual void Read(PointVectorReaderContext& context, PointVectorBase& value);

      /*!
       * Writes point vector data to arbitrary media.
       *
       * Throws an OpenGPS::Exception if this operation
       * is not permitted due to the current state of the object instance.
       *
       * @param context Methods to write point data to the media.
       * @param value Holds the vector to be written.
       */
      virtual void Write(PointVectorWriterContext& context, const PointVectorBase& value);

      /*!
       * Creates the appropriate point parser corresponding to an axis data type.
       * @remarks The returned point parser instance must be free explicitly unless
       * it is directly passed to one of the PointVectorParser::SetX, PointVectorParser::SetY,
       * PointVectorParser::SetZ methods.
       * @param dataType The axis data type to instantiate the point parser for.
       * @returns Returns the instance of the point parser or NULL on failure.
       */
      virtual DataPointParser* CreateDataPointParser(const OGPS_DataPointType dataType) const;

   private:
      /*! Instance of the point parser of the X component. */
      DataPointParser* m_X;

      /*! Instance of the point parser of the Y component. */
      DataPointParser* m_Y;

      /*! Instance of the point parser of the Z component. */
      DataPointParser* m_Z;
   };
}

#endif /* _OPENGPS_POINT_VECTOR_PARSER_HXX */

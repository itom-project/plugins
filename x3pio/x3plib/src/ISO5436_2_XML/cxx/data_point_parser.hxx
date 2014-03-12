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
 * Generic interface of a data point parser for reading/writing point
 * data of any supported data type from/to any supported media.
 */

#ifndef _OPENGPS_DATA_POINT_PARSER_HXX
#define _OPENGPS_DATA_POINT_PARSER_HXX

#ifndef _OPENGPS_CXX_OPENGPS_HXX
#  include <opengps/cxx/opengps.hxx>
#endif

namespace OpenGPS
{
   class PointVectorReaderContext;
   class PointVectorWriterContext;
   class DataPoint;

   /*!
    * Interface for reading/writing point data.
    * Reads/writes point data from/to specified media.
    * This interface is implemented
    * for every possible type of point data
    * (::OGPS_DataPointType).
    */
   class DataPointParser
   {
   public:
      /*! Destroys this instance. */
      virtual ~DataPointParser();

      /*!
       * Reads point data from a given context/media.
       * Using the appropriate access method of the given implementation of
       * OpenGPS::PointVectorReaderContext the current point value read
       * gets stored in a OpenGPS::DataPoint instance. This operation is
       * typesafe.
       *
       * A specific implementation may throw an OpenGPS::Exception if this operation
       * is not permitted due to the current state of the object instance.
       *
       * @param context Provides read access to the media.
       * @param value Buffer where the point data read gets stored.
       */
      virtual void Read(PointVectorReaderContext& context, DataPoint& value) = 0;

      /*!
       * Writes point data to a given context/media.
       * Using the appropriate access method of the given implementation of
       * OpenGPS::PointVectorWriterContext the point value currently
       * stored in the OpenGPS::DataPoint instance gets written to the media.
       * This operation is typesafe and will fail if the type of point data
       * stored in the value object does not exacly match the current implementation of
       * the OpenGPS::DataPointParser interface.
       *
       * A specific implementation may throw an OpenGPS::Exception if this operation
       * is not permitted due to the current state of the object instance.
       *
       * @param context Provides write access to the media.
       * @param value Buffer where the point data to be written is stored.
       */
      virtual void Write(PointVectorWriterContext& context, const DataPoint& value) = 0;

   protected:
      /*! Creates a new instance. */
      DataPointParser();
   };
}

#endif /* _OPENGPS_DATA_POINT_PARSER_HXX */

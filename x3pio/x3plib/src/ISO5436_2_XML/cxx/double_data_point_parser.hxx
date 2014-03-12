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
 * A data point parser for point data of type ::OGPS_DoublePointType.
 */

#ifndef _OPENGPS_DOUBLE_DATA_POINT_PARSER_HXX
#define _OPENGPS_DOUBLE_DATA_POINT_PARSER_HXX

#ifndef _OPENGPS_DATA_POINT_PARSER_HXX
#  include "data_point_parser.hxx"
#endif

namespace OpenGPS
{
   /*!
    * Reads/Writes instances of OpenGPS::DataPoint of ::OGPS_Double point data.
    */
   class DoubleDataPointParser : public DataPointParser
   {
   public:
      /*! Creates a new instance. */
      DoubleDataPointParser();

      /*! Destroys this instance. */
      virtual ~DoubleDataPointParser();

      virtual void Read(PointVectorReaderContext& context, DataPoint& value);
      virtual void Write(PointVectorWriterContext& context, const DataPoint& value);
   };
}

#endif /* _OPENGPS_DOUBLE_DATA_POINT_PARSER_HXX */

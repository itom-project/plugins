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
 * Builder which assembles a concrete instance of the generic point vector parser.
 */

#ifndef _OPENGPS_POINT_VECTOR_PARSER_BUILDER_HXX
#define _OPENGPS_POINT_VECTOR_PARSER_BUILDER_HXX

#ifndef _OPENGPS_CXX_OPENGPS_HXX
#  include <opengps/cxx/opengps.hxx>
#endif

#ifndef _OPENGPS_DATA_POINT_TYPE_H
#  include <opengps/data_point_type.h>
#endif

namespace OpenGPS
{
   class PointVectorParser;

   /*!
    * Component which handles the building process of a specialized parser
    * object for reading and writing typed point data as a three-vector.
    * @see OpenGPS::PointVectorParser, OpenGPS::DataPointParser
    */
   class PointVectorParserBuilder
   {
   public:
      /*! Creates a new instance. */
      PointVectorParserBuilder();

      /*! Destroys this instance. */
      virtual ~PointVectorParserBuilder();

      /*!
       * Builds the main parser object to be assembled.
       * @remarks This must be called firstly when starting the build process.
       */
      void BuildParser();

      /*!
       * Builds the object used to parse the X component of the vector.
       * @param dataType The data type of the X axis.
       * @see PointVectorParserBuilder::BuildParser
       */
      virtual void BuildX(const OGPS_DataPointType dataType);

      /*!
       * Builds the object used to parse the Y component of the vector.
       * @param dataType The data type of the Y axis.
       * @see PointVectorParserBuilder::BuildParser
       */
      virtual void BuildY(const OGPS_DataPointType dataType);

      /*!
       * Builds the object used to parse the Z component of the vector.
       * @param dataType The data type of the Z axis.
       * @see PointVectorParserBuilder::BuildParser
       */
      virtual void BuildZ(const OGPS_DataPointType dataType);

      /*! Gets the assembled point vector parser. */
      virtual PointVectorParser* GetParser();

   private:
      /*! The object to be built. */
      PointVectorParser* m_Parser;
   };
}

#endif /* _OPENGPS_POINT_VECTOR_PARSER_BUILDER_HXX */

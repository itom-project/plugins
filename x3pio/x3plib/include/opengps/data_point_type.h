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

/*! \addtogroup C
 *  @{
 */

/*! @file
 * An enumeration type which may describe the data type of
 * a current data point value. Used by various components throughout
 * this software library.
 */

#ifndef _OPENGPS_DATA_POINT_TYPE_H
#define _OPENGPS_DATA_POINT_TYPE_H

#ifdef __cplusplus
extern "C" {
#endif

   /*!
    * Possible types of data stored in an ::OGPS_DataPointPtr instance.
    */
   typedef enum _OGPS_DATA_POINT_TYPE {
      /*! Describes an invalid type. */
      OGPS_MissingPointType,
      /*! Describes an instance of type ::OGPS_Int16. */
      OGPS_Int16PointType,
      /*! Describes an instance of type ::OGPS_Int32. */
      OGPS_Int32PointType,
      /*! Describes an instance of type ::OGPS_Float. */
      OGPS_FloatPointType,
      /*! Describes an instance of type ::OGPS_Double. */
      OGPS_DoublePointType
   } OGPS_DataPointType; /*! Possible types of data stored in an OGPS_DataPointPtr instance. */

#ifdef __cplusplus
}
#endif

#endif /* _OPENGPS_DATA_POINT_TYPE_H */
/*! @} */

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
 * The abstract data type of the typesafe representation of a single data point value.
 * The interface supports getting and setting an encapsulated point value of a predetermined data type.
 */

#ifndef _OPENGPS_DATA_POINT_H
#define _OPENGPS_DATA_POINT_H

#ifndef _OPENGPS_H
#  include <opengps/opengps.h>
#endif

#ifndef _OPENGPS_DATA_POINT_TYPE_H
#  include <opengps/data_point_type.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

   /* TODO: HACK: with the typedef, the const qualifier is not reconised any more.
   #define OGPS_DataPointPtr struct _DATA_POINT * */


   /*!
    * Typesafe representation of a single data point value.
    *
    * In an ::OGPS_PointVectorPtr every component of that three-vector is
    * accessible through its own ::OGPS_DataPointPtr instance.
    *
    * @remarks An instance of ::OGPS_DataPointPtr cannot be created of its own.
    * Indirectly a handle of this type can be accessed through an ::OGPS_PointVectorPtr object.
    *
    * The corresponding C++ implementation is provided by OpenGPS::DataPoint.
    */
   typedef struct _OGPS_DATA_POINT * OGPS_DataPointPtr;

   /*!
    * Gets type information about the current value stored in a given data point instance.
    *
    * Returns ::OGPS_MissingPointType if this instance does not store any value at all.
    * You may check ::ogps_HasError whether this operation has been successfully executed.
    *
    * @param dataPoint Operate on this data point instance.
    */
   _OPENGPS_EXPORT OGPS_DataPointType ogps_GetPointType(const OGPS_DataPointPtr dataPoint);

   /*!
    * Gets the stored value of type ::OGPS_Int16 out of a given data point instance.
    *
    * @remarks If the current type does not equal ::OGPS_Int16, this function returns the value 0.
    * You may check ::ogps_HasError whether this operation has been successfully executed.
    *
    * @see ::ogps_GetPointType
    *
    * @param dataPoint Operate on this data point instance.
    */
   _OPENGPS_EXPORT OGPS_Int16 ogps_GetInt16(const OGPS_DataPointPtr dataPoint);

   /*!
    * Gets the stored value of type ::OGPS_Int32 out of a given data point instance.
    *
    * @remarks If the current type does not equal ::OGPS_Int32, this function returns the value 0.
    * You may check ::ogps_HasError whether this operation has been successfully executed.
    *
    * @see ::ogps_GetPointType
    *
    * @param dataPoint Operate on this data point instance.
    */
   _OPENGPS_EXPORT OGPS_Int32 ogps_GetInt32(const OGPS_DataPointPtr dataPoint);

   /*!
    * Gets the stored value of type ::OGPS_Float out of a given data point instance.
    *
    * @remarks If the current type does not equal ::OGPS_Float, this function returns the value 0.0.
    * You may check ::ogps_HasError whether this operation has been successfully executed.
    *
    * @see ::ogps_GetPointType
    *
    * @param dataPoint Operate on this data point instance.
    */
   _OPENGPS_EXPORT OGPS_Float ogps_GetFloat(const OGPS_DataPointPtr dataPoint);

   /*!
    * Gets the stored value of type ::OGPS_Double out of a given data point instance.
    *
    * @remarks If the current type does not equal ::OGPS_Double, this function returns the value 0.0.
    * You may check ::ogps_HasError whether this operation has been successfully executed.
    *
    * @see ::ogps_GetPointType
    *
    * @param dataPoint Operate on this data point instance.
    */
   _OPENGPS_EXPORT OGPS_Double ogps_GetDouble(const OGPS_DataPointPtr dataPoint);

   /*!
    * Stores a new value within a given data point instance.
    *
    * Also adjusts the current type information reflecting this set operation, if necessary.
    * You may check ::ogps_HasError whether this operation has been successfully executed.
    *
    * @param dataPoint Operate on this data point instance.
    * @param value The new value to be stored.
    */
   _OPENGPS_EXPORT void ogps_SetInt16(
      OGPS_DataPointPtr const dataPoint,
      const OGPS_Int16 value);

   /*!
    * Stores a new value within a given data point instance.
    *
    * Also adjusts the current type information reflecting this set operation, if necessary.
    * You may check ::ogps_HasError whether this operation has been successfully executed.
    *
    * @param dataPoint Operate on this data point instance.
    * @param value The new value to be stored.
    */
   _OPENGPS_EXPORT void ogps_SetInt32(
      OGPS_DataPointPtr const dataPoint,
      const OGPS_Int32 value);

   /*!
    * Stores a new value within a given data point instance.
    *
    * Also adjusts the current type information reflecting this set operation, if necessary.
    * You may check ::ogps_HasError whether this operation has been successfully executed.
    *
    * @param dataPoint Operate on this data point instance.
    * @param value The new value to be stored.
    */
   _OPENGPS_EXPORT void ogps_SetFloat(
      OGPS_DataPointPtr const dataPoint,
      const OGPS_Float value);

   /*!
    * Stores a new value within a given data point instance.
    *
    * Also adjusts the current type information reflecting this set operation, if necessary.
    * You may check ::ogps_HasError whether this operation has been successfully executed.
    *
    * @param dataPoint Operate on this data point instance.
    * @param value The new value to be stored.
    */
   _OPENGPS_EXPORT void ogps_SetDouble(
      OGPS_DataPointPtr const dataPoint,
      const OGPS_Double value);

#ifdef __cplusplus
}
#endif

#endif /* _OPENGPS_DATA_POINT_H */
/*! @} */

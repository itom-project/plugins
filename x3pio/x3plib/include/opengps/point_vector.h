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
 * The abstract data type of a point vector which holds values for
 * all components of 3D point data stored in an ISO 5436-2 X3P file.
 */

#ifndef _OPENGPS_POINT_VECTOR_H
#define _OPENGPS_POINT_VECTOR_H

#ifndef _OPENGPS_H
#    include <opengps/opengps.h>
#endif

#ifndef _OPENGPS_DATA_POINT_H
#    include <opengps/data_point.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

   /* TODO: HACK: with the typedef, the const qualifier is not reconised any more.
   #define OGPS_PointVectorPtr struct _OGPS_POINT_VECTOR * */

   /*!
    * Typesafe representation of three-dimensional point measurement data.
    *
    * The corresponding C++ implementation is provided by OpenGPS::PointVector.
    */
   typedef struct _OGPS_POINT_VECTOR * OGPS_PointVectorPtr;

   /* create* functions. */
   /* If instance points to NULL new pre-initalized memory will be allocated
   * and true is returned. Otherwise nothing will happen and false is returned.
   Must be freed with ogps_FreePointVector. */
   /*!
    * Creates an instance of type ::OGPS_PointVectorPtr.
    *
    * @remarks If your create an ::OGPS_PointVectorPtr object using
    * this function, you must call ::ogps_FreePointVector to release the object.
    *
    * @returns Returns an instance of type ::OGPS_PointVectorPtr.
    */
   _OPENGPS_EXPORT OGPS_PointVectorPtr ogps_CreatePointVector(void);

   /*!
    * Frees an instance of a point vector.
    *
    * @see ::ogps_CreatePointVector
    *
    * @param vector Operate on this point vector instance.
    */
   _OPENGPS_EXPORT void ogps_FreePointVector(OGPS_PointVectorPtr * const vector);

   /*!
    * Sets the new value for the x component of a given point vector.
    *
    * @param vector Operate on this point vector instance.
    * @param value The new value.
    */
   _OPENGPS_EXPORT void ogps_SetInt16X(
      OGPS_PointVectorPtr const vector,
      const OGPS_Int16 value);

   /*!
    * Sets the new value for the x component of a given point vector.
    *
    * @param vector Operate on this point vector instance.
    * @param value The new value.
    */
   _OPENGPS_EXPORT void ogps_SetInt32X(
      OGPS_PointVectorPtr const vector,
      const OGPS_Int32 value);

   /*!
    * Sets the new value for the x component of a given point vector.
    *
    * @param vector Operate on this point vector instance.
    * @param value The new value.
    */
   _OPENGPS_EXPORT void ogps_SetFloatX(
      OGPS_PointVectorPtr const vector,
      const OGPS_Float value);

   /*!
    * Sets the new value for the x component of a given point vector.
    *
    * @param vector Operate on this point vector instance.
    * @param value The new value.
    */
   _OPENGPS_EXPORT void ogps_SetDoubleX(
      OGPS_PointVectorPtr const vector,
      const OGPS_Double value);

   /*!
    * Sets the new value for the y component of a given point vector.
    *
    * @param vector Operate on this point vector instance.
    * @param value The new value.
    */
   _OPENGPS_EXPORT void ogps_SetInt16Y(
      OGPS_PointVectorPtr const vector,
      const OGPS_Int16 value);

   /*!
    * Sets the new value for the y component of a given point vector.
    *
    * @param vector Operate on this point vector instance.
    * @param value The new value.
    */
   _OPENGPS_EXPORT void ogps_SetInt32Y(
      OGPS_PointVectorPtr const vector,
      const OGPS_Int32 value);

   /*!
    * Sets the new value for the y component of a given point vector.
    *
    * @param vector Operate on this point vector instance.
    * @param value The new value.
    */
   _OPENGPS_EXPORT void ogps_SetFloatY(
      OGPS_PointVectorPtr const vector,
      const OGPS_Float value);

   /*!
    * Sets the new value for the y component of a given point vector.
    *
    * @param vector Operate on this point vector instance.
    * @param value The new value.
    */
   _OPENGPS_EXPORT void ogps_SetDoubleY(
      OGPS_PointVectorPtr const vector,
      const OGPS_Double value);

   /*!
    * Sets the new value for the z component of a given point vector.
    *
    * @param vector Operate on this point vector instance.
    * @param value The new value.
    */
   _OPENGPS_EXPORT void ogps_SetInt16Z(
      OGPS_PointVectorPtr const vector,
      const OGPS_Int16 value);

   /*!
    * Sets the new value for the z component of a given point vector.
    *
    * @param vector Operate on this point vector instance.
    * @param value The new value.
    */
   _OPENGPS_EXPORT void ogps_SetInt32Z(
      OGPS_PointVectorPtr const vector,
      const OGPS_Int32 value);

   /*!
    * Sets the new value for the z component of a given point vector.
    *
    * @param vector Operate on this point vector instance.
    * @param value The new value.
    */
   _OPENGPS_EXPORT void ogps_SetFloatZ(
      OGPS_PointVectorPtr const vector,
      const OGPS_Float value);

   /*!
    * Sets the new value for the z component of a given point vector.
    *
    * @param vector Operate on this point vector instance.
    * @param value The new value.
    */
   _OPENGPS_EXPORT void ogps_SetDoubleZ(
      OGPS_PointVectorPtr const vector,
      const OGPS_Double value);

   /*!
    * Gets the values of each component of the given point vector.
    *
    * @param vector Operate on this point vector instance.
    * @param x Gets the value of the x component. If this parameter is set to NULL, this does nothing.
    * @param y Gets the value of the y component. If this parameter is set to NULL, this does nothing.
    * @param z Gets the value of the z component. If this parameter is set to NULL, this does nothing.
    */
   _OPENGPS_EXPORT void ogps_GetXYZ(
      const OGPS_PointVectorPtr vector,
      OGPS_Double* const x,
      OGPS_Double* const y,
      OGPS_Double* const z);

   /*!
    * Sets the values of each component of the given point vector.
    *
    * @param vector Operate on this point vector instance.
    * @param x Sets the value of the x component.
    * @param y Sets the value of the y component.
    * @param z Sets the value of the z component.
    */
   _OPENGPS_EXPORT void ogps_SetXYZ(
      OGPS_PointVectorPtr vector,
      const OGPS_Double x,
      const OGPS_Double y,
      const OGPS_Double z);

   /*!
    * Gets the x component of the given vector.
    *
    * @param vector Operate on this point vector instance.
    */
   _OPENGPS_EXPORT OGPS_DataPointPtr const ogps_GetX(OGPS_PointVectorPtr const vector);

   /*!
    * Gets the y component of the given vector.
    *
    * @param vector Operate on this point vector instance.
    */
   _OPENGPS_EXPORT OGPS_DataPointPtr const ogps_GetY(OGPS_PointVectorPtr const vector);

   /*!
    * Gets the z component of the given vector.
    *
    * @param vector Operate on this point vector instance.
    */
   _OPENGPS_EXPORT OGPS_DataPointPtr const ogps_GetZ(OGPS_PointVectorPtr const vector);

   /*!
    * Gets the type of points of the x component of the given vector.
    *
    * @param vector Operate on this point vector instance.
    */
   _OPENGPS_EXPORT OGPS_DataPointType ogps_GetPointTypeX(const OGPS_PointVectorPtr vector);

   /*!
    * Gets the value of the x component of the given vector.
    *
    * @remarks Returns 0 if there is a type mismatch and the value of the component is stored as a
    * data type other than ::OGPS_Int16.
    *
    * @see ::OGPS_DataPointType
    *
    * @param vector Operate on this point vector instance.
    */
   _OPENGPS_EXPORT OGPS_Int16 ogps_GetInt16X(const OGPS_PointVectorPtr vector);

   /*!
    * Gets the value of the x component of the given vector.
    *
    * @remarks Returns 0 if there is a type mismatch and the value of the component is stored as a
    * data type other than ::OGPS_Int32.
    *
    * @see ::OGPS_DataPointType
    *
    * @param vector Operate on this point vector instance.
    */
   _OPENGPS_EXPORT OGPS_Int32 ogps_GetInt32X(const OGPS_PointVectorPtr vector);

   /*!
    * Gets the value of the x component of the given vector.
    *
    * @remarks Returns 0.0 if there is a type mismatch and the value of the component is stored as a
    * data type other than ::OGPS_Float.
    *
    * @see ::OGPS_DataPointType
    *
    * @param vector Operate on this point vector instance.
    */
   _OPENGPS_EXPORT OGPS_Float ogps_GetFloatX(const OGPS_PointVectorPtr vector);

   /*!
    * Gets the value of the x component of the given vector.
    *
    * @remarks Returns 0.0 if there is a type mismatch and the value of the component is stored as a
    * data type other than ::OGPS_Double.
    *
    * @see ::OGPS_DataPointType
    *
    * @param vector Operate on this point vector instance.
    */
   _OPENGPS_EXPORT OGPS_Double ogps_GetDoubleX(const OGPS_PointVectorPtr vector);

   /*!
    * Gets the type of points of the y component of the given vector.
    *
    * @param vector Operate on this point vector instance.
    */
   _OPENGPS_EXPORT OGPS_DataPointType ogps_GetPointTypeY(const OGPS_PointVectorPtr vector);

   /*!
    * Gets the value of the y component of the given vector.
    *
    * @remarks Returns 0 if there is a type mismatch and the value of the component is stored as a
    * data type other than ::OGPS_Int16.
    *
    * @see ::OGPS_DataPointType
    *
    * @param vector Operate on this point vector instance.
    */
   _OPENGPS_EXPORT OGPS_Int16 ogps_GetInt16Y(const OGPS_PointVectorPtr vector);

   /*!
    * Gets the value of the y component of the given vector.
    *
    * @remarks Returns 0 if there is a type mismatch and the value of the component is stored as a
    * data type other than ::OGPS_Int32.
    *
    * @see ::OGPS_DataPointType
    *
    * @param vector Operate on this point vector instance.
    */
   _OPENGPS_EXPORT OGPS_Int32 ogps_GetInt32Y(const OGPS_PointVectorPtr vector);

   /*!
    * Gets the value of the y component of the given vector.
    *
    * @remarks Returns 0.0 if there is a type mismatch and the value of the component is stored as a
    * data type other than ::OGPS_Float.
    *
    * @see ::OGPS_DataPointType
    *
    * @param vector Operate on this point vector instance.
    */
   _OPENGPS_EXPORT OGPS_Float ogps_GetFloatY(const OGPS_PointVectorPtr vector);

   /*!
    * Gets the value of the y component of the given vector.
    *
    * @remarks Returns 0.0 if there is a type mismatch and the value of the component is stored as a
    * data type other than ::OGPS_Double.
    *
    * @see ::OGPS_DataPointType
    *
    * @param vector Operate on this point vector instance.
    */
   _OPENGPS_EXPORT OGPS_Double ogps_GetDoubleY(const OGPS_PointVectorPtr vector);

   /*!
    * Gets the type of points of the z component of the given vector.
    *
    * @param vector Operate on this point vector instance.
    */
   _OPENGPS_EXPORT OGPS_DataPointType ogps_GetPointTypeZ(const OGPS_PointVectorPtr vector);

   /*!
    * Gets the value of the z component of the given vector.
    *
    * @remarks Returns 0 if there is a type mismatch and the value of the component is stored as a
    * data type other than ::OGPS_Int16.
    *
    * @see ::OGPS_DataPointType
    *
    * @param vector Operate on this point vector instance.
    */
   _OPENGPS_EXPORT OGPS_Int16 ogps_GetInt16Z(const OGPS_PointVectorPtr vector);

   /*!
    * Gets the value of the z component of the given vector.
    *
    * @remarks Returns 0 if there is a type mismatch and the value of the component is stored as a
    * data type other than ::OGPS_Int32.
    *
    * @see ::OGPS_DataPointType
    *
    * @param vector Operate on this point vector instance.
    */
   _OPENGPS_EXPORT OGPS_Int32 ogps_GetInt32Z(const OGPS_PointVectorPtr vector);

   /*!
    * Gets the value of the z component of the given vector.
    *
    * @remarks Returns 0.0 if there is a type mismatch and the value of the component is stored as a
    * data type other than ::OGPS_Float.
    *
    * @see ::OGPS_DataPointType
    *
    * @param vector Operate on this point vector instance.
    */
   _OPENGPS_EXPORT OGPS_Float ogps_GetFloatZ(const OGPS_PointVectorPtr vector);

   /*!
    * Gets the value of the z component of the given vector.
    *
    * @remarks Returns 0.0 if there is a type mismatch and the value of the component is stored as a
    * data type other than ::OGPS_Double.
    *
    * @see ::OGPS_DataPointType
    *
    * @param vector Operate on this point vector instance.
    */
   _OPENGPS_EXPORT OGPS_Double ogps_GetDoubleZ(const OGPS_PointVectorPtr vector);

   /*!
    * Asks if the given point vector stores a valid data point.
    *
    * A valid point vector does not have components where some or all of its values are missing.
    * Missing or invalid points are indicated by ::OGPS_MissingPointType.
    *
    * @note When using ogps_GetCurrentCoord() to get the transformed coordinate,
    *       the resulting point is allways valid but some of the components may contain NaN values.
    *
    * @param vector Operate on this point vector instance.
    * @returns Returns TRUE if this point vector contains valid point components only, FALSE otherwise.
    *
    * @see ogps_GetCurrentCoord()
    */
   _OPENGPS_EXPORT OGPS_Boolean ogps_IsValidPoint(const OGPS_PointVectorPtr vector);

#ifdef __cplusplus
}
#endif

#endif /* _OPENGPS_POINT_VECTOR_H */

/*! @} */

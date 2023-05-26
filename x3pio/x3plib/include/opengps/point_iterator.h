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
 * The abstract data type of a point iterator used for direct manipulation
 * of point data stored in an ISO 5436-2 X3P file.
 */

#ifndef _OPENGPS_POINT_ITERATOR_H
#define _OPENGPS_POINT_ITERATOR_H

#ifndef _OPENGPS_H
#    include <opengps/opengps.h>
#endif

#ifndef _OPENGPS_POINT_VECTOR_H
#    include <opengps/point_vector.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

   /*!
    * Interface to a point iterator.
    *
    * The point iterator may be used to traverse all point vectors contained in
    * an ISO5436-2 XML X3P file format container.
    *
    * @remarks An instance of ::OGPS_PointIteratorPtr can be obtained from
    * ::ogps_CreateNextPointIterator or ::ogps_CreatePrevPointIterator.
    * You must free an instance of type ::OGPS_PointIteratorPtr with
    * ::ogps_FreePointIterator when you done with it.
    *
    * The corresponding C++ implementation is provided by OpenGPS::PointIterator.
    */
   typedef struct _OGPS_POINT_ITERATOR * OGPS_PointIteratorPtr;

   /*!
    * Asks if there is another point available to iterate.
    *
    * @remarks Use this function with an iterator handle obtained from ::ogps_CreateNextPointIterator.
    *
    * @see ::ogps_MoveNextPoint, ::ogps_HasPrevPoint
    *
    * @param iterator Operate on this iterator handle.
    * @returns Returns TRUE if there is at least one more point available to iterate, FALSE otherwise.
    */
   _OPENGPS_EXPORT OGPS_Boolean ogps_HasNextPoint(const OGPS_PointIteratorPtr iterator);

   /*!
    * Asks if there is another point available to iterate.
    *
    * @remarks Use this function with an iterator handle obtained from ::ogps_CreatePrevPointIterator.
    *
    * @see ::ogps_MovePrevPoint, ::ogps_HasNextPoint
    *
    * @param iterator Operate on this iterator handle.
    * @returns Returns TRUE if there is at least one more point available to iterate, FALSE otherwise.
    */
   _OPENGPS_EXPORT OGPS_Boolean ogps_HasPrevPoint(const OGPS_PointIteratorPtr iterator);

   /*!
    * Moves the iterator forward.
    *
    * @remarks Use this function directly after initialising the iterator object with
    * ::ogps_CreateNextPointIterator to move to the first point.
    *
    * @see ::ogps_HasNextPoint
    *
    * @param iterator Operate on this iterator handle.
    * @returns Returns TRUE on success, FALSE otherwise.
    */
   _OPENGPS_EXPORT OGPS_Boolean ogps_MoveNextPoint(OGPS_PointIteratorPtr const iterator);

   /*!
    * Moves the iterator backward.
    *
    * @remarks Use this function directly after initialising the iterator object with
    * ::ogps_CreatePrevPointIterator to move to the first point.
    *
    * @see ::ogps_HasPrevPoint
    *
    * @param iterator Operate on this iterator handle.
    * @returns Returns TRUE on success, FALSE otherwise.
    */
   _OPENGPS_EXPORT OGPS_Boolean ogps_MovePrevPoint(OGPS_PointIteratorPtr const iterator);

   /*!
    * Resets the iterator to the beginning and turns this iterator instance into a forward iterator.
    *
    * @param iterator Operate on this iterator handle.
    */
   _OPENGPS_EXPORT void ogps_ResetNextPointIterator(OGPS_PointIteratorPtr const iterator);

   /*!
    * Resets the iterator to the beginning and turns this iterator instance into a backward iterator.
    *
    * @param iterator Operate on this iterator handle.
    */
   _OPENGPS_EXPORT void ogps_ResetPrevPointIterator(OGPS_PointIteratorPtr const iterator);

   /*!
    * Gets the value of the current point vector.
    *
    * Retrieves the raw point data. No offsets and increments have been applied.
    *
    * @see ::ogps_GetCurrentCoord
    * @see ::ogps_MoveNext
    *
    * On failure you may get further information by calling ::ogps_GetErrorMessage hereafter.
    *
    * @param iterator Operate on this iterator handle.
    * @param vector Gets a copy of the vector at the current iterator position.
    */
   _OPENGPS_EXPORT void ogps_GetCurrentPoint(
      const OGPS_PointIteratorPtr iterator,
      OGPS_PointVectorPtr const vector);

    /*!
    * Gets the value of the current coordinate vector.
    *
    * Retrieves coordinates where offsets and increments have been readily applied to the underlying raw point data.
    *
    * @see ::ogps_GetCurrentPoint
    * @see ::ogps_MoveNext
    *
    * On failure you may get further information by calling ::ogps_GetErrorMessage hereafter.
    *
    * @param iterator Operate on this iterator handle.
    * @param vector Gets a copy of the vector at the current iterator position.
    */
   _OPENGPS_EXPORT void ogps_GetCurrentCoord(
      const OGPS_PointIteratorPtr iterator,
      OGPS_PointVectorPtr const vector);

   /*!
    * Sets the value of the current point vector.
    *
    * @see ::ogps_MoveNext
    *
    * On failure you may get further information by calling ::ogps_GetErrorMessage hereafter.
    *
    * @param iterator Operate on this iterator handle.
    * @param vector New value of the current point vector. May be NULL to indicate an invalid point.
    */
   _OPENGPS_EXPORT void ogps_SetCurrentPoint(
      const OGPS_PointIteratorPtr iterator,
      const OGPS_PointVectorPtr vector);

   /*!
    * Gets the current position of the iterator in topology coordinates.
    *
    * @see ::ogps_MoveNext, ::ogps_GetListPosition
    *
    * @param iterator Operate on this iterator handle.
    * @param u Gets the position index of the u component of the surface. If this parameter is set to NULL, it will be safely ignored and nothing returns here.
    * @param v Gets the position index of the v component of the surface. If this parameter is set to NULL, it will be safely ignored and nothing returns here.
    * @param w Gets the position index of the w component of the surface. If this parameter is set to NULL, it will be safely ignored and nothing returns here.
    * @returns Returns TRUE on success, FALSE otherwise.
    */
   _OPENGPS_EXPORT OGPS_Boolean ogps_GetMatrixPosition(
      const OGPS_PointIteratorPtr iterator,
      OGPS_ULong * u,
      OGPS_ULong * v,
      OGPS_ULong * w);

   /*!
    * Gets the current position of the iterator.
    *
    * @see ::ogps_MoveNext, ::ogps_GetMatrixPosition
    *
    * @param iterator Operate on this iterator handle.
    * @param index Gets the position index.
    * @returns Returns TRUE on success, FALSE otherwise.
    */
   _OPENGPS_EXPORT OGPS_Boolean ogps_GetListPosition(
      const OGPS_PointIteratorPtr iterator,
      OGPS_ULong * index);

   /*!
    * Frees the point iterator instance.
    *
    * @remarks Must be called for every point iterator handle obtained through
    * ::ogps_CreateNextPointIterator or ::ogps_CreatePrevPointIterator.
    *
    * @param iterator Operate on this iterator handle.
    */
   _OPENGPS_EXPORT void ogps_FreePointIterator(OGPS_PointIteratorPtr * const iterator);

#ifdef __cplusplus
}
#endif

#endif /* _OPENGPS_POINT_ITERATOR_H */
/*! @} */

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

/*! @addtogroup Cpp
   @{ */

/*! @file
 * Interface to a point iterator used for direct manipulation
 * of point data stored in an ISO 5436-2 X3P file.
 */

#ifndef _OPENGPS_CXX_POINT_ITERATOR_HXX
#define _OPENGPS_CXX_POINT_ITERATOR_HXX

#ifndef _OPENGPS_CXX_OPENGPS_HXX
#  include <opengps/cxx/opengps.hxx>
#endif

namespace OpenGPS
{
   class ISO5436_2;
   class PointVector;

   /*!
    * Interface to a point iterator.
    *
    * The point iterator may be used to traverse all point vectors contained in
    * an ISO5436-2 XML X3P file format container.
    *
    * @remarks An instance of OpenGPS::PointIterator can be obtained from
    * OpenGPS::ISO5436_2::CreateNextPointIterator or OpenGPS::ISO5436_2::CreatePrevPointIterator.
    */
   class _OPENGPS_EXPORT PointIterator
   {
   protected:
      /*!
       * Creates a new instance.
       */
      PointIterator();

   public:
      /*!
       * Destructs this instance.
       */
      virtual ~PointIterator();

      /*!
       * Asks if there is another point available to iterate.
       *
       * @remarks Use this function with an iterator handle obtained from OpenGPS::ISO5436_2::CreateNextPointIterator.
       *
       * @see PointIterator::MoveNext, PointIterator::HasPrev
       *
       * @returns Returns TRUE if there is at least one more point available to iterate, FALSE otherwise.
       */
      virtual OGPS_Boolean HasNext() const = 0;

      /*!
       * Asks if there is another point available to iterate.
       *
       * @remarks Use this function with an iterator handle obtained from OpenGPS::ISO5436_2::CreatePrevPointIterator.
       *
       * @see PointIterator::MovePrev, PointIterator::HasNext
       *
       * @returns Returns TRUE if there is at least one more point available to iterate, FALSE otherwise.
       */
      virtual OGPS_Boolean HasPrev() const = 0;

      /*!
       * Moves the iterator forward.
       *
       * @remarks Use this function directly after initialising the iterator object with
       * OpenGPS::ISO5436_2::CreateNextPointIterator to move to the first point.
       *
       * @see PointIterator::HasNext
       *
       * @returns Returns TRUE on success, FALSE otherwise.
       */
      virtual OGPS_Boolean MoveNext() = 0;

      /*!
       * Moves the iterator backward.
       *
       * @remarks Use this function directly after initialising the iterator object with
       * OpenGPS::ISO5436_2::CreatePrevPointIterator to move to the first point.
       *
       * @see PointIterator::HasPrev
       *
       * @returns Returns TRUE on success, FALSE otherwise.
       */
      virtual OGPS_Boolean MovePrev() = 0;

      /*!
       * Resets the iterator to the beginning and turns this iterator instance into a forward iterator.
       */
      virtual void ResetNext() = 0;

      /*!
       * Resets the iterator to the beginning and turns this iterator instance into a backward iterator.
       */
      virtual void ResetPrev() = 0;

      /*!
       * Gets the value of the current point vector.
       *
       * Retrieves the raw point data without offsets and increments applied.
       * A specific instance may throw an OpenGPS::Exception on failure,
       *
       * @see PointIterator::GetCurrentCoord
       * @see PointIterator::MoveNext
       *
       * @param vector Gets a copy of the vector at the current iterator position.
       */
      virtual void GetCurrent(PointVector& vector) = 0;

      /*!
       * Gets the value of the current coordinate vector.
       *
       * Retrieves coordinate data, where offsets and increments have been readily applied to the underlying point data.
       * A specific instance may throw an OpenGPS::Exception on failure,
       *
       * @see PointIterator::GetCurrent
       * @see PointIterator::MoveNext
       *
       * @param vector Gets a copy of the coordinate at the current iterator position.
       */
      virtual void GetCurrentCoord(PointVector& vector) = 0;

      /*!
       * Sets the value of the current point vector.
       *
       * A specific instance may throw an OpenGPS::Exception on failure,
       *
       * @see PointIterator::MoveNext
       *
       * @param vector New value of the current point vector. May be NULL to indicate an invalid point.
       * @returns Returns TRUE on success, FALSE otherwise.
       */
      virtual void SetCurrent(const PointVector* const vector) = 0;

      /*!
       * Gets the current position of the iterator in topology coordinates.
       *
       * @see PointIterator::MoveNext
       *
       * @param u Gets the position index of the u component of the surface. If this parameter is set to NULL, it will be safely ignored and nothing returns here.
       * @param v Gets the position index of the v component of the surface. If this parameter is set to NULL, it will be safely ignored and nothing returns here.
       * @param w Gets the position index of the w component of the surface. If this parameter is set to NULL, it will be safely ignored and nothing returns here.
       * @returns Returns TRUE on success, FALSE otherwise.
       */
      virtual OGPS_Boolean GetPosition(
         OGPS_ULong * const u,
         OGPS_ULong * const v,
         OGPS_ULong * const w) const = 0;

      /*!
       * Gets the current position of the iterator.
       *
       * @see PointIterator::MoveNext
       *
       * @param index Gets the position index.
       * @returns Returns TRUE on success, FALSE otherwise.
       */
      virtual OGPS_Boolean GetPosition(OGPS_ULong * const index) const = 0;
   };
}

#endif /* _OPENGPS_CXX_POINT_ITERATOR_HXX */

/*! @} */

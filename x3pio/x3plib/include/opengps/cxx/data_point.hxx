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
 * Typesafe representation of a single data point value.
 * The interface supports getting and setting an encapsulated data value of a predetermined type.
 */

#ifndef _OPENGPS_CXX_DATA_POINT_HXX
#define _OPENGPS_CXX_DATA_POINT_HXX

#ifndef _OPENGPS_CXX_OPENGPS_HXX
#  include <opengps/cxx/opengps.hxx>
#endif

#ifndef _OPENGPS_DATA_POINT_TYPE_H
#  include <opengps/data_point_type.h>
#endif

namespace OpenGPS
{
   /*!
   * Typesafe representation of a single data point value.
   *
   * In an OpenGPS::PointVector every component of that three-vector is
   * accessible as its own OpenGPS::DataPoint instance.
   *
   * @remarks An instance of OpenGPS::DataPoint cannot be created of its own.
   * Indirectly a handle of this type can be accessed through an OpenGPS::PointVector object.
   */
   class _OPENGPS_EXPORT DataPoint
   {
   protected:
      /*! Creates and initializes a new object. */
      DataPoint();

   public:
      /*! Destructs this object. */
      virtual ~DataPoint() = 0;

      /*!
      * Gets type information of the current value stored within this data point.
      *
      * A specific implementation may throw an OpenGPS::Exception if this operation
      * is not permitted due to the current state of the object instance.
      *
      * @returns Returns ::OGPS_MissingPointType if this instance does not store any value at all.
      */
      virtual OGPS_DataPointType GetPointType() const = 0;

      /*!
      * Gets the stored value.
      *
      * @remarks If the current type does not equal ::OGPS_Int16, the behavior is undefined.
      * @see DataPoint::GetPointType
      *
      * A specific implementation may throw an OpenGPS::Exception if this operation
      * is not permitted due to the current state of the object instance.
      *
      * @param value Destination for the currently stored value.
      */
      virtual void Get(OGPS_Int16* const value) const = 0;

      /*!
      * Gets the stored value.
      *
      * @remarks If the current type does not equal ::OGPS_Int32, the behavior is undefined.
      * @see DataPoint::GetPointType
      *
      * A specific implementation may throw an OpenGPS::Exception if this operation
      * is not permitted due to the current state of the object instance.
      *
      * @param value Destination for the currently stored value.
      */
      virtual void Get(OGPS_Int32* const value) const = 0;

      /*!
      * Gets the stored value.
      *
      * @remarks If the current type does not equal ::OGPS_Float, the behavior is undefined.
      * @see DataPoint::GetPointType
      *
      * A specific implementation may throw an OpenGPS::Exception if this operation
      * is not permitted due to the current state of the object instance.
      *
      * @param value Destination for the currently stored value.
      */
      virtual void Get(OGPS_Float* const value) const = 0;

      /*!
      * Gets the stored value.
      *
      * @remarks If the current type does not equal ::OGPS_Double, the behavior is undefined.
      * @see DataPoint::GetPointType
      *
      * A specific implementation may throw an OpenGPS::Exception if this operation
      * is not permitted due to the current state of the object instance.
      *
      * @param value Destination for the currently stored value.
      */
      virtual void Get(OGPS_Double* const value) const = 0;

      /*!
      * Gets the stored value.
      *
      * A specific implementation may throw an OpenGPS::Exception if this operation
      * is not permitted due to the current state of the object instance.
      *
      * @returns Returns the stored value or 0.0 if this data point is empty (DataPoint::GetPointType returns ::OGPS_MissingPointType in this case).
      */
      virtual OGPS_Double Get() const = 0;

      /*!
      * Asks whether there is a valid point value stored within this instance currently.
      * This function test whether a point value is stored and in case it is a floating
      * point value whether it is Not-a-Number (NaN).
      *
      * @see DataPoint::GetPointType
      *
      * A specific implementation may throw an OpenGPS::Exception if this operation
      * is not permitted due to the current state of the object instance.
      *
      * @returns Returns TRUE if a value is stored, FALSE otherwise.
      */
      virtual OGPS_Boolean IsValid() const = 0;

      /*!
      * Stores a new value.
      *
      * Also adjusts the current type information reflecting this set operation, if necessary.
      *
      * A specific implementation may throw an OpenGPS::Exception if this operation
      * is not permitted due to the current state of the object instance.
      *
      * @param value The new value to be stored.
      */
      virtual void Set(const OGPS_Int16 value) = 0;

      /*!
      * Stores a new value.
      *
      * Also adjusts the current type information reflecting this set operation, if necessary.
      *
      * A specific implementation may throw an OpenGPS::Exception if this operation
      * is not permitted due to the current state of the object instance.
      *
      * @param value The new value to be stored.
      */
      virtual void Set(const OGPS_Int32 value) = 0;

      /*!
      * Stores a new value.
      *
      * Also adjusts the current type information reflecting this set operation, if necessary.
      *
      * A specific implementation may throw an OpenGPS::Exception if this operation
      * is not permitted due to the current state of the object instance.
      *
      * @param value The new value to be stored.
      */
      virtual void Set(const OGPS_Float value) = 0;

      /*!
      * Stores a new value.
      *
      * Also adjusts the current type information reflecting this set operation, if necessary.
      *
      * A specific implementation may throw an OpenGPS::Exception if this operation
      * is not permitted due to the current state of the object instance.
      *
      * @param value The new value to be stored.
      */
      virtual void Set(const OGPS_Double value) = 0;

      /*!
      * Stores a new value. Copies its new entry from another ::DataPoint instance.
      *
      * A specific implementation may throw an OpenGPS::Exception if this operation
      * is not permitted due to the current state of the object instance.
      *
      * @param src Object to copy from.
      */
      virtual void Set(const DataPoint& src) = 0;

      /*!
      * Resets this instance to its initial state.
      *
      * A specific implementation may throw an OpenGPS::Exception if this operation
      * is not permitted due to the current state of the object instance.
      */
      virtual void Reset() = 0;

   private:
      /*! The copy-ctor is not implemented. This prevents its usage. */
      DataPoint(const DataPoint& src);
      /*! The assignment-operator is not implemented. This prevents its usage. */
      DataPoint& operator=(const DataPoint& src);
   };
}

#endif /* _OPENGPS_CXX_DATA_POINT_HXX */

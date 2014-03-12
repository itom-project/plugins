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
 * Typesafe representation of three-dimensional point measurement data.
 * A point vector holds all three components of 3D point data stored in an ISO 5436-2 X3P file
 * and provides methods for typesafe access.
 */

#ifndef _OPENGPS_CXX_POINT_VECTOR_HXX
#define _OPENGPS_CXX_POINT_VECTOR_HXX

#ifndef _OPENGPS_CXX_OPENGPS_HXX
#  include <opengps/cxx/opengps.hxx>
#endif

#ifndef _OPENGPS_CXX_POINT_VECTOR_BASE_HXX
#  include <opengps/cxx/point_vector_base.hxx>
#endif

namespace OpenGPS
{
   class DataPoint;

   /*!
    * Typesafe representation of three-dimensional point measurement data.
    */
   class _OPENGPS_EXPORT PointVector : public PointVectorBase
   {
   public:
      /*! Creates a new instance. */
      PointVector();

      /*! Destructs this object. */
      ~PointVector();

      /*!
       * Gets typesafe direct read-only access to the x component.
       */
      virtual const DataPoint* GetX() const;

      /*!
       * Gets typesafe direct read-only access to the y component.
       */
      virtual const DataPoint* GetY() const;

      /*!
       * Gets typesafe direct read-only access to the z component.
       */
      virtual const DataPoint* GetZ() const;

      /*!
       * Gets typesafe direct access to the x component.
       */
      virtual DataPoint* GetX();

      /*!
       * Gets typesafe direct access to the y component.
       */
      virtual DataPoint* GetY();

      /*!
       * Gets typesafe direct access to the z component.
       */
      virtual DataPoint* GetZ();

      /*!
       * Copies values from another OpenGPS::PointVector instance.
       *
       * Throws an OpenGPS::Exception if this operation is not permitted
       * due to the current state of the object instance.
       *
       * @param value OpenGPS::PointVectorBase object to copy from.
       */
      virtual void Set(const PointVectorBase& value);

      /*!
       * Copies values to another OpenGPS::PointVectorBase instance.
       *
       * Throws an OpenGPS::Exception if this operation is not permitted
       * due to the current state of the object instance.
       *
       * @param value Retrieves values from the current instance as a copy.
       */
      virtual void Get(PointVectorBase& value) const;

      /*!
       * Gets the value of the x component of the given vector.
       *
       * Throws an OpenGPS::Exception if this operation is not permitted
       * due to the current state of the object instance or conflicting types.
       *
       * @see ::OGPS_DataPointType
       *
       * @param value Stores the current value on success.
       */
      void GetX(OGPS_Int16* const value) const;

      /*!
       * Gets the value of the x component of the given vector.
       *
       * Throws an OpenGPS::Exception if this operation is not permitted
       * due to the current state of the object instance or conflicting types.
       *
       * @see ::OGPS_DataPointType
       *
       * @param value Stores the current value on success.
       */
      void GetX(OGPS_Int32* const value) const;

      /*!
       * Gets the value of the x component of the given vector.
       *
       * Throws an OpenGPS::Exception if this operation is not permitted
       * due to the current state of the object instance or conflicting types.
       *
       * @see ::OGPS_DataPointType
       *
       * @param value Stores the current value on success.
       */
      void GetX(OGPS_Float* const value) const;

      /*!
       * Gets the value of the x component of the given vector.
       *
       * Throws an OpenGPS::Exception if this operation is not permitted
       * due to the current state of the object instance or conflicting types.
       *
       * @see ::OGPS_DataPointType
       *
       * @param value Stores the current value on success.
       */
      void GetX(OGPS_Double* const value) const;

      /*!
       * Gets the value of the y component of the given vector.
       *
       * Throws an OpenGPS::Exception if this operation is not permitted
       * due to the current state of the object instance or conflicting types.
       *
       * @see ::OGPS_DataPointType
       *
       * @param value Stores the current value on success.
       */
      void GetY(OGPS_Int16* const value) const;

      /*!
       * Gets the value of the y component of the given vector.
       *
       * Throws an OpenGPS::Exception if this operation is not permitted
       * due to the current state of the object instance or conflicting types.
       *
       * @see ::OGPS_DataPointType
       *
       * @param value Stores the current value on success.
       */
      void GetY(OGPS_Int32* const value) const;

      /*!
       * Gets the value of the y component of the given vector.
       *
       * Throws an OpenGPS::Exception if this operation is not permitted
       * due to the current state of the object instance or conflicting types.
       *
       * @see ::OGPS_DataPointType
       *
       * @param value Stores the current value on success.
       */
      void GetY(OGPS_Float* const value) const;

      /*!
       * Gets the value of the y component of the given vector.
       *
       * Throws an OpenGPS::Exception if this operation is not permitted
       * due to the current state of the object instance or conflicting types.
       *
       * @see ::OGPS_DataPointType
       *
       * @param value Stores the current value on success.
       */
      void GetY(OGPS_Double* const value) const;

      /*!
       * Gets the value of the z component of the given vector.
       *
       * Throws an OpenGPS::Exception if this operation is not permitted
       * due to the current state of the object instance or conflicting types.
       *
       * @see ::OGPS_DataPointType
       *
       * @param value Stores the current value on success.
       */
      void GetZ(OGPS_Int16* const value) const;

      /*!
       * Gets the value of the z component of the given vector.
       *
       * Throws an OpenGPS::Exception if this operation is not permitted
       * due to the current state of the object instance or conflicting types.
       *
       * @see ::OGPS_DataPointType
       *
       * @param value Stores the current value on success.
       */
      void GetZ(OGPS_Int32* const value) const;

      /*!
       * Gets the value of the z component of the given vector.
       *
       * Throws an OpenGPS::Exception if this operation is not permitted
       * due to the current state of the object instance or conflicting types.
       *
       * @see ::OGPS_DataPointType
       *
       * @param value Stores the current value on success.
       */
      void GetZ(OGPS_Float* const value) const;

      /*!
       * Gets the value of the z component of the given vector.
       *
       * Throws an OpenGPS::Exception if this operation is not permitted
       * due to the current state of the object instance or conflicting types.
       *
       * @see ::OGPS_DataPointType
       *
       * @param value Stores the current value on success.
       */
      void GetZ(OGPS_Double* const value) const;

      /*!
       * Sets the new value for the x component.
       *
       * @param value The new value.
       */
      void SetX(const OGPS_Int16 value);

      /*!
       * Sets the new value for the x component.
       *
       * @param value The new value.
       */
      void SetX(const OGPS_Int32 value);

      /*!
       * Sets the new value for the x component.
       *
       * @param value The new value.
       */
      void SetX(const OGPS_Float value);

      /*!
       * Sets the new value for the x component.
       *
       * @param value The new value.
       */
      void SetX(const OGPS_Double value);

      /*!
       * Sets the new value for the y component.
       *
       * @param value The new value.
       */
      void SetY(const OGPS_Int16 value);

      /*!
       * Sets the new value for the y component.
       *
       * @param value The new value.
       */
      void SetY(const OGPS_Int32 value);

      /*!
       * Sets the new value for the y component.
       *
       * @param value The new value.
       */
      void SetY(const OGPS_Float value);

      /*!
       * Sets the new value for the y component.
       *
       * @param value The new value.
       */
      void SetY(const OGPS_Double value);

      /*!
       * Sets the new value for the z component.
       *
       * @param value The new value.
       */
      void SetZ(const OGPS_Int16 value);

      /*!
       * Sets the new value for the z component.
       *
       * @param value The new value.
       */
      void SetZ(const OGPS_Int32 value);

      /*!
       * Sets the new value for the z component.
       *
       * @param value The new value.
       */
      void SetZ(const OGPS_Float value);

      /*!
       * Sets the new value for the z component.
       *
       * @param value The new value.
       */
      void SetZ(const OGPS_Double value);

      /*!
       * Gets the values of each component.
       *
       * @param x Gets the value of the x component. If this parameter is set to NULL, this does nothing.
       * @param y Gets the value of the y component. If this parameter is set to NULL, this does nothing.
       * @param z Gets the value of the z component. If this parameter is set to NULL, this does nothing.
       */
      void GetXYZ(
         OGPS_Double* const x,
         OGPS_Double* const y,
         OGPS_Double* const z) const;

      /*!
       * Sets the values of each component.
       *
       * @param x Sets the value of the x component.
       * @param y Sets the value of the y component.
       * @param z Sets the value of the z component.
       */
      void SetXYZ(
         const OGPS_Double x,
         const OGPS_Double y,
         const OGPS_Double z) const;

      /*!
       * Asks if this point vector stores a valid data point.
       *
       * A valid point vector does not have components where some or all of its values are missing.
       * Missing or invalid points are indicated by ::OGPS_MissingPointType.
       *
       * @returns Returns TRUE if this point vector contains valid point components only, FALSE otherwise.
       */
      virtual OGPS_Boolean IsValid() const;

      PointVector& operator=(const PointVector& src);

   private:
      DataPoint* m_X;
      DataPoint* m_Y;
      DataPoint* m_Z;
   };
}

#endif /* _OPENGPS_CXX_POINT_VECTOR_HXX */
/*! @} */

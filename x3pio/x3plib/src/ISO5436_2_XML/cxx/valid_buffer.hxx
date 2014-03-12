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
 * Communicate validity of point vectors through external binary file.
 */

#ifndef _OPENGPS_VALID_BUFFER_HXX
#define _OPENGPS_VALID_BUFFER_HXX

#ifndef _OPENGPS_CXX_OPENGPS_HXX
#  include <opengps/cxx/opengps.hxx>
#endif

#ifndef _OPENGPS_POINT_VALIDITY_PROVIDER_HXX
#  include "point_validity_provider.hxx"
#endif

#include <iostream>

namespace OpenGPS
{
   /*!
    * Implements the OpenGPS::PointValidityProvider as an external binary file.
    *
    * All bit values of the binary file correspond to the location of the point
    * vector of the same index. If the file bit is on (set to one) the point
    * vector is valid, if the bit at the given index is off, then the point
    * vector at the corresponding location has invalid data.
    */
   class ValidBuffer : public PointValidityProvider
   {
   public:
      /*! Destroys this instance. */
      virtual ~ValidBuffer();


      /*! Returns wheter the bit buffer has already been allocated. */
      virtual OGPS_Boolean IsAllocated() const;

      /*!
       * Maps the bit buffer from a binary stream.
       * @param stream The bit array gets copied from here.
       */
      virtual void Read(std::basic_istream<OpenGPS::Byte>& stream);

      /*!
       * Maps the bit buffer to a binary stream.
       * @param stream The internal bit array gets written to the given stream.
       * @returns Returns TRUE on success, FALSE otherwise.
       */
      virtual void Write(std::ostream& stream);

      /*!
       * Checks whether the current instance serves any point data that is marked as invalid.
       * @returns FALSE if none of the referenced point data is marked as invalid here, otherwise TRUE.
       */
      virtual OGPS_Boolean HasInvalidMarks() const;

      virtual void SetValid(const unsigned int index, const OGPS_Boolean value);
      virtual OGPS_Boolean IsValid(const unsigned int index) const;

   protected:
      /*!
       * Creates a new instance.
       * @param value The point buffer of the Z axis.
       */
      ValidBuffer(PointBuffer* const value);

      /*! Allocates the internal bit array. Initially all point vectors are assumed to be valid. */
      virtual void Allocate();

      /*! Frees allocated resources. */
      virtual void Reset();

   private:
      /*! Not to be publicly used ctor. */
      ValidBuffer();

      /*!
       * Allocates the internal bit array. Initially all point vectors are assumed to be valid.
       * @param rawSize Amount of memory to be allocated in bytes.
       * @returns Returns TRUE on success, FALSE otherwise.
       */
      virtual void AllocateRaw(const unsigned int rawSize);

      /*! Pointer to the internal bit array. */
      OpenGPS::UnsignedBytePtr m_ValidityBuffer;

      /*! Size of the bit array in bytes. */
      OGPS_ULong m_RawSize;
   };

   /*!
    * Implementation of OpenGPS::ValidBuffer for Z axis of ::OGPS_Int16 data type.
    */
   class Int16ValidBuffer : public ValidBuffer
   {
   public:
      /*!
       * Creates a new instance.
       * @param value The point buffer of the Z axis.
       */
      Int16ValidBuffer(PointBuffer* const value);

      /*! Destroys this instance. */
      ~Int16ValidBuffer();

      virtual void SetValid(const unsigned int index, const OGPS_Boolean value);
   };

   /*!
    * Implementation of OpenGPS::ValidBuffer for Z axis of ::OGPS_Int32 data type.
    */
   class Int32ValidBuffer : public ValidBuffer
   {
   public:
      /*!
       * Creates a new instance.
       * @param value The point buffer of the Z axis.
       */
      Int32ValidBuffer(PointBuffer* const value);

      /*! Destroys this instance. */
      ~Int32ValidBuffer();

      virtual void SetValid(const unsigned int index, const OGPS_Boolean value);
   };
}

#endif /* _OPENGPS_VALID_BUFFER_HXX */

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
 * Allocate static memory to store point data.
 */

#ifndef _OPENGPS_INT16_POINT_BUFFER_HXX
#define _OPENGPS_INT16_POINT_BUFFER_HXX

#ifndef _OPENGPS_POINT_BUFFER_HXX
#  include "point_buffer.hxx"
#endif

namespace OpenGPS
{
   /*!
    * Manages static memory and typesafe access.
    * Allocates an internal memory buffer to store point data of type ::OGPS_Int16.
    */
   class Int16PointBuffer : public PointBuffer
   {
   public:
      /*! Create a new instance. */
      Int16PointBuffer();

      /*! Destroys this instance. */
      virtual ~Int16PointBuffer();

      virtual void Allocate(const OGPS_ULong size);

      virtual void Set(const OGPS_ULong index, const OGPS_Int16 value);
      virtual void Get(const OGPS_ULong index, OGPS_Int16& value) const;

      virtual OGPS_DataPointType GetPointType() const;

   private:
      /*! Pointer to internal memory. */
      OGPS_Int16* m_Buffer;
   };
}

#endif /* _OPENGPS_INT16_POINT_BUFFER_HXX */

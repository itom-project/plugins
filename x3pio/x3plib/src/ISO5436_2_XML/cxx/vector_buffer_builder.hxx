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
 * Methods to assemble a OpenGPS::VectorBuffer instance.
 */

#ifndef _OPENGPS_VECTOR_BUFFER_BUILDER_HXX
#define _OPENGPS_VECTOR_BUFFER_BUILDER_HXX

#ifndef _OPENGPS_CXX_OPENGPS_HXX
#  include <opengps/cxx/opengps.hxx>
#endif

#ifndef _OPENGPS_DATA_POINT_TYPE_H
#  include <opengps/data_point_type.h>
#endif

namespace OpenGPS
{
   class PointBuffer;
   class VectorBuffer;

   /*!
    * Creates an object which is able to assemble a OpenGPS::VectorBuffer instance.
    */
   class VectorBufferBuilder
   {
   public:
      /*! Creates a new instance. */
      VectorBufferBuilder();

      /*! Destroys thisinstance. */
      ~VectorBufferBuilder();

      /*!
       * Creates the initial OpenGPS::VectorBuffer to be assembled.
       * @remarks This must preceed all other steps of the building process.
       */
      virtual OGPS_Boolean BuildBuffer();

      /*!
       * Connects the appropriate OpenGPS::PointBuffer connected with the X axis description.
       * @param dataType The type of point data connected to the X axis. A value of
       * ::OGPS_MissingPointType describes an incremental (non-explicit) axis for which
       * no point data needs to be stored.
       * @param size The amount of points stored for the X component of all available vectors.
       */
      virtual OGPS_Boolean BuildX(const OGPS_DataPointType dataType, const OGPS_ULong size);

      /*!
       * Connects the appropriate OpenGPS::PointBuffer connected with the Y axis description.
       * @param dataType The type of point data connected to the Y axis. A value of
       * ::OGPS_MissingPointType describes an incremental (non-explicit) axis for which
       * no point data needs to be stored.
       * @param size The amount of points stored for the Y component of all available vectors.
       */
      virtual OGPS_Boolean BuildY(const OGPS_DataPointType dataType, const OGPS_ULong size);

      /*!
       * Connects the appropriate OpenGPS::PointBuffer connected with the Z axis description.
       * @param dataType The type of point data connected to the X axis. Since the Z componet
       * is forced to be saved explicitly regardless whether the axis is defined as incremental
       * or not, a value of ::OGPS_MissingPointType is invalid here.
       * @param size The amount of points stored for the Z component of all available vectors.
       */
      virtual OGPS_Boolean BuildZ(const OGPS_DataPointType dataType, const OGPS_ULong size);

      /*!
       * Connects the appropriate OpenGPS::PointValidityProvider.
       * @param allowInvalidPoints Set this to TRUE, if invalid points are allowed to be contained
       * within point vector data, otherwise set this to FALSE. E.g. it makes no sense for a
       * point cloud to have invalid points (these points should not be within the cloud, afterall).
       * @remarks This is the last step of the building process.
       */
      virtual OGPS_Boolean BuildValidityProvider(const bool allowInvalidPoints);

      /*! Gets the object built. This object instance remains managed by the builder, though. */
      virtual VectorBuffer* GetBuffer();

   private:
      /*!
       * Creates the appropriate OpenGPS::PointBuffer instance depending on the type of point data to be handled.
       * @param dataType The type of point data that must be handled by the OpenGPS::PointBuffer instance to be created.
       * @param size The amount of point data to be handled.
       * @param retval Gets TRUE on success, FALSE otherwise.
       * @returns Returns an instance of a buffer object or NULL if the type of point data to be handled equals ::OGPS_MissingPointType.
       */
      PointBuffer* CreatePointBuffer(const OGPS_DataPointType dataType, const OGPS_ULong size, OGPS_Boolean* const retval) const;

      /*! The vector buffer object to be assembled. */
      VectorBuffer* m_Buffer;
   };

}

#endif /* _OPENGPS_VECTOR_BUFFER_BUILDER_HXX */

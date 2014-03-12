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
 * Handle mechanism which makes a C++ OpenGPS::PointVector object accessible
 * through the corresponding C interface of a point vector.
 */

#ifndef _OPENGPS_C_POINT_VECTOR_HXX
#define _OPENGPS_C_POINT_VECTOR_HXX

#ifndef _OPENGPS_CXX_POINT_VECTOR_HXX
#  include <opengps/cxx/point_vector.hxx>
#endif

/*!
 * Encapsulates the internal C++ structure of a point vector handle used within
 * the C interface. This fact is hidden from the public because ::OGPS_PointVectorPtr
 * is defined as an incomplete data type.
 */
typedef struct _OGPS_POINT_VECTOR
{
   /*! Gets/Sets the pointer to the internal C++ object behind the scenes. */
   OpenGPS::PointVector instance;

   /*!
    * Gets/Sets the buffered C interface wrapper for typesafe access to the X member
    * of the internal C++ OpenGPS::PointVector instance.
    */
   OGPS_DataPointPtr x;

   /*!
    * Gets/Sets the buffered C interface wrapper for typesafe access to the Y member
    * of the internal C++ OpenGPS::PointVector instance.
    */
   OGPS_DataPointPtr y;

   /*!
    * Gets/Sets the buffered C interface wrapper for typesafe access to the Z member
    * of the internal C++ OpenGPS::PointVector instance.
    */
   OGPS_DataPointPtr z;
} OGPS_PointVector, *OGPS_PointVectorPtr; /*! Encapsulates the internal C++ structure
 * of a data point handle used within the C interface. */

#endif /* _OPENGPS_C_POINT_VECTOR_HXX */

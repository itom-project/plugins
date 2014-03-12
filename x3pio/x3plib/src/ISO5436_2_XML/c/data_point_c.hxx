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
 * Handle mechanism that makes a C++ OpenGPS::DataPoint object accessible
 * through its corresponding C interface.
 */

#ifndef _OPENGPS_C_DATA_POINT_HXX
#define _OPENGPS_C_DATA_POINT_HXX

namespace OpenGPS
{
   class DataPoint;
}

/*!
 * Encapsulates the C++ structure of a data point handle used internally within
 * the C interface. This fact is hidden from the public because ::OGPS_DataPointPtr
 * is previously introduced as an incomplete data type.
 */
typedef struct _OGPS_DATA_POINT
{
   /*! Gets/Sets the pointer to the internal C++ object acting behind the scenes. */
   OpenGPS::DataPoint* instance;
} OGPS_DataPoint, *OGPS_DataPointPtr; /*! Encapsulates the C++ structure
 * of a data point handle used as a backend of its corresponding C interface. */

#endif /* _OPENGPS_C_DATA_POINT_HXX */

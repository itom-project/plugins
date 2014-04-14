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
 * Common define's and typedef's.
 * This file is an enhanced version of opengps.h for C++ maintance.
 * @see opengps.h
 */

#ifndef _OPENGPS_CXX_OPENGPS_HXX
#define _OPENGPS_CXX_OPENGPS_HXX

#ifndef _OPENGPS_H
# include <opengps/opengps.h>
#endif

/*! @namespace OpenGPS
 * The standard namespace for the openGPS software library.
 */
namespace OpenGPS
{
   /*! @namespace OpenGPS::Schemas
    * Holds the C++ representations of the structure of all
    * supported XML documents.
    */
   namespace Schemas
   {
   }

   /*! Data type of the size of one (possibly signed) byte. */
   typedef char Byte;
   /*! Pointer to a data type of the size of one (possibly signed) byte. */
   typedef Byte* BytePtr;

   /*! Data type of the size of one unsigned byte. */
   typedef unsigned char UnsignedByte;
   /*! Pointer to a data type of the size of one unsigned byte. */
   typedef UnsignedByte* UnsignedBytePtr;

   /*! Character type used to provide more information about an exception. */
   typedef char OGPS_ExceptionChar;

} /* namespace OpenGPS */

#endif    /* _OPENGPS_CXX_OPENGPS_HXX */

/*! @} */

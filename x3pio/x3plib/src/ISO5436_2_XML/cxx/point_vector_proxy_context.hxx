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
 * Indexing of point data managed by OpenGPS::VectorBuffer.
 */

#ifndef _OPENGPS_POINT_VECTOR_PROXY_CONTEXT_HXX
#define _OPENGPS_POINT_VECTOR_PROXY_CONTEXT_HXX

#ifndef _OPENGPS_CXX_OPENGPS_HXX
#  include <opengps/cxx/opengps.hxx>
#endif

namespace OpenGPS
{
   /*!
    * Indexing of point data managed by OpenGPS::VectorBuffer.
    */
   class PointVectorProxyContext
   {
   public:
      /*! Destroys this instance. */
      virtual ~PointVectorProxyContext();

      /*! Gets the current index. */
      virtual OGPS_ULong GetIndex() const;

      /*! Can the current index be incremented? */
      virtual OGPS_Boolean CanIncrementIndex() const;

      /*! Increments the current index by one. */
      virtual OGPS_Boolean IncrementIndex();

      /*! Asks whether this context is for points managed in matrices or lists. */
      virtual OGPS_Boolean IsMatrix() const = 0;

   protected:
      /*! Creates a new instance. */
      PointVectorProxyContext();
   };
}

#endif /* _OPENGPS_POINT_VECTOR_PROXY_CONTEXT_HXX */

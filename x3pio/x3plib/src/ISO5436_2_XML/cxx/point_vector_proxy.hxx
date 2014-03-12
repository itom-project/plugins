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
 * Make the internal memory structure of distinct data point buffers
 * accessable as point vector data.
 */

#ifndef _OPENGPS_POINT_VECTOR_PROXY_HXX
#define _OPENGPS_POINT_VECTOR_PROXY_HXX

#ifndef _OPENGPS_CXX_POINT_VECTOR_BASE_HXX
#  include <opengps/cxx/point_vector_base.hxx>
#endif

#ifndef _OPENGPS_CXX_DATA_POINT_HXX
#  include <opengps/cxx/data_point.hxx>
#endif

namespace OpenGPS
{
   class PointVectorProxyContext;
   class VectorBuffer;
   class PointBuffer;

   /*!
    * Serves one row of the three distinct buffers of data points managed
    * by OpenGPS::VectorBuffer and raises the expression of accessing
    * a single point vector.
    */
   class PointVectorProxy : public PointVectorBase
   {
   public:
      /*!
       * Creates a new instance.
       * @param context Provides information which data row is currently proxied.
       * This is read-only within the proxy object but can be changed from the outside.
       * @param buffer Manages the whole set of all point vectors stacked together.
       */
      PointVectorProxy(
         const PointVectorProxyContext* const context,
         VectorBuffer* const buffer);

      /*! Destroys this instance. */
      virtual ~PointVectorProxy();

      virtual const DataPoint* GetX() const;
      virtual const DataPoint* GetY() const;
      virtual const DataPoint* GetZ() const;

      virtual DataPoint* GetX();
      virtual DataPoint* GetY();
      virtual DataPoint* GetZ();

      virtual void Set(const PointVectorBase& value);
      virtual void Get(PointVectorBase& value) const;

   private:
      /*! Provides information which data row is currently proxied. */
      const PointVectorProxyContext* const m_Context;

      /*! Provides access to the point data of all three axes. */
      VectorBuffer* m_Buffer;

      /*! Provides typesafe access to the point data of the current X component. */
      DataPoint* m_X;

      /*! Provides typesafe access to the point data of the current Y component. */
      DataPoint* m_Y;

      /*! Provides typesafe access to the point data of the current Z component. */
      DataPoint* m_Z;

      /*! The copy-ctor is not implemented. This prevents its usage. */
      PointVectorProxy(const PointVectorProxy& src);
      /*! The assignment-operator is not implemented. This prevents its usage. */
      PointVectorProxy& operator=(const PointVectorProxy& src);

      /*!
       * Proxies one single data point
       * pointed to by the current context information.
       */
      class DataPointProxy : public DataPoint
      {
      public:
         /*!
          * Creates a new instance.
          * @param context Provides the information which row of point data is currently proxied.
          * @param buffer The buffer of all point data stacked together.
          */
         DataPointProxy(
             const PointVectorProxyContext* const context,
             PointBuffer* const buffer);

         /*! Destroys this instance. */
         virtual ~DataPointProxy();

         virtual OGPS_DataPointType GetPointType() const;

         virtual void Get(OGPS_Int16* const value) const;
         virtual void Get(OGPS_Int32* const value) const;
         virtual void Get(OGPS_Float* const value) const;
         virtual void Get(OGPS_Double* const value) const;

         virtual OGPS_Double Get() const;

         virtual OGPS_Boolean IsValid() const;

         virtual void Set(const OGPS_Int16 value);
         virtual void Set(const OGPS_Int32 value);
         virtual void Set(const OGPS_Float value);
         virtual void Set(const OGPS_Double value);

         virtual void Set(const DataPoint& src);

         virtual void Reset();

      private:
         /*! Provides information which data row is currently proxied. */
         const PointVectorProxyContext* const m_Context;

         /*! Provides access to the point data. */
         PointBuffer* m_Buffer;

         /*! The copy-ctor is not implemented. This prevents its usage. */
         DataPointProxy(const DataPointProxy& src);
         /*! The assignment-operator is not implemented. This prevents its usage. */
         DataPointProxy& operator=(const DataPointProxy& src);
      };
   };
}

#endif /* _OPENGPS_POINT_VECTOR_PROXY_HXX */

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

#include "point_vector_proxy.hxx"
#include "point_vector_proxy_context.hxx"

#include "vector_buffer.hxx"
#include "point_buffer.hxx"

#include "stdafx.hxx"

PointVectorProxy::PointVectorProxy(const PointVectorProxyContext* const context, VectorBuffer* const buffer)
: PointVectorBase(), m_Context(context), m_Buffer(buffer)
{
   _ASSERT(context && buffer);

   m_X = new DataPointProxy(context, buffer->GetX());
   m_Y = new DataPointProxy(context, buffer->GetY());
   m_Z = new DataPointProxy(context, buffer->GetZ());
}

PointVectorProxy::~PointVectorProxy()
{
   _OPENGPS_DELETE(m_X);
   _OPENGPS_DELETE(m_Y);
   _OPENGPS_DELETE(m_Z);
}

const DataPoint* PointVectorProxy::GetX() const
{
   _ASSERT(m_X);

   return m_X;
}

const DataPoint* PointVectorProxy::GetY() const
{
   _ASSERT(m_Y);

   return m_Y;
}

const DataPoint* PointVectorProxy::GetZ() const
{
   _ASSERT(m_Z);

   return m_Z;
}

DataPoint* PointVectorProxy::GetX()
{
   _ASSERT(m_X);

   return m_X;
}

DataPoint* PointVectorProxy::GetY()
{
   _ASSERT(m_Y);

   return m_Y;
}

DataPoint* PointVectorProxy::GetZ()
{
   _ASSERT(m_Z);

   return m_Z;
}

void PointVectorProxy::Set(const PointVectorBase& value)
{
   _ASSERT(m_X && m_Y && m_Z);

   if(m_X->IsValid())
   {
      m_X->Set(*value.GetX());
   }

   if(m_Y->IsValid())
   {
      m_Y->Set(*value.GetY());
   }

   m_Z->Set(*value.GetZ());
}

void PointVectorProxy::Get(PointVectorBase& value) const
{
   _ASSERT(m_X && m_Y && m_Z);

   DataPoint *x = value.GetX();
   DataPoint *y = value.GetY();
   DataPoint *z = value.GetZ();

   _ASSERT(x && y && z);

   x->Set(*m_X);
   y->Set(*m_Y);
   z->Set(*m_Z);
}

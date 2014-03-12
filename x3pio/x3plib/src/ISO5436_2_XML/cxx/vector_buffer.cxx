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

#include "vector_buffer.hxx"
#include "point_vector_proxy.hxx"
#include "point_buffer.hxx"

#include "stdafx.hxx"

VectorBuffer::VectorBuffer()
{
   m_X = NULL;
   m_Y = NULL;
   m_Z = NULL;

   m_ValidityProvider = NULL;
   m_ValidBuffer = NULL;
}

VectorBuffer::~VectorBuffer()
{
   _OPENGPS_DELETE(m_X);
   _OPENGPS_DELETE(m_Y);
   _OPENGPS_DELETE(m_Z);

   _OPENGPS_DELETE(m_ValidityProvider);
}

void VectorBuffer::SetX(PointBuffer* const value)
{
   _ASSERT(!m_X);

   m_X = value;
}

void VectorBuffer::SetY(PointBuffer* const value)
{
   _ASSERT(!m_Y);

   m_Y = value;
}

void VectorBuffer::SetZ(PointBuffer* const value)
{
   _ASSERT(!m_Z);

   m_Z = value;
}

void VectorBuffer::SetValidityProvider(PointValidityProvider* const value, ValidBuffer* const buffer)
{
   _ASSERT(!m_ValidityProvider);
   _ASSERT(!m_ValidBuffer);

   _ASSERT(buffer == NULL || buffer == value);

   m_ValidityProvider = value;
   m_ValidBuffer = buffer;
}

PointBuffer* VectorBuffer::GetX()
{
   return m_X;
}

PointBuffer* VectorBuffer::GetY()
{
   return m_Y;
}

PointBuffer* VectorBuffer::GetZ()
{
   return m_Z;
}

PointValidityProvider* VectorBuffer::GetValidityProvider()
{
   return m_ValidityProvider;
}

ValidBuffer* VectorBuffer::GetValidityBuffer()
{
   return m_ValidBuffer;
}

const PointBuffer* VectorBuffer::GetX() const
{
   return m_X;
}

const PointBuffer* VectorBuffer::GetY() const
{
   return m_Y;
}

const PointBuffer* VectorBuffer::GetZ() const
{
   return m_Z;
}

const PointValidityProvider* VectorBuffer::GetValidityProvider() const
{
   return m_ValidityProvider;
}

const ValidBuffer* VectorBuffer::GetValidityBuffer() const
{
   return m_ValidBuffer;
}

OGPS_Boolean VectorBuffer::HasValidityBuffer() const
{
   return m_ValidBuffer != NULL;
}

PointVectorAutoPtr VectorBuffer::GetPointVectorProxy(const PointVectorProxyContext& context)
{
   return PointVectorAutoPtr(new PointVectorProxy(&context, this));
}

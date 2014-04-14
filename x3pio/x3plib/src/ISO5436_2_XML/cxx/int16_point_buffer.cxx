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

#include "int16_point_buffer.hxx"
#include "stdafx.hxx"

Int16PointBuffer::Int16PointBuffer() : PointBuffer()
{
   m_Buffer = NULL;
}

Int16PointBuffer::~Int16PointBuffer()
{
   Free((OpenGPS::UnsignedBytePtr*)(&m_Buffer));
}

void Int16PointBuffer::Allocate(const OGPS_ULong size)
{
   _ASSERT(!m_Buffer);

   m_Buffer = (OGPS_Int16*)PointBuffer::Allocate(size, sizeof(OGPS_Int16));
}

void Int16PointBuffer::Set(const OGPS_ULong index, const OGPS_Int16 value)
{
   _ASSERT(index < GetSize() && m_Buffer);

   m_Buffer[index] = value;
}

void Int16PointBuffer::Get(const OGPS_ULong index, OGPS_Int16& value) const
{
   _ASSERT(index < GetSize() && m_Buffer);

   value = m_Buffer[index];
}

OGPS_DataPointType Int16PointBuffer::GetPointType() const
{
   return OGPS_Int16PointType;
}

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

#include "data_point_impl.hxx"

#include <opengps/cxx/point_vector.hxx>

#include "stdafx.hxx"

PointVectorBase::PointVectorBase()
{
}

PointVectorBase::~PointVectorBase()
{
}

PointVector::PointVector() : PointVectorBase()
{
   m_X = new DataPointImpl();
   m_Y = new DataPointImpl();
   m_Z = new DataPointImpl();
}

PointVector::~PointVector()
{
   _OPENGPS_DELETE(m_X);
   _OPENGPS_DELETE(m_Y);
   _OPENGPS_DELETE(m_Z);
}

const DataPoint* PointVector::GetX() const
{
   _ASSERT(m_X);
   return m_X;
}

const DataPoint* PointVector::GetY() const
{
   _ASSERT(m_Y);
   return m_Y;
}

const DataPoint* PointVector::GetZ() const
{
   _ASSERT(m_Z);
   return m_Z;
}

DataPoint* PointVector::GetX()
{
   _ASSERT(m_X);
   return m_X;
}

DataPoint* PointVector::GetY()
{
   _ASSERT(m_Y);
   return m_Y;
}

DataPoint* PointVector::GetZ()
{
   _ASSERT(m_Z);
   return m_Z;
}

void PointVector::GetX(OGPS_Int16* const value) const
{
   _ASSERT(value && m_X);
   m_X->Get(value);
}

void PointVector::GetX(OGPS_Int32* const value) const
{
   _ASSERT(value && m_X);
   m_X->Get(value);
}

void PointVector::GetX(OGPS_Float* const value) const
{
   _ASSERT(value && m_X);
   m_X->Get(value);
}

void PointVector::GetX(OGPS_Double* const value) const
{
   _ASSERT(value && m_X);
   m_X->Get(value);
}

void PointVector::GetY(OGPS_Int16* const value) const
{
   _ASSERT(value && m_Y);
   m_Y->Get(value);
}

void PointVector::GetY(OGPS_Int32* const value) const
{
   _ASSERT(value && m_Y);
   m_Y->Get(value);
}

void PointVector::GetY(OGPS_Float* const value) const
{
   _ASSERT(value && m_Y);
   m_Y->Get(value);
}

void PointVector::GetY(OGPS_Double* const value) const
{
   _ASSERT(value && m_Y);
   m_Y->Get(value);
}

void PointVector::GetZ(OGPS_Int16* const value) const
{
   _ASSERT(value && m_Z);
   m_Z->Get(value);
}

void PointVector::GetZ(OGPS_Int32* const value) const
{
   _ASSERT(value && m_Z);
   m_Z->Get(value);
}

void PointVector::GetZ(OGPS_Float* const value) const
{
   _ASSERT(value && m_Z);
   m_Z->Get(value);
}

void PointVector::GetZ(OGPS_Double* const value) const
{
   _ASSERT(value && m_Z);
   m_Z->Get(value);
}

void PointVector::SetX(const OGPS_Int16 value)
{
   _ASSERT(m_X);
   m_X->Set(value);
}

void PointVector::SetX(const OGPS_Int32 value)
{
   _ASSERT(m_X);
   m_X->Set(value);
}

void PointVector::SetX(const OGPS_Float value)
{
   _ASSERT(m_X);
   m_X->Set(value);
}

void PointVector::SetX(const OGPS_Double value)
{
   _ASSERT(m_X);
   m_X->Set(value);
}

void PointVector::SetY(const OGPS_Int16 value)
{
   _ASSERT(m_Y);
   m_Y->Set(value);
}

void PointVector::SetY(const OGPS_Int32 value)
{
   _ASSERT(m_Y);
   m_Y->Set(value);
}

void PointVector::SetY(const OGPS_Float value)
{
   _ASSERT(m_Y);
   m_Y->Set(value);
}

void PointVector::SetY(const OGPS_Double value)
{
   _ASSERT(m_Y);
   m_Y->Set(value);
}

void PointVector::SetZ(const OGPS_Int16 value)
{
   _ASSERT(m_Z);
   m_Z->Set(value);
}

void PointVector::SetZ(const OGPS_Int32 value)
{
   _ASSERT(m_Z);
   m_Z->Set(value);
}

void PointVector::SetZ(const OGPS_Float value)
{
   _ASSERT(m_Z);
   m_Z->Set(value);
}

void PointVector::SetZ(const OGPS_Double value)
{
   _ASSERT(m_Z);
   m_Z->Set(value);
}

void PointVector::GetXYZ(
                         OGPS_Double* const x,
                         OGPS_Double* const y,
                         OGPS_Double* const z) const
{
   if(x != NULL)
   {
      _ASSERT(m_X);
      *x = m_X->Get();
   }

   if(y != NULL)
   {
      _ASSERT(m_Y);
      *y = m_Y->Get();
   }

   if(z != NULL)
   {
      _ASSERT(m_Z);
      *z = m_Z->Get();
   }
}

void PointVector::SetXYZ(
                         const OGPS_Double x,
                         const OGPS_Double y,
                         const OGPS_Double z) const
{
   _ASSERT(m_X);
   m_X->Set(x);

   _ASSERT(m_Y);
   m_Y->Set(y);

   _ASSERT(m_Z);
   m_Z->Set(z);
}

OGPS_Boolean PointVector::IsValid() const
{
   _ASSERT(m_X && m_Y && m_Z);
   return m_Z->IsValid();
}

PointVector& PointVector::operator=(const PointVector& src)
{
   _ASSERT(m_X && m_Y && m_Z);
   _ASSERT(src.m_X && src.m_Y && src.m_Z);

   Set(src);

   return *this;
}

void PointVector::Set(const PointVectorBase& value)
{
   _ASSERT(m_X && m_Y && m_Z);

   _ASSERT(value.GetX() && value.GetY() && value.GetZ());

   m_X->Set(*value.GetX());
   m_Y->Set(*value.GetY());
   m_Z->Set(*value.GetZ());
}

void PointVector::Get(PointVectorBase& value) const
{
   _ASSERT(m_X && m_Y && m_Z);

   _ASSERT(value.GetX() && value.GetY() && value.GetZ());

   value.GetX()->Set(*m_X);
   value.GetY()->Set(*m_Y);
   value.GetZ()->Set(*m_Z);
}

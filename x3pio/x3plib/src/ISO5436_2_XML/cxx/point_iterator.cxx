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

#include "iso5436_2_container.hxx"
#include <opengps/cxx/point_vector.hxx>

#include "stdafx.hxx"

PointIterator::PointIterator()
{
}

PointIterator::~PointIterator()
{
}

ISO5436_2Container::PointIteratorImpl::PointIteratorImpl(
                                     ISO5436_2Container * const handle,
                                     const OGPS_Boolean isForward,
                                     const OGPS_Boolean isMatrix) : PointIterator(), m_Handle(handle)
{
   _ASSERT(handle);

   m_IsForward = isForward;
   m_IsMatrix = isMatrix;

   m_U = m_V = m_W = 0;
   m_IsReset = TRUE;
}

ISO5436_2Container::PointIteratorImpl::~PointIteratorImpl()
{
}

OGPS_Boolean ISO5436_2Container::PointIteratorImpl::HasNext() const
{
   _ASSERT(m_Handle && m_Handle->IsMatrix() == m_IsMatrix);

   if(m_IsForward)
   {
      if(m_IsReset)
      {
         _ASSERT(m_U == 0 && m_V == 0 && m_W == 0);

         return m_Handle->GetMaxU() > 0;
      }

      if(m_IsMatrix)
      {
         return ((m_W + 1) * (m_V + 1) * (m_U + 1) < m_Handle->GetMaxU() * m_Handle->GetMaxV() * m_Handle->GetMaxW());
      }
      else
      {
         _ASSERT(m_V == 0 && m_W == 0);

         return ((m_U + 1) < m_Handle->GetMaxU());
      }
   }

   return FALSE;
}

OGPS_Boolean ISO5436_2Container::PointIteratorImpl::HasPrev() const
{
   _ASSERT(m_Handle && m_Handle->IsMatrix() == m_IsMatrix);

   if(!m_IsForward)
   {
      if(m_IsReset)
      {
         _ASSERT(m_U == 0 && m_V == 0 && m_W == 0);

         return m_Handle->GetMaxU() > 0;
      }

      if(m_IsMatrix)
      {
         return (m_W > 0 && m_Handle->GetMaxW() > 0 || m_V > 0 && m_Handle->GetMaxV() > 0 || m_U > 0 && m_Handle->GetMaxU() > 0);
      }
      else
      {
         _ASSERT(m_V == 0 && m_W == 0);

         return m_U > 0 && m_Handle->GetMaxU() > 0;
      }
   }

   return FALSE;
}

OGPS_Boolean ISO5436_2Container::PointIteratorImpl::MoveNext()
{
   _ASSERT(m_Handle && m_Handle->IsMatrix() == m_IsMatrix);

   if(HasNext())
   {
      if(m_IsReset)
      {
         _ASSERT(m_U == 0 && m_V == 0 && m_W == 0);

         m_IsReset = FALSE;

         return TRUE;
      }

      if(m_IsMatrix)
      {
         if(m_U + 1 < m_Handle->GetMaxU())
         {
            ++m_U;

            return TRUE;
         }

         if(m_V + 1 < m_Handle->GetMaxV())
         {
            ++m_V;
            m_U = 0;

            return TRUE;
         }

         if(m_W + 1 < m_Handle->GetMaxW())
         {
            ++m_W;
            m_U = 0;
            m_V = 0;

            return TRUE;
         }
      }
      else
      {
         ++m_U;

         return TRUE;
      }
   }

   return FALSE;
}

OGPS_Boolean ISO5436_2Container::PointIteratorImpl::MovePrev()
{
   _ASSERT(m_Handle && m_Handle->IsMatrix() == m_IsMatrix);

   if(HasPrev())
   {
      if(m_IsMatrix)
      {
         if(m_IsReset)
         {
            _ASSERT(m_W == 0 && m_V == 0 && m_U == 0);
            _ASSERT(m_Handle->GetMaxW() > 0 && m_Handle->GetMaxV() > 0 && m_Handle->GetMaxU() > 0);

            m_W = m_Handle->GetMaxW() - 1;
            m_V = m_Handle->GetMaxV() - 1;
            m_U = m_Handle->GetMaxU() - 1;

            m_IsReset = FALSE;

            return TRUE;
         }

         if(m_W > 0)
         {
            --m_W;

            return TRUE;
         }

         if(m_V > 0)
         {
            _ASSERT(m_Handle->GetMaxW() > 0);

            m_W = m_Handle->GetMaxW() - 1;
            --m_V;

            return TRUE;
         }

         if(m_U > 0)
         {
            _ASSERT(m_Handle->GetMaxW() > 0);
            _ASSERT(m_Handle->GetMaxV() > 0);

            m_W = m_Handle->GetMaxW() - 1;
            m_V = m_Handle->GetMaxV() - 1;
            --m_U;

            return TRUE;
         }
      }
      else
      {
         if(m_U > 0)
         {
            --m_U;

            return TRUE;
         }

         if(m_IsReset)
         {
            _ASSERT(m_U == 0);
            _ASSERT(m_Handle->GetMaxU() > 0);

            m_U = m_Handle->GetMaxU() - 1;
            m_IsReset = FALSE;

            return TRUE;
         }
      }
   }

   return FALSE;
}

void ISO5436_2Container::PointIteratorImpl::ResetNext()
{
   m_U = m_V = m_W = 0;
   m_IsReset = TRUE;
   m_IsForward = TRUE;
}

void ISO5436_2Container::PointIteratorImpl::ResetPrev()
{
   m_U = m_V = m_W = 0;
   m_IsReset = TRUE;
   m_IsForward = FALSE;
}

void ISO5436_2Container::PointIteratorImpl::GetCurrent(PointVector& vector)
{
   _ASSERT(m_Handle && m_Handle->IsMatrix() == m_IsMatrix);

   if(m_IsMatrix)
   {
      m_Handle->GetMatrixPoint(m_U, m_V, m_W, vector);
   }
   else
   {
      m_Handle->GetListPoint(m_U, vector);
   }
}

void ISO5436_2Container::PointIteratorImpl::GetCurrentCoord(PointVector& vector)
{
   _ASSERT(m_Handle && m_Handle->IsMatrix() == m_IsMatrix);

   OGPS_Double x, y, z;

   if(m_IsMatrix)
   {
      m_Handle->GetMatrixCoord(m_U, m_V, m_W, &x, &y, &z);
   }
   else
   {
      m_Handle->GetListCoord(m_U, &x, &y, &z);
   }

   vector.SetXYZ(x, y, z);
}

void ISO5436_2Container::PointIteratorImpl::SetCurrent(const PointVector* const vector)
{
   _ASSERT(m_Handle && m_Handle->IsMatrix() == m_IsMatrix);

   if(m_IsMatrix)
   {
      m_Handle->SetMatrixPoint(m_U, m_V, m_W, vector);
   }
   else
   {
      // NULL vector (invalid point) makes no sense in list type
      _ASSERT(vector);

      m_Handle->SetListPoint(m_U, *vector);
   }
}

OGPS_Boolean ISO5436_2Container::PointIteratorImpl::GetPosition(OGPS_ULong * const index) const
{
   _ASSERT(index);

   if(!m_IsMatrix)
   {
      *index = m_U;
      return TRUE;
   }

   return FALSE;
}

OGPS_Boolean ISO5436_2Container::PointIteratorImpl::GetPosition(
   OGPS_ULong * const u,
   OGPS_ULong * const v,
   OGPS_ULong * const w) const
{
   if(m_IsMatrix)
   {
      if(u)
      {
         *u = m_U;
      }

      if(v)
      {
         *v = m_V;
      }

      if(w)
      {
         *w = m_W;
      }

      return TRUE;
   }

   return FALSE;
}

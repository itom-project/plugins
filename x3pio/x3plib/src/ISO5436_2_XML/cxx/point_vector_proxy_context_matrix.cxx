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

#include "point_vector_proxy_context_matrix.hxx"
#include <opengps/cxx/exceptions.hxx>
#include "stdafx.hxx"

PointVectorProxyContextMatrix::PointVectorProxyContextMatrix(
         const OGPS_ULong maxU,
         const OGPS_ULong maxV,
         const OGPS_ULong maxW)
         : PointVectorProxyContext()
{
   m_MaxU = maxU;
   m_MaxV = maxV;
   m_MaxW = maxW;

   m_U = 0;
   m_V = 0;
   m_W = 0;
}

PointVectorProxyContextMatrix::~PointVectorProxyContextMatrix()
{
}

void PointVectorProxyContextMatrix::SetIndex(
         const OGPS_ULong u,
         const OGPS_ULong v,
         const OGPS_ULong w)
{
   if(u < m_MaxU && v < m_MaxV && w < m_MaxW)
   {
      m_U = u;
      m_V = v;
      m_W = w;
   }
   else
   {
      throw OpenGPS::Exception(
         OGPS_ExInvalidOperation,
         _EX_T("Index out of range."),
         _EX_T("The data point addressed lies outside the scope of the current matrix topology."),
         _EX_T("OpenGPS::PointVectorProxyContextMatrix::SetIndex"));
   }
}

OGPS_ULong PointVectorProxyContextMatrix::GetIndex() const
{
   return m_V * m_MaxU * m_MaxW + m_U * m_MaxW + m_W;
}

OGPS_Boolean PointVectorProxyContextMatrix::CanIncrementIndex() const
{
   return ((m_W + 1) * (m_V + 1) * (m_U + 1) < m_MaxU * m_MaxV * m_MaxW);
}

OGPS_Boolean PointVectorProxyContextMatrix::IncrementIndex()
{
   if(CanIncrementIndex())
   {
      if(m_U + 1 < m_MaxU)
      {
         ++m_U;

         return TRUE;
      }

      if(m_V + 1 < m_MaxV)
      {
         ++m_V;
         m_U = 0;

         return TRUE;
      }

      if(m_W + 1 < m_MaxW)
      {
         ++m_W;
         m_U = 0;
         m_V = 0;

         return TRUE;
      }
   }

   return FALSE;
}

OGPS_Boolean PointVectorProxyContextMatrix::IsMatrix() const
{
   return TRUE;
}

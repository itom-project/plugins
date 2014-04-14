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

#include <limits>

#include "inline_validity.hxx"
#include "point_buffer.hxx"
#include <opengps/cxx/exceptions.hxx>
#include "stdafx.hxx"

FloatInlineValidity::FloatInlineValidity(PointBuffer* const value, const bool allowInvalidPoints)
: PointValidityProvider(value), m_AllowInvalidPoints(allowInvalidPoints)
{
   _ASSERT(value && value->GetPointType() == OGPS_FloatPointType);
}

FloatInlineValidity::~FloatInlineValidity()
{
}

void FloatInlineValidity::SetValid(const unsigned int index, const OGPS_Boolean value)
{
   _ASSERT(std::numeric_limits<OGPS_Float>::has_quiet_NaN);

   if(!value)
   {
      if(!m_AllowInvalidPoints)
      {
         throw OpenGPS::Exception(
            OGPS_ExInvalidOperation,
            _EX_T("This point data set may not contain invalid points."),
            _EX_T("An attempt was made to set an invalid point, although this has been explicitly forbidden. E.g. point clouds of record1 feature type PCL are not allowed to contain invalid points."),
            _EX_T("OpenGPS::FloatInlineValidity::SetValid"));
      }

      GetPointBuffer()->Set(index, std::numeric_limits<OGPS_Float>::quiet_NaN());
   }
}

OGPS_Boolean FloatInlineValidity::IsValid(const unsigned int index) const
{
   _ASSERT(std::numeric_limits<OGPS_Float>::has_quiet_NaN);

   OGPS_Float value;
   GetPointBuffer()->Get(index, value);
   // Comparing a NaN to itself is allways false
   const bool isValid = (value == value);
   if(!isValid && !m_AllowInvalidPoints)
   {
      throw OpenGPS::Exception(
         OGPS_ExWarning,
         _EX_T("This point data set may not contain invalid points."),
         _EX_T("Although this has been explicitly forbidden the current point data contains invalid points. This may be ignored, but e.g. for point clouds of record1 feature type PCL it makes no sense to contain invalid points. These points should not be within the cloud, afterall."),
         _EX_T("OpenGPS::FloatInlineValidity::IsValid"));
   }
   return isValid;
}

DoubleInlineValidity::DoubleInlineValidity(PointBuffer* const value, const bool allowInvalidPoints)
: PointValidityProvider(value), m_AllowInvalidPoints(allowInvalidPoints)
{
   _ASSERT(value && value->GetPointType() == OGPS_DoublePointType);
}

DoubleInlineValidity::~DoubleInlineValidity()
{
}

void DoubleInlineValidity::SetValid(const unsigned int index, const OGPS_Boolean value)
{
   _ASSERT(std::numeric_limits<OGPS_Double>::has_quiet_NaN);

   if(!value)
   {
      if(!m_AllowInvalidPoints)
      {
         throw OpenGPS::Exception(
            OGPS_ExInvalidOperation,
            _EX_T("This point data set may not contain invalid points."),
            _EX_T("An attempt was made to set an invalid point, although this has been explicitly forbidden. E.g. point clouds of record1 feature type PCL are not allowed to contain invalid points."),
            _EX_T("OpenGPS::DoubleInlineValidity::SetValid"));
      }

      GetPointBuffer()->Set(index, std::numeric_limits<OGPS_Double>::quiet_NaN());
   }
}

OGPS_Boolean DoubleInlineValidity::IsValid(const unsigned int index) const
{
   _ASSERT(std::numeric_limits<OGPS_Double>::has_quiet_NaN);

   OGPS_Double value;
   GetPointBuffer()->Get(index, value);
   // Comparing a NaN to itself is allways false
   const bool isValid = (value == value);
   if(!isValid && !m_AllowInvalidPoints)
   {
      throw OpenGPS::Exception(
         OGPS_ExWarning,
         _EX_T("This point data set may not contain invalid points."),
         _EX_T("Although this has been explicitly forbidden the current point data contains invalid points. This may be ignored, but e.g. for point clouds of record1 feature type PCL it makes no sense to contain invalid points. These points should not be within the cloud, afterall."),
         _EX_T("OpenGPS::DoubleInlineValidity::IsValid"));
   }
   return isValid;
}

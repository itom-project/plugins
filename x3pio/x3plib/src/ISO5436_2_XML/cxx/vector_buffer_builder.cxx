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

#include "vector_buffer_builder.hxx"
#include "vector_buffer.hxx"

#include "inline_validity.hxx"

#include "int16_point_buffer.hxx"
#include "int32_point_buffer.hxx"
#include "float_point_buffer.hxx"
#include "double_point_buffer.hxx"

#include <opengps/cxx/exceptions.hxx>

#include "stdafx.hxx"

VectorBufferBuilder::VectorBufferBuilder()
{
   m_Buffer = NULL;
}

VectorBufferBuilder::~VectorBufferBuilder()
{
   _OPENGPS_DELETE(m_Buffer);
}

OGPS_Boolean VectorBufferBuilder::BuildBuffer()
{
   _ASSERT(!m_Buffer);

   m_Buffer = new VectorBuffer();

   return TRUE;
}

OGPS_Boolean VectorBufferBuilder::BuildX(const OGPS_DataPointType dataType, const OGPS_ULong size)
{
   _ASSERT(m_Buffer);

   OGPS_Boolean success = FALSE;
   m_Buffer->SetX(CreatePointBuffer(dataType, size, &success));
   return success;
}

OGPS_Boolean VectorBufferBuilder::BuildY(const OGPS_DataPointType dataType, const OGPS_ULong size)
{
   _ASSERT(m_Buffer);

   OGPS_Boolean success = FALSE;
   m_Buffer->SetY(CreatePointBuffer(dataType, size, &success));
   return success;
}

OGPS_Boolean VectorBufferBuilder::BuildZ(const OGPS_DataPointType dataType, const OGPS_ULong size)
{
   _ASSERT(m_Buffer);
   _ASSERT(dataType != OGPS_MissingPointType);

   OGPS_Boolean success = FALSE;
   m_Buffer->SetZ(CreatePointBuffer(dataType, size, &success));
   return success;
}

OGPS_Boolean VectorBufferBuilder::BuildValidityProvider(const bool allowInvalidPoints)
{
   _ASSERT(m_Buffer);
   _ASSERT(m_Buffer->GetZ());

   PointValidityProvider* provider = NULL;
   ValidBuffer* validBuffer = NULL;

   PointBuffer * const zBuffer = m_Buffer->GetZ();
   const OGPS_DataPointType dataType = zBuffer->GetPointType();

   switch(dataType)
   {
   case OGPS_Int16PointType:
      {
         Int16ValidBuffer* validityInt16 = new Int16ValidBuffer(zBuffer);
         provider = validityInt16;
         if(allowInvalidPoints)
         {
            validBuffer = validityInt16;
         }
      } break;
   case OGPS_Int32PointType:
      {
         Int32ValidBuffer* validityInt32 = new Int32ValidBuffer(zBuffer);
         provider = validityInt32;
         if(allowInvalidPoints)
         {
            validBuffer = validityInt32;
         }
      } break;
   case OGPS_FloatPointType:
      {
         provider = new FloatInlineValidity(zBuffer, allowInvalidPoints);
      } break;
   case OGPS_DoublePointType:
      {
         provider = new DoubleInlineValidity(zBuffer, allowInvalidPoints);
      } break;
   case OGPS_MissingPointType:
      {
         _ASSERT(FALSE);
      } break;
   default:
      {
         _ASSERT(FALSE);
      } break;
   }

   if(provider)
   {
      m_Buffer->SetValidityProvider(provider, validBuffer);
      return TRUE;
   }

   return FALSE;
}

VectorBuffer* VectorBufferBuilder::GetBuffer()
{
   return m_Buffer;
}

PointBuffer* VectorBufferBuilder::CreatePointBuffer(const OGPS_DataPointType dataType, const OGPS_ULong size, OGPS_Boolean* const retval) const
{
   _ASSERT(retval);

   PointBuffer* point = NULL;

   switch(dataType)
   {
   case OGPS_Int16PointType:
      point = new Int16PointBuffer();
      *retval = TRUE;
      break;
   case OGPS_Int32PointType:
      point = new Int32PointBuffer();
      *retval = TRUE;
      break;
   case OGPS_FloatPointType:
      point = new FloatPointBuffer();
      *retval = TRUE;
      break;
   case OGPS_DoublePointType:
      point = new DoublePointBuffer();
      *retval = TRUE;
      break;
   case OGPS_MissingPointType:
      *retval = TRUE;
      break;
   default:
      *retval = FALSE;
      _ASSERT(FALSE);
      break;
   }

   if(point)
   {
      try
      {
         point->Allocate(size);
      }
      catch(const OpenGPS::Exception& ex)
      {
         _OPENGPS_DELETE(point);

         throw OpenGPS::Exception(ex);
      }
   }

   return point;
}

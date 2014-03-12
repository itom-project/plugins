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

#include <stdlib.h>
#include <string.h>

#include "valid_buffer.hxx"
#include "point_buffer.hxx"
#include "stdafx.hxx"

#include <opengps/cxx/exceptions.hxx>

ValidBuffer::ValidBuffer(PointBuffer* const value)
   : PointValidityProvider(value)
{
   m_ValidityBuffer = NULL;
   m_RawSize = 0;
}

ValidBuffer::ValidBuffer() : PointValidityProvider(NULL)
{
   _ASSERT(FALSE);
   m_ValidityBuffer = NULL;
   m_RawSize = 0;
}

ValidBuffer::~ValidBuffer()
{
   Reset();
}

void ValidBuffer::Allocate()
{
   _ASSERT(!m_ValidityBuffer && m_RawSize == 0 && GetPointBuffer());

   const unsigned int size = GetPointBuffer()->GetSize();

   if(size > 0)
   {
      unsigned int rawSize = size / 8;

      if(rawSize * 8 < size)
      {
         ++rawSize;
      }

      AllocateRaw(rawSize);
   }
}

void ValidBuffer::AllocateRaw(const unsigned int rawSize)
{
   _ASSERT(!m_ValidityBuffer);

   m_ValidityBuffer = (OpenGPS::UnsignedBytePtr)malloc(rawSize);

   if(!m_ValidityBuffer)
   {
      throw OpenGPS::Exception(
         OGPS_ExGeneral,
         _EX_T("Attempt to allocate memory failed."),
         _EX_T("Could not allocate memory using malloc. This usually means there has not been left enough virtual memory on your system to fulfill the request. You may close some applications currently running and then try again or upgrade your computer hardware."),
         _EX_T("ValidBuffer::AllocateRaw"));
   }

   memset(m_ValidityBuffer, 255, rawSize);
   m_RawSize = rawSize;
}

void ValidBuffer::Reset()
{
   _OPENGPS_FREE(m_ValidityBuffer);

   m_RawSize = 0;
}

OGPS_Boolean ValidBuffer::IsAllocated() const
{
   return (m_ValidityBuffer != NULL);
}

void ValidBuffer::SetValid(const unsigned int index, const OGPS_Boolean value)
{
   _ASSERT(index < GetPointBuffer()->GetSize());

   /*
    * Do not parse if the specific value equals TRUE and no buffer has been allocated yet,
    * since then everything is assumed to be valid by default.
    * In other words: the validity does not need to be explicitly tracked.
    */
   if(!value || m_ValidityBuffer)
   {
      if(!m_ValidityBuffer)
      {
         Allocate();
      }

      unsigned int bytePosition = index / 8;
      unsigned int bitPosition = index % 8;

      OpenGPS::UnsignedByte bitValue = (OpenGPS::UnsignedByte)(((OpenGPS::UnsignedByte)1) << bitPosition);

      OpenGPS::UnsignedBytePtr rawByte = &m_ValidityBuffer[bytePosition];

      if(value)
      {
         *rawByte |= bitValue;
      }
      else
      {
         *rawByte &= ~bitValue;
      }
   }
}

OGPS_Boolean ValidBuffer::IsValid(const unsigned int index) const
{
   if(!m_ValidityBuffer)
   {
      return TRUE;
   }

   _ASSERT(index < GetPointBuffer()->GetSize());

   unsigned int bytePosition = index / 8;
   unsigned int bitPosition = index % 8;

   OpenGPS::UnsignedByte bitValue = (OpenGPS::UnsignedByte)(((OpenGPS::UnsignedByte)1) << bitPosition);

   OpenGPS::UnsignedBytePtr rawByte = &m_ValidityBuffer[bytePosition];

   return ((*rawByte & bitValue) != 0);
}

void ValidBuffer::Read(std::basic_istream<OpenGPS::Byte>& stream)
{
   OGPS_Boolean success = FALSE;

   // get length of file:
   stream.seekg (0, std::ios::end);
   if(!stream.fail())
   {
      const unsigned int length = stream.tellg();
      if(!stream.fail())
      {
         stream.seekg (0, std::ios::beg);
         if(!stream.fail())
         {
            AllocateRaw(length);

            // read data as a block
            stream.read ((OpenGPS::BytePtr)m_ValidityBuffer,length);
            if(!stream.fail())
            {
               success = TRUE;
            }
         }
      }
   }

   if(!success)
   {
      Reset();

      throw OpenGPS::Exception(
         OGPS_ExGeneral,
         _EX_T("Could not read binary point validity file contained in the X3P archive."),
         _EX_T("Verify that the X3P file is not corrupted using the zip utility of your choice and try again."),
         _EX_T("OpenGPS::ValidBuffer::Read"));
   }
}

void ValidBuffer::Write(std::ostream& stream)
{
   _ASSERT(m_ValidityBuffer);

   stream.write((const char*)m_ValidityBuffer, m_RawSize);

   if(stream.fail())
   {
      throw OpenGPS::Exception(
         OGPS_ExGeneral,
         _EX_T("Failed to write to the point validity stream."),
         _EX_T("Check for filesystem permissions and enough space."),
         _EX_T("OpenGPS::ValidBuffer::Write"));
   }
}

OGPS_Boolean ValidBuffer::HasInvalidMarks() const
{
   if(m_ValidityBuffer)
   {
      const OGPS_ULong max_index = GetPointBuffer()->GetSize();
      for(OGPS_ULong index = 0; index < max_index; ++index)
      {
         if(!IsValid(index))
         {
            return TRUE;
         }
      }
   }
   return FALSE;
}

Int16ValidBuffer::Int16ValidBuffer(PointBuffer* const value)
: ValidBuffer(value)
{
   _ASSERT(value && value->GetPointType() == OGPS_Int16PointType);
}


Int16ValidBuffer::~Int16ValidBuffer()
{
}

void Int16ValidBuffer::SetValid(const unsigned int index, const OGPS_Boolean value)
{
   ValidBuffer::SetValid(index, value);

   if(!value)
   {
      const OGPS_Int16 invalid = 0;
      GetPointBuffer()->Set(index, invalid);
   }
}

Int32ValidBuffer::Int32ValidBuffer(PointBuffer* const value)
: ValidBuffer(value)
{
   _ASSERT(value && value->GetPointType() == OGPS_Int32PointType);
}

Int32ValidBuffer::~Int32ValidBuffer()
{
}

void Int32ValidBuffer::SetValid(const unsigned int index, const OGPS_Boolean value)
{
   ValidBuffer::SetValid(index, value);

   if(!value)
   {
      const OGPS_Int32 invalid = 0;
      GetPointBuffer()->Set(index, invalid);
   }
}

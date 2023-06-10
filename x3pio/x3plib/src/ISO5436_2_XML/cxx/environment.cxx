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

#include "environment.hxx"
#include "stdafx.hxx"

#define _OGPS_SHORT_SIZE 2
#define _OGPS_INT_SIZE 4
#define _OGPS_FLOAT_SIZE 4
#define _OGPS_DOUBLE_SIZE 8

Environment* Environment::m_Instance = NULL;

const Environment* const Environment::GetInstance()
{
   if(!m_Instance)
   {
      m_Instance = CreateInstance();
   }

   return m_Instance;
}

void Environment::Reset()
{
   _OPENGPS_DELETE(m_Instance);
}

Environment::Environment()
{
}

Environment::~Environment()
{
}

OGPS_Boolean Environment::IsLittleEndian() const
{
   OpenGPS::UnsignedByte test[2];
   test[0] = 0;
   test[1] = 1;

   short value;
   ByteSwap16(&test[0], &value);

   _ASSERT(value == 1 || value == 256);

   return (value == 1);
}

OpenGPS::UnsignedBytePtr Environment::ByteSwap(const short* const value, OpenGPS::UnsignedBytePtr dst) const
{
   // short data type should be 16bit long on any 32bit or 64bit hardware (- my guess).
   return ByteSwap16(value, dst);
}

void Environment::ByteSwap(const OpenGPS::UnsignedBytePtr src, short* const value) const
{
   // short data type should be 16bit long on any 32bit or 64bit hardware (- my guess).
   ByteSwap16(src, value);
}

OpenGPS::UnsignedBytePtr Environment::ByteSwap16(const short* const value, OpenGPS::UnsignedBytePtr dst) const
{
   _ASSERT(value && dst && sizeof(*value) == _OGPS_SHORT_SIZE);

   const OpenGPS::UnsignedBytePtr src = (const OpenGPS::UnsignedBytePtr)value;

   dst[0] = src[1];
   dst[1] = src[0];

#  if _OGPS_SHORT_SIZE != 2
#     pragma error
#  endif

   return dst;
}

void Environment::ByteSwap16(const OpenGPS::UnsignedBytePtr src, short* const value) const
{
   _ASSERT(value && src && sizeof(*value) == _OGPS_SHORT_SIZE);

   const OpenGPS::UnsignedBytePtr dst = (const OpenGPS::UnsignedBytePtr)value;

   dst[0] = src[1];
   dst[1] = src[0];

#  if _OGPS_SHORT_SIZE != 2
#     pragma error
#  endif
}

OpenGPS::UnsignedBytePtr Environment::ByteSwap(const int* const value, OpenGPS::UnsignedBytePtr dst) const
{
   _ASSERT(value && dst && sizeof(*value) == _OGPS_INT_SIZE);

   const OpenGPS::UnsignedBytePtr src = (const OpenGPS::UnsignedBytePtr)value;

#if _OGPS_INT_SIZE == 2
   dst[0] = src[1];
   dst[1] = src[0];
#endif

#if _OGPS_INT_SIZE == 4
   dst[0] = src[3];
   dst[1] = src[2];
   dst[2] = src[1];
   dst[3] = src[0];
#endif

#if _OGPS_INT_SIZE == 8
   dst[0] = src[7];
   dst[1] = src[6];
   dst[2] = src[5];
   dst[3] = src[4];
   dst[4] = src[3];
   dst[5] = src[2];
   dst[6] = src[1];
   dst[7] = src[0];
#endif

#  if _OGPS_INT_SIZE != 2 && _OGPS_INT_SIZE != 4 && _OGPS_INT_SIZE != 8
#     pragma error
#  endif

   return dst;
}

void Environment::ByteSwap(const OpenGPS::UnsignedBytePtr src, int* const value) const
{
   _ASSERT(src && value && sizeof(*value) == _OGPS_INT_SIZE);

   OpenGPS::UnsignedBytePtr dst = (OpenGPS::UnsignedBytePtr)value;

#if _OGPS_INT_SIZE == 2
   dst[0] = src[1];
   dst[1] = src[0];
#endif

#if _OGPS_INT_SIZE == 4
   dst[0] = src[3];
   dst[1] = src[2];
   dst[2] = src[1];
   dst[3] = src[0];
#endif

#if _OGPS_INT_SIZE == 8
   dst[0] = src[7];
   dst[1] = src[6];
   dst[2] = src[5];
   dst[3] = src[4];
   dst[4] = src[3];
   dst[5] = src[2];
   dst[6] = src[1];
   dst[7] = src[0];
#endif

#  if _OGPS_INT_SIZE != 2 && _OGPS_INT_SIZE != 4 && _OGPS_INT_SIZE != 8
#     pragma error
#  endif
}

OpenGPS::UnsignedBytePtr Environment::ByteSwap32(const int* const value, OpenGPS::UnsignedBytePtr dst) const
{
   _ASSERT(value && dst && sizeof(*value) == _OGPS_INT_SIZE);

   const OpenGPS::UnsignedBytePtr src = (const OpenGPS::UnsignedBytePtr)value;

#if _OGPS_INT_SIZE == 2
   dst[0] = src[3];
   dst[1] = src[2];
   dst[2] = 0;
   dst[3] = 0;
#endif

#if _OGPS_INT_SIZE == 4
   dst[0] = src[3];
   dst[1] = src[2];
   dst[2] = src[1];
   dst[3] = src[0];
#endif

#if _OGPS_INT_SIZE == 8
   dst[0] = src[7];
   dst[1] = src[6];
   dst[2] = src[5];
   dst[3] = src[4];
#endif

#  if _OGPS_INT_SIZE != 2 && _OGPS_INT_SIZE != 4 && _OGPS_INT_SIZE != 8
#     pragma error
#  endif

   return dst;
}

void Environment::ByteSwap32(const OpenGPS::UnsignedBytePtr src, int* const value) const
{
   _ASSERT(src && value && sizeof(*value) == _OGPS_INT_SIZE);

   OpenGPS::UnsignedBytePtr dst = (OpenGPS::UnsignedBytePtr)value;

#if _OGPS_INT_SIZE == 2
   dst[0] = src[3];
   dst[1] = src[2];
#endif

#if _OGPS_INT_SIZE == 4
   dst[0] = src[3];
   dst[1] = src[2];
   dst[2] = src[1];
   dst[3] = src[0];
#endif

#if _OGPS_INT_SIZE == 8
   dst[0] = 0;
   dst[1] = 0;
   dst[2] = 0;
   dst[3] = 0;
   dst[4] = src[3];
   dst[5] = src[2];
   dst[6] = src[1];
   dst[7] = src[0];
#endif

#  if _OGPS_INT_SIZE != 2 && _OGPS_INT_SIZE != 4 && _OGPS_INT_SIZE != 8
#     pragma error
#  endif
}

OpenGPS::UnsignedBytePtr Environment::ByteSwap(const float* const value, OpenGPS::UnsignedBytePtr dst) const
{
   // float type should be 32bit long on any 32bit or 64bit hardware (- my guess).
   return ByteSwap32(value, dst);
}

void Environment::ByteSwap(const OpenGPS::UnsignedBytePtr src, float* const value) const
{
   // float type should be 32bit long on any 32bit or 64bit hardware (- my guess).
   ByteSwap32(src, value);
}

OpenGPS::UnsignedBytePtr Environment::ByteSwap32(const float* const value, OpenGPS::UnsignedBytePtr dst) const
{
   _ASSERT(value && dst && sizeof(*value) == _OGPS_FLOAT_SIZE);

   const OpenGPS::UnsignedBytePtr src = (const OpenGPS::UnsignedBytePtr)value;

   dst[0] = src[3];
   dst[1] = src[2];
   dst[2] = src[1];
   dst[3] = src[0];

#  if _OGPS_FLOAT_SIZE != 4
#     pragma error
#  endif

   return dst;
}

void Environment::ByteSwap32(const OpenGPS::UnsignedBytePtr src, float* const value) const
{
   _ASSERT(src && value && sizeof(*value) == _OGPS_FLOAT_SIZE);

   OpenGPS::UnsignedBytePtr dst = (OpenGPS::UnsignedBytePtr)value;

   dst[0] = src[3];
   dst[1] = src[2];
   dst[2] = src[1];
   dst[3] = src[0];

#  if _OGPS_FLOAT_SIZE != 4
#     pragma error
#  endif
}

OpenGPS::UnsignedBytePtr Environment::ByteSwap(const double* const value, OpenGPS::UnsignedBytePtr dst) const
{
   // double type should be 64bit long on any 32bit or 64bit hardware (- my guess).
   return ByteSwap64(value, dst);
}

void Environment::ByteSwap(const OpenGPS::UnsignedBytePtr src, double* const value) const
{
   // double type should be 64bit long on any 32bit or 64bit hardware (- my guess).
   ByteSwap64(src, value);
}

OpenGPS::UnsignedBytePtr Environment::ByteSwap64(const double* const value, OpenGPS::UnsignedBytePtr dst) const
{
   _ASSERT(value && dst && sizeof(*value) == _OGPS_DOUBLE_SIZE);

   const OpenGPS::UnsignedBytePtr src = (const OpenGPS::UnsignedBytePtr)value;

   dst[0] = src[7];
   dst[1] = src[6];
   dst[2] = src[5];
   dst[3] = src[4];
   dst[4] = src[3];
   dst[5] = src[2];
   dst[6] = src[1];
   dst[7] = src[0];

#  if _OGPS_DOUBLE_SIZE != 8
#     pragma error
#  endif

   return dst;
}

void Environment::ByteSwap64(const OpenGPS::UnsignedBytePtr src, double* const value) const
{
   _ASSERT(src && value && sizeof(*value) == _OGPS_DOUBLE_SIZE);

   OpenGPS::UnsignedBytePtr dst = (OpenGPS::UnsignedBytePtr)value;

   dst[0] = src[7];
   dst[1] = src[6];
   dst[2] = src[5];
   dst[3] = src[4];
   dst[4] = src[3];
   dst[5] = src[2];
   dst[6] = src[1];
   dst[7] = src[0];

#  if _OGPS_DOUBLE_SIZE != 8
#     pragma error
#  endif
}

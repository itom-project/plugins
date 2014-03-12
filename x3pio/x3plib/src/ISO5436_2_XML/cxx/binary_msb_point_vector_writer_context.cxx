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

#include "binary_msb_point_vector_writer_context.hxx"
#include "point_vector_iostream.hxx"

#include <opengps/cxx/exceptions.hxx>

#include "environment.hxx"

#include "stdafx.hxx"

/*! Checks whether the underlying stream is valid. Throws an exception if this is not the case. */
#define _CHECK_STREAM_AND_THROW_EXCEPTION \
   if(!HasStream()) \
   { \
   throw OpenGPS::Exception( \
   OGPS_ExInvalidOperation, \
   _EX_T("No binary file stream available."), \
   _EX_T("The operation on the binary file stream failed, because the stream has been closed already."), \
   _EX_T("OpenGPS::BinaryMSBPointVectorWriterContext")); \
   }

/*! Checks whether the underlying stream is valid. Throws an exception if this is not the case. */
#define _CHECK_ISGOOD_AND_THROW_EXCEPTION \
   if(!IsGood()) \
   { \
   throw OpenGPS::Exception( \
   OGPS_ExInvalidOperation, \
   _EX_T("The underlying binary stream object became invalid."), \
   _EX_T("A read/write error occured."), \
   _EX_T("OpenGPS::BinaryMSBPointVectorWriterContext")); \
   }

BinaryMSBPointVectorWriterContext::BinaryMSBPointVectorWriterContext(zipFile handle)
: BinaryPointVectorWriterContext(handle)
{
}

BinaryMSBPointVectorWriterContext::~BinaryMSBPointVectorWriterContext()
{
   Environment::Reset();
}

void BinaryMSBPointVectorWriterContext::Write(const OGPS_Int16* const value)
{
   _ASSERT(value);

   _CHECK_STREAM_AND_THROW_EXCEPTION;

   _ASSERT(sizeof(*value) >= _OPENGPS_BINFORMAT_INT16_SIZE);
   _ASSERT(_OPENGPS_BINFORMAT_INT16_SIZE == 2);

   OpenGPS::UnsignedByte buffer[_OPENGPS_BINFORMAT_INT16_SIZE];
   GetStream()->write((const char*)Environment::GetInstance()->ByteSwap16(value, &buffer[0]), _OPENGPS_BINFORMAT_INT16_SIZE);

   _CHECK_ISGOOD_AND_THROW_EXCEPTION;
}

void BinaryMSBPointVectorWriterContext::Write(const OGPS_Int32* const value)
{
   _ASSERT(value);

   _CHECK_STREAM_AND_THROW_EXCEPTION;

   _ASSERT(sizeof(*value) >= _OPENGPS_BINFORMAT_INT32_SIZE);
   _ASSERT(_OPENGPS_BINFORMAT_INT32_SIZE == 4);

   OpenGPS::UnsignedByte buffer[_OPENGPS_BINFORMAT_INT32_SIZE];
   GetStream()->write((const char*)Environment::GetInstance()->ByteSwap32(value, &buffer[0]), _OPENGPS_BINFORMAT_INT32_SIZE);

   _CHECK_ISGOOD_AND_THROW_EXCEPTION;
}

void BinaryMSBPointVectorWriterContext::Write(const OGPS_Float* const value)
{
   _ASSERT(value);

   _CHECK_STREAM_AND_THROW_EXCEPTION;

   _ASSERT(sizeof(*value) >= _OPENGPS_BINFORMAT_FLOAT_SIZE);
   _ASSERT(_OPENGPS_BINFORMAT_FLOAT_SIZE == 4);

   OpenGPS::UnsignedByte buffer[_OPENGPS_BINFORMAT_FLOAT_SIZE];
   GetStream()->write((const char*)Environment::GetInstance()->ByteSwap32(value, &buffer[0]), _OPENGPS_BINFORMAT_FLOAT_SIZE);

   _CHECK_ISGOOD_AND_THROW_EXCEPTION;
}

void BinaryMSBPointVectorWriterContext::Write(const OGPS_Double* const value)
{
   _ASSERT(value);

   _CHECK_STREAM_AND_THROW_EXCEPTION;

   _ASSERT(sizeof(*value) >= _OPENGPS_BINFORMAT_DOUBLE_SIZE);
   _ASSERT(_OPENGPS_BINFORMAT_DOUBLE_SIZE == 8);

   OpenGPS::UnsignedByte buffer[_OPENGPS_BINFORMAT_DOUBLE_SIZE];
   GetStream()->write((const char*)Environment::GetInstance()->ByteSwap64(value, &buffer[0]), _OPENGPS_BINFORMAT_DOUBLE_SIZE);

   _CHECK_ISGOOD_AND_THROW_EXCEPTION;
}

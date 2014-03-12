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

#include "binary_point_vector_writer_context.hxx"
#include <opengps/cxx/exceptions.hxx>
#include "stdafx.hxx"

/*! Checks whether the underlying stream is valid. Throws an exception if this is not the case. */
#define _CHECK_STREAM_AND_THROW_EXCEPTION \
   if(!HasStream()) \
   { \
   throw OpenGPS::Exception( \
      OGPS_ExInvalidOperation, \
      _EX_T("No binary file stream available."), \
      _EX_T("The operation on the binary file stream failed, because the stream has been closed already."), \
      _EX_T("OpenGPS::BinaryPointVectorWriterContext")); \
   }

/*! Checks whether the underlying stream is valid. Throws an exception if this is not the case. */
#define _CHECK_ISGOOD_AND_THROW_EXCEPTION \
   if(!IsGood()) \
   { \
   throw OpenGPS::Exception( \
      OGPS_ExInvalidOperation, \
      _EX_T("The underlying binary stream object became invalid."), \
      _EX_T("A read/write error occured."), \
      _EX_T("OpenGPS::BinaryPointVectorWriterContext")); \
   }

BinaryPointVectorWriterContext::BinaryPointVectorWriterContext(zipFile handle)
: PointVectorWriterContext()
{
   m_Buffer = new ZipStreamBuffer(handle, TRUE);
   m_Stream = new ZipOutputStream(*m_Buffer);
}

BinaryPointVectorWriterContext::~BinaryPointVectorWriterContext()
{
   Close();
}

std::ostream* BinaryPointVectorWriterContext::GetStream()
{
   _ASSERT(m_Stream && m_Buffer);
   return m_Stream;
}

void BinaryPointVectorWriterContext::Close()
{
   _OPENGPS_DELETE(m_Stream);
   _OPENGPS_DELETE(m_Buffer);
}

OGPS_Boolean BinaryPointVectorWriterContext::HasStream() const
{
   if(m_Stream)
   {
      _ASSERT(m_Buffer);
      return TRUE;
   }

   return FALSE;
}

OGPS_Boolean BinaryPointVectorWriterContext::IsGood() const
{
   _ASSERT(m_Stream && m_Buffer);

   const std::ios_base::io_state state = m_Stream->rdstate();
   return (state == std::ios_base::goodbit || state == std::ios_base::eofbit);
}

void BinaryPointVectorWriterContext::Skip()
{
   _CHECK_STREAM_AND_THROW_EXCEPTION;
   _CHECK_ISGOOD_AND_THROW_EXCEPTION;
}

void BinaryPointVectorWriterContext::MoveNext()
{
}

void BinaryPointVectorWriterContext::GetMd5(OpenGPS::UnsignedByte md5[16])
{
   _ASSERT(HasStream());
   m_Buffer->GetMd5(md5);
}

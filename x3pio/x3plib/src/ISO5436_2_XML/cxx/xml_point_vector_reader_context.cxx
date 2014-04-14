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

#include "xml_point_vector_reader_context.hxx"
#include "point_vector_iostream.hxx"

#include "stdafx.hxx"

#include <opengps/cxx/string.hxx>
#include <opengps/cxx/exceptions.hxx>

/*! Checks whether the underlying stream is valid. Throws an exception if this is not the case. */
#define _CHECK_STREAM_AND_THROW_EXCEPTION \
   if(!m_Stream) \
   { \
   throw OpenGPS::Exception( \
      OGPS_ExInvalidOperation, \
      _EX_T("No stream object available."), \
      _EX_T("The operation on the stream object failed, because the stream has been released already."), \
      _EX_T("OpenGPS::XmlPointVectorReaderContext")); \
   }

/*! Checks whether the underlying stream is valid. Throws an exception if this is not the case. */
#define _CHECK_ISGOOD_AND_THROW_EXCEPTION \
   if(!IsGood()) \
   { \
   throw OpenGPS::Exception( \
      OGPS_ExInvalidOperation, \
      _EX_T("The underlying stream object became invalid."), \
      _EX_T("A read/write error occured."), \
      _EX_T("OpenGPS::XmlPointVectorReaderContext")); \
   }

PointVectorReaderContext::PointVectorReaderContext()
{
}

PointVectorReaderContext::~PointVectorReaderContext()
{
}

XmlPointVectorReaderContext::XmlPointVectorReaderContext(const StringList* const pointVectorList)
: PointVectorReaderContext(), m_PointVectorList(pointVectorList)
{
   _ASSERT(pointVectorList);

   m_Next = 0;
   m_Stream = NULL;
}

XmlPointVectorReaderContext::~XmlPointVectorReaderContext()
{
   Reset();
}

void XmlPointVectorReaderContext::Set(const OpenGPS::String& buf)
{
   _ASSERT(m_Stream);

   m_Stream->clear(); // reset current state
   m_Stream->str(buf); // set new buffer
}

void XmlPointVectorReaderContext::Reset()
{
   _OPENGPS_DELETE(m_Stream);

   m_Next = 0;
}

void XmlPointVectorReaderContext::Read(OGPS_Int16* const value)
{
   _ASSERT(value);

   _CHECK_STREAM_AND_THROW_EXCEPTION;
   *m_Stream >> *value;
   _CHECK_ISGOOD_AND_THROW_EXCEPTION;
}

void XmlPointVectorReaderContext::Read(OGPS_Int32* const value)
{
   _ASSERT(value);

   _CHECK_STREAM_AND_THROW_EXCEPTION;
   *m_Stream >> *value;
   _CHECK_ISGOOD_AND_THROW_EXCEPTION;
}

void XmlPointVectorReaderContext::Read(OGPS_Float* const value)
{
   _ASSERT(value);

   _CHECK_STREAM_AND_THROW_EXCEPTION;
   *m_Stream >> *value;
   _CHECK_ISGOOD_AND_THROW_EXCEPTION;
}

void XmlPointVectorReaderContext::Read(OGPS_Double* const value)
{
   _ASSERT(value);

   _CHECK_STREAM_AND_THROW_EXCEPTION;
   *m_Stream >> *value;
   _CHECK_ISGOOD_AND_THROW_EXCEPTION;
}

void XmlPointVectorReaderContext::Skip()
{
   _CHECK_STREAM_AND_THROW_EXCEPTION;
   _CHECK_ISGOOD_AND_THROW_EXCEPTION;
}

OGPS_Boolean XmlPointVectorReaderContext::IsGood() const
{
   _ASSERT(m_Stream);

   const std::ios_base::io_state state = m_Stream->rdstate();
   return (state == std::ios_base::goodbit || state == std::ios_base::eofbit);
}

OGPS_Boolean XmlPointVectorReaderContext::MoveNext()
{
   _ASSERT(m_PointVectorList);

   if(m_Next > 0)
   {
      if(m_Next < m_PointVectorList->size())
      {
         Set(m_PointVectorList->at(m_Next++));
         return TRUE;
      }

      Reset();
      return FALSE;
   }

   // non-empty vector list
   if(m_PointVectorList->size() > 0)
   {
      _ASSERT(!m_Stream);

      m_Stream = new PointVectorInputStringStream(m_PointVectorList->at(m_Next++));

      return TRUE;
   }

   return FALSE;
}

OGPS_Boolean XmlPointVectorReaderContext::IsValid() const
{
   return (m_Stream && m_Stream->str().length() > 0);
}

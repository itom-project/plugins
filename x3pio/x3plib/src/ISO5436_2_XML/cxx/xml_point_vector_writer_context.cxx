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

#include "xml_point_vector_writer_context.hxx"
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
      _EX_T("OpenGPS::XmlPointVectorWriterContext")); \
   }

/*! Checks whether the underlying stream is valid. Throws an exception if this is not the case. */
#define _CHECK_ISGOOD_AND_THROW_EXCEPTION \
   if(!IsGood()) \
   { \
   throw OpenGPS::Exception( \
      OGPS_ExInvalidOperation, \
      _EX_T("The underlying stream object became invalid."), \
      _EX_T("A read/write error occured."), \
      _EX_T("OpenGPS::XmlPointVectorWriterContext")); \
   }


PointVectorWriterContext::PointVectorWriterContext()
{
}

PointVectorWriterContext::~PointVectorWriterContext()
{
}

XmlPointVectorWriterContext::XmlPointVectorWriterContext(StringList* const pointVectorList)
: PointVectorWriterContext()
{
   _ASSERT(pointVectorList);

   m_Stream = new PointVectorOutputStringStream();
   m_Stream->setf(std::ios::scientific);
   m_Stream->precision(15);
   m_NeedsSeparator = FALSE;
   m_PointVectorList = pointVectorList;
}

XmlPointVectorWriterContext::~XmlPointVectorWriterContext()
{
   _OPENGPS_DELETE(m_Stream);
}

void XmlPointVectorWriterContext::Get(OpenGPS::String* const value) const
{
   _ASSERT(value);

   if(m_Stream)
   {
      *value = m_Stream->str();
   }
   else
   {
      value->clear();
   }
}

void XmlPointVectorWriterContext::Reset()
{
   if(m_Stream)
   {
      m_Stream->str(_T(""));
   }

   m_NeedsSeparator = FALSE;
}

void XmlPointVectorWriterContext::Write(const OGPS_Int16* const value)
{
   _ASSERT(value);

   _CHECK_STREAM_AND_THROW_EXCEPTION;

   AppendSeparator();
   *m_Stream << *value;

   _CHECK_ISGOOD_AND_THROW_EXCEPTION;
}

void XmlPointVectorWriterContext::Write(const OGPS_Int32* const value)
{
   _ASSERT(value);

   _CHECK_STREAM_AND_THROW_EXCEPTION;

   AppendSeparator();
   *m_Stream << *value;

   _CHECK_ISGOOD_AND_THROW_EXCEPTION;
}

void XmlPointVectorWriterContext::Write(const OGPS_Float* const value)
{
   _ASSERT(value);

   _CHECK_STREAM_AND_THROW_EXCEPTION;

   AppendSeparator();
   *m_Stream << *value;

   _CHECK_ISGOOD_AND_THROW_EXCEPTION;
}

void XmlPointVectorWriterContext::Write(const OGPS_Double* const value)
{
   _ASSERT(value);

   _CHECK_STREAM_AND_THROW_EXCEPTION;

   AppendSeparator();
   *m_Stream << *value;

   _CHECK_ISGOOD_AND_THROW_EXCEPTION;
}

void XmlPointVectorWriterContext::Skip()
{
   _CHECK_STREAM_AND_THROW_EXCEPTION;
   _CHECK_ISGOOD_AND_THROW_EXCEPTION;
}

OGPS_Boolean XmlPointVectorWriterContext::IsGood() const
{
   _ASSERT(m_Stream);

   const std::ios_base::io_state state = m_Stream->rdstate();
   return (state == std::ios_base::goodbit || state == std::ios_base::eofbit);
}

void XmlPointVectorWriterContext::AppendSeparator()
{
   _ASSERT(m_Stream);

   if(m_NeedsSeparator)
   {
      m_Stream->write(_T(";"), 1);
   }

   m_NeedsSeparator = TRUE;
}

void XmlPointVectorWriterContext::MoveNext()
{
   _ASSERT(m_Stream && m_PointVectorList);

   OpenGPS::String vector;
   Get(&vector);
   Schemas::ISO5436_2::DataListType::Datum_type datum(vector);
   m_PointVectorList->push_back(datum);

   Reset();
}

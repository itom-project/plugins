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

#include "point_vector_proxy.hxx"
#include "point_vector_proxy_context.hxx"

#include "point_buffer.hxx"

#include <opengps/cxx/exceptions.hxx>

#include "stdafx.hxx"

/*!
 * Checks whether an instance of a point buffer does exist. If not an exception is thrown.
 * The message describing the cause of failure makes sense for a refused attempt to read point data.
 */
#define _CHECK_BUFFER_AND_THROW_READ_EXCEPTION \
   if(!m_Buffer) \
   { \
      throw OpenGPS::Exception( \
         OGPS_ExInvalidOperation, \
         _EX_T("The value of a data point could not be read."), \
         _EX_T("There exists no buffer of point data where to read from. Most likely this is an attempt to read point data of an implicit axis."), \
         _EX_T("OpenGPS::PointVectorProxy::DataPointProxy::Get")); \
   }

/*!
 * Checks whether an instance of a point buffer does exist. If not an exception is thrown.
 * The message describing the cause of failure makes sense for a refused attempt to write point data.
 */
#define _CHECK_BUFFER_AND_THROW_WRITE_EXCEPTION \
   if(!m_Buffer) \
   { \
      throw OpenGPS::Exception( \
         OGPS_ExInvalidOperation, \
         _EX_T("The value of a data point could not be set."), \
         _EX_T("There exists no buffer of point data where to write to. Most likely this is an attempt to write point data of an implicit axis. There is no need to set point data explicitly then."), \
         _EX_T("OpenGPS::PointVectorProxy::DataPointProxy::Set")); \
   }

PointVectorProxy::DataPointProxy::DataPointProxy(
   const PointVectorProxyContext* const context,
   PointBuffer* const buffer)
: DataPoint(), m_Context(context), m_Buffer(buffer)
{
   _ASSERT(context);
}

PointVectorProxy::DataPointProxy::~DataPointProxy()
{
}

OGPS_DataPointType PointVectorProxy::DataPointProxy::GetPointType() const
{
   if(m_Buffer)
   {
      return m_Buffer->GetPointType();
   }

   return OGPS_MissingPointType;
}

void PointVectorProxy::DataPointProxy::Get(OGPS_Int16* const value) const
{
   _ASSERT(m_Context && value);

   _CHECK_BUFFER_AND_THROW_READ_EXCEPTION;
   m_Buffer->Get(m_Context->GetIndex(), *value);
}

void PointVectorProxy::DataPointProxy::Get(OGPS_Int32* const value) const
{
   _ASSERT(m_Context && value);

   _CHECK_BUFFER_AND_THROW_READ_EXCEPTION;
   m_Buffer->Get(m_Context->GetIndex(), *value);
}

void PointVectorProxy::DataPointProxy::Get(OGPS_Float* const value) const
{
   _ASSERT(m_Context && value);

   _CHECK_BUFFER_AND_THROW_READ_EXCEPTION;
   m_Buffer->Get(m_Context->GetIndex(), *value);
}

void PointVectorProxy::DataPointProxy::Get(OGPS_Double* const value) const
{
   _ASSERT(m_Context && value);

   _CHECK_BUFFER_AND_THROW_READ_EXCEPTION;
   m_Buffer->Get(m_Context->GetIndex(), *value);
}

OGPS_Double PointVectorProxy::DataPointProxy::Get() const
{
   throw OpenGPS::Exception(
      OGPS_ExNotImplemented,
      _EX_T("This interface method is not implemented."),
      _EX_T("Use one of the typesafe getters to obtain a value. The purpose of this specific implementation is to do just that, therefore a generic getter that converts the value to OGPS_Double does not make sense here."),
      _EX_T("OpenGPS::PointVectorProxy::DataPointProxy::Get"));
}

OGPS_Boolean PointVectorProxy::DataPointProxy::IsValid() const
{
   return m_Buffer != NULL;
}

void PointVectorProxy::DataPointProxy::Set(const OGPS_Int16 value)
{
   _ASSERT(m_Context);

   _CHECK_BUFFER_AND_THROW_WRITE_EXCEPTION;
   m_Buffer->Set(m_Context->GetIndex(), value);
}

void PointVectorProxy::DataPointProxy::Set(const OGPS_Int32 value)
{
   _ASSERT(m_Context);

   _CHECK_BUFFER_AND_THROW_WRITE_EXCEPTION;
   m_Buffer->Set(m_Context->GetIndex(), value);
}

void PointVectorProxy::DataPointProxy::Set(const OGPS_Float value)
{
   _ASSERT(m_Context);

   _CHECK_BUFFER_AND_THROW_WRITE_EXCEPTION;
   m_Buffer->Set(m_Context->GetIndex(), value);
}

void PointVectorProxy::DataPointProxy::Set(const OGPS_Double value)
{
   _ASSERT(m_Context);

   _CHECK_BUFFER_AND_THROW_WRITE_EXCEPTION;
   m_Buffer->Set(m_Context->GetIndex(), value);
}

void PointVectorProxy::DataPointProxy::Reset()
{
   // Shouldn't have a buffer, because mainly for an instance of the current
   // data point proxy implementation a reset usually means that a value is skipped
   // when reading from an input stream of vectors.
   // But this is exacly the case, when no explicit point data needs
   // to entered. So this instance should not contain a buffer, because
   // nothing will be there to get stored herein.
   if(m_Buffer)
   {
      throw OpenGPS::Exception(
         OGPS_ExInvalidOperation,
         _EX_T("Failed to reset point data."),
         _EX_T("Failed to reset proxied point data. Most likely there is an attempt to reset a data point although its corresponding axis specifies data points on it as implicit. So there is no need to reset such a point in that context."),
         _EX_T("OpenGPS::PointVectorProxy::DataPointProxy::Reset"));
   }
}

void PointVectorProxy::DataPointProxy::Set(const DataPoint& src)
{
   _ASSERT(m_Context);

   const OGPS_DataPointType srcType = src.GetPointType();
   const OGPS_DataPointType thisType = GetPointType();

   if(srcType != thisType)
   {
      throw OpenGPS::Exception(
         OGPS_ExInvalidArgument,
         _EX_T("Could not copy from the given instance of type data point."),
         _EX_T("This instance can not store the type of point data provided by the instance to be copied. The point data to be set must be exacly of the same type as defined by the targeted axis description."),
         _EX_T("OpenGPS::PointVectorProxy::DataPointProxy::Set"));
   }

   switch(srcType)
   {
   case OGPS_Int16PointType:
      OGPS_Int16 vs;
      src.Get(&vs);
      Set(vs);
      break;
   case OGPS_Int32PointType:
      OGPS_Int32 vl;
      src.Get(&vl);
      Set(vl);
      break;
   case OGPS_FloatPointType:
      OGPS_Float vf;
      src.Get(&vf);
      Set(vf);
      break;
   case OGPS_DoublePointType:
      OGPS_Double vd;
      src.Get(&vd);
      Set(vd);
      break;
   case OGPS_MissingPointType:
      // Nothing to be copied in this case.
      break;
   default:
      throw OpenGPS::Exception(
         OGPS_ExInvalidArgument,
         _EX_T("Could not copy from the given instance of type data point."),
         _EX_T("The value of the type property of the argument given is unknown. It should be one of the following: OGPS_Int16PointType, OGPS_Int32PointType, OGPS_FloatPointType, OGPS_DoublePointType, OGPS_MissingPointType."),
          _EX_T("OpenGPS::PointVectorProxy::DataPointProxy::Set"));
      break;
   }
}

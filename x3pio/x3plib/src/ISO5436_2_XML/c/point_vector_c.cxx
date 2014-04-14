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

#include <opengps/point_vector.h>
#include <opengps/cxx/point_vector.hxx>
#include <opengps/cxx/data_point.hxx>

#include "data_point_c.hxx"
#include "point_vector_c.hxx"
#include "messages_c.hxx"

#include "../cxx/stdafx.hxx"

OGPS_PointVectorPtr ogps_CreatePointVector(void)
{
   OGPS_PointVectorPtr v = new OGPS_PointVector;

   v->x = NULL;
   v->y = NULL;
   v->z = NULL;

   return v;
}

void ogps_FreePointVector(OGPS_PointVectorPtr * vector)
{
   if(*vector)
   {
      _OPENGPS_GENERIC_EXCEPTION_HANDLER( \
         _OPENGPS_DELETE((*vector)->x); \
         _OPENGPS_DELETE((*vector)->y); \
         _OPENGPS_DELETE((*vector)->z); \
         _OPENGPS_DELETE(*vector); \
         );
   }
}

void ogps_SetInt16X(
    OGPS_PointVectorPtr const vector,
    const OGPS_Int16 value)
{
   _ASSERT(vector);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER(vector->instance.SetX(value));
}

void ogps_SetInt32X(
    OGPS_PointVectorPtr const vector,
    const OGPS_Int32 value)
{
   _ASSERT(vector);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER(vector->instance.SetX(value));
}

void ogps_SetFloatX(
    OGPS_PointVectorPtr const vector,
    const OGPS_Float value)
{
   _ASSERT(vector);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER(vector->instance.SetX(value));
}

void ogps_SetDoubleX(
    OGPS_PointVectorPtr const vector,
    const OGPS_Double value)
{
   _ASSERT(vector);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER(vector->instance.SetX(value));
}


void ogps_SetInt16Y(
    OGPS_PointVectorPtr const vector,
    const OGPS_Int16 value)
{
   _ASSERT(vector);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER(vector->instance.SetY(value));
}

void ogps_SetInt32Y(
    OGPS_PointVectorPtr const vector,
    const OGPS_Int32 value)
{
   _ASSERT(vector);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER(vector->instance.SetY(value));
}

void ogps_SetFloatY(
    OGPS_PointVectorPtr const vector,
    const OGPS_Float value)
{
   _ASSERT(vector);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER(vector->instance.SetY(value));
}

void ogps_SetDoubleY(
    OGPS_PointVectorPtr const vector,
    const OGPS_Double value)
{
   _ASSERT(vector);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER(vector->instance.SetY(value));
}


void ogps_SetInt16Z(
    OGPS_PointVectorPtr const vector,
    const OGPS_Int16 value)
{
   _ASSERT(vector);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER(vector->instance.SetZ(value));
}

void ogps_SetInt32Z(
    OGPS_PointVectorPtr const vector,
    const OGPS_Int32 value)
{
   _ASSERT(vector);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER(vector->instance.SetZ(value));
}

void ogps_SetFloatZ(
    OGPS_PointVectorPtr const vector,
    const OGPS_Float value)
{
   _ASSERT(vector);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER(vector->instance.SetZ(value));
}

void ogps_SetDoubleZ(
    OGPS_PointVectorPtr const vector,
    const OGPS_Double value)
{
   _ASSERT(vector);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER(vector->instance.SetZ(value));
}


void ogps_GetXYZ(
        const OGPS_PointVectorPtr vector,
        OGPS_Double* const x,
        OGPS_Double* const y,
        OGPS_Double* const z)
{
   _ASSERT(vector);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER(vector->instance.GetXYZ(x, y, z));
}

void ogps_SetXYZ(
        OGPS_PointVectorPtr vector,
        const OGPS_Double x,
        const OGPS_Double  y,
        const OGPS_Double z)
{
   _ASSERT(vector);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER(vector->instance.SetXYZ(x, y, z));
}

OGPS_DataPointPtr const ogps_GetX(OGPS_PointVectorPtr const vector)
{
   _ASSERT(vector);

   if(!vector->x)
   {
      OpenGPS::DataPoint *p = NULL;
      _OPENGPS_GENERIC_EXCEPTION_HANDLER(p = vector->instance.GetX());
      if(p)
      {
         vector->x = new OGPS_DataPoint;
         vector->x->instance = p;
      }
   }

   return vector->x;
}

OGPS_DataPointPtr const ogps_GetY(OGPS_PointVectorPtr const vector)
{
   _ASSERT(vector);

   if(!vector->y)
   {
      OpenGPS::DataPoint* p = NULL;
      _OPENGPS_GENERIC_EXCEPTION_HANDLER(p = vector->instance.GetY());
      if(p)
      {
         vector->y = new OGPS_DataPoint;
         vector->y->instance = p;
      }
   }

   return vector->y;
}

OGPS_DataPointPtr const ogps_GetZ(OGPS_PointVectorPtr const vector)
{
   _ASSERT(vector);

   if(!vector->z)
   {
      OpenGPS::DataPoint* p = NULL;
      _OPENGPS_GENERIC_EXCEPTION_HANDLER(p = vector->instance.GetZ());
      if(p)
      {
         vector->z = new OGPS_DataPoint;
         vector->z->instance = p;
      }
   }

   return vector->z;
}

OGPS_Boolean ogps_IsValidPoint(const OGPS_PointVectorPtr vector)

{
   _ASSERT(vector);

   OGPS_Boolean retval = FALSE;
   _OPENGPS_GENERIC_EXCEPTION_HANDLER(retval = vector->instance.IsValid());
   return retval;
}

OGPS_DataPointType ogps_GetPointTypeX(const OGPS_PointVectorPtr vector)
{
   _ASSERT(vector);

   OGPS_DataPointType value = OGPS_MissingPointType;
   _OPENGPS_GENERIC_EXCEPTION_HANDLER(value = vector->instance.GetX()->GetPointType());
   return value;
}

OGPS_Int16 ogps_GetInt16X(const OGPS_PointVectorPtr vector)
{
   _ASSERT(vector);

   OGPS_Int16 value = 0;
   _OPENGPS_GENERIC_EXCEPTION_HANDLER(vector->instance.GetX(&value));
   return value;
}

OGPS_Int32 ogps_GetInt32X(const OGPS_PointVectorPtr vector)
{
   _ASSERT(vector);

   OGPS_Int32 value = 0;
   _OPENGPS_GENERIC_EXCEPTION_HANDLER(vector->instance.GetX(&value));
   return value;
}

OGPS_Float ogps_GetFloatX(const OGPS_PointVectorPtr vector)
{
   _ASSERT(vector);

   OGPS_Float value = 0.0F;
   _OPENGPS_GENERIC_EXCEPTION_HANDLER(vector->instance.GetX(&value));
   return value;
}

OGPS_Double ogps_GetDoubleX(const OGPS_PointVectorPtr vector)
{
   _ASSERT(vector);

   OGPS_Double value = 0.0;
   _OPENGPS_GENERIC_EXCEPTION_HANDLER(vector->instance.GetX(&value));
   return value;
}

OGPS_DataPointType ogps_GetPointTypeY(const OGPS_PointVectorPtr vector)
{
   _ASSERT(vector);

   OGPS_DataPointType value = OGPS_MissingPointType;
   _OPENGPS_GENERIC_EXCEPTION_HANDLER(value = vector->instance.GetY()->GetPointType());
   return value;
}

OGPS_Int16 ogps_GetInt16Y(const OGPS_PointVectorPtr vector)
{
   _ASSERT(vector);

   OGPS_Int16 value = 0;
   _OPENGPS_GENERIC_EXCEPTION_HANDLER(vector->instance.GetY(&value));
   return value;
}

OGPS_Int32 ogps_GetInt32Y(const OGPS_PointVectorPtr vector)
{
   _ASSERT(vector);

   OGPS_Int32 value = 0;
   _OPENGPS_GENERIC_EXCEPTION_HANDLER(vector->instance.GetY(&value));
   return value;
}

OGPS_Float ogps_GetFloatY(const OGPS_PointVectorPtr vector)
{
   _ASSERT(vector);

   OGPS_Float value = 0.0F;
   _OPENGPS_GENERIC_EXCEPTION_HANDLER(vector->instance.GetY(&value));
   return value;
}

OGPS_Double ogps_GetDoubleY(const OGPS_PointVectorPtr vector)
{
   _ASSERT(vector);

   OGPS_Double value = 0.0;
   _OPENGPS_GENERIC_EXCEPTION_HANDLER(vector->instance.GetY(&value));
   return value;
}

OGPS_DataPointType ogps_GetPointTypeZ(const OGPS_PointVectorPtr vector)
{
   _ASSERT(vector);

   OGPS_DataPointType value = OGPS_MissingPointType;
   _OPENGPS_GENERIC_EXCEPTION_HANDLER(value = vector->instance.GetZ()->GetPointType());
   return value;
}

OGPS_Int16 ogps_GetInt16Z(const OGPS_PointVectorPtr vector)
{
   _ASSERT(vector);

   OGPS_Int16 value = 0;
   _OPENGPS_GENERIC_EXCEPTION_HANDLER(vector->instance.GetZ(&value));
   return value;
}

OGPS_Int32 ogps_GetInt32Z(const OGPS_PointVectorPtr vector)
{
   _ASSERT(vector);

   OGPS_Int32 value = 0;
   _OPENGPS_GENERIC_EXCEPTION_HANDLER(vector->instance.GetZ(&value));
   return value;
}

OGPS_Float ogps_GetFloatZ(const OGPS_PointVectorPtr vector)
{
   _ASSERT(vector);

   OGPS_Float value = 0.0F;
   _OPENGPS_GENERIC_EXCEPTION_HANDLER(vector->instance.GetZ(&value));
   return value;
}

OGPS_Double ogps_GetDoubleZ(const OGPS_PointVectorPtr vector)
{
   _ASSERT(vector);

   OGPS_Double value = 0.0;
   _OPENGPS_GENERIC_EXCEPTION_HANDLER(vector->instance.GetZ(&value));
   return value;
}

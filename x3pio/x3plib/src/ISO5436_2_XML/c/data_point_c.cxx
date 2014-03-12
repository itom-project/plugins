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

#include <opengps/data_point.h>

#include "data_point_c.hxx"
#include "messages_c.hxx"

#include "../cxx/data_point_impl.hxx"
#include "../cxx/stdafx.hxx"

OGPS_DataPointType ogps_GetPointType(const OGPS_DataPointPtr dataPoint)
{
   _ASSERT(dataPoint && dataPoint->instance);

   OGPS_DataPointType retval = OGPS_MissingPointType;
   _OPENGPS_GENERIC_EXCEPTION_HANDLER(retval = dataPoint->instance->GetPointType());
   return retval;
}

short ogps_GetInt16(const OGPS_DataPointPtr dataPoint)
{
   _ASSERT(dataPoint && dataPoint->instance);

   short v = 0;
   _OPENGPS_GENERIC_EXCEPTION_HANDLER(dataPoint->instance->Get(&v));
   return v;
}

int ogps_GetInt32(const OGPS_DataPointPtr dataPoint)
{
   _ASSERT(dataPoint && dataPoint->instance);

   int v = 0;
   _OPENGPS_GENERIC_EXCEPTION_HANDLER(dataPoint->instance->Get(&v));
   return v;
}

float ogps_GetFloat(const OGPS_DataPointPtr dataPoint)
{
   _ASSERT(dataPoint && dataPoint->instance);

   float v = 0.0F;
   _OPENGPS_GENERIC_EXCEPTION_HANDLER(dataPoint->instance->Get(&v));
   return v;
}

double ogps_GetDouble(const OGPS_DataPointPtr dataPoint)
{
   _ASSERT(dataPoint && dataPoint->instance);

   double v = 0.0;
   _OPENGPS_GENERIC_EXCEPTION_HANDLER(dataPoint->instance->Get(&v));
   return v;
}

void ogps_SetInt16(
    OGPS_DataPointPtr const dataPoint,
    const short value)
{
   _ASSERT(dataPoint && dataPoint->instance);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER(dataPoint->instance->Set(value));
}

void ogps_SetInt32(
    OGPS_DataPointPtr const dataPoint,
    const int value)
{
   _ASSERT(dataPoint && dataPoint->instance);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER(dataPoint->instance->Set(value));
}

void ogps_SetFloat(
    OGPS_DataPointPtr const dataPoint,
    const float value)
{
   _ASSERT(dataPoint && dataPoint->instance);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER(dataPoint->instance->Set(value));
}

void ogps_SetDouble(
    OGPS_DataPointPtr const dataPoint,
    const double value)
{
   _ASSERT(dataPoint && dataPoint->instance);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER(dataPoint->instance->Set(value));
}

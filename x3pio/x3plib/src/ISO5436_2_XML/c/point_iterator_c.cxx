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

#include <opengps/point_iterator.h>

#include "point_iterator_c.hxx"
#include "point_vector_c.hxx"

#include "messages_c.hxx"

#include <opengps/cxx/point_iterator.hxx>

#include "../cxx/stdafx.hxx"

OGPS_Boolean ogps_HasNextPoint(const OGPS_PointIteratorPtr iterator)
{
   _ASSERT(iterator && iterator->instance);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER_RETVALBOOL(iterator->instance->HasNext());
}

OGPS_Boolean ogps_HasPrevPoint(const OGPS_PointIteratorPtr iterator)
{
   _ASSERT(iterator && iterator->instance);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER_RETVALBOOL(iterator->instance->HasPrev());
}

OGPS_Boolean ogps_MoveNextPoint(OGPS_PointIteratorPtr const iterator)
{
   _ASSERT(iterator && iterator->instance);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER_RETVALBOOL(iterator->instance->MoveNext());
}

OGPS_Boolean ogps_MovePrevPoint(OGPS_PointIteratorPtr const iterator)
{
   _ASSERT(iterator && iterator->instance);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER_RETVALBOOL(iterator->instance->MovePrev());
}

void ogps_ResetNextPointIterator(OGPS_PointIteratorPtr const iterator)
{
   _ASSERT(iterator && iterator->instance);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER(iterator->instance->ResetNext());
}

void ogps_ResetPrevPointIterator(OGPS_PointIteratorPtr const iterator)
{
   _ASSERT(iterator && iterator->instance);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER(iterator->instance->ResetPrev());
}

void ogps_GetCurrentPoint(
        const OGPS_PointIteratorPtr iterator,
        OGPS_PointVectorPtr const vector)
{
   _ASSERT(iterator && iterator->instance && vector);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER(iterator->instance->GetCurrent(vector->instance));
}

void ogps_GetCurrentCoord(
        const OGPS_PointIteratorPtr iterator,
        OGPS_PointVectorPtr const vector)
{
   _ASSERT(iterator && iterator->instance && vector);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER(iterator->instance->GetCurrentCoord(vector->instance));
}

void ogps_SetCurrentPoint(
        const OGPS_PointIteratorPtr iterator,
        const OGPS_PointVectorPtr vector)
{
   _ASSERT(iterator && iterator->instance);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER(iterator->instance->SetCurrent(vector ? &vector->instance : NULL));
}

OGPS_Boolean ogps_GetMatrixPosition(
        const OGPS_PointIteratorPtr iterator,
        OGPS_ULong * u,
        OGPS_ULong * v,
        OGPS_ULong * w)
{
   _ASSERT(iterator && iterator->instance);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER_RETVALBOOL(iterator->instance->GetPosition(u, v, w));
}

OGPS_Boolean ogps_GetListPosition(
        const OGPS_PointIteratorPtr iterator,
        OGPS_ULong * index)
{
   _ASSERT(iterator && iterator->instance);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER_RETVALBOOL(iterator->instance->GetPosition(index));
}

void ogps_FreePointIterator(OGPS_PointIteratorPtr * const iterator)
{
   if(*iterator)
   {
      _ASSERT((*iterator)->instance);

      _OPENGPS_GENERIC_EXCEPTION_HANDLER( \
         _OPENGPS_DELETE((*iterator)->instance); \
         _OPENGPS_DELETE(*iterator); \
         );
   }
}

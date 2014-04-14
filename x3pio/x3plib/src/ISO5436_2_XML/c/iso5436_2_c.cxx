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

#include <opengps/iso5436_2.h>
#include <opengps/cxx/iso5436_2.hxx>
#include <opengps/cxx/iso5436_2_handle.hxx>

#include "iso5436_2_handle_c.hxx"
#include "point_iterator_c.hxx"
#include "point_vector_c.hxx"

#include "messages_c.hxx"

#include "../cxx/iso5436_2_container.hxx"
#include "../cxx/stdafx.hxx"

OGPS_ISO5436_2Handle ogps_OpenISO5436_2(
        const OGPS_Character* const file,
        const OGPS_Character* const temp,
        const OGPS_Boolean readOnly)
{
   _ASSERT(file);

   OGPS_ISO5436_2Handle h = NULL;

   _OPENGPS_GENERIC_EXCEPTION_HANDLER_CLEANUP(

      /* Statement within try-block */ \
      h = new OGPS_ISO5436_2; \
      \
      h->instance = new OpenGPS::ISO5436_2Container( \
         file, \
         temp ? temp : _T("")); \
      \
      h->instance->Open(readOnly); \
      , \
      \
      /* Statement within catch-block */ \
      _OPENGPS_DELETE(h); \

   );

   return h;
}

OGPS_ISO5436_2Handle ogps_CreateMatrixISO5436_2(
   const OGPS_Character* const file,
   const OGPS_Character* const temp,
   const OpenGPS::Schemas::ISO5436_2::Record1Type& record1,
   const OpenGPS::Schemas::ISO5436_2::Record2Type* record2,
   const OpenGPS::Schemas::ISO5436_2::MatrixDimensionType& matrixDimension,
   const OGPS_Boolean useBinaryData)
{
   _ASSERT(file);

   OGPS_ISO5436_2Handle h = NULL;

   _OPENGPS_GENERIC_EXCEPTION_HANDLER_CLEANUP(

      /* Statement within try-block */ \
      h = new OGPS_ISO5436_2; \
      \
      h->instance = new OpenGPS::ISO5436_2Container( \
         file, \
         temp ? temp : _T("")); \
      \
      h->instance->Create( \
         record1, \
         record2, \
         matrixDimension, \
         useBinaryData); \
      , \
      \
      /* Statement within catch-block */ \
      _OPENGPS_DELETE(h); \

   );

   return h;
}

OGPS_ISO5436_2Handle ogps_CreateListISO5436_2(
        const OGPS_Character* file,
        const OGPS_Character* temp,
        const Schemas::ISO5436_2::Record1Type& record1,
        const Schemas::ISO5436_2::Record2Type* record2,
        const OGPS_ULong listDimension,
        const OGPS_Boolean useBinaryData)
{
   _ASSERT(file);

   OGPS_ISO5436_2Handle h = NULL;

   _OPENGPS_GENERIC_EXCEPTION_HANDLER_CLEANUP(

      /* Statement within try-block */ \
      h = new OGPS_ISO5436_2; \
      \
      h->instance = new OpenGPS::ISO5436_2Container( \
         file, \
         temp ? temp : _T("")); \
      \
      h->instance->Create( \
         record1, \
         record2, \
         listDimension, \
         useBinaryData); \
      , \
      \
      /* Statement within catch-block */ \
      _OPENGPS_DELETE(h); \

   );

   return h;
}

Schemas::ISO5436_2::ISO5436_2Type* const ogps_GetDocument(const OGPS_ISO5436_2Handle handle)
{
   _ASSERT(handle && handle->instance);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER(return handle->instance->GetDocument());

   return NULL;
}

void ogps_WriteISO5436_2(const OGPS_ISO5436_2Handle handle, const int compressionLevel)
{
   _ASSERT(handle && handle->instance);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER(handle->instance->Write(compressionLevel));
}

void ogps_CloseISO5436_2(OGPS_ISO5436_2Handle* handle)
{
   _ASSERT(handle);

   OGPS_ISO5436_2Handle h = *handle;

   if(h)
   {
      _ASSERT(h->instance);

      _OPENGPS_GENERIC_EXCEPTION_HANDLER( \
         h->instance->Close(); \
         _OPENGPS_DELETE(h->instance); \
         _OPENGPS_DELETE(h); \
         );
   }
}

OGPS_PointIteratorPtr ogps_CreateNextPointIterator(const OGPS_ISO5436_2Handle handle)
{
   _ASSERT(handle && handle->instance);

   OGPS_PointIteratorPtr iter = NULL;

   _OPENGPS_GENERIC_EXCEPTION_HANDLER_CLEANUP(

         /* Statement within try-block */ \
         iter = new OGPS_PointIterator(); \
         iter->instance = handle->instance->CreateNextPointIterator().release(); \
         , \
         /* Statement within catch-block */ \
         _OPENGPS_DELETE(iter); \
      );

   return iter;
}

OGPS_PointIteratorPtr ogps_CreatePrevPointIterator(const OGPS_ISO5436_2Handle handle)
{
   _ASSERT(handle && handle->instance);

   OGPS_PointIteratorPtr iter = NULL;

   _OPENGPS_GENERIC_EXCEPTION_HANDLER_CLEANUP(

         /* Statement within try-block */ \
         iter = new OGPS_PointIterator(); \
         iter->instance = handle->instance->CreatePrevPointIterator().release(); \
         , \
         /* Statement within catch-block */ \
         _OPENGPS_DELETE(iter); \
      );

   return iter;
}

void ogps_SetMatrixPoint(
        const OGPS_ISO5436_2Handle handle,
        const OGPS_ULong u,
        const OGPS_ULong v,
        const OGPS_ULong w,
        const OGPS_PointVectorPtr vector)
{
   _ASSERT(handle && handle->instance);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER(handle->instance->SetMatrixPoint(u, v, w, vector ? &vector->instance : NULL));
}

void ogps_GetMatrixPoint(
        const OGPS_ISO5436_2Handle handle,
        const OGPS_ULong u,
        const OGPS_ULong v,
        const OGPS_ULong w,
        OGPS_PointVectorPtr const vector)
{
   _ASSERT(handle && handle->instance && vector);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER(handle->instance->GetMatrixPoint(u, v, w, vector->instance));
}

void ogps_SetListPoint(
        const OGPS_ISO5436_2Handle handle,
        const OGPS_ULong index,
        const OGPS_PointVectorPtr vector)
{
   _ASSERT(handle && handle->instance && vector);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER(handle->instance->SetListPoint(index, vector->instance));
}

void ogps_GetListPoint(
        const OGPS_ISO5436_2Handle handle,
        const OGPS_ULong index,
        OGPS_PointVectorPtr const vector)
{
   _ASSERT(handle && handle->instance && vector);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER(handle->instance->GetListPoint(index, vector->instance));
}

void ogps_GetMatrixCoord(
        const OGPS_ISO5436_2Handle handle,
        const OGPS_ULong u,
        const OGPS_ULong v,
        const OGPS_ULong w,
        OGPS_Double* const x,
        OGPS_Double* const y,
        OGPS_Double* const z)
{
   _ASSERT(handle && handle->instance);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER(handle->instance->GetMatrixCoord(u, v, w, x, y, z));
}

OGPS_Boolean ogps_IsMatrixCoordValid(
        const OGPS_ISO5436_2Handle handle,
        const OGPS_ULong u,
        const OGPS_ULong v,
        const OGPS_ULong w)
{
   _ASSERT(handle && handle->instance);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER_RETVALBOOL(handle->instance->IsMatrixCoordValid(u, v, w));
}

void ogps_GetListCoord(
        const OGPS_ISO5436_2Handle handle,
        const OGPS_ULong index,
        OGPS_Double* const x,
        OGPS_Double* const y,
        OGPS_Double* const z)
{
   _ASSERT(handle && handle->instance);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER(handle->instance->GetListCoord(index, x, y, z));
}

void ogps_AppendVendorSpecific(
                            const OGPS_ISO5436_2Handle handle,
                            const OGPS_Character* vendorURI,
                            const OGPS_Character* filePath)
{
   _ASSERT(handle && handle->instance);
   _ASSERT(vendorURI && filePath);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER(handle->instance->AppendVendorSpecific(vendorURI, filePath));
}

OGPS_Boolean ogps_GetVendorSpecific(
                                    const OGPS_ISO5436_2Handle handle,
                                    const OGPS_Character* vendorURI,
                                    const OGPS_Character* fileName,
                                    const OGPS_Character* targetPath)
{
   _ASSERT(handle && handle->instance);
   _ASSERT(vendorURI && fileName && targetPath);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER(return handle->instance->GetVendorSpecific(vendorURI, fileName, targetPath));

   return FALSE;
}

OGPS_Boolean ogps_IsMatrix(const OGPS_ISO5436_2Handle handle)
{
   _ASSERT(handle && handle->instance);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER(return handle->instance->IsMatrix());

   return FALSE;
}

void ogps_GetMatrixDimensions(const OGPS_ISO5436_2Handle handle,
                                      OGPS_ULong * const size_u,
                                      OGPS_ULong * const size_v,
                                      OGPS_ULong * const size_w)
{
  _ASSERT(handle && handle->instance);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER(handle->instance->GetMatrixDimensions(size_u, size_v, size_w));
}

OGPS_ULong ogps_GetListDimension(const OGPS_ISO5436_2Handle handle)
  {
   _ASSERT(handle && handle->instance);

   _OPENGPS_GENERIC_EXCEPTION_HANDLER(return handle->instance->GetListDimension());

   return 0;
}

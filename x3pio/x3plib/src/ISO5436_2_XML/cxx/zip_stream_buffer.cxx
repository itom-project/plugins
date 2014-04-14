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

#include <string.h>
#include "zip_stream_buffer.hxx"
#include "stdafx.hxx"

#include <opengps/cxx/string.hxx>

ZipStreamBuffer::ZipStreamBuffer(zipFile handle, const OGPS_Boolean enable_md5)
: BaseType()
{
   m_Handle = handle;

   if(enable_md5)
   {
      m_Md5Context = new md5_context;
      md5_starts(m_Md5Context);
   }
   else
   {
      m_Md5Context = NULL;
   }
}

ZipStreamBuffer::~ZipStreamBuffer()
{
   _OPENGPS_DELETE(m_Md5Context);
}

std::streamsize ZipStreamBuffer::xsputn(const char * const s, std::streamsize n )
{
   if(m_Md5Context)
   {
      md5_update(m_Md5Context, (const unsigned char*)s, n);
   }

   if(zipWriteInFileInZip(m_Handle, s, n) == ZIP_OK)
   {
      return n;
   }

   return 0;
}

OGPS_Boolean ZipStreamBuffer::GetMd5(OpenGPS::UnsignedByte md5[16])
{
   if(m_Md5Context)
   {
      md5_finish(m_Md5Context, md5);
      md5_starts(m_Md5Context);

      return TRUE;
   }

   return FALSE;
}

ZipOutputStream::ZipOutputStream(ZipStreamBuffer& buffer)
: BaseType(&buffer)
{
}

ZipOutputStream::~ZipOutputStream()
{
}

ZipOutputStream::BaseType& ZipOutputStream::write(const char *s)
{
   if(s)
   {
      const size_t size = strlen(s);
      return BaseType::write(s, size);
   }

   return *this;
}

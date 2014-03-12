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

/*! @file
 * Integrates Info-Zip handles into the common standard io framework.
 */

#ifndef _OPENGPS_ZIP_STREAM_BUFFER_HXX
#define _OPENGPS_ZIP_STREAM_BUFFER_HXX

#include <ostream>

/* zlib/minizip */
#include <zip.h>

#include "../xyssl/md5.h"

#ifndef _OPENGPS_CXX_OPENGPS_HXX
#  include <opengps/cxx/opengps.hxx>
#endif

namespace OpenGPS
{
   class String;

   /*!
    * Provides a buffer interface suitable for streaming the zipFile
    * handle defined in the zlib/minizip package.
    * @see ZipOutputStream
    */
   class ZipStreamBuffer : public std::streambuf
   {
      /*! Data type of the super class. */
      typedef std::streambuf BaseType;

   public:
      /*!
       * Creates a new instance.
       * @param handle The Info-Zip file handle buffered binary data is written to.
       * @param enable_md5 When set to TRUE generates md5 checksums of the buffered
       * binary data, if FALSE no cheksum data will be generated.
       */
      ZipStreamBuffer(zipFile handle, const OGPS_Boolean enable_md5);

      /*! Destroys this instance. */
      ~ZipStreamBuffer();

      /*!
       * Gets the current md5 checksum. Also resets the computed md5 data internally.
       * @param md5 Gets the 128-bit md5 data.
       * @returns Returns TRUE on success, FALSE otherwise.
       */
      OGPS_Boolean GetMd5(OpenGPS::UnsignedByte md5[16]);

   protected:
      /*! Overrides the super class. */
      virtual std::streamsize xsputn( const char * s, std::streamsize n );

   private:
      /*! Handle to the zipFile where buffered data gets written to. */
      zipFile m_Handle;

      /*! The current state of md5 checksum processing. */
      md5_context *m_Md5Context;
   };

   /*!
    * Provides an output stream interface to write binary data to Info-Zip archives.
    */
   class ZipOutputStream : public std::ostream
   {
   public:
      /*! Data type of the super class. */
      typedef std::ostream BaseType;

   public:
      /*!
       * Creates a new instance.
       * @param buffer The buffer object that is streamed.
       */
      ZipOutputStream(ZipStreamBuffer& buffer);

      /*! Destroys this instance. */
      ~ZipOutputStream();

      /*!
       * Appends a string to the stream.
       * @param s The string to append.
       */
      ZipOutputStream::BaseType& write(const char *s);
   };
}

#endif /* _OPENGPS_ZIP_STREAM_BUFFER_HXX */

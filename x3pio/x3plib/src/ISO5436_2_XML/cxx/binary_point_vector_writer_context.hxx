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
 * Interface for writing typed point data to an underlying binary of point vectors.
 */

#ifndef _OPENGPS_BINARY_POINT_VECTOR_WRITER_CONTEXT_HXX
#define _OPENGPS_BINARY_POINT_VECTOR_WRITER_CONTEXT_HXX

#ifndef _OPENGPS_POINT_VECTOR_WRITER_CONTEXT_HXX
#  include "point_vector_writer_context.hxx"
#endif

#ifndef _OPENGPS_ZIP_STREAM_BUFFER_HXX
#  include "zip_stream_buffer.hxx"
#endif

namespace OpenGPS
{
   /*!
    * Implements OpenGPS::PointVectorWriterContext for writing to
    * compressed binary streams of point vectors. Usually
    * this points to a file descriptor within a zip archive.
    */
   class BinaryPointVectorWriterContext : public PointVectorWriterContext
   {
   public:
      /*!
       * Creates a new instance.
       * @param handle The zip-stream where binary data is written to.
       */
      BinaryPointVectorWriterContext(zipFile handle);

      /*!
       * Closes the internal handle to the binary stream and frees its resources.
       */
      virtual void Close();

      virtual void Skip();
      virtual void MoveNext();

      /*!
       * Gets the md5 checksum of all bytes written.
       * When called this resets the currently computed md5 sum. Future
       * calls to this method will ignore older md5 data.
       */
      void GetMd5(OpenGPS::UnsignedByte md5[16]);

   protected:
      /*! Destroys this instance. */
      virtual ~BinaryPointVectorWriterContext();

      /*!
       * Gets the internal binary stream.
       * @returns Returns the target binary stream or NULL.
       */
      std::ostream* GetStream();

      /*!
       * Asks whether there is a target stream available.
       * @returns Returns TRUE if there is an operable target
       * stream for point data, FALSE otherwise.
       * @see BinaryPointVectorWriterContext::IsGood
       */
      OGPS_Boolean HasStream() const;

      /*!
       * Asks if the underlying data stream is still valid.
       * @returns Returns TRUE if no previous access to the underlying
       * data stream was harmful. Returns FALSE if any damage occured.
       */
      OGPS_Boolean IsGood() const;

   private:
      /*! The target buffer where compressed binary data gets stored. */
      ZipStreamBuffer* m_Buffer;

      /*! The stream buffer which makes BinaryPointVectorWriterContext::m_Buffer
       * accessible through the more abstract std::ostream interface. */
      ZipOutputStream* m_Stream;
   };
}

#endif /* _OPENGPS_BINARY_POINT_VECTOR_WRITER_CONTEXT_HXX */

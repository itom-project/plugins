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
 * Locale related stuff to support locale invariant parsing of point vector data .
 */

#ifndef _OPENGPS_POINT_VECTOR_IOSTREAM_HXX
#define _OPENGPS_POINT_VECTOR_IOSTREAM_HXX

#ifndef _OPENGPS_CXX_OPENGPS_HXX
#  include <opengps/cxx/opengps.hxx>
#endif

#if linux
    #define GCC_VERSION (__GNUC__ * 10000 \
        + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
    #if GCC_VERSION >= 022600
        #include <locale.h>
    #else
        #include <xlocale.h>
    #endif
#else
    #include <xlocale>
#endif
#include <sstream>
#include <fstream>

namespace OpenGPS
{
   class String;

   /*!
    * Redefines the white space character set to make parsing of a
    * point vector easier. In an ISO5436-2 XML document the three
    * components of a point vector are separated by a semicolon.
    * This facet classifies the semicolon as white space, too. This
    * way the commonly known standard io operator can be used as
    * usual to read the three separate components of the point vector
    * from the string stream.
    */
   class PointVectorWhitespaceFacet : public std::ctype<OGPS_Character>
   {
      /*! The type of the super class. */
      typedef std::ctype<OGPS_Character> BaseType;

   public:
      /*! Creates a new instance. */
      PointVectorWhitespaceFacet(const size_t refs = 0);

      /*! Destroys this instance. */
      ~PointVectorWhitespaceFacet();

   protected:
      /*!
       * Classifies a character.
       * @param msk Check whether the character belongs to the
       * sepcified group.
       * @param ch The character to be classified.
       */
      virtual bool do_is(mask msk, OGPS_Character ch) const;
   };

   /*!
    * The culture invariant locale.
    * Uses an instance of OpenGPS::PointVectorWhitespaceFacet
    * to classify white space.
    */
   class PointVectorInvariantLocale : public std::locale
   {
      /*! The type of the super class. */
      typedef std::locale BaseType;

   public:
      /*! Creates a new instance. */
      PointVectorInvariantLocale();

      /*! Destroys this instance. */
      ~PointVectorInvariantLocale();
   };

   /*! A string stream used for parsing a point vector as defined in ISO5436-2 XML. */
   class PointVectorInputStringStream : public std::basic_istringstream<OGPS_Character>
   {
      /*! The type of the super class. */
      typedef std::basic_istringstream<OGPS_Character> BaseType;

   public:
      /*! Creates a new instance. */
      PointVectorInputStringStream();

      /*!
       * Creates a new instance.
       * @param s The source which is streamed.
       */
      PointVectorInputStringStream(const OpenGPS::String& s);

      /*! Destroys this instance. */
      ~PointVectorInputStringStream();

   private:
      /*! Invariant locale. */
      PointVectorInvariantLocale m_Locale;
   };

   /*! A locale invariant string stream used to convert a point vector object to its
    * representation as text according to the ISO5436-2 XML specification. */
   class PointVectorOutputStringStream : public std::basic_ostringstream<OGPS_Character>
   {
      /*! The type of the super class. */
      typedef std::basic_ostringstream<OGPS_Character> BaseType;

   public:
      /*! Creates a new instance. */
      PointVectorOutputStringStream();

      /*! Destroys this instance. */
      ~PointVectorOutputStringStream();

   private:
      /*! Invariant locale. */
      PointVectorInvariantLocale m_Locale;
   };

   /*! A binary stream class used for reading from binary files. */
   class InputBinaryFileStream : public std::basic_ifstream<OpenGPS::Byte>
   {
      /*! The type of the super class. */
      typedef std::basic_ifstream<OpenGPS::Byte> BaseType;

   public:
      /*!
       * Creates a new instance.
       * @param filePath Full path to the binary file to be read.
       */
      InputBinaryFileStream(const OpenGPS::String& filePath);

      /*! Destroys this instance. */
      ~InputBinaryFileStream();

   private:
      /*! Invariant locale. */
      PointVectorInvariantLocale m_Locale;
   };

   /*! A binary stream class used for writing to binary files. */
   class OutputBinaryFileStream : public std::basic_ofstream<OpenGPS::Byte>
   {
      /*! The type of the super class. */
      typedef std::basic_ofstream<OpenGPS::Byte> BaseType;

   public:
      /*!
       * Creates a new instance.
       * @param filePath Full path to the binary file to be written.
       */
      OutputBinaryFileStream(const OpenGPS::String& filePath);

      /*! Destroys this instance. */
      ~OutputBinaryFileStream();

   private:
      /*! Invariant locale. */
      PointVectorInvariantLocale m_Locale;
   };
}

#endif /* _OPENGPS_POINT_VECTOR_IOSTREAM_HXX */

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
 * An interface to provide access to the environment of different
 * architectures and operating systems.
 */

#ifndef _OPENGPS_ENVIRONMENT_HXX
#define _OPENGPS_ENVIRONMENT_HXX

#ifndef _OPENGPS_CXX_OPENGPS_HXX
#  include <opengps/cxx/opengps.hxx>
#endif

namespace OpenGPS
{
   class String;

   /*!
    * Interface for communicating with the operating system and related subjects.
    * This encapsulates the differences between hardware architectures, compilers
    * and operation systems this software library will run upon.
    */
   class Environment
   {
   public:
      /*!
       * Creates a new instance of the environment if needed.
       *
       * @remarks You must call Environment::Reset somewhere within the scope
       * this method has been accessed, e.g. the end of object lifetime within a class scope
       * or at least once for all on termination of the program.
       * Otherwise resources obtained will not get released correctly.
       */
      static const Environment* const GetInstance();

      /*!
       * Frees resources obtained through Environment::GetInstance.
       *
       * @remarks This must be called at least once at the end of some scope in your program
       * when Environment::GetInstance has been called.
       * @see Environment::GetInstance
       */
      static void Reset();

      /*!
       * E.g. gets the backslash on Microsoft Windows systems.
       */
      virtual OGPS_Character GetDirectorySeparator() const = 0;

      /*!
       * E.g. gets the forward slash on Microsoft Windows systems.
       */
      virtual OGPS_Character GetAltDirectorySeparator() const = 0;

      /*!
       * Converts a given path to a normalized version suitable for the current operating system.
       *
       * @param path A path name.
       * @param clean_path Gets a normalized path name as the result.
       * @returns Returns TRUE on success, FALSE otherwise.
       */
      virtual OGPS_Boolean GetPathName(const OpenGPS::String& path, OpenGPS::String& clean_path) const = 0;

      /*!
       * Gets the file name including its extension from a full path.
       *
       * @param The full path of a file.
       * @returns Returns just the filename.
       */
      virtual OpenGPS::String GetFileName(const OpenGPS::String& path) const = 0;

      /*!
       * Concatenates two distinct path names.
       *
       * @param path1 The first part of the path.
       * @param path2 The second part of the path.
       * @returns Returns the concatenated path name or an empty string on error.
       */
      virtual OpenGPS::String ConcatPathes(const OpenGPS::String& path1, const OpenGPS::String& path2) const = 0;

      /*!
       * Returns TRUE if a given path exists, FALSE otherwise.
       * @param file The path to check.
       */
      virtual OGPS_Boolean PathExists(const OpenGPS::String& file) const = 0;

      /*!
       * Deletes a given file.
       * @param file The path to the file to be erased.
       */
      virtual OGPS_Boolean RemoveFile(const OpenGPS::String& file) const = 0;

      /*!
       * Gets a random sequence of numeric characters.
       */
      virtual OpenGPS::String GetUniqueName() const = 0;

      /*!
       * Makes a directory.
       * @param path The directory to be created.
       * @returns Returns TRUE on success, FALSE otherwise.
       */
      virtual OGPS_Boolean CreateDir(const OpenGPS::String& path) const = 0;

      /*!
       * Deletes a given directory.
       * @param path The path to the directory to be erased.
       */
      virtual OGPS_Boolean RemoveDir(const OpenGPS::String& path) const = 0;

      /*!
       * Returns the path to the temporary directory or an empty string on error.
       */
      virtual OpenGPS::String GetTempDir() const = 0;

      /*!
       * Renames a file.
       *
       * @param src The path to the file to be renamed.
       * @param dst The new path of the renamed file.
       * @returns Returns TRUE on success, FALSE otherwise.
       */
      virtual OGPS_Boolean RenameFile(const OpenGPS::String& src, const OpenGPS::String& dst) const = 0;

      /*!
       * Gets the value of a named environment variable.
       *
       * @param varName The name of the environment variable.
       * @param value Gets the value of the given environment variable.
       * @returns Returns TRUE if the value of the environment variable could be obtained, FALSE otherwise.
       */
      virtual OGPS_Boolean GetVariable(const OpenGPS::String& varName, OpenGPS::String& value) const = 0;

      /*!
       * Returns the last error message of a failed system API call.
       */
      virtual OpenGPS::String GetLastErrorMessage() const = 0;

      /*!
       * Returns TRUE on a system with little endian byte order, FALSE on a system using big endian byte order.
       */
      virtual OGPS_Boolean IsLittleEndian() const;

      /*!
       * Swaps endianness.
       *
       * @param value The bytes to be swapped as typed data.
       * @param dst The array of bytes in swapped byte order.
       * @returns Pointer to the array of bytes in swapped byte order.
       */
      OpenGPS::UnsignedBytePtr ByteSwap(const short* const value, OpenGPS::UnsignedBytePtr dst) const;

      /*!
       * Swaps endianness.
       *
       * @param src The array of bytes to be swapped.
       * @param value The swapped bytes as typed data.
       */
      void ByteSwap(const OpenGPS::UnsignedBytePtr src, short* const value) const;

      /*!
       * Swaps endianness.
       *
       * It is assumed that dst is 16 bit long. So data is truncated if the size of the
       * typed data is greater than 16 bit.
       *
       * @param value The bytes to be swapped as typed data.
       * @param dst The array of bytes in swapped byte order.
       * @returns Pointer to the array of bytes in swapped byte order.
       */
      OpenGPS::UnsignedBytePtr ByteSwap16(const short* const value, OpenGPS::UnsignedBytePtr dst) const;

      /*!
       * Swaps endianness.
       *
       * It is assumed that src is 16 bit long.
       *
       * @param src The array of bytes to be swapped.
       * @param value The swapped bytes as typed data.
       */
      void ByteSwap16(const OpenGPS::UnsignedBytePtr src, short* const value) const;

      /*!
       * Swaps endianness.
       *
       * @param value The bytes to be swapped as typed data.
       * @param dst The array of bytes in swapped byte order.
       * @returns Pointer to the array of bytes in swapped byte order.
       */
      OpenGPS::UnsignedBytePtr ByteSwap(const int* const value, OpenGPS::UnsignedBytePtr dst) const;

      /*!
       * Swaps endianness.
       *
       * @param src The array of bytes to be swapped.
       * @param value The swapped bytes as typed data.
       */
      void ByteSwap(const OpenGPS::UnsignedBytePtr src, int* const value) const;

      /*!
       * Swaps endianness.
       *
       * It is assumed that dst is 32 bit long. So data is truncated if the size of the
       * typed data is greater than 32 bit.
       *
       * @param value The bytes to be swapped as typed data.
       * @param dst The array of bytes in swapped byte order.
       * @returns Pointer to the array of bytes in swapped byte order.
       */
      OpenGPS::UnsignedBytePtr ByteSwap32(const int* const value, OpenGPS::UnsignedBytePtr dst) const;

      /*!
       * Swaps endianness.
       *
       * It is assumed that src is 32 bit long.
       *
       * @param src The array of bytes to be swapped.
       * @param value The swapped bytes as typed data.
       */
      void ByteSwap32(const OpenGPS::UnsignedBytePtr src, int* const value) const;

      /*!
       * Swaps endianness.
       *
       * @param value The bytes to be swapped as typed data.
       * @param dst The array of bytes in swapped byte order.
       * @returns Pointer to the array of bytes in swapped byte order.
       */
      OpenGPS::UnsignedBytePtr ByteSwap(const float* const value, OpenGPS::UnsignedBytePtr dst) const;

      /*!
       * Swaps endianness.
       *
       * @param src The array of bytes to be swapped.
       * @param value The swapped bytes as typed data.
       */
      void ByteSwap(const OpenGPS::UnsignedBytePtr src, float* const value) const;

      /*!
       * Swaps endianness.
       *
       * It is assumed that dst is 32 bit long. So data is truncated if the size of the
       * typed data is greater than 32 bit.
       *
       * @param value The bytes to be swapped as typed data.
       * @param dst The array of bytes in swapped byte order.
       * @returns Pointer to the array of bytes in swapped byte order.
       */
      OpenGPS::UnsignedBytePtr ByteSwap32(const float* const value, OpenGPS::UnsignedBytePtr dst) const;

      /*!
       * Swaps endianness.
       *
       * It is assumed that src is 32 bit long.
       *
       * @param src The array of bytes to be swapped.
       * @param value The swapped bytes as typed data.
       */
      void ByteSwap32(const OpenGPS::UnsignedBytePtr src, float* const value) const;

      /*!
       * Swaps endianness.
       *
       * @param value The bytes to be swapped as typed data.
       * @param dst The array of bytes in swapped byte order.
       * @returns Pointer to the array of bytes in swapped byte order.
       */
      OpenGPS::UnsignedBytePtr ByteSwap(const double* const value, OpenGPS::UnsignedBytePtr dst) const;

      /*!
       * Swaps endianness.
       *
       * @param src The array of bytes to be swapped.
       * @param value The swapped bytes as typed data.
       */
      void ByteSwap(const OpenGPS::UnsignedBytePtr src, double* const value) const;

      /*!
       * Swaps endianness.
       *
       * It is assumed that dst is 64 bit long. So data is truncated if the size of the
       * typed data is greater than 64 bit.
       *
       * @param value The bytes to be swapped as typed data.
       * @param dst The array of bytes in swapped byte order.
       * @returns Pointer to the array of bytes in swapped byte order.
       */
      OpenGPS::UnsignedBytePtr ByteSwap64(const double* const value, OpenGPS::UnsignedBytePtr dst) const;

      /*!
       * Swaps endianness.
       *
       * It is assumed that src is 64 bit long.
       *
       * @param src The array of bytes to be swapped.
       * @param value The swapped bytes as typed data.
       */
      void ByteSwap64(const OpenGPS::UnsignedBytePtr src, double* const value) const;

   protected:
      /*! Creates a new instance. */
      Environment();

      /*! Destroys this instance. */
      virtual ~Environment();

      /*! Creates the appropriate instance for the current system. */
      static Environment* CreateInstance();

   private:
      /*! Pointer to the environment. */
      static Environment* m_Instance;
   };
}

#endif /* _OPENGPS_ENVIRONMENT_HXX */

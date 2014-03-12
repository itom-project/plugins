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

/*! @addtogroup Cpp
   @{ */

/*! @defgroup C Ansi-C Interface for X3P library
*/

/*! \addtogroup C
 *  Ansi-C Interface for X3P library
 *  @{
 */


/*! @file
 * Enhancing C++ part of the C interface of the abstract data type of the
 * ISO 5436-2 X3P file format. Since the underlying ISO 5436-2 XML document
 * can be created using a C++ interface only, the C and C++ part of the
 * abstract data type coded mainly in C are split up into two files. @see iso5436_2.h
 *
 */




#ifndef _OPENGPS_CXX_ISO5436_2_HANDLE_HXX
#define _OPENGPS_CXX_ISO5436_2_HANDLE_HXX

#ifndef _OPENGPS_CXX_OPENGPS_HXX
#  include <opengps/cxx/opengps.hxx>
#endif

namespace OpenGPS
{
   namespace Schemas
   {
      namespace ISO5436_2
      {
         class ISO5436_2Type;
         class Record1Type;
         class Record2Type;
         class MatrixDimensionType;
      }
   }
}

/*!
 * Creates a new ISO5436-2 XML X3P file.
 *
 * The Record3 object defined in the ISO5436_2 XML specification will be created automatically.
 *
 * @remarks You must release the returned handle object with ::gps_CloseISO5436_2 when done with it.
 *
 * @param file Full path to the ISO5436-2 XML X3P to be created.
 * @param temp Specifies the new absolute path to the directory where unpacked X3P data gets stored temporarily. If set to NULL the default directory for  temporary files specified by your system is used.
 * @param record1 The Record1 object defined in the ISO5436_2 XML specification. The given object instance must be valid.
 * @param record2 The Record2 object defined in the ISO5436_2 XML specification. This is optional, so the parameter can be NULL. But if set, it must point to a valid instance.
 * @param matrixDimension Specifies the topology for which point measurement data will be processed.
 * @param useBinaryData Defines whether point measurement data will be directly stored into the xml document as tag elements or if it is separately stored in a binary file within the X3P container.
 * @returns Returns the file handle or NULL on failure.
 *
 */
_OPENGPS_EXPORT OGPS_ISO5436_2Handle ogps_CreateMatrixISO5436_2(
   const OGPS_Character* const file,
   const OGPS_Character* const temp,
   const OpenGPS::Schemas::ISO5436_2::Record1Type& record1,
   const OpenGPS::Schemas::ISO5436_2::Record2Type* record2,
   const OpenGPS::Schemas::ISO5436_2::MatrixDimensionType& matrixDimension,
   const OGPS_Boolean useBinaryData = TRUE);

/*!
 * Creates a new ISO5436-2 XML X3P file.
 *
 * The Record3 object defined in the ISO5436_2 XML specification will be created automatically.
 *
 * @remarks You must release the returned handle object with ::gps_CloseISO5436_2 when done with it.
 *
 * @param file Full path to the ISO5436-2 XML X3P to be created.
 * @param temp Specifies the new absolute path to the directory where unpacked X3P data gets stored temporarily. If set to NULL the default directory for  temporary files specified by your system is used.
 * @param record1 The Record1 object defined in the ISO5436_2 XML specification. The given object instance must be valid.
 * @param record2 The Record2 object defined in the ISO5436_2 XML specification. This is optional, so the parameter can be NULL. But if set, it must point to a valid instance.
 * @param listDimension Specifies the size of point measurement data that will be processed.
 * @param useBinaryData Defines whether point measurement data will be directly stored into the xml document as tag elements or if it is separately stored in a binary file within the X3P container.
 * @returns Returns the file handle or NULL on failure.
 */
_OPENGPS_EXPORT OGPS_ISO5436_2Handle ogps_CreateListISO5436_2(
   const OGPS_Character* const file,
   const OGPS_Character* const temp,
   const OpenGPS::Schemas::ISO5436_2::Record1Type& record1,
   const OpenGPS::Schemas::ISO5436_2::Record2Type* record2,
   const OGPS_ULong listDimension,
   const OGPS_Boolean useBinaryData = TRUE);

/*!
 * Provides access to the ISO5436_2 XML document.
 *
 * @param handle Handle object to operate on.
 * @returns Returns ISO5436_2 XML document handle or NULL on failure.
  */
_OPENGPS_EXPORT OpenGPS::Schemas::ISO5436_2::ISO5436_2Type* const ogps_GetDocument(const OGPS_ISO5436_2Handle handle);

#endif /* _OPENGPS_CXX_ISO5436_2_HANDLE_HXX */
/*! @} */

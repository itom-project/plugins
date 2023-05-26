/***************************************************************************
 *   Copyright by Johannes Herwig (NanoFocus AG) 2007                      *
 *   Copyright by Georg Wiora (NanoFocus AG) 2007-2008                          *
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

/*! \addtogroup C
 *  @{
 */

/*! @file
 * The abstract data type of the ISO 5436-2 X3P file specification. It serves as the main
 * interface for communicating with this software library. Methods for opening and creating
 * X3P files, altering or parsing the contained ISO 5436-2 XML document and direct access to the point
 * data are provided. Point data can be accessed either as raw data or geometrically transformed based
 * on the axes descriptions. Also it can be chosen between an iterator interface and indexing techniques.
 * This abstract data type is split into two files because some methods involve C++ types which
 * have no representation in the C interface. @see cxx/iso5436_2_c.hxx
 */

#ifndef _OPENGPS_ISO5436_2_H
#define _OPENGPS_ISO5436_2_H

#ifndef _OPENGPS_H
#  include <opengps/opengps.h>
#endif

#ifndef _OPENGPS_POINT_VECTOR_H
#  include <opengps/point_vector.h>
#endif

#ifndef _OPENGPS_POINT_ITERATOR_H
#  include <opengps/point_iterator.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

   /*!
    * Represents the ISO5436-2 XML X3P file format container.
    *
    * Provides access to the content of an already existing or newly created X3P file container.
    * You can obtain an instance of this handle by calling ::ogps_OpenISO5436_2, ::ogps_CreateMatrixISO5436_2 or ::ogps_CreateListISO5436_2.
    *
    * @remarks An instance of ::OGPS_DataPointPtr cannot be created of its own. Instead refer to ::ogps_OpenISO5436_2, ::ogps_CreateMatrixISO5436_2 or ::ogps_CreateListISO5436_2.
    *
    * The corresponding C++ implementation is given by OpenGPS::ISO5436_2.
    */
   typedef struct _OGPS_ISO5436_2_HANDLE * OGPS_ISO5436_2Handle;

   /*!
    * Opens an existing ISO5436-2 XML X3P file.
    *
    * @remarks You must free the returned handle by calling ::ogps_CloseISO5436_2 when done with it.
    *
    * @see ::ogps_CloseISO5436_2
    *
    * @param file Full path to the ISO5436-2 XML X3P to open.
    * @param temp Optionally specifies the new absolute path to the directory where unpacked X3P data gets stored temporarily. If this parameter is set to NULL the default directory for temporary files will be used as specified by your system.
    * @param readOnly If set to TRUE subsequend operations on the returned handle object assume that you will access any obtained data as read-only and won't make any changes. This may speed up some operations. If unsure set this parameter to FALSE.
    * @returns On success returns the handle object to the opened file, otherwise a NULL pointer is returned. You may get further information about the failure by calling ::ogps_GetErrorMessage hereafter.
    */
   _OPENGPS_EXPORT OGPS_ISO5436_2Handle ogps_OpenISO5436_2(
      const OGPS_Character* const file,
      const OGPS_Character* const temp = NULL,
      const OGPS_Boolean readOnly = FALSE);

   /*!
    * Writes any changes back to the X3P file.
    *
    * Call this function before ::ogps_CloseISO5436_2 if you want to store the changes you have made.
    *
    * @see ::ogps_CreateMatrixISO5436_2, ::ogps_CreateListISO5436_2, ::ogps_OpenISO5436_2, ::ogps_CloseISO5436_2
    *
    * @param handle Operate on this handle object.
    * @param compressionLevel Optionally specifies the compression level used when writing the X3P file which is nothing else than a simple zip file container. The default value for this parameter is (-1) which enables standard compression level as a good trade-off between processing time and compression ratio. Values between 0 and 9 are possible. A value of 0 means "no compression" and a value of 9 enables the highest level compression rate at the cost of highest computation time.
    * @returns Returns TRUE on success and FALSE if anything went wrong. When FALSE no changes will be saved. You may get further information about the failure by calling ::ogps_GetErrorMessage hereafter.
    */
   _OPENGPS_EXPORT void ogps_WriteISO5436_2(const OGPS_ISO5436_2Handle handle, const int compressionLevel = -1);

   /*!
    * Closes an ::OGPS_ISO5436_2Handle file handle and releases its resources.
    *
    * @remarks This does not save any changes you made! You must call ::ogps_WriteISO5436_2 before if your changes should be saved.
    *
    * @see ::ogps_CreateMatrixISO5436_2, ::ogps_CreateListISO5436_2, ::ogps_OpenISO5436_2, ::ogps_WriteISO5436_2
    *
    * @param handle Handle object to close.
    */
   _OPENGPS_EXPORT void ogps_CloseISO5436_2(OGPS_ISO5436_2Handle* handle);

   /*!
    * Add vendorspecific file content to the X3P archive.
    *
    * Call this before ::ogps_WriteISO5436_2 and the content of a file will be added to the X3P file when written.
    * This can be called multiple times to add more than one file of your choice. These files must exist at the
    * time when ::ogps_WriteISO5436_2 is being executed. The file will be added to the root of the archive with
    * the given file name from the full path specified.
    *
    * @see ::ogps_GetVendorSpecific
    *
    * @param handle Operate on this handle object.
    * @param vendorURI Your very own vendor specifier in a URI conformant format.
    * @param filePath The absolute path to the file to be added to the document container.
    */
   _OPENGPS_EXPORT void ogps_AppendVendorSpecific(const OGPS_ISO5436_2Handle handle, const OGPS_Character* vendorURI, const OGPS_Character* filePath);

   /*!
    * Extracts vendorspecific data from the current archive to a given file location.
    *
    * If the current X3P archive contains vendorspecific data registered for a vendorURI under the given filename in
    * the root directory of the archive, the compressed file will be extracted to the given location.
    *
    * @see ::ogps_AppendVendorSpecific
    *
    * @param handle Operate on this handle object.
    * @param vendorURI Your very own vendor specifier in a URI conformant format.
    * @param fileName The name of the file to be expected in the root of the archive which is to be decompressed.
    * @param targetPath The file in the archive will get extracted here.
    * @retval FALSE if there is no file registered for the given vendorURI within the archive, TRUE if the file has been found and extracted.
    */
   _OPENGPS_EXPORT OGPS_Boolean ogps_GetVendorSpecific(const OGPS_ISO5436_2Handle handle, const OGPS_Character* vendorURI, const OGPS_Character* fileName, const OGPS_Character* targetPath);

   /*!
    * Creates an iterator to access point data contained in an ISO5436-2 X3P file.
    *
    * Iterates the point data in forward direction.
    *
    * @remarks You must free the resources occupied by the returned iterator handle by calling ::ogps_FreePointIterator.
    *
    * @param handle Operate on this handle object.
    * @returns Returns an iterator handle on success otherwise NULL.
    */
   _OPENGPS_EXPORT OGPS_PointIteratorPtr ogps_CreateNextPointIterator(const OGPS_ISO5436_2Handle handle);

   /*!
    * Creates an iterator to access point data contained in an ISO5436-2 X3P file.
    *
    * Iterates the point data in backward direction.
    *
    * @remarks You must free the resources occupied by the returned iterator handle by calling ::ogps_FreePointIterator.
    *
    * @param handle Operate on this handle object.
    * @returns Returns an iterator handle on success otherwise NULL.
    */
   _OPENGPS_EXPORT OGPS_PointIteratorPtr ogps_CreatePrevPointIterator(const OGPS_ISO5436_2Handle handle);

   /*!
    * Sets the value of a three-dimensional data point vector at a given surface position.
    *
    * Manipulates the point vector data stored in an ISO5436-2 X3P file directly.
    *
    * @remarks The data types of the values stored in the given vector parameter must correspond
    * with the data types specified in the ISO5436-2 XML document within the axes definition area.
    * Also ::ogps_SetMatrixPoint is the valid access method only if point vectors are stored in
    * matrix format with encoded topology information. If this is not the case point vectors are
    * stored in list format. Then you must use ::ogps_SetListPoint instead. What is the correct
    * format for access is revealed by the ISO5436-2 XML document.
    * Access to the ISO5436-2 XML document which is part of an X3P file container can be
    * obtained through ::ogps_GetDocument.
    *
    * @see ::ogps_SetListPoint
    *
    * On failure you may get further information by calling ::ogps_GetErrorMessage hereafter.
    *
    * @param handle Operate on this handle object.
    * @param u The u-direction of the surface position.
    * @param v The v-direction of the surface position.
    * @param w The w-direction of the surface position.
    * @param vector Set this point value at the given u,v,w position. If this parameter is set to
    * NULL, this indicates there is no measurement data available for this position. This is due
    * to the topology encoding properties of the matrix format storage.
    */
   _OPENGPS_EXPORT void ogps_SetMatrixPoint(
      const OGPS_ISO5436_2Handle handle,
      const OGPS_ULong u,
      const OGPS_ULong v,
      const OGPS_ULong w,
      const OGPS_PointVectorPtr vector);

   /*!
    * Gets the raw value of a data point vector at a given surface position.
    *
    *
    * @remarks ::ogps_GetMatrixPoint is the valid access method only if point vectors are stored in
    * matrix format with encoded topology information. If this is not the case point vectors are
    * stored in list format. Then you must use ::ogps_GetListPoint instead. What is the correct
    * format for access is revealed by the ISO5436-2 XML document.
    * Access to the ISO5436-2 XML document which is part of an X3P file container can be
    * obtained through ::ogps_GetDocument.
    *
    * @see ::ogps_GetMatrixCoord, ::ogps_GetListPoint
    *
    * On failure you may get further information by calling ::ogps_GetErrorMessage hereafter.
    *
    * @param handle Operate on this handle object.
    * @param u The u-direction of the surface position.
    * @param v The v-direction of the surface position.
    * @param w The w-direction of the surface position.
    * @param vector Returns the raw point value at the given u,v,w position.
    */
   _OPENGPS_EXPORT void ogps_GetMatrixPoint(
      const OGPS_ISO5436_2Handle handle,
      const OGPS_ULong u,
      const OGPS_ULong v,
      const OGPS_ULong w,
      OGPS_PointVectorPtr const vector);

   /*!
    * Sets the value of a three-dimensional data point vector at a given index position.
    *
    * Manipulates the point vector data stored in an ISO5436-2 X3P file directly.
    *
    * @remarks The data types of the values stored in the given vector parameter must correspond
    * with the data types specified in the ISO5436-2 XML document within the axes definition area.
    * Also ::ogps_SetListPoint is the valid access method only if point vectors are stored in
    * list format. If this is not the case and point vectors are stored in matrix format with
    * encoded topology information then you must use ::ogps_SetMatrixPoint instead. What is the correct
    * format for access is revealed by the ISO5436-2 XML document.
    * Access to the ISO5436-2 XML document which is part of an X3P file container can be
    * obtained through ::ogps_GetDocument.
    *
    * @see ::ogps_SetMatrixPoint
    *
    * On failure you may get further information by calling ::ogps_GetErrorMessage hereafter.
    *
    * @param handle Operate on this handle object.
    * @param index The index position of the point vector to manipulate.
    * @param vector Set this point value at the given index position.
    */
   _OPENGPS_EXPORT void ogps_SetListPoint(
      const OGPS_ISO5436_2Handle handle,
      const OGPS_ULong index,
      const OGPS_PointVectorPtr vector);

   /*!
    * Gets the raw value of a data point vector at a given index position.
    *
    *
    * @remarks ::ogps_GetListPoint is the valid access method only if point vectors are stored in
    * list format. If this is not the case and point vectors are stored in matrix format with
    * encoded topology information, then you must use ::ogps_GetMatrixPoint instead. What is the correct
    * format for access is revealed by the ISO5436-2 XML document.
    * Access to the ISO5436-2 XML document which is part of an X3P file container can be
    * obtained through ::ogps_GetDocument.
    *
    * @see ::ogps_GetListCoord, ::ogps_GetMatrixPoint
    *
    * On failure you may get further information by calling ::ogps_GetErrorMessage hereafter.
    *
    * @param handle Operate on this handle object.
    * @param index The index of the surface position.
    * @param vector Returns the raw point value at the given position.
    */
   _OPENGPS_EXPORT void ogps_GetListPoint(
      const OGPS_ISO5436_2Handle handle,
      const OGPS_ULong index,
      OGPS_PointVectorPtr const vector);

   /*!
    * Gets the fully transformed value of a data point vector at a given surface position.
    *
    * Other than with ::ogps_GetMatrixPoint this function also applies the axes transformation
    * specified in the axes definition area of the ISO 5436-2 XML document.
    *
    * @remarks ::ogps_GetMatrixCoord is the valid access method only if point vectors are stored in
    * matrix format with encoded topology information. If this is not the case point vectors are
    * stored in list format. Then you must use ::ogps_GetListCoord instead. What is the correct
    * format for access is revealed by the ISO5436-2 XML document.
    * Access to the ISO5436-2 XML document which is part of an X3P file container can be
    * obtained through ::ogps_GetDocument.
    *
    * @see ::ogps_GetMatrixPoint, ::ogps_GetListCoord
    *
    * On failure you may get further information by calling ::ogps_GetErrorMessage hereafter.
    *
    * @param handle Operate on this handle object.
    * @param u The u-direction of the surface position.
    * @param v The v-direction of the surface position.
    * @param w The w-direction of the surface position.
    * @param x Returns the fully transformed x component of the point value at the given u,v,w position. If this parameter is set to NULL, the x axis component will be safely ignored.
    * @param y Returns the fully transformed y component of the point value at the given u,v,w position. If this parameter is set to NULL, the y axis component will be safely ignored.
    * @param z Returns the fully transformed z component of the point value at the given u,v,w position. If this parameter is set to NULL, the z axis component will be safely ignored.
    */
   _OPENGPS_EXPORT void ogps_GetMatrixCoord(
      const OGPS_ISO5436_2Handle handle,
      const OGPS_ULong u,
      const OGPS_ULong v,
      const OGPS_ULong w,
      OGPS_Double* const x,
      OGPS_Double* const y,
      OGPS_Double* const z);

   /*!
    * Asks if there is point vector data stored at the given matrix position.
    *
    * Since the matrix storage format encodes topology information there may not exist valid point
    * vector data for every u,v,w position because there was no measurement data available.
    *
    * @see ::ogps_GetMatrixPoint, ::ogps_GetMatrixCoord, ::ogps_SetMatrixPoint
    *
    * @param handle Operate on this handle object.
    * @param u The u-direction of the surface position.
    * @param v The v-direction of the surface position.
    * @param w The w-direction of the surface position.
    * @returns Returns TRUE if the vector point data at the given position is valid, otherwise return FALSE to indicate there is no measurement data available at this particular position.
    */
   _OPENGPS_EXPORT OGPS_Boolean ogps_IsMatrixCoordValid(
      const OGPS_ISO5436_2Handle handle,
      const OGPS_ULong u,
      const OGPS_ULong v,
      const OGPS_ULong w);

   /*!
    * Gets the fully transformed value of a data point vector at a given index position.
    *
    * Other than with ::ogps_GetListPoint this function also applies the axes transformation
    * specified in the axes definition area of the ISO 5436-2 XML document.
    *
    * @remarks ::ogps_GetListCoord is the valid access method only if point vectors are stored in
    * list format. If this is not the case and point vectors are stored in matrix format with
    * encoded topology information, then you must use ::ogps_GetMatrixCoord instead. What is the correct
    * format for access is revealed by the ISO5436-2 XML document.
    * Access to the ISO5436-2 XML document which is part of an X3P file container can be
    * obtained through ::ogps_GetDocument.
    *
    * @see ::ogps_GetListPoint, ::ogps_GetMatrixCoord
    *
    * On failure you may get further information by calling ::ogps_GetErrorMessage hereafter.
    *
    * @param handle Operate on this handle object.
    * @param index The index of the surface position.
    * @param x Returns the fully transformed x component of the point value at the given index position. If this parameter is set to NULL, the x axis component will be safely ignored.
    * @param y Returns the fully transformed y component of the point value at the given index position. If this parameter is set to NULL, the y axis component will be safely ignored.
    * @param z Returns the fully transformed z component of the point value at the given index position. If this parameter is set to NULL, the z axis component will be safely ignored.
    */
   _OPENGPS_EXPORT void ogps_GetListCoord(
      const OGPS_ISO5436_2Handle handle,
      const OGPS_ULong index,
      OGPS_Double* const x,
      OGPS_Double* const y,
      OGPS_Double* const z);

   /*!
   * Gets information on the structure with which the point
   * measurement data is stored.
   * @param handle Operate on this handle object.
   * @returns Returns TRUE if the underlying document structure represents
   * the matrix topology, otherwise FALSE, which corresponds to unordered list structure.
   * Also see remarks.
   * @remarks Important: After execution check with ogps_HasError() whether the request was
   * processed correctly, otherwise futurue behavior of your program is undefined!
   */
   _OPENGPS_EXPORT OGPS_Boolean ogps_IsMatrix(const OGPS_ISO5436_2Handle handle);


   /*!
   * Gets information on the matrix dimensions of the current data structure
   * The dimensions are written to the arguments size_u, size_v, size_w.
   * Be sure that the current handle points to a matrix structure.
   * @param handle Operate on this handle object.
   * @param size_u Pointer to the return value of the matrix dimension in u-direction
   * @param size_v Pointer to the return value of the matrix dimension in v-direction
   * @param size_w Pointer to the return value of the matrix dimension in w-direction
   * Also see remarks.
   * @remarks Important: After execution check with ogps_HasError() whether the request was
   * processed correctly, otherwise future behavior of your program is undefined!
   */
   _OPENGPS_EXPORT void ogps_GetMatrixDimensions(const OGPS_ISO5436_2Handle handle,
                                                         OGPS_ULong * const size_u,
                                                         OGPS_ULong * const size_v,
                                                         OGPS_ULong * const size_w);

   /*!
   * Gets information on the list dimension of the current data structure.
   * Be sure that the current handle points to a list structure.
   * @param handle Operate on this handle object.
   * @returns The dimension of the current list structure.
   * @remarks Important: After execution check with ogps_HasError() whether the request was
   * processed correctly, otherwise future behavior of your program is undefined!
   */
   _OPENGPS_EXPORT OGPS_ULong ogps_GetListDimension(const OGPS_ISO5436_2Handle handle);


#ifdef __cplusplus
}
#endif

#endif /* _OPENGPS_ISO5436_2_H */
/*! @} */

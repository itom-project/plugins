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

/*! @file
 * Represents the ISO5436-2 XML X3P file format container. It serves as the main
 * interface for communication with this software library. Interfaces for opening and creating
 * X3P files, altering or parsing the contained ISO 5436-2 XML documents and direct access to the point
 * data are provided. Point data can be accessed as raw data or geometrically transformed based
 * on the axes descriptions. Also it can be chosen between an iterator interface and indexing techniques.
 */

#ifndef _OPENGPS_CXX_ISO5436_2_HXX
#define _OPENGPS_CXX_ISO5436_2_HXX

#ifndef _OPENGPS_CXX_OPENGPS_HXX
#  include <opengps/cxx/opengps.hxx>
#endif

#ifndef _OPENGPS_CXX_EXCEPTIONS_HXX
#  include <opengps/cxx/exceptions.hxx>
#endif

#include <memory>


namespace OpenGPS
{
   namespace Schemas
   {
      namespace ISO5436_2
      {
         class ISO5436_2Type;
      }
   }

   class String;
   class PointBuffer;
   class PointVectorBase;
   class PointIterator;
   class VectorBuffer;
   class VectorBufferBuilder;
   class PointVectorParserBuilder;

   /*! std::auto_ptr for usage with OpenGPS::PointBuffer type. */
   typedef std::auto_ptr<PointBuffer> PointBufferAutoPtr;

   /*! std::auto_ptr for usage with OpenGPS::PointVectorBase type. */
   typedef std::auto_ptr<PointVectorBase> PointVectorAutoPtr;

   /*! std::auto_ptr for usage with OpenGPS::PointIterator type. */
   typedef std::auto_ptr<PointIterator> PointIteratorAutoPtr;

   /*! std::auto_ptr for usage with OpenGPS::VectorBuffer type. */
   typedef std::auto_ptr<VectorBuffer> VectorBufferAutoPtr;

   /*! std::auto_ptr for usage with OpenGPS::VectorBufferBuilder type. */
   typedef std::auto_ptr<VectorBufferBuilder> VectorBufferBuilderAutoPtr;

   /*! std::auto_ptr for usage with OpenGPS::PointVectorParserBuilder type. */
   typedef std::auto_ptr<PointVectorParserBuilder> PointVectorParserBuilderAutoPtr;
}

namespace OpenGPS
{
   namespace Schemas
   {
      namespace ISO5436_2
      {
         class Record1Type;
         class Record2Type;
         class MatrixDimensionType;
         class ISO5436_2Type;
      }
   }

   class PointVector;

   /*!
    * Represents the ISO5436-2 XML X3P file format container.
    *
    * Provides access to the content of an already existing or newly created X3P file container.
    */
   class _OPENGPS_EXPORT ISO5436_2
   {
   protected:
      /*!
      * Creates a new instance.
      *
      * @param instance Any method calls on the newly created object will be routed to the given instance.
      */
      ISO5436_2(ISO5436_2* instance = NULL);

   public:
      /*!
      * Creates a new instance.
      *
      * @remarks Use either ISO5436_2::Open or ISO5436_2::Create to work on the ISO5436-2 XML X3P file container.
      *
      * @param file Full path to the ISO5436-2 XML X3P to operate on. This file does not need to exist.
      * @param temp Specifies a new absolute path to the directory where unpacked X3P data gets stored temporarily.
      */
      ISO5436_2(
         const OpenGPS::String& file,
         const OpenGPS::String& temp);

      /*!
      * Creates a new instance.
      *
      * @remarks Use either ISO5436_2::Open or ISO5436_2::Create to work on the ISO5436-2 XML X3P file container.
      *
      * @param file Full path to the ISO5436-2 XML X3P to operate on. This file does not need to exist.
      */
      ISO5436_2(
         const OpenGPS::String& file);

      /*!
      * Opens an existing ISO5436-2 XML X3P file.
      *
      * @see ISO5436_2::Close
      *
      * Specific implementations may raise an exception.
      *
      * @param readOnly If set to TRUE subsequend operations on the current object assume that you will access any obtained data as read-only and won't make any changes. This may speed up some operations. If unsure set this parameter to FALSE.
      */
      virtual void Open(const OGPS_Boolean readOnly = FALSE);

      /*!
       * Creates a new ISO5436-2 XML X3P file.
       *
       * @remarks The Record3 object defined in the ISO5436_2 XML specification will be created automatically.
       *
       * Specific implementations may raise an exception.
       *
       * @param record1 The Record1 object defined in the ISO5436_2 XML specification. The given object instance must be valid.
       * @param record2 The Record2 object defined in the ISO5436_2 XML specification. This is optional, so the parameter can be NULL. But if set, it must point to a valid instance.
       * @param matrixDimension Specifies the topology for which point measurement data will be processed.
       * @param useBinaryData Defines whether point measurement data will be directly stored into the xml document as tag elements or if it is separately stored in a binary file within the X3P container.
       */
      virtual void Create(
         const Schemas::ISO5436_2::Record1Type& record1,
         const Schemas::ISO5436_2::Record2Type* record2,
         const Schemas::ISO5436_2::MatrixDimensionType& matrixDimension,
         const OGPS_Boolean useBinaryData = TRUE);

      /*!
       * Creates a new ISO5436-2 XML X3P file.
       *
       * @remarks The Record3 object defined in the ISO5436_2 XML specification will be created automatically.
       *
       * Specific implementations may raise an exception.
       *
       * @param record1 The Record1 object defined in the ISO5436_2 XML specification. The given object instance must be valid.
       * @param record2 The Record2 object defined in the ISO5436_2 XML specification. This is optional, so the parameter can be NULL. But if set, it must point to a valid instance.
       * @param listDimension Specifies the size of point measurement data that will be processed.
       * @param useBinaryData Defines whether point measurement data will be directly stored into the xml document as tag elements or if it is separately stored in a binary file within the X3P container.
       */
      virtual void Create(
         const Schemas::ISO5436_2::Record1Type& record1,
         const Schemas::ISO5436_2::Record2Type* record2,
         const OGPS_ULong listDimension,
         const OGPS_Boolean useBinaryData = TRUE);

      /*! Destructs this object. */
      virtual ~ISO5436_2();

      /*!
       * Creates an iterator to access point data contained in an ISO5436-2 X3P file.
       *
       * Iterates the point data in forward direction.
       *
       * @returns Returns an iterator handle on success.
       */
      virtual PointIteratorAutoPtr CreateNextPointIterator();

      /*!
       * Creates an iterator to access point data contained in an ISO5436-2 X3P file.
       *
       * Iterates the point data in backward direction.
       *
       * @returns Returns an iterator handle on success.
       */
      virtual PointIteratorAutoPtr CreatePrevPointIterator();

      /*!
      * Sets the value of a three-dimensional data point vector at a given surface position.
      *
      * Manipulates the point vector data stored in an ISO5436-2 X3P file directly.
      *
      * A specific implementation may throw an OpenGPS::Exception if this operation
      * is not permitted due to the current state of the object instance.
      *
      * @remarks The data types of the values stored in the given vector parameter must correspond
      * with the data types specified in the ISO5436-2 XML document within the axes definition area.
      * Also ISO5436_2::SetMatrixPoint is the valid access method only if point vectors are stored in
      * matrix format with encoded topology information. If this is not the case point vectors are
      * stored in list format. Then you must use ISO5436_2::SetListPoint instead. What is the correct
      * format for access is revealed by the ISO5436-2 XML document.
      * Access to the ISO5436-2 XML document which is part of an X3P file container can be
      * obtained through ISO5436_2::GetDocument.
      *
      * @see ISO5436_2::SetListPoint
      *
      * @param u The u-direction of the surface position.
      * @param v The v-direction of the surface position.
      * @param w The w-direction of the surface position.
      * @param vector Set this point value at the given u,v,w position. If this parameter is set to
      * NULL, this indicates there is no measurement data available for this position. This is due
      * to the topology encoding properties of the matrix format storage.
      */
      virtual void SetMatrixPoint(
         const OGPS_ULong u,
         const OGPS_ULong v,
         const OGPS_ULong w,
         const PointVector* const vector);

      /*!
       * Gets the raw value of a data point vector at a given surface position.
       *
       * A specific implementation may throw an OpenGPS::Exception if this operation
       * is not permitted due to the current state of the object instance.
       *
       * @remarks ISO5436_2::GetMatrixPoint is the valid access method only if point vectors are stored in
       * matrix format with encoded topology information. If this is not the case point vectors are
       * stored in list format. Then you must use ISO5436_2::GetListPoint instead. What is the correct
       * format for access is revealed by the ISO5436-2 XML document.
       * Access to the ISO5436-2 XML document which is part of an X3P file container can be
       * obtained through ISO5436_2::GetDocument.
       *
       * @see ISO5436_2::GetMatrixCoord, ISO5436_2::GetListPoint
       *
       * @param u The u-direction of the surface position.
       * @param v The v-direction of the surface position.
       * @param w The w-direction of the surface position.
       * @param vector Returns the raw point value at the given u,v,w position.
       */
      virtual void GetMatrixPoint(
         const OGPS_ULong u,
         const OGPS_ULong v,
         const OGPS_ULong w,
         PointVector& vector);

      /*!
       * Sets the value of a three-dimensional data point vector at a given index position.
       *
       * Manipulates the point vector data stored in an ISO5436-2 X3P file directly.
       *
       * A specific implementation may throw an OpenGPS::Exception if this operation
       * is not permitted due to the current state of the object instance.
       *
       * @remarks The data types of the values stored in the given vector parameter must correspond
       * with the data types specified in the ISO5436-2 XML document within the axes definition area.
       * Also ISO5436_2::SetListPoint is the valid access method only if point vectors are stored in
       * list format. If this is not the case and point vectors are stored in matrix format with
       * encoded topology information then you must use ISO5436_2::SetMatrixPoint instead. What is the correct
       * format for access is revealed by the ISO5436-2 XML document.
       * Access to the ISO5436-2 XML document which is part of an X3P file container can be
       * obtained through ISO5436_2::GetDocument.
       *
       * @see ISO5436_2::SetMatrixPoint
       *
       * @param index The index position of the point vector to manipulate.
       * @param vector Set this point value at the given index position.
       */
      virtual void SetListPoint(
         const OGPS_ULong index,
         const PointVector& vector);

      /*!
       * Gets the raw value of a data point vector at a given index position.
       *
       * A specific implementation may throw an OpenGPS::Exception if this operation
       * is not permitted due to the current state of the object instance.
       *
       * @remarks ISO5436_2::GetListPoint is the valid access method only if point vectors are stored in
       * list format. If this is not the case and point vectors are stored in matrix format with
       * encoded topology information, then you must use ISO5436_2::GetMatrixPoint instead. What is the correct
       * format for access is revealed by the ISO5436-2 XML document.
       * Access to the ISO5436-2 XML document which is part of an X3P file container can be
       * obtained through ISO5436_2::GetDocument.
       *
       * @see ISO5436_2::GetListCoord, ISO5436_2::GetMatrixPoint
       *
       * @param index The index of the surface position.
       * @param vector Returns the raw point value at the given position.
       * @return Returns TRUE on success, FALSE otherwise.
       */
      virtual void GetListPoint(
         const OGPS_ULong index,
         PointVector& vector);

      /*!
       * Gets the fully transformed value of a data point vector at a given surface position.
       *
       * Other than with ISO5436_2::GetMatrixPoint this function also applies the axes transformation
       * specified in the axes definition area of the ISO 5436-2 XML document.
       *
       * A specific implementation may throw an OpenGPS::Exception if this operation
       * is not permitted due to the current state of the object instance.
       *
       * @remarks ISO5436_2::GetMatrixCoord is the valid access method only if point vectors are stored in
       * matrix format with encoded topology information. If this is not the case point vectors are
       * stored in list format. Then you must use ISO5436_2::GetListCoord instead. What is the correct
       * format for access is revealed by the ISO5436-2 XML document.
       * Access to the ISO5436-2 XML document which is part of an X3P file container can be
       * obtained through ISO5436_2::GetDocument.
       *
       * @see ISO5436_2::GetMatrixPoint, ISO5436_2::GetListCoord
       *
       * @param u The u-direction of the surface position.
       * @param v The v-direction of the surface position.
       * @param w The w-direction of the surface position.
       * @param x Returns the fully transformed x component of the point value at the given u,v,w position. If this parameter is set to NULL, the x axis component will be safely ignored.
       * @param y Returns the fully transformed y component of the point value at the given u,v,w position. If this parameter is set to NULL, the y axis component will be safely ignored.
       * @param z Returns the fully transformed z component of the point value at the given u,v,w position. If this parameter is set to NULL, the z axis component will be safely ignored.
       * @return Returns TRUE on success, FALSE otherwise.
       */
      virtual void GetMatrixCoord(
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
       * A specific implementation may throw an OpenGPS::Exception if this operation
       * is not permitted due to the current state of the object instance.
       *
       * @see ISO5436_2::GetMatrixPoint, ISO5436_2::GetMatrixCoord, ISO5436_2::SetMatrixPoint
       *
       * @param u The u-direction of the surface position.
       * @param v The v-direction of the surface position.
       * @param w The w-direction of the surface position.
       * @returns Returns TRUE if the vector point data at the given position is valid, otherwise return FALSE to indicate there is no measurement data available at this particular position.
    */
      virtual OGPS_Boolean IsMatrixCoordValid(
         OGPS_ULong u,
         OGPS_ULong v,
         OGPS_ULong w);

      /*!
       * Gets the fully transformed value of a data point vector at a given index position.
       *
       * Other than with ::ogps_GetListPoint this function also applies the axes transformation
       * specified in the axes definition area of the ISO 5436-2 XML document.
       *
       * A specific implementation may throw an OpenGPS::Exception if this operation
       * is not permitted due to the current state of the object instance.
       *
       * @remarks ISO5436_2::GetListCoord is the valid access method only if point vectors are stored in
       * list format. If this is not the case and point vectors are stored in matrix format with
       * encoded topology information, then you must use ISO5436_2::GetMatrixCoord instead. What is the correct
       * format for access is revealed by the ISO5436-2 XML document.
       * Access to the ISO5436-2 XML document which is part of an X3P file container can be
       * obtained through ISO5436_2::GetDocument.
       *
       * @see ISO5436_2::GetListPoint, ISO5436_2::GetMatrixCoord
       *
       * @param index The index of the surface position.
       * @param x Returns the fully transformed x component of the point value at the given index position. If this parameter is set to NULL, the x axis component will be safely ignored.
       * @param y Returns the fully transformed y component of the point value at the given index position. If this parameter is set to NULL, the y axis component will be safely ignored.
       * @param z Returns the fully transformed z component of the point value at the given index position. If this parameter is set to NULL, the z axis component will be safely ignored.
       * @return Returns TRUE on success, FALSE otherwise.
       */
      virtual void GetListCoord(
         const OGPS_ULong index,
         OGPS_Double* const x,
         OGPS_Double* const y,
         OGPS_Double* const z);

      /*!
       * Gets access to the ISO5436_2 XML document.
       */
      virtual OpenGPS::Schemas::ISO5436_2::ISO5436_2Type* const GetDocument();

      /*!
       * Gets information on the structure with which the point
       * measurement data is stored.
       * @returns Returns TRUE if point vectors are stored as a
       * matrix structure preserving topology information, or FALSE
       * when point vectors are stored sequentially within a simple list.
       */
      virtual OGPS_Boolean IsMatrix() const;

      /*!
      * Gets information on the matrix dimensions of the current data structure
      * The dimensions are written to the arguments size_u, size_v, size_w.
      * Be sure that the current handle points to a matrix structure.
      * @param handle Operate on this handle object.
      * @param size_u Pointer to the return value of the matrix dimension in u-direction
      * @param size_v Pointer to the return value of the matrix dimension in v-direction
      * @param size_w Pointer to the return value of the matrix dimension in w-direction
      */
      virtual void GetMatrixDimensions(
         OGPS_ULong * const size_u,
         OGPS_ULong * const size_v,
         OGPS_ULong * const size_w) const;

      /*!
      * Gets information on the list dimensions of the current data structure.
      * Be sure that the current handle points to a list structure.
      * @param handle Operate on this handle object.
      * @returns The dimension of the current list structure.
      */
      virtual OGPS_ULong GetListDimension() const;

      /*!
       * Writes any changes back to the X3P file.
       *
       * Call this function before ISO5436_2::Close if you want to store the changes you have made.
       *
       * @see ISO5436_2::Create, ISO5436_2::Open, ISO5436_2::Close
       *
       * Specific implementations may raise an exception.
       *
       * @param compressionLevel Optionally specifies the compression level used when writing the X3P file which is nothing else than a simple zip file container. The default value for this parameter is (-1) which enables standard compression level as a good trade-off between processing time and compression ratio. Values between 0 and 9 are possible. A value of 0 means "no compression" and a value of 9 enables the highest level compression rate at the cost of highest computation time.
       */
      virtual void Write(const int compressionLevel = -1);

      /*!
       * Closes an open file handle and frees its resources.
       * An explicit call of this method is optional. It is executed implicitly when the object of the current instance gets out of scope.
       *
       * @remarks This does not save any changes you made! You must call ISO5436_2::Write before if your changes should be saved.
       *
       * @see ISO5436_2::Create, ISO5436_2::Open, ISO5436_2::Write
       */
      virtual void Close();

      /*!
       * Add vendorspecific file content to the X3P archive.
       *
       * Call this before ISO5436_2::Write and the content of a file will be added to the X3P file when written.
       * This can be called multiple times to add more than one file of your choice. These files must exist at the
       * time when ISO5436_2::Write is being executed. The file will be added to the root of the archive with
       * the given file name from the full path specified.
       *
       * @see ISO5436_2::GetVendorSpecific
       *
       * @param vendorURI Your very own vendor specifier in a URI conformant format.
       * @param filePath The absolute path to the file to be added to the document container.
       */
      virtual void AppendVendorSpecific(const OpenGPS::String& vendorURI, const OpenGPS::String& filePath);

      /*!
       * Extracts vendorspecific data from the current archive to a given file location.
       *
       * If the current X3P archive contains vendorspecific data registered for a vendorURI under the given filename in
       * the root directory of the archive, the compressed file will be extracted to the given location.
       *
       * @see ISO5436_2::AppendVendorSpecific
       *
       * @param vendorURI Your very own vendor specifier in a URI conformant format.
       * @param fileName The name of the file to be expected in the root of the archive which is to be decompressed.
       * @param targetPath The file in the archive will get extracted here.
       * @retval FALSE if there is no file registered for the given vendorURI within the archive, TRUE if the file has been found and extracted.
       */
      virtual OGPS_Boolean GetVendorSpecific(const OpenGPS::String& vendorURI, const OpenGPS::String& fileName, const OpenGPS::String& targetPath);

   private:
      /*! When TRUE: do not allow any modifications to the data points. */
      const OGPS_Boolean m_IsProtected;

      /*! Internal object instance. Either "this" or ISO5436_2::ISO5436_2Container instance. */
      ISO5436_2* m_Instance;

      /*! The copy-ctor is not implemented. This prevents its usage. */
      ISO5436_2(const ISO5436_2& src);
      /*! The assignment-operator is not implemented. This prevents its usage. */
      ISO5436_2& operator=(const ISO5436_2& src);
   };
}


#endif /* _OPENGPS_CXX_ISO5436_2_HXX */

/*! @} */

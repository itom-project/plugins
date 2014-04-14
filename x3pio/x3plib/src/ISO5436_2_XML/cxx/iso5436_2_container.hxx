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
 * Concrete implementation of the interface of an X3P container.
 */

#ifndef _OPENGPS_ISO5436_2_CONTAINER_HXX
#define _OPENGPS_ISO5436_2_CONTAINER_HXX

#ifndef _OPENGPS_CXX_ISO5436_2_HXX
#  include <opengps/cxx/iso5436_2.hxx>
#endif

#ifndef _OPENGPS_CXX_EXCEPTIONS_HXX
#  include <opengps/cxx/exceptions.hxx>
#endif

#ifndef _OPENGPS_DATA_POINT_TYPE_H
#  include <opengps/data_point_type.h>
#endif

#ifndef _OPENGPS_AUTO_PTR_TYPES_HXX
#  include "auto_ptr_types.hxx"
#endif

#ifndef _OPENGPS_POINT_VECTOR_PROXY_CONTEXT_HXX
#  include "point_vector_proxy_context.hxx"
#endif

#ifndef _OPENGPS_CXX_POINT_ITERATOR_HXX
#  include <opengps/cxx/point_iterator.hxx>
#endif

#ifndef _OPENGPS_CXX_STRING_HXX
#  include <opengps/cxx/string.hxx>
#endif

#include <opengps/cxx/iso5436_2_xsd.hxx>

#include <zip.h>

namespace OpenGPS
{
   class PointVectorParser;
   class PointVectorParserBuilder;
   class VectorBufferBuilder;
   class PointVectorReaderContext;
   class PointVectorWriterContext;

   /*! This is the main gate to this software library. It provides all manipulation
    * methods to handle X3P archive files.
    */
   class ISO5436_2Container : public ISO5436_2
   {
   public:
      /*!
       * Creates a new instance.
       * @param file The path to the X3P file that is serverd by this instance.
       * The file may not exist yet. In this case ISO5436_2Container::Create
       * methods must be used to create a new X3P file. Otherwise use
       * ISO5436_2Container::Create to get an internal handle to an already
       * existing x3P file container.
       * @param temp If not empty this specifies an alternativ directory path
       * to save temporary files. Otherwise the default directory for temporary files
       * used by the operating system is used. Since the file data archived in the
       * X3P file needs to be decompressed before using it, this may eventually take
       * relatively large space from your storage media.
       */
      ISO5436_2Container(
         const OpenGPS::String& file,
         const OpenGPS::String& temp);

      /*! Destroys this instance. This closes all open file handles and all
      changes you may have made to an X3P document get lost unless you
      previously saved them by executing ISO5436_2Container::Write. */
      virtual ~ISO5436_2Container();

      /* Implements public ISO5436_2 interface. */

      virtual void Open(const OGPS_Boolean readOnly = TRUE);

      virtual void Create(
         const Schemas::ISO5436_2::Record1Type& record1,
         const Schemas::ISO5436_2::Record2Type* record2,
         const Schemas::ISO5436_2::MatrixDimensionType& matrixDimension,
         const OGPS_Boolean useBinaryData = TRUE);

      virtual void Create(
         const Schemas::ISO5436_2::Record1Type& record1,
         const Schemas::ISO5436_2::Record2Type* record2,
         const OGPS_ULong listDimension,
         const OGPS_Boolean useBinaryData = TRUE);

      virtual PointIteratorAutoPtr CreateNextPointIterator();
      virtual PointIteratorAutoPtr CreatePrevPointIterator();

      virtual void SetMatrixPoint(
         const OGPS_ULong u,
         const OGPS_ULong v,
         const OGPS_ULong w,
         const PointVector* const vector);

      virtual void GetMatrixPoint(
         const OGPS_ULong u,
         const OGPS_ULong v,
         const OGPS_ULong w,
         PointVector& vector);

      virtual void SetListPoint(
         const OGPS_ULong index,
         const PointVector& vector);

      virtual void GetListPoint(
         const OGPS_ULong index,
         PointVector& vector);

      virtual void GetMatrixCoord(
         const OGPS_ULong u,
         const OGPS_ULong v,
         const OGPS_ULong w,
         OGPS_Double* const x,
         OGPS_Double* const y,
         OGPS_Double* const z);

      virtual OGPS_Boolean IsMatrixCoordValid(
         OGPS_ULong u,
         OGPS_ULong v,
         OGPS_ULong w);

      virtual void GetListCoord(
         const OGPS_ULong index,
         OGPS_Double* const x,
         OGPS_Double* const y,
         OGPS_Double* const z);

      virtual OpenGPS::Schemas::ISO5436_2::ISO5436_2Type* const GetDocument();

      virtual OGPS_Boolean IsMatrix() const;

      virtual void GetMatrixDimensions(
         OGPS_ULong * const size_u,
         OGPS_ULong * const size_v,
         OGPS_ULong * const size_w) const;

      virtual OGPS_ULong GetListDimension() const;

      virtual void Write(const int compressionLevel = -1);

      virtual void Close();

      virtual void AppendVendorSpecific(const OpenGPS::String& vendorURI, const OpenGPS::String& filePath);
      virtual OGPS_Boolean GetVendorSpecific(const OpenGPS::String& vendorURI, const OpenGPS::String& fileName, const OpenGPS::String& targetPath);

   protected:

      /*!
       * Checks whether points are stored as separate binary files
       * or directly within the main ISO5436-2 XML document.
       * @returns Returns TRUE if there exist an external binary
       * file for point data within the archive, otherwise FALSE is
       * returned and the point vectors are saved within the
       * ISO5436-2 XML document also contained within the X3P archive.
       */
      OGPS_Boolean IsBinary() const;

      /*!
       * Checks whether validity information on point vector data is
       * provided by an extra binary file. Such a binary file is needed
       * in connection with non-IEEE754 data types (see the axis definition)
       * and when invalid measurements are contained within point data only.
       * @returns Returns TRUE when the existance of such a file needs
       * to be taken into account, FALSE otherwise.
       */
      OGPS_Boolean HasValidPointsLink() const;

      /*!
       * Gets the maximum index value possible in X direction.
       * For vector data in matrix topology this is the value of X direction only.
       * This is the value of the size of the data point list in the non-matrix case.
       * @see ISO5436_2Container::IsMatrix.
       * @return Returns the maximum of an index value in X direction (of matrix topology).
       * There is a fallback to the maximum amount of point vectors contained in
       * the point vector list in the non-matrix case.
       */
      OGPS_ULong GetMaxU() const;

      /*!
       * Gets the maximum index value possible in Y direction.
       * For vector data in matrix topology this is the value of Y direction only.
       * This is the value of the size of the data point list in the non-matrix case.
       * @see ISO5436_2Container::IsMatrix.
       * @return Returns the maximum of an index value in Y direction (of matrix topology).
       * There is a fallback to the maximum amount of point vectors contained in
       * the point vector list in the non-matrix case.
       */
      OGPS_ULong GetMaxV() const;

      /*!
       * Gets the maximum index value possible in Z direction.
       * For vector data in matrix topology this is the value of Z direction only.
       * This is the value of the size of the data point list in the non-matrix case.
       * @see ISO5436_2Container::IsMatrix.
       * @return Returns the maximum of an index value in Z direction (of matrix topology).
       * There is a fallback to the maximum amount of point vectors contained in
       * the point vector list in the non-matrix case.
       */
      OGPS_ULong GetMaxW() const;

      /*! Gets the file path to the X3P archive the current instance is an interface for. */
      const OpenGPS::String& GetFilePath() const;

      /*! Gets the absolute file path to the X3P archive the current instance is an interface for. */
      OpenGPS::String GetFullFilePath() const;

      /*! Gets the full path to the directory where temporary files get stored. */
      const OpenGPS::String& GetTempDir() const;

      /*! Creates a unique temporary file name. */
      OpenGPS::String CreateContainerTempFilePath() const;

      /*! Gets the relative path of the main xml document file within the zip archive. */
      OpenGPS::String GetMainArchiveName() const;

      /*! Gets the full path to the temporaryily decompressed main xml document file. */
      OpenGPS::String GetMainFileName() const;

      /*! Gets the relative path of the binary point data file within the zip archive. */
      OpenGPS::String GetPointDataArchiveName() const;

      /*! Gets the full path to the temporaryily decompressed binary point data file. */
      OpenGPS::String GetPointDataFileName();

      /*! Gets the relative path of the binary point validity data file within the zip archive. */
      OpenGPS::String GetValidPointsArchiveName() const;

      /*! Gets the full path to the temporaryily decompressed binary point validity data file. */
      OpenGPS::String GetValidPointsFileName();

      /*! Gets the relative path of the md5 checksum file within the zip archive. */
      OpenGPS::String GetChecksumArchiveName() const;

      /*! Gets the full path to the temporaryily decompressed md5 checksum file. */
      OpenGPS::String GetChecksumFileName() const;

      /*!
       * Decompresses and verifies the current X3P archive.
       *
       * @remarks If this throws an exception there may exist incorrect and incomplete data.
       * Do call ISO5436_2Container::Reset to avoid an inconsistent state.
       */
      void Decompress();

      /*!
       * Decompresses a single file within the zip archive.
       * @param src The name of the file to be decompressed. This is the relative path with
       * the archive itself set as the root element.
       * @param dst The absolute(!) target path where uncompressed data gets stored on the media.
       * @param fileNotFoundAllowed True if it is OK if a file could not be found within the archive.
       * This method proceeds without throwing an exception then and its return value is FALSE.
       * @returns Returns FALSE if a file could not be found in the archive (see the discussion above),
       * TRUE in all other cases.
       */
      OGPS_Boolean Decompress(const OpenGPS::String& src, const OpenGPS::String& dst, const OGPS_Boolean fileNotFoundAllowed = FALSE) const;

      /*!
       * Decompresses the main xml document contained within the X3P archive.
       * @see ISO5436_2Container::Decompress, ISO5436_2Container::GetMainArchiveName,
       * ISO5436_2Container::GetMainFileName
       */
      void DecompressMain() const;

      /*!
       * Decompresses the md5 checksum file contained within the X3P archive.
       * @see ISO5436_2Container::Decompress, ISO5436_2Container::GetChecksumArchiveName,
       * ISO5436_2Container::GetChecksumFileName
       */
      void DecompressChecksum();

      /*!
       * Decompresses the binary point data file contained within the X3P archive.
       * @see ISO5436_2Container::Decompress, ISO5436_2Container::GetPointDataArchiveName,
       * ISO5436_2Container::GetPointDataFileName
       */
      void DecompressDataBin();


      /*!
       * (Over)writes the current X3P archive file with the actual content.
       */
      void Compress();

      /*!
       * Creates the internal XML document tree structure.
       * The Record3 and Record4 structures defined in
       * the ISO5436-2 XML specification are created automatically.
       * @param record1 The content of the Record1 XML structure.
       * @param record2 The content of the Record2 XML structure.
       * @param matrixDimension The dimension of the matrix structure
       * or NULL if a list structure is used.
       * @param listDimension The size of the data list that stores
       * the point vectors or 0 if the matrix structure is used instead.
       * @param useBinaryData Set this to TRUE to store the point
       * data within an external binary file. If set to FALSE point vectors
       * will get storead within the ISO5436-2 XML document directly.
       */
      void CreateDocument(
         const Schemas::ISO5436_2::Record1Type* const record1,
         const Schemas::ISO5436_2::Record2Type* const record2,
         const Schemas::ISO5436_2::MatrixDimensionType* const matrixDimension,
         const OGPS_ULong listDimension,
         const OGPS_Boolean useBinaryData);

      /*!
       * Creates an instance of the internal ISO5436-2 XML document tree.
       * An instance is created from the decompressed main xml document file.
       */
      void ReadDocument();

      /*!
       * Assembles a new OpenGPS::PointVectorParser object using the OpenGPS::PointVectorParserBuilder.
       * @param builder The instance of the builder that is used to create the vector parser.
       */
      virtual void BuildPointVectorParser(PointVectorParserBuilder& builder) const;

      /*! Gets the data type of point data of the X component of a vector. */
      OGPS_DataPointType GetXaxisDataType() const;

      /*! Gets the data type of point data of the Y component of a vector. */
      OGPS_DataPointType GetYaxisDataType() const;

      /*! Gets the data type of point data of the Z component of a vector. */
      OGPS_DataPointType GetZaxisDataType() const;

      /*!
       * Gets the data type of point data of an axis.
       * @param axis The axis description to prove.
       * @param incremental TRUE if implict point values are allowed for an incremental axis. This holds true for every axis
       * except the Z axis.
       * @returns Returns the type of data points of an axis or OGPS_MissingPointType on an incremntal axis definition.
       */
      OGPS_DataPointType GetAxisDataType(const Schemas::ISO5436_2::AxisDescriptionType& axis, const OGPS_Boolean incremental) const;

      /*!
       * Assembles a new OpenGPS::VectorBuffer object using the OpenGPS::VectorBufferBuilder.
       * @param builder The instance of the builder that is used to create the vector buffer.
       * @returns Returns TRUE on success, FALSE otherwise.
       */
      virtual OGPS_Boolean BuildVectorBuffer(VectorBufferBuilder& builder) const;

      /*!
       * Sets up the internal memory storage of point data.
       * Creates and allocates the internal vector buffer and fills in point data from either the
       * ISO5436-2 main xml document or from an external binary file.
       */
      void CreatePointBuffer();

      /*!
       * Saves the current state of internal memory storage of point data either to the zip archive
       * as an external binary point file or directly into the actual tree structure of the internal
       * document handle of the main ISO5436-2 XML document depending on the current settings.
       * @param handle The handle to the zip archive where the data is to be stored.
       */
      void SavePointBuffer(zipFile handle);

      /*!
       * Removes the point list xml tag and its content from the xml document handle.
       */
      void ResetXmlPointList();

      /*!
       * Rests the valid points link xml tag of the xml document handle to its default value.
       */
      void ResetValidPointsLink();

      /*!
       * Saves the current state of internal memory storage of point validity data to the zip archive.
       * @param handle The handle to the zip archive where the data is to be stored.
       */
      void SaveValidPointsLink(zipFile handle);

      /*!
       * Saves the main md5 checksum to the zip archive.
       * @param handle The handle to the zip archive where the data is to be stored.
       * @param checksum The value of the calculated 128bit md5 checksum.
       */
      void SaveChecksumFile(zipFile handle, const OpenGPS::UnsignedByte checksum[16]);

      /*!
       * Creates an instance of appropriate access methods to read point data depending on
       * the current configuration of the already loaded main ISO5436-2 XML document.
       * @returns An instance to access raw point data for reading or NULL on failure.
       * The pointer returned must be released by the caller.
       */
      virtual PointVectorReaderContext* CreatePointVectorReaderContext();

      /*!
       * Creates an instance of appropriate access methods to write point data depending on
       * the current configuration of the main ISO5436-2 XML document.
       * @param handle The handle to the zip archive is needed by the special implementation
       * of the context for writing point data to an external binary file.
       * @returns An instance to write raw point data or NULL on failure.
       * The pointer returned must be released by the caller.
       */
      virtual PointVectorWriterContext* CreatePointVectorWriterContext(zipFile handle) const;

      /*!
       * Gets the amount of point data that is stored in the archive.
       */
      OGPS_ULong GetPointCount() const;

      /*!
       * Releases any resources allocated to handle the xml document tree or internal storage of vectors in memory.
       */
      void Reset();

      /*!
       * Returns TRUE if an ISO5436-2 XML document has been loaded into memory as a tree structure,
       * or FALSE if the internal document handle does not exist.
       */
      OGPS_Boolean HasDocument() const;

      /*!
       * Returns TRUE if the internal memory storage area of vector data had been allocated, or FALSE if not.
       */
      OGPS_Boolean HasVectorBuffer() const;

      /*!
       * Reads the main ISO5436-2 XML document contained in an X3P archive to the internal
       * document handle as a tree structure.
       */
      void ReadXmlDocument();

      /*!
       * Writes the content of the internal document handle to the main XML document present in an X3P archive.
       * @param handle The handle of the X3P archive.
       */
      void SaveXmlDocument(zipFile handle);

      /*!
       * Creates a new instance of a vector proxy context that is used to map point data saved distinctively
       * as one column for every axis definition to one single row vector. The context object provides the indexing
       * information necessary for that task.
       * @returns A new instance of a vector proxy context. The caller ist responsible of releasing this resource.
       */
      virtual PointVectorProxyContext* CreatePointVectorProxyContext() const;

      /*!
       * Checks for the current document instance and raises an excpetion if it is not instantiated.
       */
      void CheckDocumentInstance() const;


   private:
      /*! The path of the X3P archive handles. */
      OpenGPS::String m_FilePath;

      /*! The path to a temporary directory unique to the current instance. */
      OpenGPS::String m_TempPath;

      /*! The path to the global directory for temporary files. */
      OpenGPS::String m_TempBasePath;

      /*! TRUE if the user wants readonly access to the X3P file only. FALSE otherwise. */
      OGPS_Boolean m_IsReadOnly;

      /*! TRUE if an X3P archive is to be created, FALSE if an existing archive has been opened.
       * The value is undefined if nothing happened so far.
       */
      OGPS_Boolean m_IsCreating;

      /*! The temporary target path of the uncompressed binary point data file. */
      OpenGPS::String m_PointDataFileName;

      /*! The temporary target path of the uncompressed binary point validity data file. */
      OpenGPS::String m_ValidPointsFileName;

      /*! The level of compression of the zip archive. */
      int m_CompressionLevel;

      /*! The handle to a tree strucure corresponding to an ISO5436-2 XML document file. */
      Schemas::ISO5436_2::ISO5436_2Type* m_Document;

      /*! The builder used to assemble the current vector buffer. */
      VectorBufferBuilderAutoPtr m_VectorBufferBuilder;

      /*! A global point vector proxy used for index based read/writes of point vectors. */
      PointVectorProxyContextAutoPtr m_ProxyContext;

      /*! A point vector which serves as a temporary buffer. */
      PointVectorAutoPtr m_PointVector;

      /*! Creates a temporary directory in the file system and sets ISO5436_2Container::m_TempPath. */
      void CreateTempDir();

      /*! Returns TRUE if a unique temporary directory has been created or FALSE if not. */
      OGPS_Boolean HasTempDir() const;

      /*! Removes the current temporary directory from the file system and erases all of its content. */
      void RemoveTempDir();

      /*! Gets a pointer to the vector buffer or NULL. */
      VectorBuffer* GetVectorBuffer();

      /*! FALSE, if the md5 checksum could not be verified after reading. */
      OGPS_Boolean m_MainChecksum;

      /*! FALSE, if the md5 checksum could not be verified after reading. */
      OGPS_Boolean m_DataBinChecksum;

      /*! FALSE, if the md5 checksum could not be verified after reading. */
      OGPS_Boolean m_ValidBinChecksum;

      /*! ID of vendorspecific data or empty. @see ISO5436_2Container::m_VendorSpecific. */
      OpenGPS::String m_VendorURI;

      typedef std::vector<OpenGPS::String> StringList;

      /*! Vendorspecific file names to be added to the container registered with one single vendor id. @see ISO5436_2Container::m_VendorURI */
      StringList m_VendorSpecific;

      /*!
       * Writes vendorspecific files to the zip container if any.
       *
       * @param handle Handle of the target zip archive.
       * @returns Returns FALSE if not all of the files could be written. TRUE otherwise.
       */
      OGPS_Boolean WriteVendorSpecific(zipFile handle);

      /*!
       * Sets a valid namespace mapping for writing the ISO5436-2 XML document.
       * @param map Target for the valid namespace mapping.
       */
      void ConfigureNamespaceMap(xml_schema::namespace_infomap& map) const;

      /*!
       * Sets local pathes to XSD files corresponding to the global namespaces of the ISO5436-2 XML specification.
       * @param props Target for the global to local namespace mapping.
       * @returns Returns TRUE on success, FALSE otherwise.
       */
      OGPS_Boolean ConfigureNamespaceMap(xml_schema::properties& props) const;

      /*!
       * Verifies an 128bit md5 checksum.
       * @param filePath The file which checksum is to be verified.
       * @param checksum The expected checksum to verify.
       * @param size The size of the checksum buffer in bytes. This must be equal to 16 always as it is a 128bit md5 sum.
       * @returns Returns TRUE when the checksum could be verified, FALSE otherwise.
       */
      OGPS_Boolean VerifyChecksum(const OpenGPS::String& filePath, const OpenGPS::UnsignedBytePtr checksum, const size_t size) const;

      /*!
       * Verifies the checksum of the main document ISO5436-2 XML file.
       */
      void VerifyMainChecksum();

      /*!
       * Verifies the checksum of the binary point data file.
       */
      void VerifyDataBinChecksum();

      /*!
       * Verifies the checksum of the binary point validity data file.
       */
      void VerifyValidBinChecksum();

      /*!
       * Check if all checksums were verified.
       * If any one of them could not be verified this throws an OpenGPS::Exception of type ::OGPS_ExWarning that may be ignored.
       */
      void TestChecksums();

      /*!
       * Checks the document for some semantic errors.
       * If any semantic errors were found, this throws an OpenGPS::Exception of type ::OGPS_ExWarning that may be ignored.
       */
      void ValidateDocument();

      /*!
       * Checks whether the current document data is either for profiles or surfaces.
       * @remarks Throws an exception if the current feature type is undefined.
       * @returns True, if the document record specifies a profile, FALSE otherwise.
       */
      OGPS_Boolean IsProfile() const ;

      /*!
       * Checks whether the current document data is for point clouds.
       * @remarks Throws an exception if the current feature type is undefined.
       * @returns True, if the document record specifies a point cloud, FALSE otherwise.
       */
      OGPS_Boolean IsPointCloud() const;

      /*!
       * Reads the first md5 checksum from a file that contains md5 checksums of files.
       * @param fileName The path to the file that contains md5 checksums.
       * @param checksum Target of the extracted checksum.
       * @returns Returns TRUE on success, FALSE otherwise.
       */
      OGPS_Boolean ReadMd5FromFile(const OpenGPS::String& fileName, OpenGPS::UnsignedByte checksum[16]) const;

      /*!
       * Extracts the three components of a point vector.
       *
       * May throw an OpenGPS::Exception on failure due to type incompatibilities or overflow.
       * Vector components that result from a missing point are set to IEEE NaN.
       * This means that incremental axis allways produce a valid result, but absolute axis create a NaN.
       *
       * @param vector The vector which components are to be extracted.
       * @param x Target of the value of the X component of that vector. If this equals NULL, this component is ignored.
       * @param y Target of the value of the Y component of that vector. If this equals NULL, this component is ignored.
       * @param z Target of the value of the Z component of that vector. If this equals NULL, this component is ignored.
       */
      void ConvertPointToCoord(
         const  PointVector& vector,
         OGPS_Double* const x,
         OGPS_Double* const y,
         OGPS_Double* const z);

      /*! Returns TRUE if the axis has an incremental axis definition, FALSE otherwise. If TRUE point data of that axis is known implicitly. */
      OGPS_Boolean IsIncrementalAxis(Schemas::ISO5436_2::AxisType axis) const;

      /*! Returns TRUE if X has an incremental axis definition, FALSE otherwise. If TRUE point data of that axis is known implicitly. */
      OGPS_Boolean IsIncrementalX() const;

      /*! Returns TRUE if Y has an incremental axis definition, FALSE otherwise. If TRUE point data of that axis is known implicitly. */
      OGPS_Boolean IsIncrementalY() const;

      /*! Returns TRUE if Z has an incremental axis definition, FALSE otherwise. If TRUE point data of that axis is known implicitly. */
      OGPS_Boolean IsIncrementalZ() const;

      /*! Gets the increment of the X axis definition or 1.0 if there is no incremental step. */
      double GetIncrementX() const;

      /*! Gets the increment of the Y axis definition or 1.0 if there is no incremental step. */
      double GetIncrementY() const;

      /*! Gets the offset of the X axis definition or 0.0 if there is no offset at all. */
      double GetOffsetX() const;

      /*! Gets the offset of the Y axis definition or 0.0 if there is no offset at all. */
      double GetOffsetY() const;

      /*! Gets the offset of the Z axis definition or 0.0 if there is no offset at all. */
      double GetOffsetZ() const;

      /*!
       * Converts a value of unsigned long long into a smaller type.
       * Throws an exception on overflow, so this conversion is safe.
       * @param value The value to be converted.
       * @returns Returns the same value, but as a cast to a smaller data type.
       */
      OGPS_ULong ConvertULongLongToULong(const unsigned long long value) const;

      /*!
       * Converts a value of unsigned long long into a shorter data type.
       * Throws an exception on overflow, so this conversion is safe.
       * @param value The value to be converted.
       * @returns Returns the same value, but casted to a shorter data type.
       */
      OGPS_Int32 ConvertULongToInt32(const unsigned long long value) const;

      /*!
       * Multiplication of two values and conversion of the result to a shorter data type.
       * Throws an exception on overflow, so this conversion is safe.
       * @param value1 The first value of the operands.
       * @param value2 The second value of the operands.
       * @returns Returns the result of their multiplication, but casted to a shorter data type.
       */
      OGPS_ULong SafeMultipilcation(const unsigned long long value1, const unsigned long long value2) const;

      /*!
       * Converts any string to its URI conformant compangnon.
       *
       * @param src String to convert.
       * @returns Returns URI conformant string.
       */
      String ConvertToURI(const String& src) const;

      /*!
      * Implementation of the point iterator interface.
      * A point iterator can be created and initialized
      * by an instance of OpenGPS::ISO5436_2Container only.
      */
      class PointIteratorImpl : public PointIterator
      {
      public:
         /*!
         * Creates a new instance.
         * @param handle Provides access to point data based on indexes.
         * @param isForward TRUE if iterating in increasing order of
         * indexes but FALSE when starting from the end with decreasing indexes.
         * @param isMatrix TRUE to access the matrix interface methods of
         * the given OpenGPS::ISO5436_2Container instance. FALSE to access
         * the list interface methods instead.
         */
         PointIteratorImpl(ISO5436_2Container * const handle,
            const OGPS_Boolean isForward,
            const OGPS_Boolean isMatrix);

         /*! Destroys this instance. */
         virtual ~PointIteratorImpl();

         /* Implements the public PointIterator interface. */

         virtual OGPS_Boolean HasNext() const;
         virtual OGPS_Boolean HasPrev() const;

         virtual OGPS_Boolean MoveNext();
         virtual OGPS_Boolean MovePrev();

         virtual void ResetNext();
         virtual void ResetPrev();

         virtual void GetCurrent(PointVector& vector);
         virtual void GetCurrentCoord(PointVector& vector);

         virtual void SetCurrent(const PointVector* const vector);

         virtual OGPS_Boolean GetPosition(OGPS_ULong * const index) const;
         virtual OGPS_Boolean GetPosition(
            OGPS_ULong * const u,
            OGPS_ULong * const v,
            OGPS_ULong * const w) const;

      private:
         /*! Instance of the ISO5436-2 X3P to be accessed through this popint iterator. */
         ISO5436_2Container * const m_Handle;

         /*! This is the forward- or backward iterator. */
         OGPS_Boolean m_IsForward;

         /*! TRUE if in an untouched state. */
         OGPS_Boolean m_IsReset;

         /*! TRUE to use matrix indexes and access methods, but FALSE for the simple list interface. */
         OGPS_Boolean m_IsMatrix;

         /*! The current index in X direction. Used when iterating both matrices and lists. */
         OGPS_ULong m_U;

         /*! The current index in Y direction. Used with matrices only. */
         OGPS_ULong m_V;

         /*! The current index in Z direction. Used with matrices only. */
         OGPS_ULong m_W;

         /*! The copy-ctor is not implemented. This prevents its usage. */
         PointIteratorImpl(const PointIteratorImpl& src);
         /*! The assignment-operator is not implemented. This prevents its usage. */
         PointIteratorImpl& operator=(const PointIteratorImpl& src);
      };
   };
}

#endif /* _OPENGPS_ISO5436_2_CONTAINER_HXX */

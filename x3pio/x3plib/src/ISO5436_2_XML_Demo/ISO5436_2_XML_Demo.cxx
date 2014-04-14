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
 *   openGPS and the openGPS logo is a registered trademark of             *
 *   Physikalisch Technische Bundesanstalt (PTB)                           *
 *   http://www.ptb.de/                                                    *
 *                                                                         *
 *   More information about openGPS can be found at                        *
 *   http://www.opengps.eu/                                                *
 ***************************************************************************/

/*! @file
 *  Example programm demonstrating some of the features of the X3P-data format.
 */

#include <opengps/cxx/opengps.hxx>

#include <opengps/iso5436_2.h>
#include <opengps/cxx/iso5436_2.hxx>
#include <opengps/cxx/iso5436_2_handle.hxx>
#include <opengps/cxx/iso5436_2_xsd.hxx>
#include <opengps/cxx/point_iterator.hxx>
#include <opengps/cxx/point_vector.hxx>
#include <opengps/cxx/data_point.hxx>
#include <opengps/cxx/string.hxx>
#include <opengps/cxx/info.hxx>

#include <string>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <ios>
#include <ostream>
#include <fstream>
#include <cstdlib>
#include <ctime>
#include <limits>

#if linux
    #include "assert.h"
    #include "math.h"
#else
    #include <tchar.h>
#endif

#include <time.h>

using namespace std;
using namespace OpenGPS::Schemas::ISO5436_2;

/*!
  @brief Prints list or matrix dimension of a given x3p handle to cout.

  @param handle;

  @return TRUE on success, FALSE if handle is neither a list nor a matrix.
*/
OGPS_Boolean
PrintDimensions(const OGPS_ISO5436_2Handle handle)
{
#if linux
    assert(handle);
#else
  _ASSERT(handle);
#endif

  // Check for Matrix or List
  bool ismatrix=ogps_IsMatrix(handle);
  if (ogps_HasError())
  {
    cerr << "Error checking matrix type!" << endl;
    return FALSE;
  }

  // Check for Matrix or List
  if (ismatrix)
  {
    /* Get Matrix dimensions */
    unsigned long sx,sy,sz;
    ogps_GetMatrixDimensions(handle,&sx,&sy,&sz);
    cout << "Matrix dimensions are (x,y,z): " << sx << ", " << sy << ", " << sz << endl;
    return TRUE;
  }
  else
  {
    unsigned long sx = ogps_GetListDimension(handle);
    cout << "This is a list with " << sx << " elements." << endl;
    return TRUE;
  }

  return FALSE;
}


/*!
  @brief Helper function to return the current time properly formated.

  @return A string containing the time stamp from now.

  @note There is only a windows implementation yet. In other cases
  return a dummy. That is enough for testing purposes.
*/
#ifdef _WIN32
OpenGPS::String TimeStamp(void)
{
  time_t ltime;
  struct tm lt;
  // Time zone offset
  long tzoff;
  // Set timezone
  _tzset();
  // Get time zone offset
  _get_timezone(&tzoff);
  // Offset ours and minutes
  int tzoff_h,tzoff_m;
  tzoff_h = -(int)floor(((double)tzoff)/3600.);
  tzoff_m = -(int)floor(((double)tzoff)/60. + 0.5) - tzoff_h*60;

  // Get current time
  time( &ltime );
  // get local time
  localtime_s(&lt,&ltime);

  // Correct tz offset by dst
  if (lt.tm_isdst > 0)
    tzoff_h++;

  // Absolute offset for printing
  int tzoff_habs = abs(tzoff_h);
  OGPS_Character tzoffsign = tzoff_h<0 ? _T('-') : _T('+');

  // Create a string of pattern "2007-04-30T13:58:02.6+02:00"
  wostringstream sout;
//  sout << std::setfill(_T('0')) << setw(4) << (lt.tm_year+1900) << _T('-') << setw(2) << lt.tm_mon << _T('-') << setw(2) << lt.tm_mday
//      << _T('T') << lt.tm_hour << _T(':') << lt.tm_min << _T(':') << setw(2) << lt.tm_sec << _T(".0")
//      << tzoffsign << setw(2) << tzoff_habs << _T(':') << setw(2) << tzoff_m << ends;

  sout << setw(4) << (lt.tm_year+1900) << _T('-') << setw(2) << lt.tm_mon << _T('-') << setw(2) << lt.tm_mday
      << _T('T') << lt.tm_hour << _T(':') << lt.tm_min << _T(':') << setw(2) << lt.tm_sec << _T(".0")
      << tzoffsign << setw(2) << tzoff_habs << _T(':') << setw(2) << tzoff_m << ends;

//  return sout.str();
  return _T("2007-04-30T13:58:02.6+02:00");
}
#else
OpenGPS::String TimeStamp(void)
{
    time_t ltime;
    struct tm lt;
    // Time zone offset
    long tzoff;

    // Set timezone
    tzset();
    // Get time zone offset
    time_t tm1, tm2;
    struct tm *t1, *t2;
    tm1 = time(NULL);
    t2 = gmtime(&tm1);
    tm2 = mktime(t2);
    t1 = localtime(&tm1);
    tzoff = (long)(tm1 - tm2);
//    get_timezone(&tzoff);
    // Offset ours and minutes
    int tzoff_h,tzoff_m;
    tzoff_h = -(int)floor(((double)tzoff)/3600.);
    tzoff_m = -(int)floor(((double)tzoff)/60. + 0.5) - tzoff_h*60;
/*
    // Get current time
    time( &ltime );
    // get local time
    localtime_s(&lt,&ltime);

    // Correct tz offset by dst
    if (lt.tm_isdst > 0)
      tzoff_h++;
*/
    // Absolute offset for printing
    int tzoff_habs = abs(tzoff_h);
    OGPS_Character tzoffsign = tzoff_h<0 ? _T('-') : _T('+');

    // Create a string of pattern "2007-04-30T13:58:02.6+02:00"
    wostringstream sout;
    sout << std::setfill(_T('0')) << setw(4) << (lt.tm_year+1900) << _T('-') << setw(2) << lt.tm_mon << _T('-') << setw(2) << lt.tm_mday
        << _T('T') << lt.tm_hour << _T(':') << lt.tm_min << _T(':') << setw(2) << lt.tm_sec << _T(".0")
        << tzoffsign << setw(2) << tzoff_habs << _T(':') << setw(2) << tzoff_m << ends;

    return sout.str();
}

#endif


/*!
  @brief Prints all meta data available in the current document.

  @param Pointer to xml document tree

  @return TRUE on success, FALSE if no meta data available.
*/
OGPS_Boolean
PrintMetaData(const ISO5436_2Type* const document)
{
    #if linux
        assert(document);
    #else
        _ASSERT(document);
    #endif

  // Get a reference to optional record 2
  const OpenGPS::Schemas::ISO5436_2::ISO5436_2Type::Record2_optional &r2opt = document->Record2();

  // Check for presence of meta data
  if (r2opt.present() == FALSE)
  {
    cout << "Warning: This document does not contain a meta data record (Record2)!" << endl;
    return FALSE;
  }

  cout << std::endl << "Meta data in record2:" << std::endl;

  // Get the Record2
  const OpenGPS::Schemas::ISO5436_2::ISO5436_2Type::Record2_type &r2 = r2opt.get();

  // Data set creation date
//  cout << "Data set creation date: " << r2.Date() << endl;
  cout << "Data set creation date: " << r2.Date().year() << "-"
      << r2.Date().month() << "-"
      << r2.Date().day() << "-"
      << r2.Date().hours() << ":"
      << r2.Date().minutes() << ":"
      << r2.Date().seconds() << endl;
  // Check for data set creator
  if (r2.Creator().present())
  {
    if (r2.Creator().get().length() < 1)
      cout << "Warning: Data set creator name is present, but empty!" << endl;
    else
      cout << "Data set has been created by \"" << r2.Creator().get() << '"' << std::endl;
  }
  else
    cout << "Data set was created anonymously." << endl;

  // Measuring instrument
  cout << "Measuring Instrument Description: " << endl
    << "  Manufacturer: \"" << r2.Instrument().Manufacturer() << '"' << endl
    << "         Model: \"" << r2.Instrument().Model() << '"' << endl
    << " Serial Number: \"" << r2.Instrument().Serial() << '"' << endl
    << "       Version: \"" << r2.Instrument().Version() << '"' << endl;

  // Calibration
//  cout << "Instrument was calibrated: \"" << r2.CalibrationDate() << '"' << endl;
    cout << "Instrument was calibrated: \"" << r2.CalibrationDate().year() << "-"
      << r2.CalibrationDate().month() << "-"
      << r2.CalibrationDate().day() << "-"
      << r2.CalibrationDate().hours() << ":"
      << r2.CalibrationDate().minutes() << ":"
      << r2.CalibrationDate().seconds() << endl;

  // Probing system type
  cout << "Probing system type: \"" << r2.ProbingSystem().Type() << '"' << endl
    << "Probing system identification: \"" << r2.ProbingSystem().Identification() << '"' << endl;

  // Extract and print information
  if (r2.Comment().present())
  {
    if (r2.Comment().get().length() < 1)
      cout << "Warning: File comment field is present, but empty!" << endl;
    else
      cout << "File Comment: \"" << r2.Comment().get() << '"' << endl;
  }
  else
    cout << "Warning: File contains no Comment field!" << endl;

  return TRUE;
}

/*!
  @brief Prints all meta data available in the current document.

  @param handle to current X3P document

  @return TRUE on success, FALSE if no meta data available.
*/
OGPS_Boolean
PrintMetaData(const OGPS_ISO5436_2Handle handle)
{
    #if linux
        assert(handle);
    #else
        _ASSERT(handle);
    #endif

  // Get document
  const ISO5436_2Type* const document = ogps_GetDocument(handle);
    #if linux
        assert(document);
    #else
        _ASSERT(document);
    #endif

  if (ogps_HasError())
  {
    cerr << "Error accessing XML document while printing meta data!" << endl;
    return FALSE;
  }

  // Call Print function for c++ document
  return PrintMetaData(document);
}


 /*!
    * @brief Creates and writes a simple ISO5436-2 XML X3P file
    * with two incremental and one absolute axis (the z-axis).
    *
    * @param fileName Full path to the ISO5436-2 XML X3P to write.
    */
void simpleExample(const OpenGPS::String fileName)
{
   /* Simple example where we have two incremental and one absolute axis (the z-axis). */

   /* Create RECORD1 */
   Record1Type::Revision_type revision(OGPS_ISO5436_2000_REVISION_NAME);
   Record1Type::FeatureType_type featureType(OGPS_FEATURE_TYPE_SURFACE_NAME);

   Record1Type::Axes_type::CX_type::AxisType_type xaxisType(Record1Type::Axes_type::CX_type::AxisType_type::I); /* incremental */
   Record1Type::Axes_type::CX_type::DataType_type xdataType(Record1Type::Axes_type::CX_type::DataType_type::D); /* double */
   Record1Type::Axes_type::CX_type xaxis(xaxisType);
   xaxis.DataType(xdataType);

   /* Set increment and offset. */
   xaxis.Increment(1.60123456789123456789E-0002);
   xaxis.Offset(1.0123456789123456789e-1);

   Record1Type::Axes_type::CY_type::AxisType_type yaxisType(Record1Type::Axes_type::CY_type::AxisType_type::I); /* incremental */
   Record1Type::Axes_type::CY_type::DataType_type ydataType(Record1Type::Axes_type::CY_type::DataType_type::D); /* double */
   Record1Type::Axes_type::CY_type yaxis(yaxisType);
   yaxis.DataType(ydataType);

   /* Set increment and offset. */
   yaxis.Increment(1.60123456789123456789E-0002);
   yaxis.Offset(0.0);

   Record1Type::Axes_type::CZ_type::AxisType_type zaxisType(Record1Type::Axes_type::CZ_type::AxisType_type::A); /* absolute */
   Record1Type::Axes_type::CZ_type::DataType_type zdataType(Record1Type::Axes_type::CZ_type::DataType_type::D); /* double */
   Record1Type::Axes_type::CZ_type zaxis(zaxisType);
   zaxis.DataType(zdataType);
   // Default value for absolute axis
   zaxis.Increment(1);
   zaxis.Offset(-1e-1);

   Record1Type::Axes_type axis(xaxis, yaxis, zaxis);

   Record1Type record1(revision, featureType, axis);

   /* Create RECORD2 */
//   Record2Type::Date_type date(TimeStamp());
   Record2Type::Date_type date(TimeStamp(), 0);

   Record2Type::Instrument_type::Manufacturer_type manufacturer(_T("NanoFocus AG"));
   Record2Type::Instrument_type::Model_type model(_T("ISO5436_2_XML_Demo Software"));
   Record2Type::Instrument_type::Serial_type serial(_T("not available"));
   Record2Type::Instrument_type::Version_type version(_OPENGPS_VERSIONSTRING);
   Record2Type::Instrument_type instrument(manufacturer, model, serial, version);

//   Record2Type::CalibrationDate_type calibrationDate(_T("2007-04-30T13:58:02.6+02:00"));
   Record2Type::CalibrationDate_type calibrationDate(_T("2007-04-30T13:58:02.6+02:00"), 0);

   Record2Type::ProbingSystem_type::Type_type type(Record2Type::ProbingSystem_type::Type_type::Software);
   Record2Type::ProbingSystem_type::Identification_type id(_T("Random number generator"));
   Record2Type::ProbingSystem_type probingSystem(type, id);

   Record2Type::Comment_type comment(_T("This is a user comment specific to this dataset."));

   Record2Type record2(date, instrument, calibrationDate, probingSystem);
   record2.Comment(comment);


   /* Create MATRIX */
   MatrixDimensionType matrix(2, 2, 2);


   /* Create ISO5436_2 container */
   OGPS_ISO5436_2Handle handle = ogps_CreateMatrixISO5436_2(fileName.c_str(), NULL, record1, &record2, matrix, FALSE);

   /* Add data points */
   /* 1. Create point vector buffer for three points. */
   OGPS_PointVectorPtr vector = ogps_CreatePointVector();

   /* Matrix indices. */
   OGPS_ULong u = 0;
   OGPS_ULong v = 0;
   OGPS_ULong w = 0;

   /* 2. Since the z axis is of absolute type and the
   * other two are incremental, we simply set up just z
   * values and leave x an y values untouched (missing). */
   ogps_SetDoubleZ(vector, 4.86219120804151E-0001);

   /* 3. Write into document */
   ogps_SetMatrixPoint(handle, u , v, w, vector);

   /* Loop all data points we want to add... */
   ogps_SetDoubleZ(vector, 3E-0001);
   ogps_SetMatrixPoint(handle, 1 , 0, 0, vector);
   ogps_SetDoubleZ(vector, 2E-0001);
   ogps_SetMatrixPoint(handle, 0 , 1, 0, vector);
   ogps_SetDoubleZ(vector, 1E-0001);
   ogps_SetMatrixPoint(handle, 1 , 1, 0, vector);

   /* 2b/3b. We have missing point data, here. */
   ogps_SetMatrixPoint(handle, 0 , 0, 1, NULL);

   /* The above will show up in xml like this: */
   /*
   * <DataList>
   * <Datum>4.86219120804151E-0001</Datum>
   *
   * <!-- missing data points are represented by an empty tag-->
   * <Datum/>
   *
   * <Datum>8.23683772970184E-0001</Datum>
   * </DataList>
   */
   ogps_SetDoubleZ(vector, 5E-0001);
   ogps_SetMatrixPoint(handle, 1 , 0, 1, vector);
   ogps_SetDoubleZ(vector, 6E-0001);
   ogps_SetMatrixPoint(handle, 0 , 1, 1, vector);
   ogps_SetDoubleZ(vector, 7E-0001);
   ogps_SetMatrixPoint(handle, 1 , 1, 1, vector);

   /* Free buffer */
   ogps_FreePointVector(&vector);

   /* Create vendorspecific data. */
   OpenGPS::String vname(fileName.substr(0, fileName.find_last_of(_T("/\\")) + 1));
   vname.append(_T("vendor.tmp"));
   std::wofstream vendor_dat(vname.ToChar(), std::ios::trunc);
   vendor_dat << _T("Vendorspecific data.") << std::endl;
   vendor_dat.close();

   ogps_AppendVendorSpecific(handle, _T("http://www.example.com/format/version1"), vname.c_str());

   /* Print dimensions */
   PrintDimensions(handle);

   /* Finally: write container to disk. */
   ogps_WriteISO5436_2(handle);
   ogps_CloseISO5436_2(&handle);
}


void mediumComplexExample(const OpenGPS::String fileName)
{
   /* More complex example where we have three absolute axis. */

   /* Create RECORD1 */
   Record1Type::Revision_type revision(OGPS_ISO5436_2000_REVISION_NAME);
   Record1Type::FeatureType_type featureType(OGPS_FEATURE_TYPE_SURFACE_NAME);

   Record1Type::Axes_type::CX_type::AxisType_type xaxisType(Record1Type::Axes_type::CX_type::AxisType_type::A); /* absolute */
   Record1Type::Axes_type::CX_type::DataType_type xdataType(Record1Type::Axes_type::CX_type::DataType_type::L); /* int32 */
   Record1Type::Axes_type::CX_type xaxis(xaxisType);
   xaxis.DataType(xdataType);
   xaxis.Increment(10e-6); // 10 micro metre
   xaxis.Offset(1000.0e-6); // 1 milli metre

   Record1Type::Axes_type::CY_type::AxisType_type yaxisType(Record1Type::Axes_type::CY_type::AxisType_type::A); /* absolute */
   Record1Type::Axes_type::CY_type::DataType_type ydataType(Record1Type::Axes_type::CY_type::DataType_type::F); /* float */
   Record1Type::Axes_type::CX_type yaxis(xaxisType);
   yaxis.DataType(ydataType);
   yaxis.Increment(1); // set to 1 for float and double axis
   yaxis.Offset(-1000.0e-6); // -1 milli metre

   Record1Type::Axes_type::CZ_type::AxisType_type zaxisType(Record1Type::Axes_type::CZ_type::AxisType_type::A); /* absolute */
   Record1Type::Axes_type::CZ_type::DataType_type zdataType(Record1Type::Axes_type::CZ_type::DataType_type::D); /* double */
   Record1Type::Axes_type::CX_type zaxis(xaxisType);
   zaxis.DataType(zdataType);
   zaxis.Increment(1); // set to 1 for float and double axis
   zaxis.Offset(1000.0e-6); // 1 milli metre

   Record1Type::Axes_type axis(xaxis, yaxis, zaxis);

   Record1Type record1(revision, featureType, axis);

   /* Create RECORD2 */
//   Record2Type::Date_type date(TimeStamp());
   Record2Type::Date_type date(TimeStamp(), 0);

   Record2Type::Instrument_type::Manufacturer_type manufacturer(_T("NanoFocus AG"));
   Record2Type::Instrument_type::Model_type model(_T("ISO5436_2_XML_Demo Software"));
   Record2Type::Instrument_type::Serial_type serial(_T("not available"));
   Record2Type::Instrument_type::Version_type version(_OPENGPS_VERSIONSTRING);
   Record2Type::Instrument_type instrument(manufacturer, model, serial, version);

//   Record2Type::CalibrationDate_type calibrationDate(_T("2007-04-30T13:58:02.6+02:00"));
   Record2Type::CalibrationDate_type calibrationDate(_T("2007-04-30T13:58:02.6+02:00"), 0);

   Record2Type::ProbingSystem_type::Type_type type(Record2Type::ProbingSystem_type::Type_type::Software);
   Record2Type::ProbingSystem_type::Identification_type id(_T("Random number generator"));
   Record2Type::ProbingSystem_type probingSystem(type, id);

   Record2Type::Comment_type comment(_T("This is a user comment specific to this dataset."));

   Record2Type record2(date, instrument, calibrationDate, probingSystem);
   record2.Comment(comment);

   /* Create MATRIX */
   MatrixDimensionType matrix(4, 2, 2);

   /* Create ISO5436_2 container */
   OGPS_ISO5436_2Handle handle = ogps_CreateMatrixISO5436_2(fileName.c_str(), NULL, record1, &record2, matrix, TRUE);

   /* Print dimensions */
   PrintDimensions(handle);

   /* Add data points */
   /* 1. Create point vector buffer for three points. */
   OGPS_PointVectorPtr vector = ogps_CreatePointVector();

   /* 2. There are three absolute axis, so we have to
   * provide point values for all three axis. Though, we
   * do care about their data type. */

   /* Use iterator to create points in this example. */
   OGPS_PointIteratorPtr iterator = ogps_CreateNextPointIterator(handle);

   /* Create new points (current point set should be empty - of course) */
   /* Loop all 16 data points we want to add... */
   unsigned int cnt=0;
   while(ogps_MoveNextPoint(iterator))
   {
      /* Int not short! Although this wouldn't mess up
      * if we had forgotten "L": both data types are compatible,
      * and we would have accepted both (short and int) in
      * this special case here. */
      ogps_SetInt32X(vector, 4L*cnt);
      /* automatic type conversion occurs for double */
      ogps_SetFloatY(vector, 2.5F*cnt);
      /* Z has data type double */
      ogps_SetDoubleZ(vector, 4.8*cnt);

      /* 3. Write into document */
      ogps_SetCurrentPoint(iterator, vector);

      // Increment counter
      ++cnt;

      if(ogps_HasError())
      {
         break;
      }
   }

   // Free iterator
   ogps_FreePointIterator(&iterator);

   /* The above will show up in xml like this: */
   /*
   * <DataList>
   * <Datum>4;2.5;4.8</Datum>
   * </DataList>
   */

   /* Free buffer */
   ogps_FreePointVector(&vector);

   /* Print dimensions */
   PrintDimensions(handle);

   /* Finally: write container to disk. */
   ogps_WriteISO5436_2(handle);
   ogps_CloseISO5436_2(&handle);
}

void mediumComplexExampleWInvalid(const OpenGPS::String fileName)
{
   /* More complex example with integer encoded z-axis and invalid points */

   /* Create RECORD1 */
   Record1Type::Revision_type revision(OGPS_ISO5436_2000_REVISION_NAME);
   Record1Type::FeatureType_type featureType(OGPS_FEATURE_TYPE_SURFACE_NAME);

   Record1Type::Axes_type::CX_type::AxisType_type xaxisType(Record1Type::Axes_type::CX_type::AxisType_type::A); /* absolute */
   Record1Type::Axes_type::CX_type::DataType_type xdataType(Record1Type::Axes_type::CX_type::DataType_type::L); /* int32 */
   Record1Type::Axes_type::CX_type xaxis(xaxisType);
   xaxis.DataType(xdataType);
   xaxis.Increment(10e-6); // 10 micrometres
   xaxis.Offset(1000.0e-6); // 1 millimetre

   Record1Type::Axes_type::CY_type::AxisType_type yaxisType(Record1Type::Axes_type::CY_type::AxisType_type::A); /* absolute */
   Record1Type::Axes_type::CY_type::DataType_type ydataType(Record1Type::Axes_type::CY_type::DataType_type::F); /* float */
   Record1Type::Axes_type::CX_type yaxis(xaxisType);
   yaxis.DataType(ydataType);
   yaxis.Increment(1); // set to 1 for float and double axis
   yaxis.Offset(-1000.0e-6); // -1 milli metre

   Record1Type::Axes_type::CZ_type::AxisType_type zaxisType(Record1Type::Axes_type::CZ_type::AxisType_type::A); /* absolute */
   Record1Type::Axes_type::CZ_type::DataType_type zdataType(Record1Type::Axes_type::CZ_type::DataType_type::I); /* 16 bit integer */
   Record1Type::Axes_type::CX_type zaxis(xaxisType);
   zaxis.DataType(zdataType);
   zaxis.Increment(1e-6); // set to 1 for float and double axis
   zaxis.Offset(1000.0e-6); // 1 milli metre

   Record1Type::Axes_type axis(xaxis, yaxis, zaxis);

   Record1Type record1(revision, featureType, axis);

   /* Create RECORD2 */
//   Record2Type::Date_type date(TimeStamp());
   Record2Type::Date_type date(TimeStamp(), 0);

   Record2Type::Instrument_type::Manufacturer_type manufacturer(_T("NanoFocus AG"));
   Record2Type::Instrument_type::Model_type model(_T("ISO5436_2_XML_Demo Software"));
   Record2Type::Instrument_type::Serial_type serial(_T("not available"));
   Record2Type::Instrument_type::Version_type version(_OPENGPS_VERSIONSTRING);
   Record2Type::Instrument_type instrument(manufacturer, model, serial, version);

//   Record2Type::CalibrationDate_type calibrationDate(_T("2007-04-30T13:58:02.6+02:00"));
   Record2Type::CalibrationDate_type calibrationDate(_T("2007-04-30T13:58:02.6+02:00"), 0);

   Record2Type::ProbingSystem_type::Type_type type(Record2Type::ProbingSystem_type::Type_type::Software);
   Record2Type::ProbingSystem_type::Identification_type id(_T("Random number generator"));
   Record2Type::ProbingSystem_type probingSystem(type, id);

   Record2Type::Comment_type comment(_T("This is a user comment specific to this dataset."));

   Record2Type record2(date, instrument, calibrationDate, probingSystem);
   record2.Comment(comment);

   /* Create MATRIX */
   MatrixDimensionType matrix(4, 2, 2);

   /* Create ISO5436_2 container */
   OGPS_ISO5436_2Handle handle = ogps_CreateMatrixISO5436_2(fileName.c_str(), NULL, record1, &record2, matrix, TRUE);

   /* Print dimensions */
   PrintDimensions(handle);

   /* Add data points */
   /* 1. Create point vector buffer for three points. */
   OGPS_PointVectorPtr vector = ogps_CreatePointVector();

   /* 2. There are three absolute axis, so we have to
   * provide point values for all three axis. Though, we
   * do care about their data type. */

   /* Use iterator to create points in this example. */
   OGPS_PointIteratorPtr iterator = ogps_CreateNextPointIterator(handle);

   /* Create new points (current point set should be empty - of course) */
   /* Loop all 16 data points we want to add... */
   unsigned int cnt=0;
   while(ogps_MoveNextPoint(iterator))
   {
      /* Int not short! Although this wouldn't mess up
      * if we had forgotten "L": both data types are compatible,
      * and we would have accepted both (short and int) in
      * this special case here. */
      ogps_SetInt32X(vector, 4L*cnt);
      /* automatic type conversion occurs for double */
      ogps_SetFloatY(vector, 2.5F*(cnt % 4));

      //
      /* Z has data type 16 bit int */
      //ogps_SetDoubleZ(vector, 4.8*cnt);
      ogps_SetInt16Z(vector, (OGPS_Int16)floor(4.8*cnt));

      /* 3. Write into document */
      // Generate a missing point
      if (cnt != 8)
        ogps_SetCurrentPoint(iterator, vector);
      else
        ogps_SetCurrentPoint(iterator, NULL);

      // Increment counter
      ++cnt;

      if(ogps_HasError())
      {
         break;
      }
   }

   // Free iterator
   ogps_FreePointIterator(&iterator);

   /* The above will show up in xml like this: */
   /*
   * <DataList>
   * <Datum>4;2.5;4.8</Datum>
   * </DataList>
   */

   /* Free buffer */
   ogps_FreePointVector(&vector);

   /* Print dimensions */
   PrintDimensions(handle);

   /* Finally: write container to disk. */
   ogps_WriteISO5436_2(handle);
   ogps_CloseISO5436_2(&handle);
}


void readonlyExample(const OpenGPS::String fileName)
{
   /* We want to read in some file and read its point data. */
   std::cout << endl << endl << "readonlyExample(\"" << fileName << "\")" << endl;

   /* Open the file, hopefully everything went well... */
   OGPS_ISO5436_2Handle handle = ogps_OpenISO5436_2(fileName.c_str(), NULL, TRUE);

   // Check for error
   if (ogps_HasError())
   {
     // Print full description
     std::cerr << "Error opening file \"" << fileName << "\"" << endl;
     return;
   }

   if(!handle)
      return;

   /* Print meta data */
   PrintMetaData(handle);

   /* Is data list? / Is matrix? - don't care; we use point iterator. */
   /* Print dimensions */
   PrintDimensions(handle);

   /* Create point buffer. */
   OGPS_PointVectorPtr vector = ogps_CreatePointVector();

   /* Use iterator to create points in this example. */
   OGPS_PointIteratorPtr iterator = ogps_CreateNextPointIterator(handle);

   /* Iterate point data (ignoring the fact whether they were stored
    * in xml directly or in external binary file). */
   while(ogps_MoveNextPoint(iterator))
   {
      /* Get points at current position. */
      ogps_GetCurrentPoint(iterator, vector);

      if(ogps_HasError())
      {
         break;
      }

      /* Valid data point (not missing)?  */
      if(ogps_IsValidPoint(vector))
      {
         double x, y, z;
         ogps_GetXYZ(vector, &x, &y, &z);

         // Write point to console
         std::cout << ("X: ") << x << ("; Y: ") << y << ("; Z: ") << z <<std::endl;
      }
      else
      {
         std::cout << ("Invalid point") <<std::endl;
      }

   }

   std::cout << std::endl;

   // Failed?
   assert(!ogps_HasNextPoint(iterator));

   /* Free iterator/buffer */
   ogps_FreePointIterator(&iterator);
   ogps_FreePointVector(&vector);

   /* Close file */
   ogps_CloseISO5436_2(&handle);
}

void readonlyExampleMatrix(const OpenGPS::String fileName)
{
   /* Same as above, but here matrix format is assumed to provide indices within the output. */
  std::cout << endl << endl << "readonlyExampleMatrix(\"" << fileName << "\")" << endl;

   /* Open the file, hopefully everything went well... */
   OGPS_ISO5436_2Handle handle = ogps_OpenISO5436_2(fileName.c_str(), NULL, TRUE);

   // Check for error
   if (ogps_HasError())
   {
     // Print full description
     std::cerr << "Error opening file \"" << fileName << "\"" << endl;
     return;
   }

   if(!handle)
      return;

   /* Print meta data */
   PrintMetaData(handle);
   /* Print dimensions */
   PrintDimensions(handle);

   /* Assume matrix format. */

   /* Create point buffer. */
   OGPS_PointVectorPtr vector = ogps_CreatePointVector();

   /* Use iterator to create points in this example. */
   OGPS_PointIteratorPtr iterator = ogps_CreateNextPointIterator(handle);

   /* Matrix indices. */
   OGPS_ULong u = 0;
   OGPS_ULong v = 0;
   OGPS_ULong w = 0;

   /* Iterate point data (ignoring the fact whether they were stored
    * in xml directly or in external binary file). */
   while(ogps_MoveNextPoint(iterator))
   {
      // Get coordinates of this point in the matrix
      ogps_GetMatrixPosition(iterator, &u, &v, &w);

      /* Get points at current position. */
      ogps_GetCurrentPoint(iterator, vector);

      if(ogps_HasError())
      {
         break;
      }

      /* Valid data point (not missing)?  */
      if(ogps_IsValidPoint(vector))
      {
         double x, y, z;
         ogps_GetXYZ(vector, &x, &y, &z);

         // Write point to console
         std::cout << ("U: ") << u << ("; V: ") << v << ("; W: ") << w << ("X: ") << x << ("; Y: ") << y << ("; Z: ") << z <<std::endl;
      }
      else
      {
         std::cout << ("U: ") << u << ("; V: ") << v << ("; W: ") << w << (": Invalid point") <<std::endl;
      }

   }

   std::cout << std::endl;

   // Failed?
   assert(!ogps_HasNextPoint(iterator));

   /* Free iterator/buffer */
   ogps_FreePointIterator(&iterator);
   ogps_FreePointVector(&vector);

   /* Close file */
   ogps_CloseISO5436_2(&handle);
}

void readonlyExample2(const OpenGPS::String fileName)
{
   /* We want to read in some file and read its point data. */
  std::cout << endl << endl << "readonlyExample2(\"" << fileName << "\")" << endl;

   /* This time reading real typed data points. */
   /* With type information obtained from xml document. */

   /* Open the file, hopefully everything went well... */
   OGPS_ISO5436_2Handle handle = ogps_OpenISO5436_2(fileName.c_str(), NULL, TRUE);

   // Check for error
   if (ogps_HasError())
   {
     // Print full description
     std::cerr << "Error opening file \"" << fileName << "\"" << endl;
     return;
   }

   if(!handle)
      return;

   /* Obtain handle to xml document. */
   ISO5436_2Type * const document = ogps_GetDocument(handle);

   /* Z axis data type must be present (even if it is an incremental axis). */
    #if linux
        assert(document->Record1().Axes().CZ().DataType().present());
    #else
        _ASSERT(document->Record1().Axes().CZ().DataType().present());
    #endif

   /* Print meta data */
   PrintMetaData(handle);
   /* Print dimensions */
   PrintDimensions(handle);

   /* Is data list? / Is matrix? - don't care; we use point iterator. */

   /* Create point buffer. */
   OGPS_PointVectorPtr vector = ogps_CreatePointVector();

   /* Use iterator to create points in this example. */
   OGPS_PointIteratorPtr iterator = ogps_CreateNextPointIterator(handle);

   /* Iterate point data (ignoring if they were stored
   * in xml directly or in external binary file). */
   while(ogps_MoveNextPoint(iterator))
   {
      /* Get point at current position. */
      ogps_GetCurrentPoint(iterator, vector);

      if(ogps_HasError())
      {
         break;
      }

      /* Valid data point (not missing)?  */
      if(ogps_IsValidPoint(vector))
      {
        // Get transformed point at current position
        ogps_GetCurrentCoord(iterator, vector);
        std::cout << "X; Y; Z = ";

         switch(ogps_GetPointTypeX(vector))
         {
        case OGPS_Int16PointType : {
           short xs = ogps_GetInt16X(vector);
           std::cout << xs;
                                   } break;
        case OGPS_Int32PointType : {
           int xi = ogps_GetInt32X(vector);
           std::cout << xi;
                                   } break;
        case OGPS_FloatPointType : {
           float xf = ogps_GetFloatX(vector);
           std::cout << xf;
                                   } break;
        case OGPS_DoublePointType : {
           double xd = ogps_GetDoubleX(vector);
           std::cout << xd;
                                    } break;
         }

         std::cout << "; ";

         switch(ogps_GetPointTypeY(vector))
         {
        case OGPS_Int16PointType : {
           short ys = ogps_GetInt16Y(vector);
           std::cout << ys;
                                   } break;
        case OGPS_Int32PointType : {
           int yi = ogps_GetInt32Y(vector);
           std::cout << yi;
                                   } break;
        case OGPS_FloatPointType : {
           float yf = ogps_GetFloatY(vector);
           std::cout << yf;
                                   } break;
        case OGPS_DoublePointType : {
           double yd = ogps_GetDoubleY(vector);
           std::cout << yd;
                                    } break;
         }

         std::cout << "; ";

         switch(ogps_GetPointTypeZ(vector))
         {
        case OGPS_Int16PointType : {
           short zs = ogps_GetInt16Z(vector);
           std::cout << zs;
                                   } break;
        case OGPS_Int32PointType : {
           int zi = ogps_GetInt32Z(vector);
           std::cout << zi;
                                   } break;
        case OGPS_FloatPointType : {
           float zf = ogps_GetFloatZ(vector);
           std::cout << zf;
                                   } break;
        case OGPS_DoublePointType : {
           double zd = ogps_GetDoubleZ(vector);
           std::cout << zd;
                                    } break;
         }

         std::cout << std::endl;
      }
      else
      {
         std::cout << "X; Y; Z = MISSING" << std::endl;
      }
   }

   std::cout << std::endl;

   // Failed?
   assert(!ogps_HasNextPoint(iterator));

   /* Free iterator/buffer */
   ogps_FreePointIterator(&iterator);
   ogps_FreePointVector(&vector);

   /* Close file */
   ogps_CloseISO5436_2(&handle);
}

void readonlyExample3(const OpenGPS::String fileName)
{
   /* We want to read in some file and read its point data. */
   std::cout << endl << endl << "readonlyExample3(\"" << fileName << "\")" << endl;
   /* Same as above but with C++ Interface. */

   /* Open the file, hopefully everything went well... */
   OpenGPS::ISO5436_2 iso5436_2(fileName);
   // Check for error opening
   if(ogps_HasError())
   {
     std::cerr << "Error opening file \"" << fileName << "\"" << endl;
     return;
   }

   // Try to open in read only mode
   try
   {
     iso5436_2.Open(TRUE);
   }
   catch(OpenGPS::Exception &e)
   {
     OpenGPS::String err=e.details();
     std::cerr << "Error opening file \"" << fileName << "\"" << endl
               << err << endl;
     return;
   }

   /* Obtain handle to xml document. */
   const OpenGPS::Schemas::ISO5436_2::ISO5436_2Type* const document = iso5436_2.GetDocument();

   if(document)
   {
     /* Print meta data */
     PrintMetaData(document);
      /* Is data list? / Is matrix? - don't care; we use point iterator. */

      /* Use iterator to create points in this example. */
      OpenGPS::PointIteratorAutoPtr iterator = iso5436_2.CreateNextPointIterator();

      /* Iterate point data (ignoring if they were stored
      * in xml directly or in external binary file). */
      OpenGPS::PointVector vector;
      while(iterator->MoveNext())
      {
         /* Get points at current position. */
         iterator->GetCurrent(vector);

         std::cout << "X; Y; Z = ";

         /* Valid data point (not missing)?  */
         if(vector.IsValid())
         {
            switch(vector.GetX()->GetPointType()) {
        case OGPS_Int16PointType : {
           short xs;
           vector.GetX()->Get(&xs);
           std::cout << xs;
                                   } break;
        case OGPS_Int32PointType : {
           int xi;
           vector.GetX()->Get(&xi);
           std::cout << xi;
                                   } break;
        case OGPS_FloatPointType : {
           float xf;
           vector.GetX()->Get(&xf);
           std::cout << xf;
                                   } break;
        case OGPS_DoublePointType : {
           double xd;
           vector.GetX()->Get(&xd);
           std::cout << xd;
                                    } break;
        default: {
           /* incremental axis without data type */
                 } break;
            }

            std::cout << "; ";

            switch(vector.GetY()->GetPointType()) {
        case OGPS_Int16PointType : {
           short ys;
           vector.GetY()->Get(&ys);
           std::cout << ys;
                                   } break;
        case OGPS_Int32PointType : {
           int yi;
           vector.GetY()->Get(&yi);
           std::cout << yi;
                                   } break;
        case OGPS_FloatPointType : {
           float yf;
           vector.GetY()->Get(&yf);
           std::cout << yf;
                                   } break;
        case OGPS_DoublePointType : {
           double yd;
           vector.GetY()->Get(&yd);
           std::cout << yd;
                                    } break;
        default: {
           /* incremental axis without data type */
                 } break;
            }

            std::cout << "; ";

            switch(vector.GetZ()->GetPointType()) {
        case OGPS_Int16PointType : {
           short zs;
           vector.GetZ()->Get(&zs);
           std::cout << zs;
                                   } break;
        case OGPS_Int32PointType : {
           int zi;
           vector.GetZ()->Get(&zi);
           std::cout << zi;
                                   } break;
        case OGPS_FloatPointType : {
           float zf;
           vector.GetZ()->Get(&zf);
           std::cout << zf;
                                   } break;
        case OGPS_DoublePointType : {
           double zd;
           vector.GetZ()->Get(&zd);
           std::cout << zd;
                                    } break;
        default: {
           /* missing - should not happen. Z axis must be present. */
                 } break;
            }

            std::cout << std::endl;
         }
         else
         {
            std::cout << "X; Y; Z = MISSING" << std::endl;
         }
      }

      std::cout << std::endl;

      // Failed?
      assert(!iterator->HasNext());

   }

   /* Close file */
   iso5436_2.Close();
}

void readonlyExample4(const OpenGPS::String fileName)
{
   /* We want to read in some file and read its point data. */
   std::cout << endl << endl << "readonlyExample4(\"" << fileName << "\")" << endl;

   /* Open the file, hopefully everything went well... */
   OpenGPS::ISO5436_2 iso5436_2(fileName);
   // Check for error opening
   if(ogps_HasError())
   {
     std::cerr << "Error opening file \"" << fileName << "\"" << endl;
     return;
   }

   // Try to open in read only mode
   try
   {
     iso5436_2.Open(TRUE);
   }
   catch(OpenGPS::Exception &e)
   {
     OpenGPS::String err=e.details();
     std::cerr << "Error opening file \"" << fileName << "\"" << endl
               << err << endl;
     return;
   }

   if(!ogps_HasError())
   {
      /* Obtain handle to xml document. */
      const OpenGPS::Schemas::ISO5436_2::ISO5436_2Type* const document = iso5436_2.GetDocument();
      if(document)
      {
        /* Print meta data */
        PrintMetaData(document);

        /* Is data list? / Is matrix? */
        if(document->Record3().ListDimension().present())
        {
            #if linux
                assert(document->Record3().ListDimension().present());
            #else
                _ASSERT(document->Record3().ListDimension().present());
            #endif

            OpenGPS::PointVector vector;

            // safe cast, since values greater than 32bit are not supported by the ISO5436-2 XML specification.
            const OGPS_ULong maxIndex = (OGPS_ULong)document->Record3().ListDimension().get();
            for(OGPS_ULong index = 0; index < maxIndex; ++index)
            {
               iso5436_2.GetListPoint(index, vector);

               if(ogps_HasError())
               {
                  break;
               }

               iso5436_2.SetListPoint(index, vector);

               if(ogps_HasError())
               {
                  break;
               }

               if(vector.IsValid())
               {
                  double x, y, z;
                  vector.GetXYZ(&x, &y, &z);

                  std::cout << ("N: ") << index << ("X: ") << x << ("; Y: ") << y << ("; Z: ") << z << std::endl;
               }
            }
         }
      }
      iso5436_2.Close();
   }
}


/** Performance test using Int16 z-Data.
    Simple writing an unsorted points cloud. So all axis have to be absolute.
**/
void performanceInt16(OpenGPS::String fileName, OGPS_ULong dimension, OGPS_Boolean binary)
{
   std::cout << endl << endl << "performanceInt16(\"" << fileName << "\")" << endl;

   // Timer
   clock_t start = clock();

   /* Create RECORD1 */
   Record1Type::Revision_type revision(OGPS_ISO5436_2000_REVISION_NAME);
   // A list has to be stored as type point cloud
   Record1Type::FeatureType_type featureType(OGPS_FEATURE_TYPE_POINTCLOUD_NAME);

   Record1Type::Axes_type::CX_type::AxisType_type xaxisType(Record1Type::Axes_type::CX_type::AxisType_type::A); /* absolute */
   Record1Type::Axes_type::CX_type::DataType_type xdataType(Record1Type::Axes_type::CX_type::DataType_type::L); /* int32 */
   Record1Type::Axes_type::CX_type xaxis(xaxisType);
   xaxis.DataType(xdataType);
   /* Increment and offset of x and y are needed to scale the integer values to metric coordinates */
   // The x axis is scaled in units of 10 micrometer
   xaxis.Increment(10E-6);
   xaxis.Offset(0.0);

   Record1Type::Axes_type::CY_type::AxisType_type yaxisType(Record1Type::Axes_type::CY_type::AxisType_type::A); /* absolute */
   Record1Type::Axes_type::CY_type::DataType_type ydataType(Record1Type::Axes_type::CY_type::DataType_type::I); /* int16 */
   Record1Type::Axes_type::CY_type yaxis(yaxisType);
   yaxis.DataType(ydataType);
   // The y axis is scaled in units of 10 micrometer
   yaxis.Increment(10E-6);
   yaxis.Offset(0.0);

   Record1Type::Axes_type::CZ_type::AxisType_type zaxisType(Record1Type::Axes_type::CZ_type::AxisType_type::A); /* absolute */
   Record1Type::Axes_type::CZ_type::DataType_type zdataType(Record1Type::Axes_type::CZ_type::DataType_type::I); /* int16 */
   Record1Type::Axes_type::CZ_type zaxis(zaxisType);
   zaxis.DataType(zdataType);
   zaxis.Increment(1e-6);
   zaxis.Offset(-32768e-6);
   Record1Type::Axes_type axis(xaxis, yaxis, zaxis);

   Record1Type record1(revision, featureType, axis);

   /* Create RECORD2 */
//   Record2Type::Date_type date(TimeStamp());
   Record2Type::Date_type date(TimeStamp(), 0);

   Record2Type::Instrument_type::Manufacturer_type manufacturer(_T("NanoFocus AG"));
   Record2Type::Instrument_type::Model_type model(_T("ISO5436_2_XML_Demo Software"));
   Record2Type::Instrument_type::Serial_type serial(_T("not available"));
   Record2Type::Instrument_type::Version_type version(_OPENGPS_VERSIONSTRING);
   Record2Type::Instrument_type instrument(manufacturer, model, serial, version);

//   Record2Type::CalibrationDate_type calibrationDate(_T("2007-04-30T13:58:02.6+02:00"));
   Record2Type::CalibrationDate_type calibrationDate(_T("2007-04-30T13:58:02.6+02:00"), 0);

   Record2Type::ProbingSystem_type::Type_type type(Record2Type::ProbingSystem_type::Type_type::Software);
   Record2Type::ProbingSystem_type::Identification_type id(_T("Random number generator"));
   Record2Type::ProbingSystem_type probingSystem(type, id);

   Record2Type::Comment_type comment(_T("This file is a list of points written as performance test in int16 precision."));

   Record2Type record2(date, instrument, calibrationDate, probingSystem);
   record2.Comment(comment);

   /* Create ISO5436_2 container */
   OGPS_ISO5436_2Handle handle = ogps_CreateListISO5436_2(fileName.c_str(), NULL, record1, &record2, dimension, binary);

   /* Add data points */

   /* Create point vector buffer for three points. */
   OGPS_PointVectorPtr vector = ogps_CreatePointVector();

   /* Create/write random number. */
   srand((unsigned)time(0));

   for(OGPS_ULong n = 0; n < dimension; ++n)
   {
      /* Generate random number */
      short randomz = (short)((rand() - (RAND_MAX/2)) % std::numeric_limits<short>::max());
      short randomy = (short)((rand() - (RAND_MAX/2)) % std::numeric_limits<short>::max());
      ogps_SetInt32X(vector,n - (dimension/2));
      ogps_SetInt16Y(vector, randomy);
      ogps_SetInt16Z(vector, randomz);

      /* Write into document */
      ogps_SetListPoint(handle, n, vector);
   }

   /* Free buffer */
   ogps_FreePointVector(&vector);

   /* Print meta data */
   PrintMetaData(handle);
   // Print dimensions
   PrintDimensions(handle);

   /* Finally: write container to disk. */
   ogps_WriteISO5436_2(handle);
   ogps_CloseISO5436_2(&handle);

   // Timer
   clock_t stop = clock();

   std::cout << std::endl << "Writing an X3P list file containing " << dimension
             << " points in int16 " << (binary ? "binary" : "xml")
             << " format took " << (((double)(stop - start))/CLOCKS_PER_SEC)
             << " seconds." << std::endl;
}

void performanceDouble(OpenGPS::String fileName, OGPS_ULong dimension, OGPS_Boolean binary)
{
   std::cout << endl << endl << "performanceDouble(\"" << fileName << "\")" << endl;

   // Timer
   clock_t start = clock();

   /* Simple example where we have two incremental and one absolute axis (the z-axis). */

   /* Create RECORD1 */
   Record1Type::Revision_type revision(OGPS_ISO5436_2000_REVISION_NAME);
   Record1Type::FeatureType_type featureType(OGPS_FEATURE_TYPE_PROFILE_NAME);

   Record1Type::Axes_type::CX_type::AxisType_type xaxisType(Record1Type::Axes_type::CX_type::AxisType_type::I); /* incremental */
   Record1Type::Axes_type::CX_type::DataType_type xdataType(Record1Type::Axes_type::CX_type::DataType_type::I); /* int16 */
   Record1Type::Axes_type::CX_type xaxis(xaxisType);
   xaxis.DataType(xdataType);
   /* A profile is still 3D. so increment and offset of x and y have a meaning, but both are incremented from the same index! */
   // This example is a profile along x-axis
   xaxis.Increment(10E-6);
   // Test very small numbers
   xaxis.Offset(1.12345678912345678912345e-19);

   Record1Type::Axes_type::CY_type::AxisType_type yaxisType(Record1Type::Axes_type::CY_type::AxisType_type::I); /* incremental */
   Record1Type::Axes_type::CY_type::DataType_type ydataType(Record1Type::Axes_type::CY_type::DataType_type::I); /* int16 */
   Record1Type::Axes_type::CY_type yaxis(yaxisType);
   yaxis.DataType(ydataType);
   // Increment defaults to 1.
   yaxis.Increment(1);
   yaxis.Offset(0.0);

   Record1Type::Axes_type::CZ_type::AxisType_type zaxisType(Record1Type::Axes_type::CZ_type::AxisType_type::A); /* absolute */
   Record1Type::Axes_type::CZ_type::DataType_type zdataType(Record1Type::Axes_type::CZ_type::DataType_type::D); /* double */
   Record1Type::Axes_type::CZ_type zaxis(zaxisType);
   zaxis.DataType(zdataType);

   Record1Type::Axes_type axis(xaxis, yaxis, zaxis);

   Record1Type record1(revision, featureType, axis);

   /* Create RECORD2 */
//   Record2Type::Date_type date(TimeStamp());
   Record2Type::Date_type date(TimeStamp(), 0);

   Record2Type::Instrument_type::Manufacturer_type manufacturer(_T("NanoFocus AG"));
   Record2Type::Instrument_type::Model_type model(_T("ISO5436_2_XML_Demo Software"));
   Record2Type::Instrument_type::Serial_type serial(_T("not available"));
   Record2Type::Instrument_type::Version_type version(_OPENGPS_VERSIONSTRING);
   Record2Type::Instrument_type instrument(manufacturer, model, serial, version);

//   Record2Type::CalibrationDate_type calibrationDate(_T("2007-04-30T13:58:02.6+02:00"));
   Record2Type::CalibrationDate_type calibrationDate(_T("2007-04-30T13:58:02.6+02:00"), 0);

   Record2Type::ProbingSystem_type::Type_type type(Record2Type::ProbingSystem_type::Type_type::Software);
   Record2Type::ProbingSystem_type::Identification_type id(_T("Random number generator"));
   Record2Type::ProbingSystem_type probingSystem(type, id);

   Record2Type::Comment_type comment(_T("This file is a line profile written as performance test in double precision."));

   Record2Type record2(date, instrument, calibrationDate, probingSystem);
   record2.Comment(comment);

   // Set dimensions
   OpenGPS::Schemas::ISO5436_2::MatrixDimensionType mdim(dimension,1,1);

   /* Create ISO5436_2 container */
   // OGPS_ISO5436_2Handle handle = ogps_CreateListISO5436_2(fileName.c_str(), NULL, record1, &record2, dimension, binary);
   OGPS_ISO5436_2Handle handle = ogps_CreateMatrixISO5436_2(fileName.c_str(), NULL, record1, &record2, mdim, binary);

   /* Add data points */

   /* Create point vector buffer for three points. */
   OGPS_PointVectorPtr vector = ogps_CreatePointVector();

   /* Create/write random number. */
   srand((unsigned)time(0));

   for(OGPS_ULong n = 0; n < dimension; ++n)
   {
      /* Generate random number */
      double random = rand() * (1.0 / (double)RAND_MAX);
      ogps_SetDoubleZ(vector, random);

      /* Write into document */
      // Profiles have to be saved as a matrix of size n,1,m
      // So it would be wrong to use list format here
      // ogps_SetListPoint(handle, n, vector);
      ogps_SetMatrixPoint(handle,n,0,0,vector);
   }

   /* Free buffer */
   ogps_FreePointVector(&vector);

   /* Print meta data */
   PrintMetaData(handle);
   // Print dimensions
   PrintDimensions(handle);

   /* Finally: write container to disk. */
   ogps_WriteISO5436_2(handle);
   ogps_CloseISO5436_2(&handle);

   // Timer
   clock_t stop = clock();

   std::cout << std::endl << "Writing an X3P file containing " << dimension
             << " points in double " << (binary ? "binary" : "xml")
             << " format took " << (((double)(stop - start))/CLOCKS_PER_SEC)
             << " seconds." << std::endl << std::endl;
}

/* Converts a given X3P file either to binary or text format (if dstFormatIsBinary parameter equals FALSE). */
void convertFormat(OpenGPS::String srcFileName, OpenGPS::String dstFileName, const OGPS_Boolean dstFormatIsBinary)
{
   std::cout << endl << endl << "convertFormat(\"" << srcFileName << "\") to \"" << dstFileName << "\"" << endl;
   /* We want to read in some file regardless whether it was written
   in text or binary format and then copy its contents into
   a newly created file to ensure binary format of X3P.
   The original file will be overwritten! */

   /* Open the file, hopefully everything went well... */
   OGPS_ISO5436_2Handle src_handle = ogps_OpenISO5436_2(srcFileName.c_str(), NULL, TRUE);

   if(!src_handle)
      return;

   /* Print meta data */
   PrintMetaData(src_handle);
   // Print dimensions
   PrintDimensions(src_handle);

   /* Obtain handle to xml document for reading. */
   const ISO5436_2Type * const document = ogps_GetDocument(src_handle);

   /* Check whether it is matrix or list format and allocate the new file handle accordingly. */
   OGPS_ISO5436_2Handle dst_handle = NULL;
   if(ogps_IsMatrix(src_handle) && !ogps_HasError())
   {
      dst_handle = ogps_CreateMatrixISO5436_2(dstFileName.c_str(), NULL, document->Record1(), document->Record2().present() ? &*document->Record2() : NULL, *document->Record3().MatrixDimension(), dstFormatIsBinary);
   }
   else
   {
      dst_handle = ogps_CreateListISO5436_2(dstFileName.c_str(), NULL, document->Record1(), document->Record2().present() ? &*document->Record2() : NULL, *document->Record3().ListDimension(), dstFormatIsBinary);
   }

   if(dst_handle)
   {
      /* Create point buffer. */
      OGPS_PointVectorPtr vector = ogps_CreatePointVector();

      /* Use iterator to copy point vectors. */
      OGPS_PointIteratorPtr src = ogps_CreateNextPointIterator(src_handle);
      OGPS_PointIteratorPtr dst = ogps_CreateNextPointIterator(dst_handle);

      /* Iterate point data and copy. */
      while(ogps_MoveNextPoint(src) && ogps_MoveNextPoint(dst))
      {
         /* Get vector at current position from source. */
         ogps_GetCurrentPoint(src, vector);

         if(!ogps_HasError())
         {
            /* Copy vector to destination. */
            ogps_SetCurrentPoint(dst, (!ogps_IsValidPoint(vector) || ogps_HasError()) ? NULL : vector);
         }

         /* Errors? */
         if(ogps_HasError())
         {
            break;
         }
      }

      // Failed?
      assert(!ogps_HasNextPoint(src));
      assert(!ogps_HasNextPoint(dst));

      /* Free iterator/buffer */
      ogps_FreePointIterator(&src);
      ogps_FreePointIterator(&dst);
      ogps_FreePointVector(&vector);
   }

   /* Close source file */
   ogps_CloseISO5436_2(&src_handle);

   if(dst_handle)
   {
      /* Write new file. */
      ogps_WriteISO5436_2(dst_handle);
      ogps_CloseISO5436_2(&dst_handle);
   }
}

#if linux
    int main(int argc, char* argv[])
#else
    int _cdecl _tmain(int argc, _TCHAR* argv[])
#endif
{
  // Number of points to generate for performance counter
  const unsigned int performanceCounter=1000;
  if(argc != 2)
  {
    std::cout << "Usage: ISO5436_2_XML_demo <full path to sample files>/" << std::endl << std::endl <<
       "Please specify the full path to the directory where the *.x3p sample files reside. The path should also contain the terminating directory separator. Ensure that you have write access to that path." << std::endl << std::endl <<
       "This simple demo program parses the sample files and prints its contents onto the console. Do not change the names of the sample files, since these are hard coded herein. The purpose of the demo is to get you familiar with the openGPS(R) API." << std::endl;
    return 1;
  }

  // Print version and copyright information
  OpenGPS::String LibName;
  OpenGPS::String LibVersion;
  OpenGPS::String LibAbout;

  // get Info strings from Library
  OpenGPS::Info::GetName(&LibName);
  OpenGPS::Info::GetVersion(&LibVersion);
  OpenGPS::Info::GetAbout(&LibAbout);

  std::cout << "This programm uses the Library \"" << LibName
            << "\" Version " << LibVersion << endl
            << LibAbout << endl;
  OpenGPS::Info::PrintVersion();
  OpenGPS::Info::PrintCopyright();
  OpenGPS::Info::PrintLicense();

#if linux
  OpenGPS::String wpath;
  wpath.FromChar(argv[1]);
  std::wstring path = wpath;
#else
  std::wstring path = argv[1];
#endif
  std::wstring tmp;

  tmp = path; tmp += _T("ISO5436-sample1.x3p");
  readonlyExample(tmp);

  tmp = path; tmp += _T("ISO5436-sample4_bin.x3p");
  readonlyExample2(tmp);

  tmp = path; tmp += _T("ISO5436-sample3.x3p");
  readonlyExample2(tmp);

  tmp = path; tmp += _T("ISO5436-sample3.x3p");
  readonlyExample3(tmp);

  tmp = path; tmp += _T("ISO5436-sample2.x3p");
  readonlyExample4(tmp);

  tmp = path; tmp += _T("simple.x3p");
  simpleExample(tmp);

  tmp = path; tmp += _T("medium.x3p");
  mediumComplexExample(tmp);

  tmp = path; tmp += _T("medium_invalid_points.x3p");
  mediumComplexExampleWInvalid(tmp);

  std::cout << std::endl << "Starting performance tests..." << std::endl;

  tmp = path; tmp += _T("performance_int16_bin.x3p");
  performanceInt16(tmp, performanceCounter, TRUE);

  tmp = path; tmp += _T("performance_int16.x3p");
  performanceInt16(tmp, performanceCounter, FALSE);

  tmp = path; tmp += _T("performance_double_bin.x3p");
  performanceDouble(tmp, performanceCounter, TRUE);

  tmp = path; tmp += _T("performance_double.x3p");
  performanceDouble(tmp, performanceCounter, FALSE);

  return 0;
}

/* ********************************************************************
    Plugin "x3pio" for itom software
    URL: http://www.twip-os.com
    Copyright (C) 2013, twip optical solutions GmbH
    Copyright (C) 2013, Institut für Technische Optik, Universität Stuttgart

    This file is part of a plugin for the measurement software itom.

    This itom-plugin is free software; you can redistribute it and/or modify it
    under the terms of the GNU Library General Public Licence as published by
    the Free Software Foundation; either version 2 of the Licence, or (at
    your option) any later version.

    itom and its plugins are distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Library
    General Public Licence for more details.

    You should have received a copy of the GNU Library General Public License
    along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include "x3pio.h"
#include "DataObject/dataobj.h"
//#include "../../common/helperCommon.h"
#include "pluginVersion.h"
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

#include <string.h>
#include <time.h>
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

#include <QtCore/QtPlugin>
#include <qvariant.h>
#include <QtGui>
#include <string.h>
#include <qapplication.h>

using namespace OpenGPS::Schemas::ISO5436_2;

/*!
  @brief Helper function to return the current time properly formated.

  @return A string containing the time stamp from now.

  @note There is only a windows implementation yet. In other cases
  return a dummy. That is enough for testing purposes.
*/
#ifndef linux
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
  sout << std::setfill(_T('0')) << std::setw(4) << (lt.tm_year+1900) << _T('-') << std::setw(2) << lt.tm_mon << _T('-') << std::setw(2) << lt.tm_mday
      << _T('T') << std::setw(2) << lt.tm_hour << _T(':') << std::setw(2) << lt.tm_min << _T(':') << std::setw(2) << lt.tm_sec << _T(".0")
      << tzoffsign << std::setw(2) << tzoff_habs << _T(':') << std::setw(2) << tzoff_m;
  //return _T("2007-04-30T13:58:02.6+02:00");
  return sout.str();
}
#else
// There is only a windows implementation yet.
//In other cases return a dummy. That is enough for testing purposes.
OpenGPS::String TimeStamp(void)
{
//    time_t ltime;
//    struct tm lt;
    // Time zone offset
    long tzoff;

    // Set timezone
    tzset();
    // Get time zone offset
    time_t tm1, tm2;
    struct tm *lt, *t2;
    tm1 = time(NULL);
    t2 = gmtime(&tm1);
    tm2 = mktime(t2);
    lt = localtime(&tm1);
    tzoff = (long)(tm1 - tm2);
//    get_timezone(&tzoff);
    // Offset ours and minutes
    int tzoff_h,tzoff_m;
    tzoff_h = -(int)floor(((double)tzoff)/3600.);
    tzoff_m = -(int)floor(((double)tzoff)/60. + 0.5) - tzoff_h*60;

    // Get current time
//    time( &ltime );
//    // get local time
//    localtime_s(&lt,&ltime);
//
//    // Correct tz offset by dst
//    if (lt.tm_isdst > 0)
//      tzoff_h++;

    // Absolute offset for printing
    int tzoff_habs = abs(tzoff_h);
    OGPS_Character tzoffsign = tzoff_h<0 ? _T('-') : _T('+');

    // Create a string of pattern "2007-04-30T13:58:02.6+02:00"
    wostringstream sout;
    sout << std::setfill(_T('0')) << std::setw(4) << (lt->tm_year+1900) << _T('-') << std::setw(2) << lt->tm_mon << _T('-') << std::setw(2) << lt->tm_mday
        << _T('T') << std::setw(2) << lt->tm_hour << _T(':') << std::setw(2) << lt->tm_min << _T(':') << std::setw(2) << lt->tm_sec << _T(".0")
        << tzoffsign << setw(2) << tzoff_habs << _T(':') << setw(2) << tzoff_m;

    std::basic_string<wchar_t> s = sout.str();
    return sout.str();
}
#endif

//----------------------------------------------------------------------------------------------------------------------------------
OpenGPS::String TimeStamp( ::xml_schema::date_time &dateTime)
{
    wostringstream sout;
    sout << std::setfill(_T('0')) << std::setw(4) << (dateTime.year()) << _T('-') << std::setw(2) << dateTime.month() << _T('-') << std::setw(2) << dateTime.day()
        << _T('T') << std::setw(2) << dateTime.hours() << _T(':') << std::setw(2) << dateTime.minutes() << _T(':') << std::setw(2) << dateTime.seconds() << _T(".0");
    if (dateTime.zone_hours() == 0)
       sout << L"+";
    else
       sout << L"-";

    sout << std::setw(2) << abs(dateTime.zone_hours()) << _T(':') << std::setw(2) << dateTime.zone_minutes();

    return sout.str();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal X3pIOInterface::getAddInInst(ito::AddInBase **addInInst)
{
    X3pIO* newInst = new X3pIO();
    newInst->setBasePlugin(this);
    *addInInst = qobject_cast<ito::AddInBase*>(newInst);
    QList<QString> keyList = newInst->m_filterList.keys();
    for (int i = 0; i < newInst->m_filterList.size(); i++)
    {
        newInst->m_filterList[keyList[i]]->m_pBasePlugin = this;
    }

    m_InstList.append(*addInInst);

    //parse lib path:
    QDir appLibPath = QApplication::applicationDirPath();
    if(appLibPath.exists("lib"))
    {
        appLibPath.cd("lib");
    }
    else
    {
        appLibPath.cdUp();
        appLibPath.cd("lib");
    }
    QString libDir = QDir::cleanPath(appLibPath.filePath(""));

    char *newpath = NULL;
    newpath = (char*)calloc(libDir.size() + 30, sizeof(char));
    strcat(newpath, libDir.toLatin1().data());
    strcat(newpath, "/");
#if (defined WIN32 | defined _WIN64)
    QString envVar = QString("OPENGPS_LOCATION=%1/").arg( libDir );
    QByteArray ba = envVar.toLatin1();
    _putenv(ba.data());
#else
    setenv("OPENGPS_LOCATION", newpath, 1);
#endif
    free(newpath);

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal X3pIOInterface::closeThisInst(ito::AddInBase **addInInst)
{
    if (*addInInst)
    {
        delete ((X3pIO *)*addInInst);
        int idx = m_InstList.indexOf(*addInInst);
        m_InstList.removeAt(idx);
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
X3pIOInterface::X3pIOInterface()
{
    m_type = ito::typeAlgo;
    setObjectName("x3pio");

    char docstring[] = \
"This plugin provides methods to save and load dataObjects in/from the file format 'x3p'. \
This format is specified in ISO 25178 - Geometrical product specification (GPS). \n\
\n\
The library ISO 5436-2 XML, that is necessary for this plugin and included in the sources, \n\
is licensed under the LGPL license and uses further libraries. For more information about the license \n\
of the library itself see www.opengps.eu";

    m_description = QObject::tr("x3p Import/Export");
    m_detaildescription = QObject::tr(docstring);
    m_author = "C. Kohler, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("LGPL / ISO5436-2 XML under LPGL");
    m_aboutThis = QObject::tr("N.A.");
}

//----------------------------------------------------------------------------------------------------------------------------------
X3pIOInterface::~X3pIOInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
#if QT_VERSION < 0x050000
    Q_EXPORT_PLUGIN2(X3pIOInterface, X3pIOInterface)
#endif

//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
X3pIO::X3pIO() : AddInAlgo()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
X3pIO::~X3pIO()
{
    FilterDef *filter;
    foreach(filter, m_filterList)
    {
        delete filter;
    }
    m_filterList.clear();
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal X3pIO::init(QVector<ito::ParamBase> * /*paramsMand*/, QVector<ito::ParamBase> * /*paramsOpt*/, ItomSharedSemaphore * /*waitCond*/)
{
    ito::RetVal retval = ito::retOk;
    FilterDef *filter = NULL;

    filter = new FilterDef(X3pIO::saveDObj, X3pIO::saveDObjParams, QObject::tr("saves dataObject to x3p file"), ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iWriteDataObject, tr("X3P Files (*.x3p)"));
    m_filterList.insert("saveX3p", filter);

    filter = new FilterDef(X3pIO::loadDObj, X3pIO::loadDObjParams, QObject::tr("loads dataObject from x3p file"), ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iReadDataObject, tr("X3P Files (*.x3p)"));
    m_filterList.insert("loadX3p", filter);

    setInitialized(true); //init method has been finished (independent on retval)
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal X3pIO::saveDObjParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(!retval.containsError())
    {
        ito::Param param = ito::Param("sourceObject", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("DataObject").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("Destination filename").toLatin1().data());
        paramsMand->append(param);
        paramsOpt->clear();
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal fillDataMatrix(OGPS_ISO5436_2Handle &handle, ito::DataObject *dObj)
{
    // Add data points
    // 1. Create point vector buffer for three points.
    OGPS_PointVectorPtr vector = ogps_CreatePointVector();
    unsigned char *pDataRow = NULL;

    switch (dObj->getType())
    {
        case ito::tUInt8:
        case ito::tInt8:
            for (int nMat = 0; nMat < dObj->calcNumMats(); nMat++)
            {
                for (int y = 0; y < dObj->getSize(dObj->getDims() - 2); y++)
                {
                    pDataRow = (((cv::Mat *)dObj->get_mdata()[dObj->seekMat(nMat)])->ptr(y));
                    for (int x = 0; x < dObj->getSize(dObj->getDims() - 1); x++)
                    {
                        ogps_SetInt16Z(vector, ((ito::uint8*)pDataRow)[x]);
                        ogps_SetMatrixPoint(handle, x, y, nMat, vector);

                        if(ogps_HasError())
                        {
                            return ito::RetVal(ito::retError, 0, QObject::tr("error writing is5436_2 file").toLatin1().data());
                        }
                    }
                }
            }
        break;

        case ito::tUInt16:
        case ito::tInt16:
            for (int nMat = 0; nMat < dObj->calcNumMats(); nMat++)
            {
                for (int y = 0; y < dObj->getSize(dObj->getDims() - 2); y++)
                {
                    pDataRow = (((cv::Mat *)dObj->get_mdata()[dObj->seekMat(nMat)])->ptr(y));
                    for (int x = 0; x < dObj->getSize(dObj->getDims() - 1); x++)
                    {
                        ogps_SetInt16Z(vector, ((ito::uint16*)pDataRow)[x]);
                        ogps_SetMatrixPoint(handle, x, y, nMat, vector);

                        if(ogps_HasError())
                        {
                            return ito::RetVal(ito::retError, 0, QObject::tr("error writing is5436_2 file").toLatin1().data());
                        }
                    }
                }
            }
        break;

        case ito::tUInt32:
        case ito::tInt32:
            for (int nMat = 0; nMat < dObj->calcNumMats(); nMat++)
            {
                for (int y = 0; y < dObj->getSize(dObj->getDims() - 2); y++)
                {
                    pDataRow = (((cv::Mat *)dObj->get_mdata()[dObj->seekMat(nMat)])->ptr(y));
                    for (int x = 0; x < dObj->getSize(dObj->getDims() - 1); x++)
                    {
                        ogps_SetInt32Z(vector, ((ito::uint32*)pDataRow)[x]);
                        ogps_SetMatrixPoint(handle, x, y, nMat, vector);

                        if(ogps_HasError())
                        {
                            return ito::RetVal(ito::retError, 0, QObject::tr("error writing is5436_2 file").toLatin1().data());
                        }
                    }
                }
            }
        break;

        case ito::tFloat32:
            for (int nMat = 0; nMat < dObj->calcNumMats(); nMat++)
            {
                for (int y = 0; y < dObj->getSize(dObj->getDims() - 2); y++)
                {
                    pDataRow = (((cv::Mat *)dObj->get_mdata()[dObj->seekMat(nMat)])->ptr(y));
                    for (int x = 0; x < dObj->getSize(dObj->getDims() - 1); x++)
                    {
                        ogps_SetFloatZ(vector, ((ito::float32*)pDataRow)[x]);
                        ogps_SetMatrixPoint(handle, x, y, nMat, vector);

                        if(ogps_HasError())
                        {
                            return ito::RetVal(ito::retError, 0, QObject::tr("error writing is5436_2 file").toLatin1().data());
                        }
                    }
                }
            }
        break;

        case ito::tFloat64:
            for (int nMat = 0; nMat < dObj->calcNumMats(); nMat++)
            {
                for (int y = 0; y < dObj->getSize(dObj->getDims() - 2); y++)
                {
                    pDataRow = (((cv::Mat *)dObj->get_mdata()[dObj->seekMat(nMat)])->ptr(y));
                    for (int x = 0; x < dObj->getSize(dObj->getDims() - 1); x++)
                    {
                        ogps_SetDoubleZ(vector, ((ito::float64*)pDataRow)[x]);
                        ogps_SetMatrixPoint(handle, x, y, nMat, vector);

                        if(ogps_HasError())
                        {
                            return ito::RetVal(ito::retError, 0, QObject::tr("error writing is5436_2 file").toLatin1().data());
                        }
                    }
                }
            }
        break;

        case ito::tComplex64:
            for (int nMat = 0; nMat < dObj->calcNumMats(); nMat++)
            {
                for (int y = 0; y < dObj->getSize(dObj->getDims() - 2); y++)
                {
                    pDataRow = (((cv::Mat *)dObj->get_mdata()[dObj->seekMat(nMat)])->ptr(y));
                    for (int x = 0; x < dObj->getSize(dObj->getDims() - 1); x++)
                    {
                        ogps_SetFloatZ(vector, ((ito::complex64*)pDataRow)[x].real());
                        ogps_SetMatrixPoint(handle, x, y, nMat * 2, vector);
                        ogps_SetFloatZ(vector, ((ito::complex64*)pDataRow)[x].imag());
                        ogps_SetMatrixPoint(handle, x, y, nMat * 2 + 1, vector);

                        if(ogps_HasError())
                        {
                            return ito::RetVal(ito::retError, 0, QObject::tr("error writing is5436_2 file").toLatin1().data());
                        }
                    }
                }
            }
        break;

        case ito::tComplex128:
            for (int nMat = 0; nMat < dObj->calcNumMats(); nMat++)
            {
                for (int y = 0; y < dObj->getSize(dObj->getDims() - 2); y++)
                {
                    pDataRow = (((cv::Mat *)dObj->get_mdata()[dObj->seekMat(nMat)])->ptr(y));
                    for (int x = 0; x < dObj->getSize(dObj->getDims() - 1); x++)
                    {
                        ogps_SetDoubleZ(vector, ((ito::complex128*)pDataRow)[x].real());
                        ogps_SetMatrixPoint(handle, x, y, nMat * 2, vector);
                        ogps_SetDoubleZ(vector, ((ito::complex128*)pDataRow)[x].imag());
                        ogps_SetMatrixPoint(handle, x, y, nMat * 2 + 1, vector);

                        if(ogps_HasError())
                        {
                            return ito::RetVal(ito::retError, 0, QObject::tr("error writing is5436_2 file").toLatin1().data());
                        }
                    }
                }
            }
        break;
    }

    // Free buffer
    ogps_FreePointVector(&vector);

   // Quickfix: do a dummy write with an invalid value to create ValidtyBuffer ...
   // Bug should be fixed, so this code should no longer be necessary. Ck 02/05/2012
   /*
    if ((dObj->getType() == ito::tUInt8) ||
       (dObj->getType() == ito::tInt8) ||
       (dObj->getType() == ito::tUInt16) ||
       (dObj->getType() == ito::tInt16) ||
       (dObj->getType() == ito::tUInt32) ||
       (dObj->getType() == ito::tUInt32))
    {
       OGPS_PointVectorPtr vector = ogps_CreatePointVector();
       OGPS_PointIteratorPtr iterator = ogps_CreateNextPointIterator(handle);
       ogps_SetCurrentPoint(iterator, NULL);
       ogps_FreePointVector(&vector);
       ogps_FreePointIterator(&iterator);
    }
   */
    return 0;
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal loadDataMatrix(OpenGPS::ISO5436_2 &iso5436_2, _OGPS_DATA_POINT_TYPE &pointType, ito::DataObject *dObj, double zscale, double zoffset)
{
    ito::RetVal retval;

    // Use iterator to create points in this example.
    OpenGPS::PointIteratorAutoPtr iterator = iso5436_2.CreateNextPointIterator();
    unsigned char *pDataRow = NULL;

    // Iterate point data (ignoring if they were stored
    // in xml directly or in external binary file).
    OpenGPS::PointVector vector;
    OpenGPS::DataPoint *dp = NULL;

    try
    {

        switch (pointType)
        {
            case OGPS_Int16PointType:
                switch (dObj->getType())
                {
                    case ito::tUInt8:
                        for (int nMat = 0; nMat < dObj->calcNumMats(); nMat++)
                        {
                            ito::int16 tempVal;
                            for (int y = 0; y < dObj->getSize(dObj->getDims() - 2); y++)
                            {
                                pDataRow = (((cv::Mat *)dObj->get_mdata()[dObj->seekMat(nMat)])->ptr(y));
                                for (int x = 0; x < dObj->getSize(dObj->getDims() - 1); x++)
                                {
                                    iterator->GetCurrent(vector);
                                    vector.GetZ()->Get(&tempVal);
                                    ((ito::uint8*)pDataRow)[x] = (ito::uint8)(tempVal * zscale + zoffset);
                                    iterator->MoveNext();
                                }
                            }
                        }
                    break;

                    case ito::tInt8:
                        for (int nMat = 0; nMat < dObj->calcNumMats(); nMat++)
                        {
                            ito::int16 tempVal;
                            for (int y = 0; y < dObj->getSize(dObj->getDims() - 2); y++)
                            {
                                pDataRow = (((cv::Mat *)dObj->get_mdata()[dObj->seekMat(nMat)])->ptr(y));
                                for (int x = 0; x < dObj->getSize(dObj->getDims() - 1); x++)
                                {
                                    iterator->GetCurrent(vector);
                                    vector.GetZ()->Get(&tempVal);
                                    ((ito::int8*)pDataRow)[x] = (ito::int8)(tempVal * zscale + zoffset);
                                    iterator->MoveNext();
                                }
                            }
                        }
                    break;

                    case ito::tUInt16:
                        for (int nMat = 0; nMat < dObj->calcNumMats(); nMat++)
                        {
                            ito::int16 tempVal;
                            for (int y = 0; y < dObj->getSize(dObj->getDims() - 2); y++)
                            {
                                pDataRow = (((cv::Mat *)dObj->get_mdata()[dObj->seekMat(nMat)])->ptr(y));
                                for (int x = 0; x < dObj->getSize(dObj->getDims() - 1); x++)
                                {
                                    iterator->GetCurrent(vector);
                                    vector.GetZ()->Get(&tempVal);
                                    ((ito::uint16*)pDataRow)[x] = (ito::uint16)(tempVal * zscale + zoffset);
                                    iterator->MoveNext();
                                }
                            }
                        }
                    break;

                    case ito::tInt16:
                        for (int nMat = 0; nMat < dObj->calcNumMats(); nMat++)
                        {
                            ito::int16 tempVal;
                            for (int y = 0; y < dObj->getSize(dObj->getDims() - 2); y++)
                            {
                                pDataRow = (((cv::Mat *)dObj->get_mdata()[dObj->seekMat(nMat)])->ptr(y));
                                for (int x = 0; x < dObj->getSize(dObj->getDims() - 1); x++)
                                {
                                    iterator->GetCurrent(vector);
                                    vector.GetZ()->Get(&tempVal);
                                    ((ito::int16*)pDataRow)[x] = (ito::int16)(tempVal * zscale + zoffset);
                                    iterator->MoveNext();
                                }
                            }
                        }
                    break;

                    default:
                        retval += ito::RetVal(ito::retError, 0, QObject::tr("data type mismatch").toLatin1().data());
                    break;
                }
            break;

            case OGPS_Int32PointType:
                switch (dObj->getType())
                {
                   case ito::tUInt32:
                      for (int nMat = 0; nMat < dObj->calcNumMats(); nMat++)
                      {
                          ito::int32 tempVal;
                          for (int y = 0; y < dObj->getSize(dObj->getDims() - 2); y++)
                          {
                              pDataRow = (((cv::Mat *)dObj->get_mdata()[dObj->seekMat(nMat)])->ptr(y));
                              for (int x = 0; x < dObj->getSize(dObj->getDims() - 1); x++)
                              {
                                  iterator->GetCurrent(vector);
                                  vector.GetZ()->Get(&tempVal);
                                  ((ito::uint32*)pDataRow)[x] = (ito::uint32)(tempVal * zscale + zoffset);
                                  iterator->MoveNext();
                              }
                          }
                      }
                   break;

                   case ito::tInt32:
                      for (int nMat = 0; nMat < dObj->calcNumMats(); nMat++)
                      {
                          ito::int32 tempVal;
                          for (int y = 0; y < dObj->getSize(dObj->getDims() - 2); y++)
                          {
                              pDataRow = (((cv::Mat *)dObj->get_mdata()[dObj->seekMat(nMat)])->ptr(y));
                              for (int x = 0; x < dObj->getSize(dObj->getDims() - 1); x++)
                              {
                                  iterator->GetCurrent(vector);
                                  vector.GetZ()->Get(&tempVal);
                                  ((ito::int32*)pDataRow)[x] = (ito::int32)(tempVal * zscale + zoffset);
                                  iterator->MoveNext();
                              }
                          }
                      }
                   break;

                    default:
                        retval += ito::RetVal(ito::retError, 0, QObject::tr("data type mismatch").toLatin1().data());
                    break;
                }
            break;

            case OGPS_FloatPointType:
                switch (dObj->getType())
                {
                    case ito::tFloat32:
                        for (int nMat = 0; nMat < dObj->calcNumMats(); nMat++)
                        {
                            ito::float32 tempVal;
                            for (int y = 0; y < dObj->getSize(dObj->getDims() - 2); y++)
                            {
                                pDataRow = (((cv::Mat *)dObj->get_mdata()[dObj->seekMat(nMat)])->ptr(y));
                                for (int x = 0; x < dObj->getSize(dObj->getDims() - 1); x++)
                                {
                                    iterator->GetCurrent(vector);
                                    dp = vector.GetZ();

                                    if(dp->IsValid())
                                    {
                                        vector.GetZ()->Get(&tempVal);
                                        ((ito::float32*)pDataRow)[x] = tempVal * zscale + zoffset;
                                    }
                                    else
                                    {
                                        ((ito::float32*)pDataRow)[x] = std::numeric_limits<ito::float32>::signaling_NaN();
                                    }


                                    iterator->MoveNext();
                                }
                            }
                        }


                    break;

                    case ito::tComplex64:
                        for (int nMat = 0; nMat < dObj->calcNumMats(); nMat++)
                        {
                            float tempValRe, tempValIm;
                            for (int y = 0; y < dObj->getSize(dObj->getDims() - 2); y++)
                            {
                                pDataRow = (((cv::Mat *)dObj->get_mdata()[dObj->seekMat(nMat)])->ptr(y));
                                for (int x = 0; x < dObj->getSize(dObj->getDims() - 1); x++)
                                {
                                    iso5436_2.GetMatrixPoint(x, y, nMat, vector);

                                    if(vector.IsValid())
                                    {
                                        vector.GetZ()->Get(&tempValRe);
                                        iso5436_2.GetMatrixPoint(x, y, nMat * 2, vector);
                                        vector.GetZ()->Get(&tempValIm);
                                        ((ito::complex64*)pDataRow)[x].real(tempValRe * zscale + zoffset);
                                        ((ito::complex64*)pDataRow)[x].imag(tempValIm * zscale + zoffset);
                                    }
                                    else
                                    {
                                        ((ito::complex64*)pDataRow)[x] = std::numeric_limits<ito::complex64>::signaling_NaN();
                                    }

                                    iterator->MoveNext();
                                }
                            }
                        }
                    break;

                    default:
                        retval += ito::RetVal(ito::retError, 0, QObject::tr("data type mismatch").toLatin1().data());
                    break;
                }
            break;

            case OGPS_DoublePointType:
                switch (dObj->getType())
                {
                    case ito::tFloat64:
                        for (int nMat = 0; nMat < dObj->calcNumMats(); nMat++)
                        {
                            double tempVal;
                            for (int y = 0; y < dObj->getSize(dObj->getDims() - 2); y++)
                            {
                                pDataRow = (((cv::Mat *)dObj->get_mdata()[dObj->seekMat(nMat)])->ptr(y));
                                for (int x = 0; x < dObj->getSize(dObj->getDims() - 1); x++)
                                {
                                    iterator->GetCurrent(vector);
                                    dp = vector.GetZ();

                                    if(dp->IsValid())
                                    {
                                        vector.GetZ()->Get(&tempVal);
                                        ((ito::float64*)pDataRow)[x] = tempVal * zscale + zoffset;
                                    }
                                    else
                                    {
                                        ((ito::float64*)pDataRow)[x] = std::numeric_limits<ito::float64>::signaling_NaN();
                                    }
                                    iterator->MoveNext();
                                }
                            }
                        }
                    break;

                    case ito::tComplex128:
                        for (int nMat = 0; nMat < dObj->calcNumMats(); nMat++)
                        {
                            double tempValRe, tempValIm;
                            for (int y = 0; y < dObj->getSize(dObj->getDims() - 2); y++)
                            {
                                pDataRow = (((cv::Mat *)dObj->get_mdata()[dObj->seekMat(nMat)])->ptr(y));
                                for (int x = 0; x < dObj->getSize(dObj->getDims() - 1); x++)
                                {
                                    iso5436_2.GetMatrixPoint(x, y, nMat, vector);

                                    if(vector.IsValid())
                                    {
                                        vector.GetZ()->Get(&tempValRe);
                                        iso5436_2.GetMatrixPoint(x, y, nMat * 2, vector);
                                        vector.GetZ()->Get(&tempValIm);
                                        ((ito::complex128*)pDataRow)[x].real(tempValRe * zscale + zoffset);
                                        ((ito::complex128*)pDataRow)[x].imag(tempValIm * zscale + zoffset);
                                    }
                                    else
                                    {
                                        ((ito::complex128*)pDataRow)[x] = std::numeric_limits<ito::complex128>::signaling_NaN();
                                    }

                                    iterator->MoveNext();
                                }
                            }
                        }
                    break;

                    default:
                        retval += ito::RetVal(ito::retError, 0, QObject::tr("data type mismatch").toLatin1().data());
                    break;
                }
            break;

            default:
                retval += ito::RetVal(ito::retError, 0, QObject::tr("data type mismatch").toLatin1().data());
            break;
        }

        if(ogps_HasError())
        {
            retval += ito::RetVal(ito::retError, 0, QObject::tr("error reading is5436_2 file").toLatin1().data());
        }

    }
    catch(OpenGPS::Exception &e)
    {
        OpenGPS::String err=e.details();
        retval += ito::RetVal::format(ito::retError, 0, QObject::tr("error opening file: %s").toLatin1().data(), err.ToChar() );
    }


    // Free iterator/buffer
    iterator.release();

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal X3pIO::saveDObj(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * /*paramsOpt*/, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;
    wchar_t xdatat, ydatat, zdatat;
    double xscale, yscale, zscale, xoffset, yoffset, zoffset;

    ito::DataObject *dObj = (ito::DataObject*)(*paramsMand)[0].getVal<void*>();
    char *filename = NULL;
    filename = (*paramsMand)[1].getVal<char*>();

    if (!dObj)
        return ito::RetVal(ito::retError, 0, QObject::tr("empty data object").toLatin1().data());
    if (!filename)
        return ito::RetVal(ito::retError, 0, QObject::tr("no filename specified").toLatin1().data());


    switch (dObj->getType())
    {
        case ito::tInt8:
        case ito::tUInt8:
        case ito::tInt16:
        case ito::tUInt16:
            zdatat = Record1Type::Axes_type::CX_type::DataType_type::I;
        break;

        case ito::tUInt32:
        case ito::tInt32:
            zdatat = Record1Type::Axes_type::CX_type::DataType_type::L;
        break;

        case ito::tComplex64:
        case ito::tFloat32:
            zdatat = Record1Type::Axes_type::CX_type::DataType_type::F;
        break;

        case ito::tComplex128:
        case ito::tFloat64:
            zdatat = Record1Type::Axes_type::CX_type::DataType_type::D;
        break;

        default:
            return ito::RetVal(ito::retError, 0, QObject::tr("unsupported data type. Supported types are (Int8), Int16, Int32, Float32 and Float64").toLatin1().data());
        break;

    }

    if (dObj->getSize(dObj->getDims() - 1) == 1)
        xdatat = Record1Type::Axes_type::CX_type::DataType_type::I;
    else
    {
        xscale = dObj->getAxisScale(dObj->getDims() - 1);
        if (floor(xscale) == xscale)
        {
            xdatat = Record1Type::Axes_type::CX_type::DataType_type::L;
        }
        else
        {
            xdatat = Record1Type::Axes_type::CX_type::DataType_type::D;
        }
    }
    if (dObj->getSize(dObj->getDims() - 2) == 1)
        ydatat = Record1Type::Axes_type::CX_type::DataType_type::I;
    else
    {
        yscale = dObj->getAxisScale(dObj->getDims() - 2);
        if (floor(yscale) == yscale)
        {
            ydatat = Record1Type::Axes_type::CX_type::DataType_type::L;
        }
        else
        {
            ydatat = Record1Type::Axes_type::CX_type::DataType_type::D;
        }
    }

    xoffset = dObj->getAxisOffset(dObj->getDims() - 1);
    yoffset = dObj->getAxisOffset(dObj->getDims() - 2);
    zscale = dObj->getValueScale();
    zoffset = dObj->getValueOffset();

    // for simplicity we will store always as 2D data, even for vetor type data. X- and y-axis will
    // always be stored as incremental axes
    // Create RECORD1
    Record1Type::Revision_type revision(OGPS_ISO5436_2000_REVISION_NAME);
    Record1Type::FeatureType_type featureType(OGPS_FEATURE_TYPE_SURFACE_NAME);

    Record1Type::Axes_type::CX_type::AxisType_type xaxisType(Record1Type::Axes_type::CX_type::AxisType_type::I); // absolute
//    Record1Type::Axes_type::CX_type::DataType_type xdataType(Record1Type::Axes_type::CX_type::DataType_type::D); // int32
    Record1Type::Axes_type::CX_type::DataType_type xdataType((Record1Type::Axes_type::CX_type::DataType_type::value) xdatat);
    Record1Type::Axes_type::CX_type xaxis(xaxisType);
    xaxis.DataType(xdataType);
    xaxis.Increment(xscale / 1000.0); // 10 micrometres
    xaxis.Offset(xoffset * xscale); // 1 millimetre

    Record1Type::Axes_type::CY_type::AxisType_type yaxisType(Record1Type::Axes_type::CY_type::AxisType_type::I); // absolute
//    Record1Type::Axes_type::CY_type::DataType_type ydataType(Record1Type::Axes_type::CY_type::DataType_type::D); // float
    Record1Type::Axes_type::CY_type::DataType_type ydataType((Record1Type::Axes_type::CY_type::DataType_type::value) ydatat);
    Record1Type::Axes_type::CY_type yaxis(yaxisType);
    yaxis.DataType(ydataType);
    yaxis.Increment(yscale / 1000.0); // set to 1 for float and double axis
    yaxis.Offset(yoffset * yscale); // -1 milli metre

    Record1Type::Axes_type::CZ_type::AxisType_type zaxisType(Record1Type::Axes_type::CZ_type::AxisType_type::A); // absolute
//    Record1Type::Axes_type::CZ_type::DataType_type zdataType(Record1Type::Axes_type::CZ_type::DataType_type::D); // 16 bit integer
    Record1Type::Axes_type::CZ_type::DataType_type zdataType((Record1Type::Axes_type::CZ_type::DataType_type::value) zdatat); // 16 bit integer
    Record1Type::Axes_type::CZ_type zaxis(zaxisType);
    zaxis.DataType(zdataType);
    zaxis.Increment(zscale / 1000.0); // set to 1 for float and double axis
//    zaxis.Increment(1); // set to 1 for float and double axis
    zaxis.Offset(zoffset * zscale); // 1 milli metre

    Record1Type::Axes_type axis(xaxis, yaxis, zaxis);

    Record1Type record1(revision, featureType, axis);

    // Create RECORD2
    //   Record2Type::Date_type date(TimeStamp());
    Record2Type::Date_type date(TimeStamp(), 0);

    bool foundTag = 0;
    OpenGPS::String tmpStr;
    tmpStr.FromChar(dObj->getTag(std::string("manufacturer"), foundTag).getVal_ToString().data());
    Record2Type::Instrument_type::Manufacturer_type manufacturer(_T("ITO"));
    if (tmpStr.length() && foundTag)
        manufacturer = tmpStr;

    tmpStr.FromChar(dObj->getTag(std::string("model"), foundTag).getVal_ToString().data());
    Record2Type::Instrument_type::Model_type model(_T("unknown"));
    if (tmpStr.length() && foundTag)
        model = tmpStr;

    tmpStr.FromChar(dObj->getTag(std::string("serial"), foundTag).getVal_ToString().data());
    Record2Type::Instrument_type::Serial_type serial(_T("unknown"));
    if (tmpStr.length() && foundTag)
        serial = tmpStr;

    Record2Type::Instrument_type::Version_type version(_OPENGPS_VERSIONSTRING);
    Record2Type::Instrument_type instrument(manufacturer, model, serial, version);

    //   Record2Type::CalibrationDate_type calibrationDate(_T("2007-04-30T13:58:02.6+02:00"));
    tmpStr.FromChar(dObj->getTag(std::string("calibrationDate"), foundTag).getVal_ToString().data());
    Record2Type::CalibrationDate_type calibrationDate(_T("1900-01-01T00:00:00.0+00:00"), 0);
    // in case the calibration dateTime is unknown we should not use this function, as it will produce
    // a date time string that makes the created x3p file invalid
    if (tmpStr.length() && foundTag && strcmp(tmpStr.ToChar(), "{unknown}") != 0)
    {
        calibrationDate = Record2Type::CalibrationDate_type(tmpStr.data(), 0);
    }

    tmpStr.FromChar(dObj->getTag(std::string("probingSystemType"), foundTag).getVal_ToString().data());
    Record2Type::ProbingSystem_type::Type_type type(Record2Type::ProbingSystem_type::Type_type::Software);
    if (tmpStr.length() && foundTag)
    {
        if (wcscmp(tmpStr.c_str(), _T("Contacting")) == 0)
            type = Record2Type::ProbingSystem_type::Type_type::Contacting;
        else if (wcscmp(tmpStr.c_str(), _T("NonContacting")) == 0)
            type = Record2Type::ProbingSystem_type::Type_type::NonContacting;
    }

    tmpStr.FromChar(dObj->getTag(std::string("probingSystemID"), foundTag).getVal_ToString().data());
    Record2Type::ProbingSystem_type::Identification_type id(_T("unknown"));
    if (tmpStr.length() && foundTag)
        id = tmpStr;

    Record2Type::ProbingSystem_type probingSystem(type, id);

    tmpStr.FromChar(dObj->getTag(std::string("comment"), foundTag).getVal_ToString().data());
    Record2Type::Comment_type comment(_T(""));
    if (tmpStr.length() && foundTag)
        comment = tmpStr;

    Record2Type record2(date, instrument, calibrationDate, probingSystem);
    record2.Comment(comment);

    // Create MATRIX
    int nMats = dObj->calcNumMats();
    MatrixDimensionType matrix(0, 0, 0);
    if ((dObj->getType() == ito::tComplex64) || (dObj->getType() == ito::tComplex128))
    {
        matrix.SizeX(dObj->getSize(dObj->getDims() - 1));
        matrix.SizeY(dObj->getSize(dObj->getDims() - 2));
        matrix.SizeZ(nMats * 2);
    }
    else
    {
        matrix.SizeX(dObj->getSize(dObj->getDims() - 1));
        matrix.SizeY(dObj->getSize(dObj->getDims() - 2));
        matrix.SizeZ(nMats);
    }

    // Create ISO5436_2 container
    OpenGPS::String fname;
    fname.FromChar(filename);
    OGPS_ISO5436_2Handle handle = ogps_CreateMatrixISO5436_2(fname.c_str(), NULL, record1, &record2, matrix, TRUE);
//    OGPS_ISO5436_2Handle handle = ogps_CreateMatrixISO5436_2(_T("test.xp3"), NULL, record1, &record2, matrix, TRUE);

    retval += fillDataMatrix(handle, dObj);

    bool ismatrix=ogps_IsMatrix(handle);
    if (ogps_HasError())
    {
      return FALSE;
    }

    // Check for Matrix or List
    if (ismatrix)
    {
        // Get Matrix dimensions
        unsigned long sx,sy,sz;
        ogps_GetMatrixDimensions(handle,&sx,&sy,&sz);
    }

    // Finally: write container to disk.
    ogps_WriteISO5436_2(handle);
    ogps_CloseISO5436_2(&handle);

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal X3pIO::loadDObjParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(!retval.containsError())
    {
        ito::Param param = ito::Param("destinationObject", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, QObject::tr("Empty dataObject").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, NULL, QObject::tr("source file name").toLatin1().data());
        paramsMand->append(param);

    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal X3pIO::loadDObj(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * /*paramsOpt*/, QVector<ito::ParamBase> * /*paramsOut*/)
{
   ito::RetVal retval = ito::retOk;
   ito::DataObject *dObj = (ito::DataObject*)(*paramsMand)[0].getVal<void*>();
   char *filename = NULL;
   filename = (*paramsMand)[1].getVal<char*>();

   if (!dObj)
      return ito::RetVal(ito::retError, 0, QObject::tr("empty data object").toLatin1().data());
   if (!filename)
      return ito::RetVal(ito::retError, 0, QObject::tr("no filename specified").toLatin1().data());

   // Open the file, hopefully everything went well...
   OpenGPS::String fname;
   fname.FromChar(filename);
   OpenGPS::ISO5436_2 iso5436_2(fname.c_str());
   // Check for error opening
   if(ogps_HasError())
   {
      //     std::cerr << "Error opening file \"" << fileName << "\"" << endl;
      return ito::RetVal(ito::retError, 0, QObject::tr("error opening file").toLatin1().data());
   }

   // Try to open in read only mode
   try
   {
      iso5436_2.Open(TRUE);
   }
   catch(OpenGPS::Exception &e)
   {
      OpenGPS::String err=e.details();
      //     std::cerr << "Error opening file \"" << fileName << "\"" << endl
      //               << err << endl;
      //err.ToChar()
      return ito::RetVal::format(ito::retError, 0, QObject::tr("error opening file: %s").toLatin1().data(), err.ToChar() );
   }

   // Obtain handle to xml document.
   const ISO5436_2Type* const document = iso5436_2.GetDocument();

   if(document)
   {
      // Print meta data
      //     PrintMetaData(document);
      // Is data list? / Is matrix? - don't care; we use point iterator.
//      ISO5436_2Type::Record1_type record1 = document->Record1();
//      ISO5436_2Type::Record2_optional record2 = document->Record2();
//      ISO5436_2Type::Record3_type record3 = document->Record3();

// here we need to read the ITO(tm) dimensions and data type from the vendor specific extensions ;-)
        OpenGPS::PointIteratorAutoPtr iterator = iso5436_2.CreateNextPointIterator();
        OpenGPS::PointVector vector;
        int dObjType;

        //iterate through points until the first valid element comes (valid != pointTypeMissing ... )
        do
        {
            iterator->GetCurrent(vector);
            iterator->MoveNext();
        } while(!vector.IsValid());

        _OGPS_DATA_POINT_TYPE pointType = vector.GetZ()->GetPointType();

        switch (pointType)
        {
            case OGPS_Int16PointType:
                dObjType = ito::tInt16;
            break;

            case OGPS_Int32PointType:
                dObjType = ito::tInt16;
            break;

            case OGPS_FloatPointType:
                dObjType = ito::tFloat32;
            break;

            case OGPS_DoublePointType:
                dObjType = ito::tFloat64;
            break;

            default:
                dObjType = ito::tFloat32;
            break;
        }

        iterator.release();

       if (document->Record3().ListDimension().present())
       {
           Record3Type::ListDimension_type matDim = document->Record3().ListDimension().get();
           dObj->zeros(matDim, dObjType);
       }
       else if (document->Record3().MatrixDimension().present())
       {
           Record3Type::MatrixDimension_type matDim = document->Record3().MatrixDimension().get();
           dObj->zeros(matDim.SizeZ(), matDim.SizeY(), matDim.SizeX(), dObjType);
       }
       else
       {
           return ito::RetVal(ito::retError, 0, QObject::tr("could neither retrieve list nor matrix dimensions").toLatin1().data());
       }

       double xscale = 0.001, xoffset = 0.0;
       // we must check if scaling and offset are present, otherwise they are filled with random values
       if (document->Record1().Axes().CX().Increment().present())
           xscale = document->Record1().Axes().CX().Increment().get();
       if (document->Record1().Axes().CX().Offset().present())
           xoffset = document->Record1().Axes().CX().Offset().get() / xscale;
       dObj->setAxisScale(dObj->getDims() - 1, xscale * 1000.0);
       dObj->setAxisOffset(dObj->getDims() - 1, xoffset);

       double yscale = 0.001, yoffset = 0.0;
       // we must check if scaling and offset are present, otherwise they are filled with random values
       if (document->Record1().Axes().CY().Increment().present())
           yscale = document->Record1().Axes().CY().Increment().get();
       if (document->Record1().Axes().CY().Offset().present())
           yoffset = document->Record1().Axes().CY().Offset().get() / yscale;
       dObj->setAxisScale(dObj->getDims() - 2, xscale);
       dObj->setAxisOffset(dObj->getDims() - 2, yoffset);

       // we currently do not support v/zscale and offset so if they are not standard we have to
       // use them writing data into dataObject
       // we must check if scaling and offset are present, otherwise they are filled with random values
       double zscale = 0.001, zoffset = 0.0;
       if (document->Record1().Axes().CZ().Increment().present())
            zscale = document->Record1().Axes().CZ().Increment().get();
       if (document->Record1().Axes().CZ().Offset().present())
            zoffset = document->Record1().Axes().CZ().Offset().get() / zscale;
       zscale *= 1000.0;

       OpenGPS::String tempStr;
       tempStr = document->Record2()->Instrument().Manufacturer();
       dObj->setTag("manufacturer", std::string(tempStr.ToChar()));

       tempStr = document->Record2()->Instrument().Model();
       dObj->setTag("model", std::string(tempStr.ToChar()));

       tempStr = document->Record2()->Instrument().Serial();
       dObj->setTag("serial", std::string(tempStr.ToChar()));

       tempStr = TimeStamp((::xml_schema::date_time &) document->Record2()->CalibrationDate());
       dObj->setTag("calibrationDate", std::string(tempStr.ToChar()));

       tempStr = document->Record2()->ProbingSystem().Type();
       dObj->setTag("probingSystemType", std::string(tempStr.ToChar()));

       tempStr = document->Record2()->ProbingSystem().Identification();
       dObj->setTag("probingSystemID", std::string(tempStr.ToChar()));

       tempStr = document->Record2()->Comment().get();
       dObj->setTag("comment", std::string(tempStr.ToChar()));

       retval += loadDataMatrix(iso5436_2, pointType, dObj, zscale, zoffset);
   }

   iso5436_2.Close();

   return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal X3pIO::close(ItomSharedSemaphore * /*waitCond*/)
{
    ito::RetVal retval = ito::retOk;

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------

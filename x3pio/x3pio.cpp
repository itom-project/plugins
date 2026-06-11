/* ********************************************************************
    Plugin "x3pio" for itom software
    URL: http://www.twip-os.com
    Copyright (C) 2013, twip optical solutions GmbH
    Copyright (C) 2016, Institut für Technische Optik, Universität Stuttgart

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
#include "common/numeric.h"

#include "gitVersion.h"
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

#include <QtCore/QtPlugin>
#include <qvariant.h>
#include <QtGui>
#include <string.h>
#include <qapplication.h>

using namespace OpenGPS::Schemas::ISO5436_2;

/*!
  @brief Helper function to return the current time properly formatted.

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
  std::wostringstream sout;
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
    std::wostringstream sout;
    sout << std::setfill(_T('0')) << std::setw(4) << (lt->tm_year+1900) << _T('-') << std::setw(2) << lt->tm_mon << _T('-') << std::setw(2) << lt->tm_mday
        << _T('T') << std::setw(2) << lt->tm_hour << _T(':') << std::setw(2) << lt->tm_min << _T(':') << std::setw(2) << lt->tm_sec << _T(".0")
        << tzoffsign << std::setw(2) << tzoff_habs << _T(':') << std::setw(2) << tzoff_m;

    std::basic_string<wchar_t> s = sout.str();
    return sout.str();
}
#endif

//----------------------------------------------------------------------------------------------------------------------------------
OpenGPS::String TimeStamp( ::xml_schema::date_time &dateTime)
{
    std::wostringstream sout;
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
    REMOVE_PLUGININSTANCE(X3pIO)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
X3pIOInterface::X3pIOInterface()
{
    m_type = ito::typeAlgo;
    setObjectName("x3pio");

/*    char docstring[] = \
"This plugin provides methods to save and load dataObjects in/from the file format 'x3p'. \
This format is specified in ISO 25178 - Geometrical product specification (GPS). \n\
\n\
The library ISO 5436-2 XML, that is necessary for this plugin and included in the sources, \n\
is licensed under the LGPL license and uses further libraries. For more information about the license \n\
of the library itself see www.opengps.eu";
*/
    m_description = QObject::tr("x3p Import/Export");
//    m_detaildescription = QObject::tr(docstring);
    m_detaildescription = QObject::tr("This plugin provides methods to save and load dataObjects in/from the file format 'x3p'. \
This format is specified in ISO 25178 - Geometrical product specification (GPS). \n\
\n\
The library ISO 5436-2 XML, that is necessary for this plugin and included in the sources, \n\
is licensed under the LGPL license and uses further libraries. For more information about the license \n\
of the library itself see www.opengps.eu");

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
    m_aboutThis = QObject::tr(GITVERSION);
}

//----------------------------------------------------------------------------------------------------------------------------------
X3pIOInterface::~X3pIOInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

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

    filter = new FilterDef(X3pIO::saveDObj, X3pIO::saveDObjParams, QObject::tr("saves dataObject to x3p file. x3p defines all axes in meter, if the unit of any axis is m, cm, mm, \u00B5m or nm they are correctly converted to m."), ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iWriteDataObject, tr("X3P Files (*.x3p)"));  // mu m
    m_filterList.insert("saveX3p", filter);

    filter = new FilterDef(X3pIO::loadDObj, X3pIO::loadDObjParams, QObject::tr("loads dataObject from x3p file"), ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iReadDataObject, tr("X3P Files (*.x3p)"));
    m_filterList.insert("loadX3p", filter);

    setInitialized(true); //init method has been finished (independent on retval)
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal parseLastOpenGpsError(const QString &prefix)
{
    ito::RetVal retval;

    // Check for error opening
    if(ogps_HasError())
    {
        const OGPS_Character *errorMessage_ = ogps_GetErrorMessage();
        const OGPS_Character *errorDescription_ = ogps_GetErrorDescription();
        QString message;

        if (errorMessage_ && errorDescription_)
        {
            message = QString("%1 (%2)").arg(OpenGPS::String(errorMessage_).ToChar()).arg(OpenGPS::String(errorDescription_).ToChar());
        }
        else if (errorMessage_)
        {
            message = OpenGPS::String(errorMessage_).ToChar();
        }
        else
        {
            message = QObject::tr("no error details");
        }

        switch (ogps_GetErrorId())
        {
            /*! No failure condition trapped. This serves as some kind of default value. */
            case OGPS_ExNone:
                break;

            /*! A failure condition occurred, but it has not been specified in detail. */
            case OGPS_ExGeneral:
            /*! The value of at least one of the parameters passed to a function is invalid in the current context. */
            case OGPS_ExInvalidArgument:
            /*! Due to the state of the object an operation could not be performed. */
            case OGPS_ExInvalidOperation:
            /*! A specific implementation of an interface does not implement this operation. */
            case OGPS_ExNotImplemented:
            /*! An overflow occurred. There is no guarantee of the integrity of further processing steps. */
            case OGPS_ExOverflow:
                return ito::RetVal::format(ito::retError, 0, "%s%s", prefix.toLatin1().data(), message.toLatin1().data());
                break;

            /*! Indicates a non-fatal error that may be ignored. This is for informational purpose only. */
            case OGPS_ExWarning:
                return ito::RetVal::format(ito::retWarning, 0, "%s%s", prefix.toLatin1().data(), message.toLatin1().data());
                break;
        }
    }

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
        paramsOpt->append(ito::Param("binary", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 1, tr("Save data in binary (1, default) or ascii format - use binary for big objects (> 5000 Points)").toLatin1().data()));
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal fillDataMatrix(OGPS_ISO5436_2Handle &handle, const ito::DataObject *dObj, bool zScalingNecessary, double zFactor)
{
    ito::RetVal retval;

    // Add data points
    // 1. Create point vector buffer for three points.
    OGPS_PointVectorPtr vector = ogps_CreatePointVector();
    const unsigned char *pDataRow = NULL;
    int numMats = dObj->getNumPlanes();
    int ySize = dObj->getSize(dObj->getDims() - 2);
    int xSize = dObj->getSize(dObj->getDims() - 1);
    const cv::Mat* plane;

    for (int nMat = 0; nMat < numMats; ++nMat)
    {
        plane = dObj->get_mdata()[dObj->seekMat(0, numMats)];

        switch (dObj->getType())
        {
            case ito::tUInt8:
            case ito::tInt8:
                if (zScalingNecessary)
                {
                    for (int y = 0; y < ySize; y++)
                    {
                        pDataRow = plane->ptr(y);
                        for (int x = 0; x < xSize; x++)
                        {
                            ogps_SetDoubleZ(vector, zFactor * ((ito::uint8*)pDataRow)[x]);
                            ogps_SetMatrixPoint(handle, x, y, nMat, vector);

                            if(ogps_HasError())
                            {
                                retval += parseLastOpenGpsError(QObject::tr("error writing is5436_2 file: "));
                                if (retval.containsError())
                                {
                                    return retval;
                                }
                            }
                        }
                    }
                }
                else
                {
                    for (int y = 0; y < ySize; y++)
                    {
                        pDataRow = plane->ptr(y);
                        for (int x = 0; x < xSize; x++)
                        {
                            ogps_SetInt16Z(vector, ((ito::uint8*)pDataRow)[x]);
                            ogps_SetMatrixPoint(handle, x, y, nMat, vector);

                            if(ogps_HasError())
                            {
                                retval += parseLastOpenGpsError(QObject::tr("error writing is5436_2 file: "));
                                if (retval.containsError())
                                {
                                    return retval;
                                }
                            }
                        }
                    }
                }
            break;

            case ito::tUInt16:
            case ito::tInt16:
                if (zScalingNecessary)
                {
                    for (int y = 0; y < ySize; y++)
                    {
                        pDataRow = plane->ptr(y);
                        for (int x = 0; x < xSize; x++)
                        {
                            ogps_SetDoubleZ(vector, zFactor * ((ito::uint16*)pDataRow)[x]);
                            ogps_SetMatrixPoint(handle, x, y, nMat, vector);

                            if(ogps_HasError())
                            {
                                retval += parseLastOpenGpsError(QObject::tr("error writing is5436_2 file: "));
                                if (retval.containsError())
                                {
                                    return retval;
                                }
                            }
                        }
                    }
                }
                else
                {
                    for (int y = 0; y < ySize; y++)
                    {
                        pDataRow = plane->ptr(y);
                        for (int x = 0; x < xSize; x++)
                        {
                            ogps_SetInt16Z(vector, ((ito::uint16*)pDataRow)[x]);
                            ogps_SetMatrixPoint(handle, x, y, nMat, vector);

                            if(ogps_HasError())
                            {
                                retval += parseLastOpenGpsError(QObject::tr("error writing is5436_2 file: "));
                                if (retval.containsError())
                                {
                                    return retval;
                                }
                            }
                        }
                    }
                }
            break;

            case ito::tUInt32:
            case ito::tInt32:
                if (zScalingNecessary)
                {
                    for (int y = 0; y < ySize; y++)
                {
                    pDataRow = plane->ptr(y);
                    for (int x = 0; x < xSize; x++)
                    {
                        ogps_SetDoubleZ(vector, zFactor * ((ito::uint32*)pDataRow)[x]);
                        ogps_SetMatrixPoint(handle, x, y, nMat, vector);

                        if(ogps_HasError())
                        {
                            retval += parseLastOpenGpsError(QObject::tr("error writing is5436_2 file: "));
                            if (retval.containsError())
                            {
                                return retval;
                            }
                        }
                    }
                }
                }
                else
                {
                    for (int y = 0; y < ySize; y++)
                    {
                        pDataRow = plane->ptr(y);
                        for (int x = 0; x < xSize; x++)
                        {
                            ogps_SetInt32Z(vector, ((ito::uint32*)pDataRow)[x]);
                            ogps_SetMatrixPoint(handle, x, y, nMat, vector);

                            if(ogps_HasError())
                            {
                                retval += parseLastOpenGpsError(QObject::tr("error writing is5436_2 file: "));
                                if (retval.containsError())
                                {
                                    return retval;
                                }
                            }
                        }
                    }
                }
            break;

            case ito::tFloat32:
                if (zScalingNecessary)
                {
                    for (int y = 0; y < ySize; y++)
                    {
                        pDataRow = plane->ptr(y);
                        for (int x = 0; x < xSize; x++)
                        {
                            if (ito::isFinite(((ito::float32*)pDataRow)[x]))
                            {
                                ogps_SetFloatZ(vector, zFactor * ((ito::float32*)pDataRow)[x]);
                                ogps_SetMatrixPoint(handle, x, y, nMat, vector);
                            }
                            else
                            {
                                ogps_SetMatrixPoint(handle, x, y, nMat , NULL);
                            }

                            if(ogps_HasError())
                            {
                                retval += parseLastOpenGpsError(QObject::tr("error writing is5436_2 file: "));
                                if (retval.containsError())
                                {
                                    return retval;
                                }
                            }
                        }
                    }
                }
                else
                {
                    for (int y = 0; y < ySize; y++)
                    {
                        pDataRow = plane->ptr(y);
                        for (int x = 0; x < xSize; x++)
                        {
                            if (ito::isFinite(((ito::float32*)pDataRow)[x]))
                            {
                                ogps_SetFloatZ(vector, ((ito::float32*)pDataRow)[x]);
                                ogps_SetMatrixPoint(handle, x, y, nMat, vector);
                            }
                            else
                            {
                                ogps_SetMatrixPoint(handle, x, y, nMat , NULL);
                            }

                            if(ogps_HasError())
                            {
                                retval += parseLastOpenGpsError(QObject::tr("error writing is5436_2 file: "));
                                if (retval.containsError())
                                {
                                    return retval;
                                }
                            }
                        }
                    }
                }
            break;

            case ito::tFloat64:
                if (zScalingNecessary)
                {
                    for (int y = 0; y < ySize; y++)
                    {
                        pDataRow = plane->ptr(y);
                        for (int x = 0; x < xSize; x++)
                        {
                            if (ito::isFinite(((ito::float64*)pDataRow)[x]))
                            {
                                ogps_SetDoubleZ(vector, zFactor * ((ito::float64*)pDataRow)[x]);
                                ogps_SetMatrixPoint(handle, x, y, nMat, vector);
                            }
                            else
                            {
                                ogps_SetMatrixPoint(handle, x, y, nMat , NULL);
                            }

                            if(ogps_HasError())
                            {
                                retval += parseLastOpenGpsError(QObject::tr("error writing is5436_2 file: "));
                                if (retval.containsError())
                                {
                                    return retval;
                                }
                            }
                        }
                    }
                }
                else
                {
                    for (int y = 0; y < ySize; y++)
                    {
                        pDataRow = plane->ptr(y);
                        for (int x = 0; x < xSize; x++)
                        {
                            if (ito::isFinite(((ito::float64*)pDataRow)[x]))
                            {
                                ogps_SetDoubleZ(vector, ((ito::float64*)pDataRow)[x]);
                                ogps_SetMatrixPoint(handle, x, y, nMat, vector);
                            }
                            else
                            {
                                ogps_SetMatrixPoint(handle, x, y, nMat , NULL);
                            }

                            if(ogps_HasError())
                            {
                                retval += parseLastOpenGpsError(QObject::tr("error writing is5436_2 file: "));
                                if (retval.containsError())
                                {
                                    return retval;
                                }
                            }
                        }
                    }
                }
            break;

            case ito::tComplex64:
                for (int y = 0; y < ySize; y++)
                {
                    pDataRow = plane->ptr(y);
                    for (int x = 0; x < xSize; x++)
                    {
                        if (ito::isFinite(((ito::complex64*)pDataRow)[x]))
                        {
                            ogps_SetFloatZ(vector, zFactor * ((ito::complex64*)pDataRow)[x].real());
                            ogps_SetMatrixPoint(handle, x, y, nMat * 2, vector);
                            ogps_SetFloatZ(vector, zFactor * ((ito::complex64*)pDataRow)[x].imag());
                            ogps_SetMatrixPoint(handle, x, y, nMat * 2 + 1, vector);
                        }
                        else
                        {
                            ogps_SetMatrixPoint(handle, x, y, nMat * 2 , NULL);
                            ogps_SetMatrixPoint(handle, x, y, nMat * 2 + 1 , NULL);
                        }

                        if(ogps_HasError())
                        {
                            retval += parseLastOpenGpsError(QObject::tr("error writing is5436_2 file: "));
                            if (retval.containsError())
                            {
                                return retval;
                            }
                        }
                    }
                }
            break;

            case ito::tComplex128:
                for (int y = 0; y < ySize; y++)
                {
                    pDataRow = plane->ptr(y);
                    for (int x = 0; x < xSize; x++)
                    {
                        if (ito::isFinite(((ito::complex128*)pDataRow)[x]))
                        {
                            ogps_SetDoubleZ(vector, zFactor * ((ito::complex128*)pDataRow)[x].real());
                            ogps_SetMatrixPoint(handle, x, y, nMat * 2, vector);
                            ogps_SetDoubleZ(vector, zFactor * ((ito::complex128*)pDataRow)[x].imag());
                            ogps_SetMatrixPoint(handle, x, y, nMat * 2 + 1, vector);
                        }
                        else
                        {
                            ogps_SetMatrixPoint(handle, x, y, nMat * 2 , NULL);
                            ogps_SetMatrixPoint(handle, x, y, nMat * 2 + 1 , NULL);
                        }

                        if(ogps_HasError())
                        {
                            retval += parseLastOpenGpsError(QObject::tr("error writing is5436_2 file: "));
                            if (retval.containsError())
                            {
                                return retval;
                            }
                        }
                    }
                }
            break;
        }
    } //end for loop over nMats

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
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal loadDataMatrix(OpenGPS::ISO5436_2 &iso5436_2, OGPS_DataPointType &pointType, ito::DataObject *dObj, int xSize, int ySize, double zscale, double zoffset)
{
    ito::RetVal retval;

    // Use iterator to create points in this example.
    OpenGPS::PointIteratorAutoPtr iterator = iso5436_2.CreateNextPointIterator();
    unsigned char *pDataRow = NULL;

    // Iterate point data (ignoring if they were stored
    // in xml directly or in external binary file).
    OpenGPS::PointVector vector;
    OpenGPS::DataPoint *dp = NULL;
    int numMats = dObj->getNumPlanes();
    cv::Mat* plane;

    try
    {
        switch (pointType)
        {
            case OGPS_Int16PointType:
            {
                ito::int16 tempVal;
                ito::int16 *rowPtr;
                for (int nMat = 0; nMat < numMats; nMat++)
                {
                    plane = dObj->get_mdata()[dObj->seekMat(nMat, numMats)];
                    for (int y = 0; y < ySize; y++)
                    {
                        rowPtr = plane->ptr<ito::int16>(y);
                        for (int x = 0; x < xSize; x++)
                        {
                            iterator->GetCurrent(vector);
                            vector.GetZ()->Get(&tempVal);
                            rowPtr[x] = (ito::int16)(tempVal * zscale + zoffset);
                            iterator->MoveNext();
                        }
                    }
                }
            }
            break;

            case OGPS_Int32PointType:
            {
                ito::int32 tempVal;
                ito::int32 *rowPtr;
                for (int nMat = 0; nMat < numMats; nMat++)
                {
                    plane = dObj->get_mdata()[dObj->seekMat(nMat, numMats)];
                    for (int y = 0; y < ySize; y++)
                    {
                        rowPtr = plane->ptr<ito::int32>(y);
                        for (int x = 0; x < xSize; x++)
                        {
                            iterator->GetCurrent(vector);
                            vector.GetZ()->Get(&tempVal);
                            rowPtr[x] = (ito::int32)(tempVal * zscale + zoffset);
                            iterator->MoveNext();
                        }
                    }
                }
            }
            break;

            case OGPS_FloatPointType:
            {
                ito::float32 tempVal;
                ito::float32 *rowPtr;
                for (int nMat = 0; nMat < numMats; nMat++)
                {
                    plane = dObj->get_mdata()[dObj->seekMat(nMat, numMats)];
                    for (int y = 0; y < ySize; y++)
                    {
                        rowPtr = plane->ptr<ito::float32>(y);
                        for (int x = 0; x < xSize; x++)
                        {
                            iterator->GetCurrent(vector);
                            dp = vector.GetZ();

                            if(dp->IsValid())
                            {
                                vector.GetZ()->Get(&tempVal);
                                rowPtr[x] = tempVal * zscale + zoffset;
                            }
                            else
                            {
                                rowPtr[x] =  std::numeric_limits<ito::float32>::quiet_NaN();
                            }

                            iterator->MoveNext();
                        }
                    }
                }
            }
            break;

            case OGPS_DoublePointType:
            {
                ito::float64 tempVal;
                ito::float64 *rowPtr;
                for (int nMat = 0; nMat < numMats; nMat++)
                {
                    plane = dObj->get_mdata()[dObj->seekMat(nMat, numMats)];
                    for (int y = 0; y < ySize; y++)
                    {
                        rowPtr = plane->ptr<ito::float64>(y);
                        for (int x = 0; x < xSize; x++)
                        {
                            iterator->GetCurrent(vector);
                            dp = vector.GetZ();

                            if(dp->IsValid())
                            {
                                vector.GetZ()->Get(&tempVal);
                                rowPtr[x] =  tempVal * zscale + zoffset;
                            }
                            else
                            {
                                rowPtr[x] =  std::numeric_limits<ito::float64>::quiet_NaN();
                            }
                            iterator->MoveNext();
                        }
                    }
                }
            }
            break;

            default:
                retval += ito::RetVal(ito::retError, 0, QObject::tr("data type mismatch").toLatin1().data());
            break;
        }

        if(ogps_HasError())
        {
            retval += parseLastOpenGpsError(QObject::tr("error reading is5436_2 file: "));
        }

    }
    catch(OpenGPS::Exception &e)
    {
        OpenGPS::String err = e.details();
        if (e.id() == OGPS_ExWarning)
        {
            retval += ito::RetVal::format(ito::retWarning, 0, QObject::tr("warning while opening the file: %s").toLatin1().data(), err.ToChar());
        }
        else if (e.id() != OGPS_ExNone)
        {
            retval += ito::RetVal::format(ito::retError, 0, QObject::tr("error while opening the file: %s").toLatin1().data(), err.ToChar());
            return retval;
        }
    }

    // Free iterator/buffer
    iterator.release();

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal X3pIO::saveDObj(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;
    Record1Type::Axes_type::CX_type::DataType_type::value xdatat;
    Record1Type::Axes_type::CY_type::DataType_type::value ydatat;
    Record1Type::Axes_type::CZ_type::DataType_type::value zdatat;
    double xscale, yscale, zscale, xoffset, yoffset, zoffset;
    bool binary = paramsOpt->at(0).getVal<int>() ? true : false;

    const ito::DataObject *dObj = (*paramsMand)[0].getVal<const ito::DataObject*>();
    char *filename = (*paramsMand)[1].getVal<char*>();
    bool zScalingNecessary = false;

    if (!dObj)
    {
        return ito::RetVal(ito::retError, 0, QObject::tr("empty data object").toLatin1().data());
    }
    if (!filename)
    {
        return ito::RetVal(ito::retError, 0, QObject::tr("no filename specified").toLatin1().data());
    }
    if (dObj->getDims() < 2)
    {
        return ito::RetVal(ito::retError, 0, tr("data object must have at least two dimensions").toLatin1().data());
    }

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
    {
        xdatat = Record1Type::Axes_type::CX_type::DataType_type::I;
        xscale = 1.0;
    }
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
    {
        ydatat = Record1Type::Axes_type::CX_type::DataType_type::I;
        yscale = 1.0;
    }
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

    //check if the units are known, if yes get the conversion to meter, which is the default unit of x3p in all dimensions
    double xUnitScale = 1.0;
    double yUnitScale = 1.0;
    double valueUnitScale = 1.0;
    bool valid;
    std::string unitString = dObj->getAxisUnit(dObj->getDims() - 1, valid);

    if (valid)
    {
        retval += parseUnit(unitString, xUnitScale);
    }

    unitString = dObj->getAxisUnit(dObj->getDims() - 2, valid);
    if (valid)
    {
        retval += parseUnit(unitString, yUnitScale);
    }

    unitString = dObj->getValueUnit();
    if (valid)
    {
        retval += parseUnit(unitString, valueUnitScale);
    }

    // for simplicity we will store always as 2D data, even for vector type data. X- and y-axis will
    // always be stored as incremental axes
    // Create RECORD1
    Record1Type::Revision_type revision(OGPS_ISO5436_2000_REVISION_NAME);
    Record1Type::FeatureType_type featureType(OGPS_FEATURE_TYPE_SURFACE_NAME);

    Record1Type::Axes_type::CX_type::AxisType_type xaxisType(Record1Type::Axes_type::CX_type::AxisType_type::I); // incremental
    //Record1Type::Axes_type::CX_type::DataType_type xdataType(xdatat);
    Record1Type::Axes_type::CX_type xaxis(xaxisType);
    xaxis.DataType(xdatat);
    xaxis.Increment(xscale * xUnitScale); // scale must be in meter/pixel
    xaxis.Offset(xoffset * xscale * xUnitScale); // offset must be in meter/pixel

    Record1Type::Axes_type::CY_type::AxisType_type yaxisType(Record1Type::Axes_type::CY_type::AxisType_type::I); // incremental
    Record1Type::Axes_type::CY_type yaxis(yaxisType);
    yaxis.DataType(ydatat);
    yaxis.Increment(yscale * yUnitScale); // scale must be in meter/pixel
    yaxis.Offset(yoffset * yscale * yUnitScale); // offset must be in meter/pixel

    Record1Type::Axes_type::CZ_type::AxisType_type zaxisType(Record1Type::Axes_type::CZ_type::AxisType_type::A); // absolute (must always be absolute)
    Record1Type::Axes_type::CZ_type zaxis(zaxisType);

    if (std::abs(zscale * valueUnitScale - 1.0) > std::numeric_limits<double>::epsilon())
    {
        if (zdatat != Record1Type::Axes_type::CX_type::DataType_type::F && zdatat != Record1Type::Axes_type::CX_type::DataType_type::D)
        {
            retval += ito::RetVal(ito::retWarning, 0, tr("x3p stores its data in meter, therefore a scaling factor has to be applied. The format of the stored data is changed to double").toLatin1().data());
            zdatat = Record1Type::Axes_type::CX_type::DataType_type::D;
        }
        zaxis.DataType(zdatat);
        zaxis.Increment(1); // scale must be in meter/pixel
        zaxis.Offset(zoffset * zscale * valueUnitScale); // offset must be in meter/pixel
        zScalingNecessary = true;
    }
    else
    {
        zaxis.DataType(zdatat);
        zaxis.Increment(1); // scale must be in meter/pixel
        zaxis.Offset(zoffset); // offset must be in meter/pixel
        zScalingNecessary = false;
    }

    Record1Type::Axes_type axis(xaxis, yaxis, zaxis);

    double r11, r12, r13, r21, r22, r23, r31,r32, r33;
    dObj->getXYRotationalMatrix(r11, r12, r13, r21, r22, r23, r31,r32, r33);
    if (r11 != 1 || r22 != 1 || r33 != 1)
    {
        axis.Rotation(AxesType::Rotation_type(r11,r12,r13,r21,r22,r23,r31,r32,r33));
    }

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
        {
            type = Record2Type::ProbingSystem_type::Type_type::Contacting;
        }
        else if (wcscmp(tmpStr.c_str(), _T("NonContacting")) == 0)
        {
            type = Record2Type::ProbingSystem_type::Type_type::NonContacting;
        }
        else if (wcscmp(tmpStr.c_str(), _T("Software")) == 0)
        {
            type = Record2Type::ProbingSystem_type::Type_type::Software;
        }
    }

    tmpStr.FromChar(dObj->getTag(std::string("probingSystemID"), foundTag).getVal_ToString().data());
    Record2Type::ProbingSystem_type::Identification_type id(_T("unknown"));
    if (tmpStr.length() && foundTag)
    {
        id = tmpStr;
    }

    Record2Type::ProbingSystem_type probingSystem(type, id);

    tmpStr.FromChar(dObj->getTag(std::string("comment"), foundTag).getVal_ToString().data());
    Record2Type::Comment_type comment(_T(""));
    if (tmpStr.length() && foundTag)
    {
        comment = tmpStr;
    }

    Record2Type record2(date, instrument, calibrationDate, probingSystem);
    record2.Comment(comment);

    // Create MATRIX
    int nMats = dObj->getNumPlanes();
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
    OGPS_ISO5436_2Handle handle = ogps_CreateMatrixISO5436_2(fname.c_str(), NULL, record1, &record2, matrix, binary ? TRUE : FALSE);

    retval += fillDataMatrix(handle, dObj, zScalingNecessary, zscale * valueUnitScale);

    bool ismatrix=ogps_IsMatrix(handle);
    if (ogps_HasError())
    {
       retval += parseLastOpenGpsError(QObject::tr("data set is no matrix: "));
       if (retval.containsError())
       {
           return retval;
       }
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
ito::RetVal X3pIO::parseUnit(const std::string &unitString, double &unitScale)
{
    ito::RetVal retval;
    if (unitString == "" || unitString == "m")
    {
        unitScale = 1.0;
    }
    else if (unitString == "cm")
    {
        unitScale = 1.0e-2;
    }
    else if (unitString == "mm")
    {
        unitScale = 1.0e-3;
    }
    else if (unitString == "\u00B5m" || (unitString.size() == 2 && unitString.data()[0] == -75 && unitString.data()[1] == 'm'))  // mu m
    {
        unitScale = 1.0e-6;
    }
    else if (unitString == "nm")
    {
        unitScale = 1.0e-9;
    }
    else
    {
        unitScale = 1.0;
        retval += ito::RetVal::format(ito::retWarning, 0, tr("unit '%s' cannot be interpreted. Meter as default unit is assumed").toLatin1().data(), unitString.data());
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal X3pIO::loadDObjParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(!retval.containsError())
    {
        ito::Param param = ito::Param("destinationObject", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Empty dataObject").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("source file name").toLatin1().data());
        paramsMand->append(param);

        param = ito::Param("xyUnit", ito::ParamBase::String | ito::ParamBase::In, "m", tr("Unit of x and y axes. x3p assumes to have m as default unit, this can be scaled using other values than m. Default: m (Be careful that other units than 'm' lead to a multiplication of all values that might exceed the data type limit.)").toLatin1().data());
        ito::StringMeta sm(ito::StringMeta::String, "m");
        sm.addItem("cm");
        sm.addItem("mm");
        sm.addItem("\u00B5m");  // mu m
        sm.addItem("nm");
        param.setMeta(&sm, false);
        paramsOpt->append(param);

        param = ito::Param("valueUnit", ito::ParamBase::String | ito::ParamBase::In, "m", tr("Unit of value axis. x3p assumes to have m as default unit, this can be scaled using other values than m. Default: m (Be careful that other units than 'm' lead to a multiplication of all values that might exceed the data type limit.)").toLatin1().data());
        ito::StringMeta sm2(ito::StringMeta::String, "m");
        sm2.addItem("cm");
        sm2.addItem("mm");
        sm2.addItem("\u00B5m");  // mu m
        sm2.addItem("nm");
        param.setMeta(&sm2, false);
        paramsOpt->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal X3pIO::loadDObj(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
   ito::RetVal retval = ito::retOk;
   ito::DataObject *dObjIn = (*paramsMand)[0].getVal<ito::DataObject*>();
   char *filename = (*paramsMand)[1].getVal<char*>();
   std::string xyUnit = paramsOpt->at(0).getVal<char*>();
   std::string valueUnit = paramsOpt->at(1).getVal<char*>();

   if (!dObjIn)
   {
      return ito::RetVal(ito::retError, 0, tr("empty data object").toLatin1().data());
   }

   if (!filename)
   {
      return ito::RetVal(ito::retError, 0, tr("no filename specified").toLatin1().data());
   }

   ito::DataObject dObj;

   double xyScaleFactor = 1.0;

   if (xyUnit == "m")
   {
       xyScaleFactor = 1.0; //m is the default unit of x3p
   }
   else if (xyUnit == "cm")
   {
       xyScaleFactor = 100.0;
   }
   else if (xyUnit == "mm")
   {
       xyScaleFactor = 1000.0;
   }
   else if (xyUnit == "\u00B5m" || (xyUnit.size() == 2 && xyUnit.data()[0] == -75 && xyUnit.data()[1] == 'm'))  // mu m
   {
       xyScaleFactor = 1.0e6;
   }
   else if (xyUnit == "nm")
   {
       xyScaleFactor = 1.0e9;
   }

   double valueScaleFactor = 1.0;

   if (valueUnit == "m")
   {
       valueScaleFactor = 1.0; //m is the default unit of x3p
   }
   else if (valueUnit == "cm")
   {
       valueScaleFactor = 100.0;
   }
   else if (valueUnit == "mm")
   {
       valueScaleFactor = 1000.0;
   }
   else if (valueUnit == " \u00B5m" || (valueUnit.size() == 2 && valueUnit.data()[0] == -75 && valueUnit.data()[1] == 'm'))  // mu m
   {
       valueScaleFactor = 1.0e6;
   }
   else if (valueUnit == "nm")
   {
       valueScaleFactor = 1.0e9;
   }

    // Open the file, hopefully everything went well...
    OpenGPS::String fname;
    fname.FromChar(filename);
    OpenGPS::ISO5436_2 iso5436_2(fname);

    // Check for error opening
    if(ogps_HasError())
    {
        retval += parseLastOpenGpsError(tr("error opening file: "));
        if (retval.containsError())
        {
            return retval;
        }
    }

    // Try to open in read only mode
    try
    {
        iso5436_2.Open(TRUE);
    }
    catch(OpenGPS::Exception &e)
    {
        OpenGPS::String err = e.details();
        if (e.id() == OGPS_ExWarning)
        {
            retval += ito::RetVal::format(ito::retWarning, 0, tr("warning while opening the file: %s").toLatin1().data(), err.ToChar());
        }
        else if (e.id() != OGPS_ExNone)
        {
            retval += ito::RetVal::format(ito::retError, 0, tr("error while opening the file: %s").toLatin1().data(), err.ToChar());
            return retval;
        }
    }

    // Obtain handle to xml document.
    const ISO5436_2Type* document = iso5436_2.GetDocument();

    if(document)
    {
        // Is data list? / Is matrix? - don't care; we use point iterator.
        const ISO5436_2Type::Record1_type record1 = document->Record1();
        const ISO5436_2Type::Record2_optional record2 = document->Record2();
        const ISO5436_2Type::Record3_type record3 = document->Record3();

        int x_idx = 2;
        int y_idx = 1;
        int z_idx = 0;
        int x_size, y_size, z_size;
        ito::tDataType dtype;
        double xscale = 1.0, xoffset = 0.0;
        double yscale = 1.0, yoffset = 0.0;
        double zscale = 1.0, zoffset = 0.0;
        OGPS_DataPointType pointType = OGPS_MissingPointType;
        bool isListType = false;

        if (record1.FeatureType() != Record1Type::FeatureType_type::SUR &&
            record1.FeatureType() != Record1Type::FeatureType_type::PRF)
        {
            retval += ito::RetVal(ito::retError, 0, tr("only feature types SUR (surface) or PRF (profile) are supported.").toLatin1().data());
        }
        else
        {
            //Record 3: Data.
            if (record3.MatrixDimension().present())
            {
                const Record3Type::MatrixDimension_type matDim = record3.MatrixDimension().get();
                x_size = matDim.SizeX();
                y_size = matDim.SizeY();
                z_size = matDim.SizeZ();

                if (x_size * y_size * z_size == 0)
                {
                    retval += ito::RetVal(ito::retError, 0, tr("x3p file does not contain any data").toLatin1().data());
                }
                else
                {
                    if (matDim.SizeZ() <= 1)
                    {
                        z_size = 0;
                        x_idx = 1;
                        y_idx = 0;
                    }
                }
            }
            else if (record3.ListDimension().present())
            {
                const Record3Type::ListDimension_type listDim = record3.ListDimension().get();
                y_size = 1;
                x_size = listDim;
                z_size = 0;
                x_idx = 1;
                y_idx = 0;
                isListType = true;
            }
            else
            {
                retval += ito::RetVal(ito::retError, 0, tr("x3p file does not contain organized matrix or list data. Unordered list data is not supported.").toLatin1().data());
            }
        }

        if (!retval.containsError())
        {
            // here we need to read the ITO(tm) dimensions and data type from the vendor specific extensions ;-)
            OpenGPS::PointIteratorAutoPtr iterator = iso5436_2.CreateNextPointIterator();
            OpenGPS::PointVector vector;
            iterator->GetCurrent(vector);

            //iterate through points until the first valid element comes (valid != pointTypeMissing ... )
            while(!vector.IsValid() && iterator->HasNext())
            {
                iterator->MoveNext();
                iterator->GetCurrent(vector);
            }

            pointType = vector.GetZ()->GetPointType();

            switch (pointType)
            {
                case OGPS_Int16PointType:
                    dtype = ito::tInt16;
                break;

                case OGPS_Int32PointType:
                    dtype = ito::tInt16;
                break;

                case OGPS_FloatPointType:
                    dtype = ito::tFloat32;
                break;

                case OGPS_DoublePointType:
                    dtype = ito::tFloat64;
                break;

                default:
                    dtype = ito::tFloat32;
                break;
            }

            iterator.release();

            //create dObj with appropriate dimensions...
            if (z_size > 0)
            {
                dObj.zeros(z_size, y_size, x_size, dtype);
            }
            else
            {
                dObj.zeros(y_size, x_size, dtype);
            }
        }

        //x- and y- axis must be incremental
        if (!retval.containsError())
        {
            if (x_size > 1 && y_size > 1)
            {
                //x- and y- axis must be incremental
                if (record1.Axes().CX().AxisType() != AxisType::I ||
                    record1.Axes().CY().AxisType() != AxisType::I)
                {
                    retval += ito::RetVal(ito::retError, 0, tr("x- and y-axes must have an incremental axis type. absolute x- and y-axes not supported.").toLatin1().data());
                }
            }
            else if (y_size == 1) //list, x- axis must be incremental
            {
                if (record1.Axes().CX().AxisType() != AxisType::I)
                {
                    retval += ito::RetVal(ito::retError, 0, tr("x-axis must have an incremental axis type.").toLatin1().data());
                }
            }
        }

        //check Record1 for scaling, rotation matrix...
        if (!retval.containsError())
        {
            // we must check if scaling and offset are present, otherwise they are filled with random values
            if (record1.Axes().CX().Increment().present())
            {
                xscale = document->Record1().Axes().CX().Increment().get();          // the lateral unit of x3p is m/px
            }
            if (record1.Axes().CX().Offset().present())
            {
                xoffset = record1.Axes().CX().Offset().get() / xscale;   // in itom, the offset is in pixel, x3p returns the offset in m/px
            }
            dObj.setAxisScale(x_idx, xscale * xyScaleFactor); //
            dObj.setAxisOffset(x_idx, xoffset);
            dObj.setAxisUnit(x_idx, xyUnit);

            // we must check if scaling and offset are present, otherwise they are filled with random values
            if (record1.Axes().CY().Increment().present())
            {
                yscale = record1.Axes().CY().Increment().get();          // the lateral unit of x3p is m/px
            }

            if (record1.Axes().CY().Offset().present())
            {
                yoffset = record1.Axes().CY().Offset().get() / yscale;   // in itom, the offset is in pixel, x3p returns the offset in m/px
            }
            dObj.setAxisScale(y_idx, yscale * xyScaleFactor);
            dObj.setAxisOffset(y_idx, yoffset);
            dObj.setAxisUnit(y_idx, xyUnit);

            // we must check if scaling and offset are present, otherwise they are filled with random values
            if (record1.Axes().CZ().AxisType() == AxisType::A)
            {
                zscale = 1.0; //per definition, the scaling of an absolute axis is 1.0
            }
            else if (record1.Axes().CZ().Increment().present())
            {
                zscale = record1.Axes().CZ().Increment().get();
            }

            //offset can always be available, since it is part of the rotation and translation component
            if (record1.Axes().CZ().Offset().present())
            {
                zoffset = record1.Axes().CZ().Offset().get() / zscale;
            }
            zscale *= valueScaleFactor;
            dObj.setValueUnit(valueUnit);

            if (record1.Axes().Rotation().present())
            {
                const AxesType::Rotation_type &rot = record1.Axes().Rotation().get();
                dObj.setXYRotationalMatrix(rot.r11(), rot.r12(), rot.r13(), rot.r21(), rot.r22(), rot.r23(), rot.r31(), rot.r32(), rot.r33());
            }
        }

        //Read meta information from Record2
        if (!retval.containsError())
        {
            //Read meta information from Record2
            OpenGPS::String tempStr;
            tempStr = record2->Instrument().Manufacturer();
            dObj.setTag("manufacturer", std::string(tempStr.ToChar()));

            tempStr = record2->Instrument().Model();
            dObj.setTag("model", std::string(tempStr.ToChar()));

            tempStr = record2->Instrument().Serial();
            dObj.setTag("serial", std::string(tempStr.ToChar()));

            tempStr = TimeStamp((::xml_schema::date_time &) document->Record2()->CalibrationDate());
            dObj.setTag("calibrationDate", std::string(tempStr.ToChar()));

            tempStr = record2->ProbingSystem().Type();
            dObj.setTag("probingSystemType", std::string(tempStr.ToChar()));

            tempStr = record2->ProbingSystem().Identification();
            dObj.setTag("probingSystemID", tempStr.ToChar());

            if (record2->Comment().present())
            {
                tempStr = record2->Comment().get();
                dObj.setTag("comment", tempStr.ToChar());
            }
        }

        if (!retval.containsError())
        {
            //load data
            retval += loadDataMatrix(iso5436_2, pointType, &dObj, x_size, y_size, zscale, zoffset);
        }
   }

   iso5436_2.Close();

   if (!retval.containsError())
   {
       (*dObjIn) = dObj;
   }

   return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal X3pIO::close(ItomSharedSemaphore * /*waitCond*/)
{
    ito::RetVal retval = ito::retOk;

    return retval;
}

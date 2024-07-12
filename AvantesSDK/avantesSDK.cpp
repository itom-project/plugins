/* ********************************************************************
    Plugin "AvantesSDK" for itom software
    URL: https://github.com/itom-project/plugins
    Copyright (C) 2016, Institut fuer Technische Optik, Universitaet Stuttgart

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

#include "avantesSDK.h"

#include <QFile>
#include <qstring.h>
#include <qstringlist.h>
#include <qelapsedtimer.h>
#include <QtCore/QtPlugin>
#include "pluginVersion.h"
#include "gitVersion.h"

#include "dockWidgetAvantesSDK.h"

uint32 const    MAX_INTEGRATION_TIME = 600000; // 600 seconds

#ifdef WIN32
    #include <Windows.h>
#else
    #define LOBYTE(w) ((unsigned char)(((w)) & 0xff))
    #define HIBYTE(w) ((unsigned char)((((w)) >> 8) & 0xff))
#endif

//static unsigned long AvantesSDK000Base = 0x10000L; // or 64k for Master Mode
//static int AvantesSDKADLow=31, AvantesSDKADHigh=133;

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvantesSDKInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(AvantesSDK)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvantesSDKInterface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(AvantesSDK)
    return ito::retOk;
}


//----------------------------------------------------------------------------------------------------------------------------------
AvantesSDKInterface::AvantesSDKInterface() : AddInInterfaceBase()
{
    m_type = ito::typeDataIO | ito::typeGrabber;
    setObjectName("AvantesSDK");

    m_description = QObject::tr("Avantes SDK");
    m_detaildescription = QObject::tr(
    "This DLL integrates the Avantis AvantesSDK spectrometer series into itom. \
    It has been tested with the following spectrometers: \
    \
    * ");
    m_author = "F. Hetzel, B. Bertschinger, M. Gronle, R. Hahn, W. Lyda, ITO, University Stuttgart";
    m_license = QObject::tr("LGPL");
    m_version           = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer        = MINVERSION;
    m_maxItomVer        = MAXVERSION;
    m_aboutThis         = tr(GITVERSION);

    m_autoLoadPolicy = ito::autoLoadNever;
    m_autoSavePolicy = ito::autoSaveNever;

    ito::Param paramVal = ito::Param("VendorID", ito::ParamBase::Int, 0, std::numeric_limits<int>::max(), 0x1992, tr("VendorID of spectrometer, 0x1992 for SDK-3648, 0x471 for SDK-ULS3648...").toLatin1().data());
    paramVal.getMetaT<ito::IntMeta>()->setRepresentation(ito::ParamMeta::HexNumber);
    m_initParamsMand.append(paramVal);
    paramVal = ito::Param("ProductID", ito::ParamBase::Int, 0, std::numeric_limits<int>::max(), 0x0667, tr("ProductID of spectrometer, 0x0667 for spectrometer").toLatin1().data());
    paramVal.getMetaT<ito::IntMeta>()->setRepresentation(ito::ParamMeta::HexNumber);
    m_initParamsMand.append(paramVal);
    paramVal = ito::Param("isUSB3", ito::ParamBase::Int, 0, 1, 0, tr("Indicates if the device is a USB3 device").toLatin1().data());
    m_initParamsOpt.append(paramVal);
}

//----------------------------------------------------------------------------------------------------------------------------------
AvantesSDKInterface::~AvantesSDKInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------


std::string AvantesSDK:: returnErrorCodes(int errCode)
{
    switch (errCode)
        {
        case ERR_SUCCESS:
            return "ERR_SUCCESS";
            break;
        case ERR_INVALID_PARAMETER:
            return "ERR_INVALID_PARAMETER";
            break;
        case ERR_OPERATION_NOT_SUPPORTED:
            return "ERR_OPERATION_NOT_SUPPORTED";
            break;
        case ERR_DEVICE_NOT_FOUND:
            return "ERR_DEVICE_NOT_FOUND";
            break;
        case ERR_OPERATION_PENDING:
            return "ERR_OPERATION_PENDING";
            break;
        case ERR_TIMEOUT:
            return "ERR_TIMEOUT";
            break;
        case ERR_INVALID_PASSWORD:
            return "ERR_INVALID_PASSWORD";
            break;
        case ERR_INVALID_MEAS_DATA:
            return "ERR_INVALID_MEAS_DATA";
            break;
        case ERR_INVALID_SIZE:
            return "ERR_INVALID_SIZE";
            break;
        case ERR_INVALID_PIXEL_RANGE:
            return "ERR_INVALID_PIXEL_RANGE";
            break;
        case ERR_INVALID_INT_TIME:
            return "ERR_INVALID_INT_TIME";
            break;
        case ERR_INVALID_COMBINATION:
            return "ERR_INVALID_COMBINATION";
            break;
        case ERR_INVALID_CONFIGURATION:
            return "ERR_INVALID_CONFIGURATION";
            break;
        case ERR_NO_MEAS_BUFFER_AVAIL:
            return "ERR_NO_MEAS_BUFFER_AVAIL";
            break;
        case ERR_UNKNOWN:
            return "ERR_UNKNOWN";
            break;
        case ERR_COMMUNICATION:
            return "ERR_COMMUNICATION + ERR_CONNECTION_FAILURE";
            break;
        case ERR_NO_SPECTRA_IN_RAM:
            return "ERR_NO_SPECTRA_IN_RAM";
            break;
        case ERR_INVALID_DLL_VERSION:
            return "ERR_INVALID_DLL_VERSION";
            break;
        case ERR_NO_MEMORY:
            return "ERR_NO_MEMORY";
            break;
        case ERR_DLL_INITIALISATION:
            return "ERR_DLL_INITIALISATION";
            break;
        case ERR_INVALID_STATE:
            return "ERR_INVALID_STATE";
            break;
        case ERR_INVALID_REPLY:
            return "ERR_INVALID_REPLY";
            break;   
        case ERR_ACCESS:
            return "ERR_ACCESS";
            break;  
        case ERR_INTERNAL_READ:
            return "ERR_INTERNAL_READ";
            break; 
        case ERR_INTERNAL_WRITE:
            return "ERR_INTERNAL_WRITE";
            break; 
        case ERR_ETHCONN_REUSE:
            return "ERR_ETHCONN_REUSE";
            break; 
        case ERR_INVALID_DEVICE_TYPE:
            return "ERR_INVALID_DEVICE_TYPE";
            break; 
        case ERR_SECURE_CFG_NOT_READ:
            return "ERR_SECURE_CFG_NOT_READ";
            break; 
        case ERR_UNEXPECTED_MEAS_RESPONSE:
            return "ERR_UNEXPECTED_MEAS_RESPONSE";
            break;     
        case ERR_MEAS_STOPPED:
            return "ERR_MEAS_STOPPED";
            break;    
        default:
            return "ERR_UNKNOWN_TYPE";
            break;     
        }
}


//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal AvantesSDK::showConfDialog(void)
{
    ito::RetVal retValue(ito::retOk);
    return retValue;
}
//----------------------------------------------------------------------------------------------------------------------------------
AvantesSDK::AvantesSDK() :
    AddInGrabber(),
    m_isgrabbing(false)
{
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "AvantesSDK", "plugin name");
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("integration_time", ito::ParamBase::Double, 0.00001, (double)MAX_INTEGRATION_TIME / 1000.0, 0.002, tr("Integration time of CCD programmed in s, some devices do not accept the full range of allowed values (see SDK for real minimum value of your device).").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    int roi[] = {0, 0, 4096, 1};
    paramVal = ito::Param("roi", ito::ParamBase::IntArray, 4, roi, tr("ROI (x,y,width,height)").toLatin1().data());
    ito::RectMeta *rm = new ito::RectMeta(ito::RangeMeta(0, roi[2]-1), ito::RangeMeta(0, roi[3]-1,1,1,1,1));
    paramVal.setMeta(rm, true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("sizex", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 4096, 4096, tr("current width of ROI").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("sizey", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 1, 1, tr("current height").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("bpp", ito::ParamBase::Int | ito::ParamBase::Readonly, 64, 64, 64, tr("Bit depth. The output object is float64 for all cases.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("average", ito::ParamBase::Int, 1, 65000, 1, tr("Number of averages for every frame").toLatin1().data()); //0xffffffff --> timeout, also in libusb
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("dark_correction", ito::ParamBase::Int, 0, 2, 0, tr("Some detectors have dark pixels, that can be used for a dark detection. If enabled, the output \n\
dataObject will always be float32. Static (1) subtracts the mean value of all dark pixels from all values. \n\
Dynamic (2) is only available for some devices (see if dyn. dark correction is enabled in the software \n\
SDK) and subtracts different mean values for odd and even pixels. Off (0): default.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("lambda_coeffs", ito::ParamBase::DoubleArray | ito::ParamBase::Readonly, NULL, tr("Coefficients for polynom that determines lambda_table (lambda_table[idx] = c[0] + c[1]*idx + c[2]*idx^2 + ... + c[4]*idx^4)").toLatin1().data());
    double coeffs[NR_WAVELEN_POL_COEF];
    paramVal.setVal<double*>(coeffs, NR_WAVELEN_POL_COEF);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("lambda_table", ito::ParamBase::DoubleArray | ito::ParamBase::Readonly, NULL, tr("Vector with the wavelength of all active pixels").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("serial_number", ito::ParamBase::String | ito::ParamBase::Readonly, NULL, tr("Serial number of spectrometer. Same as identifier.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("detector_name", ito::ParamBase::String | ito::ParamBase::Readonly, NULL, tr("Name of the detector.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    //now create dock widget for this plugin

    if (hasGuiSupport())
    {
        DockWidgetAvantesSDK *toolbox = new DockWidgetAvantesSDK(this);
        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas; //areas where the toolbox can be positioned (see Qt documentation)
        //define some features, saying if the toolbox can be closed, can be undocked (floatable) and moved...
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | \
            QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        //register the toolbox
        createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, toolbox);
    }


}

//----------------------------------------------------------------------------------------------------------------------------------
AvantesSDK::~AvantesSDK()
{
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvantesSDK::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    int l_Ret = ERR_SUCCESS;
    int la_Port = 0;
    unsigned int         l_Size = 0;
	unsigned int         l_RequiredSize = 0;
	int                  l_NrDevices = 0;
	char*                l_pDataDiscoDevs = NULL; // Data pointer for list of discovered devices
	char*				 l_pDataIds = NULL; // Data pointer for list of IDs
	AvsIdentityType*     l_pId = NULL;
	BroadcastAnswerType* l_pAnswer = NULL;

    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    // init communication
    l_Ret = AVS_Init(la_Port);
    if(l_Ret < 0)
    {
        retValue += ito::RetVal(ito::retError, 0, "ERROR_AVS_Init");
        AVS_Done();
    }

    // update 
    l_NrDevices = AVS_UpdateUSBDevices();

    // get serialnumbers
    l_NrDevices = AVS_GetList(0, &l_RequiredSize, NULL);
    l_pDataIds = new char[l_RequiredSize];
    l_pId = (AvsIdentityType*)l_pDataIds;
    l_Size = l_RequiredSize;
    l_NrDevices = AVS_GetList(l_Size, &l_RequiredSize, l_pId);

    if(l_NrDevices < 0)
    {
        retValue += ito::RetVal(ito::retError, 0, "ERROR_INVALID_SIZE; AVS_GetList: l_NrDevices < 0");
    }else
    {
        m_params["serial_number"].setVal<char*>(l_pId->SerialNumber); //[idx]
    }


    // Activation
    hDevice = AVS_Activate(l_pId);

    // Heartbeat
    HeartbeatRespType lHbResp;
	HeartbeatReqType lHbReq = 0; // Just heartbeat
    int l_Res = AVS_Heartbeat(hDevice, &lHbReq, &lHbResp);

    // Get Parameter
    DeviceConfigType l_DeviceData;
    l_Size = sizeof(DeviceConfigType);
    l_Res = AVS_GetParameter(hDevice, l_Size, &l_Size, &l_DeviceData);

    if (l_Res != ERR_SUCCESS)
    {
        retValue += ito::RetVal(ito::retError, 0, (const char*) returnErrorCodes(l_Res).c_str());
    }
    else
    {
        switch (l_DeviceData.m_Detector.m_SensorType)
        {
        case SENS_HAMS8378_256:
            m_params["detector_name"].setVal<const char*>("HAMS8378_256");
            break;
        case SENS_HAMS8378_1024:
            m_params["detector_name"].setVal<const char*>("HAMS8378_1024");
            break;
        case SENS_ILX554:
            m_params["detector_name"].setVal<const char*>("ILX554");
            break;
        case SENS_HAMS9201:
            m_params["detector_name"].setVal<const char*>("HAMS9201");
            break;
        case SENS_TCD1304:
            m_params["detector_name"].setVal<const char*>("TCD1304");
            break;
        case SENS_TSL1301:
            m_params["detector_name"].setVal<const char*>("TSL1301");
            break;
        case SENS_TSL1401:
            m_params["detector_name"].setVal<const char*>("TSL1401");
            break;
        case SENS_HAMS8378_512:
            m_params["detector_name"].setVal<const char*>("HAMS8378_512");
            break;
        case SENS_HAMS9840:
            m_params["detector_name"].setVal<const char*>("HAMS9840");
            break;
        case SENS_ILX511:
            m_params["detector_name"].setVal<const char*>("ILX511");
            break;
        case SENS_HAMS10420_11850:
            m_params["detector_name"].setVal<const char*>("HAMS10420_11850");
            break;
        case SENS_HAMS11071_2048X64:
            m_params["detector_name"].setVal<const char*>("HAMS11071_2048X64");
            break;
        case SENS_HAMS7031_11501:
            m_params["detector_name"].setVal<const char*>("HAMS7031_11501");
            break;
        case SENS_HAMS7031_1024X58:
            m_params["detector_name"].setVal<const char*>("HAMS7031_1024X58");
            break;
        case SENS_HAMS11071_2048X16:
            m_params["detector_name"].setVal<const char*>("HAMS11071_2048X16");
            break;
        case SENS_HAMS11155_2048:
            m_params["detector_name"].setVal<const char*>("HAMS11155_2048");
            break;
        case SENS_SU256LSB:
            m_params["detector_name"].setVal<const char*>("SU256LSB");
            break;
        case SENS_SU512LDB:
            m_params["detector_name"].setVal<const char*>("SU512LDB");
            break;
        case SENS_HAMS11638:
            m_params["detector_name"].setVal<const char*>("HAMS11638");
            break;
        case SENS_HAMS11639:
            m_params["detector_name"].setVal<const char*>("HAMS11639");
            break;
        case SENS_HAMS12443:
            m_params["detector_name"].setVal<const char*>("HAMS12443");
            break;
        case SENS_HAMG9208_512:
            m_params["detector_name"].setVal<const char*>("HAMG9208_512");
            break;
        case SENS_HAMS12198_1024:
            m_params["detector_name"].setVal<const char*>("HAMS12198_1024");
            break;
        case SENS_HAMS11155_2048_02_SINGLE:
            m_params["detector_name"].setVal<const char*>("HAMS11155_2048_02_SINGLE");
            break;
        case SENS_HAMS11155_2048_02_DIFF:
            m_params["detector_name"].setVal<const char*>("HAMS11155_2048_02_DIFF");
            break;

        default:
            m_params["detector_name"].setVal<const char*>("unknown");
            break;
        }

        // Retrieve current device config and set to readonly. Dark correction value can only be changed in Avantes Avaspec.
        m_params["dark_correction"].setVal<int>(l_DeviceData.m_StandAlone.m_Meas.m_CorDynDark.m_Enable);
        m_params["dark_correction"].setFlags(ito::ParamBase::Readonly);
    }
            
    
    double m_pLambda[4096];
    

    l_Res = AVS_GetNumPixels(hDevice, &nrPixels);
    if (ERR_SUCCESS != l_Res)
    {
        retValue += ito::RetVal(ito::retError, 0, returnErrorCodes(l_Res).c_str());
    }    

    l_Res = AVS_GetLambda(hDevice, m_pLambda);
    if (ERR_SUCCESS != l_Res) 
    {
        retValue += ito::RetVal(ito::retError, 0, returnErrorCodes(l_Res).c_str());
    }
    else
    {
        m_params["lambda_coeffs"].setVal<float*>(l_DeviceData.m_Detector.m_aFit, NR_WAVELEN_POL_COEF);

        m_params["lambda_table"].setVal<double*>(m_pLambda, nrPixels);
    }

    m_params["sizex"].setVal<int>(nrPixels);
    m_params["sizex"].setMeta(new ito::IntMeta(1, nrPixels, 1), true);
    m_params["sizex"].setFlags(ito::ParamBase::Readonly);

    m_params["roi"].getVal<int*>()[2] = nrPixels;
    m_params["roi"].setMeta(new ito::RectMeta(ito::RangeMeta(0, nrPixels-1, 1, 1, nrPixels, 1), ito::RangeMeta(0, 0)), true);

    m_params["bpp"].setVal<int>(ito::tFloat64);
    m_params["bpp"].setFlags(ito::ParamBase::Readonly);

    emit parametersChanged(m_params);

    setIdentifier(QString::number(getID()));

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    setInitialized(true); //init method has been finished (independent on retval)
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvantesSDK::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    int l_Res = AVS_StopMeasure(hDevice);
    if (ERR_SUCCESS != l_Res)
    {
        retValue += ito::RetVal(ito::retError, 0, returnErrorCodes(l_Res).c_str());
    }
    AVS_Deactivate(hDevice);
    AVS_Done();

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \details This method copies the complete tparam of the corresponding parameter to val

    \param [in,out] val  is a input of type::tparam containing name, value and further informations
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk in case that everything is ok, else retError
    \sa ito::tParam, ItomSharedSemaphore
*/
ito::RetVal AvantesSDK::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;
    QString key;
    bool hasIndex = false;
    int index;
    QString suffix;
    QMap<QString,ito::Param>::iterator it;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if(retValue == ito::retOk)
    {
        //gets the parameter key from m_params map (read-only is allowed, since we only want to get the value).
        retValue += apiGetParamFromMapByKey(m_params, key, it, false);
    }

    if(!retValue.containsError())
    {
        if (hasIndex)
        {
            *val = apiGetParam(*it, hasIndex, index, retValue);
        }
        else
        {
            *val = *it;
        }
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail This method copies the value of val to to the m_params-parameter and sets the corresponding camera parameters.

    \param [in] val  is a input of type::tparam containing name, value and further informations
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk in case that everything is ok, else retError
    \sa ito::tParam, ItomSharedSemaphore
*/
ito::RetVal AvantesSDK::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex;
    int index;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName( val->getName(), key, hasIndex, index, suffix );

    if(!retValue.containsError())
    {
        //gets the parameter key from m_params map (read-only is not allowed and leads to ito::retError).
        retValue += apiGetParamFromMapByKey(m_params, key, it, true);
    }

    if(!retValue.containsError())
    {
        //if you program for itom 1.4.0 or higher (Interface version >= 1.3.1) you should use this
        //API method instead of the one above: The difference is, that incoming parameters that are
        //compatible but do not have the same type than the corresponding m_params value are cast
        //to the type of the internal parameter and incoming double values are rounded to the
        //next value (depending on a possible step size, if different than 0.0)
        retValue += apiValidateAndCastParam(*it, *val, false, true, true);
    }

    if(!retValue.containsError())
    {
        if(key == "integration_time" || key == "roi" || key == "average" || key == "dark_correction")
        {
            int started = grabberStartedCount();
            if (started > 0)
            {
                this->setGrabberStarted(1);
                retValue += this->stopDevice(NULL);
            }

            if (!retValue.containsError())
            {
                //check the new value and if ok, assign it to the internal parameter
                retValue += it->copyValueFrom( &(*val) );

                if (!retValue.containsError() && key == "roi")
                {
                    m_params["sizex"].setVal<int>(val->getVal<int*>()[2]);
                    m_params["sizey"].setVal<int>(val->getVal<int*>()[3]);
                }
            }

            if (started > 0)
            {
                retValue += this->startDevice(NULL);
                setGrabberStarted(started);
            }

            
        }
        else
        {
            //all parameters that don't need further checks can simply be assigned
            //to the value in m_params (the rest is already checked above)
            retValue += it->copyValueFrom( &(*val) );
        }
    }

    if(!retValue.containsError())
    {
        emit parametersChanged(m_params); //send changed parameters to any connected dialogs or dock-widgets
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvantesSDK::startDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    incGrabberStarted(); //increment a counter to see how many times startDevice has been called

    if(grabberStartedCount() == 1)  //formaly grabberStartedCount() == 0
    {
        MeasConfigType config;   //SendMeasConfigType config;
        const int *roi = m_params["roi"].getVal<int*>();
        float int_time_ms = m_params["integration_time"].getVal<double>() * 1e3;
        uint32 average = m_params["average"].getVal<int>();

        /*
        config.prefix[00]=0x20;
        config.prefix[01]=0x00; //REVERSE: 0x00;
        config.prefix[02]=0x2B;   // length of the command
        config.prefix[03]=0x00;
        config.prefix[04]=0x05;   // prepare_measurement
        config.prefix[05]=0x00; //REVERSE: 0x00
        */

        config.m_StartPixel = roi[0];
        config.m_StopPixel = roi[0] + roi[2] - 1;
        config.m_IntegrationTime = int_time_ms;
        config.m_IntegrationDelay = 0;
        config.m_NrAverages = average;
        config.m_CorDynDark.m_Enable = 0;
        config.m_CorDynDark.m_ForgetPercentage = 0;
        config.m_Smoothing.m_SmoothPix = 0;
        config.m_Smoothing.m_SmoothModel = 0;
        config.m_SaturationDetection = 0;
        config.m_Trigger.m_Mode = 2; //Single scan
        config.m_Trigger.m_Source = 0; //Synchronized
        config.m_Trigger.m_SourceType = 0; //Edge
        config.m_Control.m_StrobeControl = 0;
        config.m_Control.m_LaserDelay = 0;
        config.m_Control.m_LaserWidth = 0;
        config.m_Control.m_LaserWaveLength = 0;
        config.m_Control.m_StoreToRam = 0;

        //send config
        int l_Res = AVS_PrepareMeasure(hDevice, &config);
        if (ERR_SUCCESS != l_Res) 
        {
        retValue += ito::RetVal(ito::retError, 0, returnErrorCodes(l_Res).c_str());
        }

    	/*
        int config_size = sizeof(config);
        int config_answer_size = m_answerLength;
        unsigned char config_answer[8];
        retValue += this->sendCommand((const char*)&config, config_size, config_answer, config_answer_size);
        if (!retValue.containsError())
        {
            if ((config_answer_size != m_answerLength) || (config_answer[4] != 0x85) || (config_answer[2] != 0x02))
            {
                if (config_answer[2] == 0x03) //error message, obtain the last missing two characters
                {
                    int len = 8 - config_answer_size;
                    retValue += sendCommand(NULL, 0, &(config_answer[config_answer_size]), len);

                    if (!retValue.containsError())
                    {
                        retValue += this->checkAnswerForError(config_answer, 0x85, false, "Setting configuration: ");
                    }
                    else
                    {
                        retValue += ito::RetVal(ito::retError, 0, "unknown error while setting configuration (v1).");
                    }
                }
                else
                {
                    retValue += ito::RetVal(ito::retError, 0, "unknown error while setting configuration (v2).");
                }
            }
        }
    }

    if (!retValue.containsError())
    {
        retValue += checkData();
        incGrabberStarted();
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();,
        */
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvantesSDK::stopDevice(ItomSharedSemaphore *waitCond)
{

    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    decGrabberStarted();

    if (grabberStartedCount() < 0)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("The grabber has already been stopped.").toLatin1().data());
        setGrabberStarted(0);
    }

    if(grabberStartedCount() == 0)
    {
        //stop measurement
        int l_Res = AVS_StopMeasure(hDevice);
        if (ERR_SUCCESS != l_Res) 
            {
            retValue += ito::RetVal(ito::retError, 0, returnErrorCodes(l_Res).c_str());
            }     

        //free all allocated image buffers of the camera
        

        /*
        unsigned char cmd[] = {0x20, 0x00, 0x02, 0x00, 0x1F stop_measurement, 0x00};
        unsigned char answer[8];
        int size = 8;
        retValue += sendCommand((char*)cmd, sizeof(cmd), answer, size);
        //retValue += checkAnswerForError(answer, 0x8F, true);
        */
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return ito::retOk;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvantesSDK::acquire(const int trigger, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if (grabberStartedCount() <= 0)
    {
        retValue = ito::RetVal(ito::retError, 0, tr("Tried to acquire without starting device").toLatin1().data());
    }
    else
    {
        m_isgrabbing = true;
    }

    //prepare measurement
    int l_Res = AVS_UseHighResAdc(hDevice, 1);
    if (ERR_SUCCESS != l_Res) 
        {
        retValue += ito::RetVal(ito::retError, 0, returnErrorCodes(l_Res).c_str());
        } 

    // start measurement
    l_Res = AVS_Measure(hDevice, NULL, 1); 
    if (ERR_SUCCESS != l_Res) 
        {
        retValue += ito::RetVal(ito::retError, 0, returnErrorCodes(l_Res).c_str());
        } 

    while (AVS_PollScan(hDevice)<1){
        Sleep(100);
        }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    double l_pSpectrum[MAX_NR_PIXELS];
    unsigned int l_Time;
    l_Res = AVS_GetScopeData(hDevice, &l_Time, l_pSpectrum);
    if (ERR_SUCCESS == l_Res)
    {
        memcpy( (ito::float64*)m_data.rowPtr(0, 0), l_pSpectrum, nrPixels*sizeof(double));	// We want to get here !!! unsigned short nrPixels; aus init
    }
    

    /*
    if (!retValue.containsError())
    {
        m_isgrabbing = true;
        uint32 average = m_params["average"].getVal<int>();
        int darkCorrection = m_params["dark_correction"].getVal<int>() > 0;
        if (m_numberOfCorrectionValues == 0)
        {
            darkCorrection = 0;
        }
        m_acquisitionRetVal = ito::retOk;
        int xsize = m_data.getSize(1);

        int request_size;
        if (average <= 1)
        {
            //at first get the first 6 bytes (prefix) to check for the size of
            //the data. Then obtain the data.
            memset(singleMeasdata.pixels, 0, sizeof(singleMeasdata.pixels));
            request_size = sizeof(singleMeasdata.prefix);
            m_acquisitionRetVal += readWithFixedLength((char*)&singleMeasdata, request_size);
            //m_acquisitionRetVal += checkAnswerForError((unsigned char*)&singleMeasdata, 0xB0, false);
            //request_size = singleMeasdata.prefix[3] * 256 + singleMeasdata.prefix[2]+ m_imageBufferLengthModifier;
            //m_acquisitionRetVal += readWithFixedLength((char*)&(singleMeasdata.timestamp), request_size);

            if (!m_acquisitionRetVal.containsError())
            {

                request_size += 6;
                int expected_size = (sizeof(AvsSingleMeasdata) - sizeof(singleMeasdata.pixels) + (xsize + std::max(0, m_numberDeadPixels)) * sizeof(uint16));
                if (request_size != expected_size)
                {
                    if (m_numberDeadPixels == -1)
                    {
                        m_numberDeadPixels = (request_size - expected_size) / sizeof(uint16);
                    }
                    else
                    {
                        qDebug() << "received buffer has " << request_size << " bytes. " << expected_size << " bytes expected.";
                    }
                }


                ito::float32 darkOddEvenCorrection[] = {0.0, 0.0}; //[even - mean, odd - mean]

                if (m_numberOfCorrectionValues > 0)
                {
                    ito::int32 darkEven = 0;
                    ito::int32 darkOdd = 0;
                    for (int teller = m_startCorrectionIndex; teller < m_startCorrectionIndex + m_numberOfCorrectionValues; teller += 2)
                    {
                        darkEven += singleMeasdata.pixels[teller];
                        darkOdd += singleMeasdata.pixels[teller + 1];
                    }
                    m_data.setTag("dark", (double)(darkEven + darkOdd) / m_numberOfCorrectionValues);
                    if (darkCorrection == 1)
                    {
                        darkOddEvenCorrection[0] = darkOddEvenCorrection[1] = (ito::float32)(darkEven + darkOdd) / m_numberOfCorrectionValues;
                    }
                    else if (darkCorrection == 2)
                    {
                        darkOddEvenCorrection[0] = 2 * (ito::float32)darkEven / m_numberOfCorrectionValues;
                        darkOddEvenCorrection[1] = 2 * (ito::float32)darkOdd / m_numberOfCorrectionValues;
                    }
                }

                if (darkCorrection > 0)
                {
                    ito::float32 *vals = (ito::float32*)m_data.rowPtr(0, 0);
                    for (int teller = 0; teller < xsize; ++teller)
                    {
                        vals[teller] = (ito::float32)swap16IfNeeded(singleMeasdata.pixels[teller + m_numberDeadPixels]) - darkOddEvenCorrection[(teller + m_numberDeadPixels) % 2];
                    }
                }
                else
                {
                    ito::uint16 *vals = (ito::uint16*)m_data.rowPtr(0, 0);
                    for (int teller = 0; teller < xsize; ++teller)
                    {
                        vals[teller] = singleMeasdata.pixels[teller + m_numberDeadPixels];
                    }
                }

                m_data.setTag("timestamp", (double)singleMeasdata.timestamp) * 1e-5; //timestamp is in 10us units
            }
        }
        else //average > 1
        {
            //at first get the first 6 bytes (prefix) to check for the size of
            //the data. Then obtain the data.
            
            memset(multiMeasdata.pixels, 0, sizeof(multiMeasdata.pixels));
            request_size = sizeof(multiMeasdata.prefix);
            if (m_readAveragedImageInOneChunk)// for some reason the entire multiMeasdata must be read in a single chunk from device this may be realted to usb3
            {
                //we can not read prefix first to get the number of bytes-> calculate it
                request_size += m_params["sizex"].getVal<int>() * 4 + sizeof(multiMeasdata.timestamp) + sizeof(multiMeasdata.averages)+m_imageBufferLengthModifier;
                if (m_numberDeadPixels > 0)
                {
                    request_size += m_numberDeadPixels * 4;
                }
                m_acquisitionRetVal += readWithFixedLength((char*)&multiMeasdata, request_size);
                m_acquisitionRetVal += checkAnswerForError((unsigned char*)&multiMeasdata, 0xB1, false);
            }
            else
            {
                m_acquisitionRetVal += readWithFixedLength((char*)&multiMeasdata, request_size);
                m_acquisitionRetVal += checkAnswerForError((unsigned char*)&multiMeasdata, 0xB1, false);
                request_size = multiMeasdata.prefix[3] * 256 + multiMeasdata.prefix[2] + m_imageBufferLengthModifier;
                m_acquisitionRetVal += readWithFixedLength((char*)&(multiMeasdata.timestamp), request_size);
            }
            if (!m_acquisitionRetVal.containsError())
            {
                request_size += 6;
                int expected_size = (sizeof(AvsMultiMeasdata) - sizeof(multiMeasdata.pixels) + (xsize + std::max(0, m_numberDeadPixels)) * sizeof(uint32));
                if (request_size != expected_size)
                {
                    if (m_numberDeadPixels == -1)
                    {
                        m_numberDeadPixels = (request_size - expected_size) / sizeof(uint32);
                    }
                    else
                    {
                        qDebug() << "received buffer has " << request_size << " bytes. " << expected_size << " bytes expected.";
                    }
                }

                ito::float32 darkOddEvenCorrection[] = {0.0, 0.0}; //[even - mean, odd - mean] multiplied by average

                if (m_numberOfCorrectionValues > 0)
                {
                    ito::uint32 darkEven = 0;
                    ito::uint32 darkOdd = 0;
                    for (int teller = m_startCorrectionIndex; teller < m_startCorrectionIndex + m_numberOfCorrectionValues; teller += 2)
                    {
                        darkEven += multiMeasdata.pixels[teller];
                        darkOdd += multiMeasdata.pixels[teller + 1];
                    }
                    m_data.setTag("dark", average * (double)(darkEven + darkOdd) / m_numberOfCorrectionValues);

                    if (darkCorrection == 1)
                    {
                        darkOddEvenCorrection[0] = darkOddEvenCorrection[1] = (ito::float32)(darkEven + darkOdd) / m_numberOfCorrectionValues;
                    }
                    else if (darkCorrection == 2)
                    {
                        darkOddEvenCorrection[0] = 2 * (ito::float32)darkEven / m_numberOfCorrectionValues;
                        darkOddEvenCorrection[1] = 2 * (ito::float32)darkOdd / m_numberOfCorrectionValues;
                    }
                }

                if (darkCorrection > 0)
                {
                    ito::float32 *vals = (ito::float32*)m_data.rowPtr(0, 0);
                    for (int teller = 0; teller < xsize; ++teller)
                    {
                        vals[teller] = ((ito::float32)swap32IfNeeded(multiMeasdata.pixels[teller + m_numberDeadPixels]) - darkOddEvenCorrection[((teller + m_numberDeadPixels) % 2)]) / (ito::float32)average;
                    }
                }
                else
                {
                    ito::float32 *vals = (ito::float32*)m_data.rowPtr(0, 0);
                    for (int teller = 0; teller < xsize; ++teller)
                    {
                        vals[teller] = ((ito::float32)swap32IfNeeded(multiMeasdata.pixels[teller + m_numberDeadPixels])) / (ito::float32)average;
                    }
                }



                m_data.setTag("timestamp", (double)multiMeasdata.timestamp) * 1e-5; //timestamp is in 10us units
            }
        }

        unsigned char cmd2[] = {0x21, 0x00, 0x02, 0x00, 0xC0 /*acknowledge, 0x00};
        m_acquisitionRetVal += m_pUsb->setVal((const char*)cmd2, sizeof(cmd2), NULL);
    }
    */

    return retValue + m_acquisitionRetVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvantesSDK::retrieveData(ito::DataObject *externalDataObject)
{
    ito::RetVal retValue(ito::retOk);

    ito::DataObject *dataObj = externalDataObject ? externalDataObject : &m_data;

    bool hasListeners = (m_autoGrabbingListeners.size() > 0);
    bool copyExternal = (externalDataObject != NULL);

    const int bufferWidth = m_params["sizex"].getVal<int>();
    const int bufferHeight = m_params["sizey"].getVal<int>();

    /*
    bool hasListeners = false;
    bool copyExternal = false;
    if(m_autoGrabbingListeners.size() > 0)
    {
        hasListeners = true;
    }
    if(externalDataObject != NULL)
    {
        copyExternal = true;
        retVal += checkData(externalDataObject);
    }*/

//  retVal += externalDataObject->copyFromData2D<ito::tFloat64>((ito::tFloat64*)m_data.rowPtr(0,0), m_params["sizex"].getVal<int>(), m_params["sizey"].getVal<int>()); // Should be replaced by ROI? And is this reasonable, because Double is needed?

    if (m_isgrabbing == false)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("Tried to get picture without triggering exposure").toLatin1().data());
    }
    else
    {
        //step 1: create m_data (if not yet available)
        if (externalDataObject && hasListeners)
        {
            //retValue += checkData(NULL); //update m_data
            //retValue += checkData(externalDataObject); //update external object
        }
        else
        {
            //retValue += checkData(externalDataObject); //update external object or m_data
        }

        if (!retValue.containsError())
        {
            if (m_data.getType() == ito::tFloat64)
            {
                if (copyExternal)
                {
                    retValue += externalDataObject->copyFromData2D<ito::float64>((ito::float64*) bufferPtr, bufferWidth, bufferHeight);
                }
                if (!copyExternal || hasListeners)
                {
                    retValue += m_data.copyFromData2D<ito::float64>((ito::float64*) bufferPtr, bufferWidth, bufferHeight);
                }
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 1002, tr("copying image buffer not possible since unsupported type.").toLatin1().data());
            }
        }

        m_isgrabbing = false;
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*ito::RetVal AvantesSDK::checkData(ito::DataObject *externalDataObject)
{
    int futureHeight = m_params["sizey"].getVal<int>();
    int futureWidth = m_params["sizex"].getVal<int>();
    int futureType;

    int bpp = m_params["bpp"].getVal<int>();
    int darkCorrection = m_params["dark_correction"].getVal<int>();
    int average = m_params["average"].getVal<int>();

    if (bpp <= 16 && (darkCorrection == 0 || m_numberOfCorrectionValues == 0) && average == 1 )
    {
        futureType = ito::tUInt16;
    }
    else
    {
        futureType = ito::tFloat32;
    }

    if (externalDataObject == NULL)
    {
        if (m_data.getDims() < 2 || m_data.getSize(0) != (unsigned int)futureHeight || m_data.getSize(1) != (unsigned int)futureWidth || m_data.getType() != futureType)
        {
            m_data = ito::DataObject(futureHeight,futureWidth,futureType);
        }
    }
    else
    {
        int dims = externalDataObject->getDims();
        if (externalDataObject->getDims() == 0)
        {
            *externalDataObject = ito::DataObject(futureHeight,futureWidth,futureType);
        }
        else if (externalDataObject->calcNumMats () != 1)
        {
            return ito::RetVal(ito::retError, 0, tr("Error during check data, external dataObject invalid. Object has more or less than 1 plane. It must be of right size and type or an uninitilized image.").toLatin1().data());
        }
        else if (externalDataObject->getSize(dims - 2) != (unsigned int)futureHeight || externalDataObject->getSize(dims - 1) != (unsigned int)futureWidth || externalDataObject->getType() != futureType)
        {
            return ito::RetVal(ito::retError, 0, tr("Error during check data, external dataObject invalid. Object must be of right size and type or a uninitilized image.").toLatin1().data());
        }
    }

    return ito::retOk;
}
*/
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvantesSDK::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);

    retValue += retrieveData();

    if(!retValue.containsError())
    {
        sendDataToListeners(0); //don't wait for live data, since user should get the data as fast as possible.

        if(dObj)
        {
            (*dObj) = this->m_data; // vlt ohne this->
        }
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvantesSDK::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);

    if(!dObj)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Empty object handle retrieved from caller").toLatin1().data());
    }
    /*else
    {
        retValue += checkData(dObj);
    }*/

    if(!retValue.containsError())
    {
        retValue += retrieveData(dObj); //checkData is executed inside of retrieveData
    }

    if(!retValue.containsError())
    {
        sendDataToListeners(0); 
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
// Moved to addInGrabber.cpp, equal for all grabbers / ADDA

//----------------------------------------------------------------------------------------------------------------------------------
/* never used
void AvantesSDK::updateParameters(QMap<QString, ito::ParamBase> params)
{
    foreach(const ito::ParamBase &param1, params)
    {
        setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase(param1)), NULL);
    }
}
*/ 




//-------------------------------------------------------------------------------------------------------------------------------------------------
/* deeted because no more libUSB use
ito::RetVal AvantesSDK::sendCommand(const char* cmd, int cmd_size, unsigned char* buf, int &buf_size)
{
    ito::RetVal retVal;

    if (cmd_size > 0)
    {
        retVal = m_pUsb->setVal(cmd, cmd_size, NULL);
    }

    if (!retVal.containsError() && buf != NULL)
    {
        QSharedPointer<int> chuck_size(new int);
        int read_bytes = 0;

        QSharedPointer<char> chunk_buffer((char*)buf, idleCharDeleter);
        *chuck_size = buf_size - read_bytes;
        retVal += m_pUsb->getVal(chunk_buffer, chuck_size, NULL);
        read_bytes += *chuck_size;

        if (read_bytes < buf_size)
        {
            QElapsedTimer timer;
            timer.start();
            while (read_bytes < buf_size && timer.elapsed() < 2000 && !retVal.containsError())
            {
                QSharedPointer<char> chunk_buffer((char*)(&(buf[read_bytes])), idleCharDeleter);
                *chuck_size = (buf_size - read_bytes);
                retVal += m_pUsb->getVal(chunk_buffer, chuck_size, NULL);
                read_bytes += *chuck_size;
            }
        }

        if (buf_size > read_bytes)
        {
            retVal += ito::RetVal(ito::retError, 0, "timeout while reading required data from usb port");
        }

        buf_size = read_bytes;
    }

    return retVal;
}
*/
//-------------------------------------------------------------------------------------------------------------------------------------------------
/*ito::RetVal AvantesSDK::readWithFixedLength(char* buf, int &buf_size)
{

    
    if (ERR_SUCCESS == AVS_GetScopeData(hDevice,&l_Time,l_pSpectrum))
    {
        // ui.plot->update_plot();
        if (ERR_SUCCESS == AVS_GetSaturatedPixels(hDevice,l_saturated))
        {
            bool saturated = false;
            extern unsigned short m_StartPixel;
            extern unsigned short m_StopPixel;
            for (int j = 0; j <= m_StopPixel-m_StartPixel; j++) {
                saturated = saturated || (l_saturated[j] != 0);
            }
            ui.SaturatedChk->setChecked(saturated);
        }
        l_Dif = l_Time - m_PreviousTimeStamp;  // l_Time in 10 us ticks
        m_PreviousTimeStamp = l_Time;
        if (l_Dif != 0)
        {
            ui.LastScanEdt->setText(QString("%1").arg(l_Dif/100.0,7,'f',2)); //millisec
        }
    }
    

    ito::RetVal retVal;
    QSharedPointer<int> chuck_size(new int);
    int read_bytes = 0;

    QSharedPointer<char> chunk_buffer(buf, idleCharDeleter);

    *chuck_size = buf_size - read_bytes;
    retVal += m_pUsb->getVal(chunk_buffer, chuck_size, NULL);
    read_bytes += *chuck_size;

    if (read_bytes < buf_size)
    {
        QElapsedTimer timer;
        timer.start();
        while (read_bytes < buf_size && timer.elapsed() < 2000 && !retVal.containsError())
        {
            QSharedPointer<char> chunk_buffer(&(buf[read_bytes]), idleCharDeleter);
            *chuck_size = (buf_size - read_bytes);
            retVal += m_pUsb->getVal(chunk_buffer, chuck_size, NULL);
            read_bytes += *chuck_size;
        }
    }

    if (buf_size > read_bytes)
    {
        retVal += ito::RetVal(ito::retError, 0, "timeout while reading required data from usb port");
    }

    buf_size = read_bytes;
    return retVal;
}*/
// AvantesSDK

//-------------------------------------------------------------------------------------------------------------------------------------------------
/*void AvantesSDK::dummyRead()
{
    ito::RetVal retValue;

    //dummy read to clear input buffer
    char buf[1024];
    QSharedPointer<int> size(new int);
    *size = 1024;
    QSharedPointer<char> buffer(buf, idleCharDeleter);
    m_pUsb->getVal(buffer, size, NULL);
}

*/
//-------------------------------------------------------------------------------------------------------------------------------------------------
/*ito::RetVal AvantesSDK::checkAnswerForError(const unsigned char* buf, const unsigned char &desiredCmd, bool warningNotError = false, const char *prefix /*= "")
{
    if (buf[4] == desiredCmd)
    {
        return ito::retOk;
    }
    else if (buf[2] == 0x03 && buf[4] == 0x00)
    {
        ito::RetVal retVal;
        switch (buf[6])
        {
        case 0x00:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "%serror %i: unknown", prefix, buf[6]);
            break;
        case 0x01:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "%serror %i: invalid parameter", prefix, buf[6]);
            break;
        case 0x02:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "%serror %i: invalid password", prefix, buf[6]);
            break;
        case 0x03:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "%serror %i: invalid command", prefix, buf[6]);
            break;
        case 0x04:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "%serror %i: invalid size", prefix, buf[6]);
            break;
        case 0x05:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "%serror %i: measurement pending", prefix, buf[6]);
            break;
        case 0x06:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "%serror %i: invalid pixel range", prefix, buf[6]);
            break;
        case 0x07:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "%serror %i: invalid integration time", prefix, buf[6]);
            break;
        case 0x08:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "%serror %i: operation not supported", prefix, buf[6]);
            break;
        case 0x09:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "%serror %i: invalid combination", prefix, buf[6]);
            break;
        case 0x0A:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "%serror %i: no buffer available", prefix, buf[6]);
            break;
        case 0x0B:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "%serror %i: no spectra available", prefix, buf[6]);
            break;
        case 0x0C:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "%serror %i: invalid state", prefix, buf[6]);
            break;
        case 0x0D:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "%serror %i: unexpected dma int", prefix, buf[6]);
            break;
        case 0x0E:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "%serror %i: invalid fpga file", prefix, buf[6]);
            break;
        default:
        //case 0x00:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "%serror %i", prefix, buf[6]);
        }
        return retVal;
    }
    else
    {
        return ito::RetVal(warningNotError ? ito::retWarning : ito::retError, 0, "unknown answer from device");
    }
}*/
//----------------------------------------------------------------------------------------------------------------------------------
//! slot called if the dock widget of the plugin becomes (in)visible
/*!
    Overwrite this method if the plugin has a dock widget. If so, you can connect the parametersChanged signal of the plugin
    with the dock widget once its becomes visible such that no resources are used if the dock widget is not visible. Right after
    a re-connection emit parametersChanged(m_params) in order to send the current status of all plugin parameters to the dock widget.
*/
void AvantesSDK::dockWidgetVisibilityChanged(bool visible)
{
    if (getDockWidget())
    {
        QWidget *widget = getDockWidget()->widget();
        if (visible)
        {
            connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), widget, SLOT(parametersChanged(QMap<QString, ito::Param>)));

            emit parametersChanged(m_params);
        }
        else
        {
            disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), widget, SLOT(parametersChanged(QMap<QString, ito::Param>)));
        }
    }
}

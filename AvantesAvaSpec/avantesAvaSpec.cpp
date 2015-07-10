/* ********************************************************************
    Plugin "AvantesAvaSpec" for itom software
    URL: http://www.bitbucket.org/itom/plugins
	Copyright (C) 2014, Institut f�r Technische Optik, Universit�t Stuttgart

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

#include "avantesAvaSpec.h"
#include "pluginVersion.h"

#include <QFile>
#include <qstring.h>
#include <qstringlist.h>
#include <qelapsedtimer.h>
#include <QtCore/QtPlugin>
#include "pluginVersion.h"

#include "dockWidgetAvantesAvaSpec.h"

#ifdef WIN32
    #include <Windows.h>
#else
    #define LOBYTE(w) ((unsigned char)(((w)) & 0xff))
    #define HIBYTE(w) ((unsigned char)((((w)) >> 8) & 0xff))
#endif

static unsigned long AvantesAvaSpec000Base = 0x10000L; // or 64k for Master Mode
static int AvantesAvaSpecADLow=31, AvantesAvaSpecADHigh=133;

float swapsingle(float floatin)
{
    union s
    {
        char sa[4];
        float res;
    } temp;
    temp.res = floatin;
    s floatout;
    for (int teller=0; teller<4; ++teller){
        floatout.sa[teller]=temp.sa[3-teller];
    }
    return floatout.res;
}

uint32 swap32(uint32 uint32in)
{
    union s
    {
        char sa[4];
        uint32 res;
    } temp;
    temp.res = uint32in;
    s uint32out;
    for (int teller=0; teller<4; ++teller){
        uint32out.sa[teller]=temp.sa[3-teller];
    }
    return uint32out.res;
}

uint16 swap16(uint16 uint16in)
{
    union s
    {
        char sa[2];
        uint16 res;
    } temp;
    temp.res = uint16in;
    s uint16out;
    for (int teller=0; teller<2; ++teller){
        uint16out.sa[teller]=temp.sa[1-teller];
    }
    return uint16out.res;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvantesAvaSpecInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(AvantesAvaSpec)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvantesAvaSpecInterface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(AvantesAvaSpec)
    return ito::retOk;
}


//----------------------------------------------------------------------------------------------------------------------------------
AvantesAvaSpecInterface::AvantesAvaSpecInterface() : AddInInterfaceBase()
{
    m_type = ito::typeDataIO | ito::typeGrabber;
    setObjectName("AvantesAvaSpec");

    m_description = QObject::tr("Avantes AvaSpec");
    m_detaildescription = QObject::tr("This DLL integrates the Avantis AvantesAvaSpec spectrometer series into itoM. It is used for CCM / CCSI with the AvaSpec 3468 USB-Spectrometer.");
    m_author = "M. Gronle, W. Lyda, ITO, University Stuttgart";
    m_license = QObject::tr("LGPL");
    m_version           = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer        = MINVERSION;
    m_maxItomVer        = MAXVERSION;
    m_aboutThis         = tr("");  

    m_autoLoadPolicy = ito::autoLoadNever;
    m_autoSavePolicy = ito::autoSaveNever;

    ito::Param paramVal = ito::Param("VendorID", ito::ParamBase::Int, 0, std::numeric_limits<int>::max(), 0x1992, tr("VendorID of spectrometer, 0x1992 for AvaSpec-3648, 0x471 for AvaSpec-ULS3648...").toLatin1().data());
    m_initParamsMand.append(paramVal);
    paramVal = ito::Param("ProductID", ito::ParamBase::Int, 0, std::numeric_limits<int>::max(), 0x0667, tr("ProductID of spectrometer, 0x0667 for spectrometer").toLatin1().data());
    m_initParamsMand.append(paramVal);
}

//----------------------------------------------------------------------------------------------------------------------------------
AvantesAvaSpecInterface::~AvantesAvaSpecInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
#if QT_VERSION < 0x050000
    Q_EXPORT_PLUGIN2(AvantesAvaSpecInterface, AvantesAvaSpecInterface)
#endif

//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal AvantesAvaSpec::showConfDialog(void)
{
    ito::RetVal retValue(ito::retOk);
    return retValue;
}
//----------------------------------------------------------------------------------------------------------------------------------
AvantesAvaSpec::AvantesAvaSpec() :  
    AddInGrabber(), 
    m_pUsb(NULL),
    m_isGrabbing(false)
{
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "AvantesAvaSpec", "PlugIn-Name");
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("integration_time", ito::ParamBase::Double, 0.00001, (double)MAX_INTEGRATION_TIME / 1000.0, 0.001, tr("Integrationtime of CCD programmed in s").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("frame_time", ito::ParamBase::Double | ito::ParamBase::Readonly, 0.0037, 0.0037, 0.0037, tr("Shortest time between two frames of CCD in s").toLatin1().data());
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

    paramVal = ito::Param("bpp", ito::ParamBase::Int | ito::ParamBase::Readonly, 16, 32, 16, tr("Bit depth. 16 (uint16), if single acquisition. 32 (float32), if averaging.").toLatin1().data()); 
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("average", ito::ParamBase::Int, 1, 10, 1, tr("Number of averages for every frame").toLatin1().data()); //0xffffffff --> timeout, also in libusb
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("time_out", ito::ParamBase::Double, 0.001, 4095.0, 1.0, tr("Timeout for grabbing in s").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("trigger_enable", ito::ParamBase::Int | ito::ParamBase::NoAutosave | ito::ParamBase::Readonly, 0, 0, 0, tr("Enable triggermode").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("lambda_coeffs", ito::ParamBase::DoubleArray | ito::ParamBase::Readonly, NULL, tr("Coefficients for polynom that determines lambda_table (lambda_table[idx] = c[0] + c[1]*idx + c[2]*idx^2 + ... + c[4]*idx^4)").toLatin1().data());
    double coeffs[NR_WAVELEN_POL_COEF];
    paramVal.setVal<double*>(coeffs, NR_WAVELEN_POL_COEF);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("lambda_table", ito::ParamBase::DoubleArray | ito::ParamBase::Readonly, NULL, tr("Vector with the wavelength of all active pixels").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("trigger_mode", ito::ParamBase::Int, -1, 2, 0, tr("Set Triggermode").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("serial_number", ito::ParamBase::String | ito::ParamBase::Readonly, NULL, tr("Serial number of spectrometer. Same as identifier.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    //now create dock widget for this plugin
   
    DockWidgetAvantesAvaSpec *toolbox = new DockWidgetAvantesAvaSpec(this);

    Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas; //areas where the toolbox can be positioned (see Qt documentation)

    //define some features, saying if the toolbox can be closed, can be undocked (floatable) and moved...
    QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | \
    QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;

    //register the toolbox
    createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, toolbox);
    

}

//----------------------------------------------------------------------------------------------------------------------------------
AvantesAvaSpec::~AvantesAvaSpec()
{
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvantesAvaSpec::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    
    ito::RetVal retValue(ito::retOk);

    //open libUSB as further plugin instance.
    int libUSBNo;
    QVector<ito::Param> *libUSBParamsMand = NULL;
    QVector<ito::Param> *libUSBParamsOpt = NULL;
    retValue += apiAddInGetInitParams("LibUSB", ito::typeDataIO | ito::typeRawIO, &libUSBNo, libUSBParamsMand, libUSBParamsOpt);
    
    if (!retValue.containsError())
    {
        QVector<ito::ParamBase> mands, opts;
        foreach(const ito::Param &p, *libUSBParamsMand)
        {
            mands.append(p);
        }
        foreach(const ito::Param &p, *libUSBParamsOpt)
        {
            opts.append(p);
        }
        mands[0].setVal<int>( paramsMand->at(0).getVal<int>() ); //VendorID
        mands[1].setVal<int>( paramsMand->at(1).getVal<int>() ); //ProductID
        mands[2].setVal<int>( 2 ); //endpoint ID

        retValue += apiAddInOpenDataIO("LibUSB", libUSBNo, false, &mands, &opts, m_pUsb);
    }

    if (!retValue.containsError())
    {
        retValue += m_pUsb->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("endpoint_read", ito::ParamBase::Int, 6)), NULL);
    }

    if (!retValue.containsError())
    {
        //get Serial Number
        unsigned char cmd[] = {0x20, 0x00, 0x02 /*length of command*/, 0x00, 0x13 /*get ident*/, 0x00};
        AvsIdentityType avs_id;
        int size = sizeof(avs_id);
        retValue += sendCommand((char*)cmd, sizeof(cmd), (unsigned char*)&avs_id, size);
        retValue += checkAnswerForError(avs_id.prefix, 0x93, false);

        if (!retValue.containsError())
        {
            setIdentifier(QString("Avantes %1").arg(QString(QByteArray(avs_id.SerialNumber,AVS_SERIAL_LEN))));
            m_params["serial_number"].setVal<char*>(avs_id.SerialNumber);
        }
    }

    if (!retValue.containsError())
    {
        //get Reset Reason
        unsigned char cmd[] = {0x20, 0x00, 0x02 /*length of command*/, 0x00, 0x12 /*get ident*/, 0x00};
        unsigned char answer[8];
        memset(answer, 0, 8 * sizeof(unsigned char));
        int size = 8;
        retValue += sendCommand((char*)cmd, sizeof(cmd), answer, size);
        retValue += checkAnswerForError(answer, 0x92, true);
    }

    if (!retValue.containsError())
    {
        //get device config
        unsigned char cmd[] = {0x20, 0x00, 0x02 /*length of command*/, 0x00, 0x01 /*get device config*/, 0x00};
        int size = sizeof(m_deviceConfig);
        retValue += sendCommand((char*)cmd, sizeof(cmd), (unsigned char*)(&m_deviceConfig), size);

        if (size != sizeof(m_deviceConfig) || m_deviceConfig.prefix[2] != 0xFE || m_deviceConfig.prefix[4] != 0x81)
        {
            retValue += ito::RetVal(ito::retError, 0, "error reading entire device configuration");
        }
        else
        {
            double *fit = m_params["lambda_coeffs"].getVal<double*>();
            for (int t = 0; t < NR_WAVELEN_POL_COEF; ++t)
            {
                fit[t] = swapsingle(m_deviceConfig.m_Detector.m_aFit[t]);
            }

            uint16 nrPixels = swap16(m_deviceConfig.m_Detector.m_NrPixels);
            double *lambda = new double[nrPixels];
            for (int t = 0; t < nrPixels; ++t)
            {
                lambda[t] = fit[0] +
                            fit[1]*t*1.0 +
                            fit[2]*t*t*1.0 +
                            fit[3]*t*t*t*1.0 +
                            fit[4]*t*t*t*t*1.0;
            }
            m_params["lambda_table"].setVal<double*>(lambda, nrPixels);

            m_params["sizex"].setVal<int>(nrPixels);
            m_params["sizex"].setMeta(new ito::IntMeta(1, nrPixels, 1), true);

            m_params["roi"].getVal<int*>()[2] = nrPixels;
            m_params["roi"].setMeta(new ito::RectMeta(ito::RangeMeta(0, nrPixels-1, 1, 1, nrPixels, 1), ito::RangeMeta(0, 0)), true);
            
        }
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    setInitialized(true); //init method has been finished (independent on retval)    
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvantesAvaSpec::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if (m_pUsb)
    {
        retValue += apiAddInClose(m_pUsb);
        m_pUsb = NULL;
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
    \details This method copies the complete tparam of the corresponding parameter to val

    \param [in,out] val  is a input of type::tparam containing name, value and further informations
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk in case that everything is ok, else retError
    \sa ito::tParam, ItomSharedSemaphore
*/
ito::RetVal AvantesAvaSpec::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
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
ito::RetVal AvantesAvaSpec::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
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
        if((key == "average") && (val->getVal<int>() <= 1))
        {
            m_params["bpp"].setVal<int>(16);
        }
        if((key == "average") && (val->getVal<int>() > 1))
        {
            m_params["bpp"].setVal<int>(32);
        }
    }
    
    if(!retValue.containsError())
    {
        if(key == "integration_time" || key == "roi" || key == "average")
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
ito::RetVal AvantesAvaSpec::startDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if(grabberStartedCount() == 0)
    {
        SendMeasConfigType config;
        const int *roi = m_params["roi"].getVal<int*>();
        double int_time_ms = m_params["integration_time"].getVal<double>() * 1e3;
        uint32 average = m_params["average"].getVal<int>();

        config.prefix[00]=0x20;
        config.prefix[01]=0x00;
        config.prefix[02]=0x2B;   // length of the command
        config.prefix[03]=0x00;
        config.prefix[04]=0x05;   // prepare_measurement
        config.prefix[05]=0x00;

        config.m_Meas.m_StartPixel = swap16(roi[0]);
        config.m_Meas.m_StopPixel = swap16(roi[0] + roi[2] - 1);
        config.m_Meas.m_IntegrationTime = swapsingle(int_time_ms);
        config.m_Meas.m_IntegrationDelay = 0;
        config.m_Meas.m_NrAverages = swap32(average);
        config.m_Meas.m_CorDynDark.m_Enable = 0;
        config.m_Meas.m_CorDynDark.m_ForgetPercentage = 0;
        config.m_Meas.m_Smoothing.m_SmoothPix = 0;
        config.m_Meas.m_Smoothing.m_SmoothModel = 0;
        config.m_Meas.m_SaturationDetection = 0;
        config.m_Meas.m_Trigger.m_Mode = 0; //Hardware
        config.m_Meas.m_Trigger.m_Source = 0; //Synchronized
        config.m_Meas.m_Trigger.m_SourceType = 0; //Edge
        config.m_Meas.m_Control.m_StrobeControl = 0;
        config.m_Meas.m_Control.m_LaserDelay = 0;
        config.m_Meas.m_Control.m_LaserWidth = 0;
        config.m_Meas.m_Control.m_LaserWaveLength = 0;
        config.m_Meas.m_Control.m_StoreToRam = 0;

        //send config
        int config_size = sizeof(config);
        int config_answer_size = 6;
        unsigned char config_answer[6];
        retValue += this->sendCommand((const char*)&config, config_size, config_answer, config_answer_size);
        if (!retValue.containsError())
        {
            if ((config_answer_size != 6) || (config_answer[4] != 0x85) || (config_answer[2] != 0x02))
            {
                retValue += ito::RetVal(ito::retError, 0, "error while setting configuration");
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
        waitCond->release();
    }
    return retValue;
}
         
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvantesAvaSpec::stopDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    decGrabberStarted();

    if(grabberStartedCount() == 0)
    {
        //stop measurement
        unsigned char cmd[] = {0x20, 0x00, 0x02 /*length of command*/, 0x00, 0x1F /*stop_measurement*/, 0x00};
        unsigned char answer[8];
        int size = 8;
        retValue += sendCommand((char*)cmd, sizeof(cmd), answer, size);
        //retValue += checkAnswerForError(answer, 0x8F, true);
    }
    else if(grabberStartedCount() < 0)
    {
        setGrabberStarted(0);
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return ito::retOk;
}
         
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvantesAvaSpec::acquire(const int trigger, ItomSharedSemaphore *waitCond)
{ItomSharedSemaphoreLocker locker(waitCond);

    ito::RetVal retValue(ito::retOk);

    if (grabberStartedCount() <= 0)
    {
        retValue = ito::RetVal(ito::retError, 0, tr("Tried to acquire without starting device").toLatin1().data());

        if (waitCond)
        {
            waitCond->returnValue = retValue;
            waitCond->release();  
        }
    }
    else
    {
        //start measurement
        unsigned char cmd[] = {0x20, 0x00, 0x04 /*length of command*/, 0x00, 0x06 /*start_measurement*/, 0x00, HIBYTE(1), LOBYTE(1)};
        unsigned char answer[6];
        int size = 6;
        retValue += sendCommand((char*)cmd, sizeof(cmd), answer, size);
        retValue += checkAnswerForError(answer, 0x86, false);

        if (waitCond)
        {
            waitCond->returnValue = retValue;
            waitCond->release();  
        }

        if (!retValue.containsError())
        {
            m_isGrabbing = true;
            uint32 average = m_params["average"].getVal<int>();
            int bpp = m_params["bpp"].getVal<int>();
            m_acquisitionRetVal = ito::retOk;
            int xsize = m_data.getSize(1);

            int request_size;
            if ((average <= 1) && (bpp == 16)) 
            {
                request_size = sizeof(sony_single_meas) - sizeof(sony_single_meas.pixels) + xsize * sizeof(uint16);
                memset(sony_single_meas.pixels, 0, sizeof(sony_single_meas.pixels));
                m_acquisitionRetVal += readWithFixedLength((char*)&sony_single_meas, request_size);
                m_acquisitionRetVal += checkAnswerForError((unsigned char*)&sony_single_meas, 0xB0, false);
                ito::uint16 *vals = (ito::uint16*)m_data.rowPtr(0,0);
                
                if (!m_acquisitionRetVal.containsError())
                {
                    for (int teller=0; teller < xsize; ++teller) 
                        vals[teller] = swap16(sony_single_meas.pixels[teller]); 
                }
            }
            else if ((average > 1) && (bpp == 32)) 
            {
                request_size = sizeof(sony_multi_meas) - sizeof(sony_multi_meas.pixels) + xsize * sizeof(uint32);
                memset(sony_multi_meas.pixels, 0, sizeof(sony_multi_meas.pixels));
                m_acquisitionRetVal += readWithFixedLength((char*)&sony_multi_meas, request_size);
                m_acquisitionRetVal += checkAnswerForError((unsigned char*)&sony_multi_meas, 0xB1, false);
                ito::float32 *vals = (ito::float32*)m_data.rowPtr(0,0);

                if (!m_acquisitionRetVal.containsError())
                {
                    for (int teller=0; teller < xsize; ++teller) 
                    {
                        vals[teller] = swap32(sony_multi_meas.pixels[teller]); 
                        vals[teller] = vals[teller]/average;
                    }
                }
            }

            unsigned char cmd2[] = {0x21, 0x00, 0x02 /*length of command*/, 0x00, 0xC0 /*acknowledge*/, 0x00};
            m_acquisitionRetVal += m_pUsb->setVal((const char*)cmd2, sizeof(cmd2), NULL);

            //QSharedPointer<char> f(new char[500]);
            //QSharedPointer<int> i(new int);
            //*i = 500;
            //m_acquisitionRetVal += m_pUsb->getVal(f, i, NULL);
            //int j = 0;
        }
    }

    
    return retValue + m_acquisitionRetVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvantesAvaSpec::retrieveData(ito::DataObject *externalDataObject)
{
    ito::RetVal retVal = m_acquisitionRetVal;
    int ret = 0;

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
    }

    if (!m_isGrabbing)
    {
        retVal += ito::RetVal(ito::retError, 0, "no image has been acquired");
    }


    if (!retVal.containsError())
    {      
        if (externalDataObject)
        {
            switch (m_params["bpp"].getVal<int>())
            {
                case 16:
                    retVal += externalDataObject->copyFromData2D<ito::uint16>((ito::uint16*)m_data.rowPtr(0,0), m_params["sizex"].getVal<int>(), m_params["sizey"].getVal<int>());
                    break;
                case 32:
                    retVal += externalDataObject->copyFromData2D<ito::float32>((ito::float32*)m_data.rowPtr(0,0), m_params["sizex"].getVal<int>(), m_params["sizey"].getVal<int>());
                    break;
                default:
                    retVal += ito::RetVal(ito::retError, 0, tr("Wrong picture type").toLatin1().data());
                    break;
            }
        }

        m_isGrabbing = false;
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvantesAvaSpec::checkData(ito::DataObject *externalDataObject)
{
    int futureHeight = m_params["sizey"].getVal<int>();
    int futureWidth = m_params["sizex"].getVal<int>();
    int futureType;

    int bpp = m_params["bpp"].getVal<int>();
    
    if (bpp <= 16)
    {
        futureType = ito::tUInt16;
    }
    else if (bpp <= 32)
    {
        futureType = ito::tFloat32;
    }
    else 
    {
        futureType = ito::tFloat64; // not necessary
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

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvantesAvaSpec::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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
            (*dObj) = this->m_data;
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
ito::RetVal AvantesAvaSpec::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);

    if(!dObj)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Empty object handle retrieved from caller").toLatin1().data());
    }
    else
    {
        retValue += checkData(dObj);  
    }

    if(!retValue.containsError())
    {
        retValue += retrieveData(dObj);  
    }

    if(!retValue.containsError())
    {
        sendDataToListeners(0); //don't wait for live data, since user should get the data as fast as possible.
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
void AvantesAvaSpec::updateParameters(QMap<QString, ito::ParamBase> params)
{ 
    foreach(const ito::ParamBase &param1, params)
    {    
        setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase(param1)), NULL);
    }
}




//-------------------------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvantesAvaSpec::sendCommand(const char* cmd, int cmd_size, unsigned char* buf, int &buf_size)
{
    ito::RetVal retVal = m_pUsb->setVal(cmd, cmd_size, NULL);
    
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

//-------------------------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvantesAvaSpec::readWithFixedLength(char* buf, int &buf_size)
{
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
}
// AvantesAvaSpec


//-------------------------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvantesAvaSpec::checkAnswerForError(const unsigned char* buf, const unsigned char &desiredCmd, bool warningNotError /*= false*/)
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
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "error %i: unknown", buf[6]);
            break;
        case 0x01:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "error %i: invalid parameter", buf[6]);
            break;
        case 0x02:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "error %i: invalid password", buf[6]);
            break;
        case 0x03:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "error %i: invalid command", buf[6]);
            break;
        case 0x04:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "error %i: invalid size", buf[6]);
            break;
        case 0x05:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "error %i: measurement pending", buf[6]);
            break;
        case 0x06:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "error %i: invalid pixel range", buf[6]);
            break;
        case 0x07:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "error %i: invalid integration time", buf[6]);
            break;
        case 0x08:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "error %i: operation not supported", buf[6]);
            break;
        case 0x09:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "error %i: invalid combination", buf[6]);
            break;
        case 0x0A:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "error %i: no buffer available", buf[6]);
            break;
        case 0x0B:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "error %i: no spectra available", buf[6]);
            break;
        case 0x0C:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "error %i: invalid state", buf[6]);
            break;
        case 0x0D:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "error %i: unexpected dma int", buf[6]);
            break;
        case 0x0E:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "error %i: invalid fpga file", buf[6]);
            break;
        default:
        //case 0x00:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "error %i", buf[6]);
        }
        return retVal;
    }
    else
    {
        return ito::RetVal(warningNotError ? ito::retWarning : ito::retError, 0, "unknown answer from device");
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
//! slot called if the dock widget of the plugin becomes (in)visible
/*!
    Overwrite this method if the plugin has a dock widget. If so, you can connect the parametersChanged signal of the plugin
    with the dock widget once its becomes visible such that no resources are used if the dock widget is not visible. Right after
    a re-connection emit parametersChanged(m_params) in order to send the current status of all plugin parameters to the dock widget.
*/
void AvantesAvaSpec::dockWidgetVisibilityChanged(bool visible)
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
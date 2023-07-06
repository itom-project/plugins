/* ********************************************************************
    Plugin "OceanOpticsSpec" for itom software
    URL: https://github.com/itom-project/plugins
    Copyright (C) 2020, Institut fuer Technische Optik, Universitaet Stuttgart

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

#include "OceanOpticsSpec.h"

#include <QFile>
#include <qstring.h>
#include <qstringlist.h>
#include <qelapsedtimer.h>
#include <QtCore/QtPlugin>
#include "pluginVersion.h"
#include "gitVersion.h"
# include "OceanOpticsSpecDefines.h"

#include "dockWidgetOceanOpticsSpec.h"
#include <QDateTime>

#ifdef WIN32
    #include <Windows.h>
#else
    #define LOBYTE(w) ((unsigned char)(((w)) & 0xff))
    #define HIBYTE(w) ((unsigned char)((((w)) >> 8) & 0xff))
#endif


// byteswaps
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

ito::uint32 swap32(ito::uint32 uint32in)
{
    union s
    {
        char sa[4];
        ito::uint32 res;
    } temp;
    temp.res = uint32in;
    s uint32out;
    for (int teller=0; teller<4; ++teller){
        uint32out.sa[teller]=temp.sa[3-teller];
    }
    return uint32out.res;
}

ito::uint16 swap16(ito::uint16 uint16in)
{
    union s
    {
        char sa[2];
        ito::uint16 res;
    } temp;
    temp.res = uint16in;
    s uint16out;
    for (int teller=0; teller<2; ++teller){
        uint16out.sa[teller]=temp.sa[1-teller];
    }
    return uint16out.res;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OceanOpticsSpecInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(OceanOpticsSpec)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OceanOpticsSpecInterface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(OceanOpticsSpec)
    return ito::retOk;
}


//----------------------------------------------------------------------------------------------------------------------------------
OceanOpticsSpecInterface::OceanOpticsSpecInterface() : AddInInterfaceBase()
{
    m_type = ito::typeDataIO | ito::typeGrabber;
    setObjectName("OceanOpticsSpec");

    m_description = QObject::tr("Ocean Optics Spectrometers");
    m_detaildescription = QObject::tr(
"This DLL integrates the OceanOptics spectrometers which use the Ocean Byte Protocol (OBP) into itom. \
This includes series: STS, QE Pro, Ocean FX, and Ocean HDX  (and probably future releases)\n\
It uses the libUSB interface to communicate with the devices.\n\
!This was ONLY fully implemented for the STS series! \
Others most likely won't work properly yet.\n\
Tested with: \
\
* OceanOptics STS-UV (25 um slit)");
    m_author = "J. Drozella, ITO, University Stuttgart";
    m_license = QObject::tr("LGPL");
    m_version           = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer        = MINVERSION;
    m_maxItomVer        = MAXVERSION;
    m_aboutThis         = tr(GITVERSION);

    m_autoLoadPolicy = ito::autoLoadNever;
    m_autoSavePolicy = ito::autoSaveNever;

    ito::Param paramVal = ito::Param("VendorID", ito::ParamBase::Int, 0, std::numeric_limits<int>::max(), 0x2457, tr("VendorID of spectrometer, 0x2457 for OceanOptics STS").toLatin1().data());
    paramVal.getMetaT<ito::IntMeta>()->setRepresentation(ito::ParamMeta::HexNumber);
    m_initParamsMand.append(paramVal);
    paramVal = ito::Param("ProductID", ito::ParamBase::Int, 0, std::numeric_limits<int>::max(), 0x4000, tr("ProductID of spectrometer, 0x4000 for STS").toLatin1().data());
    paramVal.getMetaT<ito::IntMeta>()->setRepresentation(ito::ParamMeta::HexNumber);
    m_initParamsMand.append(paramVal);
}

//----------------------------------------------------------------------------------------------------------------------------------
OceanOpticsSpecInterface::~OceanOpticsSpecInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal OceanOpticsSpec::showConfDialog(void)
{
    ito::RetVal retValue(ito::retOk);
    return retValue;
}
//----------------------------------------------------------------------------------------------------------------------------------
OceanOpticsSpec::OceanOpticsSpec() :
    AddInGrabber(),
    m_pUsb(NULL),
    m_isGrabbing(false),
    m_numberDeadPixels(0),
    m_numberOfCorrectionValues(0),
    m_startCorrectionIndex(0)
{
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "OceanOpticsSpec", "plugin name");
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("integration_time", ito::ParamBase::Double, 0.00001, (double)MAX_INTEGRATION_TIME / 1000.0, 0.002, tr("Integration time of CCD programmed in s.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    int roi[] = {0, 0, 1024, 1};
    paramVal = ito::Param("roi", ito::ParamBase::IntArray, 4, roi, tr("ROI (x,y,width,height)").toLatin1().data());
    ito::RectMeta *rm = new ito::RectMeta(ito::RangeMeta(0, roi[2]-1), ito::RangeMeta(0, roi[3]-1,1,1,1,1));
    paramVal.setMeta(rm, true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("sizex", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 1024, 1024, tr("current width of ROI").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("sizey", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 1, 1, tr("current height").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("bpp", ito::ParamBase::Int | ito::ParamBase::Readonly, 16, 16, 16, tr("Bit depth. The output object is float32 for all cases but uint16 only if no averaging is enabled.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("average", ito::ParamBase::Int, 1, 50000, 1, tr("Number of averages for every frame").toLatin1().data()); //0xffffffff --> timeout, also in libusb
    m_params.insert(paramVal.getName(), paramVal);

    /*paramVal = ito::Param("dark_correction", ito::ParamBase::Int, 0, 2, 0, tr("Some detectors have dark pixels, that can be used for a dark detection. If enabled, the output \n\
dataObject will always be float32. Static (1) subtracts the mean value of all dark pixels from all values. \n\
Dynamic (2) is only available for some devices (see if dyn. dark correction is enabled in the software \n\
AvaSpec) and subtracts different mean values for odd and even pixels. Off (0): default.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);*/

    paramVal = ito::Param("lambda_coeffs", ito::ParamBase::DoubleArray | ito::ParamBase::Readonly, NULL, tr("Coefficients for polynom that determines lambda_table (lambda_table[idx] = c[0] + c[1]*idx + c[2]*idx^2 + c[3]*idx^3)").toLatin1().data());
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
        DockWidgetOceanOpticsSpec *toolbox = new DockWidgetOceanOpticsSpec(this);
        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas; //areas where the toolbox can be positioned (see Qt documentation)
        //define some features, saying if the toolbox can be closed, can be undocked (floatable) and moved...
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | \
            QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        //register the toolbox
        createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, toolbox);
    }


}

//----------------------------------------------------------------------------------------------------------------------------------
OceanOpticsSpec::~OceanOpticsSpec()
{
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OceanOpticsSpec::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
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
        mands[2].setVal<int>( 1 ); //endpoint ID //2

        retValue += apiAddInOpenDataIO("LibUSB", libUSBNo, false, &mands, &opts, m_pUsb);
    }

    if (!retValue.containsError())
    {
        //retValue += m_pUsb->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("endpoint_write", ito::ParamBase::Int, 1)), NULL);
        //retValue += m_pUsb->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("endpoint_read", ito::ParamBase::Int, 129)), NULL); //6
    }

    if (!retValue.containsError())
    {
        OcHeader header;
        header.message_type = 0x00000001; //reset defaults

        //get Serial Number

#define SN_MAX_LENGTH  20

        int len;

        unsigned char answer[64];
        int a_len = 64;

        unsigned char cmd[4];

        *(uint32*)cmd = 0x00000100; //get serial number 0x000 001 00


        retValue += sendCommand(cmd, sizeof(cmd), (unsigned char*)answer, a_len);
        len = (uint8)answer[23];
        //std::string serialnumber = std::string(answer[24], len);
        //char *serialnumber = (char*)malloc(len * sizeof(char));
        //memcpy(serialnumber, &answer[24], len * sizeof(char));
        char sn[OC_SERIAL_LEN+1];
        if (len > OC_SERIAL_LEN) {
            retValue += ito::RetVal(ito::retError, -1, "Returned serial number significantly longer than anticipated, most like transfer/message error. Stopping to prevent damage.");
            return retValue;
        }
        for (int i = 0; i < len; ++i) {
            sn[i] = (char)answer[24 + i];
        }
        sn[len] = '\0';



        if (!retValue.containsError())
        {
            setIdentifier(QString("STS %1").arg(QString(QByteArray(sn))));
            m_params["serial_number"].setVal<char*>(sn);
        }

        m_params["detector_name"].setVal<char*>(tr("Default").toLatin1().data());
        m_numberDeadPixels = -1; //none expected, but -1 for correction

        *(uint32*)cmd = 0x00180100;// 0x00011800; // 0x001 801 00 - get wavelength coefficient count
        retValue += sendCommand(cmd, sizeof(cmd), (unsigned char*)answer, a_len);


        uint8 n_wvl_c = (uint8)answer[24];
        double *fit = m_params["lambda_coeffs"].getVal<double*>();
        *(uint32*)cmd = 0x00180101;// 0x01011800; // 0x001 801 01 - get wavelength coefficient [nr in data]
        char payload[] = { 0x00 };
        float coeff = 0.f;
        for (int t = 0; t < n_wvl_c; ++t)
        {
            *payload = (char)t;
            a_len = 64;
            retValue += sendCommand(cmd, sizeof(cmd), (unsigned char*)answer, a_len, (char*)payload, 1 );
            fit[t] = *(float*)&answer[24]; //swapsingle()maybe no swap, because already typecast readout?
        }

        int nrPixels = MAX_NR_PIXELS; // atm
        if (n_wvl_c > 4) {
            retValue += ito::RetVal::format(ito::retWarning, n_wvl_c, 0, "Warning: spectrometer returned %i wavelength coefficients, while the lambda table is only calculated up to 4.", n_wvl_c);
        }
        double *lambda = new double[nrPixels];
        for (int t = 0; t < nrPixels; ++t)
        {
            lambda[t] = fit[0] +
                fit[1] * t*1.0 +
                fit[2] * t*t*1.0 +
                fit[3] * t*t*t*1.0;// +
                //fit[4] * t*t*t*t*1.0;
        }
        m_params["lambda_table"].setVal<double*>(lambda, nrPixels);

        m_params["sizex"].setVal<int>(nrPixels);
        m_params["sizex"].setMeta(new ito::IntMeta(1, nrPixels, 1), true);

        m_params["roi"].getVal<int*>()[2] = nrPixels;
        m_params["roi"].setMeta(new ito::RectMeta(ito::RangeMeta(0, nrPixels - 1, 1, 1, nrPixels, 1), ito::RangeMeta(0, 0)), true);

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
ito::RetVal OceanOpticsSpec::close(ItomSharedSemaphore *waitCond)
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
ito::RetVal OceanOpticsSpec::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
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
ito::RetVal OceanOpticsSpec::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
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
        if(key == "integration_time" || key == "roi" || key == "average")// || key == "dark_correction")
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

            //calc minimum timeout for libusb
            double timeout = 3.0 + m_params["average"].getVal<int>() * m_params["integration_time"].getVal<double>();
            retValue += m_pUsb->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("timeout", ito::ParamBase::Double, timeout)), NULL);
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
ito::RetVal OceanOpticsSpec::startDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if(grabberStartedCount() == 0)
    {
        OcHeader header;

        const int *roi = m_params["roi"].getVal<int*>();
        uint32 int_time_ms = (uint32)rint(m_params["integration_time"].getVal<double>() * 1e3);
        double readouttime = m_params["integration_time"].getVal<double>();
        uint16 average = (uint16)m_params["average"].getVal<int>();
        unsigned char answer[64];
        int a_len = 64;

        // integration time
        // cmd: 0x001 100 10
        unsigned char cmd[4];
        *(uint32*)cmd = 0x00110010;
        //unsigned char cmd[4] = { 0x00,0x11,0x00,0x10 };// { 0x10, 0x00, 0x11, 0x00 };
        // immediate: 32Bit, LSB, 10-1E7 us, 0x01 00 00 00 //LSB FIRST
        unsigned char payload[4];
        *(uint32*)payload = (uint32)rint(int_time_ms * 1000); //no swap // time in us
        retValue += sendCommand(cmd, sizeof(cmd), (unsigned char*)answer, a_len,(char*)payload, 4);

        if (retValue.containsError())
        {
            std::cout << "couldn't set integration time" << std::endl;
            return retValue;
        }

        /* set in aquire()
        // average
        // cmd: 0x001 200 10
        a_len = 64;
        *(uint32*)cmd = 0x00120010;// (uint32)0x10001200;
        // immediate: 16Bit, LSB first, 1-5000
        *(uint32*)payload = 0;
        *(uint16*)payload = (uint16)average; //no swap
        retValue += sendCommand(cmd, sizeof(cmd), (unsigned char*)answer, a_len, (char*)payload, 2);

        if (retValue.containsError())
        {
            std::cout << "couldn't set averaging" << std::endl;
            return retValue;
        }

        // roi
        // cmd: 0x001 020 10 [set partial spectrum mode], no response
        // immediate:0x02 00 SS SS II II CC CC [band mode: Start, Interval, Count, 16b LSB]
        //if (roi[0] != 0 || roi[2] != MAX_NR_PIXELS) { // roi always needs to be set for aquire() to work

            a_len = 64;
            *(uint32*)cmd = 0x00102010;// (uint32)0x10201000;
            unsigned char roipayload[] = { 0x02, 0x00, LOBYTE((uint16)roi[0]), HIBYTE((uint16)roi[0]), 0x01, 0x00, LOBYTE((uint16)(roi[2]-1)), HIBYTE((uint16)roi[2]-1) }; // losing 1 px, but device returns h/w error otherwise(?!)
            //unsigned char roipayload[8];
            //*(long long*)roipayload = 0x0200000001000004;//  //0x02 00 00 00 01 00 00 04;
            retValue += sendCommand(cmd, sizeof(cmd), (unsigned char*)answer, a_len, (char*)roipayload,sizeof(roipayload));
            */
        //}
        if (retValue.containsError())
        {
            std::cout << "couldn't set roi" << std::endl;
            return retValue;
        }

        m_measConfig.m_StartPixel = roi[0];
        m_measConfig.m_StopPixel = roi[0] + roi[2] - 1;
        m_measConfig.m_IntegrationTime = int_time_ms;
        m_measConfig.m_NrAverages = average;

        // maybe set timeout here, again?
        double timeout = 3.0 + m_params["average"].getVal<int>() * m_params["integration_time"].getVal<double>();
        retValue += m_pUsb->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("timeout", ito::ParamBase::Double, timeout)), NULL);


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
ito::RetVal OceanOpticsSpec::stopDevice(ItomSharedSemaphore *waitCond)
{

    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    decGrabberStarted();

    if(grabberStartedCount() == 0)
    {
        //stop measurement: reset to default values (just to be safe, prevent autotriggers and so on)
        //unsigned char cmd[4];
        //*(uint32*)cmd = 0x00000001;//reset to device defaults - reset takes ~1sec & removes it briefly from available USB devices - might lead to problems
        //char answer[64];
        //int size = 64;
        //retValue += sendCommand(cmd, sizeof(cmd), (unsigned char*)answer, size);
        //retValue += checkAnswerForError((unsigned char*)answer, (const unsigned char)&cmd, true);
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
ito::RetVal OceanOpticsSpec::acquire(const int trigger, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);

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

        const int *roi = m_params["roi"].getVal<int*>();
        uint32 int_time_ms = (uint32)rint(m_params["integration_time"].getVal<double>() * 1e3);
        double inttime = m_params["integration_time"].getVal<double>();

        unsigned char cmd[4];
        unsigned char payload[4];
        unsigned char answer[sizeof(singleMeasdata.pixels) + 64];
        int a_len = 64;

        // get average count
        uint16 average = (uint16)m_params["average"].getVal<int>();

        // set integration time
        *(uint32*)cmd = 0x00110010;
        // immediate: 32Bit, LSB, 10-1E7 us, 0x01 00 00 00 //LSB FIRST
        //unsigned char payload[4];
        *(uint32*)payload = 0;
        *(uint32*)payload = (uint32)rint(int_time_ms * 1000); // time in us
        retValue += sendCommand(cmd, sizeof(cmd), (unsigned char*)answer, a_len, (char*)payload, 4);

        int xsize = m_data.getSize(1);

        int roilen = roi[2];
        ito::uint16 *vals16 = nullptr;
        ito::float32 *vals32 = nullptr;
        if (average == 1) {
            vals16 = (ito::uint16*)m_data.rowPtr(0, 0);
            memset(vals16, 0, sizeof(ito::uint16)*xsize);
        }
        else {
            vals32 = (ito::float32*)m_data.rowPtr(0, 0);
            memset(vals32, 0, sizeof(ito::float32)*xsize);
        }

        // set readout mode:
        if (roilen == MAX_NR_PIXELS) { // roi == all pixels

            *(uint32*)cmd = 0x00101000; // get and send corrected spectrum immediately
        }
        else { // adjust roi and readout mode

            a_len = 64;
            *(uint32*)cmd = 0x00102010;// (uint32)0x10201000;
            unsigned char roipayload[] = { 0x02, 0x00, LOBYTE((uint16)roi[0]), HIBYTE((uint16)roi[0]), 0x01, 0x00, LOBYTE((uint16)(roi[2])), HIBYTE((uint16)roi[2]) };
            retValue += sendCommand(cmd, sizeof(cmd), (unsigned char*)answer, a_len, (char*)roipayload, sizeof(roipayload));

            // set average to 1, averaging is done in loop, but needs to be set for "partial corrected spectrum" mode to work, in case of ROI
            *(uint32*)cmd = 0x00120010; // { 0x10, 0x00, 0x12, 0x00 }; //0x001 200 10 : 16b (1-5000) LSB, MSB
            *(uint32*)payload = 0;
            *(uint16*)payload = (uint16)1;
            retValue += sendCommand(cmd, sizeof(cmd), (unsigned char*)answer, a_len, (char*)payload, 2);

            *(uint32*)cmd = 0x00102080;// get and send partial corrected spectrum - corrected means including averages (set to 1, done outside) and ROI, baseline corrected
        }

        memset(singleMeasdata.pixels, 0, sizeof(singleMeasdata.pixels));

        if (!retValue.containsError())
        {
            for (int a = 0; a < average; ++a) {
                m_acquisitionRetVal = ito::retOk;
                int size = 64;
                int request_size = sizeof(singleMeasdata.pixels) + 64;
                //start measurement
                m_acquisitionRetVal += sendCommand(cmd, sizeof(cmd), (unsigned char*)&singleMeasdata.header, request_size);

                if (waitCond)
                {
                    waitCond->returnValue = retValue + m_acquisitionRetVal;
                    waitCond->release();
                }

                m_isGrabbing = true;



                uint32 timestamp = (uint32)((long)QDateTime::currentMSecsSinceEpoch() / 1000);

                singleMeasdata.timestamp = timestamp;
                if (!m_acquisitionRetVal.containsError())
                {
                    int payloadlength = *(uint32*)&singleMeasdata.header[40] - 20;
                    int byteoffset = 44;
                    if (payloadlength < 1) {
                        payloadlength = *(uint8*)&singleMeasdata.header[23];
                        memcpy(&singleMeasdata.pixels[0], (uint16*)&singleMeasdata.header[24], payloadlength);
                        }
                    payloadlength = (int)(payloadlength / sizeof(uint16));


                    //int expected_size = sizeof(OcSingleMeasdata) - sizeof(uint32) - sizeof(uint16)*(MAX_NR_DARKPIXELS + m_params["roi"].getVal<int*>()[2] /*width*/) + (xsize + std::max(0, (int)MAX_NR_DARKPIXELS)) * sizeof(uint16);
                    int expected_size = sizeof(OcSingleMeasdata) - sizeof(uint32)/*timestamp*/ - sizeof(uint16)*(MAX_NR_DARKPIXELS + roilen + (MAX_NR_PIXELS - roilen)) + (xsize + std::max(0, (int)MAX_NR_DARKPIXELS)) * sizeof(uint16);
                    if (request_size != expected_size) // both request and expected include 64B for header&footer
                    {
                        if (m_numberDeadPixels == -1)
                        {
                            m_numberDeadPixels = (expected_size - request_size) / sizeof(uint16); //order is okay: request_size changes depending on returned data length.
                        }
                        else
                        {
                            qDebug() << "received buffer has " << request_size << " bytes. " << expected_size << " bytes expected.";
                        }
                    }


                    if (average == 1) { //slower to do this within the loop
                        for (int teller = 0; teller < payloadlength/*(xsize - m_numberDeadPixels)*/; ++teller) /*STS return length varies, doesnt return for dead ones(?)(worst case: returns 0 for last pixel (checksum)) */
                        {
                            //vals[teller] = *(uint16*)&(singleMeasdata.header[byteoffset + 2*teller]);//swap16(singleMeasdata.pixels[teller + m_numberDeadPixels]);
                            vals16[teller] = singleMeasdata.pixels[teller];
                        }
                    }
                    else {
                        for (int teller = 0; teller < payloadlength/*(xsize - m_numberDeadPixels)*/; ++teller) /*STS return length varies, doesnt return for dead ones(?)(worst case: returns 0 for last pixel (checksum)) */
                        {
                            vals32[teller] += (ito::float32)singleMeasdata.pixels[teller] / (ito::float32)average;
                        }
                    }


                    m_data.setTag("timestamp", timestamp); //timestamp in sec since UTC0
                }


            }
        }
    }

    return retValue + m_acquisitionRetVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OceanOpticsSpec::retrieveData(ito::DataObject *externalDataObject)
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
            switch (m_data.getType())
            {
                case ito::tUInt16:
                    retVal += externalDataObject->copyFromData2D<ito::uint16>((ito::uint16*)m_data.rowPtr(0,0), m_params["sizex"].getVal<int>(), m_params["sizey"].getVal<int>());
                    break;
                case ito::tFloat32:
                    retVal += externalDataObject->copyFromData2D<ito::float32>((ito::float32*)m_data.rowPtr(0,0), m_params["sizex"].getVal<int>(), m_params["sizey"].getVal<int>());
                    break;
                default:
                    retVal += ito::RetVal(ito::retError, 0, tr("Wrong picture type").toLatin1().data());
                    break;
            }

            bool valid;
            externalDataObject->setTag("timestamp", m_data.getTag("timestamp", valid));
            if (m_data.existTag("dark"))
            {
                externalDataObject->setTag("dark", m_data.getTag("dark", valid));
            }
        }

        m_isGrabbing = false;
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OceanOpticsSpec::checkData(ito::DataObject *externalDataObject)
{
    int futureHeight = m_params["sizey"].getVal<int>();
    int futureWidth = m_params["sizex"].getVal<int>();
    int futureType;

    int bpp = m_params["bpp"].getVal<int>();
    //int darkCorrection = m_params["dark_correction"].getVal<int>();
    int average = m_params["average"].getVal<int>();

    if (bpp <= 16 && m_numberOfCorrectionValues == 0 && average == 1 )
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
            m_data *= 0;
        }
    }
    else
    {
        int dims = externalDataObject->getDims();
        if (externalDataObject->getDims() == 0)
        {
            *externalDataObject = ito::DataObject(futureHeight,futureWidth,futureType);
            *externalDataObject *= 0;
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
ito::RetVal OceanOpticsSpec::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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
ito::RetVal OceanOpticsSpec::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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
void OceanOpticsSpec::updateParameters(QMap<QString, ito::ParamBase> params)
{
    foreach(const ito::ParamBase &param1, params)
    {
        setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase(param1)), NULL);
    }
}


// Buid header [header data] + [message type] (swapped) + [immediate-data-ptr] + command/query
// returns header including footer.
ito::RetVal OceanOpticsSpec::buildheader(char **headout, OcHeader *headin, unsigned char* cmd, int cmd_len, char *data /*=nullptr*/, int dlen /*=-1*/, bool command  /*=true command or query*/)
{
    //int sztest = sizeof(cmd);

    if (cmd_len != 4)
    {
        std::cout << "Error: length of command not 4 Bytes" << std::endl;
        return ito::retError;
    }

    //uint32 mtype = (uint32)cmd[0];
    //uint32 *comm = (uint32*)cmd;
    headin->message_type = *(uint32*)cmd;// swap32(*(uint32*)cmd);// *comm); //no swap if uint32 input format is used (default now, fitting to documentation)
    int a = sizeof(OcHeader);

    if (command == true) {
        headin->flags = 0x0004; //reqACK
    }

    *headout = (char*)headin; //default state, replaced if new memory is allocated

    if (dlen > 0) { // command has a payload (in general)

        if (dlen <= 16 ) // build immediate data
        {
            headin->immediate_data_length = (uint8)dlen;
            for (int i = 0; i < dlen; ++i)
            {
                headin->immediate_data[i] = data[i];
            }
        }

        else // add/modify post-header-payload
        {
            headin->immediate_data_length = 0x00; //using nothing or payload, depending on command
            headin->bytes_remaining = (uint32)(20 + dlen);
            /*if (command == TRUE) {
                headin->flags = swap16(0x0002); //request ACK
            }
            else {
                headin->flags = 0x0000;
            }*/
            char *payload = (char*)malloc(dlen);
            memcpy(payload, data, dlen);
            char  *headernew = (char*)malloc(64 + dlen); //64 Bytes is overall header length for oceanSTS
            headernew = (char*)memcpy(headernew, (void*)headin, 40); // copy header pre-footer

            for (int i = 0; i < dlen; ++i) //dlen approx max 1024 with only one operation: probably no parallelization helpful?
            {
                headernew[40 + i] = data[i]; //copy after bytes_remaining
            }

            //memcpy((void*)headernew[64 + dlen], (void*)&headin[60], 4); //cpy bytes remaining
            headernew[60+dlen] = 0xC5C4C3C2;
            *headout = headernew;

        }
    }
    else {
        headin->immediate_data_length = 0x00;
        memset((uint8*)headin->immediate_data, 0, 16);
    }


    return ito::retOk;


}

//-------------------------------------------------------------------------------------------------------------------------------------------------
// sends command and reads response header until the bytes_remaining field, which indicates if there is any payload included
ito::RetVal OceanOpticsSpec::sendCommand(unsigned char* cmd, int cmd_size, unsigned char* buf, int &buf_size, char *immediate_data/*nullptr*/, int dlen /*-1*/)
{
    ito::RetVal retVal;
    char *headout = nullptr;
    if (cmd_size > 0)
    {
        if (immediate_data != nullptr && dlen == -1) {
            retVal += ito::RetVal(ito::retError, -1, "Payload given but no length, can't safely proceed");
            return retVal;
        }
        retVal += buildheader(&headout,&m_header,cmd,cmd_size,immediate_data,dlen,true);
        int sendbytes = (uint32)*(uint32*)((unsigned char*)headout +40)+44; //swap32()
        //int ahsdf = (uint32)(headout[40] + 44);
        //int asendbytes = (uint32)swap32(*(uint32*)headout[40]) + 40; //bytes remaining + header til bytes_remaining
        if (sendbytes > 64) {
            std::cout << "Notice: unusual amount of data to be sent: " << sendbytes-64 << " bytes more than usual \r\n." << std::endl;
        }
        retVal = m_pUsb->setVal((char*)headout, sendbytes, NULL);

    }

    //
    //if (!retVal.containsError() && buf != NULL)
    //{
    /*
        QSharedPointer<int> chuck_size(new int);
        int read_bytes = 0;

        QSharedPointer<char> chunk_buffer((char*)buf, idleCharDeleter);
        *chuck_size = buf_size - read_bytes;
        retVal += m_pUsb->getVal(chunk_buffer, chuck_size, NULL);
        read_bytes += *chuck_size;
        */
    //

    if (!retVal.containsError() && buf != NULL)
    {

        // first get header until bytes remaining:
        QSharedPointer<int> chuck_size(new int);
        *chuck_size = buf_size;// values <64 seem to be problematic (return packet is wrong then and doesn't contain data?)
        int byte_offset = 0;
        int read_bytes = 0; //bytes already read

        QSharedPointer<char> chunk_buffer((char*)buf, idleCharDeleter);

        retVal += m_pUsb->getVal(chunk_buffer, chuck_size, NULL);
        read_bytes += *chuck_size;

        if (!retVal.containsError())
        {
            uint8 *asdf = nullptr;


            uint8 immediate_bytes = (uint8)(buf[23]);

            //uint8 blah = (uint8)*((uint8*)((unsigned char*)buf + 23));


            uint32 bytes_remaining = *((uint32*)(&buf[40])); //

            int returnlen = bytes_remaining > 20 ? 60 : 64; // return 60B of data if no payload, else full header/size

            uint16 flags = (uint16)swap16(*(uint16*)(&buf[4]));
            retVal += checkAnswerForError(buf, *cmd);
            if (retVal.containsWarningOrError()){// || flags >= 8) {
                retVal += ito::RetVal(ito::retError, 0, "Problem in sendCommand(): either Device Error code (in retVal), or NACK");
                return retVal;
            }

            if (read_bytes < buf_size)
            {

                QElapsedTimer timer;
                timer.start();

                while (read_bytes < returnlen && timer.elapsed() < 2000 && !retVal.containsError())
                {
                    QSharedPointer<char> chunk_buffer((char*)(&(buf[read_bytes])), idleCharDeleter);
                    *chuck_size = buf_size - read_bytes; // anpassen?
                    retVal += m_pUsb->getVal(chunk_buffer, chuck_size, NULL);
                    read_bytes += *chuck_size;
                }

                if (returnlen > read_bytes)
                {
                    retVal += ito::RetVal(ito::retError, 0, "timeout while reading required data from usb port");
                }

                if (read_bytes <= 64 && *(uint32*)(&buf[60]) != 0xC5C4C3C2) {
                    retVal += ito::RetVal(ito::retError, 0, "Read-out header length doesn't seem to fit with footer/type");
                }
                buf_size = read_bytes;
            }
        }
    }

    return retVal;
}





//-------------------------------------------------------------------------------------------------------------------------------------------------
void OceanOpticsSpec::dummyRead()
{
    ito::RetVal retValue;

    //dummy read to clear input buffer
    char buf[1024];
    QSharedPointer<int> size(new int);
    *size = 1024;
    QSharedPointer<char> buffer(buf, idleCharDeleter);
    m_pUsb->getVal(buffer, size, NULL);
}


//-------------------------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OceanOpticsSpec::checkAnswerForError(unsigned char* buf, const unsigned char &desiredCmd, bool warningNotError /*= false*/, const char *prefix /*= ""*/)
{
    ito::RetVal retVal;

    uint16 returnflag = *(uint16*)&buf[4];
    uint32 incmd = *(uint32*)&desiredCmd;
    if (*(uint32*)&buf[8] == *(uint32*)&desiredCmd && *(uint16*)&buf[6] < 0x0008) // flag==0x0003 -> answer to request & ACK, anything < 8 is no error
    {
        return ito::retOk;
    }
    else if (returnflag > 7) // flag >= byte 3 set
    {
        switch (*(uint32*)&buf[6]) // read flags
        {
        case 0x0000:
            retVal += ito::retOk;
            break;
        case 0x0001:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "%serror %i: invalid protocol", prefix, buf[6]);
            break;
        case 0x0002:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "%serror %i: unknown message type", prefix, buf[6]);
            break;
        case 0x0003:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "%serror %i: bad checksum", prefix, buf[6]);
            break;
        case 0x0004:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "%serror %i: invalid size", prefix, buf[6]);
            break;
        case 0x0005:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "%serror %i: payload length does not match message type", prefix, buf[6]);
            break;
        case 0x0006:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "%serror %i: payload data invalid", prefix, buf[6]);
            break;
        case 0x0007:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "%serror %i: device not ready for given message type", prefix, buf[6]);
            break;
        case 0x0008:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "%serror %i: unknown checksum type", prefix, buf[6]);
            break;
        case 0x0009:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "%serror %i: unexpected device reset", prefix, buf[6]);
            break;
        case 0x000A:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "%serror %i: too many buses try to access", prefix, buf[6]);
            break;
        case 0x000B:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "%serror %i: out of memory (cant allocate)", prefix, buf[6]);
            break;
        case 0x000C:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "%serror %i: valid command, but desired information not existing", prefix, buf[6]);
            break;
        case 0x000D:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "%serror %i: internal device error. May be unrecoverable.", prefix, buf[6]);
            break;
        case 0x0064:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "%serror %i: could not decrypt properly", prefix, buf[6]);
            break;
        case 0x0065:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "%serror %i: data packet was wrong size (not 64 bytes)", prefix, buf[6]);
            break;
        case 0x0066:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "%serror %i: hardware revision not compatible with firmware", prefix, buf[6]);
            break;
        case 0x0067:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "%serror %i: existing flash map not compatible with firmware", prefix, buf[6]);
            break;
        case 0x000FF:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "%serror %i: Operation / Response deferred. Op will take some time to complete. Do not ACK or NACK yet", prefix, buf[6]);
            break;
        default:
            //case 0x00:
            retVal += ito::RetVal::format(warningNotError ? ito::retWarning : ito::retError, buf[6], "%serror %i", prefix, buf[6]);
        }

    // read flags for details
    if (((*(uint16*)&buf[6]) & 8) == 8) {
            retVal += ito::RetVal::format(ito::retError, buf[6], "Returned NACK to command");
            return retVal;
        }

    if (((*(uint16*)&buf[6]) & 16) == 16) {
        retVal += ito::RetVal::format(ito::retError, buf[6], "Returned Hardware error, but command seems okay");
        return retVal;
    }

    if (((*(uint16*)&buf[6]) & 32) == 32) {
        retVal += ito::RetVal::format(ito::retError, buf[6], "Returned unknown/unsupported protocol version");
        return retVal;
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
void OceanOpticsSpec::dockWidgetVisibilityChanged(bool visible)
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

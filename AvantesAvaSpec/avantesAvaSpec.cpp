/* ********************************************************************
    Plugin "AvantesAvaSpec" for itom software
    URL: https://github.com/itom-project/plugins
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

#include "avantesAvaSpec.h"

#include <QFile>
#include <qstring.h>
#include <qstringlist.h>
#include <qelapsedtimer.h>
#include <QtCore/QtPlugin>
#include "pluginVersion.h"
#include "gitVersion.h"

#include "dockWidgetAvantesAvaSpec.h"

#ifdef WIN32
    #include <Windows.h>
#else
    #define LOBYTE(w) ((unsigned char)(((w)) & 0xff))
    #define HIBYTE(w) ((unsigned char)((((w)) >> 8) & 0xff))
#endif

static unsigned long AvantesAvaSpec000Base = 0x10000L; // or 64k for Master Mode
static int AvantesAvaSpecADLow=31, AvantesAvaSpecADHigh=133;

float AvantesAvaSpec::swapsingleIfNeeded(float floatin)
{
    if (!m_swapNeeded)
    {
        return floatin;
    }
        union s
        {
            char sa[4];
            float res;
        } temp;
        temp.res = floatin;
        s floatout;
        for (int teller = 0; teller < 4; ++teller) {
            floatout.sa[teller] = temp.sa[3 - teller];
        }
        return floatout.res;


}

uint32 AvantesAvaSpec::swap32IfNeeded(uint32 uint32in)
{
    if (!m_swapNeeded)
    {
        return uint32in;
    }
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

uint16 AvantesAvaSpec::swap16IfNeeded(uint16 uint16in)
{
    if (!m_swapNeeded)
    {
        return uint16in;
    }
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
    m_detaildescription = QObject::tr(
"This DLL integrates the Avantis AvantesAvaSpec spectrometer series into itom. \
It uses a low-level libusb connection to communicate with the devices and has been \
tested with the following spectrometers: \
\
* AvaSpec 3468 USB-Spectrometer \
* AvaSpec 2048 USB-Spectrometer \
* AvaSpec-ULS2048CL-EVO USB3-Spectrometer.");
    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
    m_aboutThis = QObject::tr(GITVERSION);

    m_autoLoadPolicy = ito::autoLoadNever;
    m_autoSavePolicy = ito::autoSaveNever;

    ito::Param paramVal = ito::Param("VendorID", ito::ParamBase::Int, 0, std::numeric_limits<int>::max(), 0x1992, tr("VendorID of spectrometer, 0x1992 for AvaSpec-3648, 0x471 for AvaSpec-ULS3648...").toLatin1().data());
    paramVal.getMetaT<ito::IntMeta>()->setRepresentation(ito::ParamMeta::HexNumber);
    m_initParamsMand.append(paramVal);
    paramVal = ito::Param("ProductID", ito::ParamBase::Int, 0, std::numeric_limits<int>::max(), 0x0667, tr("ProductID of spectrometer, 0x0667 for spectrometer").toLatin1().data());
    paramVal.getMetaT<ito::IntMeta>()->setRepresentation(ito::ParamMeta::HexNumber);
    m_initParamsMand.append(paramVal);
    paramVal = ito::Param("isUSB3", ito::ParamBase::Int, 0, 1, 0, tr("Indicates if the device is a USB3 device").toLatin1().data());
    m_initParamsOpt.append(paramVal);
}

//----------------------------------------------------------------------------------------------------------------------------------
AvantesAvaSpecInterface::~AvantesAvaSpecInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------


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
    m_isGrabbing(false),
    m_numberDeadPixels(0),
    m_numberOfCorrectionValues(0),
    m_startCorrectionIndex(0),
    m_swapNeeded(true),
    m_answerLength(6),
    m_imageBufferLengthModifier(-2),
    m_readAveragedImageInOneChunk(false),
    m_isUsb3(false)
{
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "AvantesAvaSpec", "plugin name");
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("integration_time", ito::ParamBase::Double, 0.00001, (double)MAX_INTEGRATION_TIME / 1000.0, 0.002, tr("Integration time of CCD programmed in s, some devices do not accept the full range of allowed values (see AvaSpec for real minimum value of your device).").toLatin1().data());
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

    paramVal = ito::Param("bpp", ito::ParamBase::Int | ito::ParamBase::Readonly, 16, 16, 16, tr("Bit depth. The output object is float32 for all cases but uint16 only if no averaging is enabled and the dark_correction is disabled or no dark correction pixels are available for this sensor.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("average", ito::ParamBase::Int, 1, 65000, 1, tr("Number of averages for every frame").toLatin1().data()); //0xffffffff --> timeout, also in libusb
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("dark_correction", ito::ParamBase::Int, 0, 2, 0, tr("Some detectors have dark pixels, that can be used for a dark detection. If enabled, the output \n\
dataObject will always be float32. Static (1) subtracts the mean value of all dark pixels from all values. \n\
Dynamic (2) is only available for some devices (see if dyn. dark correction is enabled in the software \n\
AvaSpec) and subtracts different mean values for odd and even pixels. Off (0): default.").toLatin1().data());
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
        DockWidgetAvantesAvaSpec *toolbox = new DockWidgetAvantesAvaSpec(this);
        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas; //areas where the toolbox can be positioned (see Qt documentation)
        //define some features, saying if the toolbox can be closed, can be undocked (floatable) and moved...
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | \
            QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        //register the toolbox
        createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, toolbox);
    }


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
        retValue += checkAnswerForError(avs_id.prefix, 0x93, false, "Get serial number from device: ");

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
        retValue += checkAnswerForError(answer, 0x92, true, "Receive reason for last reset: ");
    }

    if (!retValue.containsError())
    {
        //get device config
        unsigned char cmd[] = {0x20, 0x00, 0x02 /*length of command*/, 0x00, 0x01 /*get device config*/, 0x00};
        int size = sizeof(m_deviceConfig);
        retValue += sendCommand((char*)cmd, sizeof(cmd), (unsigned char*)(&m_deviceConfig), size);


        if (size != sizeof(m_deviceConfig) || m_deviceConfig.prefix[2] != 0xFE || m_deviceConfig.prefix[4] != 0x81)
        {
            retValue += checkAnswerForError((unsigned char*)(&m_deviceConfig), 0x81, false, "Get configuration of device: ");
        }
        else
        {
            m_numberDeadPixels = 0;
            m_isUsb3 = bool(paramsOpt->at(0).getVal<int>());
            switch (m_deviceConfig.m_Detector.m_SensorType)
            {
            case SENS_HAMS8378_256:
                m_params["detector_name"].setVal<const char*>("HAMS8378_256");
                break;

            case SENS_HAMS8378_1024:
                m_params["detector_name"].setVal<const char*>("HAMS8378_1024");
                break;
            case SENS_ILX554:
                m_params["detector_name"].setVal<const char*>("ILX554");
                m_numberDeadPixels = 18; //uint8 const ILX_FIRST_USED_DARK_PIXEL = 2; uint8 const ILX_USED_DARK_PIXELS = 14; uint8 const ILX_TOTAL_DARK_PIXELS = 18;
                m_numberOfCorrectionValues = 14;
                m_startCorrectionIndex = 2;
                break;
            case SENS_HAMS9201:
                m_params["detector_name"].setVal<const char*>("HAMS9201");
                break;
            case SENS_TCD1304:
                m_params["detector_name"].setVal<const char*>("TCD1304");
                m_numberDeadPixels = 13; //uint8 const TCD_FIRST_USED_DARK_PIXEL = 0; uint8 const TCD_USED_DARK_PIXELS = 12; uint8 const TCD_TOTAL_DARK_PIXELS = 13;
                m_numberOfCorrectionValues = 12;
                m_startCorrectionIndex = 0;
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
                m_numberDeadPixels = 8; //uint8 const HAMS9840_FIRST_USED_DARK_PIXEL = 0; uint8 const HAMS9840_USED_DARK_PIXELS = 8; uint8 const HAMS9840_TOTAL_DARK_PIXELS = 8;
                m_numberOfCorrectionValues = 8;
                m_startCorrectionIndex = 0;
                break;
            case SENS_ILX511:
                m_params["detector_name"].setVal<const char*>("ILX511");
                m_numberDeadPixels = 18; //uint8 const ILX_FIRST_USED_DARK_PIXEL = 2; uint8 const ILX_USED_DARK_PIXELS = 14; uint8 const ILX_TOTAL_DARK_PIXELS = 18;
                m_numberOfCorrectionValues = 14;
                m_startCorrectionIndex = 2;
                break;
            case SENS_HAMS10420_2048X64:
                m_params["detector_name"].setVal<const char*>("HAMS10420_2048X64");
                m_numberDeadPixels = 4; //uint8 const     HAMS10420_FIRST_USED_DARK_PIXEL = 0; uint8 const     HAMS10420_USED_DARK_PIXELS = 4; uint8 const     HAMS10420_TOTAL_DARK_PIXELS = 4;
                m_numberOfCorrectionValues = 4;
                m_startCorrectionIndex = 0;
                break;
            case SENS_HAMS11071_2048X64:
                m_params["detector_name"].setVal<const char*>("HAMS11071_2048X64");
                m_numberDeadPixels = 4; //uint8 const     HAMS11071_FIRST_USED_DARK_PIXEL = 0; uint8 const     HAMS11071_USED_DARK_PIXELS = 4; uint8 const     HAMS11071_TOTAL_DARK_PIXELS = 4;
                m_numberOfCorrectionValues = 4;
                m_startCorrectionIndex = 0;
                break;
            case SENS_HAMS7031_1024X122:
                m_params["detector_name"].setVal<const char*>("HAMS7031_1024X122");
                m_numberDeadPixels = 4; //uint8 const     HAMS7031_FIRST_USED_DARK_PIXEL = 0; uint8 const     HAMS7031_USED_DARK_PIXELS = 4; uint8 const     HAMS7031_TOTAL_DARK_PIXELS = 4;
                m_numberOfCorrectionValues = 4;
                m_startCorrectionIndex = 0;
                break;
            case SENS_HAMS7031_1024X58:
                m_params["detector_name"].setVal<const char*>("HAMS7031_1024X58");
                m_numberDeadPixels = 4; //uint8 const     HAMS7031_FIRST_USED_DARK_PIXEL = 0; uint8 const     HAMS7031_USED_DARK_PIXELS = 4; uint8 const     HAMS7031_TOTAL_DARK_PIXELS = 4;
                m_numberOfCorrectionValues = 4;
                m_startCorrectionIndex = 0;
                break;
            case SENS_HAMS11071_2048X16:
                m_params["detector_name"].setVal<const char*>("HAMS11071_2048X16");
                m_numberDeadPixels = 4; //uint8 const     HAMS11071_FIRST_USED_DARK_PIXEL = 0; uint8 const     HAMS11071_USED_DARK_PIXELS = 4; uint8 const     HAMS11071_TOTAL_DARK_PIXELS = 4;
                m_numberOfCorrectionValues = 4;
                m_startCorrectionIndex = 0;
                break;
            case SENS_HAMS11155:
                m_params["detector_name"].setVal<const char*>("HAMS11155");
                m_numberDeadPixels = -1; //guess it!
                break;
            case SENS_SU256LSB:
                m_params["detector_name"].setVal<const char*>("SU256LSB");
                m_numberDeadPixels = -1; //guess it!
                break;
            case SENS_SU512LDB:
                m_params["detector_name"].setVal<const char*>("SU512LDB");
                m_numberDeadPixels = -1; //guess it!
                break;
            case SENS_HAMS11638:
                m_params["detector_name"].setVal<const char*>("SENS_HAMS11638");
                m_numberDeadPixels = -1; //guess it!
                break;
            case SENS_HAMS11639:
                m_params["detector_name"].setVal<const char*>("SENS_HAMS11639");
                m_numberDeadPixels = 0;
                m_isUsb3 = true; //this sensor is only available in USB3 devices
                break;
            case SENS_HAMS12443:
                m_params["detector_name"].setVal<const char*>("SENS_HAMS12443");
                m_numberDeadPixels = -1; //guess it!
                break;
            case SENS_HAMG9208_512:
                m_params["detector_name"].setVal<const char*>("SENS_HAMG9208_512");
                m_numberDeadPixels = -1; //guess it!
                m_isUsb3 = true; //this sensor is only available in USB3 devices
                break;
            case SENS_HAMG13913:
                m_params["detector_name"].setVal<const char*>("SENS_HAMG13913");
                m_numberDeadPixels = -1; //guess it!
                break;
            case SENS_HAMS13496:
                m_params["detector_name"].setVal<const char*>("SENS_HAMS13496");
                m_numberDeadPixels = -1; //guess it!
                m_isUsb3 = true; //this sensor is only available in USB3 devices
                break;
            default:
                m_params["detector_name"].setVal<const char*>("unknown");
                m_numberDeadPixels = -1; //guess it!
                break;
            }
            //There seem to be some differences between the usb3 devices. Here are some additional settings.
            //These settings must be tested if they are valid for all USB3 devices. What happens with Sensors available in USB2 and USB3 devices.
            if (m_isUsb3)
            {
                m_swapNeeded = false;
                m_answerLength = 8;
                m_imageBufferLengthModifier = 0;
                m_readAveragedImageInOneChunk = true;
            }

            if (m_numberOfCorrectionValues == 0)
            {
                m_params["dark_correction"].setVal<int>(0);
                m_params["dark_correction"].setFlags(ito::ParamBase::Readonly);
            }

            double *fit = m_params["lambda_coeffs"].getVal<double*>();
            for (int t = 0; t < NR_WAVELEN_POL_COEF; ++t)
            {
                fit[t] = swapsingleIfNeeded(m_deviceConfig.m_Detector.m_aFit[t]);
            }

            uint16 nrPixels = swap16IfNeeded(m_deviceConfig.m_Detector.m_NrPixels);
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
    if (!retValue.containsError())
    {
        QSharedPointer<int> readBytes(new int);
        *readBytes = 1000;
        QElapsedTimer timer;
        timer.start();
        ito::RetVal tmpRetValue(ito::retOk);
        while (*readBytes != 0 && timer.elapsed() < 2000 && !tmpRetValue.containsError())
        //while (*readBytes != 0 && !retValue.containsError())
        {
            QSharedPointer<char> bufferToDump(new char[1000], idleCharDeleter);
            *readBytes = 1000;
            tmpRetValue += m_pUsb->getVal(bufferToDump, readBytes, NULL);
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

    \param [in,out] val  is a input of type::tparam containing name, value and further information
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

    \param [in] val  is a input of type::tparam containing name, value and further information
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
ito::RetVal AvantesAvaSpec::startDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if(grabberStartedCount() == 0)
    {
        SendMeasConfigType config;
        const int *roi = m_params["roi"].getVal<int*>();
        float int_time_ms = m_params["integration_time"].getVal<double>() * 1e3;
        uint32 average = m_params["average"].getVal<int>();


        config.prefix[00]=0x20;
        config.prefix[01]=0x00; //REVERSE: 0x00;
        config.prefix[02]=0x2B;   // length of the command
        config.prefix[03]=0x00;
        config.prefix[04]=0x05;   // prepare_measurement
        config.prefix[05]=0x00; //REVERSE: 0x00


        config.m_Meas.m_StartPixel = swap16IfNeeded(roi[0]);
        config.m_Meas.m_StopPixel = swap16IfNeeded(roi[0] + roi[2] - 1);
        config.m_Meas.m_IntegrationTime = swapsingleIfNeeded(int_time_ms);
        config.m_Meas.m_IntegrationDelay = 0;
        config.m_Meas.m_NrAverages = swap32IfNeeded(average);
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
        unsigned char cmd[] = { 0x20, 0x00, 0x04 /*length of command*/, 0x00, 0x06 /*start_measurement*/, 0x00, HIBYTE(1), LOBYTE(1) };
        //start measurement
        if (!m_swapNeeded)
        {
            cmd[6] = LOBYTE(1);
            cmd[7] = HIBYTE(1);
        }
        //unsigned char* answer = new unsigned char[m_answerLength];
        unsigned char answer[8];
        int size = m_answerLength;
        retValue += sendCommand((char*)cmd, sizeof(cmd), answer, size);
        retValue += checkAnswerForError(answer, 0x86, false);
        //delete[] answer;
        //answer = NULL;
        if (waitCond)
        {
            waitCond->returnValue = retValue;
            waitCond->release();
        }

        if (!retValue.containsError())
        {
            m_isGrabbing = true;
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
                m_acquisitionRetVal += checkAnswerForError((unsigned char*)&singleMeasdata, 0xB0, false);
                request_size = singleMeasdata.prefix[3] * 256 + singleMeasdata.prefix[2]+ m_imageBufferLengthModifier;
                m_acquisitionRetVal += readWithFixedLength((char*)&(singleMeasdata.timestamp), request_size);

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
                            darkEven += swap16IfNeeded(singleMeasdata.pixels[teller]);
                            darkOdd += swap16IfNeeded(singleMeasdata.pixels[teller + 1]);
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
                            vals[teller] = swap16IfNeeded(singleMeasdata.pixels[teller + m_numberDeadPixels]);
                        }
                    }

                    m_data.setTag("timestamp", (double)swap32IfNeeded(singleMeasdata.timestamp) * 1e-5); //timestamp is in 10us units
                }
            }
            else //average > 1
            {
                //at first get the first 6 bytes (prefix) to check for the size of
                //the data. Then obtain the data.
                memset(multiMeasdata.pixels, 0, sizeof(multiMeasdata.pixels));
                request_size = sizeof(multiMeasdata.prefix);
                if (m_readAveragedImageInOneChunk)// for some reason the entire multiMeasdata must be read in a single chunk from device this may be related to usb3
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
                            darkEven += swap32IfNeeded(multiMeasdata.pixels[teller]);
                            darkOdd += swap32IfNeeded(multiMeasdata.pixels[teller + 1]);
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



                    m_data.setTag("timestamp", (double)swap32IfNeeded(multiMeasdata.timestamp) * 1e-5); //timestamp is in 10us units
                }
            }

            unsigned char cmd2[] = {0x21, 0x00, 0x02 /*length of command*/, 0x00, 0xC0 /*acknowledge*/, 0x00};
            m_acquisitionRetVal += m_pUsb->setVal((const char*)cmd2, sizeof(cmd2), NULL);
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
ito::RetVal AvantesAvaSpec::checkData(ito::DataObject *externalDataObject)
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
void AvantesAvaSpec::dummyRead()
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
ito::RetVal AvantesAvaSpec::checkAnswerForError(const unsigned char* buf, const unsigned char &desiredCmd, bool warningNotError /*= false*/, const char *prefix /*= ""*/)
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

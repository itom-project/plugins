/* ********************************************************************
    Plugin "Ximea" for itom software
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

#include "Ximea.h"
#include "XimeaFuncsImport.h"
#include "pluginVersion.h"
#include "dockWidgetXimea.h"

#include "common/sharedFunctionsQt.h"

#if linux
    #include <dlfcn.h>
    #include <unistd.h>
#endif
#include <QFile>
#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>

Q_DECLARE_METATYPE(ito::DataObject)

//int XimeaInterface::m_instCounter = 5;  // initialization starts with five due to normal boards are 0..4

static char InitList[5] = {0, 0, 0, 0, 0};  /*!<A map with successfull initialized boards (max = 5) */
static char Initnum = 0;    /*!< Number of successfull initialized cameras */

#if linux
    void *ximeaLib = NULL;
#else
    HMODULE ximeaLib = NULL;
#endif

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal XimeaInterface::getAddInInst(ito::AddInBase **addInInst)
{
    Ximea* newInst = new Ximea();
    newInst->setBasePlugin(this);
    *addInInst = qobject_cast<ito::AddInBase*>(newInst);

    m_InstList.append(*addInInst);

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal XimeaInterface::closeThisInst(ito::AddInBase **addInInst)
{
    if (*addInInst)
    {
        delete ((Ximea *)*addInInst);
        int idx = m_InstList.indexOf(*addInInst);
        m_InstList.removeAt(idx);
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
XimeaInterface::XimeaInterface()
{
    m_type = ito::typeDataIO | ito::typeGrabber;
    setObjectName("Ximea");

    m_description = QObject::tr("Ximea xiQ-Camera");
    m_detaildescription = QObject::tr("Developed for Windows only. Tested with Ximea xIQ-cameras for black and white.");
    m_author = "C. Kohler, twip optical solutions GmbH, Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("LGPL / do not copy Ximea-DLLs");
    m_aboutThis = QObject::tr("N.A.");     
    
    ito::Param paramVal = ito::Param("camera Number", ito::ParamBase::Int | ito::ParamBase::In, 0, 3, 0, "The index of the addressed camera starting with 0");
    m_initParamsOpt.append(paramVal);

    paramVal = ito::Param("restoreLast", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, "Toogle if the driver should try to connect to the last initilized camera");
    m_initParamsOpt.append(paramVal);

    paramVal = ito::Param("bandwidthLimit", ito::ParamBase::Int | ito::ParamBase::In, 0, 100000, 0, "bandwidth limit in Mb/sec. If 0 the maximum bandwidth of the USB3 controller is used [default]. The allowed value range depends on the device and will be checked at startup.");
    m_initParamsOpt.append(paramVal);

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
XimeaInterface::~XimeaInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
Q_EXPORT_PLUGIN2(Ximeainterface, XimeaInterface)

//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal Ximea::showConfDialog(void)
{
    ito::RetVal retValue(ito::retOk);

    dialogXimea *confDialog = new dialogXimea(this);

    connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), confDialog, SLOT(valuesChanged(QMap<QString, ito::Param>)));

    QMetaObject::invokeMethod(this, "sendParameterRequest");

    if (confDialog->exec())
    {
        disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), confDialog, SLOT(valuesChanged(QMap<QString, ito::Param>)));
        confDialog->sendVals();
    }
    else
    {
        disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), confDialog, SLOT(valuesChanged(QMap<QString, ito::Param>)));
    }
    delete confDialog;

    return retValue;
}
//----------------------------------------------------------------------------------------------------------------------------------
Ximea::Ximea() : AddInGrabber(), m_numDevices(0), m_device(-1), m_saveParamsOnClose(false), m_handle(NULL), m_isgrabbing(0)
{
    //qRegisterMetaType<ito::DataObject>("ito::DataObject");
    //qRegisterMetaType<QMap<QString, ito::Param> >("QMap<QString, ito::Param>");

   ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly | ito::ParamBase::NoAutosave, "Ximea", NULL);
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("integration_time", ito::ParamBase::Double, 0.000010, 4.0, 0.005, tr("Integrationtime of CCD programmed in s").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("gain", ito::ParamBase::Double, 0.0, 1.0, 0.0, tr("gain in dB").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("offset", ito::ParamBase::Double, 0.0, 1.0, 0.0, tr("Currently not used").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);

   paramVal = ito::Param("x0", ito::ParamBase::Int, 0, 1279, 0, tr("Startvalue for ROI").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("y0", ito::ParamBase::Int, 0, 1023, 0, tr("Startvalue for ROI").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("x1", ito::ParamBase::Int, 0, 1279, 1279, tr("Stopvalue for ROI").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("y1", ito::ParamBase::Int, 0, 1023, 1023, tr("Stopvalue for ROI").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("sizex", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 2048, 1280, tr("ROI-Size in x").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("sizey", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 2048, 1024, tr("ROI-Size in y").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);

   paramVal = ito::Param("bpp", ito::ParamBase::Int, 8, 12, 12, tr("Grabdepth in bpp").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);

   paramVal = ito::Param("binning", ito::ParamBase::Int, 101, 202, 101, tr("Activate 2x2 binning").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("trigger_mode", ito::ParamBase::Int, 0, 4, 0, tr("Set Triggermode, 0: free run, 1: ext. rising edge, 2: ext. falling edge, 3: software").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("trigger_mode2", ito::ParamBase::Int, 0, 3, 1, tr("Set Triggermode2, 0: single image, 1: frame duration, 2: burst, 3: burst with frame duration").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("timing_mode", ito::ParamBase::Int, 0, 1, 1, tr("Acquisition timing: 0: free run, 1: by frame rate").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("framerate", ito::ParamBase::Double, 0.0, 1000.0, 60.0, tr("Set Triggermode, currently not implemented").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("camNumber", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::NoAutosave, 0, 4, 0, tr("Number / ximea-internal IDX of this camera").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);

    //now create dock widget for this plugin
    DockWidgetXimea *XI = new DockWidgetXimea(m_params, getID());

    connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), XI, SLOT(valuesChanged(QMap<QString, ito::Param>)));
    connect(XI, SIGNAL(OffsetPropertiesChanged(double)), this, SLOT(OffsetPropertiesChanged(double)));
    connect(XI, SIGNAL(GainPropertiesChanged(double)), this, SLOT(GainPropertiesChanged(double)));
    connect(XI, SIGNAL(IntegrationPropertiesChanged(double)), this, SLOT(IntegrationPropertiesChanged(double)));

    Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
    QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
    createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, XI);
}

//----------------------------------------------------------------------------------------------------------------------------------
Ximea::~Ximea()
{
   m_params.clear();
   delete ximeaLib;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Ximea::getErrStr(const int error)
{
    switch (error)
    {
        case 0:
            return ito::RetVal(ito::retOk, error, "");
        break;
        //errors from m3Api.h
        case 1:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Invalid handle").toAscii().data());
        break;
        case 2:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Register read error").toAscii().data());
        break;
        case 3:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Register write error").toAscii().data());
        break;
        case 4:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Freeing resources error").toAscii().data());
        break;
        case 5:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Freeing channel error").toAscii().data());
        break;
        case 6:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Freeing bandwith error").toAscii().data());
        break;
        case 7:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Read block error").toAscii().data());
        break;
        case 8:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Write block error").toAscii().data());
        break;
        case 9:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): No image").toAscii().data());
        break;
        case 10:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Timeout").toAscii().data());
        break;
        case 11:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Invalid arguments supplied").toAscii().data());
        break;
        case 12:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Not supported").toAscii().data());
        break;
        case 13:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Attach buffers error").toAscii().data());
        break;
        case 14:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Overlapped result").toAscii().data());
        break;
        case 15:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Memory allocation error").toAscii().data());
        break;
        case 16:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): DLL context is NULL").toAscii().data());
        break;
        case 17:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): DLL context is non zero").toAscii().data());
        break;
        case 18:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): DLL context exists").toAscii().data());
        break;
        case 19:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Too many devices connected").toAscii().data());
        break;
        case 20:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Camera context error").toAscii().data());
        break;
        case 21:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Unknown hardware").toAscii().data());
        break;
        case 22:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Invalid TM file").toAscii().data());
        break;
        case 23:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Invalid TM tag").toAscii().data());
        break;
        case 24:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Incomplete TM").toAscii().data());
        break;
        case 25:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Bus reset error").toAscii().data());
        break;
        case 26:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Not implemented").toAscii().data());
        break;
        case 27:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Shading too bright").toAscii().data());
        break;
        case 28:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Shading too dark").toAscii().data());
        break;
        case 29:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Gain is too low").toAscii().data());
        break;
        case 30:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Invalid bad pixel list").toAscii().data());
        break;
        case 31:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Bad pixel list realloc error").toAscii().data());
        break;
        case 32:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Invalid pixel list").toAscii().data());
        break;
        case 33:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Invalid Flash File System").toAscii().data());
        break;
        case 34:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Invalid profile").toAscii().data());
        break;
        case 35:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Invalid bad pixel list").toAscii().data());
        break;
        case 36:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Invalid buffer").toAscii().data());
        break;
        case 38:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Invalid data").toAscii().data());
        break;
        case 39:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Timing generator is busy").toAscii().data());
        break;
        case 40:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Wrong operation open/write/read/close").toAscii().data());
        break;
        case 41:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Acquisition already started").toAscii().data());
        break;
        case 42:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Old version of device driver installed to the system").toAscii().data());
        break;
        case 43:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): To get error code please call GetLastError function").toAscii().data());
        break;
        case 44:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Data can't be processed").toAscii().data());
        break;
        case 45:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Error occured and acquisition has been stoped or didn't start").toAscii().data());
        break;
        case 46:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Acquisition has been stoped with error").toAscii().data());
        break;
        case 47:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Input ICC profile missed or corrupted").toAscii().data());
        break;
        case 48:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Output ICC profile missed or corrupted").toAscii().data());
        break;
        case 49:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Device not ready to operate").toAscii().data());
        break;
        case 50:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Shading too contrast").toAscii().data());
        break;
        case 51:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Module already initialized").toAscii().data());
        break;
        case 52:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Application doesn't enough privileges(one or more applications opened)").toAscii().data());
        break;
        case 53:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): installed driver incompatible with current software").toAscii().data());
        break;
        case 54:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): TM file was not loaded successfully from resources").toAscii().data());
        break;
        case 55:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Device has been reseted, abnormal initial state").toAscii().data());
        break;
        case 56:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): No Devices found").toAscii().data());
        break;
        case 57:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): Resource (device) or function  locked by mutex").toAscii().data());
        break;

        //errors from xiApi.h
        case 100:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): unknown parameter").toAscii().data());
        break;
        case 101:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): wrong parameter value").toAscii().data());
        break;
        case 103:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): wrong parameter type").toAscii().data());
        break;
        case 104:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): wrong parameter size").toAscii().data());
        break;
        case 105:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): input buffer too small").toAscii().data());
        break;
        case 106:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): parameter info not supported").toAscii().data());
        break;
        case 107:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): parameter info not supported").toAscii().data());
        break;
        case 108:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): data format not supported").toAscii().data());
        break;
        case 109:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): read only parameter").toAscii().data());
        break;
        case 110:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): no devices found").toAscii().data());
        break;
        case 111:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): this camera does not support currently available bandwidth").toAscii().data());
        break;
        default:
            return ito::RetVal(ito::retError, error, tr("Ximea (m3api): unknown error code").toAscii().data());
        break;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Ximea::LoadLib(void)
{
    ito::RetVal retValue = ito::retOk;

    if (!ximeaLib)
    {
#if linux
        ximeaLib = dlopen("./lib/libm3api.so", RTLD_LAZY);
        if (!ximeaLib)
        {
            char *error = dlerror();
            return ito::RetVal(ito::retError, 0, error);
        }
#else
#if _WIN64
#if UNICODE
        ximeaLib = LoadLibrary(L"./lib/m3apiX64.dll");
#else
        ximeaLib = LoadLibrary("./lib/m3apiX64.dll");
#endif
        //ximeaLib = LoadLibrary("./plugins/Ximea/m3apiX64.dll");
        if (!ximeaLib)
        {
            return ito::RetVal(ito::retError, 0, tr("LoadLibrary(\"m3apiX64.dll\")").toAscii().data());
        }
#else
#if UNICODE
        ximeaLib = LoadLibrary(L"m3api.dll");
#else
        ximeaLib = LoadLibrary("m3api.dll");
#endif
        if (!ximeaLib)
        {
            return ito::RetVal(ito::retError, 0, tr("LoadLibrary(\"./plugins/Ximea/m3api.dll\")").toAscii().data());
        }
#endif
#endif

#if linux
        if ((pxiGetNumberDevices = (XI_RETURN(*)(PDWORD)) dlsym(ximeaLib, "xiGetNumberDevices")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiGetNumberDevices").toAscii().data());

        if ((pxiOpenDevice = (XI_RETURN(*)(DWORD,PHANDLE)) dlsym(ximeaLib, "xiOpenDevice")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiOpenDevice").toAscii().data());

        if ((pxiCloseDevice = (XI_RETURN(*)(HANDLE)) dlsym(ximeaLib, "xiCloseDevice")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiCloseDevice").toAscii().data());

        if ((pxiStartAcquisition = (XI_RETURN(*)(HANDLE)) dlsym(ximeaLib, "xiStartAcquisition")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiStartAcquisition").toAscii().data());

        if ((pxiStopAcquisition = (XI_RETURN(*)(HANDLE)) dlsym(ximeaLib, "xiStopAcquisition")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiStopAcquisition").toAscii().data());

        if ((pxiGetImage = (XI_RETURN(*)(HANDLE,DWORD,LPXI_IMG)) dlsym(ximeaLib, "xiGetImage")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiGetImage").toAscii().data());

        if ((pxiSetParam = (XI_RETURN(*)(HANDLE,const char*,void*,DWORD,XI_PRM_TYPE)) dlsym(ximeaLib, "xiSetParam")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiSetParam").toAscii().data());

        if ((pxiGetParam = (XI_RETURN(*)(HANDLE,const char*,void*,DWORD*,XI_PRM_TYPE*)) dlsym(ximeaLib, "xiGetParam")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiGetParam").toAscii().data());
#else
        if ((pxiGetNumberDevices = (XI_RETURN(*)(PDWORD)) GetProcAddress(ximeaLib, "xiGetNumberDevices")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiGetNumberDevices").toAscii().data());

        if ((pxiOpenDevice = (XI_RETURN(*)(DWORD,PHANDLE)) GetProcAddress(ximeaLib, "xiOpenDevice")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiOpenDevice").toAscii().data());

        if ((pxiCloseDevice = (XI_RETURN(*)(HANDLE)) GetProcAddress(ximeaLib, "xiCloseDevice")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiCloseDevice").toAscii().data());

        if ((pxiStartAcquisition = (XI_RETURN(*)(HANDLE)) GetProcAddress(ximeaLib, "xiStartAcquisition")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiStartAcquisition").toAscii().data());

        if ((pxiStopAcquisition = (XI_RETURN(*)(HANDLE)) GetProcAddress(ximeaLib, "xiStopAcquisition")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiStopAcquisition").toAscii().data());

        if ((pxiGetImage = (XI_RETURN(*)(HANDLE,DWORD,LPXI_IMG)) GetProcAddress(ximeaLib, "xiGetImage")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiGetImage").toAscii().data());

        if ((pxiSetParam = (XI_RETURN(*)(HANDLE,const char*,void*,DWORD,XI_PRM_TYPE)) GetProcAddress(ximeaLib, "xiSetParam")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiSetParam").toAscii().data());

        if ((pxiGetParam = (XI_RETURN(*)(HANDLE,const char*,void*,DWORD*,XI_PRM_TYPE*)) GetProcAddress(ximeaLib, "xiGetParam")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiGetParam").toAscii().data());
#endif
    }

    if ((retValue != ito::retOk))
    {
        if(!Initnum)
        {
#if linux
            dlclose(ximeaLib);
#else
            FreeLibrary(ximeaLib);
#endif
            ximeaLib = NULL;
        }
        else
        {
            std::cerr << "DLLs not unloaded due to further running grabber instances\n" << std::endl;
        }
        return retValue;
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
ito::RetVal Ximea::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key = val->getName();

    if(key == "")
    {
        retValue += ito::RetVal(ito::retError, 0, tr("name of requested parameter is empty.").toAscii().data());
    }
    else
    {
        QMap<QString, ito::Param>::const_iterator paramIt = m_params.constFind(key);
        if (paramIt != m_params.constEnd())
        {
            *val = paramIt.value();
        }
        else
        {
            retValue += ito::RetVal(ito::retError, 0, tr("parameter not found in m_params.").toAscii().data());
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
ito::RetVal Ximea::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    int maxxsize = 0;
    int maxysize = 0;
    int x0old = m_params["x0"].getVal<int>();
    int y0old = m_params["y0"].getVal<int>();
    int x1old = m_params["x1"].getVal<int>();
    int y1old = m_params["y1"].getVal<int>();

    int running = 0;
    int bin = 1;
    float gain = 0;
    int bitppix = 10;
    int bitppix_old = m_params["bpp"].getVal<int>();
    int shift = 0;

/*
    int trigger_mode = XI_TRG_OFF;
    int trigger_mode2 = XI_TRG_SEL_EXPOSURE_ACTIVE;
    int timing_mode = XI_ACQ_TIMING_MODE_FRAME_RATE;
*/
    int trigger_mode = m_params["trigger_mode"].getVal<int>();
    int trigger_mode2 = m_params["trigger_mode2"].getVal<int>();
    int timing_mode = m_params["timing_mode"].getVal<int>();
    float frameRate = m_params["framerate"].getVal<double>();
    //int trigger_mode = XI_TRG_OUT;    //in new api trg_out does not exist anymore, so we just use free run
    int integration_time = 2000;
    XI_RETURN ret;

    QString key = val->getName();

    if(key == "")    // Check if the key is valied
    {
        retValue += ito::RetVal(ito::retError, 0, tr("name of given parameter is empty.").toAscii().data());
    }
    else    // key valid so go on
    {
        QMap<QString, ito::Param>::iterator paramIt = m_params.find(key);    // try to find the parameter in the parameter list

        if (paramIt != m_params.end()) // Okay the camera has this parameter so go on
        {

            if (grabberStartedCount())
            {
                running = grabberStartedCount();
                setGrabberStarted(1);
                retValue += this->stopDevice(0);
            }
            else
            {
                setGrabberStarted(1);
                this->stopDevice(0);
            }

            if(paramIt->getFlags() & ito::ParamBase::Readonly)
            {
                retValue += ito::RetVal(ito::retWarning, 0, tr("Parameter is read only, input ignored").toAscii().data());
                goto end;
            }
            else if(val->isNumeric() && paramIt->isNumeric())
            {
                double curval = val->getVal<double>();
                if( curval > paramIt->getMax())
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("New value is larger than parameter range, input ignored").toAscii().data());
                    goto end;
                }
                else if(curval < paramIt->getMin())
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("New value is smaller than parameter range, input ignored").toAscii().data());
                    goto end;
                }
                else
                {
                    paramIt.value().setVal<double>(curval);
                }
            }
            else if (paramIt->getType() == val->getType())
            {
                retValue += paramIt.value().copyValueFrom( &(*val) );
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 0, tr("Parameter type conflict").toAscii().data());
                goto end;
            }

            Sleep(5);

            integration_time = (int)(m_params["integration_time"].getVal<double>() * 1000000);
            trigger_mode = m_params["trigger_mode"].getVal<int>();

            bin = m_params["binning"].getVal<int>();
            gain = (int)(m_params["gain"].getVal<double>() * 10 + 0.5);
            bitppix = (int) m_params["bpp"].getVal<int>();

            if (strcmp(paramIt.value().getName(),"bpp") == 0)
            {
                if ((bitppix != 8) && (bitppix != 10))
                {
                    m_params["bpp"].setVal<int>(bitppix_old);
                    retValue += ito::RetVal(ito::retError, 0, tr("Tried to set invalid Bits per Pixe").toAscii().data());
                    return retValue;
                }
                if (bitppix == 8)
                {
                    int bpp = XI_RAW8;
                    if ((ret = pxiSetParam(m_handle, XI_PRM_IMAGE_DATA_FORMAT, &bpp, sizeof(int), xiTypeInteger)))
                        retValue += getErrStr(ret);
                }
                else
                {
                    int bpp = XI_RAW16;
                    //int bpp = XI_MONO16;
                    if ((ret = pxiSetParam(m_handle, XI_PRM_IMAGE_DATA_FORMAT, &bpp, sizeof(int), xiTypeInteger)))
                        retValue += getErrStr(ret);
                }
            }
            else if (strcmp(paramIt.value().getName(),"binning") == 0)
            {
                int curxsize;
                int curysize;
                int xbin = (int)(bin / 100);
                int ybin = bin - xbin * 100;
                if(xbin != ybin)
                {
                    retValue = ito::RetVal(ito::retWarning, 0, tr("Set binning faild. Must be 101 or 202.").toAscii().data());
                    goto end;
                }

                if ((ret = pxiSetParam(m_handle, XI_PRM_DOWNSAMPLING, &bin, sizeof(int), xiTypeInteger)))
                    retValue += getErrStr(ret);

                DWORD pSize = sizeof(int);
                XI_PRM_TYPE pType = xiTypeInteger;

                ret = pxiGetParam(m_handle, XI_PRM_WIDTH XI_PRM_INFO_MAX, &maxxsize, &pSize, &pType);
                ret = pxiGetParam(m_handle, XI_PRM_HEIGHT XI_PRM_INFO_MAX, &maxysize, &pSize, &pType);
                ret = pxiGetParam(m_handle, XI_PRM_WIDTH, &curxsize, &pSize, &pType);
                ret = pxiGetParam(m_handle, XI_PRM_HEIGHT, &curysize, &pSize, &pType);

                maxxsize = (int)(maxxsize / (xbin));
                maxysize = (int)(maxysize / (ybin));

                static_cast<ito::IntMeta*>( m_params["sizex"].getMeta() )->setMax(maxxsize);
                static_cast<ito::IntMeta*>( m_params["sizey"].getMeta() )->setMax(maxysize);
                //m_params["sizex"].setMax(maxxsize);
                //m_params["sizey"].setMax(maxysize);
                m_params["sizex"].setVal(curxsize);
                m_params["sizey"].setVal(curysize);

                m_params["x0"].setMeta( new ito::IntMeta(0,maxxsize-1), true );
                m_params["y0"].setMeta( new ito::IntMeta(0,maxysize-1), true );
                /*m_params["x0"].setMin(0);
                m_params["y0"].setMin(0);
                m_params["x0"].setMax(maxxsize-1);
                m_params["y0"].setMax(maxysize-1);*/
                m_params["x0"].setVal<int>(0);
                m_params["y0"].setVal<int>(0);

                m_params["x1"].setMeta( new ito::IntMeta(0,maxxsize-1), true );
                m_params["y1"].setMeta( new ito::IntMeta(0,maxysize-1), true );
               /* m_params["x1"].setMax(maxxsize-1);
                m_params["y1"].setMax(maxysize-1);
                m_params["x1"].setMin(0);
                m_params["y1"].setMin(0);*/
                m_params["x1"].setVal<int>(curxsize-1);
                m_params["y1"].setVal<int>(curysize-1);
            }
            else if (strcmp(paramIt.value().getName(),"integration_time") == 0)
            {
                if ((ret = pxiSetParam(m_handle, XI_PRM_EXPOSURE, &integration_time, sizeof(int), xiTypeInteger)))
                    retValue += getErrStr(ret);
            }
            else if (strcmp(paramIt.value().getName(),"trigger_mode") == 0)
            {
                if ((ret = pxiSetParam(m_handle, XI_PRM_TRG_SOURCE, &trigger_mode, sizeof(int), xiTypeInteger)))
                    retValue += getErrStr(ret);
            }
            else if (strcmp(paramIt.value().getName(),"trigger_mode2") == 0)
            {
                if ((ret = pxiSetParam(m_handle, XI_PRM_TRG_SELECTOR, &trigger_mode2, sizeof(int), xiTypeInteger)))
                    retValue += getErrStr(ret);
            }
            else if (strcmp(paramIt.value().getName(),"timing_mode") == 0)
            {
                if ((ret = pxiSetParam(m_handle, XI_PRM_ACQ_TIMING_MODE, &timing_mode, sizeof(int), xiTypeInteger)))
                    retValue += getErrStr(ret);
            }
            else if (strcmp(paramIt.value().getName(),"framerate") == 0)
            {
                if ((ret = pxiSetParam(m_handle, XI_PRM_FRAMERATE, &frameRate, sizeof(float), xiTypeFloat)))
                    retValue += getErrStr(ret);
            }
            else if (strcmp(paramIt.value().getName(),"gain") == 0)
            {
                if ((ret = pxiSetParam(m_handle, XI_PRM_GAIN, &gain, sizeof(float), xiTypeFloat)))
                    retValue += getErrStr(ret);
            }
            else if (strcmp(paramIt.value().getName(),"x1") == 0)
            {
                int size = m_params["x1"].getVal<int>() - m_params["x0"].getVal<int>() + 1;

                if(size % 2 != 0)
                {
                    retValue = ito::RetVal(ito::retWarning, 0, tr("Size must be multiply of 2").toAscii().data());
                    m_params["x1"].setVal<int>(x1old);
                    goto end;
                }

                if ((ret = pxiSetParam(m_handle, XI_PRM_WIDTH, &size, sizeof(int), xiTypeInteger)))
                {
                    retValue += getErrStr(ret);
                }
                else
                {
                    static_cast<ito::IntMeta*>( m_params["x0"].getMeta() )->setMax( m_params["x1"].getVal<int>() );
                    //m_params["x0"].setMax(m_params["x1"].getVal<int>());
                    m_params["sizex"].setVal<int>(size);
                }
            }
            else if (strcmp(paramIt.value().getName(),"y1") == 0)
            {
                int size = m_params["y1"].getVal<int>() - m_params["y0"].getVal<int>() + 1;
                if ((ret = pxiSetParam(m_handle, XI_PRM_HEIGHT, &size, sizeof(int), xiTypeInteger)))
                    retValue += getErrStr(ret);
                else
                {
                    static_cast<ito::IntMeta*>( m_params["y0"].getMeta() )->setMax( m_params["y1"].getVal<int>() );
                    //m_params["y0"].setMax(m_params["y1"].getVal<int>());
                    m_params["sizey"].setVal<int>(size);
                }
            }
            else if (strcmp(paramIt.value().getName(),"x0") == 0)
            {
                int offset = m_params["x0"].getVal<int>();
                int maxsize = m_params["sizex"].getMax();
                int size = m_params["x1"].getVal<int>() - offset + 1;

                if(size % 2 != 0)
                {
                    retValue = ito::RetVal(ito::retWarning, 0, tr("Size must be multiply of 2").toAscii().data());
                    m_params["x0"].setVal<int>(x0old);
                    goto end;
                }

                if (x0old < offset)
                {
                    m_params["sizex"].setVal<int>(size);
                    if ((ret = pxiSetParam(m_handle, XI_PRM_WIDTH, &size, sizeof(int), xiTypeInteger)))
                        retValue += getErrStr(ret);
                    if ((ret = pxiSetParam(m_handle, XI_PRM_OFFSET_X, &offset, sizeof(int), xiTypeInteger)))
                        retValue += getErrStr(ret);
                }
                else
                {
                    if ((ret = pxiSetParam(m_handle, XI_PRM_OFFSET_X, &offset, sizeof(int), xiTypeInteger)))
                        retValue += getErrStr(ret);
                    if ((ret = pxiSetParam(m_handle, XI_PRM_WIDTH, &size, sizeof(int), xiTypeInteger)))
                        retValue += getErrStr(ret);
                }

                m_params["sizex"].setVal<int>(size);
                //m_params["x1"].setMin(m_params["x0"].getVal<int>());
                static_cast<ito::IntMeta*>( m_params["x1"].getMeta() )->setMin( m_params["x0"].getVal<int>() );

            }
            else if (strcmp(paramIt.value().getName(),"y0") == 0)
            {
                int offset = m_params["y0"].getVal<int>();
                int size = m_params["y1"].getVal<int>() - offset + 1;
                
                if (y0old < offset)
                {
                    if ((ret = pxiSetParam(m_handle, XI_PRM_HEIGHT, &size, sizeof(int), xiTypeInteger)))
                        retValue += getErrStr(ret);
                    if ((ret = pxiSetParam(m_handle, XI_PRM_OFFSET_Y, &offset, sizeof(int), xiTypeInteger)))
                        retValue += getErrStr(ret);
                }
                else
                {
                    if ((ret = pxiSetParam(m_handle, XI_PRM_OFFSET_Y, &offset, sizeof(int), xiTypeInteger)))
                        retValue += getErrStr(ret);                    
                    if ((ret = pxiSetParam(m_handle, XI_PRM_HEIGHT, &size, sizeof(int), xiTypeInteger)))
                        retValue += getErrStr(ret);
                }
                m_params["sizey"].setVal<int>(size);
                //m_params["y1"].setMin(m_params["y0"].getVal<int>());
                static_cast<ito::IntMeta*>( m_params["y1"].getMeta() )->setMin( m_params["y0"].getVal<int>() );
            }
        }
        else
        {
            retValue = ito::RetVal(ito::retWarning, 0, tr("Parameter not found").toAscii().data());
        }
    }

end:

    retValue += checkData();

    if (running)
    {
        retValue += this->startDevice(0);
        setGrabberStarted(running);
    }

    if (!retValue.containsWarningOrError())
    {
        emit parametersChanged(m_params);
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

   return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Ximea::setXimeaParam(const char *paramName, int newValue)
{
    int min;
    int max;
    int inc;
    ito::RetVal retval;
    DWORD pSize = sizeof(int);
    XI_PRM_TYPE pType = xiTypeInteger;


    QByteArray name;

    //get parameter ranges
    name = QByteArray(paramName) + XI_PRM_INFO_MIN;
    retval +=  getErrStr(pxiGetParam(m_handle, name.data(), &min, &pSize, &pType));

    name = QByteArray(paramName) + XI_PRM_INFO_MAX;
    retval +=  getErrStr(pxiGetParam(m_handle, name.data(), &max, &pSize, &pType));

    name = QByteArray(paramName) + XI_PRM_INFO_INCREMENT;
    retval +=  getErrStr(pxiGetParam(m_handle, name.data(), &inc, &pSize, &pType));

    if (!retval.containsError())
    {
        //check incoming parameter
        if (newValue < min || newValue > max)
        {
            retval += ito::RetVal::format(ito::retError,0, "xiApi-Parameter '%s' is out of allowed range [%i,%i]", paramName, min, max);
        }
        else if ( (newValue - min) % inc != 0)
        {
            retval += ito::RetVal::format(ito::retError,0, "xiApi-Parameter '%s' must have an increment of %i (minimum value %i)", paramName, inc, min);
        }

        if (!retval.containsError())
        {
            retval += getErrStr(pxiSetParam(m_handle, paramName, &newValue, sizeof(int), xiTypeInteger));
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Ximea::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);

    ito::RetVal retValue(ito::retOk);
    int iCamNumber = (*paramsOpt)[0].getVal<int>();
    int bandwidthLimit = paramsOpt->at(2).getVal<int>(); //0 auto bandwidth calculation
    int maxxsize = 0;
    int maxysize = 0;
    int curxsize = 0;
    int curysize = 0;
    int x0 = 0;
    int y0 = 0;
    int bitppix = 10;
    XI_RETURN ret;

    int integration_time = 2;
    int trigger_mode = 0;
    int trigger_mode2 = 0;
    int timing_mode = 0;
    float framerate = 30;
    double gain = 0;
    QFile paramFile;

    // Load parameterlist from XML-file
    int loadPrev = (*paramsOpt)[1].getVal<int>();

    DWORD pSize = sizeof(int);
    XI_PRM_TYPE pType = xiTypeInteger;
    XI_PRM_TYPE strType = xiTypeString;

    retValue += LoadLib();

    m_params["camNumber"].setVal<int>(iCamNumber);

    Initnum++;  // so we have a new running instance of this grabber (or not)

    if( ++InitList[iCamNumber] > 1)    // It does not matter if the rest works or not. The close command will fix this anyway
    {
        retValue = ito::RetVal(ito::retError, 0, tr("Camera already initialized. Try with another camera number").toAscii().data());
    }
    else
    {
        ret = pxiOpenDevice(iCamNumber, &m_handle);
        if (!m_handle || ret != XI_OK)
        {
            m_handle = NULL;
            retValue += getErrStr(ret);
            retValue += ito::RetVal(ito::retError, 0, tr("Unable open camera").toAscii().data());
        }
        else
        {
            char strBuf[1024];
            int serialNumber;
            DWORD strBufSize = 1024 * sizeof(char);
            retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_DEVICE_NAME, &strBuf, &strBufSize, &strType));

            retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_DEVICE_SN, &serialNumber, &pSize, &pType));

            if (!retValue.containsError())
            {
                QString serialNumberHex = QString::number(serialNumber, 16);
                m_identifier = QString("%1 (SN:%2)").arg(strBuf).arg(serialNumberHex);
            }
        }

        
    
        if (!retValue.containsError())
        {
            //get available bandwidth in Mb/sec
            //int availableBandwidth;
            //retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_AVAILABLE_BANDWIDTH, &availableBandwidth, &pSize, &pType));
            //std::cout << "available bandwidth: " << availableBandwidth << std::endl;
            if (bandwidthLimit > 0) //manually set bandwidthLimit
            {
                retValue += setXimeaParam(XI_PRM_AUTO_BANDWIDTH_CALCULATION, XI_OFF);
                retValue += setXimeaParam(XI_PRM_LIMIT_BANDWIDTH, bandwidthLimit);
            }
        }

        // Load parameterlist from XML-file
        if(loadPrev && !retValue.containsError())
        {
            QMap<QString, ito::Param> paramListXML;
            if (!retValue.containsError())
            {
                retValue += ito::generateAutoSaveParamFile(this->getBasePlugin()->getFilename(), paramFile);
            }

            // Read parameter list from file to paramListXML
            if (!retValue.containsError())
            {
                retValue += ito::loadXML2QLIST(&paramListXML, QString::number(iCamNumber), paramFile);
            }

            // Merge parameter list from file to paramListXML with current mparams
            if (!retValue.containsError())
            {

            }
            paramListXML.clear();
        }

        if (!retValue.containsError())
        {
            // Camera-exposure is set in µsec, itom uses s
            integration_time = m_params["integration_time"].getVal<int>();
            trigger_mode = m_params["trigger_mode"].getVal<int>();
            trigger_mode2 = m_params["trigger_mode2"].getVal<int>();
            timing_mode = m_params["timing_mode"].getVal<int>();
            framerate = m_params["framerate"].getVal<double>();

            if ((ret = pxiSetParam(m_handle, XI_PRM_EXPOSURE, &integration_time, sizeof(int), xiTypeInteger)))
                retValue += getErrStr(ret);
            if ((ret = pxiSetParam(m_handle, XI_PRM_TRG_SOURCE, &trigger_mode, sizeof(int), xiTypeInteger)))
                retValue += getErrStr(ret);
            // Though in api the dll reports not supported ...
    //        if ((ret = pxiSetParam(m_handle, XI_PRM_TRG_SELECTOR, &trigger_mode2, sizeof(int), xiTypeInteger)))
    //            retValue += getErrStr(ret);
            if ((ret = pxiSetParam(m_handle, XI_PRM_ACQ_TIMING_MODE, &timing_mode, sizeof(int), xiTypeInteger)))
                retValue += getErrStr(ret);
            // Though in api the dll reports not supported ...
    //        if ((ret = pxiSetParam(m_handle, XI_PRM_FRAMERATE, &framerate, sizeof(float), xiTypeFloat)))
    //            retValue += getErrStr(ret);
            switch (m_params["bpp"].getVal<int>())
            {
                case 8:
                    bitppix = XI_RAW8;
                break;
                default:
                    bitppix = XI_RAW16;
                    //bitppix = XI_MONO16;
                break;
            }
            if ((ret = pxiSetParam(m_handle, XI_PRM_IMAGE_DATA_FORMAT, &bitppix, sizeof(int), xiTypeInteger)))
                retValue += getErrStr(ret);

            ret = pxiGetParam(m_handle, XI_PRM_IMAGE_DATA_FORMAT, &bitppix, &pSize, &pType);
            if ((bitppix == XI_RAW8) || (bitppix == XI_MONO8))
                m_params["bpp"].setVal<int>(8);
            else
                m_params["bpp"].setVal<int>(12);
        }

        if (!retValue.containsError())
        {
        

            ret = pxiGetParam(m_handle, XI_PRM_WIDTH XI_PRM_INFO_MAX, &maxxsize, &pSize, &pType);
            ret = pxiGetParam(m_handle, XI_PRM_HEIGHT XI_PRM_INFO_MAX, &maxysize, &pSize, &pType);

            curxsize = m_params["sizex"].getVal<int>();
            curysize = m_params["sizey"].getVal<int>();
            x0 = m_params["x0"].getVal<int>();
            y0 = m_params["y0"].getVal<int>();
            if ((ret = pxiSetParam(m_handle, XI_PRM_WIDTH, &curxsize, sizeof(int), xiTypeInteger)))
            {
                if ((ret = pxiSetParam(m_handle, XI_PRM_WIDTH, &maxxsize, sizeof(int), xiTypeInteger)))
                    retValue += getErrStr(ret);
            }
            if ((ret = pxiSetParam(m_handle, XI_PRM_HEIGHT, &curysize, sizeof(int), xiTypeInteger)))
            {
                if ((ret = pxiSetParam(m_handle, XI_PRM_HEIGHT, &maxysize, sizeof(int), xiTypeInteger)))
                    retValue += getErrStr(ret);
            }
            if ((ret = pxiSetParam(m_handle, XI_PRM_OFFSET_X, &x0, sizeof(int), xiTypeInteger)))
            {
                x0 = 0;
                if ((ret = pxiSetParam(m_handle, XI_PRM_OFFSET_X, &x0, sizeof(int), xiTypeInteger)))
                    retValue += getErrStr(ret);
            }
            if ((ret = pxiSetParam(m_handle, XI_PRM_OFFSET_Y, &y0, sizeof(int), xiTypeInteger)))
            {
                y0 = 0;
                if ((ret = pxiSetParam(m_handle, XI_PRM_OFFSET_Y, &y0, sizeof(int), xiTypeInteger)))
                    retValue += getErrStr(ret);
            }

            ret = pxiGetParam(m_handle, XI_PRM_WIDTH, &curxsize, &pSize, &pType);
            ret = pxiGetParam(m_handle, XI_PRM_HEIGHT, &curysize, &pSize, &pType);
            ret = pxiGetParam(m_handle, XI_PRM_OFFSET_X, &x0, &pSize, &pType);
            ret = pxiGetParam(m_handle, XI_PRM_OFFSET_Y, &y0, &pSize, &pType);
            //m_params["sizex"].setMax(maxxsize);
            //m_params["sizey"].setMax(maxysize);
            static_cast<ito::IntMeta*>(m_params["sizex"].getMeta())->setMax(maxxsize);
            static_cast<ito::IntMeta*>(m_params["sizey"].getMeta())->setMax(maxysize);
            m_params["sizex"].setVal(curxsize);
            m_params["sizey"].setVal(curysize);
            //m_params["x0"].setMax(maxxsize-1);
            //m_params["y0"].setMax(maxysize-1);
            static_cast<ito::IntMeta*>(m_params["x0"].getMeta())->setMax(maxxsize-1);
            static_cast<ito::IntMeta*>(m_params["y0"].getMeta())->setMax(maxysize-1);
            m_params["x0"].setVal<int>(x0);
            m_params["y0"].setVal<int>(y0);

            //m_params["x1"].setMax(maxxsize-1);
            //m_params["y1"].setMax(maxysize-1);
            static_cast<ito::IntMeta*>(m_params["x1"].getMeta())->setMax(maxxsize-1);
            static_cast<ito::IntMeta*>(m_params["y1"].getMeta())->setMax(maxysize-1);
            m_params["x1"].setVal<int>(curxsize - x0 + 1);
            m_params["y1"].setVal<int>(curysize - y0 + 1);
            //m_params["x1"].setMin(x0);
            //m_params["y1"].setMin(y0);
            static_cast<ito::IntMeta*>(m_params["x1"].getMeta())->setMin(x0);
            static_cast<ito::IntMeta*>(m_params["y1"].getMeta())->setMin(y0);
        }

        if (!retValue.containsError())
        {
            retValue += checkData();
        }
    }

    if (retValue.containsError())
    {
        if(Initnum <= 1) //this instance already incremented Initnum in any cases, it will be decremented in case of error in the close method
        {
            if (ximeaLib)
            {
#if linux
                dlclose(ximeaLib);
#else
                FreeLibrary(ximeaLib);
#endif
                ximeaLib = NULL;
            }
        }
        else
        {
            //std::cerr << "DLLs not unloaded due to further running grabber instances\n" << std::endl;
        }
    }
    else
    {
        m_saveParamsOnClose = true;
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
ito::RetVal Ximea::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    XI_RETURN ret;

    if (this->m_handle == NULL || 0xffffffffffffffff)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("Camera handle deleted before closing procedure").toAscii().data());
        goto endclose;
    }

    setGrabberStarted(1);
    retValue += this->stopDevice(0);
    Sleep(50);

    if ((ret = pxiCloseDevice(m_handle)))
    {
        retValue += getErrStr(ret);
    }

endclose:
    int nr = m_params["camNumber"].getVal<int>();
    InitList[nr] = 0;
    Initnum--; // so we closed a further instance of this grabber

    if(!Initnum)
    {
        if (ximeaLib)
        {
#if linux
            dlclose(ximeaLib);
#else
            FreeLibrary(ximeaLib);
#endif
            ximeaLib = NULL;
        }
    }
    else
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("DLLs not unloaded due to still living instances of PCO-Cams").toAscii().data());
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Ximea::startDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if(grabberStartedCount() < 1)
    {
        setGrabberStarted(0);
        XI_RETURN ret;
        ret = pxiStartAcquisition(m_handle);
        if (ret)
            retValue += getErrStr(ret);
    }

    if (!retValue.containsError())
    {
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
ito::RetVal Ximea::stopDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    decGrabberStarted();

    if(grabberStartedCount() < 1)
    {
        XI_RETURN ret;
        if ((ret = pxiStopAcquisition(m_handle)))
            retValue += getErrStr(ret);
        m_isgrabbing = false;
    }
    if(grabberStartedCount() < 0)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("Cameraflag was < 0").toAscii().data());
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
ito::RetVal Ximea::acquire(const int trigger, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    int triggermode = m_params["trigger_mode"].getVal<int>();

    if (grabberStartedCount() <= 0)
    {
        retValue = ito::RetVal(ito::retError, 0, tr("Tried to acquire without starting device").toAscii().data());
    }
    else
    {
        XI_RETURN ret;
        if(!m_isgrabbing)
        {
            if (triggermode == XI_TRG_SOFTWARE)
                if ((ret = pxiSetParam(m_handle, XI_PRM_TRG_SOFTWARE, 0, sizeof(int), xiTypeInteger))) //TODO: isn't it necessary to set the value to XI_TRG_SOFTWARE here?
                    retValue += getErrStr(ret);
        }
        else
            retValue = ito::RetVal(ito::retWarning, 0, tr("Tried to acquire multiple times without calling getVal. This acquire was ignored.").toAscii().data());
        m_isgrabbing = true;
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Ximea::retrieveData(ito::DataObject *externalDataObject)
{
    ito::RetVal retValue(ito::retOk);

    int iPicTimeOut = 2000; //timeout in ms

    long lcopysize = 0;
    int maxxsize = (int)m_params["sizex"].getMax();
    int maxysize = (int)m_params["sizey"].getMax();
    int curxsize = m_params["sizex"].getVal<int>();
    int curysize = m_params["sizey"].getVal<int>();
    int x0 = m_params["x0"].getVal<int>();
    int y0 = m_params["y0"].getVal<int>();

    bool hasListeners;
    bool copyExternal;
    XI_IMG img;
    XI_RETURN ret;

    if(m_autoGrabbingListeners.size() > 0)
        hasListeners = true;
    else
        hasListeners = false;

    if(externalDataObject != NULL)
        copyExternal = true;
    else
        copyExternal = false;

    if (this->m_isgrabbing == false)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("Tried to get picture without triggering exposure").toAscii().data());
    }
    else
    {
        //here we wait until the Event is set to signaled state
        //or the timeout runs out
        img.size = sizeof(XI_IMG);
        img.bp = NULL;
        img.bp_size = 0;
        if ((ret = pxiGetImage(m_handle, iPicTimeOut, &img)))
            retValue += getErrStr(ret);
        else
        {// Now we shoud have a picture in the camera buffer
            switch (m_params["bpp"].getVal<int>())
            {
                case 8:
                    //retValue += CopyBuf2Obj<uint8>(dObj, m_params, this->m_pAdr[this->m_nextbuf]);
                    if (curxsize == img.width)
                    {
                        if(copyExternal)
                            retValue += externalDataObject->copyFromData2D<ito::uint8>((ito::uint8*)img.bp, img.width, curysize);
                        if(!copyExternal || hasListeners)
                            retValue += m_data.copyFromData2D<ito::uint8>((ito::uint8*)img.bp, maxxsize, curysize);
                    }
                    else
                    {
                        if(copyExternal)
                            retValue += externalDataObject->copyFromData2D<ito::uint8>((ito::uint8*)img.bp, img.width, img.height, x0, y0, curxsize, curysize);
                        if(!copyExternal || hasListeners)
                            retValue += m_data.copyFromData2D<ito::uint8>((ito::uint8*)img.bp, img.width, img.height, x0, y0, curxsize, curysize);
                    }
                break;

                case 10:
                case 12:
                case 16:
                    if (curxsize == maxxsize)
                    {
                        if(copyExternal)
                            retValue += externalDataObject->copyFromData2D<ito::uint16>((ito::uint16*)img.bp, img.width, curysize);
                        if(!copyExternal || hasListeners)
                            retValue += m_data.copyFromData2D<ito::uint16>((ito::uint16*)img.bp, img.width, curysize);
                    }
                    else
                    {
                        if(copyExternal)
                            retValue += externalDataObject->copyFromData2D<ito::uint16>((ito::uint16*)img.bp, img.width, img.height, x0, y0, curxsize, curysize);
                        if(!copyExternal || hasListeners)
                            retValue += m_data.copyFromData2D<ito::uint16>((ito::uint16*)img.bp, img.width, img.height, x0, y0, curxsize, curysize);
                    }
                break;

                default:
                    retValue += ito::RetVal(ito::retError, 0, tr("F Wrong picture Type").toAscii().data());
                break;
            }
        }
        this->m_isgrabbing = false;
    }

    return retValue;
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Ximea::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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
/*!
    \detail This method copies the recently grabbed camera frame to the given DataObject. Therefore this camera size must fit to the data structure of the
    DataObject.

    \note This method is similar to VideoCapture::retrieve() of openCV

    \param [in,out] vpdObj is the pointer to a given dataObject (this pointer should be cast to ito::DataObject*) where the acquired data is deep copied to.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if everything is ok, retError is camera has not been started or no data has been acquired by the method acquire.
    \sa DataObject, acquire
*/
ito::RetVal Ximea::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);

    if(!dObj)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Empty object handle retrieved from caller").toAscii().data());
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
void Ximea::updateParameters(QMap<QString, ito::Param> params)
{
    int bitppix_old = 0;
    int binning_old = 0;
    int bitppix_new = 0;
    int binning_new = 0;
    double offset_new = 0.0;
    double value = 0.0;

    char name[40]={0};

    bitppix_old = m_params["bpp"].getVal<int>();
    binning_old = m_params["binning"].getVal<int>();

    foreach(const ito::Param &param1, params)
    {
        memset(name,0,sizeof(name));
        sprintf(name,"%s", param1.getName());
        if(!strlen(name))
            continue;
        QMap<QString, ito::Param>::iterator paramIt = m_params.find(name);
        if (paramIt != m_params.end() && paramIt->isNumeric())
        {
            value = param1.getVal<double>();
            if((value <= paramIt->getMax()) || (value >= paramIt->getMin()))
            {
                paramIt.value().setVal<double>(value);
            }
        }
    }

    bitppix_new = m_params["bpp"].getVal<int>();
    binning_new = m_params["binning"].getVal<int>();
    offset_new = m_params["offset"].getVal<double>();

    if (bitppix_new != bitppix_old)
    {
        setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("bpp", ito::ParamBase::Int, bitppix_new)), NULL);
    }
    else if (binning_new != binning_old)
    {
        setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("binning", ito::ParamBase::Int, binning_new)), NULL);
    }
    else
    {
        setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("offset", ito::ParamBase::Double, offset_new)), NULL);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
//! slot invoked if gain parameters in docking toolbox have been manually changed
/*!
    \param [in] gain
    \param [in] offset
*/
void Ximea::GainPropertiesChanged(double gain)
{
    if( gain <= m_params["gain"].getMax() &&
        gain >= m_params["gain"].getMin())
    {
        setParam( QSharedPointer<ito::ParamBase>(new ito::ParamBase("gain", m_params["gain"].getType(), gain)));
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
//! slot invoked if  offset parameters in docking toolbox have been manually changed
/*!
    \param [in] gain
    \param [in] offset
*/
void Ximea::OffsetPropertiesChanged(double offset)
{
    if( offset <= m_params["offset"].getMax() &&
        offset >= m_params["offset"].getMin())
    {
        setParam( QSharedPointer<ito::ParamBase>(new ito::ParamBase("offset", m_params["offset"].getType(), offset)));
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
//! slot invoked if integrationtime parameters in docking toolbox have been manually changed
/*!
    \param [in] gain
    \param [in] offset
*/
void Ximea::IntegrationPropertiesChanged(double integrationtime)
{
    if( integrationtime <= m_params["integration_time"].getMax() &&
        integrationtime >= m_params["integration_time"].getMin())
    {
        setParam( QSharedPointer<ito::ParamBase>(new ito::ParamBase("integration_time", m_params["integration_time"].getType(), integrationtime)));
    }
}

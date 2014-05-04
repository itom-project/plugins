/* ********************************************************************
    Plugin "PGRFlyCapture" for itom software
    URL: http://www.twip-os.com
    Copyright (C) 2014, twip optical solutions GmbH
	Copyright (C) 2014, Institut für Technische Optik, Universität Stuttgart

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

#include "PGRFlyCapture.h"
#include "pluginVersion.h"
#define _USE_MATH_DEFINES  // needs to be defined to enable standard declartions of PI constant
#include "math.h"

#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>
#include <qmetaobject.h>

#include <qdockwidget.h>
#include <qpushbutton.h>
#include <qmetaobject.h>
#include "dockWidgetPGRFlyCapture.h"

#include "common/helperCommon.h"

//#include <qdebug.h>
//#include <qmessagebox.h>

static signed char InitList[MAXPGR + 1];
static char Initnum = 0;

#define EVALSPEED 0

Q_DECLARE_METATYPE(ito::DataObject)


//----------------------------------------------------------------------------------------------------------------------------------

/*!
    \class PGRFlyCaptureInterface
    \brief Small interface class for class PGRFlyCapture. This class contains basic information about PGRFlyCapture as is able to
        create one or more new instances of PGRFlyCapture.
*/

//----------------------------------------------------------------------------------------------------------------------------------
//! creates new instance of PGRFlyCapture and returns the instance-pointer.
/*!
    \param [in,out] addInInst is a double pointer of type ito::AddInBase. The newly created PGRFlyCapture-instance is stored in *addInInst
    \return retOk
    \sa PGRFlyCapture
*/
ito::RetVal PGRFlyCaptureInterface::getAddInInst(ito::AddInBase **addInInst)
{
    PGRFlyCapture* newInst = new PGRFlyCapture();
    newInst->setBasePlugin(this);
    *addInInst = qobject_cast<ito::AddInBase*>(newInst);

    m_InstList.append(*addInInst);

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! deletes instance of PGRFlyCapture. This instance is given by parameter addInInst.
/*!
    \param [in] double pointer to the instance which should be deleted.
    \return retOk
    \sa PGRFlyCapture
*/
ito::RetVal PGRFlyCaptureInterface::closeThisInst(ito::AddInBase **addInInst)
{
   if (*addInInst)
   {
      delete ((PGRFlyCapture *)*addInInst);
      int idx = m_InstList.indexOf(*addInInst);
      m_InstList.removeAt(idx);
   }

   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! constructor for interace
/*!
    defines the plugin type (dataIO and grabber) and sets the plugins object name. If the real plugin (here: PGRFlyCapture) should or must
    be initialized (e.g. by a Python call) with mandatory or optional parameters, please initialize both vectors m_initParamsMand
    and m_initParamsOpt within this constructor.
*/
PGRFlyCaptureInterface::PGRFlyCaptureInterface()
{
    m_autoLoadPolicy = ito::autoLoadKeywordDefined;
    m_autoSavePolicy = ito::autoSaveAlways;

    m_type = ito::typeDataIO | ito::typeGrabber;
    setObjectName("PGRFlyCapture");

    FlyCapture2::FC2Version fc2Version;
    FlyCapture2::Utilities::GetLibraryVersion( &fc2Version );

    char version[128] = {0};
    _snprintf(
        version,
        128,
        "FlyCapture2 library version: %d.%d.%d.%d\n", 
        (int)fc2Version.major, (int)fc2Version.minor, (int)fc2Version.type, (int)fc2Version.build );

    char timeStamp[512] = {0};
    _snprintf( timeStamp, 512, "Application build date: %s %s\n\n", __DATE__, __TIME__ );

    m_description = QObject::tr("Plugin for PGR FlyCapture2 Camerainterface");
    m_detaildescription.clear();
    m_detaildescription.append("The PGRFlyCapture is a dataIO-Plugin for the FlyCapture2 Camera-Interface by Point Grey Research. It can be used e.g. with the USB 3.0 Flea3 camera device.\n\n");
    m_detaildescription.append(version);
    m_detaildescription.append(timeStamp);

    m_author = "W. Lyda, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("LGPL / copyright of the external DLLs belongs to PointGrayResearch");
    m_aboutThis = QObject::tr("N.A.");    
    
    m_initParamsMand.clear();

    ito::Param param("cameraNumber", ito::ParamBase::Int, -1, 5, -1, QObject::tr("Numeric identifier of the camera. Auto-select is -1").toAscii().data());
    m_initParamsOpt.append(param);
    param = ito::Param("limitBPPRange", ito::ParamBase::Int, -1, 16, -1, QObject::tr("Limit the grabdepth to increase speed [8, 12, 16]. Auto-select is -1").toAscii().data());
    m_initParamsOpt.append(param);
    param = ito::Param("forceSync", ito::ParamBase::Int, 0, 1, 0, QObject::tr("Direct enable software sync if present").toAscii().data());
    m_initParamsOpt.append(param);

    memset(InitList, -1, (MAXPGR + 1) * sizeof(signed char));

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! destructor
/*!
    clears both vectors m_initParamsMand and m_initParamsOpt.
*/
PGRFlyCaptureInterface::~PGRFlyCaptureInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
// this makro registers the class PGRFlyCaptureInterface with the name PGRFlyCaptureinterface as plugin for the Qt-System (see Qt-DOC)
Q_EXPORT_PLUGIN2(PGRFlyCaptureinterface, PGRFlyCaptureInterface)

//----------------------------------------------------------------------------------------------------------------------------------

/*!
    \class PGRFlyCapture
    \brief Class for the PGRFlyCapture. The PGRFlyCapture is a dataIO-Plugin for the FlyCapture2 Camera-Interface by Point Grey Research. It can be used e.g. with the USB 3.0 Flea3 camera device.

    Usually every method in this class can be executed in an own thread. Only the constructor, destructor, showConfDialog will be executed by the
    main (GUI) thread.
*/

//----------------------------------------------------------------------------------------------------------------------------------
//! shows the configuration dialog. This method must be executed in the main (GUI) thread and is usually called by the addIn-Manager.
/*!
    creates new instance of dialogPGRFlyCapture, calls the method setVals of dialogPGRFlyCapture, starts the execution loop and if the dialog
    is closed, reads the new parameter set and deletes the dialog.

    \return retOk
    \sa dialogPGRFlyCapture
*/
const ito::RetVal PGRFlyCapture::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogPGRFlyCapture(this));
}

//----------------------------------------------------------------------------------------------------------------------------------
//! constructor for PGRFlyCapture
/*!
    In this constructor the m_params-vector with all parameters, which are accessible by getParam or setParam, is built.
    Additionally the optional docking widget for the PGRFlyCapture's toolbar is instantiated and created by createDockWidget.

    \param [in] uniqueID is an unique identifier for this PGRFlyCapture-instance
    \sa ito::tParam, createDockWidget, setParam, getParam
*/
PGRFlyCapture::PGRFlyCapture() :
    AddInGrabber(),
    m_isgrabbing(false),
    m_camIdx(-1),
    m_colorCam(false),
    m_RunSync(true),
    m_RunSoftwareSync(false),
    m_isInFormat7(false),
    m_gainMax(1.0),
    m_gainMin(0.0),
    m_offsetMax(1.0),
    m_offsetMin(0.0)
{
    //qRegisterMetaType<QMap<QString, ito::Param> >("QMap<QString, ito::Param>");
    //qRegisterMetaType<ito::DataObject>("ito::DataObject");

    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "PGRFlyCapture", "GrabberName");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("integration_time", ito::ParamBase::Double, 0.000006, 1/1.875, 1/1.875, tr("Integrationtime of CCD programmed in s").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("frame_time", ito::ParamBase::Double, 1/240.0, 1/1.875, 1/1.875, tr("Time between two frames").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("gain", ito::ParamBase::Double, 0.0, 1.0, 0.5, tr("gain").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("offset", ito::ParamBase::Double, 0.0, 1.0, 0.05, tr("offset mapped to brightness").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("exposureEV", ito::ParamBase::Int, 0, 1023, 480, tr("Camera brightness control (EV)").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sharpness", ito::ParamBase::Int, 0, 4095, 0, tr("PGR on chip filter function").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("gamma", ito::ParamBase::Int, 500, 4095, 1024, tr("Gamma adjustment").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("binning", ito::ParamBase::Int | ito::ParamBase::Readonly, 101, 101, 101, tr("Binning of different pixel").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("sizex", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 2048, 2048, tr("Pixelsize in x (cols)").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizey", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 2048, 2048, tr("Pixelsize in y (rows)").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("x0", ito::ParamBase::Int, 0, 2047, 0, tr("Pixelsize in x (cols)").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("y0", ito::ParamBase::Int, 0, 2047, 0, tr("Pixelsize in y (rows)").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("x1", ito::ParamBase::Int, 0, 2047, 2047, tr("Pixelsize in x (cols)").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("y1", ito::ParamBase::Int, 0, 2047, 2047, tr("Pixelsize in y (rows)").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("bpp", ito::ParamBase::Int, 8, 16, 16, tr("Grabdepth of the images").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("time_out", ito::ParamBase::Double, 0.1, 60.0, 60.0, tr("Timeout for acquiring images").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("trigger_mode", ito::ParamBase::Int, -1, 2, 0, tr("-1: Complete free run, 0: Disable trigger, 1: enable trigger mode, 2: enable software-trigger").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("videoMode", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, FlyCapture2::NUM_VIDEOMODES - 1, FlyCapture2::VIDEOMODE_FORMAT7, tr("Current video mode, default is Mode7").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("supported_frame_time", ito::ParamBase::IntArray | ito::ParamBase::Readonly, NULL, tr("Possible valued for the frame_time in frames per second").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("camSerialNumber", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, tr("Serial number of the attachted camera").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("camModel", ito::ParamBase::String | ito::ParamBase::Readonly, "n.a.", tr("Model identifier of the attachted camera").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("camVendor", ito::ParamBase::String | ito::ParamBase::Readonly, "n.a.", tr("Name of the attachted camera vendor").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("camSensor", ito::ParamBase::String | ito::ParamBase::Readonly, "n.a.", tr("Identifier of the chip in attachted camera").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("camResolution", ito::ParamBase::String | ito::ParamBase::Readonly, "n.a.", tr("Resolution of the chip in attachted camera").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("camFirmwareVersion", ito::ParamBase::String | ito::ParamBase::Readonly, "n.a.", tr("Serial number of the firmware used in the attachted camera").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("camFirmwareBuildTime", ito::ParamBase::String | ito::ParamBase::Readonly, "n.a.", tr("Built time of the firmware used in the attachted camera").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

	paramVal = ito::Param("timestamp", ito::ParamBase::Double | ito::ParamBase::Readonly, 0.0, 10000000.0, 0.0, tr("Time in ms since last image (end of exposure)").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    //now create dock widget for this plugin
    DockWidgetPGRFlyCapture *dw = new DockWidgetPGRFlyCapture(this);

    Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
    QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
    createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dw);

    checkData();
}

//----------------------------------------------------------------------------------------------------------------------------------
//! destructor
/*!
    \sa ~AddInBase
*/
PGRFlyCapture::~PGRFlyCapture()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
//! returns parameter of m_params with key name.
/*!
    This method copies the string of the corresponding parameter to val with a maximum length of maxLen.

    \param [in] name is the key name of the parameter
    \param [in,out] val is a shared-pointer of type char*.
    \param [in] maxLen is the maximum length which is allowed for copying to val
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk in case that everything is ok, else retError
    \sa ito::tParam, ItomSharedSemaphore
*/
ito::RetVal PGRFlyCapture::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;
    QString key;
    bool hasIndex = false;
    int index;
    QString suffix;
    QMap<QString,ito::Param>::iterator it;

    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if(retValue == ito::retOk)
    {
        retValue += apiGetParamFromMapByKey(m_params, key, it, false);
    }

    if(!retValue.containsError())
    {
        *val = it.value();
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! sets parameter of m_params with key name.
/*!
    This method copies the given value  to the m_params-parameter.

    \param [in] name is the key name of the parameter
    \param [in] val is the double value to set.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk in case that everything is ok, else retError
    \sa ito::tParam, ItomSharedSemaphore
*/
ito::RetVal PGRFlyCapture::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex;
    int index;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;
    int running = 0;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName( val->getName(), key, hasIndex, index, suffix );

    if(!retValue.containsError())
    {
        retValue += apiGetParamFromMapByKey(m_params, key, it, true);
    }

    if(!retValue.containsError())
    {
        retValue += apiValidateParam(*it, *val, false, true);
    }

    if(!retValue.containsError())
    {
        // Adapted parameters and send out depending parameter
        if(key == "x0" || key == "x1" ||
            key == "y0" || key == "y1")
        {
            retValue += it->copyValueFrom( &(*val) );
            
            static_cast<ito::IntMeta*>( m_params["x0"].getMeta() )->setMax(m_params["x1"].getVal<int>());
            static_cast<ito::IntMeta*>( m_params["y0"].getMeta() )->setMax(m_params["y1"].getVal<int>());
                    
            static_cast<ito::IntMeta*>( m_params["x1"].getMeta() )->setMin(m_params["x0"].getVal<int>());
            static_cast<ito::IntMeta*>( m_params["y1"].getMeta() )->setMin(m_params["y0"].getVal<int>());

            m_params["sizex"].setVal<int>(m_params["x1"].getVal<int>()-m_params["x0"].getVal<int>()+1);
            m_params["sizey"].setVal<int>(m_params["y1"].getVal<int>()-m_params["y0"].getVal<int>()+1);
        }
        else if(key == "frame_time")
        {
            float value = 1.0 / val->getVal<double>();
            retValue += flyCapSetAndGetParameter("frame_time", value, FlyCapture2::FRAME_RATE, true, false, true);
            
            //frame_time changes integration_time as well
            if (!retValue.containsError())
            {
                double frameTime = 1.0/value;
                m_params["frame_time"].setVal<double>(frameTime);
                //m_params["integration_time"].setMax(frameTime);
                static_cast<ito::DoubleMeta*>( m_params["integration_time"].getMeta() )->setMax(frameTime);

                retValue += flyCapSetAndGetParameter("integration_time", value, FlyCapture2::SHUTTER, true, false, true);

                if (!retValue.containsError())
                {
                    m_params["integration_time"].setVal<double>(value / 1000.0);
                }
            }
        }
        else if (key == "integration_time")
        {
            float value = val->getVal<double>() * 1000.0;
            retValue += flyCapSetAndGetParameter("integration_time", value, FlyCapture2::SHUTTER, true, false, true);

            if (!retValue.containsError())
            {
                m_params["integration_time"].setVal<double>(value / 1000.0);
            }
        }
        else if (key == "gain")
        {
            unsigned int value = (uint)(val->getVal<double>() * (m_gainMax - m_gainMin) + m_gainMin + 0.5);
            retValue += flyCapSetAndGetParameter("gain", value, FlyCapture2::GAIN, false, false, true);
            
            if (!retValue.containsError())
            {
                double gain = (value - m_gainMin)/(m_gainMax - m_gainMin);
                m_params["gain"].setVal<double>(gain);
            }
        }
        else if (key == "offset")
        {
            unsigned int value = (uint)(val->getVal<double>() * (m_offsetMax - m_offsetMin) + m_offsetMin + 0.5);
            retValue += flyCapSetAndGetParameter("offset", value, FlyCapture2::BRIGHTNESS, false, false, true);

            if (!retValue.containsError())
            {
                double offset = (value - m_offsetMin)/(m_offsetMax - m_offsetMin);
                m_params["offset"].setVal<double>(offset);
            }
        }
        else if (key == "gamma")
        {
            unsigned int value = (uint)(val->getVal<int>());
            retValue += flyCapSetAndGetParameter("gamma", value, FlyCapture2::GAMMA, false, false, true);

            if (!retValue.containsError())
            {
                m_params["gamma"].setVal<int>(value);
            }
        }
        else if (key == "sharpness")
        {
            unsigned int value = (uint)(val->getVal<int>());
            retValue += flyCapSetAndGetParameter("sharpness", value, FlyCapture2::SHARPNESS, false, false, true);

            if (!retValue.containsError())
            {
                m_params["sharpness"].setVal<int>(value);
            }

        }
        else if (key =="exposureEV")
        {
            unsigned int value = (uint)(val->getVal<int>());
            retValue += flyCapSetAndGetParameter("exposureEV", value, FlyCapture2::AUTO_EXPOSURE, false, false, true);

            if (!retValue.containsError())
            {
                m_params["exposureEV"].setVal<int>(value);
            }
        }
        else if (key == "trigger_mode")
        {

            if (grabberStartedCount() > 0)
            {
                running = grabberStartedCount();
                setGrabberStarted(1);
                retValue += stopDevice(NULL);
            }

            // The trigger must be present --> because the m_param is not readonly
            FlyCapture2::TriggerMode triggerModeSetup;
            m_myCam.GetTriggerMode(&triggerModeSetup);

            int triggerMode = val->getVal<int>();
                    
            switch(triggerMode)
            {
                case 0: //Basic Mode
                    m_RunSync = true;   // Force a synchronisation by delay
                    m_RunSoftwareSync = false;  // Force synchronisation by software trigger during aquire
                    triggerModeSetup.onOff = false;
                    break;
                case 1: //Hardware Trigger
                    m_RunSync = false;
                    m_RunSoftwareSync = false;
                    triggerModeSetup.onOff = true; 
                    break;
                case 2: //Software Trigger
                    m_RunSync = false;
                    m_RunSoftwareSync = true;
                    triggerModeSetup.onOff = true;
                    break;
                case -1: //Free run
                    m_RunSync = false;
                    m_RunSoftwareSync = false;
                    triggerModeSetup.onOff = false;
                    break;
            }

            FlyCapture2::Error retError = m_myCam.SetTriggerMode(&triggerModeSetup);
            if (retError != FlyCapture2::PGRERROR_OK)
            {
                retValue += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in setParam-function: %s", retError.GetDescription());
            }
            else
            {
                m_params["trigger"].setVal<int>(triggerMode);
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
        retValue += checkData(); //check if image must be reallocated

        if (running)
        {
            retValue += startDevice(NULL);
            setGrabberStarted(running);
        }

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
ito::RetVal PGRFlyCapture::flyCapSetAndGetParameter(const QString &name, unsigned int &value, FlyCapture2::PropertyType type, bool absControl /*= false*/, bool autoManualMode /*= false*/, bool onOff /*= true*/)
{
    FlyCapture2::Property prop(type);
    prop.absControl = absControl;
    prop.autoManualMode = autoManualMode;
    prop.onOff = onOff;
    prop.valueA = value;
    FlyCapture2::Error retErr = m_myCam.SetProperty( &prop );

    if (retErr != FlyCapture2::PGRERROR_OK)
    {
        return ito::RetVal::format(ito::retError, (int)retErr.GetType(), "Error setting parameter %s: %s", name.toLatin1().data(), retErr.GetDescription());
    }

    retErr = m_myCam.GetProperty( &prop );
    if (retErr == FlyCapture2::PGRERROR_OK)
    {
        value = prop.valueA;
    }
    else
    {
        return ito::RetVal::format(ito::retError, (int)retErr.GetType(), "Error getting parameter %s: %s", name.toLatin1().data(), retErr.GetDescription());
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PGRFlyCapture::flyCapSetAndGetParameter(const QString &name, float &value, FlyCapture2::PropertyType type, bool absControl /*= false*/, bool autoManualMode /*= false*/, bool onOff /*= true*/)
{
    FlyCapture2::Property prop(type);
    prop.absControl = absControl;
    prop.autoManualMode = autoManualMode;
    prop.onOff = onOff;
    prop.absValue = value;
    FlyCapture2::Error retErr = m_myCam.SetProperty( &prop );

    if (retErr != FlyCapture2::PGRERROR_OK)
    {
        return ito::RetVal::format(ito::retError, (int)retErr.GetType(), "Error setting parameter %s: %s", name.toLatin1().data(), retErr.GetDescription());
    }

    retErr = m_myCam.GetProperty( &prop );
    if (retErr == FlyCapture2::PGRERROR_OK)
    {
        value = prop.absValue;
    }
    else
    {
        return ito::RetVal::format(ito::retError, (int)retErr.GetType(), "Error getting parameter %s: %s", name.toLatin1().data(), retErr.GetDescription());
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PGRFlyCapture::flyCapGetParameter(const QString &name, unsigned int &value, FlyCapture2::PropertyType type, bool absControl /*= false*/, bool autoManualMode /*= false*/, bool onOff /*= true*/)
{
    FlyCapture2::Property prop(type);
    prop.absControl = absControl;
    prop.autoManualMode = autoManualMode;
    prop.onOff = onOff;
    prop.valueA = value;

    FlyCapture2::Error retErr = m_myCam.GetProperty( &prop );
    if (retErr == FlyCapture2::PGRERROR_OK)
    {
        value = prop.valueA;
    }
    else
    {
        return ito::RetVal::format(ito::retError, (int)retErr.GetType(), "Error getting parameter %s: %s", name.toLatin1().data(), retErr.GetDescription());
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PGRFlyCapture::flyCapGetParameter(const QString &name, float &value, FlyCapture2::PropertyType type, bool absControl /*= false*/, bool autoManualMode /*= false*/, bool onOff /*= true*/)
{
    FlyCapture2::Property prop(type);
    prop.absControl = absControl;
    prop.autoManualMode = autoManualMode;
    prop.onOff = onOff;
    prop.absValue = value;

    FlyCapture2::Error retErr = m_myCam.GetProperty( &prop );
    if (retErr == FlyCapture2::PGRERROR_OK)
    {
        value = prop.absValue;
    }
    else
    {
        return ito::RetVal::format(ito::retError, (int)retErr.GetType(), "Error getting parameter %s: %s", name.toLatin1().data(), retErr.GetDescription());
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! init method which is called by the addInManager after the initiation of a new instance of PGRFlyCapture.
/*!
    This init method gets the mandatory and optional parameter vectors of type tParam and must copy these given parameters to the
    internal m_params-vector. Notice that this method is called after that this instance has been moved to its own (non-gui) thread.

    \param [in] paramsMand is a pointer to the vector of mandatory tParams.
    \param [in] paramsOpt is a pointer to the vector of optional tParams.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
*/
ito::RetVal PGRFlyCapture::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);

    FlyCapture2::Error retError;
    FlyCapture2::BusManager busMgr;
    FlyCapture2::CameraInfo camInfo;
    FlyCapture2::PixelFormat pixelFormat;
    FlyCapture2::FrameRate frameRate;
    FlyCapture2::VideoMode videoMode;
    FlyCapture2::FC2Config pCamConfig;
        
    int maxBitsPerPixel;
    int minBitsPerPixel;

    ito::RetVal retVal(ito::retOk,0,"");

    Initnum++;
    unsigned int numberOfCam = 0;
        
    retError = busMgr.GetNumOfCameras(&numberOfCam);
    if (retError != FlyCapture2::PGRERROR_OK)
    {
        retVal += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in init-function: %s", retError.GetDescription());
    }

    m_camIdx = (*paramsOpt)[0].getVal<int>();    // the first parameter in optional list is for the camera index
    int limitBPP = (*paramsOpt)[1].getVal<int>();    // the second parameter in optional list is for the max BPP
    bool startSyncronized = (*paramsOpt)[2].getVal<int>();    // the third parameter in optional list handels synchronisation
    signed char numberOfCam_ = (signed char)numberOfCam;

    if(m_camIdx < 0)
    {
        bool found;
        signed char curCamIdx;
        
        //take the first free camera
        for(curCamIdx = 0; curCamIdx < numberOfCam_; curCamIdx++)
        {
            //check whether this camera is not used yet
            found = false;
            for(int i = 0; i < MAXPGR; i++)
            {
                if(InitList[i] == curCamIdx)
                {
                    found = true;
                    break;
                }
            }

            if (!found) //curCamIdx has not been used yet
            {
                m_camIdx = curCamIdx;
                break;
            }

        }
        if ( m_camIdx < 0)
        {
            retVal += ito::RetVal(ito::retError, 0, "No free camera found.");
        }
    }
    else
    {
        if(m_camIdx < numberOfCam_)
        {
            for(int i = 0; i < MAXPGR; i++)
            {
                if(InitList[i] == m_camIdx)
                {
                    m_camIdx = -1;
                    retVal += ito::RetVal(ito::retError, 0, "Camera already initialized");
                }
            }
        }
        else
        {
            m_camIdx = -1;
            retVal += ito::RetVal(ito::retError, 0, "Camera index out of range");
        }
    }

    if(!retVal.containsError())
    {    
        retError = busMgr.GetCameraFromIndex(m_camIdx, &m_myGUID);
        if (retError != FlyCapture2::PGRERROR_OK)
        {
            m_camIdx = -1;       
            retVal += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in init-function: %s", retError.GetDescription());
        }
        else
        {
            for(int i = 0; i < MAXPGR; i++)
            {
                if(InitList[i] == -1)
                {
                    InitList[i] = m_camIdx;
                    break;
                }
            }            
        }
    }

    if(!retVal.containsError())
    {
        
        retError = m_myCam.Connect(&m_myGUID);
        if (retError != FlyCapture2::PGRERROR_OK)
        {
            retVal += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in init-function: %s", retError.GetDescription());
        }
    }

    if(!retVal.containsError())
    {
        retError = m_myCam.GetCameraInfo(&camInfo);
        if (retError != FlyCapture2::PGRERROR_OK)
        {
            retVal += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in init-function: %s", retError.GetDescription());
        }
        else
        {
            m_colorCam = camInfo.isColorCamera;
        }
    }

    FlyCapture2::Format7Info p7Info;
    bool hasFormat7;
    
    if(!retVal.containsError())
    {
        retError = m_myCam.GetFormat7Info(&p7Info, &hasFormat7);
        if (retError != FlyCapture2::PGRERROR_OK)
        {
            retVal += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in init-function: %s", retError.GetDescription());
        }
    }

    if(!retVal.containsError())
    {
        retError = m_myCam.GetVideoModeAndFrameRate(&videoMode, &frameRate);
        if (retError != FlyCapture2::PGRERROR_OK)
        {
            retVal += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in init-function: %s", retError.GetDescription());
        }
    }
            
    if(!retVal.containsError())
    {
        if(hasFormat7)    // camera has mode mode 7
        {
            if(videoMode != FlyCapture2::VIDEOMODE_FORMAT7)
            {
                retError = m_myCam.SetVideoModeAndFrameRate(FlyCapture2::VIDEOMODE_FORMAT7, FlyCapture2::FRAMERATE_FORMAT7);
                if (retError != FlyCapture2::PGRERROR_OK)
                {
                    retVal += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in init-function: %s", retError.GetDescription());
                }
            }

            if(!retVal.containsError())
            {
                FlyCapture2::Format7ImageSettings f7ImageSettings;
                unsigned int packetSize;
                float percentage;

                retError = m_myCam.GetFormat7Configuration(&f7ImageSettings, &packetSize, &percentage);
                if (retError != FlyCapture2::PGRERROR_OK)
                {
                    retVal += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in init-function: %s", retError.GetDescription());
                }
                else
                {
                    m_params["videoMode"].setVal<int>(FlyCapture2::VIDEOMODE_FORMAT7);
                    m_isInFormat7 = true;

                    f7ImageSettings.mode = FlyCapture2::MODE_0;
                    f7ImageSettings.width = p7Info.maxWidth; // 2.0;
                    f7ImageSettings.height = p7Info.maxHeight; // 2.0;                   

                    minBitsPerPixel = 32;
                    maxBitsPerPixel = 8;

                    if(FlyCapture2::PIXEL_FORMAT_RAW8 & p7Info.pixelFormatBitField || FlyCapture2::PIXEL_FORMAT_RAW8 & p7Info.pixelFormatBitField)
                    {
                        if(minBitsPerPixel > 8) minBitsPerPixel = 8;
                        if(maxBitsPerPixel < 8) maxBitsPerPixel = 8;
                    }

                    if(FlyCapture2::PIXEL_FORMAT_MONO12 & p7Info.pixelFormatBitField || FlyCapture2::PIXEL_FORMAT_RAW12 & p7Info.pixelFormatBitField)
                    {
                        if(minBitsPerPixel > 12) minBitsPerPixel = 12;
                        if(maxBitsPerPixel < 12) maxBitsPerPixel = 12;
                    }

                    if(FlyCapture2::PIXEL_FORMAT_MONO16 & p7Info.pixelFormatBitField || FlyCapture2::PIXEL_FORMAT_RAW16 & p7Info.pixelFormatBitField)
                    {
                        if(minBitsPerPixel > 16) minBitsPerPixel = 16;
                        if(maxBitsPerPixel < 16) maxBitsPerPixel = 16;
                    }
                    
                    if((maxBitsPerPixel  < 9) || (limitBPP == 8))
                    {
                        f7ImageSettings.pixelFormat = FlyCapture2::PIXEL_FORMAT_RAW8;
                    }
                    else if (limitBPP == 12)
                    {
                        f7ImageSettings.pixelFormat = FlyCapture2::PIXEL_FORMAT_MONO12;
                    }
                    else if ((maxBitsPerPixel < 17) || (limitBPP == 16))
                    {
                        // This comes from the SDK and seems to wrap the little endian / big endian problem
                        const unsigned int k_imageDataFmtReg = 0x1048;
                        unsigned int value = 0;
                        retError = m_myCam.ReadRegister( k_imageDataFmtReg, &value );
                        if ( retError != FlyCapture2::PGRERROR_OK )
                        {
                            // Error
                        }
//#ifdef _DEBUG
//                        std::cout << "Read value: " << QString::number(value).toLatin1().data() << "\n";
//#endif
                        value &= ~(0x1 << 0);
//#ifdef _DEBUG
//                        std::cout << "Wrote value: " << QString::number(value).toLatin1().data() << " from " << QString::number((0x1 << 0)).toLatin1().data() << "\n";
//#endif
                        retError = m_myCam.WriteRegister( k_imageDataFmtReg, value );
                        if ( retError != FlyCapture2::PGRERROR_OK )
                        {
                            // Error
                        }
                        f7ImageSettings.pixelFormat = FlyCapture2::PIXEL_FORMAT_MONO16;
                    }
                    
                    retError = m_myCam.SetFormat7Configuration(&f7ImageSettings, percentage);
                    if (retError != FlyCapture2::PGRERROR_OK)
                    {
                        retVal += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in init-function: %s", retError.GetDescription());
                    }
                    else
                    {
                        retError = m_myCam.GetFormat7Configuration(&f7ImageSettings, &packetSize, &percentage);
                        if (retError != FlyCapture2::PGRERROR_OK)
                        {
                            retVal += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in init-function: %s", retError.GetDescription());
                        }
                    }
                    
                    m_params["sizex"].setVal<int>(f7ImageSettings.width);
                    static_cast<ito::IntMeta*>( m_params["sizex"].getMeta() )->setMax(f7ImageSettings.width);
                    static_cast<ito::IntMeta*>( m_params["x0"].getMeta() )->setMax(f7ImageSettings.width-1);
                    static_cast<ito::IntMeta*>( m_params["x1"].getMeta() )->setMax(f7ImageSettings.width-1);
                    /*m_params["sizex"].setMax((double)f7ImageSettings.width);
                    m_params["x0"].setMax((double)f7ImageSettings.width-1.0);
                    m_params["x1"].setMax((double)f7ImageSettings.width-1.0);*/
                    m_params["x1"].setVal<int>(f7ImageSettings.width-1);

                    m_params["sizey"].setVal<int>(f7ImageSettings.height);
                   /* m_params["sizey"].setMax((double)f7ImageSettings.height);
                    m_params["y0"].setMax((double)f7ImageSettings.height-1.0);
                    m_params["y1"].setMax((double)f7ImageSettings.height-1.0);*/
                    static_cast<ito::IntMeta*>( m_params["sizey"].getMeta() )->setMax(f7ImageSettings.height);
                    static_cast<ito::IntMeta*>( m_params["y0"].getMeta() )->setMax(f7ImageSettings.height-1);
                    static_cast<ito::IntMeta*>( m_params["y1"].getMeta() )->setMax(f7ImageSettings.height-1);
                    m_params["y1"].setVal<int>(f7ImageSettings.height-1);

                    pixelFormat = f7ImageSettings.pixelFormat;

                }
            }

        }
        else
        {
            m_isInFormat7 = false;

            int width;
            int height;
            bool isStippled = false;
            
            FlyCapture2::PropertyInfo propInfo;
            propInfo.type = FlyCapture2::WHITE_BALANCE;

            m_myCam.GetPropertyInfo(&propInfo);

            if (propInfo.present)
            {
                isStippled = true;
            }

            if (!GetResolutionFromVideoMode(videoMode, width, height))
            {
                retVal += ito::RetVal(ito::retError, 0, "Get resolution failed");
            }
            else
            {
                m_params["videoMode"].setVal<int>(videoMode);

                m_params["sizex"].setVal<int>(width);
                /*m_params["sizex"].setMax((double)width);
                m_params["x0"].setMax((double)width-1.0);
                m_params["x1"].setMax((double)width-1.0);*/
                static_cast<ito::DoubleMeta*>( m_params["sizex"].getMeta() )->setMax(height);
                static_cast<ito::DoubleMeta*>( m_params["x0"].getMeta() )->setMax(height-1);
                static_cast<ito::DoubleMeta*>( m_params["x1"].getMeta() )->setMax(height-1);
                m_params["x1"].setVal<int>(width-1);

                m_params["sizey"].setVal<int>(height);
                /*m_params["sizey"].setMax((double)height);
                m_params["y0"].setMax((double)height-1.0);
                m_params["y1"].setMax((double)height-1.0);*/
                static_cast<ito::DoubleMeta*>( m_params["sizey"].getMeta() )->setMax(height);
                static_cast<ito::DoubleMeta*>( m_params["y0"].getMeta() )->setMax(height-1);
                static_cast<ito::DoubleMeta*>( m_params["y1"].getMeta() )->setMax(height-1);
                m_params["y1"].setVal<int>(height-1);          
            }

            if (!GetPixelFormatFromVideoMode(videoMode, isStippled, &pixelFormat))
            {
                pixelFormat = FlyCapture2::PIXEL_FORMAT_RAW8;
            }
        }
    }
    
    if(!retVal.containsError())
    {
        retError = m_myCam.GetConfiguration(&pCamConfig);
        if (retError != FlyCapture2::PGRERROR_OK)
        {
            retVal += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in init-function: %s", retError.GetDescription());
        }
    }

    if(!retVal.containsError())
    {
        FlyCapture2:: PropertyInfo propInfo;
        propInfo.type = FlyCapture2::FRAME_RATE;

        retError = m_myCam.GetPropertyInfo( &propInfo );

        if (retError != FlyCapture2::PGRERROR_OK)
        {
            retVal += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in init-function: %s", retError.GetDescription());
        }
        else
        {
            if(propInfo.present && propInfo.absValSupported && propInfo.manualSupported)
            {
                //m_params["frame_time"].setMax(1/propInfo.absMax);
                //m_params["frame_time"].setMin(1/propInfo.absMin); 
                m_params["frame_time"].setMeta( new ito::DoubleMeta(1.0/propInfo.absMin, 1.0/propInfo.absMax ), true);

                FlyCapture2::Property prop;
                prop.type = FlyCapture2::FRAME_RATE;
                prop.autoManualMode = true;
                prop.onOff = true;
                prop.absControl = true;
                prop.absControl = propInfo.absMax;

                retError = m_myCam.SetProperty( &prop );
                if (retError != FlyCapture2::PGRERROR_OK)
                {
                    retVal += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in init-function: %s", retError.GetDescription());
                }

                retError = m_myCam.GetProperty( &prop );
                if (retError != FlyCapture2::PGRERROR_OK)
                {
                    retVal += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in init-function: %s", retError.GetDescription());
                }
                else
                {
                    double frameTime = 1.0/prop.absValue;
                    m_params["frame_time"].setVal<double>(frameTime);
                    //m_params["integration_time"].setMax(frameTime);
                    static_cast<ito::DoubleMeta*>( m_params["integration_time"].getMeta() )->setMax(frameTime);

                    prop.autoManualMode = false;
                    prop.onOff = true;
                    prop.absControl = true;

                    retError = m_myCam.SetProperty( &prop );

                    if (retError != FlyCapture2::PGRERROR_OK)
                    {
                        retVal += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in init-function: %s", retError.GetDescription());
                    }
                }

            }
            else
            {
                m_params["frame_time"].setFlags(ito::ParamBase::Readonly);
            }
        }
    }

    if(!retVal.containsError())
    {
        FlyCapture2:: PropertyInfo propInfo;
        propInfo.type = FlyCapture2::SHUTTER;

        retError = m_myCam.GetPropertyInfo( &propInfo );

        if (retError != FlyCapture2::PGRERROR_OK)
        {
            retVal += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in init-function: %s", retError.GetDescription());
        }
        else
        {
            if(propInfo.present && propInfo.absValSupported && propInfo.manualSupported)
            {
                //m_params["integration_time"].setMin(propInfo.absMin / 1000.0);   
                static_cast<ito::DoubleMeta*>( m_params["integration_time"].getMeta() )->setMin(propInfo.absMin / 1000.0);

                FlyCapture2::Property prop;
                prop.type = FlyCapture2::SHUTTER;
                prop.absControl = true;

                retError = m_myCam.GetProperty( &prop );
                if (retError != FlyCapture2::PGRERROR_OK)
                {
                    retVal += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in init-function: %s", retError.GetDescription());
                }
                else
                {
                    double intTime = prop.absValue / 1000.0;
                    m_params["integration_time"].setVal<double>(intTime);

                    prop.autoManualMode = false;
                    prop.onOff = true;
                    retError = m_myCam.SetProperty( &prop );

                    if (retError != FlyCapture2::PGRERROR_OK)
                    {
                        retVal += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in init-function: %s", retError.GetDescription());
                    }
                }
            }
            else
            {
                m_params["integration_time"].setFlags(ito::ParamBase::Readonly);
            }
        }
    }

    if(!retVal.containsError())
    {

        FlyCapture2:: PropertyInfo propInfo;

        propInfo.type = FlyCapture2::GAIN;
        retError = m_myCam.GetPropertyInfo( &propInfo );

        if (retError != FlyCapture2::PGRERROR_OK)
        {
            retVal += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in init-function: %s", retError.GetDescription());
        }
        else
        {

            if(propInfo.present && propInfo.manualSupported)
            {
                m_gainMax = propInfo.max;
                m_gainMin = propInfo.min;    

                FlyCapture2::Property prop;
                prop.type = FlyCapture2::GAIN;

                retError = m_myCam.GetProperty( &prop );
                if (retError != FlyCapture2::PGRERROR_OK)
                {
                    retVal += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in init-function: %s", retError.GetDescription());
                }
                else
                {
                    double gain = (prop.valueA - m_gainMin) / (m_gainMax-m_gainMin) ;
                    m_params["gain"].setVal<double>(gain);

                    prop.autoManualMode = false;
                    prop.onOff = true;
                    prop.absControl = false;

                    retError = m_myCam.SetProperty( &prop );

                    if (retError != FlyCapture2::PGRERROR_OK)
                    {
                        retVal += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in init-function: %s", retError.GetDescription());
                    }
                }
            }
            else
            {
                m_params["gain"].setFlags(ito::ParamBase::Readonly);
            }
        }
    }

    if(!retVal.containsError())
    {

        FlyCapture2:: PropertyInfo propInfo;

        propInfo.type = FlyCapture2::BRIGHTNESS;
        retError = m_myCam.GetPropertyInfo( &propInfo );

        if (retError != FlyCapture2::PGRERROR_OK)
        {
            retVal += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in init-function: %s", retError.GetDescription());
        }
        else
        {

            if(propInfo.present && propInfo.manualSupported)
            {
                m_offsetMax = propInfo.max;
                m_offsetMin = propInfo.min;    

                FlyCapture2::Property prop;
                prop.type = FlyCapture2::BRIGHTNESS;

                retError = m_myCam.GetProperty( &prop );
                if (retError != FlyCapture2::PGRERROR_OK)
                {
                    retVal += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in init-function: %s", retError.GetDescription());
                }
                else
                {
                    double gain = (prop.valueA - m_offsetMin) / (m_offsetMax - m_offsetMin) ;
                    m_params["offset"].setVal<double>(gain);

                    prop.autoManualMode = false;
                    prop.onOff = true;
                    prop.absControl = false;

                    retError = m_myCam.SetProperty( &prop );

                    if (retError != FlyCapture2::PGRERROR_OK)
                    {
                        retVal += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in init-function: %s", retError.GetDescription());
                    }
                }
            }
            else
            {
                m_params["offset"].setFlags(ito::ParamBase::Readonly);
            }
        }
    }

    if(!retVal.containsError())
    {

        FlyCapture2:: PropertyInfo propInfo;

        propInfo.type = FlyCapture2::GAMMA;
        retError = m_myCam.GetPropertyInfo( &propInfo );

        if (retError != FlyCapture2::PGRERROR_OK)
        {
            retVal += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in init-function: %s", retError.GetDescription());
        }
        else
        {

            if(propInfo.present && propInfo.manualSupported)
            {

                FlyCapture2::Property prop;
                prop.type = FlyCapture2::GAMMA;

                retError = m_myCam.GetProperty( &prop );
                if (retError != FlyCapture2::PGRERROR_OK)
                {
                    retVal += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in init-function: %s", retError.GetDescription());
                }
                else
                {
                    m_params["gamma"].setVal<int>(prop.valueA);
                    m_params["gamma"].setMeta( new ito::IntMeta((int)propInfo.min, (int)propInfo.max), true );
                    prop.autoManualMode = false;
                    prop.onOff = true;
                    prop.absControl = false;

                    retError = m_myCam.SetProperty( &prop );

                    if (retError != FlyCapture2::PGRERROR_OK)
                    {
                        retVal += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in init-function: %s", retError.GetDescription());
                    }
                }
            }
            else
            {
                m_params["gamma"].setFlags(ito::ParamBase::Readonly);
            }
        }
    }

    if(!retVal.containsError())
    {

        FlyCapture2:: PropertyInfo propInfo;

        propInfo.type = FlyCapture2::SHARPNESS;
        retError = m_myCam.GetPropertyInfo( &propInfo );

        if (retError != FlyCapture2::PGRERROR_OK)
        {
            retVal += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in init-function: %s", retError.GetDescription());
        }
        else
        {

            if(propInfo.present && propInfo.manualSupported)
            {

                FlyCapture2::Property prop;
                prop.type = FlyCapture2::SHARPNESS;

                retError = m_myCam.GetProperty( &prop );
                if (retError != FlyCapture2::PGRERROR_OK)
                {
                    retVal += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in init-function: %s", retError.GetDescription());
                }
                else
                {
                    m_params["sharpness"].setVal<int>(prop.valueA);
                    m_params["sharpness"].setMeta( new ito::IntMeta((int)propInfo.min, (int)propInfo.max), true );
                    prop.autoManualMode = false;
                    prop.onOff = true;
                    prop.absControl = false;

                    retError = m_myCam.SetProperty( &prop );

                    if (retError != FlyCapture2::PGRERROR_OK)
                    {
                        retVal += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in init-function: %s", retError.GetDescription());
                    }
                }
            }
            else
            {
                m_params["sharpness"].setFlags(ito::ParamBase::Readonly);
            }
        }
    }

    if(!retVal.containsError())
    {

        FlyCapture2:: PropertyInfo propInfo;

        propInfo.type = FlyCapture2::AUTO_EXPOSURE;
        retError = m_myCam.GetPropertyInfo( &propInfo );

        if (retError != FlyCapture2::PGRERROR_OK)
        {
            retVal += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in init-function: %s", retError.GetDescription());
        }
        else
        {

            if(propInfo.present && propInfo.manualSupported)
            {

                FlyCapture2::Property prop;
                prop.type = FlyCapture2::AUTO_EXPOSURE;

                retError = m_myCam.GetProperty( &prop );
                if (retError != FlyCapture2::PGRERROR_OK)
                {
                    retVal += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in init-function: %s", retError.GetDescription());
                }
                else
                {
                    m_params["exposureEV"].setVal<int>(prop.valueA);
                    m_params["exposureEV"].setMeta( new ito::IntMeta((int)propInfo.min, (int)propInfo.max), true );
                    prop.autoManualMode = false;
                    prop.onOff = true;
                    prop.absControl = false;

                    retError = m_myCam.SetProperty( &prop );

                    if (retError != FlyCapture2::PGRERROR_OK)
                    {
                        retVal += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in init-function: %s", retError.GetDescription());
                    }
                }
            }
            else
            {
                m_params["exposureEV"].setFlags(ito::ParamBase::Readonly);
            }
        }
    }

    if(!retVal.containsError())
    {
        int timeOutMS = pCamConfig.grabTimeout;
        timeOutMS = (int)(m_params["time_out"].getVal<double>() * 1000 + 0.5);
        pCamConfig.grabTimeout = timeOutMS;
        retError = m_myCam.SetConfiguration(&pCamConfig);
        if (retError != FlyCapture2::PGRERROR_OK)
        {
            retVal += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in init-function: %s", retError.GetDescription());
        }
    }

    if(!retVal.containsError())
    {
        FlyCapture2::TriggerModeInfo pTriggerInfo;
        FlyCapture2::TriggerMode triggerModeSetup;

        m_myCam.GetTriggerModeInfo(&pTriggerInfo);
        m_myCam.GetTriggerMode(&triggerModeSetup);
        
        m_RunSync = true;
        m_RunSoftwareSync = false;

        if(pTriggerInfo.present && pTriggerInfo.softwareTriggerSupported)
        {
            triggerModeSetup.onOff = false;
            m_myCam.SetTriggerMode(&triggerModeSetup);
            m_params["trigger_mode"].setVal<int>(0);  

            if(pTriggerInfo.softwareTriggerSupported)
            {
                //m_params["trigger_mode"].setMax(2); 
                static_cast<ito::IntMeta*>( m_params["trigger_mode"].getMeta() )->setMax(2);
            }
            else
            {
                //m_params["trigger_mode"].setMax(1);
                static_cast<ito::IntMeta*>( m_params["trigger_mode"].getMeta() )->setMax(1);
            }
        }
        else
        {
            m_params["trigger_mode"].setVal<int>(0);
            m_params["trigger_mode"].setFlags(ito::ParamBase::Readonly);
            //m_params["trigger_mode"].setMax(0);
            static_cast<ito::IntMeta*>( m_params["trigger_mode"].getMeta() )->setMax(0);
        }

        if(startSyncronized && (m_params["trigger_mode"].getMax() == 2))
        {
            retVal += setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("trigger_mode", ito::ParamBase::Int, 2)), 0);
            if(!retVal.containsError())
            {
                m_myCam.GetTriggerMode(&triggerModeSetup);
                triggerModeSetup.onOff = false;
                m_myCam.SetTriggerMode(&triggerModeSetup);
                m_params["trigger_mode"].setVal<int>(0);
                //m_params["trigger_mode"].setMax(1); 
                static_cast<ito::IntMeta*>( m_params["trigger_mode"].getMeta() )->setMax(1);
            }
        }

    }

    if(!retVal.containsError())
    {
        int curBpp = GetBppFromPixelFormat(pixelFormat);
        if(curBpp < 8)
        {
            retVal += ito::RetVal(ito::retError, 0, "Get bits per pixel failed");
        }
        else
        {
            if(m_colorCam)
            {
                if(curBpp <25)
                {
                    m_params["bpp"].setVal<int>(24);
                    m_params["bpp"].setMeta( new ito::IntMeta(24,24), true );
                }
                else if (curBpp <33)
                {
                    m_params["bpp"].setVal<int>(30);  
                    m_params["bpp"].setMeta( new ito::IntMeta(30,30), true );
                }
                else
                {
                    m_params["bpp"].setVal<int>(48); 
                    m_params["bpp"].setMeta( new ito::IntMeta(48,48), true );
                }
            }
            else
            {
                m_params["bpp"].setVal<int>(curBpp);   
                m_params["bpp"].setMeta( new ito::IntMeta(minBitsPerPixel,maxBitsPerPixel), true );
            }
        
        }
    }

    if(!retVal.containsError())
    {
        /*
                    paramVal = ito::tParam("integration_time", ito::ParamBase::Double, 0.005, 100.0, 12.5, tr("Integrationtime of CCD programmed in s").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::tParam("frame_time", ito::ParamBase::Double, 0.01, 1.0, 0.1, tr("Time between two frames").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::tParam("gain", ito::ParamBase::Double, 0.0, 1.0, 1.0, tr("gain").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::tParam("offset", ito::ParamBase::Double, 0.0, 1.0, 0.5, tr("offset").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    */
    }



    if(!retVal.containsError())
    {
        
    /*
            if(sizeX % 2 != 0)
            {
                m_params["binning"].setMax(102);
            }

            if(sizeY % 2 != 0)
            {
                int binning = (int)m_params["binning"].getMax();
                m_params["binning"].setMax(binning - 1);
            }
       */    
        m_params["camSerialNumber"].setVal<int>((int)camInfo.serialNumber);
        setIdentifier(QString("%1 (%2)").arg( camInfo.modelName ).arg( camInfo.serialNumber ));
        m_params["camModel"].setVal<char*>(camInfo.modelName, (int)strlen(camInfo.modelName));
        m_params["camVendor"].setVal<char*>(camInfo.vendorName, (int)strlen(camInfo.vendorName));
        m_params["camSensor"].setVal<char*>(camInfo.sensorInfo, (int)strlen(camInfo.sensorInfo));
        m_params["camResolution"].setVal<char*>(camInfo.sensorResolution, (int)strlen(camInfo.sensorResolution));
        m_params["camFirmwareVersion"].setVal<char*>(camInfo.firmwareVersion, (int)strlen(camInfo.firmwareVersion));
        m_params["camFirmwareBuildTime"].setVal<char*>(camInfo.firmwareBuildTime, (int)strlen(camInfo.firmwareBuildTime));
    }


    if(!retVal.containsError())
    {
        checkData(); //check if image must be reallocated
    }

    /*if(!retVal.containsError())
    {
        if(camInfo.serialNumber > 10) m_uniqueID = camInfo.serialNumber;
    }*/
    

    if(waitCond)
    {
        waitCond->returnValue = retVal;
        waitCond->release();

        setInitialized(true); //init method has been finished (independent on retval)
        return waitCond->returnValue;
    }
    else
    {
        setInitialized(true); //init method has been finished (independent on retval)
        return ito::retOk;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
//! close method which is called before that this instance is deleted by the PGRFlyCaptureInterface
/*!
    notice that this method is called in the actual thread of this instance.

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal PGRFlyCapture::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    FlyCapture2::Error retError;
    ito::RetVal retValue(ito::retOk, 0,"");

    if(m_camIdx > -1)
    {
        retError = m_myCam.Disconnect();
        if (retError != FlyCapture2::PGRERROR_OK)
        {
                retValue += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in close-function: %s", retError.GetDescription());
        }

        for(int i = 0; i < MAXPGR; i++)
        {
            if(InitList[i] == m_camIdx)
            {
                InitList[i] = -1;
                break;
            }
        }
    }
    Initnum--;

    if(Initnum < 1)
    {
        for(int i = 0; i < MAXPGR; i++)
        {
            if(InitList[i] > -1)
            {
                retValue += ito::RetVal(ito::retWarning, 0, "Found some uninitilized but still present grabber");
            }
        }        
    }

    if(m_timerID > 0)
    {
        killTimer(m_timerID);
        m_timerID = 0;
    }

    if(waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();

        return retValue;
    }
    else
    {
        return retValue;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
// function moved into addInGrabber.cpp -> standard for all cameras / ADDA


//----------------------------------------------------------------------------------------------------------------------------------
//! With startDevice this camera is initialized.
/*!
    In the PGRFlyCapture, this method does nothing. In general, the hardware camera should be intialized in this method and necessary memory should be allocated.

    \note This method is similar to VideoCapture::open() of openCV

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if starting was successfull, retWarning if startDevice has been calling at least twice.
*/
ito::RetVal PGRFlyCapture::startDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = ito::retOk;
    FlyCapture2::Error retError;

    checkData(); //this will be reallocated in this method.

    incGrabberStarted();

    if(grabberStartedCount() == 1)
    {
        retError = m_myCam.StartCapture();
        if (retError != FlyCapture2::PGRERROR_OK)
        {
            retValue += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in startDevice-function: %s", retError.GetDescription());
        }
    }

    if(waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! With stopDevice the camera device is stopped (opposite to startDevice)
/*!
    In this PGRFlyCapture, this method does nothing. In general, the hardware camera should be closed in this method.

    \note This method is similar to VideoCapture::release() of openCV

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if everything is ok, retError if camera wasn't started before
    \sa startDevice
*/
ito::RetVal PGRFlyCapture::stopDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = ito::retOk;
    FlyCapture2::Error retError;

    decGrabberStarted();
    if(grabberStartedCount() == 0)
    {
        retError = m_myCam.StopCapture();
        if (retError != FlyCapture2::PGRERROR_OK)
        {
            retValue += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in stopDevice-function: %s", retError.GetDescription());
        }

    }
    else if(grabberStartedCount() < 0)
    {
        retValue += ito::RetVal(ito::retError, 1001, tr("StopDevice of PGRFlyCapture can not be executed, since camera has not been started.").toAscii().data());
        setGrabberStarted(0);
    }


    if(waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Call this method to trigger a new image.
/*!
    By this method a new image is trigger by the camera, that means the acquisition of the image starts in the moment, this method is called.
    The new image is then stored either in internal camera memory or in internal memory of this class.

    \note This method is similar to VideoCapture::grab() of openCV

    \param [in] trigger may describe the trigger parameter (unused here)
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if everything is ok, retError if camera has not been started or an older image lies in memory which has not be fetched by getVal, yet.
    \sa getVal
*/
ito::RetVal PGRFlyCapture::acquire(const int trigger, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = ito::retOk;

    if(grabberStartedCount() <= 0)
    {
        retValue += ito::RetVal(ito::retError, 1002, tr("Acquire of PGRFlyCapture can not be executed, since camera has not been started.").toAscii().data());
    }
    else
    {
		m_last_acquireTime = m_acquireTime;
        m_acquireTime = (double)(cv::getTickCount())/cv::getTickFrequency();
		m_params["timestamp"].setVal<double>((m_acquireTime-m_last_acquireTime)*1000);
        FlyCapture2::Error retError;
        this->m_isgrabbing = true;
        if(m_RunSoftwareSync)
        {
            retError = m_myCam.FireSoftwareTrigger();
            if (retError != FlyCapture2::PGRERROR_OK)
            {
                retValue += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in acquire-function: %s", retError.GetDescription());
            }
        }
        else
        {
            /*FlyCapture2::Image pImage;
            retError = m_myCam.RetrieveBuffer( &pImage );
            if (retError != FlyCapture2::PGRERROR_OK)
            {
                _snprintf(errBuff, 255, "Error in acquire-function: %s", retError.GetDescription());
                retValue += ito::RetVal(ito::retError, (int)retError.GetType(), errBuff);
            }*/

            
        }
        /*
        retError = m_myCam.( CameraStats .GetCameraInfo.GetCycleTime(&m_timeStamp);
        if (retError != FlyCapture2::PGRERROR_OK)
        {
            _snprintf(errBuff, 255, "Error in acquire-function: %s", retError.GetDescription());
            retValue += ito::RetVal(ito::retError, (int)retError.GetType(), errBuff);
        }
        */
#if EVALSPEED
        double tempTime = (double)(cv::getTickCount())/cv::getTickFrequency();
        std::cout << "\nAcquire in camera thread\t" << tempTime-m_acquireTime << "\n";
#endif
    }


    if(waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Returns the grabbed camera frame as a shallow copy.
/*!
    This method copies the recently grabbed camera frame to the given DataObject-handle

    \note This method is similar to VideoCapture::retrieve() of openCV

    \param [in,out] vpdObj is the pointer to a given dataObject (this pointer should be cast to ito::DataObject*) where the acquired image is shallow-copied to.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if everything is ok, retError is camera has not been started or no image has been acquired by the method acquire.
    \sa DataObject, acquire
*/
ito::RetVal PGRFlyCapture::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);

    ito::RetVal retValue(ito::retOk);

    retValue += retrieveData();

    if(!retValue.containsError())
    {
        if(dObj == NULL)
        {
            retValue += ito::RetVal(ito::retError, 1004, tr("data object of getVal is NULL or cast failed").toAscii().data());
        }
        else
        {
            retValue += sendDataToListeners(0); //don't wait for live image, since user should get the image as fast as possible.

            (*dObj) = this->m_data;
        }
    }

    if (waitCond)
    {
        waitCond->returnValue=retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Returns the grabbed camera frame as a deep copy.
/*!
    This method copies the recently grabbed camera frame to the given DataObject. Therefore this camera size must fit to the data structure of the
    DataObject.

    \note This method is similar to VideoCapture::retrieve() of openCV

    \param [in,out] vpdObj is the pointer to a given dataObject (this pointer should be cast to ito::DataObject*) where the acquired image is deep copied to.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if everything is ok, retError is camera has not been started or no image has been acquired by the method acquire.
    \sa DataObject, acquire
*/
ito::RetVal PGRFlyCapture::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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
        sendDataToListeners(0); //don't wait for live image, since user should get the image as fast as possible.
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PGRFlyCapture::retrieveData(ito::DataObject *externalDataObject)
{
    ito::RetVal retValue(ito::retOk);
    FlyCapture2::Error retError;

    unsigned long imglength = 0;
    long lcopysize = 0;
    long lsrcstrpos = 0;
    int y  = 0;
    int maxxsize = (int)m_params["sizex"].getMax();
    int maxysize = (int)m_params["sizey"].getMax();
    int curxsize = m_params["sizex"].getVal<int>();
    int curysize = m_params["sizey"].getVal<int>();
    int x0 = m_params["x0"].getVal<int>();
    int y0 = m_params["y0"].getVal<int>();

    bool hasListeners = false;
    bool copyExternal = false;
    if(m_autoGrabbingListeners.size() > 0)
    {
        hasListeners = true;
    }
    if(externalDataObject != NULL)
    {
        copyExternal = true;
    }
        
    if(grabberStartedCount() <= 0 || this->m_isgrabbing != true)
    {
        retValue += ito::RetVal(ito::retError, 0, "Camera not startet or triggered");
        return retValue;
    }

    FlyCapture2::Image pImage;
    
    // time to sleep

#if EVALSPEED    
    double intermTime1 = (double)(cv::getTickCount())/cv::getTickFrequency();
#endif

    double getValTime = (m_params["frame_time"].getVal<double>() - ((double)(cv::getTickCount())/cv::getTickFrequency() - m_acquireTime)) * 1000.0;
    
    if(m_RunSync)
    {
        double getValTime2 = getValTime;
        while (getValTime2 > 0.0)
        {
            getValTime2 = (m_params["frame_time"].getVal<double>() - ((double)(cv::getTickCount())/cv::getTickFrequency() - m_acquireTime)) * 1000.0;
        }
        //if(getValTime > 0.0 )
        //{
        //    Sleep( (DWORD) getValTime );
        //}
    }

    //retError = m_myCam.WaitForBufferEvent( &pImage, 0);


#if EVALSPEED    
    double intermTime2 = (double)(cv::getTickCount())/cv::getTickFrequency();
#endif

    retError = m_myCam.RetrieveBuffer( &pImage );
    
#if EVALSPEED
    double totalTime = (double)(cv::getTickCount())/cv::getTickFrequency();
#endif

    if (retError != FlyCapture2::PGRERROR_OK)
    {
        retValue += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error in retrieveData-function: %s", retError.GetDescription());
    }
    else
    {
        //int bpp = m_params["bpp"].getVal<int>();
        int bpp = pImage.GetBitsPerPixel();
        //int cols = pImage.GetCols();
        //int rows = pImage.GetRows();
        //int buffer = pImage.GetDataSize();
        //FlyCapture2::TimeStamp cur_timeStamp = pImage.GetTimeStamp();

        //if(cur_timeStamp.seconds < m_timeStamp.seconds && cur_timeStamp.microSeconds < m_timeStamp.microSeconds)
        //{
        //    retError = m_myCam.RetrieveBuffer( &pImage );
        //    if (retError != FlyCapture2::PGRERROR_OK)
        //    {
        //        _snprintf(errBuff, 255, "Error in retrieveData-function: %s", retError.GetDescription());
        //        retValue += ito::RetVal(ito::retError, (int)retError.GetType(), errBuff);
        //    }
        //}

        if(bpp <= 8)
        {
            ito::uint8 *cbuf=(ito::uint8*)pImage.GetData();
            if(cbuf == NULL)
            {
                retValue += ito::RetVal(ito::retError, 1002, tr("getVal of PGRFlyCapture failed, since retrived NULL-Pointer.").toAscii().data());
            }
            else if (curxsize == maxxsize)
            {
                lsrcstrpos = y0 * maxxsize;
                if(copyExternal) retValue += externalDataObject->copyFromData2D<ito::uint8>((ito::uint8*)cbuf+lsrcstrpos, maxxsize, curysize);
                if(!copyExternal || hasListeners) retValue += m_data.copyFromData2D<ito::uint8>((ito::uint8*)cbuf+lsrcstrpos, maxxsize, curysize);
            }
            else
            {
                if(copyExternal) retValue += externalDataObject->copyFromData2D<ito::uint8>((ito::uint8*)cbuf, maxxsize, maxysize, x0, y0, curxsize, curysize);
                if(!copyExternal || hasListeners) retValue += m_data.copyFromData2D<ito::uint8>((ito::uint8*)cbuf, maxxsize, maxysize, x0, y0, curxsize, curysize);
            }
        }
        else if(bpp <= 16)
        {
            ito::uint16 *cbuf=(ito::uint16*)pImage.GetData();
            if(cbuf == NULL)
            {
                retValue += ito::RetVal(ito::retError, 1002, tr("getVal of PGRFlyCapture failed, since retrived NULL-Pointer.").toAscii().data());
            }
            else if (curxsize == maxxsize)
            {
                lsrcstrpos = y0 * maxxsize;
                if(copyExternal) retValue += externalDataObject->copyFromData2D<ito::uint16>((ito::uint16*)cbuf+lsrcstrpos, maxxsize, curysize);
                if(!copyExternal || hasListeners) retValue += m_data.copyFromData2D<ito::uint16>((ito::uint16*)cbuf+lsrcstrpos, maxxsize, curysize);
            }
            else
            {
                if(copyExternal) retValue += externalDataObject->copyFromData2D<ito::uint16>((ito::uint16*)cbuf, maxxsize, maxysize, x0, y0, curxsize, curysize);
                if(!copyExternal || hasListeners) retValue += m_data.copyFromData2D<ito::uint16>((ito::uint16*)cbuf, maxxsize, maxysize, x0, y0, curxsize, curysize);
            }
        }
        else if(bpp <= 32)
        {
            ito::int32 *cbuf=(ito::int32*)pImage.GetData();
            if(cbuf == NULL)
            {
                retValue += ito::RetVal(ito::retError, 1002, tr("getVal of PGRFlyCapture failed, since retrived NULL-Pointer.").toAscii().data());
            }
            else if (curxsize == maxxsize)
            {
                lsrcstrpos = y0 * maxxsize;
                if(copyExternal) retValue += externalDataObject->copyFromData2D<ito::int32>((ito::int32*)cbuf+lsrcstrpos, maxxsize, curysize);
                if(!copyExternal || hasListeners) retValue += m_data.copyFromData2D<ito::int32>((ito::int32*)cbuf+lsrcstrpos, maxxsize, curysize);
            }
            else
            {
                if(copyExternal) retValue += externalDataObject->copyFromData2D<ito::int32>((ito::int32*)cbuf, maxxsize, maxysize, x0, y0, curxsize, curysize);
                if(!copyExternal || hasListeners) retValue += m_data.copyFromData2D<ito::int32>((ito::int32*)cbuf, maxxsize, maxysize, x0, y0, curxsize, curysize);
            }
        }
        else
        {
            retValue += ito::RetVal(ito::retError, 1002, tr("getVal of PGRFlyCapture failed, since undefined bitdepth.").toAscii().data());
        }
        this->m_isgrabbing = false;
    }

#if EVALSPEED
    double totalTime2 = (double)(cv::getTickCount())/cv::getTickFrequency();

    std::cout << "\nFramtime \t" << m_params["frame_time"].getVal<double>() * 1000.0 << "\n";
    std::cout << "\nAcquire to timer \t" << (intermTime1 - m_acquireTime) * 1000.0 << "\n";
    std::cout << "\nWait till end of frame \t" << (intermTime2 - intermTime1)* 1000.0 << " Shoudbe:" << getValTime << "\n";
    std::cout << "\nGrabbing from Buffer \t" << (totalTime - intermTime2) * 1000.0  << "\n";

    std::cout << "\nAcquire till got buffer \t" << (totalTime -  m_acquireTime) * 1000.0 << "\n";
    std::cout << "\nAcquire till copied buffer \t" << (totalTime2 -  m_acquireTime) * 1000.0 << "\n";
#endif

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
void PGRFlyCapture::dockWidgetVisibilityChanged(bool visible)
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

//----------------------------------------------------------------------------------------------------------------------------------
bool PGRFlyCapture::GetResolutionFromVideoMode( FlyCapture2::VideoMode mode, int &sizeX, int &sizeY)
{
    switch(mode)
    {
        case FlyCapture2::VIDEOMODE_160x120YUV444:
            sizeX = 160;
            sizeY = 120;
            break;

        case FlyCapture2::VIDEOMODE_320x240YUV422:
            sizeX = 320;
            sizeY = 240;
            break;

        case FlyCapture2::VIDEOMODE_640x480Y8:
        case FlyCapture2::VIDEOMODE_640x480Y16:
        case FlyCapture2::VIDEOMODE_640x480RGB:
        case FlyCapture2::VIDEOMODE_640x480YUV422:
        case FlyCapture2::VIDEOMODE_640x480YUV411:
            sizeX = 640;
            sizeY = 480;
            break;

        case FlyCapture2::VIDEOMODE_800x600Y8:
        case FlyCapture2::VIDEOMODE_800x600Y16:
        case FlyCapture2::VIDEOMODE_800x600RGB:
        case FlyCapture2::VIDEOMODE_800x600YUV422:     
            sizeX = 800;
            sizeY = 600;
            break;        

        case FlyCapture2::VIDEOMODE_1024x768Y8:
        case FlyCapture2::VIDEOMODE_1024x768Y16:
        case FlyCapture2::VIDEOMODE_1024x768RGB:
        case FlyCapture2::VIDEOMODE_1024x768YUV422:
            sizeX = 1024;
            sizeY = 768;
            break;

        case FlyCapture2::VIDEOMODE_1280x960Y8:
        case FlyCapture2::VIDEOMODE_1280x960Y16:
        case FlyCapture2::VIDEOMODE_1280x960RGB:
        case FlyCapture2::VIDEOMODE_1280x960YUV422:
            sizeX = 1280;
            sizeY = 960;
            break;

        case FlyCapture2::VIDEOMODE_1600x1200Y8:   
        case FlyCapture2::VIDEOMODE_1600x1200Y16:   
        case FlyCapture2::VIDEOMODE_1600x1200RGB:
        case FlyCapture2::VIDEOMODE_1600x1200YUV422:
            sizeX = 1600;
            sizeY = 1200;
            break;

        case FlyCapture2::VIDEOMODE_FORMAT7:
            sizeX = 0;
            sizeY = 0;
            return false;
        
        default:
            sizeX = 0;
            sizeY = 0;
            return false;
    }

    return false;    
}

//----------------------------------------------------------------------------------------------------------------------------------
bool PGRFlyCapture::GetPixelFormatFromVideoMode( FlyCapture2::VideoMode mode, bool stippled, FlyCapture2::PixelFormat* pixFormat)
{
    switch(mode)
    {
        case FlyCapture2::VIDEOMODE_640x480Y8:
        case FlyCapture2::VIDEOMODE_800x600Y8:
        case FlyCapture2::VIDEOMODE_1024x768Y8:
        case FlyCapture2::VIDEOMODE_1280x960Y8:
        case FlyCapture2::VIDEOMODE_1600x1200Y8:
            if( stippled )
            {
                *pixFormat = FlyCapture2::PIXEL_FORMAT_RAW8;
            }
            else
            {
                *pixFormat = FlyCapture2::PIXEL_FORMAT_MONO8;
            }
            break;
        case FlyCapture2::VIDEOMODE_640x480Y16:
        case FlyCapture2::VIDEOMODE_800x600Y16:
        case FlyCapture2::VIDEOMODE_1024x768Y16:
        case FlyCapture2::VIDEOMODE_1280x960Y16:
        case FlyCapture2::VIDEOMODE_1600x1200Y16:
            if( stippled )
            {
                *pixFormat = FlyCapture2::PIXEL_FORMAT_RAW16;
            }
            else
            {
                *pixFormat = FlyCapture2::PIXEL_FORMAT_MONO16;
            }            
            break;
        case FlyCapture2::VIDEOMODE_640x480RGB:
        case FlyCapture2::VIDEOMODE_800x600RGB:
        case FlyCapture2::VIDEOMODE_1024x768RGB:
        case FlyCapture2::VIDEOMODE_1280x960RGB:
        case FlyCapture2::VIDEOMODE_1600x1200RGB:
            *pixFormat = FlyCapture2::PIXEL_FORMAT_RGB8;
            break;

        case FlyCapture2::VIDEOMODE_320x240YUV422:
        case FlyCapture2::VIDEOMODE_640x480YUV422:
        case FlyCapture2::VIDEOMODE_800x600YUV422:       
        case FlyCapture2::VIDEOMODE_1024x768YUV422:
        case FlyCapture2::VIDEOMODE_1280x960YUV422:
        case FlyCapture2::VIDEOMODE_1600x1200YUV422:
            *pixFormat = FlyCapture2::PIXEL_FORMAT_422YUV8;
            break;      

        case FlyCapture2::VIDEOMODE_160x120YUV444:
            *pixFormat = FlyCapture2::PIXEL_FORMAT_444YUV8;
            break;

        case FlyCapture2::VIDEOMODE_640x480YUV411:
            *pixFormat = FlyCapture2::PIXEL_FORMAT_411YUV8;
            break;

        case FlyCapture2::VIDEOMODE_FORMAT7:
            return false;
        
        default:
            return false;
    }

    return false;    
}

//----------------------------------------------------------------------------------------------------------------------------------
unsigned int PGRFlyCapture::GetBppFromPixelFormat( FlyCapture2::PixelFormat pixelFormat )
{
    switch(pixelFormat)
    {
        case FlyCapture2::PIXEL_FORMAT_MONO8:
        case FlyCapture2::PIXEL_FORMAT_RAW8:
            return 8;
            break;

        case FlyCapture2::PIXEL_FORMAT_411YUV8:
        case FlyCapture2::PIXEL_FORMAT_MONO12:
        case FlyCapture2::PIXEL_FORMAT_RAW12:
            return 12;
            break;

        case FlyCapture2::PIXEL_FORMAT_MONO16:
        case FlyCapture2::PIXEL_FORMAT_S_MONO16:
        case FlyCapture2::PIXEL_FORMAT_422YUV8:
        case FlyCapture2::PIXEL_FORMAT_RAW16:
            return 16;
            break;

        case FlyCapture2::PIXEL_FORMAT_444YUV8:
        case FlyCapture2::PIXEL_FORMAT_RGB8:
        case FlyCapture2::PIXEL_FORMAT_BGR:
            return 24;
            break;

        case FlyCapture2::PIXEL_FORMAT_BGRU:
        case FlyCapture2::PIXEL_FORMAT_RGBU:
            return 32;
            break;

        case FlyCapture2::PIXEL_FORMAT_S_RGB16:
        case FlyCapture2::PIXEL_FORMAT_RGB16:
        case FlyCapture2::PIXEL_FORMAT_BGR16:
            return 48;
            break;

        default:
            return 0;
            break;
    }
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------------------
double PGRFlyCapture::GetFrameTimeFromFrameRate( FlyCapture2::FrameRate frameRate )
{
    switch(frameRate)
    {
        case FlyCapture2::FRAMERATE_1_875:
            return 1/1.875;
        case FlyCapture2::FRAMERATE_3_75:
            return 1/3.75;
        case FlyCapture2::FRAMERATE_7_5:
            return 1/7.5;
        case FlyCapture2::FRAMERATE_15:
            return 1/15;
        case FlyCapture2::FRAMERATE_30:
            return 1/30;
        case FlyCapture2::FRAMERATE_60:
            return 1/60;
        case FlyCapture2::FRAMERATE_120:
            return 1/120;
        case FlyCapture2::FRAMERATE_240:
            return 1/240;

        default:
            return 0.0;
            break;
    }
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------------------
FlyCapture2::FrameRate PGRFlyCapture::GetSuitAbleFrameRateFromFrameTime( double frameTime  )
{
    if(frameTime <= 1/240) return FlyCapture2::FRAMERATE_240;
    if(frameTime <= 1/120) return FlyCapture2::FRAMERATE_120;
    if(frameTime <= 1/60) return FlyCapture2::FRAMERATE_60;
    if(frameTime <= 1/30) return FlyCapture2::FRAMERATE_30;
    if(frameTime <= 1/15) return FlyCapture2::FRAMERATE_15;
    if(frameTime <= 1/7.5) return FlyCapture2::FRAMERATE_7_5;
    if(frameTime <= 1/3.75) return FlyCapture2::FRAMERATE_3_75;
    
    return FlyCapture2::FRAMERATE_1_875;
}
/* ********************************************************************
    Plugin "PGRFlyCapture" for itom software
    URL: http://www.twip-os.com
    Copyright (C) 2017, twip optical solutions GmbH
    Copyright (C) 2017, Institut fuer Technische Optik, Universitaet Stuttgart

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
#include <qdatetime.h>
#include "PGRFlyCapture.h"
#include "pluginVersion.h"
#include "gitVersion.h"
#define _USE_MATH_DEFINES  // needs to be defined to enable standard declartions of PI constant
#include "math.h"
#include <bitset>

#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>
#include <qmetaobject.h>
#include <qnumeric.h>

#include <qdockwidget.h>
#include <qmetaobject.h>
#include "dockWidgetPGRFlyCapture.h"

#include "common/helperCommon.h"

#include "FlyCapture2Defs.h"

static signed char InitList[MAXPGR + 1];
static char Initnum = 0;

#define EVALSPEED 0

#ifdef WIN32
#include <Windows.h>
#endif

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
    NEW_PLUGININSTANCE(PGRFlyCapture)
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
    REMOVE_PLUGININSTANCE(PGRFlyCapture)
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
    FlyCapture2::Utilities::GetLibraryVersion(&fc2Version);

    /*        char docstring[] = \
    "This plugin supports Point Grey cameras (currently USB models only) that can be run by the FlyCapture2 interface from Point Grey Research. \n\
    This plugin was mainly tested with Flea3 (monochrome) and Grasshopper3 cameras (monochrome, color) under Windows and Linux.\n\
    \n\
    The plugin has been compiled using the FlyCapture2 library version %1.%2.%3.%4. \n\
    \n\
    In order to compile the plugin by yourself, you need to install the FlyCapture2 SDK in 32bit or 64bit (depending on itom) and make \n\
    sure that the development files (include files and libraries) are installed as well (option Cross Platform Dev Files). Then set the CMake variable PGRFLYCAP_INCLUDE_DIR \n\
    to the include directory of the FlyCapture2 SDK. The depending variable PGRFYLCAP_API_DIR is then automatically set (if not, delete it and \n\
    re-configure CMake). \n\
    \n\
    This plugin automatically copies the necessary FlyCapture2 DLLs to the lib-folder of itom.";
        m_detaildescription = QObject::tr(docstring).arg(fc2Version.major).arg(fc2Version.minor).arg(fc2Version.type).arg(fc2Version.build);*/
    m_detaildescription = QObject::tr("This plugin supports Point Grey cameras (currently USB models only) that can be run by the FlyCapture2 interface from Point Grey Research. \n\
This plugin was mainly tested with Flea3 (monochrome) and Grasshopper3 cameras (monochrome, color) under Windows and Linux.\n\
\n\
The plugin has been compiled using the FlyCapture2 library version %1.%2.%3.%4. \n\
\n\
In order to compile the plugin by yourself, you need to install the FlyCapture2 SDK in 32bit or 64bit (depending on itom) and make \n\
sure that the development files (include files and libraries) are installed as well (option Cross Platform Dev Files). Then set the CMake variable PGRFLYCAP_INCLUDE_DIR \n\
to the include directory of the FlyCapture2 SDK. The depending variable PGRFYLCAP_API_DIR is then automatically set (if not, delete it and \n\
re-configure CMake). \n\
\n\
This plugin automatically copies the necessary FlyCapture2 DLLs to the lib-folder of itom.").arg(fc2Version.major).arg(fc2Version.minor).arg(fc2Version.type).arg(fc2Version.build);

    m_description = QObject::tr("Point Grey FlyCapture2 Cameras");

    m_author = "W. Lyda, M. Gronle, P. Wagner, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("GPL / this plugin needs to link agains the FlyCapture2 SDK from Point Grey Research, that comes with its own license. The FlyCapture2 SDK contains components that are licensed under GPL.");
    m_aboutThis = QObject::tr(GITVERSION);

    m_initParamsMand.clear();

    ito::Param param("cameraNumber", ito::ParamBase::Int, -1, 10, -1, \
        QObject::tr("Continuous camera number [0,10], default: -1 uses the next free camera.").toLatin1().data());
    m_initParamsOpt.append(param);
    param = ito::Param("bppLimit", ito::ParamBase::Int, -1, 16, -1, \
        QObject::tr("Limits the bitdepth to the given level [8, 12, 16]. As default the maximum level is used (-1)").toLatin1().data());
    m_initParamsOpt.append(param);
    param = ito::Param("forceSync", ito::ParamBase::Int, 0, 1, 0, \
        QObject::tr("Direct enable software sync if present").toLatin1().data());
    m_initParamsOpt.append(param);

    param = ito::Param("colorMode", ito::ParamBase::String, "gray", \
        tr("colorMode: 'gray' (default) or 'color' if color camera. In color mode, bpp is limited to 8 per color.").toLatin1().data());
    ito::StringMeta *sm = new ito::StringMeta(ito::StringMeta::String, "gray");
    sm->addItem("color");
    param.setMeta(sm, true);
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
    m_RunSoftwareSync(false),
    m_hasFormat7(false),
    m_gainMax(1.0),
    m_gainMin(0.0),
    m_offsetMax(1.0),
    m_offsetMin(0.0),
    m_extendedShutter(UNINITIALIZED),
    m_colouredOutput(false),
    m_firstTimestamp(std::numeric_limits<double>::quiet_NaN()),
    m_pendingIdleGrabs(false)
{
    ito::RetVal retVal(ito::retOk);
    //register exec functions
    QVector<ito::Param> pMand = QVector<ito::Param>();
    QVector<ito::Param> pOpt = QVector<ito::Param>();
    QVector<ito::Param> pOut = QVector<ito::Param>();
    retVal += registerExecFunc("printParameterInfo", pMand, pOpt, pOut, \
        tr("print all current parameters of the camera for internal checks."));

    pMand = QVector<ito::Param>();
    pOpt = QVector<ito::Param>();
    pOut = QVector<ito::Param>();
    pMand << ito::Param("source", ito::ParamBase::In | ito::ParamBase::Int,
        0, 0xFF, 0, tr("the GPIO pin to be edited").toLatin1().data()); //TODO check how many pins there are 
    pOpt << ito::Param("onOff", ito::ParamBase::In | ito::ParamBase::Int, 0, 1, 1,
        tr("ON or OFF this function; 0: OFF, 1 : ON").toLatin1().data());
    pOpt << ito::Param("polarity", ito::ParamBase::In | ito::ParamBase::Int, 0, 1, 0,
        tr("0 = active low; 1 = active high").toLatin1().data());
    pOpt <<  ito::Param("delay", ito::ParamBase::In | ito::ParamBase::Double, 0., 100., 0.,
        tr("delay after start of exposure until the strobe signal asserts in ms").toLatin1().data());//TOdO standard value
    pOpt << ito::Param("duration", ito::ParamBase::In | ito::ParamBase::Double, 0.0, 100., 0.,
        tr("duration of the strobe signal in ms, a value of 0 means de-assert at the end of exposure, if required").toLatin1().data());//TOdO standard value
    retVal += registerExecFunc("setStrobeMode", pMand, pOpt, pOut,
        tr("sets the strobe mode with the given parameters"));

    pMand = QVector<ito::Param>();
    pOpt = QVector<ito::Param>();
    pOut = QVector<ito::Param>();
    pMand << ito::Param("source", ito::ParamBase::In | ito::ParamBase::Int,
        0, 0xFF, 0, tr("the GPO pin to get the mode from").toLatin1().data()); //TODO check how many pins there are 
    pOut << ito::Param("onOff", ito::ParamBase::Out | ito::ParamBase::Int, 0, 1, 1,
        tr("ON or OFF this function; 0: OFF, 1 : ON").toLatin1().data());
    pOut << ito::Param("polarity", ito::ParamBase::Out | ito::ParamBase::Int, 0, 1, 0,
        tr("0 = active low; 1 = active high").toLatin1().data());
    pOut << ito::Param("delay", ito::ParamBase::Out | ito::ParamBase::Double, 0., 100., 0.,
        tr("delay after start of exposure until the strobe signal asserts in ms").toLatin1().data());//TOdO standard value
    pOut << ito::Param("duration", ito::ParamBase::Out | ito::ParamBase::Double, 0.0, 100., 0.,
        tr("duration of the strobe signal in ms, a value of 0 means de-assert at the end of exposure, if required").toLatin1().data());//TOdO standard value
    retVal += registerExecFunc("getStrobeMode", pMand, pOpt, pOut,
        tr("gets the strobe mode of the given source pin"));

    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, \
        "PGRFlyCapture", tr("name of the camera").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("integration_time", ito::ParamBase::Double, 0.000006, 1 / 1.875, 1 / 1.875, \
        tr("Integrationtime of CCD programmed in seconds.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("frame_time", \
        ito::ParamBase::Double, 1 / 240.0, 1 / 1.875, 1 / 1.875, \
        tr("Frame rate in seconds. This is only considered if the camera is not in an extended shutter mode. The frame_time might influence the integration time.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("extended_shutter", \
        ito::ParamBase::Int, 0, 1, 1, \
        tr("1 (default): extended shutter is on (long integration times are supported and frame_time becomes invalid), 0: frames are only acquired in the pulse given by frame_time.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("gain", \
        ito::ParamBase::Double, 0.0, 1.0, 0.5, tr("gain (normalized value 0..1)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("offset", \
        ito::ParamBase::Double, 0.0, 1.0, 0.05, \
        tr("offset (normalized value 0..1, mapped to PG-parameter BRIGHTNESS)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("exposure_ev", \
        ito::ParamBase::Int, 0, 1023, 480, tr("Camera brightness control (EV)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sharpness", \
        ito::ParamBase::Int, 0, 4095, 0, tr("Sharpness").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("gamma", \
        ito::ParamBase::Int, 500, 4095, 1024, tr("Gamma adjustment").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("metadata", ito::ParamBase::Int, 0, 1, 1, \
        tr("If 1 (default), the timestamp, frame counter and roi position (depending on the camera model) will be acquired and added into the first pixels of the image (available as tag of the data object as well), 0: metadata disabled").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    //binning is GigE only

    paramVal = ito::Param("sizex", ito::ParamBase::Int | ito::ParamBase::Readonly, \
        1, 2048, 2048, \
        tr("Pixelsize in x (cols)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizey", ito::ParamBase::Int | ito::ParamBase::Readonly, \
        1, 2048, 2048, \
        tr("Pixelsize in y (rows)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    int roi[] = { 0, 0, 2048, 2048 };
    paramVal = ito::Param("roi", ito::ParamBase::IntArray, 4, roi, \
        tr("region of interest, ROI (x,y,width,height)").toLatin1().data());
    ito::RectMeta *rm = new ito::RectMeta(ito::RangeMeta(0, 2047), ito::RangeMeta(0, 2047));
    paramVal.setMeta(rm, true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("color_mode", ito::ParamBase::String | ito::ParamBase::Readonly, \
        "gray", tr("colorMode: 'gray' (default) or 'color' if color camera").toLatin1().data());
    ito::StringMeta *sm = new ito::StringMeta(ito::StringMeta::String, "gray");
    sm->addItem("color");
    paramVal.setMeta(sm, true);
    m_params.insert(paramVal.getName(), paramVal);


    paramVal = ito::Param("bpp", ito::ParamBase::Int, 8, 16, 16, tr("bitdepth of each pixel").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("timeout", ito::ParamBase::Double, 0.001, 60.0, 2.0, tr("Timeout for acquiring images in seconds").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("trigger_mode", ito::ParamBase::Int, \
        - 1, 3, -1, \
        tr("-1: Complete free run, 0: enable standard external trigger (PtGrey mode 0), 1: Software Trigger (PtGrey mode 0, Software Source), 2: Bulb shutter external trigger (PtGrey mode 1), 3: Overlapped external trigger (PtGrey mode 14)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("grab_mode", ito::ParamBase::Int, \
        0, 3, 0, \
        tr("The grab strategy employed during image transfer. 0: DROP_FRAMES: Grabs the newest image in the user buffer each time the RetrieveBuffer() function is called, 1: BUFFER_FRAMES: Images accumulate in the user buffer, 2: UNSPECIFIED_GRAB_MODE, 3: GRAB_MODE_FORCE_32BITS)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("trigger_polarity", ito::ParamBase::Int, \
        0, 1, 0, \
        tr("For hardware trigger only: Set the polarity of the trigger (0: trigger active low, 1: trigger active high)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("packetsize", ito::ParamBase::Int, \
        0, 48048, 48048, \
        tr("Packet size of current image settings").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("video_mode", ito::ParamBase::Int | ito::ParamBase::Readonly, \
        0, FlyCapture2::NUM_VIDEOMODES - 1, FlyCapture2::VIDEOMODE_FORMAT7, \
        tr("Current video mode, default is Mode7").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("start_delay", ito::ParamBase::Double, \
        0.0, 2.0, 0.0, \
        tr("On some computers, a blue screen sometimes occurs if the time gap between starting the camera and acquiring an image is too short. In this case, try to increase this parameter (in s).").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("cam_serial_number", ito::ParamBase::Int | ito::ParamBase::Readonly, \
        0, tr("Serial number of the connected camera").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("cam_model", ito::ParamBase::String | ito::ParamBase::Readonly, \
        "n.a.", tr("Model identifier of the connected camera").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("cam_vendor", ito::ParamBase::String | ito::ParamBase::Readonly, \
        "n.a.", tr("Name of the camera vendor").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("cam_sensor", ito::ParamBase::String | ito::ParamBase::Readonly, \
        "n.a.", tr("Identifier of the chip in connected camera").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("cam_resolution", ito::ParamBase::String | ito::ParamBase::Readonly, \
        "n.a.", tr("Resolution of the chip in connected camera").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("cam_firmware_version", ito::ParamBase::String | ito::ParamBase::Readonly, \
        "n.a.", tr("Serial number of the firmware used in the connected camera").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("cam_firmware_build_time", ito::ParamBase::String | ito::ParamBase::Readonly, \
        "n.a.", tr("Built time of the firmware used in the connected camera").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("cam_interface", ito::ParamBase::String | ito::ParamBase::Readonly, \
        "n.a.", tr("Interface of camera").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("cam_register", ito::ParamBase::Int, \
        std::numeric_limits<int>::min(), std::numeric_limits<int>::max(), 0, \
        tr("Direct read/write of registers, use the hex-number of the register as suffix to read/write a specific register (e.g. getParam('cam_register:0xA01F'))").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("num_idle_grabs_after_param_change", ito::ParamBase::Int, \
        0, 20, 0, \
        tr("With some cameras, parameter changes like the exposure time or gain will only take effect x images after the change. If this parameter is set to > 0, the given number of images are acquired after changing any parameter in order to delete the intermediate images.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    if (hasGuiSupport())
    {
        //now create dock widget for this plugin
        DockWidgetPGRFlyCapture *dw = new DockWidgetPGRFlyCapture(this);

        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dw);
    }

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
    QMap<QString, ito::Param>::iterator it;

    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (retValue == ito::retOk)
    {
        retValue += apiGetParamFromMapByKey(m_params, key, it, false);
    }

    if (!retValue.containsError())
    {
        if (key == "cam_register")
        {
            if (suffix != "")
            {
                bool ok;
                unsigned int address = suffix.toInt(&ok, 16);
                unsigned int pValue;
                if (ok)
                {
                    retValue += checkError(m_myCam.ReadRegister(address, &pValue));
                    it->setVal<int>(pValue);
                }
                else
                {
                    retValue += ito::RetVal::format(ito::retError, 0, tr("suffix '%s' could not be interpreted as hex number").toLatin1().data(), suffix.toLatin1().data());
                }
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 0, tr("cam_register requires a suffix (hex-address in string representation, e.g. '0xA01F')").toLatin1().data());
            }
        }

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
    ParamMapIterator it;
    int running = 0;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (!retValue.containsError())
    {
        retValue += apiGetParamFromMapByKey(m_params, key, it, true);
    }

    if (!retValue.containsError())
    {
#if defined(ITOM_ADDININTERFACE_VERSION) && ITOM_ADDININTERFACE_VERSION < 0x010300
        //old style api, round the incoming double value to the allowed step size.
        //in a new itom api, this is automatically done by a new api function.
        if (val->getType() == ito::ParamBase::Double || val->getType() == ito::ParamBase::Int)
        {
            double value = val->getVal<double>();
            if (it->getType() == ito::ParamBase::Double)
            {
                ito::DoubleMeta *meta = (ito::DoubleMeta*)it->getMeta();
                if (meta)
                {
                    double step = meta->getStepSize();
                    if (step != 0.0)
                    {
                        int multiple = qRound((value - meta->getMin()) / step);
                        value = meta->getMin() + multiple * step;
                        value = qBound(meta->getMin(), value, meta->getMax());
                        val->setVal<double>(value);
                    }
                }
            }
        }
        retValue += apiValidateParam(*it, *val, false, true);
#else
        retValue += apiValidateAndCastParam(*it, *val, false, true, true);
#endif
    }

    if (!retValue.containsError())
    {
        if (key == "roi")
        {
            if (hasIndex)
            {
                const unsigned int k_imagePosition = 0xA08;
                const unsigned int k_imageSize = 0xA0C;

                unsigned int read_value = 0;
                unsigned int new_value = val->getVal<int>();
                unsigned int write_value = 0;

                int *roi = it->getVal<int*>();
                FlyCapture2::Error err;

                switch (index)
                {
                case 0: //x0

                    // Linksshift um 16 Stellen, da zur Anpassung des Offsets in x-Richtung die ersten 16 Bits des Registers,
                    // welches zusammen mit dem y-Offset 32 Bit umfasst, gesetzt werden muessen.
                    write_value = (new_value << 16) + roi[1];

                    // Direkter Schreibzugriff auf das entsprechende Kameraregister zum einstellen des x-Offsets
                    err = m_myCam.WriteRegister(k_imagePosition, write_value);
                    if (err == FlyCapture2::PGRERROR_OK)
                    {
                        roi[0] = new_value;
                    }
                    else
                    {
                        retValue += checkError(err);
                        m_myCam.ReadRegister(k_imagePosition, &read_value);
                        unsigned int val = (read_value >> 16);
                        roi[0] = val;
                    }

                    break;

                case 1: //y0
                    write_value = (roi[0] << 16) + new_value;

                    // Direkter Schreibzugriff auf das entsprechende Kameraregister zum einstellen des x-Offsets
                    err = m_myCam.WriteRegister(k_imagePosition, write_value);
                    if (err == FlyCapture2::PGRERROR_OK)
                    {
                        roi[1] = new_value;
                    }
                    else
                    {
                        retValue += checkError(err);
                        m_myCam.ReadRegister(k_imagePosition, &read_value);
                        unsigned int val = (read_value & 0xffff);
                        roi[1] = val;
                    }

                    break;

                case 2:
                    // Linksshift um 16 Stellen, da zur Anpassung der Groesse in x-Richtung die ersten 16 Bits des Registers,
                    // welches zusammen mit der Groesse in y-Richtung 32 Bit umfasst, gesetzt werden muessen.
                    write_value = (new_value << 16) + roi[3];

                    // Direkter Schreibzugriff auf das entsprechende Kameraregister zum einstellen des x-Offsets
                    err = m_myCam.WriteRegister(k_imageSize, write_value);
                    if (err == FlyCapture2::PGRERROR_OK)
                    {
                        roi[2] = new_value;
                        m_params["sizex"].setVal<int>(new_value);
                    }
                    else
                    {
                        retValue += checkError(err);
                        m_myCam.ReadRegister(k_imageSize, &read_value);
                        unsigned int val = (read_value >> 16);
                        roi[2] = val;
                        m_params["sizex"].setVal<int>(val);
                    }

                    break;

                case 3:
                    write_value = (roi[2] << 16) + new_value;

                    // Direkter Schreibzugriff auf das entsprechende Kameraregister zum einstellen des x-Offsets
                    err = m_myCam.WriteRegister(k_imageSize, write_value);
                    if (err == FlyCapture2::PGRERROR_OK)
                    {
                        roi[3] = new_value;
                        m_params["sizey"].setVal<int>(new_value);
                    }
                    else
                    {
                        retValue += checkError(err);
                        m_myCam.ReadRegister(k_imageSize, &read_value);
                        unsigned int val = (read_value & 0xffff);
                        roi[3] = val;
                        m_params["sizey"].setVal<int>(val);
                    }

                    break;

                default:
                    retValue += ito::RetVal(ito::retError, 0, tr("only indices in the range [0,3] are allowed").toLatin1().data());
                    break;
                }
            }
            else
            {
                const unsigned int k_imagePosition = 0xA08;

                unsigned int write_value = 0;

                int *old_roi = it->getVal<int*>();
                const int *new_roi = val->getVal<int*>();

                if (old_roi[2] != new_roi[2] || old_roi[3] != new_roi[3])
                {
                    //height or width changed -> full reconfiguration:
                    retValue += flyCapChangeFormat7_(false, true, -1, new_roi[0], new_roi[1], new_roi[2], new_roi[3]);
                }
                else
                {
                    write_value = (new_roi[0] << 16) + new_roi[1];

                    m_myCam.WriteRegister(k_imagePosition, write_value);

                    //overwrite old roi
                    memcpy(old_roi, new_roi, 4 * sizeof(int));
                }
            }
            bool valid;
            FlyCapture2::Format7Info packetInfos;

            m_myCam.GetFormat7Info(&packetInfos, &valid);
            m_params["packetsize"].setVal<int>(packetInfos.packetSize);
        }
        else if (key == "extended_shutter")
        {
            retValue += flyCapSetExtendedShutter(val->getVal<int>() > 0);
            retValue += flyCapSynchronizeFrameRateShutter();
            m_pendingIdleGrabs = true;
        }
        else if (key == "metadata")
        {
            bool enabled = val->getVal<int>() > 0;
            if (!checkError(m_myCam.GetEmbeddedImageInfo(&m_embeddedInfo)).containsError() && m_embeddedInfo.timestamp.available)
            {
                if (m_embeddedInfo.timestamp.available)
                {
                    m_embeddedInfo.timestamp.onOff = enabled;
                }
                if (m_embeddedInfo.gain.available)
                {
                    m_embeddedInfo.gain.onOff = false;
                }
                if (m_embeddedInfo.shutter.available)
                {
                    m_embeddedInfo.shutter.onOff = false;
                }
                if (m_embeddedInfo.brightness.available)
                {
                    m_embeddedInfo.brightness.onOff = false;
                }
                if (m_embeddedInfo.exposure.available)
                {
                    m_embeddedInfo.exposure.onOff = false;
                }
                if (m_embeddedInfo.whiteBalance.available)
                {
                    m_embeddedInfo.whiteBalance.onOff = false;
                }
                if (m_embeddedInfo.frameCounter.available)
                {
                    m_embeddedInfo.frameCounter.onOff = enabled;
                }
                if (m_embeddedInfo.strobePattern.available)
                {
                    m_embeddedInfo.strobePattern.onOff = false;
                }
                if (m_embeddedInfo.GPIOPinState.available)
                {
                    m_embeddedInfo.GPIOPinState.onOff = false;
                }
                if (m_embeddedInfo.ROIPosition.available)
                {
                    m_embeddedInfo.ROIPosition.onOff = enabled;
                }

                retValue += checkError(m_myCam.SetEmbeddedImageInfo(&m_embeddedInfo));

                if (!retValue.containsError())
                {
                    it->copyValueFrom(&(*val));
                }

                m_pendingIdleGrabs = true;
            }
        }
        else if (key == "frame_time")
        {
            FlyCapture2::Property prop;
            prop.type = FlyCapture2::FRAME_RATE;
            retValue += checkError(m_myCam.GetProperty(&prop));

            if (retValue != ito::retError && prop.onOff == true) //if off, extended shutter is on
            {
                prop.absControl = true; //floating point
                prop.autoManualMode = false; //manual
                prop.onOff = true;
                prop.absValue = 1.0 / val->getVal<double>();
                retValue += checkError(m_myCam.SetProperty(&prop));
            }

            retValue += flyCapSynchronizeFrameRateShutter();
            m_pendingIdleGrabs = true;
        }
        else if (key == "integration_time")
        {
            FlyCapture2::Property prop;
            prop.type = FlyCapture2::SHUTTER;
            retValue += checkError(m_myCam.GetProperty(&prop));

            if (retValue != ito::retError)
            {
                prop.absControl = true; //floating point
                prop.autoManualMode = false; //manual
                prop.onOff = true;
                prop.absValue = val->getVal<double>() * 1000.0;
                retValue += checkError(m_myCam.SetProperty(&prop));
            }

            retValue += flyCapSynchronizeFrameRateShutter();
            m_pendingIdleGrabs = true;
        }
        else if (key == "gain")
        {
            unsigned int value = (uint)(val->getVal<double>() * (m_gainMax - m_gainMin) + m_gainMin + 0.5);
            retValue += flyCapSetAndGetParameter("gain", value, FlyCapture2::GAIN, false, true);

            if (!retValue.containsError())
            {
                double gain = (value - m_gainMin) / (m_gainMax - m_gainMin);
                m_params["gain"].setVal<double>(gain);
            }
            m_pendingIdleGrabs = true;
        }
        else if (key == "offset")
        {
            unsigned int value = (uint)(val->getVal<double>() * (m_offsetMax - m_offsetMin) + m_offsetMin + 0.5);
            retValue += flyCapSetAndGetParameter("offset", value, FlyCapture2::BRIGHTNESS, false, true);

            if (!retValue.containsError())
            {
                double offset = (value - m_offsetMin) / (m_offsetMax - m_offsetMin);
                m_params["offset"].setVal<double>(offset);
            }
            m_pendingIdleGrabs = true;
        }
        else if (key == "gamma")
        {
            unsigned int value = (uint)(val->getVal<int>());
            retValue += flyCapSetAndGetParameter("gamma", value, FlyCapture2::GAMMA, false, true);

            if (!retValue.containsError())
            {
                m_params["gamma"].setVal<int>(value);
            }
            m_pendingIdleGrabs = true;
        }
        else if (key == "sharpness")
        {
            unsigned int value = (uint)(val->getVal<int>());
            retValue += flyCapSetAndGetParameter("sharpness", value, FlyCapture2::SHARPNESS, false, true);

            if (!retValue.containsError())
            {
                m_params["sharpness"].setVal<int>(value);
            }
            m_pendingIdleGrabs = true;
        }
        else if (key == "exposure_ev")
        {
            unsigned int value = (uint)(val->getVal<int>());
            retValue += flyCapSetAndGetParameter("exposure_ev", value, FlyCapture2::AUTO_EXPOSURE, false, true);

            if (!retValue.containsError())
            {
                m_params["exposure_ev"].setVal<int>(value);
            }
            m_pendingIdleGrabs = true;
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
            FlyCapture2::TriggerMode currentSetup;
            m_myCam.GetTriggerMode(&triggerModeSetup);

            int triggerMode = val->getVal<int>();

            switch (triggerMode)
            {
            case -1: //Free run
                m_RunSoftwareSync = false;
                triggerModeSetup.onOff = false;
                triggerModeSetup.parameter = 0; //only important for multi-acquisition (burst) mode 15
                triggerModeSetup.mode = 0;
                break;
            case 0: //Standard external hardware trigger [Mode 0, Source External]
                m_RunSoftwareSync = false;  // Force synchronisation by software trigger during aquire
                triggerModeSetup.onOff = true;
                triggerModeSetup.parameter = 0; //only important for multi-acquisition (burst) mode 15
                triggerModeSetup.mode = 0;
                triggerModeSetup.source = 0;
                break;
            case 1: //Software Trigger [Mode 0, Source Software]
                m_RunSoftwareSync = true;
                triggerModeSetup.onOff = true;
                triggerModeSetup.parameter = 0; //only important for multi-acquisition (burst) mode 15
                triggerModeSetup.mode = 0;
                triggerModeSetup.source = 7;
                break;
            case 2: //Bulb shutter external hardware trigger (Exposure duration = trigger width) [Mode 1]
                m_RunSoftwareSync = false;
                triggerModeSetup.onOff = true;
                triggerModeSetup.parameter = 0; //only important for multi-acquisition (burst) mode 15
                triggerModeSetup.mode = 1;
                triggerModeSetup.source = 0;
                break;
            case 3: //Overlapped external hardware trigger [Mode 14]
                m_RunSoftwareSync = false;
                triggerModeSetup.onOff = true;
                triggerModeSetup.parameter = 0; //only important for multi-acquisition (burst) mode 15
                triggerModeSetup.mode = 14;
                triggerModeSetup.source = 0;
                break;
            }

            FlyCapture2::Error retError = m_myCam.SetTriggerMode(&triggerModeSetup);
            if (retError != FlyCapture2::PGRERROR_OK)
            {
                retValue += ito::RetVal::format(ito::retError, (int)retError.GetType(), "Error setting trigger_mode: %s", retError.GetDescription());
            }
            else
            {
                m_params["trigger_mode"].setVal<int>(triggerMode);
            }
        }
        else if (key == "grab_mode")
        {
            FlyCapture2::FC2Config pCamConfig;
            retValue += checkError(m_myCam.GetConfiguration(&pCamConfig));

            int grabmod = val->getVal<int>();

            switch (grabmod)
            {
            case 0:
            {
                if (retValue != ito::retError)
                {
                    pCamConfig.grabMode = FlyCapture2::GrabMode::DROP_FRAMES;
                    retValue += checkError(m_myCam.SetConfiguration(&pCamConfig));


                }
                break;
            }
            case 1:
            {
                if (retValue != ito::retError)
                {
                    pCamConfig.grabMode = FlyCapture2::GrabMode::BUFFER_FRAMES;
                    retValue += checkError(m_myCam.SetConfiguration(&pCamConfig));


                }
                break;
            }
            case 2:
            {
                if (retValue != ito::retError)
                {
                    pCamConfig.grabMode = FlyCapture2::GrabMode::UNSPECIFIED_GRAB_MODE;
                    retValue += checkError(m_myCam.SetConfiguration(&pCamConfig));


                }
                break;
            }
            case 3:
            {
                if (retValue != ito::retError)
                {
                    pCamConfig.grabMode = FlyCapture2::GrabMode::GRAB_MODE_FORCE_32BITS;
                    retValue += checkError(m_myCam.SetConfiguration(&pCamConfig));


                }
                break;
            }
            }
            if (retValue != ito::retError)
            {
                retValue += it->copyValueFrom(&(*val));
            }
        }
        else if (key == "packetsize")
        {

            bool valid;
            float percentSpeed;
            unsigned int packetSize;

            if (grabberStartedCount() > 0)
            {
                running = grabberStartedCount();
                setGrabberStarted(1);
                retValue += stopDevice(NULL);
            }

            FlyCapture2::Format7Info packetInfos;
            FlyCapture2::Format7ImageSettings imageSettings;
            FlyCapture2::Format7PacketInfo packetAdvice;

            m_myCam.GetFormat7Configuration(&imageSettings, &packetInfos.packetSize, &packetInfos.percentage);
            m_myCam.GetFormat7Info(&packetInfos, &valid);

            packetSize = val->getVal<int>();
            percentSpeed = ((float)packetSize / (float)packetInfos.maxPacketSize) * 100;

            if (percentSpeed > 100.0)
            {
                percentSpeed = 100.0;
            }

            FlyCapture2::Error retError = m_myCam.SetFormat7Configuration(&imageSettings, percentSpeed);
            m_myCam.GetFormat7Info(&packetInfos, &valid);

            if (retError != FlyCapture2::PGRERROR_OK)
            {
                retValue += ito::RetVal::format(ito::retError, (int)retError.GetType(), tr("Error setting packetsize: %s").toLatin1().data(), retError.GetDescription());
            }
            else
            {
                m_params["packetsize"].setVal<int>(packetInfos.packetSize);
            }
        }
        else if (key == "trigger_polarity")
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

            triggerModeSetup.polarity = val->getVal<int>();

            FlyCapture2::Error retError = m_myCam.SetTriggerMode(&triggerModeSetup);
            if (retError != FlyCapture2::PGRERROR_OK)
            {
                retValue += ito::RetVal::format(ito::retError, (int)retError.GetType(), tr("Error setting trigger_polarity: %s").toLatin1().data(), retError.GetDescription());
            }
            else
            {
                m_params["trigger_polarity"].setVal<int>(triggerModeSetup.polarity);
            }
        }
        else if (key == "timeout")
        {
            FlyCapture2::FC2Config pCamConfig;
            retValue += checkError(m_myCam.GetConfiguration(&pCamConfig));

            if (retValue != ito::retError)
            {
                pCamConfig.grabTimeout = val->getVal<double>() * 1000;
                retValue += checkError(m_myCam.SetConfiguration(&pCamConfig));

                if (retValue != ito::retError)
                {
                    retValue += it->copyValueFrom(&(*val));
                }
            }
        }
        else if (key == "bpp")
        {
            retValue += flyCapChangeFormat7_(true, false, val->getVal<int>());
        }
        else
        {
            //all parameters that don't need further checks can simply be assigned
            //to the value in m_params (the rest is already checked above)
            retValue += it->copyValueFrom(&(*val));
        }
    }

    if (!retValue.containsError())
    {
        //restart camera if it has been started as was stopped by this method
        if (running)
        {
            retValue += startDevice(NULL);
            setGrabberStarted(running);
        }
        else if (grabberStartedCount() > 0 && m_pendingIdleGrabs)
        {
            for (int i = 0; i < m_params["num_idle_grabs_after_param_change"].getVal<int>(); ++i)
            {
                acquire(0);
            }

            m_isgrabbing = false;
            m_pendingIdleGrabs = false;
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
ito::RetVal PGRFlyCapture::flyCapSetAndGetParameter(const QString &name, unsigned int &value, FlyCapture2::PropertyType type, bool autoManualMode /*= false*/, bool onOff /*= true*/)
{
    FlyCapture2::Property prop(type);
    prop.absControl = false; //integer based
    prop.autoManualMode = autoManualMode;
    prop.onOff = onOff;
    prop.valueA = value;
    FlyCapture2::Error retErr = m_myCam.SetProperty(&prop);

    if (retErr != FlyCapture2::PGRERROR_OK)
    {
        return ito::RetVal::format(ito::retError, (int)retErr.GetType(), tr("Error setting parameter %s: %s").toLatin1().data(), name.toLatin1().data(), retErr.GetDescription());
    }

    retErr = m_myCam.GetProperty(&prop);
    if (retErr == FlyCapture2::PGRERROR_OK)
    {
        value = prop.valueA;
    }
    else
    {
        return ito::RetVal::format(ito::retError, (int)retErr.GetType(), tr("Error getting parameter %s: %s").toLatin1().data(), name.toLatin1().data(), retErr.GetDescription());
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PGRFlyCapture::flyCapSetAndGetParameter(const QString &name, float &value, FlyCapture2::PropertyType type, bool autoManualMode /*= false*/, bool onOff /*= true*/)
{
    FlyCapture2::Property prop(type);
    prop.absControl = true; //float based
    prop.autoManualMode = autoManualMode;
    prop.onOff = onOff;
    prop.absValue = value;
    FlyCapture2::Error retErr = m_myCam.SetProperty(&prop);

    if (retErr != FlyCapture2::PGRERROR_OK)
    {
        return ito::RetVal::format(ito::retError, (int)retErr.GetType(), tr("Error setting parameter %s: %s").toLatin1().data(), name.toLatin1().data(), retErr.GetDescription());
    }

    retErr = m_myCam.GetProperty(&prop);
    if (retErr == FlyCapture2::PGRERROR_OK)
    {
        value = prop.absValue;
    }
    else
    {
        return ito::RetVal::format(ito::retError, (int)retErr.GetType(), tr("Error getting parameter %s: %s").toLatin1().data(), name.toLatin1().data(), retErr.GetDescription());
    }

    return ito::retOk;
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PGRFlyCapture::flyCapGetParameter(const QString &name, unsigned int &value, FlyCapture2::PropertyType type)
{
    FlyCapture2::Property prop(type);

    FlyCapture2::Error retErr = m_myCam.GetProperty(&prop);
    if (retErr == FlyCapture2::PGRERROR_OK)
    {
        value = prop.valueA;
    }
    else
    {
        return ito::RetVal::format(ito::retError, (int)retErr.GetType(), tr("Error getting parameter %s: %s").toLatin1().data(), name.toLatin1().data(), retErr.GetDescription());
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PGRFlyCapture::flyCapGetParameter(const QString &name, float &value, FlyCapture2::PropertyType type)
{
    FlyCapture2::Property prop(type);

    FlyCapture2::Error retErr = m_myCam.GetProperty(&prop);
    if (retErr == FlyCapture2::PGRERROR_OK)
    {
        value = prop.absValue;
    }
    else
    {
        return ito::RetVal::format(ito::retError, (int)retErr.GetType(), tr("Error getting parameter %s: %s").toLatin1().data(), name.toLatin1().data(), retErr.GetDescription());
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
    int maxBpp = 0;
    int minBpp = 1000;
    unsigned int numberOfCam = 0;
    ito::RetVal retVal;

    m_camIdx = (*paramsOpt)[0].getVal<int>();    // the first parameter in optional list is for the camera index
    int limitBPP = (*paramsOpt)[1].getVal<int>();    // the second parameter in optional list is for the max BPP
    bool startSyncronized = (*paramsOpt)[2].getVal<int>();    // the third parameter in optional list handels synchronisation
    QString colorMode = paramsOpt->at(3).getVal<char*>();

    Initnum++;

    retVal += checkError(busMgr.GetNumOfCameras(&numberOfCam));

    if (m_camIdx < 0)
    {
        bool found;
        signed char curCamIdx;

        //take the first free camera
        for (curCamIdx = 0; curCamIdx < (signed char)numberOfCam; curCamIdx++)
        {
            //check whether this camera is not used yet
            found = false;
            for (int i = 0; i < MAXPGR; i++)
            {
                if (InitList[i] == curCamIdx)
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
        if (m_camIdx < 0)
        {
            retVal += ito::RetVal(ito::retError, 0, tr("No free camera found.").toLatin1().data());
        }
    }
    else
    {
        if (m_camIdx < numberOfCam)
        {
            for (int i = 0; i < MAXPGR; i++)
            {
                if (InitList[i] == m_camIdx)
                {
                    m_camIdx = -1;
                    retVal += ito::RetVal(ito::retError, 0, tr("Camera already initialized").toLatin1().data());
                }
            }
        }
        else
        {
            m_camIdx = -1;
            retVal += ito::RetVal(ito::retError, 0, tr("Camera index out of range").toLatin1().data());
        }
    }

    if (!retVal.containsError())
    {
        retVal += checkError(busMgr.GetCameraFromIndex(m_camIdx, &m_myGUID));
        if (retVal.containsError())
        {
            m_camIdx = -1;
            retVal += ito::RetVal::format(ito::retError, (int)retError.GetType(), tr("Error in init-function: %s").toLatin1().data(), retError.GetDescription());
        }
        else
        {
            for (int i = 0; i < MAXPGR; i++)
            {
                if (InitList[i] == -1)
                {
                    InitList[i] = m_camIdx;
                    break;
                }
            }

            FlyCapture2::InterfaceType m_interfaceType;
            busMgr.GetInterfaceTypeFromGuid(&m_myGUID, &m_interfaceType);

            switch (m_interfaceType)
            {
            case FlyCapture2::INTERFACE_USB2:
                m_params["cam_interface"].setVal<const char*>("USB2");
                break;
            case FlyCapture2::INTERFACE_USB3:
                m_params["cam_interface"].setVal<const char*>("USB3");
                break;
            default:
                retVal += ito::RetVal(ito::retError, 0, tr("unsupported interface (GigE, IEEE1394...)").toLatin1().data());
                break;
            }
        }
    }

    if (!retVal.containsError())
    {
        retVal += checkError(m_myCam.Connect(&m_myGUID));
    }

    if (!retVal.containsError())
    {
        retVal += checkError(m_myCam.GetCameraInfo(&camInfo));
        if (!retVal.containsError())
        {
            m_colorCam = camInfo.isColorCamera;

            if (!m_colorCam && QString::compare(colorMode, "gray", Qt::CaseInsensitive) != 0)
            {
                retVal += ito::RetVal(ito::retError, 0, tr("parameter 'colorMode' must be 'gray' if a monochrome camera is connected").toLatin1().data());
            }
            else if (QString::compare(colorMode, "gray", Qt::CaseInsensitive) == 0)
            {
                m_params["color_mode"].setVal<const char*>("gray");
                m_colouredOutput = false;
            }
            else
            {
                m_params["color_mode"].setVal<const char*>("color");
                m_colouredOutput = true;
            }
        }
    }

    m_format7Info.mode = FlyCapture2::MODE_0;

    if (!retVal.containsError())
    {
        retVal += checkError(m_myCam.GetFormat7Info(&m_format7Info, &m_hasFormat7));
        retVal += checkError(m_myCam.GetVideoModeAndFrameRate(&videoMode, &frameRate));
    }

    if (!retVal.containsError())
    {
        if (m_hasFormat7)    // camera has mode mode 7
        {
            if (videoMode != FlyCapture2::VIDEOMODE_FORMAT7)
            {
                retError = m_myCam.SetVideoModeAndFrameRate(FlyCapture2::VIDEOMODE_FORMAT7, FlyCapture2::FRAMERATE_FORMAT7);
                if (retError != FlyCapture2::PGRERROR_OK)
                {
                    retVal += ito::RetVal::format(ito::retError, (int)retError.GetType(), tr("Error in init-function: %s").toLatin1().data(), retError.GetDescription());
                }
            }

            if (!retVal.containsError())
            {
                FlyCapture2::Format7Info packetInfos;

                bool valid;
                float percentspeed;

                //get defaults from info struct
                ito::IntMeta *im;
                im = static_cast<ito::IntMeta*>(m_params["sizex"].getMeta());
                im->setMax(m_format7Info.maxWidth);
                im->setStepSize(m_format7Info.imageHStepSize);

                im = static_cast<ito::IntMeta*>(m_params["sizey"].getMeta());
                im->setMax(m_format7Info.maxHeight);
                im->setStepSize(m_format7Info.imageVStepSize);

                m_myCam.GetFormat7Info(&packetInfos, &valid);

                percentspeed = packetInfos.percentage;
                m_params["packetsize"].setVal<int>(packetInfos.packetSize);

                retVal += checkError(m_myCam.GetFormat7Configuration(&m_currentFormat7Settings, &packetInfos.packetSize, &packetInfos.percentage));
                im = (ito::IntMeta*)m_params["packetsize"].getMeta();
                im->setMin(packetInfos.minPacketSize);
                im->setMax(packetInfos.maxPacketSize);

                //set bpp
                int desiredBpp;
                m_params["video_mode"].setVal<int>(FlyCapture2::VIDEOMODE_FORMAT7);

                if (!retVal.containsError() && m_currentFormat7Settings.mode != FlyCapture2::MODE_0)
                {
                    m_currentFormat7Settings.mode = FlyCapture2::MODE_0;
                    retVal += checkError(m_myCam.SetFormat7Configuration(&m_currentFormat7Settings, percentspeed));
                }

                if (m_colouredOutput)
                {
                    if (FlyCapture2::PIXEL_FORMAT_BGRU & m_format7Info.pixelFormatBitField)
                    {
                        minBpp = std::min(minBpp, 8);
                        maxBpp = std::max(maxBpp, 8);
                    }

                    if ((FlyCapture2::PIXEL_FORMAT_BGRU & m_format7Info.pixelFormatBitField) == 0)
                    {
                        retVal += ito::RetVal(ito::retError, 0, tr("colorMode 'rgb' requested, but the camera does not support one of the implemented formats RGB").toLatin1().data());
                    }
                }
                else
                {
                    if ((FlyCapture2::PIXEL_FORMAT_MONO8 | FlyCapture2::PIXEL_FORMAT_RAW8) & m_format7Info.pixelFormatBitField)
                    {
                        minBpp = std::min(minBpp, 8);
                        maxBpp = std::max(maxBpp, 8);
                    }

                    if ((FlyCapture2::PIXEL_FORMAT_MONO12 | FlyCapture2::PIXEL_FORMAT_RAW12) & m_format7Info.pixelFormatBitField)
                    {
                        minBpp = std::min(minBpp, 12);
                        maxBpp = std::max(maxBpp, 12);
                    }

                    if ((FlyCapture2::PIXEL_FORMAT_MONO16 | FlyCapture2::PIXEL_FORMAT_RAW16) & m_format7Info.pixelFormatBitField)
                    {
                        minBpp = std::min(minBpp, 16);
                        maxBpp = std::max(maxBpp, 16);
                    }
                }

                if (limitBPP <= 8 || maxBpp <= 8)
                {
                    desiredBpp = 8;
                }
                else if (limitBPP <= 12)
                {
                    desiredBpp = 12;
                }
                else if (limitBPP <= 16 || maxBpp <= 16)
                {
                    desiredBpp = 16;
                }

                if (!retVal.containsError())
                {
                    flyCapChangeFormat7_(true, false, desiredBpp);
                    pixelFormat = m_currentFormat7Settings.pixelFormat;
                }
            }
        }
        else
        {
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
                retVal += ito::RetVal(ito::retError, 0, tr("Get resolution failed").toLatin1().data());
            }
            else
            {
                m_params["video_mode"].setVal<int>(videoMode);

                m_params["sizex"].setVal<int>(width);
                static_cast<ito::IntMeta*>(m_params["sizex"].getMeta())->setMax(width);

                m_params["sizey"].setVal<int>(height);
                static_cast<ito::IntMeta*>(m_params["sizey"].getMeta())->setMax(height);
            }

            if (!GetPixelFormatFromVideoMode(videoMode, isStippled, &pixelFormat))
            {
                pixelFormat = FlyCapture2::PIXEL_FORMAT_RAW8;
            }
        }
    }

    if (!retVal.containsError())
    {
        retVal += flyCapSetExtendedShutter(m_params["extended_shutter"].getVal<int>() > 0);
    }

    if (!retVal.containsError())
    {
        retVal += checkError(m_myCam.GetEmbeddedImageInfo(&m_embeddedInfo));
        if (!retVal.containsError() && m_embeddedInfo.timestamp.available)
        {
            if (m_embeddedInfo.timestamp.available)
            {
                m_embeddedInfo.timestamp.onOff = true;
            }
            if (m_embeddedInfo.gain.available)
            {
                m_embeddedInfo.gain.onOff = false;
            }
            if (m_embeddedInfo.shutter.available)
            {
                m_embeddedInfo.shutter.onOff = false;
            }
            if (m_embeddedInfo.brightness.available)
            {
                m_embeddedInfo.brightness.onOff = false;
            }
            if (m_embeddedInfo.exposure.available)
            {
                m_embeddedInfo.exposure.onOff = false;
            }
            if (m_embeddedInfo.whiteBalance.available)
            {
                m_embeddedInfo.whiteBalance.onOff = false;
            }
            if (m_embeddedInfo.frameCounter.available)
            {
                m_embeddedInfo.frameCounter.onOff = true;
            }
            if (m_embeddedInfo.strobePattern.available)
            {
                m_embeddedInfo.strobePattern.onOff = false;
            }
            if (m_embeddedInfo.GPIOPinState.available)
            {
                m_embeddedInfo.GPIOPinState.onOff = false;
            }
            if (m_embeddedInfo.ROIPosition.available)
            {
                m_embeddedInfo.ROIPosition.onOff = true;
            }

            retVal += checkError(m_myCam.SetEmbeddedImageInfo(&m_embeddedInfo));

            m_hasFrameInfo = false;
            const unsigned int k_frameInfoReg = 0x12F8;
            unsigned int frameInfoRegVal = 0;
            if (m_myCam.ReadRegister(k_frameInfoReg, &frameInfoRegVal) == FlyCapture2::PGRERROR_OK && (frameInfoRegVal >> 31) != 0)
            {
                m_hasFrameInfo = true;
                m_params["metadata"].setVal<int>(1);
                m_params["metadata"].setFlags(0);
            }
            else
            {
                m_params["metadata"].setVal<int>(0);
                m_params["metadata"].setFlags(ito::ParamBase::Readonly);
            }
        }
    }

    if (!retVal.containsError())
    {
        FlyCapture2::PropertyInfo propInfo;
        propInfo.type = FlyCapture2::SHUTTER;

        retVal += checkError(m_myCam.GetPropertyInfo(&propInfo));

        if (!retVal.containsError())
        {
            if (propInfo.present && propInfo.absValSupported && propInfo.manualSupported)
            {
                FlyCapture2::Property prop;
                prop.type = FlyCapture2::SHUTTER;
                m_myCam.GetProperty(&prop);
                prop.absControl = true;
                prop.autoManualMode = false;
                prop.onOff = true;
                retVal += checkError(m_myCam.SetProperty(&prop));
            }

            retVal += flyCapSynchronizeFrameRateShutter();
        }
    }

    if (!retVal.containsError())
    {
        FlyCapture2::PropertyInfo propInfo;

        propInfo.type = FlyCapture2::GAIN;
        retVal += checkError(m_myCam.GetPropertyInfo(&propInfo));

        if (!retVal.containsError())
        {
            if (propInfo.present && propInfo.manualSupported)
            {
                m_gainMax = propInfo.max;
                m_gainMin = propInfo.min;

                unsigned int valA;
                retVal += flyCapGetParameter("gain", valA, FlyCapture2::GAIN);

                if (!retVal.containsError())
                {
                    double gain = (valA - m_gainMin) / (m_gainMax - m_gainMin);
                    m_params["gain"].setVal<double>(gain);

                    retVal += flyCapSetAndGetParameter("gain", valA, FlyCapture2::GAIN, false, true);
                }
            }
            else
            {
                m_params["gain"].setFlags(ito::ParamBase::Readonly);
            }
        }
    }

    if (!retVal.containsError())
    {
        FlyCapture2::PropertyInfo propInfo;

        propInfo.type = FlyCapture2::BRIGHTNESS;
        retVal += checkError(m_myCam.GetPropertyInfo(&propInfo));

        if (!retVal.containsError())
        {
            if (propInfo.present && propInfo.manualSupported)
            {
                m_offsetMax = propInfo.max;
                m_offsetMin = propInfo.min;

                unsigned int valA;
                retVal += flyCapGetParameter("offset", valA, FlyCapture2::BRIGHTNESS);

                if (!retVal.containsError())
                {
                    double offset = (valA - m_offsetMin) / (m_offsetMax - m_offsetMin);
                    m_params["offset"].setVal<double>(offset);

                    retVal += flyCapSetAndGetParameter("offset", valA, FlyCapture2::BRIGHTNESS, false, true);
                }
            }
            else
            {
                m_params["offset"].setFlags(ito::ParamBase::Readonly);
            }
        }
    }

    if (!retVal.containsError())
    {

        FlyCapture2::PropertyInfo propInfo;

        propInfo.type = FlyCapture2::GAMMA;
        retVal += checkError(m_myCam.GetPropertyInfo(&propInfo));

        if (!retVal.containsError())
        {
            if (propInfo.present && propInfo.manualSupported)
            {

                FlyCapture2::Property prop;
                prop.type = FlyCapture2::GAMMA;

                unsigned int valA;
                retVal += flyCapGetParameter("gamma", valA, FlyCapture2::GAMMA);

                if (!retVal.containsError())
                {
                    m_params["gamma"].setVal<int>(valA);
                    m_params["gamma"].setMeta(new ito::IntMeta((int)propInfo.min, (int)propInfo.max), true);
                    retVal += flyCapSetAndGetParameter("gamma", valA, FlyCapture2::GAMMA, false, true);
                }
            }
            else
            {
                m_params["gamma"].setFlags(ito::ParamBase::Readonly);
            }
        }
    }

    if (!retVal.containsError())
    {

        FlyCapture2::PropertyInfo propInfo;

        propInfo.type = FlyCapture2::SHARPNESS;
        retVal += checkError(m_myCam.GetPropertyInfo(&propInfo));

        if (!retVal.containsError())
        {
            if (propInfo.present && propInfo.manualSupported)
            {

                FlyCapture2::Property prop;
                prop.type = FlyCapture2::SHARPNESS;

                unsigned int valA;
                retVal += flyCapGetParameter("sharpness", valA, FlyCapture2::SHARPNESS);

                if (!retVal.containsError())
                {
                    m_params["sharpness"].setVal<int>(valA);
                    m_params["sharpness"].setMeta(new ito::IntMeta((int)propInfo.min, (int)propInfo.max), true);
                    retVal += flyCapSetAndGetParameter("sharpness", valA, FlyCapture2::SHARPNESS, false, true);
                }
            }
            else
            {
                m_params["sharpness"].setFlags(ito::ParamBase::Readonly);
            }
        }
    }

    if (!retVal.containsError())
    {

        FlyCapture2::PropertyInfo propInfo;

        propInfo.type = FlyCapture2::AUTO_EXPOSURE;
        retVal += checkError(m_myCam.GetPropertyInfo(&propInfo));

        if (!retVal.containsError())
        {
            if (propInfo.present && propInfo.manualSupported)
            {

                FlyCapture2::Property prop;
                prop.type = FlyCapture2::AUTO_EXPOSURE;

                unsigned int valA;
                retVal += flyCapGetParameter("exposure_ev", valA, FlyCapture2::AUTO_EXPOSURE);

                if (!retVal.containsError())
                {
                    m_params["exposure_ev"].setVal<int>(valA);
                    m_params["exposure_ev"].setMeta(new ito::IntMeta((int)propInfo.min, (int)propInfo.max), true);
                    retVal += flyCapSetAndGetParameter("exposure_ev", valA, FlyCapture2::AUTO_EXPOSURE, false, true);
                }
            }
            else
            {
                m_params["exposure_ev"].setFlags(ito::ParamBase::Readonly);
            }
        }
    }

    //set the camera configuration
    if (!retVal.containsError())
    {
        retError = m_myCam.GetConfiguration(&pCamConfig);
        pCamConfig.grabTimeout = m_params["timeout"].getVal<double>() * 1000;
        retVal += checkError(m_myCam.SetConfiguration(&pCamConfig));
    }

    if (!retVal.containsError())
    {
        FlyCapture2::TriggerModeInfo pTriggerInfo;
        FlyCapture2::TriggerMode triggerModeSetup;

        m_myCam.GetTriggerModeInfo(&pTriggerInfo);
        m_myCam.GetTriggerMode(&triggerModeSetup);

        m_RunSoftwareSync = false;

        if (pTriggerInfo.present && pTriggerInfo.polaritySupported)
        {
            m_params["trigger_polarity"].setVal<int>(triggerModeSetup.polarity);
        }
        else
        {
            m_params["trigger_polarity"].setFlags(ito::ParamBase::Readonly);
        }

        //set default trigger to -1: freerun
        setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("trigger_mode", ito::ParamBase::Int, -1)), NULL);

        if (pTriggerInfo.present && pTriggerInfo.softwareTriggerSupported)
        {
            if (pTriggerInfo.softwareTriggerSupported)
            {
                //m_params["trigger_mode"].setMax(3); 
                static_cast<ito::IntMeta*>(m_params["trigger_mode"].getMeta())->setMax(3);
            }
            else
            {
                //m_params["trigger_mode"].setMax(1);
                static_cast<ito::IntMeta*>(m_params["trigger_mode"].getMeta())->setMax(1);
            }
        }
        else
        {
            m_params["trigger_mode"].setVal<int>(0);
            m_params["trigger_mode"].setFlags(ito::ParamBase::Readonly);
            //m_params["trigger_mode"].setMax(0);
            static_cast<ito::IntMeta*>(m_params["trigger_mode"].getMeta())->setMax(0);
        }
    }

    if (!retVal.containsError())
    {
        int curBpp = GetBppFromPixelFormat(pixelFormat);
        if (curBpp < 8)
        {
            retVal += ito::RetVal(ito::retError, 0, tr("Get bits per pixel failed").toLatin1().data());
        }
        else
        {
            m_params["bpp"].setVal<int>(curBpp);
            m_params["bpp"].setMeta(new ito::IntMeta(minBpp, maxBpp, 4), true);
        }
    }

    if (!retVal.containsError())
    {
        m_params["cam_serial_number"].setVal<int>((int)camInfo.serialNumber);
        setIdentifier(QString("%1 (%2)").arg(camInfo.modelName).arg(camInfo.serialNumber));
        m_params["cam_model"].setVal<char*>(camInfo.modelName, (int)strlen(camInfo.modelName));
        m_params["cam_vendor"].setVal<char*>(camInfo.vendorName, (int)strlen(camInfo.vendorName));
        m_params["cam_sensor"].setVal<char*>(camInfo.sensorInfo, (int)strlen(camInfo.sensorInfo));
        m_params["cam_resolution"].setVal<char*>(camInfo.sensorResolution, (int)strlen(camInfo.sensorResolution));
        m_params["cam_firmware_version"].setVal<char*>(camInfo.firmwareVersion, (int)strlen(camInfo.firmwareVersion));
        m_params["cam_firmware_build_time"].setVal<char*>(camInfo.firmwareBuildTime, (int)strlen(camInfo.firmwareBuildTime));
    }

    if (!retVal.containsError())
    {
        checkData(); //check if image must be reallocated
    }

    if (waitCond)
    {
        waitCond->returnValue = retVal;
        waitCond->release();
    }

    setInitialized(true); //init method has been finished (independent on retval)
    return retVal;
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
    ito::RetVal retValue(ito::retOk, 0, "");

    if (m_camIdx > -1)
    {
        //stop camera if started
        if (grabberStartedCount() > 0)
        {
            setGrabberStarted(1);
            retValue += stopDevice(NULL);
        }

        retError = m_myCam.Disconnect();
        if (retError != FlyCapture2::PGRERROR_OK)
        {
            retValue += ito::RetVal::format(ito::retError, (int)retError.GetType(), tr("Error in close-function: %s").toLatin1().data(), retError.GetDescription());
        }

        for (int i = 0; i < MAXPGR; i++)
        {
            if (InitList[i] == m_camIdx)
            {
                InitList[i] = -1;
                break;
            }
        }
    }
    Initnum--;

    if (Initnum < 1)
    {
        for (int i = 0; i < MAXPGR; i++)
        {
            if (InitList[i] > -1)
            {
                retValue += ito::RetVal(ito::retWarning, 0, tr("Found some uninitilized but still present grabber").toLatin1().data());
            }
        }
    }

    if (m_timerID > 0)
    {
        killTimer(m_timerID);
        m_timerID = 0;
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
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

    if (grabberStartedCount() == 1)
    {
        if (m_params["start_delay"].getVal<double>() > 0)
        {
            Sleep(m_params["start_delay"].getVal<double>() * 1000.0);
            m_firstTimestamp = std::numeric_limits<double>::quiet_NaN(); //reset timer
            retError = m_myCam.StartCapture();
            Sleep(m_params["start_delay"].getVal<double>() * 1000.0);
        }
        else
        {
            m_firstTimestamp = std::numeric_limits<double>::quiet_NaN(); //reset timer
            retError = m_myCam.StartCapture();
        }
        if (retError != FlyCapture2::PGRERROR_OK)
        {
            retValue += ito::RetVal::format(ito::retError, (int)retError.GetType(), tr("Error in startDevice-function: %s").toLatin1().data(), retError.GetDescription());
        }
    }

    if (grabberStartedCount() > 0 && m_pendingIdleGrabs)
    {
        for (int i = 0; i < m_params["num_idle_grabs_after_param_change"].getVal<int>(); ++i)
        {
            acquire(0);
        }

        m_isgrabbing = false;
        m_pendingIdleGrabs = false;
    }

    if (waitCond)
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

    if (grabberStartedCount() == 0)
    {
#if QT_VERSION >= 0x050200
        QThread::msleep(100); //prevents possible crashes due to a heap allocation error when stopping the camera too early
#else
        Sleep(100); //prevents possible crashes due to a heap allocation error when stopping the camera too early
#endif
        retValue += checkError(m_myCam.StopCapture());
    }
    else if (grabberStartedCount() < 0)
    {
        retValue += ito::RetVal(ito::retError, 1001, tr("StopDevice of PGRFlyCapture can not be executed, since camera has not been started.").toLatin1().data());
        setGrabberStarted(0);
    }

    if (waitCond)
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

    if (grabberStartedCount() <= 0)
    {
        retValue += ito::RetVal(ito::retError, 1002, tr("Acquire of PGRFlyCapture can not be executed, since camera has not been started.").toLatin1().data());
        m_acquisitionStatus = ito::retOk;

        if (waitCond)
        {
            waitCond->returnValue = retValue;
            waitCond->release();
        }
    }
    else
    {
        this->m_isgrabbing = true;
        if (m_RunSoftwareSync)
        {
            retValue += checkError(m_myCam.FireSoftwareTrigger());
        }

        if (waitCond)
        {
            waitCond->returnValue = retValue;
            waitCond->release();
        }

        if (!retValue.containsError())
        {
            m_acquisitionStatus = checkError(m_myCam.RetrieveBuffer(&m_imageBuffer));

            if (m_acquisitionStatus.containsError() && m_acquisitionStatus.errorCode() == FlyCapture2::PGRERROR_IMAGE_CONSISTENCY_ERROR)
            {
                //try one more time
                m_acquisitionStatus = checkError(m_myCam.RetrieveBuffer(&m_imageBuffer));
            }
        }
        else
        {
            m_acquisitionStatus = ito::retOk;
        }
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

    if (!retValue.containsError())
    {
        if (dObj == NULL)
        {
            retValue += ito::RetVal(ito::retError, 1004, tr("data object of getVal is NULL or cast failed").toLatin1().data());
        }
        else
        {
            retValue += sendDataToListeners(0); //don't wait for live image, since user should get the image as fast as possible.

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

    if (!dObj)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Empty object handle retrieved from caller").toLatin1().data());
    }
    else
    {
        retValue += checkData(dObj);
    }

    if (!retValue.containsError())
    {
        retValue += retrieveData(dObj);
    }

    if (!retValue.containsError())
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
    ito::RetVal retValue = m_acquisitionStatus;

    bool hasListeners = false;
    bool copyExternal = false;
    if (m_autoGrabbingListeners.size() > 0)
    {
        hasListeners = true;
    }
    if (externalDataObject != NULL)
    {
        copyExternal = true;
    }

    //if (grabberStartedCount() <= 0 || m_isgrabbing != true)
    //{
    //    retValue += ito::RetVal(ito::retError, 0, tr("Camera not started or triggered").toLatin1().data());
    //    return retValue;
    //}

    FlyCapture2::Image convertedImage;

    if (!retValue.containsError())
    {
        int bpp = m_imageBuffer.GetBitsPerPixel();
        FlyCapture2::PixelFormat format = m_imageBuffer.GetPixelFormat();

        if (bpp <= 8 && !m_colouredOutput)
        {
            if (format != FlyCapture2::PIXEL_FORMAT_MONO8)
            {
                retValue += checkError(m_imageBuffer.Convert(FlyCapture2::PIXEL_FORMAT_MONO8, &convertedImage));

                if (!retValue.containsError())
                {
                    ito::uint8 *cbuf = (ito::uint8*)convertedImage.GetData();
                    if (cbuf == NULL)
                    {
                        retValue += ito::RetVal(ito::retError, 1002, tr("getVal of PGRFlyCapture failed, since retrived NULL-Pointer.").toLatin1().data());
                    }
                    else
                    {
                        if (copyExternal) retValue += externalDataObject->copyFromData2D<ito::uint8>(cbuf, convertedImage.GetCols(), convertedImage.GetRows());
                        if (!copyExternal || hasListeners) retValue += m_data.copyFromData2D<ito::uint8>(cbuf, convertedImage.GetCols(), convertedImage.GetRows());
                    }
                }
            }
            else
            {
                ito::uint8 *cbuf = (ito::uint8*)m_imageBuffer.GetData();
                if (cbuf == NULL)
                {
                    retValue += ito::RetVal(ito::retError, 1002, tr("getVal of PGRFlyCapture failed, since retrived NULL-Pointer.").toLatin1().data());
                }
                else
                {
                    if (copyExternal) retValue += externalDataObject->copyFromData2D<ito::uint8>(cbuf, m_imageBuffer.GetCols(), m_imageBuffer.GetRows());
                    if (!copyExternal || hasListeners) retValue += m_data.copyFromData2D<ito::uint8>(cbuf, m_imageBuffer.GetCols(), m_imageBuffer.GetRows());
                }
            }
        }
        else if (bpp <= 8 && m_colouredOutput)
        {
            if (format != FlyCapture2::PIXEL_FORMAT_BGRU)
            {
                retValue += checkError(m_imageBuffer.Convert(FlyCapture2::PIXEL_FORMAT_BGRU, &convertedImage));

                if (!retValue.containsError())
                {
                    ito::Rgba32 *cbuf = (ito::Rgba32*)convertedImage.GetData();
                    if (cbuf == NULL)
                    {
                        retValue += ito::RetVal(ito::retError, 1002, tr("getVal of PGRFlyCapture failed, since retrived NULL-Pointer.").toLatin1().data());
                    }
                    else
                    {
                        if (copyExternal) retValue += externalDataObject->copyFromData2D<ito::Rgba32>(cbuf, convertedImage.GetCols(), convertedImage.GetRows());
                        if (!copyExternal || hasListeners) retValue += m_data.copyFromData2D<ito::Rgba32>(cbuf, convertedImage.GetCols(), convertedImage.GetRows());
                    }
                }
            }
            else
            {
                ito::Rgba32 *cbuf = (ito::Rgba32*)m_imageBuffer.GetData();
                if (cbuf == NULL)
                {
                    retValue += ito::RetVal(ito::retError, 1002, tr("getVal of PGRFlyCapture failed, since retrived NULL-Pointer.").toLatin1().data());
                }
                else
                {
                    if (copyExternal) retValue += externalDataObject->copyFromData2D<ito::Rgba32>(cbuf, m_imageBuffer.GetCols(), m_imageBuffer.GetRows());
                    if (!copyExternal || hasListeners) retValue += m_data.copyFromData2D<ito::Rgba32>(cbuf, m_imageBuffer.GetCols(), m_imageBuffer.GetRows());
                }
            }
        }
        else if (bpp <= 12)
        {
            if (format != FlyCapture2::PIXEL_FORMAT_MONO16)
            {
                retValue += checkError(m_imageBuffer.Convert(FlyCapture2::PIXEL_FORMAT_MONO16, &convertedImage));

                if (!retValue.containsError())
                {
                    ito::uint16 *cbuf = (ito::uint16*)convertedImage.GetData();
                    if (cbuf == NULL)
                    {
                        retValue += ito::RetVal(ito::retError, 1002, tr("getVal of PGRFlyCapture failed, since retrived NULL-Pointer.").toLatin1().data());
                    }
                    else
                    {
                        if (copyExternal)
                        {
                            retValue += externalDataObject->copyFromData2D<ito::uint16>(cbuf, convertedImage.GetCols(), convertedImage.GetRows());
                            *externalDataObject >>= 4;
                        }
                        if (!copyExternal || hasListeners)
                        {
                            retValue += m_data.copyFromData2D<ito::uint16>(cbuf, convertedImage.GetCols(), convertedImage.GetRows());
                            m_data >>= 4;
                        }
                    }
                }
            }
            else
            {
                ito::uint16 *cbuf = (ito::uint16*)m_imageBuffer.GetData();
                if (cbuf == NULL)
                {
                    retValue += ito::RetVal(ito::retError, 1002, tr("getVal of PGRFlyCapture failed, since retrived NULL-Pointer.").toLatin1().data());
                }
                else
                {
                    if (copyExternal)
                    {
                        retValue += externalDataObject->copyFromData2D<ito::uint16>(cbuf, m_imageBuffer.GetCols(), m_imageBuffer.GetRows());
                        *externalDataObject >>= 4;
                    }
                    if (!copyExternal || hasListeners)
                    {
                        retValue += m_data.copyFromData2D<ito::uint16>(cbuf, m_imageBuffer.GetCols(), m_imageBuffer.GetRows());
                        m_data >>= 4;
                    }
                }
            }
        }
        else if (bpp <= 16)
        {
            if (format != FlyCapture2::PIXEL_FORMAT_MONO16)
            {
                retValue += checkError(m_imageBuffer.Convert(FlyCapture2::PIXEL_FORMAT_MONO16, &convertedImage));

                if (!retValue.containsError())
                {
                    ito::uint16 *cbuf = (ito::uint16*)convertedImage.GetData();
                    if (cbuf == NULL)
                    {
                        retValue += ito::RetVal(ito::retError, 1002, tr("getVal of PGRFlyCapture failed, since retrived NULL-Pointer.").toLatin1().data());
                    }
                    else
                    {
                        if (copyExternal) retValue += externalDataObject->copyFromData2D<ito::uint16>(cbuf, convertedImage.GetCols(), convertedImage.GetRows());
                        if (!copyExternal || hasListeners) retValue += m_data.copyFromData2D<ito::uint16>(cbuf, convertedImage.GetCols(), convertedImage.GetRows());
                    }
                }
            }
            else
            {
                if (!retValue.containsError())
                {
                    ito::uint16 *cbuf = (ito::uint16*)m_imageBuffer.GetData();
                    if (cbuf == NULL)
                    {
                        retValue += ito::RetVal(ito::retError, 1002, tr("getVal of PGRFlyCapture failed, since retrived NULL-Pointer.").toLatin1().data());
                    }
                    else
                    {
                        if (copyExternal) retValue += externalDataObject->copyFromData2D<ito::uint16>(cbuf, m_imageBuffer.GetCols(), m_imageBuffer.GetRows());
                        if (!copyExternal || hasListeners) retValue += m_data.copyFromData2D<ito::uint16>(cbuf, m_imageBuffer.GetCols(), m_imageBuffer.GetRows());
                    }
                }
            }
        }
        else
        {
            retValue += ito::RetVal::format(ito::retError, 1002, tr("retrieveData of PGRFlyCapture failed, since bitdepth %i not implemented.").toLatin1().data(), bpp);
        }

        if (!retValue.containsError() && m_hasFrameInfo && m_params["metadata"].getVal<int>() > 0)
        {
            double timestamp = timeStampToDouble(m_imageBuffer.GetTimeStamp());
            FlyCapture2::ImageMetadata metadata = m_imageBuffer.GetMetadata();
            if (copyExternal)
            {
                externalDataObject->setTag("timestamp", timestamp);
                externalDataObject->setTag("frame_counter", metadata.embeddedFrameCounter);
                externalDataObject->setTag("roi_x0", metadata.embeddedROIPosition >> 16);
                externalDataObject->setTag("roi_y0", metadata.embeddedROIPosition & 0x0000ffff);
            }
            if (!copyExternal || hasListeners)
            {
                m_data.setTag("timestamp", timestamp);
                m_data.setTag("frame_counter", metadata.embeddedFrameCounter);
                m_data.setTag("roi_x0", metadata.embeddedROIPosition >> 16);
                m_data.setTag("roi_y0", metadata.embeddedROIPosition & 0x0000ffff);
            }
        }

        this->m_isgrabbing = false;
    }

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
bool PGRFlyCapture::GetResolutionFromVideoMode(FlyCapture2::VideoMode mode, int &sizeX, int &sizeY)
{
    switch (mode)
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
bool PGRFlyCapture::GetPixelFormatFromVideoMode(FlyCapture2::VideoMode mode, bool stippled, FlyCapture2::PixelFormat* pixFormat)
{
    switch (mode)
    {
    case FlyCapture2::VIDEOMODE_640x480Y8:
    case FlyCapture2::VIDEOMODE_800x600Y8:
    case FlyCapture2::VIDEOMODE_1024x768Y8:
    case FlyCapture2::VIDEOMODE_1280x960Y8:
    case FlyCapture2::VIDEOMODE_1600x1200Y8:
        if (stippled)
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
        if (stippled)
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
unsigned int PGRFlyCapture::GetBppFromPixelFormat(FlyCapture2::PixelFormat pixelFormat)
{
    switch (pixelFormat)
    {
    case FlyCapture2::PIXEL_FORMAT_MONO8:
    case FlyCapture2::PIXEL_FORMAT_RAW8:
    case FlyCapture2::PIXEL_FORMAT_411YUV8:
    case FlyCapture2::PIXEL_FORMAT_422YUV8:
    case FlyCapture2::PIXEL_FORMAT_422YUV8_JPEG:
    case FlyCapture2::PIXEL_FORMAT_444YUV8:
    case FlyCapture2::PIXEL_FORMAT_BGR:
    case FlyCapture2::PIXEL_FORMAT_BGRU:
    case FlyCapture2::PIXEL_FORMAT_RGBU:
        return 8;
        break;

    case FlyCapture2::PIXEL_FORMAT_MONO12:
    case FlyCapture2::PIXEL_FORMAT_RAW12:
        return 12;
        break;

    case FlyCapture2::PIXEL_FORMAT_MONO16:
    case FlyCapture2::PIXEL_FORMAT_S_MONO16:
    case FlyCapture2::PIXEL_FORMAT_RAW16:
    case FlyCapture2::PIXEL_FORMAT_RGB16:
    case FlyCapture2::PIXEL_FORMAT_S_RGB16:
    case FlyCapture2::PIXEL_FORMAT_BGR16:
        return 16;
        break;

    default:
        return 0;
        break;
    }

    return 0;
}

//----------------------------------------------------------------------------------------------------------------------------------
double PGRFlyCapture::GetFrameTimeFromFrameRate(FlyCapture2::FrameRate frameRate)
{
    switch (frameRate)
    {
    case FlyCapture2::FRAMERATE_1_875:
        return 1 / 1.875;
    case FlyCapture2::FRAMERATE_3_75:
        return 1 / 3.75;
    case FlyCapture2::FRAMERATE_7_5:
        return 1 / 7.5;
    case FlyCapture2::FRAMERATE_15:
        return 1 / 15;
    case FlyCapture2::FRAMERATE_30:
        return 1 / 30;
    case FlyCapture2::FRAMERATE_60:
        return 1 / 60;
    case FlyCapture2::FRAMERATE_120:
        return 1 / 120;
    case FlyCapture2::FRAMERATE_240:
        return 1 / 240;

    default:
        return 0.0;
        break;
    }

    return 0;
}

//----------------------------------------------------------------------------------------------------------------------------------
FlyCapture2::FrameRate PGRFlyCapture::GetSuitAbleFrameRateFromFrameTime(double frameTime)
{
    if (frameTime <= 1 / 240) return FlyCapture2::FRAMERATE_240;
    if (frameTime <= 1 / 120) return FlyCapture2::FRAMERATE_120;
    if (frameTime <= 1 / 60) return FlyCapture2::FRAMERATE_60;
    if (frameTime <= 1 / 30) return FlyCapture2::FRAMERATE_30;
    if (frameTime <= 1 / 15) return FlyCapture2::FRAMERATE_15;
    if (frameTime <= 1 / 7.5) return FlyCapture2::FRAMERATE_7_5;
    if (frameTime <= 1 / 3.75) return FlyCapture2::FRAMERATE_3_75;

    return FlyCapture2::FRAMERATE_1_875;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PGRFlyCapture::checkError(const FlyCapture2::Error &error)
{
    if (error == FlyCapture2::PGRERROR_OK)
    {
        return ito::retOk;
    }

    return ito::RetVal(ito::retError, (int)error.GetType(), error.GetDescription());
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PGRFlyCapture::flyCapChangeFormat7_(bool changeBpp, bool changeROI, int bpp /*=-1*/, int x0 /*=-1*/, int y0 /*=-1*/, int width /*=-1*/, int height /*=-1*/)
{
    ito::RetVal retValue;
    int running = 0;

    if (m_hasFormat7) //set in init
    {
        FlyCapture2::Format7ImageSettings f7ImageSettings = m_currentFormat7Settings;

        if (changeBpp)
        {
            switch (bpp)
            {
            case -1:
                break; //unchanged
            case 8:
                if (m_colouredOutput)
                {
                    f7ImageSettings.pixelFormat = FlyCapture2::PIXEL_FORMAT_BGRU;
                }
                else
                {
                    f7ImageSettings.pixelFormat = FlyCapture2::PIXEL_FORMAT_MONO8;
                }
                break;
            case 12:
                f7ImageSettings.pixelFormat = FlyCapture2::PIXEL_FORMAT_MONO12;
                break;
            case 16:
            {
                f7ImageSettings.pixelFormat = FlyCapture2::PIXEL_FORMAT_MONO16;
                // This comes from the SDK and seems to wrap the little endian / big endian problem
                // Force the camera to PGR's Y16 endianness
                const unsigned int k_imageDataFmtReg = 0x1048;
                unsigned int value = 0;
                m_myCam.ReadRegister(k_imageDataFmtReg, &value);
                value &= ~(0x1 << 0);
                m_myCam.WriteRegister(k_imageDataFmtReg, value);
            }
            break;
            default:
                retValue += ito::RetVal::format(ito::retError, 0, tr("Bitdepth of %i bit is not supported by this plugin.").toLatin1().data(), bpp);
            }

            if ((m_format7Info.pixelFormatBitField & f7ImageSettings.pixelFormat) == 0)
            {
                retValue += ito::RetVal::format(ito::retError, 0, tr("Bitdepth of %i bit not supported by this camera.").toLatin1().data(), bpp);
            }
        }

        if (changeROI)
        {
            if (x0 == -1) x0 = f7ImageSettings.offsetX;
            if (y0 == -1) y0 = f7ImageSettings.offsetY;
            if (width == -1) width = f7ImageSettings.width;
            if (height == -1) height = f7ImageSettings.height;

            if (x0 < 0 || x0 >= (m_format7Info.maxWidth - 1) || (x0 % m_format7Info.offsetHStepSize) != 0)
            {
                retValue += ito::RetVal::format(ito::retError, 0, tr("x0 must be in range [0:%i:%i]").toLatin1().data(), m_format7Info.offsetHStepSize, m_format7Info.maxWidth - 1);
            }

            if (y0 < 0 || y0 >= (m_format7Info.maxHeight - 1) || (y0 % m_format7Info.offsetVStepSize) != 0)
            {
                retValue += ito::RetVal::format(ito::retError, 0, tr("y0 must be in range [0:%i:%i]").toLatin1().data(), m_format7Info.offsetVStepSize, m_format7Info.maxHeight - 1);
            }

            if (width < 0 || ((x0 + width) > m_format7Info.maxWidth) || ((width) % m_format7Info.imageHStepSize) != 0)
            {
                retValue += ito::RetVal::format(ito::retError, 0, tr("width must be in range [0:%i:%i]").toLatin1().data(), m_format7Info.imageHStepSize, m_format7Info.maxWidth - x0);
            }

            if (height < 0 || ((y0 + height) > m_format7Info.maxHeight) || ((height) % m_format7Info.imageVStepSize) != 0)
            {
                retValue += ito::RetVal::format(ito::retError, 0, tr("height must be in range [0:%i:%i]").toLatin1().data(), m_format7Info.imageVStepSize, m_format7Info.maxHeight - y0);
            }

            if (!retValue.containsError())
            {
                f7ImageSettings.offsetX = x0;
                f7ImageSettings.offsetY = y0;
                f7ImageSettings.width = width;
                f7ImageSettings.height = height;
            }
        }

        if (memcmp(&f7ImageSettings, &m_currentFormat7Settings, sizeof(FlyCapture2::Format7ImageSettings)) != 0) //changes
        {
            // Validate the settings to make sure that they are valid
            bool valid;
            retValue += checkError(m_myCam.ValidateFormat7Settings(&f7ImageSettings, &valid, &m_currentPacketInfo));

            if (!retValue.containsError())
            {
                if (!valid)
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("the format options as result of the camera parameters are invalid").toLatin1().data());
                }
                else
                {
                    //stop camera if started
                    if (grabberStartedCount() > 0)
                    {
                        running = grabberStartedCount();
                        setGrabberStarted(1);
                        retValue += stopDevice(NULL);
                    }

                    retValue += checkError(m_myCam.SetFormat7Configuration(&f7ImageSettings, m_currentPacketInfo.recommendedBytesPerPacket));
                }
            }
        }

        //in any cases get now the current information back
        unsigned int packetSize;
        float percentage;
        ito::RetVal ret2 = checkError(m_myCam.GetFormat7Configuration(&m_currentFormat7Settings, &packetSize, &percentage));

        if (!ret2.containsError())
        {
            ParamMapIterator it = m_params.find("roi");
            int *roi = it->getVal<int*>();
            roi[0] = m_currentFormat7Settings.offsetX;
            roi[1] = m_currentFormat7Settings.offsetY;
            roi[2] = m_currentFormat7Settings.width;
            roi[3] = m_currentFormat7Settings.height;
            ito::RangeMeta widthMeta(0, m_format7Info.maxWidth - 1, m_format7Info.offsetHStepSize, m_format7Info.imageHStepSize, m_format7Info.maxWidth, m_format7Info.imageHStepSize);
            ito::RangeMeta heightMeta(0, m_format7Info.maxHeight - 1, m_format7Info.offsetVStepSize, m_format7Info.imageVStepSize, m_format7Info.maxHeight, m_format7Info.imageVStepSize);
            it->setMeta(new ito::RectMeta(widthMeta, heightMeta), true);

            m_params["sizex"].setVal<int>(m_currentFormat7Settings.width);
            m_params["sizey"].setVal<int>(m_currentFormat7Settings.height);

            m_params["bpp"].setVal<int>(GetBppFromPixelFormat(m_currentFormat7Settings.pixelFormat));
        }

        retValue += ret2;

        retValue += checkData(); //check if image must be reallocated

        //restart camera if it has been started as was stopped by this method
        if (running)
        {
            retValue += startDevice(NULL);
            setGrabberStarted(running);
        }
    }
    else
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Changing the image format is only implemented if the camera provides the format7 option").toLatin1().data());
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PGRFlyCapture::flyCapSetExtendedShutter(bool enabled)
{
    ito::RetVal retVal;

    if (enabled && m_extendedShutter != DRAGONFLY_EXTENDED_SHUTTER && m_extendedShutter != GENERAL_EXTENDED_SHUTTER)
    {
        //set the extended shutter

        // Check if the camera supports the FRAME_RATE property
        FlyCapture2::PropertyInfo propInfoFrameRate;
        propInfoFrameRate.type = FlyCapture2::FRAME_RATE;
        retVal += checkError(m_myCam.GetPropertyInfo(&propInfoFrameRate));

        if (propInfoFrameRate.present == true)
        {
            // Turn off frame rate

            FlyCapture2::Property prop;
            prop.type = FlyCapture2::FRAME_RATE;
            retVal += checkError(m_myCam.GetProperty(&prop));

            if (!retVal.containsError())
            {
                prop.autoManualMode = false;
                prop.onOff = false;

                retVal += checkError(m_myCam.SetProperty(&prop));

                if (retVal != ito::retError)
                {
                    m_extendedShutter = GENERAL_EXTENDED_SHUTTER;
                }
            }
        }
        else
        {
            // Frame rate property does not appear to be supported.
            // Disable the extended shutter register instead.
            // This is only applicable for Dragonfly.

            const unsigned int k_extendedShutter = 0x1028;
            unsigned int extendedShutterRegVal = 0;

            retVal += checkError(m_myCam.ReadRegister(k_extendedShutter, &extendedShutterRegVal));

            if (retVal != ito::retError)
            {
                std::bitset<32> extendedShutterBS((int)extendedShutterRegVal);
                if (extendedShutterBS[31] == true)
                {
                    // Set the camera into extended shutter mode
                    retVal += checkError(m_myCam.WriteRegister(k_extendedShutter, 0x80020000));

                    if (retVal != ito::retError)
                    {
                        m_extendedShutter = DRAGONFLY_EXTENDED_SHUTTER;
                    }
                }
                else
                {
                    retVal += ito::RetVal(ito::retWarning, 0, tr("frame rate and extended shutter are not supported. Could not set the extended shutter").toLatin1().data());
                }
            }
        }

        if (m_extendedShutter == UNINITIALIZED)
        {
            m_extendedShutter = NO_EXTENDED_SHUTTER;
        }
    }
    else if (!enabled && m_extendedShutter != NO_EXTENDED_SHUTTER)
    {
        //disable it

        if (m_extendedShutter == GENERAL_EXTENDED_SHUTTER)
        {
            FlyCapture2::Property prop;
            prop.type = FlyCapture2::FRAME_RATE;
            retVal += checkError(m_myCam.GetProperty(&prop));

            if (retVal != ito::retError)
            {
                prop.autoManualMode = true;
                prop.onOff = true;

                retVal += checkError(m_myCam.SetProperty(&prop));

                if (retVal != ito::retError)
                {
                    m_extendedShutter = NO_EXTENDED_SHUTTER;
                }
            }
        }
        else if (m_extendedShutter == DRAGONFLY_EXTENDED_SHUTTER)
        {
            const unsigned int k_extendedShutter = 0x1028;
            unsigned int extendedShutterRegVal = 0;

            retVal += checkError(m_myCam.ReadRegister(k_extendedShutter, &extendedShutterRegVal));

            if (retVal != ito::retError)
            {
                std::bitset<32> extendedShutterBS((int)extendedShutterRegVal);
                if (extendedShutterBS[31] == true)
                {
                    // Set the camera into extended shutter mode
                    retVal += checkError(m_myCam.WriteRegister(k_extendedShutter, 0x80000000));
                }
            }

            if (retVal != ito::retError)
            {
                m_extendedShutter = NO_EXTENDED_SHUTTER;
            }
        }
        else
        {
            //no extended shutter, disabled per default
            m_extendedShutter = NO_EXTENDED_SHUTTER;
        }
    }

    m_params["extended_shutter"].setVal<int>(m_extendedShutter == NO_EXTENDED_SHUTTER ? 0 : 1);

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PGRFlyCapture::flyCapSynchronizeFrameRateShutter()
{
    ito::RetVal retVal;

    //############################
    //  READ FRAME_RATE
    //############################
    FlyCapture2::PropertyInfo propInfoFrameRate;
    propInfoFrameRate.type = FlyCapture2::FRAME_RATE;
    retVal += checkError(m_myCam.GetPropertyInfo(&propInfoFrameRate));

    // Check if the camera supports the FRAME_RATE property
    if (propInfoFrameRate.present)
    {
        static_cast<ito::DoubleMeta*>(m_params["frame_time"].getMeta())->setMin(1.0 / propInfoFrameRate.absMax);
        static_cast<ito::DoubleMeta*>(m_params["frame_time"].getMeta())->setMax(1.0 / propInfoFrameRate.absMin);

        //get prop FRAME_RATE
        FlyCapture2::Property prop;
        prop.type = FlyCapture2::FRAME_RATE;
        retVal += checkError(m_myCam.GetProperty(&prop));

        if (retVal != ito::retError)
        {
            if (prop.onOff == false && (m_params["extended_shutter"].getVal<int>() == 1)) //off -> extended shutter, no frame_rate
            {
                m_params["frame_time"].setFlags(ito::ParamBase::Readonly);
                m_params["frame_time"].setVal<double>(1.0 / prop.absValue);
            }
            else
            {
                m_params["frame_time"].setFlags(0);
                m_params["frame_time"].setVal<double>(1.0 / prop.absValue);
            }
        }
    }
    else
    {
        m_params["frame_time"].setFlags(ito::ParamBase::Readonly);
    }

    //############################
    //  READ SHUTTER
    //############################
    FlyCapture2::PropertyInfo propInfoShutter;
    propInfoShutter.type = FlyCapture2::SHUTTER;
    retVal += checkError(m_myCam.GetPropertyInfo(&propInfoShutter));

    if (propInfoShutter.present && propInfoShutter.absValSupported && propInfoShutter.manualSupported)
    {
        m_params["integration_time"].setFlags(0);
        static_cast<ito::DoubleMeta*>(m_params["integration_time"].getMeta())->setMin(propInfoShutter.absMin / 1000.0);
        static_cast<ito::DoubleMeta*>(m_params["integration_time"].getMeta())->setMax(propInfoShutter.absMax / 1000.0);

        //get prop SHUTTER
        FlyCapture2::Property prop;
        prop.type = FlyCapture2::SHUTTER;
        retVal += checkError(m_myCam.GetProperty(&prop));

        if (retVal != ito::retError)
        {
            m_params["integration_time"].setVal<double>(prop.absValue / 1000.0);
        }
    }
    else
    {
        m_params["integration_time"].setFlags(ito::ParamBase::Readonly);
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PGRFlyCapture::checkData(ito::DataObject *externalDataObject)
{
    int futureHeight = m_params["sizey"].getVal<int>();
    int futureWidth = m_params["sizex"].getVal<int>();
    int futureType;

    int bpp = m_params["bpp"].getVal<int>();
    if (bpp <= 8)
    {
        if (m_colouredOutput)
        {
            futureType = ito::tRGBA32;
        }
        else
        {
            futureType = ito::tUInt8;
        }
    }
    else if (bpp <= 16)
    {
        futureType = ito::tUInt16;
    }
    else if (bpp <= 32)
    {
        futureType = ito::tInt32;
    }
    else
    {
        futureType = ito::tFloat64;
    }

    if (externalDataObject == NULL)
    {
        if (m_data.getDims() < 2 || m_data.getSize(0) != (unsigned int)futureHeight || m_data.getSize(1) != (unsigned int)futureWidth || m_data.getType() != futureType)
        {
            m_data = ito::DataObject(futureHeight, futureWidth, futureType);
        }
    }
    else
    {
        int dims = externalDataObject->getDims();
        if (externalDataObject->getDims() == 0)
        {
            *externalDataObject = ito::DataObject(futureHeight, futureWidth, futureType);
        }
        else if (externalDataObject->calcNumMats() > 1)
        {
            return ito::RetVal(ito::retError, 0, tr("Error during check data, external dataObject invalid. Object has more than 1 plane. It must be of right size and type or a uninitilized image.").toLatin1().data());
        }
        else if (externalDataObject->getSize(dims - 2) != (unsigned int)futureHeight || externalDataObject->getSize(dims - 1) != (unsigned int)futureWidth || externalDataObject->getType() != futureType)
        {
            return ito::RetVal(ito::retError, 0, tr("Error during check data, external dataObject invalid. Object must be of right size and type or a uninitilized image.").toLatin1().data());
        }
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------
double PGRFlyCapture::timeStampToDouble(const FlyCapture2::TimeStamp &timestamp)
{
    /*cycleSecond increments from 0 to 127
      cycleCount increments from 0 to 7999, which equals one second
      cycleOffset increments from 0 to x depending on implementation, where x equals one cycleCount

      m_firstTimestamp is only used to unwrap the timestamp by the modulo 128
    */

    double secOffset;
    double time1 = (double)timestamp.cycleSeconds + (((double)timestamp.cycleCount + ((double)timestamp.cycleOffset / 3072.0)) / 8000.0);

    if (qIsNaN(m_firstTimestamp))
    {
        m_firstTimestamp = (double)(cv::getTickCount()) / cv::getTickFrequency() - time1; //time (in seconds) when the zero-cycle in this cycle-phase of the camera started (e.g. +- 5sec)
        secOffset = 0.0;
    }
    else if (timestamp.cycleSeconds < 64) //timestamp is in the first half of its entire cycle of 128 seconds
    {
        qint64 secondsToFirstTimestamp = qRound64((double)(cv::getTickCount()) / cv::getTickFrequency() - m_firstTimestamp);
        int phasesOf128 = secondsToFirstTimestamp / 128;
        int remainder = secondsToFirstTimestamp % 128;

        if (remainder > 96)
        {
            phasesOf128++;
        }

        secOffset = phasesOf128 << 7;

    }
    else //timestamp is in the second half of its entire cycle of 128 seconds
    {
        qint64 secondsToFirstTimestamp = qRound64((double)(cv::getTickCount()) / cv::getTickFrequency() - m_firstTimestamp);
        int phasesOf128 = secondsToFirstTimestamp / 128;
        int remainder = secondsToFirstTimestamp % 128;

        if (remainder < 32)
        {
            phasesOf128--;
        }

        secOffset = phasesOf128 << 7;

        /*double secondsToFirstTimestamp = (double)(cv::getTickCount()) / cv::getTickFrequency() - m_firstTimestamp;

        double temp = (double)(cv::getTickCount())/cv::getTickFrequency();
        secOffset = 128.0 * ((unsigned int)((temp - m_firstTimestamp)) >> 7);*/
    }

    return secOffset + time1;
}



//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PGRFlyCapture::execFunc(const QString funcName,
    QSharedPointer<QVector<ito::ParamBase> > paramsMand, QSharedPointer<QVector<ito::ParamBase> > paramsOpt,
    QSharedPointer<QVector<ito::ParamBase> > paramsOut, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retValue = ito::retOk;
    ito::ParamBase *param1 = NULL;
    ito::ParamBase *param2 = NULL;
    ito::ParamBase *param3 = NULL;

    if (funcName == "printParameterInfo")
    {
        //this is an internal test function to print out all camera parameters including their settings.
        QMap<QByteArray, int> properties;
        properties["SHUTTER"] = FlyCapture2::SHUTTER;
        properties["BRIGHTNESS"] = FlyCapture2::BRIGHTNESS;
        properties["AUTO_EXPOSURE"] = FlyCapture2::AUTO_EXPOSURE;
        properties["SHARPNESS"] = FlyCapture2::SHARPNESS;
        properties["WHITE_BALANCE"] = FlyCapture2::WHITE_BALANCE;
        properties["HUE"] = FlyCapture2::HUE;
        properties["SATURATION"] = FlyCapture2::SATURATION;
        properties["GAMMA"] = FlyCapture2::GAMMA;
        properties["IRIS"] = FlyCapture2::IRIS;
        properties["FOCUS"] = FlyCapture2::FOCUS;
        properties["ZOOM"] = FlyCapture2::ZOOM;
        properties["PAN"] = FlyCapture2::PAN;
        properties["TILT"] = FlyCapture2::TILT;
        properties["SHUTTER"] = FlyCapture2::SHUTTER;
        properties["GAIN"] = FlyCapture2::GAIN;
        properties["TRIGGER_MODE"] = FlyCapture2::TRIGGER_MODE;
        properties["TRIGGER_DELAY"] = FlyCapture2::TRIGGER_DELAY;
        properties["FRAME_RATE"] = FlyCapture2::FRAME_RATE;
        properties["TEMPERATURE"] = FlyCapture2::TEMPERATURE;

        QMapIterator<QByteArray, int> i(properties);
        while (i.hasNext())
        {
            i.next();

            //get prop SHUTTER
            FlyCapture2::Property prop;
            prop.type = (FlyCapture2::PropertyType)i.value();
            m_myCam.GetProperty(&prop);
            std::cout << "Parameter " << i.key().data() << ": \n" << std::endl;
            std::cout << prop.absControl << " " << prop.absValue << " " << prop.autoManualMode << " " << prop.onePush \
                << " " << prop.onOff << " " << prop.present << " " << prop.reserved << " " << prop.type << " " << prop.valueA << " " << prop.valueB << "\n" << std::endl;
        }
    }
    else if (funcName == "setStrobeMode") {
        FlyCapture2::StrobeControl strobeControl;
        FlyCapture2::StrobeInfo strobeInfo;
        ito::ParamBase* param;

        unsigned int source = ito::getParamByName(&(*paramsMand), "source", &retValue)->getVal<int>();
        strobeInfo.source = source;
        retValue += checkError(m_myCam.GetStrobeInfo(&strobeInfo));

        if (!retValue.containsError()) {
            if (!strobeInfo.present) {
                retValue += ito::RetVal(ito::retError, 0, "strobe feature is not available for this pin on the given camera");
            }
        }

        if (!retValue.containsError()) {
            strobeControl.source = source;
            retValue += checkError(m_myCam.GetStrobe(&strobeControl));
        }

        if (!retValue.containsError()) {
            param = ito::getParamByName(&(*paramsOpt), "onOff");
            if (param != nullptr) {
                if (!strobeInfo.onOffSupported) {
                    retValue += ito::RetVal(ito::retError, 0, "strobe feature cannot be switched on or off for this pin on the given camera");
                }
                else {
                    strobeControl.onOff = param->getVal<int>();
                }
            }
        }

        if (!retValue.containsError()) {
            param = ito::getParamByName(&(*paramsOpt), "polarity");
            if (param != nullptr) {
                if (!strobeInfo.polaritySupported) {
                    retValue += ito::RetVal(ito::retError, 0, "polarity can not be changed for this pin on the given camera");
                }
                else {
                    strobeControl.polarity = param->getVal<int>();
                }
            }
        }

        if (!retValue.containsError()) {
            param = ito::getParamByName(&(*paramsOpt), "delay");
            if (param != nullptr) {
                strobeControl.delay = (float)param->getVal<double>();
            }

            param = ito::getParamByName(&(*paramsOpt), "duration");
            if (param != nullptr) {
                strobeControl.duration = (float)param->getVal<double>();
            }
            retValue += checkError(m_myCam.SetStrobe(&strobeControl));
        }
    }
    else if (funcName == "getStrobeMode") {
        FlyCapture2::StrobeControl strobeControl;
        FlyCapture2::StrobeInfo strobeInfo;
        ito::ParamBase* param;

        unsigned int source = ito::getParamByName(&(*paramsMand), "source", &retValue)->getVal<int>();
        strobeInfo.source = source;
        retValue += checkError(m_myCam.GetStrobeInfo(&strobeInfo));

        if (!retValue.containsError()) {
            if (!strobeInfo.present) {
                retValue += ito::RetVal(ito::retError, 0, "strobe feature is not available for this pin on the given camera");
            }
        }

        if (!retValue.containsError()) {
            strobeControl.source = source;
            retValue += checkError(m_myCam.GetStrobe(&strobeControl));
        }

        if (!retValue.containsError()) {
            param = ito::getParamByName(&(*paramsOut), "onOff");
            if (param != nullptr) {
                if (!strobeInfo.onOffSupported) {
                    retValue += ito::RetVal(ito::retError, 0, "strobe feature cannot be switched on or off for this pin on the given camera");
                }
                else {
                    retValue += param->setVal<int>(strobeControl.onOff);
                }
            }
        }

        if (!retValue.containsError()) {
            param = ito::getParamByName(&(*paramsOut), "polarity");
            if (param != nullptr) {
                if (!strobeInfo.polaritySupported) {
                    retValue += ito::RetVal(ito::retError, 0, "polarity can not be changed for this pin on the given camera");
                }
                else {
                    retValue += param->setVal<int>(strobeControl.polarity);
                }
            }
        }

        if (!retValue.containsError()) {
            param = ito::getParamByName(&(*paramsOut), "delay");
            if (param != nullptr) {
                retValue += param->setVal<double>(strobeControl.delay);
            }

            param = ito::getParamByName(&(*paramsOut), "duration");
            if (param != nullptr) {
                retValue += param->setVal<double>(strobeControl.duration);
            }
        }
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

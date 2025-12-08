/* ********************************************************************
Plugin "Roughness" for itom software
URL : http ://www.uni-stuttgart.de/ito
Copyright(C) 2016, Institut fuer Technische Optik (ITO),
Universitaet Stuttgart, Germany;
IPROM, TU Braunschweig, Germany

This file is part of a plugin for the measurement software itom.

This itom - plugin is free software; you can redistribute it and / or modify it
under the terms of the GNU Library General Public Licence as published by
the Free Software Foundation; either version 2 of the Licence, or(at
your option) any later version.

itom and its plugins are distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the GNU Library
General Public Licence for more details.

You should have received a copy of the GNU Library General Public License
along with itom.If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include "tucsen.h"
#include "tucsenConst.h"
#include <TUCamApi.h>
#include "pluginVersion.h"
#include "gitVersion.h"

#include <qstring.h>
#include <qstringlist.h>
#include <qplugin.h>
#include <qmessagebox.h>
#include <regex>

#include "dockWidgetTucsen.h"

//----------------------------------------------------------------------------------------------------------------------------------
//! Constructor of Interface Class.
/*!
    \todo add necessary information about your plugin here.
*/

//static char InitList[5] = {0, 0, 0, 0, 0};  /*!<A map with successful initialized Cameras (max = 5) */
//static char Initnum = 0;    /*!< Number of successful initialized Cameras */

TucsenInterface::TucsenInterface()
{
    m_type = ito::typeDataIO | ito::typeGrabber; //any grabber is a dataIO device AND its subtype grabber (bitmask -> therefore the OR-combination).
    setObjectName("Tucsen");

    m_description = QObject::tr("Tucsen camera interface");

    m_detaildescription = QObject::tr(
"This plugin supports Allied Vision GigE and firewire cameras and has currently been tested with the following models: \n\
\n\
- Marlin, F033 (monochrome, Firewire) \n\
- Manta G-917B and G-146B (monochrome, GigE) \n\
- Alvium 1800 U-811c (monochrome, USB) \n\
\n\
The plugin was tested with AVT Vimba 1.3.0, 1.4.0, 2.5.0. \n\
\n\
In order to run your camera, please install the Vimba SDK in the right version such that the necessary drivers are installed. \n\
Color formats are not supported.");

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
    m_aboutThis = QObject::tr(GITVERSION);

    //add mandatory and optional parameters for the initialization here.
    //append them to m_initParamsMand or m_initParamsOpt.
    ito::Param paramVal("CameraNumber", ito::ParamBase::Int, 0, 100, 0, "camera number");
    m_initParamsOpt.append(paramVal);

    //paramVal = ito::Param("GigEPacketSize", ito::ParamBase::Int, 0, new ito::IntMeta(0, 8228, 1), "ethernet packet size (GigE cameras only). If 0, the camera tries to startup with a value of 8228 bytes/sec (network card must have jumbo frames enabled). Else set it to 1500 bytes/sec, which is a safe setting for all GigE Ethernet network cards");
    //m_initParamsOpt.append(paramVal);
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Destructor of Interface Class.
/*!

*/
TucsenInterface::~TucsenInterface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal TucsenInterface::getAddInInst(ito::AddInBase** addInInst)
{
    NEW_PLUGININSTANCE(Tucsen) // the argument of the macro is the classname of the plugin
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal TucsenInterface::closeThisInst(ito::AddInBase** addInInst)
{
   REMOVE_PLUGININSTANCE(Tucsen) // the argument of the macro is the classname of the plugin
   return ito::retOk;
}



//----------------------------------------------------------------------------------------------------------------------------------
//! Constructor of plugin.
/*!
    \todo add internal parameters of the plugin to the map m_params. It is allowed to append or remove entries from m_params
    in this constructor or later in the init method
*/
Tucsen::Tucsen() :
    AddInGrabber(),
    m_isgrabbing(false),
    //m_camera(CameraPtr()),
    m_aliveTimer(NULL),
    m_aliveTimerThread(NULL)
{
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "Tucsen", tr("Name of plugin").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

#if defined(ITOM_ADDININTERFACE_VERSION) && ITOM_ADDININTERFACE_VERSION > 0x010300
    int roi[] = {0, 0, 2048, 2048};
    paramVal = ito::Param("roi", ito::ParamBase::IntArray, 4, roi, tr("ROI (x,y,width,height) [this replaces the values x0,x1,y0,y1]").toLatin1().data());
    ito::RectMeta *rm = new ito::RectMeta(ito::RangeMeta(roi[0], roi[2]-1), ito::RangeMeta(roi[1], roi[3])); //RangeMeta includes the last value, therefore -1
    paramVal.setMeta(rm, true);
    m_params.insert(paramVal.getName(), paramVal);
#else
    paramVal = ito::Param("x0", ito::ParamBase::Int | ito::ParamBase::In, 0, 2048, 0, tr("first pixel index in ROI (x-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("y0", ito::ParamBase::Int | ito::ParamBase::In, 0, 2048, 0, tr("first pixel index in ROI (y-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("x1", ito::ParamBase::Int | ito::ParamBase::In, 0, 1279, 1279, tr("last pixel index in ROI (x-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("y1", ito::ParamBase::Int | ito::ParamBase::In, 0, 1023, 1023, tr("last pixel index in ROI (y-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
#endif
    
    //paramVal = ito::Param("binning", ito::ParamBase::Int | ito::ParamBase::In, 101, 101, 101, tr("binning (horizontal_factor * 100 + vertical_factor)").toLatin1().data());
    //m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("timeout", ito::ParamBase::Double | ito::ParamBase::In, 0.0, std::numeric_limits<double>::max(), 2.0, tr("timeout for image acquisition in sec").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("sizex", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::In, 684, 5472, 5472, tr("width of ROI").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizey", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::In, 456, 3648, 3648, tr("height of ROI").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("camera_number", ito::ParamBase::Int | ito::ParamBase::In, 0, 10, 0, tr("Camera Number").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("bpp", ito::ParamBase::Int | ito::ParamBase::In, 8, 16, 16, tr("Bit depth of sensor").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("integration_time", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 60.0, 0.01, tr("Integrationtime of CCD [s]").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("frame_rate", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 60.0, 0.01, tr("Frame rate of camera [s]").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("offset", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 33.0, 0.0, tr("Offset as physical value that is a DC offset applied to the video signal. This values changes the blacklevel.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("gain", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 33.0, 0.0, tr("Gain of AD in dB, set it to 0.0 for best image quality.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    //paramVal = ito::Param("gain_auto", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("auto-controlled gain (0: off, 1: continuously varies the gain; gain will be read-only then)").toLatin1().data());
    //m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("gamma", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 0.0, tr("Gamma value").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    //paramVal = ito::Param("stream_bps", ito::ParamBase::Int | ito::ParamBase::In, 1000000, 124000000, 124000000, tr("Bandwidth allocation for each camera. Must be adapted if multiple cameras are connected to the same ethernet adapter").toLatin1().data());
    //m_params.insert(paramVal.getName(), paramVal);
    //paramVal = ito::Param("packet_size", ito::ParamBase::Int | ito::ParamBase::In, 500, 16384, 8228, tr("Bandwidth allocation for each camera. Must be adapted if multiple cameras are connected to the same ethernet adapter").toLatin1().data());
    //m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("device_temperature", ito::ParamBase::Double | ito::ParamBase::Readonly | ito::ParamBase::In, 1.0, 100.0, 25.0, tr("device temperature of sensor in �C").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    //paramVal = ito::Param("trigger_mode", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("trigger mode (0: Off, 1: On)").toLatin1().data());
    //m_params.insert(paramVal.getName(),paramVal);

    //paramVal = ito::Param("trigger_source", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("trigger source (Freerun, Line1, Line2, Line3, Line4, FixedRate, Software, InputLines). Not all values are supported for all cameras.").toLatin1().data());
    //m_params.insert(paramVal.getName(),paramVal);

    //paramVal = ito::Param("trigger_activation", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("trigger activation (RisingEdge, FallingEdge, AnyEdge, LevelHigh, LevelLow). Not all values are supported for all cameras.").toLatin1().data());
    //m_params.insert(paramVal.getName(),paramVal);

    //the following lines create and register the plugin's dock widget. Delete these lines if the plugin does not have a dock widget.
    DockWidgetTucsen *dw = new DockWidgetTucsen(this);

    Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
    QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
    createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dw);
}

//----------------------------------------------------------------------------------------------------------------------------------
Tucsen::~Tucsen()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
//ito::RetVal Tucsen::getParamListRaw()
//{
//    TUCAM_ELEMENT node;
//    node.pName = (char*)"Root";
//
//    char access[][3] = {"NI", "NA", "WO", "RO", "RW"};
//    char elemType[][20] = {
//        "Value",
//        "Base",
//        "Integer",
//        "Boolean",
//        "Command",
//        "Float",
//        "String",
//        "Register",
//        "Category",
//        "Enumeration",
//        "EnumEntry",
//        "Port"};
//
//    TUCAM_GenICam_ElementAttrNext(m_opCam.hIdxTUCam, &node, node.pName, TU_CAMERA_XML);
//
//}

//----------------------------------------------------------------------------------------------------------------------------------
//! initialization of plugin
/*!
    \sa close
*/
ito::RetVal Tucsen::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    unsigned int cameraNumber = static_cast<unsigned int>(paramsOpt->at(0).getVal<int>());

    m_params["camera_number"].setVal<int>(cameraNumber);
    timeoutMS = sToMs(m_params["timeout"].getVal<double>());

    m_itApi.uiCamCount = 0;
    char curPath[4] = "./";
    m_itApi.pstrConfigPath = curPath;
    TUCAMRET turet = TUCAM_Api_Init(&m_itApi);
    if (turet != TUCAMRET_SUCCESS || 0 == m_itApi.uiCamCount)
        return ito::RetVal(ito::retError, 0, tr("No cameras found, check driver and connection!").toLatin1().data());

    if (cameraNumber >= m_itApi.uiCamCount)
    {
        return ito::RetVal(
            ito::retError,
            TUCAMRET_OUT_OF_RANGE,
            tr("Camera number out of range!").toLatin1().data());
        return TUCAMRET_OUT_OF_RANGE;
    }
    m_opCam.uiIdxOpen = cameraNumber;
    turet = TUCAM_Dev_Open(&m_opCam);
    if (turet != TUCAMRET_SUCCESS)
        return ito::RetVal(
            ito::retError,
            turet,
            tr("Error opening camera!").toLatin1().data());

	for (int i = (int)TUIDP_GLOBALGAIN; i < (int)TUIDP_ENDPROPERTY; ++i)
    {
        memset(&(m_props[i - (int)TUIDP_GLOBALGAIN]), 0, sizeof(TUCAM_PROP_ATTR));
        m_props[i - (int)TUIDP_GLOBALGAIN].idProp = i;

        turet = TUCAM_Prop_GetAttr(m_opCam.hIdxTUCam, &m_props[i - (int)TUIDP_GLOBALGAIN]);
        if (TUCAMRET_SUCCESS ==
            TUCAM_Prop_GetAttr(m_opCam.hIdxTUCam, &m_props[i - (int)TUIDP_GLOBALGAIN]))
            m_hasProp[i - (int)TUIDP_GLOBALGAIN] = true;
        else
            m_hasProp[i - (int)TUIDP_GLOBALGAIN] = false;

        if ((i == TUIDP_GLOBALGAIN) && m_hasProp[i - (int)TUIDP_GLOBALGAIN])
        {
            m_params["gain"].setMeta(
                new ito::DoubleMeta(
                    m_props[i - (int)TUIDP_GLOBALGAIN].dbValMin,
                    m_props[i - (int)TUIDP_GLOBALGAIN].dbValMax,
                    m_props[i - (int)TUIDP_GLOBALGAIN].dbValStep),
                true);
            double dVal = 0;
            TUCAM_Prop_GetValue(m_opCam.hIdxTUCam, i, &dVal, 0);
            m_params["gain"].setVal<double>(dVal);
        }
        else if ((i == TUIDP_EXPOSURETM) && m_hasProp[i - (int)TUIDP_GLOBALGAIN])
        {
            m_params["integration_time"].setMeta(
                new ito::DoubleMeta(
                    m_props[i - (int)TUIDP_EXPOSURETM].dbValMin,
                    m_props[i - (int)TUIDP_EXPOSURETM].dbValMax,
                    m_props[i - (int)TUIDP_EXPOSURETM].dbValStep),
                true);
            double dVal = 0;
            TUCAM_Prop_GetValue(m_opCam.hIdxTUCam, i, &dVal, 0);
            m_params["integration_time"].setVal<double>(dVal);
        }

        else if ((i == TUIDP_BLACKLEVEL) && m_hasProp[i - (int)TUIDP_GLOBALGAIN])
        {
            m_params["offset"].setMeta(
                new ito::DoubleMeta(
                    m_props[i - (int)TUIDP_BLACKLEVEL].dbValMin,
                    m_props[i - (int)TUIDP_BLACKLEVEL].dbValMax,
                    m_props[i - (int)TUIDP_BLACKLEVEL].dbValStep),
                true);
            double dVal = 0;
            TUCAM_Prop_GetValue(m_opCam.hIdxTUCam, i, &dVal, 0);
            m_params["offset"].setVal<double>(dVal);
        }
        else if ((i == TUIDP_GAMMA) && m_hasProp[i - (int)TUIDP_GLOBALGAIN])
        {
            m_params["gamma"].setMeta(
                new ito::DoubleMeta(
                    m_props[i - (int)TUIDP_GAMMA].dbValMin,
                    m_props[i - (int)TUIDP_GAMMA].dbValMax,
                    m_props[i - (int)TUIDP_GAMMA].dbValStep),
                true);
            double dVal = 0;
            TUCAM_Prop_GetValue(m_opCam.hIdxTUCam, i, &dVal, 0);
            m_params["gamma"].setVal<double>(dVal);
        }
        else if ((i == TUIDP_FRAME_RATE) && m_hasProp[i - (int)TUIDP_GLOBALGAIN])
        {
            m_params["frame_rate"].setMeta(
                new ito::DoubleMeta(
                    m_props[i - (int)TUIDP_FRAME_RATE].dbValMin,
                    m_props[i - (int)TUIDP_FRAME_RATE].dbValMax,
                    m_props[i - (int)TUIDP_FRAME_RATE].dbValStep),
                true);
            double dVal = 0;
            TUCAM_Prop_GetValue(m_opCam.hIdxTUCam, i, &dVal, 0);
            m_params["frame_rate"].setVal<double>(dVal);
        }
    }

    TUCAM_CAPA_ATTR capa;
    TUCAM_VALUE_TEXT valText;

    memset(&capa, 0, sizeof(TUCAM_CAPA_ATTR));
    memset(&valText, 0, sizeof(TUCAM_VALUE_TEXT));
    char szRes[64] = {0};
    valText.pText = szRes;
    valText.nTextSize = 64;
    ito::int32 capaVal = 0;

    capa.idCapa = TUIDC_RESOLUTION;
    turet = TUCAM_Capa_GetAttr(m_opCam.hIdxTUCam, &capa);
    TUCAM_Capa_GetValue(m_opCam.hIdxTUCam, TUIDC_RESOLUTION, &capaVal);
    valText.nID = capaVal;
    TUCAMRET n = TUCAM_Capa_GetValueText(m_opCam.hIdxTUCam, &valText);
    std::regex rexp(R"(\d+)");
    std::vector<ito::uint32> numbers;
    std::string valStr{valText.pText};
    for (std::sregex_iterator it(valStr.begin(), valStr.end(), rexp), endit; it != endit; it++)
    {
        numbers.push_back(atoi(it->str().data()));
    }
    m_params["sizex"].setMeta(new ito::IntMeta(numbers[0], numbers[0], 1), true);
    m_params["sizex"].setVal<ito::int32>(numbers[0]);
    m_params["sizey"].setMeta(new ito::IntMeta(numbers[1], numbers[1], 1), true);
    m_params["sizey"].setVal<ito::int32>(numbers[1]);

    TUCAM_Capa_GetValue(m_opCam.hIdxTUCam, TUIDC_BITOFDEPTH, &capaVal);
    //m_params["bpp"].setMeta(new ito::IntMeta(8, 16, 8), true);
    m_params["bpp"].setVal<ito::int32>(capaVal);
    

    if (!retValue.containsError())
    {
        retValue += checkData();
    }

    if (!retValue.containsError())
    {
        emit parametersChanged(m_params);

        //configure aliveTimer
        m_aliveTimerThread = new QThread(this);
        m_aliveTimer = new QTimer(NULL);
        m_aliveTimer->setInterval(2000);
        m_aliveTimer->stop();
        m_aliveTimer->moveToThread(m_aliveTimerThread);
        m_aliveTimerThread->start();
        connect(m_aliveTimer, SIGNAL(timeout()), this, SLOT(aliveTimer_fire()), Qt::DirectConnection);
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
//! shutdown of plugin
/*!
    \sa init
*/
ito::RetVal Tucsen::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    //todo:
    // - disconnect the device if not yet done
    // - this function is considered to be the "inverse" of init.

    int nr = m_params["camera_number"].getVal<int>();

    TUCAMRET tret = TUCAM_Dev_Close(m_opCam.hIdxTUCam);
    if (tret != TUCAMRET_SUCCESS)
        retValue += ito::RetVal(ito::retError, tret, tr("Error closing camera").toLatin1().data());
    tret = TUCAM_Api_Uninit();
    if (tret != TUCAMRET_SUCCESS)
        retValue += ito::RetVal(ito::retError, tret, tr("Error uninitializing API").toLatin1().data());


    if (m_aliveTimer)
    {
        m_aliveTimer->stop();
        m_aliveTimer->deleteLater();
    }

    if (m_aliveTimerThread)
    {
        m_aliveTimerThread->exit();
        m_aliveTimerThread->wait();
        m_aliveTimerThread->deleteLater();
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Tucsen::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;
    QString key;
    bool hasIndex = false;
    int index;
    QString suffix;
    ParamMapIterator it;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (retValue == ito::retOk)
    {
        //gets the parameter key from m_params map (read-only is allowed, since we only want to get the value).
        retValue += apiGetParamFromMapByKey(m_params, key, it, false);
    }

    if (!retValue.containsError())
    {
        if (key == "device_temperature")
        {
            double dVal;
            TUCAM_Prop_GetValue(m_opCam.hIdxTUCam, TUIDP_TEMPERATURE, &dVal);
            retValue+= it->setVal<double>(dVal);
        }

        //finally, save the desired value in the argument val (this is a shared pointer!)
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
ito::RetVal Tucsen::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex;
    int index;
    QString suffix;
    ParamMapIterator it;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName( val->getName(), key, hasIndex, index, suffix );

    if (!retValue.containsError())
    {
        //gets the parameter key from m_params map (read-only is not allowed and leads to ito::retError).
        retValue += apiGetParamFromMapByKey(m_params, key, it, true);
    }

    if (!retValue.containsError())
    {
        //here the new parameter is checked whether its type corresponds or can be cast into the
        // value in m_params and whether the new type fits to the requirements of any possible
        // meta structure.
        retValue += apiValidateParam(*it, *val, false, true);
    }

    if (!retValue.containsError())
    {
        if (key == "integration_time")
        {
            TUCAMRET tret = TUCAM_Prop_SetValue(m_opCam.hIdxTUCam, TUIDP_EXPOSURETM, val->getVal<double>(), 0);
            if (tret != TUCAMRET_SUCCESS)
                retValue += ito::RetVal(ito::retError, tret, tr("Error setting integration time").toLatin1().data());
        }
        else if (key == "bpp")
        {
            //enum_idx = -1;
            //switch (val->getVal<int>())
            //{
            //case 8:
            //    enum_idx = m_bppEnum.bppMono8;
            //    break;
            //case 10:
            //    enum_idx = m_bppEnum.bppMono10;
            //    break;
            //case 12:
            //    enum_idx = m_bppEnum.bppMono12;
            //    break;
            //case 14:
            //    enum_idx = m_bppEnum.bppMono14;
            //    break;
            //}

            //if (enum_idx > -1)
            //{
            //    retValue += setEnumFeature("PixelFormat", enum_idx);
            //}
            //else
            //{
            //    retValue += ito::RetVal(ito::retError, 0, "non supported bpp");
            //}

            if (!retValue.containsError())
            {
                //retValue += synchronizeParameters(fBpp);
            }
        }
        else if (key == "offset")
        {
            TUCAMRET tret =
                TUCAM_Prop_SetValue(m_opCam.hIdxTUCam, TUIDP_BLACKLEVEL, val->getVal<double>(), 0);
            if (tret != TUCAMRET_SUCCESS)
                retValue += ito::RetVal(
                    ito::retError, tret, tr("Error setting integration time").toLatin1().data());
        }
        else if (key == "gain")
        {
            TUCAMRET tret = TUCAM_Prop_SetValue(m_opCam.hIdxTUCam, TUIDP_GLOBALGAIN, val->getVal<double>(), 0);
            if (tret != TUCAMRET_SUCCESS)
                retValue += ito::RetVal(
                    ito::retError,
                    tret,
                    tr("Error setting integration time").toLatin1().data());
        }
        else if (key == "gamma")
        {
            TUCAMRET tret =
                TUCAM_Prop_SetValue(m_opCam.hIdxTUCam, TUIDP_GAMMA, val->getVal<double>(), 0);
            if (tret != TUCAMRET_SUCCESS)
                retValue += ito::RetVal(
                    ito::retError, tret, tr("Error setting integration time").toLatin1().data());
        }
        else if (key == "frame_rate")
        {
            TUCAMRET tret =
                TUCAM_Prop_SetValue(m_opCam.hIdxTUCam, TUIDP_FRAME_RATE, val->getVal<double>(), 0);
            if (tret != TUCAMRET_SUCCESS)
                retValue += ito::RetVal(
                    ito::retError, tret, tr("Error setting integration time").toLatin1().data());
        }
        else if (key == "roi")
        {
            if (!hasIndex)
            {
                const int* old_roi = it->getVal<const int*>();
                const int* roi = val->getVal<const int*>();

	            TUCAM_ROI_ATTR turoi;
                turoi.bEnable = true;
                turoi.nHOffset = roi[0];
                turoi.nVOffset = roi[1];
                turoi.nWidth = roi[2];
                turoi.nHeight = roi[3];

                TUCAMRET tret = TUCAM_Cap_SetROI(m_opCam.hIdxTUCam, turoi);
                if (tret != TUCAMRET_SUCCESS)
                    retValue += ito::RetVal(
                        ito::retError,
                        tret,
                        tr("Error setting integration time").toLatin1().data());
            }
            else
            {
                TUCAM_ROI_ATTR turoi;
                turoi.bEnable = true;
                TUCAMRET tret = TUCAM_Cap_GetROI(m_opCam.hIdxTUCam, &turoi);
                if (tret != TUCAMRET_SUCCESS)
                    retValue += ito::RetVal(
                        ito::retError,
                        tret,
                        tr("Error getting ROI").toLatin1().data());
                switch (index)
                {
                case 0:
                    turoi.nHOffset = val->getVal<int>();
                    tret = TUCAM_Cap_GetROI(m_opCam.hIdxTUCam, &turoi);
                    break;
                case 1:
                    turoi.nVOffset = val->getVal<int>();
                    tret = TUCAM_Cap_GetROI(m_opCam.hIdxTUCam, &turoi);
                    break;
                case 2:
                    turoi.nWidth = val->getVal<int>();
                    tret = TUCAM_Cap_GetROI(m_opCam.hIdxTUCam, &turoi);
                    break;
                case 3:
                    turoi.nHeight = val->getVal<int>();
                    tret = TUCAM_Cap_GetROI(m_opCam.hIdxTUCam, &turoi);
                    break;
                }
            }

            //retValue += synchronizeParameters(fSize);
        }
        else if (key == "x0")
        {
            //DEPRECATED, therefore redirections to roi parameter
            QSharedPointer<ito::ParamBase> p(new ito::ParamBase("roi[0]", ito::ParamBase::Int, val->getVal<int>()));
            retValue += setParam(p, NULL);
        }
        else if (key == "y0")
        {
            //DEPRECATED, therefore redirections to roi parameter
            QSharedPointer<ito::ParamBase> p(new ito::ParamBase("roi[1]", ito::ParamBase::Int, val->getVal<int>()));
            retValue += setParam(p, NULL);
        }
        else if (key == "x1")
        {
            //DEPRECATED, therefore redirections to roi parameter
            QSharedPointer<ito::ParamBase> p(new ito::ParamBase("roi[2]", ito::ParamBase::Int, 1 + val->getVal<int>() - m_params["roi"].getVal<int*>()[0]));
            retValue += setParam(p, NULL);
        }
        else if (key == "y1")
        {
            //DEPRECATED, therefore redirections to roi parameter
            QSharedPointer<ito::ParamBase> p(new ito::ParamBase("roi[3]", ito::ParamBase::Int, 1 + val->getVal<int>() - m_params["roi"].getVal<int*>()[1]));
            retValue += setParam(p, NULL);
        }
        else
        {
            //all parameters that don't need further checks can simply be assigned
            //to the value in m_params (the rest is already checked above)
            retValue += it->copyValueFrom( &(*val) );
        }
    }

    if (!retValue.containsError())
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
ito::RetVal Tucsen::startDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    incGrabberStarted(); //increment a counter to see how many times startDevice has been called

    if (grabberStartedCount() == 1)
    {
        m_frame.pBuffer = NULL;
        m_frame.ucFormatGet = TUFRM_FMT_RAW;
        m_frame.uiRsdSize = 1;

        TUCAM_Cap_GetTrigger(m_opCam.hIdxTUCam, &m_trg);
        m_trg.nTgrMode = (INT32)TUCCM_TRIGGER_SOFTWARE;
        m_trg.nFrames = 1;
        TUCAM_Cap_SetTrigger(m_opCam.hIdxTUCam, m_trg);

        TUCAM_Buf_Alloc(m_opCam.hIdxTUCam, &m_frame);

        long lRet = (long)TUCAMRET_NOT_SUPPORT;

        TUCAMRET turet = TUCAM_Cap_Start(m_opCam.hIdxTUCam, (UINT32)TUCCM_TRIGGER_SOFTWARE);
        if (turet != TUCAMRET_SUCCESS)
            retValue += ito::RetVal(ito::retError, turet, tr("Error starting camera").toLatin1().data());
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Tucsen::stopDevice(ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    decGrabberStarted(); //decrements the counter (see startDevice)

    if (grabberStartedCount() < 0)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("The grabber has already been stopped.").toLatin1().data());
        setGrabberStarted(0);
    }
    else if (grabberStartedCount() == 0)
    {
        TUCAMRET turet;
        turet = TUCAM_Buf_AbortWait(m_opCam.hIdxTUCam);
        if (turet != TUCAMRET_SUCCESS)
            retValue += ito::RetVal(ito::retError, turet, tr("Error waiting for buffer abort").toLatin1().data());
        turet = TUCAM_Cap_Stop(m_opCam.hIdxTUCam);
        if (turet != TUCAMRET_SUCCESS)
            retValue += ito::RetVal(
                ito::retError, turet, tr("Error stopping camera").toLatin1().data());
        TUCAM_Buf_Release(m_opCam.hIdxTUCam);
        if (turet != TUCAMRET_SUCCESS)
            retValue +=
                ito::RetVal(ito::retError, turet, tr("Error releasing camera").toLatin1().data());
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Tucsen::acquire(const int trigger, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    bool RetCode;
    TUCAMRET turet = TUCAMRET_INVALID_CAMERA;

    if (grabberStartedCount() <= 0)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Tried to acquire an image without having started the device.").toLatin1().data());
    }
    else
    {
        m_isgrabbing = true;
		turet = TUCAM_Cap_DoSoftwareTrigger(m_opCam.hIdxTUCam);
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    if (turet == TUCAMRET_SUCCESS)
    {
        if (timeoutMS > 2000)
            {
                QMetaObject::invokeMethod(m_aliveTimer, "start");
                turet = TUCAM_Buf_WaitForFrame(m_opCam.hIdxTUCam, &m_frame);
                QMetaObject::invokeMethod(m_aliveTimer, "stop");
            }
        else
        {
            turet = TUCAM_Buf_WaitForFrame(m_opCam.hIdxTUCam, &m_frame);
        }
    }
    else
    {
        retValue += ito::RetVal(ito::retError, turet, tr("Error acquiring image").toLatin1().data());
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Tucsen::retrieveData(ito::DataObject* externalDataObject)
{
    //todo: this is just a basic example for getting the buffered image to m_data or the externally given data object
    //enhance it and adjust it for your needs
    ito::RetVal retValue(ito::retOk);

    ito::DataObject *dataObj = externalDataObject ? externalDataObject : &m_data;

    bool hasListeners = (m_autoGrabbingListeners.size() > 0);
    bool copyExternal = (externalDataObject != NULL);

    retValue += m_acquisitionStatus; //e.g. timeout in acquisition (from acquire)

    if (!retValue.containsError())
    {

        if (m_isgrabbing == false)
        {
            retValue += ito::RetVal(ito::retWarning, 0, tr("Tried to get picture without triggering exposure").toLatin1().data());
        }
        else
        {
            //step 1: update size of externalDataObject. The internal one m_data is already checked after any parameter change (in synchronizeParameters method)
            if (externalDataObject)
            {
                retValue += checkData(externalDataObject);
            }

            if (!retValue.containsError())
            {
                ito::uint32 bufferWidth = m_frame.usWidth;
                ito::uint32 bufferHeight = m_frame.usHeight;
                ito::uint32 frameImgSize = bufferHeight * bufferWidth;

                if (m_data.getType() == ito::tUInt8)
                {
                    if (copyExternal)
                    {
                        retValue += externalDataObject->copyFromData2D<ito::uint8>((ito::uint8*) 
                            m_frame.pBuffer, bufferWidth, bufferHeight);
                    }
                    if (!copyExternal || hasListeners)
                    {
                        retValue += m_data.copyFromData2D<ito::uint8>(
                            (ito::uint8*)m_frame.pBuffer, bufferWidth, bufferHeight);
                    }
                }
                else if (m_data.getType() == ito::tUInt16)
                {
                    if (copyExternal)
                    {
                        retValue += externalDataObject->copyFromData2D<ito::uint16>(
                            (ito::uint16*)m_frame.pBuffer, bufferWidth, bufferHeight);
                    }
                    if (!copyExternal || hasListeners)
                    {
                        retValue += m_data.copyFromData2D<ito::uint16>(
                            (ito::uint16*)m_frame.pBuffer, bufferWidth, bufferHeight);
                    }
                }
                else
                {
                    retValue += ito::RetVal(ito::retError, 1002, tr("copying image buffer not possible since unsupported type.").toLatin1().data());
                }
            }
        }
    }

    m_isgrabbing = false;

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
// usually it is not necessary to implement the checkData method, since the default implementation from AddInGrabber is already
// sufficient.
//
// What is does:
// - it obtains the image size from sizex, sizey, bpp
// - it checks whether the rows, cols and type of m_data are unequal to the requested dimensions and type
// - if so, m_data is reallocated, else nothing is done
// - if an external data object is given (from copyVal), this object is checked in place of m_data
// - the external data object is only reallocated if it is empty, else its size or its region of interest must exactly
//    fit to the given size restrictions
//
// if you need to do further things, overload checkData and implement your version there
/*ito::RetVal MyGrabber::checkData(ito::DataObject *externalDataObject)
{
    return ito::retOk;
}*/


//----------------------------------------------------------------------------------------------------------------------------------
//! Returns the grabbed camera frame as reference.
/*!
    This method returns a reference to the recently acquired image. Therefore this camera size must fit to the data structure of the
    DataObject.

    This method returns a reference to the internal dataObject m_data of the camera where the currently acquired image data is copied to (either
    in the acquire method or in retrieve data). Please remember, that the reference may directly change if a new image is acquired.

    \param [in,out] vpdObj is the pointer to a given dataObject (this pointer should be cast to ito::DataObject*). After the call, the dataObject is a reference to the internal m_data dataObject of the camera.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if everything is ok, retError is camera has not been started or no image has been acquired by the method acquire.

    \sa retrieveImage, copyVal
*/
ito::RetVal Tucsen::getVal(void* vpdObj, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);

    //call retrieveData without argument. Retrieve data should then put the currently acquired image into the dataObject m_data of the camera.
    retValue += retrieveData();

    if (!retValue.containsError())
    {
        //send newly acquired image to possibly connected live images
        sendDataToListeners(0); //don't wait for live data, since user should get the data as fast as possible.

        if (dObj)
        {
            (*dObj) = m_data; //copy reference to externally given object
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
    This method copies the recently grabbed camera frame to the given DataObject.

    The given dataObject must either have an empty size (then it is resized to the size and type of the camera image) or its size or adjusted region of
    interest must exactly fit to the size of the camera. Then, the acquired image is copied inside of the given region of interest (copy into a subpart of
    an image stack is possible then)

    \param [in,out] vpdObj is the pointer to a given dataObject (this pointer should be cast to ito::DataObject*) where the acquired image is deep copied to.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if everything is ok, retError is camera has not been started or no image has been acquired by the method acquire.

    \sa retrieveImage, getVal
*/
ito::RetVal Tucsen::copyVal(void* vpdObj, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);

    if (!dObj)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Empty object handle retrieved from caller").toLatin1().data());
    }

    if (!retValue.containsError())
    {
        //this method calls retrieveData with the passed dataObject as argument such that retrieveData is able to copy the image obtained
        //by the camera directly into the given, external dataObject
        retValue += retrieveData(dObj);
    }

    if (!retValue.containsError())
    {
        //send newly acquired image to possibly connected live images
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
//! slot called if the dock widget of the plugin becomes (in)visible
/*!
    Overwrite this method if the plugin has a dock widget. If so, you can connect the parametersChanged signal of the plugin
    with the dock widget once its becomes visible such that no resources are used if the dock widget is not visible. Right after
    a re-connection emit parametersChanged(m_params) in order to send the current status of all plugin parameters to the dock widget.
*/
void Tucsen::dockWidgetVisibilityChanged(bool visible)
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
//! method called to show the configuration dialog
/*!
    This method is called from the main thread from itom and should show the configuration dialog of the plugin.
    If the instance of the configuration dialog has been created, its slot 'parametersChanged' is connected to the signal 'parametersChanged'
    of the plugin. By invoking the slot sendParameterRequest of the plugin, the plugin's signal parametersChanged is immediately emitted with
    m_params as argument. Therefore the configuration dialog obtains the current set of parameters and can be adjusted to its values.

    The configuration dialog should emit reject() or accept() depending if the user wanted to close the dialog using the ok or cancel button.
    If ok has been clicked (accept()), this method calls applyParameters of the configuration dialog in order to force the dialog to send
    all changed parameters to the plugin. If the user clicks an apply button, the configuration dialog itself must call applyParameters.

    If the configuration dialog is inherited from AbstractAddInConfigDialog, use the api-function apiShowConfigurationDialog that does all
    the things mentioned in this description.

    Remember that you need to implement hasConfDialog in your plugin and return 1 in order to signalize itom that the plugin
    has a configuration dialog.

    \sa hasConfDialog
*/
const ito::RetVal Tucsen::showConfDialog(void)
{
    //return apiShowConfigurationDialog(this, new DialogTucsen(this, &m_bppEnum/*, &m_triggerSourceEnum, &m_triggerActivationEnum*/));
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Tucsen::aliveTimer_fire()
{
    //this method is regularly called from m_awakeTimer if an acquisition is in process whose timeout is > 2sec.
    setAlive();
}

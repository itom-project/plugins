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

#include "avtVimba.h"
#include "pluginVersion.h"
#include "gitVersion.h"

#include <qstring.h>
#include <qstringlist.h>
#include <qplugin.h>
#include <qmessagebox.h>

#include "dockWidgetAvtVimba.h"

//----------------------------------------------------------------------------------------------------------------------------------
//! Constructor of Interface Class.
/*!
    \todo add necessary information about your plugin here.
*/

static char InitList[5] = {0, 0, 0, 0, 0};  /*!<A map with successfull initialized Cameras (max = 5) */
static char Initnum = 0;    /*!< Number of successfull initialized Cameras */ 

AvtVimbaInterface::AvtVimbaInterface()
{
    m_type = ito::typeDataIO | ito::typeGrabber; //any grabber is a dataIO device AND its subtype grabber (bitmask -> therefore the OR-combination).
    setObjectName("AVTVimba");

    m_description = QObject::tr("AVT GigE, firewire and USB cameras using Vimba interface");

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

    m_author = "J. Nitsche (IPROM Uni Braunschweig), M. Gronle (ITO Uni Stuttgart)";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("Licensed under LGPL");
    m_aboutThis = QObject::tr(GITVERSION); 

    //add mandatory and optional parameters for the initialization here.
    //append them to m_initParamsMand or m_initParamsOpt.
    ito::Param paramVal("CameraNumber", ito::ParamBase::Int, 0, 100, 0, "camera number");
    m_initParamsOpt.append(paramVal);

    paramVal = ito::Param("GigEPacketSize", ito::ParamBase::Int, 0, new ito::IntMeta(0, 8228, 1), "ethernet packet size (GigE cameras only). If 0, the camera tries to startup with a value of 8228 bytes/sec (network card must have jumbo frames enabled). Else set it to 1500 bytes/sec, which is a safe setting for all GigE Ethernet network cards");
    m_initParamsOpt.append(paramVal);
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Destructor of Interface Class.
/*!
    
*/
AvtVimbaInterface::~AvtVimbaInterface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvtVimbaInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(AvtVimba) //the argument of the macro is the classname of the plugin
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvtVimbaInterface::closeThisInst(ito::AddInBase **addInInst)
{
   REMOVE_PLUGININSTANCE(AvtVimba) //the argument of the macro is the classname of the plugin
   return ito::retOk;
}



//----------------------------------------------------------------------------------------------------------------------------------
//! Constructor of plugin.
/*!
    \todo add internal parameters of the plugin to the map m_params. It is allowed to append or remove entries from m_params
    in this constructor or later in the init method
*/
AvtVimba::AvtVimba() : 
    AddInGrabber(), 
    m_isgrabbing(false),  
    m_camera(CameraPtr()),
    m_aliveTimer(NULL),
    m_aliveTimerThread(NULL)
{
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "AVTVimba", tr("Name of plugin").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("interface", ito::ParamBase::String | ito::ParamBase::Readonly, "Unknown", tr("Interface type (Firewire, GigE)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("serial_no", ito::ParamBase::String | ito::ParamBase::Readonly, "Unknown", tr("Serial number").toLatin1().data());
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

    paramVal = ito::Param("binning", ito::ParamBase::Int | ito::ParamBase::In, 101, 101, 101, tr("binning (horizontal_factor * 100 + vertical_factor)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("timeout", ito::ParamBase::Double | ito::ParamBase::In, 0.0, std::numeric_limits<double>::max(), 2.0, tr("timeout for image acquisition in sec").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("sizex", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::In, 1, 2048, 2048, tr("width of ROI").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizey", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::In, 1, 2048, 2048, tr("height of ROI").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("camera_number", ito::ParamBase::Int | ito::ParamBase::In, 0, 10, 0, tr("Camera Number").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("bpp", ito::ParamBase::Int | ito::ParamBase::In, 8, 14, 8, tr("Bit depth of sensor").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("integration_time", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 60.0, 0.01, tr("Integrationtime of CCD [s]").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("offset", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 33.0, 0.0, tr("Offset as physical value that is a DC offset applied to the video signal. This values changes the blacklevel.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("gain", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 33.0, 0.0, tr("Gain of AD in dB, set it to 0.0 for best image quality.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("gain_auto", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("auto-controlled gain (0: off, 1: continuously varies the gain; gain will be read-only then)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("gamma", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 0.0, tr("Gamma value").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("stream_bps", ito::ParamBase::Int | ito::ParamBase::In, 1000000, 124000000, 124000000, tr("Bandwidth allocation for each camera. Must be adapted if multiple cameras are connected to the same ethernet adapter").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("packet_size", ito::ParamBase::Int | ito::ParamBase::In, 500, 16384, 8228, tr("Bandwidth allocation for each camera. Must be adapted if multiple cameras are connected to the same ethernet adapter").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("device_temperature", ito::ParamBase::Double | ito::ParamBase::Readonly | ito::ParamBase::In, 1.0, 100.0, 25.0, tr("device temperature of sensor in °C").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("trigger_mode", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("trigger mode (0: Off, 1: On)").toLatin1().data());
    m_params.insert(paramVal.getName(),paramVal);

    paramVal = ito::Param("trigger_source", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("trigger source (Freerun, Line1, Line2, Line3, Line4, FixedRate, Software, InputLines). Not all values are supported for all cameras.").toLatin1().data());
    m_params.insert(paramVal.getName(),paramVal);

    paramVal = ito::Param("trigger_activation", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("trigger activation (RisingEdge, FallingEdge, AnyEdge, LevelHigh, LevelLow). Not all values are supported for all cameras.").toLatin1().data());
    m_params.insert(paramVal.getName(),paramVal);

    //the following lines create and register the plugin's dock widget. Delete these lines if the plugin does not have a dock widget.
    DockWidgetAvtVimba *dw = new DockWidgetAvtVimba(this);
    
    Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
    QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
    createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dw);   
}

//----------------------------------------------------------------------------------------------------------------------------------
AvtVimba::~AvtVimba()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvtVimba::checkError(VmbErrorType errCode, const char* prefix /*= NULL*/)
{
    if (errCode == VmbErrorSuccess)
    {
        return ito::retOk;
    }
    else
    {
        const char *p = (prefix ? prefix : "Vimba API error");
        switch (errCode)
        {
        case VmbErrorApiNotStarted:
            return ito::RetVal::format(ito::retError,errCode, "%s: Vimba API not started.", p);
        case VmbErrorTimeout:
            return ito::RetVal::format(ito::retError,errCode, "%s: Timeout while acquiring image", p);
        case VmbErrorDeviceNotOpen:
            return ito::RetVal::format(ito::retError,errCode, "%s: Device was not opened for usage", p);
        default:
            return ito::RetVal::format(ito::retError,errCode, "%s: Undefined error %i.", p, errCode);
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
//! initialization of plugin
/*!
    \sa close
*/
ito::RetVal AvtVimba::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    std::string name, serialNumber, DeviceID;
    FeaturePtr pFeature;
    VmbInt64_t enum_idx;
    std::string enum_name;

    int packetSize = paramsOpt->at(1).getVal<int>();

    unsigned int cameraNumber = static_cast<unsigned int>(paramsOpt->at(0).getVal<int>());

    m_params["camera_number"].setVal<int>(cameraNumber);

    timeoutMS = sToMs(m_params["timeout"].getVal<double>());

    VimbaSystem& sys = VimbaSystem::GetInstance();
    CameraPtrVector cameras;

    //startup API
    if (Initnum==0)
    {
        retValue += checkError(sys.Startup());
    }
    if (InitList[cameraNumber]<1)
    {

        //obtain all cameras
        if (!retValue.containsError())
        {
            retValue += checkError(sys.GetCameras(cameras));
        }

        if (!retValue.containsError())
        {
            if (cameraNumber >= 0 && cameraNumber < cameras.size())
            {
                retValue += checkError(cameras[cameraNumber]->Open(VmbAccessModeFull));

                if (!retValue.containsError())
                {
                    ++Initnum;
                    ++InitList[cameraNumber];
                    m_camera = cameras[cameraNumber];
                    m_camera->GetName(name);
                    m_camera->GetSerialNumber(serialNumber);
                    m_camera->GetInterfaceID(DeviceID);
                    
                    m_camera->GetInterfaceType(m_interfaceType);

                    QString identifier = QString::fromStdString(name) + " (" + QString::fromStdString(serialNumber) + ") @ " + QString::fromStdString(DeviceID);
                    setIdentifier(identifier);

                    m_params["serial_no"].setVal<char*>( (char*)(serialNumber.data()) );

                    /*AVT::VmbAPI::FeaturePtrVector v;
                    std::string blub;
                    m_camera->GetFeatures(v);
                    for (int i = 0; i < v.size(); ++i)
                    {
                        v[i]->GetName(blub);
                        std::cout << blub.data() << "\n" << std::endl;
                    }*/
                    
                    switch (m_interfaceType)
                    {
                    case VmbInterfaceEthernet:
                        m_params["interface"].setVal<const char*>("GigE");
                        break;
                    case VmbInterfaceFirewire:
                        m_params["interface"].setVal<const char*>("Firewire");
                        break;
                    case VmbInterfaceUsb:
                        m_params["interface"].setVal<const char*>("USB");
                        break;
                    default:
                        retValue += ito::RetVal(ito::retError, 0, "unknown or unsupported transport type (GigE, Firewire...)");
                        break;
                    }
                }

                if (!retValue.containsError())
                {
                    AVT::VmbAPI::FeaturePtr feature;
                        if (m_camera->GetFeatureByName("ExposureTime", feature) == VmbErrorSuccess)
                        {
                            nameExposureTime = "ExposureTime";
                        }
                        else if (m_camera->GetFeatureByName("ExposureTimeAbs", feature) == VmbErrorSuccess)
                        {
                            nameExposureTime = "ExposureTimeAbs";
                        }
                        else
                        {
                            retValue += ito::RetVal(ito::retError, 0, "Feature 'ExposureTime' or 'ExposureTimeAbs' not available");
                            nameExposureTime = "";
                        }

                    if (m_interfaceType == VmbInterfaceEthernet)
                    {
                        //some defaults (they are not changed in this plugin)
                        setEnumFeature("ExposureAuto", "Off");
                        setEnumFeature("ExposureMode", "Timed");
                        setEnumFeature("TriggerSelector", "FrameStart"); //if the trigger source is changed, it changes the trigger for starting an exposure
                        setEnumFeature("TriggerMode", "On"); //trigger off makes no sense
                        setEnumFeature("BlackLevelSelector", "All");

                        if (packetSize == 0)
                        {
                            packetSize = 8228;
                        }
                        retValue += setIntFeature("GVSPPacketSize", packetSize);
                    }
                    else if (m_interfaceType == VmbInterfaceFirewire)
                    {
                        //some defaults (they are not changed in this plugin)
                        setEnumFeature("TriggerMode", "Off"); //trigger off makes no sense
                        setEnumFeature("ExposureAuto", "Off");
                        setEnumFeature("ExposureMode", "Timed");
                        setEnumFeature("TriggerSelector", "ExposureStart"); //if the trigger source is changed, it changes the trigger for starting an exposure
                        setEnumFeature("BlackLevelSelector", "All");

                        //remove unused parameters
                        m_params.remove("stream_bps");
                        m_params.remove("packet_size");
                        m_params.remove("device_temperature");

                        retValue += getEnumFeatureByName("TriggerSelector", enum_name, enum_idx);
                    }
                    else if (m_interfaceType == VmbInterfaceUsb)
                    {
                        // some defaults (they are not changed in this plugin)
                        setEnumFeature("TriggerMode", "Off"); // trigger off makes no sense
                        setEnumFeature("ExposureAuto", "Off");
                        setEnumFeature("ExposureMode", "Timed");
                        setEnumFeature(
                            "TriggerSelector",
                            "ExposureStart"); // if the trigger source is changed, it changes the
                                              // trigger for starting an exposure
                        setEnumFeature("BlackLevelSelector", "All");

                        // remove unused parameters
                        m_params.remove("stream_bps");
                        m_params.remove("packet_size");

                        retValue += getEnumFeatureByName("TriggerSelector", enum_name, enum_idx);

                    }
                }

                if (!retValue.containsError())
                {
                    //GET PIXELFORMAT ENUMERATION
                    retValue += checkError(m_camera->GetFeatureByName("PixelFormat", pFeature));
                    if (!retValue.containsError())
                    {
                        AVT::VmbAPI::StringVector pixelTypesStr;
                        AVT::VmbAPI::Int64Vector pixelTypesIdx;
                        int minBpp = 16;
                        int maxBpp = 8;
                        bool available;

                        //due to a official bug in Vimba 1.3.0 with GetEntries(...) the workaround with GetValues must be used.
                        if (pFeature->GetValues(pixelTypesStr) == VmbErrorSuccess && pFeature->GetValues(pixelTypesIdx) == VmbErrorSuccess)
                        {
                            for (AVT::VmbAPI::StringVector::size_type i = 0; i < pixelTypesStr.size(); ++i)
                            {
                                pFeature->IsValueAvailable(pixelTypesIdx[i], available);
                                if (available)
                                {
                                    enum_name = pixelTypesStr[i];
                                    enum_idx = pixelTypesIdx[i];

                                    if (enum_name == "Mono8")
                                    {
                                        minBpp = std::min(8, minBpp);
                                        maxBpp = std::max(8, maxBpp);
                                        m_bppEnum.bppMono8 = enum_idx;
                                    }
                                    else if (enum_name == "Mono10")
                                    {
                                        minBpp = std::min(10, minBpp);
                                        maxBpp = std::max(10, maxBpp);
                                        m_bppEnum.bppMono10 = enum_idx;
                                    }
                                    else if (enum_name == "Mono12")
                                    {
                                        minBpp = std::min(12, minBpp);
                                        maxBpp = std::max(12, maxBpp);
                                        m_bppEnum.bppMono12 = enum_idx;
                                    }
                                    else if (enum_name == "Mono14")
                                    {
                                        minBpp = std::min(14, minBpp);
                                        maxBpp = std::max(14, maxBpp);
                                        m_bppEnum.bppMono14 = enum_idx;
                                    }
                                }
                            }
                        }

                        if (minBpp > maxBpp)
                        {
                            retValue += ito::RetVal(ito::retError, 0, "error determining the possible bit depths. No monochrome bitdepths are available");
                        }
                        else
                        {
                            enum_idx = m_bppEnum.bppMono14;
                            if (enum_idx == -1) enum_idx = m_bppEnum.bppMono12;
                            if (enum_idx == -1) enum_idx = m_bppEnum.bppMono10;
                            if (enum_idx == -1) enum_idx = m_bppEnum.bppMono8;
                            retValue += setEnumFeature("PixelFormat", enum_idx);
                            m_params["bpp"].setMeta(new ito::IntMeta(minBpp, maxBpp, 2), true);
                        }
                    }
                }

                //GET TRIGGER SOURCE ENUMERATION
                retValue += checkError(m_camera->GetFeatureByName("TriggerSource", pFeature));
                if (!retValue.containsError())
                {
                    AVT::VmbAPI::StringVector triggerSourceStr;
                    AVT::VmbAPI::Int64Vector triggerSourceIdx;
                    bool available;
                    ito::StringMeta *sm = new ito::StringMeta(ito::StringMeta::String);

                    //due to a official bug in Vimba 1.3.0 with GetEntries(...) the workaround with GetValues must be used.
                    if (pFeature->GetValues(triggerSourceStr) == VmbErrorSuccess && pFeature->GetValues(triggerSourceIdx) == VmbErrorSuccess)
                    {
                        for (AVT::VmbAPI::StringVector::size_type i = 0; i < triggerSourceStr.size(); ++i)
                        {
                            pFeature->IsValueAvailable(triggerSourceIdx[i], available);
                            if (available)
                            {
                                enum_name = triggerSourceStr[i];
                                enum_idx = triggerSourceIdx[i];
                                sm->addItem(enum_name.data());

                                if (enum_name == "Freerun")
                                {
                                    m_triggerSourceEnum.triggerFreerun = enum_idx;
                                }
                                else if (enum_name == "Line1")
                                {
                                    m_triggerSourceEnum.triggerLine1 = enum_idx;
                                }
                                else if (enum_name == "Line2")
                                {
                                    m_triggerSourceEnum.triggerLine2 = enum_idx;
                                }
                                else if (enum_name == "Line3")
                                {
                                    m_triggerSourceEnum.triggerLine3 = enum_idx;
                                }
                                else if (enum_name == "Line4")
                                {
                                    m_triggerSourceEnum.triggerLine4 = enum_idx;
                                }
                                else if (enum_name == "FixedRate")
                                {
                                    m_triggerSourceEnum.triggerFixedRate = enum_idx;
                                }
                                else if (enum_name == "Software")
                                {
                                    m_triggerSourceEnum.triggerSoftware = enum_idx;
                                }
                                else if (enum_name == "InputLines")
                                {
                                    m_triggerSourceEnum.triggerInputLines = enum_idx;
                                }
                            }
                        }

                        if (m_triggerSourceEnum.triggerSoftware != -1)
                        {
                            retValue += setEnumFeature("TriggerSource", m_triggerSourceEnum.triggerSoftware);
                        }
                    }

                    m_params["trigger_source"].setMeta(sm, true);
                }

                //GET TRIGGER ACTIVATION ENUMERATION
                retValue += checkError(m_camera->GetFeatureByName("TriggerActivation", pFeature));
                if (!retValue.containsError())
                {
                    AVT::VmbAPI::StringVector triggerActivationStr;
                    AVT::VmbAPI::Int64Vector triggerActivationIdx;
                    bool available;
                    ito::StringMeta *sm = new ito::StringMeta(ito::StringMeta::String);

                    //due to a official bug in Vimba 1.3.0 with GetEntries(...) the workaround with GetValues must be used.
                    if (pFeature->GetValues(triggerActivationStr) == VmbErrorSuccess && pFeature->GetValues(triggerActivationIdx) == VmbErrorSuccess)
                    {
                        for (AVT::VmbAPI::StringVector::size_type i = 0; i < triggerActivationStr.size(); ++i)
                        {
                            pFeature->IsValueAvailable(triggerActivationIdx[i], available);
                            if (available)
                            {
                                enum_name = triggerActivationStr[i];
                                enum_idx = triggerActivationIdx[i];
                                sm->addItem(enum_name.data());

                                if (enum_name == "RisingEdge")
                                {
                                    m_triggerActivationEnum.taRisingEdge = enum_idx;
                                }
                                else if (enum_name == "FallingEdge")
                                {
                                    m_triggerActivationEnum.taFallingEdge = enum_idx;
                                }
                                else if (enum_name == "AnyEdge")
                                {
                                    m_triggerActivationEnum.taAnyEdge = enum_idx;
                                }
                                else if (enum_name == "LevelHigh")
                                {
                                    m_triggerActivationEnum.taLevelHigh = enum_idx;
                                }
                                else if (enum_name == "LevelLow")
                                {
                                    m_triggerActivationEnum.taLevelLow = enum_idx;
                                }
                            }
                        }
                    }

                    m_params["trigger_activation"].setMeta(sm, true);
                }


                if (!retValue.containsError())
                {
                    retValue += synchronizeParameters(fAll);
                }
            }
            else
            {
                retValue += ito::RetVal::format(ito::retError,0,"cameraNumber %i is out of range [0,%i]", cameraNumber, cameras.size()-1);
            }
        }
    }
    else
    {
        retValue += ito::RetVal::format(ito::retError,0,"cameraNumber %i already opened", cameraNumber);
    }
    if (!retValue.containsError())
    {
        //// Set the GeV packet size to the highest possible value
  //      // (In this example we do not test whether this cam actually is a GigE cam)
  //      FeaturePtr pCommandFeature;
  //      if ( VmbErrorSuccess == m_camera->GetFeatureByName( "GVSPAdjustPacketSize", pCommandFeature ))
  //      {
  //          if ( VmbErrorSuccess == pCommandFeature->RunCommand() )
  //          {
  //              bool bIsCommandDone = false;
  //              do
  //              {
  //                  if ( VmbErrorSuccess != pCommandFeature->IsCommandDone( bIsCommandDone ))
  //                  {
  //                      break;
  //                  }
  //              } while ( false == bIsCommandDone );
  //          }
  //      }
    }
        
    
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
ito::RetVal AvtVimba::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    
    //todo:
    // - disconnect the device if not yet done
    // - this funtion is considered to be the "inverse" of init.

    int nr = m_params["camera_number"].getVal<int>();
    

    if (m_camera.get())
    {
        retValue += checkError(m_camera->Close());
        m_camera = CameraPtr();
        InitList[nr] = 0;
        --Initnum;
    }
    if (Initnum<1)
    {
        VimbaSystem& sys = VimbaSystem::GetInstance();
    

        //shutdown API
        retValue += checkError(sys.Shutdown());
    }

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
ito::RetVal AvtVimba::getIntFeatureByName(const char *name, VmbInt64_t &value)
{
    ito::RetVal retValue;
    FeaturePtr pFeature;

    retValue += checkError(m_camera->GetFeatureByName(name, pFeature), name);

    if (!retValue.containsError())
    {
        retValue += checkError(pFeature->GetValue(value), name);
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal  AvtVimba::getIntFeatureByName(const char *name, VmbInt64_t &value, VmbInt64_t &max, VmbInt64_t &min, VmbInt64_t &inc)
{
    ito::RetVal retValue;
    FeaturePtr pFeature;

    retValue += checkError(m_camera->GetFeatureByName(name, pFeature), name);

    if (!retValue.containsError())
    {
        retValue += checkError(pFeature->GetValue(value), name);
        retValue += checkError(pFeature->GetRange(min, max), name);
        retValue += checkError(pFeature->GetIncrement(inc), name);
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvtVimba::getDblFeatureByName(const char *name, double &value)
{
    ito::RetVal retValue;
    FeaturePtr pFeature;

    retValue += checkError(m_camera->GetFeatureByName(name, pFeature), name);

    if (!retValue.containsError())
    {
        retValue += checkError(pFeature->GetValue(value), name);
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvtVimba::getDblFeatureByName(const char *name, double &value, double &min, double &max)
{
    ito::RetVal retValue;
    FeaturePtr pFeature;

    retValue += checkError(m_camera->GetFeatureByName(name, pFeature), name);

    if (!retValue.containsError())
    {
        retValue += checkError(pFeature->GetValue(value), name);
        retValue += checkError(pFeature->GetRange(min, max), name);
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvtVimba::getEnumFeatureByName(const char *name, std::string &value, VmbInt64_t &idx)
{
    ito::RetVal retValue;
    FeaturePtr pFeature;

    retValue += checkError(m_camera->GetFeatureByName(name, pFeature), name);

    if (!retValue.containsError())
    {
        retValue += checkError(pFeature->GetValue(value), name);
        retValue += checkError(pFeature->GetValue(idx), name);
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvtVimba::setDblFeature(const char *name, const double &fValue)
{
    FeaturePtr pFeature;
    ito::RetVal retValue;

    retValue += checkError(m_camera->GetFeatureByName(name, pFeature), name);

    if (!retValue.containsError())
        {
            retValue += checkError(pFeature->SetValue(fValue), name);
            if (!retValue.containsError())
            {
                //std::cout << "Feature " << name << " set to " << fValue << std::endl;
            }    
        }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvtVimba::setIntFeature(const char *name, const int &iValue)
{
    FeaturePtr pFeature;
    ito::RetVal retValue;

    retValue += checkError(m_camera->GetFeatureByName(name, pFeature), name);

    if (!retValue.containsError())
        {
            retValue += checkError(pFeature->SetValue(iValue), name);
            if (!retValue.containsError())
            {
                //std::cout << "Feature " << name << " set to " << iValue << std::endl;
            }    
        }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvtVimba::setEnumFeature(const char *name, const char *eValue)
{
    FeaturePtr pFeature;
    ito::RetVal retValue;

    retValue += checkError(m_camera->GetFeatureByName(name, pFeature), name);

    if (!retValue.containsError())
        {
            retValue += checkError(pFeature->SetValue(eValue), name);
        }
    
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvtVimba::setEnumFeature(const char *name, VmbInt64_t value)
{
    FeaturePtr pFeature;
    ito::RetVal retValue;

    retValue += checkError(m_camera->GetFeatureByName(name, pFeature), name);

    if (!retValue.containsError())
        {
            retValue += checkError(pFeature->SetValue(value), name);
        }
    
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvtVimba::getRange(const char *name, VmbInt64_t &max, VmbInt64_t &min, VmbInt64_t &inc)
{
    ito::RetVal retValue;
    FeaturePtr pFeature;

    retValue += checkError(m_camera->GetFeatureByName(name, pFeature), name);

    if (!retValue.containsError())
    {
        retValue += checkError(pFeature->GetRange(min,max), name);
        retValue += checkError(pFeature->GetIncrement(inc), name);
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvtVimba::getRange(const char *name, double &max, double &min)
{
    ito::RetVal retValue;
    FeaturePtr pFeature;

    retValue += checkError(m_camera->GetFeatureByName(name, pFeature), name);

    if (!retValue.containsError())
    {
        retValue += checkError(pFeature->GetRange(min,max), name);
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvtVimba::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
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
            double tmpDouble;
            retValue += getDblFeatureByName("DeviceTemperature",tmpDouble);
            if (!retValue.containsError())
            {
                retValue+= it->setVal<double>(tmpDouble);
            }
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
ito::RetVal AvtVimba::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex;
    int index;
    QString suffix;
    FeaturePtr pFeature;
    ParamMapIterator it;
    VmbInt64_t enum_idx;

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
            retValue += setDblFeature(nameExposureTime,sToMus(val->getVal<double>()));
            if (!retValue.containsError())
            {
                retValue += synchronizeParameters(fExposure);
            }
        }
        else if (key == "timeout")
        {
            timeoutMS = sToMs(val->getVal<double>());
            retValue += it->copyValueFrom( &(*val) );
        }
        else if (key == "bpp")
        {
            enum_idx = -1;
            switch (val->getVal<int>())
            {
            case 8:
                enum_idx = m_bppEnum.bppMono8;
                break;
            case 10:
                enum_idx = m_bppEnum.bppMono10;
                break;
            case 12:
                enum_idx = m_bppEnum.bppMono12;
                break;
            case 14:
                enum_idx = m_bppEnum.bppMono14;
                break;
            }

            if (enum_idx > -1)
            {
                retValue += setEnumFeature("PixelFormat", enum_idx);
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 0, "non supported bpp");
            }

            if (!retValue.containsError())
            {
                retValue += synchronizeParameters(fBpp);
            }
        }
        else if (key == "binning")
        {
            int vBin = val->getVal<int>() % 100;
            int hBin  = (val->getVal<int>() - vBin) / 100;
            retValue += setIntFeature("BinningHorizontal", hBin);
            retValue += setIntFeature("BinningVertical", vBin);
            retValue += synchronizeParameters(fSize | fBinning);
        }
        else if (key == "offset")
        {
            retValue += setDblFeature("BlackLevel", val->getVal<double>());
            retValue += synchronizeParameters(fOffset);
        }
        else if (key == "gain")
        {
            retValue += setDblFeature("Gain", val->getVal<double>());
            retValue += synchronizeParameters(fGain);
        }
        else if (key == "gain_auto")
        {
            retValue += setEnumFeature("GainAuto", val->getVal<int>() == 0 ? "Off" : "Continuous");
            retValue += synchronizeParameters(fGain);
        }
        else if (key == "gamma")
        {
            retValue += setDblFeature("Gamma", val->getVal<double>());
            retValue += synchronizeParameters(fGamma);
        }
        else if (key == "stream_bps")
        {
            retValue += setIntFeature("StreamBytesPerSecond", val->getVal<int>());
            retValue += synchronizeParameters(fGigETransport);
        }
        else if (key == "packet_size")
        {
            retValue += setIntFeature("GVSPPacketSize", val->getVal<int>());
            retValue += synchronizeParameters(fGigETransport);
        }
        else if (key == "trigger_mode")
        {
            retValue += setEnumFeature("TriggerMode", val->getVal<int>() == 0 ? "Off" : "On");
            retValue += synchronizeParameters(fTrigger);
        }
        else if (key == "trigger_source")
        {
            retValue += setEnumFeature("TriggerSource", val->getVal<char*>());
            retValue += synchronizeParameters(fTrigger);
        }
        else if (key == "trigger_activation")
        {
            retValue += setEnumFeature("TriggerActivation", val->getVal<char*>());
            retValue += synchronizeParameters(fTrigger);
        }
        
        else if (key == "roi")
        {
            if (!hasIndex)
            {
                const int* old_roi = it->getVal<const int*>();
                const int* roi = val->getVal<const int*>();

                if (old_roi[0] >= roi[0])
                {
                    //offset is decreased, do it first, then width
                    retValue += setIntFeature("OffsetX", roi[0]);
                    retValue += setIntFeature("Width", roi[2]);
                }
                else
                {
                    //offset is increased, decrease width at first, then increase offset
                    retValue += setIntFeature("Width", roi[2]);
                    retValue += setIntFeature("OffsetX", roi[0]);  
                }

                if (old_roi[1] >= roi[1])
                {
                    //offset is decreased, do it first, then width
                    retValue += setIntFeature("OffsetY", roi[1]);
                    retValue += setIntFeature("Height", roi[3]);
                }
                else
                {
                    //offset is increased, decrease width at first, then increase offset
                    retValue += setIntFeature("Height", roi[3]);
                    retValue += setIntFeature("OffsetY", roi[1]);
                }
            }
            else
            {
                switch (index)
                {
                case 0:
                    retValue += setIntFeature("OffsetX", val->getVal<int>());
                    break;
                case 1:
                    retValue += setIntFeature("OffsetY", val->getVal<int>());
                    break;
                case 2:
                    retValue += setIntFeature("Width", val->getVal<int>());
                    break;
                case 3:
                    retValue += setIntFeature("Height", val->getVal<int>());
                    break;
                }
            }

            retValue += synchronizeParameters(fSize);
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
ito::RetVal AvtVimba::startDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    
    incGrabberStarted(); //increment a counter to see how many times startDevice has been called

    /*if (grabberStartedCount() == 1)
    {
        retValue += checkError(m_camera->StartCapture());
    }*/
    
    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}
         
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvtVimba::stopDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    decGrabberStarted(); //decrements the counter (see startDevice)

    if (grabberStartedCount() < 0)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("The grabber has already been stopped.").toLatin1().data());
        setGrabberStarted(0);
    }
    /*else if (grabberStartedCount() == 0)
    {
        retValue += checkError(m_camera->EndCapture());
    }*/

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return ito::retOk;
}
         
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvtVimba::acquire(const int trigger, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    bool RetCode = false;

    if (grabberStartedCount() <= 0)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Tried to acquire an image without having started the device.").toLatin1().data());
    }
    else
    {
        m_isgrabbing = true;
        AVT::VmbAPI::FeaturePtr feature;
        if (m_camera->GetFeatureByName("TriggerSoftware", feature) == VmbErrorSuccess)
        {
            retValue += checkError(feature->RunCommand());
        }

    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();  
    }
    
    if (timeoutMS > 2000)
    {
        QMetaObject::invokeMethod(m_aliveTimer, "start");
        m_acquisitionStatus = checkError(m_camera->AcquireSingleImage(m_frame, timeoutMS));
        QMetaObject::invokeMethod(m_aliveTimer, "stop");
    }
    else
    {
        m_acquisitionStatus = checkError(m_camera->AcquireSingleImage(m_frame, timeoutMS));
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvtVimba::retrieveData(ito::DataObject *externalDataObject)
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
                VmbUint32_t frameImgSize, bufferHeight, bufferWidth;
                VmbUchar_t *bufferPtr;
                m_frame->GetImageSize(frameImgSize);
                m_frame->GetImage(bufferPtr);
                m_frame->GetHeight(bufferHeight);
                m_frame->GetWidth(bufferWidth);

                if (m_data.getType() == ito::tUInt8)
                {
                    if (copyExternal)
                    {
                        retValue += externalDataObject->copyFromData2D<ito::uint8>((ito::uint8*) bufferPtr, bufferWidth, bufferHeight);
                    }
                    if (!copyExternal || hasListeners)
                    {
                        retValue += m_data.copyFromData2D<ito::uint8>((ito::uint8*) bufferPtr, bufferWidth, bufferHeight);
                    }
                }
                else if (m_data.getType() == ito::tUInt16)
                {
                    if (copyExternal)
                    {
                        retValue += externalDataObject->copyFromData2D<ito::uint16>((ito::uint16*) bufferPtr, bufferWidth, bufferHeight);
                    }
                    if (!copyExternal || hasListeners)
                    {
                        retValue += m_data.copyFromData2D<ito::uint16>((ito::uint16*) bufferPtr, bufferWidth, bufferHeight);            
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
ito::RetVal AvtVimba::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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
ito::RetVal AvtVimba::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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
void AvtVimba::dockWidgetVisibilityChanged(bool visible)
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
    all changed parameters to the plugin. If the user clicks an apply button, the configuration dialog itsself must call applyParameters.
    
    If the configuration dialog is inherited from AbstractAddInConfigDialog, use the api-function apiShowConfigurationDialog that does all
    the things mentioned in this description.
    
    Remember that you need to implement hasConfDialog in your plugin and return 1 in order to signalize itom that the plugin
    has a configuration dialog.
    
    \sa hasConfDialog
*/
const ito::RetVal AvtVimba::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogAvtVimba(this, &m_bppEnum/*, &m_triggerSourceEnum, &m_triggerActivationEnum*/));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvtVimba::synchronizeParameters(int features)
{
    ito::RetVal retval, ret_;
    VmbInt64_t enumIdx;
    std::string enumVal;

    if (features & AvtVimba::fBpp)
    {
        ret_ += getEnumFeatureByName("PixelFormat", enumVal, enumIdx);
        if (!ret_.containsError())
        {
            if (enumIdx == m_bppEnum.bppMono8)
            {
                m_params["bpp"].setVal<int>(8);
            }
            if (enumIdx == m_bppEnum.bppMono10)
            {
                m_params["bpp"].setVal<int>(10);
            }
            if (enumIdx == m_bppEnum.bppMono12)
            {
                m_params["bpp"].setVal<int>(12);
            }
            if (enumIdx == m_bppEnum.bppMono14)
            {
                m_params["bpp"].setVal<int>(14);
            }
        }

        retval += ret_;
    }

    if (features & fSize)
    {
        VmbInt64_t x0, x0_min, x0_max, x0_inc;
        VmbInt64_t y0, y0_min, y0_max, y0_inc;
        VmbInt64_t w, w_min, w_max, w_inc;
        VmbInt64_t h, h_min, h_max, h_inc;
        VmbInt64_t width_max, height_max;

        ret_ = getIntFeatureByName("OffsetX", x0, x0_max, x0_min, x0_inc);
        ret_ += getIntFeatureByName("OffsetY", y0, y0_max, y0_min, y0_inc);
        ret_ += getIntFeatureByName("Width", w, w_max, w_min, w_inc);
        ret_ += getIntFeatureByName("Height", h, h_max, h_min, h_inc);
        ret_ += getIntFeatureByName("WidthMax", width_max);
        ret_ += getIntFeatureByName("HeightMax", height_max);

        if (!ret_.containsError())
        {
            ParamMapIterator it = m_params.find("roi");
            if (it != m_params.end())
            {
                ito::RectMeta roiMeta(\
                    ito::RangeMeta(x0_min, width_max - 1, x0_inc, w_min, width_max - x0_min, w_inc), \
                    ito::RangeMeta(y0_min, height_max - 1, y0_inc, h_min, height_max - y0_min, h_inc));
                it->setMeta(&roiMeta, false);
                int roi[] = {x0, y0, w, h};
                it->setVal<int*>(roi, 4);
            }
            else
            {
                m_params["x0"].setMeta(new ito::IntMeta(x0_min, x0 + w - 2, x0_inc), true);
                m_params["x0"].setVal<int>(x0);

                m_params["y0"].setMeta(new ito::IntMeta(y0_min, y0 + h - 2, y0_inc), true);
                m_params["y0"].setVal<int>(y0);

                m_params["x1"].setMeta(new ito::IntMeta(x0 + w_min - 1, width_max - 1, w_inc), true);
                m_params["x1"].setVal<int>(x0 + w - 1);

                m_params["y1"].setMeta(new ito::IntMeta(y0 + h_min - 1, height_max - 1, h_inc), true);
                m_params["y1"].setVal<int>(y0 + h- 1);
            }

            m_params["sizex"].setMeta(new ito::IntMeta(w_min, width_max, w_inc), true);
            m_params["sizex"].setVal<int>(w);

            m_params["sizey"].setMeta(new ito::IntMeta(h_min, height_max, h_inc), true);
            m_params["sizey"].setVal<int>(h);
        }

        retval += ret_;
    }

    if (features & fBinning)
    {
        VmbInt64_t binH, binH_max, binH_min, binH_inc;
        VmbInt64_t binV, binV_max, binV_min, binV_inc;
        AVT::VmbAPI::FeaturePtr feature;
        if (m_camera->GetFeatureByName("horizontalBinning", feature) == VmbErrorNotFound)
        {
            m_params["binning"].setFlags(ito::ParamBase::Readonly);
        }
        else
        {
            ret_ += getIntFeatureByName("BinningHorizontal", binH, binH_max, binH_min, binH_inc);
            ret_ += getIntFeatureByName("BinninVertical", binV, binV_max, binV_min, binV_inc);

            //some cameras don't have any binning parameters
            if (!ret_.containsError())
            {
                m_params["binning"].setMeta(new ito::IntMeta(binH_min*100+binV_min, binH_max*100+binH_max, binV_inc), true);
                m_params["binning"].setVal<int>(binH*100+binV);
                m_params["binning"].setFlags(0);
            }
        }            

        retval += ret_;
    }

    if (features & fExposure)
    {
        double val, min, max;
        ret_ = getDblFeatureByName(nameExposureTime, val, min, max);
        if (!ret_.containsError())
        {
            ParamMapIterator it = m_params.find("integration_time");
            it->setMeta(new ito::DoubleMeta(musToS(min), musToS(max)), true);
            it->setVal<double>(musToS(val));
        }
        retval += ret_;
    }

    if ((features & fGigETransport) && m_interfaceType == VmbInterfaceEthernet)
    {
        VmbInt64_t intVal, intMin, intMax, intInc;
        ret_ = getIntFeatureByName("StreamBytesPerSecond",intVal, intMax, intMin, intInc);
        ParamMapIterator it = m_params.find("stream_bps");
        if (!ret_.containsError())
        {
            it->setMeta(new ito::IntMeta(intMin, intMax, intInc), true);
            it->setVal<int>(intVal);
        }

        ret_ = getIntFeatureByName("GVSPPacketSize",intVal, intMax, intMin, intInc);
        it = m_params.find("packet_size");
        if (!ret_.containsError())
        {
            it->setMeta(new ito::IntMeta(intMin, intMax, intInc), true);
            it->setVal<int>(intVal);
        }
    }

    if (features & fTrigger)
    {
        ret_ = getEnumFeatureByName("TriggerMode", enumVal, enumIdx);
        if (!ret_.containsError())
        {
            m_params["trigger_mode"].setVal<int>( enumVal == "On" ? 1 : 0 );
        }
        retval += ret_;

        ret_ = getEnumFeatureByName("TriggerSource", enumVal, enumIdx);
        if (!ret_.containsError())
        {
            m_params["trigger_source"].setVal<char*>((char*)(enumVal.data()));
        }
        retval += ret_;

        ret_ = getEnumFeatureByName("TriggerActivation", enumVal, enumIdx);
        if (!ret_.containsError())
        {
            m_params["trigger_activation"].setVal<char*>((char*)(enumVal.data()));
        }
        retval += ret_;
    }

    if (features & fGain)
    {
        ret_ += getEnumFeatureByName("GainAuto", enumVal, enumIdx);
        if (!ret_.containsError())
        {
            if (enumVal == "Off")
            {
                m_params["gain"].setFlags(0);
                m_params["gain_auto"].setVal<int>(0);
            }
            else if (enumVal == "Continuous")
            {
                m_params["gain"].setFlags(ito::ParamBase::Readonly);
                m_params["gain_auto"].setVal<int>(1);
            }
            else
            {
                if (!setEnumFeature("GainAuto", "Off").containsError())
                {
                    m_params["gain"].setFlags(0);
                    m_params["gain_auto"].setVal<int>(0);
                }
                else
                {
                    retval += ito::RetVal(ito::retError, 0, "error setting gainAuto to Off");
                }
            }
        }
		else
		{
			m_params["gain_auto"].setFlags(ito::ParamBase::Readonly);
			ret_ = ito::retOk;
		}
        retval += ret_;

        double gain, gainMax, gainMin;
        ret_ = getDblFeatureByName("Gain", gain, gainMin, gainMax);
        if (!ret_.containsError())
        {
            ((ito::DoubleMeta*)(m_params["gain"].getMeta()))->setMin(gainMin);
            ((ito::DoubleMeta*)(m_params["gain"].getMeta()))->setMax(gainMax);
            m_params["gain"].setVal<double>(gain);
        }
		else
		{
			m_params["gain"].setFlags(ito::ParamBase::Readonly);
			ret_ = ito::retOk;
		}
        retval += ret_;
    }

    if (features & fOffset)
    {
        double offset, offsetMax, offsetMin;
        ret_ += getDblFeatureByName("BlackLevel", offset, offsetMin, offsetMax);
        if (!ret_.containsError())
        {
            ParamMapIterator it = m_params.find("offset");
            ((ito::DoubleMeta*)(it->getMeta()))->setMin(offsetMin);
            ((ito::DoubleMeta*)(it->getMeta()))->setMax(offsetMax);
            it->setVal<double>(offset);
        }
		else
		{
			m_params["offset"].setFlags(ito::ParamBase::Readonly);
			ret_ = ito::retOk;
		}
        retval += ret_;
    }

    if (features & fGamma)
    {
        double offset, offsetMax, offsetMin;
        ret_ += getDblFeatureByName("Gamma", offset, offsetMin, offsetMax);
        if (!ret_.containsError())
        {
            ParamMapIterator it = m_params.find("gamma");
            ((ito::DoubleMeta*)(it->getMeta()))->setMin(offsetMin);
            ((ito::DoubleMeta*)(it->getMeta()))->setMax(offsetMax);
            it->setVal<double>(offset);
        }
		else
		{
			m_params["gamma"].setFlags(ito::ParamBase::Readonly);
			ret_ = ito::retOk;
		}
        retval += ret_;
    }

    retval += checkData();

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
void AvtVimba::aliveTimer_fire()
{
    //this method is regularly called from m_awakeTimer if an acquisition is in process whose timeout is > 2sec.
    setAlive();
}
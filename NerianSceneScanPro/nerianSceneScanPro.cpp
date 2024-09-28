/* ********************************************************************
Plugin "NerianSceneScanPro" for itom software
URL: http://www.uni-stuttgart.de/ito
Copyright (C) 2019, Institut für Technische Optik (ITO),
Universität Stuttgart, Germany

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

#include "nerianSceneScanPro.h"
#include "pluginVersion.h"
#include "gitVersion.h"

#include <qstring.h>
#include <qvector.h>
#include <qstringlist.h>
#include <qplugin.h>
#include <qmessagebox.h>
#include <visiontransfer/deviceenumeration.h>
#include <visiontransfer/deviceparameters.h>
//include all 3rd Party which come with openCV functions if CV_MAJOR_VERSION is defined. We won't use those functions since this would force the same openCV version of visiontransfer and itom
#include<visiontransfer/imagetransfer.h>
#include <visiontransfer/imagepair.h>



#include "dockWidgetNerianSceneScanPro.h"

#ifdef _MSC_VER
// Visual studio does not come with snprintf
#define snprintf _snprintf_s
#endif
using namespace visiontransfer;
//----------------------------------------------------------------------------------------------------------------------------------
//! Constructor of Interface Class.
/*!
    \todo add necessary information about your plugin here.
*/
NerianSceneScanProInterface::NerianSceneScanProInterface()
{
    m_type = ito::typeDataIO | ito::typeGrabber; //any grabber is a dataIO device AND its subtype grabber (bitmask -> therefore the OR-combination).
    setObjectName("NerianSceneScanPro");

    m_description = QObject::tr("NerianSceneScanPro");

    //for the docstring, please don't set any spaces at the beginning of the line.
    char docstring[] = \
"This template can be used to control a Nerian SceneScan Pro device. This plugin uses the Nerian Visiontransfer SDK. The typical IP of a Nerian Scene Scan is 192.168.10\n\
\n\
The device has a web interface which allows access to further parameters.The interface can be accessed under the device ip";
    m_detaildescription = QObject::tr(docstring);

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
    m_aboutThis = QObject::tr(GITVERSION);

    m_initParamsOpt.append(ito::Param("device", ito::ParamBase::String, "", tr("device name (IP address) that should be opened, an empty string opens the first device that is found (default). Pass '<scan>' for displaying all detected devices.").toLatin1().data()));
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Destructor of Interface Class.
/*!

*/
NerianSceneScanProInterface::~NerianSceneScanProInterface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NerianSceneScanProInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(NerianSceneScanPro) //the argument of the macro is the classname of the plugin
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NerianSceneScanProInterface::closeThisInst(ito::AddInBase **addInInst)
{
   REMOVE_PLUGININSTANCE(NerianSceneScanPro) //the argument of the macro is the classname of the plugin
   return ito::retOk;
}




//----------------------------------------------------------------------------------------------------------------------------------
//! Constructor of plugin.
/*!
    \todo add internal parameters of the plugin to the map m_params. It is allowed to append or remove entries from m_params
    in this constructor or later in the init method
*/
NerianSceneScanPro::NerianSceneScanPro() : AddInGrabber(), m_isgrabbing(false), m_pParamsObj(NULL), m_pImageTransferObj(NULL), m_pImagePair(NULL)
{
    ito::DoubleMeta *dm;
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "NerianSceneScanPro", NULL);
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "General"), true);
    m_params.insert(paramVal.getName(), paramVal);

    //ROI setting not implemented, as no offset of roi can be set on the device.
    /*paramVal = ito::Param("x0", ito::ParamBase::Int | ito::ParamBase::Readonly, NULL, tr("first pixel index in ROI (x-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("y0", ito::ParamBase::Int | ito::ParamBase::Readonly, NULL, tr("first pixel index in ROI (y-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("x1", ito::ParamBase::Int | ito::ParamBase::Readonly, NULL, tr("last pixel index in ROI (x-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("y1", ito::ParamBase::Int | ito::ParamBase::Readonly, NULL, tr("last pixel index in ROI (y-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);*/
    paramVal = ito::Param("sizex", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::In, NULL, tr("width of ROI (x-direction)").toLatin1().data());
    paramVal.setMeta(new ito::IntMeta(1, std::numeric_limits<int>::max(),1 ,"ImageFormatControl"), true);
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizey", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::In, NULL, tr("height of ROI (y-direction)").toLatin1().data());
    paramVal.setMeta(new ito::IntMeta(1, std::numeric_limits<int>::max(), 1, "ImageFormatControl"), true);
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizez", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::In, NULL, tr("number of planes of returned dataObject").toLatin1().data());
    paramVal.setMeta(new ito::IntMeta(2, 2, 2, "ImageFormatControl"), true);
    m_params.insert(paramVal.getName(), paramVal);

    int roi[] = { 0, 0, 2048, 2048 };
#if defined(ITOM_ADDININTERFACE_VERSION) && ITOM_ADDININTERFACE_VERSION > 0x010300
    paramVal = ito::Param("roi", ito::ParamBase::IntArray|ito::ParamBase::Readonly , 4, roi, tr("ROI (x,y,width,height) [this replaces the values x0,x1,y0,y1]").toLatin1().data());
    ito::RectMeta *rm = new ito::RectMeta(ito::RangeMeta(0, std::numeric_limits<int>::max()), ito::RangeMeta(0, std::numeric_limits<int>::max()));
    rm->setCategory("ImageFormatControl");
    paramVal.setMeta(rm, true);
    m_params.insert(paramVal.getName(), paramVal);
#endif

    paramVal = ito::Param("bpp", ito::ParamBase::Int | ito::ParamBase::Readonly, NULL, tr("Bit depth of the output data from camera in bpp (can differ from sensor bit depth).").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    m_params["bpp"].setMeta(new ito::IntMeta(8, 16, 8, "ImageFormatControl"), true);

    paramVal = ito::Param("operationMode", ito::ParamBase::Int | ito::ParamBase::In, 0, 2, 0, tr("0: Pass through, 1: rectify, 2: stereo matching").toLatin1().data());
    paramVal.getMetaT<ito::IntMeta>()->setCategory("ProcessingControl");
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("disparityOffset", ito::ParamBase::Int | ito::ParamBase::In, 0, 255, 0, tr("current offset of the evaluated disparity range").toLatin1().data());
    paramVal.getMetaT<ito::IntMeta>()->setCategory("ProcessingControl");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("stereoMatchingP1Edge", ito::ParamBase::Int | ito::ParamBase::In, 0, 255, 0, tr("SGM penalty P1 for small disparity changes at image edges").toLatin1().data());
    paramVal.getMetaT<ito::IntMeta>()->setCategory("ProcessingControl");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("stereoMatchingP1NoEdge", ito::ParamBase::Int | ito::ParamBase::In, 0, 255, 0, tr("SGM penalty P1 for small disparity changes outside image edges").toLatin1().data());
    paramVal.getMetaT<ito::IntMeta>()->setCategory("ProcessingControl");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("stereoMatchingP2Edge", ito::ParamBase::Int | ito::ParamBase::In, 0, 255, 0, tr("SGM penalty P2 for small disparity changes at image edges").toLatin1().data());
    paramVal.getMetaT<ito::IntMeta>()->setCategory("ProcessingControl");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("stereoMatchingP2NoEdge", ito::ParamBase::Int | ito::ParamBase::In, 0, 255, 0, tr("SGM penalty P2 for small disparity changes outside image edges").toLatin1().data());
    paramVal.getMetaT<ito::IntMeta>()->setCategory("ProcessingControl");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("stereoMatchingEdgeSensitivity", ito::ParamBase::Int | ito::ParamBase::In, 0, 255, 0, tr("edge sensitivity of the SGM algorithm").toLatin1().data());
    paramVal.getMetaT<ito::IntMeta>()->setCategory("ProcessingControl");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("maskBorderPixels", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 1, tr("1 if border pixels are removed from the computed").toLatin1().data());
    paramVal.getMetaT<ito::IntMeta>()->setCategory("ProcessingControl");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("consistencyCheck", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 1, tr("1 if the consistency check is enabled").toLatin1().data());
    paramVal.getMetaT<ito::IntMeta>()->setCategory("ProcessingControl");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("consistencyCheckSensitivity", ito::ParamBase::Int | ito::ParamBase::In, 0, 15, 0, tr("sensitivity value for the consistency check").toLatin1().data());
    paramVal.getMetaT<ito::IntMeta>()->setCategory("ProcessingControl");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("uniquenessCheck", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 1, tr("1 if the uniqueness check is enabled").toLatin1().data());
    paramVal.getMetaT<ito::IntMeta>()->setCategory("ProcessingControl");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("uniquenessCheckSensitivity", ito::ParamBase::Int | ito::ParamBase::In, 0, 256, 1, tr("sensitivity value for the uniqueness check").toLatin1().data());
    paramVal.getMetaT<ito::IntMeta>()->setCategory("ProcessingControl");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("textureFilter", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 1, tr("1 if the texture filter is enabled").toLatin1().data());
    paramVal.getMetaT<ito::IntMeta>()->setCategory("ProcessingControl");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("textureFilterSensitivity", ito::ParamBase::Int | ito::ParamBase::In, 0, 63, 1, tr("sensitivity value for the texture filter").toLatin1().data());
    paramVal.getMetaT<ito::IntMeta>()->setCategory("ProcessingControl");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("gapInterpolation", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 1, tr("1 if the texture gap interpolation is enabled").toLatin1().data());
    paramVal.getMetaT<ito::IntMeta>()->setCategory("ProcessingControl");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("noiseReduction", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 1, tr("1 if the noise reduction filter is enabled").toLatin1().data());
    paramVal.getMetaT<ito::IntMeta>()->setCategory("ProcessingControl");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("speckleFilterIterations", ito::ParamBase::Int | ito::ParamBase::In, 0, 2, 2, tr("set the number of speckle reduction iterations").toLatin1().data());
    paramVal.getMetaT<ito::IntMeta>()->setCategory("ProcessingControl");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("exposureGainMode", ito::ParamBase::Int | ito::ParamBase::In, 0, 3, 0, tr("0: auto exposure and gain, 1: auto exposure manual gain, 2: manual exposure auto gain, 3: manual exposure and gain").toLatin1().data());
    paramVal.getMetaT<ito::IntMeta>()->setCategory("AutoExposureControl");
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("autoROI", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("1 if an ROI for automatic exposure and gain control is enabled.").toLatin1().data());
    paramVal.getMetaT<ito::IntMeta>()->setCategory("AutoExposureControl");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("autoExposureGainControlROI", ito::ParamBase::IntArray | ito::ParamBase::In, 4,roi, tr("ROI for automatic exposure and gain control [x,y,width,height]. X and y are the offset of the ROI from the image center. A value of 0 means the ROI is horizontally centered").toLatin1().data());
    ito::RectMeta *meta = new ito::RectMeta(ito::RangeMeta(0, std::numeric_limits<int>::max()), ito::RangeMeta(0, std::numeric_limits<int>::max()));
    meta->setCategory("AutoExposureControl");
    paramVal.setMeta(meta, true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("autoTargetIntensity", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 0.0, tr("target image intensity (from 0.0 to 1.0) of the automatic exposure and gain control.").toLatin1().data());
    paramVal.getMetaT<ito::DoubleMeta>()->setCategory("AutoExposureControl");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("autoIntensityDelta", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 0.0, tr("minimum intensity change that is required for adjusting the camera settings.Intensity values are relatively to the target intensity. A value of 0.01 represents a change of 1 %.").toLatin1().data());
    paramVal.getMetaT<ito::DoubleMeta>()->setCategory("AutoExposureControl");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("autoTargetFrame", ito::ParamBase::Int | ito::ParamBase::In, 0, 2, 0, tr("target frame for automatic exposure and gain control. 0: left frame, 1: right frame, 2 both frames.").toLatin1().data());
    paramVal.getMetaT<ito::IntMeta>()->setCategory("AutoExposureControl");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("autoSkippedFrames", ito::ParamBase::Int | ito::ParamBase::In, NULL, tr("current interval at which the automatic exposure and gain control is run. The value indicates the number of skipped frames between each adjustment.Typically a value > 0 is desired to give the cameras enough time to react to the new setting.").toLatin1().data());
    paramVal.setMeta(new ito::IntMeta(1, std::numeric_limits<int>::max(), 1, "AutoExposureControl"), true);
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("autoMaxExposureTime", ito::ParamBase::Double | ito::ParamBase::In, NULL, tr("maximum exposure time that can be selected automatically in microseconds.").toLatin1().data());
    paramVal.setMeta(new ito::DoubleMeta(0.0, std::numeric_limits<double>::max(), 0.0, "AutoExposureControl"), true);
    paramVal.getMetaT<ito::DoubleMeta>()->setUnit("ms");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("autoMaxGain", ito::ParamBase::Double | ito::ParamBase::In, NULL, tr("maximum gain that can be selected automatically in dB.").toLatin1().data());
    paramVal.setMeta(new ito::DoubleMeta(0.0, std::numeric_limits<double>::max(), 0.0, "AutoExposureControl"), true);
    paramVal.getMetaT<ito::DoubleMeta>()->setUnit("dB");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("manualExposureTime", ito::ParamBase::Double | ito::ParamBase::In, NULL, tr("manually selected exposure time in microseconds. This parameter is only relevant if the auto mode is set to manual exposure.").toLatin1().data());
    paramVal.setMeta(new ito::DoubleMeta(0.0, std::numeric_limits<double>::max(), 0.0, "AutoExposureControl"), true);
    paramVal.getMetaT<ito::DoubleMeta>()->setUnit("ms");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("manualGain", ito::ParamBase::Double | ito::ParamBase::In, NULL, tr("manually selected gain in dB. This parameter is only relevant if the auto mode is set to manual gain.").toLatin1().data());
    paramVal.setMeta(new ito::DoubleMeta(0.0, std::numeric_limits<double>::max(), 0.0, "AutoExposureControl"), true);
    paramVal.getMetaT<ito::DoubleMeta>()->setUnit("dB");
    m_params.insert(paramVal.getName(), paramVal);


    paramVal = ito::Param("maxFrameTimeDifference", ito::ParamBase::Int | ito::ParamBase::In, NULL, tr("maximum allowed time difference between two corresponding frames.").toLatin1().data());
    paramVal.setMeta(new ito::IntMeta(0, std::numeric_limits<int>::max(), 1, "TriggerPairingControl"), true);
    paramVal.getMetaT<ito::IntMeta>()->setUnit("ms");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("triggerFrequency", ito::ParamBase::Double | ito::ParamBase::In, NULL, tr("frequency of the trigger signal in Hz.").toLatin1().data());
    paramVal.setMeta(new ito::DoubleMeta(0.0, std::numeric_limits<double>::max(), 0.0, "TriggerPairingControl"), true);
    paramVal.getMetaT<ito::DoubleMeta>()->setUnit("Hz");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("trigger0", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("1 if trigger signal 0 is enabled.").toLatin1().data());
    paramVal.getMetaT<ito::IntMeta>()->setCategory("TriggerPairingControl");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("trigger1", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("1 if trigger signal 1 is enabled.").toLatin1().data());
    paramVal.getMetaT<ito::IntMeta>()->setCategory("TriggerPairingControl");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("trigger0PulseWidth", ito::ParamBase::Double | ito::ParamBase::In, NULL, tr("trigger 0 pulse width in milliseconds.").toLatin1().data());
    paramVal.setMeta(new ito::DoubleMeta(0.0, std::numeric_limits<double>::max(), 0.0, "TriggerPairingControl"), true);
    paramVal.getMetaT<ito::DoubleMeta>()->setUnit("ms");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("trigger1PulseWidth", ito::ParamBase::Double | ito::ParamBase::In, NULL, tr("trigger 1 pulse width in milliseconds.").toLatin1().data());
    paramVal.setMeta(new ito::DoubleMeta(0.0, std::numeric_limits<double>::max(), 0.0, "TriggerPairingControl"), true);
    paramVal.getMetaT<ito::DoubleMeta>()->setUnit("ms");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("trigger1Offset", ito::ParamBase::Double | ito::ParamBase::In, NULL, tr("time offset between trigger signal 1 and signal 0 in milliseconds.").toLatin1().data());
    paramVal.setMeta(new ito::DoubleMeta(0.0, std::numeric_limits<double>::max(), 0.0, "TriggerPairingControl"), true);
    paramVal.getMetaT<ito::DoubleMeta>()->setUnit("ms");
    m_params.insert(paramVal.getName(), paramVal);


    paramVal = ito::Param("autoRecalibration", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("1 auto re-calibration is enabled.").toLatin1().data());
    paramVal.getMetaT<ito::IntMeta>()->setCategory("CalibrationControl");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("saveAutoRecalibration", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("1 persistent storage of auto re-calibration results.").toLatin1().data());
    paramVal.getMetaT<ito::IntMeta>()->setCategory("CalibrationControl");
    m_params.insert(paramVal.getName(), paramVal);



    //the following lines create and register the plugin's dock widget. Delete these lines if the plugin does not have a dock widget.
    DockWidgetNerianSceneScanPro *dw = new DockWidgetNerianSceneScanPro(this);

    Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
    QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
    createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dw);
}

//----------------------------------------------------------------------------------------------------------------------------------
NerianSceneScanPro::~NerianSceneScanPro()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
//! initialization of plugin
/*!
    \sa close
*/
ito::RetVal NerianSceneScanPro::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString deviceName = paramsOpt->at(0).getVal<char*>();
    DeviceEnumeration deviceEnum;
    DeviceEnumeration::DeviceList devices;
    devices = deviceEnum.discoverDevices();
    unsigned int deviceIdx = 0;
    retValue += m_params["sizez"].setVal<int>(2);
    if (devices.size() == 0)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("No devices found").toLatin1().data());
    }
    if (!retValue.containsError())
    {
        if (deviceName == "<scan>")
        {
            std::cout << "Nerian SceneScan Pro devices \n" << std::endl;

            for (int i = 0; i < devices.size(); ++i)
            {
                std::cout << "Dev. " << i << ": " << devices[i].toString() << std::endl;

            }
            retValue += ito::RetVal(ito::retError, 0, tr("The initialization is terminated since only a list of found devices has been requested ('<scan>')").toLatin1().data());
        }
    }
    if (!retValue.containsError())
    {
        if (!deviceName.isEmpty())
        {
            bool found = false;
            for (int i = 0; i < devices.size(); ++i)
            {
                if (!devices[i].getIpAddress().compare(deviceName.toStdString()))
                {
                    found = true;
                    deviceIdx = i;
                }

            }
            if (!found)
            {
                retValue += ito::RetVal(ito::retError, 0, tr("Could not find device with ip: %1").arg(deviceName).toLatin1().data());
            }
        }
    }
    if (!retValue.containsError())
    {
        m_pParamsObj = new DeviceParameters(devices[deviceIdx]);
        m_pImageTransferObj = new ImageTransfer(devices[deviceIdx]);
        m_pImagePair = new ImagePair();


        retValue += m_params["name"].setVal<char*>(&devices[deviceIdx].toString()[0]);
        m_identifier = QString(&devices[deviceIdx].toString()[0]);
        setIdentifier(m_identifier);
        //take a test image
        try
        {
            while (!m_pImageTransferObj->receiveImagePair(*m_pImagePair))
            {
                //wait till done
            }
        }catch(const std::exception &ex)
        {
            retValue += ito::RetVal(ito::retError, 0, QString("error while acquiering test image: %1").arg(ex.what()).toLatin1().data());
        }
        retValue += syncParams();
        if (!retValue.containsError())
        {

        }




    }
    //steps todo:
    // - get all initialization parameters
    // - try to detect your device
    // - establish a connection to the device
    // - synchronize the current parameters of the device with the current values of parameters inserted in m_params
    // - if an identifier string of the device is available, set it via setIdentifier("yourIdentifier")
    // - call checkData() in order to reconfigure the temporary image buffer m_data (or other structures) depending on the current size, image type...
    // - call emit parametersChanged(m_params) in order to propagate the current set of parameters in m_params to connected dock widgets...
    // - call setInitialized(true) to confirm the end of the initialization (even if it failed)

    if (!retValue.containsError())
    {
        retValue += checkData();
    }

    if (!retValue.containsError())
    {
        emit parametersChanged(m_params);
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
ito::RetVal NerianSceneScanPro::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    //todo:
    // - disconnect the device if not yet done
    // - this function is considered to be the "inverse" of init.
    delete m_pParamsObj, m_pImageTransferObj, m_pImagePair;
    m_pParamsObj, m_pImageTransferObj, m_pImagePair = NULL;

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NerianSceneScanPro::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
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

    if (retValue == ito::retOk)
    {
        //gets the parameter key from m_params map (read-only is allowed, since we only want to get the value).
        retValue += apiGetParamFromMapByKey(m_params, key, it, false);
    }

    if (!retValue.containsError())
    {
        //put your switch-case.. for getting the right value here

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
ito::RetVal NerianSceneScanPro::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
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
        if (key == "operationMode")
        {
            DeviceParameters::OperationMode mode =(DeviceParameters::OperationMode) val->getVal<int>();
            try {
                m_pParamsObj->setOperationMode(mode);
            }
            catch (const std::exception& ex)
            {
                std::string msg = "Error while setting parameter operationMode:";
                retValue += ito::RetVal(ito::retError, 0, tr("Error while setting parameter operationMode: %1").arg(ex.what()).toLatin1().data());
            }
            syncParams(sOperationMode);
            syncParams(sImageFormat);
        }
        else if (key == "disparityOffset")
        {
            int offset = val->getVal<int>();
            try {
                m_pParamsObj->setDisparityOffset(offset);
            }
            catch (const std::exception& ex)
            {
                retValue += ito::RetVal(ito::retError, 0, tr(QString("Error while setting parameter %1: %2").arg(key).arg(ex.what()).toLatin1().data()).toLatin1().data());
            }
            syncParams(sDisparityOffset);
        }
        else if (key == "stereoMatchingP1Edge")
        {
            int edge = val->getVal<int>();
            try {
                m_pParamsObj->setStereoMatchingP1Edge(edge);
            }
            catch (const std::exception& ex)
            {
                retValue += ito::RetVal(ito::retError, 0, tr(QString("Error while setting parameter %1: %2").arg(key).arg(ex.what()).toLatin1().data()).toLatin1().data());
            }
            syncParams(sStereoMatching);
        }
        else if (key == "stereoMatchingP1NoEdge")
        {
            int edge = val->getVal<int>();
            try
            {
                m_pParamsObj->setStereoMatchingP1NoEdge(edge);
            }
            catch (const std::exception& ex)
            {
                retValue += ito::RetVal(ito::retError, 0, tr(QString("Error while setting parameter %1: %2").arg(key).arg(ex.what()).toLatin1().data()).toLatin1().data());
            }
            syncParams(sStereoMatching);
        }
        else if (key == "stereoMatchingP2Edge")
        {
            int edge = val->getVal<int>();
            try
            {
                m_pParamsObj->setStereoMatchingP2Edge(edge);
            }
            catch (const std::exception& ex)
            {
                retValue += ito::RetVal(ito::retError, 0, tr(QString("Error while setting parameter %1: %2").arg(key).arg(ex.what()).toLatin1().data()).toLatin1().data());
            }
            syncParams(sStereoMatching);
        }
        else if (key == "stereoMatchingP2NoEdge")
        {
            int edge = val->getVal<int>();
            try {
                m_pParamsObj->setStereoMatchingP2NoEdge(edge);
            }
            catch (const std::exception& ex)
            {
                retValue += ito::RetVal(ito::retError, 0, tr(QString("Error while setting parameter %1: %2").arg(key).arg(ex.what()).toLatin1().data()).toLatin1().data());
            }
            syncParams(sStereoMatching);
        }
        else if (key == "stereoMatchingEdgeSensitivity")
        {
            int edge = val->getVal<int>();
            try
            {
                m_pParamsObj->setStereoMatchingEdgeSensitivity(edge);
            }
            catch (const std::exception& ex)
            {
                retValue += ito::RetVal(ito::retError, 0, tr(QString("Error while setting parameter %1: %2").arg(key).arg(ex.what()).toLatin1().data()).toLatin1().data());
            }
            syncParams(sStereoMatching);
        }
        else if (key == "maskBorderPixels")
        {
            int enable = val->getVal<int>();
            try {
                m_pParamsObj->setMaskBorderPixelsEnabled(enable == 0 ? false : true);
            }
            catch (const std::exception& ex)
            {
                retValue += ito::RetVal(ito::retError, 0, tr(QString("Error while setting parameter %1: %2").arg(key).arg(ex.what()).toLatin1().data()).toLatin1().data());
            }
            syncParams(sMaskBorderPixels);
        }
        else if (key == "consistencyCheck")
        {
            int enable = val->getVal<int>();
            try
            {
                m_pParamsObj->setConsistencyCheckEnabled(enable == 0 ? false : true);
            }
            catch (const std::exception& ex)
            {
                retValue += ito::RetVal(ito::retError, 0, tr(QString("Error while setting parameter %1: %2").arg(key).arg(ex.what()).toLatin1().data()).toLatin1().data());
            }
            syncParams(sConsistencyCheck);
        }
        else if (key == "consistencyCheckSensitivity")
        {
            int sensitivity = val->getVal<int>();
            try {
                m_pParamsObj->setConsistencyCheckSensitivity(sensitivity);
            }
            catch (const std::exception& ex)
            {
                retValue += ito::RetVal(ito::retError, 0, tr(QString("Error while setting parameter %1: %2").arg(key).arg(ex.what()).toLatin1().data()).toLatin1().data());
            }
            syncParams(sConsistencyCheck);
        }
        else if (key == "uniquenessCheck")
        {
            int enable = val->getVal<int>();
            try {
                m_pParamsObj->setUniquenessCheckEnabled(enable == 0 ? false : true);
            }
            catch (const std::exception& ex)
            {
                retValue += ito::RetVal(ito::retError, 0, tr(QString("Error while setting parameter %1: %2").arg(key).arg(ex.what()).toLatin1().data()).toLatin1().data());
            }
            syncParams(sUniquenessCheck);
        }
        else if (key == "uniquenessCheckSensitivity")
        {
            int sensitivity = val->getVal<int>();
            try {
                m_pParamsObj->setUniquenessCheckSensitivity(sensitivity);
            }
            catch (const std::exception& ex)
            {
                retValue += ito::RetVal(ito::retError, 0, tr(QString("Error while setting parameter %1: %2").arg(key).arg(ex.what()).toLatin1().data()).toLatin1().data());
            }
            syncParams(sUniquenessCheck);
        }
        else if (key == "textureFilter")
        {
            int enable = val->getVal<int>();
            try {
                m_pParamsObj->setTextureFilterEnabled(enable == 0 ? false : true);
            }
            catch (const std::exception& ex)
            {
                retValue += ito::RetVal(ito::retError, 0, tr(QString("Error while setting parameter %1: %2").arg(key).arg(ex.what()).toLatin1().data()).toLatin1().data());
            }
            syncParams(sTextureFilter);
        }
        else if (key == "textureFilterSensitivity")
        {
            int sensitivity = val->getVal<int>();
            try {
                m_pParamsObj->setTextureFilterSensitivity(sensitivity);
            }
            catch (const std::exception& ex)
            {
                retValue += ito::RetVal(ito::retError, 0, tr(QString("Error while setting parameter %1: %2").arg(key).arg(ex.what()).toLatin1().data()).toLatin1().data());
            }
            syncParams(sTextureFilter);
        }
        else if (key == "gapInterpolation")
        {
            int enable = val->getVal<int>();
            try {
                m_pParamsObj->setGapInterpolationEnabled(enable == 0 ? false : true);
            }
            catch (const std::exception& ex)
            {
                retValue += ito::RetVal(ito::retError, 0, tr(QString("Error while setting parameter %1: %2").arg(key).arg(ex.what()).toLatin1().data()).toLatin1().data());
            }
            syncParams(sGapInterpolation);
        }
        else if (key == "noiseReduction")
        {
            int enable = val->getVal<int>();
            try
            {
                m_pParamsObj->setNoiseReductionEnabled(enable == 0 ? false : true);
            }
            catch (const std::exception& ex)
            {
                retValue += ito::RetVal(ito::retError, 0, tr(QString("Error while setting parameter %1: %2").arg(key).arg(ex.what()).toLatin1().data()).toLatin1().data());
            }
            syncParams(sNoiseReduction);
        }
        else if (key == "speckleFilterIterations")
        {
            int iter = val->getVal<int>();
            try
            {
                m_pParamsObj->setSpeckleFilterIterations(iter);
            }
            catch (const std::exception& ex)
            {
                retValue += ito::RetVal(ito::retError, 0, tr(QString("Error while setting parameter %1: %2").arg(key).arg(ex.what()).toLatin1().data()).toLatin1().data());
            }
            syncParams(sSpeckleFilterIterations);
        }
        else if (key == "exposureGainMode")
        {
            DeviceParameters::AutoMode mode = (DeviceParameters::AutoMode) val->getVal<int>();
            try
            {
                m_pParamsObj->setAutoMode(mode);
            }
            catch (const std::exception& ex)
            {
                retValue += ito::RetVal(ito::retError, 0, tr(QString("Error while setting parameter %1: %2").arg(key).arg(ex.what()).toLatin1().data()).toLatin1().data());
            }
            syncParams(sAutoMode);
        }
        else if (key == "autoTargetIntensity")
        {
            double intensity = val->getVal<double>();
            try {
                m_pParamsObj->setAutoTargetIntensity(intensity);
            }
            catch (const std::exception& ex)
            {
                retValue += ito::RetVal(ito::retError, 0, tr(QString("Error while setting parameter %1: %2").arg(key).arg(ex.what()).toLatin1().data()).toLatin1().data());
            }
            syncParams(sAutoTargetIntensity);
        }
        else if (key == "autoIntensityDelta")
        {
            double delta = val->getVal<double>();
            try {
                m_pParamsObj->setAutoIntensityDelta(delta);
            }
            catch (const std::exception& ex)
            {
                retValue += ito::RetVal(ito::retError, 0, tr(QString("Error while setting parameter %1: %2").arg(key).arg(ex.what()).toLatin1().data()).toLatin1().data());
            }
            syncParams(sAutoIntensityDelta);
        }
        else if (key == "autoTargetFrame")
        {
            DeviceParameters::TargetFrame frame = (DeviceParameters::TargetFrame) val->getVal<int>();
            try {
                m_pParamsObj->setAutoTargetFrame(frame);
            }
            catch (const std::exception& ex)
            {
                retValue += ito::RetVal(ito::retError, 0, tr(QString("Error while setting parameter %1: %2").arg(key).arg(ex.what()).toLatin1().data()).toLatin1().data());
            }
            syncParams(sAutoTargetFrame);
        }
        else if (key == "autoSkippedFrames")
        {
            int frames = val->getVal<int>();
            try
            {
                m_pParamsObj->setAutoSkippedFrames(frames);
            }
            catch (const std::exception& ex)
            {
                retValue += ito::RetVal(ito::retError, 0, tr(QString("Error while setting parameter %1: %2").arg(key).arg(ex.what()).toLatin1().data()).toLatin1().data());
            }
            syncParams(sAutoSkippedFrames);
        }
        else if (key == "autoMaxExposureTime")
        {
            double time = val->getVal<double>();
            try
            {
                m_pParamsObj->setAutoMaxExposureTime(time);
            }
            catch (const std::exception& ex)
            {
                retValue += ito::RetVal(ito::retError, 0, tr(QString("Error while setting parameter %1: %2").arg(key).arg(ex.what()).toLatin1().data()).toLatin1().data());
            }
            syncParams(sAutoMaxExposureTime);
        }
        else if (key == "autoMaxGain")
        {
            double gain = val->getVal<double>();
            try {
                m_pParamsObj->setAutoMaxGain(gain);
            }
            catch (const std::exception& ex)
            {
                retValue += ito::RetVal(ito::retError, 0, tr(QString("Error while setting parameter %1: %2").arg(key).arg(ex.what()).toLatin1().data()).toLatin1().data());
            }
            syncParams(sAutoMaxGain);
        }
        else if (key == "trigger0")
        {
            int enable = val->getVal<int>();
            try {
                m_pParamsObj->setTrigger0Enabled(enable == 0 ? false : true);
            }
            catch (const std::exception& ex)
            {
                retValue += ito::RetVal(ito::retError, 0, tr(QString("Error while setting parameter %1: %2").arg(key).arg(ex.what()).toLatin1().data()).toLatin1().data());
            }
            syncParams(sTrigger);
        }
        else if (key == "manualExposureTime")
        {
            double time = val->getVal<double>();
            try
            {
                m_pParamsObj->setManualExposureTime(time);
            }
            catch (const std::exception& ex)
            {
                retValue += ito::RetVal(ito::retError, 0, tr(QString("Error while setting parameter %1: %2").arg(key).arg(ex.what()).toLatin1().data()).toLatin1().data());
            }
            syncParams(sManualExposureTime);
        }
        else if (key == "manualGain")
        {
            double gain = val->getVal<double>();
            try {
                m_pParamsObj->setManualGain(gain);
            }
            catch (const std::exception& ex)
            {
                retValue += ito::RetVal(ito::retError, 0, tr(QString("Error while setting parameter %1: %2").arg(key).arg(ex.what()).toLatin1().data()).toLatin1().data());
            }
            syncParams(sManualGain);
        }
        else if (key == "autoROI")
        {
            int enable = val->getVal<int>();
            try {
                m_pParamsObj->setAutoROIEnabled(enable == 0 ? false : true);
            }
            catch (const std::exception& ex)
            {
                retValue += ito::RetVal(ito::retError, 0, tr(QString("Error while setting parameter %1: %2").arg(key).arg(ex.what()).toLatin1().data()).toLatin1().data());
            }
            syncParams(sAutoROI);

        }
        else if(key == "autoExposureGainControlROI")
        {
            int* roi = val->getVal<int*>();
            try {
                m_pParamsObj->setAutoROI(roi[0], roi[1], roi[2], roi[3]);
            }
            catch (const std::exception& ex)
            {
                retValue += ito::RetVal(ito::retError, 0, tr(QString("Error while setting parameter %1: %2").arg(key).arg(ex.what()).toLatin1().data()).toLatin1().data());
            }
            syncParams(sAutoROI);
        }
        else if (key == "maxFrameTimeDifference")
        {
            int time = val->getVal<int>();
            try {
                m_pParamsObj->setMaxFrameTimeDifference(time);
            }
            catch (const std::exception& ex)
            {
                retValue += ito::RetVal(ito::retError, 0, tr(QString("Error while setting parameter %1: %2").arg(key).arg(ex.what()).toLatin1().data()).toLatin1().data());
            }
            syncParams(sMaxFrameTimeDifference);
        }
        else if (key == "triggerFrequency")
        {
            double frequency = val->getVal<double>();
            try
            {
                m_pParamsObj->setTriggerFrequency(frequency);
            }
            catch (const std::exception& ex)
            {
                retValue += ito::RetVal(ito::retError, 0, tr(QString("Error while setting parameter %1: %2").arg(key).arg(ex.what()).toLatin1().data()).toLatin1().data());
            }
            syncParams(sTrigger);
        }
        else if (key == "trigger1")
        {
            int enable = val->getVal<int>();
            try {
                m_pParamsObj->setTrigger1Enabled(enable == 0 ? false : true);
            }
            catch (const std::exception& ex)
            {
                retValue += ito::RetVal(ito::retError, 0, tr(QString("Error while setting parameter %1: %2").arg(key).arg(ex.what()).toLatin1().data()).toLatin1().data());
            }
            syncParams(sTrigger);
        }
        else if (key == "trigger0PulseWidth")
        {
            double width = val->getVal<double>();
            try {
                m_pParamsObj->setTrigger0PulseWidth(width);
            }
            catch (const std::exception& ex)
            {
                retValue += ito::RetVal(ito::retError, 0, tr(QString("Error while setting parameter %1: %2").arg(key).arg(ex.what()).toLatin1().data()).toLatin1().data());
            }
            syncParams(sTrigger);
        }
        else if (key == "trigger1PulseWidth")
        {
            double width = val->getVal<double>();
            try {
                m_pParamsObj->setTrigger1PulseWidth(width);
            }
            catch (const std::exception& ex)
            {
                retValue += ito::RetVal(ito::retError, 0, tr(QString("Error while setting parameter %1: %2").arg(key).arg(ex.what()).toLatin1().data()).toLatin1().data());
            }
            syncParams(sTrigger);
        }
        else if (key == "trigger1Offset")
        {
            double offset = val->getVal<double>();
            try {
                m_pParamsObj->setTrigger1Offset(offset);
            }
            catch (const std::exception& ex)
            {
                retValue += ito::RetVal(ito::retError, 0, tr(QString("Error while setting parameter %1: %2").arg(key).arg(ex.what()).toLatin1().data()).toLatin1().data());
            }
            syncParams(sTrigger);
        }
        else if (key == "autoRecalibration")
        {
            int enable = val->getVal<int>();
            try {
                m_pParamsObj->setAutoRecalibrationEnabled(enable == 0 ? false : true);
            }
            catch (const std::exception& ex)
            {
                retValue += ito::RetVal(ito::retError, 0, tr(QString("Error while setting parameter %1: %2").arg(key).arg(ex.what()).toLatin1().data()).toLatin1().data());
            }
            syncParams(sAutoRecalibration);
        }
        else if (key == "saveAutoRecalibration")
        {
            int enable = val->getVal<int>();
            try {
                m_pParamsObj->setSaveAutoRecalibration(enable == 0 ? false : true);
            }
            catch (const std::exception& ex)
            {
                retValue += ito::RetVal(ito::retError, 0, tr(QString("Error while setting parameter %1: %2").arg(key).arg(ex.what()).toLatin1().data()).toLatin1().data());
            }
            syncParams(sAutoRecalibration);
        }
        /*else if (key == "bpp") it seems to be read only
        {
            if (m_pImagePair)
            {
                int bpp = val->getVal<int>();
                if (bpp == 8)
                {
                    //m_pImagePair->setPixelFormat(0, ImagePair::FORMAT_8_BIT_MONO);
                    //m_pImagePair->setPixelFormat(1, ImagePair::FORMAT_8_BIT_MONO);
                }
                else if (bpp == 16)
                {
                    //m_pImagePair->setPixelFormat(0, ImagePair::FORMAT_12_BIT_MONO);
                    //m_pImagePair->setPixelFormat(1, ImagePair::FORMAT_12_BIT_MONO);
                }
                else
                {
                    retValue += ito::RetVal(ito::retError, 0, QString("Can not set bpp to %1 bit").arg(bpp).toLatin1().data());

                }
                syncParams(sImageFormat);
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 0, QString("ImagePair instance not available").toLatin1().data());
            }
        }*/
        else
        {
            retValue += ito::RetVal(ito::retError, 0, QString("could not find key matching: %1").arg(key).toLatin1().data());
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
ito::RetVal NerianSceneScanPro::startDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    incGrabberStarted(); //increment a counter to see how many times startDevice has been called

    //todo:
    // if this function has been called for the first time (grabberStartedCount() == 1),
    // start the camera, allocate necessary buffers or do other work that is necessary
    // to prepare the camera for image acquisitions.

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NerianSceneScanPro::stopDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    decGrabberStarted(); //decrements the counter (see startDevice)

    if (grabberStartedCount() < 0)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("The grabber has already been stopped.").toLatin1().data());
        setGrabberStarted(0);
    }

    //todo:
    // if the counter (obtained by grabberStartedCount()) drops to zero again, stop the camera, free all allocated
    // image buffers of the camera... (it is the opposite from all things that have been started, allocated... in startDevice)

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NerianSceneScanPro::acquire(const int trigger, ItomSharedSemaphore *waitCond)
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
    }
    try
    {
        //wait till frame is available
        while (!m_pImageTransferObj->receiveImagePair(*m_pImagePair))
        {

        }
    }
    catch(const std::exception& ex)
    {
        retValue += ito::RetVal(ito::retError, 0, tr(QString("Error while acquiering image: %1").arg(ex.what()).toLatin1().data()).toLatin1().data());
    }
    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    //todo:
    // it is possible now, to wait here until the acquired image is ready
    // if you want to do this here, wait for the finished image, get it and save it
    // to any accessible buffer, for instance the m_data dataObject that is finally delivered
    // via getVal or copyVal.
    //
    // you can also implement this waiting and obtaining procedure in retrieveImage.
    // If you do it here, the camera thread is blocked until the image is obtained, such that calls to getParam, setParam, stopDevice...
    // are not executed during the waiting operation. They are queued and executed once the image is acquired and transmitted to the plugin.

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NerianSceneScanPro::retrieveData(ito::DataObject *externalDataObject)
{
    //todo: this is just a basic example for getting the buffered image to m_data or the externally given data object
    //enhance it and adjust it for your needs
    ito::RetVal retValue(ito::retOk);

    ito::DataObject *dataObj = externalDataObject ? externalDataObject : &m_data;

    bool hasListeners = (m_autoGrabbingListeners.size() > 0);
    bool copyExternal = (externalDataObject != NULL);

    const int bufferWidth = m_params["sizex"].getVal<int>();
    const int bufferHeight = m_params["sizey"].getVal<int>();
    const int numChannel = m_params["sizez"].getVal<int>();


    if (m_isgrabbing == false)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("Tried to get picture without triggering exposure").toLatin1().data());
    }
    else
    {
        //step 1: create m_data (if not yet available)
        if (externalDataObject && hasListeners)
        {
            retValue += checkData(NULL); //update m_data
            retValue += checkData(externalDataObject); //update external object
        }
        else
        {
            retValue += checkData(externalDataObject); //update external object or m_data
        }

        if (!retValue.containsError())
        {
            if (m_pImagePair)
            {
                if (copyExternal)
                {

                    ImagePair::ImageFormat format1 = m_pImagePair->getPixelFormat(0);
                    ImagePair::ImageFormat format2 = m_pImagePair->getPixelFormat(1);
                    cv::Mat* matPtr1 = externalDataObject->getCvPlaneMat(0);
                    cv::Mat* matPtr2 = externalDataObject->getCvPlaneMat(1);
                    unsigned char* imPtr1 = m_pImagePair->getPixelData(0);
                    unsigned char* imPtr2 = m_pImagePair->getPixelData(1);
                    const int pixelStep1 = m_pImagePair->getBytesPerPixel(0);
                    const int pixelStep2 = m_pImagePair->getBytesPerPixel(1);
                    const int imageRowStep1 = m_pImagePair->getRowStride(0);
                    const int imageRowStep2 = m_pImagePair->getRowStride(1);
                    const int rowStep = matPtr1->step1(0)*matPtr1->elemSize(); //in bytes
                    if (format1 == format2)
                    {
                        uchar *ptr1;
                        uchar *ptr2;
                        for (int row = 0; row < bufferHeight; ++row)
                        {
                            ptr1 = matPtr1->data + row*rowStep;
                            ptr2 = matPtr2->data + row*rowStep;
                            memcpy(ptr1, imPtr1 + row*imageRowStep1, pixelStep1*bufferWidth);
                            memcpy(ptr2, imPtr2 + row*imageRowStep2, pixelStep2*bufferWidth);
                        }
                    }
                    else
                    {
                        if (format1 == ImagePair::FORMAT_8_BIT_MONO)
                        {
                            ito::uint16 *ptr1;
                            uchar *ptr2;
                            for (int row = 0; row < bufferHeight; ++row)
                            {
                                ptr1 = (ito::uint16*)(matPtr1->data + row*rowStep);
                                ptr2 = matPtr2->data + row*rowStep;
                                for (int col = 0; col < bufferWidth; ++col)
                                {
                                    ptr1[col] = (ito::uint16)*(imPtr1 + row*imageRowStep1 + col*pixelStep1);
                                }
                                memcpy(ptr2, imPtr2 + row*imageRowStep2, pixelStep2*bufferWidth);
                            }
                        }
                        else if (format2 == ImagePair::FORMAT_8_BIT_MONO)
                        {
                            uchar *ptr1;
                            ito::uint16  *ptr2;
                            for (int row = 0; row < bufferHeight; ++row)
                            {
                                ptr2 = (ito::uint16*)(matPtr2->data + row*rowStep);
                                ptr1 = matPtr1->data + row*rowStep;
                                for (int col = 0; col < bufferWidth; ++col)
                                {
                                    ptr2[col] = (ito::uint16)*(imPtr2 + row*imageRowStep2 + col*pixelStep2);
                                }
                                memcpy(ptr1, imPtr1 + row*imageRowStep1, pixelStep1*bufferWidth);
                            }

                        }
                    }

                }
                if (!copyExternal || hasListeners)
                {
                    ImagePair::ImageFormat format1 = m_pImagePair->getPixelFormat(0);
                    ImagePair::ImageFormat format2 = m_pImagePair->getPixelFormat(1);
                    cv::Mat* matPtr1 = m_data.getCvPlaneMat(0);
                    cv::Mat* matPtr2 = m_data.getCvPlaneMat(1);
                    unsigned char* imPtr1 = m_pImagePair->getPixelData(0);
                    unsigned char* imPtr2 = m_pImagePair->getPixelData(1);
                    const int pixelStep1 = m_pImagePair->getBytesPerPixel(0);
                    const int pixelStep2 = m_pImagePair->getBytesPerPixel(1);
                    const int imageRowStep1 = m_pImagePair->getRowStride(0);
                    const int imageRowStep2 = m_pImagePair->getRowStride(1);
                    const int rowStep = matPtr1->step1(0)*matPtr1->elemSize(); //in bytes
                    if (format1 == format2)
                    {
                        uchar *ptr1;
                        uchar *ptr2;
                        for (int row = 0; row < bufferHeight; ++row)
                        {
                            ptr1 = matPtr1->data + row*rowStep;
                            ptr2 = matPtr2->data + row*rowStep;
                            memcpy(ptr1, imPtr1 + row*imageRowStep1, pixelStep1*bufferWidth);
                            memcpy(ptr2, imPtr2 + row*imageRowStep2, pixelStep2*bufferWidth);
                        }
                    }
                    else
                    {
                        if (format1 == ImagePair::FORMAT_8_BIT_MONO)
                        {
                            ito::uint16 *ptr1;
                            uchar *ptr2;
                            for (int row = 0; row < bufferHeight; ++row)
                            {
                                ptr1 = (ito::uint16*)(matPtr1->data + row*rowStep);
                                ptr2 = matPtr2->data + row*rowStep;
                                for (int col = 0; col < bufferWidth; ++col)
                                {
                                    ptr1[col] = (ito::uint16)*(imPtr1 + row*imageRowStep1 + col*pixelStep1);
                                }
                                memcpy(ptr2, imPtr2 + row*imageRowStep2, pixelStep2*bufferWidth);
                            }
                        }
                        else if (format2 == ImagePair::FORMAT_8_BIT_MONO)
                        {
                            uchar *ptr1;
                            ito::uint16  *ptr2;
                            for (int row = 0; row < bufferHeight; ++row)
                            {
                                ptr2 = (ito::uint16*)(matPtr2->data + row*rowStep);
                                ptr1 = matPtr1->data + row*rowStep;
                                for (int col = 0; col < bufferWidth; ++col)
                                {
                                    ptr2[col] = (ito::uint16)*(imPtr2 + row*imageRowStep2 + col*pixelStep2);
                                }
                                memcpy(ptr1, imPtr1 + row*imageRowStep1, pixelStep1*bufferWidth);
                            }

                        }
                    }
                }

        }
    }

        m_isgrabbing = false;
    }

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
/*ito::RetVal NerianSceneScanPro::checkData(ito::DataObject *externalDataObject)
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
ito::RetVal NerianSceneScanPro::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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
ito::RetVal NerianSceneScanPro::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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
        retValue += retrieveData(dObj);  //checkData is executed inside of retrieveData
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
void NerianSceneScanPro::dockWidgetVisibilityChanged(bool visible)
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
const ito::RetVal NerianSceneScanPro::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogNerianSceneScanPro(this));
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NerianSceneScanPro::syncParams(SyncParams what /*=sAll*/)
{
    ito::RetVal retVal(ito::retOk);
    if (m_pParamsObj)
    {
        std::map<std::string, ParameterInfo> test = m_pParamsObj->getAllParameters();
        if (what & sOperationMode)
        {
            int min, max, inc,val;

            retVal += getParamInfo<int>(min, max, inc, val, "operation_mode");
                //int val = m_pParamsObj->getOperationMode();


            if (!retVal.containsError())
            {
                retVal += m_params["operationMode"].setVal<int>(val);
                ito::IntMeta* meta = m_params["operationMode"].getMetaT<ito::IntMeta>();
                meta->setMin(min);
                meta->setMax(max);
                meta->setStepSize(inc);
            }
        }
        if (what & sDisparityOffset)
        {
            int min, max, inc, val;

            retVal += getParamInfo<int>(min, max, inc, val, "disparity_offset");

            //int val = m_pParamsObj->getOperationMode();


            if (!retVal.containsError())
            {
                retVal += m_params["disparityOffset"].setVal<int>(val);
                ito::IntMeta* meta = m_params["disparityOffset"].getMetaT<ito::IntMeta>();
                meta->setMin(min);
                meta->setMax(max);
                meta->setStepSize(inc);
            }

            //int val = m_pParamsObj->getDisparityOffset();
        }
        if (what & sStereoMatching)
        {
            int min, max, inc, val;
            retVal += getParamInfo<int>(min, max, inc, val, "sgm_p1_edge");
            if (!retVal.containsError())
            {
                retVal += m_params["stereoMatchingP1Edge"].setVal<int>(val);
                ito::IntMeta* meta = m_params["stereoMatchingP1Edge"].getMetaT<ito::IntMeta>();
                meta->setMin(min);
                meta->setMax(max);
                meta->setStepSize(inc);
            }

            retVal += getParamInfo<int>(min, max, inc, val, "sgm_p1_no_edge");
            if (!retVal.containsError())
            {
                retVal += m_params["stereoMatchingP1NoEdge"].setVal<int>(val);
                ito::IntMeta* meta = m_params["stereoMatchingP1NoEdge"].getMetaT<ito::IntMeta>();
                meta->setMin(min);
                meta->setMax(max);
                meta->setStepSize(inc);
            }

            retVal += getParamInfo<int>(min, max, inc, val, "sgm_p2_edge");
            if (!retVal.containsError())
            {
                retVal += m_params["stereoMatchingP2Edge"].setVal<int>(val);
                ito::IntMeta* meta = m_params["stereoMatchingP2Edge"].getMetaT<ito::IntMeta>();
                meta->setMin(min);
                meta->setMax(max);
                meta->setStepSize(inc);
            }

            retVal += getParamInfo<int>(min, max, inc, val, "sgm_p1_no_edge");
            if (!retVal.containsError())
            {
                retVal += m_params["stereoMatchingP2NoEdge"].setVal<int>(val);
                ito::IntMeta* meta = m_params["stereoMatchingP2NoEdge"].getMetaT<ito::IntMeta>();
                meta->setMin(min);
                meta->setMax(max);
                meta->setStepSize(inc);
            }

            retVal += getParamInfo<int>(min, max, inc, val, "sgm_edge_sensitivity");
            if (!retVal.containsError())
            {
                retVal += m_params["stereoMatchingEdgeSensitivity"].setVal<int>(val);
                ito::IntMeta* meta = m_params["stereoMatchingEdgeSensitivity"].getMetaT<ito::IntMeta>();
                meta->setMin(min);
                meta->setMax(max);
                meta->setStepSize(inc);
            }



            /*int val;
            val = m_pParamsObj->getStereoMatchingP1Edge();
            retVal += m_params["stereoMatchingP1Edge"].setVal<int>(val);
            val = m_pParamsObj->getStereoMatchingP1NoEdge();
            retVal += m_params["stereoMatchingP1NoEdge"].setVal<int>(val);
            val = m_pParamsObj->getStereoMatchingP2Edge();
            retVal += m_params["stereoMatchingP2Edge"].setVal<int>(val);
            val = m_pParamsObj->getStereoMatchingP2NoEdge();
            retVal += m_params["stereoMatchingP2NoEdge"].setVal<int>(val);
            val = m_pParamsObj->getStereoMatchingEdgeSensitivity();
            retVal += m_params["stereoMatchingEdgeSensitivity"].setVal<int>(val);*/
        }
        if (what &sMaskBorderPixels)
        {
            int min, max, inc, val;
            retVal += getParamInfo<int>(min, max, inc, val, "mask_border_pixels_enabled");
            if (!retVal.containsError())
            {
                retVal += m_params["maskBorderPixels"].setVal<int>(val);
                ito::IntMeta* meta = m_params["maskBorderPixels"].getMetaT<ito::IntMeta>();
                meta->setMin(min);
                meta->setMax(max);
                meta->setStepSize(inc);
            }

            /*bool val = m_pParamsObj->getMaskBorderPixelsEnabled();
            retVal += m_params["maskBorderPixels"].setVal<int>(val? 1 : 0);*/
        }
        if (what & sConsistencyCheck)
        {
            int min, max, inc, val;
            retVal += getParamInfo<int>(min, max, inc, val, "consistency_check_enabled");
            if (!retVal.containsError())
            {
                retVal += m_params["consistencyCheck"].setVal<int>(val);
                ito::IntMeta* meta = m_params["consistencyCheck"].getMetaT<ito::IntMeta>();
                meta->setMin(min);
                meta->setMax(max);
                meta->setStepSize(inc);
            }

            retVal += getParamInfo<int>(min, max, inc, val, "consistency_check_sensitivity");
            if (!retVal.containsError())
            {
                retVal += m_params["consistencyCheckSensitivity"].setVal<int>(val);
                ito::IntMeta* meta = m_params["consistencyCheckSensitivity"].getMetaT<ito::IntMeta>();
                meta->setMin(min);
                meta->setMax(max);
                meta->setStepSize(inc);
            }

            /*bool val = m_pParamsObj->getConsistencyCheckEnabled();
            retVal += m_params["consistencyCheck"].setVal<int>(val ? 1 : 0);
            int i = m_pParamsObj->getConsistencyCheckSensitivity();
            retVal += m_params["consistencyCheckSensitivity"].setVal<int>(i);*/
        }
        if (what & sUniquenessCheck)
        {

            int min, max, inc, val;
            retVal += getParamInfo<int>(min, max, inc, val, "uniqueness_check_enabled");
            if (!retVal.containsError())
            {
                retVal += m_params["uniquenessCheck"].setVal<int>(val);
                ito::IntMeta* meta = m_params["uniquenessCheck"].getMetaT<ito::IntMeta>();
                meta->setMin(min);
                meta->setMax(max);
                meta->setStepSize(inc);
            }
            retVal += getParamInfo<int>(min, max, inc, val, "uniqueness_check_sensitivity");
            if (!retVal.containsError())
            {
                retVal += m_params["uniquenessCheckSensitivity"].setVal<int>(val);
                ito::IntMeta* meta = m_params["uniquenessCheckSensitivity"].getMetaT<ito::IntMeta>();
                meta->setMin(min);
                meta->setMax(max);
                meta->setStepSize(inc);
            }

            /*bool val = m_pParamsObj->getUniquenessCheckEnabled();
            retVal += m_params["uniquenessCheck"].setVal<int>(val ? 1 : 0);
            int i = m_pParamsObj->getUniquenessCheckSensitivity();
            retVal += m_params["uniquenessCheckSensitivity"].setVal<int>(i);*/

        }
        if (what & sTextureFilter)
        {
            int min, max, inc, val;
            retVal += getParamInfo<int>(min, max, inc, val, "texture_filter_enabled");
            if (!retVal.containsError())
            {
                retVal += m_params["textureFilter"].setVal<int>(val);
                ito::IntMeta* meta = m_params["textureFilter"].getMetaT<ito::IntMeta>();
                meta->setMin(min);
                meta->setMax(max);
                meta->setStepSize(inc);
            }
            retVal += getParamInfo<int>(min, max, inc, val, "texture_filter_sensitivity");
            if (!retVal.containsError())
            {
                retVal += m_params["textureFilterSensitivity"].setVal<int>(val);
                ito::IntMeta* meta = m_params["textureFilterSensitivity"].getMetaT<ito::IntMeta>();
                meta->setMin(min);
                meta->setMax(max);
                meta->setStepSize(inc);
            }
            /*bool val = m_pParamsObj->getTextureFilterEnabled();
            retVal += m_params["textureFilter"].setVal<int>(val ? 1 : 0);
            int i = m_pParamsObj->getTextureFilterSensitivity();
            retVal += m_params["textureFilterSensitivity"].setVal<int>(i);*/
        }
        if (what & sGapInterpolation)
        {
            int min, max, inc, val;
            retVal += getParamInfo<int>(min, max, inc, val, "gap_interpolation_enabled");
            if (!retVal.containsError())
            {
                retVal += m_params["gapInterpolation"].setVal<int>(val);
                ito::IntMeta* meta = m_params["gapInterpolation"].getMetaT<ito::IntMeta>();
                meta->setMin(min);
                meta->setMax(max);
                meta->setStepSize(inc);
            }
            /*bool val = m_pParamsObj->getGapInterpolationEnabled();
            retVal += m_params["gapInterpolation"].setVal<int>(val ? 1 : 0);*/
        }
        if (what & sNoiseReduction)
        {
            int min, max, inc, val;
            retVal += getParamInfo<int>(min, max, inc, val, "noise_reduction_enabled");
            if (!retVal.containsError())
            {
                retVal += m_params["noiseReduction"].setVal<int>(val);
                ito::IntMeta* meta = m_params["noiseReduction"].getMetaT<ito::IntMeta>();
                meta->setMin(min);
                meta->setMax(max);
                meta->setStepSize(inc);
            }
            /*bool val = m_pParamsObj->getNoiseReductionEnabled();
            retVal += m_params["noiseReduction"].setVal<int>(val ? 1 : 0);*/
        }
        if (what & sSpeckleFilterIterations)
        {
            int min, max, inc, val;
            retVal += getParamInfo<int>(min, max, inc, val, "speckle_filter_iterations");
            if (!retVal.containsError())
            {
                retVal += m_params["speckleFilterIterations"].setVal<int>(val);
                ito::IntMeta* meta = m_params["speckleFilterIterations"].getMetaT<ito::IntMeta>();
                meta->setMin(min);
                meta->setMax(max);
                meta->setStepSize(inc);
            }
            /*int val = m_pParamsObj->getSpeckleFilterIterations();
            retVal += m_params["speckleFilterIterations"].setVal<int>(val);*/
        }
        if (what & sAutoMode)
        {
            int min, max, inc, val;
            retVal += getParamInfo<int>(min, max, inc, val, "auto_exposure_mode");
            if (!retVal.containsError())
            {
                retVal += m_params["exposureGainMode"].setVal<int>(val);
                ito::IntMeta* meta = m_params["exposureGainMode"].getMetaT<ito::IntMeta>();
                meta->setMin(min);
                meta->setMax(max);
                meta->setStepSize(inc);
            }
            /*int val = m_pParamsObj->getAutoMode();
            retVal += m_params["exposureGainMode"].setVal<int>(val);*/
        }
        if (what & sAutoTargetIntensity)
        {
            double min, max, inc, val;
            retVal += getParamInfo<double>(min, max, inc, val, "auto_target_intensity");
            if (!retVal.containsError())
            {
                retVal += m_params["autoTargetIntensity"].setVal<double>(val);
                ito::DoubleMeta* meta = m_params["autoTargetIntensity"].getMetaT<ito::DoubleMeta>();
                meta->setMin(min);
                meta->setMax(max);
                meta->setStepSize(inc);
            }
            /*double val = m_pParamsObj->getAutoTargetIntensity();
            retVal += m_params["autoTargetIntensity"].setVal<double>(val);*/
        }
        if (what & sAutoIntensityDelta)
        {
            double min, max, inc, val;
            retVal += getParamInfo<double>(min, max, inc, val, "auto_intensity_delta");
            if (!retVal.containsError())
            {
                retVal += m_params["autoIntensityDelta"].setVal<double>(val);
                ito::DoubleMeta* meta = m_params["autoIntensityDelta"].getMetaT<ito::DoubleMeta>();
                meta->setMin(min);
                meta->setMax(max);
                meta->setStepSize(inc);
            }
            /*double val = m_pParamsObj->getAutoIntensityDelta();
            retVal += m_params["autoIntensityDelta"].setVal<double>(val);*/
        }
        if (what & sAutoTargetFrame)
        {
            int min, max, inc, val;
            retVal += getParamInfo<int>(min, max, inc, val, "auto_target_frame");
            if (!retVal.containsError())
            {
                retVal += m_params["autoTargetFrame"].setVal<int>(val);
                ito::IntMeta* meta = m_params["autoTargetFrame"].getMetaT<ito::IntMeta>();
                meta->setMin(min);
                meta->setMax(max);
                meta->setStepSize(inc);
            }
            /*int val = m_pParamsObj->getAutoTargetFrame();
            retVal += m_params["autoTargetFrame"].setVal<int>(val);*/
        }
        if (what &sAutoSkippedFrames)
        {
            int min, max, inc, val;
            retVal += getParamInfo<int>(min, max, inc, val, "auto_skipped_frames");
            if (!retVal.containsError())
            {
                retVal += m_params["autoSkippedFrames"].setVal<int>(val);
                ito::IntMeta* meta = m_params["autoSkippedFrames"].getMetaT<ito::IntMeta>();
                meta->setMin(min);
                meta->setMax(max);
                meta->setStepSize(inc);
            }
            /*int val = m_pParamsObj->getAutoSkippedFrames();
            retVal += m_params["autoSkippedFrames"].setVal<int>(val);*/
        }
        if (what & sAutoMaxExposureTime)
        {
            double min, max, inc, val;
            retVal += getParamInfo<double>(min, max, inc, val, "auto_maximum_exposure_time");
            if (!retVal.containsError())
            {
                retVal += m_params["autoMaxExposureTime"].setVal<double>(val);
                ito::DoubleMeta* meta = m_params["autoMaxExposureTime"].getMetaT<ito::DoubleMeta>();
                meta->setMin(min);
                meta->setMax(max);
                meta->setStepSize(inc);
            }
            /*double val = m_pParamsObj->getAutoMaxExposureTime();
            retVal += m_params["autoMaxExposureTime"].setVal<double>(val);*/
        }
        if (what & sAutoMaxGain)
        {
            double min, max, inc, val;
            retVal += getParamInfo<double>(min, max, inc, val, "auto_maximum_gain");
            if (!retVal.containsError())
            {
                retVal += m_params["autoMaxGain"].setVal<double>(val);
                ito::DoubleMeta* meta = m_params["autoMaxGain"].getMetaT<ito::DoubleMeta>();
                meta->setMin(min);
                meta->setMax(max);
                meta->setStepSize(inc);
            }
            /*double val = m_pParamsObj->getAutoMaxGain();
            retVal += m_params["autoMaxGain"].setVal<double>(val);*/
        }
        if (what & sManualExposureTime)
        {
            double min, max, inc, val;
            retVal += getParamInfo<double>(min, max, inc, val, "manual_exposure_time");
            if (!retVal.containsError())
            {
                retVal += m_params["manualExposureTime"].setVal<double>(val);
                ito::DoubleMeta* meta = m_params["manualExposureTime"].getMetaT<ito::DoubleMeta>();
                meta->setMin(min);
                meta->setMax(max);
                meta->setStepSize(inc);
            }
            /*double val = m_pParamsObj->getManualExposureTime();
            retVal += m_params["manualExposureTime"].setVal<double>(val);*/
        }
        if (what & sManualGain)
        {
            double min, max, inc, val;
            retVal += getParamInfo<double>(min, max, inc, val, "manual_gain");
            if (!retVal.containsError())
            {
                retVal += m_params["manualGain"].setVal<double>(val);
                ito::DoubleMeta* meta = m_params["manualGain"].getMetaT<ito::DoubleMeta>();
                meta->setMin(min);
                meta->setMax(max);
                meta->setStepSize(inc);
            }
            /*double val = m_pParamsObj->getManualGain();
            retVal += m_params["manualGain"].setVal<double>(val);*/
        }
        if (what & sAutoROI)
        {
            int min, max, inc, val;
            retVal += getParamInfo<int>(min, max, inc, val, "auto_exposure_roi_enabled");
            if (!retVal.containsError())
            {
                retVal += m_params["autoROI"].setVal<int>(val);
                ito::IntMeta* meta = m_params["autoROI"].getMetaT<ito::IntMeta>();
                meta->setMin(min);
                meta->setMax(max);
                meta->setStepSize(inc);
            }

            int min1, max1, inc1, val1;
            retVal += getParamInfo<int>(min1, max1, inc1, val1, "auto_exposure_roi_x");

            int min2, max2, inc2, val2;
            retVal += getParamInfo<int>(min2, max2, inc2, val2, "auto_exposure_roi_y");

            int min3, max3, inc3, val3;
            retVal += getParamInfo<int>(min3, max3, inc3, val3, "auto_exposure_roi_width");

            int min4, max4, inc4, val4;
            retVal += getParamInfo<int>(min4, max4, inc4, val4, "auto_exposure_roi_height");
            if (!retVal.containsError())
            {
                int roi[4];
                roi[0] = val1;
                roi[1] = val2;
                roi[2] = val3;
                roi[3] = val4;

                retVal += m_params["autoExposureGainControlROI"].setVal<int*>(roi, 4);
                ito::RectMeta* meta = m_params["autoExposureGainControlROI"].getMetaT<ito::RectMeta>();
                meta->setWidthRangeMeta(ito::RangeMeta(min3, max3, inc3));
                meta->setHeightRangeMeta(ito::RangeMeta(min4, max4, inc4));
            }

           // m_pParamsObj->getAutoROI();
        }
        if (what & sMaxFrameTimeDifference)
        {
            int min, max, inc, val;
            retVal += getParamInfo<int>(min, max, inc, val, "max_frame_time_difference_ms");
            if (!retVal.containsError())
            {
                retVal += m_params["maxFrameTimeDifference"].setVal<int>(val);
                ito::IntMeta* meta = m_params["maxFrameTimeDifference"].getMetaT<ito::IntMeta>();
                meta->setMin(min);
                meta->setMax(max);
                meta->setStepSize(inc);
            }
            /*int val = m_pParamsObj->getMaxFrameTimeDifference();
            retVal += m_params["maxFrameTimeDifference"].setVal<int>(val);*/
        }
        if (what & sTrigger)
        {
            double min, max, inc, val;
            retVal += getParamInfo<double>(min, max, inc, val, "trigger_frequency");
            if (!retVal.containsError())
            {
                retVal += m_params["triggerFrequency"].setVal<double>(val);
                ito::DoubleMeta* meta = m_params["triggerFrequency"].getMetaT<ito::DoubleMeta>();
                meta->setMin(min);
                meta->setMax(max);
                meta->setStepSize(inc);
            }
            int mini, maxi, vali, inci;
            retVal += getParamInfo<int>(mini, maxi, inci, vali, "trigger_0_enabled");
            if (!retVal.containsError())
            {
                retVal += m_params["trigger0"].setVal<int>(val);
                ito::IntMeta* meta = m_params["trigger0"].getMetaT<ito::IntMeta>();
                meta->setMin(mini);
                meta->setMax(maxi);
                meta->setStepSize(inci);
            }
            retVal += getParamInfo<int>(mini, maxi, inci, vali, "trigger_1_enabled");
            if (!retVal.containsError())
            {
                retVal += m_params["trigger1"].setVal<int>(val);
                ito::IntMeta* meta = m_params["trigger1"].getMetaT<ito::IntMeta>();
                meta->setMin(mini);
                meta->setMax(maxi);
                meta->setStepSize(inci);
            }
            retVal += getParamInfo<double>(min, max, inc, val, "trigger_0_pulse_width");
            if (!retVal.containsError())
            {
                retVal += m_params["trigger0PulseWidth"].setVal<double>(val);
                ito::DoubleMeta* meta = m_params["trigger0PulseWidth"].getMetaT<ito::DoubleMeta>();
                meta->setMin(min);
                meta->setMax(max);
                meta->setStepSize(inc);
            }
            retVal += getParamInfo<double>(min, max, inc, val, "trigger_1_pulse_width");
            if (!retVal.containsError())
            {
                retVal += m_params["trigger1PulseWidth"].setVal<double>(val);
                ito::DoubleMeta* meta = m_params["trigger1PulseWidth"].getMetaT<ito::DoubleMeta>();
                meta->setMin(min);
                meta->setMax(max);
                meta->setStepSize(inc);
            }
            retVal += getParamInfo<double>(min, max, inc, val, "trigger_1_offset");
            if (!retVal.containsError())
            {
                retVal += m_params["trigger1Offset"].setVal<double>(val);
                ito::DoubleMeta* meta = m_params["trigger1Offset"].getMetaT<ito::DoubleMeta>();
                meta->setMin(min);
                meta->setMax(max);
                meta->setStepSize(inc);
            }

            /*double val = m_pParamsObj->getTriggerFrequency();
            retVal += m_params["triggerFrequency"].setVal<double>(val);
            bool val1 = m_pParamsObj->getTrigger0Enabled();
            retVal += m_params["trigger0"].setVal<int>(val1 ? 1 : 0);
            bool val2 = m_pParamsObj->getTrigger1Enabled();
            retVal += m_params["trigger1"].setVal<int>(val2 ? 1 : 0);
            double val3 = m_pParamsObj->getTrigger0PulseWidth();
            retVal += m_params["trigger0PulseWidth"].setVal<double>(val3);
            double val4 = m_pParamsObj->getTrigger1PulseWidth();
            retVal += m_params["trigger1PulseWidth"].setVal<double>(val4);
            double val5 = m_pParamsObj->getTrigger1Offset();
            retVal += m_params["trigger1Offset"].setVal<double>(val5);*/

        }
        if (what & sAutoRecalibration)
        {
            int min, max, inc, val;
            retVal += getParamInfo<int>(min, max, inc, val, "auto_recalibration_enabled");
            if (!retVal.containsError())
            {
                retVal += m_params["autoRecalibration"].setVal<int>(val);
                ito::IntMeta* meta = m_params["autoRecalibration"].getMetaT<ito::IntMeta>();
                meta->setMin(min);
                meta->setMax(max);
                meta->setStepSize(inc);
            }
            retVal += getParamInfo<int>(min, max, inc, val, "auto_recalibration_permanent");
            if (!retVal.containsError())
            {
                retVal += m_params["saveAutoRecalibration"].setVal<int>(val);
                ito::IntMeta* meta = m_params["saveAutoRecalibration"].getMetaT<ito::IntMeta>();
                meta->setMin(min);
                meta->setMax(max);
                meta->setStepSize(inc);
            }
            /*bool val = m_pParamsObj->getAutoRecalibrationEnabled();
            retVal += m_params["autoRecalibration"].setVal<int>(val ? 1 : 0);
            val = m_pParamsObj->getSaveAutoReclabration();
            retVal += m_params["saveAutoRecalibration"].setVal<int>(val ? 1 : 0);*/

        }
        if (what & sImageFormat)
        {
            if (m_pImagePair && m_pParamsObj)
            {
                for (int i = 0; i < 2; ++i)// Changes are often only visible after the 2nd image
                {
                    while (!m_pImageTransferObj->receiveImagePair(*m_pImagePair))
                    {
                        //wait till done
                    }
                }
                ImagePair::ImageFormat format1 = m_pImagePair->getPixelFormat(0);
                ImagePair::ImageFormat format2 = m_pImagePair->getPixelFormat(1);
                if (format1 != format2)
                {
                    if (format1 == ImagePair::FORMAT_8_BIT_RGB || format2 == ImagePair::FORMAT_8_BIT_RGB)
                    {
                        retVal += ito::RetVal(ito::retError, 0, tr("camera delivers rgb image. This is not implemented yet").toLatin1().data());

                    }
                    else
                    {
                        m_params["bpp"].setVal(16);
                    }
                }
                else
                {
                    if (format1 == ImagePair::FORMAT_8_BIT_MONO)
                    {
                        m_params["bpp"].setVal(8);
                    }
                    else
                    {
                        m_params["bpp"].setVal(16);
                    }

                }

                int width = m_pImagePair->getWidth();
                int height = m_pImagePair->getHeight();
                int *roi = m_params["roi"].getVal<int*>();
                roi[0] = 0;
                roi[1] = 0;
                roi[2] = width;
                roi[3] = height;
                m_params["sizex"].setVal<int>(width);
                m_params["sizey"].setVal<int>(height);
            }
            else
            {
                retVal += ito::RetVal(ito::retError, 0, tr("ImagePair instance not available").toLatin1().data());
            }
        }
    }
    else
    {
        retVal = ito::RetVal(ito::retError, 0, QString("parameter instance not callable").toLatin1().data());
    }
    return retVal;
}
//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> inline ito::RetVal NerianSceneScanPro::getParamInfo(_Tp & min, _Tp & max, _Tp &inc,_Tp & value, const char * name)
{
    ito::RetVal retVal = ito::RetVal(ito::retOk);
    if (m_pParamsObj)
    {
        std::map<std::string,ParameterInfo> paramMap =  m_pParamsObj->getAllParameters();
        std::map<std::string, ParameterInfo>::iterator it = paramMap.find(name);


        if ( it != paramMap.end())
        {
            ParameterInfo::ParameterType t = it->second.getType();
            try {
                switch (t)
                {
                case visiontransfer::ParameterInfo::TYPE_INT:
                    inc = it->second.getInc<_Tp>();
                    min = it->second.getMin<_Tp>();
                    max = it->second.getMax<_Tp>();
                    value = it->second.getValue<_Tp>();
                    break;
                case visiontransfer::ParameterInfo::TYPE_DOUBLE:
                    inc = 0.0;
                    min = it->second.getMin<_Tp>();
                    max = it->second.getMax<_Tp>();
                    value = it->second.getValue<_Tp>();
                    break;
                case visiontransfer::ParameterInfo::TYPE_BOOL:
                    inc = 1;
                    min = 0;
                    max = 1;
                    bool val;
                    val = it->second.getValue<_Tp>();
                    value = val ? 1 : 0;
                    break;
                default:
                    break;
                }
            }
            catch (std::exception &ex)
            {
                retVal += ito::RetVal(ito::retError, 0, tr(QString("Error while reading parameter %1: %2").arg(name).arg(ex.what()).toLatin1().data()).toLatin1().data());
            }


        }
        else
        {
            retVal += ito::RetVal(ito::retError, 0, tr(QString("%1 is not included in parameter map.").arg(name).toLatin1().data()).toLatin1().data());
        }
    }
    else
    {
        retVal += ito::RetVal(ito::retError, 0, QString("parameter instance not callable").toLatin1().data());
    }
    return retVal;
}

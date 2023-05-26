/* ********************************************************************
    Plugin "Ximea" for itom software
    URL: http://www.twip-os.com
    Copyright (C) 2015, twip optical solutions GmbH
    Copyright (C) 2018, Institut fuer Technische Optik, Universitaet Stuttgart

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
#include "dialogXimea.h"
#include "gitVersion.h"

#include "common/sharedFunctionsQt.h"

#include "common/helperCommon.h"

#if linux
    #include <dlfcn.h>
    #include <unistd.h>
#endif
#include <QFile>
#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>
#include <qmetaobject.h>

//int XimeaInterface::m_instCounter = 5;  // initialization starts with five due to normal boards are 0..4

static char InitList[5] = {0, 0, 0, 0, 0};  /*!<A map with successfull initialized boards (max = 5) */
static char Initnum = 0;    /*!< Number of successfull initialized cameras */

#ifndef XI_PRMM_DIRECT_UPDATE //workaround for very old APIs
    #define XI_PRMM_DIRECT_UPDATE ""
#endif

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal XimeaInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(Ximea)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal XimeaInterface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(Ximea)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
XimeaInterface::XimeaInterface()
{
    m_type = ito::typeDataIO | ito::typeGrabber;
    setObjectName("Ximea");

    m_description = QObject::tr("Ximea xiQ-Camera");
    m_detaildescription = QObject::tr("Plugin for cameras from XIMEA that run with the XIMEA API. \n\
This plugin has been tested using monchrome USB3.0 cameras (e.g. MQ013MG-E2, MQ042RG-CM) under Windows.");
    m_author = "C. Kohler, twip optical solutions GmbH, Stuttgart, J. Krauter, M. Gronle, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("LGPL / do not copy Ximea-DLLs");
    m_aboutThis = QObject::tr(GITVERSION);

    ito::Param paramVal = ito::Param("cameraNumber", ito::ParamBase::Int | ito::ParamBase::In, 0, 254, 0, "The index of the addressed camera starting with 0");
    m_initParamsOpt.append(paramVal);

    paramVal = ito::Param("restoreLast", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, "Toggle if the driver should try to connect to the last initialized camera");
    m_initParamsOpt.append(paramVal);

    paramVal = ito::Param("bandwidthLimit", ito::ParamBase::Int | ito::ParamBase::In, 0, 100000, 0, "bandwidth limit in Mb/sec. If 0 the maximum bandwidth of the USB3 controller is used [default]. The allowed value range depends on the device and will be checked at startup.");
    m_initParamsOpt.append(paramVal);

	paramVal = ito::Param("lensAvialable", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, "Toggle if lens settings are avialable");
	m_initParamsOpt.append(paramVal);
}

//----------------------------------------------------------------------------------------------------------------------------------
XimeaInterface::~XimeaInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal Ximea::showConfDialog(void)
{
   return apiShowConfigurationDialog(this, new DialogXimea(this));
}

//----------------------------------------------------------------------------------------------------------------------------------
Ximea::Ximea() :
    AddInGrabber(),
    m_saveParamsOnClose(false),
    m_handle(NULL),
    m_isgrabbing(false),
    m_acqRetVal(ito::retOk),
    m_pvShadingSettings(NULL),
    ximeaLib(NULL),
    m_family(familyUnknown),
    m_numFrameBurst(1),
    m_maxOutputBitDepth(8),
    m_channelNumberChanged(false)
{
    //register exec functions
    QVector<ito::Param> pMand = QVector<ito::Param>()
        << ito::Param("dark_image", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Dark Image, if null, empty image will be generated").toLatin1().data())
        << ito::Param("white_image", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("White Image, if null, empty image will be generated").toLatin1().data())
        << ito::Param("x0", ito::ParamBase::Int | ito::ParamBase::In, 0, 1280, 0, tr("Position of ROI in x").toLatin1().data())
        << ito::Param("y0", ito::ParamBase::Int | ito::ParamBase::In, 0, 1024, 0, tr("Position of ROI in y").toLatin1().data());
    QVector<ito::Param> pOpt = QVector<ito::Param>();

    QVector<ito::Param> pOut = QVector<ito::Param>();
    registerExecFunc("initialize_shading", pMand, pOpt, pOut, tr("Initialize pixel shading correction. At the moment you can only use one set of data which will be rescaled each time"));

    pMand = QVector<ito::Param>()
        << ito::Param("illumination", ito::ParamBase::Int | ito::ParamBase::In, 0, 9, 0, tr("Current intensity value").toLatin1().data());
    pOpt = QVector<ito::Param>();
    pOut = QVector<ito::Param>();
    registerExecFunc("update_shading", pMand, pOpt, pOut, tr("Change value of the shading correction"));

    pMand = QVector<ito::Param>()
        << ito::Param("integration_time", ito::ParamBase::Double, 0.000000, 0.000000, 0.000000, tr("Integrationtime of CCD programmed in s").toLatin1().data())
        << ito::Param("shading_correction_factor", ito::ParamBase::DoubleArray | ito::ParamBase::In, NULL, tr("Corresponding values for shading correction").toLatin1().data());
    pOpt = QVector<ito::Param>();
    pOut = QVector<ito::Param>();
    registerExecFunc("shading_correction_values", pMand, pOpt, pOut, tr("Change value of the shading correction"));
    /*
    pMand = QVector<ito::Param>();
    pOpt = QVector<ito::Param>() << ito::Param("dark_image", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Dark Image, if null, empty image will be generated").toLatin1().data())
                                 << ito::Param("white_image", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("White Image, if null, empty image will be generated").toLatin1().data());
    registerExecFunc("update_shading", pMand, pOpt, pOut, tr("Initialize pixel shading correction"));
    registerExecFunc("calculateShading", pMand, pOpt, pOut, tr("Initialize pixel shading correction"));
    pOpt.clear();
    */
    //end register exec functions

   ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly | ito::ParamBase::NoAutosave, "Ximea", tr("name of the camera").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("integration_time", ito::ParamBase::Double, 0.00000, 0.000, 0.0000, tr("Exposure time (in seconds).").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("gain", ito::ParamBase::Double, 0.0, 1.0, 0.0, tr("Gain in % (the percentage is mapped to the dB-values).").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("offset", ito::ParamBase::Double | ito::ParamBase::Readonly, 0.0, 1.0, 0.0, tr("Currently not used.").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);

   paramVal = ito::Param("gamma", ito::ParamBase::Double, 0.0, 1.0, 1.0, tr("Luminosity gamma value in %.").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("gammaColor", ito::ParamBase::Double | ito::ParamBase::Readonly, 0.3, 1.0, 1.0, tr("Chromaticity gamma value in %. Only for color cameras.").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("sharpness", ito::ParamBase::Double, 0.0, 1.0, 1.0, tr("Sharpness strength in %.").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);

   paramVal = ito::Param("hdr_enable", ito::ParamBase::Int, 0, 1, 0, tr("Enable HDR mode. default is OFF (not supported by all devices).").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("hdr_knee1", ito::ParamBase::Int, 0, 100, 40, tr("First kneepoint (% of sensor saturation - not supported by all devices).").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("hdr_knee2", ito::ParamBase::Int, 0, 100, 60, tr("Second kneepoint (% of sensor saturation - not supported by all devices).").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("hdr_it1", ito::ParamBase::Int, 0, 100, 50, tr("Exposure time of first slope (in % of exposure time - not supported by all devices).").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("hdr_it2", ito::ParamBase::Int, 0, 100, 75, tr("Exposure time of second slope (in % of exposure time - not supported by all devices).").toLatin1().data());
   m_params.insert(paramVal.getName(), paramVal);

   int roi[] = {0, 0, 0, 0};
   paramVal = ito::Param("roi", ito::ParamBase::IntArray, 4, roi, tr("ROI (x, y, width, height) [this replaces the values x0, x1, y0, y1].").toLatin1().data());
   ito::RectMeta *rm = new ito::RectMeta(ito::RangeMeta(0, 2048), ito::RangeMeta(0, 2048));
   paramVal.setMeta(rm, true);
   m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("x0", ito::ParamBase::Int, 0, 0, 0, tr("First horizontal index within current ROI (deprecated, use parameter 'roi' instead).").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("y0", ito::ParamBase::Int, 0, 0, 0, tr("First vertical index within current ROI (deprecated, use parameter 'roi' instead).").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("x1", ito::ParamBase::Int, 0, 0, 0, tr("Last horizontal index within current ROI (deprecated, use parameter 'roi' instead).").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("y1", ito::ParamBase::Int, 0, 0, 0, tr("Last vertical index within current ROI (deprecated, use parameter 'roi' instead).").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizex", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 0, 0, tr("Width of ROI (number of columns).").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizey", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 0, 0, tr("Height of ROI (number of rows).").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizez", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 25, 1, tr("number of channels. This Param is readonly and controlled via \"filter_pattern_size\" ").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("timeout", ito::ParamBase::Double, 0.0, 300.0 , 2.0, tr("Acquisition timeout in s.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("bpp", ito::ParamBase::Int, 8, 8, 8, tr("Bit depth of the output data from camera in bpp (can differ from sensor bit depth). For color cameras set bpp to 32 in order to obtain the color data.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("max_sensor_bitdepth", ito::ParamBase::Int | ito::ParamBase::Readonly, 8, 16, 8, tr("maximum bitdepth of the sensor.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("binning", ito::ParamBase::Int, 101, 404, 101, tr("1x1 (101), 2x2 (202) or 4x4 (404) binning if available. See param 'binning_type' for setting the way binning is executed.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("binning_type", ito::ParamBase::Int, XI_BINNING, XI_SKIPPING, XI_BINNING, tr("Type of binning if binning is enabled. %1: pixels are interpolated, %2: pixels are skipped (faster).").arg(XI_BINNING).arg(XI_SKIPPING).toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("trigger_mode", ito::ParamBase::Int, XI_TRG_OFF, XI_TRG_SOFTWARE, XI_TRG_SOFTWARE, tr("Set triggermode, %1: free run, %2: ext. rising edge, %3: ext. falling edge, %4: software.").arg(XI_TRG_OFF).arg(XI_TRG_EDGE_RISING).arg(XI_TRG_EDGE_FALLING).arg(XI_TRG_SOFTWARE).toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("trigger_selector", ito::ParamBase::Int, XI_TRG_SEL_FRAME_START, XI_TRG_SEL_FRAME_BURST_ACTIVE, XI_TRG_SEL_FRAME_START , tr("Set trigger selector, %1: Exposure Frame Start, %2: Exposure Frame duration, %3: Frame Burst Start, %4: Frame Burst duration (this parameter was called trigger_mode2 in a previous version of this plugin).").arg(XI_TRG_SEL_FRAME_START).arg(XI_TRG_SEL_EXPOSURE_ACTIVE).arg(XI_TRG_SEL_FRAME_BURST_START).arg(XI_TRG_SEL_FRAME_BURST_ACTIVE).toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("frame_burst_count", ito::ParamBase::Int, 0, 3, 1, tr("Define and set the number of frames in a burst (trigger_mode2 should be set to XI_TRG_SEL_FRAME_BURST_START.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("buffers_queue_size", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 3, 1, tr("Number of buffers in the queue.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("timing_mode", ito::ParamBase::Int, XI_ACQ_TIMING_MODE_FREE_RUN, XI_ACQ_TIMING_MODE_FRAME_RATE, XI_ACQ_TIMING_MODE_FREE_RUN, tr("Acquisition timing: %1: free run (default), %2: by frame rate.").arg(XI_ACQ_TIMING_MODE_FREE_RUN).arg(XI_ACQ_TIMING_MODE_FRAME_RATE).toLatin1().data());

    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("framerate", ito::ParamBase::Double, 0.0, 1000.0, 60.0, tr("Framerate of image acquisition (in fps). This parameter reflects the current framerate. If timing_mode is in XI_ACQ_TIMING_MODE_FREE_RUN (%1, default), the framerate is readonly and fixed to the highest possible rate. For xiQ cameras only, timing_mode can be set to XI_ACQ_TIMING_MODE_FRAME_RATE (%2) and the framerate is adjustable to a fixed value.").arg(XI_ACQ_TIMING_MODE_FREE_RUN).arg(XI_ACQ_TIMING_MODE_FRAME_RATE).toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("cam_number", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::NoAutosave, 0, 4, 0, tr("Index of the camera device.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("bad_pixel", ito::ParamBase::Int, 0, 1, 1, tr("Enable bad pixel correction.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("aeag", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 1, 0, tr("Enable / Disable Automatic exposure / gain. AEAG is disabled in the current implementation.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("gpo_mode", ito::ParamBase::IntArray, NULL, tr("Set the output pin modes for all available gpo pins. This is a list whose lengths corresponds to the number of available pins. Use gpo_mode[i] to access the i-th pin.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("gpi_mode", ito::ParamBase::IntArray, NULL, tr("Set the input pin modes for all available gpi pins. This is a list whose lengths corresponds to the number of available pins. Use gpi_mode[i] to access the i-th pin. %1: Off, %2: trigger, %3: external signal input (not implemented by Ximea api)").arg(XI_GPI_OFF).arg(XI_GPI_TRIGGER).arg(XI_GPI_EXT_EVENT).toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("gpi_level", ito::ParamBase::IntArray | ito::ParamBase::Readonly, NULL, tr("Current level of all available gpi pins. (0: low level, 1: high level)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("serial_number", ito::ParamBase::String |ito::ParamBase::Readonly, "unknown", tr("Serial number of device.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sensor_type", ito::ParamBase::String |ito::ParamBase::Readonly, "unknown", tr("Sensor type of the attached camera").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("device_type", ito::ParamBase::String |ito::ParamBase::Readonly, "unknown", tr("Device type (1394, USB2.0, CURRERA, ...)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("api_version", ito::ParamBase::String |ito::ParamBase::Readonly, "unknown", tr("XIMEA API version").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("device_driver", ito::ParamBase::String |ito::ParamBase::Readonly, "unknown", tr("Current device driver version").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("color_camera", ito::ParamBase::Int |ito::ParamBase::Readonly, 0, 1, 0, tr("0: monochrome camera, 1: color camera - set bpp to 32 to obtain color image.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("filter_pattern_size", ito::ParamBase::String , NULL, tr("Size of the hyperspectral bayer pattern size (number of channels). Allowed are 1, 16 and 25 channels.").toLatin1().data());
    ito::StringMeta *sm = new ito::StringMeta(ito::StringMeta::RegExp,"(\\b1\\b|\\b16\\b|\\b25\\b)");
    paramVal.setMeta(sm, true);
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("filter_pattern_offset_x", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 1, 0, tr("Offset of the hyperspectral bayer pattern in x direction").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("filter_pattern_offset_y", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 1, 0, tr("Offset of the hyperspectral bayer pattern in y direction").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
	paramVal = ito::Param("aperture_value", ito::ParamBase::Double | ito::ParamBase::Readonly, 0.0, 5.0 ,0.0 , tr("Current aperture Value").toLatin1().data());
	m_params.insert(paramVal.getName(), paramVal);




    if (hasGuiSupport())
    {
        //now create dock widget for this plugin
        DockWidgetXimea *m_dockWidget = new DockWidgetXimea(getID(), this);

        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, m_dockWidget);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
Ximea::~Ximea()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Ximea::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    int bandwidthLimit = paramsOpt->at(2).getVal<int>(); //0 auto bandwidth calculation
    int icam_number = (*paramsOpt)[0].getVal<int>();
    int iscolor = 0;
	bool lensAvialable = paramsOpt->at(3).getVal<int>() == 1;
    XI_RETURN ret;

    QFile paramFile;

    // Load parameterlist from XML-file
    int loadPrev = (*paramsOpt)[1].getVal<int>();

    DWORD pSize = sizeof(int);
    DWORD intSize = sizeof(int);
    DWORD floatSize = sizeof(float);
    DWORD charSize = sizeof(char);
    XI_PRM_TYPE intType = xiTypeInteger;
    XI_PRM_TYPE strType = xiTypeString;
    XI_PRM_TYPE floatType = xiTypeFloat;
    ParamMapIterator it;
    retValue += LoadLib();

    if (!retValue.containsError())
    {

        m_params["cam_number"].getVal<int>(icam_number);
        Initnum++;  // so we have a new running instance of this grabber (or not)

        if (++InitList[icam_number] > 1)    // It does not matter if the rest works or not. The close command will fix this anyway
        {
            retValue = ito::RetVal(ito::retError, 0, tr("Camera already initialized. Try another camera number.").toLatin1().data());
        }
        else
        {
            ret = pxiOpenDevice(icam_number, &m_handle);
            if (!m_handle || ret != XI_OK)
            {
                m_handle = NULL;
                retValue += checkError(ret, "pxiOpenDevice", QString::number(icam_number));
            }
            else
            {
                //especially under Windows, pxiOpenDevice does not fail if camera is already opened by another process. However
                //any xiSetParam command will fail and no image acquisition is possible (http://www.ximea.com/support/wiki/apis/XiAPI). This should be checked in the following small code:
                float testGain = 0.0;
                if (pxiSetParam(m_handle, XI_PRM_GAIN, &testGain, sizeof(float), xiTypeFloat) == XI_RESOURCE_OR_FUNCTION_LOCKED)
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("this camera cannot be configured and used since it is propably used by another process").toLatin1().data());
                }
            }

            if (!retValue.containsError())
            {
                m_params["cam_number"].setVal<int>(icam_number);

                char strBuf[1024];
                DWORD strBufSize = 1024 * sizeof(char);
                retValue += checkError(pxiGetParam(m_handle, XI_PRM_DEVICE_NAME, &strBuf, &strBufSize, &strType), "get: " XI_PRM_DEVICE_NAME);
                if (!retValue.containsError())
                {
                    m_params["sensor_type"].setVal<char*>(strBuf);
                }

                char serial_number[20] = "";
                DWORD strSize = 20 * sizeof(char);
                retValue += checkError(pxiGetParam(m_handle, XI_PRM_DEVICE_SN, &serial_number, &strSize, &strType), "get:  XI_PRM_DEVICE_SN");
                if (!retValue.containsError())
                {
                    m_params["serial_number"].setVal<char*>(serial_number);
                    m_identifier = QString("%1 (SN:%2)").arg(strBuf).arg(serial_number);
                }

                strBufSize = 1024 * sizeof(char);
                retValue += checkError(pxiGetParam(m_handle, XI_PRM_DEVICE_TYPE, &strBuf, &strBufSize, &strType), "get: " XI_PRM_DEVICE_TYPE);
                if (!retValue.containsError())
                {
                    m_params["device_type"].setVal<char*>(strBuf);
                }

                strBufSize = 1024 * sizeof(char);
                retValue += checkError(pxiGetParam(m_handle, XI_PRM_API_VERSION, &strBuf, &strBufSize, &strType), "get: " XI_PRM_API_VERSION);
                if (!retValue.containsError())
                {
                    m_params["api_version"].setVal<char*>(strBuf);
                }

                strBufSize = 1024 * sizeof(char);
                retValue += checkError(pxiGetParam(m_handle, XI_PRM_DRV_VERSION, &strBuf, &strBufSize, &strType), "get: " XI_PRM_DRV_VERSION);
                if (!retValue.containsError())
                {
                    m_params["device_driver"].setVal<char*>(strBuf);
                }

                retValue += checkError(pxiGetParam(m_handle, XI_PRM_IMAGE_IS_COLOR, &iscolor, &pSize, &intType), "get: " XI_PRM_IMAGE_IS_COLOR);
                if (!retValue.containsError())
                {
                    m_params["color_camera"].setVal<int>(iscolor);
                    m_params["gammaColor"].setFlags(iscolor ? 0 : ito::ParamBase::Readonly);
                }

#if defined(XI_PRM_DEVICE_MODEL_ID)

                int device_model_id = 0;
                retValue += checkError(pxiGetParam(m_handle, XI_PRM_DEVICE_MODEL_ID, &device_model_id, &pSize, &intType), "get: " XI_PRM_DEVICE_MODEL_ID);
                if (!retValue.containsError())
                {
                    //given by file m3Identify.h
                    if ((device_model_id >= 0x1 && device_model_id <= 0x18) || device_model_id == 0x116 || \
                        device_model_id == 0x21 || device_model_id == 0x75 || device_model_id == 0x79  || device_model_id == 0x80  || device_model_id == 0x81 \
                         || device_model_id == 0x83  || device_model_id == 0x84  || device_model_id == 0x85  || device_model_id == 0x86 \
                          || device_model_id == 0x90 || device_model_id == 0x91 || device_model_id == 0x92 || device_model_id == 0x93 \
                           || device_model_id == 0x96 || (device_model_id >= 0x100 && device_model_id <= 0x108) || device_model_id == 0x111 || device_model_id == 0x114 \
                            || device_model_id == 0x115)
                    {
                        m_family = familyMR;
                    }
                    else if (device_model_id == 0x142 /*MODEL_ID_CAL_Simulator*/)
                    {
                        m_family = familyUnknown;
                    }
                    else if (device_model_id >= 0x148 && device_model_id <= 0x151)
                    {
                        m_family = familyMT;
                    }
                    else if (device_model_id >= 0x132 && device_model_id <= 0x133)
                    {
                        m_family = familyCB;
                    }
                    else if ((device_model_id >= 0x120 && device_model_id <= 0x140))
                    {
                        //0x132, 0x133 are familyCB
                        m_family = familyMD;
                    }
                    else if (device_model_id == 0x110 || device_model_id == 0x112 || device_model_id == 0x113 || device_model_id == 0x82 || device_model_id == 0x87 || device_model_id == 0x88)
                    {
                        m_family = familyMH;
                    }
                    else if ((device_model_id >= 0x25 && device_model_id <= 0x35) || device_model_id == 0x74)
                    {
                        m_family = familyCURRERA;
                    }
                    else if ((device_model_id >= 0x20 && device_model_id <= 0x24) || device_model_id == 0x62 || device_model_id == 0x117  || device_model_id == 0x118)
                    {
                        //0x21 is familyMR, 0x62 is familyMU
                        m_family = familyMU;
                    }
                    else if ((device_model_id >= 0x49 && device_model_id <= 0x78) || device_model_id == 0x134 || device_model_id == 0x135  || device_model_id == 0x136 || (device_model_id >= 0x141 && device_model_id <= 0x147))
                    {
                        //0x74 is currera, 0x75 is mr, 0x79 is mr, 0x142 is simulator
                        m_family = familyMQ;
                    }
                }
#endif
            }

            if (!retValue.containsError())
            {
                //get available bandwidth in Mb/sec
                //int availableBandwidth;
                //retValue += getErrStr(pxiGetParam(m_handle, XI_PRM_AVAILABLE_BANDWIDTH, &availableBandwidth, &pSize, &intType), "XI_PRM_AVAILABLE_BANDWIDTH", QString::number(availableBandwidth));
                //std::cout << "available bandwidth: " << availableBandwidth << std::endl;
                if (bandwidthLimit > 0) //manually set bandwidthLimit
                {
                    retValue += setXimeaParam(XI_PRM_AUTO_BANDWIDTH_CALCULATION, XI_OFF);
                    retValue += setXimeaParam(XI_PRM_LIMIT_BANDWIDTH, bandwidthLimit);
                }
            }

            // Load parameterlist from XML-file
            if (loadPrev && !retValue.containsError())
            {
                QMap<QString, ito::Param> paramListXML;
                if (!retValue.containsError())
                {
                    retValue += ito::generateAutoSaveParamFile(this->getBasePlugin()->getFilename(), paramFile);
                }

                // Read parameter list from file to paramListXML
                if (!retValue.containsError())
                {
                    retValue += ito::loadXML2QLIST(&paramListXML, QString::number(icam_number), paramFile);
                }

                // Merge parameter list from file to paramListXML with current mparams

                paramListXML.clear();
            }

            if (!retValue.containsError())
            {
                ret = pxiGetParam(m_handle, XI_PRM_GPO_SELECTOR XI_PRM_INFO_MAX, &m_numGPOPins, &pSize, &intType);

                if (ret == 12)
                {
                    m_params["gpi_level"].setFlags(ito::ParamBase::Readonly);
                    m_params["gpi_mode"].setFlags(ito::ParamBase::Readonly);
                    m_params["gpo_mode"].setFlags(ito::ParamBase::Readonly);
                }
                else
                {
                    retValue += checkError(ret, "get XI_PRM_GPO_SELECTOR:max");
                    retValue += checkError(pxiGetParam(m_handle, XI_PRM_GPI_SELECTOR XI_PRM_INFO_MAX, &m_numGPIPins, &pSize, &intType), "get XI_PRM_GPI_SELECTOR:max");
                    int dummy[] = { 0,0,0,0,0,0,0,0 };
                    m_params["gpi_level"].setMeta(new ito::IntArrayMeta(0, 1, 1, m_numGPIPins, m_numGPIPins, 1), true);
                    m_params["gpi_level"].setVal<int*>(dummy, m_numGPIPins);
                    m_params["gpi_mode"].setMeta(new ito::IntArrayMeta(0, 1, 1, m_numGPIPins, m_numGPIPins, 1), true);
                    m_params["gpi_mode"].setVal<int*>(dummy, m_numGPIPins);
                    m_params["gpo_mode"].setMeta(new ito::IntArrayMeta(0, 12, 1, m_numGPOPins, m_numGPIPins, 1), true);
                    m_params["gpo_mode"].setVal<int*>(dummy, m_numGPOPins);
                }

                retValue += readCameraIntParam(XI_PRM_HDR, "hdr_enable", false);
                retValue += readCameraIntParam(XI_PRM_HDR_T1, "hdr_it1", false);
                retValue += readCameraIntParam(XI_PRM_HDR_T2, "hdr_it2", false);
                retValue += readCameraIntParam(XI_PRM_KNEEPOINT1, "hdr_knee1", false);
                retValue += readCameraIntParam(XI_PRM_KNEEPOINT2, "hdr_knee2", false);
                retValue += readCameraIntParam(XI_PRM_TRG_SOURCE, "trigger_mode", true);
                retValue += readCameraIntParam(XI_PRM_ACQ_FRAME_BURST_COUNT, "frame_burst_count", false);
                m_numFrameBurst = m_params["frame_burst_count"].getVal<int>();
                retValue += readCameraIntParam(XI_PRM_BUFFERS_QUEUE_SIZE, "buffers_queue_size", false);

#ifdef XI_PRM_ACQ_TIMING_MODE
                retValue += readCameraIntParam(XI_PRM_ACQ_TIMING_MODE, "timing_mode", false);

                if (m_family == familyMQ || m_family == familyUnknown)
                {
                    //xiQ cameras may support the fixed-framerate feature, for unknown cameras (or newer cameras) we let the camera decide if it is possible (via the max value of timing_mode)
                }
                else
                {
                    //usually even not all xiQ cameras support fixed framerates, however we don't want to implement a fix restriction table here
                    m_params["timing_mode"].setVal<int>(XI_ACQ_TIMING_MODE_FREE_RUN);
                    m_params["timing_mode"].setMeta(new ito::IntMeta(XI_ACQ_TIMING_MODE_FREE_RUN, XI_ACQ_TIMING_MODE_FREE_RUN), true);
                    m_params["timing_mode"].setFlags(ito::ParamBase::Readonly);
                }
#else
                m_params["timing_mode"].setFlags(ito::ParamBase::Readonly);
#endif
                //check the min-max-bitdepth by changing the format and reading the data_bit_depth value
                ret = pxiGetParam(m_handle, XI_PRM_OUTPUT_DATA_BIT_DEPTH, &m_maxOutputBitDepth, &pSize, &intType);
                if (ret == 12)
                {
                    m_params["max_sensor_bitdepth"].setFlags(ito::ParamBase::Readonly);
                }
                else
                {
                    retValue += checkError(ret, "get XI_PRM_OUTPUT_DATA_BIT_DEPTH");
                    m_params["max_sensor_bitdepth"].setVal<int>(m_maxOutputBitDepth);
                }

                int bitppix = XI_MONO8;
                retValue += checkError(pxiSetParam(m_handle, XI_PRM_IMAGE_DATA_FORMAT, &bitppix, pSize, intType), "set XI_PRM_IMAGE_DATA_FORMAT", QString::number(bitppix));
                if (!retValue.containsError())
                {
                    m_params["bpp"].setMeta(new ito::IntMeta(8, 8, 2), true);
                    //static_cast<ito::IntMeta*>(m_params["bpp"].getMeta())->setMin(8);
                    //static_cast<ito::IntMeta*>(m_params["bpp"].getMeta())->setMax(8);
                    //static_cast<ito::IntMeta*>(m_params["bpp"].getMeta())->setStepSize(2);
                }

                bitppix = XI_MONO16;
                int temp;
                ito::RetVal retValue1 = checkError(pxiSetParam(m_handle, XI_PRM_IMAGE_DATA_FORMAT, &bitppix, pSize, intType), "set XI_PRM_IMAGE_DATA_FORMAT", QString::number(bitppix));

                if (!retValue1.containsError())
                {
                    retValue += checkError(pxiGetParam(m_handle, XI_PRM_OUTPUT_DATA_BIT_DEPTH XI_PRM_INFO_MAX, &temp, &pSize, &intType), "get XI_PRM_OUTPUT_DATA_BIT_DEPTH MAX");
                    m_maxOutputBitDepth = qMax(m_maxOutputBitDepth, temp);
                    m_params["max_sensor_bitdepth"].setVal<int>(m_maxOutputBitDepth);

                    static_cast<ito::IntMeta*>(m_params["bpp"].getMeta())->setMax(m_maxOutputBitDepth);
                }

                if (iscolor)
                {
                    //check color format
                    bitppix = XI_RGB32;
                    ito::RetVal retValue2 = checkError(pxiSetParam(m_handle, XI_PRM_IMAGE_DATA_FORMAT, &bitppix, pSize, intType), "set XI_PRM_IMAGE_DATA_FORMAT", QString::number(bitppix));
                    if (!retValue2.containsError())
                    {
                        static_cast<ito::IntMeta*>(m_params["bpp"].getMeta())->setMax(32);
                    }
                }
				if (!retValue.containsError())
				{
					//check if lens settings are avialable
/*					XI_SWITCH value = XI_ON;
					XI_RETURN error = pxiSetParam(m_handle, XI_PRM_LENS_MODE, &value, sizeof(int), intType);
					if (error != 100)
					{
						m_params["aperture_value"].setFlags(0);
					}*/

					if (lensAvialable)
					{
						m_params["aperture_value"].setFlags(0);
					}

				}

                // reset timestamp for MQ and MD cameras
                if (m_family == familyMD || m_family == familyMQ)
                {
#if defined(XI_TS_RST_SRC_SW) && defined(XI_TS_RST_ARM_ONCE)
                    // reset camera timestamp
                    int val = XI_TS_RST_SRC_SW;
                    pxiSetParam(m_handle, XI_PRM_TS_RST_SOURCE, &val, pSize, intType);
                    val = XI_TS_RST_ARM_ONCE;
                    pxiSetParam(m_handle, XI_PRM_TS_RST_MODE, &val, pSize, intType);
#endif
                }

                int badpix = 0;// bad pixel correction default = 0
                ret = pxiSetParam(m_handle, XI_PRM_BPC, &badpix, pSize, intType);
                if (ret == 12)
                {
                    m_params["bad_pixel"].setFlags(ito::ParamBase::Readonly);
                }
                else
                {
                    retValue += checkError(ret, "set XI_PRM_BPC", QString::number(badpix));
                    retValue += readCameraIntParam(XI_PRM_BPC, "bad_pixel", false);
                }



                //disable AEAG
                retValue += checkError(pxiSetParam(m_handle, XI_PRM_AEAG, &badpix, pSize, intType), "set XI_PRM_AEAG", QString::number(badpix));
                retValue += readCameraIntParam(XI_PRM_AEAG, "aeag", false);

                //disable LUT
                pxiSetParam(m_handle, XI_PRM_LUT_EN, &badpix, pSize, intType);

                m_params["filter_pattern_size"].setVal<const char*>("1");
            }

            if (!retValue.containsError())
            {
                int val = XI_BP_SAFE;
                retValue += checkError(pxiSetParam(m_handle, XI_PRM_BUFFER_POLICY, &val, pSize, intType), "set XI_PRM_BUFFER_POLICY", QString::number(val));
            }

            int device_exist = 0;

            if (!retValue.containsError())
            {
                if (m_handle)
                {
                    retValue += checkError(pxiGetParam(m_handle, XI_PRM_IS_DEVICE_EXIST, &device_exist, &pSize, &intType), "get XI_PRM_IS_DEVICE_EXIST");
                }
                if (device_exist != 1)
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("Camera is not connected or does not work properly!").toLatin1().data());
                }
                else
                {
                    retValue += synchronizeCameraSettings();
                }
            }

            if (!retValue.containsError())
            {
                retValue += checkData();
            }
        }

        if (!retValue.containsError())
        {
            m_saveParamsOnClose = true;
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
ito::RetVal Ximea::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if (this->m_handle == NULL)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("Camera handle deleted before closing procedure").toLatin1().data());
    }
    else
    {
        setGrabberStarted(1);
        retValue += this->stopDevice(0);
        Sleep(50);

        retValue += checkError(pxiCloseDevice(m_handle), "pxiCloseDevice");
        m_handle = NULL;
    }

    int nr = m_params["cam_number"].getVal<int>();
    InitList[nr] = 0;
    Initnum--; // so we closed a further instance of this grabber

    if (!Initnum)
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
        retValue += ito::RetVal(ito::retWarning, 0, tr("DLLs not unloaded due to still living instances of Ximea-Cams").toLatin1().data());
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Ximea::getErrStr(const int error, const QString &command, const QString &value)
{
    return checkError(error, command, value);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Ximea::checkError(const XI_RETURN &error, const QString &command, const QString &value /*= QString()*/)
{
    if (error == XI_OK)
    {
        return ito::retOk;
    }

    QString msg = "";

    switch (error)
    {
        case 0:
            return ito::RetVal(ito::retOk, error, "");
        break;
        //errors from m3Api.h
        case 1:
            msg = tr("Invalid handle during sending").toLatin1().data();
        break;
        case 2:
            msg = tr("Register read error").toLatin1().data();
        break;
        case 3:
            msg = tr("Register write error").toLatin1().data();
        break;
        case 4:
            msg = tr("Freeing resources error").toLatin1().data();
        break;
        case 5:
            msg = tr("Freeing channel error").toLatin1().data();
        break;
        case 6:
            msg = tr("Freeing bandwith error").toLatin1().data();
        break;
        case 7:
            msg = tr("Read block error").toLatin1().data();
        break;
        case 8:
            msg = tr("Write block error").toLatin1().data();
        break;
        case 9:
            msg = tr("No image").toLatin1().data();
        break;
        case 10:
            msg = tr("Timeout").toLatin1().data();
        break;
        case 11:
            msg = tr("Invalid arguments supplied").toLatin1().data();
        break;
        case 12:
            msg = tr("Not supported").toLatin1().data();
        break;
        case 13:
            msg = tr("Attach buffers error").toLatin1().data();
        break;
        case 14:
            msg = tr("Overlapped result").toLatin1().data();
        break;
        case 15:
            msg = tr("Memory allocation error").toLatin1().data();
        break;
        case 16:
            msg = tr("DLL context is NULL").toLatin1().data();
        break;
        case 17:
            msg = tr("DLL context is non zero").toLatin1().data();
        break;
        case 18:
            msg = tr("DLL context exists").toLatin1().data();
        break;
        case 19:
            msg = tr("Too many devices connected").toLatin1().data();
        break;
        case 20:
            msg = tr("Camera context error").toLatin1().data();
        break;
        case 21:
            msg = tr("Unknown hardware").toLatin1().data();
        break;
        case 22:
            msg = tr("Invalid TM file").toLatin1().data();
        break;
        case 23:
            msg = tr("Invalid TM tag").toLatin1().data();
        break;
        case 24:
            msg = tr("Incomplete TM").toLatin1().data();
        break;
        case 25:
            msg = tr("Bus reset error").toLatin1().data();
        break;
        case 26:
            msg = tr("Not implemented").toLatin1().data();
        break;
        case 27:
            msg = tr("Shading too bright").toLatin1().data();
        break;
        case 28:
            msg = tr("Shading too dark").toLatin1().data();
        break;
        case 29:
            msg = tr("Gain is too low").toLatin1().data();
        break;
        case 30:
            msg = tr("Invalid bad pixel list").toLatin1().data();
        break;
        case 31:
            msg = tr("Bad pixel list realloc error").toLatin1().data();
        break;
        case 32:
            msg = tr("Invalid pixel list").toLatin1().data();
        break;
        case 33:
            msg = tr("Invalid Flash File System").toLatin1().data();
        break;
        case 34:
            msg = tr("Invalid profile").toLatin1().data();
        break;
        case 35:
            msg = tr("Invalid bad pixel list").toLatin1().data();
        break;
        case 36:
            msg = tr("Invalid buffer").toLatin1().data();
        break;
        case 38:
            msg = tr("Invalid data").toLatin1().data();
        break;
        case 39:
            msg = tr("Timing generator is busy").toLatin1().data();
        break;
        case 40:
            msg = tr("Wrong operation open/write/read/close").toLatin1().data();
        break;
        case 41:
            msg = tr("Acquisition already started").toLatin1().data();
        break;
        case 42:
            msg = tr("Old version of device driver installed to the system").toLatin1().data();
        break;
        case 43:
            msg = tr("To get error code please call GetLastError function").toLatin1().data();
        break;
        case 44:
            msg = tr("Data can't be processed").toLatin1().data();
        break;
        case 45:
            msg = tr("Error occured and acquisition has been stoped or didn't start").toLatin1().data();
        break;
        case 46:
            msg = tr("Acquisition has been stoped with error").toLatin1().data();
        break;
        case 47:
            msg = tr("Input ICC profile missed or corrupted").toLatin1().data();
        break;
        case 48:
            msg = tr("Output ICC profile missed or corrupted").toLatin1().data();
        break;
        case 49:
            msg = tr("Device not ready to operate").toLatin1().data();
        break;
        case 50:
            msg = tr("Shading too contrast").toLatin1().data();
        break;
        case 51:
            msg = tr("Module already initialized").toLatin1().data();
        break;
        case 52:
            msg = tr("Application doesn't enough privileges(one or more applications opened)").toLatin1().data();
        break;
        case 53:
            msg = tr("installed driver incompatible with current software").toLatin1().data();
        break;
        case 54:
            msg = tr("TM file was not loaded successfully from resources").toLatin1().data();
        break;
        case 55:
            msg = tr("Device has been reseted, abnormal initial state").toLatin1().data();
        break;
        case 56:
            msg = tr("No Devices found").toLatin1().data();
        break;
        case 57:
            msg = tr("Resource (device) or function  locked by mutex").toLatin1().data();
        break;

        //errors from xiApi.h
        case 100:
            msg = tr("unknown parameter").toLatin1().data();
        break;
        case 101:
            msg = tr("wrong parameter value").toLatin1().data();
        break;
        case 103:
            msg = tr("wrong parameter type").toLatin1().data();
        break;
        case 104:
            msg = tr("wrong parameter size").toLatin1().data();
        break;
        case 105:
            msg = tr("input buffer too small").toLatin1().data();
        break;
        case 106:
            msg = tr("parameter not supported").toLatin1().data();
        break;
        case 107:
            msg = tr("parameter info not supported").toLatin1().data();
        break;
        case 108:
            msg = tr("data format not supported").toLatin1().data();
        break;
        case 109:
            msg = tr("read only parameter").toLatin1().data();
        break;
        case 110:
            msg = tr("no devices found").toLatin1().data();
        break;
        case 111:
            msg = tr("this camera does not support currently available bandwidth").toLatin1().data();
        break;
        default:
            msg = tr("unknown error code").toLatin1().data();
        break;
    }

    QString msg_final;

    if (value != "")
    {
        msg_final = tr("ximea m3api error '%1' (%2). Value: %3").arg(msg, command, value);
    }
    else
    {
        msg_final = tr("ximea m3api error '%1' (%2)").arg(msg, command);
    }

    return ito::RetVal(ito::retError, error, msg_final.toLatin1().data());
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Ximea::LoadLib(void)
{
    ito::RetVal retValue = ito::retOk;

    if (!ximeaLib)
    {
#if linux
        ximeaLib = dlopen("libm3api.so", RTLD_LAZY);
        if (!ximeaLib)
        {
            char *error = dlerror();
            return ito::RetVal(ito::retError, 0, error);
        }
#else
#if _WIN64
#if UNICODE
#if defined(USE_API_4_10)
    ximeaLib = LoadLibrary(L"./lib/xiapi64.dll"); //L"./lib/m3apiX64.dll");
#else
    ximeaLib = LoadLibrary(L"./lib/xiapi64.dll"); //L"./lib/m3apiX64.dll");
#endif
#else
#if defined(USE_API_4_10) || defined(USE_API_3_16)
    ximeaLib = LoadLibrary("./lib/xiapiX64.dll"); //"./lib/m3apiX64.dll");
#else
    ximeaLib = LoadLibrary("./lib/xiapi64.dll"); //"./lib/m3apiX64.dll");
#endif

#endif
        if (!ximeaLib)
        {
			int error = GetLastError();
            return ito::RetVal::format(ito::retError, 0, tr("LoadLibrary(\"xiapiX64.dll\"). Error code: %i").toLatin1().data(), error);
        }
#else
#if UNICODE
 #if defined(USE_API_4_10) || defined(USE_API_3_16)
      ximeaLib = LoadLibrary(L"./lib/m3api.dll");
#else
     ximeaLib = LoadLibrary(L"./lib/xiapi32.dll");
 #endif
#else
#if defined(USE_API_4_10) || defined(USE_API_3_16)
     ximeaLib = LoadLibrary(L"./lib/m3api.dll");
#else
    ximeaLib = LoadLibrary(L"./lib/xiapi32.dll");
#endif
#endif
        if (!ximeaLib)
        {
			int error = GetLastError();
            return ito::RetVal::format(ito::retError, 0, tr("LoadLibrary(\"xiapiX64.dll\"). Error code: %i").toLatin1().data(), error);
        }
#endif
#endif

#if linux
        if ((pxiGetNumberDevices = (XI_RETURN(*)(PDWORD)) dlsym(ximeaLib, "xiGetNumberDevices")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiGetNumberDevices").toLatin1().data());

        if ((pxiOpenDevice = (XI_RETURN(*)(DWORD,PHANDLE)) dlsym(ximeaLib, "xiOpenDevice")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiOpenDevice").toLatin1().data());

        if ((pxiCloseDevice = (XI_RETURN(*)(HANDLE)) dlsym(ximeaLib, "xiCloseDevice")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiCloseDevice").toLatin1().data());

        if ((pxiStartAcquisition = (XI_RETURN(*)(HANDLE)) dlsym(ximeaLib, "xiStartAcquisition")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiStartAcquisition").toLatin1().data());

        if ((pxiStopAcquisition = (XI_RETURN(*)(HANDLE)) dlsym(ximeaLib, "xiStopAcquisition")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiStopAcquisition").toLatin1().data());

        if ((pxiGetImage = (XI_RETURN(*)(HANDLE,DWORD,LPXI_IMG)) dlsym(ximeaLib, "xiGetImage")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiGetImage").toLatin1().data());

        if ((pxiSetParam = (XI_RETURN(*)(HANDLE,const char*,void*,DWORD,XI_PRM_TYPE)) dlsym(ximeaLib, "xiSetParam")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiSetParam").toLatin1().data());

        if ((pxiGetParam = (XI_RETURN(*)(HANDLE,const char*,void*,DWORD*,XI_PRM_TYPE*)) dlsym(ximeaLib, "xiGetParam")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiGetParam").toLatin1().data());


#if defined(USE_API_4_10)
        if ((pUpdateFrameShading = (MM40_RETURN(*)(HANDLE,HANDLE,LPMMSHADING)) dlsym(ximeaLib, "mmUpdateFrameShading")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function mmUpdateFrameShading").toLatin1().data());

        if ((pCalculateShading = (MM40_RETURN(*)(HANDLE, LPMMSHADING, DWORD, DWORD, LPWORD, LPWORD)) dlsym(ximeaLib, "mmCalculateShading")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function mmCalculateShading").toLatin1().data());

        /*if ((pCalculateShadingRaw = (MM40_RETURN(*)(LPMMSHADING, DWORD, DWORD, LPWORD, LPWORD)) dlsym(ximeaLib, "mmCalculateShadingRaw")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function mmCalculateShading").toLatin1().data());*/


        if ((pInitializeShading  = (MM40_RETURN(*)(HANDLE, LPMMSHADING,  DWORD , DWORD , WORD , WORD)) dlsym(ximeaLib, "mmInitializeShading")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function mmInitializeShading").toLatin1().data());

        /*if ((pSetShadingRaw = (MM40_RETURN(*)(LPMMSHADING)) dlsym(ximeaLib, "mmSetShadingRaw")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function mmSetShadingRaw").toLatin1().data());*/

        if ((pProcessFrame = (MM40_RETURN(*)(HANDLE)) dlsym(ximeaLib, "mmProcessFrame")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function mmProcessFrame").toLatin1().data());
#endif defined(USE_API_4_10)

#else //#linux
        if ((pxiGetNumberDevices = (XI_RETURN(*)(PDWORD)) GetProcAddress(ximeaLib, "xiGetNumberDevices")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiGetNumberDevices").toLatin1().data());

        if ((pxiOpenDevice = (XI_RETURN(*)(DWORD,PHANDLE)) GetProcAddress(ximeaLib, "xiOpenDevice")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiOpenDevice").toLatin1().data());

        if ((pxiCloseDevice = (XI_RETURN(*)(HANDLE)) GetProcAddress(ximeaLib, "xiCloseDevice")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiCloseDevice").toLatin1().data());

        if ((pxiStartAcquisition = (XI_RETURN(*)(HANDLE)) GetProcAddress(ximeaLib, "xiStartAcquisition")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiStartAcquisition").toLatin1().data());

        if ((pxiStopAcquisition = (XI_RETURN(*)(HANDLE)) GetProcAddress(ximeaLib, "xiStopAcquisition")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiStopAcquisition").toLatin1().data());

        if ((pxiGetImage = (XI_RETURN(*)(HANDLE,DWORD,LPXI_IMG)) GetProcAddress(ximeaLib, "xiGetImage")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiGetImage").toLatin1().data());

        if ((pxiSetParam = (XI_RETURN(*)(HANDLE,const char*,void*,DWORD,XI_PRM_TYPE)) GetProcAddress(ximeaLib, "xiSetParam")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiSetParam").toLatin1().data());

        if ((pxiGetParam = (XI_RETURN(*)(HANDLE,const char*,void*,DWORD*,XI_PRM_TYPE*)) GetProcAddress(ximeaLib, "xiGetParam")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function xiGetParam").toLatin1().data());

#if defined(USE_API_4_10)
        if ((pUpdateFrameShading = (MM40_RETURN(*)(HANDLE,HANDLE,LPMMSHADING)) GetProcAddress(ximeaLib, "mmUpdateFrameShading")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function mmUpdateFrameShading").toLatin1().data());

        if ((pCalculateShading = (MM40_RETURN(*)(HANDLE, LPMMSHADING, DWORD, DWORD, LPWORD, LPWORD)) GetProcAddress(ximeaLib, "mmCalculateShading")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function mmCalculateShading").toLatin1().data());


        /*if ((pCalculateShadingRaw = (MM40_RETURN(*)(LPMMSHADING, DWORD, DWORD, LPWORD, LPWORD)) GetProcAddress(ximeaLib, "mmCalculateShadingRaw")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function mmCalculateShadingRaw").toLatin1().data());*/

        if ((pInitializeShading  = (MM40_RETURN(*)(HANDLE, LPMMSHADING,  DWORD , DWORD , WORD , WORD)) GetProcAddress(ximeaLib, "mmInitializeShading")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function mmInitializeShading").toLatin1().data());

        /*if ((pSetShadingRaw = (MM40_RETURN(*)(LPMMSHADING)) GetProcAddress(ximeaLib, "mmSetShadingRaw")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function mmSetShadingRaw").toLatin1().data());*/

        if ((pProcessFrame = (MM40_RETURN(*)(HANDLE)) GetProcAddress(ximeaLib, "mmProcessFrame")) == NULL)
            retValue += ito::RetVal(ito::retError, 0, tr("Cannot get function mmProcessFrame").toLatin1().data());
#endif //!defined(USE_API_4_10)

#endif //linux
    }

    if ((retValue != ito::retOk))
    {
        if (!Initnum)
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
        if (key == "gpi_level")
        {
            retValue += synchronizeCameraSettings(sGpiGpo);
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
    QString key;
    bool hasIndex;
    int index;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;

    XI_PRM_TYPE intType = xiTypeInteger;
    XI_PRM_TYPE floatType = xiTypeFloat;
    DWORD intSize = sizeof(int);
    DWORD floatSize = sizeof(float);

    int running = 0;

    //int trigger_mode = XI_TRG_OUT;    //in new api trg_out does not exist anymore, so we just use free run

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (!retValue.containsError())
    {
        //gets the parameter key from m_params map (read-only is not allowed and leads to ito::retError).
        retValue += apiGetParamFromMapByKey(m_params, key, it, true);
    }

    if (!retValue.containsError())
    {
        retValue += apiValidateAndCastParam(*it, *val, false, true, true);
    }

    if (!retValue.containsError())
    {
        if (grabberStartedCount() && key != "integration_time" && key != "gain") //gain and integration_time does not need to stop the camera
        {
            running = grabberStartedCount();
            setGrabberStarted(1);
            retValue += this->stopDevice(0);
        }

        if (QString::compare(key, "bpp", Qt::CaseInsensitive) == 0)
        {
            int bitppix = val->getVal<int>();
            int format = 0;
            switch (bitppix)
            {
            case 8:
                format = XI_MONO8;
                break;
            case 10:
            case 12:
            case 14:
            case 16:
                if (bitppix <= m_maxOutputBitDepth)
                {
                    format = XI_MONO16;
                }
                else
                {
                    retValue = ito::RetVal::format(ito::retError, 0, tr("maximum gray-value bit depth is %i").toLatin1().data(), m_maxOutputBitDepth);
                }
                break;
            case 32:
                format = XI_RGB32;
				bitppix = m_params["max_sensor_bitdepth"].getVal<int>();
                break;
            default:
                retValue = ito::RetVal(ito::retError, 0, tr("bpp value not supported").toLatin1().data());
            }

            if (!retValue.containsError())
            {
                retValue += checkError(pxiSetParam(m_handle, XI_PRM_IMAGE_DATA_FORMAT, &format, sizeof(int), xiTypeInteger), "set XI_PRM_IMAGE_DATA_FORMAT", QString::number(format));
				retValue += checkError(pxiSetParam(m_handle, XI_PRM_OUTPUT_DATA_BIT_DEPTH, &bitppix, sizeof(int), xiTypeInteger), "set XI_PRM_OUTPUT_DATA_BIT_DEPTH", QString::number(bitppix));
            }

#ifdef XI_PRM_IMAGE_DATA_FORMAT_RGB32_ALPHA
			//does not work for api 4.04 or 4.06
            /*if (!retValue.containsError() && bpp == XI_RGB32)
            {
                bpp = 255;
                retValue += checkError(pxiSetParam(m_handle, XI_PRM_IMAGE_DATA_FORMAT_RGB32_ALPHA, &bpp, sizeof(int), xiTypeInteger), "set XI_PRM_IMAGE_DATA_FORMAT_RGB32_ALPHA", QString::number(255));
            }*/
#endif

            retValue += synchronizeCameraSettings(sBpp | sExposure | sRoi);
        }
        else if (QString::compare(key, "binning", Qt::CaseInsensitive) == 0)
        {
            int bin = val->getVal<int>(); //requested binning value from user
            if (bin != 101 && bin != 202 && bin != 404)
            {
                retValue = ito::RetVal(ito::retError, 0, tr("Binning value must be 101 (1x1), 202 (2x2) or 404 (4x4) (depending on supported binning values)").toLatin1().data());
            }
            else
            {
                bin = (bin % 100); //101 -> 1, 202 -> 2, 404 -> 4
                retValue += checkError(pxiSetParam(m_handle, XI_PRM_DOWNSAMPLING, &bin, sizeof(int), xiTypeInteger), "set XI_PRM_DOWNSAMPLING", QString::number(bin));
            }

            retValue += synchronizeCameraSettings(sBinning | sRoi | sExposure);
        }
        else if (QString::compare(key, "binning_type", Qt::CaseInsensitive) == 0)
        {
            int type = val->getVal<int>();
            retValue += checkError(pxiSetParam(m_handle, XI_PRM_DOWNSAMPLING, &type, sizeof(int), xiTypeInteger), "set XI_PRM_DOWNSAMPLING", QString::number(type));
            retValue += readCameraIntParam(XI_PRM_DOWNSAMPLING_TYPE, "binning_type", false);
        }
        else if (QString::compare(key, "bad_pixel", Qt::CaseInsensitive) == 0)
        {
            int enable = val->getVal<int>();
            retValue += checkError(pxiSetParam(m_handle, XI_PRM_BPC, &enable, sizeof(int), xiTypeInteger), "set XI_PRM_BPC", QString::number(enable));
            retValue += readCameraIntParam(XI_PRM_BPC, "bad_pixel", false);
        }
        else if (QString::compare(key, "gpo_mode", Qt::CaseInsensitive) == 0)
        {
            const int *newvals = val->getVal<int*>();
            const int *vals = it->getVal<int*>();

            for (int i = 1; i <= m_numGPOPins; ++i)
            {
                if (newvals[i-1] != vals[i-1])
                {
                    int mode = newvals[i-1];
                    retValue += checkError(pxiSetParam(m_handle, XI_PRM_GPO_SELECTOR, &i, sizeof(int), xiTypeInteger), "set XI_PRM_GPO_SELECTOR", QString::number(i));
                    retValue += checkError(pxiSetParam(m_handle, XI_PRM_GPO_MODE, &mode, sizeof(int), xiTypeInteger), "set XI_PRM_GPO_MODE", QString::number(mode));
                }
            }

            retValue += synchronizeCameraSettings(sGpiGpo);
        }
        else if (QString::compare(key, "gpi_mode", Qt::CaseInsensitive) == 0)
        {
            const int *newvals = val->getVal<int*>();
            const int *vals = it->getVal<int*>();

            for (int i = 1; i <= m_numGPIPins; ++i)
            {
                if (newvals[i-1] != vals[i-1])
                {
                    int mode = newvals[i-1];
                    retValue += checkError(pxiSetParam(m_handle, XI_PRM_GPI_SELECTOR, &i, sizeof(int), xiTypeInteger), "set XI_PRM_GPI_SELECTOR", QString::number(i));
                    retValue += checkError(pxiSetParam(m_handle, XI_PRM_GPI_MODE, &mode, sizeof(int), xiTypeInteger), "set XI_PRM_GPI_MODE", QString::number(mode));
                }
            }

            retValue += synchronizeCameraSettings(sGpiGpo);
        }
        else if (QString::compare(key, "hdr_enable", Qt::CaseInsensitive) == 0)
        {
            int enable = val->getVal<int>() > 0 ? 1 : 0;
            int intTime1 = (int)m_params["hdr_it1"].getVal<int>();
            int intTime2 = (int)m_params["hdr_it2"].getVal<int>();
            int knee1 = (int)m_params["hdr_knee1"].getVal<int>();
            int knee2 = (int)m_params["hdr_knee2"].getVal<int>();
            retValue += checkError(pxiSetParam(m_handle, XI_PRM_HDR , &enable, sizeof(int), intType), XI_PRM_HDR, QString::number(enable));
            if (!retValue.containsError())
                it->copyValueFrom(&(*val)); //copy value from user to m_params, represented by iterator it
        }
        else if (QString::compare(key, "hdr_it1", Qt::CaseInsensitive) == 0)
        {
            int hdr1 = val->getVal<int>();
            retValue += checkError(pxiSetParam(m_handle, XI_PRM_HDR_T1 , &hdr1, sizeof(int), intType), XI_PRM_HDR_T1, QString::number(hdr1));
            if (!retValue.containsError())
                it->copyValueFrom(&(*val)); //copy value from user to m_params, represented by iterator it
        }
        else if (QString::compare(key, "hdr_it2", Qt::CaseInsensitive) == 0)
        {
            int hdr2 = val->getVal<int>();
            retValue += checkError(pxiSetParam(m_handle, XI_PRM_HDR_T2 , &hdr2, sizeof(int), intType), XI_PRM_HDR_T2, QString::number(hdr2));
            if (!retValue.containsError())
                it->copyValueFrom(&(*val)); //copy value from user to m_params, represented by iterator it
        }
        else if (QString::compare(key, "hdr_knee1", Qt::CaseInsensitive) == 0)
        {
            int knee1 = val->getVal<int>();
            retValue += checkError(pxiSetParam(m_handle, XI_PRM_KNEEPOINT1 , &knee1, sizeof(int), intType), XI_PRM_KNEEPOINT1, QString::number(knee1));
            if (!retValue.containsError())
                it->copyValueFrom(&(*val)); //copy value from user to m_params, represented by iterator it
        }
        else if (QString::compare(key, "hdr_knee2", Qt::CaseInsensitive) == 0)
        {
            int knee2 = val->getVal<int>();
            retValue += checkError(pxiSetParam(m_handle, XI_PRM_KNEEPOINT2 , &knee2, sizeof(int), intType), XI_PRM_KNEEPOINT2, QString::number(knee2));
            if (!retValue.containsError())
                it->copyValueFrom(&(*val)); //copy value from user to m_params, represented by iterator it
        }
        else if (QString::compare(key, "integration_time", Qt::CaseInsensitive) == 0)
        {
            int integration_time = secToMusec(val->getVal<double>());
            retValue += checkError(pxiSetParam(m_handle, XI_PRM_EXPOSURE XI_PRMM_DIRECT_UPDATE, &integration_time, sizeof(int), xiTypeInteger), "set XI_PRM_EXPOSURE", QString("%1 ms").arg(integration_time));
            int timeout = secToMusec(m_params["timeout"].getVal<double>());
            if (timeout < integration_time)
            {
                m_params["timeout"].setVal<double>(musecToSec(integration_time + 1000000));
            }
            retValue += synchronizeCameraSettings(sExposure | sFrameRate | sGain);
        }
        else if (QString::compare(key, "sharpness", Qt::CaseInsensitive) == 0)
        {
            float minimum, maximum;
            pxiGetParam(m_handle, XI_PRM_SHARPNESS XI_PRM_INFO_MIN, &minimum, &floatSize, &floatType);
            pxiGetParam(m_handle, XI_PRM_SHARPNESS XI_PRM_INFO_MAX, &maximum, &floatSize, &floatType);
            float sharpness = minimum + val->getVal<double>() * (maximum - minimum);
            retValue += checkError(pxiSetParam(m_handle, XI_PRM_SHARPNESS, &sharpness, sizeof(float), xiTypeFloat), "set XI_PRM_SHARPNESS", QString::number(sharpness));
            retValue += synchronizeCameraSettings(sSharpness);
        }
        else if (QString::compare(key, "gamma", Qt::CaseInsensitive) == 0)
        {
            float minimum, maximum;
            pxiGetParam(m_handle, XI_PRM_GAMMAY XI_PRM_INFO_MIN, &minimum, &floatSize, &floatType);
            pxiGetParam(m_handle, XI_PRM_GAMMAY XI_PRM_INFO_MAX, &maximum, &floatSize, &floatType);
            float gamma = minimum + val->getVal<double>() * (maximum - minimum);
            retValue += checkError(pxiSetParam(m_handle, XI_PRM_GAMMAY, &gamma, sizeof(float), xiTypeFloat), "set XI_PRM_GAMMAY", QString::number(gamma));
            retValue += synchronizeCameraSettings(sGamma);
        }
        else if (QString::compare(key, "gammaColor", Qt::CaseInsensitive) == 0)
        {
            float minimum, maximum;
            pxiGetParam(m_handle, XI_PRM_GAMMAC XI_PRM_INFO_MIN, &minimum, &floatSize, &floatType);
            pxiGetParam(m_handle, XI_PRM_GAMMAC XI_PRM_INFO_MAX, &maximum, &floatSize, &floatType);
            float gamma = minimum + val->getVal<double>() * (maximum - minimum);
            retValue += checkError(pxiSetParam(m_handle, XI_PRM_GAMMAC, &gamma, sizeof(float), xiTypeFloat), "set XI_PRM_GAMMAC", QString::number(gamma));
            retValue += synchronizeCameraSettings(sGamma);
        }
        else if (QString::compare(key, "frame_burst_count", Qt::CaseInsensitive) == 0)
        {
            int trigger_mode = m_params["trigger_mode"].getVal<int>();
            int trigger_selector = m_params["trigger_selector"].getVal<int>();
            int count = val->getVal<int>();

            if (count == 1)
            {
                int numBuffers = std::max((int)(m_params["buffers_queue_size"].getMin()), 2);
                retValue += checkError(pxiSetParam(m_handle, XI_PRM_ACQ_FRAME_BURST_COUNT, &count, sizeof(int), intType), "set:" XI_PRM_ACQ_FRAME_BURST_COUNT, "1");
                retValue += checkError(pxiSetParam(m_handle, XI_PRM_BUFFERS_QUEUE_SIZE, &numBuffers, sizeof(int), intType), "set:" XI_PRM_BUFFERS_QUEUE_SIZE, "2");
            }
            else
            {
                if (trigger_mode == XI_TRG_OFF || trigger_selector != XI_TRG_SEL_FRAME_BURST_START)
                {
                    retValue += ito::RetVal::format(ito::retError, 0, tr("frame_burst_count != 1 can only be applied if trigger_selector = frame_burst_start (%i) and trigger_mode != off (0)").toLatin1().data(), XI_TRG_SEL_FRAME_BURST_START);
                }
                else if (count > m_params["buffers_queue_size"].getMax() || count == 0)
                {
                    retValue += ito::RetVal::format(ito::retWarning, 0, tr("frame_burst_count > %i or infinite frame_burst_count (0) is not recommended since the maximum number of buffers in this plugin is %i").toLatin1().data(), m_params["buffers_queue_size"].getMax(), m_params["buffers_queue_size"].getMax());
                }

                if (!retValue.containsError())
                {
                    int numBuffers = std::min((int)m_params["buffers_queue_size"].getMax(), std::max((int)m_params["buffers_queue_size"].getMin()+1,count+1));
                    retValue += checkError(pxiSetParam(m_handle, XI_PRM_ACQ_FRAME_BURST_COUNT, &count, sizeof(int), intType), "set:" XI_PRM_ACQ_FRAME_BURST_COUNT);
                    retValue += checkError(pxiSetParam(m_handle, XI_PRM_BUFFERS_QUEUE_SIZE, &numBuffers, sizeof(int), intType), "set:" XI_PRM_BUFFERS_QUEUE_SIZE);
                    if (QString::compare(m_params["filter_pattern_size"].getVal<char*>(), "1") != 1)
                    {
                        retValue += ito::RetVal(ito::retWarning, 0, tr("Parallel usage of burst mode and hyperspectral filter pattern is not implemented yet. Number of bursts is set to one again.").toLatin1().data());
                        QSharedPointer<ito::ParamBase> ptr(new ito::ParamBase("filter_pattern_size", ito::ParamBase::String, "1"));
                        this->setParam(ptr);
                    }
                }

            }

            retValue += readCameraIntParam(XI_PRM_ACQ_FRAME_BURST_COUNT, "frame_burst_count", false);
            retValue += readCameraIntParam(XI_PRM_BUFFERS_QUEUE_SIZE, "buffers_queue_size", false);


            m_numFrameBurst = m_params["frame_burst_count"].getVal<int>();
        }
        else if (QString::compare(key, "trigger_mode", Qt::CaseInsensitive) == 0)
        {
            int trigger_mode = val->getVal<int>();
            retValue += checkError(pxiSetParam(m_handle, XI_PRM_TRG_SOURCE, &trigger_mode, sizeof(int), xiTypeInteger), "set XI_PRM_TRG_SOURCE", QString::number(trigger_mode));
            retValue += synchronizeCameraSettings(sTriggerMode | sTriggerSelector | sFrameRate | sExposure);
        }

        else if (QString::compare(key, "trigger_selector", Qt::CaseInsensitive) == 0)
        {
            int trigger_mode2 = val->getVal<int>();
            retValue += checkError(pxiSetParam(m_handle, XI_PRM_TRG_SELECTOR, &trigger_mode2, sizeof(int), xiTypeInteger), "set XI_PRM_TRG_SELECTOR", QString::number(trigger_mode2));
            retValue += synchronizeCameraSettings(sTriggerMode | sTriggerSelector | sFrameRate | sExposure);
        }

        else if (QString::compare(key, "timing_mode", Qt::CaseInsensitive) == 0)
        {
            int timing_mode = val->getVal<int>();
            retValue += checkError(pxiSetParam(m_handle, XI_PRM_ACQ_TIMING_MODE, &timing_mode, sizeof(int), xiTypeInteger), "set XI_PRM_ACQ_TIMING_MODE", QString::number(timing_mode));
            retValue += readCameraIntParam(XI_PRM_ACQ_TIMING_MODE, "timing_mode", false);
            retValue += synchronizeCameraSettings(sFrameRate);

        }
        else if (QString::compare(key, "framerate", Qt::CaseInsensitive) == 0)
        {
            float frameRate = val->getVal<double>();//TODO set Readonly, if not available
            retValue += checkError(pxiSetParam(m_handle, XI_PRM_FRAMERATE, &frameRate, sizeof(float), xiTypeFloat), "set XI_PRM_FRAMERATE", QString::number(frameRate));
            retValue += synchronizeCameraSettings(sExposure | sFrameRate);
        }
        else if (QString::compare(key, "gain", Qt::CaseInsensitive) == 0)
        {
            float minimum, maximum;
            pxiGetParam(m_handle, XI_PRM_GAIN XI_PRM_INFO_MIN, &minimum, &floatSize, &floatType);
            pxiGetParam(m_handle, XI_PRM_GAIN XI_PRM_INFO_MAX, &maximum, &floatSize, &floatType);
            float gain = minimum + val->getVal<double>() * (maximum - minimum);
            retValue += checkError(pxiSetParam(m_handle, XI_PRM_GAIN XI_PRMM_DIRECT_UPDATE, &gain, sizeof(float), xiTypeFloat), XI_PRM_GAIN XI_PRMM_DIRECT_UPDATE, QString::number(gain));
            retValue += synchronizeCameraSettings(sGain | sFrameRate | sExposure);
        }
        else if (QString::compare(key, "x1", Qt::CaseInsensitive) == 0)
        {
            int size = val->getVal<int>() - m_params["x0"].getVal<int>() + 1;
            QSharedPointer<ito::ParamBase> val(new ito::ParamBase("roi[2]", ito::ParamBase::Int, size));
            retValue += setParam(val, NULL);
        }
        else if (QString::compare(key, "y1", Qt::CaseInsensitive) == 0)
        {
            int size = val->getVal<int>() - m_params["y0"].getVal<int>() + 1;
            QSharedPointer<ito::ParamBase> val(new ito::ParamBase("roi[3]", ito::ParamBase::Int, size));
            retValue += setParam(val, NULL);
        }
        else if (QString::compare(key, "x0", Qt::CaseInsensitive) == 0)
        {
            QSharedPointer<ito::ParamBase> val(new ito::ParamBase("roi[0]", ito::ParamBase::Int, val->getVal<int>()));
            retValue += setParam(val, NULL);
        }
        else if (QString::compare(key, "y0", Qt::CaseInsensitive) == 0)
        {
            QSharedPointer<ito::ParamBase> val(new ito::ParamBase("roi[1]", ito::ParamBase::Int, val->getVal<int>()));
            retValue += setParam(val, NULL);
        }

        else if (QString::compare(key, "roi", Qt::CaseInsensitive) == 0)
        {
            if (!hasIndex)
            {
                if (val->getLen() !=4)
                {
                    retValue += ito::RetVal(ito::retError, 0, "roi must have 4 values");
                }
            }

            if (!retValue.containsError())
            {
                int *roi_old = m_params["roi"].getVal<int*>();
                int *roi_set = val->getVal<int*>();
                int offset_x = hasIndex && index == 0 ? val->getVal<int>() : roi_set[0];
                int offset_y = hasIndex && index == 1 ? val->getVal<int>() : roi_set[1];
                int width =    hasIndex && index == 2 ? val->getVal<int>() : roi_set[2];
                int height =   hasIndex && index == 3 ? val->getVal<int>() : roi_set[3];

                ito::RectMeta *rm = static_cast<ito::RectMeta*>(m_params["roi"].getMeta());
                int offset_x_old = roi_old[0];
                int offset_y_old = roi_old[1];
                int width_old = roi_old[2];
                int height_old = roi_old[3];

                //set x roi
                if ((offset_x + width_old) <= width)
                {
                    retValue += checkError(pxiSetParam(m_handle, XI_PRM_OFFSET_X, &offset_x, sizeof(int), xiTypeInteger), "set XI_PRM_OFFSET_X", QString::number(offset_x));
                    retValue += checkError(pxiSetParam(m_handle, XI_PRM_WIDTH, &width, sizeof(int), xiTypeInteger), "set XI_PRM_WIDTH", QString::number(width));
                }
                else
                {
                    retValue += checkError(pxiSetParam(m_handle, XI_PRM_WIDTH, &width, sizeof(int), xiTypeInteger), "set XI_PRM_WIDTH", QString::number(width));
                    retValue += checkError(pxiSetParam(m_handle, XI_PRM_OFFSET_X, &offset_x, sizeof(int), xiTypeInteger), "set XI_PRM_OFFSET_X", QString::number(offset_x));
                }

                // set y roi
                if ((offset_y + height_old) <= height)
                {
                    retValue += checkError(pxiSetParam(m_handle, XI_PRM_OFFSET_Y, &offset_y, sizeof(int), xiTypeInteger), "set XI_PRM_OFFSET_Y", QString::number(offset_y));
                    retValue += checkError(pxiSetParam(m_handle, XI_PRM_HEIGHT, &height, sizeof(int), xiTypeInteger), "set XI_PRM_HEIGHT", QString::number(height));
                }
                else
                {
                    retValue += checkError(pxiSetParam(m_handle, XI_PRM_HEIGHT, &height, sizeof(int), xiTypeInteger), "set XI_PRM_HEIGHT", QString::number(height));
                    retValue += checkError(pxiSetParam(m_handle, XI_PRM_OFFSET_Y, &offset_y, sizeof(int), xiTypeInteger), "set XI_PRM_OFFSET_Y", QString::number(offset_y));
                }

                if (!retValue.containsError())
                {
                    retValue += synchronizeCameraSettings(sExposure | sRoi | sFrameRate);
                }
            }
        }
        else if (QString::compare(key, "filter_pattern_size", Qt::CaseInsensitive) == 0)
        {
           char* size = val->getVal<char*>();
           if (QString::compare(size, m_params["filter_pattern_size"].getVal<char*>()) != 0)
           {
               m_channelNumberChanged = true;
           }
           m_params["filter_pattern_size"].setVal<char*>(size);
           int channels = QString(size).toInt();
           if (channels == 1)
           {
               m_params["filter_pattern_offset_x"].setFlags(ito::ParamBase::Readonly);
               m_params["filter_pattern_offset_y"].setFlags(ito::ParamBase::Readonly);
           }
           else if(channels == 25)
           {
               m_params["filter_pattern_offset_x"].setFlags(0);
               m_params["filter_pattern_offset_x"].setMeta(new ito::IntMeta(0, 4, 1), true);
               m_params["filter_pattern_offset_y"].setFlags(0);
               m_params["filter_pattern_offset_y"].setMeta(new ito::IntMeta(0, 4, 1), true);
           }
           else if (channels == 16)
           {
               m_params["filter_pattern_offset_x"].setFlags(0);
               m_params["filter_pattern_offset_x"].setMeta(new ito::IntMeta(0, 3, 1), true);
               m_params["filter_pattern_offset_y"].setFlags(0);
               m_params["filter_pattern_offset_y"].setMeta(new ito::IntMeta(0, 3, 1), true);
           }
           else
           {
               retValue += ito::RetVal::format(ito::retError, 0, tr("pattern size %i not implemented yet").toLatin1().data(), channels);
           }

           if (QString(size).toInt() != 1 && m_numFrameBurst != 1)
           {
               retValue += ito::RetVal(ito::retWarning, 0, tr("Parallel usage of burst mode and hyperspectral filter pattern is not implemented yet. Number of bursts is set to one again.").toLatin1().data());
               QSharedPointer<ito::ParamBase> ptr(new ito::ParamBase("frame_burst_count", ito::ParamBase::Int, 1));
               this->setParam(ptr);
           }
           m_params["sizez"].setVal<int>(channels);
           synchronizeCameraSettings(sRoi);
        }
		else if (QString::compare(key, "aperture_value",Qt::CaseInsensitive) == 0)
		{
		float aperture = val->getVal<double>();
		retValue += checkError(pxiSetParam(m_handle, XI_PRM_LENS_APERTURE_VALUE, &aperture, sizeof(float), xiTypeFloat), "set XI_PRM_LENS_APERTURE_VALUE", QString::number(aperture));
		retValue += synchronizeCameraSettings(sLens);

		}
        else
        {
            it->copyValueFrom(&(*val));
        }
    }

    if (!retValue.containsError())
    {
        emit parametersChanged(m_params); //send changed parameters to any connected dialogs or dock-widgets
        retValue += checkData();
    }

    if (running)
    {
        retValue += this->startDevice(0);
        setGrabberStarted(running);
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
    int min = std::numeric_limits<int>::max();
    int max = 0;
    int inc = 1;
    ito::RetVal retval;
    DWORD pSize = sizeof(int);
    XI_PRM_TYPE pType = xiTypeInteger;

    QByteArray name;

    //get parameter ranges
    name = QByteArray(paramName) + XI_PRM_INFO_MIN;
    retval += checkError(pxiGetParam(m_handle, name.data(), &min, &pSize, &pType), name);
    name = QByteArray(paramName) + XI_PRM_INFO_MAX;
    retval += checkError(pxiGetParam(m_handle, name.data(), &max, &pSize, &pType), name);

#ifndef USE_API_3_16
    name = QByteArray(paramName) + XI_PRM_INFO_INCREMENT;
    retval += checkError(pxiGetParam(m_handle, name.data(), &inc, &pSize, &pType), name);
#endif

    if (!retval.containsError())
    {
        //check incoming parameter
        if (newValue < min || newValue > max)
        {
            retval += ito::RetVal::format(ito::retError, 0, tr("xiApi-Parameter '%s' is out of allowed range [%i,%i]").toLatin1().data(), paramName, min, max);
        }
#ifndef USE_API_3_16
        else if ((newValue - min) % inc != 0)
        {
            retval += ito::RetVal::format(ito::retError,0, tr("xiApi-Parameter '%s' must have an increment of %i (minimum value %i)").toLatin1().data(), paramName, inc, min);
        }
#endif

        if (!retval.containsError())
        {
            retval += checkError(pxiSetParam(m_handle, paramName, &newValue, sizeof(int), xiTypeInteger), paramName, QString::number(newValue));
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Ximea::readCameraIntParam(const char *ximeaParamName, const QString &paramName, bool mandatory /*= false*/)
{
    DWORD intSize = sizeof(int);
    XI_PRM_TYPE intType = xiTypeInteger;
    int val, minVal, maxVal, stepVal;

    ito::RetVal retval = checkError(pxiGetParam(m_handle, ximeaParamName, &val, &intSize, &intType), ximeaParamName);

    if (retval.containsError())
    {
        if (!mandatory)
        {
            retval = ito::retOk;
            m_params[paramName].setFlags(ito::ParamBase::Readonly);
        }
    }
    else
    {
        m_params[paramName].setVal<int>(val);

        QByteArray name = ximeaParamName;
        QByteArray minInfo = name + XI_PRM_INFO_MIN;
        QByteArray maxInfo = name + XI_PRM_INFO_MAX;
        QByteArray stepInfo = name + XI_PRM_INFO_INCREMENT;
        //get info
        retval += checkError(pxiGetParam(m_handle, minInfo.data(), &minVal, &intSize, &intType), minInfo.data());
        retval += checkError(pxiGetParam(m_handle, maxInfo.data(), &maxVal, &intSize, &intType), maxInfo.data());
        retval += checkError(pxiGetParam(m_handle, stepInfo.data(), &stepVal, &intSize, &intType), stepInfo.data());

        if (!retval.containsError())
        {
            m_params[paramName].setMeta(new ito::IntMeta(minVal, maxVal, stepVal), true);
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Ximea::readCameraFloatParam(const char *ximeaParamName, const QString &paramName, bool mandatory /*= false*/)
{
    DWORD floatSize = sizeof(float);
    XI_PRM_TYPE floatType = xiTypeFloat;
    float val, minVal, maxVal, stepVal;

    ito::RetVal retval = checkError(pxiGetParam(m_handle, ximeaParamName, &val, &floatSize, &floatType), ximeaParamName);

    if (retval.containsError())
    {
        if (!mandatory)
        {
            retval = ito::retOk;
            m_params[paramName].setFlags(ito::ParamBase::Readonly);
        }
    }
    else
    {
        m_params[paramName].setVal<double>(val);

        QByteArray name = ximeaParamName;
        QByteArray minInfo = name + XI_PRM_INFO_MIN;
        QByteArray maxInfo = name + XI_PRM_INFO_MAX;
        QByteArray stepInfo = name + XI_PRM_INFO_INCREMENT;
        //get info
        retval += checkError(pxiGetParam(m_handle, minInfo.data(), &minVal, &floatSize, &floatType), minInfo.data());
        retval += checkError(pxiGetParam(m_handle, maxInfo.data(), &maxVal, &floatSize, &floatType), maxInfo.data());
        retval += checkError(pxiGetParam(m_handle, stepInfo.data(), &stepVal, &floatSize, &floatType), stepInfo.data());

        if (!retval.containsError())
        {
            m_params[paramName].setMeta(new ito::DoubleMeta(minVal, maxVal, stepVal), true);
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Ximea::synchronizeCameraSettings(int what /*= sAll */)
{
    ito::RetVal retValue;
    ito::RetVal retValTemp;
    ParamMapIterator it;

    DWORD intSize = sizeof(int);
    DWORD floatSize = sizeof(float);
    XI_PRM_TYPE intType = xiTypeInteger;
    XI_PRM_TYPE strType = xiTypeString;
    XI_PRM_TYPE floatType = xiTypeFloat;

    if (what & sExposure)
    {
        it = m_params.find("integration_time");

        // Camera-exposure is set in m/s, itom uses s
        int integration_time, integration_max, integration_min, integration_step;
        retValue += checkError(pxiGetParam(m_handle, XI_PRM_EXPOSURE XI_PRM_INFO_MIN, &integration_min, &intSize, &intType), XI_PRM_EXPOSURE XI_PRM_INFO_MIN);
        retValue += checkError(pxiGetParam(m_handle, XI_PRM_EXPOSURE XI_PRM_INFO_MAX, &integration_max, &intSize, &intType), XI_PRM_EXPOSURE XI_PRM_INFO_MIN);
        retValue += checkError(pxiGetParam(m_handle, XI_PRM_EXPOSURE XI_PRM_INFO_INCREMENT, &integration_step, &intSize, &intType), XI_PRM_EXPOSURE XI_PRM_INFO_INCREMENT);
        retValue += checkError(pxiGetParam(m_handle, XI_PRM_EXPOSURE, &integration_time, &intSize, &intType), XI_PRM_EXPOSURE);

        if (!retValue.containsError())
        {
            if (integration_step == 0)
            {
                integration_step = integration_max - integration_min;
            }
            if (integration_time == 0)
            {
                integration_time = integration_min;
            }

            it->setVal<double>(musecToSec(integration_time));
            it->setMeta(new ito::DoubleMeta(musecToSec(integration_min), musecToSec(integration_max), musecToSec(integration_step)), true);
            it->setFlags(0);
        }
        else
        {
            it->setFlags(ito::ParamBase::Readonly);
        }
    }
    if (what & sBinning)
    {
        it = m_params.find("binning");

        // set binning
        int binning, binning_min, binning_max;
        retValue += checkError(pxiGetParam(m_handle, XI_PRM_DOWNSAMPLING XI_PRM_INFO_MIN, &binning_min, &intSize, &intType), XI_PRM_DOWNSAMPLING XI_PRM_INFO_MIN);
        retValue += checkError(pxiGetParam(m_handle, XI_PRM_DOWNSAMPLING XI_PRM_INFO_MAX, &binning_max, &intSize, &intType), XI_PRM_DOWNSAMPLING XI_PRM_INFO_MIN);
        retValue += checkError(pxiGetParam(m_handle, XI_PRM_DOWNSAMPLING, &binning, &intSize, &intType), XI_PRM_DOWNSAMPLING);

        if (!retValue.containsError())
        {
            it->setVal<int>(binning * 101);
            it->setMeta(new ito::IntMeta(binning_min * 101, binning_max * 101, 101), true);
            it->setFlags(0);
        }
        else
        {
            it->setFlags(ito::ParamBase::Readonly);
        }
    }
    if (what & sFrameRate)
    {
        retValue += readCameraFloatParam(XI_PRM_FRAMERATE, "framerate", false);
        if (!retValue.containsError())
        {
            float val = m_params["framerate"].getVal<double>();
            if (pxiSetParam(m_handle, XI_PRM_FRAMERATE, &val, sizeof(float), floatType) == XI_READ_ONLY_PARAM || (m_params["timing_mode"].getVal<int>() == XI_ACQ_TIMING_MODE_FREE_RUN))
            {
                m_params["framerate"].setFlags(ito::ParamBase::Readonly);
            }
            else
            {
                m_params["framerate"].setFlags(0);
            }
        }
    }
    if (what & sRoi)
    {
        int offset_x, offsetMin_x, offsetMax_x, offsetInc_x;
        int size_x, sizeMin_x, sizeMax_x, sizeInc_x;
        int offset_y, offsetMin_y, offsetMax_y, offsetInc_y;
        int size_y, sizeMin_y, sizeMax_y, sizeInc_y;

        retValue += checkError(pxiGetParam(m_handle, XI_PRM_WIDTH, &size_x, &intSize, &intType), "get XI_PRM_WIDTH");
        retValue += checkError(pxiGetParam(m_handle, XI_PRM_WIDTH XI_PRM_INFO_MIN, &sizeMin_x, &intSize, &intType), "get XI_PRM_WIDTH" XI_PRM_INFO_MIN);
        retValue += checkError(pxiGetParam(m_handle, XI_PRM_WIDTH XI_PRM_INFO_MAX, &sizeMax_x, &intSize, &intType), "get XI_PRM_WIDTH" XI_PRM_INFO_MAX);
        retValue += checkError(pxiGetParam(m_handle, XI_PRM_WIDTH XI_PRM_INFO_INCREMENT, &sizeInc_x, &intSize, &intType), "get XI_PRM_WIDTH" XI_PRM_INFO_INCREMENT);
        if (sizeInc_x == 0)
        {
            sizeInc_x = 1;
        }

        retValue += checkError(pxiGetParam(m_handle, XI_PRM_OFFSET_X, &offset_x, &intSize, &intType), "get XI_PRM_OFFSET_X");
        retValue += checkError(pxiGetParam(m_handle, XI_PRM_OFFSET_X XI_PRM_INFO_MIN, &offsetMin_x, &intSize, &intType), "get XI_PRM_OFFSET_X" XI_PRM_INFO_MIN);
        retValue += checkError(pxiGetParam(m_handle, XI_PRM_OFFSET_X XI_PRM_INFO_MAX, &offsetMax_x, &intSize, &intType), "get XI_PRM_OFFSET_X" XI_PRM_INFO_MAX);
        retValue += checkError(pxiGetParam(m_handle, XI_PRM_OFFSET_X XI_PRM_INFO_INCREMENT, &offsetInc_x, &intSize, &intType), "get XI_PRM_OFFSET_X" XI_PRM_INFO_INCREMENT);
        if (offsetInc_x == 0)
        {
            offsetInc_x = 1;
        }

        retValue += checkError(pxiGetParam(m_handle, XI_PRM_HEIGHT, &size_y, &intSize, &intType), "get XI_PRM_HEIGHT");
        retValue += checkError(pxiGetParam(m_handle, XI_PRM_HEIGHT XI_PRM_INFO_MIN, &sizeMin_y, &intSize, &intType), "get XI_PRM_HEIGHT" XI_PRM_INFO_MIN);
        retValue += checkError(pxiGetParam(m_handle, XI_PRM_HEIGHT XI_PRM_INFO_MAX, &sizeMax_y, &intSize, &intType), "get XI_PRM_HEIGHT" XI_PRM_INFO_MAX);
        retValue += checkError(pxiGetParam(m_handle, XI_PRM_HEIGHT XI_PRM_INFO_INCREMENT, &sizeInc_y, &intSize, &intType), "get XI_PRM_HEIGHT" XI_PRM_INFO_INCREMENT);
        if (sizeInc_y == 0)
        {
            sizeInc_y = 1;
        }

        retValue += checkError(pxiGetParam(m_handle, XI_PRM_OFFSET_Y, &offset_y, &intSize, &intType), "get XI_PRM_OFFSET_Y");
        retValue += checkError(pxiGetParam(m_handle, XI_PRM_OFFSET_Y XI_PRM_INFO_MIN, &offsetMin_y, &intSize, &intType), "get XI_PRM_OFFSET_Y" XI_PRM_INFO_MIN);
        retValue += checkError(pxiGetParam(m_handle, XI_PRM_OFFSET_Y XI_PRM_INFO_MAX, &offsetMax_y, &intSize, &intType), "get XI_PRM_OFFSET_Y" XI_PRM_INFO_MAX);
        retValue += checkError(pxiGetParam(m_handle, XI_PRM_OFFSET_Y XI_PRM_INFO_INCREMENT, &offsetInc_y, &intSize, &intType), "get XI_PRM_OFFSET_Y" XI_PRM_INFO_INCREMENT);
        if (offsetInc_y == 0)
        {
            offsetInc_y = 1;
        }
        if (m_channelNumberChanged)
        {
            if (offset_x != offsetMin_x || offset_y != offsetMin_y || size_y != sizeMax_y || size_x != sizeMax_x)
            {
                retValue += ito::RetVal(ito::retWarning, 0, tr("Camera switched back to full frame size since the number of channels was changed").toLatin1().data()); // check muss auch weiter oben rein, damit auch ein wechsel auf channels ==0 nicht geht
                int newROI[4] = { 0, 0, sizeMax_x, sizeMax_y };
                m_channelNumberChanged = false;
                QSharedPointer<ito::ParamBase> ptr(new ito::ParamBase("roi", ito::ParamBase::IntArray, 4, newROI));
                this->setParam(ptr);
            }
            m_channelNumberChanged = false;
        }
        if (QString::compare(m_params["filter_pattern_size"].getVal<char*>(), "1")==0)
        {

            m_params["x0"].setVal<int>(offset_x);
            m_params["x0"].setMeta(new ito::IntMeta(offsetMin_x, sizeMax_x - sizeMin_x, offsetInc_x), true);

            m_params["x1"].setVal<int>(offset_x + size_x - 1);
            m_params["x1"].setMeta(new ito::IntMeta(offset_x + sizeMin_x - 1, sizeMax_x - 1, sizeInc_x), true);

            m_params["y0"].setVal<int>(offset_y);
            m_params["y0"].setMeta(new ito::IntMeta(offsetMin_y, sizeMax_y - sizeMin_y, offsetInc_y), true);

            m_params["y1"].setVal<int>(offset_y + size_y - 1);
            m_params["y1"].setMeta(new ito::IntMeta(offset_y + sizeMin_y - 1, sizeMax_y - 1, sizeInc_y), true);

            m_params["sizex"].setVal<int>(size_x);
            m_params["sizex"].setMeta(new ito::IntMeta(sizeMin_x, sizeMax_x, sizeInc_x), true);

            m_params["sizey"].setVal<int>(size_y);
            m_params["sizey"].setMeta(new ito::IntMeta(sizeMin_y, sizeMax_y, sizeInc_y), true);

            it = m_params.find("roi");
            int *roi = it->getVal<int*>();
            roi[0] = offset_x;
            roi[1] = offset_y;
            roi[2] = size_x;
            roi[3] = size_y;
            if (sizeMin_x % sizeInc_x != 0)
            {
                sizeMin_x = sizeMin_x - (sizeMin_x % sizeInc_x) + sizeInc_x;
            }
            if (sizeMin_y % sizeInc_y != 0)
            {
                sizeMin_y = sizeMin_y - (sizeMin_x % sizeInc_x) + sizeInc_x;
            }

            ito::RangeMeta widthMeta(offsetMin_x, sizeMax_x + offset_x - 1, offsetInc_x, sizeMin_x, sizeMax_x + offset_x, sizeInc_x);
            ito::RangeMeta heightMeta(offsetMin_y, sizeMax_y + offset_y - 1, offsetInc_y, sizeMin_y, sizeMax_y + offset_y, sizeInc_y);
            it->setMeta(new ito::RectMeta(widthMeta, heightMeta), true);
        }
        else
        {

            short filter_pattern_size = QString::compare(m_params["filter_pattern_size"].getVal<char*>(), "16") == 0 ? 16 : 25;
            short patternEdgeLength = filter_pattern_size == 25 ? 5 : 4;
            m_params["sizex"].setVal<int>(size_x / patternEdgeLength);
            m_originalSizeX = size_x;
            m_params["sizex"].setMeta(new ito::IntMeta(sizeMin_x / patternEdgeLength, (sizeMax_x - m_params["filter_pattern_offset_x"].getVal<int>()) / patternEdgeLength, sizeInc_x / patternEdgeLength + 1));

            m_params["sizey"].setVal<int>(size_y / patternEdgeLength);
            m_originalSizeY = size_y;
            m_params["sizey"].setMeta(new ito::IntMeta(sizeMin_y / patternEdgeLength, (sizeMax_y - m_params["filter_pattern_offset_y"].getVal<int>())/ patternEdgeLength, sizeInc_y / patternEdgeLength + 1));


            int x0 = std::ceil((offset_x - m_params["filter_pattern_offset_x"].getVal<int>()) / patternEdgeLength); //todo is ceil a good option? waht happens if ceil(0)...
            m_params["x0"].setVal<int>(x0);
            int hyperSizeMin_x = std::ceil(float(offsetMin_x - m_params["filter_pattern_offset_x"].getVal<int>()) / patternEdgeLength);
            int hyperSizeMax_x = std::floor(float(sizeMax_x - sizeMin_x) / patternEdgeLength);
            m_params["x0"].setMeta(new ito::IntMeta(hyperSizeMin_x, hyperSizeMax_x, offsetInc_x/patternEdgeLength+1));

            int y0 = std::ceil((offset_y - m_params["filter_pattern_offset_y"].getVal<int>()) / patternEdgeLength); //todo is ceil a good option? waht happens if ceil(0)...
            m_params["y0"].setVal<int>(y0);
            int hyperSizeMin_y = std::ceil(float(offsetMin_y - m_params["filter_pattern_offset_y"].getVal<int>()) / patternEdgeLength);
            int hyperSizeMax_y = std::floor(float(sizeMax_y - sizeMin_y) / patternEdgeLength);
            m_params["y0"].setMeta(new ito::IntMeta(hyperSizeMin_y, hyperSizeMax_y, offsetInc_y / patternEdgeLength + 1));

            m_params["x1"].setVal<int>(x0 + size_x / patternEdgeLength-1);
            float test = (float)sizeMin_x / patternEdgeLength;
            int test2 = std::ceil((float)sizeMin_x / patternEdgeLength);
            m_params["x1"].setMeta(new ito::IntMeta(x0 + std::ceil((float)sizeMin_x / patternEdgeLength), ((sizeMax_x - m_params["filter_pattern_offset_x"].getVal<int>()) / patternEdgeLength)-1, sizeInc_x / patternEdgeLength + 1));

            m_params["y1"].setVal<int>(y0 + size_y / patternEdgeLength-1);
            m_params["y1"].setMeta(new ito::IntMeta(y0 + std::ceil((float)sizeMin_y / patternEdgeLength), ((sizeMax_y - m_params["filter_pattern_offset_y"].getVal<int>()) / patternEdgeLength)-1, sizeInc_y / patternEdgeLength + 1));

        }



    }
    if (what & sGain)
    {
        it = m_params.find("gain");
        //sets gain value interval
        float gain, gain_min, gain_max, gain_inc;
        retValue += checkError(pxiGetParam(m_handle, XI_PRM_GAIN XI_PRM_INFO_MIN, &gain_min, &floatSize, &floatType), XI_PRM_GAIN XI_PRM_INFO_MIN);
        retValue += checkError(pxiGetParam(m_handle, XI_PRM_GAIN XI_PRM_INFO_MAX, &gain_max, &floatSize, &floatType), XI_PRM_GAIN XI_PRM_INFO_MIN);
        retValue += checkError(pxiGetParam(m_handle, XI_PRM_GAIN XI_PRM_INFO_INCREMENT, &gain_inc, &floatSize, &floatType), XI_PRM_GAIN XI_PRM_INFO_INCREMENT);
        retValue += checkError(pxiGetParam(m_handle, XI_PRM_GAIN, &gain, &floatSize, &floatType), XI_PRM_GAIN);

        if (!retValue.containsError() && (gain_max - gain_min) > 0)
        {
            it->setVal<double>((gain - gain_min) / (gain_max - gain_min));
            it->setMeta(new ito::DoubleMeta(0.0, 1.0, gain_inc / (gain_max - gain_min)), true);
            it->setFlags(0);
        }
        else
        {
            it->setFlags(ito::ParamBase::Readonly);
        }
    }
    if (what & sOffset)
    {
        //retValue += readCameraFloatParam(XI_PRM_IMAGE_BLACK_LEVEL, "offset", false); //optional, not always available
    }
    if (what & sGamma)
    {
        it = m_params.find("gamma");
        //sets gamma value interval
        float gamma, gamma_min, gamma_max, gamma_inc;
        retValue += checkError(pxiGetParam(m_handle, XI_PRM_GAMMAY XI_PRM_INFO_MIN, &gamma_min, &floatSize, &floatType), XI_PRM_GAMMAY XI_PRM_INFO_MIN);
        retValue += checkError(pxiGetParam(m_handle, XI_PRM_GAMMAY XI_PRM_INFO_MAX, &gamma_max, &floatSize, &floatType), XI_PRM_GAMMAY XI_PRM_INFO_MIN);
        retValue += checkError(pxiGetParam(m_handle, XI_PRM_GAMMAY XI_PRM_INFO_INCREMENT, &gamma_inc, &floatSize, &floatType), XI_PRM_GAMMAY XI_PRM_INFO_INCREMENT);
        retValue += checkError(pxiGetParam(m_handle, XI_PRM_GAMMAY, &gamma, &floatSize, &floatType), XI_PRM_GAMMAY);

        if (!retValue.containsError() && (gamma_max - gamma_min) > 0)
        {
            it->setVal<double>((gamma - gamma_min) / (gamma_max - gamma_min));
            it->setMeta(new ito::DoubleMeta(0.0, 1.0, gamma_inc / (gamma_max - gamma_min)), true);
            it->setFlags(0);
        }
        else
        {
            it->setFlags(ito::ParamBase::Readonly);
        }

        it = m_params.find("gammaColor");
        if (!(it->getFlags() & ito::ParamBase::Readonly))
        {
            //sets gamma value interval
            float gamma, gamma_min, gamma_max, gamma_inc;
            retValue += checkError(pxiGetParam(m_handle, XI_PRM_GAMMAC XI_PRM_INFO_MIN, &gamma_min, &floatSize, &floatType), XI_PRM_GAMMAC XI_PRM_INFO_MIN);
            retValue += checkError(pxiGetParam(m_handle, XI_PRM_GAMMAC XI_PRM_INFO_MAX, &gamma_max, &floatSize, &floatType), XI_PRM_GAMMAC XI_PRM_INFO_MIN);
            retValue += checkError(pxiGetParam(m_handle, XI_PRM_GAMMAC XI_PRM_INFO_INCREMENT, &gamma_inc, &floatSize, &floatType), XI_PRM_GAMMAC XI_PRM_INFO_INCREMENT);
            retValue += checkError(pxiGetParam(m_handle, XI_PRM_GAMMAC, &gamma, &floatSize, &floatType), XI_PRM_GAMMAC);

            if (!retValue.containsError() && (gamma_max - gamma_min) > 0)
            {
                it->setVal<double>((gamma - gamma_min) / (gamma_max - gamma_min));
                it->setMeta(new ito::DoubleMeta(0.0, 1.0, gamma_inc / (gamma_max - gamma_min)), true);
                it->setFlags(0);
            }
            else
            {
                it->setFlags(ito::ParamBase::Readonly);
            }
        }
    }
    if (what & sSharpness)
    {
        it = m_params.find("sharpness");
        //sets gain value interval
        float sharpness, sharpness_min, sharpness_max, sharpness_inc;
        retValue += checkError(pxiGetParam(m_handle, XI_PRM_SHARPNESS XI_PRM_INFO_MIN, &sharpness_min, &floatSize, &floatType), XI_PRM_SHARPNESS XI_PRM_INFO_MIN);
        retValue += checkError(pxiGetParam(m_handle, XI_PRM_SHARPNESS XI_PRM_INFO_MAX, &sharpness_max, &floatSize, &floatType), XI_PRM_SHARPNESS XI_PRM_INFO_MIN);
        retValue += checkError(pxiGetParam(m_handle, XI_PRM_SHARPNESS XI_PRM_INFO_INCREMENT, &sharpness_inc, &floatSize, &floatType), XI_PRM_SHARPNESS XI_PRM_INFO_INCREMENT);
        retValue += checkError(pxiGetParam(m_handle, XI_PRM_SHARPNESS, &sharpness, &floatSize, &floatType), XI_PRM_SHARPNESS);

        if (!retValue.containsError() && (sharpness_max - sharpness_min) > 0)
        {
            it->setVal<double>((sharpness - sharpness_min) / (sharpness_max - sharpness_min));
            it->setMeta(new ito::DoubleMeta(0.0, 1.0, sharpness_inc / (sharpness_max - sharpness_min)), true);
            it->setFlags(0);
        }
        else
        {
            it->setFlags(ito::ParamBase::Readonly);
        }
    }
    if (what & sTriggerMode)
    {
        retValue += readCameraIntParam(XI_PRM_TRG_SOURCE, "trigger_mode", true);
    }
    if (what & sTriggerSelector)
    {
        retValue += readCameraIntParam(XI_PRM_TRG_SELECTOR, "trigger_selector", true);
    }
    if (what & sBpp)
    {
        it = m_params.find("bpp");
        int output_bit_depth = 0;
        int data_format = 0;
        retValTemp += checkError(pxiGetParam(m_handle, XI_PRM_IMAGE_DATA_FORMAT, &data_format, &intSize, &intType), "get XI_PRM_IMAGE_DATA_FORMAT");
        retValTemp = checkError(pxiGetParam(m_handle, XI_PRM_OUTPUT_DATA_BIT_DEPTH, &output_bit_depth, &intSize, &intType), "get XI_PRM_OUTPUT_DATA_BIT_DEPTH");
        if (!retValTemp.containsError())
        {
            if (data_format == XI_RGB32)
            {
                it->setVal<int>(32);
            }
            else if (data_format == XI_MONO8 || data_format == XI_RAW8)
            {
                it->setVal<int>(8);
            }
            else
            {
                it->setVal<int>(output_bit_depth);
            }
        }
        retValue += retValTemp;
    }

    if (what & sGpiGpo)
    {
        int *gpi_mode = m_params["gpi_mode"].getVal<int*>();
        int *gpi_level = m_params["gpi_level"].getVal<int*>();
        int *gpo_mode = m_params["gpo_mode"].getVal<int*>();

        for (int i = 1; i <= m_numGPIPins; ++i)
        {
            retValue += checkError(pxiSetParam(m_handle, XI_PRM_GPI_SELECTOR, &i, intSize, intType), XI_PRM_GPI_SELECTOR);
            retValue += checkError(pxiGetParam(m_handle, XI_PRM_GPI_MODE, &(gpi_mode[i-1]),&intSize, &intType), "get:" XI_PRM_GPI_MODE);
            retValue += checkError(pxiGetParam(m_handle, XI_PRM_GPI_LEVEL, &(gpi_level[i-1]),&intSize, &intType), "get:" XI_PRM_GPI_LEVEL);
        }

        for (int i = 1; i <= m_numGPOPins; ++i)
        {
            retValue += checkError(pxiSetParam(m_handle, XI_PRM_GPO_SELECTOR, &i, intSize, intType), XI_PRM_GPO_SELECTOR);
            retValue += checkError(pxiGetParam(m_handle, XI_PRM_GPO_MODE, &(gpo_mode[i-1]),&intSize, &intType), "get:" XI_PRM_GPO_MODE);
        }
    }
	if (what & sLens)
	{
		if (!(m_params["aperture_value"].getFlags() & ito::ParamBase::Readonly))
		{
			float maxAperture, minAperture, aperture;
			retValue += checkError(pxiGetParam(m_handle, XI_PRM_LENS_APERTURE_VALUE XI_PRM_INFO_MAX, &maxAperture, &floatSize, &floatType), "get XI_PRM_LENS_APERTURE_VALUE MAX");
			retValue += checkError(pxiGetParam(m_handle, XI_PRM_LENS_APERTURE_VALUE XI_PRM_INFO_MIN, &minAperture, &floatSize, &floatType), "get XI_PRM_LENS_APERTURE_VALUE MIN");
			retValue += checkError(pxiGetParam(m_handle, XI_PRM_LENS_APERTURE_VALUE, &aperture, &floatSize, &floatType), "get XI_PRM_LENS_APERTURE_VALUE");
			minAperture = std::round(minAperture * 10.) / 10.;
			maxAperture = std::round(maxAperture * 10.) / 10.;
			m_params["aperture_value"].setMeta(new ito::DoubleMeta(minAperture, maxAperture, 0.1), true);
			m_params["aperture_value"].setVal<double>(aperture);
		}
	}

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Ximea::startDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if (grabberStartedCount() < 1)
    {
        setGrabberStarted(0);
        retValue += checkError(pxiStartAcquisition(m_handle), "pxiStartAcquisition");
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

    if (grabberStartedCount() < 1)
    {
        retValue += checkError(pxiStopAcquisition(m_handle), "pxiStopAquisition");
    }

    if (grabberStartedCount() < 0)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("stopDevice ignored since camera was not started.").toLatin1().data());
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
        retValue = ito::RetVal(ito::retError, 0, tr("Tried to acquire without starting device").toLatin1().data());
        m_isgrabbing = false;

        if (waitCond)
        {
            waitCond->returnValue = retValue;
            waitCond->release();
        }
    }
    else
    {
        m_isgrabbing = true;
        if (triggermode == XI_TRG_SOFTWARE)
        {
            int val = 1;
            XI_RETURN ret = pxiSetParam(m_handle, XI_PRM_TRG_SOFTWARE, &val, sizeof(int), xiTypeInteger);
            if (ret == XI_DEVICE_NOT_READY) //in frame burst mode, the camera sometimes needs some delay before acquiring the next series (especially for very small integration times)
            {
                for (int c = 1; c <= 10; ++c)
                {
                    Sleep(c * 25);
                    ret = pxiSetParam(m_handle, XI_PRM_TRG_SOFTWARE, &val, sizeof(int), xiTypeInteger);

                    if (ret != XI_DEVICE_NOT_READY)
                    {
                        break;
                    }
                }

                retValue += checkError(ret, XI_PRM_TRG_SOFTWARE, "(no success after 10 tries)");
            }
            else
            {
                retValue += checkError(ret, XI_PRM_TRG_SOFTWARE);
            }

            if (retValue != ito::retOk)
            {
                m_isgrabbing = false;
            }
        }

        if (waitCond)
        {
            waitCond->returnValue = retValue;
            waitCond->release();
        }

        XI_IMG img;

        if (!retValue.containsError())
        {
            int iPicTimeOut = (int)(m_params["timeout"].getVal<double>() * 1000 + 1.0); //timeout in ms
            int curxsize, curysize;
            if (QString(m_params["filter_pattern_size"].getVal<char*>()).toInt() == 1)
            {
                curxsize = m_params["sizex"].getVal<int>();
                curysize = m_params["sizey"].getVal<int>();
            }
            else
            {
                curysize = m_originalSizeY;
                curxsize = m_originalSizeX;
            }
            std::string number;

            for (int i = 0; i < m_numFrameBurst; ++i)
            {
                img.size = sizeof(XI_IMG);
                img.bp = m_data.rowPtr(i, 0);
                switch (m_data.getType())
                {
                case ito::tUInt8:
                    img.bp_size = curxsize * curysize;
                    break;
                case ito::tUInt16:
                    img.bp_size = curxsize * curysize * 2;
                    break;
                case ito::tRGBA32:
                    img.bp_size = curxsize * curysize * 4;
                    break;
                }

				m_acqRetVal += checkError(pxiGetImage(m_handle, iPicTimeOut, &img), "pxiGetImage");

                if (!m_acqRetVal.containsError())
                {
//#ifndef XI_PRM_IMAGE_DATA_FORMAT_RGB32_ALPHA
					//set alpha values to 255.
                    if (m_data.getType() == ito::tRGBA32)
                    {
                        ito::Rgba32 *colordata = (ito::Rgba32*)(img.bp);
                        for (int i = 0; i < curxsize * curysize; ++i)
                        {
                            colordata[i].alpha() = 255;
                        }
                    }
//#endif

                    if (m_numFrameBurst == 1)
                    {
                        if (m_family != familyMU)
                        {
                            m_data.setTag("timestamp", (double)img.tsSec + ((double)img.tsUSec * 1.0e-6));
                        }
                        m_data.setTag("frame_counter", img.nframe);
                    }
                    else
                    {
#if __cplusplus > 199711L //C++11 support
                        number = std::to_string((long long)i);
#else
                        std::stringstream out;
                        out << i;
                        number = out.str();
#endif
                        if (m_family != familyMU)
                        {
                            m_data.setTag("timestamp" + number, (double)img.tsSec + ((double)img.tsUSec * 1.0e-6));
                        }
                        m_data.setTag("frame_counter" + number, img.nframe);
                    }

                    if (i == 0)
                    {
#ifdef SIZE_XI_IMG_V5 //AbsoluteOffsetX and AbsoluteOffsetY is only defined if SIZE_XI_IMG_V5 is defined
                        m_data.setTag("roi_x0", img.AbsoluteOffsetX);
                        m_data.setTag("roi_y0", img.AbsoluteOffsetY);
#endif
                    }

                    if (m_shading.active)
                    {
                        ito::uint16* ptrSub = m_shading.sub;
                        ito::uint16* ptrMul = m_shading.mul;
                        ito::uint16* ptrDst = (ito::uint16*)m_data.rowPtr(i, m_shading.y0);
                        ptrDst += m_shading.x0;
                        ito::int32 stepY = img.width - m_shading.xsize;
                        for (int y = 0; y < m_shading.ysize; y++)
                        {
                            for (int x = 0; x < m_shading.xsize; x++)
                            {
                                if (*ptrSub > *ptrDst)
                                {
                                    *ptrDst = 0;
                                }
                                else
                                {
                                    *ptrDst -= *ptrSub;
                                    //*ptrDst *= *ptrMul;
                                }
                                ptrDst++;
                                ptrMul++;
                                ptrSub++;
                            }
                            ptrDst += stepY;
                        }
                    }
                }
                else
                {
                    break;
                }
            }
        }
    }

    return retValue;
}
//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> void makeHyperCube(ito::DataObject* srcObj, ito::DataObject *dstObj, const int &offsetX, const int &offsetY, const int &filter_pattern_size)
{
    unsigned int patternEdgeLength = filter_pattern_size == 25 ? 5 : 4;
    const int dstSizeX = dstObj->getSize(dstObj->getDims() - 1);
    const int dstSizeY = dstObj->getSize(dstObj->getDims() - 2);
    const int numCannels = dstObj->getSize(dstObj->getDims() - 3);
    cv::Mat* srcPlane = srcObj->get_mdata()[srcObj->seekMat(0)];
    int srcRowStep = srcPlane->step1();
    int macroStepX = 0;
    int macroStepY = 0;

    int row, col;
    _Tp* dstPtr;
    const _Tp* srcPtr;
    _Tp* dstRowPtr;
    for (int channel = 0; channel < numCannels; ++channel)
    {
        for (row = 0; row < dstSizeY; ++row)
        {
            dstRowPtr = dstObj->rowPtr<_Tp>(dstObj->seekMat(channel), row);
            srcPtr = ((const _Tp*)(srcPlane->data))+ (srcRowStep*(offsetY + row*patternEdgeLength + macroStepY) + offsetX + macroStepX);

            for (col = 0; col < dstSizeX; ++col)
            {
                dstRowPtr[col] = srcPtr[col*patternEdgeLength];
            }
        }
        if (macroStepX == 4)
        {
            ++macroStepY;
            macroStepX = 0;
        }
        else
        {
            ++macroStepX;
        }

    }

}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Ximea::retrieveData(ito::DataObject *externalDataObject)
{
    ito::RetVal retValue(ito::retOk);

    bool copyExternal = externalDataObject != NULL;

    if (grabberStartedCount() <= 0)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("Tried to get picture without starting device").toLatin1().data());
    }
    else if (!m_isgrabbing)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Tried to get picture without triggering exposure").toLatin1().data());
    }
    else
    {
        retValue += m_acqRetVal;
        m_acqRetVal = ito::retOk;
    }


        if (!retValue.containsError())
        {
            int patternSize = QString(m_params["filter_pattern_size"].getVal<char*>()).toInt();

            if (copyExternal)
            {
                if (patternSize == 1)
                {
                    //here we wait until the Event is set to signaled state
                    //or the timeout runs out
                    int planes = m_data.getNumPlanes();
                    bool valid;
                    std::string number;

                    for (int i = 0; i < planes; ++i)
                    {
                        const cv::Mat* internalMat = m_data.getCvPlaneMat(i);
                        cv::Mat* externalMat = externalDataObject->getCvPlaneMat(i);


                        if (externalMat->isContinuous())
                        {
                            memcpy(externalMat->ptr(0), internalMat->ptr(0), internalMat->cols * internalMat->rows * externalMat->elemSize());
                        }
                        else
                        {
                            for (int y = 0; y < internalMat->rows; y++)
                            {
                                memcpy(externalMat->ptr(y), internalMat->ptr(y), internalMat->cols * externalMat->elemSize());
                            }
                        }

                        if (planes <= 1)
                        {
                            if (m_family != familyMU)
                            {
                                externalDataObject->setTag("timestamp", m_data.getTag("timestamp", valid));
                            }
                            externalDataObject->setTag("frame_counter", m_data.getTag("frame_counter", valid));
                        }
                        else
                        {
#if __cplusplus > 199711L //C++11 support
                            number = std::to_string((long long)i);
#else
                            std::stringstream out;
                            out << i;
                            number = out.str();
#endif
                            if (m_family != familyMU)
                            {
                                externalDataObject->setTag("timestamp" + number, m_data.getTag("timestamp" + number, valid));
                            }
                            externalDataObject->setTag("frame_counter" + number, m_data.getTag("frame_counter" + number, valid));
                        }
                    }

                    externalDataObject->setTag("roi_x0", m_data.getTag("roi_x0", valid));
                    externalDataObject->setTag("roi_y0", m_data.getTag("roi_y0", valid));
                }
                else
                {
                    switch (m_data.getType())
                    {
                    case ito::tUInt8:
                        makeHyperCube<ito::uint8>(&m_data, externalDataObject, m_params["filter_pattern_offset_x"].getVal<int>(), m_params["filter_pattern_offset_y"].getVal<int>(), patternSize);
                        break;
                    case ito::tUInt16:
                        makeHyperCube<ito::uint16>(&m_data, externalDataObject, m_params["filter_pattern_offset_x"].getVal<int>(), m_params["filter_pattern_offset_y"].getVal<int>(), patternSize);
                        break;
                    case ito::tInt32:
                        makeHyperCube<ito::int32>(&m_data, externalDataObject, m_params["filter_pattern_offset_x"].getVal<int>(), m_params["filter_pattern_offset_y"].getVal<int>(), patternSize);
                        break;
                    case ito::tFloat64:
                        makeHyperCube<ito::float64>(&m_data, externalDataObject, m_params["filter_pattern_offset_x"].getVal<int>(), m_params["filter_pattern_offset_y"].getVal<int>(), patternSize);
                        break;
                    default:
                        retValue += ito::RetVal(ito::retError, 0, tr("data type is not supported for hyperspectral images.").toLatin1().data());

                    }
                }
            }
            else //do not copy external, but maybe we have to create a data cube for hyperspectral data
            {
                if (patternSize != 1)
                {
                    switch (m_data.getType())
                    {
                    case ito::tUInt8:
                        makeHyperCube<ito::uint8>( &m_data, &m_hyperspectralCubeObj, m_params["filter_pattern_offset_x"].getVal<int>(), m_params["filter_pattern_offset_y"].getVal<int>(), patternSize);
                        break;
                    case ito::tUInt16:
                        makeHyperCube<ito::uint16>(&m_data, &m_hyperspectralCubeObj, m_params["filter_pattern_offset_x"].getVal<int>(), m_params["filter_pattern_offset_y"].getVal<int>(), patternSize);
                        break;
                    case ito::tInt32:
                        makeHyperCube<ito::int32>(&m_data, &m_hyperspectralCubeObj, m_params["filter_pattern_offset_x"].getVal<int>(), m_params["filter_pattern_offset_y"].getVal<int>(), patternSize);
                        break;
                    case ito::tFloat64:
                        makeHyperCube<ito::float64>(&m_data, &m_hyperspectralCubeObj, m_params["filter_pattern_offset_x"].getVal<int>(), m_params["filter_pattern_offset_y"].getVal<int>(), patternSize);
                        break;
                    default:
                        retValue += ito::RetVal(ito::retError, 0, tr("data type is not supported for hyperspectral images.").toLatin1().data());

                    }
                }

            }
        }

    m_isgrabbing = false;

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Ximea::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);

    retValue += retrieveData();

    if (!retValue.containsError())
    {
        sendDataToListeners(0); //don't wait for live data, since user should get the data as fast as possible.

        if (dObj)
        {
            if (QString(m_params["filter_pattern_size"].getVal<char *>()).toInt() == 1)
            {
                (*dObj) = this->m_data;
            }
            else
            {
                (*dObj) = this->m_hyperspectralCubeObj;
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
    int offset_new = 0;
    double value = 0.0;

    char name[40]={0};

    bitppix_old = m_params["bpp"].getVal<int>();
    binning_old = m_params["binning"].getVal<int>();

    foreach(const ito::Param &param1, params)
    {
        memset(name,0,sizeof(name));
        sprintf(name,"%s", param1.getName());
        if (!strlen(name))
            continue;
        QMap<QString, ito::Param>::iterator paramIt = m_params.find(name);
        if (paramIt != m_params.end() && paramIt->isNumeric())
        {
            value = param1.getVal<double>();
            if ((value <= paramIt->getMax()) || (value >= paramIt->getMin()))
            {
                paramIt.value().setVal<double>(value);
            }
        }
    }

    bitppix_new = m_params["bpp"].getVal<int>();
    binning_new = m_params["binning"].getVal<int>();
    offset_new = m_params["offset"].getVal<int>();

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
        setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("offset", ito::ParamBase::Int, offset_new)), NULL);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Ximea::execFunc(const QString funcName, QSharedPointer<QVector<ito::ParamBase> > paramsMand, QSharedPointer<QVector<ito::ParamBase> > paramsOpt, QSharedPointer<QVector<ito::ParamBase> > /*paramsOut*/, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retValue = ito::retOk;
    ito::ParamBase *param1 = NULL;
    ito::ParamBase *param2 = NULL;
    ito::ParamBase *param3 = NULL;
    ito::ParamBase *param4 = NULL;
    /*
    if (m_pvShadingSettings == NULL)
    {
        m_pvShadingSettings = (void*) new MMSHADING();
        ((LPMMSHADING)m_pvShadingSettings)->shCX = 0;
        ((LPMMSHADING)m_pvShadingSettings)->shCX = 0;
        ((LPMMSHADING)m_pvShadingSettings)->lpMul = NULL;
        ((LPMMSHADING)m_pvShadingSettings)->lpSub = NULL;
    }

    LPMMSHADING shading = (LPMMSHADING)m_pvShadingSettings;
    */

    int illm = 0;

    if (funcName == "update_shading")
    {
        param1 = ito::getParamByName(&(*paramsMand), "illumination", &retValue);

        if (!retValue.containsError())
        {
            illm = param1->getVal<int>();
            updateShadingCorrection(illm);
        }
    }
    else if (funcName == "shading_correction_values")
    {
        param1 = ito::getParamByName(&(*paramsMand), "integration_time", &retValue);
        param2 = ito::getParamByName(&(*paramsMand), "shading_correction_factor", &retValue);
        int intTime = 0;
        if (!retValue.containsError())
        {
            intTime = (int)(param1->getVal<double>() * 1000);
            if (param2->getLen() < 20)
            {
                retValue += ito::RetVal(ito::retError, 1, tr("Fill shading correction factor failed").toLatin1().data());
            }


        }

        if (!retValue.containsError())
        {
            QVector<QPointF> newVals(10);
            double* dptr = param2->getVal<double*>();
            newVals[0] = QPointF(0.0, 1.0);
            for (int i = 1; i < 10; i ++)
            {
                newVals[i].setX(dptr[i*2]);
                newVals[i].setY(dptr[i*2+1]);
            }
            m_shading.m_correction.insert(intTime, newVals);
        }
    }
    else if (funcName == "initialize_shading")
    {
        param1 = ito::getParamByName(&(*paramsMand), "dark_image", &retValue);
        param2 = ito::getParamByName(&(*paramsMand), "white_image", &retValue);

        param3 = ito::getParamByName(&(*paramsMand), "x0", &retValue);
        param4 = ito::getParamByName(&(*paramsMand), "y0", &retValue);

        int xsize = m_params["sizex"].getVal<int>();
        int ysize = m_params["sizey"].getVal<int>();

        if (!retValue.containsError())
        {
            int x0 = param3->getVal<int>();
            int y0 = param4->getVal<int>();

            ito::DataObject* darkObj = (ito::DataObject*)(param1->getVal<void*>());
            ito::DataObject* whiteObj = (ito::DataObject*)(param2->getVal<void*>());
            if (darkObj == NULL || whiteObj == NULL)
            {
                m_shading.valid = false;
                m_shading.active = false;
            }
            else if (darkObj->getType() != ito::tUInt16 || darkObj->getDims() != 2 || (darkObj->getSize(0) + y0) > ysize || (darkObj->getSize(1) + x0) > xsize)
            {
                m_shading.valid = false;
                m_shading.active = false;
            }
            else if (whiteObj->getType() != ito::tUInt16 || whiteObj->getDims() != 2 || (whiteObj->getSize(0) + y0)> ysize || (whiteObj->getSize(1) + x0) > xsize)
            {
                m_shading.valid = false;
                m_shading.active = false;
            }
            else if (whiteObj->getSize(0) != darkObj->getSize(0) || whiteObj->getSize(1) != darkObj->getSize(1))
            {
                m_shading.valid = false;
                m_shading.active = false;
            }
            else
            {
                m_shading.valid = true;
                m_shading.active = true;
                if (m_shading.mul != NULL) delete m_shading.mul;
                if (m_shading.sub != NULL) delete m_shading.sub;
                if (m_shading.subBase != NULL) delete m_shading.subBase;
                if (m_shading.mulBase != NULL) delete m_shading.mulBase;

                m_shading.mul = new ito::uint16[whiteObj->getSize(0)*whiteObj->getSize(1)];
                m_shading.sub = new ito::uint16[whiteObj->getSize(0)*whiteObj->getSize(1)];
                m_shading.mulBase = new ito::uint16[whiteObj->getSize(0)*whiteObj->getSize(1)];
                m_shading.subBase = new ito::uint16[whiteObj->getSize(0)*whiteObj->getSize(1)];
                m_shading.x0 = x0;
                m_shading.y0 = y0;
                m_shading.xsize = whiteObj->getSize(1);
                m_shading.ysize = whiteObj->getSize(0);
                for (int y = 0; y < m_shading.ysize; y++)
                {
                    ito::uint16* darkPtr = (ito::uint16*)(darkObj->rowPtr(0, y));
                    ito::uint16* whitePtr = (ito::uint16*)(whiteObj->rowPtr(0, y));
                    for (int x = 0; x < m_shading.xsize; x++)
                    {

                        m_shading.subBase[y*m_shading.xsize + x] = whitePtr[x];
                        m_shading.mulBase[y*m_shading.xsize + x] = darkPtr[x];
                    }
                }
                updateShadingCorrection(0);
            }
        }
        /*
        if (!retValue.containsError())
        {
            int ret = pinitialize_shading(m_handle, shading, this->m_params["sizex"].getVal<int>(), this->m_params["sizey"].getVal<int>(), param1->getVal<int>(), param2->getVal<int>());
            if (ret != MM40_OK)
            {
                retValue += ito::RetVal(ito::retError, ret, tr("mmInitializeShading failed").toLatin1().data());
            }
            else
            {

                //ret = pSetShadingRaw(shading);
                //if (ret != MM40_OK)
                //{
                //    retValue += ito::RetVal(ito::retError, ret, tr("mmSetShadingRaw failed").toLatin1().data());
                //}

            }
        }
        */
    }
/*
    else if (funcName == "update_shading")
    {
        param1 = ito::getParamByName(&(*paramsOpt), "dark_image", &retValue);
        param2 = ito::getParamByName(&(*paramsOpt), "white_image", &retValue);

        if (!retValue.containsError())
        {
            int ret = pUpdateFrameShading(m_handle, NULL, shading);
            if (ret != MM40_OK)
            {
                retValue += ito::RetVal(ito::retError, ret, tr("pUpdateFrameShading failed").toLatin1().data());
            }

        }
    }
    else if (funcName == "calculateShading")
    {
        param1 = ito::getParamByName(&(*paramsOpt), "dark_image", &retValue);
        param2 = ito::getParamByName(&(*paramsOpt), "white_image", &retValue);

        cv::Mat_<WORD> darkMat = cv::Mat_<WORD>::zeros(cv::Size(this->m_params["sizex"].getVal<int>(), this->m_params["sizey"].getVal<int>()));
        cv::Mat_<WORD> whiteMat = cv::Mat_<WORD>::ones(cv::Size(this->m_params["sizex"].getVal<int>(), this->m_params["sizey"].getVal<int>()));

        ito::DataObject* darkObj = (ito::DataObject*)(param1->getVal<void*>());
        ito::DataObject* whiteObj = (ito::DataObject*)(param2->getVal<void*>());

        if (darkObj && darkObj->getType() == ito::tUInt16 && darkObj->getDims() == 2 && darkObj->getSize(0) == darkMat.rows && darkObj->getSize(1) == darkMat.cols)
        {
            for (int y = 0; y < darkMat.rows; y++)
            {
                memcpy(darkMat.ptr(y), darkObj->rowPtr(0,y), 2 * darkMat.cols);
            }
        }
        if (whiteObj && whiteObj->getType() == ito::tUInt16 && whiteObj->getDims() == 2 && whiteObj->getSize(0) == whiteMat.rows && whiteObj->getSize(1) == whiteMat.cols)
        {
            for (int y = 0; y < darkMat.rows; y++)
            {
                memcpy(whiteMat.ptr(y), whiteObj->rowPtr(0,y), 2 * whiteMat.cols);
            }
        }

        LPWORD pBlack = darkMat.ptr<WORD>();
        LPWORD pWhite = whiteMat.ptr<WORD>();

        if (!retValue.containsError())
        {
            int ret = pCalculateShading(m_handle, shading, this->m_params["sizex"].getVal<int>(), this->m_params["sizey"].getVal<int>(), pBlack, pWhite);
            //int ret = pCalculateShadingRaw(shading, this->m_params["sizex"].getVal<int>(), this->m_params["sizey"].getVal<int>(), pBlack, pWhite);
            if (ret != MM40_OK)
            {
                retValue += ito::RetVal(ito::retError, ret, tr("pCalculateShading failed").toLatin1().data());
            }
            else
            {
            }

        }
    }
*/
    else
    {
        retValue += ito::RetVal(ito::retError, 0, tr("function name '%1' does not exist").arg(funcName.toLatin1().data()).toLatin1().data());
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
        waitCond->deleteSemaphore();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Ximea::updateShadingCorrection(int value)
{
    QPointF correction(0.0, 1.0);

    if (!m_shading.valid)
        return;
    value = value < 0 ? 0 : value > 9 ? 9 : value;
    int intTime = (int)(m_params["integration_time"].getVal<double>() * 1000);
    if (m_shading.m_correction.contains(intTime) && m_shading.m_correction[intTime].size() > value)
        correction = m_shading.m_correction[intTime][value];

    float x = correction.x();
    float y = correction.y();
    for (int px = 0; px < m_shading.ysize * m_shading.xsize; px++)
    {
        m_shading.sub[px] = m_shading.subBase[px] * x;
        m_shading.mul[px] = 1.0;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Ximea::activateShadingCorrection(bool enable)
{
    if (!m_shading.valid)
    {
        m_shading.active = false;
        return;
    }

    m_shading.active = enable;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Ximea::dockWidgetVisibilityChanged(bool visible)
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
ito::RetVal Ximea::checkData(ito::DataObject *externalDataObject /*= NULL*/)
{
    int futureHeight = m_params["sizey"].getVal<int>();
    int futureWidth = m_params["sizex"].getVal<int>();
    int futureType;

    int bpp = m_params["bpp"].getVal<int>();
    if (bpp <= 8)
    {
        futureType = ito::tUInt8;
    }
    else if (bpp <= 16)
    {
        futureType = ito::tUInt16;
    }
    else if (bpp < 32)
    {
        futureType = ito::tInt32;
    }
    else if (bpp == 32)
    {
        futureType = ito::tRGBA32;
    }
    else
    {
        futureType = ito::tFloat64;
    }
    int filter_pattern_size = QString(m_params["filter_pattern_size"].getVal<char *>()).toInt();
    if (externalDataObject == NULL)
    {

        if (m_numFrameBurst == 1 && filter_pattern_size == 1)
        {
            if (m_data.getDims() < 2 || m_data.getSize(0) != (unsigned int)futureHeight || m_data.getSize(1) != (unsigned int)futureWidth || m_data.getType() != futureType)
            {
                m_data = ito::DataObject(futureHeight,futureWidth,futureType);
            }
        }
        else
        {
            if (m_numFrameBurst != 1)
            {
                if (m_data.getDims() != 3 || m_data.getSize(0) != m_numFrameBurst || m_data.getSize(1) != (unsigned int)futureHeight || m_data.getSize(2) != (unsigned int)futureWidth || m_data.getType() != futureType)
                {
                    m_data = ito::DataObject(m_numFrameBurst, futureHeight, futureWidth, futureType);
                }
            }
            else//hyperspectral Check
            {
                if (m_data.getDims() < 2 || m_data.getSize(m_data.getDims()-2) != (unsigned int)futureHeight || m_data.getSize(m_data.getDims() - 1) != (unsigned int)futureWidth || m_data.getType() != futureType)
                {
                    m_data = ito::DataObject( m_originalSizeY, m_originalSizeX, futureType);
                }
                if (m_hyperspectralCubeObj.getDims() < 3 || m_hyperspectralCubeObj.getSize(m_hyperspectralCubeObj.getDims() - 3) != (unsigned int)filter_pattern_size || m_hyperspectralCubeObj.getSize(m_hyperspectralCubeObj.getDims() - 2) != (unsigned int)futureHeight || m_hyperspectralCubeObj.getSize(m_hyperspectralCubeObj.getDims() - 1) != (unsigned int)futureWidth || m_hyperspectralCubeObj.getType() != futureType)
                {
                    m_hyperspectralCubeObj = ito::DataObject(filter_pattern_size, futureHeight, futureWidth, futureType);
                }
            }
        }
    }
    else
    {
        int dims = externalDataObject->getDims();
        if (m_numFrameBurst == 1)
        {
            if (externalDataObject->getDims() == 0 && filter_pattern_size == 1)
            {
                *externalDataObject = ito::DataObject(futureHeight,futureWidth,futureType);
            }
            else if (externalDataObject->getDims() == 0 && filter_pattern_size != 1)
            {
                *externalDataObject = ito::DataObject(filter_pattern_size, futureHeight, futureWidth, futureType);
            }
            else if (externalDataObject->calcNumMats () != 1 && filter_pattern_size == 1)
            {
                return ito::RetVal(ito::retError, 0, tr("Error during check data, external dataObject invalid. Object has more or less than 1 plane. It must be of right size and type or an uninitilized image.").toLatin1().data());
            }
            else if (externalDataObject->getSize(dims - 2) != (unsigned int)futureHeight || externalDataObject->getSize(dims - 1) != (unsigned int)futureWidth || externalDataObject->getType() != futureType)
            {
                return ito::RetVal(ito::retError, 0, tr("Error during check data, external dataObject invalid. Object must be of right size and type or a uninitilized image.").toLatin1().data());
            }
        }
        else
        {
            if (externalDataObject->getDims() == 0)
            {
                *externalDataObject = ito::DataObject(m_numFrameBurst,futureHeight,futureWidth,futureType);
            }
            else if (externalDataObject->calcNumMats () != m_numFrameBurst)
            {
                return ito::RetVal(ito::retError, 0, tr("Error during check data, external dataObject invalid. Frame burst is %1, the external object must then have %1 planes.").arg(m_numFrameBurst).toLatin1().data());
            }
            else if (externalDataObject->getSize(dims - 2) != (unsigned int)futureHeight || externalDataObject->getSize(dims - 1) != (unsigned int)futureWidth || externalDataObject->getType() != futureType)
            {
                return ito::RetVal(ito::retError, 0, tr("Error during check data, external dataObject invalid. Object must be of right size and type or a uninitilized image.").toLatin1().data());
            }
        }
    }

    return ito::retOk;
}

/* ********************************************************************
    Plugin "IDSuEye" for itom software
    URL: https://github.com/itom-project/plugins
    Copyright (C) 2014, Pulsar Photonics GmbH, Aachen
    Copyright (C) 2017, Institut für Technische Optik, Universität Stuttgart

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

#include "IDSuEye.h"
#include "pluginVersion.h"

#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>
#include <qmetaobject.h>

#include "DockWidgetIDS.h"
#include "DialogIDS.h"

#include "common/helperCommon.h"



//----------------------------------------------------------------------------------------------------------------------------------
//! constructor for IDSuEye
/*!
    In this constructor the m_params-vector with all parameters, which are accessible by getParam or setParam, is built.
    Additionally the optional docking widget for the IDSuEye's toolbar is instantiated and created by createDockWidget.

    \param [in] uniqueID is an unique identifier for this IDSuEye-instance
    \sa ito::tParam, createDockWidget, setParam, getParam
*/
IDSuEye::IDSuEye() :
    AddInGrabber(),
    m_camera(IS_INVALID_HIDS),
    m_colouredOutput(false),
    m_captureVideoActive(false),
    m_seqAvailable(false),
    m_seqEventInit(false)
{

    m_blacklevelRange.s32Inc = m_blacklevelRange.s32Min = m_blacklevelRange.s32Max = 0;

    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "IDSuEye", "GrabberName");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("integration_time", ito::ParamBase::Double, 0.0, 1.0, 0.005, tr("Exposure time of chip (in seconds).").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("long_integration_time_enabled", ito::ParamBase::Int, 0, 1, 0, tr("If long exposure time is available, this parameter let you enable this. If this value is changed, the range and value of integration_time might change, too.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("pixel_clock", ito::ParamBase::Int, 0, 0, 0, tr("Pixel clock in MHz. If the pixel clock is too high, data packages might be lost. A change of the pixel clock might influence the exposure time.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("binning", ito::ParamBase::Int, 101, 1616, 101, tr("Horizontal and vertical binning, depending on camera ability. 104 means a 1x binning in horizontal and 4x binning in vertical direction. (values up to 1x, 2x, 3x, 4x, 5x, 6x, 8x, 12x are valid; if read-only binning is not supported; some cameras only support certain combinations of binnings.)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("frame_rate", ito::ParamBase::Double, 0.0, 10000.0, 0.5, tr("frame rate in fps (will affect the allowed range of the integration_time, this frame_rate is only considered if trigger_mode == 'off'.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("gain", ito::ParamBase::Double, 0.0, 1.0, 0.5, tr("Gain (normalized value 0..1)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("gain_rgb", ito::ParamBase::DoubleArray, NULL, tr("RGB-gain values (normalized value 0..1)").toLatin1().data());
    double rgbGain[] = {0.5, 0.5, 0.5};
    paramVal.setVal<double*>(rgbGain,3);
    paramVal.setMeta(new ito::DoubleArrayMeta(0.0, 1.0), true);
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("gain_boost_enabled", ito::ParamBase::Int, 0, 1, 0, tr("enables / disables an additional analog hardware gain (gain boost). Readonly if not supported.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("offset", ito::ParamBase::Double, 0.0, 1.0, 0.5, tr("Offset (leads to blacklevel offset) (normalized value 0..1). Readonly if not adjustable.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("auto_blacklevel_enabled", ito::ParamBase::Int, 0, 0, 0, tr("If the camera supports an auto blacklevel correction (auto offset in addition to offset), this feature can be enabled / disabled by this parameter.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizex", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 2048, 2048, tr("Pixelsize in x (cols)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizey", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 2048, 2048, tr("Pixelsize in y (rows)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

#if defined(ITOM_ADDININTERFACE_VERSION) && ITOM_ADDININTERFACE_VERSION > 0x010300
    int roi[] = {0, 0, 2048, 2048};
    paramVal = ito::Param("roi", ito::ParamBase::IntArray, 4, roi, tr("ROI (x,y,width,height) [this replaces the values x0,x1,y0,y1]").toLatin1().data());
    ito::RectMeta *rm = new ito::RectMeta(ito::RangeMeta(0, 2047), ito::RangeMeta(0, 2047));
    paramVal.setMeta(rm, true);
    m_params.insert(paramVal.getName(), paramVal);
#endif

    paramVal = ito::Param("x0", ito::ParamBase::Int, 0, 2047, 0, tr("Index of left boundary pixel within ROI").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("y0", ito::ParamBase::Int, 0, 2047, 0, tr("Index of top boundary pixel within ROI").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("x1", ito::ParamBase::Int, 0, 2047, 2047, tr("Index of right boundary pixel within ROI").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("y1", ito::ParamBase::Int, 0, 2047, 2047, tr("Index of bottom boundary pixel within ROI").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("trigger_mode", ito::ParamBase::String, "software", tr("trigger modes for starting a new image acquisition, depending on the camera the following modes are supported: 'off' (fixed frame_rate), without fixed frame_rate: 'software', 'hi_lo', 'lo_hi', 'pre_hi_lo', 'pre_lo_hi'").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("cam_model", ito::ParamBase::String | ito::ParamBase::Readonly, "n.a.", tr("Model identifier of the attached camera").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("serial_number", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("Serial number of camera").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("cam_id", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 255, 0, tr("ID of the camera").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("sensor_type", ito::ParamBase::String | ito::ParamBase::Readonly, "n.a.", tr("Sensor type of the attached camera").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("color_mode", ito::ParamBase::String, "gray", tr("color_mode: 'gray' (default) or 'color' if color camera").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    //meta information is added when camera is synchronized and allows 'gray' and possibly 'color'

    paramVal = ito::Param("bpp", ito::ParamBase::Int, 8, 16, 16, tr("Bitdepth of each pixel").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("fps", ito::ParamBase::Double | ito::ParamBase::Readonly, 0.0, 1000.0, 0.0, tr("current fps reported by camera").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("timeout", ito::ParamBase::Double, 0.04, 1000.0, 3.0, tr("timeout in seconds when waiting for the next image. For sequences x1000.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("num_buffer", ito::ParamBase::Int, 1, 2000, 1, tr("Number of Buffers used for acquisition. Note that if this is > 1, a sequence of images will be acquired.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    if (hasGuiSupport())
    {
        //now create dock widget for this plugin
        DockWidgetIDS *dw = new DockWidgetIDS(this);

        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dw);
    }

    //globally disable message box based error messages
    is_SetErrorReport(0, IS_DISABLE_ERR_REP);
}

//----------------------------------------------------------------------------------------------------------------------------------
//! destructor
/*!
    \sa ~AddInBase
*/
IDSuEye::~IDSuEye()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
//! init method which is called by the addInManager after the initiation of a new instance of IDSuEye.
/*!
    This init method gets the mandatory and optional parameter vectors of type tParam and must copy these given parameters to the
    internal m_params-vector. Notice that this method is called after that this instance has been moved to its own (non-gui) thread.

    \param [in] paramsMand is a pointer to the vector of mandatory tParams.
    \param [in] paramsOpt is a pointer to the vector of optional tParams.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
*/
ito::RetVal IDSuEye::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retVal;

    HIDS camera_id = static_cast<HIDS>(paramsOpt->at(0).getVal<int>());
    QString init_color_mode = paramsOpt->at(1).getVal<char*>();

    m_NumberOfBuffers = m_params["num_buffer"].getVal<int>();

    if (paramsOpt->at(2).getVal<int>() > 0)
    {
        is_SetErrorReport(0, IS_ENABLE_ERR_REP);
    }

    bool valid_camera_id = false;

    if (camera_id > 0) //we first need to check whether the desired camera is not in use yet
    {
        INT numberOfCameras = 0;
        retVal += checkError(is_GetNumberOfCameras(&numberOfCameras));

        if (!retVal.containsError())
        {
            bool id_found = false;
            bool in_use = false;
            // get the list of cameras that is currently attached to the system
            UEYE_CAMERA_LIST *pucl = (UEYE_CAMERA_LIST*) new BYTE [sizeof (ULONG) + numberOfCameras * sizeof (UEYE_CAMERA_INFO)];
            pucl->dwCount = numberOfCameras;
            retVal += checkError(is_GetCameraList(pucl));
            if (!retVal.containsError())
            {
                for (int idx = 0; idx < numberOfCameras; ++idx)
                {
                    if (pucl->uci[idx].dwCameraID == camera_id)
                    {
                        m_params["serial_number"].setVal<const char*>(pucl->uci[idx].SerNo);
                        id_found = true;
                        in_use = (pucl->uci[idx].dwInUse > 0);
                        break;
                    }
                }
            }
            delete[] pucl;
            pucl = NULL;

            if (!id_found)
            {
                retVal += ito::RetVal::format(ito::retError, 0, "A camera with the desired ID %i could not be found.", camera_id);
            }
            else if (in_use)
            {
                retVal += ito::RetVal::format(ito::retError, 0, "A camera with the desired ID %i is already in use.", camera_id);
            }
        }
    }

    if (!retVal.containsError())
    {
        retVal += checkError(is_InitCamera(&camera_id, NULL));
    }

    if (!retVal.containsError())
    {
        if (paramsOpt->at(0).getVal<int>() <= 0) //auto-camera has been detected -> get serial number now
        {
            INT numberOfCameras = 0;
            retVal += checkError(is_GetNumberOfCameras(&numberOfCameras));
            if (!retVal.containsError())
            {
                // get the list of cameras that is currently attached to the system
                UEYE_CAMERA_LIST *pucl = (UEYE_CAMERA_LIST*) new BYTE [sizeof (ULONG) + numberOfCameras * sizeof (UEYE_CAMERA_INFO)];
                pucl->dwCount = numberOfCameras;
                retVal += checkError(is_GetCameraList(pucl));
                if (!retVal.containsError())
                {
                    for (int idx = 0; idx < numberOfCameras; ++idx)
                    {
                        if (pucl->uci[idx].dwDeviceID == camera_id)
                        {
                            m_params["serial_number"].setVal<const char*>(pucl->uci[idx].SerNo);
                            break;
                        }
                    }
                }
                delete[] pucl;
                pucl = NULL;
            }
        }

        //camera successfully opened...
        m_camera = camera_id;
        m_params["cam_id"].setVal<int>(m_camera);

        //get sensor info
        retVal += loadSensorInfo();

        setIdentifier(QString("%1 (%2)").arg(m_params["cam_model"].getVal<char*>()).arg(m_params["serial_number"].getVal<char*>()));

        if (!retVal.containsError())
        {
            //try to set different color modes in order to check if they are supported
            //crosscheck if intended color mode lies in range of supported bit depths of the sensor if get_supported_sensor_bid_depths is supported
            UINT bitDepths = 0;
            is_DeviceFeature(m_camera, IS_DEVICE_FEATURE_CMD_GET_SUPPORTED_SENSOR_BIT_DEPTHS, (void*)&bitDepths, sizeof(bitDepths));

            int minBpp = 100;
            int maxBpp = 0;
            int maxMonochrome = IS_CM_MONO8;

            int test = bitDepths & IS_SENSOR_BIT_DEPTH_8_BIT;

            if (is_SetColorMode(m_camera, IS_CM_MONO8) == IS_SUCCESS && (bitDepths & IS_SENSOR_BIT_DEPTH_8_BIT || bitDepths==0))
            {
                minBpp = std::min(minBpp,8);
                maxBpp = std::max(maxBpp,8);
            }
            if (is_SetColorMode(m_camera, IS_CM_MONO10) == IS_SUCCESS && (bitDepths & IS_SENSOR_BIT_DEPTH_10_BIT || bitDepths==0))
            {
                minBpp = std::min(minBpp,10);
                maxBpp = std::max(maxBpp,10);
                maxMonochrome = IS_CM_MONO10;
            }
            if (is_SetColorMode(m_camera, IS_CM_MONO12) == IS_SUCCESS && (bitDepths & IS_SENSOR_BIT_DEPTH_12_BIT|| bitDepths==0))
            {
                minBpp = std::min(minBpp,12);
                maxBpp = std::max(maxBpp,12);
                maxMonochrome = IS_CM_MONO12;
            }
            if (is_SetColorMode(m_camera, IS_CM_MONO16) == IS_SUCCESS && bitDepths == 0) // IS_SENSOR_BIT_DEPTH_16_BIT not supported by IDS API
            {
                minBpp = std::min(minBpp,16);
                maxBpp = std::max(maxBpp,16);
                maxMonochrome = IS_CM_MONO16;
            }

            m_monochromeBitDepthRange.s32X = minBpp;
            m_monochromeBitDepthRange.s32Y = maxBpp;

            m_params["bpp"].setMeta(new ito::IntMeta(minBpp, maxBpp), true);



            //try to set color mode to either mono8, mono10, mono12 for monochrome cameras or
            //to bgra8 for color cameras (since this format is the only supported color format of itom
            if (m_sensorInfo.nColorMode & IS_COLORMODE_MONOCHROME)
            {
                if (init_color_mode == "color")
                {
                    retVal += ito::RetVal(ito::retError, 0, "monochrome camera loaded. color mode 'color' not possible");
                }
                else //init_color_mode == "auto" | "gray"
                {
                    retVal += checkError(is_SetColorMode(m_camera, maxMonochrome));
                }
            }
            else if (m_sensorInfo.nColorMode & IS_COLORMODE_BAYER)
            {
                if (init_color_mode == "color" || init_color_mode == "auto")
                {
                    retVal += checkError(is_SetColorMode(m_camera, IS_CM_BGRA8_PACKED));
                }
                else //init_color_mode == "gray"
                {
                    retVal += checkError(is_SetColorMode(m_camera, maxMonochrome));
                }
            }
            else
            {
                retVal += ito::RetVal(ito::retError, 0, "currently only bayer pattern based color cameras or monochrome cameras are supported");
            }
        }

        if (!retVal.containsError())
        {
            //set some non-changeable values
            retVal += checkError(is_SetRopEffect(m_camera, IS_SET_ROP_MIRROR_UPDOWN, 0, 0));
            retVal += checkError(is_SetRopEffect(m_camera, IS_SET_ROP_MIRROR_LEFTRIGHT, 0, 0));
            retVal += setMeanFrameRate();
            retVal += synchronizeCameraSettings();
        }

        if (!retVal.containsError())
        {
            // init event
            IS_INIT_EVENT event = {IS_SET_EVENT_SEQ, FALSE, FALSE};
            retVal += checkError(is_Event(m_camera, IS_EVENT_CMD_INIT, &event, sizeof(event)), "initEvent");

            if (!retVal.containsError())
            {
                // enable event
                UINT events[] = { IS_SET_EVENT_SEQ };
                retVal += checkError(is_Event(m_camera, IS_EVENT_CMD_ENABLE, events, sizeof(events)), "enableEvent");
                m_seqEventInit = true;
            }
        }
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
//! close method which is called before that this instance is deleted by the IDSuEyeInterface
/*!
    notice that this method is called in the actual thread of this instance.

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal IDSuEye::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;

    if (this->grabberStartedCount() > 0)
    {
        setGrabberStarted(1);
        retValue += stopDevice(NULL);
    }


    if (!m_vpcSeqImgMem.empty())
    {
        for (int i = (m_oldNumBuf - 1); i >= 0; i--)
        {
            // free buffers
            if (is_FreeImageMem(m_camera, m_vpcSeqImgMem.at(i), m_viSeqMemId.at(i)) != IS_SUCCESS)
            {
                break;
            }
        }

        // no valid buffers any more
        m_viSeqMemId.clear();
        m_vpcSeqImgMem.clear();
    }

    if (m_seqEventInit)
    {
        UINT events[] = { IS_SET_EVENT_SEQ };
        // disable event
        is_Event(m_camera, IS_EVENT_CMD_DISABLE, events, sizeof(events));
        // exit event
        is_Event(m_camera, IS_EVENT_CMD_EXIT, events, sizeof(events));
        m_seqEventInit = false;
    }

    if (m_camera != IS_INVALID_HIDS)
    {
        retValue += checkError(is_ExitCamera(m_camera));
        m_camera = IS_INVALID_HIDS;
    }

    if (waitCond)
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
ito::RetVal IDSuEye::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;
    QString key;
    bool hasIndex = false;
    int index;
    QString suffix;
    ParamMapIterator it;

    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (retValue == ito::retOk)
    {
        retValue += apiGetParamFromMapByKey(m_params, key, it, false);
    }

    if (!retValue.containsError())
    {
        if (key == "fps")
        {
            double tmpFps;
            is_GetFramesPerSecond (m_camera, &tmpFps);
            it->setVal<double>(tmpFps);
            *val = *it;
        }
        else if (hasIndex)
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
//! sets parameter of m_params with key name.
/*!
    This method copies the given value  to the m_params-parameter.

    \param [in] name is the key name of the parameter
    \param [in] val is the double value to set.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk in case that everything is ok, else retError
    \sa ito::tParam, ItomSharedSemaphore
*/
ito::RetVal IDSuEye::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex;
    int index;
    QString suffix;
    ParamMapIterator it;
    int running = 0;
    int startDeviceCount = 0; // if > 0, the device is started at the end of the function and the counter is set to this value.

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName( val->getName(), key, hasIndex, index, suffix );

    if (!retValue.containsError())
    {
        retValue += apiGetParamFromMapByKey(m_params, key, it, true);
    }

    if (!retValue.containsError())
    {
        retValue += apiValidateAndCastParam(*it, *val, false, true, true);
    }

    if (!retValue.containsError())
    {
        if (key == "x0" || key == "x1" || key == "y0" || key == "y1" || key == "roi")
        {
            IS_RECT rectAOI;
            retValue += checkError(is_AOI(m_camera, IS_AOI_IMAGE_GET_AOI, (void*)&rectAOI, sizeof(rectAOI)));

            if (!retValue.containsError())
            {
                if (key == "x0")
                {
                    rectAOI.s32X = val->getVal<int>();
                    rectAOI.s32Width = 1 + m_params["x1"].getVal<int>() - rectAOI.s32X;
                }
                else if (key == "y0")
                {
                    rectAOI.s32Y = val->getVal<int>();
                    rectAOI.s32Height = 1 + m_params["y1"].getVal<int>() - rectAOI.s32Y;
                }
                else if (key == "x1")
                {
                    rectAOI.s32Width = 1 + val->getVal<int>() - m_params["x0"].getVal<int>();
                }
                else if (key == "y1")
                {
                    rectAOI.s32Height = 1 + val->getVal<int>() - m_params["y0"].getVal<int>();
                }
                else if (key == "roi")
                {
                    if (!hasIndex)
                    {
                        if (val->getLen() != 4)
                        {
                            retValue += ito::RetVal(ito::retError, 0, "roi must have 4 values");
                        }
                        else
                        {
                            int *roi = val->getVal<int*>();
                            rectAOI.s32X = roi[0];
                            rectAOI.s32Y = roi[1];
                            rectAOI.s32Width = roi[2];
                            rectAOI.s32Height = roi[3];
                        }
                    }
                    else
                    {
                        switch (index)
                        {
                        case 0:
                            rectAOI.s32X = val->getVal<int>();
                            break;
                        case 1:
                            rectAOI.s32Y = val->getVal<int>();
                            break;
                        case 2:
                            rectAOI.s32Width = val->getVal<int>();
                            break;
                        case 3:
                            rectAOI.s32Height = val->getVal<int>();
                            break;
                        }
                    }
                }

                //roi needs restart of device
                startDeviceCount = grabberStartedCount();
                if (startDeviceCount > 0)
                {
                    setGrabberStarted(1);
                    stopDevice(NULL);
                }

                retValue += checkError(is_AOI(m_camera, IS_AOI_IMAGE_SET_AOI, (void*)&rectAOI, sizeof(rectAOI)));
                //exposure time might be changed, therefore try to set it to currently set value without checking error
                //the real value is then synchronized below
                double timeMs = m_params["integration_time"].getVal<double>() * 1.0e3;
                is_Exposure(m_camera, IS_EXPOSURE_CMD_SET_EXPOSURE, (void*)&timeMs, sizeof(timeMs));
            }

            if (!retValue.containsError())
            {
                retValue += synchronizeCameraSettings(sExposure | sRoi | sFrameTime);
            }


        }
        else if (key == "integration_time")
        {
            double timeMs = val->getVal<double>() * 1.0e3;
            retValue += checkError(is_Exposure(m_camera, IS_EXPOSURE_CMD_SET_EXPOSURE, (void*)&timeMs, sizeof(timeMs)));

            if (!retValue.containsError())
            {
                it->setVal<double>(timeMs * 1.0e-3); //after a call, timeMs contains the really set exposure time
            }
        }
        else if (key == "long_integration_time_enabled")
        {
            if (it->getMax() > 0 && val->getVal<int>() != it->getVal<int>()) //if max is 0: this feature is not supported and val->getVal<int>() must be 0.
            {
                UINT v = val->getVal<int>();
                retValue += checkError(is_Exposure(m_camera, IS_EXPOSURE_CMD_SET_LONG_EXPOSURE_ENABLE, (void*)&v, sizeof(v)));

                if (!retValue.containsError())
                {
                    retValue += synchronizeCameraSettings(sExposure);
                }
            }
        }
        else if (key == "pixel_clock")
        {
            //pixel clock needs restart of device
            startDeviceCount = grabberStartedCount();
            if (startDeviceCount > 0)
            {
                setGrabberStarted(1);
                stopDevice(NULL);
            }

            UINT clock = val->getVal<int>();
            retValue += checkError(is_PixelClock(m_camera, IS_PIXELCLOCK_CMD_SET, (void*)&clock, sizeof(clock)));

            if (!retValue.containsError())
            {
                retValue += synchronizeCameraSettings(sPixelClock | sExposure | sFrameTime);
            }
        }
        else if (key == "binning")
        {
            int b = val->getVal<int>();
            int v = b % 100;
            int h = (b-v) / 100;
            INT mode = 0;
            switch (v)
            {
                case 1: break;
                case 2: mode |= IS_BINNING_2X_VERTICAL; break;
                case 3: mode |= IS_BINNING_3X_VERTICAL; break;
                case 4: mode |= IS_BINNING_4X_VERTICAL; break;
                case 5: mode |= IS_BINNING_5X_VERTICAL; break;
                case 6: mode |= IS_BINNING_6X_VERTICAL; break;
                case 8: mode |= IS_BINNING_8X_VERTICAL; break;
                case 16: mode |= IS_BINNING_16X_VERTICAL; break;
                default:
                    retValue += ito::RetVal::format(ito::retError, 0, "unknown vertical binning factor (%ix)", v);
            }
            switch (h)
            {
                case 1: break;
                case 2: mode |= IS_BINNING_2X_HORIZONTAL; break;
                case 3: mode |= IS_BINNING_3X_HORIZONTAL; break;
                case 4: mode |= IS_BINNING_4X_HORIZONTAL; break;
                case 5: mode |= IS_BINNING_5X_HORIZONTAL; break;
                case 6: mode |= IS_BINNING_6X_HORIZONTAL; break;
                case 8: mode |= IS_BINNING_8X_HORIZONTAL; break;
                case 16: mode |= IS_BINNING_16X_HORIZONTAL; break;
                default:
                    retValue += ito::RetVal::format(ito::retError, 0, "unknown horizontal binning factor (%ix)", v);
            }

            if (!retValue.containsError())
            {
                //binning needs restart of device
                startDeviceCount = grabberStartedCount();
                if (startDeviceCount > 0)
                {
                    setGrabberStarted(1);
                    stopDevice(NULL);
                }

                retValue += checkError(is_SetBinning(m_camera, mode), "Set binning");
            }

            if (!retValue.containsError())
            {
                retValue += synchronizeCameraSettings(sBinning | sRoi | sExposure | sFrameTime);
            }
        }
        else if (key == "gain")
        {
            retValue += checkError(is_SetHardwareGain(m_camera, qRound(val->getVal<double>()*100.0), IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER), "Set gain");

            if (!retValue.containsError())
            {
                retValue += synchronizeCameraSettings(sGain);
            }
        }
        else if (key == "gain_rgb")
        {
            if (hasIndex)
            {
                switch (index)
                {
                case 0:
                    retValue += checkError(is_SetHardwareGain(m_camera, IS_IGNORE_PARAMETER, qRound(val->getVal<double>()*100.0), IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER), "set hardware gain");
                    break;
                case 1:
                    retValue += checkError(is_SetHardwareGain(m_camera, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, qRound(val->getVal<double>()*100.0), IS_IGNORE_PARAMETER), "set hardware gain");
                    break;
                case 2:
                    retValue += checkError(is_SetHardwareGain(m_camera, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, qRound(val->getVal<double>()*100.0)), "set hardware gain");
                    break;
                default:
                    retValue += ito::RetVal(ito::retError, 0, "index of gain_rgb must be between 0..2");
                }
            }
            else if (val->getLen() != 3)
            {
                retValue += ito::RetVal(ito::retError, 0, "gain_rgb must have three values for red, green and blue gain");
            }
            else
            {
                double *vals = val->getVal<double*>();
                retValue += checkError(is_SetHardwareGain(m_camera, IS_IGNORE_PARAMETER, qRound(vals[0]*100.0), IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER), "set hardware gain");
                retValue += checkError(is_SetHardwareGain(m_camera, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, qRound(vals[1]*100.0), IS_IGNORE_PARAMETER), "set hardware gain");
                retValue += checkError(is_SetHardwareGain(m_camera, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, qRound(vals[2]*100.0)), "set hardware gain");
            }

            if (!retValue.containsError())
            {
                retValue += synchronizeCameraSettings(sGain);
            }
        }
        else if (key == "gain_boost_enabled")
        {
            retValue += checkError(is_SetGainBoost(m_camera, val->getVal<int>() == 1 ? IS_SET_GAINBOOST_ON : IS_SET_GAINBOOST_OFF), "set gain boost");
            if (!retValue.containsError())
            {
                retValue += synchronizeCameraSettings(sGain);
            }
        }
        else if (key == "offset")
        {
            if (m_blacklevelRange.s32Max == 0 && m_blacklevelRange.s32Min == 0)
            {
                retValue += ito::RetVal(ito::retError, 0, "no valid blacklevel offset range available");
            }
            else
            {
                INT offset = (m_blacklevelRange.s32Max - m_blacklevelRange.s32Min) * val->getVal<double>();
                offset = m_blacklevelRange.s32Min + offset - (offset % m_blacklevelRange.s32Inc);
                retValue += checkError(is_Blacklevel(m_camera, IS_BLACKLEVEL_CMD_SET_OFFSET, (void*)&offset, sizeof(offset)));
            }

            if (!retValue.containsError())
            {
                retValue += synchronizeCameraSettings(sOffset);
            }
        }
        else if (key == "auto_blacklevel_enabled")
        {
            if (it->getMax() > 0)
            {
                INT nMode = val->getVal<int>() > 0 ? IS_AUTO_BLACKLEVEL_ON : IS_AUTO_BLACKLEVEL_OFF;
                retValue += checkError(is_Blacklevel(m_camera, IS_BLACKLEVEL_CMD_SET_MODE, (void*)&nMode , sizeof(nMode )));

                if (!retValue.containsError())
                {
                    retValue += synchronizeCameraSettings(sOffset);
                }
            }
        }
        else if (key == "trigger_mode")
        {
            QString mode = val->getVal<char*>();
            INT t;
            if (mode == "software")
            {
                t = IS_SET_TRIGGER_SOFTWARE;
            }
            else if (mode == "off")
            {
                t = IS_SET_TRIGGER_OFF;
            }
            else if (mode == "hi_lo")
            {
                t = IS_SET_TRIGGER_HI_LO;
            }
            else if (mode == "lo_hi")
            {
                t = IS_SET_TRIGGER_LO_HI;
            }
            else if (mode == "pre_hi_lo")
            {
                t = IS_SET_TRIGGER_PRE_HI_LO;
            }
            else if (mode == "pre_lo_hi")
            {
                t = IS_SET_TRIGGER_PRE_LO_HI;
            }
            else
            {
                retValue += ito::RetVal::format(ito::retError, 0, "unsupported or unknown trigger value '%s'", val->getVal<char*>());
            }

            if (!retValue.containsError())
            {
                //trigger_mode needs restart of device
                startDeviceCount = grabberStartedCount();
                if (startDeviceCount > 0)
                {
                    setGrabberStarted(1);
                    stopDevice(NULL);
                }

                retValue += checkError(is_SetExternalTrigger(m_camera, t), "set external trigger");
            }

            if (!retValue.containsError())
            {
                retValue += synchronizeCameraSettings(sTriggerMode);
            }
        }
        else if (key == "bpp")
        {
            if (strcmp(m_params["color_mode"].getVal<char*>(),"color")==0)
            {
                retValue += ito::RetVal(ito::retError, 0, "bpp cannot be changed if color_mode is 'color'");
            }
            else
            {
                //bpp needs restart of device
                startDeviceCount = grabberStartedCount();
                if (startDeviceCount > 0)
                {
                    setGrabberStarted(1);
                    stopDevice(NULL);
                }

                switch (val->getVal<int>())
                {
                case 8:
                    retValue += checkError(is_SetColorMode(m_camera, IS_CM_MONO8));
                    break;
                case 10:
                    retValue += checkError(is_SetColorMode(m_camera, IS_CM_MONO10));
                    break;
                case 12:
                    retValue += checkError(is_SetColorMode(m_camera, IS_CM_MONO12));
                    break;
                case 16:
                    retValue += checkError(is_SetColorMode(m_camera, IS_CM_MONO16));
                    break;
                default:
                    retValue += ito::RetVal(ito::retError, 0, "unsupported bitdepth for gray value color mode");
                }
            }

            if (!retValue.containsError())
            {
                retValue += synchronizeCameraSettings(sBppAndColorMode | sPixelClock | sExposure | sFrameTime);

            }
        }
        else if (key == "color_mode")
        {
            //color_mode needs restart of device
            startDeviceCount = grabberStartedCount();
            if (startDeviceCount > 0)
            {
                setGrabberStarted(1);
                stopDevice(NULL);
            }

            if (strcmp(val->getVal<char*>(), "color") == 0)
            {
                retValue += checkError(is_SetColorMode(m_camera, IS_CM_BGRA8_PACKED));
            }
            else
            {
                switch (m_params["bpp"].getVal<int>())
                {
                case 8:
                    retValue += checkError(is_SetColorMode(m_camera, IS_CM_MONO8));
                    break;
                case 10:
                    retValue += checkError(is_SetColorMode(m_camera, IS_CM_MONO10));
                    break;
                case 12:
                    retValue += checkError(is_SetColorMode(m_camera, IS_CM_MONO12));
                    break;
                case 16:
                    retValue += checkError(is_SetColorMode(m_camera, IS_CM_MONO16));
                    break;
                default:
                    retValue += ito::RetVal(ito::retError, 0, "unsupported bitdepth for gray value color mode");
                }
            }

            if (!retValue.containsError())
            {
                retValue += synchronizeCameraSettings(sBppAndColorMode | sPixelClock | sExposure | sFrameTime);
            }
        }
        else if (key == "frame_rate")
        {
            retValue += setFrameRate(val->getVal<double>());
            retValue += synchronizeCameraSettings(sExposure | sFrameTime);
        }
        else if (key == "num_buffer")
        {
            m_NumberOfBuffers = val->getVal<int>();
            m_params["num_buffer"].setVal<int>(m_NumberOfBuffers);
        }
        else
        {
            //e.g. timeout
            it->copyValueFrom(val.data());
        }
    }

    if (!retValue.containsError())
    {
        retValue += checkData();
    }

    if (startDeviceCount > 0)
    {
        retValue += startDevice(NULL);
        setGrabberStarted(startDeviceCount);
    }

    emit parametersChanged(m_params); //send changed parameters to any connected dialogs or dock-widgets

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
    In the IDSuEye, this method does nothing. In general, the hardware camera should be initialized in this method and necessary memory should be allocated.

    \note This method is similar to VideoCapture::open() of openCV

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if starting was successful, retWarning if startDevice has been calling at least twice.
*/
ito::RetVal IDSuEye::startDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = ito::retOk;

    retValue += checkData(); //this will be reallocated in this method.

    if (grabberStartedCount() == 0)
    {
        retValue += checkError(is_ImageQueue(m_camera, IS_IMAGE_QUEUE_CMD_INIT, NULL, 0), "initImageQueue");
    }

    incGrabberStarted();

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
    In this IDSuEye, this method does nothing. In general, the hardware camera should be closed in this method.

    \note This method is similar to VideoCapture::release() of openCV

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if everything is ok, retError if camera wasn't started before
    \sa startDevice
*/
ito::RetVal IDSuEye::stopDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = ito::retOk;

    decGrabberStarted();
    if (grabberStartedCount() == 0)
    {
        retValue += checkError(is_ImageQueue(m_camera, IS_IMAGE_QUEUE_CMD_EXIT, NULL, 0));
    }
    else if (grabberStartedCount() < 0)
    {
        retValue += ito::RetVal(ito::retError, 1001, tr("StopDevice of IDSuEye can not be executed, since camera has not been started.").toLatin1().data());
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
ito::RetVal IDSuEye::acquire(const int trigger, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = ito::retOk;

    if (grabberStartedCount() <= 0)
    {
        retValue += ito::RetVal(ito::retError, 1002, tr("Acquire of IDSuEye can not be executed, since camera has not been started.").toLatin1().data());

        //only release it if not yet done
        if (waitCond)
        {
            waitCond->returnValue = retValue;
            waitCond->release();
        }
    }
    else
    {
        retValue += checkError(is_CaptureVideo(m_camera,IS_DONT_WAIT), "captureVideo"); //start image acquisition

        UINT events[] = { IS_SET_EVENT_SEQ };
        UINT timeoutMs = static_cast<UINT>(m_params["timeout"].getVal<double>() * 1000.0 * (double)m_NumberOfBuffers);
        IS_WAIT_EVENTS wait_events = { events, 1, FALSE, timeoutMs, 0, 0 };
        INT eventRet = is_Event(m_camera, IS_EVENT_CMD_WAIT, &wait_events, sizeof(wait_events));

        if (eventRet == IS_TIMED_OUT)
        {
            retValue += ito::RetVal(ito::retError, 0, "timeout while acquiring image.");
        }
        else if (wait_events.nSignaled != IS_SET_EVENT_SEQ)
        {
            retValue += ito::RetVal(ito::retError, 0, "wrong event signalling when acquiring image.");
        }

        retValue += checkError(is_StopLiveVideo(m_camera, IS_FORCE_VIDEO_STOP));

        if (!retValue.containsError())
        {
            int width = m_params["sizex"].getVal<int>();
            int height = m_params["sizey"].getVal<int>();
            int bpp = m_params["bpp"].getVal<int>();

            for (int i = 0; i < m_NumberOfBuffers; i++)
            {
                int pnX, pnY, pnBits, pnPitch;
                retValue += checkError(is_InquireImageMem(m_camera, m_vpcSeqImgMem.at(i), m_viSeqMemId.at(i), &pnX, &pnY, &pnBits, &pnPitch));
                //width should be pnX, bitspixel should be pnBits!!!
                int bytesPerLine = width * m_bitspixel / 8;

                if (m_colouredOutput)
                {
                    UCHAR *sourcePtr = (UCHAR*)(m_vpcSeqImgMem.at(i) + 3);
                    //set alpha value in image to 255 (else it is transparent)
                    for (int y = 0; y < height; ++y)
                    {
                        for (int x = 0; x < width; ++x)
                        {
                            sourcePtr[x * 4] = 255;
                        }

                        sourcePtr += bytesPerLine;
                    }
                }

                if (pnPitch == bytesPerLine)
                {
                    memcpy(m_data.rowPtr(i, 0), m_vpcSeqImgMem.at(i), height * bytesPerLine);
                }
                else
                {
                    char *imgBuffer;
                    imgBuffer = m_vpcSeqImgMem.at(i);

                    for (int y = 0; y < height; y++)
                    {
                        memcpy(m_data.rowPtr(i, y), imgBuffer, bytesPerLine);
                        imgBuffer += pnPitch;
                    }
                }
            }

            m_seqAvailable = true;
            m_acquisitionRetVal = retValue;
        }

        for (int i = 0; i < m_NumberOfBuffers; i++)
        {
            retValue += checkError(is_UnlockSeqBuf(m_camera, m_viSeqMemId.at(i), m_vpcSeqImgMem.at(i)));
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
//! Returns the grabbed camera frame as a shallow copy.
/*!
    This method copies the recently grabbed camera frame to the given DataObject-handle

    \note This method is similar to VideoCapture::retrieve() of openCV

    \param [in,out] vpdObj is the pointer to a given dataObject (this pointer should be cast to ito::DataObject*) where the acquired image is shallow-copied to.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if everything is ok, retError is camera has not been started or no image has been acquired by the method acquire.
    \sa DataObject, acquire
*/
ito::RetVal IDSuEye::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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
ito::RetVal IDSuEye::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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
ito::RetVal IDSuEye::retrieveData(ito::DataObject *externalDataObject)
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

    if (!m_seqAvailable)
    {
        retVal += ito::RetVal(ito::retError, 0, "no image or sequence has been acquired");
    }


    if (!retVal.containsError())
    {
        if (m_NumberOfBuffers == 1)
        {
            if (externalDataObject)
            {
                switch (m_data.getType())
                {
                case ito::tUInt8:
                    retVal += externalDataObject->copyFromData2D<ito::uint8>((ito::uint8*)m_data.rowPtr(0, 0), m_data.getSize(1), m_data.getSize(0)); //m_params["sizex"].getVal<int>(), m_params["sizey"].getVal<int>());
                    break;
                case ito::tUInt16:
                    retVal += externalDataObject->copyFromData2D<ito::uint16>((ito::uint16*)m_data.rowPtr(0, 0), m_data.getSize(1), m_data.getSize(0)); //m_params["sizex"].getVal<int>(), m_params["sizey"].getVal<int>());
                    break;
                case ito::tRGBA32:
                    retVal += externalDataObject->copyFromData2D<ito::Rgba32>((ito::Rgba32*)m_data.rowPtr(0, 0), m_data.getSize(1), m_data.getSize(0)); //m_params["sizex"].getVal<int>(), m_params["sizey"].getVal<int>());
                    break;
                default:
                    retVal += ito::RetVal(ito::retError, 0, tr("wrong picture type").toLatin1().data());
                    break;
                }
            }

            m_seqAvailable = false;
        }
        else
        {
            if (externalDataObject)
            {
                int planes = m_data.getNumPlanes();

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
                }
            }

            m_seqAvailable = false;
        }
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
void IDSuEye::dockWidgetVisibilityChanged(bool visible)
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
ito::RetVal IDSuEye::checkError(const int &code, const char* prefix /*= ""*/)
{
    if (code != IS_SUCCESS)
    {
        INT code_;
        IS_CHAR *msg_ = NULL;
        int err = is_GetError(m_camera, &code_, &msg_);

        switch (err)
        {
        case IS_SUCCESS:
            if (msg_)
            {
                if (prefix && prefix[0] != '\0')
                {
                    return ito::RetVal::format(ito::retError, code_, "%s: %s", prefix, msg_);
                }
                return ito::RetVal::format(ito::retError, code_, msg_);
            }
            else
            {
                if (prefix && prefix[0] != '\0')
                {
                    return ito::RetVal::format(ito::retError, code_, "%s: camera error with code %i (error message not resolvable)", prefix, code);
                }
                return ito::RetVal::format(ito::retError, code_, "camera error with code %i (error message not resolvable)", code);
            }
        case IS_INVALID_CAMERA_HANDLE:
            if (prefix && prefix[0] != '\0')
            {
                return ito::RetVal::format(ito::retError, code_, "%s: camera error with code %i (error message not resolvable due to invalid camera handle)", prefix, code);
            }
            return ito::RetVal::format(ito::retError, code, "camera error with code %i (error message not resolvable due to invalid camera handle)", code);
        case IS_INVALID_PARAMETER:
            if (prefix && prefix[0] != '\0')
            {
                return ito::RetVal::format(ito::retError, code_, "%s: camera error with code %i (error message not resolvable due to invalid parameter)", prefix, code);
            }
            return ito::RetVal::format(ito::retError, code, "camera error with code %i (error message not resolvable due to invalid parameter)", code);
        default:
            if (prefix && prefix[0] != '\0')
            {
                return ito::RetVal::format(ito::retError, code_, "%s: camera error with code %i (error message not resolvable)", prefix, code);
            }
            return ito::RetVal::format(ito::retError, code, "camera error with code %i (error message not resolvable)", code);
        }
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal IDSuEye::synchronizeCameraSettings(int what /*= sAll*/)
{
    UINT uintVal;
    UINT uintVal3[3];
    double dVal;
    double dVal3[3];

    ito::RetVal retval, rettemp;
    QMap<QString, ito::Param>::iterator it;

    if (what & sPixelClock)
    {
        //get pixelclock and ranges
        it = m_params.find("pixel_clock");

        //UINT nPixelClockList[150];
        //UINT nNumberOfSupportedPixelClocks = 0;
        //INT nRet = is_PixelClock(m_camera, IS_PIXELCLOCK_CMD_GET_NUMBER, (void*)&nNumberOfSupportedPixelClocks, sizeof(nNumberOfSupportedPixelClocks));

        //if ((nRet == IS_SUCCESS) && (nNumberOfSupportedPixelClocks > 0))
        //{
        //    is_PixelClock(m_camera, IS_PIXELCLOCK_CMD_GET_LIST, (void*)nPixelClockList, nNumberOfSupportedPixelClocks * sizeof(UINT));
        //}

        rettemp = checkError(is_PixelClock(m_camera, IS_PIXELCLOCK_CMD_GET_RANGE, (void*)uintVal3, sizeof(uintVal3)), "pixelClock");
        rettemp += checkError(is_PixelClock(m_camera, IS_PIXELCLOCK_CMD_GET, (void*)&uintVal, sizeof(uintVal)), "pixelClock");

        if (!rettemp.containsError())
        {
            it->setVal<int>(uintVal);
            // ck 13.07.15 otherwise we get a logic error from setMeta here
            if (uintVal3[2] < 1)
                uintVal3[2] = 1;
            it->setMeta(new ito::IntMeta(uintVal3[0], uintVal3[1], uintVal3[2]),true);
        }
        else
        {
            it->setFlags(ito::ParamBase::Readonly);
        }

        retval += rettemp;
    }

    if (what & sFrameTime)
    {
        double minTime, maxTime, incTime;
        ito::RetVal retVal = checkError(is_GetFrameTimeRange(m_camera, &minTime, &maxTime, &incTime));

        if (!retVal.containsError())
        {
            m_params["frame_rate"].setMeta(new ito::DoubleMeta(1.0/maxTime, 1.0/minTime), true);

            double framerate;
            retVal += checkError(is_SetFrameRate(m_camera, IS_GET_FRAMERATE, &framerate));

            if (!retVal.containsError())
            {
                m_params["frame_rate"].setVal<double>(framerate);
            }
        }
    }

    if (what & sExposure)
    {
        //get exposure time, exposure modes and ranges
        it = m_params.find("integration_time");
        rettemp = checkError(is_Exposure(m_camera, IS_EXPOSURE_CMD_GET_CAPS, (void*)&uintVal, sizeof(uintVal)), "Exposure");

        if (!rettemp.containsError())
        {
            QMap<QString, ito::Param>::iterator it2;

            //check standard exposure mode
            if (uintVal & IS_EXPOSURE_CAP_EXPOSURE)
            {
                it->setFlags(0);
                is_Exposure(m_camera, IS_EXPOSURE_CMD_GET_EXPOSURE, &dVal, sizeof(dVal));
                is_Exposure(m_camera, IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE, (void*)dVal3, sizeof(dVal3));
                it->setVal<double>(dVal * 1.0e-3);
                it->setMeta(new ito::DoubleMeta(dVal3[0] * 1.0e-3, dVal3[1] * 1.0e-3, dVal3[2] * 1.0e-3), true);
            }
            else
            {
                it->setFlags(ito::ParamBase::Readonly);
            }

            //check fine exposure and overwrite range and increment values of standard exposure if fine exposure is available
            if (uintVal & IS_EXPOSURE_CAP_FINE_INCREMENT)
            {
                is_Exposure(m_camera, IS_EXPOSURE_CMD_GET_FINE_INCREMENT_RANGE, (void*)dVal3, sizeof(dVal3));
                it->setMeta(new ito::DoubleMeta(dVal3[0] * 1.0e-3, dVal3[1] * 1.0e-3, dVal3[2] * 1.0e-3), true);
            }

            //check if long exposure time is available and if so verify if it is enabled. if so change the range of exposure time
            it2 = m_params.find("long_integration_time_enabled");

            if (uintVal & IS_EXPOSURE_CAP_LONG_EXPOSURE)
            {
                is_Exposure(m_camera, IS_EXPOSURE_CMD_GET_LONG_EXPOSURE_ENABLE, (void*)&uintVal, sizeof(uintVal));
                it2->setFlags(0);
                it2->setVal<int>(uintVal);

                if (uintVal > 0) //enabled, get range
                {
                    is_Exposure(m_camera, IS_EXPOSURE_CMD_GET_LONG_EXPOSURE_RANGE, (void*)dVal3, sizeof(dVal3));
                    it->setMeta(new ito::DoubleMeta(dVal3[0] * 1.0e-3, dVal3[1] * 1.0e-3, dVal3[2] * 1.0e-3), true);
                }
                else
                {
                    it2->setVal<int>(0);
                    it2->setMeta(new ito::IntMeta(0, 0), true);
                    it2->setFlags(ito::ParamBase::Readonly);
                }
            }
            else
            {
                it2->setVal<int>(0);
                it2->setMeta(new ito::IntMeta(0,0),true);
                it2->setFlags(ito::ParamBase::Readonly);
            }
        }
        else
        {
            it->setFlags(ito::ParamBase::Readonly);
        }
        retval += rettemp;
    }

    if (what & sBinning)
    {
        //get binning
        it = m_params.find("binning");
        int binVert = is_SetBinning(m_camera, IS_GET_BINNING_FACTOR_VERTICAL);
        int binHorz = is_SetBinning(m_camera, IS_GET_BINNING_FACTOR_HORIZONTAL);
        int ability = is_SetBinning(m_camera, IS_GET_SUPPORTED_BINNING);

        if (ability == 0) //no binning abilities
        {
            it->setFlags(ito::ParamBase::Readonly);
        }
        else
        {
            int maxHorz = (ability & IS_BINNING_16X_HORIZONTAL) ? 16 : ( \
                      (ability & IS_BINNING_8X_HORIZONTAL) ? 8 : ( \
                      (ability & IS_BINNING_6X_HORIZONTAL) ? 6 : ( \
                      (ability & IS_BINNING_5X_HORIZONTAL) ? 5 : ( \
                      (ability & IS_BINNING_4X_HORIZONTAL) ? 4 : ( \
                      (ability & IS_BINNING_3X_HORIZONTAL) ? 3 : ( \
                      (ability & IS_BINNING_2X_HORIZONTAL) ? 2 : 1))))));

            int maxVert = (ability & IS_BINNING_16X_VERTICAL) ? 16 : ( \
                      (ability & IS_BINNING_8X_VERTICAL) ? 8 : ( \
                      (ability & IS_BINNING_6X_VERTICAL) ? 6 : ( \
                      (ability & IS_BINNING_5X_VERTICAL) ? 5 : ( \
                      (ability & IS_BINNING_4X_VERTICAL) ? 4 : ( \
                      (ability & IS_BINNING_3X_VERTICAL) ? 3 : ( \
                      (ability & IS_BINNING_2X_VERTICAL) ? 2 : 1))))));

            it->setFlags(0);
            it->setMeta(new ito::IntMeta(101, maxHorz * 100 + maxVert), true);
        }

        it->setVal<int>(binHorz*100+binVert);
    }

    if (what & sRoi)
    {
        //get current roi and adjust min/max values of roi
        IS_POINT_2D offset, offsetMin, offsetMax, offsetInc;
        IS_POINT_2D size, sizeMin, sizeMax, sizeInc;
        rettemp = checkError(is_AOI(m_camera, IS_AOI_IMAGE_GET_POS, (void*)&offset, sizeof(offset)));
        rettemp = checkError(is_AOI(m_camera, IS_AOI_IMAGE_GET_SIZE, (void*)&size, sizeof(size)));
        rettemp = checkError(is_AOI(m_camera, IS_AOI_IMAGE_GET_POS_MIN, (void*)&offsetMin, sizeof(offsetMin)));
        rettemp = checkError(is_AOI(m_camera, IS_AOI_IMAGE_GET_POS_MAX, (void*)&offsetMax, sizeof(offsetMax)));
        rettemp = checkError(is_AOI(m_camera, IS_AOI_IMAGE_GET_POS_INC, (void*)&offsetInc, sizeof(offsetInc)));
        rettemp = checkError(is_AOI(m_camera, IS_AOI_IMAGE_GET_SIZE_MIN, (void*)&sizeMin, sizeof(sizeMin)));
        rettemp = checkError(is_AOI(m_camera, IS_AOI_IMAGE_GET_SIZE_MAX, (void*)&sizeMax, sizeof(sizeMax)));
        rettemp = checkError(is_AOI(m_camera, IS_AOI_IMAGE_GET_SIZE_INC, (void*)&sizeInc, sizeof(sizeInc)));

        if (!rettemp.containsError())
        {
            int currentX1 = size.s32X + offset.s32X - 1;
            int currentY1 = size.s32Y + offset.s32Y - 1;
            it = m_params.find("sizex");
            it->setVal<int>(size.s32X);
            it->setMeta(new ito::IntMeta(sizeMin.s32X, size.s32X + offsetMax.s32X, sizeInc.s32X), true);
            it = m_params.find("sizey");
            it->setVal<int>(size.s32Y);
            it->setMeta(new ito::IntMeta(sizeMin.s32Y, size.s32Y + offsetMax.s32Y, sizeInc.s32Y), true);
            it = m_params.find("x0");
            it->setVal<int>(offset.s32X);
            //x0 max is one smaller than the current x1 value, considering the increment value
            int x0max = currentX1 - (currentX1 - offsetMin.s32X) % offsetInc.s32X;
            it->setMeta(new ito::IntMeta(offsetMin.s32X, x0max, offsetInc.s32X), true);
            it->setFlags(0);
            it = m_params.find("y0");
            it->setVal<int>(offset.s32Y);
            //x0 max is one smaller than the current x1 value, considering the increment value
            int y0max = currentY1 - (currentY1 - offsetMin.s32Y) % offsetInc.s32Y;
            it->setMeta(new ito::IntMeta(offsetMin.s32Y, y0max, offsetInc.s32Y), true);
            it->setFlags(0);
            it = m_params.find("x1");
            it->setVal<int>(currentX1);
            it->setMeta(new ito::IntMeta(offset.s32X + sizeMin.s32X - 1, size.s32X + offsetMax.s32X - 1, sizeInc.s32X), true);
            it->setFlags(0);
            it = m_params.find("y1");
            it->setVal<int>(currentY1);
            it->setMeta(new ito::IntMeta(offset.s32Y + sizeMin.s32Y - 1, size.s32Y + offsetMax.s32Y - 1, sizeInc.s32Y), true);
            it->setFlags(0);

#if defined(ITOM_ADDININTERFACE_VERSION) && ITOM_ADDININTERFACE_VERSION > 0x010300
            it = m_params.find("roi");
            int *roi = it->getVal<int*>();
            roi[0] = offset.s32X;
            roi[1] = offset.s32Y;
            roi[2] = size.s32X;
            roi[3] = size.s32Y;
            ito::RangeMeta widthMeta(offsetMin.s32X, size.s32X + offsetMax.s32X - 1, offsetInc.s32X, sizeMin.s32X, size.s32X + offsetMax.s32X, sizeInc.s32X);
            ito::RangeMeta heightMeta(offsetMin.s32Y, size.s32Y + offsetMax.s32Y - 1, offsetInc.s32Y, sizeMin.s32Y, size.s32Y + offsetMax.s32Y, sizeInc.s32Y);
            it->setMeta(new ito::RectMeta(widthMeta, heightMeta), true);
#endif
        }
        else
        {
            m_params["x0"].setFlags(ito::ParamBase::Readonly);
            m_params["x1"].setFlags(ito::ParamBase::Readonly);
            m_params["y0"].setFlags(ito::ParamBase::Readonly);
            m_params["y1"].setFlags(ito::ParamBase::Readonly);
#if defined(ITOM_ADDININTERFACE_VERSION) && ITOM_ADDININTERFACE_VERSION > 0x010300
            m_params["roi"].setFlags(ito::ParamBase::Readonly);
#endif
        }
    }

    if (what & sGain)
    {
        //get pixelclock and ranges
        it = m_params.find("gain");
        it->setVal<double>((double)is_SetHardwareGain(m_camera, IS_GET_MASTER_GAIN, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER) / 100.0);

        if (m_sensorInfo.nColorMode & IS_COLORMODE_MONOCHROME)
        {
            it->setFlags(0);
        }
        else
        {
            it->setFlags(ito::ParamBase::Readonly);
        }

        it = m_params.find("gain_rgb");
        double rgbGain[] = {0.0,0.0,0.0};
        rgbGain[0] = is_SetHardwareGain(m_camera, IS_GET_RED_GAIN, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER) / 100.0;
        rgbGain[1] = is_SetHardwareGain(m_camera, IS_GET_GREEN_GAIN, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER) / 100.0;
        rgbGain[2] = is_SetHardwareGain(m_camera, IS_GET_BLUE_GAIN, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER) / 100.0;
        it->setVal<double*>(rgbGain, 3);

        if (m_sensorInfo.nColorMode & IS_COLORMODE_BAYER)
        {
            it->setFlags(0);
        }
        else
        {
            it->setFlags(ito::ParamBase::Readonly);
        }

        it = m_params.find("gain_boost_enabled");

        if (is_SetGainBoost(m_camera, IS_GET_SUPPORTED_GAINBOOST) == IS_SET_GAINBOOST_ON)
        {
            it->setFlags(0);
            int activated = is_SetGainBoost(m_camera, IS_GET_GAINBOOST);
            it->setVal<int>( activated == IS_SET_GAINBOOST_ON ? 1 : 0 );
        }
        else
        {
            it->setFlags(ito::ParamBase::Readonly);
            it->setVal<int>(0);
        }
    }

    if (what & sOffset)
    {
        //offset is represented by the manual blacklevel correction that is added to each value. If this feature cannot be changed the
        //parameter is set to readonly
        it = m_params.find("offset");
        UINT capabilities, val2;
        ito::RetVal retTemp = checkError(is_Blacklevel(m_camera, IS_BLACKLEVEL_CMD_GET_CAPS, (void*)&capabilities, sizeof(capabilities)));

        if (!retTemp.containsError())
        {
            it->setFlags((capabilities & IS_BLACKLEVEL_CAP_SET_OFFSET) ? 0 : ito::ParamBase::Readonly);

            retTemp += checkError(is_Blacklevel(m_camera, IS_BLACKLEVEL_CMD_GET_OFFSET_RANGE, (void*)&m_blacklevelRange, sizeof(m_blacklevelRange)));
            retTemp += checkError(is_Blacklevel(m_camera, IS_BLACKLEVEL_CMD_GET_OFFSET, (void*)&val2, sizeof(val2)));

            if (!retTemp.containsError())
            {
                it->setVal<double>((double)(val2 - m_blacklevelRange.s32Min) / ((double)(m_blacklevelRange.s32Max -m_blacklevelRange.s32Min)));
                it->setMeta(new ito::DoubleMeta(0.0,1.0,0.0), true);
            }
            else
            {
                it->setVal<int>(0.0);
                it->setMeta(new ito::DoubleMeta(0.0,1.0,0.0), true);
            }

            it = m_params.find("auto_blacklevel_enabled");

            if (capabilities & IS_BLACKLEVEL_CAP_SET_AUTO_BLACKLEVEL)
            {
                it->setMeta(new ito::IntMeta(0,1), true);
                it->setFlags(0);
                retTemp += checkError(is_Blacklevel(m_camera, IS_BLACKLEVEL_CMD_GET_MODE, (void*)&val2, sizeof(val2)));

                if (!retTemp.containsError())
                {
                    it->setVal<int>( val2 == IS_AUTO_BLACKLEVEL_ON ? 1 : 0 );
                }
            }
            else
            {
                it->setFlags(ito::ParamBase::Readonly);
                retTemp += checkError(is_Blacklevel(m_camera, IS_BLACKLEVEL_CMD_GET_MODE, (void*)&val2, sizeof(val2)));

                if (!retTemp.containsError())
                {
                    int v = val2 == IS_AUTO_BLACKLEVEL_ON ? 1 : 0;
                    it->setVal<int>( v );
                    it->setMeta(new ito::IntMeta(v,v), true);
                }
            }
        }
    }

    if (what & sTriggerMode)
    {
        it = m_params.find("trigger_mode");
        INT supportedModes = is_SetExternalTrigger(m_camera, IS_GET_SUPPORTED_TRIGGER_MODE);
        ito::StringMeta *m = new ito::StringMeta(ito::StringMeta::String);
        m->addItem("off");

        if (supportedModes & IS_SET_TRIGGER_SOFTWARE) m->addItem("software");
        if (supportedModes & IS_SET_TRIGGER_HI_LO) m->addItem("hi_lo");
        if (supportedModes & IS_SET_TRIGGER_LO_HI) m->addItem("lo_hi");
        if (supportedModes & IS_SET_TRIGGER_PRE_HI_LO) m->addItem("pre_hi_lo");
        if (supportedModes & IS_SET_TRIGGER_PRE_LO_HI) m->addItem("pre_lo_hi");

        it->setMeta(m, true);

        INT trigger = is_SetExternalTrigger(m_camera, IS_GET_EXTERNALTRIGGER);

        switch (trigger)
        {
        case IS_SET_TRIGGER_OFF:
            it->setVal<const char*>("off");
            break;
        case IS_SET_TRIGGER_SOFTWARE:
            it->setVal<const char*>("software");
            break;
        case IS_SET_TRIGGER_HI_LO:
            it->setVal<const char*>("hi_lo");
            break;
        case IS_SET_TRIGGER_LO_HI:
            it->setVal<const char*>("lo_hi");
            break;
        case IS_SET_TRIGGER_PRE_HI_LO:
            it->setVal<const char*>("pre_hi_lo");
            break;
        case IS_SET_TRIGGER_PRE_LO_HI:
            it->setVal<const char*>("pre_lo_hi");
            break;
        default:
            //trigger is in a non-supported mode, set it to software trigger
            retval += checkError(is_SetExternalTrigger(m_camera, IS_SET_TRIGGER_SOFTWARE));
            it->setVal<const char*>("software");
        }

    }

    if (what & sBppAndColorMode)
    {
        it = m_params.find("bpp");

        if (m_sensorInfo.nColorMode == IS_COLORMODE_MONOCHROME)
        {
            //check if bitdepth different than 8bit is applicable, else only 8bit is possible
            UINT bitDepths = 0;

            if (IS_NOT_SUPPORTED == is_DeviceFeature(m_camera, IS_DEVICE_FEATURE_CMD_GET_SUPPORTED_SENSOR_BIT_DEPTHS, (void*)&bitDepths, sizeof(bitDepths)))
            {
                it->setMeta(new ito::IntMeta(m_monochromeBitDepthRange.s32X,m_monochromeBitDepthRange.s32Y), true);

                int colorMode = is_SetColorMode(m_camera, IS_GET_COLOR_MODE);

                if (colorMode == IS_CM_MONO8)
                {
                    it->setVal<int>(8);
                    m_bitspixel = 8;
                }
                else if (colorMode == IS_CM_MONO10)
                {
                    it->setVal<int>(10);
                    m_bitspixel = 16;
                }
                else if (colorMode == IS_CM_MONO12)
                {
                    it->setVal<int>(12);
                    m_bitspixel = 16;
                }
                else if (colorMode == IS_CM_MONO16)
                {
                    it->setVal<int>(16);
                    m_bitspixel = 16;
                }
            }
            else
            {
                //is_DeviceFeature(m_camera, IS_DEVICE_FEATURE_CMD_GET_SENSOR_BIT_DEPTH, (void*)&uintVal, sizeof(uintVal));
                int min = 8;
                int max = 8;
                int step = 4;
                int current = 8;
                int colorMode = is_SetColorMode(m_camera, IS_GET_COLOR_MODE);

                if (bitDepths & IS_SENSOR_BIT_DEPTH_10_BIT)
                {
                    max = 10;
                    step = 2;
                }

                if (bitDepths & IS_SENSOR_BIT_DEPTH_12_BIT)
                {
                    max = 12;
                }

                if (colorMode == IS_CM_MONO8)
                {
                    current = 8;
                }
                else if (colorMode == IS_CM_MONO10)
                {
                    current=10;
                }
                else if (colorMode == IS_CM_MONO12)
                {
                    current=12;
                }
                else if (colorMode == IS_CM_MONO16)
                {
                    current=16;
                }

                it->setMeta(new ito::IntMeta(min,max,step), true);
                m_bitspixel = (current == 8) ? 8 : 16;
                it->setVal<int>(current);
            }

            m_colouredOutput = false;
            m_params["color_mode"].setVal<const char*>("gray");
            m_params["color_mode"].setMeta(new ito::StringMeta(ito::StringMeta::String, "gray"), true);
        }
        else //color
        {
            it->setMeta(new ito::IntMeta(8,8), true);
            it->setVal<int>(8); //color cameras always have 8 bit

            int colorMode = is_SetColorMode(m_camera, IS_GET_COLOR_MODE);

            if (colorMode == IS_CM_MONO8)
            {
                m_params["color_mode"].setVal<const char*>("gray");
                m_bitspixel = 8;
                m_colouredOutput = false;
            }
            else if (colorMode == IS_CM_BGRA8_PACKED)
            {
                m_params["color_mode"].setVal<const char*>("color");
                m_bitspixel = 32;
                m_colouredOutput = true;
            }
            else
            {
                retval += ito::RetVal(ito::retError, 0, "color mode of camera is not supported");
                m_bitspixel = 8;
            }

            ito::StringMeta *sm = new ito::StringMeta(ito::StringMeta::String, "gray");
            sm->addItem("color");
            m_params["color_mode"].setMeta(sm, true);
        }

        retval += rettemp;
    }

    return retval;
}

//---------------------------------------------------------------------------------
ito::RetVal IDSuEye::loadSensorInfo()
{
    ito::RetVal retVal = checkError(is_GetSensorInfo(m_camera, &m_sensorInfo));

    if (!retVal.containsError())
    {
        m_params["cam_model"].setVal<const char*>(m_sensorInfo.strSensorName);
        QMap<QString, ito::Param>::iterator it = m_params.find("sensor_type");

        switch (m_sensorInfo.SensorID)
        {
            case IS_SENSOR_UI141X_M: it->setVal<const char*>("UI141X_M, VGA rolling shutter, monochrome"); break;
            case IS_SENSOR_UI141X_C: it->setVal<const char*>("UI141X_C, VGA rolling shutter, color"); break;
            case IS_SENSOR_UI144X_M: it->setVal<const char*>("UI144X_M, SXGA rolling shutter, monochrome"); break;
            case IS_SENSOR_UI144X_C: it->setVal<const char*>("UI144X_C, SXGA rolling shutter, SXGA color"); break;

            case IS_SENSOR_UI154X_M: it->setVal<const char*>("UI154X_M, SXGA rolling shutter, monochrome"); break;
            case IS_SENSOR_UI154X_C: it->setVal<const char*>("UI154X_C, SXGA rolling shutter, color"); break;
            case IS_SENSOR_UI145X_C: it->setVal<const char*>("UI145X_C, UXGA rolling shutter, color"); break;

            case IS_SENSOR_UI146X_C: it->setVal<const char*>("UI146X_C, QXGA rolling shutter, color"); break;
            case IS_SENSOR_UI148X_M: it->setVal<const char*>("UI148X_M, 5MP rolling shutter, monochrome"); break;
            case IS_SENSOR_UI148X_C: it->setVal<const char*>("UI148X_C, 5MP rolling shutter, color"); break;

            case IS_SENSOR_UI121X_M: it->setVal<const char*>("UI121X_M, VGA global shutter, monochrome"); break;
            case IS_SENSOR_UI121X_C: it->setVal<const char*>("UI121X_C, VGA global shutter, VGA color"); break;
            case IS_SENSOR_UI122X_M: it->setVal<const char*>("UI122X_M, WVGA global shutter, monochrome"); break;
            case IS_SENSOR_UI122X_C: it->setVal<const char*>("UI122X_C, WVGA global shutter, color"); break;

            case IS_SENSOR_UI164X_C: it->setVal<const char*>("UI164X_C, SXGA rolling shutter, color"); break;

            case IS_SENSOR_UI155X_C: it->setVal<const char*>("UI155X_C, UXGA rolling shutter, color"); break;

            case IS_SENSOR_UI1223_M: it->setVal<const char*>("UI1223_M, WVGA global shutter, monochrome"); break;
            case IS_SENSOR_UI1223_C: it->setVal<const char*>("UI1223_C, WVGA global shutter, color"); break;

            case IS_SENSOR_UI149X_M: it->setVal<const char*>("UI149X_M, 10MP rolling shutter, monochrome"); break;
            case IS_SENSOR_UI149X_C: it->setVal<const char*>("UI149X_C, 10MP rolling shutter, color"); break;

            case IS_SENSOR_UI1225_M: it->setVal<const char*>("UI1225_M, WVGA global shutter, monochrome, LE model"); break;
            case IS_SENSOR_UI1225_C: it->setVal<const char*>("UI1225_C, WVGA global shutter, color, LE model"); break;

            case IS_SENSOR_UI1645_C: it->setVal<const char*>("UI1645_C, SXGA rolling shutter, color, LE model"); break;
            case IS_SENSOR_UI1555_C: it->setVal<const char*>("UI1555_C, UXGA rolling shutter, color, LE model"); break;
            case IS_SENSOR_UI1545_M: it->setVal<const char*>("UI1545_M, SXGA rolling shutter, monochrome, LE model"); break;
            case IS_SENSOR_UI1545_C: it->setVal<const char*>("UI1545_C, SXGA rolling shutter, color, LE model"); break;
            case IS_SENSOR_UI1455_C: it->setVal<const char*>("UI1455_C, UXGA rolling shutter, color, LE model"); break;
            case IS_SENSOR_UI1465_C: it->setVal<const char*>("UI1465_C, QXGA rolling shutter, color, LE model"); break;
            case IS_SENSOR_UI1485_M: it->setVal<const char*>("UI1485_M, 5MP rolling shutter, monochrome, LE model"); break;
            case IS_SENSOR_UI1485_C: it->setVal<const char*>("UI1485_C, 5MP rolling shutter, color, LE model"); break;
            case IS_SENSOR_UI1495_M: it->setVal<const char*>("UI1495_M, 10MP rolling shutter, monochrome, LE model"); break;
            case IS_SENSOR_UI1495_C: it->setVal<const char*>("UI1495_C, 10MP rolling shutter, color, LE model"); break;

            case IS_SENSOR_UI112X_M: it->setVal<const char*>("UI112X_M, 0768x576, HDR sensor, monochrome"); break;
            case IS_SENSOR_UI112X_C: it->setVal<const char*>("UI112X_C, 0768x576, HDR sensor, color"); break;

            case IS_SENSOR_UI1008_M: it->setVal<const char*>("UI1008_M"); break;
            case IS_SENSOR_UI1008_C: it->setVal<const char*>("UI1008_C"); break;

#if defined IS_SENSOR_UIF005_M //SDK < 4.81
            case IS_SENSOR_UIF005_M: it->setVal<const char*>("UIF005_M"); break;
            case IS_SENSOR_UIF005_C: it->setVal<const char*>("UIF005_C"); break;
#endif

            case IS_SENSOR_UI1005_M: it->setVal<const char*>("UI1005_M"); break;
            case IS_SENSOR_UI1005_C: it->setVal<const char*>("UI1005_C"); break;

#if defined IS_SENSOR_UI1007_M //SDK >= 4.95
            case IS_SENSOR_UI1007_M: it->setVal<const char*>("5MP rolling shutter, color"); break;
            case IS_SENSOR_UI1007_C: it->setVal<const char*>("5MP rolling shutter, color"); break;
#endif

            case IS_SENSOR_UI1240_M: it->setVal<const char*>("UI1240_M, SXGA global shutter, monochrome"); break;
            case IS_SENSOR_UI1240_C: it->setVal<const char*>("UI1240_C, SXGA global shutter, color"); break;
            case IS_SENSOR_UI1240_NIR: it->setVal<const char*>("UI1240_NIR, SXGA global shutter, NIR"); break;

            case IS_SENSOR_UI1240LE_M: it->setVal<const char*>("UI1240LE_M, SXGA global shutter, monochrome, single board"); break;
            case IS_SENSOR_UI1240LE_C: it->setVal<const char*>("UI1240LE_C, SXGA global shutter, color, single board"); break;
            case IS_SENSOR_UI1240LE_NIR: it->setVal<const char*>("UI1240LE_NIR, SXGA global shutter, NIR, single board"); break;

            case IS_SENSOR_UI1240ML_M: it->setVal<const char*>("UI1240ML_M, SXGA global shutter, monochrome, single board"); break;
            case IS_SENSOR_UI1240ML_C: it->setVal<const char*>("UI1240ML_C, SXGA global shutter, color, single board"); break;
            case IS_SENSOR_UI1240ML_NIR: it->setVal<const char*>("UI1240ML_NIR, SXGA global shutter, NIR, single board"); break;

            case IS_SENSOR_UI1243_M_SMI: it->setVal<const char*>("UI1243_M_SMI"); break;
            case IS_SENSOR_UI1243_C_SMI: it->setVal<const char*>("UI1243_C_SMI"); break;

            case IS_SENSOR_UI1543_M: it->setVal<const char*>("UI1543_M, SXGA rolling shutter, monochrome, single board"); break;
            case IS_SENSOR_UI1543_C: it->setVal<const char*>("UI1543_C, SXGA rolling shutter, color, single board"); break;

            case IS_SENSOR_UI1544_M: it->setVal<const char*>("UI1544_M, SXGA rolling shutter, monochrome, single board"); break;
            case IS_SENSOR_UI1544_C: it->setVal<const char*>("UI1544_C, SXGA rolling shutter, color, single board"); break;
            case IS_SENSOR_UI1543_M_WO: it->setVal<const char*>("UI1543_M_WO, SXGA rolling shutter, monochrome, single board"); break;
            case IS_SENSOR_UI1543_C_WO: it->setVal<const char*>("UI1543_C_WO, SXGA rolling shutter, color, single board"); break;
            case IS_SENSOR_UI1453_C: it->setVal<const char*>("UI1453_C, UXGA rolling shutter, color, single board"); break;
            case IS_SENSOR_UI1463_C: it->setVal<const char*>("UI1463_C, QXGA rolling shutter, color, single board"); break;
            case IS_SENSOR_UI1483_M: it->setVal<const char*>("UI1483_M, QSXG rolling shutter, monochrome, single board"); break;
            case IS_SENSOR_UI1483_C: it->setVal<const char*>("UI1483_C, QSXG rolling shutter, color, single board"); break;
            case IS_SENSOR_UI1493_M: it->setVal<const char*>("UI1493_M, 10Mp rolling shutter, monochrome, single board"); break;
            case IS_SENSOR_UI1493_C: it->setVal<const char*>("UI1493_C, 10MP rolling shutter, color, single board"); break;

            case IS_SENSOR_UI1463_M_WO: it->setVal<const char*>("UI1463_M_WO, QXGA rolling shutter, monochrome, single board"); break;
            case IS_SENSOR_UI1463_C_WO: it->setVal<const char*>("UI1463_C_WO, QXGA rolling shutter, color, single board"); break;

            case IS_SENSOR_UI1553_C_WN: it->setVal<const char*>("UI1553_C_WN, UXGA rolling shutter, color, single board"); break;
            case IS_SENSOR_UI1483_M_WO: it->setVal<const char*>("UI1483_M_WO, QSXGA rolling shutter, monochrome, single board"); break;
            case IS_SENSOR_UI1483_C_WO: it->setVal<const char*>("UI1483_C_WO, QSXGA rolling shutter, color, single board"); break;

            case IS_SENSOR_UI1580_M: it->setVal<const char*>("UI1580_M, 5MP rolling shutter, monochrome"); break;
            case IS_SENSOR_UI1580_C: it->setVal<const char*>("UI1580_C, 5MP rolling shutter, color"); break;
            case IS_SENSOR_UI1580LE_M: it->setVal<const char*>("UI1580LE_M, 5MP rolling shutter, monochrome, single board"); break;
            case IS_SENSOR_UI1580LE_C: it->setVal<const char*>("UI1580LE_C, 5MP rolling shutter, color, single board"); break;

            case IS_SENSOR_UI1360M: it->setVal<const char*>("UI1360M, 2.2MP global shutter, monochrome"); break;
            case IS_SENSOR_UI1360C: it->setVal<const char*>("UI1360C, 2.2MP global shutter, color"); break;
            case IS_SENSOR_UI1360NIR: it->setVal<const char*>("UI1360NIR, 2.2MP global shutter, NIR"); break;

            case IS_SENSOR_UI1370M: it->setVal<const char*>("UI1370M, 4.2MP global shutter, monochrome"); break;
            case IS_SENSOR_UI1370C: it->setVal<const char*>("UI1370C, 4.2MP global shutter, color"); break;
            case IS_SENSOR_UI1370NIR: it->setVal<const char*>("UI1370NIR, 4.2MP global shutter, NIR"); break;

            case IS_SENSOR_UI1250_M: it->setVal<const char*>("UI1250_M, 2MP global shutter, monochrome"); break;
            case IS_SENSOR_UI1250_C: it->setVal<const char*>("UI1250_C, 2MP global shutter, color"); break;
            case IS_SENSOR_UI1250_NIR: it->setVal<const char*>("UI1250_NIR, 2MP global shutter, NIR"); break;

            case IS_SENSOR_UI1250LE_M: it->setVal<const char*>("UI1250LE_M, 2MP global shutter, monochrome, single board"); break;
            case IS_SENSOR_UI1250LE_C: it->setVal<const char*>("UI1250LE_C, 2MP global shutter, color, single board"); break;
            case IS_SENSOR_UI1250LE_NIR: it->setVal<const char*>("UI1250LE_NIR, 2MP global shutter, NIR, single board"); break;

            case IS_SENSOR_UI1250ML_M: it->setVal<const char*>("UI1250ML_M, 2MP global shutter, monochrome, single board"); break;
            case IS_SENSOR_UI1250ML_C: it->setVal<const char*>("UI1250ML_C, 2MP global shutter, color, single board"); break;
            case IS_SENSOR_UI1250ML_NIR: it->setVal<const char*>(", 2MP global shutter, NIR, single board"); break;

            case IS_SENSOR_UI1493_M_AR: it->setVal<const char*>("UI1493_M_AR"); break;
            case IS_SENSOR_UI1493_C_AR: it->setVal<const char*>("UI1493_C_AR"); break;

#if defined IS_SENSOR_UI1060_M //>= SDK 4.61
            case IS_SENSOR_UI1060_M: it->setVal<const char*>("UI1060_M, 2.3MP global shutter, monochrome"); break;
            case IS_SENSOR_UI1060_C: it->setVal<const char*>("UI1060_C, 2.3MP global shutter, color"); break;
            case IS_SENSOR_UI1013XC: it->setVal<const char*>("UI1013XC, 13MP, color"); break;
#endif

#if defined IS_SENSOR_UI1140M  //>= SDK 4.80
            case IS_SENSOR_UI1140M: it->setVal<const char*>("UI1140M, 1.3MP global shutter, monochrome"); break;
            case IS_SENSOR_UI1140C: it->setVal<const char*>("UI1140C, 1.3MP global shutter, color"); break;
            case IS_SENSOR_UI1140NIR: it->setVal<const char*>("UI1140NIR, 1.3MP global shutter, NIR"); break;

            case IS_SENSOR_UI1590M: it->setVal<const char*>("UI1590M, 18MP rolling shutter, monochrome"); break;
            case IS_SENSOR_UI1590C: it->setVal<const char*>("UI1590C, 18MP rolling shutter, color"); break;

            case IS_SENSOR_UI1260_M: it->setVal<const char*>("UI1260_M, 2.3MP global shutter, monochrome"); break;
            case IS_SENSOR_UI1260_C: it->setVal<const char*>("UI1260_C, 2.3MP global shutter, color"); break;

            case IS_SENSOR_UI1130_M: it->setVal<const char*>("UI1130_M, SVGA global shutter, monochrome"); break;
            case IS_SENSOR_UI1130_C: it->setVal<const char*>("UI1130_C, SVGA global shutter, color"); break;

            case IS_SENSOR_UI1160_M: it->setVal<const char*>("UI1160_M, 2.3MP global shutter, monochrome"); break;
            case IS_SENSOR_UI1160_C: it->setVal<const char*>("UI1160_C, 2.3MP global shutter, color"); break;

            case IS_SENSOR_UI1180_M: it->setVal<const char*>("UI1180_M, 5.3MP global shutter, monochrome"); break;
            case IS_SENSOR_UI1180_C: it->setVal<const char*>("UI1180_C, 5.3MP global shutter, color"); break;

            case IS_SENSOR_UI1080_M: it->setVal<const char*>("UI1080_M, 5MP global shutter, monochrome"); break;
            case IS_SENSOR_UI1080_C: it->setVal<const char*>("UI1080_C, 5MP global shutter, color"); break;
#endif

#if defined IS_SENSOR_UI1280_M //>= SDK 4.81
            case IS_SENSOR_UI1280_M: it->setVal<const char*>("UI1280_M, 5MP global shutter, monochrome"); break;
            case IS_SENSOR_UI1280_C: it->setVal<const char*>("UI1280_C, 5MP global shutter, color"); break;

            case IS_SENSOR_UI1860_M: it->setVal<const char*>("UI1860_M, 2MP rolling shutter, monochrome"); break;
            case IS_SENSOR_UI1860_C: it->setVal<const char*>("UI1860_C, 2MP rolling shutter, color"); break;

            case IS_SENSOR_UI1880_M: it->setVal<const char*>("UI1880_M, 6MP rolling shutter, monochrome"); break;
            case IS_SENSOR_UI1880_C: it->setVal<const char*>("UI1880_C, 6MP rolling shutter, color"); break;

            case IS_SENSOR_UI1270_M: it->setVal<const char*>("UI1270_M, 3.2MP global shutter, monochrome"); break;
            case IS_SENSOR_UI1270_C: it->setVal<const char*>("UI1270_C, 3.2MP global shutter, color"); break;

            case IS_SENSOR_UI1070_M: it->setVal<const char*>("UI1070_M, 3.2MP global shutter, monochrome"); break;
            case IS_SENSOR_UI1070_C: it->setVal<const char*>("UI1070_C, 3.2MP global shutter, color"); break;

            case IS_SENSOR_UI1130LE_M: it->setVal<const char*>("UI1130LE_M, SVGA global shutter, monochrome"); break;
            case IS_SENSOR_UI1130LE_C: it->setVal<const char*>("UI1130LE_C, SVGA global shutter, color"); break;
#endif

#if defined IS_SENSOR_UI1290_M //>= SDK 4.95
            case IS_SENSOR_UI1290_M: it->setVal<const char*>("8.9MP global shutter, monochrome"); break;
            case IS_SENSOR_UI1290_C: it->setVal<const char*>("8.9MP global shutter, color"); break;

            case IS_SENSOR_UI1090_M: it->setVal<const char*>("8.9MP global shutter, monochrome"); break;
            case IS_SENSOR_UI1090_C: it->setVal<const char*>("8.9MP global shutter, color"); break;

            case IS_SENSOR_UI1000_M: it->setVal<const char*>("12.3MP global shutter, monochrome"); break;
            case IS_SENSOR_UI1000_C: it->setVal<const char*>("12.3MP global shutter, color"); break;

            case IS_SENSOR_UI1200_M: it->setVal<const char*>("12.3MP global shutter, monochrome"); break;
            case IS_SENSOR_UI1200_C: it->setVal<const char*>("12.3MP global shutter, color"); break;

            case IS_SENSOR_UI1040_M: it->setVal<const char*>("1.6MP global shutter, monochrome"); break;
            case IS_SENSOR_UI1040_C: it->setVal<const char*>("1.6MP global shutter, color"); break;

            case IS_SENSOR_UI1180_M_S2: it->setVal<const char*>("5.3MP global shutter, monochrome"); break;
            case IS_SENSOR_UI1180_C_S2: it->setVal<const char*>("5.3MP global shutter, color"); break;

            case IS_SENSOR_UI1160_M_S2: it->setVal<const char*>("2.3MP global shutter, monochrome"); break;
            case IS_SENSOR_UI1160_C_S2: it->setVal<const char*>("2.3MP global shutter, color"); break;

            case IS_SENSOR_UI1020_M: it->setVal<const char*>("0.4MP global shutter, monochrome"); break;
            case IS_SENSOR_UI1020_C: it->setVal<const char*>("0.4MP global shutter, color"); break;
#endif

            case IS_SENSOR_UI223X_M: it->setVal<const char*>("UI223X_M, Sony CCD sensor - XGA monochrome"); break;
            case IS_SENSOR_UI223X_C: it->setVal<const char*>("UI223X_C, Sony CCD sensor - XGA color"); break;
            case IS_SENSOR_UI241X_M: it->setVal<const char*>("UI241X_M, Sony CCD sensor - VGA monochrome"); break;
            case IS_SENSOR_UI241X_C: it->setVal<const char*>("UI241X_C, Sony CCD sensor - VGA color"); break;
            case IS_SENSOR_UI234X_M: it->setVal<const char*>("UI234X_M, Sony CCD sensor - SXGA monochrome"); break;
            case IS_SENSOR_UI234X_C: it->setVal<const char*>("UI234X_C, Sony CCD sensor - SXGA color"); break;
            case IS_SENSOR_UI221X_M: it->setVal<const char*>("UI221X_M, Sony CCD sensor - VGA monochrome"); break;
            case IS_SENSOR_UI221X_C: it->setVal<const char*>("UI221X_C, Sony CCD sensor - VGA color"); break;
            case IS_SENSOR_UI231X_M: it->setVal<const char*>("UI231X_M, Sony CCD sensor - VGA monochrome"); break;
            case IS_SENSOR_UI231X_C: it->setVal<const char*>("UI231X_C, Sony CCD sensor - VGA color"); break;
            case IS_SENSOR_UI222X_M: it->setVal<const char*>("UI222X_M, Sony CCD sensor - CCIR / PAL monochrome"); break;
            case IS_SENSOR_UI222X_C: it->setVal<const char*>("UI222X_C, Sony CCD sensor - CCIR / PAL color"); break;
            case IS_SENSOR_UI224X_M: it->setVal<const char*>("UI224X_M, Sony CCD sensor - SXGA monochrome"); break;
            case IS_SENSOR_UI224X_C: it->setVal<const char*>("UI224X_C, Sony CCD sensor - SXGA color"); break;
            case IS_SENSOR_UI225X_M: it->setVal<const char*>("UI225X_M, Sony CCD sensor - UXGA monochrome"); break;
            case IS_SENSOR_UI225X_C: it->setVal<const char*>("UI225X_C, Sony CCD sensor - UXGA color"); break;
            case IS_SENSOR_UI214X_M: it->setVal<const char*>("UI214X_M, Sony CCD sensor - SXGA monochrome"); break;
            case IS_SENSOR_UI214X_C: it->setVal<const char*>("UI214X_C, Sony CCD sensor - SXGA color"); break;
            case IS_SENSOR_UI228X_M: it->setVal<const char*>("UI228X_M, Sony CCD sensor - QXGA monochrome"); break;
            case IS_SENSOR_UI228X_C: it->setVal<const char*>("UI228X_C, Sony CCD sensor - QXGA color"); break;
            case IS_SENSOR_UI251X_M: it->setVal<const char*>("UI251X_M or UI241X_M_R2, Sony CCD sensor - VGA monochrome"); break;
            case IS_SENSOR_UI251X_C: it->setVal<const char*>("UI251X_C or UI241X_C_R2, Sony CCD sensor - VGA color"); break;
            case IS_SENSOR_UI2130_M: it->setVal<const char*>("UI2130_M, Sony CCD sensor - WXGA monochrome"); break;
            case IS_SENSOR_UI2130_C: it->setVal<const char*>("UI2130_C, Sony CCD sensor - WXGA color"); break;

            default: it->setVal<const char*>("unknown sensor type"); break;
        }
    }

    return retVal;
}


//-------------------------------------------------------------------------------------------------
ito::RetVal IDSuEye::checkData(ito::DataObject *externalDataObject)
{
    int futureHeight = m_params["sizey"].getVal<int>();
    int futureWidth = m_params["sizex"].getVal<int>();
    int futureType;
    bool allocMemNew = false;
    ito::RetVal retval;

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
        if (m_NumberOfBuffers == 1)
        {
            if (m_data.getDims() < 2 || m_data.getSize(0) != (unsigned int)futureHeight || m_data.getSize(1) != (unsigned int)futureWidth || m_data.getType() != futureType)
            {
                m_data = ito::DataObject(futureHeight, futureWidth, futureType);
            }
        }
        else
        {
            if (m_data.getDims() != 3 || m_data.getSize(0) != m_NumberOfBuffers || m_data.getSize(1) != (unsigned int)futureHeight || m_data.getSize(2) != (unsigned int)futureWidth || m_data.getType() != futureType)
            {
                m_data = ito::DataObject(m_NumberOfBuffers, futureHeight, futureWidth, futureType);
            }
        }

    }
    else
    {
        int dims = externalDataObject->getDims();
        if (m_NumberOfBuffers == 1)
        {
            if (externalDataObject->getDims() == 0)
            {
                *externalDataObject = ito::DataObject(futureHeight, futureWidth, futureType);
            }
            else if (externalDataObject->calcNumMats() != 1)
            {
                return ito::RetVal(ito::retError, 0, tr("Error during check data, external dataObject invalid. Object has more than 1 plane or zero planes. It must be of right size and type or an uninitialized image.").toLatin1().data());
            }
            else if (externalDataObject->getSize(dims - 2) != (unsigned int)futureHeight || externalDataObject->getSize(dims - 1) != (unsigned int)futureWidth || externalDataObject->getType() != futureType)
            {
                return ito::RetVal(ito::retError, 0, tr("Error during check data, external dataObject invalid. Object must be of right size and type or a uninitialized image.").toLatin1().data());
            }
        }
        else
        {
            if (externalDataObject->getDims() == 0)
            {
                *externalDataObject = ito::DataObject(m_NumberOfBuffers, futureHeight, futureWidth, futureType);
            }
            else if (externalDataObject->calcNumMats() != m_NumberOfBuffers)
            {
                return ito::RetVal(ito::retError, 0, tr("Error during check data, external dataObject invalid. Frame burst is %1, the external object must then have %1 planes.").arg(m_NumberOfBuffers).toLatin1().data());
            }
            else if (externalDataObject->getSize(dims - 2) != (unsigned int)futureHeight || externalDataObject->getSize(dims - 1) != (unsigned int)futureWidth || externalDataObject->getType() != futureType)
            {
                return ito::RetVal(ito::retError, 0, tr("Error during check data, external dataObject invalid. Object must be of right size and type or a uninitilized image.").toLatin1().data());
            }

        }
    }

    //now check if camera memory needs to be (re)allocated

    if (m_vpcSeqImgMem.empty())
    {
        allocMemNew = true;
    }
    else
    {
        int pnX, pnY, pnBits, pnPitch;
        retval += checkError(is_InquireImageMem(m_camera, m_vpcSeqImgMem.at(0), m_viSeqMemId.at(0), &pnX, &pnY, &pnBits, &pnPitch));

        if (futureWidth != pnX || futureHeight != pnY || m_bitspixel != pnBits || m_NumberOfBuffers != m_oldNumBuf)
        {
            for (int i = (m_oldNumBuf - 1); i >= 0; i--)
            {
                // free buffers
                if (is_FreeImageMem(m_camera, m_vpcSeqImgMem.at(i), m_viSeqMemId.at(i)) != IS_SUCCESS)
                {
                    break;
                }
            }

            m_viSeqMemId.clear();
            m_vpcSeqImgMem.clear();

            allocMemNew = true;
        }
    }

    if (allocMemNew)
    {
        is_ClearSequence(m_camera);
        for (int i = 0; i < m_NumberOfBuffers; i++)
        {
            INT iImgMemID = 0;
            char* pcImgMem = 0;


            retval += checkError(is_AllocImageMem(m_camera, futureWidth, futureHeight, m_bitspixel, &pcImgMem, &iImgMemID));

            if (retval.containsError())
            {
                return retval;
            }
            retval += checkError(is_AddToSequence(m_camera, pcImgMem, iImgMemID));
            if (retval.containsError())
            {
                is_FreeImageMem(m_camera, pcImgMem, iImgMemID);
                return retval;
            }
            m_viSeqMemId.push_back(iImgMemID);
            m_vpcSeqImgMem.push_back(pcImgMem);
        }

        m_oldNumBuf = m_NumberOfBuffers;

    }


    return retval;
}


//----------------------------------------------------------------------------------------
ito::RetVal IDSuEye::setMeanFrameRate()
{
    //get frametime range and set the fps to the mean value in order to achieve a huge exposure time range (frame rate is not important in non-free-run mode)
    double minTime, maxTime, incTime, newFps;
    ito::RetVal retVal = checkError(is_GetFrameTimeRange(m_camera, &minTime, &maxTime, &incTime));
    if (!retVal.containsError())
    {
        retVal += checkError(is_SetFrameRate(m_camera, 0.5 * (1.0 / maxTime + 1.0 / minTime), &newFps));
        m_params["frame_rate"].setVal<double>(newFps);
    }
    return retVal;
}

//----------------------------------------------------------------------------------------
ito::RetVal IDSuEye::setFrameRate(ito::float64 framerate)
{
    double minTime, maxTime, incTime;
    ito::RetVal retVal = checkError(is_GetFrameTimeRange(m_camera, &minTime, &maxTime, &incTime));
    if (!retVal.containsError())
    {
        if (framerate > 1.0 / minTime)
        {
            framerate = 1.0 / minTime;
            retVal += ito::RetVal(ito::retWarning, 0, tr("Warning framerate is out of bounds, set to closest value possible").toLatin1().data());
        }
        else if (framerate < 1.0 / maxTime)
        {
            framerate = 1.0 / maxTime;
            retVal += ito::RetVal(ito::retWarning, 0, tr("Warning framerate is out of bounds, set to closest value possible").toLatin1().data());
        }

        retVal += checkError(is_SetFrameRate(m_camera, framerate, &framerate));
        m_params["frame_rate"].setVal<double>(framerate);
    }
    return retVal;
}

//----------------------------------------------------------------------------------------
const ito::RetVal IDSuEye::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogIDS(this));
}

//----------------------------------------------------------------------------------------

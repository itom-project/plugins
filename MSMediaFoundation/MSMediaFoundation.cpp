/* ********************************************************************
    Plugin "MSMediaFoundation" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut für Technische Optik (ITO),
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

#include "MSMediaFoundation.h"
#include "pluginVersion.h"
#include "gitVersion.h"
#include "opencv2/imgproc/imgproc.hpp"

#include <strmif.h>
#include <wchar.h>

#define _USE_MATH_DEFINES  // needs to be defined to enable standard declarations of PI constant

#include <math.h>
#include <float.h>
#include <qstring.h>
#include <qstringlist.h>
#include <qplugin.h>
#include <qmessagebox.h>

#include "dockWidgetMSMediaFoundation.h"

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MSMediaFoundationInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(MSMediaFoundation)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MSMediaFoundationInterface::closeThisInst(ito::AddInBase **addInInst)
{
   REMOVE_PLUGININSTANCE(MSMediaFoundation)
   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
MSMediaFoundationInterface::MSMediaFoundationInterface()
{
    m_type = ito::typeDataIO | ito::typeGrabber;
    setObjectName("MSMediaFoundation");

    m_description = QObject::tr("MSMediaFoundation");

    m_detaildescription = QObject::tr(
"This plugin uses the Microsoft Media Foundation framework (Windows Vista, 7, 8) for capturing supported camera devices (e.g. ordinary USB or integrated cameras). \n\
Cameras must provide the UVC 1.1 interface for USB devices. \n\
\n\
This driver detects an internal list of connected cameras. The parameter *cameraNumber* indicates the device to open (until now, there is no mechanism to open the next \n\
not yet opened device!). The camera can either be used as colored camera, as gray valued camera or it is also possible to only select one color channel that is mapped \n\
to the gray output. \n\
\n\
Any detected and supported device can offer multiple framerates and sizes. Use the parameter *mediaTypeID* to select the right value. Open your device with *mediaTypeID* = -1 \n\
to let the plugin print a list of supported formats (the plugin initialization then stops with a desired error). \n\
\n\
Build requirements \n\
------------------- \n\
For building this plugin the Windows SDK needs to be installed and Windows Vista or higher is required. \n\
\n\
Affiliation \n\
------------ \n\
This plugin internally uses a modified version of VideoInput, proposed by Evgeny Pereguda and published under \n\
http://www.codeproject.com/Articles/559437/Capturing-video-from-web-camera-on-Windows-7-and-8 (Code Project Open License)");

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
    m_aboutThis = QObject::tr(GITVERSION);

    ito::Param paramVal = ito::Param("cameraNumber", ito::ParamBase::Int, 0, 255, 0, tr("consecutive number of the connected camera (starting with 0, default)").toLatin1().data());
    m_initParamsOpt.append(paramVal);

    paramVal = ito::Param("colorMode", ito::ParamBase::String, "auto", tr("color mode of camera (auto|color|red|green|blue|gray, default: auto -> color or gray)").toLatin1().data());
    ito::StringMeta meta(ito::StringMeta::String);
    meta.addItem("auto");
    meta.addItem("color");
    meta.addItem("red");
    meta.addItem("green");
    meta.addItem("blue");
    meta.addItem("gray");
    paramVal.setMeta(&meta, false);
    m_initParamsOpt.append(paramVal);

    this->m_callInitInNewThread = false;

    paramVal = ito::Param("mediaTypeID", ito::ParamBase::Int, -1, 1000, 0, tr("ID of the media format. 0 (default) takes the first from the list (must be MFVideoFormat_RGBA24 as subtype). -1: prints out a list of devices and quits the initialization, other: other index from the list of available types").toLatin1().data());
    m_initParamsOpt.append(paramVal);

    paramVal = ito::Param("flipImage", ito::ParamBase::Int, 0, 1, 0, tr("if 1 image is vertically flipped (default: 0)").toLatin1().data());
    m_initParamsOpt.append(paramVal);

    //paramVal = ito::Param("Init-Dialog", ito::ParamBase::Int, 0, 1, 0, tr("If true, a camera selection dialog is opened during startup").toLatin1().data());
    //m_initParamsOpt.append(paramVal);

   return;
}

//----------------------------------------------------------------------------------------------------------------------------------
MSMediaFoundationInterface::~MSMediaFoundationInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------------------
void StopEvent(int deviceID, void *userData)
{
    QSharedPointer<VideoInput> vi = MSMediaFoundation::VideoInputInstance;

    if (!vi.isNull())
    {
        vi->closeDevice(deviceID);
    }
}


/*static*/ QMutex MSMediaFoundation::VideoInputCreateMutex;
/*static*/ int MSMediaFoundation::VideoInputRefCount = 0;
/*static*/ QSharedPointer<VideoInput> MSMediaFoundation::VideoInputInstance;

//----------------------------------------------------------------------------------------------------------------------------------
//! shows the configuration dialog. This method must be executed in the main (GUI) thread and is usually called by the addIn-Manager.
/*!
    creates new instance of dialogDummyGrabber, calls the method setVals of DialogMSMediaFoundation, starts the execution loop and if the dialog
    is closed, reads the new parameter set and deletes the dialog.

    \return retOk
    \sa DialogMSMediaFoundation
*/
const ito::RetVal MSMediaFoundation::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogMSMediaFoundation(this));
}

//----------------------------------------------------------------------------------------------------------------------------------
MSMediaFoundation::MSMediaFoundation() : AddInGrabber(), m_isgrabbing(false), m_deviceID(0), m_camStatusChecked(false), m_initState(initNotTested)
{
    VideoInputCreateMutex.lock();

    if (VideoInputRefCount == 0)
    {
        QSharedPointer<DebugPrintOut> dpo(new DebugPrintOut());
        VideoInputInstance = QSharedPointer<VideoInput>(new VideoInput(dpo));
    }

    VideoInputRefCount++;
    m_videoInput = VideoInputInstance;

    VideoInputCreateMutex.unlock();

    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "MSMediaFoundation", "name of the plugin");
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("deviceName", ito::ParamBase::String | ito::ParamBase::Readonly, "", "name of device");
    m_params.insert(paramVal.getName(), paramVal);

#if defined(ITOM_ADDININTERFACE_VERSION) && ITOM_ADDININTERFACE_VERSION > 0x010300
    int roi[] = {0, 0, 4048, 4048};
    paramVal = ito::Param("roi", ito::ParamBase::IntArray, 4, roi, tr("ROI (x,y,width,height) [this replaces the values x0,x1,y0,y1]").toLatin1().data());
    ito::RectMeta *rm = new ito::RectMeta(ito::RangeMeta(0, 4047), ito::RangeMeta(0, 4047));
    paramVal.setMeta(rm, true);
    m_params.insert(paramVal.getName(), paramVal);
#endif

    paramVal = ito::Param("x0", ito::ParamBase::Int | ito::ParamBase::In, 0, 4047, 0, tr("first pixel index in ROI (x-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("y0", ito::ParamBase::Int | ito::ParamBase::In, 0, 4047, 0, tr("first pixel index in ROI (y-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("x1", ito::ParamBase::Int | ito::ParamBase::In, 0, 4047, 4047, tr("last pixel index in ROI (x-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("y1", ito::ParamBase::Int | ito::ParamBase::In, 0, 4047, 4047, tr("last pixel index in ROI (y-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizex", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::In, 1, 4048, 4048, tr("width of ROI (x-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizey", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::In, 1, 4048, 4048, tr("height of ROI (y-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("bpp", ito::ParamBase::Int | ito::ParamBase::In, 8, 8, 8, tr("bpp").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("integrationTime", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 0.01, tr("Integrationtime of CCD in seconds").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("integrationTimeAuto", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("auto-controlled integration time of CCD (on:1, off:0)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("brightness", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 1.0, tr("brightness [0..1]").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("contrast", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 1.0, tr("contrast [0..1]").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("saturation", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 1.0, tr("saturation [0..1]").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sharpness", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 1.0, tr("sharpness [0..1]").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("hue", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 0.0, tr("hue [0..1]").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("gain", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 0.0, tr("gain [0..1]").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("focus", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 1.0, tr("focus [0..1]").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("gamma", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 1.0, tr("gamma [0..1]").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("iris", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 1.0, tr("iris [0..1]").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("zoom", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 0.0, tr("zoom [0..1]").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("backlightCompensation", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 0.0, tr("backlightCompensation [0..1]").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("whiteBalance", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 0.0, tr("whiteBalance [0..1]").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("gamma", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 0.0, tr("gamma [0..1]").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("brightnessAuto", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 1, tr("auto-controlled brightness (on:1, off:0)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("contrastAuto", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 1, tr("auto-controlled contrast (on:1, off:0)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("saturationAuto", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 1, tr("auto-controlled saturation (on:1, off:0)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sharpnessAuto", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 1, tr("auto-controlled sharpness (on:1, off:0)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("hueAuto", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 1, tr("auto-controlled hue (on:1, off:0)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("gainAuto", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 1, tr("auto-controlled gain (on:1, off:0)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("focusAuto", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 1, tr("auto-controlled focus (on:1, off:0)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("gammaAuto", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 1, tr("auto-controlled gamma (on:1, off:0)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("irisAuto", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 1, tr("auto-controlled iris (on:1, off:0)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("zoomAuto", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("auto-controlled zoom (on:1, off:0)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("backlightCompensationAuto", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("auto-controlled backlightCompensation (on:1, off:0)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("whiteBalanceAuto", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("auto-controlled whiteBalance (on:1, off:0)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("gammaAuto", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("auto-controlled gamma (on:1, off:0)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    /*paramVal = ito::Param("channel", ito::ParamBase::Int, 0, 3, 0, tr("selected color channel (all available (0, default), R (1), G (2), B (3)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("colorConversion", ito::ParamBase::Int, 0, 1, 1, tr("no conversion (0), RGB->Grayscale (1, default). If the camera image only has one channel or channel>0, this parameter is ignored").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);*/

    paramVal = ito::Param("colorMode", ito::ParamBase::String, "auto", tr("color mode of camera (auto|color|red|green|blue|gray, default: auto -> color or gray)").toLatin1().data());
    ito::StringMeta meta(ito::StringMeta::String);
    meta.addItem("auto");
    meta.addItem("color");
    meta.addItem("red");
    meta.addItem("green");
    meta.addItem("blue");
    meta.addItem("gray");
    paramVal.setMeta(&meta, false);
    m_params.insert(paramVal.getName(), paramVal);

    DockWidgetMSMediaFoundation *dw = new DockWidgetMSMediaFoundation(this);

    Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
    QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
    createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dw);
}

//----------------------------------------------------------------------------------------------------------------------------------
MSMediaFoundation::~MSMediaFoundation()
{
   m_params.clear();

   VideoInputCreateMutex.lock();

   VideoInputRefCount--;
   m_videoInput.clear();

   if (VideoInputRefCount == 0)
   {
       VideoInputInstance.clear();
   }

   VideoInputCreateMutex.unlock();
}

//----------------------------------------------------------------------------------------------------------------------------------
// Function to set and update data information
ito::RetVal MSMediaFoundation::checkCameraAbilities()
{
    bool camRetVal = false;
    m_camStatusChecked = false;
    ito::RetVal retValue;

    //acquire test image in order to get knownledge about camera's abilities
    //camRetVal = m_pCam->grab();
    //camRetVal = m_pCam->retrieve(m_pDataMatBuffer);

    m_imgChannels = 3;
    m_imgCols = m_videoInput->getWidth(m_deviceID);
    m_imgRows = m_videoInput->getHeight(m_deviceID);
    m_imgBpp = 8;

    static_cast<ito::IntMeta*>(m_params["sizex"].getMeta())->setMax(m_imgCols);
    static_cast<ito::IntMeta*>(m_params["sizey"].getMeta())->setMax(m_imgRows);
    m_params["sizex"].setVal<int>(m_imgCols);
    m_params["sizey"].setVal<int>(m_imgRows);

    static_cast<ito::IntMeta*>(m_params["x0"].getMeta())->setMax(m_imgCols-1);
    static_cast<ito::IntMeta*>(m_params["y0"].getMeta())->setMax(m_imgRows-1);
    m_params["x0"].setVal<int>(0);
    m_params["y0"].setVal<int>(0);

    static_cast<ito::IntMeta*>(m_params["x1"].getMeta())->setMax(m_imgCols-1);
    static_cast<ito::IntMeta*>(m_params["y1"].getMeta())->setMax(m_imgRows-1);
    m_params["x1"].setVal<int>(m_imgCols-1);
    m_params["y1"].setVal<int>(m_imgRows-1);

#if defined(ITOM_ADDININTERFACE_VERSION) && ITOM_ADDININTERFACE_VERSION > 0x010300
    int roi[] = {0, 0, m_imgCols, m_imgRows};
    m_params["roi"].setVal<int*>(roi, 4);
    ito::RectMeta *rm = new ito::RectMeta(ito::RangeMeta(0, m_imgCols - 1/*, 1, 1, m_imgCols*/), ito::RangeMeta(0, m_imgRows - 1/*, 1, 1, m_imgRows*/));
    m_params["roi"].setMeta(rm, true);
#endif

    //m_params["bpp"].setMin(8);
    //m_params["bpp"].setMax(elemSize1*8);
    m_params["bpp"].setMeta(new ito::IntMeta(8, m_imgBpp), true);
    m_params["bpp"].setVal<int>(m_imgBpp);

    if (m_imgBpp < 8 || m_imgBpp > 32)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("unknown bpp").toLatin1().data());
    }

    m_camStatusChecked = true;

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
ito::RetVal MSMediaFoundation::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;
    QString key;
    bool hasIndex = false;
    int index;
    QString suffix;
    QMap<QString,ito::Param>::iterator it;

    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (retValue == ito::retOk)
    {
        retValue += apiGetParamFromMapByKey(m_params, key, it, false);
    }

    if (!retValue.containsError())
    {
        if (m_camParamsHash.contains(key))
        {
            synchronizeCameraParametersToParams(false);
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

    \param [in] val  is a input of type::tparam containing name, value and further information
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk in case that everything is ok, else retError
    \sa ito::tParam, ItomSharedSemaphore
*/
ito::RetVal MSMediaFoundation::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    QMap<QString,ito::Param>::iterator it;
    QString key;
    bool hasIndex;
    int index;
    QString suffix;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    ito::RetVal retValue = apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (!retValue.containsError())
    {
        //gets the parameter key from m_params map (read-only is not allowed and leads to ito::retError).
        retValue += apiGetParamFromMapByKey(m_params, key, it, true);
    }

    if (!retValue.containsError())
    {
        //here the parameter val is checked if it can be casted to *it and if the
        //possible meta information requirements of *it are met.
        retValue += apiValidateAndCastParam(*it, *val, false, true, true);
    }

    if (!retValue.containsError())
    {
        if (m_camParamsHash.contains(key))
        {
            if (key.endsWith("Auto"))
            {
                QString keyWithoutAuto = key.left(key.size() - 4);
                retValue += updateCamParam(*(m_camParamsHash[key]), m_params[keyWithoutAuto], *val);
            }
            else
            {
                retValue += updateCamParam(*(m_camParamsHash[key]), *val, m_params[key + "Auto"]);
            }

            if (!retValue.containsError() && m_videoInput)
            {
                m_videoInput->setParameters(m_deviceID, m_camParams);
            }
        }

        if (key == "colorMode")
        {
            const char *mode = val->getVal<char*>();

            if (m_imgChannels == 1)
            {
                if (QString::compare(mode,"auto") != 0 || QString::compare(mode, "gray") != 0)
                {
                    retValue += ito::RetVal(ito::retError, 0, "The connected grayscale camera cannot be operated in any colored colorMode");
                }
            }
            else if (m_imgChannels == 3 && m_imgBpp > 8)
            {
                retValue += ito::RetVal(ito::retError, 0, "The connected color camera cannot output an color image since the bit depth is > 8");
            }

            if (!retValue.containsError())
            {
                switch(mode[0])
                {
                case 'a':
                    m_colorMode = modeAuto;
                    break;
                case 'c':
                    m_colorMode = modeColor;
                    break;
                case 'g':
                    if (mode[2] == 'a')
                        m_colorMode = modeGray;
                    else
                        m_colorMode = modeGreen;
                    break;
                case 'r':
                    m_colorMode = modeRed;
                    break;
                case 'b':
                    m_colorMode = modeBlue;
                    break;
                }
            }
        }

        if (!retValue.containsError())
        {
            //here you can add specific sub-checks for every keyword and finally put the value into (*it).
            retValue += it->copyValueFrom(&(*val));
        }

        if (key == "roi")
        {
            const int* roi = val->getVal<int*>();
            m_params["x0"].setVal<int>(roi[0]);
            m_params["y0"].setVal<int>(roi[1]);
            m_params["x1"].setVal<int>(roi[0] + roi[2] - 1);
            m_params["y1"].setVal<int>(roi[1] + roi[3] - 1);
            m_params["sizex"].setVal<int>(roi[2]);
            m_params["sizey"].setVal<int>(roi[3]);
        }

        if (key == "x0" || key == "x1")
        {
            m_params["sizex"].setVal<int>(1 + m_params["x1"].getVal<int>() - m_params["x0"].getVal<int>());
#if defined(ITOM_ADDININTERFACE_VERSION) && ITOM_ADDININTERFACE_VERSION > 0x010300
            int *roi = m_params["roi"].getVal<int*>();
            roi[0] = m_params["x0"].getVal<int>();
            roi[2] = 1 + m_params["x1"].getVal<int>() - roi[0];
            m_params["roi"].setVal<int*>(roi, 4);
#endif
        }
        else if (key == "y0" || key == "y1")
        {
            m_params["sizey"].setVal<int>(1 + m_params["y1"].getVal<int>() - m_params["y0"].getVal<int>());
#if defined(ITOM_ADDININTERFACE_VERSION) && ITOM_ADDININTERFACE_VERSION > 0x010300
            int *roi = m_params["roi"].getVal<int*>();
            roi[1] = m_params["y0"].getVal<int>();
            roi[3] = 1 + m_params["y1"].getVal<int>() - roi[1];
            m_params["roi"].setVal<int*>(roi, 4);
#endif
        }
    }

    if (!retValue.containsError())
    {
        //retValue += checkCameraAbilities();
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
ito::RetVal MSMediaFoundation::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    m_deviceID = paramsOpt->at(0).getVal<int>();
    int mediaTypeID = paramsOpt->at(2).getVal<int>();
    m_flipImage = paramsOpt->at(3).getVal<int>() > 0 ? true : false;
    QSharedPointer<ito::ParamBase> colorMode(new ito::ParamBase(paramsOpt->at(1)));

    m_videoInput->setVerbose(false);

    int numDevices = m_videoInput->listDevices();

    if (m_deviceID >= numDevices)
    {
        retValue += ito::RetVal::format(ito::retError, 0, tr("invalid cameraNumber. Only %i cameras found").toLatin1().data(), numDevices);
    }
    else
    {
        QString deviceName = QString::fromUtf16((const ushort*)m_videoInput->getNameVideoDevice(m_deviceID));

        if (deviceName == "Empty")
        {
            retValue += ito::RetVal(ito::retError, 0, tr("desired device does not exist").toLatin1().data());
        }
        else
        {
            m_params["deviceName"].setVal<char*>(deviceName.toLatin1().data());
            setIdentifier(deviceName);
            MediaType mediaType;

            if (mediaTypeID == -1)
            {
                std::cout << "Media types for selected camera '" << deviceName.toLatin1().data() << "'\n---------------------------------------------------------------\n" << std::endl;
            }

            int j = 0;
            int foundID = -1;

            for (unsigned int i = 0; i < m_videoInput->getCountFormats(m_deviceID); ++i)
            {
                mediaType = m_videoInput->getFormat(m_deviceID, i);

                if (wcscmp(mediaType.pMF_MT_MAJOR_TYPEName, L"MFMediaType_Video") == 0)
                {
                    if (wcscmp(mediaType.pMF_MT_SUBTYPEName, L"MFVideoFormat_RGB24") == 0)
                    {
                        if (mediaTypeID == -1)
                        {
                            std::cout << "ID " << j << ": " << mediaType.height << " x " << mediaType.width << " px, " << mediaType.MF_MT_FRAME_RATE << " fps (RGB24, Video)\n" << std::endl;
                        }
                        else if (mediaTypeID == j)
                        {
                            foundID = i;
                            break;
                        }
                        j++;
                    }
                    else if (wcscmp(mediaType.pMF_MT_SUBTYPEName, L"MFVideoFormat_YUY2") == 0)
                    {
                        if (mediaTypeID == -1)
                        {
                            std::cout << "ID " << j << ": " << mediaType.height << " x " << mediaType.width << " px, " << mediaType.MF_MT_FRAME_RATE << " fps (YUY2, Video)\n" << std::endl;
                        }
                        else if (mediaTypeID == j)
                        {
                            foundID = i;
                            break;
                        }
                        j++;
                    }
                }
            }

            if (mediaTypeID == -1)
            {
                retValue += ito::RetVal(ito::retError, 0, tr("Camera initialization aborted since only list of media types requested").toLatin1().data());
            }
            else if (foundID == -1)
            {
                retValue += ito::RetVal(ito::retError, 0, tr("Number of available media types was smaller than the given mediaTypeID. Try mediaTypeID = -1 to print a list of available types.").toLatin1().data());
            }
            else if (!m_videoInput->setupDevice(m_deviceID, foundID))
            {
                retValue += ito::RetVal::format(ito::retError, 0, tr("Camera (%i) could not be opened").toLatin1().data(), m_deviceID);
            }
            else
            {
                if (m_videoInput->isFrameNew(m_deviceID))
                {
                    m_videoInput->setEmergencyStopEvent(m_deviceID, NULL, StopEvent);
                }
                else
                {
                    retValue += ito::RetVal::format(ito::retError, 0, tr("No frame could be acquired from device %i").toLatin1().data(), numDevices);
                }
            }
        }
    }

    if (!retValue.containsError())
    {
        retValue += synchronizeCameraParametersToParams(true);
    }

    if (!retValue.containsError())
    {
        retValue += checkCameraAbilities();
        retValue += setParam(colorMode,NULL);
    }

    if (!retValue.containsError())
    {
        retValue += checkData();

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
ito::RetVal MSMediaFoundation::checkInitState()
{
    if (m_initState == initNotTested)
    {
        m_initState = initNotSuccessfull;

        //acquire test image to wait for the MediaFoundation threads to be initialized and the first image callback arrived
        int loopy = 3000/50; //max 3 seconds
        while (loopy > 0)
        {
            if (m_videoInput->isFrameNew(m_deviceID))
            {
                m_initState = initSuccessfull;
                break;
            }
            else
            {
                Sleep(50);
                loopy--;
            }
        }

        if (loopy <= 0)
        {
            return ito::RetVal(ito::retWarning, 0, "A first test image could not be acquired (timeout after 3 sec)");
        }
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MSMediaFoundation::synchronizeParam(const Parameter &parameter, ito::Param &paramDbl, ito::Param &paramAutoInt)
{
    ito::DoubleMeta *dm = (ito::DoubleMeta*)(paramDbl.getMeta());

    if (parameter.Available && (parameter.Max > parameter.Min))
    {
        if (paramDbl.getName()[0] != 'i' && paramDbl.getName()[1] != 'n') //not: IntegrationTime
        {
            dm->setStepSize(static_cast<double>(parameter.Step) / static_cast<double>(parameter.Max - parameter.Min));
            paramDbl.setVal<double>(static_cast<double>(parameter.CurrentValue - parameter.Min) / static_cast<double>(parameter.Max - parameter.Min));
            paramAutoInt.setVal<int>(parameter.Flag == VideoProcAmp_Flags_Auto ? 1 : 0);
        }
        else
        {
            dm->setStepSize(0.0);
            paramDbl.setVal<double>(std::pow(2.0, parameter.CurrentValue));
            paramDbl.setMeta(new ito::DoubleMeta(std::pow(2.0, parameter.Min), std::pow(2.0, parameter.Max), 0.0), true);
            paramAutoInt.setVal<int>(parameter.Flag == VideoProcAmp_Flags_Auto ? 1 : 0);
        }
    }
    else
    {
        return ito::RetVal(ito::retError, 0, tr("Parameter not available or useless range").toLatin1().data());
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MSMediaFoundation::updateCamParam(Parameter &parameter, const ito::ParamBase &paramDbl, const ito::ParamBase &paramAutoInt)
{
    if (paramAutoInt.getVal<int>() > 0)
    {
        parameter.Flag = VideoProcAmp_Flags_Auto;
    }
    else
    {
        parameter.Flag = VideoProcAmp_Flags_Manual;
    }

    if (paramDbl.getName()[0] != 'i' && paramDbl.getName()[1] != 'n') //not: IntegrationTime
    {
        parameter.CurrentValue = parameter.Min + (long)(paramDbl.getVal<double>() * (parameter.Max - parameter.Min));
    }
    else
    {
        parameter.CurrentValue = qBound(parameter.Min, (long)(qRound(log10(paramDbl.getVal<double>())/log10(2.0))), parameter.Max);
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MSMediaFoundation::synchronizeCameraParametersToParams(bool firstCall /*= false*/)
{
    if (m_videoInput)
    {
        m_camParams = m_videoInput->getParameters(m_deviceID);

        if (m_camParams.Brightness.Available && m_camParams.Brightness.Max > m_camParams.Brightness.Min)
        {
            m_camParamsHash["brightness"] = &m_camParams.Brightness;
            m_camParamsHash["brightnessAuto"] = &m_camParams.Brightness;
            synchronizeParam(m_camParams.Brightness, m_params["brightness"], m_params["brightnessAuto"]);
        }
        else if (firstCall)
        {
            m_params.remove("brightness");
            m_params.remove("brightnessAuto");
        }

        if (m_camParams.Contrast.Available && m_camParams.Contrast.Max > m_camParams.Contrast.Min)
        {
            m_camParamsHash["contrast"] = &m_camParams.Brightness;
            m_camParamsHash["contrastAuto"] = &m_camParams.Brightness;
            synchronizeParam(m_camParams.Contrast, m_params["contrast"], m_params["contrastAuto"]);
        }
        else if (firstCall)
        {
            m_params.remove("contrast");
            m_params.remove("contrastAuto");
        }

        if (m_camParams.Hue.Available && m_camParams.Hue.Max > m_camParams.Hue.Min)
        {
            m_camParamsHash["hue"] = &m_camParams.Hue;
            m_camParamsHash["hueAuto"] = &m_camParams.Hue;
            synchronizeParam(m_camParams.Hue, m_params["hue"], m_params["hueAuto"]);
        }
        else if (firstCall)
        {
            m_params.remove("hue");
            m_params.remove("hueAuto");
        }

        if (m_camParams.Saturation.Available && m_camParams.Saturation.Max > m_camParams.Saturation.Min)
        {
            m_camParamsHash["saturation"] = &m_camParams.Saturation;
            m_camParamsHash["saturationAuto"] = &m_camParams.Saturation;
            synchronizeParam(m_camParams.Saturation, m_params["saturation"], m_params["saturationAuto"]);
        }
        else if (firstCall)
        {
            m_params.remove("saturation");
            m_params.remove("saturationAuto");
        }

        if (m_camParams.Sharpness.Available && m_camParams.Sharpness.Max > m_camParams.Sharpness.Min)
        {
            m_camParamsHash["sharpness"] = &m_camParams.Sharpness;
            m_camParamsHash["sharpnessAuto"] = &m_camParams.Sharpness;
            synchronizeParam(m_camParams.Sharpness, m_params["sharpness"], m_params["sharpnessAuto"]);
        }
        else if (firstCall)
        {
            m_params.remove("sharpness");
            m_params.remove("sharpnessAuto");
        }

        if (m_camParams.Gain.Available && m_camParams.Gain.Max > m_camParams.Gain.Min)
        {
            m_camParamsHash["gain"] = &m_camParams.Gain;
            m_camParamsHash["gainAuto"] = &m_camParams.Gain;
            synchronizeParam(m_camParams.Gain, m_params["gain"], m_params["gainAuto"]);
        }
        else if (firstCall)
        {
            m_params.remove("gain");
            m_params.remove("gainAuto");
        }

        if (m_camParams.Exposure.Available && m_camParams.Exposure.Max > m_camParams.Exposure.Min)
        {
            m_camParamsHash["integrationTime"] = &m_camParams.Exposure;
            m_camParamsHash["integrationTimeAuto"] = &m_camParams.Exposure;
            synchronizeParam(m_camParams.Exposure, m_params["integrationTime"], m_params["integrationTimeAuto"]);
        }
        else if (firstCall)
        {
            m_params.remove("integrationTime");
            m_params.remove("integrationTimeAuto");
        }

        if (m_camParams.Focus.Available && m_camParams.Focus.Max > m_camParams.Focus.Min)
        {
            m_camParamsHash["focus"] = &m_camParams.Focus;
            m_camParamsHash["focusAuto"] = &m_camParams.Focus;
            synchronizeParam(m_camParams.Focus, m_params["focus"], m_params["focusAuto"]);
        }
        else if (firstCall)
        {
            m_params.remove("focus");
            m_params.remove("focusAuto");
        }

        if (m_camParams.Gamma.Available && m_camParams.Gamma.Max > m_camParams.Gamma.Min)
        {
            m_camParamsHash["gamma"] = &m_camParams.Gamma;
            m_camParamsHash["gammaAuto"] = &m_camParams.Gamma;
            synchronizeParam(m_camParams.Gamma, m_params["gamma"], m_params["gammaAuto"]);
        }
        else if (firstCall)
        {
            m_params.remove("gamma");
            m_params.remove("gammaAuto");
        }

        if (m_camParams.Iris.Available && m_camParams.Iris.Max > m_camParams.Iris.Min)
        {
            m_camParamsHash["iris"] = &m_camParams.Iris;
            m_camParamsHash["irisAuto"] = &m_camParams.Iris;
            synchronizeParam(m_camParams.Iris, m_params["iris"], m_params["irisAuto"]);
        }
        else if (firstCall)
        {
            m_params.remove("iris");
            m_params.remove("irisAuto");
        }

        if (m_camParams.Zoom.Available && m_camParams.Zoom.Max > m_camParams.Zoom.Min)
        {
            m_camParamsHash["zoom"] = &m_camParams.Zoom;
            m_camParamsHash["zoomAuto"] = &m_camParams.Zoom;
            synchronizeParam(m_camParams.Zoom, m_params["zoom"], m_params["zoomAuto"]);
        }
        else if (firstCall)
        {
            m_params.remove("zoom");
            m_params.remove("zoomAuto");
        }

        if (m_camParams.BacklightCompensation.Available && m_camParams.BacklightCompensation.Max > m_camParams.BacklightCompensation.Min)
        {
            m_camParamsHash["backlightCompensation"] = &m_camParams.BacklightCompensation;
            m_camParamsHash["backlightCompensationAuto"] = &m_camParams.BacklightCompensation;
            synchronizeParam(m_camParams.BacklightCompensation, m_params["backlightCompensation"], m_params["backlightCompensationAuto"]);
        }
        else if (firstCall)
        {
            m_params.remove("backlightCompensation");
            m_params.remove("backlightCompensationAuto");
        }

        if (m_camParams.WhiteBalance.Available && m_camParams.WhiteBalance.Max > m_camParams.WhiteBalance.Min)
        {
            m_camParamsHash["whiteBalance"] = &m_camParams.WhiteBalance;
            m_camParamsHash["whiteBalanceAuto"] = &m_camParams.WhiteBalance;
            synchronizeParam(m_camParams.WhiteBalance, m_params["whiteBalance"], m_params["whiteBalanceAuto"]);
        }
        else if (firstCall)
        {
            m_params.remove("whiteBalance");
            m_params.remove("whiteBalanceAuto");
        }

        if (m_camParams.Gamma.Available && m_camParams.Gamma.Max > m_camParams.Gamma.Min)
        {
            m_camParamsHash["gamma"] = &m_camParams.Gamma;
            m_camParamsHash["gammaAuto"] = &m_camParams.Gamma;
            synchronizeParam(m_camParams.Gamma, m_params["gamma"], m_params["gammaAuto"]);
        }
        else if (firstCall)
        {
            m_params.remove("gamma");
            m_params.remove("gammaAuto");
        }
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MSMediaFoundation::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if (m_timerID > 0)
    {
        killTimer(m_timerID);
        m_timerID=0;
    }

    retValue += stopDevice(NULL);

    m_videoInput->closeDevice(m_deviceID);

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MSMediaFoundation::startDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = checkInitState();

    incGrabberStarted();

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MSMediaFoundation::stopDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = checkInitState();

    decGrabberStarted();

    if (grabberStartedCount() < 0)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("the grabber already had zero users.").toLatin1().data());
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
ito::RetVal MSMediaFoundation::acquire(const int trigger, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = checkInitState();
    bool RetCode = false;

    if (grabberStartedCount() <= 0)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Tried to acquire without starting device").toLatin1().data());
    }
    else if (m_camStatusChecked == false)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Cannot acquire image since camera status is unverified").toLatin1().data());
    }
    else
    {
        m_isgrabbing = true;
        m_timeout = false;
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    if (!retValue.containsError())
    {
        int i = 0;

        while (1)
        {
            if (m_videoInput->isFrameNew(m_deviceID))
            {
                m_videoInput->getPixels(m_deviceID, (unsigned char *)m_pDataMatBuffer.data, false, m_flipImage);
                break;
            }

            if (++i > 200)
            {
                m_timeout = true;
                break;
            }

            Sleep(3);
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MSMediaFoundation::retrieveData(ito::DataObject *externalDataObject)
{
    ito::RetVal retValue(ito::retOk);

    bool RetCode = false;
    cv::Mat *internalMat = NULL;

    int curxsize = m_params["sizex"].getVal<int>();
    int curysize = m_params["sizey"].getVal<int>();
    int x0 = m_params["x0"].getVal<int>();
    int y0 = m_params["y0"].getVal<int>();
    bool resizeRequired = (x0 > 0 || y0 > 0);

    ito::DataObject *dataObj = &m_data;
    if (externalDataObject)
    {
        dataObj = externalDataObject;
    }

    bool hasListeners = false;
    if (m_autoGrabbingListeners.size() > 0)
    {
        hasListeners = true;
    }

    if (m_isgrabbing == false)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("Tried to get picture without triggering exposure").toLatin1().data());
    }
    else if (m_timeout == true)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("Timeout while acquiring image").toLatin1().data());
    }
    else
    {
        if (!retValue.containsError())
        {
            int desiredBpp = m_params["bpp"].getVal<int>();
            cv::Mat tempImage;

            if (m_imgCols != curxsize || m_imgRows != curysize)
            {
                resizeRequired = true;
            }

            if (m_imgChannels != 1 && m_imgChannels != 3)
            {
                retValue += ito::RetVal(ito::retError, 0, tr("Error: channels sizes other than 1 or 3 not allowed.").toLatin1().data());
            }
            else if ((desiredBpp != 8 && desiredBpp != 16))
            {
                retValue += ito::RetVal(ito::retError, 0, tr("Error: desired bpp must be 8 or 16 bit.").toLatin1().data());
            }
            else
            {
                //step 1. check ROI
                if (resizeRequired == false)
                {
                    tempImage = m_pDataMatBuffer;
                }
                else
                {
                    cv::Range ranges[] = { cv::Range(y0, y0 + curysize), cv::Range(x0, x0 + curxsize) };
                    tempImage = cv::Mat(m_pDataMatBuffer, ranges);
                }

                //step 2. check whether 3 channel color should be transformed to 1 channel grayscale
                if (m_imgChannels == 3 && m_colorMode == modeGray)
                {
#if (CV_MAJOR_VERSION >= 4)
                    cv::cvtColor(tempImage, tempImage, cv::COLOR_BGR2GRAY, 0); //camera provides BGR images in OpenCV
#else
                    cv::cvtColor(tempImage, tempImage, CV_BGR2GRAY, 0); //camera provides BGR images in OpenCV
#endif
                }

                //step 3: create m_data (if not yet available)
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

                    //step 4: check whether tempImage must be converted to other type
                    if (desiredBpp != m_imgBpp)
                    {
                        if (desiredBpp == 8)
                        {
                            tempImage.convertTo(tempImage, CV_8U);
                        }
                        else if (desiredBpp == 16)
                        {
                            tempImage.convertTo(tempImage, CV_16U);
                        }
                        else
                        {
                            retValue += ito::RetVal(ito::retError, 0, tr("Error while converting data format. Unsupported format.").toLatin1().data());
                        }
                    }
                }

                if (!retValue.containsError())
                {
                    if (tempImage.channels() == 1)
                    {
                        internalMat = dataObj->getCvPlaneMat(0);
                        tempImage.copyTo(*(internalMat));

                        if (externalDataObject && hasListeners)
                        {
                            internalMat = m_data.get_mdata()[0];
                            tempImage.copyTo(*(internalMat));
                        }
                    }
                    else if (tempImage.channels() == 3 && (m_colorMode == modeAuto || m_colorMode == modeColor))
                    {
                        cv::Mat out[] = { *(dataObj->getCvPlaneMat(0)) }; //{ *(cv::Mat*)(dataObj->get_mdata()[0]) , *(cv::Mat*)(dataObj->get_mdata()[1]) , *(cv::Mat*)(dataObj->get_mdata()[2]) };
                        int fromTo[] = {0, 0, 1, 1, 2, 2}; //{0,2,1,1,2,0}; //implicit BGR (camera) -> BGR (dataObject style) conversion

                        cv::mixChannels(&tempImage, 1, out, 1, fromTo, 3);

                        if (externalDataObject && hasListeners)
                        {
                            cv::Mat out[] = { *(dataObj->getCvPlaneMat(0)) }; //{ *(cv::Mat*)(m_data.get_mdata()[0]) , *(cv::Mat*)(m_data.get_mdata()[1]) , *(cv::Mat*)(m_data.get_mdata()[2]) };
                            cv::mixChannels(&tempImage, 1, out, 1, fromTo, 3);
                        }
                    }
                    else if (tempImage.channels() == 3) //R,G,B selection
                    {
                        cv::Mat out[] = { *(dataObj->getCvPlaneMat(0)) };
                        int fromTo[] = {0, 0};
                        switch(m_colorMode)
                        {
                            case modeRed: fromTo[0] = 2; break; //red
                            case modeGreen: fromTo[0] = 1; break; //green
                            default /*3*/: fromTo[0] = 0; break; //blue
                        }
                        cv::mixChannels(&tempImage, 1, out, 1, fromTo, 1);

                        if (externalDataObject && hasListeners)
                        {
                            cv::Mat out[] = { *(m_data.get_mdata()[0]) };
                            cv::mixChannels(&tempImage, 1, out, 1, fromTo, 1);
                        }
                    }
                    else
                    {
                        retValue += ito::RetVal(ito::retError, 0, tr("unknown color, conversion... combination in retrieveImage").toLatin1().data());
                    }
                }
            }
        }

        m_isgrabbing = false;
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MSMediaFoundation::checkData(ito::DataObject *externalDataObject)
{
    if (!m_camStatusChecked)
    {
        return ito::RetVal(ito::retError, 0, tr("current camera status is undefined").toLatin1().data());
    }

    int futureHeight = m_params["sizey"].getVal<int>();
    int futureWidth = m_params["sizex"].getVal<int>();
    int futureChannels;
    int futureType;

    const char *colorMode = m_params["colorMode"].getVal<char*>();
    if (m_imgChannels == 1 && (m_colorMode == modeGray || m_colorMode == modeAuto))
    {
        futureChannels = 1;
    }
    else if (m_imgChannels == 3 && (m_colorMode == modeColor || m_colorMode == modeAuto))
    {
        futureChannels = 3;
    }
    else
    {
        futureChannels = 1;
    }

    int bpp = m_params["bpp"].getVal<int>();
    if (bpp <= 8 && futureChannels == 1)
    {
        futureType = ito::tUInt8;
    }
    else if (bpp <= 16 && futureChannels == 1)
    {
        futureType = ito::tUInt16;
    }
    else if (bpp <= 32 && futureChannels == 1)
    {
        futureType = ito::tInt32;
    }
    else  if (futureChannels == 1)
    {
        futureType = ito::tFloat64;
    }
    else if (futureChannels == 3 && bpp <= 8)
    {
        futureType = ito::tRGBA32;
    }
    else
    {
        return ito::RetVal(ito::retError, 0, tr("A camera with a bitdepth > 8 cannot be operated in color mode.").toLatin1().data());
    }

    if (futureType == ito::tRGBA32 && (m_alphaChannel.cols != futureWidth || m_alphaChannel.rows != futureHeight))
    {
        m_alphaChannel = cv::Mat(futureHeight, futureWidth, CV_8UC1, cv::Scalar(255));
    }

    if (m_pDataMatBuffer.rows != m_imgRows || m_pDataMatBuffer.cols != m_imgCols) //always original chip size, resize to roi in retrieveImage
    {
        m_pDataMatBuffer = cv::Mat(m_imgRows, m_imgCols, CV_8UC3);
    }

    if (!externalDataObject)
    {
        if (m_data.getDims() != 2 || m_data.getSize(0) != (unsigned int)futureHeight || m_data.getSize(1) != (unsigned int)futureWidth || m_data.getType() != futureType)
        {
            m_data = ito::DataObject(futureHeight,futureWidth,futureType);

            if (futureType == ito::tRGBA32)
            {
                //copy alpha channel to 4th channel in m_data
                const int relations[] = {0,3};
                cv::mixChannels(&m_alphaChannel, 1, (cv::Mat*)m_data.get_mdata()[0], 1, relations, 1);
            }
        }
    }
    else
    {
        int dims = externalDataObject->getDims();
        if (externalDataObject->getDims() == 0) //empty external dataObject
        {
            *externalDataObject = ito::DataObject(futureHeight,futureWidth,futureType);
        }
        else if (externalDataObject->calcNumMats () != 1)
        {
            return ito::RetVal(ito::retError, 0, tr("Error during check data, external dataObject invalid. Object has more than 1 plane. It must be of right size and type or an uninitialized image.").toLatin1().data());
        }
        else if (externalDataObject->getSize(dims - 2) != (unsigned int)futureHeight || externalDataObject->getSize(dims - 1) != (unsigned int)futureWidth || externalDataObject->getType() != futureType)
        {
            return ito::RetVal(ito::retError, 0, tr("Error during check data, external dataObject invalid. Object must be of right size and type or a uninitialized image.").toLatin1().data());
        }

        if (futureType == ito::tRGBA32)
        {
            //copy alpha channel to 4th channel in m_data
            const int relations[] = {0,3};
            cv::mixChannels(&m_alphaChannel, 1, (cv::Mat*)externalDataObject->get_mdata()[externalDataObject->seekMat(0)], 1, relations, 1);
        }
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MSMediaFoundation::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = checkInitState();
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);

    retValue += retrieveData();

    if (!retValue.containsError())
    {
        sendDataToListeners(0); //don't wait for live data, since user should get the data as fast as possible.

        if (dObj)
        {
            (*dObj) = m_data;
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
ito::RetVal MSMediaFoundation::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = checkInitState();
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);

    if (!dObj)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Empty object handle retrieved from caller").toLatin1().data());
    }

    if (!retValue.containsError())
    {
        retValue += retrieveData(dObj);  //checkData is executed inside of retrieveData
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
void MSMediaFoundation::dockWidgetVisibilityChanged(bool visible)
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

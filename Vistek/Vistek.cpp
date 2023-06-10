/* ********************************************************************
    Plugin "Vistek" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut fuer Technische Optik (ITO),
    Universitaet Stuttgart, Germany

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

#include "Vistek.h"
#include "VistekInterface.h"
#include "VistekContainer.h"

#include "dialogVistek.h"

#define _USE_MATH_DEFINES  // needs to be defined to enable standard declartions of PI constant
#include "math.h"

#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>
#include <qmetaobject.h>

#include <qdockwidget.h>
#include <qpushbutton.h>
#include <qmetaobject.h>
#include "dockWidgetVistek.h"
#include <qelapsedtimer.h>
#include <qthread.h>

#ifdef WIN32
    #include <Windows.h>
#endif

VistekContainer* VistekContainer::m_pVistekContainer = NULL;

Q_DECLARE_METATYPE(ito::DataObject)

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \class Vistek
    \brief Class that can handle SVS Vistek GigE Cameras.

    Usually every method in this class can be executed in an own thread. Only the constructor, destructor, showConfDialog will be executed by the
    main (GUI) thread.
*/

//----------------------------------------------------------------------------------------------------------------------------------
//! Shows the configuration dialog. This method must be executed in the main (GUI) thread and is usually called by the addIn-Manager.
/*!
    Creates new instance of dialogVistek, calls the method setVals of dialogVistek, starts the execution loop and if the dialog
    is closed, reads the new parameter set and deletes the dialog.

    \return ito::RetVal retOk
    \sa dialogVistek
*/
const ito::RetVal Vistek::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogVistek(this, &m_features));
}

//----------------------------------------------------------------------------------------------------------------------------------
//! constructor for Vistek
/*!
    In the constructor the m_params-vector with all parameters, which are accessible by getParam or setParam, is built.
    Additionally the optional docking widget for the Vistek's toolbar is instantiated and created by createDockWidget.

    \param [in] uniqueID is an unique identifier for this Vistek-instance
    \param [in] parent is the parent object for this grabber (default: NULL)
    \sa ito::tParam, createDockWidget, setParam, getParam
*/
Vistek::Vistek(QObject *parent) :
    AddInGrabber(),
    m_pVistekContainer(NULL),
    m_cam(SVGigE_NO_CAMERA),
    TriggerViolationCount(0),
    m_streamingChannel(SVGigE_NO_STREAMING_CHANNEL),
    m_eventID(SVGigE_NO_EVENT),
    m_gainIncrement(0.1f),
    m_exposureIncrement(0.001f)
{
    qRegisterMetaType<VistekFeatures>("VistekFeatures");

    m_acquiredImage.status = asNoImageAcquired;

    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "Vistek", NULL);
    m_params.insert(paramVal.getName(), paramVal);

    // Camera specific information
    paramVal = ito::Param("cameraModel", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("Camera Model ID").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String), true);
    paramVal.getMetaT<ito::ParamMeta>()->setCategory("General");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("cameraManufacturer", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("Camera manufacturer").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String), true);
    paramVal.getMetaT<ito::ParamMeta>()->setCategory("General");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("cameraVersion", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("Camera firmware version").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String), true);
    paramVal.getMetaT<ito::ParamMeta>()->setCategory("General");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("cameraSerialNo", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("Serial number of the camera (see camera housing)").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String), true);
    paramVal.getMetaT<ito::ParamMeta>()->setCategory("General");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("cameraIP", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("IP adress of the camera").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String), true);
    paramVal.getMetaT<ito::ParamMeta>()->setCategory("General");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("camnum", ito::ParamBase::Int, 0, 63, 0, tr("Camera Number").toLatin1().data());
    paramVal.getMetaT<ito::ParamMeta>()->setCategory("General");
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("exposure", ito::ParamBase::Double, 0.00001, 2.0, 0.0, tr("Exposure time in [s] (deprecated: use integration_time instead; this is an alias for integration_time only)").toLatin1().data());
    paramVal.getMetaT<ito::ParamMeta>()->setCategory("AcquisitionControl");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("integration_time", ito::ParamBase::Double, 0.00001, 2.0, 0.0, tr("Exposure time in [s].").toLatin1().data());
    paramVal.getMetaT<ito::ParamMeta>()->setCategory("AcquisitionControl");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("gain", ito::ParamBase::Double, 0.0, 18.0, 0.0, tr("Gain [0..18 dB]").toLatin1().data());
    paramVal.getMetaT<ito::ParamMeta>()->setCategory("AcquisitionControl");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("offset", ito::ParamBase::Double, 0.0, 1.0, 0.0, tr("Offset [0.0..1.0]").toLatin1().data());
    paramVal.getMetaT<ito::ParamMeta>()->setCategory("AcquisitionControl");
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("binning", ito::ParamBase::Int, 0, 404, 0, tr("Binning mode (OFF = 0 [default], HORIZONTAL = 1 (or 102), VERTICAL = 2 (or 201),  2x2 = 3 (or 202), 3x3 = 4 (or 303), 4x4 = 5 (or 404)").toLatin1().data());
    paramVal.getMetaT<ito::ParamMeta>()->setCategory("ImageFormatControl");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizex", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 4096, 1024, tr("Width of current camera frame").toLatin1().data());
    paramVal.getMetaT<ito::ParamMeta>()->setCategory("ImageFormatControl");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizey", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 4096, 1024, tr("Height of current camera frame").toLatin1().data());
    paramVal.getMetaT<ito::ParamMeta>()->setCategory("ImageFormatControl");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("bpp", ito::ParamBase::Int, 8, 64, 8, tr("bit-depth for camera buffer").toLatin1().data());
    paramVal.getMetaT<ito::ParamMeta>()->setCategory("ImageFormatControl");
    m_params.insert(paramVal.getName(), paramVal);

    int roi[] = { 0, 0, 2048, 2048 };
    paramVal = ito::Param("roi", ito::ParamBase::IntArray, 4, roi, tr("ROI (x0, y0, width, height)").toLatin1().data());
    ito::RectMeta *rm = new ito::RectMeta(ito::RangeMeta(0, 2047), ito::RangeMeta(0, 2047), "ImageFormatControl");
    paramVal.setMeta(rm, true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("timestamp", ito::ParamBase::Double | ito::ParamBase::Readonly, 0.0, 10000000.0, 0.0, tr("Time in ms since last image (end of exposure)").toLatin1().data());
    paramVal.getMetaT<ito::ParamMeta>()->setCategory("General");
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("streamingPacketSize", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 16000, 1500, tr("Used streaming packet size (in bytes, more than 1500 usually only possible if you enable jumbo-frames at your network adapter)").toLatin1().data());
    paramVal.getMetaT<ito::ParamMeta>()->setCategory("General");
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("logLevel", ito::ParamBase::Int, 0, 7, 0, tr("Log level. The logfile is Vistek_SVGigE.log in the current directory. 0 - logging off (default),  1 - CRITICAL errors that prevent from further operation, 2 - ERRORs that prevent from proper functioning, 3 - WARNINGs which usually do not affect proper work, 4 - INFO for listing camera communication (default), 5 - DIAGNOSTICS for investigating image callbacks, 6 - DEBUG for receiving multiple parameters for image callbacks, 7 - DETAIL for receiving multiple signals for each image callback").toLatin1().data());
    paramVal.getMetaT<ito::ParamMeta>()->setCategory("General");
    m_params.insert(paramVal.getName(), paramVal);

    if (hasGuiSupport())
    {
        //now create dock widget for this plugin
        DockWidgetVistek *dw = new DockWidgetVistek(this);

        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dw);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
//! This is the destructor of the Vistek class.
/*!
    \sa ~AddInBase
*/
Vistek::~Vistek()
{
   m_params.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Returns parameter of m_params with key name.
/*!
    This method copies the string of the corresponding parameter to val with a maximum length of maxLen.

    \param [in,out] val is a shared-pointer pointing to a Param.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return ito::RetVal retOk in case that everything is ok, else retError
    \sa ito::Param, setParam ItomSharedSemaphore
*/
ito::RetVal Vistek::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
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
//! Sets parameter of m_params with key name.
/*!
    This method copies the given string of the char pointer val with a size of len to the m_params-parameter.

    \param [in] val is the pointer to the ParamBase.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return ito::RetVal retOk in case that everything is ok, else retError
    \sa ito::Param, ItomSharedSemaphore
*/
ito::RetVal Vistek::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex;
    int index;
    QString suffix;
    ParamMapIterator it;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

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
        bool set = false;
        if (!key.compare("logLevel"))
        {
            retValue += checkError(tr("set log level").toLatin1().data(), Camera_registerForLogMessages(m_cam,val->getVal<int>(), "Vistek_SVGigE.log", NULL));
        }
        else if (!key.compare("exposure") || !key.compare("integration_time"))
        {
            // Camera_setExposureTime expects mu/s, so multiply by 10^6
            retValue += checkError(tr("set exposure time").toLatin1().data(), Camera_setExposureTime(m_cam, val->getVal<double>() * 1.e6));

            float val;
            Camera_getExposureTime(m_cam, &val);
            m_params["exposure"].setVal<double>(val / 1.e6);
            m_params["integration_time"].setVal<double>(val / 1.e6);
            set = true;
        }
        else if (!key.compare("gain"))
        {
            // Camera_setGain expects a value between 0 and 18 in dB
            retValue += checkError(tr("set gain").toLatin1().data(), Camera_setGain(m_cam, val->getVal<double>()));
        }
        else if (!key.compare("offset"))
        {
            // Camera_setOffset expects a value between 0..255 mapped from 0.0 .. 1.0
            retValue += checkError(tr("set offset").toLatin1().data(), Camera_setOffset(m_cam, val->getVal<double>() * 255));
        }
        else if (!key.compare("roi"))
        {
            if (hasIndex)
            {
                int roi[] = { 0, 0, 0, 0 };
                memcpy(roi, it->getVal<int*>(), 4 * sizeof(int));

                switch (index)
                {
                case 0:
                    roi[0] = val->getVal<int>();
                    break;
                case 1:
                    roi[1] = val->getVal<int>();
                    break;
                case 2:
                    roi[2] = val->getVal<int>();
                    break;
                case 3:
                    roi[3] = val->getVal<int>();
                    break;
                default:
                    retValue += ito::RetVal::format(ito::retError, 0, "invalid index [0..3]");
                    break;
                }

                if (!retValue.containsError())
                {
                    if (grabberStartedCount() >= 1 && m_cam != SVGigE_NO_CAMERA)
                    {
                        retValue += checkError(tr("stop camera").toLatin1().data(), Camera_setAcquisitionControl(m_cam, ACQUISITION_CONTROL_STOP));
                    }

                    retValue += stopStreamAndDeleteCallbacks();
                    retValue += checkError(tr("set region of interest").toLatin1().data(), Camera_setAreaOfInterest(m_cam, roi[2], roi[3], roi[0], roi[1]));
                    retValue += startStreamAndRegisterCallbacks();

                    if (grabberStartedCount() >= 1 && m_cam != SVGigE_NO_CAMERA)
                    {
                        retValue += checkError(tr("restart camera 1").toLatin1().data(), Camera_setAcquisitionMode(m_cam, ACQUISITION_MODE_SOFTWARE_TRIGGER));
                        retValue += checkError(tr("restart camera 2").toLatin1().data(), Camera_setAcquisitionControl(m_cam, ACQUISITION_CONTROL_START));
                    }
                }

                if (!retValue.containsError())
                {
                    it->setVal<int*>((int*)roi, 4);
                }
            }
            else
            {
                const int *roi = val->getVal<const int*>();

                if (grabberStartedCount() >= 1 && m_cam != SVGigE_NO_CAMERA)
                {
                    retValue += checkError(tr("stop camera").toLatin1().data(), Camera_setAcquisitionControl(m_cam, ACQUISITION_CONTROL_STOP));
                }

                retValue += stopStreamAndDeleteCallbacks();
                retValue += checkError(tr("set region of interest").toLatin1().data(), Camera_setAreaOfInterest(m_cam, roi[2], roi[3], roi[0], roi[1]));
                retValue += startStreamAndRegisterCallbacks();

                if (grabberStartedCount() >= 1 && m_cam != SVGigE_NO_CAMERA)
                {
                    retValue += checkError(tr("restart camera 1").toLatin1().data(), Camera_setAcquisitionMode(m_cam, ACQUISITION_MODE_SOFTWARE_TRIGGER));
                    retValue += checkError(tr("restart camera 2").toLatin1().data(), Camera_setAcquisitionControl(m_cam, ACQUISITION_CONTROL_START));
                }

                if (!retValue.containsError())
                {
                    it->setVal<int*>((int*)roi, 4);
                    set = true;
                }
            }
        }
        else if (!key.compare("binning"))
        {
            BINNING_MODE mode;
            BINNING_MODE currentMode;
            Camera_getBinningMode(m_cam, &currentMode);

            switch (val->getVal<int>())
            {
            case 0:
                mode = BINNING_MODE_OFF;
                break;
            case 1:
            case 102:
                mode = BINNING_MODE_HORIZONTAL;
                break;
            case 2:
            case 201:
                mode = BINNING_MODE_VERTICAL;
                break;
            case 3:
            case 202:
                mode = BINNING_MODE_2x2;
                break;
            case 4:
            case 303:
                mode = BINNING_MODE_3x3;
                break;
            case 5:
            case 404:
                mode = BINNING_MODE_4x4;
                break;
            default:
                retValue += ito::RetVal(ito::retError, 0, tr("binning invalid: Accepted values are OFF = 0 [default], HORIZONTAL = 1 (or 102), VERTICAL = 2 (or 201),  2x2 = 3 (or 202), 3x3 = 4 (or 303), 4x4 = 5 (or 404)").toLatin1().data());
                break;
            }

            if (!retValue.containsError() && mode != currentMode)
            {
                m_binningMode = mode;

                if (grabberStartedCount() >= 1 && m_cam != SVGigE_NO_CAMERA)
                {
                    retValue += checkError(tr("stop camera").toLatin1().data(), Camera_setAcquisitionControl(m_cam, ACQUISITION_CONTROL_STOP));
                }

                retValue += stopStreamAndDeleteCallbacks();
                retValue += checkError(tr("set binning").toLatin1().data(), Camera_setBinningMode(m_cam, mode));
                retValue += startStreamAndRegisterCallbacks();

                if (grabberStartedCount() >= 1 && m_cam != SVGigE_NO_CAMERA)
                {
                    retValue += checkError(tr("restart camera 1").toLatin1().data(), Camera_setAcquisitionMode(m_cam, ACQUISITION_MODE_SOFTWARE_TRIGGER));
                    retValue += checkError(tr("restart camera 2").toLatin1().data(), Camera_setAcquisitionControl(m_cam, ACQUISITION_CONTROL_START));
                }

                set = true;
            }
        }
        else if (!key.compare("bpp"))
        {
            SVGIGE_PIXEL_DEPTH depth;
            switch (val->getVal<int>())
            {
            case 8:
                if (m_features.has8bit == false)
                {
                    retValue += ito::RetVal(ito::retError ,0, tr("8bit not supported by this camera").toLatin1().data());
                }
                depth = SVGIGE_PIXEL_DEPTH_8;
                break;
            case 10:
                if (m_features.has10bit == false)
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("10bit not supported by this camera").toLatin1().data());
                }
                retValue += ito::RetVal(ito::retError, 0, tr("10bit not supported by this driver version").toLatin1().data());
                //depth = SVGIGE_PIXEL_DEPTH_10;
                break;
            case 12:
                if (m_features.has12bit == false)
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("12bit not supported by this camera").toLatin1().data());
                }
                depth = SVGIGE_PIXEL_DEPTH_12;
                break;
            case 16:
                if (m_features.has16bit == false)
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("16bit not supported by this camera").toLatin1().data());
                }
                depth = SVGIGE_PIXEL_DEPTH_16;
                break;
            default:
                retValue += ito::RetVal(ito::retError, 0, tr("unknown bpp value (use 8bit, 10bit, 12bit or 16bit)").toLatin1().data());
                break;
            }

            if (!retValue.containsError())
            {
                if (grabberStartedCount() >= 1 && m_cam != SVGigE_NO_CAMERA)
                {
                    retValue += checkError(tr("stop camera").toLatin1().data(), Camera_setAcquisitionControl(m_cam, ACQUISITION_CONTROL_STOP));
                }

                retValue += stopStreamAndDeleteCallbacks();
                retValue += checkError(tr("set pixel depth").toLatin1().data(), Camera_setPixelDepth(m_cam, depth));
                retValue += startStreamAndRegisterCallbacks();

                if (grabberStartedCount() >= 1 && m_cam != SVGigE_NO_CAMERA)
                {
                    retValue += checkError(tr("restart camera 1").toLatin1().data(), Camera_setAcquisitionMode(m_cam, ACQUISITION_MODE_SOFTWARE_TRIGGER));
                    retValue += checkError(tr("restart camera 2").toLatin1().data(), Camera_setAcquisitionControl(m_cam, ACQUISITION_CONTROL_START));
                }
            }
        }

        if (!retValue.containsError() && !set) //binning is already set earlier
        {
            retValue += it->copyValueFrom(&(*val));
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
//! The init method is called by the addInManager after the initiation of a new instance of Vistek.
/*!
    This init method gets the mandatory and optional parameter vectors of type tParam and must copy these given parameters to the
    internal m_params-vector. Notice that this method is called after that this instance has been moved to its own (non-gui) thread.

    \param [in] paramsMand is a pointer to the vector of mandatory ParamBase.
    \param [in] paramsOpt is a pointer to the vector of optional ParamBase.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return ito::RetVal retOk
*/
ito::RetVal Vistek::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    int NumberOfCams = 0;
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal ReturnValue = ito::retOk;
    int camNum = -1;
    VistekCam TempCam;

    //*********************************************
    m_params["cameraSerialNo"].setVal<char*>(paramsOpt->at(0).getVal<char*>());

    m_params["streamingPacketSize"].setVal<int>(paramsOpt->at(1).getVal<int>());

    m_numBuf = paramsOpt->at(2).getVal<int>();

    m_pVistekContainer = VistekContainer::getInstance();
    ReturnValue += m_pVistekContainer->initCameraContainer();

    if (!ReturnValue.containsError())
    {
        // Check if a valid Serial number is specified
        if (m_params["cameraSerialNo"].getVal<char*>() != NULL && m_params["cameraSerialNo"].getVal<char*>() != "")
        {
            camNum = m_pVistekContainer->getCameraBySN(m_params["cameraSerialNo"].getVal<char*>());
        }

        if (camNum < 0)
        {
            camNum = m_pVistekContainer->getNextFreeCam();
        }

        if (camNum >= 0)
        {
            m_params["camnum"].setVal<int>(camNum);

            // Get camera info
            TempCam = m_pVistekContainer->getCamInfo(camNum);
            m_params["cameraModel"].setVal<char*>(TempCam.camModel.toLatin1().data());
            m_params["cameraSerialNo"].setVal<char*>(TempCam.camSerialNo.toLatin1().data());
            m_params["cameraVersion"].setVal<char*>(TempCam.camVersion.toLatin1().data());
            m_params["cameraIP"].setVal<char*>(TempCam.camIP.toLatin1().data());
            m_params["cameraManufacturer"].setVal<char*>(TempCam.camManufacturer.toLatin1().data());
            m_identifier = TempCam.camSerialNo;

            ReturnValue += initCamera(camNum);
        }
        else
        {
            ReturnValue += ito::RetVal(ito::retError,0,tr("No free camera found").toLatin1().data());
        }
        //*********************************************
    }

    if (waitCond)
    {
        waitCond->returnValue = ReturnValue;
        waitCond->release();
    }


    if (!ReturnValue.containsError())
    {
        emit parametersChanged(m_params); //send changed parameters to any connected dialogs or dock-widgets
    }

    setInitialized(true); //init method has been finished (independent on retval)

    return ReturnValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! The close method is called before an instance is deleted by the VistekInterface
/*!
    Notice that this method is called in the actual thread of the instance.

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return ito::RetVal retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal Vistek::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);

    if (m_timerID > 0)
    {
        killTimer(m_timerID);
        m_timerID = 0;
    }

    // ONE TIME DEINIT HERE (Holger)
    //*********************************************
    if (m_cam != SVGigE_NO_CAMERA)
    {
        std::cout << "Forcing camera shutdown.\n" << std::endl;

        //stop camera
        Camera_setAcquisitionMode(m_cam, ACQUISITION_MODE_NO_ACQUISITION);
        //invalidate image buffer
        m_acquiredImage.status = asNoImageAcquired;
        // Unregister callbacks
        Stream_unregisterEventCallback(m_streamingChannel, m_eventID, &MessageCallback);
        Stream_closeEvent(m_streamingChannel, m_eventID);
        m_eventID = SVGigE_NO_EVENT;

        // delete stream
        StreamingChannel_delete(m_streamingChannel);
        m_streamingChannel = SVGigE_NO_STREAMING_CHANNEL;

        Camera_closeConnection(m_cam); // This is necessary to prevent an access violation if m gets terminated with an open cam connection
        m_cam = SVGigE_NO_CAMERA;
        m_pVistekContainer->freeCameraStatus(m_params["camnum"].getVal<int>());
    }
    //*********************************************

    if (waitCond)
    {
        waitCond->returnValue = ito::retOk;
        waitCond->release();

        return waitCond->returnValue;
    }
    else
    {
        return ito::retOk;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Error conversion between SVGigE_RETURN and ito::RetVal
/*!
    Returns retOk if returnCode is SVGigE_SUCCESS, else retError with appropriate error message
    \return ito::RetVal
*/
ito::RetVal Vistek::checkError(const char *prependStr, SVGigE_RETURN returnCode)
{
    ito::RetVal retval;
    if (returnCode != SVGigE_SUCCESS)
    {
        const char *str = prependStr;
        if (prependStr == NULL)
        {
            str = "";
        }

        const char *msg = Error_getMessage(returnCode);
        if (msg)
        {
            retval += ito::RetVal::format(ito::retError, returnCode, tr("%s: Vistek DLL error %i '%s' occurred").toLatin1().data(), str, returnCode, msg);
        }
        else
        {
            retval += ito::RetVal::format(ito::retError, returnCode, tr("%s: unknown Vistek DLL error %i occurred").toLatin1().data(), str, returnCode);
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! With startDevice the camera aquisition is started.
/*!
    This method sets the acquisition mode of the camera to software triggering.

    \note This method is similar to VideoCapture::open() of openCV

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return ito::RetVal retOk if starting was successfull, retWarning if startDevice has been calling at least twice.
    \sa stopDevice
*/
ito::RetVal Vistek::startDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = ito::retOk;

    if (m_streamingChannel == SVGigE_NO_STREAMING_CHANNEL)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("streaming server not started").toLatin1().data());
    }
    else
    {
        //streaming server started, events connected, timeout set, m_data is correct, intermediate buffer is ok

        incGrabberStarted();

        if (grabberStartedCount() == 1)
        {
            //*********************************************
            if (m_cam != SVGigE_NO_CAMERA)
            {
                retValue += checkError(tr("set software trigger and start 1").toLatin1().data(), Camera_setAcquisitionMode(m_cam, ACQUISITION_MODE_SOFTWARE_TRIGGER));
                retValue += checkError(tr("set software trigger and start 2").toLatin1().data(), Camera_setAcquisitionControl(m_cam, ACQUISITION_CONTROL_START));
            }
            //*********************************************
        }

        // Dont count a grabber that is not really started ...
        if (retValue != ito::retOk)
        {
            decGrabberStarted();
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
//! With stopDevice the camera device is stopped (opposite to startDevice)
/*!
    This method sets the acquisition mode of the camera to off.

    \note This method is similar to VideoCapture::release() of openCV

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return ito::RetVal retOk if everything is ok, retError if camera wasn't started before
    \sa startDevice
*/
ito::RetVal Vistek::stopDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = ito::retOk;

    decGrabberStarted();
    if (grabberStartedCount() == 0)
    {
        //*********************************************
        if (m_cam != SVGigE_NO_CAMERA)
        {
            retValue += checkError(tr("stop camera").toLatin1().data(), Camera_setAcquisitionControl(m_cam, ACQUISITION_CONTROL_STOP));
        }
        //*********************************************
    }
    else if (grabberStartedCount() < 0)
    {
        retValue += ito::RetVal(ito::retError, 1001, tr("StopDevice of Vistek can not be executed, since camera has not been started.").toLatin1().data());
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
//! This method triggers a new data acquisition.
/*!
    With this method a new data acquisition is triggered by the camera, that means the acquisition of the data starts at the moment, this method is called.
    The new data is then stored either in internal camera memory or in internal memory of this class.

    \note This method is similar to VideoCapture::grab() of openCV

    \param [in] trigger may describe the trigger parameter (unused here)
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return ito::RetVal retOk if everything is ok, retError if camera has not been started or an older data lies in memory which has not be fetched by getVal, yet.
    \sa getVal, retrieveData
*/
ito::RetVal Vistek::acquire(const int trigger, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = ito::retOk;

    int timeout = 0;

    if (grabberStartedCount() <= 0)
    {
        retValue += ito::RetVal(ito::retError, 1002, tr("acquisition not possible, since Vistek camera has not been started.").toLatin1().data());
    }
    else
    {

        /*bool b;
        SVGigE_RETURN ret2 = StreamingChannel_getReadoutTransfer(m_streamingChannel, &b);*/
        m_acquisitionRetVal = ito::retOk;
        m_acquiredImage.mutex.lock();
        m_acquiredImage.status = asWaitingForTransfer; //start acquisition
        m_acquiredImage.frameCompleted = false;
        m_acquiredImage.mutex.unlock();
        //qDebug() << "software trigger...";
        SVGigE_RETURN ret = Camera_softwareTrigger(m_cam);
        if (ret != SVGigE_SUCCESS)
        {
            m_acquiredImage.mutex.lock();
            m_acquiredImage.status = asNoImageAcquired;
            m_acquiredImage.mutex.unlock();
            retValue += checkError(tr("Camera trigger failed.").toLatin1().data(), ret);

            if (waitCond)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
            }
        }
        else
        {
            if (waitCond)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
            }

            QElapsedTimer timer;
            timer.start();
            while (!m_acquiredImage.frameCompleted)
            {
                if (timer.elapsed() > 2000)
                {
                    m_acquiredImage.mutex.lock();
                    m_acquiredImage.status = asNoImageAcquired;
                    m_acquiredImage.mutex.unlock();
                    m_acquisitionRetVal  = checkError(tr("Frame not completed within given timeout.").toLatin1().data(), ret);
                    break;
                }
            }
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! This method copies the acquired image data to externalDataObject or m_data.
/*!
    If *externalDataObject is NULL then it will be reassigned to &m_data so the image data is copied to m_data instead.

    \param [in] *externalDataObject is the data object where the image is to be stored. (Defaults to NULL)
    \return ito::RetVal retOk if everything is ok, retError if camera has not been started or no image has been acquired.
    \sa DataObject, getVal, acquire
*/
ito::RetVal Vistek::retrieveData(ito::DataObject *externalDataObject)
{
    //qDebug() << "retrieveData: threadID:" << QThread::currentThreadId();
    ito::RetVal retValue = m_acquisitionRetVal;

    if (retValue.containsError())
    {
        return retValue;
    }

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

    if (grabberStartedCount() <= 0)
    {
        retValue += ito::RetVal(ito::retError, 1002, tr("getVal of Vistek can not be executed, since camera has not been started.").toLatin1().data());
    }
    if (m_acquiredImage.status == asNoImageAcquired)
    {
        retValue += ito::RetVal(ito::retError, 1002, tr("getVal of Vistek can not be executed, since no image has been acquired.").toLatin1().data());
    }
    else
    {
        /*bool isReadoutTransfer;
        StreamingChannel_getReadoutTransfer(m_streamingChannel, &isReadoutTransfer);
        qDebug() << "isReadoutTransfer" << isReadoutTransfer;*/
        /*float ftimeout;
        StreamingChannel_getChannelTimeout(m_streamingChannel, &ftimeout);
        qDebug() << "timeout" << ftimeout;*/

        int timeout = 0;
        //qDebug() << "...start";
        while(m_acquiredImage.status == asWaitingForTransfer)
        {
            if (timeout > 2000)
            {
                m_acquiredImage.mutex.lock();
                m_acquiredImage.status = asTimeout;
                m_acquiredImage.mutex.unlock();
                break;
            }
            Sleep(1);
            timeout++;
        }
        //qDebug() << "...end";

        if (m_acquiredImage.status == asImageReady && m_acquiredImage.sizex >= 0)
        {
            m_acquiredImage.mutex.lock();
            if (m_data.getType() == ito::tUInt8)
            {
                if (copyExternal) retValue += externalDataObject->copyFromData2D<ito::uint8>((ito::uint8*) m_acquiredImage.buffer.data(), m_acquiredImage.sizex, m_acquiredImage.sizey);
                if (!copyExternal || hasListeners) retValue += m_data.copyFromData2D<ito::uint8>((ito::uint8*) m_acquiredImage.buffer.data(), m_acquiredImage.sizex, m_acquiredImage.sizey);
            }
            else if (m_data.getType() == ito::tUInt16)
            {
                if (copyExternal) retValue += externalDataObject->copyFromData2D<ito::uint16>((ito::uint16*) m_acquiredImage.buffer.data(), m_acquiredImage.sizex, m_acquiredImage.sizey);
                if (!copyExternal || hasListeners) retValue += m_data.copyFromData2D<ito::uint16>((ito::uint16*) m_acquiredImage.buffer.data(), m_acquiredImage.sizex, m_acquiredImage.sizey);
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 1002, tr("copy buffer during getVal of Vistek can not be executed, since no DataType unknown.").toLatin1().data());
            }

            m_acquiredImage.status = asNoImageAcquired; //release frame
            m_acquiredImage.mutex.unlock();
        }
        else if (m_acquiredImage.sizex == asWaitingForTransfer)
        {
            retValue += ito::RetVal(ito::retError, 0, tr("invalid image data").toLatin1().data());
        }
        else if (m_acquiredImage.status == asTimeout)
        {
            retValue += ito::RetVal(ito::retError, 0, tr("timeout while retrieving image").toLatin1().data());
        }
        else if (m_acquiredImage.status >= asOtherError)
        {
            int offset = m_acquiredImage.status - asOtherError;
            switch (offset)
            {
                case SVGigE_SIGNAL_FRAME_COMPLETED: retValue += ito::RetVal(ito::retError,1, tr("new image available, transfer was successful").toLatin1().data()); break;
                case SVGigE_SIGNAL_FRAME_ABANDONED: retValue += ito::RetVal(ito::retError,2, tr("an image could not be completed in time and was therefore abandoned").toLatin1().data()); break;
                case SVGigE_SIGNAL_END_OF_EXPOSURE: retValue += ito::RetVal(ito::retError,3, tr("end of exposure is currently mapped to transfer started").toLatin1().data()); break;
                case SVGigE_SIGNAL_BANDWIDTH_EXCEEDED: retValue += ito::RetVal(ito::retError,4, tr("available network bandwidth has been exceeded").toLatin1().data()); break;
                case SVGigE_SIGNAL_OLD_STYLE_DATA_PACKETS: retValue += ito::RetVal(ito::retError,5, tr("driver problem due to old-style driver behavior (prior to 2003, not WDM driver)").toLatin1().data()); break;
                case SVGigE_SIGNAL_TEST_PACKET: retValue += ito::RetVal(ito::retError,6, tr("a test packet arrived").toLatin1().data()); break;
                case SVGigE_SIGNAL_CAMERA_IMAGE_TRANSFER_DONE: retValue += ito::RetVal(ito::retError,7, tr("the camera has finished an image transfer").toLatin1().data()); break;
                case SVGigE_SIGNAL_CAMERA_CONNECTION_LOST: retValue += ito::RetVal(ito::retError,8, tr("connection to camera got lost").toLatin1().data()); break;
                case SVGigE_SIGNAL_MULTICAST_MESSAGE: retValue += ito::RetVal(ito::retError,9, tr("an exceptional situation occurred during a multicast transmission").toLatin1().data()); break;
                case SVGigE_SIGNAL_FRAME_INCOMPLETE: retValue += ito::RetVal(ito::retError,10, tr("a frame could not be properly completed").toLatin1().data()); break;
                case SVGigE_SIGNAL_MESSAGE_FIFO_OVERRUN: retValue += ito::RetVal(ito::retError,11, tr("a next entry was put into the message FIFO before the old one was released").toLatin1().data()); break;
                case SVGigE_SIGNAL_CAMERA_SEQ_DONE: retValue += ito::RetVal(ito::retError,12, tr("the camera has finished a shutter sequence").toLatin1().data()); break;
                case SVGigE_SIGNAL_CAMERA_TRIGGER_VIOLATION: retValue += ito::RetVal(ito::retError,13, tr("the camera detected a trigger violation").toLatin1().data()); break;
                default: retValue += ito::RetVal(ito::retError, 0, tr("any error occurred").toLatin1().data()); break;
            }
        }
        else if (m_acquiredImage.status == asConnectionLost)
        {
            retValue += ito::RetVal(ito::retError, 0, tr("connection to camera lost").toLatin1().data());
        }
        else
        {
            retValue += ito::RetVal::format(ito::retError, 0, tr("error while retrieving image: %i").toLatin1().data(), (int)(m_acquiredImage.status)); //camera data could not be received", m_acquiredImage.status); //ito::RetVal(ito::retError, 1002, tr("getVal of Vistek can not be executed, since no Data has been acquired.").toLatin1().data());
        }
    }



    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Returns the grabbed camera frame.
/*!
    This method copies the recently grabbed camera frame to the given DataObject. Therefore this camera size must fit to the data structure of the
    DataObject.

    \note This method is similar to VideoCapture::retrieve() of openCV

    \param [in,out] vpdObj is the pointer to a given dataObject (this pointer should be cast to ito::DataObject*) where the acquired data is copied to.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if everything is ok, retError is camera has not been started or no data has been acquired by the method acquire.
    \sa DataObject, acquire, retrieveData
*/
ito::RetVal Vistek::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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
            retValue += sendDataToListeners(0);

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
ito::RetVal Vistek::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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
        retValue += checkData(dObj);  //intermediate buffer is already ok
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
//! This method is only called during init and initializes the camera with the number CameraNumber.
/*!
    If the camera has no valid IP address, it is forced to a new one. The DataCallback for the camera is registered here.
    \sa DataCallback, init
*/
ito::RetVal Vistek::initCamera(int CameraNumber)
{
    // Some Variables that might be needed
    SVGigE_RETURN SVGigERet;
    unsigned int IPAddress = 0;
    unsigned int SubnetMask = 0;
    char Msg[256];
    ito::RetVal retval;

    // Check if there already is an open handle for a camera
    if (m_cam != SVGigE_NO_CAMERA)
    {
        // TODO: Shutdown the cam first.
        std::cout << "Camera handle exists, trying to close an existing connection." << std::endl;
        StreamingChannel_delete(m_streamingChannel);
        Camera_closeConnection(m_cam);
        m_cam = SVGigE_NO_CAMERA;
    }

    // Get a Camera Handle
    std::cout << "Getting camera handle ... " << std::endl;
    m_cam = CameraContainer_getCamera(m_pVistekContainer->getCameraContainerHandle(), CameraNumber);
    if (SVGigE_NO_CAMERA == m_cam)
    {
        return ito::RetVal(ito::retError, 1004, tr("Requested camera could not be selected.").toLatin1().data());
    }
    std::cout << "done!\n" << std::endl;

    // Open the camera connection
    std::cout << "Opening camera connection ... " << std::endl;
    if (SVGigE_SUCCESS != Camera_openConnection(m_cam, 30))
    {
        std::cout << "\nOpening camera failed. Trying to enforce valid network settings.\n" << std::endl;
        // Check for valid network settings & doing necessary steps
        SVGigERet = Camera_forceValidNetworkSettings(m_cam, &IPAddress, &SubnetMask);
        if (SVGigERet == SVGigE_SVCAM_STATUS_CAMERA_OCCUPIED)
        {
            retval += checkError(tr("Requested camera is occupied by another application").toLatin1().data(), SVGigERet);
        }
        else
        {
            retval += checkError(tr("Enforcing valid network settings failed").toLatin1().data(), SVGigERet);
        }

        if (!retval.containsError())
        {

            // Try to open the connection again
            SVGigERet = Camera_openConnection(m_cam, 30);
            retval += checkError(tr("Selected camera could not be opened.").toLatin1().data(), SVGigERet);

            // print out the new valid network settings
            memset(Msg,0,sizeof(Msg));
            sprintf_s(Msg, 255, "Camera has been forced to a valid IP '%d.%d.%d.%d'\n",
                (IPAddress >> 24)%256, (IPAddress >> 16)%256, (IPAddress >> 8)%256, IPAddress%256);
            std::cout << Msg << std::endl;
        }
    }

    if (!retval.containsError())
    {
        //check some necessary features
        if (!Camera_isCameraFeature(m_cam, CAMERA_FEATURE_SOFTWARE_TRIGGER))
        {
            retval += ito::RetVal(ito::retError, 0, tr("Camera does not support software triggers. This camera cannot be used by this plugin.").toLatin1().data());
        }
    }

    if (!retval.containsError())
    {
        retval += checkError("",Camera_setAcquisitionMode(m_cam, ACQUISITION_MODE_NO_ACQUISITION));
        retval += checkError("",Camera_setAcquisitionControl(m_cam, ACQUISITION_CONTROL_STOP));

        Camera_registerForLogMessages(m_cam,m_params["logLevel"].getVal<int>(), "Vistek_SVGigE.log",NULL);

        m_features.adjustExposureTime = Camera_isCameraFeature(m_cam, CAMERA_FEATURE_EXPOSURE_TIME);
        m_features.adjustGain = Camera_isCameraFeature(m_cam, CAMERA_FEATURE_GAIN);
        m_features.adjustBinning = Camera_isCameraFeature(m_cam, CAMERA_FEATURE_BINNING);
        m_features.adjustOffset = Camera_isCameraFeature(m_cam, CAMERA_FEATURE_ADCOFFSET);
        m_features.has8bit = Camera_isCameraFeature(m_cam, CAMERA_FEATURE_COLORDEPTH_8BPP);
        m_features.has10bit = Camera_isCameraFeature(m_cam, CAMERA_FEATURE_COLORDEPTH_10BPP);
        m_features.has12bit = Camera_isCameraFeature(m_cam, CAMERA_FEATURE_COLORDEPTH_12BPP);
        m_features.has16bit = Camera_isCameraFeature(m_cam, CAMERA_FEATURE_COLORDEPTH_16BPP);

        bool autoVal;
        float valMin, valMax, val;

        //check auto gain/exposure
        //set auto gain to false
        Camera_getAutoGainEnabled(m_cam, &autoVal);
        if (autoVal)
        {
            std::cout << "auto gain has been enabled, will be disabled" << std::endl;
            Camera_setAutoGainEnabled(m_cam, false);
        }

        //obtain gain value
        Camera_getGainMax(m_cam, &valMax);
        Camera_getGain(m_cam, &val);
        Camera_getGainIncrement(m_cam, &m_gainIncrement);
        ito::DoubleMeta *dm = (ito::DoubleMeta*)(m_params["gain"].getMeta());
        dm->setMin(0.0);
        dm->setMax(valMax);
        m_params["gain"].setVal<double>(val);
        if (m_features.adjustGain == false)
        {
            m_params["gain"].setFlags(ito::ParamBase::Readonly);
        }

        //obtain offset value
        Camera_getOffset(m_cam, &val);
        Camera_getOffsetMax(m_cam, &valMax);
        dm = (ito::DoubleMeta*)(m_params["offset"].getMeta());
        dm->setMin(0.0);
        dm->setMax(valMax);
        m_params["offset"].setVal<double>(val);
        if (m_features.adjustOffset == false)
        {
            m_params["offset"].setFlags(ito::ParamBase::Readonly);
        }

        //obtain exposure value
        Camera_getExposureTimeRange(m_cam, &valMin, &valMax);
        Camera_getExposureTime(m_cam, &val);
        Camera_getExposureTimeIncrement(m_cam, &m_exposureIncrement);
        m_exposureIncrement /= 1.e6;
        dm = (ito::DoubleMeta*)(m_params["exposure"].getMeta());
        dm->setMin(valMin / 1.e6);
        dm->setMax(valMax / 1.e6);
        m_params["exposure"].setVal<double>(val / 1.e6);
        if (m_features.adjustExposureTime == false)
        {
            m_params["exposure"].setFlags(ito::ParamBase::Readonly);
        }

        dm = (ito::DoubleMeta*)(m_params["integration_time"].getMeta());
        dm->setMin(valMin / 1.e6);
        dm->setMax(valMax / 1.e6);
        m_params["integration_time"].setVal<double>(val / 1.e6);
        if (m_features.adjustExposureTime == false)
        {
            m_params["integration_time"].setFlags(ito::ParamBase::Readonly);
        }

        //binning and bpp is read in registerCallbacks later
        if (m_features.adjustBinning == false)
        {
            m_params["binning"].setFlags(ito::ParamBase::Readonly);
        }

        std::cout << "done!\n" << std::endl;

        retval += startStreamAndRegisterCallbacks();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Vistek::updateTimestamp()
{
    double timestamp = (MessageTimestampStartOfTransfer - MessageTimestampLastStartOfTransfer) * 1000;
    m_params["timestamp"].setVal<double>(timestamp);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Vistek::stopStreamAndDeleteCallbacks()
{
    ito::RetVal retValue;
    //1. first check if there is already a stream initialized and if so, delete it
    if (m_streamingChannel != SVGigE_NO_STREAMING_CHANNEL)
    {
        //stop camera
        //Camera_setAcquisitionMode(m_cam, ACQUISITION_MODE_NO_ACQUISITION);
        //Camera_setAcquisitionControl(m_cam, ACQUISITION_CONTROL_STOP);

        //invalidate image buffer
        m_acquiredImage.mutex.lock();
        m_acquiredImage.status = asNoImageAcquired;
        m_acquiredImage.mutex.unlock();
        // Unregister callbacks
        Stream_unregisterEventCallback(m_streamingChannel, m_eventID, &MessageCallback);
        Stream_closeEvent(m_streamingChannel, m_eventID);
        m_eventID = SVGigE_NO_EVENT;

        // delete stream
        StreamingChannel_delete(m_streamingChannel);
        m_streamingChannel = SVGigE_NO_STREAMING_CHANNEL;
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Vistek::startStreamAndRegisterCallbacks()
{
    ito::RetVal retval;
    SVGigE_RETURN SVGigERet;
    ParamMapIterator it;


    //1. first check if there is already a stream initialized and if so, delete it
    if (m_streamingChannel != SVGigE_NO_STREAMING_CHANNEL)
    {
        retval += stopStreamAndDeleteCallbacks();
    }

    //2. sychronize current settings of camera with m_params
    // Obtain geometry data
    int width, height;
    SVGigERet = Camera_getSizeX(m_cam, &width);
    retval += checkError(tr("Error getting image size").toLatin1().data(), SVGigERet);
    SVGigERet = Camera_getSizeY(m_cam, &height);
    retval += checkError(tr("Error getting image size").toLatin1().data(), SVGigERet);

    //Obtain ROI
    int sizexinc, sizeyinc, offsetxinc, offsetyinc;
    int sizexmin, sizeymin, sizexmax, sizeymax;
    int sizex, sizey, offsetx, offsety;
    Camera_getAreaOfInterestIncrement(m_cam, &sizexinc, &sizeyinc, &offsetxinc, &offsetyinc);
    Camera_getAreaOfInterestRange(m_cam, &sizexmin, &sizeymin, &sizexmax, &sizeymax);
    Camera_getAreaOfInterest(m_cam, &sizex, &sizey, &offsetx, &offsety);

    it = m_params.find("roi");
    ito::RectMeta *rm = new ito::RectMeta(ito::RangeMeta(0, sizexmax - 1, offsetxinc, sizexmin, sizexmax, sizexinc), ito::RangeMeta(0, sizeymax - 1, offsetyinc, sizeymin, sizeymax, sizeyinc), "ImageFormatControl");
    it->setMeta(rm, true);
    int roi[] = { offsetx, offsety, sizex, sizey };
    it->setVal<int*>(roi, 4);

    it = m_params.find("sizex");
    it->setVal<int>(sizex);
    ito::IntMeta *im = it->getMetaT<ito::IntMeta>();
    im->setMin(sizexmin);
    im->setMax(sizexmax);
    im->setStepSize(sizexinc);

    it = m_params.find("sizey");
    it->setVal<int>(sizey);
    im = it->getMetaT<ito::IntMeta>();
    im->setMin(sizeymin);
    im->setMax(sizeymax);
    im->setStepSize(sizeyinc);

    //obtain binning mode
    BINNING_MODE currentBinning;
    Camera_getBinningMode(m_cam, &currentBinning);
    m_params["binning"].setVal<int>((int)currentBinning);

    GVSP_PIXEL_TYPE pixeltype;
    Camera_getPixelType(m_cam, &pixeltype);

    switch (pixeltype)
    {
    case GVSP_PIX_MONO8:
        m_params["bpp"].setVal<int>(8);
        break;
    /*case GVSP_PIX_MONO10: //not supported by current Vistek driver
    case GVSP_PIX_MONO10_PACKED:
        m_params["bpp"].setVal<int>(10);
        break;*/
    case GVSP_PIX_MONO12:
    case GVSP_PIX_MONO12_PACKED:
        m_params["bpp"].setVal<int>(12);
        break;
    case GVSP_PIX_MONO16:
        m_params["bpp"].setVal<int>(16);
        break;
    default:
        retval += ito::RetVal(ito::retError, 0, tr("given pixeltype not supported. Supported is only MONO8, MONO12, MONO12_PACKED and MONO16").toLatin1().data());
        break;
    }

    // Get Tick Frequency
    SVGigERet = Camera_getTimestampTickFrequency(m_cam, &TimestampTickFrequency);
    if (SVGigERet != SVGigE_SUCCESS)
    {
        std::cout<<"TimestampTickFrequency: "<<TimestampTickFrequency<<"\n";
    }

    //3. determine (and set) maximal possible network packet size (if you have jumbo-frames enabled)
    Camera_setMulticastMode(m_cam, MULTICAST_MODE_NONE); //image is only delivered to this computer, not to other computers... therefore we can also (usually) use the maximum available package size

    int maximalPacketSize;
    SVGigERet = Camera_evaluateMaximalPacketSize(m_cam, &maximalPacketSize);
    retval += checkError(tr("maximal packet size determination").toLatin1().data(), SVGigERet);

    if (!retval.containsError())
    {
        ((ito::IntMeta*)(m_params["streamingPacketSize"].getMeta()))->setMax(maximalPacketSize);
        if (m_params["streamingPacketSize"].getVal<int>() > maximalPacketSize)
        {
            m_params["streamingPacketSize"].setVal<int>(maximalPacketSize); //maximum is already set as default
        }
        else if (m_params["streamingPacketSize"].getVal<int>() == -1)
        {
            m_params["streamingPacketSize"].setVal<int>(maximalPacketSize); //maximum is already set as default
        }
        //else
        //{
            SVGigERet = Camera_setStreamingPacketSize(m_cam, m_params["streamingPacketSize"].getVal<int>());
            retval += checkError(tr("set streaming packet size").toLatin1().data(), SVGigERet);
        //}
    }

    //4. Register data callback
    if (!retval.containsError())
    {
        // Register data callback
        std::cout << "Registering data callback ..." << std::endl;

        try
        {
            SVGigERet = StreamingChannel_create (&m_streamingChannel,                                // a streaming channel handle will be returned
                                        m_pVistekContainer->getCameraContainerHandle(),                 // a valid camera container client handle
                                        m_cam,                                                          // a valid camera handle
                                        m_numBuf,                                                              // buffer count 0 => 3 buffers (big buffer is necessary in order to avoid huge timeouts)
                                        &DataCallback,                                                  // callback function pointer where datas are delivered to
                                        this);                                                          // current class pointer will be passed through as context
        }
        catch(std::bad_alloc &/*ba*/)
        {
            retval += ito::RetVal::format(ito::retError, 0, tr("memory allocation error when creating stream with %i buffers. Retry and use less buffers").toLatin1().data(), m_numBuf);
        }

        // Check if data callback registration was successful
        retval += checkError(tr("Streaming channel creation failed").toLatin1().data(), SVGigERet);

        if (SVGigERet != SVGigE_SUCCESS)
        {
            m_streamingChannel = SVGigE_NO_STREAMING_CHANNEL;
        }
    }

    //5. register events
    if (!retval.containsError())
    {
        std::cout << "done!\n" << std::endl;

        // Create event
        SVGigERet = Stream_createEvent(m_streamingChannel,&m_eventID,100);
        retval += checkError(tr("Event creation failed").toLatin1().data(), SVGigERet);

        if (!retval.containsError())
        {
            // Register messages
            Stream_addMessageType(m_streamingChannel,m_eventID,SVGigE_SIGNAL_FRAME_COMPLETED);
            Stream_addMessageType(m_streamingChannel,m_eventID,SVGigE_SIGNAL_START_OF_TRANSFER);
            Stream_addMessageType(m_streamingChannel,m_eventID,SVGigE_SIGNAL_CAMERA_TRIGGER_VIOLATION);
            Stream_addMessageType(m_streamingChannel,m_eventID,SVGigE_SIGNAL_FRAME_ABANDONED);
            Stream_addMessageType(m_streamingChannel,m_eventID,SVGigE_SIGNAL_END_OF_EXPOSURE);

            Stream_addMessageType(m_streamingChannel,m_eventID,SVGigE_SIGNAL_BANDWIDTH_EXCEEDED);
            Stream_addMessageType(m_streamingChannel,m_eventID,SVGigE_SIGNAL_OLD_STYLE_DATA_PACKETS);
            Stream_addMessageType(m_streamingChannel,m_eventID,SVGigE_SIGNAL_TEST_PACKET);
            Stream_addMessageType(m_streamingChannel,m_eventID,SVGigE_SIGNAL_CAMERA_IMAGE_TRANSFER_DONE);
            Stream_addMessageType(m_streamingChannel,m_eventID,SVGigE_SIGNAL_CAMERA_CONNECTION_LOST);
            Stream_addMessageType(m_streamingChannel,m_eventID,SVGigE_SIGNAL_MULTICAST_MESSAGE);
            Stream_addMessageType(m_streamingChannel,m_eventID,SVGigE_SIGNAL_FRAME_INCOMPLETE);
            Stream_addMessageType(m_streamingChannel,m_eventID,SVGigE_SIGNAL_MESSAGE_FIFO_OVERRUN);
            Stream_addMessageType(m_streamingChannel,m_eventID,SVGigE_SIGNAL_CAMERA_SEQ_DONE);
            //Stream_addMessageType(m_streamingChannel,m_eventID,SVGigE_SIGNAL_CAMERA_TRIGGER_VIOLATION);

            // Register message callback
            std::cout << "Registering message callback ..." << std::endl;
            SVGigERet = Stream_registerEventCallback(m_streamingChannel, m_eventID, &MessageCallback, this);
            // Check if message callback registration was successful
            retval += checkError(tr("Message callback registration failed").toLatin1().data(), SVGigERet);
        }
    }

    //6. prepare m_data and intermediate buffer structure
    if (!retval.containsError())
    {
        retval += checkData();

        m_acquiredImage.mutex.lock();

        if (m_params["bpp"].getVal<int>() == 8)
        {
            m_acquiredImage.buffer.resize(sizex * sizey); //8bit content
        }
        else
        {
            m_acquiredImage.buffer.resize(2 * sizex * sizey); //2*8bit = 16bit content
        }

        m_acquiredImage.status = asNoImageAcquired;

        m_acquiredImage.mutex.unlock();
    }

    if (!retval.containsError())
    {
        std::cout << "done!\n" << std::endl;
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Vistek::dockWidgetVisibilityChanged(bool visible)
{
    if (getDockWidget())
    {
        DockWidgetVistek *dw = qobject_cast<DockWidgetVistek*>(getDockWidget()->widget());
        if (visible)
        {
            connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), dw, SLOT(parametersChanged(QMap<QString, ito::Param>)));

            emit parametersChanged(m_params);
        }
        else
        {
            disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), dw, SLOT(parametersChanged(QMap<QString, ito::Param>)));
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
SVGigE_RETURN __stdcall DataCallback(Image_handle data, void* context)
{
    //qDebug() << "DataCallback: threadID:" << QThread::currentThreadId();
    // Check for valid context
    Vistek *v = reinterpret_cast<Vistek*>(context);

    if (v)
    {
        if (v->m_acquiredImage.mutex.tryLock(1000))
        {

            qDebug() << Image_getDataPointer(data) << Image_getSignalType(data) << Image_getSizeX(data) << Image_getSizeY(data);

            if (v == NULL)
            {
                v->m_acquiredImage.mutex.unlock();
                return SVGigE_INVALID_PARAMETERS;
            }

            if (Image_getDataPointer(data) == NULL)
            {
                if (Image_getSignalType(data) == SVGigE_SIGNAL_CAMERA_CONNECTION_LOST)
                {
                    v->m_acquiredImage.status = Vistek::asConnectionLost;
                }
                else
                {
                    v->m_acquiredImage.status = Vistek::asOtherError + Image_getSignalType(data);
                }
                v->m_acquiredImage.sizex = -1;
                v->m_acquiredImage.sizey = -1;
                v->m_acquiredImage.mutex.unlock();
                return SVGigE_SUCCESS;
            }

            if (v->m_acquiredImage.status == Vistek::asNoImageAcquired || v->m_acquiredImage.status == Vistek::asTimeout)
            {
                //nobody is waiting for an image -> kill it
                Image_release(data);
                v->m_acquiredImage.mutex.unlock();
                return SVGigE_IMAGE_SKIPPED_IN_CALLBACK;
            }
            else
            {
                //the status is set by the messaging system
                v->m_acquiredImage.pixelType = Image_getPixelType(data);
                v->m_acquiredImage.sizex = Image_getSizeX(data);
                v->m_acquiredImage.sizey = Image_getSizeY(data);
                v->m_acquiredImage.dataID = Image_getImageID(data);
                v->m_acquiredImage.timestamp = Image_getTimestamp(data);
                v->m_acquiredImage.transferTime = Image_getTransferTime(data);
                v->m_acquiredImage.packetCount = Image_getPacketCount(data);

                int elems = v->m_acquiredImage.sizex * v->m_acquiredImage.sizey;

                if (v->m_acquiredImage.pixelType == GVSP_PIX_MONO8)
                {
                    if (v->m_acquiredImage.buffer.size() != elems)
                    {
                        v->m_acquiredImage.buffer.resize(elems);
                    }
                    memcpy(v->m_acquiredImage.buffer.data(), Image_getDataPointer(data), sizeof(ito::uint8) * elems);
                }
                else if (v->m_acquiredImage.pixelType == GVSP_PIX_MONO12 || v->m_acquiredImage.pixelType == GVSP_PIX_MONO12_PACKED)
                {
                    if (v->m_acquiredImage.buffer.size() != 2 * elems)
                    {
                        v->m_acquiredImage.buffer.resize(2 * elems);
                    }
                    Image_getImage12bitAs16bit(data, v->m_acquiredImage.buffer.data(), sizeof(ito::uint16) * elems);

                    //make a 4bit right shift to push all values in a range [0,4096]
                    ito::uint16 *val = (ito::uint16*)v->m_acquiredImage.buffer.data();
                    for (size_t i = 0; i < elems; ++i)
                    {
                        *val >>= 4;
                        val++;
                    }
                }
                else if (v->m_acquiredImage.pixelType == GVSP_PIX_MONO16)
                {
                    if (v->m_acquiredImage.buffer.size() != 2 * elems)
                    {
                        v->m_acquiredImage.buffer.resize(2 * elems);
                    }
                    memcpy(v->m_acquiredImage.buffer.data(), Image_getDataPointer(data), sizeof(ito::uint16) * elems);
                }
                else
                {
                    Image_release(data);
                    v->m_acquiredImage.mutex.unlock();
                    return SVGigE_ERROR;
                }

                v->m_acquiredImage.status = Vistek::asImageReady;

                Image_release(data);
            }

            v->m_acquiredImage.mutex.unlock();
        }
        else
        {
            qDebug() << "DataCallback: could not lock mutex";
        }
    }

    return SVGigE_SUCCESS;
};

//----------------------------------------------------------------------------------------------------------------------------------
SVGigE_RETURN __stdcall MessageCallback(Event_handle eventID, void* context)
{
    // Check for valid context
    Vistek *v = reinterpret_cast<Vistek*>(context);
    if (v == NULL)
        return SVGigE_SUCCESS;

    Message_handle MessageID;
    SVGigE_SIGNAL_TYPE MessageType;

    // Get the signal
    if (SVGigE_SUCCESS != Stream_getMessage(v->m_streamingChannel, eventID, &MessageID, &MessageType))
    {
        qDebug() << "message callback. error in getMessage";
        return SVGigE_SUCCESS;
    }

    //qDebug() << "message:" << MessageType;

    // Get Message timestamp
    if (MessageType == SVGigE_SIGNAL_START_OF_TRANSFER)
    {
        v->MessageTimestampLastStartOfTransfer = v->MessageTimestampStartOfTransfer;
        Stream_getMessageTimestamp(v->m_streamingChannel,eventID,MessageID,&v->MessageTimestampStartOfTransfer);
        v->updateTimestamp();
    }
    if (MessageType == SVGigE_SIGNAL_FRAME_COMPLETED)
    {
        v->m_acquiredImage.mutex.lock();
        v->m_acquiredImage.frameCompleted = true;
        v->m_acquiredImage.mutex.unlock();
        //Stream_getMessageTimestamp(v->m_streamingChannel,eventID,MessageID,&v->MessageTimestampFrameCompleted);
    }
    if (MessageType == SVGigE_SIGNAL_CAMERA_TRIGGER_VIOLATION)
    {
        // Count trigger violations in current streaming channel
        v->TriggerViolationCount++;
    }

    //if (MessageType == SVGigE_SIGNAL_FRAME_ABANDONED)
    //{
    //    // Not implemented yet
    //}
    if (MessageType == SVGigE_SIGNAL_END_OF_EXPOSURE)
    {
        //Stream_getMessageTimestamp(v->m_streamingChannel,eventID,MessageID, &v->MessageTimestampEndOfExposure);
    }

    // Release message
    Stream_releaseMessage(v->m_streamingChannel,eventID,MessageID);

    return SVGigE_SUCCESS;
}

/* ********************************************************************
    Plugin "QCam" for itom software
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

#include "QCam.h"
#include "dockWidgetQCam.h"

#include <QFile>
#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>

#include "pluginVersion.h"
#include "gitVersion.h"

Q_DECLARE_METATYPE(ito::DataObject)

int QCam::instanceCounter = 0;

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QCamInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(QCam)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QCamInterface::closeThisInst(ito::AddInBase **addInInst)
{
   REMOVE_PLUGININSTANCE(QCam)
   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
QCamInterface::QCamInterface() : AddInInterfaceBase()
{
   m_type = ito::typeDataIO | ito::typeGrabber;
   setObjectName("QCam");

    m_description = QObject::tr("Firewire QCam cameras from QImaging");

    //for the docstring, please don't set any spaces at the beginning of the line.
    m_detaildescription = QObject::tr("This plugin is currently under development.");
    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
    m_aboutThis = QObject::tr(GITVERSION);
}

//----------------------------------------------------------------------------------------------------------------------------------
QCamInterface::~QCamInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------------------
QCam::QCam() :
    AddInGrabber(),
    m_frameCallbackRetVal(ito::retOk),
    m_frameCallbackFrameIdx(-1),
    m_waitingForAcquire(false)
{
    // make sure the structs are filled with 0 so we can check for allocated memory later
    for (int nf = 0; nf < NUMBERBUFFERS; nf++)
    {
        memset((void*)&m_frames[nf], 0, sizeof(QCam_Frame));
    }

    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly | ito::ParamBase::NoAutosave, "QCam", NULL);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("integration_time", ito::ParamBase::Double, 0.0, 1.0, 0.0, tr("Integration time of CCD programmed in s").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("gain", ito::ParamBase::Double, 0.0, 1.0, 0.0, tr("Gain").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("offset", ito::ParamBase::Double, 0.0, 1.0, 0.0, tr("Offset").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("x0", ito::ParamBase::Int, 0, 1391, 0, tr("first pixel of ROI in x-direction").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("y0", ito::ParamBase::Int, 0, 1023, 0, tr("first pixel of ROI in y-direction").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("x1", ito::ParamBase::Int, 0, 1391, 1391, tr("last pixel of ROI in x-direction").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("y1", ito::ParamBase::Int, 0, 1039, 1039, tr("last pixel of ROI in y-direction").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizex", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 1340, 1040, tr("width of ROI").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizey", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 1340, 1040, tr("height of ROI").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("bpp", ito::ParamBase::Int, 8, 16, 8, tr("bit depth per pixel").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    paramVal = ito::Param("cooled", ito::ParamBase::Int, 0, 1, 1, tr("CCD cooler").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

    if (hasGuiSupport())
    {
        //now create dock widget for this plugin
        DockWidgetQCam *dockWidget = new DockWidgetQCam();
        connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), dockWidget, SLOT(valuesChanged(QMap<QString, ito::Param>)));
        connect(dockWidget, SIGNAL(changeParameters(QMap<QString, ito::ParamBase>)), this, SLOT(updateParameters(QMap<QString, ito::ParamBase>)));

        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dockWidget);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
QCam::~QCam()
{
    for (int nf = 0; nf < NUMBERBUFFERS; nf++)
    {
        if (m_frames[nf].pBuffer)
        {
            free(m_frames[nf].pBuffer);
            m_frames[nf].pBuffer = NULL;
        }
    }
    m_params.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QCam::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);

    ito::RetVal retValue(ito::retOk);
    QCam_CamListItem  camList[10];
    unsigned long camListLen = 10;
    m_camHandle = 0;

    if (instanceCounter == 0)
    {
        retValue += errorCheck(QCam_LoadDriver());
    }
    instanceCounter++;

    if (!retValue.containsError())
    {
        //get list of cameras
        retValue += errorCheck(QCam_ListCameras(camList, &camListLen)); //camListLen is now the number of cameras available. It may be larger than your QCam_CamListItem array length!
        if (!retValue.containsError())
        {
            if (camListLen > 0)
            {
                int i = 0;
                while ((i < 10) && (camList[i].isOpen > 0))
                {
                    ++i;
                }

                if (i < 10)
                {
                    retValue += errorCheck(QCam_OpenCamera(camList[i].cameraId, &m_camHandle));
                }
                else
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("the first 10 connected cameras are already opened.").toLatin1().data());
                }
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 0, tr("no QCam cameras are connected to this computer").toLatin1().data());
            }
        }
    }

    if (!retValue.containsError())
    {
        retValue += errorCheck(QCam_SetStreaming(m_camHandle, true));
    }

    if (!retValue.containsError())
    {
        unsigned long cameraType;
        QCam_GetInfo(m_camHandle, qinfCameraType,  &cameraType);
        m_identifier = QString("type: %1").arg(cameraType);

        //initialize the setting structure
        m_camSettings.size = sizeof(m_camSettings);

        //read the default settings the camera has
        QCam_ReadDefaultSettings(m_camHandle, &m_camSettings);

        QCam_Abort(m_camHandle);

        QCam_SetParam(&m_camSettings, qprmTriggerType, qcTriggerSoftware);

        //get size and bit depth of camera
        unsigned long size, height, width;
        unsigned long maxBitDepth;
        double integration_time;
        unsigned long long integ_time, integ_timeMax, integ_timeMin;

        QCam_GetInfo(m_camHandle, qinfBitDepth, &maxBitDepth);

        if (maxBitDepth <= 8)
        {
            QCam_SetParam(&m_camSettings, qprmImageFormat, qfmtMono8);
        }
        else
        {
            QCam_SetParam(&m_camSettings, qprmImageFormat, qfmtMono16);
        }

        //adjust range of bpp-param
        ito::IntMeta *paramMeta = (ito::IntMeta*)(m_params["bpp"].getMeta());
        paramMeta->setMin(8);
        paramMeta->setMax(maxBitDepth);
        m_params["bpp"].setVal<int>(maxBitDepth);

        QCam_GetInfo(m_camHandle, qinfImageSize, &size);
        QCam_GetInfo(m_camHandle, qinfImageHeight, &height);
        QCam_GetInfo(m_camHandle, qinfImageWidth, &width);

        QCam_GetParam64(&m_camSettings, qprm64Exposure, (unsigned long long *)&integ_time);
        QCam_GetParam64Max(&m_camSettings, qprm64Exposure, (unsigned long long *)&integ_timeMax);
        QCam_GetParam64Min(&m_camSettings, qprm64Exposure, (unsigned long long *)&integ_timeMin);
        integration_time = double(integ_time)/1e9;

        paramMeta = (ito::IntMeta*)(m_params["x0"].getMeta());
        paramMeta->setMin(0);
        paramMeta->setMax(width);
        m_params["x0"].setVal<int>(0);

        paramMeta = (ito::IntMeta*)(m_params["x1"].getMeta());
        paramMeta->setMin(0);
        paramMeta->setMax(width);
        m_params["x1"].setVal<int>(width-1);

        paramMeta = (ito::IntMeta*)(m_params["y0"].getMeta());
        paramMeta->setMin(0);
        paramMeta->setMax(height);
        m_params["y0"].setVal<int>(0);

        paramMeta = (ito::IntMeta*)(m_params["y1"].getMeta());
        paramMeta->setMin(0);
        paramMeta->setMax(height);
        m_params["y1"].setVal<int>(height-1);

        paramMeta = (ito::IntMeta*)(m_params["sizex"].getMeta());
        paramMeta->setMin(0);
        paramMeta->setMax(width);
        m_params["sizex"].setVal<int>(width);

        paramMeta = (ito::IntMeta*)(m_params["sizey"].getMeta());
        paramMeta->setMin(0);
        paramMeta->setMax(height);
        m_params["sizey"].setVal<int>(height);

        ito::DoubleMeta *paramMetadouble;
        paramMetadouble = (ito::DoubleMeta*)(m_params["integration_time"].getMeta());
        paramMetadouble->setMin(double(integ_timeMin)/1e9);
        paramMetadouble->setMax(double(integ_timeMax)/1e9);
        m_params["integration_time"].setVal<double>(integration_time);

        //ask camera for gain, intensity...
        unsigned long gainMin, gainMax, gain;

        QCam_GetParamMax(&m_camSettings, qprmNormalizedGain, &gainMax);
        QCam_GetParamMin(&m_camSettings, qprmNormalizedGain, &gainMin);
        QCam_GetParam(&m_camSettings, qprmNormalizedGain, &gain);


        //transform normalized gain into 0-1-gain
        double gainDbl = (double)(gain - gainMin) / (double)(gainMax - gainMin);
        m_params["gain"].setVal<double>(gainDbl);

        retValue += checkData(); //resize the internal dataObject m_data to the right size and type

        //finally send the new settings to the camera
        retValue += errorCheck(QCam_SendSettingsToCam(m_camHandle,&m_camSettings));
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
ito::RetVal QCam::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if (m_timerID > 0)
    {
        killTimer(m_timerID);
        m_timerID=0;
    }

    //stop device, if not yet done (only if still started - counter > 0)
    if (grabberStartedCount() > 0)
    {
        setGrabberStarted(1); //force the counter to 0, such that stopDevice drops it to 0 and really stops the device!
        retValue += stopDevice(NULL);
    }

    if (m_camHandle)
    {
        retValue += errorCheck(QCam_SetStreaming(m_camHandle, false));
        retValue += errorCheck(QCam_CloseCamera(m_camHandle));
    }

    instanceCounter--;
    if (instanceCounter == 0)
    {
        QCam_ReleaseDriver();
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
    \details This method copies the complete tparam of the corresponding parameter to val

    \param [in,out] val  is a input of type::tparam containing name, value and further information
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk in case that everything is ok, else retError
    \sa ito::tParam, ItomSharedSemaphore
*/
ito::RetVal QCam::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
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
/*!
    \detail This method copies the value of val to to the m_params-parameter and sets the corresponding camera parameters.

    \param [in] val  is a input of type::tparam containing name, value and further information
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk in case that everything is ok, else retError
    \sa ito::tParam, ItomSharedSemaphore
*/
ito::RetVal QCam::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex;
    int index;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;

    bool needToReallocate = false; //true, if the image size, roi, bpp have been changed

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
        if (key == "gain")
        {
            unsigned long gainMax, gainMin;

            double gain = val->getVal<double>();
            QCam_GetParamMax(&m_camSettings, qprmNormalizedGain, &gainMax);
            QCam_GetParamMin(&m_camSettings, qprmNormalizedGain, &gainMin);

            //calculate normalized-gain from 0-1-gain
            gain = gainMin + gain * (gainMax - gainMin);

            retValue += errorCheck(QCam_SetParam(&m_camSettings, qprmNormalizedGain, gain));

            if (!retValue.containsError())
            {
                retValue += it->copyValueFrom(&(*val)); //copy obtained gain value to the internal m_params["gain"]
            }
        }
        else if (key == "integration_time")
        {
            unsigned long long expMax, expMin;

            double integration_time = val->getVal<double>();
            QCam_GetParam64Min(&m_camSettings, qprm64Exposure, (unsigned long long *)&expMax);
            QCam_GetParam64Max(&m_camSettings, qprm64Exposure, (unsigned long long *)&expMin);

            retValue += errorCheck(QCam_SetParam64(&m_camSettings, qprm64Exposure, uint64(integration_time*1000000000)));

            if (!retValue.containsError())
            {
                retValue += it->copyValueFrom(&(*val)); //copy obtained integration_time value to the internal m_params["integration_time"]
            }
        }
        else if (key == "x0") //x0 is the left border of the image
        {
            unsigned long  RoiXMax, RoiXMin;

            int x0 = val->getVal<int>(); //x0 is an integer

            if (x0 != it->getVal<int>()) //only do something if x0 really changed (it->... is pointing to m_params["x0"])
            {
                QCam_GetParamMin(&m_camSettings, qprmRoiX, (unsigned long *)&RoiXMax);
                QCam_GetParamMax(&m_camSettings, qprmRoiX, (unsigned long *)&RoiXMin);

                retValue += rangeCheck(RoiXMin, RoiXMax, x0, "x0");

                if (!retValue.containsError())
                {
                    retValue += errorCheck(QCam_SetParam(&m_camSettings, qprmRoiX, uint(x0)));
                }

                if (!retValue.containsError())
                {
                    // stop the camera, clear the buffers and send the parameter to the camera.  then restart the camera.
                    // if it was running live, restart that also
                    needToReallocate = true;
                    m_params["x0"].setVal<int>(x0);
                    int x1 = m_params["x1"].getVal<int>();
                    m_params["sizex"].setVal<int>(x1-x0+1);
                }
            }
        }
        else if (key == "y0") //y0 is the top of the image
        {
            unsigned long  RoiYMax, RoiYMin;

            int y0 = val->getVal<int>();

            if (y0 != it->getVal<int>()) //only do something if y0 really changed (it->... is pointing to m_params["y0"])
            {
                QCam_GetParamMin(&m_camSettings, qprmRoiY, (unsigned long *)&RoiYMax);
                QCam_GetParamMax(&m_camSettings, qprmRoiY, (unsigned long *)&RoiYMin);

                retValue += rangeCheck(RoiYMin, RoiYMax, y0, "y0");

                if (!retValue.containsError())
                {
                    retValue += errorCheck(QCam_SetParam(&m_camSettings, qprmRoiY, uint(y0)));
                }

                if (!retValue.containsError())
                {
                    // stop the camera, clear the buffers and send the parameter to the camera.  then restart the camera.
                    // if it was running live, restart that also
                    needToReallocate = true;
                    m_params["y0"].setVal<int>(y0);
                    int y1 = m_params["y1"].getVal<int>();
                    m_params["sizey"].setVal<int>(y1-y0+1);
                }
            }
        }
        else if (key == "x1") //x1 is the pixel-coordinate of the right border
        {
            unsigned long  roiWidthMin, roiWidthMax;

            int x1 = val->getVal<int>();
            if (x1 != it->getVal<int>()) //only do something if x1 really changed (it->... is pointing to m_params["x1"])
            {
                int newWidth = 1 + x1 - m_params["x0"].getVal<int>();

                QCam_GetParamMin(&m_camSettings, qprmRoiWidth, (unsigned long *)&roiWidthMin);
                QCam_GetParamMax(&m_camSettings, qprmRoiWidth, (unsigned long *)&roiWidthMax);

                retValue += rangeCheck(roiWidthMin, roiWidthMax, newWidth, "x1");

                if (!retValue.containsError())
                {
                    retValue += errorCheck(QCam_SetParam(&m_camSettings, qprmRoiWidth, uint(newWidth)));
                }

                if (!retValue.containsError())
                {
                    // stop the camera, clear the buffers and send the parameter to the camera.  then restart the camera.
                    // if it was running live, restart that also
                    needToReallocate = true;
                    m_params["x1"].setVal<int>(x1);
                    m_params["sizex"].setVal<int>(newWidth);
                }
            }
        }
        else if (key == "y1") //y1 is the pixel-coordinate of the bottom
        {
            unsigned long  roiHeightMin, roiHeightMax;

            int y1 = val->getVal<int>();

            if (y1 != it->getVal<int>()) //only do something if y1 really changed (it->... is pointing to m_params["y1"])
            {
                int newHeight = 1 + y1 - m_params["y0"].getVal<int>();
                QCam_GetParamMin(&m_camSettings, qprmRoiHeight, (unsigned  long *)&roiHeightMax);
                QCam_GetParamMax(&m_camSettings, qprmRoiHeight, (unsigned  long *)&roiHeightMin);

                retValue += rangeCheck(roiHeightMin, roiHeightMax, newHeight, "y1");

                if (!retValue.containsError())
                {
                    retValue += errorCheck(QCam_SetParam(&m_camSettings, qprmRoiHeight, uint(newHeight)));
                }

                if (!retValue.containsError())
                {
                    // stop the camera, clear the buffers and send the parameter to the camera.  then restart the camera.
                    // if it was running live, restart that also
                    needToReallocate = true;
                    m_params["y1"].setVal<int>(y1);
                    m_params["sizey"].setVal<int>(newHeight);
                }
            }
        }
        else if (key == "bpp")
        {
            retValue += ito::RetVal(ito::retError, 0, tr("not implemented yet. todo").toLatin1().data());
        }
        else
        {
            //all parameters that don't need further checks can simply be assigned
            //to the value in m_params (the rest is already checked above)
            retValue += it->copyValueFrom(&(*val));
        }
    }

    if (!retValue.containsError() && needToReallocate)
    {
        bool acquisitionInterrupted = false;
        if (grabberStartedCount() > 0)
        {
            QCam_Abort(m_camHandle);

            if (m_waitingForAcquire) acquisitionInterrupted = true;
            //m_frameCallbackFrameIdx = -1;
            //m_waitingForAcquire = false;

            // delete buffer(s)
            for (int i = 0; i < NUMBERBUFFERS; ++i)
            {
                if (m_frames[i].pBuffer)
                {
                    free(m_frames[i].pBuffer);
                }
                m_frames[i].pBuffer = NULL;
            }
        }

        retValue += errorCheck(QCam_SendSettingsToCam(m_camHandle, &m_camSettings));
        checkData(); //also resize the internal data object m_data to the new size given by sizex, sizey and bpp

        if (grabberStartedCount() > 0)
        {
            //reallocate buffer(s)
            unsigned long size;
            QCam_GetInfo(m_camHandle, qinfImageSize, &size);

            for (int i = 0; i < NUMBERBUFFERS; ++i)
            {
                //MUST be 4 byte aligned according to header file!
                if (m_frames[i].pBuffer)
                    free(m_frames[i].pBuffer);
                m_frames[i].pBuffer = calloc(size, sizeof(unsigned char));
                m_frames[i].bufferSize = size;
                memset(m_frames[i].pBuffer, 0, size * sizeof(unsigned char));
                QCam_QueueFrame(m_camHandle, &(m_frames[i]), qCamFrameCallback, qcCallbackDone, this, i);
            }

            if (acquisitionInterrupted)
            {
                retValue += acquire(0); //reacquire a new image, since we aborted the last acquisition
            }
        }
    }
    else if (!retValue.containsError() && !needToReallocate)
    {
        //only send the parameters, but camera must not be reallocated or stopped
        retValue += errorCheck(QCam_SendSettingsToCam(m_camHandle, &m_camSettings));
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
ito::RetVal QCam::rangeCheck(const unsigned long &min, const unsigned long &max, const unsigned long &value, const QByteArray &name)
{
    if (value > max || value < min)
    {
        return ito::RetVal::format(ito::retError, 0, tr("value '%s' out of bounds.").toLatin1().data(), name.data());
    }
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
//this method is called by the main thread!!! (not the camera thread)
const ito::RetVal QCam::showConfDialog(void)
{
    ito::RetVal retValue(ito::retOk);
    int bitppix_old = 12;
    int binning_old = 0;
    int bitppix_new = 12;
    int binning_new = 0;
    double offset_new = 0.0;

    DialogQCam *confDialog = new DialogQCam(this);

    connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), confDialog, SLOT(valuesChanged(QMap<QString, ito::Param>)));
    QMetaObject::invokeMethod(this, "sendParameterRequest");

    if (confDialog->exec())
    {
        confDialog->getVals();
    }

    delete confDialog;

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QCam::startDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if (grabberStartedCount() < 1)
    {
        QCam_Abort(m_camHandle);

        //start your camera
        unsigned long size;
        QCam_GetInfo(m_camHandle, qinfImageSize, &size);

        for (int i = 0; i < NUMBERBUFFERS; ++i)
        {
            if (m_frames[i].pBuffer)
            {
                free(m_frames[i].pBuffer);
            }
            m_frames[i].pBuffer = calloc(size, sizeof(unsigned char));
            m_frames[i].bufferSize = size;
            memset(m_frames[i].pBuffer, 0, size * sizeof(unsigned char));
            QCam_QueueFrame(m_camHandle, &(m_frames[i]), qCamFrameCallback, qcCallbackDone, this, i);
        }

        checkData(); //also resize the internal data object m_data to the new size given by sizex, sizey and bpp
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
ito::RetVal QCam::stopDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    decGrabberStarted();

    if (grabberStartedCount() < 1)
    {
        //stop your camera
        QCam_Abort(m_camHandle);

        for (int i = 0; i < NUMBERBUFFERS; ++i)
        {
            if (m_frames[i].pBuffer)
            {
                free(m_frames[i].pBuffer);
            }
            m_frames[i].pBuffer = NULL;
        }
    }

    if (grabberStartedCount() < 0)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("camera has already stopped").toLatin1().data());
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
ito::RetVal QCam::acquire(const int trigger, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);

    ito::RetVal retValue(ito::retOk);

    if (grabberStartedCount() <= 0)
    {
        retValue = ito::RetVal(ito::retError, 0, tr("Tried to acquire without starting device").toLatin1().data());
    }
    else
    {
        //invalidate all callback-related member variables
        m_frameCallbackFrameIdx = -1;
        m_frameCallbackRetVal = ito::retOk;
        m_waitingForAcquire = true;
        QCam_Trigger(m_camHandle);
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
void QCam::frameCallback(unsigned long userData, QCam_Err errcode, unsigned long flags)
{
    qDebug() << "frameCallback" << userData;
    if (flags & qcCallbackDone)
    {
        if (userData < 0 || userData >= NUMBERBUFFERS)
        {
            m_frameCallbackRetVal = ito::RetVal(ito::retError, 0, tr("callback-done failed. Corrupt data!").toLatin1().data());
        }
        else
        {
            if (errcode == qerrSuccess)
            {
                m_frameCallbackFrameIdx = userData;
            }
            else if (errcode == qerrBlackFill)
            {
                m_frameCallbackRetVal = ito::RetVal(ito::retWarning, 0, tr("errors while data transmission. Pixels might be filled with black values").toLatin1().data());
                m_frameCallbackFrameIdx = userData;
            }
        }

        m_waitingForAcquire = false;
    }
    //else(...)
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QCam::retrieveData(ito::DataObject *externalDataObject)
{
    ito::RetVal retValue(ito::retOk);

    if (m_waitingForAcquire)
    {
        while(m_waitingForAcquire)
        {
            QCoreApplication::processEvents();
        }
    }

    if (m_frameCallbackFrameIdx == -1)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("no grabbed image data available").toLatin1().data());
        retValue += requeueFrame();
        return retValue;
    }

    QCam_Frame *currentFrame = &(m_frames[m_frameCallbackFrameIdx]);

    int width = m_params["sizex"].getVal<int>();
    int height = m_params["sizey"].getVal<int>();
    int bpp = m_params["bpp"].getVal<int>();

    //check whether the data in currentFrame correspond to width and height
    if (currentFrame->width != width || currentFrame->height != height)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("size of image data does not fit to size parameters of plugin").toLatin1().data());
        retValue += requeueFrame();
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

    switch(bpp)
    {
    case 8:
        if (copyExternal)
        {
            retValue += externalDataObject->copyFromData2D<ito::uint8>((const ito::uint8*)(currentFrame->pBuffer), width, height);
        }
        if (!copyExternal || hasListeners)
        {
            retValue += m_data.copyFromData2D<ito::uint8>((const ito::uint8*)(currentFrame->pBuffer), width, height);
        }
        break;
    case 10:
    case 12:
    case 14:
    case 16:
        if (copyExternal)
        {
            retValue += externalDataObject->copyFromData2D<ito::uint16>((const ito::uint16*)(currentFrame->pBuffer), width, height);
        }
        if (!copyExternal || hasListeners)
        {
            retValue += m_data.copyFromData2D<ito::uint16>((const ito::uint16*)(currentFrame->pBuffer), width, height);
        }
        break;
    default:
        retValue += ito::RetVal(ito::retError, 0, tr("unsupported bpp").toLatin1().data());
    }

    retValue += requeueFrame();

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QCam::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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
ito::RetVal QCam::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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
ito::RetVal QCam::requeueFrame()
{
    if (m_frameCallbackFrameIdx >= 0)
    {
        QCam_QueueFrame(m_camHandle, &(m_frames[m_frameCallbackFrameIdx]), qCamFrameCallback, qcCallbackDone, this, m_frameCallbackFrameIdx);
    }

    m_frameCallbackFrameIdx = -1;
    m_frameCallbackRetVal = ito::retOk;

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
// Moved to addInGrabber.cpp, equal for all grabbers / ADDA


//----------------------------------------------------------------------------------------------------------------------------------
void QCam::updateParameters(QMap<QString, ito::ParamBase> params)
{
    foreach(const ito::ParamBase &param1, params)
    {
        setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase(param1)), NULL);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QCam::errorCheck(QCam_Err errcode)
{
    ito::RetVal retval;

    switch(errcode)
    {
    case qerrSuccess:
        break;
    case qerrNotSupported:    // Function is not supported for this device
        retval += ito::RetVal(ito::retError, 1, tr("Function is not supported for this device").toLatin1().data());
        break;
    case qerrInvalidValue:
        retval += ito::RetVal(ito::retError, 2, tr("A parameter used was invalid").toLatin1().data());
        break;
    case qerrBadSettings:
        retval += ito::RetVal(ito::retError, 3, tr("The QCam_Settings structure is corrupted").toLatin1().data());
        break;
    case qerrNoUserDriver:
        retval += ito::RetVal(ito::retError, 4, tr("No user driver").toLatin1().data());
        break;
    case qerrNoFirewireDriver:
        retval += ito::RetVal(ito::retError, 5, tr("Firewire device driver is missing").toLatin1().data());
        break;
    case qerrDriverConnection:
        retval += ito::RetVal(ito::retError, 6, tr("Driver connection error").toLatin1().data());
        break;
    case qerrDriverAlreadyLoaded:
        retval += ito::RetVal(ito::retError, 7, tr("The driver has already been loaded").toLatin1().data());
        break;
    case qerrDriverNotLoaded:
        retval += ito::RetVal(ito::retError, 8, tr("The driver has not been loaded.").toLatin1().data());
        break;
    case qerrInvalidHandle:
        retval += ito::RetVal(ito::retError, 9, tr("The QCam_Handle has been corrupted").toLatin1().data());
        break;
    case qerrUnknownCamera:
        retval += ito::RetVal(ito::retError, 10, tr("Camera model is unknown to this version of QCam").toLatin1().data());
        break;
    case qerrInvalidCameraId:
        retval += ito::RetVal(ito::retError, 11, tr("Camera id used in QCam_OpenCamera is invalid").toLatin1().data());
        break;
    case qerrNoMoreConnections:
        retval += ito::RetVal(ito::retError, 12, tr("Deprecated").toLatin1().data());
        break;
    case qerrHardwareFault:
        retval += ito::RetVal(ito::retError, 13, tr("Hardware fault").toLatin1().data());
        break;
    case qerrFirewireFault:
        retval += ito::RetVal(ito::retError, 14, tr("Firewire fault").toLatin1().data());
        break;
    case qerrCameraFault:
        retval += ito::RetVal(ito::retError, 15, tr("Camera fault").toLatin1().data());
        break;
    case qerrDriverFault:
        retval += ito::RetVal(ito::retError, 16, tr("Driver fault").toLatin1().data());
        break;
    case qerrInvalidFrameIndex:
        retval += ito::RetVal(ito::retError, 17, tr("Invalid frame index").toLatin1().data());
        break;
    case qerrBufferTooSmall:
        retval += ito::RetVal(ito::retError, 18, tr("Frame buffer (pBuffer) is too small for image").toLatin1().data());
        break;
    case qerrOutOfMemory:
        retval += ito::RetVal(ito::retError, 19, tr("Out of memory").toLatin1().data());
        break;
    case qerrOutOfSharedMemory:
        retval += ito::RetVal(ito::retError, 20, tr("Out of shared memory").toLatin1().data());
        break;
    case qerrBusy:
        retval += ito::RetVal(ito::retError, 21, tr("The function used cannot be processed at this time").toLatin1().data());
        break;
    case qerrQueueFull:
        retval += ito::RetVal(ito::retError, 22, tr("The queue for frame and settings changes is full").toLatin1().data());
        break;
    case qerrCancelled:
        retval += ito::RetVal(ito::retError, 23, tr("Cancelled").toLatin1().data());
        break;
    case qerrNotStreaming:
        retval += ito::RetVal(ito::retError, 24, tr("The function used requires that streaming be on").toLatin1().data());
        break;
    case qerrLostSync:
        retval += ito::RetVal(ito::retError, 25, tr("The host and the computer are out of sync, the frame returned is invalid").toLatin1().data());
        break;
    case qerrBlackFill:
        retval += ito::RetVal(ito::retError, 26, tr("Data is missing in the frame returned").toLatin1().data());
        break;
    case qerrFirewireOverflow:
        retval += ito::RetVal(ito::retError, 27, tr("The host has more data than it can process, restart streaming.").toLatin1().data());
        break;
    case qerrUnplugged:
        retval += ito::RetVal(ito::retError, 28, tr("The camera has been unplugged or turned off").toLatin1().data());
        break;
    case qerrAccessDenied:
        retval += ito::RetVal(ito::retError, 29, tr("The camera is already open").toLatin1().data());
        break;
    case qerrStreamFault:
        retval += ito::RetVal(ito::retError, 30, tr("Stream allocation failed, there may not be enough bandwidth").toLatin1().data());
        break;
    case qerrQCamUpdateNeeded:
        retval += ito::RetVal(ito::retError, 31, tr("QCam needs to be updated").toLatin1().data());
        break;
    case qerrRoiTooSmall:
        retval += ito::RetVal(ito::retError, 32, tr("The ROI used is too small").toLatin1().data());
        break;
    case qerr_last:
        retval += ito::RetVal(ito::retError, 33, tr("last").toLatin1().data());
        break;
    case _qerr_force32:
        retval += ito::RetVal(ito::retError, -1, tr("force32").toLatin1().data());
        break;
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QCam::supportedFormats(bool &mono, bool &colorFilter, bool &colorBayer)
{
    unsigned long ccdType;
    ito::RetVal retval;
    QCam_Err qerr;

    if (m_camHandle)
    {
        QCam_GetInfo(m_camHandle, qinfCcdType, &ccdType);
        if (ccdType == qcCcdMonochrome)
        {
            // Check to see if a Color Filter Wheel is supported
            // before we say that it is.
            qerr = QCam_IsRangeTable (&m_camSettings, qprmColorWheel);

            if (qerr == qerrSuccess)
            {
                mono = true;
                colorFilter = true;
            }
            else
            {
                mono = true;
                colorFilter = false;
            }
        }
        else if (ccdType == qcCcdColorBayer)
        {
            mono = false;
            colorFilter = false;
            colorBayer = true;
        }
    }
    else
    {
        retval += ito::RetVal(ito::retError, 0, tr("invalid camera handle").toLatin1().data());
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
void QCam::dockWidgetVisibilityChanged(bool visible)
{
    if (getDockWidget())
    {
        DockWidgetQCam *dw = qobject_cast<DockWidgetQCam*>(getDockWidget()->widget());
        if (visible)
        {
            connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), dw, SLOT(valuesChanged(QMap<QString, ito::Param>)));

            emit parametersChanged(m_params);
        }
        else
        {
            disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), dw, SLOT(valuesChanged(QMap<QString, ito::Param>)));
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void QCAMAPI qCamFrameCallback(void * userPtr, unsigned long userData, QCam_Err errcode, unsigned long flags)
{
    ((QCam*)userPtr)->frameCallback(userData, errcode, flags);
}

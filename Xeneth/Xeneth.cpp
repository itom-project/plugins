/* ********************************************************************
    Plugin "Xeneth" for itom software
    URL: https://github.com/itom-project/plugins
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

#include "Xeneth.h"
#include "pluginVersion.h"

#include <qstring.h>
#include <qstringlist.h>
#include <qelapsedtimer.h>
#include <QtCore/QtPlugin>
#include <qmetaobject.h>

#include "dockWidgetXeneth.h"
#include "dialogXeneth.h"

#include "common/helperCommon.h"



//----------------------------------------------------------------------------------------------------------------------------------
//! constructor for IDSuEye
/*!
    In this constructor the m_params-vector with all parameters, which are accessible by getParam or setParam, is built.
    Additionally the optional docking widget for the IDSuEye's toolbar is instantiated and created by createDockWidget.

    \param [in] uniqueID is an unique identifier for this IDSuEye-instance
    \sa ito::tParam, createDockWidget, setParam, getParam
*/
Xeneth::Xeneth() :
    AddInGrabber(),
    m_handle(0),
    m_isGrabbing(false)
{
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "Xeneth", "GrabberName");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("integration_time", ito::ParamBase::Double, 0.0, 1.0, 0.005, tr("Exposure time of chip (in seconds).").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("binning", ito::ParamBase::Int, 101, 1616, 101, tr("Horizontal and vertical binning, depending on camera ability. 104 means a 1x binning in horizontal and 4x binning in vertical direction. (values up to 1x, 2x, 3x, 4x, 5x, 6x, 8x, 12x are valid; if read only binning is not supported)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("frame_rate", ito::ParamBase::Double | ito::ParamBase::Readonly, 0.0, 10000.0, 0.5, tr("frame rate in fps. This value is always set to the minimum value in order to allow huge exposure times. Since the camera is not run in free-run mode, the frame rate is not important.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("gain", ito::ParamBase::Double, 0.0, 1.0, 0.5, tr("Gain (normalized value 0..1)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("offset", ito::ParamBase::Double, 0.0, 1.0, 0.5, tr("Offset (leads to blacklevel offset) (normalized value 0..1). Readonly if not adjustable.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizex", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 2048, 2048, tr("Pixelsize in x (cols)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizey", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 2048, 2048, tr("Pixelsize in y (rows)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("bpp", ito::ParamBase::Int | ito::ParamBase::Readonly, 8, 16, 16, tr("bit depth").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("timeout", ito::ParamBase::Double, 0.0, 1000.0, 2.0, tr("timeout in seconds for image acquisition").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("x0", ito::ParamBase::Int, 0, 608, 1, tr("left end of the ROI").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("x1", ito::ParamBase::Int, 32, 640, 1, tr("right end of the ROI").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("y0", ito::ParamBase::Int, 0, 508, 1, tr("upper end of the ROI").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("y1", ito::ParamBase::Int, 4, 512, 1, tr("downer end of the ROI").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    int roi[] = {0, 0, 640, 512};
    paramVal = ito::Param("roi", ito::ParamBase::IntArray, 4, roi, tr("ROI (x,y,width,height) [this replaces the values x0,x1,y0,y1]").toLatin1().data());
    ito::RectMeta *rm = new ito::RectMeta(ito::RangeMeta(0, 639), ito::RangeMeta(0, 511));
    paramVal.setMeta(rm, true);
    m_params.insert(paramVal.getName(), paramVal);


    ////now create dock widget for this plugin
    //DockWidgetIDS *dw = new DockWidgetIDS(this);

    //Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
    //QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
    //createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dw);

}

//----------------------------------------------------------------------------------------------------------------------------------
//! destructor
/*!
    \sa ~AddInBase
*/
Xeneth::~Xeneth()
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
ito::RetVal Xeneth::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retVal;

    const char *deviceName = paramsOpt->at(0).getVal<char*>();

    m_handle = XC_OpenCamera(deviceName);

    if (m_handle == 0)
    {
        retVal += ito::RetVal(ito::retError, 0, "no camera found");
    }

    if (!retVal.containsError())
    {
        int width = XC_GetWidth(m_handle);
        int height = XC_GetHeight(m_handle);
        FrameType frameType = XC_GetFrameType(m_handle);

        if (frameType != FT_8_BPP_GRAY && frameType != FT_16_BPP_GRAY)
        {
            retVal += ito::RetVal(ito::retError, 0, "unsupported frame type. Only 8 bit and 1 bit gray values are supported.");
        }
        else
        {
            if (frameType == FT_8_BPP_GRAY)
            {
                m_params["bpp"].setVal<int>(8);
            }
            else
            {
                m_params["bpp"].setVal<int>(16);
            }

            m_params["sizex"].setVal<int>(width);
            m_params["sizex"].setMeta(new ito::IntMeta(1, XC_GetMaxWidth(m_handle)), true);
            m_params["sizey"].setVal<int>(height);
            m_params["sizey"].setMeta(new ito::IntMeta(1, XC_GetMaxHeight(m_handle)), true);

            char serialNumber[128];
            retVal += checkError(XC_GetPropertyValue(m_handle, "_CAM_SER", serialNumber, 128));
            serialNumber[127] = '\0';

            if (!retVal.containsError())
            {
                setIdentifier(serialNumber);
            }

            for (int i = 0; i < XC_GetPropertyCount (m_handle); ++i)
            {
                char buf[256];
                XC_GetPropertyName(m_handle, i, buf, 256);
                std::cout << i << ": " << buf << "\n" << std::endl;
            }

            if (!retVal.containsError())
            {
                XPropType type;
                retVal += checkError(XC_GetPropertyType(m_handle, "GainControl", &type));
                retVal += checkError(XC_SetPropertyValueF(m_handle, "GainControl", 1, ""));;

            }
            if (!retVal.containsError())
            {
                retVal += checkError(XC_SetPropertyValueF(m_handle, "OffsetControl", 1, ""));;
            }

            retVal += synchronize();
        }
    }

    if(!retVal.containsError())
    {
        checkData(); //check if image must be reallocated
    }

    if(waitCond)
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
ito::RetVal Xeneth::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;

    // When the handle to the camera is still initialised ...
    if (m_handle != 0 && XC_IsInitialised(m_handle))
    {
        if (XC_IsCapturing(m_handle))
        {
            retValue += checkError(XC_StopCapture(m_handle));
        }

        XC_CloseCamera(m_handle);
        m_handle = 0;
    }


    if(waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
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
ito::RetVal Xeneth::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
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
        if (hasIndex)
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
ito::RetVal Xeneth::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
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
        retValue += apiValidateAndCastParam(*it, *val, false, true, true);
    }

    if (!retValue.containsError())
    {
        if (key == "integration_time")
        {
            retValue += checkError(XC_SetPropertyValueL(m_handle, "IntegrationTime", val->getVal<double>() * 1e6, NULL));
            retValue += synchronize(integration_time);
        }
        if (key == "gain")
        {

            retValue += checkError(XC_SetPropertyValueL(m_handle, "ManualModeGain", qBound<long>(1, val->getVal<double>()*255+1, 255.99), NULL));
            retValue += synchronize(gain);
        }
        if (key == "offset")
        {

            retValue += checkError(XC_SetPropertyValueL(m_handle, "ManualModeOffset", val->getVal<double>() *200-100, NULL));
            retValue += synchronize(offset);
        }

        if (key == "x0" || key == "x1" || key == "y0" || key == "y1" || key == "roi")
        {
            int started = this->grabberStartedCount();
            if (started > 0)
            {
                setGrabberStarted(1);
                stopDevice(NULL);
            }

            if (key == "roi")
            {
                if (hasIndex)
                {
                    switch (index)
                    {
                    case 0:
                        retValue += checkError(XC_SetPropertyValueL(m_handle, "OffsetX", val->getVal<int>(), ""));
                        break;
                    case 1:
                        retValue += checkError(XC_SetPropertyValueL(m_handle, "Width", val->getVal<int>(), ""));
                        break;
                    case 2:
                        retValue += checkError(XC_SetPropertyValueL(m_handle, "OffsetY", val->getVal<int>(), ""));
                        break;
                    case 3:
                        retValue += checkError(XC_SetPropertyValueL(m_handle, "Height", val->getVal<int>(), ""));
                        break;
                    default:
                        retValue += ito::RetVal(ito::retError, 0, "invalid index");
                    }

                    retValue += synchronize(roi);
                }
                else
                {
                    const int *roi_ = val->getVal<int*>();
                    retValue += checkError(XC_SetPropertyValueL(m_handle, "OffsetX", roi_[0], ""));
                    retValue += checkError(XC_SetPropertyValueL(m_handle, "Width", roi_[2], ""));
                    retValue += checkError(XC_SetPropertyValueL(m_handle, "OffsetY", roi_[1], ""));
                    retValue += checkError(XC_SetPropertyValueL(m_handle, "Height", roi_[3], ""));
                    retValue += synchronize(roi);
                }
            }

            else if (key == "x0")
            {


                retValue += checkError(XC_SetPropertyValueL(m_handle, "OffsetX", val->getVal<int>(), NULL));
                retValue += synchronize(roi);
            }

            else if (key == "x1")
            {
                long current_x0;
                retValue += checkError(XC_GetPropertyValueL(m_handle, "OffsetX", &current_x0));
                retValue += checkError(XC_SetPropertyValueL(m_handle, "Width", val->getVal<int>()-current_x0, NULL));
                retValue += synchronize(roi);
            }

            else if (key == "y0")
            {

                retValue += checkError(XC_SetPropertyValueL(m_handle, "OffsetY", val->getVal<int>(), NULL));
                retValue += synchronize(roi);
            }

            else if (key == "y1")
            {
                long current_y0;
                retValue += checkError(XC_GetPropertyValueL(m_handle, "OffsetY", &current_y0));
                retValue += checkError(XC_SetPropertyValueL(m_handle, "Hight", val->getVal<int>()-current_y0, NULL));
                retValue += synchronize(roi);
            }

            if (started > 0)
            {
                startDevice(NULL);
                setGrabberStarted(started);
            }
        }

    }



    /*if(!retValue.containsError())
    {
        retValue += checkData();
    }*/

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
ito::RetVal Xeneth::startDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = ito::retOk;

    if (!XC_IsInitialised(m_handle))
    {
        retValue += ito::RetVal(ito::retError, 0, "camera device is not initialised. start device failed.");
    }
    else
    {
        checkData(); //this will be reallocated in this method.

        incGrabberStarted();
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
    In this IDSuEye, this method does nothing. In general, the hardware camera should be closed in this method.

    \note This method is similar to VideoCapture::release() of openCV

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if everything is ok, retError if camera wasn't started before
    \sa startDevice
*/
ito::RetVal Xeneth::stopDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = ito::retOk;

    decGrabberStarted();
    if(grabberStartedCount() < 0)
    {
        retValue += ito::RetVal(ito::retError, 1001, tr("StopDevice of Xeneth can not be executed, since camera has not been started.").toLatin1().data());
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
ito::RetVal Xeneth::acquire(const int trigger, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = ito::retOk;

    if (grabberStartedCount() <= 0)
    {
        retValue += ito::RetVal(ito::retError, 1002, tr("Acquire of Xeneth can not be executed, since camera has not been started.").toLatin1().data());
    }
    else
    {
        m_isGrabbing = true;
        retValue += checkError(XC_StartCapture(m_handle));
        m_acquisitionRetVal = ito::retOk;
    }

    if(waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    int bpp = m_params["bpp"].getVal<int>();
    int width = m_params["sizex"].getVal<int>();
    int height = m_params["sizey"].getVal<int>();
    double timeoutMS = m_params["timeout"].getVal<double>() * 1000.0;
    QElapsedTimer timer;
    ErrCode getFrameCode = E_NO_FRAME;
    bool done = false;
    FrameType frameType = (bpp == 8) ? FT_8_BPP_GRAY : FT_16_BPP_GRAY;

    int size = (bpp == 8) ? 1 : 2 * (width * height * sizeof(char));
    int size2 = XC_GetFrameSize(m_handle);
    timer.start();

    while (getFrameCode == E_NO_FRAME && timer.elapsed() < timeoutMS)
    {
        getFrameCode = XC_GetFrame(m_handle, /*FT_NATIVE, XGF_Blocking*/ frameType, XGF_NoConversion, m_data.rowPtr(0,0), size);
    }

    if (getFrameCode == E_NO_FRAME)
    {
        m_acquisitionRetVal += ito::RetVal(ito::retError, 0, "timeout while getting frame");
    }
    else
    {
        m_acquisitionRetVal += checkError(getFrameCode);
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
ito::RetVal Xeneth::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);

    ito::RetVal retValue(ito::retOk);

    retValue += retrieveData();

    if(!retValue.containsError())
    {
        if(dObj == NULL)
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
ito::RetVal Xeneth::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);

    if(!dObj)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Empty object handle retrieved from caller").toLatin1().data());
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
ito::RetVal Xeneth::retrieveData(ito::DataObject *externalDataObject)
{
    ito::RetVal retValue(ito::retOk);

    if (m_isGrabbing == false)
    {
        retValue += ito::RetVal(ito::retError, 1002, tr("image could not be obtained since no image has been acquired.").toLatin1().data());
    }
    else
    {
        retValue += m_acquisitionRetVal;
    }

    if (!retValue.containsError())
    {
        if (externalDataObject)
        {
            m_data.deepCopyPartial(*externalDataObject);
        }
    }

    m_isGrabbing = false;

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Xeneth::dockWidgetVisibilityChanged(bool visible)
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
ito::RetVal Xeneth::checkError(const ErrCode &code)
{
    switch (code)
    {
    case I_OK:
        return ito::retOk;
    case I_DIRTY:
        return ito::RetVal(ito::retError, code, "Internal error");
    case E_BUG:
        return ito::RetVal(ito::retError, code, "Generic error");
    case E_NOINIT:
        return ito::RetVal(ito::retError, code, "Camera was not successfully initialised");
    case E_LOGICLOADFAILED:
        return ito::RetVal(ito::retError, code, "Invalid logic file");
    case E_INTERFACE_ERROR:
        return ito::RetVal(ito::retError, code, "Command interface failure");
    case E_OUT_OF_RANGE:
        return ito::RetVal(ito::retError, code, "Provided value is incapable of being produced by the hardware");
    case E_NOT_SUPPORTED:
        return ito::RetVal(ito::retError, code, "Functionality not supported by this camera");
    case E_NOT_FOUND:
        return ito::RetVal(ito::retError, code, "File/Data not found");
    case E_FILTER_DONE:
        return ito::RetVal(ito::retError, code, "Filter has finished processing, and will be removed  ");
    case E_NO_FRAME:
        return ito::RetVal(ito::retError, code, "A frame was requested by calling GetFrame, but none was available");
    case E_SAVE_ERROR:
        return ito::RetVal(ito::retError, code, "Couldn't save to file");
    case E_MISMATCHED:
        return ito::RetVal(ito::retError, code, "Buffer size mismatch");
    case E_BUSY:
        return ito::RetVal(ito::retError, code, "The API can not read a temperature because the camera is busy");
    case E_INVALID_HANDLE:
        return ito::RetVal(ito::retError, code, "An unknown handle was passed to the C API");
    case E_TIMEOUT:
        return ito::RetVal(ito::retError, code, "Operation timed out");
    case E_FRAMEGRABBER:
        return ito::RetVal(ito::retError, code, "Frame grabber error");
    case E_NO_CONVERSION:
        return ito::RetVal(ito::retError, code, "GetFrame could not convert the image data to the requested format");
    case E_FILTER_SKIP_FRAME:
        return ito::RetVal(ito::retError, code, "Filter indicates the frame should be skipped");
    case E_WRONG_VERSION:
        return ito::RetVal(ito::retError, code, "Version mismatch");
    case E_PACKET_ERROR:
        return ito::RetVal(ito::retError, code, "The requested frame cannot be provided because at least one packet has been lost");
    case E_WRONG_FORMAT:
        return ito::RetVal(ito::retError, code, "The emissivity map you tried to set should be a 16 bit grayscale png");
    case E_WRONG_SIZE:
        return ito::RetVal(ito::retError, code, "The emissivity map you tried to set has the wrong dimensions (w,h)");
    case E_CAPSTOP:
        return ito::RetVal(ito::retError, code, "Internal error");
    default:
        return ito::RetVal(ito::retError, code, "unknown");
    }
}

//----------------------------------------------------------------------------------------
const ito::RetVal Xeneth::showConfDialog(void)
{
    return ito::retOk; //apiShowConfigurationDialog(this, new DialogIDS(this));
}

//----------------------------------------------------------------------------------------
ito::RetVal Xeneth::synchronize(const sections &what /*= all*/)
{
    ito::RetVal retval;
    ito::RetVal retval_temp;

    if (what & integration_time)
    {
        long low, high, current;
        retval_temp = checkError(XC_GetPropertyRangeL(m_handle, "IntegrationTime", &low, &high));
        retval_temp += checkError(XC_GetPropertyValueL(m_handle, "IntegrationTime", &current));
        if (!retval_temp.containsError())
        {
            m_params["integration_time"].setMeta(new ito::DoubleMeta((double)low * 1.0e-6, (double)high * 1.0e-6), true);
            m_params["integration_time"].setVal<double>((current) * 1.0e-6);
        }

        retval += retval_temp;
    }

    if (what & gain)
    {
        long low, high, current;

        retval_temp += checkError(XC_GetPropertyRangeL(m_handle, "ManualModeGain", &low, &high));
        retval_temp += checkError(XC_GetPropertyValueL(m_handle, "ManualModeGain", &current));
        if (!retval_temp.containsError())
        {
            m_params["gain"].setMeta(new ito::DoubleMeta(((double)low -1)/255, ((double)high-1)/255), true);
            m_params["gain"].setVal<double>(((current) -1)/255);
        }

        retval += retval_temp;
    }

        if (what & offset)
    {
        long low, high, current;

        retval_temp += checkError(XC_GetPropertyRangeL(m_handle, "ManualModeOffset", &low, &high));
        retval_temp += checkError(XC_GetPropertyValueL(m_handle, "ManualModeOffset", &current));
        if (!retval_temp.containsError())
        {
            m_params["offset"].setMeta(new ito::DoubleMeta(((double)low +100)/200, ((double)high+100)/200), true);
            m_params["offset"].setVal<double>(((current) +100)/200);
        }

        retval += retval_temp;
    }

    if (what & roi)
    {
        long current_x0, xmax, current_x1, current_y0, ymax, current_y1;




        retval_temp += checkError(XC_GetPropertyValueL(m_handle, "WidthMax", &xmax));
        retval_temp += checkError(XC_GetPropertyValueL(m_handle, "Width", &current_x1));
        retval_temp += checkError(XC_GetPropertyValueL(m_handle, "OffsetX", &current_x0));
        retval_temp += checkError(XC_GetPropertyValueL(m_handle, "HightMax", &ymax));
        retval_temp += checkError(XC_GetPropertyValueL(m_handle, "OffsetY", &current_y0));
        retval_temp += checkError(XC_GetPropertyValueL(m_handle, "Hight", &current_y1));

        if (!retval_temp.containsError())
        {
            m_params["x0"].setMeta(new ito::IntMeta(0, xmax, 1), true);
            m_params["x0"].setVal<int>(current_x0);
            m_params["roi"].getVal<int*>()[0] = current_x0;
            m_params["x1"].setMeta(new ito::IntMeta(0, xmax, 1), true);
            m_params["x1"].setVal<int>(current_x1+current_x0);
            m_params["roi"].getVal<int*>()[2] = current_x1;
            m_params["y0"].setMeta(new ito::IntMeta(0, ymax, 1), true);
            m_params["y0"].setVal<int>(current_y0);
            m_params["roi"].getVal<int*>()[1] = current_y0;
            m_params["y1"].setMeta(new ito::IntMeta(0, ymax, 1), true);
            m_params["y1"].setVal<int>(current_y1+current_y0);
            m_params["roi"].getVal<int*>()[3] = current_y1;

            m_params["sizey"].setVal<int>(234);
            m_params["sizex"].setVal<int>(234);

        }


        retval += retval_temp;
    }


    return retval;
}

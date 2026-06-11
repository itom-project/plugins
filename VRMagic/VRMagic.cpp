/* ********************************************************************
    Plugin "VRMagic" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2016, Institut für Technische Optik, Universität Stuttgart

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

#include "VRMagic.h"
#include "pluginVersion.h"
#include "gitVersion.h"
#include "dockWidgetVRMagic.h"
#include "dialogVRMagic.h"

#include "common/sharedFunctionsQt.h"
#include "common/helperCommon.h"

#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>
#include <qmetaobject.h>

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal VRMagicInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(VRMagic)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal VRMagicInterface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(VRMagic)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
VRMagicInterface::VRMagicInterface()
{
    m_type = ito::typeDataIO | ito::typeGrabber;
    setObjectName("VRMagic");

    m_description = QObject::tr("VRMagic Camera / Framegrabber");
    m_detaildescription = QObject::tr("Plugin for cameras / framgrabbers from VRMagic that run with the VRMagic API. \n\
Currently, only monochrome cameras with 8, 10 or 16 bit depth are supported. \n\
This plugin has been tested using the VRmAVC-2 grabber  under Windows.");

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
    m_aboutThis = QObject::tr(GITVERSION);

    m_initParamsMand.clear();

    ito::Param param("device_num", ito::ParamBase::Int, 0, 5, 0, tr("Device number.").toLatin1().data());
    m_initParamsOpt.append(param);

    param = ito::Param("portnum_num", ito::ParamBase::Int, 0, 5, 0, tr("Port number of chosen device.").toLatin1().data());
    m_initParamsOpt.append(param);
}

//----------------------------------------------------------------------------------------------------------------------------------
VRMagicInterface::~VRMagicInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();

}


//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal VRMagic::showConfDialog(void)
{
   return apiShowConfigurationDialog(this, new DialogVRMagic(this));
}

//----------------------------------------------------------------------------------------------------------------------------------
VRMagic::VRMagic() :
    AddInGrabber(),
    m_handle(NULL),
    m_acqRetVal(ito::retOk),
    m_device_key(NULL),
    m_imagesAreInterlaced(false)
{
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly | ito::ParamBase::NoAutosave, "VRMagic", tr("name of the camera").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("serial_number", ito::ParamBase::String |ito::ParamBase::Readonly, tr("unknown").toLatin1().data(), tr("Serial number of device.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("vendor_ID", ito::ParamBase::Int |ito::ParamBase::Readonly, 0, tr("Vendor ID of device (vid).").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("product_ID", ito::ParamBase::Int |ito::ParamBase::Readonly, 0, tr("Product ID of device (pid).").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("group_ID", ito::ParamBase::Int |ito::ParamBase::Readonly, 0, tr("Group ID of device (gid).").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("format", ito::ParamBase::String |ito::ParamBase::Readonly, tr("unknown").toLatin1().data(), tr("Format of the image. See documentation of SDK for explanation.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("brightness", ito::ParamBase::Int, 0, 255, 128, tr("Brightness of camera / framegrabber (0 to 255)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("hue", ito::ParamBase::Int, 0, 255, 128, tr("Hue of camera / framegrabber (0 to 255)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("contrast", ito::ParamBase::Int, 0, 255, 128, tr("Contrast of camera / framegrabber (0 to 255)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("saturation", ito::ParamBase::Int, 0, 255, 128, tr("Saturation of camera / framegrabber (0 to 255)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizex", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 4096, 1, tr("Width of ROI (number of columns).").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizey", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 4096, 1, tr("Height of ROI (number of rows).").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("bpp", ito::ParamBase::Int, 8, 16, 8, tr("Bit depth of the output data from camera in bpp (can differ from sensor bit depth).").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("signal_source", ito::ParamBase::String, tr("svideo").toLatin1().data(), tr("Signal source of the grabber [svideo, composite, yc].").toLatin1().data());
    ito::StringMeta sm(ito::StringMeta::String);
    sm.addItem(tr("svideo").toLatin1().data());
    sm.addItem(tr("composite").toLatin1().data());
    sm.addItem(tr("yc").toLatin1().data());
    paramVal.setMeta(&sm, false);
    m_params.insert(paramVal.getName(), paramVal);

    if (hasGuiSupport())
    {
        //now create dock widget for this plugin

        DockWidgetVRMagic *m_dockWidget = new DockWidgetVRMagic(getID(), this);

        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, m_dockWidget);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
VRMagic::~VRMagic()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal VRMagic::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    VRmDWORD device_num = paramsOpt->at(0).getVal<int>(); // first parameter is device number
    VRmDWORD port_num = paramsOpt->at(1).getVal<int>(); // second parameter is port number of device

    if (!retValue.containsError())
    {
        VRmDWORD device_list_size=0;

        retValue += checkError(VRmUsbCamUpdateDeviceKeyList(), tr("Update the device key list").toLatin1().data());
        if (!retValue.containsError())
        {
            retValue += checkError(VRmUsbCamGetDeviceKeyListSize(&device_list_size), tr("Get device key list size").toLatin1().data());
        }
        if (!retValue.containsError())
        {
            if (device_list_size == 0)
            {
                retValue += ito::RetVal(ito::retError, 0, tr("no VRmagic device connected to this computer.").toLatin1().data());
            }
            else if (device_num < 0 || device_num >= device_list_size)
            {
                retValue += ito::RetVal::format(ito::retError, 0, tr("device_num is out of range [0,%i]. %i connected devices detected.").toLatin1().data(), device_list_size - 1, device_list_size);
            }
        }
        if (!retValue.containsError())
        {
            retValue += checkError(VRmUsbCamGetDeviceKeyListEntry(device_num, &m_device_key), tr("Get device key of chosen device").toLatin1().data());
        }

        if (!retValue.containsError() && !m_device_key->m_busy)
        {
            // read device information
            VRmWORD vendor_id;
            retValue += checkError(VRmUsbCamGetVendorId(m_device_key, &vendor_id), tr("Get vendor ID").toLatin1().data());
            if (!retValue.containsError())
            {
                m_params["vendor_ID"].setVal<int>((int)vendor_id);
            }

            VRmWORD product_id;
            retValue += checkError(VRmUsbCamGetProductId(m_device_key, &product_id), tr("Get product ID").toLatin1().data());
            if (!retValue.containsError())
            {
                m_params["product_ID"].setVal<int>((int)product_id);
            }

            VRmWORD group_id;
            retValue += checkError(VRmUsbCamGetGroupId(m_device_key, &group_id), tr("Get group ID").toLatin1().data());
            if (!retValue.containsError())
            {
                m_params["group_ID"].setVal<int>((int)group_id);
            }

            VRmSTRING serial_string;
            retValue += checkError(VRmUsbCamGetSerialString(m_device_key, &serial_string), tr("Get serial string").toLatin1().data());
            if (!retValue.containsError())
            {
                m_params["serial_number"].setVal<char*>((char*)serial_string);
            }

            // open the device an get the camera/grabber handle m_handle
            retValue += checkError(VRmUsbCamOpenDevice(m_device_key, &m_handle), tr("Open the device").toLatin1().data());

            VRmDWORD pl_size;
            if (!retValue.containsError())
            {
                retValue += checkError(VRmUsbCamGetSensorPortListSize(m_handle, &pl_size), tr("Get sensor port list size").toLatin1().data());
            }

            if (!retValue.containsError())
            {
                retValue += checkError(VRmUsbCamGetSensorPortListEntry(m_handle, port_num, &m_port), tr("Open port of sensor").toLatin1().data());
            }

            VRmImageFormat sformat;
            if (!retValue.containsError())
            {
                retValue += checkError(VRmUsbCamGetSourceFormatEx(m_handle, m_port, &sformat), tr("Get format of port").toLatin1().data());
            }

            if (sformat.m_image_modifier & VRM_INTERLACED_FIELD01)
            {
                m_imagesAreInterlaced = true; //two images need to be transferred from the camera to obtain one real image.
            }

            VRmDWORD pixeldepth;
            if (!retValue.containsError())
            {
                retValue += checkError(VRmUsbCamGetPixelDepthFromColorFormat(sformat.m_color_format, &pixeldepth), tr("Get bpp").toLatin1().data());
            }
            m_params["bpp"].setVal<int>((int)(pixeldepth*8));

            VRmSTRING formatstring;
            if (!retValue.containsError())
            {
                retValue += checkError(VRmUsbCamGetSourceFormatDescription(m_handle, m_port, &formatstring), tr("Get format description of 1st port of sensor").toLatin1().data());
            }
            if (!retValue.containsError())
            {
                m_params["format"].setVal<char*>((char*)formatstring);
            }

            /*VRmDWORD fl_size;
            if (!retValue.containsError())
            {
                retValue += checkError(VRmUsbCamGetTargetFormatListSizeEx2(m_handle, m_port, &fl_size), "Get number of possible formats for 1st port");
                VRmImageFormat fl_format;

                for (VRmDWORD idx = 0; idx < fl_size; ++idx)
                {
                    retValue += checkError(VRmUsbCamGetTargetFormatListEntryEx2(m_handle, m_port, idx, &fl_format), "Get formats of 1st port");
                }
            }*/

            VRmSizeI size;
            if (!retValue.containsError())
            {
                retValue += checkError(VRmUsbCamGetPropertyValueSizeI(m_handle, VRM_PROPID_CAM_SENSOR_SIZE_I, &size), tr("Get sensor size").toLatin1().data());
            }
            m_params["sizex"].setVal<int>((int)size.m_width);
            m_params["sizey"].setVal<int>((int)size.m_height);
        }
        else
        {
            retValue += ito::RetVal(ito::retError, 0, tr("Camera already in use by another program or no device detected.").toLatin1().data());
        }

        if (!retValue.containsError())
        {
            setIdentifier(m_params["serial_number"].getVal<char*>());
        }

        if (!retValue.containsError())
        {
            retValue += synchronizeCameraSettings();
            retValue += checkData();
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
ito::RetVal VRMagic::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if (this->grabberStartedCount() > 0)
    {
        setGrabberStarted(1);
        retValue += this->stopDevice(0);
    }

    if (m_handle)
    {
        retValue += checkError(VRmUsbCamCloseDevice(m_handle), tr("Close the device").toLatin1().data());
        m_handle = NULL;
    }

    if (m_device_key)
    {
        retValue += checkError(VRmUsbCamFreeDeviceKey(&m_device_key), tr("Free device key").toLatin1().data());
        m_device_key = NULL;
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal VRMagic::checkError(const VRmRetVal &error, const char* command /*= NULL*/)
{
    if (error == VRM_SUCCESS)
    {
        return ito::retOk;
    }

    int code = VRmUsbCamGetLastErrorCode();
    const char* err = VRmUsbCamGetLastError();

    if (command != NULL)
    {
        return ito::RetVal::format(ito::retError, code, "%s: %s", command, err);
    }
    else
    {
        return ito::RetVal(ito::retError, code, err);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \details This method copies the complete tparam of the corresponding parameter to val

    \param [in,out] val  is a input of type::tparam containing name, value and further information
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk in case that everything is ok, else retError
    \sa ito::tParam, ItomSharedSemaphore
*/
ito::RetVal VRMagic::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
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
ito::RetVal VRMagic::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex;
    int index;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;
    DWORD intSize = sizeof(int);
    DWORD floatSize = sizeof(float);

    int running = 0;

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
        if (grabberStartedCount())
        {
            running = grabberStartedCount();
            setGrabberStarted(1);
            retValue += this->stopDevice(0);
        }

        if (QString::compare(key, "brightness", Qt::CaseInsensitive) == 0)
        {
            int brightness = val->getVal<int>();
            brightness = brightness - 128; // internally saved as int from -128 to 127
            retValue += checkError(VRmUsbCamSetPropertyValueI(m_handle, VRM_PROPID_CAM_BRIGHTNESS_I , &brightness), tr("Set brightness").toLatin1().data());
            retValue += synchronizeCameraSettings(sBrightness);
        }

        else if (QString::compare(key, "hue", Qt::CaseInsensitive) == 0)
        {
            int hue = val->getVal<int>();
            hue = hue - 128; // internally saved as int from -128 to 127
            retValue += checkError(VRmUsbCamSetPropertyValueI(m_handle, VRM_PROPID_CAM_HUE_I , &hue), tr("Set hue").toLatin1().data());
            retValue += synchronizeCameraSettings(sHue);
        }

        else if (QString::compare(key, "contrast", Qt::CaseInsensitive) == 0)
        {
            int contrast = val->getVal<int>();
            retValue += checkError(VRmUsbCamSetPropertyValueI(m_handle, VRM_PROPID_CAM_CONTRAST_I , &contrast), tr("Set contrast").toLatin1().data());
            retValue += synchronizeCameraSettings(sContrast);
        }

        else if (QString::compare(key, "saturation", Qt::CaseInsensitive) == 0)
        {
            int saturation = val->getVal<int>();
            retValue += checkError(VRmUsbCamSetPropertyValueI(m_handle, VRM_PROPID_CAM_SATURATION_I , &saturation), tr("Set saturation").toLatin1().data());
            retValue += synchronizeCameraSettings(sSaturation);
        }

        else if (QString::compare(key, "signal_source", Qt::CaseInsensitive) == 0)
        {
            char* signalsource = val->getVal<char*>();

            if (QString::compare(signalsource, "svideo", Qt::CaseInsensitive) == 0)
            {
                VRmPropId setsource = VRM_PROPID_CAM_SIGNAL_SOURCE_SVIDEO;
                retValue += checkError(VRmUsbCamSetPropertyValueE(m_handle, VRM_PROPID_CAM_SIGNAL_SOURCE_E , &setsource), tr("Set source to svideo").toLatin1().data());
            }
            else if (QString::compare(signalsource, "composite", Qt::CaseInsensitive) == 0)
            {
                VRmPropId setsource = VRM_PROPID_CAM_SIGNAL_SOURCE_COMPOSITE;
                retValue += checkError(VRmUsbCamSetPropertyValueE(m_handle, VRM_PROPID_CAM_SIGNAL_SOURCE_E , &setsource), tr("Set source to composite").toLatin1().data());
            }
            else if (QString::compare(signalsource, "yc", Qt::CaseInsensitive) == 0)
            {
                VRmPropId setsource = VRM_PROPID_CAM_SIGNAL_SOURCE_YC;
                retValue += checkError(VRmUsbCamSetPropertyValueE(m_handle, VRM_PROPID_CAM_SIGNAL_SOURCE_E , &setsource), tr("Set source to yc").toLatin1().data());
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 0, tr("Unknown signal source. Choose from: svideo, composite, yc.").toLatin1().data());
            }
            retValue += synchronizeCameraSettings(sSignalSource);
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
ito::RetVal VRMagic::synchronizeCameraSettings(int what /*= sAll */)
{
    ito::RetVal retValue;
    ito::RetVal retValTemp;
    ParamMapIterator it;

    DWORD intSize = sizeof(int);
    DWORD floatSize = sizeof(float);

    if (what & sBrightness)
    {
        it = m_params.find("brightness");
        int brightness;
        retValue += checkError(VRmUsbCamGetPropertyValueI(m_handle, VRM_PROPID_CAM_BRIGHTNESS_I, &brightness), tr("Get brightness").toLatin1().data());

        it->setVal<int>(brightness+128); // internally saved as int from -128 to 127
    }

    if (what & sHue)
    {
        it = m_params.find("hue");
        int hue;
        retValue += checkError(VRmUsbCamGetPropertyValueI(m_handle, VRM_PROPID_CAM_HUE_I, &hue), tr("Get hue").toLatin1().data());

        it->setVal<int>(hue+128); // internally saved as int from -128 to 127
    }

    if (what & sContrast)
    {
        it = m_params.find("contrast");
        int contrast;
        retValue += checkError(VRmUsbCamGetPropertyValueI(m_handle, VRM_PROPID_CAM_CONTRAST_I, &contrast), tr("Get contrast").toLatin1().data());

        it->setVal<int>(contrast);
    }

    if (what & sSaturation)
    {
        it = m_params.find("saturation");
        int saturation;
        retValue += checkError(VRmUsbCamGetPropertyValueI(m_handle, VRM_PROPID_CAM_SATURATION_I, &saturation), tr("Get saturation").toLatin1().data());

        it->setVal<int>(saturation);
    }

    if (what & sSignalSource)
    {
        it = m_params.find("signal_source");
        VRmPropId signalsource;
        retValue += checkError(VRmUsbCamGetPropertyValueE(m_handle, VRM_PROPID_CAM_SIGNAL_SOURCE_E, &signalsource), tr("Get signal source").toLatin1().data());

        if (!retValue.containsError())
        {
            if (signalsource == VRM_PROPID_CAM_SIGNAL_SOURCE_SVIDEO)
            {
                it->setVal<const char*>("svideo");
            }
            else if (signalsource == VRM_PROPID_CAM_SIGNAL_SOURCE_COMPOSITE)
            {
                it->setVal<const char*>("composite");
            }
            else if (signalsource == VRM_PROPID_CAM_SIGNAL_SOURCE_YC)
            {
                it->setVal<const char*>("yc");
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 0, tr("Unknown signal source").toLatin1().data());
            }
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal VRMagic::startDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if (grabberStartedCount() < 1)
    {
        setGrabberStarted(0);
        retValue += checkError(VRmUsbCamStart(m_handle), tr("Start Camera").toLatin1().data());
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
ito::RetVal VRMagic::stopDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    decGrabberStarted();

    if (grabberStartedCount() < 1)
    {
        retValue += checkError(VRmUsbCamStop(m_handle), tr("Stop Camera").toLatin1().data());
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

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal VRMagic::acquire(const int trigger, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    m_acqRetVal = ito::retOk;

    VRmImage* image;
    VRmDWORD droppedframes;
    int timeout_ms = 100;
    int elem_size = 1; //bytes

    for (int repeats = 0; repeats < (m_imagesAreInterlaced ? 2 : 1); ++repeats)
    {
        VRmRetVal ret = VRmUsbCamLockNextImageEx2(m_handle, m_port, &image, &droppedframes, timeout_ms);

        if (ret == VRM_FAILED)
        {
            m_acqRetVal += ito::RetVal(ito::retError, 0, tr("VRM failed. Couldn't get image from camera. Possible reasons: timeout while acquireing, wrong signal source,...").toLatin1().data());
        }
        else
        {
            VRmImageFormat &image_format = image->m_image_format;

            if (image_format.m_image_modifier & VRM_VERTICAL_MIRRORED || image_format.m_image_modifier & VRM_HORIZONTAL_MIRRORED)
            {
                m_acqRetVal += ito::RetVal(ito::retWarning, 0, tr("image was vertically or horizontally mirrored. This property is currently ignored").toLatin1().data());
            }

            switch (image_format.m_color_format)
            {

            case VRM_GRAY_10:
            case VRM_GRAY_16:
                elem_size = 2; //both 10 and 16 bit pixels are stored in 2 bytes
            case VRM_GRAY_8:
                elem_size = std::max(elem_size, 1); //only 8 bit pixel is stored in 1 byte (max is there in order to let 10 and 16 bit case pass)
                {
                    cv::Mat *dest_image = m_data.getCvPlaneMat(0);
                    VRmBYTE *src_ptr = image->mp_buffer;

                    if (image_format.m_image_modifier & VRM_INTERLACED_FIELD0)
                    {
                        for (VRmDWORD row = 0; row < 2 * image_format.m_height; row += 2)
                        {
                            memcpy(dest_image->ptr(row), src_ptr, image_format.m_width * elem_size);
                            src_ptr += image->m_pitch * elem_size;
                        }
                    }
                    else if (image_format.m_image_modifier & VRM_INTERLACED_FIELD1)
                    {
                        for (VRmDWORD row = 1; row < 2 * image_format.m_height; row += 2)
                        {
                            memcpy(dest_image->ptr(row), src_ptr, image_format.m_width * elem_size);
                            src_ptr += image->m_pitch * elem_size;
                        }
                    }
                    else //full-image
                    {
                        for (VRmDWORD row = 0; row < image_format.m_height; ++row)
                        {
                            memcpy(dest_image->ptr(row), src_ptr, image_format.m_width * elem_size);
                            src_ptr += image->m_pitch * elem_size;
                        }
                    }
                }
                break;
            default:
                {
                    m_acqRetVal += ito::RetVal::format(ito::retError, 0, tr("color format (%i) not implemented yet").toLatin1().data(), image_format.m_color_format);
                }
                break;
            }

            m_acqRetVal += checkError(VRmUsbCamUnlockNextImage(m_handle, &image), tr("Unlock next image").toLatin1().data());
        }
    }

    return retValue + m_acqRetVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal VRMagic::retrieveData(ito::DataObject *externalDataObject)
{
    ito::RetVal retValue(ito::retOk);

    bool copyExternal = externalDataObject != NULL;

    if (grabberStartedCount() <= 0)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("Tried to get picture without starting device").toLatin1().data());
    }
    else
    {
        retValue += m_acqRetVal;
        m_acqRetVal = ito::retOk;
    }

    if (!retValue.containsError())
    {
        if (copyExternal)
        {
            const cv::Mat* internalMat = m_data.getCvPlaneMat(0);
            cv::Mat* externalMat = externalDataObject->getCvPlaneMat(0);

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

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal VRMagic::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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
/*!
    \detail This method copies the recently grabbed camera frame to the given DataObject. Therefore this camera size must fit to the data structure of the
    DataObject.

    \note This method is similar to VideoCapture::retrieve() of openCV

    \param [in,out] vpdObj is the pointer to a given dataObject (this pointer should be cast to ito::DataObject*) where the acquired data is deep copied to.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if everything is ok, retError is camera has not been started or no data has been acquired by the method acquire.
    \sa DataObject, acquire
*/
ito::RetVal VRMagic::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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
void VRMagic::dockWidgetVisibilityChanged(bool visible)
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

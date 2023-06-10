/* ********************************************************************
    Plugin "CommonVisionBlox" for itom software
    URL: https://github.com/itom-project/plugins
    Copyright (C) 2014, Institut fuer Technische Optik, Universitaet Stuttgart

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

#include "commonVisionBlox.h"
#include "pluginVersion.h"

#include <qstring.h>
#include <qstringlist.h>
#include <qelapsedtimer.h>
#include <QtCore/QtPlugin>
#include <qmetaobject.h>

#include "common/helperCommon.h"

// constant for path length
static const size_t DRIVERPATHSIZE = 256;

//----------------------------------------------------------------------------------------------------------------------------------
//! constructor for IDSuEye
/*!
    In this constructor the m_params-vector with all parameters, which are accessible by getParam or setParam, is built.
    Additionally the optional docking widget for the IDSuEye's toolbar is instantiated and created by createDockWidget.

    \param [in] uniqueID is an unique identifier for this IDSuEye-instance
    \sa ito::tParam, createDockWidget, setParam, getParam
*/
CommonVisionBlox::CommonVisionBlox() :
    AddInGrabber(),
    m_isGrabbing(false),
    m_hNodeMap(NULL),
    m_hCamera(NULL)
{
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "CommonVisionBlox", "GrabberName");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("integration_time", ito::ParamBase::Double, 0.0, 1000.0, 0.005, tr("Exposure time of chip (in seconds).").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("heartbeat_timeout", ito::ParamBase::Int, 0, 10000, 100, tr("Heartbeat timeout of GigE Vision Transport Layer.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("acquisition_mode", ito::ParamBase::String, "snap", tr("'snap' is a single image acquisition (only possible in trigger_mode 'off'), 'grab' is a continuous acquisition").toLatin1().data());
    ito::StringMeta sm(ito::StringMeta::String);
    sm.addItem("snap");
    sm.addItem("grab");
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("trigger_mode", ito::ParamBase::String, "off", tr("'off': camera is not explicitly triggered but operated in freerun mode. The next acquired image is provided upon acquire, 'software' sets trigger mode to On and fires a software trigger at acquire (only possible in acquisition_mode 'grab').").toLatin1().data());
    ito::StringMeta sm2(ito::StringMeta::String);
    sm2.addItem("off");
    sm2.addItem("software");
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("sizex", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 2048, 2048, tr("Pixelsize in x (cols)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizey", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 2048, 2048, tr("Pixelsize in y (rows)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("bpp", ito::ParamBase::Int | ito::ParamBase::Readonly, 8, 16, 16, tr("bit depth").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

#if 0
    paramVal = ito::Param("x0", ito::ParamBase::Int, 0, 608, 1, tr("left end of the ROI").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("x1", ito::ParamBase::Int, 32, 640, 1, tr("right end of the ROI").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("y0", ito::ParamBase::Int, 0, 508, 1, tr("upper end of the ROI").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("y1", ito::ParamBase::Int, 4, 512, 1, tr("downer end of the ROI").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
#endif

    paramVal = ito::Param("raw", ito::ParamBase::String ,"", tr("use raw:paramname to set internal paramname of camera to value. paramname is the original GenICam parameter name.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("vendor_name", ito::ParamBase::String | ito::ParamBase::Readonly ,"unknown", tr("vendor name").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("model_name", ito::ParamBase::String | ito::ParamBase::Readonly ,"unknown", tr("model name").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    int roi[] = {0, 0, 640, 512};
    paramVal = ito::Param("roi", ito::ParamBase::IntArray, 4, roi, tr("ROI (x,y,width,height) [this replaces the values x0,x1,y0,y1]").toLatin1().data());
    ito::RectMeta *rm = new ito::RectMeta(ito::RangeMeta(roi[0], roi[2]-1), ito::RangeMeta(roi[1], roi[3]-1));
    paramVal.setMeta(rm, true);
    m_params.insert(paramVal.getName(), paramVal);
}

//----------------------------------------------------------------------------------------------------------------------------------
//! destructor
/*!
    \sa ~AddInBase
*/
CommonVisionBlox::~CommonVisionBlox()
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
ito::RetVal CommonVisionBlox::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retVal;

    bool scan = paramsOpt->at(0).getVal<int>() > 0;
    int bpp = paramsOpt->at(1).getVal<int>();

    if (scan)
    {
        retVal += scan_for_cameras();
    }

    if (!retVal.containsError())
    {

        // load the first camera
        char driverPath[DRIVERPATHSIZE] = { 0 };
        TranslateFileName("%CVB%\\Drivers\\GenICam.vin", driverPath, DRIVERPATHSIZE);
        cvbbool_t success = LoadImageFile(driverPath, m_hCamera);

        if (!success)
        {
            if (scan)
            {
                retVal += ito::RetVal::format(ito::retError, 0, tr("Error loading %s driver! Probably no cameras were found during discovery").toLatin1().data(), driverPath);
            }
            else
            {
                retVal += ito::RetVal::format(ito::retError, 0, tr("Error loading %s driver! Probably no cameras are configured / reachable!").toLatin1().data(), driverPath);
            }
        }
        else if (CanNodeMapHandle(m_hCamera))
        {
            retVal += checkError(NMHGetNodeMap(m_hCamera, m_hNodeMap));
        }
        else if (!CanGrab2(m_hCamera))
        {
            retVal += ito::RetVal(ito::retError, 0, tr("camera does not support the Grab2 interface from Common Vision Blox").toLatin1().data());
        }
        else if (!CanGrabber(m_hCamera))
        {
            retVal += ito::RetVal(ito::retError, 0, tr("camera does not support the Grabber interface from Common Vision Blox").toLatin1().data());
        }
        /*else if (!CanSoftwareTrigger (m_hCamera))
        {
            retVal += ito::RetVal(ito::retError, 0, tr("camera does not support the SoftwareTrigger interface from Common Vision Blox").toLatin1().data());
        }*/
    }

    if (!retVal.containsError())
    {
        QString desiredPixelType;
        desiredPixelType = QString("Mono%1").arg(bpp);
        ito::RetVal r = setParamString("PixelFormat", desiredPixelType.toLatin1().data());

        if (r.containsError())
        {
            if (r.errorCode() == CVC_E_PARAMETER)
            {
                retVal += ito::RetVal(ito::retError, 0, tr("chosen bitdepth is not supported by this camera.").toLatin1().data());
            }
            else
            {
                 retVal += r;
            }
        }

        if (!retVal.containsError())
        {
            QByteArray pixelFormat;
            retVal += getParamString("PixelFormat", pixelFormat);

            //filter string meta for usable items

            if (pixelFormat == "Mono8")
            {
                m_params["bpp"].setVal<int>(8);
                m_params["bpp"].setMeta(new ito::IntMeta(8,8), true);
            }
            else if (pixelFormat == "Mono10")
            {
                m_params["bpp"].setVal<int>(10);
                m_params["bpp"].setMeta(new ito::IntMeta(10,10), true);
            }
            else if (pixelFormat == "Mono12")
            {
                m_params["bpp"].setVal<int>(12);
                m_params["bpp"].setMeta(new ito::IntMeta(12,12), true);
            }
            else if (pixelFormat == "Mono14")
            {
                m_params["bpp"].setVal<int>(14);
                m_params["bpp"].setMeta(new ito::IntMeta(14,14), true);
            }
            else if (pixelFormat == "Mono16")
            {
                m_params["bpp"].setVal<int>(16);
                m_params["bpp"].setMeta(new ito::IntMeta(16,16), true);
            }
            else
            {
                retVal += ito::RetVal::format(ito::retError, 0, tr("unsupported pixel format %s (supported is Mono8 and Mono16").toLatin1().data(), pixelFormat.data());
            }
        }

        QByteArray serialNumber;
        retVal += getParamString("DeviceID" /*"_CAM_SER"*/, serialNumber);

        if (!retVal.containsError())
        {
            setIdentifier(serialNumber);
        }

        if (!getParamString("DeviceVendorName", serialNumber).containsError())
        {
            m_params["vendor_name"].setVal<char*>(serialNumber.data());
        }

        if (!getParamString("DeviceModelName", serialNumber).containsError())
        {
            m_params["model_name"].setVal<char*>(serialNumber.data());
        }

        //try to set some default things

        //Xenics Bobcat:
        setParamString("RawMode", "RAW_Disabed");

        ito::IntMeta intMeta(0,0);
        if (getParamIntInfo("GevHeartbeatTimeout", intMeta).containsError())
        {
            m_params["heartbeat_timeout"].setFlags(ito::ParamBase::Readonly);
        }
        else
        {
            m_params["heartbeat_timeout"].setMeta(&intMeta);
            cvbint64_t value;
            getParamInt("GevHeartbeatTimeout", value);
            m_params["heartbeat_timeout"].setVal<int>(value);
        }

        //Dalsa
        setParamString("ExposureMode", "Timed");
        setParamString("ExposureAuto", "Off");

        if (nodeExists("ExposureTimeAbs"))
        {
            m_nameConverter["integration_time"] = "ExposureTimeAbs";
        }
        else if (nodeExists("ExposureTime"))
        {
            m_nameConverter["integration_time"] = "ExposureTime";
        }
        else
        {
            retVal += ito::RetVal(ito::retError, 0, tr("no node 'ExposureTime' or 'ExposureTimeAbs' available").toLatin1().data());
        }

        if (setParamString("TriggerMode", "Off").containsError())
        {
            m_params["trigger_mode"].setFlags(ito::ParamBase::Readonly);
        }

        retVal += synchronize();
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
ito::RetVal CommonVisionBlox::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;

    if (grabberStartedCount() > 0)
    {
        setGrabberStarted(1);
        retValue += stopDevice(NULL);
    }

    if (m_hNodeMap)
    {
        ReleaseObject(m_hNodeMap);
        m_hNodeMap = NULL;
    }

    if (m_hCamera)
    {
        ReleaseObject(m_hCamera);
    }

    if (waitCond)
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
ito::RetVal CommonVisionBlox::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
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
        if (hasIndex)
        {
            *val = apiGetParam(*it, hasIndex, index, retValue);
        }
        else if (key == "raw")
        {
            if (suffix == "")
            {
                cvbdim_t nodeCount;
                retValue += checkError(NMNodeCount(m_hNodeMap, nodeCount));
                if (!retValue.containsError())
                {
                    char nodeName[128] = {0};
                    char info[128] = {8};
                    size_t nodeNameSize = 128;
                    QByteArray value;
                    NODE node = NULL;
                    for (cvbdim_t i = 0; i < nodeCount; ++i)
                    {

                        NMListNode(m_hNodeMap, i, nodeName, nodeNameSize);

                        NMGetNode(m_hNodeMap, nodeName, node);
                        NInfoAsString(node, NI_AccessMode, info, nodeNameSize);
                        ReleaseObject(node);

                        //if (strcmp(info, "Read Only") != 0)
                        //{
                            //NI_AccessMode
                            this->getParamString(nodeName, value);
                            std::cout << i << ": " << nodeName << ": " << value.data() << "(" << info << ")\n" << std::endl;
                        //}
                    }
                }
            }
            else
            {
                QByteArray nodeName_ = suffix.toLatin1();
                const char* nodeName = nodeName_.data();
                NODE node = NULL;
                TNodeType nodeType;
                retValue += checkError(NMGetNode(m_hNodeMap, nodeName, node), nodeName);

                if (!IsNode(node))
                {
                    retValue += ito::RetVal::format(ito::retError, 0, tr("%s is no node").toLatin1().data(), nodeName);
                }
                if (!retValue.containsError())
                {
                    char info[128] = {0};
                    size_t nodeInfoSize = 128;

                    NInfoAsString(node, NI_AccessMode, info, nodeInfoSize);

                    NType(node, nodeType);
                    switch (nodeType)
                    {
                    case NT_Boolean:
                        {
                            bool value;
                            retValue += getParamBool(nodeName, value);
                            if (!retValue.containsError())
                            {
                                it->setVal<const char*>(value ? "true":"false");
                            }

                            std::cout << nodeName << ": " << value << "(bool, " << info << ")\n" << std::endl;
                        }
                        break;
                    case NT_Integer:
                        {
                            cvbint64_t value;
                            retValue += getParamInt(nodeName, value);
                            if (!retValue.containsError())
                            {
                                QString n = QString::number(value);
                                it->setVal<char*>(n.toLatin1().data());
                            }

                            std::cout << nodeName << ": " << value << "(int, " << info << ")\n" << std::endl;
                        }
                        break;
                    case NT_Float:
                        {
                            double value;
                            retValue += getParamFloat(nodeName, value);
                            if (!retValue.containsError())
                            {
                                QString n = QString::number(value);
                                it->setVal<char*>(n.toLatin1().data());
                            }

                            std::cout << nodeName << ": " << value << "(float, " << info << ")\n" << std::endl;
                        }
                        break;
                    case NT_String:
                        {
                            QByteArray value;
                            retValue += getParamString(nodeName, value);
                            if (!retValue.containsError())
                            {
                                it->setVal<char*>(value.data());
                            }

                            std::cout << nodeName << ": " << value.data() << "(string, " << info << ")\n" << std::endl;
                        }
                        break;
                    case NT_EnumEntry:
                        {
                            QByteArray value;
                            retValue += getParamString(nodeName, value);
                            if (!retValue.containsError())
                            {
                                it->setVal<char*>(value.data());
                            }

                            std::cout << nodeName << ": " << value.data() << "(enum entry, " << info << ")\n" << std::endl;
                        }
                        break;
                    case NT_Enumeration:
                        {
                            QByteArray value;
                            retValue += getParamString(nodeName, value);
                            if (!retValue.containsError())
                            {
                                it->setVal<char*>(value.data());
                            }

                            ito::StringMeta meta(ito::StringMeta::String);
                            retValue += getParamEnumerationInfo(nodeName, meta);

                            std::cout << nodeName << ": " << value.data() << "(enum, " << info << ", values:";
                            for (int i = 0; i < meta.getLen(); ++i)
                            {
                                std::cout << meta.getString(i) <<",";
                            }
                            std::cout << ")\n" << std::endl;
                        }
                        break;
                    default:
                        retValue += ito::RetVal::format(ito::retError, 0, tr("unsupported property type (%i)").toLatin1().data(), nodeType);
                    }
                }

                ReleaseObject(node);
            }

            *val = *it;
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
ito::RetVal CommonVisionBlox::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
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
    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

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
        if (key == "integration_time")
        {
            retValue += setParamInt(m_nameConverter["integration_time"], (cvbint64_t)(val->getVal<double>() * 1e6));
            retValue += synchronize(integration_time);
        }
        else if (key == "heartbeat_timeout")
        {
            retValue += setParamInt("GevHeartbeatTimeout", (cvbint64_t)(val->getVal<int>()));
        }
        else if (key == "raw")
        {
            if (suffix != "")
            {
                retValue += setParamString(suffix.toLatin1().data(), val->getVal<char*>());
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 0, tr("you need to indiciate a suffix for the node you want to set").toLatin1().data());
            }
        }
        else if (key == "x0" || key == "x1" || key == "y0" || key == "y1" || key == "roi" || key == "acquisition_mode" || key == "trigger_mode")
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
                        retValue += setParamInt("OffsetX", val->getVal<int>());
                        break;
                    case 1:
                        retValue += setParamInt("OffsetY", val->getVal<int>());
                        break;
                    case 2:
                        retValue += setParamInt("Width", val->getVal<int>());
                        break;
                    case 3:
                        retValue += setParamInt("Height", val->getVal<int>());
                        break;
                    default:
                        retValue += ito::RetVal(ito::retError, 0, tr("invalid index").toLatin1().data());
                    }

                    retValue += synchronize(roi);
                    checkData();
                }
                else
                {
                    const int *roi_ = val->getVal<int*>();
                    const int *current_roi = it->getVal<int*>();

                    int max_width = m_params["sizex"].getMax();
                    int max_height = m_params["sizey"].getMax();

                    if (roi_[0] + current_roi[2] > max_width)
                    {
                        retValue += setParamInt("Width", roi_[2]);
                        retValue += setParamInt("OffsetX", roi_[0]);

                    }
                    else
                    {
                        retValue += setParamInt("OffsetX", roi_[0]);
                        retValue += setParamInt("Width", roi_[2]);
                    }

                    if (roi_[1] + current_roi[3] > max_height)
                    {
                        retValue += setParamInt("Height", roi_[3]);
                        retValue += setParamInt("OffsetY", roi_[1]);
                    }
                    else
                    {
                        retValue += setParamInt("OffsetY", roi_[1]);
                        retValue += setParamInt("Height", roi_[3]);
                    }
                    retValue += synchronize(roi);

                    checkData();
                }
            }
            else if (key == "trigger_mode")
            {
                if (val->getVal<char*>()[0] == 's')
                {
                    if (m_params["acquisition_mode"].getVal<char*>()[0] == 's')
                    {
                        retValue += ito::RetVal(ito::retError, 0, tr("trigger_mode 'software' can only be set in acquisition_mode 'grab'").toLatin1().data());
                    }
                    else
                    {
                        retValue += setParamString("TriggerMode", "On");
                    }
                }
                else
                {
                    retValue += setParamString("TriggerMode", "Off");
                }

                if (!retValue.containsError())
                {
                    it->copyValueFrom(&(*val));
                }
            }
            else if (key == "acquisition_mode")
            {
                if (val->getVal<char*>()[0] == 's' && m_params["trigger_mode"].getVal<char*>()[0] == 's')
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("acquisition_mode 'snap' can only be set in trigger_mode 'off'").toLatin1().data());
                }
                else
                {
                    it->copyValueFrom(&(*val));
                }
            }
#if 0
            else if (key == "x0")
            {


                //retValue += checkError(XC_SetPropertyValueL(m_handle, "OffsetX", val->getVal<int>(), NULL));
                retValue += synchronize(roi);
            }

            else if (key == "x1")
            {
                long current_x0;
                /*retValue += getParamInt("OffsetX", &current_x0));
                retValue += checkError(XC_SetPropertyValueL(m_handle, "Width", val->getVal<int>()-current_x0, NULL));*/
                retValue += synchronize(roi);
            }

            /*else if (key == "y0")
            {

                retValue += checkError(XC_SetPropertyValueL(m_handle, "OffsetY", val->getVal<int>(), NULL));
                retValue += synchronize(roi);
            }

            else if (key == "y1")
            {
                long current_y0;
                retValue += getParamInt("OffsetY", &current_y0));
                retValue += checkError(XC_SetPropertyValueL(m_handle, "Hight", val->getVal<int>()-current_y0, NULL));
                retValue += synchronize(roi);
            }*/
#endif
            if (started > 0)
            {
                startDevice(NULL);
                setGrabberStarted(started);
            }
        }

    }



    /*if (!retValue.containsError())
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
    In the IDSuEye, this method does nothing. In general, the hardware camera should be intialized in this method and necessary memory should be allocated.

    \note This method is similar to VideoCapture::open() of openCV

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if starting was successfull, retWarning if startDevice has been calling at least twice.
*/
ito::RetVal CommonVisionBlox::startDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = ito::retOk;

    if (0 /*!XC_IsInitialised(m_handle)*/)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("camera device is not initialised. start device failed.").toLatin1().data());
    }
    else
    {
        incGrabberStarted();

        checkData(); //this will be reallocated in this method.

        if (grabberStartedCount() == 1)
        {
            if (strcmp(m_params["acquisition_mode"].getVal<char*>(), "grab") == 0)
            {
                retValue += checkError(G2Grab(m_hCamera));
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
//! With stopDevice the camera device is stopped (opposite to startDevice)
/*!
    In this IDSuEye, this method does nothing. In general, the hardware camera should be closed in this method.

    \note This method is similar to VideoCapture::release() of openCV

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if everything is ok, retError if camera wasn't started before
    \sa startDevice
*/
ito::RetVal CommonVisionBlox::stopDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = ito::retOk;

    decGrabberStarted();

    if (grabberStartedCount() == 0)
    {
        double active = 0;
        if (G2GetGrabStatus(m_hCamera, GRAB_INFO_GRAB_ACTIVE, active) == CVC_E_OK && active)
        {
            // stop the grab (kill = false: wait for ongoing frame acquisition to stop)
            retValue += checkError(G2Freeze(m_hCamera, true));
        }
    }

    if (grabberStartedCount() < 0)
    {
        retValue += ito::RetVal(ito::retError, 1001, tr("StopDevice of CommonVisionBlox can not be executed, since camera has not been started.").toLatin1().data());
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
ito::RetVal CommonVisionBlox::acquire(const int trigger, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = ito::retOk;

    if (grabberStartedCount() <= 0)
    {
        retValue += ito::RetVal(ito::retError, 1002, tr("Acquire of CommonVisionBlox can not be executed, since camera has not been started.").toLatin1().data());
    }
    else
    {
        m_isGrabbing = true;
        m_acquisitionRetVal = ito::retOk;

        if (strcmp(m_params["acquisition_mode"].getVal<char*>(), "grab") == 0)
        {
            double images_pendig;

            //delete all old frames in grab mode
            while (G2GetGrabStatus(m_hCamera, GRAB_INFO_NUMBER_IMAGES_PENDIG, images_pendig) == CVC_E_OK && images_pendig > 0)
            {
                G2Wait(m_hCamera);
            }
        }

        if (m_params["trigger_mode"].getVal<char*>()[0] == 's') //software trigger
        {
            retValue += setParamBool("TriggerSoftware", true);
        }
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    if (!retValue.containsError())
    {
        int bpp = m_params["bpp"].getVal<int>();
        int width = m_params["sizex"].getVal<int>();
        int height = m_params["sizey"].getVal<int>();
        const int* roi = m_params["roi"].getVal<int*>();


        //checkStatus();

        if (strcmp(m_params["acquisition_mode"].getVal<char*>(), "grab") == 0)
        {
            retValue += checkError(G2Wait(m_hCamera));
        }
        else
        {
            retValue += checkError(Snap(m_hCamera));
        }

        //checkStatus();

        if (!retValue.containsError())
        {
            // do image processing
            void *pBase = NULL;
            intptr_t xInc = 0;
            intptr_t yInc = 0;
            cvbdatatype_t type = ImageDatatype(m_hCamera, 0);
            cvbdim_t h = ImageHeight(m_hCamera);
            cvbdim_t w = ImageWidth(m_hCamera);
            cvbval_t bppImg = BitsPerPixel(type);
            bool test = GetLinearAccess(m_hCamera, 0, &pBase, &xInc, &yInc);

            if (bppImg != bpp)
            {
                char iniPath[DRIVERPATHSIZE] = { 0 };
                TranslateFileName("%CVBDATA%\\Drivers\\GenICam.ini", iniPath, DRIVERPATHSIZE);
                retValue += ito::RetVal::format(ito::retError, 0, tr("Obtained image has %i bits per pixel instead of %i given by the camera. Both values must be equal. Change property PixelFormat in %s of Common Vision Blox or use the configuration tool to adjust the CVB Color Format").toLatin1().data(), bppImg, bpp, iniPath);
            }
            else if (xInc != ((int)(std::ceil((float)bpp/8.0))))
            {
                retValue += ito::RetVal(ito::retError, 0, tr("currently unsupported image format").toLatin1().data());
            }
            else
            {
                if (bpp == 16 || bpp == 14 || bpp == 12 || bpp == 10)
                {
                    ito::uint16 *rowPtr = NULL;
                    cv::Mat *plane = m_data.getCvPlaneMat(0);
                    const ito::uint8 *srcPtr = (ito::uint8*)pBase;
                    cvbval_t value;

                    for (int y = 0; y < height; ++y)
                    {
                        rowPtr = plane->ptr<ito::uint16>(y);
                        memcpy(rowPtr, srcPtr, sizeof(ito::uint16) * width);
                        srcPtr += (xInc * width) /*yInc*/;
                    }
                }
                else if (bpp == 8)
                {
                    ito::uint8 *rowPtr = NULL;
                    cv::Mat *plane = m_data.getCvPlaneMat(0);
                    const ito::uint8 *srcPtr = (ito::uint8*)pBase;
                    cvbval_t value;

                    for (int y = 0; y < height; ++y)
                    {
                        rowPtr = plane->ptr<ito::uint8>(y);
                        memcpy(rowPtr, srcPtr, sizeof(ito::uint8) * width);
                        srcPtr += (xInc * width) /*yInc*/;
                    }
                }
                else
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("currently unsupported bit depth of image").toLatin1().data());
                }
            }

        }
        m_acquisitionRetVal = retValue;
    }
    else
    {
        m_isGrabbing = false;
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
void CommonVisionBlox::checkStatus()
{
    double cam_detected, triggers_lost, active, images_pendig, images_locked, images_lost_locked, images_lost, images_acquired;

    if (G2GetGrabStatus(m_hCamera, GRAB_INFO_NUMBER_IMAGES_AQCUIRED, images_acquired) != CVC_E_OK)
    {
        images_acquired = -1;
    }
    if (G2GetGrabStatus(m_hCamera, GRAB_INFO_NUMBER_IMAGES_LOST, images_lost) != CVC_E_OK)
    {
        images_lost = -1;
    }
    if (G2GetGrabStatus(m_hCamera, GRAB_INFO_NUMBER_IMAGES_LOST_LOCKED, images_lost_locked) != CVC_E_OK)
    {
        images_lost_locked = -1;
    }
    if (G2GetGrabStatus(m_hCamera, GRAB_INFO_NUMBER_IMAGES_LOCKED, images_locked) != CVC_E_OK)
    {
        images_locked = -1;
    }
    if (G2GetGrabStatus(m_hCamera, GRAB_INFO_NUMBER_IMAGES_PENDIG, images_pendig) != CVC_E_OK)
    {
        images_pendig = -1;
    }
    if (G2GetGrabStatus(m_hCamera, GRAB_INFO_GRAB_ACTIVE, active) != CVC_E_OK)
    {
        active = -1;
    }
    if (G2GetGrabStatus(m_hCamera, GRAB_INFO_NUMBER_TRIGGERS_LOST, triggers_lost) != CVC_E_OK)
    {
        triggers_lost = -1;
    }
    if (G2GetGrabStatus(m_hCamera, GRAB_INFO_CAMERA_DETECTED, cam_detected) != CVC_E_OK)
    {
        cam_detected = -1;
    }

    std::cout << "CAM_DETECTED:" << cam_detected << ", NUM_TRIGGERS_LOST:" << triggers_lost << ", GRAB_ACTIVE:" << active << ", IMAGES_PENDIG:" << images_pendig;
    std::cout << ", IMG_LOCKED:" << images_locked << ", IMG_LOST_LOCKED:" << images_lost_locked << "IMG_LOST:" << images_lost << "IMG_ACQUIRED:" << images_acquired << "\n" << std::endl;

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
ito::RetVal CommonVisionBlox::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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
ito::RetVal CommonVisionBlox::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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
ito::RetVal CommonVisionBlox::retrieveData(ito::DataObject *externalDataObject)
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
void CommonVisionBlox::dockWidgetVisibilityChanged(bool visible)
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

//----------------------------------------------------------------------------------------
const ito::RetVal CommonVisionBlox::showConfDialog(void)
{
    return ito::retOk; //apiShowConfigurationDialog(this, new DialogIDS(this));
}

//----------------------------------------------------------------------------------------
ito::RetVal CommonVisionBlox::synchronize(const sections &what /*= all*/)
{
    ito::RetVal retval;
    ito::RetVal retval_temp;

    if (what & integration_time)
    {
        cvbint64_t current;
        //retval_temp = checkError(XC_GetPropertyRangeF(m_handle, "IntegrationTime", &low, &high));
        retval_temp = getParamInt(m_nameConverter["integration_time"], current);
        if (!retval_temp.containsError())
        {
            ito::DoubleMeta dm(0.0, 0.0);

            //with DALSA Genie there was an error obtaining the max value. Afterwards, no new image could be acquired.
            if (strcmp(m_params["vendor_name"].getVal<char*>(), "DALSA") != 0)
            {
                if (getParamFloatInfo(m_nameConverter["integration_time"], dm, 1e-6).containsError() == false)
                {
                    m_params["integration_time"].setMeta(&dm);
                }
                else
                {
                    m_params["integration_time"].setFlags(ito::ParamBase::Readonly);
                }
            }

            m_params["integration_time"].setVal<double>(double(current) * 1.0e-6);

        }

        retval += retval_temp;
    }

    if (what & roi)
    {
        cvbint64_t current_x0, xmax, current_width, current_y0, ymax, current_height;

        retval_temp = getParamInt("WidthMax", xmax);
        retval_temp += getParamInt("Width", current_width);
        retval_temp += getParamInt("OffsetX", current_x0);
        retval_temp += getParamInt("HeightMax", ymax);
        retval_temp += getParamInt("OffsetY", current_y0);
        retval_temp += getParamInt("Height", current_height);

        if (!retval_temp.containsError())
        {
            m_params["sizex"].setMeta(new ito::IntMeta(0, xmax, 1), true); //replace 1 by step size of ROI
            m_params["sizex"].setVal<int>(current_width);

            m_params["sizey"].setMeta(new ito::IntMeta(0, ymax, 1), true); //replace 1 by step size of ROI
            m_params["sizey"].setVal<int>(current_height);

            ito::RangeMeta width_meta = ito::RangeMeta(0, xmax-1, 1, 32, xmax, 32);
            ito::RangeMeta height_meta = ito::RangeMeta(0, ymax-1, 1, 4, ymax, 4);
            ito::RectMeta *rect_meta = new ito::RectMeta(width_meta, height_meta);
            m_params["roi"].setMeta(rect_meta, true);
            int roi[] = {current_x0, current_y0, current_width, current_height};
            m_params["roi"].setVal<int*>(roi, 4);

#if 0
            m_params["x0"].setMeta(new ito::IntMeta(0, xmax - 33, 1), true);
            m_params["x0"].setVal<int>(current_x0);

            m_params["x1"].setMeta(new ito::IntMeta(current_x0 + 31, xmax - 1, 1), true);
            m_params["x1"].setVal<int>(current_x0 + current_width - 1);

            m_params["y0"].setMeta(new ito::IntMeta(0, ymax - 5, 1), true);
            m_params["y0"].setVal<int>(current_y0);

            m_params["y1"].setMeta(new ito::IntMeta(current_y0 + 3, ymax - 1, 1), true);
            m_params["y1"].setVal<int>(current_y0 + current_height - 1);
#endif
        }

        //x0,y0,width,height
        retval += retval_temp;
    }


    return retval;
}

//-----------------------------------------------------------------------------------
ito::RetVal CommonVisionBlox::getParamInt(const char *name, cvbint64_t &value)
{
    ito::RetVal retVal;

    if (m_hNodeMap)
    {
        // get width feature node
        NODE node = NULL;
        retVal += checkError(NMGetNode(m_hNodeMap, name, node), name);
        if (!retVal.containsError())
        {
            // value camera dependent
            retVal += checkError(NGetAsInteger(node, value), name);

            ReleaseObject(node);
        }
    }
    else
    {
        retVal += ito::RetVal(ito::retError, 0, "node map not avaible");
    }

    return retVal;
}

//--------------------------------------------------------------------------------------
ito::RetVal CommonVisionBlox::getParamFloat(const char *name, double &value)
{
    ito::RetVal retVal;

    if (m_hNodeMap)
    {
        // get width feature node
        NODE node = NULL;
        retVal += checkError(NMGetNode(m_hNodeMap, name, node), name);
        if (!retVal.containsError())
        {
            // value camera dependent
            retVal += checkError(NGetAsFloat(node, value), name);

            ReleaseObject(node);
        }
    }
    else
    {
        retVal += ito::RetVal(ito::retError, 0, tr("node map not avaible").toLatin1().data());
    }

    return retVal;
}

//--------------------------------------------------------------------------------------
ito::RetVal CommonVisionBlox::getParamBool(const char *name, bool &value)
{
    ito::RetVal retVal;

    if (m_hNodeMap)
    {
        // get width feature node
        NODE node = NULL;
        retVal += checkError(NMGetNode(m_hNodeMap, name, node), name);
        if (!retVal.containsError())
        {
            // value camera dependent
            cvbbool_t val = 0;
            retVal += checkError(NGetAsBoolean(node, val), name);

            if (!retVal.containsError())
            {
                value = val;
            }

            ReleaseObject(node);
        }
    }
    else
    {
        retVal += ito::RetVal(ito::retError, 0, tr("node map not avaible").toLatin1().data());
    }

    return retVal;
}

//--------------------------------------------------------------------------------------
ito::RetVal CommonVisionBlox::getParamString(const char *name, QByteArray &value)
{
    ito::RetVal retVal;

    if (m_hNodeMap)
    {
        // get width feature node
        NODE node = NULL;
        retVal += checkError(NMGetNode(m_hNodeMap, name, node), name);
        if (!retVal.containsError())
        {
            // value camera dependent
            char val[256] = {0};
            size_t size = 256;
            retVal += checkError(NGetAsString(node, val, size), name);

            if (!retVal.containsError())
            {
                value = val;
            }

            ReleaseObject(node);
        }
    }
    else
    {
        retVal += ito::RetVal(ito::retError, 0, tr("node map not avaible").toLatin1().data());
    }

    return retVal;
}

//-----------------------------------------------------------------------------------
ito::RetVal CommonVisionBlox::setParamInt(const char *name, const cvbint64_t &value)
{
    ito::RetVal retVal;

    if (m_hNodeMap)
    {
        // get width feature node
        NODE node = NULL;
        retVal += checkError(NMGetNode(m_hNodeMap, name, node), name);
        if (!retVal.containsError())
        {
            // value camera dependent
            retVal += checkError(NSetAsInteger(node, value), name);

            ReleaseObject(node);
        }
    }
    else
    {
        retVal += ito::RetVal(ito::retError, 0, tr("node map not avaible").toLatin1().data());
    }

    return retVal;
}


//--------------------------------------------------------------------------------------
ito::RetVal CommonVisionBlox::setParamFloat(const char *name, const double &value)
{
    ito::RetVal retVal;

    if (m_hNodeMap)
    {
        // get width feature node
        NODE node = NULL;
        retVal += checkError(NMGetNode(m_hNodeMap, name, node), name);
        if (!retVal.containsError())
        {
            // value camera dependent
            retVal += checkError(NSetAsFloat(node, value), name);

            ReleaseObject(node);
        }
    }
    else
    {
        retVal += ito::RetVal(ito::retError, 0, tr("node map not avaible").toLatin1().data());
    }

    return retVal;
}

//--------------------------------------------------------------------------------------
ito::RetVal CommonVisionBlox::setParamBool(const char *name, const bool &value)
{
    ito::RetVal retVal;

    if (m_hNodeMap)
    {
        // get width feature node
        NODE node = NULL;
        retVal += checkError(NMGetNode(m_hNodeMap, name, node), name);
        if (!retVal.containsError())
        {
            // value camera dependent
            retVal += checkError(NSetAsBoolean(node, value), name);

            ReleaseObject(node);
        }
    }
    else
    {
        retVal += ito::RetVal(ito::retError, 0, tr("node map not avaible").toLatin1().data());
    }

    return retVal;
}

//--------------------------------------------------------------------------------------
ito::RetVal CommonVisionBlox::setParamString(const char *name, const char *value)
{
    ito::RetVal retVal;

    if (m_hNodeMap)
    {
        // get width feature node
        NODE node = NULL;
        retVal += checkError(NMGetNode(m_hNodeMap, name, node), name);
        if (!retVal.containsError())
        {
            // value camera dependent
            size_t size = 256;
            retVal += checkError(NSetAsString(node, value), name);

            ReleaseObject(node);
        }
    }
    else
    {
        retVal += ito::RetVal(ito::retError, 0, tr("node map not avaible").toLatin1().data());
    }

    return retVal;
}

//--------------------------------------------------------------------------------------
ito::RetVal CommonVisionBlox::getParamFloatInfo(const char *name, ito::DoubleMeta &meta, const double &scale /*= 1.0*/)
{
    ito::RetVal retVal;

    if (m_hNodeMap)
    {
        // get width feature node
        NODE node = NULL;
        retVal += checkError(NMGetNode(m_hNodeMap, name, node), name);
        if (!retVal.containsError())
        {
            TNodeType type;
            NType(node, type);
            double mi, ma, inc;

            if (type == NT_Integer)
            {
                cvbint64_t mi_, ma_, inc_;
                // value camera dependent
                retVal += checkError(NInfoAsInteger(node, NI_Min, mi_), name);
                retVal += checkError(NInfoAsInteger(node, NI_Max, ma_), name);
                cvbres_t res = NInfoAsInteger(node, NI_Increment, inc_);
                if (CVC_ERROR_FROM_HRES(res) != CVC_E_NOTSUPPORTED)
                {
                    retVal += checkError(res, name);
                }
                else
                {
                    inc_ = 0.0;
                }

                mi = mi_;
                ma = ma_;
                inc = inc_;
            }
            else
            {

                // value camera dependent
                retVal += checkError(NInfoAsFloat(node, NI_Min, mi), name);
                retVal += checkError(NInfoAsFloat(node, NI_Max, ma), name);
                cvbres_t res = NInfoAsFloat(node, NI_Increment, inc);
                if (CVC_ERROR_FROM_HRES(res) != CVC_E_NOTSUPPORTED)
                {
                    retVal += checkError(res, name);
                }
                else
                {
                    inc = 0.0;
                }
            }

            meta.setMin(mi * scale);
            meta.setMax(ma * scale);
            meta.setStepSize(inc * scale);

            ReleaseObject(node);
        }
    }
    else
    {
        retVal += ito::RetVal(ito::retError, 0, tr("node map not avaible").toLatin1().data());
    }

    return retVal;
}

//-----------------------------------------------------------------------------------------------------
ito::RetVal CommonVisionBlox::getParamIntInfo(const char *name, ito::IntMeta &meta)
{
    ito::RetVal retVal;

    if (m_hNodeMap)
    {
        // get width feature node
        NODE node = NULL;
        retVal += checkError(NMGetNode(m_hNodeMap, name, node), name);
        if (!retVal.containsError())
        {
            cvbint64_t mi, ma, inc;
            // value camera dependent
            retVal += checkError(NInfoAsInteger(node, NI_Min, mi), name);
            retVal += checkError(NInfoAsInteger(node, NI_Max, ma), name);
            cvbres_t res = NInfoAsInteger(node, NI_Increment, inc);
            if (CVC_ERROR_FROM_HRES(res) != CVC_E_NOTSUPPORTED)
            {
                retVal += checkError(res, name);
            }
            else
            {
                inc = 0;
            }

            meta.setMin(mi);
            meta.setMax(ma);
            meta.setStepSize(inc);

            ReleaseObject(node);
        }
    }
    else
    {
        retVal += ito::RetVal(ito::retError, 0, tr("node map not avaible").toLatin1().data());
    }

    return retVal;
}

//--------------------------------------------------------------------------------------
ito::RetVal CommonVisionBlox::getParamEnumerationInfo(const char *name, ito::StringMeta &meta)
{
    ito::RetVal retVal;

    if (m_hNodeMap)
    {
        // get width feature node
        NODE node = NULL;
        retVal += checkError(NMGetNode(m_hNodeMap, name, node), name);
        if (!retVal.containsError())
        {
            TNodeType nodeType;
            NType(node, nodeType);
            cvbdim_t nodeCount;
            meta = ito::StringMeta(ito::StringMeta::String);
            size_t entrySize = 128;
            char entry[128] = {0};

            if (nodeType == NT_Enumeration)
            {
                NListCount(node, NL_EnumEntry, nodeCount);

                for (cvbdim_t c = 0; c < nodeCount; ++c)
                {
                    NList(node, NL_EnumEntry, c, entry, entrySize);
                    meta.addItem(entry);
                }
            }
            else
            {
                retVal += ito::RetVal::format(ito::retError, 0, tr("node %s is no enumeration").toLatin1().data(), name);
            }
        }

        ReleaseObject(node);
    }
    else
    {
        retVal += ito::RetVal(ito::retError, 0, tr("node map not avaible").toLatin1().data());
    }

    return retVal;
}

//--------------------------------------------------------------------------------------
bool CommonVisionBlox::nodeExists(const char *name) const
{
    if (m_hNodeMap)
    {
        // get width feature node
        NODE node = NULL;
        if (checkError(NMGetNode(m_hNodeMap, name, node)).containsError())
        {
            ReleaseObject(node);
            return false;
        }
        else
        {
            ReleaseObject(node);
            return true;
        }
    }

    return false;
}

//--------------------------------------------------------------------------------------
// sets the ini-file entry which populates the ini with currently attached
// cameras
// (the flag is auto-reset when the driver was loaded)
ito::RetVal CommonVisionBlox::scan_for_cameras()
{
    char iniPath[DRIVERPATHSIZE] = { 0 };
    TranslateFileName("%CVBDATA%\\Drivers\\GenICam.ini", iniPath, DRIVERPATHSIZE);

    BOOL result = WritePrivateProfileStringA("SYSTEM", "CreateAutoIni", "1", iniPath);

    if (!result)
    {
        return ito::RetVal(ito::retError, 0, tr("failure while scanning for cameras").toLatin1().data());
    }
    return ito::retOk;
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal CommonVisionBlox::checkError(const cvbres_t &code, const char *prefix) const
{
    if (code >= 0)
    {
        return ito::retOk;
    }
    else
    {
        int code_ = CVC_ERROR_FROM_HRES(code);
        const char *prefix_ = prefix ? prefix : "";
        const char *prefix2_ = prefix ? ": " : "";

        switch (code_)
        {
        case CVC_E_ERROR :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sAn unspecified error occurred for which no further information is available. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_PARAMETER :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sA function was called with an invalid parameter value for one of the function's arguments. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_FILEIO :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sAn error occurred during a File I/O operation (e.g. while loading/saving data from/to a file). (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_TIMEOUT :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sA timeout occurred in an asynchronous function call. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_MEMORY :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sThe allocation of a block of memory failed, typically because the amount of memory available to the application was insufficient for the operation at hand. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_INVALIDPLANE :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sA plane index was specified that was either negative or bigger than or equal to the number of planes in the image. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_UNSUPPORTEDDATATYPE :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sThe data type of the input image is not supported by the function that has been called. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_INVALIDCAMERAPORT :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sAn attempt to switch to an invalid/unavailable camera port was made. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_BOARDSELECT :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sAn attempt to switch to an invalid/unavailable board was made. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_INVALIDTRIGGERMODE :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sAn attempt to select and invalid/unsupported trigger mode was made. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_PROPERTYREAD :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sAn attempt to read a property was unsuccessful, either because the property does not exist or because the device or property is in a state that prevents the property from being read. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_PROPERTYWRITE :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sAn attempt to write a property was unsuccessful, either because the property does not exist or because the device or property is in a state that prevents the property from being written. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_INVALIDPORT :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sAn invalid/unavailable port was selected. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_PORTREAD :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sA port read operation failed. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_PORTWRITE :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sA port write operation failed. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_NOIMAGE :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sThe handle to at least one of the input images does not point to a valid image. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_NOINTERFACE :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sThe input image object does not support the interface required for this operation. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_BUSY :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sThe operation failed because the hardware is not in a state where it can handle that operation. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_NOTSUPPORTED :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sThe requested feature is not supported. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_GRABABORTED :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sThe current grab operation was aborted. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_NOPIXELLIST :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sAt least one of the input handles does not point to a valid pixel list. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_NOTENOUGHDATA :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sThe amount of input data is insufficient for the requested operation. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_NOTRANSFORMATION :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sThe object handle that was passed to the function does not point to a valid non linear transformation object. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_LINEAR_ONLY :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sThe function only works on images with a linear VPAT layout. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_DIVISIONBYZERO :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sA division by zero was attempted. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_INVALIDDIMENSION :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sAn invalid number of planes was specified. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_INVALIDCOLORMODEL :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sAn invalid/undefined color model mode has been passed to the function. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_INVALIDDATATYPE :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sAn invalid data type descriptor has been passed to the function. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_WRONGOBJECT :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sAt least one of the input handles points to the wrong type of object. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_NOTREADY :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sThe device or object is not ready to handle the requested operation. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_NOANGLE :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sThe input handle does not point to a valid angle object. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_NOVECTOR2D :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sThe input handle does not point to a valid 2D vector object. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_NOLINE2D :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sThe input handle does not point to a valid 2D line object. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_VECTOR2D_ZERO_LENGTH :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sThe operation failed because the vector passed to this function has length zero. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_VECTORS_IDENTICAL :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sThe function call failed because the input vectors to this operation are identical. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_LINE2D_VERTICAL :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sThe operation failed because the line object passed to it is vertical. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_LINE2D_HORIZONTAL :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sThe operation failed because the line object passed to it is horizontal. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_NOARGUMENT :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sThe argument cannot be calculated for a vector of length zero. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_LINE2D_UNDEFINED :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sThe operation failed because the input line 2D object has not yet been defined  (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_NOINTERSECTION :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sThe operation failed because the the line objects provided to the function do not intersect  (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_NOCLIPPING :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sNo clipping points available. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_NOTENOUGHLINES :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sNot enough lines available to calculate the intersection reliably. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_OVERFLOW :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sAn input value was too big or did lead to a too big result. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_NOCIRCLE :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sThe input handle does not point to a valid 2D circle object. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_ACCESS :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sA feature access failed. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_NOTPRESENT :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sThe requested operation failed because the selected feature is not present. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_UNSUPPORTEDFEATURE :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sThe requested feature is not supported (may happen if a specific interface implementation does not implement all functionality). (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_INVALIDINDEX :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sAn index value in an indexed access exceeded its limits. (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        case CVC_E_NOOVERLAY :
            return ito::RetVal::format(ito::retError, code_, tr("%s%sAn image object that is expected to have overlay bits does not have overlay bits (see #DT_Overlay). (%i)").toLatin1().data(), prefix_, prefix2_, CVC_ERROR_FROM_HRES(code));
        default:
            return ito::RetVal::format(ito::retError, code_, tr("%s%sCommon Vision Blox Error %i").toLatin1().data(), CVC_ERROR_FROM_HRES(code));
        }
    }
}

/* ********************************************************************
    Plugin "HidApi" for itom software
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

#include "itomHidApi.h"
#include "dockWidgetHidApi.h"

#include <qstring.h>
#include <qbytearray.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>

#include "pluginVersion.h"
#include "gitVersion.h"

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomHidApiInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(ItomHidApi)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomHidApiInterface::closeThisInst(ito::AddInBase **addInInst)
{
   REMOVE_PLUGININSTANCE(ItomHidApi)
   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ItomHidApiInterface::ItomHidApiInterface()
{
    m_type = ito::typeDataIO | ito::typeRawIO;
    setObjectName("HidApi");

    m_description = tr("itom-plugin for a usb HID API communication");

    //for the docstring, please don't set any spaces at the beginning of the line.
    m_detaildescription = tr( \
"HidApi is a plugin which gives direct/raw access to HID compliant devices (e.g. via USB).\n\
It can be used by plugins for communication analog to the serial port.\n\
The plugin is implemented for Windows, Linux and Mac.\n\
\n\
To connect to a device you need the vendor id and the product id.\n\
\n\
The setVal and getVal functions will write and read on the output or on the feature.");

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
    m_aboutThis = QObject::tr(GITVERSION);

    ito::Param paramVal("vendor_id", ito::ParamBase::Int, 0, std::numeric_limits<unsigned short>::max(), 0, tr("The vendor id of the device to connect to (0 will return a list of all devices if 'print_info_about_all_devices' is 1).").toLatin1().data());
    m_initParamsMand.append(paramVal);
    paramVal = ito::Param("product_id", ito::ParamBase::Int, 0, std::numeric_limits<unsigned short>::max(), 0, tr("The product id of the device to connect to (0 will return a list of all devices if 'print_info_about_all_devices' is 1).").toLatin1().data());
    m_initParamsMand.append(paramVal);

    paramVal = ito::Param("serial_number", ito::ParamBase::String, "", tr("Optional serial number of device that should be opened.").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("print_info_about_all_devices", ito::ParamBase::Int, 0, 1, 0, tr("If true, all information about connected devices is print to the console.").toLatin1().data());
    m_initParamsOpt.append(paramVal);

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
ItomHidApiInterface::~ItomHidApiInterface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal ItomHidApi::showConfDialog(void)
{
/*
    dialogItomHidApi *confDialog = new dialogItomHidApi((void*)this);
    QVariant qvar = m_params["port"].getVal<double>();
    confDialog->setVals(&m_params);
    if (confDialog->exec())
    {
//        confDialog->getVals(&m_paramNames);
    }
    delete confDialog;
*/
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ItomHidApi::ItomHidApi() : AddInDataIO(), m_debugMode(false), m_pDevice(NULL)
{
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly | ito::ParamBase::NoAutosave, "HidApi", "name of device");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("debug", ito::ParamBase::Int, 0, 5, 0, tr("If true, all out and inputs are written to dockingWidget").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("manufacturer", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("manufacturer string").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("product", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("product string").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("serial_number", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("serial number string").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("use_feature_report_not_output", ito::ParamBase::Int, 0, 1, 0, tr("if true, getVal and setVal will operate on feature reports, else on the output buffer (default)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    if (hasGuiSupport())
    {
        //now create dock widget for this plugin
        DockWidgetHidApi *dw = new DockWidgetHidApi(m_params, getID());
        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dw);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ItomHidApi::~ItomHidApi()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomHidApi::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);

    ito::RetVal retval(ito::retOk);

    int vendorId = paramsMand->at(0).getVal<int>();
    int productId = paramsMand->at(1).getVal<int>();
    const char *serialNumber = paramsOpt->at(0).getVal<char*>();
    if (strcmp(serialNumber, "") == 0)
    {
        serialNumber = NULL;
    }
    bool printInfo = paramsOpt->at(1).getVal<int>() > 0;

    if (printInfo)
    {
        struct hid_device_info *devs, *cur_dev;
        int idx = 0;
        devs = hid_enumerate(vendorId, productId);
        cur_dev = devs;
        while (cur_dev)
        {
            QString output = QString("Device %1: vendor_id: %2, product_id: %3, path: %4, serial_number: %5, manufacturer: %6, product: %7\n").arg(idx).arg(cur_dev->vendor_id).\
                arg(cur_dev->product_id).arg(cur_dev->path).arg(QString::fromWCharArray(cur_dev->serial_number)).arg(QString::fromWCharArray(cur_dev->manufacturer_string)).arg(QString::fromWCharArray(cur_dev->product_string));
            std::cout << output.toLatin1().data() << std::endl;
            cur_dev = cur_dev->next;
        }
        hid_free_enumeration(devs);
    }

    // Open the device using the VID, PID,
    // and optionally the Serial number.
    wchar_t *serial = NULL;
    if (serialNumber)
    {
        serial = new wchar_t[strlen(serialNumber) + 4];
        memset(serial, 0, sizeof(wchar_t) * (strlen(serialNumber) + 4));
        QString(serialNumber).toWCharArray(serial);
    }
    m_pDevice = hid_open(vendorId, productId, serial);
    delete serial;
    serial = NULL;

    if (!m_pDevice)
    {
        retval += ito::RetVal(ito::retError, 0, "no valid device could be found and opened");
    }
    else
    {
        #define MAX_STR 255
        wchar_t wstr[MAX_STR];
        // Read the Manufacturer String
        if (hid_get_manufacturer_string(m_pDevice, wstr, MAX_STR) >= 0)
        {
            m_params["manufacturer"].setVal<char*>(QString::fromWCharArray(wstr).toLatin1().data());
        }

        // Read the Product String
        if (hid_get_product_string(m_pDevice, wstr, MAX_STR) >= 0)
        {
            m_params["product"].setVal<char*>(QString::fromWCharArray(wstr).toLatin1().data());
        }

        // Read the Serial Number String
        if (hid_get_serial_number_string(m_pDevice, wstr, MAX_STR) >= 0)
        {
            m_params["serial_number"].setVal<char*>(QString::fromWCharArray(wstr).toLatin1().data());
        }

        // Set the hid_read() function to be non-blocking.
        hid_set_nonblocking(m_pDevice, 1);
    }

    if (!retval.containsError()) /* We need to claim the first interface */
    {
        emit parametersChanged(m_params);
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    setInitialized(true); //init method has been finished (independent on retval)
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomHidApi::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval;

    if (m_pDevice)
    {
        hid_close(m_pDevice);
        m_pDevice = NULL;
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
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
ito::RetVal ItomHidApi::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
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
        //if the requested parameter name has an index, e.g. roi[0], then the sub-value of the
        //array is split and returned using the api-function apiGetParam
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
ito::RetVal ItomHidApi::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
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
        retValue += apiValidateAndCastParam(*it, *val, false, true, true);
    }

    if (!retValue.containsError())
    {
        if (key == "debug")
        {
            //check the new value and if ok, assign it to the internal parameter
            retValue += it->copyValueFrom( &(*val) );

            if (!retValue.containsError())
            {
                m_debugMode = it->getVal<int>() > 0;
                //libusb_set_debug(NULL, it->getVal<int>());
            }
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
ito::RetVal ItomHidApi::startDevice(ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval = ito::RetVal(ito::retWarning, 0, tr("StartDevice not necessary").toLatin1().data());

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
        waitCond->deleteSemaphore();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomHidApi::stopDevice(ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval = ito::RetVal(ito::retWarning, 0, tr("StopDevice not necessary").toLatin1().data());

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
        waitCond->deleteSemaphore();
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomHidApi::acquire(const int /*trigger*/, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval = ito::RetVal(ito::retWarning, 0, tr("Acquire not necessary").toLatin1().data());

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
        waitCond->deleteSemaphore();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomHidApi::getVal(QSharedPointer<char> data, QSharedPointer<int> length, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval;

    int numread;
    if (m_params["use_feature_report_not_output"].getVal<int>())
    {
        numread = hid_get_feature_report(m_pDevice, (unsigned char*)data.data(), *length);
    }
    else
    {
        numread = hid_read(m_pDevice, (unsigned char*)data.data(), *length);
    }

    if (numread < 0)
    {
        QString err = QString::fromWCharArray(hid_error(m_pDevice));
        retval += ito::RetVal::format(ito::retError, 0, "error reading from HID device: %s", err.toLatin1().data());
        numread = 0;
    }

    *length = numread;

    if (m_debugMode)
    {
        emit serialLog(QByteArray(data.data(),*length), '<');
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomHidApi::setVal(const char *data, const int datalength, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval(ito::retOk);

    int actual_length = 0;

    if (m_debugMode)
    {
        emit serialLog(QByteArray(data, datalength), '>');
    }

    int res;
    if (m_params["use_feature_report_not_output"].getVal<int>())
    {
        res = hid_send_feature_report(m_pDevice, (const unsigned char*)data, datalength);
    }
    else
    {
        res = hid_write(m_pDevice, (const unsigned char*)data, datalength);
    }

    if (res < 0)
    {
        QString err = QString::fromWCharArray(hid_error(m_pDevice));
        retval += ito::RetVal::format(ito::retError, 0, "error writing to HID device: %s", err.toLatin1().data());
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomHidApi::execFunc(const QString funcName, QSharedPointer<QVector<ito::ParamBase> > paramsMand, QSharedPointer<QVector<ito::ParamBase> > paramsOpt, QSharedPointer<QVector<ito::ParamBase> > paramsOut, ItomSharedSemaphore *waitCond /*= NULL*/)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval;

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomHidApi::dockWidgetVisibilityChanged(bool visible)
{
    if (getDockWidget())
    {

        DockWidgetHidApi *dw = qobject_cast<DockWidgetHidApi*>(getDockWidget()->widget());
        if (visible)
        {
            connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), dw, SLOT(valuesChanged(QMap<QString, ito::Param>)));
            connect(this, SIGNAL(serialLog(QByteArray, const char)), dw, SLOT(serialLog(QByteArray, const char)));

            emit parametersChanged(m_params);
        }
        else
        {
            disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), dw, SLOT(valuesChanged(QMap<QString, ito::Param>)));
            disconnect(this, SIGNAL(serialLog(QByteArray, const char)), dw, SLOT(serialLog(QByteArray, const char)));
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------

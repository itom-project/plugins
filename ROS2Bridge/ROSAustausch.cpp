/* ********************************************************************
    Plugin "ItomUSBDevice" for itom software
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

#include "ItomLibUSB.h"
#include "dockWidgetLibUSB.h"

#define _USE_MATH_DEFINES  // needs to be defined to enable standard declarations of PI constant
#include "math.h"

#include <qstring.h>
#include <qbytearray.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>

#include "pluginVersion.h"
#include "gitVersion.h"

//#include <qdebug.h>
//#include <qmessagebox.h>

#ifndef linux
    #include <Windows.h>
#endif

/*static*/ QVector<USBDevice> ItomUSBDevice::openedDevices;
/*static*/ QMutex ItomUSBDevice::openedDevicesReadWriteMutex;

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomUSBDeviceInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(ItomUSBDevice)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomUSBDeviceInterface::closeThisInst(ito::AddInBase **addInInst)
{
   REMOVE_PLUGININSTANCE(ItomUSBDevice)
   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ItomUSBDeviceInterface::ItomUSBDeviceInterface()
{
    m_type = ito::typeDataIO | ito::typeRawIO;
    setObjectName("LibUSB");

    m_description = tr("itom-plugin for a usb port communication");

    //for the docstring, please don't set any spaces at the beginning of the line.
/*    char docstring[] = \
"LibUSB is a itom-Plugin which gives direct/raw access to a device connected to the serial port.\nIt can be used by plugins for communication analog to the serial port.\n\
The plugin is implemented for Windows, but Linux should be possible due to libUSB is also available on Linux.\n\
\n\
To connect to a device you need the vendor id and the product id.\n\
\n\
The setVal and getVal functions will write and read on the specified endpoint.";
    m_detaildescription = tr(docstring);*/
    m_detaildescription = tr(
"LibUSB is a itom-Plugin which gives direct/raw access to a device connected to the serial port.\nIt can be used by plugins for communication analog to the serial port.\n\
The plugin is implemented for Windows, but Linux should be possible due to libUSB is also available on Linux.\n\
\n\
To connect to a device you need the vendor id and the product id.\n\
\n\
The setVal and getVal functions will write and read on the specified endpoint.");

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
    m_aboutThis = QObject::tr(GITVERSION);

    ito::Param paramVal("VendorID", ito::ParamBase::Int, 0, std::numeric_limits<unsigned short>::max(), 0x1cbe, tr("The vendor id of the device to connect to").toLatin1().data());
    m_initParamsMand.append(paramVal);
    paramVal = ito::Param("ProductID", ito::ParamBase::Int, 0, std::numeric_limits<unsigned short>::max(), 0x0003, tr("The product id of the device to connect to").toLatin1().data());
    m_initParamsMand.append(paramVal);
    paramVal = ito::Param("endpoint", ito::ParamBase::Int, 0, 127, 1, tr("The endpoint to communicate with.").toLatin1().data());
    m_initParamsMand.append(paramVal);

    paramVal = ito::Param("timeout", ito::ParamBase::Double, 0.0, 65.0, 4.0, tr("Timeout for reading commands in [s]").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("debugLevel", ito::ParamBase::Int, 0, 5, 0, tr("Debug level: 0 (LIBUSB_LOG_LEVEL_NONE): no messages ever printed by the library. 1 (ERROR): error messages are printed to stderr, 2 (WARNING): warning and error messages are printed to stderr, 3 (INFO): informational messages are printed to stdout, warning and error messages are printed to stderr, 4 (DEBUG): like 3 but debug messages are also printed to stdout.").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("printInfoAboutAllDevices", ito::ParamBase::Int, 0, 1, 0, tr("If true, all information about connected devices is print to the console.").toLatin1().data());
    m_initParamsOpt.append(paramVal);

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
ItomUSBDeviceInterface::~ItomUSBDeviceInterface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal ItomUSBDevice::showConfDialog(void)
{
/*
    dialogItomUSBDevice *confDialog = new dialogItomUSBDevice((void*)this);
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
ItomUSBDevice::ItomUSBDevice() : AddInDataIO(), m_debugMode(false), m_pDevice(NULL), m_autoDetach(true), m_timeoutMS(4000), m_endpoint_read(1), m_endpoint_write(1)
{
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly | ito::ParamBase::NoAutosave, "ItomUSBDevice", "name of device");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("timeout", ito::ParamBase::Double | ito::ParamBase::NoAutosave, 0.0, 65.0, 4.0, tr("Timeout for reading commands in [s]").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("debug", ito::ParamBase::Int, 0, 5, 0, tr("If true, all out and inputs are written to dockingWidget").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("endpoint_read", ito::ParamBase::Int, 0, 255, 1, tr("Endpoint index for reading operations. The used index is LIBUSB_ENDPOINT_IN + endpoint_read, with LIBUSB_ENDPOINT_IN = %1 (default: initialization parameter 'endpoint')").arg(LIBUSB_ENDPOINT_IN).toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("endpoint_write", ito::ParamBase::Int, 0, 255, 1, tr("Endpoint index for writing operations. The used index is LIBUSB_ENDPOINT_OUT + endpoint_write, with LIBUSB_ENDPOINT_OUT = %1  (default: initialization parameter 'endpoint')").arg(LIBUSB_ENDPOINT_OUT).toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    if (hasGuiSupport())
    {
        //now create dock widget for this plugin

        //now create dock widget for this plugin
        DockWidgetLibUSB *dw = new DockWidgetLibUSB(m_params, getID());
        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dw);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ItomUSBDevice::~ItomUSBDevice()
{
   m_params.clear();
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
ito::RetVal ItomUSBDevice::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
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
ito::RetVal ItomUSBDevice::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
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
        if (key == "timeout")
        {
            //check the new value and if ok, assign it to the internal parameter
            retValue += it->copyValueFrom( &(*val) );

            if (!retValue.containsError())
            {
                m_timeoutMS = (int)(it->getVal<double>() * 1000 + 0.5);
            }
        }
        else if (key == "debug")
        {
            //check the new value and if ok, assign it to the internal parameter
            retValue += it->copyValueFrom( &(*val) );

            if (!retValue.containsError())
            {
                m_debugMode = it->getVal<int>() > 0;
                libusb_set_debug(NULL, it->getVal<int>());
            }
        }
        else if (key == "endpoint_read")
        {
            //check the new value and if ok, assign it to the internal parameter
            retValue += it->copyValueFrom( &(*val) );

            if (!retValue.containsError())
            {
                m_endpoint_read = it->getVal<int>();
            }
        }
        else if (key == "endpoint_write")
        {
            //check the new value and if ok, assign it to the internal parameter
            retValue += it->copyValueFrom( &(*val) );

            if (!retValue.containsError())
            {
                m_endpoint_write = it->getVal<int>();
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
ito::RetVal ItomUSBDevice::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);

    ito::RetVal retval(ito::retOk);

    const char *device_id = NULL;
    const char *device_path = getenv("DEVICE");
    const char *type = NULL;

    int status = 0;

    libusb_device *currentDevice = NULL;
    libusb_device **deviceList = NULL;

    retval += m_params["endpoint_read"].copyValueFrom(&((*paramsMand)[2]));
    retval += m_params["endpoint_write"].copyValueFrom(&((*paramsMand)[2]));

    retval += m_params["timeout"].copyValueFrom(&((*paramsOpt)[0]));
    retval += m_params["debug"].copyValueFrom(&((*paramsOpt)[1]));

    m_timeoutMS = (int)(m_params["timeout"].getVal<double>() * 1000 + 0.5);
    m_endpoint_read = m_params["endpoint_read"].getVal<int>();
    m_endpoint_write = m_params["endpoint_write"].getVal<int>();
    m_debugMode = m_params["debug"].getVal<int>() > 0;
    bool printInfo = paramsOpt->at(2).getVal<int>() > 0;

    uint16_t vid = paramsMand->at(0).getVal<int>() & 0x0000FFFF;
    uint16_t pid = paramsMand->at(1).getVal<int>() & 0x0000FFFF;

    unsigned int busnum = 0, devaddr = 0, tmpBusnum, tmpDevaddr;

    /* open the device using libusb */
    if (!retval.containsError())
    {
        status = libusb_init(NULL);
        if (status < 0)
        {
            retval += ito::RetVal(ito::retError, status, libusb_error_name(status));
        }
        else
        {
            libusb_set_debug(NULL, m_params["debug"].getVal<int>());
        }
    }

    if (!retval.containsError())
    {
        status = libusb_get_device_list(NULL, &deviceList); //deviceList must be freed with libusb_free_device_list (if != NULL)
        if (status < 0)
        {
            retval += ito::RetVal(ito::retError, status, libusb_error_name(status));
        }
        else
        {
            int listIdx = 0;
            struct libusb_device_descriptor desc;
            QMap<int,USBDevice> possibleIndices;
            USBDevice device;

            while(deviceList[listIdx] != NULL)
            {
                currentDevice = deviceList[listIdx];

                status = libusb_get_device_descriptor(currentDevice, &desc);
                if (status < 0)
                {
                    retval += ito::RetVal(ito::retError, status, libusb_error_name(status));
                }
                else
                {
                    tmpBusnum = libusb_get_bus_number(currentDevice);
                    tmpDevaddr = libusb_get_device_address(currentDevice);

                    if ((type != NULL) && (device_path != NULL))
                    {
                        tmpBusnum = libusb_get_bus_number(currentDevice);
                        tmpDevaddr = libusb_get_device_address(currentDevice);

                        // if both a type and bus,addr were specified, we just need to find our match
                        if ((tmpBusnum == busnum) && (tmpDevaddr == devaddr))
                        {
                            device = USBDevice(desc.idVendor, desc.idProduct, tmpBusnum, tmpDevaddr);
                            possibleIndices[listIdx] = device;
                            break;
                        }
                    }
                    else if (vid > 0 && pid > 0 && desc.idVendor == vid && desc.idProduct == pid)
                    {
                        device = USBDevice(desc.idVendor, desc.idProduct, tmpBusnum, tmpDevaddr);
                        possibleIndices[listIdx] = device;
                    }

                    if (printInfo)
                    {
                        char s[128] = {0};
                        char m[128] = {0};
                        char t[128] = {0};

                        if (libusb_open(deviceList[listIdx], &m_pDevice) >= 0)
                        {
                            libusb_get_string_descriptor_ascii(m_pDevice, desc.iSerialNumber, (unsigned char*)s, 128);
                            libusb_get_string_descriptor_ascii(m_pDevice, desc.iManufacturer, (unsigned char*)m, 128);
                            libusb_get_string_descriptor_ascii(m_pDevice, desc.iProduct, (unsigned char*)t, 128);

                            QString output = QString("Device %1: Bus: %2, Address %3, VendorID: %4, ProductID: %5, %6, %7, %8\n").arg(listIdx).arg(tmpBusnum).arg(tmpDevaddr).arg(desc.idVendor).arg(desc.idProduct).arg(m).arg(t).arg(s);
                            std::cout << output.toLatin1().data() << std::endl;

                            libusb_close(m_pDevice);
                            m_pDevice = NULL;
                        }
                        else
                        {
                            QString output = QString("Device %1: Bus: %2, Address %3, VendorID: %4, ProductID: %5, no further information\n").arg(listIdx).arg(tmpBusnum).arg(tmpDevaddr).arg(desc.idVendor).arg(desc.idProduct);
                            std::cout << output.toLatin1().data() << std::endl;
                        }
                    }
                }

                listIdx++;
            }

            int indexToOpen = -1;

            if (possibleIndices.size() == 0)
            {
                retval += ito::RetVal(ito::retError, 0, tr("could not find a known device - please specify type and/or vid:pid and/or bus,dev").toLatin1().data());
            }
            else if (possibleIndices.size() == 1)
            {
                QMap<int, USBDevice>::const_iterator i = possibleIndices.constBegin();
                indexToOpen = i.key();
            }
            else /*if (possibleIndices.size() > 1)*/
            {
                openedDevicesReadWriteMutex.lock();
                QMap<int, USBDevice>::const_iterator i = possibleIndices.constBegin();

                while (i != possibleIndices.constEnd())
                {
                    if (openedDevices.contains(i.value()) == false)
                    {
                        //not yet opened, open it
                        indexToOpen = i.key();
                        break;
                    }
                    i++;
                }

                if (indexToOpen < 0)
                {
                    retval += ito::RetVal(ito::retError, 0, tr("no of the %1 devices that fit to the vendor and product ID can be opened since they are already in use.").arg(possibleIndices.size()).toLatin1().data());
                }

                openedDevicesReadWriteMutex.unlock();
            }

            if (indexToOpen == -1)
            {
                retval += ito::RetVal(ito::retError, 0, tr("no device found to open").toLatin1().data());
            }
            else if (!retval.containsError())
            {
                status = libusb_open(deviceList[indexToOpen], &m_pDevice);
                if (status < 0)
                {
                    retval += ito::RetVal(ito::retError, status, libusb_error_name(status));
                }
                else
                {
                    if (libusb_get_device_descriptor(deviceList[indexToOpen], &desc) >= 0)
                    {
                        char s[128] = {0};
                        char m[128] = {0};
                        char t[128] = {0};

                        libusb_get_string_descriptor_ascii(m_pDevice, desc.iSerialNumber, (unsigned char*)s, 128);
                        libusb_get_string_descriptor_ascii(m_pDevice, desc.iManufacturer, (unsigned char*)m, 128);
                        libusb_get_string_descriptor_ascii(m_pDevice, desc.iProduct, (unsigned char*)t, 128);

                        QString serial = s;
                        QString manufacturer = m;
                        QString product = t;
                        setIdentifier(QString("%1, %2 (%3)").arg(manufacturer.trimmed(),product.trimmed(),serial.trimmed()));
                    }

                    openedDevicesReadWriteMutex.lock();
                    m_currentDevice = possibleIndices[indexToOpen];
                    openedDevices.append(possibleIndices[indexToOpen]);
                    openedDevicesReadWriteMutex.unlock();
                }
            }
        }

        if (deviceList != NULL)
        {
            libusb_free_device_list(deviceList, 1);
        }
    }

    if (!retval.containsError()) /* We need to claim the first interface */
    {
        int test = 0;

        m_autoDetach  = libusb_set_auto_detach_kernel_driver(m_pDevice, 1) == 0;
        if (!m_autoDetach)
        {
            libusb_detach_kernel_driver(m_pDevice, 0);
        }
        struct libusb_config_descriptor *conf_desc = NULL;
        status = libusb_get_active_config_descriptor(libusb_get_device(m_pDevice), &conf_desc);

        if (conf_desc == NULL)
        {
            status = libusb_get_config_descriptor(libusb_get_device(m_pDevice), 1, &conf_desc);

            if (conf_desc)
            {
                test = conf_desc->bConfigurationValue;
            }
        }

        if (printInfo && conf_desc)
        {
            std::cout << "Interfaces: " << (int)conf_desc->bNumInterfaces <<"\n-----------------------------\n" << std::endl;
            const libusb_interface *inter;
            const libusb_interface_descriptor *interdesc;
            const libusb_endpoint_descriptor *epdesc;
            for(int i = 0; i<(int)conf_desc->bNumInterfaces; i++)
            {
                inter = &conf_desc->interface[i];
                std::cout<<"Number of alternate settings: "<<inter->num_altsetting<<"\n" << std::endl;
                for(int j=0; j<inter->num_altsetting; j++)
                {
                    interdesc = &inter->altsetting[j];
                    std::cout<<"    Interface Number: "<<(int)interdesc->bInterfaceNumber<<"\n";
                    std::cout<<"    Number of endpoints: "<<(int)interdesc->bNumEndpoints<<"\n" << std::endl;

                    for(int k=0; k<(int)interdesc->bNumEndpoints; k++)
                    {
                        epdesc = &interdesc->endpoint[k];
                        std::cout << "        Descriptor Type: "<<(int)epdesc->bDescriptorType<<"\n";
                        std::cout << "        EP Address: "<<(int)epdesc->bEndpointAddress<<"\n";
                        if (epdesc->bEndpointAddress > LIBUSB_ENDPOINT_IN)
                        {
                            std::cout << "        EP Address (IN): " << LIBUSB_ENDPOINT_IN << "+" << ((int)epdesc->bEndpointAddress - LIBUSB_ENDPOINT_IN) << "\n";
                        }
                        else
                        {
                            std::cout << "        EP Address (OUT): " << LIBUSB_ENDPOINT_OUT << "+" << ((int)epdesc->bEndpointAddress - LIBUSB_ENDPOINT_OUT) << "\n";
                        }
                        std::cout<<"        Attributes: "<<(int)epdesc->bmAttributes<<"\n" << std::endl;
                    }
                }
            }
        }

        status = libusb_set_configuration(m_pDevice, test);
        if (status < 0)
        {
            status = libusb_set_configuration(m_pDevice, 1);
            if (status < 0)
            {
                status = libusb_set_configuration(m_pDevice, 0);
                if (status < 0)
                {
                    retval += ito::RetVal(ito::retError, status, libusb_error_name(status));
                }
            }
        }

        test = 0;
        status = libusb_claim_interface(m_pDevice, test);
        if (status < 0)
        {
            retval += ito::RetVal(ito::retError, status, libusb_error_name(status));
        }
        libusb_free_config_descriptor(conf_desc);
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
ito::RetVal ItomUSBDevice::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval;

    if (m_pDevice)
    {
        openedDevicesReadWriteMutex.lock();

        QVector<USBDevice>::iterator it = openedDevices.begin();
        while (it != openedDevices.end())
        {
            if ((*it) == m_currentDevice)
            {
                openedDevices.erase(it);
                break;
            }
            it++;
        }
        openedDevicesReadWriteMutex.unlock();

        libusb_release_interface(m_pDevice, 0);
        if (!m_autoDetach)
        {
            libusb_attach_kernel_driver(m_pDevice, 0);
        }
        libusb_close(m_pDevice);
    }
    libusb_exit(NULL);

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomUSBDevice::startDevice(ItomSharedSemaphore *waitCond)
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
ito::RetVal ItomUSBDevice::stopDevice(ItomSharedSemaphore *waitCond)
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
ito::RetVal ItomUSBDevice::acquire(const int /*trigger*/, ItomSharedSemaphore *waitCond)
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
ito::RetVal ItomUSBDevice::getVal(QSharedPointer<char> data, QSharedPointer<int> length, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval;
    int actual_length = 0;

    int status = libusb_bulk_transfer(m_pDevice, LIBUSB_ENDPOINT_IN + m_endpoint_read, (unsigned char*)data.data(), *length, &actual_length, m_timeoutMS);
    if (status != LIBUSB_SUCCESS)
    {
        retval += ito::RetVal(ito::retError, status, libusb_error_name(status));
    }

    *length = actual_length;

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
ito::RetVal ItomUSBDevice::setVal(const char *data, const int datalength, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval(ito::retOk);

    int actual_length = 0;

    if (m_debugMode)
    {
        emit serialLog(QByteArray(data, datalength), '>');
    }

    int status = libusb_bulk_transfer(m_pDevice, LIBUSB_ENDPOINT_OUT + m_endpoint_write, (unsigned char*)data, datalength, &actual_length, m_timeoutMS);
    if (status != LIBUSB_SUCCESS)
    {
        retval += ito::RetVal(ito::retError, status, libusb_error_name(status));
    }
    else if (actual_length != datalength)
    {
        retval += ito::RetVal(ito::retError, 0, tr("Number of written characters differ from designated size").toLatin1().data());
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomUSBDevice::execFunc(const QString funcName, QSharedPointer<QVector<ito::ParamBase> > paramsMand, QSharedPointer<QVector<ito::ParamBase> > paramsOpt, QSharedPointer<QVector<ito::ParamBase> > paramsOut, ItomSharedSemaphore *waitCond /*= NULL*/)
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
void ItomUSBDevice::dockWidgetVisibilityChanged(bool visible)
{
    if (getDockWidget())
    {

        DockWidgetLibUSB *dw = qobject_cast<DockWidgetLibUSB*>(getDockWidget()->widget());
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

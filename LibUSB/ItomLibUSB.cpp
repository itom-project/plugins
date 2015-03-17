/* ********************************************************************
    Plugin "ItomUSBDevice" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2013, Institut für Technische Optik (ITO),
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

#define _USE_MATH_DEFINES  // needs to be defined to enable standard declartions of PI constant
#include "math.h"

#include <qstring.h>
#include <qbytearray.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>

#include "pluginVersion.h"

//#include <qdebug.h>
//#include <qmessagebox.h>

#ifndef linux
    #include <Windows.h>

#else

#endif


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomUSBDeviceInterface::getAddInInst(ito::AddInBase **addInInst)
{
    ItomUSBDevice* newInst = new ItomUSBDevice();
    newInst->setBasePlugin(this);
    *addInInst = qobject_cast<ito::AddInBase*>(newInst);

    m_InstList.append(*addInInst);

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomUSBDeviceInterface::closeThisInst(ito::AddInBase **addInInst)
{
   if (*addInInst)
   {
        delete ((ItomUSBDevice *)*addInInst);
        int idx = m_InstList.indexOf(*addInInst);
        m_InstList.removeAt(idx);
   }

   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ItomUSBDeviceInterface::ItomUSBDeviceInterface()
{
    m_type = ito::typeDataIO | ito::typeRawIO;
    setObjectName("LibUSB");

    m_description = tr("itom-plugin for a usb port communication");

    //for the docstring, please don't set any spaces at the beginning of the line.
    char docstring[] = \
"LibUSB is a itom-Plugin which gives direct/raw access to a device connected to the serial port.\nIt can be used by plugins for communication analog to the serial port.\n\
The plugin is implemented for Windows, but Linux should be possible due to libUSB is also availble on Linux.\n\
\n\
To connect to a device you need the vendor id and the product id.\n\
\n\
The setVal and getVal functions will write and read on the specified endpoint.";

    m_detaildescription = tr(docstring);
    m_author = "W. Lyda, twip optical solutions GmbH Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("licensed under LGPL");
    m_aboutThis = QObject::tr("This plugin can be used for raw / lowlevel comminication with USB-devices");  

    ito::Param paramVal("VendorID", ito::ParamBase::Int, 0, std::numeric_limits<unsigned short>::max(), 0x1cbe, tr("The vendor id of the device to connect to").toLatin1().data());
    m_initParamsMand.append(paramVal);
    paramVal = ito::Param("ProdictID", ito::ParamBase::Int, 0, std::numeric_limits<unsigned short>::max(), 0x0003, tr("The product id of the device to connect to").toLatin1().data());
    m_initParamsMand.append(paramVal);
    paramVal = ito::Param("endpoint", ito::ParamBase::Int, 0, 127, 1, tr("The endpoint to communicate with.").toLatin1().data());
    m_initParamsMand.append(paramVal);
    
    paramVal = ito::Param("timeout", ito::ParamBase::Double, 0.0, 65.0, 4.0, tr("Timeout for reading commands in [s]").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("enableDebug", ito::ParamBase::Int, 0, 5, 0, tr("Initialised 'debug'-parameter with given value. If debug-param is true, all out and inputs are written to dockingWidget").toLatin1().data());
    m_initParamsOpt.append(paramVal);

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
ItomUSBDeviceInterface::~ItomUSBDeviceInterface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
#if QT_VERSION < 0x050000
    Q_EXPORT_PLUGIN2(ItomUSBDeviceinterface, ItomUSBDeviceInterface)
#endif

//----------------------------------------------------------------------------------------------------------------------------------
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
ItomUSBDevice::ItomUSBDevice() : AddInDataIO(), m_debugMode(false), m_pDevice(NULL), m_autoDetach(true), m_timeoutMS(4000), m_endpoint(1)
{
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::NoAutosave, "ItomUSBDevice", NULL);
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("timeout", ito::ParamBase::Double | ito::ParamBase::NoAutosave, 0.0, 65.0, 4.0, tr("Timeout for reading commands in [s]").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("debug", ito::ParamBase::Int, 0, 5, 0, tr("If true, all out and inputs are written to dockingWidget").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("endpoint", ito::ParamBase::Int, 0, 255, 1, tr("If true, all out and inputs are written to dockingWidget").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    //register exec functions
    QVector<ito::Param> pMand;
    QVector<ito::Param> pOpt;
    QVector<ito::Param> pOut;
    registerExecFunc("clearInputBuffer", pMand, pOpt, pOut, tr("Clears the input buffer of serial port"));
    registerExecFunc("clearOutputBuffer", pMand, pOpt, pOut, tr("Clears the output buffer of serial port"));

    pMand << ito::Param("bufferType", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("Clears input (0) or output (1) buffer").toLatin1().data());
    registerExecFunc("clearBuffer", pMand, pOpt, pOut, tr("Clears the input or output buffer of serial port"));

    //now create dock widget for this plugin

    qRegisterMetaType<QMap<QString, ito::Param> >("QMap<QString, ito::Param>");

    //now create dock widget for this plugin
    DockWidgetLibUSB *dw = new DockWidgetLibUSB(m_params, getID() );
    Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
    QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
    createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dw);
}

//----------------------------------------------------------------------------------------------------------------------------------
ItomUSBDevice::~ItomUSBDevice()
{
   m_pThread->quit();
   m_pThread->wait(5000);
   delete m_pThread;
   m_pThread = NULL;

   m_params.clear();

   return;
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
    ito::RetVal retValue(ito::retOk);
    QString key = val->getName();

    if (key == "")
    {
        retValue += ito::RetVal(ito::retError, 0, tr("name of requested parameter is empty.").toLatin1().data());
    }
    else
    {
        QMap<QString, ito::Param>::const_iterator paramIt = m_params.constFind(key);
        if (paramIt != m_params.constEnd())
        {
            *val = paramIt.value();
        }
        else
        {
            retValue += ito::RetVal(ito::retError, 0, tr("parameter not found in m_params.").toLatin1().data());
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
    QString key = val->getName();

    if (key == "")
    {
        retValue += ito::RetVal(ito::retError, 0, tr("name of given parameter is empty.").toLatin1().data());
    }
    else
    {
        QMap<QString, ito::Param>::iterator paramIt = m_params.find(key);
        if (paramIt != m_params.end())
        {
          

            if ( /*paramIt->getFlags() & ito::ParamBase::Readonly*/ true)    //check read-only
            {
                retValue += ito::RetVal(ito::retWarning, 0, tr("Parameter is read only, input ignored").toLatin1().data());
                goto end;
            }
            else if (val->isNumeric() && paramIt->isNumeric())
            {
                double curval = val->getVal<double>();
                if (curval > paramIt->getMax())
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("New value is larger than parameter range, input ignored").toLatin1().data());
                    goto end;
                }
                else if (curval < paramIt->getMin())
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("New value is smaller than parameter range, input ignored").toLatin1().data());
                    goto end;
                }
                else
                {
                    paramIt.value().setVal<double>(curval);
                }
            }
            else if (paramIt->getType() == val->getType())
            {
                retValue += paramIt.value().copyValueFrom(&(*val));
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 0, tr("Parameter type conflict").toLatin1().data());
                goto end;
            }

            m_timeoutMS = (int)(m_params["timeout"].getVal<double>() * 1000 + 0.5);
            m_endpoint = m_params["endpoint"].getVal<int>();
            m_debugMode = m_params["debug"].getVal<int>() > 0;

        }
        else
        {
            retValue += ito::RetVal(ito::retError, 0, tr("Parameter not found").toLatin1().data());
        }
    }
    emit parametersChanged(m_params);

end:
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
    bool foundDevice = false;

    libusb_device *currentDevice = NULL, **deviceList = NULL;
	
    // mandatory parameters
    if (paramsMand == NULL)
    {
        retval += ito::RetVal(ito::retError, 0, QObject::tr("Mandatory paramers are NULL").toLatin1().data());
    }
    else
    {
        retval += m_params["endpoint"].copyValueFrom(&((*paramsMand)[2]));
    }
    // optional parameters
    if (paramsOpt == NULL)
    {
        retval = ito::RetVal(ito::retError, 0, QObject::tr("Optinal paramers are NULL").toLatin1().data());
    }
    else
    {
        retval += m_params["timeout"].copyValueFrom(&((*paramsOpt)[0]));
        retval += m_params["debug"].copyValueFrom(&((*paramsOpt)[1]));
    }

    m_timeoutMS = (int)(m_params["timeout"].getVal<double>() * 1000 + 0.5);
    m_endpoint = m_params["endpoint"].getVal<int>();
    m_debugMode = m_params["debug"].getVal<int>() > 0;

	unsigned int vid = (unsigned int)((*paramsMand)[0].getVal<int>() & 0x0000FFFF);
    unsigned int pid = (unsigned int)((*paramsMand)[1].getVal<int>() & 0x0000FFFF);

	unsigned int busnum = 0, devaddr = 0, tmpBusnum, tmpDevaddr;

    /* open the device using libusb */
    if(!retval.containsError())
    {
        status = libusb_init(NULL);
	    if (status < 0) 
        {
            retval += ito::RetVal(ito::retError, status, libusb_error_name(status));
	    }
        if(!retval.containsError() && m_debugMode)
        {
	        libusb_set_debug(NULL, m_params["debug"].getVal<int>());
        } 
    }


	
/* We think we know what we are doing!!!! */
        
    if (vid > 0 &&  pid > 0) 
    {
        if(!retval.containsError())
        {
		    m_pDevice = libusb_open_device_with_vid_pid(NULL, (uint16_t)vid, (uint16_t)pid);
		    if (m_pDevice == NULL) 
            {
                retval += ito::RetVal(ito::retError, 0, tr("Try to open device directly failed!").toLatin1().data());
            }
            else
            {
                foundDevice = true;
            }
        }
    }
    else //((type == NULL) || (device_id == NULL) || (device_path != NULL)) /* try to pick up missing parameters from known devices */
    {
        if(!retval.containsError())
        {
            status = libusb_get_device_list(NULL, &deviceList);
		    if (status < 0) 
            {
			    retval += ito::RetVal(ito::retError, status, libusb_error_name(status));
		    }
        
        }

        /* try to pick up missing parameters from known devices */
        if(!retval.containsError())
        {

            int listIdx = 0;
            while(deviceList[listIdx] != NULL && !foundDevice)
            {
                currentDevice = deviceList[listIdx];

			    tmpBusnum = libusb_get_bus_number(currentDevice);
			    tmpDevaddr = libusb_get_device_address(currentDevice);

                struct libusb_device_descriptor desc;
				status = libusb_get_device_descriptor(currentDevice, &desc);

			    if ((type != NULL) && (device_path != NULL)) 
                {
				    // if both a type and bus,addr were specified, we just need to find our match
				    if ((tmpBusnum == busnum) && (tmpDevaddr == devaddr))
			        {
                        foundDevice = true;
                        break;
                    }
			    }
                /*
                else 
                {
                    struct libusb_device_descriptor desc;
				    status = libusb_get_device_descriptor(currentDevice, &desc);
				    if (status >= 0) 
                    {
					    if (debugLevel >= 3) 
                        {
                            qDebug( QString("examining %1:%2 (%3,%4)\n").arg(QString::number(desc.idVendor), QString::number(desc.idProduct), QString::number(tmpBusnum), QString::number(tmpDevaddr) ).toLatin1().data() );
					    }
                        int curKnownDeviceIndex;
					    for (curKnownDeviceIndex = 0; curKnownDeviceIndex < ARRAYSIZE(known_device); curKnownDeviceIndex++) 
                        {
						    if ((desc.idVendor == known_device[curKnownDeviceIndex].vid) && (desc.idProduct == known_device[curKnownDeviceIndex].pid)) 
                            {
							    if (// nothing was specified
								    ((type == NULL) && (device_id == NULL) && (device_path == NULL)) ||
								    // vid:pid was specified and we have a match
								    ((type == NULL) && (device_id != NULL) && (vid == desc.idVendor) && (pid == desc.idProduct)) ||
								    // bus,addr was specified and we have a match
								    ((type == NULL) && (device_path != NULL) && (busnum == tmpBusnum) && (devaddr == tmpDevaddr)) ||
								    // type was specified and we have a match
								    ((type != NULL) && (device_id == NULL) && (device_path == NULL) && (fx_type == known_device[curKnownDeviceIndex].type)) ) 
                                {
								    fx_type = known_device[curKnownDeviceIndex].type;
								    vid = desc.idVendor;
								    pid = desc.idProduct;
								    busnum = tmpBusnum;
								    devaddr = tmpDevaddr;
                                    foundDevice = true;
								    break;
							    }
						    }
					    }
					    if (curKnownDeviceIndex < ARRAYSIZE(known_device)) 
                        {
						    if (debugLevel)
                            {
                                qDebug( QString("found device %1:%2 (%3,%4)\n").arg(QString(known_device[curKnownDeviceIndex].designation), QString::number(vid), QString::number(pid), QString::number(busnum), QString::number(devaddr) ).toLatin1().data() );
                                foundDevice = true;
                                break;
                            }
					    }
				    }
			    }*/
                listIdx++;
		    }
        }

        if (currentDevice == NULL  || !foundDevice) 
        {
            retval += ito::RetVal(ito::retError, status, tr("could not find a known device - please specify type and/or vid:pid and/or bus,dev").toLatin1().data());
		}

        if(!retval.containsError())
        {
		    status = libusb_open(currentDevice, &m_pDevice);
		    if (status < 0) 
            {
			    retval += ito::RetVal(ito::retError, status, libusb_error_name(status));
		    }
		    libusb_free_device_list(deviceList, 1);
	    } 
    }

    if(deviceList != NULL)
    {
    	libusb_free_device_list(deviceList, 1);
    }

    if(!retval.containsError()) /* We need to claim the first interface */
    {
        int test = 0;
        
        m_autoDetach  = libusb_set_auto_detach_kernel_driver(m_pDevice, 1) == 0;
        if (!m_autoDetach) 
        {
            libusb_detach_kernel_driver(m_pDevice, 0);
		}
        struct libusb_config_descriptor *conf_desc = NULL;
        status = libusb_get_active_config_descriptor(libusb_get_device(m_pDevice), &conf_desc);
        
        if(conf_desc == NULL)
        {
            status = libusb_get_config_descriptor(libusb_get_device(m_pDevice), 1, &conf_desc);   

            if(conf_desc)
            {
                test = conf_desc->bConfigurationValue;
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
        

        //status = libusb_set_configuration(m_pDevice, test);
		//if (status < 0) 
        //{
		//	retval += ito::RetVal(ito::retError, status, libusb_error_name(status));
		//}

        

        test = 0;
	    status = libusb_claim_interface(m_pDevice, test);
		if (status < 0) 
        {
			retval += ito::RetVal(ito::retError, status, libusb_error_name(status));
		}
        libusb_free_config_descriptor(conf_desc);
//        status = libusb_release_interface(m_pDevice, test);
//		if (status < 0) 
//        {
//			retval += ito::RetVal(ito::retError, status, libusb_error_name(status));
//		}        
	}

    if(!retval.containsError()) /* We need to claim the first interface */
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

    if(m_pDevice)
    {
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
//    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval;
    //retval = m_serport.sread(data.data(), length.data(), 0);
    int status = 0;
    unsigned char *dataIn = NULL;
    int actual_length = 0;

    dataIn = (unsigned char*)calloc(*(length.data()) * 2, sizeof(unsigned char));
    //int status = libusb_claim_interface(m_pDevice, 0);
    status = libusb_bulk_transfer(m_pDevice, LIBUSB_ENDPOINT_IN + m_endpoint, dataIn, *(length.data()), &actual_length, m_timeoutMS);
    if (status != 0) 
    {
        retval += ito::RetVal(ito::retError, status, libusb_error_name(status));
    }

    *(length.data()) = actual_length; 

    memcpy(data.data(), dataIn, actual_length < *(length.data()) ? actual_length : *(length.data()));
    free(dataIn);

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
    unsigned char *buf = (unsigned char*)data;
    ito::RetVal retval(ito::retOk);

    int status = 0;

    int actual_length = 0;

    //m_serport.getendline(endline);
    if (m_debugMode)
    {
        emit serialLog(QByteArray((char*)buf,datalength), '>');
    }
    //status = libusb_claim_interface(m_pDevice, 0);
    status = libusb_bulk_transfer(m_pDevice, LIBUSB_ENDPOINT_OUT + m_endpoint, buf, datalength, &actual_length, m_timeoutMS);
    if (status != 0) 
    {
        retval += ito::RetVal(ito::retError, status, libusb_error_name(status));
    }
    if(actual_length != datalength)
    {
        //retval += ito::RetVal(ito::retError, 0, tr("Number of written characters differ from designated dataLength").toLatin1().data());
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

    if (funcName == "clearInputBuffer")
    {
        //retval = m_serport.sclearbuffer(0);
    }
    else if (funcName == "clearOutputBuffer")
    {
        //retval = m_serport.sclearbuffer(1);
    }
    else if (funcName == "clearBuffer")
    {
        ito::ParamBase *bufferType = NULL;
        bufferType = &((*paramsMand)[0]);
        if (!retval.containsError())
        {
            //retval = m_serport.sclearbuffer(static_cast<bool>(bufferType->getVal<int>()));
        }
    }

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

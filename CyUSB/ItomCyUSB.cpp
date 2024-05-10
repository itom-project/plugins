/* ********************************************************************
    Plugin "ItomCyUSB" for itom software
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

//SKD link: http://www.cypress.com/file/135301?finished=1

#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include "ItomCyUSB.h"
#include "dockWidgetItomCyUSB.h"

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

//#include "CyUSB.h"
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomCyUSBInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(ItomCyUSB)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomCyUSBInterface::closeThisInst(ito::AddInBase **addInInst)
{
   REMOVE_PLUGININSTANCE(ItomCyUSB)
   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ItomCyUSBInterface::ItomCyUSBInterface()
{
    m_type = ito::typeDataIO | ito::typeRawIO;
    setObjectName("CyUSB");

    m_description = tr("itom-plugin for a USB port communication with Cypress Semiconductor USB chips.");

    m_detaildescription = tr(
"ItomCyUSB is a itom-Plugin which gives direct/raw access to a device connected to the serial port.\n\
It can be used by plugins for communication analog to the USB port.\n\
The plugin is implemented for Window.\n\
\n\
To connect to a device you need the vendor ID and the product ID.\n\
\n\
The setVal and getVal functions will write and read on the specified endpoint.\n\
\n\
!Only the setVal function could be tested!");

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
    m_aboutThis = QObject::tr(GITVERSION);

    ito::Param paramVal("VendorID", ito::ParamBase::Int, 0, std::numeric_limits<unsigned short>::max(), 0x04B4, tr("The vendor ID of the device to connect to").toLatin1().data()); //default denselight device
    m_initParamsMand.append(paramVal);
    paramVal = ito::Param("ProductID", ito::ParamBase::Int, 0, std::numeric_limits<unsigned short>::max(), 0xF100, tr("The product ID of the device to connect to").toLatin1().data()); //default denselight device
    m_initParamsMand.append(paramVal);
    paramVal = ito::Param("printInfoAboutAllDevices", ito::ParamBase::Int, 0, 1, 0, tr("If true, all information about connected devices is printed to the console.").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("debug", ito::ParamBase::Int, 0, 1, 0, tr("If true, all communication commands are printed to the dockWidget.").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("timeout", ito::ParamBase::Double, 0.0, 65.0, 4.0, tr("Timeout for reading commands in [s].").toLatin1().data());
    m_initParamsOpt.append(paramVal);

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
ItomCyUSBInterface::~ItomCyUSBInterface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal ItomCyUSB::showConfDialog(void)
{
/*
    dialogItomCyUSB *confDialog = new dialogItomCyUSB((void*)this);
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
ItomCyUSB::ItomCyUSB() : AddInDataIO(),
    cyHandle(NULL),
    m_cyDevices(NULL),
    m_endPoints(NULL),
    m_isocInEndPoint(NULL),
    m_isocOutEndPoint(NULL),
    m_bulkInEndPoint(NULL),
    m_bulkOutEndPoint(NULL),
    m_interruptInEndPoint(NULL),
    m_interruptOutEndPoint(NULL),
    m_controlEndPoint(NULL)
{
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::NoAutosave | ito::ParamBase::Readonly, "ItomCyUSB", "name of device");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("timeout", ito::ParamBase::Double | ito::ParamBase::NoAutosave, 0.0, 65.0, 4.0, tr("Timeout for reading commands in [s].").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("debug", ito::ParamBase::Int, 0, 1, 0, tr("If true, all out and inputs are written to the dockWidget.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("endpoint_read", ito::ParamBase::Int, 0, 255, 1, tr("Endpoint index for reading operations. (default: first detected input endpoint.)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("endpoint_write", ito::ParamBase::Int, 0, 255, 1, tr("Endpoint index for writing operations. (default: first detected output endpoint.)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("number_of_devices", ito::ParamBase::Int, 0, std::numeric_limits<int>::max(), 0, tr("maximum number of detected devices").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    if (hasGuiSupport())
    {
        //now create dock widget for this plugin
        DockWidgetItomCyUSB *dw = new DockWidgetItomCyUSB(m_params, getID());
        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dw);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ItomCyUSB::~ItomCyUSB()
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
ito::RetVal ItomCyUSB::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
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
ito::RetVal ItomCyUSB::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
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
                int countEndPoint = m_cyDevices->EndPointCount() - 1;
                for (int cnt = 0; cnt < countEndPoint; cnt++)
                {
                    m_endPoints[cnt + 1]->TimeOut = m_params["timeout"].getVal<int>() * 1000;
                }
            }

        }
        else if (key == "debug")
        {
            //check the new value and if ok, assign it to the internal parameter
            retValue += it->copyValueFrom( &(*val) );

        }
        else if (key == "endpoint_read")
        {
            //check the new value and if ok, assign it to the internal parameter
            retValue += it->copyValueFrom( &(*val) );

        }
        else if (key == "endpoint_write")
        {
            //check the new value and if ok, assign it to the internal parameter
            retValue += it->copyValueFrom( &(*val) );

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
ito::RetVal ItomCyUSB::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);

    ito::RetVal retval(ito::retOk);

    bool printInfo = paramsOpt->at(0).getVal<int>() > 0; //if printInfoAboutAllDevices is checked, all connected Devices/Parameter are printed in itom command box
    retval += m_params["debug"].copyValueFrom(&((*paramsOpt)[1]));
    retval += m_params["timeout"].copyValueFrom(&((*paramsOpt)[2]));


    uint16_t vendorID = paramsMand->at(0).getVal<int>() & 0x0000FFFF;
    uint16_t productID = paramsMand->at(1).getVal<int>() & 0x0000FFFF;
    m_cyDevices = new CCyUSBDevice(cyHandle, CYUSBDRV_GUID, true);
    //m_cyDevices = new CCyUSBDevice(NULL);

    //m_cyDevices->BulkInEndPt->TimeOut = m_params["timeout"].getVal<int>();
    //m_cyDevices->BulkOutEndPt->TimeOut = m_params["timeout"].getVal<int>();

    int numDevices = m_cyDevices->DeviceCount();
    m_params["number_of_devices"].setVal<int>(numDevices);

    for (int cnt = 0; cnt < numDevices; cnt++)
    {

        //if ((USBDevice->VendorID == VENDOR_ID) && (USBDevice->ProductID == PRODUCT_ID))
        //    break;
        if (printInfo)
        {
            m_cyDevices->Open(cnt);

            //usb device descriptor
            for (int c = 0; c<m_cyDevices->ConfigCount(); c++)
            {
                CCyUSBConfig cfg = m_cyDevices->GetUSBConfig(c);

                std::cout << "-----------------------------\n"
                    << "USB DESCRIPTOR:\n"
                    << "USBDevice Name: " << m_cyDevices->FriendlyName << " \n"
                    << "USBDevice Vendor ID: " << m_cyDevices->VendorID << " \n"
                    << "USBDevice Product ID: " << m_cyDevices->ProductID << " \n"
                    << "USBDevice Serial number: " << m_cyDevices->SerialNumber << " \n"
                    << "USBDevice Driver version: " << m_cyDevices->DriverVersion << " \n"
                    << "USBDevice USB address: " << (int)m_cyDevices->USBAddress << " \n"
                    << "\n-----------------------------\n" << std::endl;

                //interface descriptor
                for (int i = 0; i<cfg.AltInterfaces; ++i)
                {
                    m_cyDevices->SetAltIntfc(i);
                    CCyUSBInterface *ifc = cfg.Interfaces[i];

                    std::cout << "-----------------------------\n"
                        << "INTERFACE DESCRIPTOR:\n"
                        << "Interface bLength: " << (int)ifc->bLength << " \n"
                        << "Interface bDescriptorType: " << (int)ifc->bDescriptorType << " \n"
                        << "Interface bInterfaceNumber: " << (int)ifc->bInterfaceNumber << " \n"
                        << "Interface bAlternateSetting: " << (int)ifc->bAlternateSetting << " \n"
                        << "Interface bNumEndpoints: " << (int)ifc->bNumEndpoints << " \n"
                        << "Interface bInterfaceClass: " << (int)ifc->bInterfaceClass << " \n"
                        << "Interface wTotalLength: " << ifc->wTotalLength << " \n"
                        << "\n-----------------------------\n" << std::endl;

                    //endpoint descriptor
                    std::cout << "number of endpoints: " << (int)ifc->bNumEndpoints << " \n" << std::endl;

                    for (int e = 0; e < ifc->bNumEndpoints; ++e)
                    {

                        CCyUSBEndPoint *ept = ifc->EndPoints[e + 1];

                        std::cout << "-----------------------------\n"
                            << "ENDPOINT DESCRIPTOR:\n"
                            << "Endpoint bLength: " << (int)ept->DscLen << " \n"
                            << "Endpoint bDescriptorType: " << (int)ept->DscType << " \n"
                            << "Endpoint bEndpointAddress: " << (int)ept->Address << " \n"
                            << "Endpoint bmAttributes: " << (int)ept->Attributes << " \n"
                            << "Endpoint wMaxPacketSize: " << ept->MaxPktSize << " \n"
                            << "Endpoint bInterval: " << (int)ept->Interval << " \n"
                            << "Endpoint Direction: " << (ept->bIn ? "In " : "Out ") << " \n"
                            << "\n-----------------------------\n" << std::endl;

                    }
                }
            }

            m_cyDevices->Close();
        }

    }

    if (!retval.containsError())
    {

        for (int cnt = 0; cnt < numDevices; cnt++)
        {
            m_cyDevices->Open(cnt);
            int vendorIDDevice = m_cyDevices->VendorID;
            int productIDDevice = m_cyDevices->ProductID;

            if ((vendorID = vendorIDDevice) && (productID = productIDDevice))
            {
                m_endPoints = m_cyDevices->EndPoints;
                m_endPoints[0] = m_cyDevices->ControlEndPt;

                int countEndPoint = m_cyDevices->EndPointCount() - 1;

                //CCyUSBConfig config = m_cyDevices->GetUSBConfig(cnt);
                //m_controlEndPoint = m_cyDevices->ControlEndPt; //endpoint at 0 is Control Endpoint
                UCHAR bIn;
                UCHAR attrib;

                int sizeMin_write = 0;
                int sizeMax_write = 0;
                int sizeMin_read = 0;
                int sizeMax_read = 0;

                bool endpoint_read_detected = false;
                bool endpoint_write_detected = false;


                for (int cnt = 0; cnt < countEndPoint; cnt++)
                {
                    bIn = m_endPoints[cnt + 1]->Address & 0x80;
                    attrib = m_endPoints[cnt + 1]->Attributes;

                    m_endPoints[cnt + 1]->XferMode = XMODE_DIRECT;
                    m_endPoints[cnt + 1]->TimeOut = m_params["timeout"].getVal<int>() * 1000;
                    //get read endpoints
                    if (bIn)
                    {
                        if (!endpoint_read_detected)
                        {
                            sizeMin_read = cnt + 1;
                            m_params["endpoint_read"].setVal<int>(cnt + 1);
                            endpoint_read_detected = true;
                            /*
                            //get read endpoints
                            if ((m_isocInEndPoint == NULL) && (attrib == 1) && bIn)
                            {
                                m_isocInEndPoint = (CCyIsocEndPoint *)m_endPoints[cnt + 1];
                                std::cout << "-----------------------------\n"
                                    << "Input endPoint : CCyIsocEndPoint\n" << std::endl;
                            }
                            if ((m_bulkInEndPoint == NULL) && (attrib == 2) && bIn)
                            {
                                m_bulkInEndPoint = (CCyBulkEndPoint *)m_endPoints[cnt + 1];
                                std::cout << "-----------------------------\n"
                                    << "Input endPoint is: CCyBulkEndPoint\n" << std::endl;
                            }
                            if ((m_interruptInEndPoint == NULL) && (attrib == 3) && bIn)
                            {
                                m_interruptInEndPoint = (CCyInterruptEndPoint *)m_endPoints[cnt + 1];
                                std::cout << "-----------------------------\n"
                                    << "Input endPoint is: CCyInterruptEndPoint\n" << std::endl;
                            }
                            */
                        }

                        sizeMax_read = cnt + 1;
                    }

                    //get write endpoints
                    if (!bIn)
                    {
                        if (!endpoint_write_detected)
                        {
                            sizeMin_write = cnt + 1;
                            m_params["endpoint_write"].setVal<int>(cnt + 1);
                            endpoint_write_detected = true;

                            /*
                            //get write entpoints
                            if ((m_isocOutEndPoint == NULL) && (attrib == 1) && !bIn)
                            {
                                m_isocOutEndPoint = (CCyIsocEndPoint *)m_endPoints[cnt + 1];
                                std::cout << "-----------------------------\n"
                                    << "Output endPoint is: CCyIsocEndPoint\n" << std::endl;
                            }
                            if ((m_bulkOutEndPoint == NULL) && (attrib == 2) && !bIn)
                            {
                                m_bulkOutEndPoint = (CCyBulkEndPoint *)m_endPoints[cnt + 1];
                                std::cout << "-----------------------------\n"
                                    << "Output endPoint is: CCyBulkEndPoint\n" << std::endl;
                            }
                            if ((m_interruptOutEndPoint == NULL) && (attrib == 3) && !bIn)
                            {
                                m_interruptOutEndPoint = (CCyInterruptEndPoint *)m_endPoints[cnt + 1];
                                std::cout << "-----------------------------\n"
                                    << "Output endPoint is: CCyInterruptEndPoint\n" << std::endl;
                            }
                            */
                        }

                        sizeMax_write = cnt + 1;

                    }

                }

                m_params["endpoint_write"].setMeta(new ito::IntMeta(sizeMin_write, sizeMax_write, 1), true);
                m_params["endpoint_read"].setMeta(new ito::IntMeta(sizeMin_read, sizeMax_read, 1), true);

            }
            else
            {
                retval += ito::RetVal(ito::retError, 0, tr("given vendor- or product-ID not detected").toLatin1().data());
            }
        }


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
ito::RetVal ItomCyUSB::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval;

    if (m_cyDevices)
    {
        m_cyDevices->Close();
        m_cyDevices = NULL;
    }


    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomCyUSB::startDevice(ItomSharedSemaphore *waitCond)
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
ito::RetVal ItomCyUSB::stopDevice(ItomSharedSemaphore *waitCond)
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
ito::RetVal ItomCyUSB::acquire(const int /*trigger*/, ItomSharedSemaphore *waitCond)
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
ito::RetVal ItomCyUSB::getVal(QSharedPointer<char> data, QSharedPointer<int> length, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;

    if (m_cyDevices->IsOpen() && m_endPoints != NULL)
    {
        CCyUSBEndPoint *endPoint = m_endPoints[m_params["endpoint_read"].getVal<int>()];
        UCHAR bIn = endPoint->Address & 0x80;
        UCHAR attrib = endPoint->Attributes;

        if (bIn && attrib == 1) //Incoming, isocronous endpoint
        {
            long len = *length;
            PUCHAR buffer = (PUCHAR)data.data();
            int pkts;
            // Allocate the IsoPktInfo objects, and find-out how many were allocated
            CCyIsoPktInfo *isoPktInfos = ((CCyIsocEndPoint *)endPoint)->CreatePktInfos(len, pkts);

            bool status = endPoint->XferData(buffer, len, isoPktInfos);

            if (!status)
            {
                *length = 0;
                //ULONG errCode = endPoint->NtStatus;
                retval += ito::RetVal(ito::retError, 0, tr("error while obtaining data. (isoc input endpoint).").toLatin1().data());
            }
            else
            {
                *length = 0;
                for (int i = 0; i < pkts; ++i)
                {
                    if (isoPktInfos[i].Status == 0)
                    {
                        *length += isoPktInfos[i].Length;
                    }
                }
            }

            delete[] isoPktInfos;
        }
        else if (bIn) //bulk or interrupt endpoint
        {
            long len = *length;
            PUCHAR buffer = (PUCHAR)data.data();
            bool status = endPoint->XferData(buffer, len, NULL);

            if (!status)
            {
                *length = 0;
                ULONG errCode = endPoint->NtStatus;
                retval += ito::RetVal(ito::retError, 0, tr("error while obtaining data. (bulk or interrupt input endpoint).").toLatin1().data());
            }
            else
            {
                *length = len;
            }
        }
        else
        {
            retval += ito::RetVal(ito::retError, 0, "endpoint does not handle incoming transfer.");
        }

        /*bool status = false;
        OVERLAPPED inOvLap;
        inOvLap.hEvent = CreateEvent(NULL, false, false, L"CYUSB_IN");

        unsigned char inBuf[128];

        ZeroMemory(inBuf, 128);

        LONG length = sizeof(inBuf);
        int endpointNum = m_params["endpoint_read"].getVal<int>();
        UCHAR *inContext = m_endPoints[endpointNum]->BeginDataXfer(inBuf, length, &inOvLap);

        if (!m_endPoints[endpointNum]->WaitForXfer(&inOvLap, 100) && !retval.containsError())
        {
            retval += ito::RetVal(ito::retError, 0, tr("setVal error during WaitForXFer").toLatin1().data());
        }

        if (!m_endPoints[endpointNum]->FinishDataXfer(inBuf, length, &inOvLap, inContext) && !retval.containsError())
        {
            retval += ito::RetVal(ito::retError, 0, tr("setVal error during FinishDataXfer").toLatin1().data());
        }

        if (!retval.containsError())
        {
            *data = inBuf[0];
        }

        if (!CloseHandle(inOvLap.hEvent) && !retval.containsError())
        {
            retval += ito::RetVal(ito::retError, 0, tr("setVal error during CloseHandle").toLatin1().data());
        }

        if (retval.containsError())
        {
            CloseHandle(inOvLap.hEvent);
        }*/

    }
    else
    {
        retval += ito::RetVal(ito::retError, 0, tr("devices is not open or endpoint pointer is NULL.").toLatin1().data());
    }

    if (m_params["debug"].getVal<int>() == 1)
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
ito::RetVal ItomCyUSB::setVal(const char *data, const int datalength, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval(ito::retOk);

    if (m_cyDevices->IsOpen() && m_endPoints != NULL)
    {
        CCyUSBEndPoint *endPoint = m_endPoints[m_params["endpoint_write"].getVal<int>()];
        UCHAR bIn = endPoint->Address & 0x80;
        UCHAR attrib = endPoint->Attributes;

        if (!bIn && attrib == 1) //Incoming, isocronous endpoint
        {
            long len = datalength;
            PUCHAR buffer = (PUCHAR)data[0];
            int pkts;
            // Allocate the IsoPktInfo objects, and find-out how many were allocated
            CCyIsoPktInfo *isoPktInfos = ((CCyIsocEndPoint *)endPoint)->CreatePktInfos(len, pkts);

            bool status = endPoint->XferData(buffer, len, isoPktInfos);

            if (!status)
            {
                ULONG errCode = endPoint->NtStatus;
                retval += ito::RetVal(ito::retError, 0, tr("error while obtaining data. (isoc output endpoint).").toLatin1().data());
            }


            delete[] isoPktInfos;
        }
        else if (!bIn) //bulk or interrupt endpoint
        {
            long len = datalength;
            PUCHAR buffer = (PUCHAR)data;
            bool status = endPoint->XferData(buffer, len, NULL);

            if (!status)
            {
                //ULONG errCode = endPoint->NtStatus;
                retval += ito::RetVal(ito::retError, 0, tr("error while obtaining data (bulk or interrupt output endpoint).").toLatin1().data());
            }

        }
        else
        {
            retval += ito::RetVal(ito::retError, 0, "endpoint does not handle incoming transfer.");
        }


    }
    else
    {
        retval += ito::RetVal(ito::retError, 0, tr("devices is not open or endpoint pointer is NULL.").toLatin1().data());
    }

    if (m_params["debug"].getVal<int>() == 1)
    {
        emit serialLog(QByteArray(data, datalength), '>');
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomCyUSB::execFunc(const QString funcName, QSharedPointer<QVector<ito::ParamBase> > paramsMand, QSharedPointer<QVector<ito::ParamBase> > paramsOpt, QSharedPointer<QVector<ito::ParamBase> > paramsOut, ItomSharedSemaphore *waitCond /*= NULL*/)
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
void ItomCyUSB::dockWidgetVisibilityChanged(bool visible)
{
    if (getDockWidget())
    {

        DockWidgetItomCyUSB *dw = qobject_cast<DockWidgetItomCyUSB*>(getDockWidget()->widget());
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

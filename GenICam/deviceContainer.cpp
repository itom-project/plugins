/* ********************************************************************
    Plugin "GenICam" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2024, Institut für Technische Optik (ITO),
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

#include "deviceContainer.h"

#include "gccommon.h"

#include <qfileinfo.h>
#include <qdebug.h>
#include <qset.h>
#include "common/sharedStructures.h"
#include <iostream>

/*static*/ GenTLOrganizer *GenTLOrganizer::m_pOrganizer = nullptr;
/*static*/ const QByteArray GenTLInterface::SerialNumberPrefix = "Serial:";
//----------------------------------------------------------------------------------------------------------------------------------

/*static*/ GenTLOrganizer * GenTLOrganizer::instance(void)
{
    static GenTLOrganizerSingleton w;
    if (GenTLOrganizer::m_pOrganizer == nullptr)
    {
        #pragma omp critical
        {
            if (GenTLOrganizer::m_pOrganizer == nullptr)
            {
                GenTLOrganizer::m_pOrganizer = new GenTLOrganizer();
            }
        }
    }
    return GenTLOrganizer::m_pOrganizer;
}

//----------------------------------------------------------------------------------------------------------------------------------
GenTLOrganizer::GenTLOrganizer(void)
{
}

//----------------------------------------------------------------------------------------------------------------------------------
GenTLOrganizer::~GenTLOrganizer(void)
{
}

//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer<GenTLSystem> GenTLOrganizer::getSystem(const QString &filename, ito::RetVal &retval)
{
    //check if this filename has already be opened
    foreach( const QWeakPointer<GenTLSystem> &system, m_systems)
    {
        if (system.toStrongRef().data() && system.toStrongRef().data()->getCtiFile() == filename)
        {
            return system.toStrongRef();
        }
    }

    //nothing found
    QSharedPointer<GenTLSystem> system(new GenTLSystem());
    retval += system->init(filename);

    if (retval.containsError())
    {
        return QSharedPointer<GenTLSystem>();
    }

    m_systems.append( system.toWeakRef() );
    return system;
}

//----------------------------------------------------------------------------------------------------------------------------------
GenTLSystem::GenTLSystem() :
    GCInitLib(nullptr),
    GCCloseLib(nullptr),
    GCGetInfo(nullptr),
    GCGetLastError(nullptr),
    TLOpen(nullptr),
    TLClose(nullptr),
    TLUpdateInterfaceList(nullptr),
    m_initialized(false),
    m_systemInit(false),
    m_systemOpened(false),
    m_handle(GENTL_INVALID_HANDLE),
    m_verbose(0)
{
    m_lib = QSharedPointer<QLibrary>(new QLibrary());
}

//----------------------------------------------------------------------------------------------------------------------------------
GenTLSystem::~GenTLSystem()
{
    if (m_lib->isLoaded() && m_initialized)
    {
        if (TLClose && m_systemOpened)
        {
            TLClose(m_handle);
            m_handle = nullptr;
        }

        if (GCCloseLib && m_systemInit)
        {
            GCCloseLib();
        }
    }
}

//------------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GenTLSystem::init(const QString &filename)
{
    ito::RetVal retval;
    QFileInfo info(filename);

    if (!info.exists())
    {
        retval += ito::RetVal::format(ito::retError, 0, "file '%s' does not exist", filename.toLatin1().constData());
    }
    else if (filename.endsWith(".cti") == false)
    {
        retval += ito::RetVal::format(ito::retError, 0, "file '%s' is no *.cti file", filename.toLatin1().constData());
    }
    else
    {
#ifdef WIN32
    #ifdef _WIN64
        if (filename.contains("sv_gev_tl_x64.cti", Qt::CaseInsensitive))
        {
            //hack: Vistek cameras: it seems that the sv_gev_cl_x64.cti has to be loaded before the GenICam transport layer is opened... (2018-01-26)
            QString filename_cl = filename;
            filename_cl.replace("sv_gev_tl_x64.cti", "sv_cl_tl_x64.cti", Qt::CaseInsensitive);
            filename_cl.replace("TLGigE", "TLCL", Qt::CaseSensitive);
            if (QFileInfo(filename_cl).exists())
            {
                QLibrary lib;
                lib.setFileName(filename_cl);
                lib.load();
            }
            else
            {
                retval += ito::RetVal::format(ito::retWarning, 0, "It seems, that a SVS Vistek, GigE camera should be loaded. Due to unknown reasons, the library sv_cl_tl_x64.cti has to be loaded first. However, the path '%s' could not be found.", filename_cl.toLatin1().constData());
            }
        }
    #else
        if (filename.contains("sv_cl_tl.cti", Qt::CaseInsensitive))
        {
            //hack: Vistek cameras: it seems that the sv_gev_cl_x64.cti has to be loaded before the GenICam transport layer is opened... (2018-01-26)
            QString filename_cl = filename;
            filename_cl.replace("sv_gev_tl.cti", "sv_cl_tl.cti", Qt::CaseInsensitive);
            filename_cl.replace("TLGigE", "TLCL", Qt::CaseSensitive);
            if (QFileInfo(filename_cl).exists())
            {
                QLibrary lib;
                lib.setFileName(filename_cl);
                lib.load();
            }
            else
            {
                retval += ito::RetVal::format(ito::retWarning, 0, "It seems, that a SVS Vistek, GigE camera should be loaded. Due to unknown reasons, the library sv_cl_tl.cti has to be loaded first. However, the path '%s' could not be found.", filename_cl.toLatin1().constData());
            }
        }
    #endif
#endif



        m_lib->setFileName(filename);
        if (!m_lib->load())
        {
            retval += ito::RetVal::format(ito::retError, 0, "error loading library '%s' (%s)", filename.toLatin1().constData(), m_lib->errorString().toLatin1().constData());
        }
        else
        {
            m_initialized = true;

            GCInitLib = (GenTL::PGCInitLib)m_lib->resolve("GCInitLib");
            GCCloseLib = (GenTL::PGCCloseLib)m_lib->resolve("GCCloseLib");
            GCGetInfo = (GenTL::PGCGetInfo)m_lib->resolve("GCGetInfo"); //only necessary for getStringInfo. Check for existence there.
            TLOpen = (GenTL::PTLOpen)m_lib->resolve("TLOpen");
            TLClose = (GenTL::PTLClose)m_lib->resolve("TLClose");
            TLUpdateInterfaceList = (GenTL::PTLUpdateInterfaceList)m_lib->resolve("TLUpdateInterfaceList");
            TLGetNumInterfaces = (GenTL::PTLGetNumInterfaces)m_lib->resolve("TLGetNumInterfaces");
            TLGetInterfaceID = (GenTL::PTLGetInterfaceID)m_lib->resolve("TLGetInterfaceID");
            TLGetInterfaceInfo = (GenTL::PTLGetInterfaceInfo)m_lib->resolve("TLGetInterfaceInfo");
            TLOpenInterface = (GenTL::PTLOpenInterface)m_lib->resolve("TLOpenInterface");
            GCGetLastError = (GenTL::PGCGetLastError)m_lib->resolve("GCGetLastError");

            if (GCInitLib == NULL || GCCloseLib == NULL || \
                TLClose == NULL || TLOpen == NULL || !TLUpdateInterfaceList || \
                !TLGetNumInterfaces || !TLGetInterfaceID || !TLGetInterfaceInfo || !TLOpenInterface)
            {
                retval += ito::RetVal(ito::retError, 0, QObject::tr("cti file '%1' does not export all necessary methods of the GenTL standard.").arg(filename).toLatin1().constData());
            }


        }
    }

    if (!retval.containsError())
    {
        retval += checkGCError(GCInitLib());

        if (retval.containsError())
        {
            GCCloseLib();
        }
        else
        {
            m_systemInit = true;
            m_ctiFile = filename;
        }
    }
    else
    {
        m_lib->unload();
        m_initialized = false;
    }


    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GenTLSystem::openSystem()
{
    ito::RetVal retval;

    if (!m_systemOpened)
    {

        if (!m_initialized || !m_systemInit || !TLOpen)
        {
            retval += ito::RetVal(ito::retError, 0, "System not initialized");
        }
        else
        {
            retval += checkGCError(TLOpen(&m_handle));
        }

        if (!retval.containsError())
        {
            //create and update interface list (done once for each system)
            //bool8_t pbChanged;
            retval += checkGCError(TLUpdateInterfaceList(m_handle, NULL, GENTL_INFINITE), "Update interface list");
            m_systemOpened = true;
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
QByteArray GenTLSystem::getStringInfo(GenTL::TL_INFO_CMD_LIST cmd, ito::RetVal &retval) const
{

    if (!m_initialized || !GCGetInfo)
    {
        retval += ito::RetVal(ito::retError, 0, "System not initialized or method GCGetInfo in transport layer not available.");
    }
    else
    {
        GenTL::INFO_DATATYPE piType;
        char pBuffer[512];
        size_t piSize = 512;
        retval += checkGCError(GCGetInfo(cmd, &piType, &pBuffer, &piSize));

        if (!retval.containsError())
        {
            if (piType == GenTL::INFO_DATATYPE_STRING)
            {
                return QByteArray(pBuffer);
            }
            else
            {
                retval += ito::RetVal(ito::retError,0,"info type is no string");
            }
        }
    }

    return QByteArray();
}

//----------------------------------------------------------------------------------------------------------------------------------
QByteArray GenTLSystem::getInterfaceInfo(GenTL::INTERFACE_INFO_CMD_LIST cmd, const char *sIfaceID, ito::RetVal &retval) const
{
    if (!m_initialized || !m_systemInit || !TLGetInterfaceInfo)
    {
        retval += ito::RetVal(ito::retError, 0, "System not initialized");
    }
    else
    {
        GenTL::INFO_DATATYPE piType;
        char pBuffer[512];
        size_t piSize = 512;
        retval += checkGCError(TLGetInterfaceInfo(m_handle, sIfaceID, cmd, &piType, &pBuffer, &piSize));

        if (!retval.containsError())
        {
            if (piType == GenTL::INFO_DATATYPE_STRING)
            {
                return QByteArray(pBuffer);
            }
            else
            {
                retval += ito::RetVal(ito::retError,0,"info type is no string");
            }
        }
    }

    return QByteArray();
}

//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer<GenTLInterface> GenTLSystem::getInterface(const QByteArray &interfaceID, ito::RetVal &retval)
{
    if (!m_initialized || !m_systemInit)
    {
        retval += ito::RetVal(ito::retError, 0, "System not initialized");
    }
    else
    {
        //check existing, opened interfaces
        for (int i = 0; i < m_interfaces.size(); ++i)
        {
            if (m_interfaces[i].isNull() == false && m_interfaces[i].toStrongRef().data()->getIfaceID() == interfaceID)
            {
                return m_interfaces[i].toStrongRef();
            }
        }

        uint32_t piNumIfaces = 0;
        retval += checkGCError(TLGetNumInterfaces(m_handle, &piNumIfaces));
        char sIfaceID[512];
        size_t piSize = 512;
        bool found = false;
        QByteArray id, displayname, tltype;
        ito::RetVal tlretval;
        QByteArray interfaceIDToOpen = interfaceID;

        if (!retval.containsError())
        {
            if (interfaceID == "" || m_verbose >= VERBOSE_INFO)
            {
                std::cout << "Available interfaces\n----------------------------------------\n" << std::endl;
            }

            for (uint32_t i = 0; i < piNumIfaces; ++i)
            {
                piSize = 512;
                retval += checkGCError(TLGetInterfaceID(m_handle, i, sIfaceID, &piSize));

                if (interfaceIDToOpen == "auto")
                {
                    GenTL::IF_HANDLE ifHandle;
                    ito::RetVal retvalIFace;
                    if (checkGCError(TLOpenInterface(m_handle, sIfaceID, &ifHandle), QString("Error opening interface '%1'").arg(QLatin1String(sIfaceID))) == ito::retOk)
                    {
                        GenTLInterface *gtli = new GenTLInterface(m_lib, ifHandle, sIfaceID, m_verbose, retvalIFace);

                        if (gtli && retvalIFace == ito::retOk)
                        {
                            if (gtli->getNumDevices() > 0)
                            {
                                interfaceIDToOpen = sIfaceID;
                            }
                        }

                        DELETE_AND_SET_NULL(gtli);
                    }
                }

                tlretval = ito::retOk;
                tltype = getInterfaceInfo(GenTL::INTERFACE_INFO_TLTYPE, sIfaceID, tlretval);

                if (interfaceID == "" || m_verbose >= VERBOSE_INFO)
                {
                    id = getInterfaceInfo(GenTL::INTERFACE_INFO_ID, sIfaceID, retval);
                    displayname = getInterfaceInfo(GenTL::INTERFACE_INFO_DISPLAYNAME, sIfaceID, retval);
                    std::cout << (i + 1) << ". ID: " << sIfaceID << ", name: " << displayname.constData() << ", transport layer type: " << tltype.constData() << "\n" << std::endl;
                }
            }

            if (interfaceID == "")
            {
                std::cout << "----------------------------------------\nUse the 'ID' value as interface parameter\n" << std::endl;
            }
        }

        if (interfaceID == "")
        {
            retval += ito::RetVal(ito::retError, 0, "No valid interface chosen");
        }
        else if (interfaceIDToOpen == "auto")
        {
            retval += ito::RetVal(ito::retError, 0, "No devices could be automatically detected. Set the interface parameter to an empty string in order to get a list of all detected devices");
        }

        if (!retval.containsError())
        {
            if (m_verbose >= VERBOSE_INFO)
            {
                std::cout << "Trying to open interface '" << interfaceIDToOpen.constData() << "'...";
            }

            GenTL::IF_HANDLE ifHandle = GENTL_INVALID_HANDLE;
            GenTL::GC_ERROR err = TLOpenInterface(m_handle, interfaceIDToOpen.constData(), &ifHandle);

            if (err != GenTL::GC_ERR_SUCCESS && GCGetLastError)
            {
                size_t errorTextSize = 1024;
                char errorText[1024];

                GCGetLastError(&err, errorText, &errorTextSize);

                retval += checkGCError(err, QString("Error opening interface '%1' (Detailed error text: %2)").arg(QLatin1String(interfaceIDToOpen)).arg(QLatin1String(QByteArray(errorText, errorTextSize))));
            }
            else
            {
                retval += checkGCError(err, QString("Error opening interface '%1'").arg(QLatin1String(interfaceIDToOpen)));
            }

            if (!retval.containsError())
            {


                GenTLInterface *gtli = new GenTLInterface(m_lib, ifHandle, interfaceIDToOpen, m_verbose, retval);
                if (!retval.containsError())
                {
                    if (m_verbose >= VERBOSE_INFO)
                    {
                        std::cout << "OK\n" << std::endl;
                    }

                    QSharedPointer<GenTLInterface> sharedPtr(gtli);
                    m_interfaces.append(sharedPtr.toWeakRef());
                    return sharedPtr;
                }
                else
                {
                    if (m_verbose >= VERBOSE_INFO)
                    {
                        std::cout << "Error: " << retval.errorMessage() << "\n" << std::endl;
                    }

                    delete gtli;
                    gtli = NULL;
                }
            }
            else if (m_verbose >= VERBOSE_INFO)
            {
                std::cout << "Error: " << retval.errorMessage() << "\n" << std::endl;
            }
        }
        else
        {
            //try to open the interface without
            retval += ito::RetVal(ito::retError,0,"no interface found");
        }
    }

    return QSharedPointer<GenTLInterface>();
}







//----------------------------------------------------------------------------------------------------------------------------------
GenTLInterface::GenTLInterface(QSharedPointer<QLibrary> lib, GenTL::IF_HANDLE ifHandle, QByteArray ifID, int verbose, ito::RetVal &retval) :
    m_lib(lib),
    m_handle(ifHandle),
    m_IfaceID(ifID),
    IFOpenDevice(NULL),
    IFClose(NULL),
    IFGetDeviceInfo(NULL),
    IFGetDeviceID(NULL),
    IFUpdateDeviceList(NULL),
    IFGetNumDevices(NULL),
    m_verbose(verbose)
{
    IFOpenDevice = (GenTL::PIFOpenDevice)m_lib->resolve("IFOpenDevice");
    IFClose = (GenTL::PIFClose)m_lib->resolve("IFClose");
    IFGetDeviceInfo = (GenTL::PIFGetDeviceInfo)m_lib->resolve("IFGetDeviceInfo");
    IFGetDeviceID = (GenTL::PIFGetDeviceID)m_lib->resolve("IFGetDeviceID");
    IFUpdateDeviceList = (GenTL::PIFUpdateDeviceList)m_lib->resolve("IFUpdateDeviceList");
    IFGetNumDevices = (GenTL::PIFGetNumDevices)m_lib->resolve("IFGetNumDevices");

    if (IFGetDeviceID == NULL || IFGetDeviceInfo == NULL || IFClose == NULL || \
        IFOpenDevice == NULL || !IFUpdateDeviceList || !IFGetNumDevices)
    {
        retval += ito::RetVal(ito::retError, 0, QObject::tr("cti file does not export all functions of the GenTL protocol.").toLatin1().constData());
    }
    else
    {
        retval += checkGCError(IFUpdateDeviceList(m_handle, NULL, 5000 /*timeout in ms*/), "Interface: Update device list");
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
GenTLInterface::~GenTLInterface()
{
    if (m_lib->isLoaded() && IFClose)
    {
        IFClose(m_handle);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
int GenTLInterface::getNumDevices() const
{
    uint32_t piNumDevices = 0;
    if (IFGetNumDevices(m_handle, &piNumDevices) == GenTL::GC_ERR_SUCCESS)
    {
        return piNumDevices;
    }
    else
    {
        return -1;
    }
}

//-------------------------------------------------------------------------------------
/*
\param deviceID: ID of device to be opened, if empty, the first available device is used.
*/
QSharedPointer<GenTLDevice> GenTLInterface::getDevice(
    const QByteArray &deviceID,
    GenTL::DEVICE_ACCESS_FLAGS deviceAccess,
    ito::RetVal &retval)
{
    bool fetchSerialNumberNotDeviceID = deviceID.startsWith(SerialNumberPrefix);
    QByteArray requestedSerialNumber = "";

    if (fetchSerialNumberNotDeviceID)
    {
        requestedSerialNumber = deviceID.mid(SerialNumberPrefix.size());
    }

    if (!IFGetDeviceInfo)
    {
        retval += ito::RetVal(ito::retError, 0, "System not initialized");
    }
    else
    {
        QSet<QByteArray> usedDeviceIDs;

        //check if already exists, this time, this is an error
        for (int i = 0; i < m_devices.size(); ++i)
        {
            if (m_devices[i].isNull() == false)
            {
                const auto dev = m_devices[i].toStrongRef();

                usedDeviceIDs.insert(dev->getDeviceID());

                if (fetchSerialNumberNotDeviceID)
                {
                    if (requestedSerialNumber != "" && dev->getSerialNumber() == requestedSerialNumber)
                    {
                        retval += ito::RetVal::format(
                            ito::retError,
                            0,
                            "device with serial number '%s' already in use",
                            requestedSerialNumber.constData()
                        );
                    }
                }
                else
                {
                    if (dev->getDeviceID() == deviceID)
                    {
                        retval += ito::RetVal::format(ito::retError, 0, "device '%s' already in use", deviceID.constData());
                    }
                }
            }
        }

        uint32_t piNumDevices = 0;
        retval += checkGCError(IFGetNumDevices(m_handle, &piNumDevices));
        char sDeviceID[512];
        size_t piSize = 512;
        bool found = false;
        ito::RetVal localRetVal;

        if (!retval.containsError())
        {
            if (m_verbose >= VERBOSE_INFO)
            {
                std::cout << "Detected devices: " << piNumDevices << "\n" << std::endl;
            }

            if (piNumDevices == 0)
            {
                retval += ito::RetVal(ito::retError, 0, "no devices detected for given vendor and interface type");
            }

            for (uint32_t i = 0; i < piNumDevices; ++i)
            {
                piSize = 512;
                localRetVal = checkGCError(IFGetDeviceID(m_handle, i, sDeviceID, &piSize));

                //check access status
                GenTL::DEVICE_ACCESS_STATUS accessStatus = GenTL::DEVICE_ACCESS_STATUS_UNKNOWN;
                QByteArray deviceSerialNumber(200, ' ');

                if (!localRetVal.containsError())
                {
                    GenTL::INFO_DATATYPE piType;
                    char pBuffer[512];
                    size_t piSize2 = sizeof(pBuffer);
                    GenTL::GC_ERROR err = IFGetDeviceInfo(m_handle, sDeviceID, GenTL::DEVICE_INFO_ACCESS_STATUS, &piType, &pBuffer, &piSize2);

                    if (err == GenTL::GC_ERR_SUCCESS && piType == GenTL::INFO_DATATYPE_INT32)
                    {
                        accessStatus = ((ito::int32*)pBuffer)[0];

                        if (accessStatus == GenTL::DEVICE_ACCESS_STATUS_UNKNOWN)
                        {
                            accessStatus = GenTL::DEVICE_ACCESS_STATUS_READWRITE;
                            localRetVal += ito::RetVal::format(
                                ito::retWarning,
                                0,
                                "Device '%s' reports an unknown access status. Therefore it is assumed that this camera can be accessed!",
                                sDeviceID
                            );
                        }
                    }
                    else
                    {
                        accessStatus = GenTL::DEVICE_ACCESS_STATUS_READWRITE;
                        localRetVal += ito::RetVal::format(
                            ito::retWarning,
                            0,
                            "Device '%s' does not provide information about its access status. Therefore it is assumed "
                            "that this camera can be accessed!",
                            sDeviceID);
                    }


                    piSize2 = deviceSerialNumber.size();
                    if (IFGetDeviceInfo(m_handle, sDeviceID, GenTL::DEVICE_INFO_SERIAL_NUMBER, &piType, deviceSerialNumber.data(), &piSize2) != GenTL::GC_ERR_SUCCESS)
                    {
                        deviceSerialNumber = "";
                    }
                    else
                    {
                        deviceSerialNumber = deviceSerialNumber.left(piSize2 - 1);
                    }
                }

                if (!localRetVal.containsError())
                {
                    if (sDeviceID == "" && usedDeviceIDs.contains(sDeviceID)) //open next free device
                    {
                        found = true;
                    }
                    else if (deviceID == "" && accessStatus == GenTL::DEVICE_ACCESS_STATUS_READWRITE)
                    {
                        found = true;
                    }
                    else if (fetchSerialNumberNotDeviceID && deviceSerialNumber == requestedSerialNumber)
                    {
                        found = true;
                    }
                    else if (!fetchSerialNumberNotDeviceID && deviceID == sDeviceID)
                    {
                        found = true;
                    }

                    if ((found && m_verbose >= VERBOSE_ERROR) || (m_verbose >= VERBOSE_INFO) )
                    {
                        localRetVal += printDeviceInfo(sDeviceID);
                    }

                    if (found)
                    {
                        retval += localRetVal;
                        localRetVal = ito::retOk;
                        break;
                    }
                }
            }

            retval += localRetVal;
        }

        if (!retval.containsError() && !found && deviceID == "")
        {
            retval += ito::RetVal(ito::retError, 0, "No free camera devices detected for this interface.");
        }

        if (!retval.containsError())
        {
            if (!found)
            {
                //try to open the interface with the given interfaceID
                sDeviceID[0] = '\0';
                memcpy(sDeviceID, deviceID.constData(), sizeof(char) * std::min<int>(deviceID.size(), (int)piSize));
                sDeviceID[piSize - 1] = '\0';
            }

            if (m_verbose >= VERBOSE_INFO)
            {
                std::cout << "Trying to open the device '" << sDeviceID << "'...";
            }

            GenTL::DEV_HANDLE devHandle;
            GenTL::GC_ERROR err = IFOpenDevice(m_handle, sDeviceID, deviceAccess, &devHandle);
            if (err == GenTL::GC_ERR_ACCESS_DENIED)
            {

                switch (deviceAccess)
                {
                case GenTL::DEVICE_ACCESS_EXCLUSIVE:
                    std::cout << "The access to the device was denied with the flag GenTL::DEVICE_ACCESS_EXCLUSIVE. Retry with an exclusive access...\n" << std::endl;
                    break;
                case GenTL::DEVICE_ACCESS_CONTROL:
                    std::cout << "The access to the device was denied with the flag GenTL::DEVICE_ACCESS_CONTROL. Retry with an exclusive access...\n" << std::endl;
                    break;
                case GenTL::DEVICE_ACCESS_READONLY:
                    std::cout << "The access to the device was denied with the flag GenTL::DEVICE_ACCESS_READONLY. Retry with an exclusive access...\n" << std::endl;
                    break;
                default:
                    std::cout << "The access to the device was denied. Retry with an exclusive access...\n" << std::endl;
                    break;
                }

                deviceAccess = GenTL::DEVICE_ACCESS_EXCLUSIVE;
                retval += checkGCError(IFOpenDevice(m_handle, sDeviceID, deviceAccess, &devHandle), "Opening device in exclusive mode");
            }
            else
            {
                retval += checkGCError(err, "Opening device");
            }


            if (!retval.containsError())
            {
                GenTL::INFO_DATATYPE piType;
                QByteArray id(200, ' ');
                piSize = id.size();
                if (IFGetDeviceInfo(m_handle, sDeviceID, GenTL::DEVICE_INFO_ID, &piType, id.data(), &piSize) != GenTL::GC_ERR_SUCCESS)
                {
                    id = "";
                }
                else
                {
                    id = id.left(piSize - 1);
                }

                QByteArray vendor(200, ' ');
                piSize = vendor.size();
                if (IFGetDeviceInfo(m_handle, sDeviceID, GenTL::DEVICE_INFO_VENDOR, &piType, vendor.data(), &piSize) != GenTL::GC_ERR_SUCCESS)
                {
                    vendor = "";
                }
                else
                {
                    vendor = vendor.left(piSize - 1);
                }

                QByteArray model(200, ' ');
                piSize = model.size();
                if (IFGetDeviceInfo(m_handle, sDeviceID, GenTL::DEVICE_INFO_MODEL, &piType, model.data(), &piSize) != GenTL::GC_ERR_SUCCESS)
                {
                    model = "";
                }
                else
                {
                    model = model.left(piSize - 1);
                }

                QByteArray serialNumber(200, ' ');
                piSize = serialNumber.size();
                if (IFGetDeviceInfo(m_handle, sDeviceID, GenTL::DEVICE_INFO_SERIAL_NUMBER, &piType, serialNumber.data(), &piSize) != GenTL::GC_ERR_SUCCESS)
                {
                    serialNumber = "";
                }
                else
                {
                    serialNumber = serialNumber.left(piSize - 1);
                }

                QByteArray identifier(200, ' ');
                piSize = identifier.size();
                if (IFGetDeviceInfo(m_handle, sDeviceID, GenTL::DEVICE_INFO_DISPLAYNAME, &piType, identifier.data(), &piSize) != GenTL::GC_ERR_SUCCESS)
                {
                    identifier = "unknown camera";
                }
                else
                {
                    identifier = identifier.left(piSize - 1);

                    if (m_verbose >= VERBOSE_INFO)
                    {
                        std::cout << "OK. Device '" << identifier.constData() << "' opened.\n" << std::endl;
                    }
                }

                if (vendor != "" && model != "")
                {
                    identifier = (QLatin1String(vendor) + " " + QLatin1String(model)).toLatin1();

                    if (id != "")
                    {
                        identifier += " (" + QString::fromLatin1(id).toLatin1() + ")";
                    }
                }

                qDebug() << id << vendor << model << identifier << serialNumber;


                GenTLDevice *gtld = new GenTLDevice(m_lib, devHandle, sDeviceID, model, identifier, serialNumber, m_verbose, retval);

                if (!retval.containsError())
                {
                    return QSharedPointer<GenTLDevice>(gtld);
                }
                else
                {
                    DELETE_AND_SET_NULL(gtld);
                }
            }
            else if (m_verbose >= VERBOSE_INFO)
            {
                std::cout << "Error: " << retval.errorMessage() << "\n" << std::endl;
            }
        }
    }

    return QSharedPointer<GenTLDevice>();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GenTLInterface::printDeviceInfo(const char* sDeviceID) const
{
    ito::RetVal retval;

    if (!IFGetDeviceInfo)
    {
        retval += ito::RetVal(ito::retError, 0, "System not initialized");
    }
    else
    {
        GenTL::INFO_DATATYPE piType;
        char pBuffer[512];
        size_t piSize;

        GenTL::DEVICE_INFO_CMD cmds[] = { GenTL::DEVICE_INFO_ID, \
            GenTL::DEVICE_INFO_VENDOR, \
            GenTL::DEVICE_INFO_MODEL, \
            GenTL::DEVICE_INFO_TLTYPE, \
            GenTL::DEVICE_INFO_DISPLAYNAME, \
            GenTL::DEVICE_INFO_USER_DEFINED_NAME, \
            GenTL::DEVICE_INFO_SERIAL_NUMBER, \
            GenTL::DEVICE_INFO_VERSION,
            GenTL::DEVICE_INFO_ACCESS_STATUS };

        const char* names[] = { "ID:", \
            "Vendor:", \
            "Model:", \
            "TL Type:", \
            "Display Name:", \
            "User Defined Name:", \
            "Serial Number:", \
            "Device Info Version:", \
            "Access Status:" };

        std::cout << "Device information for device " << sDeviceID << "\n------------------------------------------------------------------------\n" << std::endl;

        for (int i = 0; i < sizeof(cmds) / sizeof(GenTL::DEVICE_INFO_CMD); ++i)
        {
            piSize = sizeof(pBuffer);
            GenTL::GC_ERROR err = IFGetDeviceInfo(m_handle, sDeviceID, cmds[i], &piType, &pBuffer, &piSize);

            if (err == GenTL::GC_ERR_SUCCESS)
            {
                if (piType == GenTL::INFO_DATATYPE_STRING)
                {
                    std::cout << names[i] << " " << pBuffer << "\n" << std::endl;
                }
                else if (cmds[i] == GenTL::DEVICE_INFO_ACCESS_STATUS)
                {
                    ito::int32 s = ((ito::int32*)(pBuffer))[0];
                    switch (s)
                    {
                    case GenTL::DEVICE_ACCESS_STATUS_UNKNOWN:
                        std::cout << names[i] << " unknown\n" << std::endl;
                        break;
                    case GenTL::DEVICE_ACCESS_STATUS_READWRITE:
                        std::cout << names[i] << " read/write\n" << std::endl;
                        break;
                    case GenTL::DEVICE_ACCESS_STATUS_READONLY:
                        std::cout << names[i] << " readonly\n" << std::endl;
                        break;
                    case GenTL::DEVICE_ACCESS_STATUS_NOACCESS:
                        std::cout << names[i] << " no access\n" << std::endl;
                        break;
                    case GenTL::DEVICE_ACCESS_STATUS_BUSY:
                        std::cout << names[i] << " busy\n" << std::endl;
                        break;
                    case GenTL::DEVICE_ACCESS_STATUS_OPEN_READWRITE:
                    case GenTL::DEVICE_ACCESS_STATUS_OPEN_READONLY:
                        std::cout << names[i] << " resource in use\n" << std::endl;
                        break;
                    default:
                        std::cout << names[i] << " other (" << s << ")\n" << std::endl;
                        break;
                    }
                }
                else if (piType == GenTL::INFO_DATATYPE_INT32)
                {
                    std::cout << names[i] << " " << ((ito::int32*)(pBuffer))[0] << "\n" << std::endl;
                }
            }
            else if (err == GenTL::GC_ERR_INVALID_HANDLE || err == GenTL::GC_ERR_NOT_INITIALIZED)
            {
                retval += checkGCError(err, "Device Info");
                break;
            }
        }

        std::cout << "Interface: " << m_IfaceID.constData() << "\n" << std::endl;
    }

    return retval;
}

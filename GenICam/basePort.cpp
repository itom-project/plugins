/* ********************************************************************
    Plugin "GenICam" for itom software
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

#include "basePort.h"

#include "gccommon.h"

#include <qfileinfo.h>
#include <qdebug.h>
#include <qregularexpression.h>
#include <qset.h>
#include <qurl.h>
#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"
#include <iostream>
#include <qfile.h>

#include <Base/GCBase.h>
#include <GenApi/GenApi.h>


using namespace GENAPI_NAMESPACE;
using namespace GENICAM_NAMESPACE;

/*static*/ QHash<INode*, BasePort*> BasePort::nodeDeviceHash;

//----------------------------------------------------------------------------------------------------------------------------------
BasePort::BasePort(QSharedPointer<QLibrary> lib, PortType deviceType, int verbose, ito::RetVal &retval) :
    m_lib(lib),
    m_portType(deviceType),
    m_verbose(verbose),
    DevClose(NULL),
    DevGetNumDataStreams(NULL),
    DevGetDataStreamID(NULL),
    GCGetPortInfo(NULL),
    m_portHandle(GENTL_INVALID_HANDLE),
    m_genApiConnected(false),
    m_pCallbackParameterChangedReceiver(NULL)
{

    switch (m_portType)
    {
    case TypeCamera:
        m_deviceName = "Camera";
        m_paramPrefix = "";
        break;
    case TypeFramegrabber:
        m_deviceName = "Framegrabber";
        m_paramPrefix = FRAMEGRABBER_PREFIX;
        break;
    }
    

    DevClose = (GenTL::PDevClose)m_lib->resolve("DevClose");
    DevGetNumDataStreams = (GenTL::PDevGetNumDataStreams)m_lib->resolve("DevGetNumDataStreams");
    DevGetDataStreamID = (GenTL::PDevGetDataStreamID)m_lib->resolve("DevGetDataStreamID");
    DevGetPort = (GenTL::PDevGetPort)m_lib->resolve("DevGetPort");
    GCReadPort = (GenTL::PGCReadPort)m_lib->resolve("GCReadPort");
    GCWritePort = (GenTL::PGCWritePort)m_lib->resolve("GCWritePort");
    DevOpenDataStream = (GenTL::PDevOpenDataStream)m_lib->resolve("DevOpenDataStream");
    GCGetNumPortURLs = (GenTL::PGCGetNumPortURLs)m_lib->resolve("GCGetNumPortURLs");
    GCGetPortURLInfo = (GenTL::PGCGetPortURLInfo)m_lib->resolve("GCGetPortURLInfo");
    GCGetPortInfo = (GenTL::PGCGetPortInfo)m_lib->resolve("GCGetPortInfo");

    

    if (!DevClose || !DevGetNumDataStreams || !DevGetDataStreamID || \
        !DevGetPort || !GCReadPort || !DevOpenDataStream || !GCGetNumPortURLs \
        || !GCGetPortURLInfo || !GCWritePort || !GCGetPortInfo)
    {
        retval += ito::RetVal(ito::retError, 0, QObject::tr("cti file does not export all functions of the GenTL protocol.").toLatin1().constData());
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
BasePort::~BasePort()
{
    QHash<QString, GCType*>::iterator it = m_paramMapping.begin();
    while (it != m_paramMapping.end())
    {
        delete it.value();
        ++it;
    }
    m_paramMapping.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
EAccessMode BasePort::GetAccessMode() const // if the driver is open, return RW (= read/write), otherwise NA (= not available)
{
    if (m_genApiConnected && m_portHandle != GENTL_INVALID_HANDLE)
    {
        return RW; //read/write
    }
    else
    {
        return NA; //not available
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void BasePort::Read(void *pBuffer, int64_t Address, int64_t Length) //overloded from IPort
{
    // Fetch <Length> bytes starting as <Address> from the device
    // and copy them to <pBuffer>
    if (m_portHandle != GENTL_INVALID_HANDLE)
    {
        size_t piSize = Length;
        GenTL::GC_ERROR err = GCReadPort(m_portHandle, Address, pBuffer, &piSize);

        if (m_verbose >= VERBOSE_ALL)
        {
            QByteArray data((const char*)pBuffer, piSize);

            if (data.size() <= 64)
            {
                data = data.toHex();
            }
            else
            {
                data = data.left(64).toHex() + "...";
            }

            if (err == GenTL::GC_ERR_SUCCESS)
            {
                std::cout << m_deviceName.constData() << ": Reading from port " << Address << ": Hex " << data.constData() << " (" << piSize << " Bytes).\n" << std::endl;
            }
            else
            {
                std::cerr << m_deviceName.constData() << ": Error reading from port " << Address << ": Hex " << data.constData() << " (" << piSize << " Bytes), Code " << err << ".\n" << std::endl;
            }
        }

    }
    else if (m_verbose >= VERBOSE_ALL)
    {
        std::cerr << m_deviceName.constData() << ": Error reading from port " << Address << ": port handle not available.\n" << std::endl;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void BasePort::Write(const void *pBuffer, int64_t Address, int64_t Length) //overloded from IPort
{
    // Copy <Length> bytes from <pBuffer> to the device
    // starting as <Address>
    if (m_portHandle != GENTL_INVALID_HANDLE)
    {
        size_t piSize = Length;
        GenTL::GC_ERROR err = GCWritePort(m_portHandle, Address, pBuffer, &piSize);

        if (m_verbose >= VERBOSE_ALL)
        {
            QByteArray data((const char*)pBuffer, Length);

            if (data.size() <= 64)
            {
                data = data.toHex();
            }
            else
            {
                data = data.left(64).toHex() + "...";
            }

            if (err == GenTL::GC_ERR_SUCCESS)
            {
                std::cout << m_deviceName.constData() << ": Writing to port " << Address << ": Hex " << data.constData() << " (" << Length << " Bytes).\n" << std::endl;
            }
            else
            {
                std::cerr << m_deviceName.constData() << ": Error writing to port " << Address << ": Hex " << data.constData() << " (" << Length << " Bytes), Code " << err << ".\n" << std::endl;
            }
        }
    }
    else if (m_verbose >= VERBOSE_ALL)
    {
        std::cerr << m_deviceName.constData() << ": Error writing to port " << Address << ": port handle not available.\n" << std::endl;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal BasePort::connectToGenApi(ito::uint32 portIndex)
{
    if (m_genApiConnected)
    {
        m_device._Destroy();
        m_genApiConnected = false;
    }

    ito::RetVal retval;
    QByteArray xmlFile;
    bool isXmlNotZip = true;
    QString infoString;

    if (m_portHandle == GENTL_INVALID_HANDLE)
    {
        retval += ito::RetVal(ito::retError, 0, "no port handle defined");
        return retval;
    }

    ito::uint32 numURLS;
    GCGetNumPortURLs(m_portHandle, &numURLS);

    if (portIndex >= numURLS)
    {
        retval += ito::RetVal::format(ito::retError, 0, "port index for xml description file out of range [0,%i]", numURLS - 1);
        return retval;
    }

    if (m_verbose >= VERBOSE_INFO)
    {
        std::cout << "Number of available ports for configuration files: " << numURLS << "\n" << std::endl;
        std::cout << "Trying to connect to port index " << portIndex << "...";
    }

    //get URL
    QByteArray url;
    GenTL::INFO_DATATYPE piType;
    char pBuffer[512];
    size_t piSize = sizeof(pBuffer);
    retval += checkGCError(GCGetPortURLInfo(m_portHandle, portIndex, GenTL::URL_INFO_URL, &piType, &pBuffer, &piSize), "get xml description url");
    if (retval.containsError())
    {
        if (m_verbose >= VERBOSE_INFO)
        {
            std::cout << "Error: " << retval.errorMessage() << "\n" << std::endl;
        }

        return retval;
        
    }

    

    if (piType == GenTL::INFO_DATATYPE_STRING)
    {
        url = pBuffer;
        infoString += QString("* XML URL: %1\n").arg(QLatin1String(url));
    }
    else
    {
        retval += ito::RetVal(ito::retError, 0, "URL_INFO_URL of port does not return a string for the required xml address.");

        if (m_verbose >= VERBOSE_INFO)
        {
            std::cout << "Error: " << retval.errorMessage() << "\n" << std::endl;
        }

        return retval;
    }
    
    //examples for URLs
    //url = "File:///C|\\Program Files\\Active Silicon\\GenICam_XML_File\\CXP_MC258xS11.xml?SchemaVersion=1.1.0";
    //url = "File:///C:\\Program Files\\Active Silicon\\GenICam_XML_File\\CXP_MC258xS11.xml?SchemaVersion=1.1.0";
    //url = "local:tlguru_system_rev1.xml;F0F00000;3BF?SchemaVersion=1.0.0";
    //url = "Local:Mikrotron_GmbH_MC258xS11_Rev1_25_0.zip;8001000;273A?SchemaVersion=1.1.0";
    //url = "local:///IDS Imaging Development Systems GmbH_U3-320xSE-M_7.1.1.1.100001f.1.zip;150048;AF5F?SchemaVersion=1.1.0";
    //url = "File:///C:\\IDS\\IDS_DevicePort_rev0.xml?SchemaVersion=1.1.0";
    //url = "file:///C:/IDS/ids_peak/ids_u3vgentl/64/IDS Imaging Development Systems GmbH_U3-320xSE-M_7.1.1.1.100001f.1.zip";


    if (url.toLower().startsWith("local:"))
    {
        QRegularExpression regExp("^local:(///)?([a-zA-Z0-9\\._\\- "
            "]+);([A-Fa-f0-9]+);([A-Fa-f0-9]+)(\\?SchemaVersion=.+)?$",
            QRegularExpression::CaseInsensitiveOption);

        infoString += QString("* XML file location: %1 device\n").arg(QLatin1String(m_deviceName));

        QRegularExpressionMatch match = regExp.match(url);

        if (match.hasMatch())
        {
            if (match.captured(2).endsWith("zip", Qt::CaseInsensitive))
            {
                isXmlNotZip = false;
            }

            bool ok;
            qulonglong addr = match.captured(3).toLatin1().toULongLong(&ok, 16);
            if (!ok)
            {
                retval += ito::RetVal::format(
                    ito::retError,
                    0,
                    "cannot parse '%s' as hex address",
                    match.captured(3).toLatin1().constData());
            }

            int size = match.captured(4).toLatin1().toInt(&ok, 16);
            if (!ok)
            {
                retval += ito::RetVal::format(
                    ito::retError,
                    0,
                    "cannot parse '%s' as size",
                    match.captured(4).toLatin1().constData());
            }

            if (!retval.containsError())
            {
                xmlFile.resize(size);
                size_t size_ = size;
                retval += checkGCError(GCReadPort(m_portHandle, addr, xmlFile.data(), &size_));
            }
        }
        else
        {
            retval += ito::RetVal::format(ito::retError, 0 , "the xml URL '%s' is no valid URL", url.constData());
        }
    }
    else if (url.toLower().startsWith("file:"))
    {
        infoString += QString("* XML file location: File system\n");

        QRegularExpression regExp(
            "^file:(///)?([a-zA-Z0-9\\._\\-:\\/\\\\|%\\$ -]+)(\\?schemaVersion=.+)?$",
            QRegularExpression::CaseInsensitiveOption);
        
        QRegularExpressionMatch match = regExp.match(url);

        if (match.hasMatch())
        {
            QString url1 = match.captured(2);
#ifdef WIN32
            if (url1.size() >= 2 && url1[1] == '|')
            {
                url1[1] = ':';
            }
#endif
            QUrl url2("file:///" + url1);
            QString url3 = url2.toLocalFile();

            QFile file(url3);

            if (url3.endsWith("zip", Qt::CaseInsensitive))
            {
                isXmlNotZip = false;
            }

            if (file.exists())
            {
                if (file.open(QIODevice::ReadOnly))
                {
                    xmlFile = file.readAll();
                    file.close();
                }
                else
                {
                    retval += ito::RetVal::format(ito::retError, 0, "file '%s' could not be opened (local filename: '%s').", url.constData(), url3.toLatin1().constData());
                }
            }
            else
            {
                retval += ito::RetVal::format(ito::retError, 0, "file '%s' does not exist (local filename: '%s').", url.constData(), url3.toLatin1().constData());
            }
        }
        else
        {
            retval += ito::RetVal::format(ito::retError, 0 , "the xml URL '%s' is no valid URL.", url.constData());
        }

        
    }
    else //internet resource
    {
        infoString += QString("* XML file location: Internet resource (??)\n");

        retval += ito::RetVal::format(ito::retError, 0 , "xml description file '%s' seems to be an internet resource. Cannot get this.", url.constData());
    }

    if (isXmlNotZip)
    {
        infoString += QString("* XML file type: Ascii XML\n");
    }
    else
    {
        infoString += QString("* XML file type: Zipped XML\n");
    }

    if (!retval.containsError())
    {
        if (isXmlNotZip)
        {
            m_device._LoadXMLFromString(xmlFile.constData());

            if (m_verbose >= VERBOSE_ALL)
            {
                QString filename = QString::fromLatin1(m_deviceName) + QLatin1String("_parameters.xml");
                QFile dump(filename);
                if (dump.open(QIODevice::WriteOnly | QIODevice::Text))
                {
                    dump.write(xmlFile);
                    dump.close();
                    std::cerr << m_deviceName.constData() << ": XML data saved for debugging in '" << QFileInfo(filename).absoluteFilePath().toLatin1().constData() << "'.\n" << std::endl;
                }
                else
                {
                    std::cerr << m_deviceName.constData() << ": Error opening file '" << QFileInfo(filename).absoluteFilePath().toLatin1().constData() << "' for saving xml file for debugging reasons.\n" << std::endl;
                }

            }
        }
        else
        {
            m_device._LoadXMLFromZIPData(xmlFile.constData(), xmlFile.size());

            if (m_verbose >= VERBOSE_ALL)
            {
                QString filename = QString::fromLatin1(m_deviceName) + QLatin1String("_parameters.zip");
                QFile dump(filename);
                if (dump.open(QIODevice::WriteOnly | QIODevice::Text))
                {
                    dump.write(xmlFile);
                    dump.close();
                    std::cerr << m_deviceName.constData() << ": Zipped xml data saved for debugging in '" << QFileInfo(filename).absoluteFilePath().toLatin1().constData() << "'.\n" << std::endl;
                }
                else
                {
                    std::cerr << m_deviceName.constData() << ": Error opening file '" << QFileInfo(filename).absoluteFilePath().toLatin1().constData() << "' for saving zipped xml file for debugging reasons.\n" << std::endl;
                }

            }
        }

        ito::RetVal retval_temp;
        QByteArray portname = getPortInfoString(GenTL::PORT_INFO_PORTNAME, retval_temp);
        if (retval_temp.containsError() || (portname == ""))
        {
            portname = "Device";
        }

        m_device._Connect(this, portname.constData());
        m_genApiConnected = true;

        if (m_verbose >= VERBOSE_INFO)
        {
            std::cout << "OK: \n " << infoString.toLatin1().constData() << "\n" << std::endl;

            ito::RetVal retval_;

            if (m_verbose >= VERBOSE_INFO)
            {
                std::cout << "Port Information\n----------------------------------------\n";
                std::cout << "* ID: " << getPortInfoString(GenTL::PORT_INFO_ID, retval_).constData() << "\n";
                std::cout << "* Vendor: " << getPortInfoString(GenTL::PORT_INFO_VENDOR, retval_).constData() << "\n";
                std::cout << "* Model: " << getPortInfoString(GenTL::PORT_INFO_MODEL, retval_).constData() << "\n";
                std::cout << "* TLType: " << getPortInfoString(GenTL::PORT_INFO_TLTYPE, retval_).constData() << "\n";
                std::cout << "* Module: " << getPortInfoString(GenTL::PORT_INFO_MODULE, retval_).constData() << "\n";
                std::cout << "* Version: " << getPortInfoString(GenTL::PORT_INFO_VERSION, retval_).constData() << "\n";
                std::cout << "* Portname: " << getPortInfoString(GenTL::PORT_INFO_PORTNAME, retval_).constData() << "\n";
            }
        }
    }
    else if (m_verbose >= VERBOSE_INFO)
    {
        std::cout << "Error: " << retval.errorMessage() << "\n" << std::endl;
    }

    return retval;
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal BasePort::printPortInfo(ito::uint32 index) const
{
    ito::RetVal retval;

    if (!GCGetPortURLInfo)
    {
        retval += ito::RetVal(ito::retError, 0, "System not initialized");
    }
    else
    {
        GenTL::INFO_DATATYPE piType;
        char pBuffer[512];
        size_t piSize;

        GenTL::DEVICE_INFO_CMD cmds[] = { GenTL::URL_INFO_URL };

        const char*  names[] = { "URL: " };

        std::cout << "* " << m_deviceName.constData() << "-Port " << index << ":\n" << std::endl;

        for (int i = 0; i < sizeof(cmds) / sizeof(GenTL::DEVICE_INFO_CMD); ++i)
        {
            piSize = sizeof(pBuffer);
            GenTL::GC_ERROR err = GCGetPortURLInfo(m_portHandle, index, cmds[i], &piType, &pBuffer, &piSize);

            if (err == GenTL::GC_ERR_SUCCESS)
            {
                if (piType == GenTL::INFO_DATATYPE_STRING)
                {
                    std::cout << "    - " << names[i] << pBuffer << "\n" << std::endl;
                }
                else if (piType == GenTL::INFO_DATATYPE_INT32)
                {
                    std::cout << "    - " << names[i] << ((ito::int32*)(pBuffer))[0] << "\n" << std::endl;
                }
            }
            else if (err == GenTL::GC_ERR_INVALID_HANDLE || err == GenTL::GC_ERR_NOT_INITIALIZED)
            {
                retval += checkGCError(err, QString("%1: Device Info").arg(QLatin1String(m_deviceName)));
                break;
            }
        }
    }

    if (retval.containsError())
    {
        retval = ito::RetVal(ito::retWarning, retval.errorCode(), retval.errorMessage());
    }

    return retval;
}


//--------------------------------------------------------------------------------------------------------
bool BasePort::isDeviceParam(const ParamMapIterator &it) const
{
    return m_paramMapping.contains(it->getName());
}

//--------------------------------------------------------------------------------------------------------
ito::RetVal BasePort::setDeviceParam(QSharedPointer<ito::ParamBase> newVal, ParamMapIterator it)
{
    QHash<QString, GCType*>::iterator mapit = m_paramMapping.find(it->getName());
    if (mapit == m_paramMapping.end())
    {
        return ito::RetVal(ito::retError, 0, "parameter not available in device parameters");
    }
    else
    {
        ito::RetVal retval;
        try
        {
            retval += mapit.value()->setValue(newVal.data());
        }
        catch (GenericException &ex)
        {
            retval += ito::RetVal::format(ito::retError, 0, "Error setting parameter '%s': %s", it->getName(), ex.GetDescription());
        }
        return retval;
    }
}

//--------------------------------------------------------------------------------------------------------
void BasePort::setParamsLocked(bool locked)
{
    try
    {
        CIntegerPtr pInt = m_device._GetNode("TLParamsLocked");
        if (pInt.IsValid())
        {
            *pInt = (locked ? 1 : 0);
            if (m_verbose >= VERBOSE_DEBUG)
            {
                std::cout << "Set TLParamsLocked (of " << m_deviceName.constData() << ") to " << *pInt() << "\n" << std::endl;
            }
        }
    }
    catch (GenericException & ex)
    {
        if (m_verbose >= VERBOSE_DEBUG)
        {
            std::cout << "Error setting TLParamsLocked (of device): " << ex.GetDescription() << "\n" << std::endl;
        }
    }
}

//--------------------------------------------------------------------------------------------------------
void BasePort::setCallbackParameterChangedReceiver(QObject* receiver)
{
    m_pCallbackParameterChangedReceiver = receiver;
}

//--------------------------------------------------------------------------------------------------------
/*static*/ void BasePort::callbackParameterChanged(INode *pNode)
{
    QHash<INode*, BasePort*>::iterator it = nodeDeviceHash.find(pNode);
    if (it != nodeDeviceHash.end())
    {
        it.value()->callbackParameterChanged_(pNode);
    }
}

//--------------------------------------------------------------------------------------------------------
ito::RetVal BasePort::createParamsFromDevice(QMap<QString, ito::Param> &params, int visibilityLevel /*= GenApi::Guru*/)
{
    if (m_verbose >= VERBOSE_DEBUG)
    {
        std::cout << "Parameter scan of " << m_deviceName.constData() << "\n----------------------------------------\n" << std::endl;
    }

    ito::RetVal retval;
    ito::RetVal tempRetVal;
    GenApi::NodeList_t nodes;
    m_device._GetNodes(nodes);
    GenApi::INode *node;
    GenApi::EInterfaceType interfaceType;
    GenApi::EVisibility visibility;
    bool addIt = false;
    QMap<GenApi::INode*, ito::ByteArray> categoryMap;
    ito::ByteArray category;

    //at first look for all categories:
    for (int i = 0; i < nodes.size(); ++i)
    {
        node = nodes[i];
        interfaceType = node->GetPrincipalInterfaceType();

        QByteArray name = m_paramPrefix + node->GetName().c_str();
        visibility = node->GetVisibility();

        if (visibility <= visibilityLevel)
        {
            try
            {
                if (interfaceType == GenApi::intfICategory)
                {
                    GenApi::CCategoryPtr categoryPtr(node);
                    GenApi::FeatureList_t features;
                    categoryPtr->GetFeatures(features);
                    if (m_verbose >= VERBOSE_DEBUG)
                    {
                        std::cout << "Category " << name.constData() << " (Access: " << (int)node->GetAccessMode() << ")\n" << std::endl;
                    }

                    for (int i = 0; i < features.size(); ++i)
                    {
                        categoryMap[features[i]->GetNode()] = name;

                        if (m_verbose >= VERBOSE_DEBUG)
                        {
                            const INode *child = features[i]->GetNode();

                            switch (child->GetPrincipalInterfaceType())
                            {
                            case intfIValue:
                                std::cout << " -- " << child->GetName() << " (A:" << (int)child->GetAccessMode() << ", I:Value, V:" << child->GetVisibility() << ")\n";
                                break;
                            case intfIBase:
                                std::cout << " -- " << child->GetName() << " (A:" << (int)child->GetAccessMode() << ", I:Integer, V:" << child->GetVisibility() << ")\n";
                                break;
                            case intfIInteger:
                                std::cout << " -- " << child->GetName() << " (A:" << (int)child->GetAccessMode() << ", I:Integer, V:" << child->GetVisibility() << ")\n";
                                break;
                            case intfIBoolean:
                                std::cout << " -- " << child->GetName() << " (A:" << (int)child->GetAccessMode() << ", I:Boolean, V:" << child->GetVisibility() << ")\n";
                                break;
                            case intfICommand:
                                std::cout << " -- " << child->GetName() << " (A:" << (int)child->GetAccessMode() << ", I:Command, V:" << child->GetVisibility() << ")\n";
                                break;
                            case intfIFloat:
                                std::cout << " -- " << child->GetName() << " (A:" << (int)child->GetAccessMode() << ", I:Float, V:" << child->GetVisibility() << ")\n";
                                break;
                            case intfIString:
                                std::cout << " -- " << child->GetName() << " (A:" << (int)child->GetAccessMode() << ", I:String, V:" << child->GetVisibility() << ")\n";
                                break;
                            case intfICategory:
                                std::cout << " -- " << child->GetName() << " (A:" << (int)child->GetAccessMode() << ", I:Category, V:" << child->GetVisibility() << ")\n";
                                break;
                            case intfIRegister:
                                std::cout << " -- " << child->GetName() << " (A:" << (int)child->GetAccessMode() << ", I:Register, V:" << child->GetVisibility() << ")\n";
                                break;
                            case intfIEnumeration:
                                std::cout << " -- " << child->GetName() << " (A:" << (int)child->GetAccessMode() << ", I:Enumeration, V:" << child->GetVisibility() << ")\n";
                                break;
                            case intfIEnumEntry:
                                std::cout << " -- " << child->GetName() << " (A:" << (int)child->GetAccessMode() << ", I:EnumEntry, V:" << child->GetVisibility() << ")\n";
                                break;
                            case intfIPort:
                                std::cout << " -- " << child->GetName() << " (A:" << (int)child->GetAccessMode() << ", I:Port, V:" << child->GetVisibility() << ")\n";
                                break;
                            }


                            //std::cout << "        " << (child->GetAlias() ? child->GetAlias()->GetName() : "no alias") << child->IsFeature() << child->IsStreamable() <<" - " << nodeList.size() << (nodeList.size() > 0 ? nodeList[0]->GetName() : "") << "\n";
                        }
                    }
                }
            }
            catch (GenericException & ex)
            {
                //
                if (m_verbose >= VERBOSE_DEBUG)
                {
                    std::cerr << name.constData() << "::" << ex.what() << "\n" << std::endl;
                }
            }
        }
    }

    if (m_verbose >= VERBOSE_DEBUG)
    {
        std::cout << "----------------------------------------\n" << std::endl;
    }


    //now scan for all the rest
    for (int i = 0; i < nodes.size(); ++i)
    {
        node = nodes[i];
        interfaceType = node->GetPrincipalInterfaceType();
        visibility = node->GetVisibility();
        addIt = false;

        QByteArray name = m_paramPrefix + node->GetName().c_str();

        if (visibility <= visibilityLevel)
        {

            category = categoryMap.contains(node) ? categoryMap[node] : ito::ByteArray();

            try
            {
                switch (interfaceType)
                {
                case GenApi::intfIInteger:
                    tempRetVal += createIntParamFromDevice(node, params, category);
                    addIt = true;
                    break;
                case GenApi::intfIFloat:
                    tempRetVal += createFloatParamFromDevice(node, params, category);
                    addIt = true;
                    break;
                case GenApi::intfIString:
                    tempRetVal += createStringParamFromDevice(node, params, category);
                    addIt = true;
                    break;
                case GenApi::intfIBoolean:
                    tempRetVal += createBoolParamFromDevice(node, params, category);
                    addIt = true;
                    break;
                case GenApi::intfIEnumeration:
                    tempRetVal += createEnumParamFromDevice(node, params, category);
                    addIt = true;
                    break;
                case GenApi::intfICommand:
                    {
                        m_commandNodes.insert(name.constData(), CCommandPtr(node));
                        qDebug() << "Command " << node->GetName() << " (" << interfaceType << "): " << (int)node->GetAccessMode();
                        if (m_verbose >= VERBOSE_DEBUG)
                        {
                            addIt = true;
                        }
                    }
                    break;
                case GenApi::intfICategory:
                    //
                    break;
                default:
                    qDebug() << "Property " << name << " (" << interfaceType << "): " << (int)node->GetAccessMode();
                    addIt = true;
                    break;
                }

                if (addIt)
                {
                    nodeDeviceHash[node] = this;
                    Register(node, &callbackParameterChanged);
                }
            }
            catch (GenericException &ex)
            {
                retval += ito::RetVal::format(ito::retWarning, 0, "Error parsing parameter '%s': %s", name.constData(), ex.GetDescription());
            }
        }
    }

    return retval;
}


//--------------------------------------------------------------------------------------------------------
ito::RetVal BasePort::createIntParamFromDevice(GenApi::INode *node, QMap<QString, ito::Param> &params, ito::ByteArray &category)
{
    QString name = QLatin1String(m_paramPrefix) + node->GetName().c_str();
    ParamMapIterator it = params.find(name);
    QByteArray description = node->GetDescription().c_str();
    GenApi::EAccessMode accessMode = node->GetAccessMode();

    if (accessMode == GenApi::RO || accessMode == GenApi::RW || accessMode == GenApi::NA) //Write-only is also not allowed
    {
        if (it == params.end())
        {
            it = params.insert(name, ito::Param(name.toLatin1().constData(), ito::ParamBase::Int | ito::ParamBase::In, NULL, description.constData()));
            ito::IntMeta *meta = new ito::IntMeta(0, 0, 1);
            meta->setCategory(category);
            it->setMeta(meta, true);
        }

        if (!m_paramMapping.contains(name))
        {
            GenApi::CIntegerPtr enumPtr(node);
            m_paramMapping[name] = new GCIntType(&params, name, enumPtr);
            m_paramMapping2[node] = m_paramMapping[name];
        }
        GCType *gctype = m_paramMapping[name];
        gctype->update(false);
    }
    else
    {
        if (it != params.end())
        {
            //remove it
            params.erase(it);

            if (m_paramMapping.contains(name))
            {
                delete m_paramMapping[name]; //deletes the GCIntType
                m_paramMapping.remove(name);
            }
            if (m_paramMapping2.contains(node))
            {
                m_paramMapping2.remove(node);
            }
        }
    }

    return ito::retOk;
}

//--------------------------------------------------------------------------------------------------------
ito::RetVal BasePort::createFloatParamFromDevice(GenApi::INode *node, QMap<QString, ito::Param> &params, ito::ByteArray &category)
{
    QString name = QLatin1String(m_paramPrefix) + node->GetName().c_str();
    ParamMapIterator it = params.find(name);
    QByteArray description = node->GetDescription().c_str();
    GenApi::EAccessMode accessMode = node->GetAccessMode();

    if (accessMode == GenApi::RO || accessMode == GenApi::RW || accessMode == GenApi::NA) //Write-only is also not allowed
    {
        if (it == params.end())
        {
            it = params.insert(name, ito::Param(name.toLatin1().constData(), ito::ParamBase::Double | ito::ParamBase::In, NULL, description.constData()));
            it->setMeta(new ito::DoubleMeta(0.0, 0.0, 0.0, category), true);
        }

        
        if (!m_paramMapping.contains(name))
        {
            GenApi::CFloatPtr enumPtr(node);
            m_paramMapping[name] = new GCFloatType(&params, name, enumPtr);
            m_paramMapping2[node] = m_paramMapping[name];
        }
        GCType *gctype = m_paramMapping[name];
        gctype->update(false);
    }
    else
    {
        if (it != params.end())
        {
            //remove it
            params.erase(it);

            
            if (m_paramMapping.contains(name))
            {
                delete m_paramMapping[name]; //deletes the GCFloatType
                m_paramMapping.remove(name);
            }
            if (m_paramMapping2.contains(node))
            {
                m_paramMapping2.remove(node);
            }
        }
    }

    return ito::retOk;
}

//--------------------------------------------------------------------------------------------------------
ito::RetVal BasePort::createStringParamFromDevice(GenApi::INode *node, QMap<QString, ito::Param> &params, ito::ByteArray &category)
{
    QString name = QLatin1String(m_paramPrefix) + node->GetName().c_str();
    ParamMapIterator it = params.find(name);
    QByteArray description = node->GetDescription().c_str();
    GenApi::EAccessMode accessMode = node->GetAccessMode();

    if (accessMode == GenApi::RO || accessMode == GenApi::RW || accessMode == GenApi::NA) //Write-only is also not allowed
    {
        if (it == params.end())
        {
            it = params.insert(name, ito::Param(name.toLatin1().constData(), ito::ParamBase::String | ito::ParamBase::In, NULL, description.constData()));
            ito::StringMeta *meta = new ito::StringMeta(ito::StringMeta::Wildcard, "*", category);
            it->setMeta(meta, true);
        }

        
        if (!m_paramMapping.contains(name))
        {
            GenApi::CStringPtr enumPtr(node);

            if (m_verbose >= VERBOSE_ALL)
            {
                std::cout << "String parameter " << name.toLatin1().constData() << ": MaxLength:" << enumPtr->GetMaxLength() << ", Current value: " << enumPtr->GetValue(false, true) << "\n" << std::endl;
            }
            m_paramMapping[name] = new GCStringType(&params, name, enumPtr);
            m_paramMapping2[node] = m_paramMapping[name];
        }
        GCType *gctype = m_paramMapping[name];
        gctype->update(false);
    }
    else
    {
        if (it != params.end())
        {
            //remove it
            params.erase(it);

            
            if (m_paramMapping.contains(name))
            {
                delete m_paramMapping[name]; //deletes the GCStringType
                m_paramMapping.remove(name);
            }
            if (m_paramMapping2.contains(node))
            {
                m_paramMapping2.remove(node);
            }
        }
    }

    return ito::retOk;
}

//--------------------------------------------------------------------------------------------------------
ito::RetVal BasePort::createBoolParamFromDevice(GenApi::INode *node, QMap<QString, ito::Param> &params, ito::ByteArray &category)
{
    QString name = QLatin1String(m_paramPrefix) + node->GetName().c_str();
    ParamMapIterator it = params.find(name);
    QByteArray description = node->GetDescription().c_str();
    GenApi::EAccessMode accessMode = node->GetAccessMode();

    if (accessMode == GenApi::RO || accessMode == GenApi::RW || accessMode == GenApi::NA) //Write-only is also not allowed
    {
        if (it == params.end())
        {
            it = params.insert(name, ito::Param(name.toLatin1().constData(), ito::ParamBase::Int | ito::ParamBase::In, NULL, description.constData()));
            ito::IntMeta *meta = new ito::IntMeta(0, 1, 1);
            meta->setRepresentation(ito::ParamMeta::Boolean);
            meta->setCategory(category);
            it->setMeta(meta, true);
        }

        
        if (!m_paramMapping.contains(name))
        {
            GenApi::CBooleanPtr enumPtr(node);
            m_paramMapping[name] = new GCBoolType(&params, name, enumPtr);
            m_paramMapping2[node] = m_paramMapping[name];
        }
        GCType *gctype = m_paramMapping[name];
        gctype->update(false);
    }
    else
    {
        if (it != params.end())
        {
            //remove it
            params.erase(it);

            
            if (m_paramMapping.contains(name))
            {
                delete m_paramMapping[name]; //deletes the GCBoolType
                m_paramMapping.remove(name);
            }
            if (m_paramMapping2.contains(node))
            {
                m_paramMapping2.remove(node);
            }
        }
    }

    return ito::retOk;
}

//--------------------------------------------------------------------------------------------------------
ito::RetVal BasePort::createEnumParamFromDevice(GenApi::INode *node, QMap<QString, ito::Param> &params, ito::ByteArray &category)
{
    QString name = QLatin1String(m_paramPrefix) + node->GetName().c_str();
    ParamMapIterator it = params.find(name);
    QByteArray description = node->GetDescription().c_str();
    GenApi::EAccessMode accessMode = node->GetAccessMode();

    if (accessMode == GenApi::RO || accessMode == GenApi::RW || accessMode == GenApi::NA) //Write-only is also not allowed
    {
        if (it == params.end())
        {
            it = params.insert(name, ito::Param(name.toLatin1().constData(), ito::ParamBase::String | ito::ParamBase::In, NULL, description.constData()));
            it->setMeta(new ito::StringMeta(ito::StringMeta::String, NULL, category), true);
        }

        
        if (!m_paramMapping.contains(name))
        {
            GenApi::CEnumerationPtr enumPtr(node);
            m_paramMapping[name] = new GCEnumerationType(&params, name, enumPtr);
            m_paramMapping2[node] = m_paramMapping[name];
        }
        GCType *gctype = m_paramMapping[name];
        gctype->update(false);
    }
    else
    {
        if (it != params.end())
        {
            //remove it
            params.erase(it);

            
            if (m_paramMapping.contains(name))
            {
                delete m_paramMapping[name]; //deletes the GCBoolType
                m_paramMapping.remove(name);
            }
            if (m_paramMapping2.contains(node))
            {
                m_paramMapping2.remove(node);
            }
        }
    }

    return ito::retOk;
}

QByteArray BasePort::getPortInfoString(GenTL::PORT_INFO_CMD_LIST cmd, ito::RetVal &retval) const
{

    if (!GCGetPortInfo)
    {
        retval += ito::RetVal(ito::retError, 0, "System not initialized or method GCGetInfo in transport layer not available.");
    }
    else
    {
        GenTL::INFO_DATATYPE piType;
        char pBuffer[512];
        size_t piSize = 512;
        retval += checkGCError(GCGetPortInfo(m_portHandle, cmd, &piType, &pBuffer, &piSize));

        if (!retval.containsError())
        {
            if (piType == GenTL::INFO_DATATYPE_STRING)
            {
                return QByteArray(pBuffer);
            }
            else
            {
                retval += ito::RetVal(ito::retError, 0, "info type is no string");
            }
        }
    }

    return QByteArray();
}

//--------------------------------------------------------------------------------------------------------
ito::RetVal BasePort::invokeCommandNode(const gcstring &name, ito::tRetValue errorLevel /*=ito::retError*/)
{
    if (m_commandNodes.contains(name))
    {
        try
        {
            GenApi::CCommandPtr &command = m_commandNodes[name];

            if (m_verbose >= VERBOSE_DEBUG && command->GetNode())
            {
                std::cout << m_deviceName.constData() << ": invoke command " << command->GetNode()->GetName() << ", access: " << command->GetNode()->GetAccessMode() << " (" << command->GetAccessMode() << ")\n" << std::endl;
            }

            command->Execute();
        }
        catch (GenericException &ex)
        {
            if (m_verbose >= VERBOSE_DEBUG)
            {
                std::cout << m_deviceName.constData() << ": error invoking command " << name.c_str() << " Description: " << ex.GetDescription();
            }

            if (errorLevel == ito::retError)
            {
                return ito::RetVal::format(ito::retError, 0, "%s: Error invoking command '%s': %s", m_deviceName.constData(), name.c_str(), ex.GetDescription());
            }
            else if (errorLevel == ito::retWarning)
            {
                return ito::RetVal::format(ito::retWarning, 0, "%s: Warning invoking command '%s': %s", m_deviceName.constData(), name.c_str(), ex.GetDescription());
            }
        }

        return ito::retOk;
    }
    else
    {
        if (m_verbose >= VERBOSE_DEBUG)
        {
            std::cout << m_deviceName.constData() << ": command cannot be invoked since not available: " << name.c_str();
        }

        if (errorLevel == ito::retError)
        {
            return ito::RetVal::format(ito::retError, 0, "%s: Command '%s' not available", m_deviceName.constData(), name.c_str());
        }
        else if (errorLevel == ito::retWarning)
        {
            return ito::RetVal::format(ito::retWarning, 0, "%s: Command '%s' not available", m_deviceName.constData(), name.c_str());
        }

        return ito::retOk;
    }
}

//--------------------------------------------------------------------------------------------------------
QList<gcstring> BasePort::getCommandNames() const
{
    return m_commandNodes.keys();
}


//------------------------------------------------------------------------------------------------
//call this to update the m_params["sizex"], ["sizey"] and ["bpp"]
/* The fallbackDevice might be useful for framegrabbers. In many cases the basic image
format parameters are available by the framegrabber. However, there are framegrabbers
where these parameters are only available by the camera device itself.
*/
ito::RetVal BasePort::syncImageParameters(QMap<QString, ito::Param> &params, QSharedPointer<BasePort> fallbackDevice /*= nullptr*/) 
{
    ito::RetVal retval;
    CIntegerPtr pInt;
    CEnumerationPtr pEnum;
    ito::IntMeta *intMeta;
    ParamMapIterator it, itColor;
    int sensorWidth, sensorHeight;
    int width = 0;
    int height = 0;
    int offsetX = 0;
    int offsetY = 0;
    bool roi_readonly = false;

    //SensorWidth
    it = params.find("sizex");
    intMeta = it->getMetaT<ito::IntMeta>();
    pInt = m_device._GetNode("SensorWidth");

    if (!pInt.IsValid() && fallbackDevice)
    {
        pInt = fallbackDevice->device()._GetNode("SensorWidth");
    }

    if (pInt.IsValid() && (pInt->GetAccessMode() & (RO | RW)))
    {
        intMeta->setMin(0);
        intMeta->setMax(pInt->GetValue());
        sensorWidth = pInt->GetValue();

        if (pInt->GetAccessMode() & RW)
        {
            it->setFlags(0);
        }
        else
        {
            it->setFlags(ito::ParamBase::Readonly);
        }
    }
    else
    {
        pInt = m_device._GetNode("WidthMax");

        if (!pInt.IsValid() && fallbackDevice)
        {
            pInt = fallbackDevice->device()._GetNode("WidthMax");
        }

        if (pInt.IsValid() && (pInt->GetAccessMode() & (RO | RW)))
        {
        
            intMeta->setMin(0);
            intMeta->setMax(pInt->GetValue());
            sensorWidth = pInt->GetValue();
            
            if (pInt->GetAccessMode() & RW)
            {
                it->setFlags(0);
            }
            else
            {
                it->setFlags(ito::ParamBase::Readonly);
            }
        }
    }

    ito::IntMeta widthMeta(1, intMeta->getMax(), 1);
    pInt = m_device._GetNode("Width");

    if (!pInt.IsValid() && fallbackDevice)
    {
        pInt = fallbackDevice->device()._GetNode("Width");
    }

    if (pInt.IsValid() && (pInt->GetAccessMode() & (RO | RW)))
    {
        intMetaFromInteger(pInt, intMeta);
        intMetaFromInteger(pInt, &widthMeta);
        it->setVal<ito::int32>(pInt->GetValue());
        width = pInt->GetValue();

        roi_readonly |= (pInt->GetAccessMode() != RW);
    }
    
    //SensorHeight
    it = params.find("sizey");
    intMeta = it->getMetaT<ito::IntMeta>();
    pInt = m_device._GetNode("SensorHeight");

    if (!pInt.IsValid() && fallbackDevice)
    {
        pInt = fallbackDevice->device()._GetNode("SensorHeight");
    }

    if (pInt.IsValid() && (pInt->GetAccessMode() & (RO | RW)))
    {
        
        intMeta->setMin(0);
        intMeta->setMax(pInt->GetValue());
        sensorHeight = pInt->GetValue();
        
        if (pInt->GetAccessMode() & RW)
        {
            it->setFlags(0);
        }
        else
        {
            it->setFlags(ito::ParamBase::Readonly);
        }
    }
    else
    {
        pInt = m_device._GetNode("HeightMax");

        if (!pInt.IsValid() && fallbackDevice)
        {
            pInt = fallbackDevice->device()._GetNode("HeightMax");
        }

        if (pInt.IsValid() && (pInt->GetAccessMode() & (RO | RW)))
        {
        
            intMeta->setMin(0);
            intMeta->setMax(pInt->GetValue());
            sensorHeight = pInt->GetValue();

            if (pInt->GetAccessMode() & RW)
            {
                it->setFlags(0);
            }
            else
            {
                it->setFlags(ito::ParamBase::Readonly);
            }
        }
    }

    ito::IntMeta heightMeta(1, intMeta->getMax(), 1);
    pInt = m_device._GetNode("Height");

    if (!pInt.IsValid() && fallbackDevice)
    {
        pInt = fallbackDevice->device()._GetNode("Height");
    }

    if (pInt.IsValid() && (pInt->GetAccessMode() & (RO | RW)))
    {
        intMetaFromInteger(pInt, intMeta);
        intMetaFromInteger(pInt, &heightMeta);
        height = pInt->GetValue();
        it->setVal<ito::int32>(pInt->GetValue());

        roi_readonly |= (pInt->GetAccessMode() != RW);
    }

    //roi
    ito::IntMeta offsetXMeta(0, 0, 1);
    ito::IntMeta offsetYMeta(0, 0, 1);
    pInt = m_device._GetNode("OffsetX");

    if (!pInt.IsValid() && fallbackDevice)
    {
        pInt = fallbackDevice->device()._GetNode("OffsetX");
    }

    if (pInt.IsValid() && (pInt->GetAccessMode() & (RO | RW)))
    {
        offsetX = pInt->GetValue();
        intMetaFromInteger(pInt, &offsetXMeta);
        roi_readonly |= (pInt->GetAccessMode() != RW);
    }

    pInt = m_device._GetNode("OffsetY");

    if (!pInt.IsValid() && fallbackDevice)
    {
        pInt = fallbackDevice->device()._GetNode("OffsetY");
    }

    if (pInt.IsValid() && (pInt->GetAccessMode() & (RO | RW)))
    {
        offsetY = pInt->GetValue();
        intMetaFromInteger(pInt, &offsetYMeta);
        roi_readonly |= (pInt->GetAccessMode() != RW);
    }
    

    it = params.find("roi");
    ito::RectMeta *rectMeta = it->getMetaT<ito::RectMeta>();
    rectMeta->setHeightRangeMeta(ito::RangeMeta(offsetYMeta.getMin(), offsetY + heightMeta.getMax() - 1, offsetYMeta.getStepSize(), heightMeta.getMin(), offsetY + heightMeta.getMax(), heightMeta.getStepSize()));
    rectMeta->setWidthRangeMeta(ito::RangeMeta(offsetXMeta.getMin(), offsetX + widthMeta.getMax() - 1, offsetXMeta.getStepSize(), widthMeta.getMin(), offsetX + widthMeta.getMax(), widthMeta.getStepSize()));
    int roi[] = {offsetX, offsetY, width, height}; //x,y,w,h
    it->setVal<int*>(roi, 4);
    if (roi_readonly)
    {
        it->setFlags(ito::ParamBase::Readonly);
    }
    else
    {
        it->setFlags(0);
    }

    //bpp and color
    it = params.find("bpp");
    itColor = params.find("color");
    pEnum = m_device._GetNode("PixelFormat");

    if (!pEnum.IsValid() && fallbackDevice)
    {
        pEnum = fallbackDevice->device()._GetNode("PixelFormat");
    }

    intMeta = it->getMetaT<ito::IntMeta>();

    if (pEnum.IsValid() == false || !(pEnum->GetAccessMode() & (RO | RW)))
    {
        retval += ito::RetVal(ito::retError, 0, "mandatory property 'PixelFormat' not contained in GenApi xml file");
    }
    else
    {
        if (pEnum->GetAccessMode() & RW)
        {
            it->setFlags(0);
        }
        else
        {
            it->setFlags(ito::ParamBase::Readonly);
        }

        //check if current PixelFormat is within supportedImageFormats
        supportedImageFormats();
        int idx = -1;
        int intVal = pEnum->GetIntValue();
        for (int i = 0; i < m_supportedFormats.size(); ++i)
        {
            if (m_supportedFormats[i] == intVal)
            {
                idx = i;
                break;
            }
        }

        if (idx >= 0)
        {
            it->setVal<int>(m_supportedFormatsBpp[idx]);
            intMeta->setMin(*std::min_element(m_supportedFormatsBpp.begin(), m_supportedFormatsBpp.end()));
            intMeta->setMax(*std::max_element(m_supportedFormatsBpp.begin(), m_supportedFormatsBpp.end()));
            itColor->setVal<int>(m_supportedFormatsColor[idx]);
        }
        else
        {
            //check possible pixel formats and use the first, that is supported.
            NodeList_t nodes;
            pEnum->GetEntries(nodes);
            QByteArray val;
            bool found = false;
            for (int i = 0; i < nodes.size(); ++i)
            {
                val = ((CEnumEntryPtr)nodes[i])->GetSymbolic();
                if (m_supportedFormatsNames.contains(val))
                {
                    *pEnum = val.constData();
                    //pEnum->SetIntValue(((CEnumEntryPtr)nodes[i])->GetNumericValue());
                    idx = m_supportedFormatsNames.indexOf(QLatin1String(val));
                    it->setVal<int>(m_supportedFormatsBpp[idx]);
                    intMeta->setMin(m_supportedFormatsBpp[idx]);
                    intMeta->setMax(m_supportedFormatsBpp[idx]);
                    itColor->setVal<int>(m_supportedFormatsColor[idx]);
                    found = true;
                    break;
                }
            }

            if (!found)
            {
                retval += ito::RetVal(ito::retError, 0, "no pixel format corresponds to supported formats of this plugin");
            }
        }
    }

    return retval;
}


//------------------------------------------------------------------------------------------------
void BasePort::intMetaFromInteger(const CIntegerPtr &iPtr, ito::IntMeta *intMeta) const
{

    int64_t minimum = qBound((int64_t)INT_MIN, iPtr->GetMin(), (int64_t)INT_MAX);
    int64_t maximum = qBound((int64_t)INT_MIN, iPtr->GetMax(), (int64_t)INT_MAX);

    if (minimum == -1 && maximum == -1)
    {
        intMeta->setMin(std::numeric_limits<int>::min());
        intMeta->setMax(std::numeric_limits<int>::max());
        intMeta->setStepSize(1);
    }
    else if (iPtr->GetIncMode() == noIncrement)
    {
        intMeta->setMin(minimum);
        intMeta->setMax(maximum);
        intMeta->setStepSize(1);
    }
    else if (iPtr->GetIncMode() == fixedIncrement)
    {
        intMeta->setMin(minimum);
        intMeta->setMax(maximum);
        intMeta->setStepSize(qMax((int64_t)1, iPtr->GetInc()));
    }
    else
    {
        intMeta->setMin(minimum);
        intMeta->setMax(maximum);
    }
}

//--------------------------------------------------------------------------------------------------------
QVector<PfncFormat> BasePort::supportedImageFormats(QVector<int> *bitdepths /*= NULL*/, QStringList *formatNames /*= NULL*/, QVector<int> *colortypes /*= NULL*/)
{
    if (m_supportedFormats.size() == 0)
    {

        m_supportedFormats      << Mono8 << Mono10 << Mono10Packed << Mono10p << Mono12 << Mono12Packed << Mono12p << Mono14 << Mono16 << RGB8 << YCbCr422_8 << BGR8 << BGR10p << BGR12p;
        m_supportedFormatsBpp   << 8     << 10     << 10           << 10      << 12     << 12           << 12      << 14     << 16     << 8    << 8          << 8    << 10     << 12;
        m_supportedFormatsColor << 0     << 0      << 0            << 0       << 0      << 0            << 0       << 0      << 0      << 1    << 1          << 1    << 1      << 1;

        for (int i = 0; i < m_supportedFormats.size(); ++i)
        {
            m_supportedFormatsNames << GetPixelFormatName(m_supportedFormats[i]);
        }
    }

    if (bitdepths)
    {
        *bitdepths = m_supportedFormatsBpp;
    }

    if (formatNames)
    {
        *formatNames = m_supportedFormatsNames;
    }

    if (colortypes)
    {
        *colortypes = m_supportedFormatsColor;
    }

    return m_supportedFormats;
}
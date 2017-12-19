/* ********************************************************************
    Plugin "GenICam" for itom software
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

#include "device.h"

#include "gccommon.h"

#include <qfileinfo.h>
#include <qdebug.h>
#include <qset.h>
#include <qregexp.h>
#include <qurl.h>
#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"
#include <iostream>

#include <Base/GCBase.h>
#include <GenApi/GenApi.h>


using namespace GENAPI_NAMESPACE;
using namespace GENICAM_NAMESPACE;

/*static*/ QHash<INode*, GenTLDevice*> GenTLDevice::nodeDeviceHash;

//----------------------------------------------------------------------------------------------------------------------------------
GenTLDevice::GenTLDevice(QSharedPointer<QLibrary> lib, GenTL::DEV_HANDLE devHandle, QByteArray deviceID, const QByteArray &identifier, ito::RetVal &retval) :
    m_handle(devHandle),
    m_portHandle(GENTL_INVALID_HANDLE),
    m_deviceID(deviceID),
    m_lib(lib),
    DevClose(NULL),
    DevGetNumDataStreams(NULL),
    DevGetDataStreamID(NULL),
	m_genApiConnected(false),
	m_deviceName(identifier),
	m_pCallbackParameterChangedReceiver(NULL)
{
    DevClose = (GenTL::PDevClose)m_lib->resolve("DevClose");
    DevGetNumDataStreams = (GenTL::PDevGetNumDataStreams)m_lib->resolve("DevGetNumDataStreams");
    DevGetDataStreamID = (GenTL::PDevGetDataStreamID)m_lib->resolve("DevGetDataStreamID");
    DevGetPort = (GenTL::PDevGetPort)m_lib->resolve("DevGetPort");
    GCReadPort = (GenTL::PGCReadPort)m_lib->resolve("GCReadPort");
	GCWritePort = (GenTL::PGCWritePort)m_lib->resolve("GCWritePort");
    DevOpenDataStream = (GenTL::PDevOpenDataStream)m_lib->resolve("DevOpenDataStream");
    GCGetNumPortURLs = (GenTL::PGCGetNumPortURLs)m_lib->resolve("GCGetNumPortURLs");
    GCGetPortURLInfo = (GenTL::PGCGetPortURLInfo)m_lib->resolve("GCGetPortURLInfo");

    if (!DevClose || !DevGetNumDataStreams || !DevGetDataStreamID || \
        !DevGetPort || !GCReadPort || !DevOpenDataStream || !GCGetNumPortURLs \
		|| !GCGetPortURLInfo || !GCWritePort)
    {
        retval += ito::RetVal(ito::retError, 0, QObject::tr("cti file does not export all functions of the GenTL protocol.").toLatin1().data());
    }
    else
    {
        retval += checkGCError(DevGetPort(m_handle, &m_portHandle));

        ito::uint32 numURLS;
        GCGetNumPortURLs(m_portHandle, &numURLS);

        std::cout << "Available loations for XML parameter description\n-------------------------------------------------------\n";

        for (ito::uint32 i = 0; i < numURLS; ++i)
        {
            printPortInfo(i);
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
GenTLDevice::~GenTLDevice()
{
    if (m_lib->isLoaded() && DevClose)
    {
        DevClose(m_handle);
    }

	QHash<QString, GCType*>::iterator it = m_paramMapping.begin();
	while (it != m_paramMapping.end())
	{
		delete it.value();
		++it;
	}
	m_paramMapping.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GenTLDevice::printPortInfo(ito::uint32 index) const
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

        std::cout << "Port information for port " << index << "\n-------------------------------------------\n" << std::endl;

        for (int i = 0; i < sizeof(cmds) / sizeof(GenTL::DEVICE_INFO_CMD); ++i)
        {
            piSize = sizeof(pBuffer);
            GenTL::GC_ERROR err = GCGetPortURLInfo(m_portHandle, index, cmds[i], &piType, &pBuffer, &piSize);

            if (err == GenTL::GC_ERR_SUCCESS)
            {
                if (piType == GenTL::INFO_DATATYPE_STRING)
                {
                    std::cout << names[i] << pBuffer << "\n" << std::endl;
                }
                else if (piType == GenTL::INFO_DATATYPE_INT32)
                {
                    std::cout << names[i] << ((ito::int32*)(pBuffer))[0] << "\n" << std::endl;
                }
            }
            else if (err == GenTL::GC_ERR_INVALID_HANDLE || err == GenTL::GC_ERR_NOT_INITIALIZED)
            {
                retval += checkGCError(err, "Device Info");
                break;
            }
        }

		std::cout << "-----------------------------------------------\n" << std::endl;
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer<GenTLDataStream> GenTLDevice::getDataStream(ito::int32 streamIndex, bool printInfoAboutAllStreams, ito::RetVal &retval)
{
    QSharedPointer<GenTLDataStream> stream;
    if (!m_handle)
    {
        retval += ito::RetVal(ito::retError, 0, "invalid device handle");
    }
    else
    {
        //get available data streams
        ito::uint32 numDataStreams;
        retval += checkGCError(DevGetNumDataStreams(m_handle, &numDataStreams), "Get number of data streams");

        char dataStreamID[512];
        size_t pSize;
        QByteArray streamIdToOpen = "";

        if (printInfoAboutAllStreams)
        {
            std::cout << "Stream information of device\n---------------------------------------\n" << std::endl;
        }

        if (!retval.containsError())
        {
            
            for (ito::uint32 i = 0; i < numDataStreams; ++i)
            {
                pSize = sizeof(dataStreamID);
                if (DevGetDataStreamID(m_handle, i, dataStreamID, &pSize) == GenTL::GC_ERR_SUCCESS)
                {
                    if (streamIndex == i)
                    {
                        streamIdToOpen = QByteArray(dataStreamID);
                    }

                    if (printInfoAboutAllStreams)
                    {
                        std::cout << i << ". Name: " << dataStreamID << "\n" << std::endl;
                    }
                }
            }
        }

        if (!retval.containsError())
        {
            if (streamIdToOpen == "")
            {
                retval += ito::RetVal::format(ito::retError, 0, "No stream with index %i found.", streamIndex);
            }
            else
            {
                GenTL::DS_HANDLE streamHandle;
                retval += checkGCError(DevOpenDataStream(m_handle, streamIdToOpen.data(), &streamHandle), "open data stream");

                if (!retval.containsError())
                {
                    stream = QSharedPointer<GenTLDataStream>(new GenTLDataStream(m_lib, streamHandle, retval));
                    if (retval.containsError())
                    {
                        stream.clear();
                    }
                }
            }
        }
    }

    return stream;
}

//----------------------------------------------------------------------------------------------------------------------------------
EAccessMode GenTLDevice::GetAccessMode() const // if the driver is open, return RW (= read/write), otherwise NA (= not available)
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
void GenTLDevice::Read(void *pBuffer, int64_t Address, int64_t Length) //overloded from IPort
{
	// Fetch <Length> bytes starting as <Address> from the camera
	// and copy them to <pBuffer>
	if (m_portHandle != GENTL_INVALID_HANDLE)
	{
		size_t piSize = Length;
		GCReadPort(m_portHandle, Address, pBuffer, &piSize);
	}
}

//----------------------------------------------------------------------------------------------------------------------------------
void GenTLDevice::Write(const void *pBuffer, int64_t Address, int64_t Length) //overloded from IPort
{
	// Copy <Length> bytes from <pBuffer> to the camera
	// starting as <Address>
	if (m_portHandle != GENTL_INVALID_HANDLE)
	{
		size_t piSize = Length;
		GCWritePort(m_portHandle, Address, pBuffer, &piSize);
	}
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GenTLDevice::connectToGenApi(ito::uint32 portIndex)
{
	if (m_genApiConnected)
	{
		m_camera._Destroy();
		m_genApiConnected = false;
	}

	ito::RetVal retval;
	QByteArray xmlFile;
    bool isXmlNotZip = true;

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

	std::cout << "Connecting to port index " << portIndex << "\n" << std::endl;

    //get URL
    QByteArray url;
    GenTL::INFO_DATATYPE piType;
    char pBuffer[512];
    size_t piSize = sizeof(pBuffer);
    retval += checkGCError(GCGetPortURLInfo(m_portHandle, portIndex, GenTL::URL_INFO_URL, &piType, &pBuffer, &piSize), "get xml description url");
    if (retval.containsError())
    {
		return retval;
    }

    if (piType == GenTL::INFO_DATATYPE_STRING)
    {
        url = pBuffer;
    }
    else
    {
        retval += ito::RetVal(ito::retError, 0, "URL_INFO_URL of port does not return a string for the required xml address.");
		return retval;
    }
    
	//examples for URLs
	//url = "File:///C|\\Program Files\\Active Silicon\\GenICam_XML_File\\CXP_MC258xS11.xml?SchemaVersion=1.1.0";
	//url = "File:///C:\\Program Files\\Active Silicon\\GenICam_XML_File\\CXP_MC258xS11.xml?SchemaVersion=1.1.0";
	//url = "local:tlguru_system_rev1.xml;F0F00000;3BF?SchemaVersion=1.0.0";
	//url = "Local:Mikrotron_GmbH_MC258xS11_Rev1_25_0.zip;8001000;273A?SchemaVersion=1.1.0";

    if (url.toLower().startsWith("local:"))
    {
        QRegExp regExp("^local:(///)?([a-zA-Z0-9._]+);([A-Fa-f0-9]+);([A-Fa-f0-9]+)(\\?schemaVersion=.+)?$");
        regExp.setCaseSensitivity(Qt::CaseInsensitive);

        if (regExp.indexIn(url) >= 0)
        {
            if (regExp.cap(2).endsWith("zip", Qt::CaseInsensitive))
            {
                isXmlNotZip = false;
            }

            bool ok;
            qulonglong addr = regExp.cap(3).toLatin1().toULongLong(&ok, 16);
            if (!ok)
            {
                retval += ito::RetVal::format(ito::retError, 0 , "cannot parse '%s' as hex address", regExp.cap(3).toLatin1().constData());
            }

            int size = regExp.cap(4).toLatin1().toInt(&ok, 16);
            if (!ok)
            {
                retval += ito::RetVal::format(ito::retError, 0 , "cannot parse '%s' as size", regExp.cap(4).toLatin1().constData());
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
        QRegExp regExp("^file:(///)?([a-zA-Z0-9._:\\/\\\\|%\\$ -]+)(\\?schemaVersion=.+)?$");
        regExp.setCaseSensitivity(Qt::CaseInsensitive);

        if (regExp.indexIn(url) >= 0)
        {
			QString url1 = regExp.cap(2);
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
                    retval += ito::RetVal::format(ito::retError, 0, "file '%s' could not be opened (local filename: '%s').", url.data(), url3.toLatin1().constData());
                }
            }
            else
            {
                retval += ito::RetVal::format(ito::retError, 0, "file '%s' does not exist (local filename: '%s').", url.data(), url3.toLatin1().constData());
            }
        }
        else
        {
            retval += ito::RetVal::format(ito::retError, 0 , "the xml URL '%s' is no valid URL.", url.constData());
        }

        
    }
    else //internet resource
    {
        retval += ito::RetVal::format(ito::retError, 0 , "xml description file '%s' seems to be an internet resource. Cannot get this.", url.constData());
    }

	if (!retval.containsError())
	{
        if (isXmlNotZip)
        {
		    m_camera._LoadXMLFromString(xmlFile.constData());
        }
        else
        {
            m_camera._LoadXMLFromZIPData(xmlFile.constData(), xmlFile.size());
        }

		m_camera._Connect(this, "Device");
		m_genApiConnected = true;
	}

	/*gcstring xmlData(xmlFile.data());
	CNodeMapFactory::NodeStatistics_t stats;
	gcstring_vector schemaVersions;

	CNodeMapFactory nodeMapFactory(xmlData);
	CNodeMapRef camera;
	nodeMapFactory.GetNodeStatistics(stats);
	nodeMapFactory.GetSupportedSchemaVersions(schemaVersions);

	for (int i = 0; i < schemaVersions.size(); ++i)
	{
		qDebug() << schemaVersions[i].c_str();
	}*/

	/*INodeMap *nodeMap = nodeMapFactory.CreateNodeMap();
	INode *node = NULL;

	qDebug() << nodeMap->GetDeviceName().c_str();
	GenApi::NodeList_t nodes;
	nodeMap->GetNodes(nodes);

	for (int i = 0; i < nodes.size(); ++i)
	{
		qDebug() << nodes[i]->GetName().c_str() << nodes[i]->GetDescription().c_str();
	}*/

	/*node = nodeMap->GetNode("PayloadSize");
	node->GetPropertyNames()
	node->GetProperty("pPort", )*/

    return retval;
}

//------------------------------------------------------------------------------------------------
int64_t GenTLDevice::getIntParam(const gcstring &name, bool *valid /*= NULL*/)
{
	CIntegerPtr p = m_camera._GetNode(name);

	if (valid)
	{
		*valid = p.IsValid();
	}

	if (p.IsValid())
	{
		return p->GetValue();
	}
	
	return 0;
}

//------------------------------------------------------------------------------------------------
void GenTLDevice::intMetaFromInteger(const CIntegerPtr &iPtr, ito::IntMeta *intMeta) const
{

	int64_t minimum = qBound((int64_t)INT_MIN, iPtr->GetMin(), (int64_t)INT_MAX);
	int64_t maximum = qBound((int64_t)INT_MIN, iPtr->GetMax(), (int64_t)INT_MAX);

	if (minimum == -1 && maximum == -1)
	{
		intMeta->setMin(std::numeric_limits<int>::min());
		intMeta->setMax(std::numeric_limits<int>::max());
		intMeta->setStepSize(1);
	}
	else if (iPtr->GetIncMode() == EIncMode::noIncrement)
	{
		intMeta->setMin(minimum);
		intMeta->setMax(maximum);
		intMeta->setStepSize(1);
	}
	else if (iPtr->GetIncMode() == EIncMode::fixedIncrement)
	{
		intMeta->setMin(minimum);
		intMeta->setMax(maximum);
		intMeta->setStepSize(iPtr->GetInc());
	}
	else
	{
		intMeta->setMin(minimum);
		intMeta->setMax(maximum);
	}
}

//------------------------------------------------------------------------------------------------
ito::RetVal GenTLDevice::syncImageParameters(QMap<QString, ito::Param> &params) //call this to update the m_params["sizex"], ["sizey"] and ["bpp"]
{
	ito::RetVal retval;
	CIntegerPtr pInt;
	CEnumerationPtr pEnum;
	ito::IntMeta *intMeta;
	ParamMapIterator it;
	int heightMax, widthMax;
	int sensorWidth, sensorHeight;

	//WidthMax
	pInt = m_camera._GetNode("WidthMax");
	if (pInt.IsValid())
	{
		widthMax = pInt->GetValue();
	}

	//HeightMax
	pInt = m_camera._GetNode("HeightMax");
	if (pInt.IsValid())
	{
		heightMax = pInt->GetValue();
	}

	//SensorWidth
	it = params.find("sizex");
	intMeta = it->getMetaT<ito::IntMeta>();
	pInt = m_camera._GetNode("SensorWidth");
	if (pInt.IsValid())
	{
		intMeta->setMin(0);
		intMeta->setMax(pInt->GetValue());
		sensorWidth = pInt->GetValue();
		params["sizex"].setFlags(ito::ParamBase::Readonly);
	}

	ito::IntMeta widthMeta(1, intMeta->getMax(), 1);
	int width = 0;
	pInt = m_camera._GetNode("Width");
	if (pInt.IsValid())
	{
		intMetaFromInteger(pInt, intMeta);
		intMetaFromInteger(pInt, &widthMeta);
		it->setVal<ito::int32>(pInt->GetValue());
		width = pInt->GetValue();
	}
	
	//SensorHeight
	it = params.find("sizey");
	intMeta = it->getMetaT<ito::IntMeta>();
	pInt = m_camera._GetNode("SensorHeight");
	if (pInt.IsValid())
	{
		
		intMeta->setMin(0);
		intMeta->setMax(pInt->GetValue());
		sensorHeight = pInt->GetValue();
        params["sizey"].setFlags(ito::ParamBase::Readonly);
	}

	ito::IntMeta heightMeta(1, intMeta->getMax(), 1);
	int height = 0;
	pInt = m_camera._GetNode("Height");
	if (pInt.IsValid())
	{
		intMetaFromInteger(pInt, intMeta);
		intMetaFromInteger(pInt, &heightMeta);
		height = pInt->GetValue();
		it->setVal<ito::int32>(pInt->GetValue());
	}

	//roi
	int offsetX = 0;
	ito::IntMeta offsetXMeta(0, 0, 1);
	int offsetY = 0;
	ito::IntMeta offsetYMeta(0, 0, 1);
	pInt = m_camera._GetNode("OffsetX");
	if (pInt.IsValid())
	{
		offsetX = pInt->GetValue();
		intMetaFromInteger(pInt, &offsetXMeta);
	}
	pInt = m_camera._GetNode("OffsetY");
	if (pInt.IsValid())
	{
		offsetY = pInt->GetValue();
		intMetaFromInteger(pInt, &offsetYMeta);
	}
	

	it = params.find("roi");
	ito::RectMeta *rectMeta = it->getMetaT<ito::RectMeta>();
	rectMeta->setHeightRangeMeta(ito::RangeMeta(offsetYMeta.getMin(), offsetY + heightMeta.getMax() - 1, offsetYMeta.getStepSize(), heightMeta.getMin(), offsetY + heightMeta.getMax(), heightMeta.getStepSize()));
	rectMeta->setWidthRangeMeta(ito::RangeMeta(offsetXMeta.getMin(), offsetX + widthMeta.getMax() - 1, offsetXMeta.getStepSize(), widthMeta.getMin(), offsetX + widthMeta.getMax(), widthMeta.getStepSize()));
	int roi[] = {offsetX, offsetY, width, height}; //x,y,w,h
	it->setVal<int*>(roi, 4);

	//bpp
	it = params.find("bpp");
	pEnum = m_camera._GetNode("PixelFormat");
	intMeta = it->getMetaT<ito::IntMeta>();

	if (pEnum.IsValid() == false)
	{
		retval += ito::RetVal(ito::retError, 0, "mandatory property 'PixelFormat' not contained in GenApi xml file");
	}
	else
	{
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
		}
		else
		{
			//check possible pixel formats and use the first, that is supported.
			NodeList_t nodes;
			pEnum->GetEntries(nodes);
			QString val;
			bool found = false;
			for (int i = 0; i < nodes.size(); ++i)
			{
				val = ((CEnumEntryPtr)nodes[i])->GetSymbolic();
				if (m_supportedFormatsNames.contains(val))
				{
					pEnum->SetIntValue(((CEnumEntryPtr)nodes[i])->GetNumericValue());
					idx = m_supportedFormatsNames.indexOf(val);
					it->setVal<int>(m_supportedFormatsBpp[idx]);
					intMeta->setMin(m_supportedFormatsBpp[idx]);
					intMeta->setMax(m_supportedFormatsBpp[idx]);
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



//--------------------------------------------------------------------------------------------------------
int GenTLDevice::getPayloadSize() const
{
	CIntegerPtr pInt = m_camera._GetNode("PayloadSize");
	if (pInt.IsValid())
	{
		return pInt->GetValue();
	}

	return 0;
}

//--------------------------------------------------------------------------------------------------------
QVector<PfncFormat> GenTLDevice::supportedImageFormats(QVector<int> *bitdepths /*= NULL*/, QStringList *formatNames /*= NULL*/)
{
	if (m_supportedFormats.size() == 0)
	{
		m_supportedFormats << PfncFormat::Mono8 << PfncFormat::Mono10 << PfncFormat::Mono12 << PfncFormat::Mono14 << PfncFormat::Mono16 << PfncFormat::Mono12Packed;
		m_supportedFormatsBpp << 8 << 10 << 12 << 14 << 16 << 12;

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

	return m_supportedFormats;
}

//--------------------------------------------------------------------------------------------------------
ito::RetVal GenTLDevice::createParamsFromDevice(QMap<QString, ito::Param> &params, int visibilityLevel /*= GenApi::Guru*/)
{
	ito::RetVal retval;
	ito::RetVal tempRetVal;
	GenApi::NodeList_t nodes;
	m_camera._GetNodes(nodes);
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

		QByteArray name = node->GetName().c_str();
		const char* d = name.data();
		visibility = node->GetVisibility();

		if ((visibility != GenApi::Invisible) && (visibility <= visibilityLevel))
		{
			try
			{
				if (node->GetPrincipalInterfaceType() == GenApi::intfICategory)
				{
					GenApi::CCategoryPtr categoryPtr(node);
					GenApi::FeatureList_t features;
					categoryPtr->GetFeatures(features);
					qDebug() << "Category " << node->GetName() << " (" << interfaceType << "): " << (int)node->GetAccessMode();
					for (int i = 0; i < features.size(); ++i)
					{
						categoryMap[features[i]->GetNode()] = node->GetName().c_str();
						qDebug() << " -- " << features[i]->GetNode()->GetName().c_str();
					}
				}
			}
			catch (GenericException & /*ex*/)
			{
				//
			}
		}
	}


	//now scan for all the rest
	for (int i = 0; i < nodes.size(); ++i)
	{
		node = nodes[i];
		interfaceType = node->GetPrincipalInterfaceType();
		visibility = node->GetVisibility();
		addIt = false;

		QByteArray name = node->GetName().c_str();
		const char* d = name.data();

		if ((visibility != GenApi::Invisible) && (visibility <= visibilityLevel))
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
					m_commandNodes.insert(node->GetName(), CCommandPtr(node));
					qDebug() << "Command " << node->GetName() << " (" << interfaceType << "): " << (int)node->GetAccessMode();
					//case GenApi::EInterfaceType::intfIEnumEntry:
					//qDebug() << "Property " << name << " (Enum Entry): " << description << accessMode;
					break;
				case GenApi::intfICategory:
					//
					break;
				default:
					qDebug() << "Property " << node->GetName() << " (" << interfaceType << "): " << (int)node->GetAccessMode();
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
				retval += ito::RetVal::format(ito::retWarning, 0, "Error parsing parameter '%s': %s", node->GetName().c_str(), ex.GetDescription());
			}
		}
		else
		{
			qDebug() << "Property " << node->GetName() << " (" << interfaceType << "): invisible";
		}
	}

	return retval;
}

//--------------------------------------------------------------------------------------------------------
/*static*/ void GenTLDevice::callbackParameterChanged(INode *pNode)
{
	QHash<INode*, GenTLDevice*>::iterator it = nodeDeviceHash.find(pNode);
	if (it != nodeDeviceHash.end())
	{
		it.value()->callbackParameterChanged_(pNode);
	}
}

//--------------------------------------------------------------------------------------------------------
void GenTLDevice::callbackParameterChanged_(INode *pNode)
{
	/*std::cout << "The node '" << pNode->GetName() << "' has been invalidated\n";
	CValuePtr ptrValue = pNode;
	if (ptrValue.IsValid())
	{
		std::cout << "Value = " << ptrValue->ToString() << ", access: " << ptrValue->GetAccessMode() << "\n" << std::endl;
	}*/

	if (m_callbackParameterChangedTimer.isNull())
	{
		m_callbackParameterChangedTimer = QSharedPointer<QTimer>(new QTimer());
		m_callbackParameterChangedTimer->setSingleShot(true);
		m_callbackParameterChangedTimer->setInterval(5);
		m_callbackParameterChangedTimer->stop();
		QObject::connect(m_callbackParameterChangedTimer.data(), SIGNAL(timeout()), m_pCallbackParameterChangedReceiver, SLOT(parameterChangedTimerFired()));
	}

	if (m_paramMapping2.contains(pNode))
	{
		m_paramMapping2[pNode]->update(false);
	}

	m_callbackParameterChangedTimer->start();
}

//--------------------------------------------------------------------------------------------------------
void GenTLDevice::setCallbackParameterChangedReceiver(QObject* receiver)
{
	m_pCallbackParameterChangedReceiver = receiver;
}

//--------------------------------------------------------------------------------------------------------
ito::RetVal GenTLDevice::createIntParamFromDevice(GenApi::INode *node, QMap<QString, ito::Param> &params, ito::ByteArray &category)
{
	QString name = node->GetName().c_str();
	ParamMapIterator it = params.find(name);
	QByteArray description = node->GetDescription().c_str();
	GenApi::EAccessMode accessMode = node->GetAccessMode();

	if (accessMode == GenApi::RO || accessMode == GenApi::RW || accessMode == GenApi::NA) //Write-only is also not allowed
	{
		if (it == params.end())
		{
			it = params.insert(name, ito::Param(name.toLatin1().data(), ito::ParamBase::Int | ito::ParamBase::In, NULL, description.data()));
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
ito::RetVal GenTLDevice::createFloatParamFromDevice(GenApi::INode *node, QMap<QString, ito::Param> &params, ito::ByteArray &category)
{
	QString name = node->GetName().c_str();
	ParamMapIterator it = params.find(name);
	QByteArray description = node->GetDescription().c_str();
	GenApi::EAccessMode accessMode = node->GetAccessMode();

	if (accessMode == GenApi::RO || accessMode == GenApi::RW || accessMode == GenApi::NA) //Write-only is also not allowed
	{
		if (it == params.end())
		{
			it = params.insert(name, ito::Param(name.toLatin1().data(), ito::ParamBase::Double | ito::ParamBase::In, NULL, description.data()));
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
ito::RetVal GenTLDevice::createStringParamFromDevice(GenApi::INode *node, QMap<QString, ito::Param> &params, ito::ByteArray &category)
{
	QString name = node->GetName().c_str();
	ParamMapIterator it = params.find(name);
	QByteArray description = node->GetDescription().c_str();
	GenApi::EAccessMode accessMode = node->GetAccessMode();

	if (accessMode == GenApi::RO || accessMode == GenApi::RW || accessMode == GenApi::NA) //Write-only is also not allowed
	{
		if (it == params.end())
		{
			it = params.insert(name, ito::Param(name.toLatin1().data(), ito::ParamBase::String | ito::ParamBase::In, NULL, description.data()));
			ito::StringMeta *meta = new ito::StringMeta(ito::StringMeta::tType::Wildcard, "*", category);
			it->setMeta(meta, true);
		}

		
        if (!m_paramMapping.contains(name))
		{
			GenApi::CStringPtr enumPtr(node);
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
ito::RetVal GenTLDevice::createBoolParamFromDevice(GenApi::INode *node, QMap<QString, ito::Param> &params, ito::ByteArray &category)
{
	QString name = node->GetName().c_str();
	ParamMapIterator it = params.find(name);
	QByteArray description = node->GetDescription().c_str();
	GenApi::EAccessMode accessMode = node->GetAccessMode();

	if (accessMode == GenApi::RO || accessMode == GenApi::RW || accessMode == GenApi::NA) //Write-only is also not allowed
	{
		if (it == params.end())
		{
			it = params.insert(name, ito::Param(name.toLatin1().data(), ito::ParamBase::Int | ito::ParamBase::In, NULL, description.data()));
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
ito::RetVal GenTLDevice::createEnumParamFromDevice(GenApi::INode *node, QMap<QString, ito::Param> &params, ito::ByteArray &category)
{
	QString name = node->GetName().c_str();
	ParamMapIterator it = params.find(name);
    qDebug() << name << &it.value() << " meta:" << (void*)it->getMeta();
	QByteArray description = node->GetDescription().c_str();
	GenApi::EAccessMode accessMode = node->GetAccessMode();

	if (accessMode == GenApi::RO || accessMode == GenApi::RW || accessMode == GenApi::NA) //Write-only is also not allowed
	{
		if (it == params.end())
		{
			it = params.insert(name, ito::Param(name.toLatin1().data(), ito::ParamBase::String | ito::ParamBase::In, NULL, description.data()));
			it->setMeta(new ito::StringMeta(ito::StringMeta::String, NULL), true);
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

//--------------------------------------------------------------------------------------------------------
bool GenTLDevice::isDeviceParam(const ParamMapIterator &it) const
{
	return m_paramMapping.contains(it->getName());
}

//--------------------------------------------------------------------------------------------------------
ito::RetVal GenTLDevice::setDeviceParam(QSharedPointer<ito::ParamBase> newVal, ParamMapIterator it)
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
		//autoUpdateDependentNodes();
		return retval;
	}
}

//--------------------------------------------------------------------------------------------------------
bool GenTLDevice::autoUpdateDependentNodes() //check all nodes for a change in their access mode and return true if any node including the corresponding ito::Param has been updated, else false
{
	bool changed = false;
	GCType *type;
	QHash<QString, GCType*>::iterator it = m_paramMapping.begin();
	int newFlag = 0;

	while (it != m_paramMapping.end())
	{
		type = it.value();
		newFlag = flagsFromAccessMode(type->node()->GetAccessMode());

		if (type->param().getFlags() != newFlag)
		{
			type->param().setFlags(newFlag);
			changed = true;
		}


		++it;
	}
	
	return changed;
}

//--------------------------------------------------------------------------------------------------------
int GenTLDevice::flagsFromAccessMode(const GenApi::EAccessMode &accessMode) const
{
	int flag = 0;
	switch (accessMode)
	{
	case EAccessMode::NA:
		flag = ito::ParamBase::Readonly | ito::ParamBase::NotAvailable;
		break;
	case EAccessMode::RO:
		flag = ito::ParamBase::Readonly;
		break;
	case EAccessMode::RW:
		flag = 0;
		break;
	case EAccessMode::WO:
		flag = 0; //todo: good?
		break;
	default:
		flag = 0;
	}

	return flag;
}

//--------------------------------------------------------------------------------------------------------
ito::RetVal GenTLDevice::invokeCommandNode(const gcstring &name, ito::tRetValue errorLevel /*=ito::retError*/)
{
	if (m_commandNodes.contains(name))
	{
		try
		{
			m_commandNodes[name]->Execute();
		}
		catch (GenericException &ex)
		{
			if (errorLevel == ito::retError)
			{
				return ito::RetVal::format(ito::retError, 0, "Error invoking command '%s': %s", name.c_str(), ex.GetDescription());
			}
			else if (errorLevel == ito::retWarning)
			{
				return ito::RetVal::format(ito::retWarning, 0, "Warning invoking command '%s': %s", name.c_str(), ex.GetDescription());
			}
		}
		return ito::retOk;
	}
	else
	{
		return ito::RetVal::format(ito::retError, 0, "command '%s' not available", name.c_str());
	}
}
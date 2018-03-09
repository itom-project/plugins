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



//----------------------------------------------------------------------------------------------------------------------------------
GenTLDevice::GenTLDevice(QSharedPointer<QLibrary> lib, GenTL::DEV_HANDLE devHandle, QByteArray deviceID, const QByteArray &identifier, int verbose, ito::RetVal &retval) :
    BasePort(lib, BasePort::TypeCamera, verbose, retval),
    m_cameraHandle(devHandle),
    m_deviceID(deviceID),
	m_identifier(identifier),
	
    m_errorEvent(GENTL_INVALID_HANDLE)
{
    if (!retval.containsError())
    {
        retval += checkGCError(DevGetPort(m_cameraHandle, &m_portHandle));

        ito::uint32 numURLS;
        GCGetNumPortURLs(m_portHandle, &numURLS);

        if (m_verbose >= VERBOSE_INFO)
        {
            std::cout << "Available loations for XML parameter description\n----------------------------------------\n";

            for (ito::uint32 i = 0; i < numURLS; ++i)
            {
                retval += printPortInfo(i);
            }

            std::cout << "\n" << std::endl;
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
GenTLDevice::~GenTLDevice()
{
	
}



//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer<GenTLFramegrabber> GenTLDevice::getFramegrabber(ito::RetVal &retval)
{
    //only for CoaxPress and CameraLink
    QSharedPointer<GenTLFramegrabber> framegrabber(new GenTLFramegrabber(m_lib, m_cameraHandle, m_verbose, retval));
    return framegrabber;
}

//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer<GenTLDataStream> GenTLDevice::getDataStream(ito::int32 streamIndex, ito::RetVal &retval)
{
    QSharedPointer<GenTLDataStream> stream;
    if (!m_cameraHandle)
    {
        retval += ito::RetVal(ito::retError, 0, "invalid device handle");
    }
    else
    {
        //get available data streams
        ito::uint32 numDataStreams;
        retval += checkGCError(DevGetNumDataStreams(m_cameraHandle, &numDataStreams), "Get number of data streams");

        char dataStreamID[512];
        size_t pSize;
        QByteArray streamIdToOpen = "";

        

        if (!retval.containsError())
        {
            if (m_verbose >= VERBOSE_INFO)
            {
                std::cout << "Stream information of device\n----------------------------------------\n" << std::endl;
            }

            for (ito::uint32 i = 0; i < numDataStreams; ++i)
            {
                pSize = sizeof(dataStreamID);
                if (DevGetDataStreamID(m_cameraHandle, i, dataStreamID, &pSize) == GenTL::GC_ERR_SUCCESS)
                {
                    if (streamIndex == i)
                    {
                        streamIdToOpen = QByteArray(dataStreamID);
                    }

                    if (m_verbose >= VERBOSE_INFO)
                    {
                        std::cout << i << ". Name: " << dataStreamID << "\n" << std::endl;
                    }
                }
            }

            if (m_verbose >= VERBOSE_INFO)
            {
                std::cout << "----------------------------------------\n" << std::endl;
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
                if (m_verbose >= VERBOSE_INFO)
                {
                    std::cout << "Trying to open stream '" << streamIdToOpen.constData() << "'...";
                }

                GenTL::DS_HANDLE streamHandle;
                retval += checkGCError(DevOpenDataStream(m_cameraHandle, streamIdToOpen.constData(), &streamHandle), "open data stream");

                if (!retval.containsError())
                {
                    if (m_verbose >= VERBOSE_INFO)
                    {
                        std::cout << "OK\n" << std::endl;
                    }

                    stream = QSharedPointer<GenTLDataStream>(new GenTLDataStream(m_lib, streamHandle, m_verbose, retval));
                    if (retval.containsError())
                    {
                        stream.clear();
                    }
                }
                else if (m_verbose >= VERBOSE_INFO)
                {
                    std::cout << "Error: " << retval.errorMessage() << "\n" << std::endl;
                }
            }
        }
    }

    return stream;
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
	//int heightMax, widthMax;
	int sensorWidth, sensorHeight;

    /*
	//WidthMax
	pInt = m_device._GetNode("WidthMax");
	if (pInt.IsValid())
	{
		widthMax = pInt->GetValue();
	}

	//HeightMax
	pInt = m_device._GetNode("HeightMax");
	if (pInt.IsValid())
	{
		heightMax = pInt->GetValue();
	}
    */

	//SensorWidth
	it = params.find("sizex");
	intMeta = it->getMetaT<ito::IntMeta>();
	pInt = m_device._GetNode("SensorWidth");
	if (pInt.IsValid() && (pInt->GetAccessMode() & (RO | RW)))
	{
		intMeta->setMin(0);
		intMeta->setMax(pInt->GetValue());
		sensorWidth = pInt->GetValue();
		params["sizex"].setFlags(ito::ParamBase::Readonly);
	}

	ito::IntMeta widthMeta(1, intMeta->getMax(), 1);
	int width = 0;
	pInt = m_device._GetNode("Width");
	if (pInt.IsValid() && (pInt->GetAccessMode() & (RO | RW)))
	{
		intMetaFromInteger(pInt, intMeta);
		intMetaFromInteger(pInt, &widthMeta);
		it->setVal<ito::int32>(pInt->GetValue());
		width = pInt->GetValue();
	}
	
	//SensorHeight
	it = params.find("sizey");
	intMeta = it->getMetaT<ito::IntMeta>();
	pInt = m_device._GetNode("SensorHeight");
	if (pInt.IsValid() && (pInt->GetAccessMode() & (RO | RW)))
	{
		
		intMeta->setMin(0);
		intMeta->setMax(pInt->GetValue());
		sensorHeight = pInt->GetValue();
        params["sizey"].setFlags(ito::ParamBase::Readonly);
	}

	ito::IntMeta heightMeta(1, intMeta->getMax(), 1);
	int height = 0;
	pInt = m_device._GetNode("Height");
	if (pInt.IsValid() && (pInt->GetAccessMode() & (RO | RW)))
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
	pInt = m_device._GetNode("OffsetX");
	if (pInt.IsValid() && (pInt->GetAccessMode() & (RO | RW)))
	{
		offsetX = pInt->GetValue();
		intMetaFromInteger(pInt, &offsetXMeta);
	}
	pInt = m_device._GetNode("OffsetY");
	if (pInt.IsValid() && (pInt->GetAccessMode() & (RO | RW)))
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
	pEnum = m_device._GetNode("PixelFormat");
	intMeta = it->getMetaT<ito::IntMeta>();

	if (pEnum.IsValid() == false || !(pInt->GetAccessMode() & (RO | RW)))
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
	CIntegerPtr pInt = m_device._GetNode("PayloadSize");
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
		m_supportedFormats << Mono8 << Mono10 << Mono10Packed << Mono10p << Mono12 << Mono12Packed << Mono12p << Mono14 << Mono16;
		m_supportedFormatsBpp << 8 << 10 << 10 << 10 << 12 << 12 << 12 << 14 << 16;

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
/*virtual*/ void GenTLDevice::callbackParameterChanged_(INode *pNode)
{
	CValuePtr ptrValue = pNode;
    
    if (m_verbose >= VERBOSE_ALL)
    {
		try
		{
			std::cout << "The node '" << pNode->GetName() << "' changed (" << pNode->GetAccessMode() << ")\n";
			qDebug() << "The node '" << pNode->GetName() << "' changed (" << pNode->GetAccessMode() << ")";

			if (ptrValue.IsValid())
			{
				if (pNode->GetAccessMode() & (RO | RW))
				{
					std::cout << "New value = " << ptrValue->ToString() << ", access: " << ptrValue->GetAccessMode() << "\n" << std::endl;
				}
				else
				{
					std::cout << "New value not readable. No read access. Access: " << ptrValue->GetAccessMode() << "\n" << std::endl;
				}
			}
		}
		catch (GenericException ex)
		{
			qDebug() << ex.GetDescription() << ex.what();
		}
    }

	if (m_callbackParameterChangedTimer.isNull())
	{
		m_callbackParameterChangedTimer = QSharedPointer<QTimer>(new QTimer());
		m_callbackParameterChangedTimer->setSingleShot(true);
		m_callbackParameterChangedTimer->setInterval(50);
		m_callbackParameterChangedTimer->stop();
		QObject::connect(m_callbackParameterChangedTimer.data(), SIGNAL(timeout()), m_pCallbackParameterChangedReceiver, SLOT(parameterChangedTimerFired()));
	}

	if (pNode && m_paramMapping2.contains(pNode))
	{
		m_paramMapping2[pNode]->update(false);
        m_callbackParameterChangedTimer->start();
	}
}

//--------------------------------------------------------------------------------------------------------
void GenTLDevice::resyncAllParameters()
{
    QHash<INode*, GCType*>::const_iterator i = m_paramMapping2.constBegin();
    while (i != m_paramMapping2.constEnd()) 
    {
        try
        {
            i.value()->update(false);
        }
        catch (GenericException& ex)
        {
            std::cout << "Error updating parameter '" << i.key()->GetName() << "': " << ex.GetDescription() << "\n" << std::endl;
        }
        ++i;
    }

    if (m_pCallbackParameterChangedReceiver)
    {
        QMetaObject::invokeMethod(m_pCallbackParameterChangedReceiver, "parameterChangedTimerFired");
    }
}



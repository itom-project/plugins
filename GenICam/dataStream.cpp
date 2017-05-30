/* ********************************************************************
    Plugin "GenICam" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2016, Institut für Technische Optik (ITO),
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

#include "dataStream.h"

#include "gccommon.h"

#include <qfileinfo.h>
#include <qdebug.h>
#include <qset.h>
#include <qregexp.h>
#include "common/sharedStructures.h"
#include <iostream>
#define PFNC_INCLUDE_HELPERS
#include "PFNC.h"

//----------------------------------------------------------------------------------------------------------------------------------
GenTLDataStream::GenTLDataStream(QSharedPointer<QLibrary> lib, GenTL::DS_HANDLE handle, ito::RetVal &retval) :
    m_handle(handle),
    m_lib(lib),
    m_newBufferEvent(GENTL_INVALID_HANDLE),
    m_errorEvent(GENTL_INVALID_HANDLE),
    m_acquisitionStarted(false),
	m_payloadSize(0),
	m_timeoutMS(0)
{
    GCRegisterEvent = (GenTL::PGCRegisterEvent)m_lib->resolve("GCRegisterEvent");
    GCUnregisterEvent = (GenTL::PGCUnregisterEvent)m_lib->resolve("GCUnregisterEvent");
    DSClose = (GenTL::PDSClose)m_lib->resolve("DSClose");
    EventGetData = (GenTL::PEventGetData)m_lib->resolve("EventGetData");
    EventGetDataInfo = (GenTL::PEventGetDataInfo)m_lib->resolve("EventGetDataInfo");
    DSQueueBuffer = (GenTL::PDSQueueBuffer)m_lib->resolve("DSQueueBuffer");
	DSRevokeBuffer = (GenTL::PDSRevokeBuffer)m_lib->resolve("DSRevokeBuffer");
    DSFlushQueue = (GenTL::PDSFlushQueue)m_lib->resolve("DSFlushQueue");
    DSGetBufferInfo = (GenTL::PDSGetBufferInfo)m_lib->resolve("DSGetBufferInfo");
    DSStartAcquisition = (GenTL::PDSStartAcquisition) m_lib->resolve("DSStartAcquisition");
    DSStopAcquisition = (GenTL::PDSStopAcquisition) m_lib->resolve("DSStopAcquisition");
    DSGetInfo = (GenTL::PDSGetInfo)m_lib->resolve("DSGetInfo");
	DSAllocAndAnnounceBuffer = (GenTL::PDSAllocAndAnnounceBuffer)m_lib->resolve("DSAllocAndAnnounceBuffer");

    if (!GCRegisterEvent || !GCUnregisterEvent || !DSClose || \
        !EventGetData || !DSQueueBuffer || !DSFlushQueue || !DSGetBufferInfo || \
		!DSStopAcquisition || !DSStartAcquisition || !DSGetInfo || !DSAllocAndAnnounceBuffer || \
		!DSRevokeBuffer)
    {
        retval += ito::RetVal(ito::retError, 0, QObject::tr("cti file does not export all functions of the GenTL protocol.").toLatin1().data());
    }

    if (!retval.containsError())
    {
        retval += checkGCError(GCRegisterEvent(m_handle, GenTL::EVENT_NEW_BUFFER, &m_newBufferEvent));
        retval += checkGCError(GCRegisterEvent(m_handle, GenTL::EVENT_ERROR, &m_errorEvent));
    }

    if (!retval.containsError())
    {
        //check how much memory the event need to have and allocate it
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
GenTLDataStream::~GenTLDataStream()
{
    if (GCUnregisterEvent && m_newBufferEvent != GENTL_INVALID_HANDLE)
    {
        GCUnregisterEvent(m_handle, GenTL::EVENT_NEW_BUFFER);
    }

    if (GCUnregisterEvent && m_errorEvent != GENTL_INVALID_HANDLE)
    {
        GCUnregisterEvent(m_handle, GenTL::EVENT_ERROR);
    }

    if (DSClose && m_handle != GENTL_INVALID_HANDLE)
    {
        if (m_acquisitionStarted)
        {
            stopAcquisition(GenTL::ACQ_STOP_FLAGS_KILL);
        }

        DSClose(m_handle);
        m_handle = GENTL_INVALID_HANDLE;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GenTLDataStream::allocateAndAnnounceBuffers(int nrOfBuffers, size_t bytesPerBuffer /*= 0*/)
{
    GenTL::BUFFER_HANDLE handle;
    ito::RetVal retval;
    GenTL::INFO_DATATYPE type;
    bool definesPayloadsize = false;
    size_t size;
    size_t payloadSize = 0;
    
    if (bytesPerBuffer == 0) //estimate payload
    {
        size = sizeof(definesPayloadsize);
        if (DSGetInfo(m_handle, GenTL::STREAM_INFO_DEFINES_PAYLOADSIZE, &type, &definesPayloadsize, &size) == GenTL::GC_ERR_SUCCESS)
        {
            if (type != GenTL::INFO_DATATYPE_BOOL8)
            {
                definesPayloadsize = false;
            }
        }

        if (definesPayloadsize)
        {
            size = sizeof(payloadSize);
            if (DSGetInfo(m_handle, GenTL::STREAM_INFO_PAYLOAD_SIZE, &type, &payloadSize, &size) == GenTL::GC_ERR_SUCCESS)
            {
                if (type != GenTL::INFO_DATATYPE_SIZET)
                {
                    payloadSize = 0;
                }
            }
        }

        if (payloadSize == 0)
        {
			payloadSize = m_payloadSize;
            //has to be obtained by node... TODO
        }

		if (payloadSize == 0)
		{
			retval += ito::RetVal(ito::retError, 0, "could not get a valid payload size.");
			return retval;
		}

        bytesPerBuffer = payloadSize;
    }

    for (int i = 0; i < nrOfBuffers; ++i)
    {
        retval += checkGCError(DSAllocAndAnnounceBuffer(m_handle, bytesPerBuffer, this, &handle));

        if (!retval.containsError())
        {
            m_idleBuffers.append(handle);
        }
        else
        {
            break;
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GenTLDataStream::revokeAllBuffers()
{
	ito::RetVal retval;
	foreach(GenTL::BUFFER_HANDLE handle, m_idleBuffers)
	{
		retval += checkGCError(DSRevokeBuffer(m_handle, handle, NULL, NULL), "revoke buffer");
	}
	m_idleBuffers.clear();
	return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GenTLDataStream::queueOneBufferForAcquisition()
{
    if (m_queuedBuffers.size() > 0) //there are still queued and unhandled items
    {
        DSFlushQueue(m_handle, GenTL::ACQ_QUEUE_ALL_DISCARD);

        QSet<GenTL::BUFFER_HANDLE>::iterator it = m_queuedBuffers.begin();
        while ( it != m_queuedBuffers.end())
        {
            m_idleBuffers.enqueue(*it);
			it++;
        }
        m_queuedBuffers.clear();
    }

	if (m_idleBuffers.size() == 0)
	{
		return ito::RetVal(ito::retError, 0, "no free buffers available to queue for new acquisition.");
	}

    GenTL::BUFFER_HANDLE buffer = m_idleBuffers.dequeue();
    ito::RetVal retval = checkGCError(DSQueueBuffer(m_handle, buffer));
    if (!retval.containsError())
    {
        m_queuedBuffers.insert(buffer);
    }
    else //error: redo all
    {
        m_idleBuffers.enqueue(buffer);
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GenTLDataStream::unlockBuffer(GenTL::BUFFER_HANDLE buffer)
{
    if (m_lockedBuffers.contains(buffer))
    {
		DSFlushQueue(m_handle, GenTL::ACQ_QUEUE_ALL_DISCARD);

        m_lockedBuffers.remove(buffer);
        m_idleBuffers.enqueue(buffer);
    }
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GenTLDataStream::checkForNewBuffer(GenTL::BUFFER_HANDLE &buffer)
{
    GenTL::S_EVENT_NEW_BUFFER newBuffer;
    size_t pSize = sizeof(newBuffer);

	GenTL::GC_ERROR err = EventGetData(m_newBufferEvent, &newBuffer, &pSize, m_timeoutMS);

    if (err == GenTL::GC_ERR_TIMEOUT)
    {
        buffer = GENTL_INVALID_HANDLE;

        //check if a possible error has been signaled
        if (m_errorEvent != GENTL_INVALID_HANDLE)
        {
            char bufferIn[1024];
            size_t sizeIn = sizeof(bufferIn);
            err = EventGetData(m_errorEvent, &bufferIn, &sizeIn, 0);
            if (err == GenTL::GC_ERR_SUCCESS)
            {
                if (EventGetDataInfo)
                {
                    GenTL::INFO_DATATYPE piType;
                    char bufferOut[512];
                    size_t pSizeOut = sizeof(bufferOut);

                    GenTL::DEVICE_INFO_CMD cmds[] = { GenTL::URL_INFO_URL };
                    GenTL::GC_ERROR err2 = EventGetDataInfo(m_errorEvent, &bufferIn, sizeIn, GenTL::EVENT_DATA_VALUE, &piType, &bufferOut, &pSizeOut);

                    if (err2 == GenTL::GC_ERR_SUCCESS)
                    {
                        if (piType == GenTL::INFO_DATATYPE_STRING)
                        {
                            bufferOut[511] = '\0';
                            return ito::RetVal::format(ito::retError, 0, "Timeout occurred. Signaled error: %s.", bufferOut);
                        }
                        else
                        {
                            return ito::RetVal(ito::retError, 0, "Timeout occurred. An error has been signaled, however no more information could be fetched due to unsupported datatype returned from method 'EventGetDataInfo'");
                        }
                    }
                    else
                    {
                        return ito::RetVal(ito::retError, 0, "Timeout occurred. An error has been signaled, however no more information could be fetched due to error returned from method 'EventGetDataInfo'");
                    }
                }
                else
                {
                    return ito::RetVal(ito::retError, 0, "Timeout occurred. An error has been signaled, however no more information could be fetched due to missing method 'EventGetDataInfo'");
                }
            }
        }

        return ito::RetVal(ito::retError, 0, "Timeout occurred.");
    }
    else if (checkGCError(err) == ito::retOk)
    {
        buffer = newBuffer.BufferHandle;
        m_queuedBuffers.remove(buffer);
        m_lockedBuffers.insert(buffer); //lock the buffer that is passed to the caller by reference until unlockBuffer has been called
        return ito::retOk;
    }

    return ito::retError;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GenTLDataStream::startAcquisition(GenTL::ACQ_START_FLAGS startFlags /*= GenTL::ACQ_START_FLAGS_DEFAULT*/)
{
    if (m_acquisitionStarted)
    {
        return ito::RetVal(ito::retWarning, 0, "acquisition already started");
    }

    ito::RetVal ret = checkGCError(DSStartAcquisition(m_handle, startFlags, GENTL_INFINITE), "start acquisition");
    if (!ret.containsError())
    {
        m_acquisitionStarted = true;
    }

    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GenTLDataStream::stopAcquisition(GenTL::ACQ_STOP_FLAGS stopFlags /*= GenTL::ACQ_STOP_FLAGS_DEFAULT*/)
{
    if (!m_acquisitionStarted)
    {
        return ito::RetVal(ito::retWarning, 0, "acquisition could not be stopped since it is currently not running.");
    }

    ito::RetVal ret = checkGCError(DSStopAcquisition(m_handle, stopFlags), "stop acquisition");
    if (!ret.containsError())
    {
        m_acquisitionStarted = false;
    }

    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------
bool GenTLDataStream::setPayloadSize(int payloadSize)
{
	if (m_payloadSize != payloadSize)
	{
		m_payloadSize = payloadSize;
		return true;
	}
	return false;
}

//----------------------------------------------------------------------------------------------------------------------------------
void GenTLDataStream::setTimeoutSec(double timeout)
{
	m_timeoutMS = timeout * 1000.0;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GenTLDataStream::copyBufferToDataObject(const GenTL::BUFFER_HANDLE buffer, ito::DataObject &dobj)
{
	ito::RetVal retval;
	GenTL::INFO_DATATYPE dtype;
	size_t pSize;
	size_t sizeFilled;
	size_t size;
	size_t temp;
	uint64_t temp64;
	void* ptr;
	GenTL::PIXELENDIANNESS_IDS endianess;
	GenTL::PAYLOADTYPE_INFO_IDS payloadType;
	GenTL::PIXELFORMAT_NAMESPACE_IDS pixelformatNamespace;
	uint64_t pixelformat;
	size_t height;
	size_t width;

	//request mandatory parameter: size of buffer
	pSize = sizeof(size);
	retval += checkGCError(DSGetBufferInfo(m_handle, buffer, GenTL::BUFFER_INFO_SIZE, &dtype, &size, &pSize), "request size of image buffer");
	if (!retval.containsError())
	{
		if (dtype != GenTL::INFO_DATATYPE_SIZET)
		{
			retval += ito::RetVal(ito::retError, 0, "request size of image buffer: returned value is not of expected type (size_t)");
		}
	}

	//request mandatory parameter: width and height of image
	pSize = sizeof(width);
	retval += checkGCError(DSGetBufferInfo(m_handle, buffer, GenTL::BUFFER_INFO_WIDTH, &dtype, &width, &pSize), "request width of image buffer");
	pSize = sizeof(height);
	retval += checkGCError(DSGetBufferInfo(m_handle, buffer, GenTL::BUFFER_INFO_HEIGHT, &dtype, &height, &pSize), "request height of image buffer");

	//request mandatory parameter: base pointer
	pSize = sizeof(ptr);
	retval += checkGCError(DSGetBufferInfo(m_handle, buffer, GenTL::BUFFER_INFO_BASE, &dtype, &ptr, &pSize), "request pointer of image buffer");
	if (!retval.containsError())
	{
		if (dtype != GenTL::INFO_DATATYPE_PTR)
		{
			retval += ito::RetVal(ito::retError, 0, "request pointer of image buffer: returned value is not of expected type (ptr)");
		}
	}

	if (!retval.containsError())
	{
		pSize = sizeof(sizeFilled);
		if (DSGetBufferInfo(m_handle, buffer, GenTL::BUFFER_INFO_SIZE_FILLED, &dtype, &sizeFilled, &pSize) == GenTL::GC_ERR_SUCCESS)
		{
			if (sizeFilled != size)
			{
				retval += ito::RetVal::format(ito::retError, 0, "returned image buffer is only partially filled.");
			}
		}
	}

	if (!retval.containsError())
	{
		//payload type -> only accept PAYLOAD_TYPE_IMAGE
		pSize = sizeof(temp);
		retval += checkGCError(DSGetBufferInfo(m_handle, buffer, GenTL::BUFFER_INFO_PAYLOADTYPE, &dtype, &temp, &pSize), "get payload type of image buffer");

		if (!retval.containsError())
		{
			payloadType = (GenTL::PAYLOADTYPE_INFO_IDS)temp;

			if (payloadType != GenTL::PAYLOAD_TYPE_IMAGE)
			{
				retval += ito::RetVal(ito::retError, 0, "currently only buffers that contain image data are supported");
			}
		}

		//pixelformat namespace
		pSize = sizeof(temp64);
		retval += checkGCError(DSGetBufferInfo(m_handle, buffer, GenTL::BUFFER_INFO_PIXELFORMAT_NAMESPACE, &dtype, &temp64, &pSize), "get pixelformat namespace of image buffer");

		if (!retval.containsError())
		{
			pixelformatNamespace = (GenTL::PIXELFORMAT_NAMESPACE_IDS)temp64;
		}

		//pixelformat
		pSize = sizeof(pixelformat);
		retval += checkGCError(DSGetBufferInfo(m_handle, buffer, GenTL::BUFFER_INFO_PIXELFORMAT, &dtype, &pixelformat, &pSize), "get pixelformat of image buffer");

		//get pixel endianess
		pSize = sizeof(temp);
		if (DSGetBufferInfo(m_handle, buffer, GenTL::BUFFER_INFO_PIXEL_ENDIANNESS, &dtype, &temp, &pSize) != GenTL::GC_ERR_SUCCESS)
		{
			endianess = GenTL::PIXELENDIANNESS_LITTLE; //default
		}
		else
		{
			endianess = (GenTL::PIXELENDIANNESS_IDS)temp;

			if (endianess == GenTL::PIXELENDIANNESS_UNKNOWN)
			{
				retval += ito::RetVal(ito::retWarning, 0, "endianess of image buffer is assumed to be little endian");
				endianess = GenTL::PIXELENDIANNESS_LITTLE; //default
			}
		}
	}

	if (!retval.containsError())
	{
		if ((pixelformatNamespace == GenTL::PIXELFORMAT_NAMESPACE_UNKNOWN) || (pixelformat == 0))
		{
			//try to guess right pixelformat
			if (dobj.getType() == ito::tUInt16 && size >= (sizeof(ito::uint16) * (width * height)))
			{
				pixelformat = PFNC_Mono16;
			}
			else if (dobj.getType() == ito::tUInt8 && size >= (sizeof(ito::uint8) * (width * height)))
			{
				pixelformat = PFNC_Mono8;
			}
		}

		switch (pixelformat)
		{
		case PFNC_Mono8:
			retval += copyMono8ToDataObject(ptr, width, height, endianess == GenTL::PIXELENDIANNESS_LITTLE, dobj);
			break;
		case PFNC_Mono10:
		case PFNC_Mono12:
		case PFNC_Mono14:
		case PFNC_Mono16:
			retval += copyMono10to16ToDataObject(ptr, width, height, endianess == endianess, dobj);
			break;
		case GVSP_Mono12Packed: //GigE specific
			retval += copyMono12pToDataObject(ptr, width, height, endianess == endianess, dobj);
		default:
			retval += ito::RetVal::format(ito::retError, 0, "Pixel format %i (%s) is not yet supported.", pixelformat, GetPixelFormatName((PfncFormat)pixelformat));
			break;
		}
	}

	return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GenTLDataStream::copyMono8ToDataObject(const void* ptr, const size_t &width, const size_t &height, bool littleEndian, ito::DataObject &dobj)
{
	//little or big endian is idle for mono8:
	return dobj.copyFromData2D<ito::uint8>((const ito::uint8*)ptr, width, height);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GenTLDataStream::copyMono10to16ToDataObject(const void* ptr, const size_t &width, const size_t &height, bool littleEndian, ito::DataObject &dobj)
{
	if (littleEndian)
	{
		return dobj.copyFromData2D<ito::uint16>((const ito::uint16*)ptr, width, height);
	}
	else
	{
		return ito::RetVal(ito::retError, 0, "big endian for mono10, mono12, mono14, mono16 currently not supported.");
	}
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GenTLDataStream::copyMono12pToDataObject(const void* ptr, const size_t &width, const size_t &height, bool littleEndian, ito::DataObject &dobj)
{
	return ito::RetVal(ito::retError, 0, "data converter for mono12p or mono12packed not implemented yet.");
}
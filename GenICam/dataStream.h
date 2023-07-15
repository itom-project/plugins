/* ********************************************************************
    Plugin "GenICam" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2022, Institut für Technische Optik (ITO),
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

#ifndef DATASTREAM_H
#define DATASTREAM_H

#include <qlibrary.h>
#include <qstring.h>
#include <qpointer.h>
#include <qsharedpointer.h>
#include <qqueue.h>
#include <qset.h>

#include "DataObject/dataobj.h"

#include "common/retVal.h"

#include "GenTL_v1_5.h"

////////////////////////////////////////////////////////////////////////////////////////////
/*
*/
class GenTLDataStream
{
public:
    GenTLDataStream(QSharedPointer<QLibrary> lib, GenTL::DS_HANDLE handle, int verbose, ito::RetVal &retval);
    ~GenTLDataStream();

    ito::RetVal allocateAndAnnounceBuffers(int nrOfBuffers, size_t bytesPerBuffer = 0); //bytesPerBuffer = 0 means that the payload for each buffer should be automatically detected
    ito::RetVal revokeAllBuffers();

    ito::RetVal flushBuffers(GenTL::ACQ_QUEUE_TYPE queueType = GenTL::ACQ_QUEUE_ALL_DISCARD);
    ito::RetVal waitForNewestBuffer(ito::DataObject &destination);
    ito::RetVal startAcquisition(GenTL::ACQ_START_FLAGS startFlags = GenTL::ACQ_START_FLAGS_DEFAULT);
    ito::RetVal stopAcquisition(GenTL::ACQ_STOP_FLAGS stopFlags = GenTL::ACQ_STOP_FLAGS_DEFAULT);

    bool setPayloadSize(int payloadSize); //use this method to signal payload size from GenApi XML., returns true if payloadSize is different than before
    void setTimeoutSec(double timeout);

    QByteArray getInfoString(GenTL::STREAM_INFO_CMD cmd, int maxSize, const QByteArray &defaultValue, GenTL::GC_ERROR *returnCode = NULL) const;

    QByteArray getTLType(ito::RetVal *retval = NULL) const;

    ito::RetVal copyBufferToDataObject(const GenTL::BUFFER_HANDLE buffer, ito::DataObject &dobj);

    bool flushAllBuffersToInput() const {
        return m_flushAllBuffersToInput;
    }

    void setFlushAllBuffersToInput(bool enable) {
        m_flushAllBuffersToInput = enable;
    }

protected:
    ito::RetVal copyMono8ToDataObject(const char* ptr, const size_t &width, const size_t &height, bool littleEndian, ito::DataObject &dobj);
    ito::RetVal copyRGB8ToDataObject(const char* ptr, const size_t& width, const size_t& height, bool littleEndian, ito::DataObject& dobj);
    ito::RetVal copyBGR8ToDataObject(const char* ptr, const size_t& width, const size_t& height, bool littleEndian, ito::DataObject& dobj);
    ito::RetVal copyYCbCr422ToDataObject(const char* ptr, const size_t &width, const size_t &height, bool littleEndian, ito::DataObject &dobj);
    ito::RetVal copyBayerRG8ToDataObject(const char* ptr, const size_t& width, const size_t& height, bool littleEndian, ito::DataObject& dobj);
    ito::RetVal copyMono10to16ToDataObject(const char* ptr, const size_t &width, const size_t &height, bool littleEndian, ito::DataObject &dobj);
    ito::RetVal copyMono12pToDataObject(const char* ptr, const size_t &width, const size_t &height, bool littleEndian, ito::DataObject &dobj);
    ito::RetVal copyMono12PackedToDataObject(const char* ptr, const size_t &width, const size_t &height, bool littleEndian, ito::DataObject &dobj);
    ito::RetVal copyMono10pToDataObject(const char* ptr, const size_t &width, const size_t &height, bool littleEndian, ito::DataObject &dobj);
    ito::RetVal copyMono10PackedToDataObject(const char* ptr, const size_t &width, const size_t &height, bool littleEndian, ito::DataObject &dobj);

    bool checkForErrorEvent(ito::RetVal &retval, const QString &errorPrefix); //return true, if error event is available and has been reported, else false

    void printBufferInfo(const char *prefix, GenTL::BUFFER_HANDLE buffer);

    GenTL::DS_HANDLE m_handle;
    GenTL::EVENT_HANDLE m_newBufferEvent;
    GenTL::EVENT_HANDLE m_errorEvent;

    GenTL::PDSAllocAndAnnounceBuffer DSAllocAndAnnounceBuffer;
    GenTL::PDSRevokeBuffer DSRevokeBuffer;
    GenTL::PGCRegisterEvent GCRegisterEvent;
    GenTL::PGCUnregisterEvent GCUnregisterEvent;
    GenTL::PEventGetData EventGetData;
    GenTL::PEventGetDataInfo EventGetDataInfo;
    GenTL::PDSQueueBuffer DSQueueBuffer;
    GenTL::PDSFlushQueue DSFlushQueue;
    GenTL::PDSGetBufferInfo DSGetBufferInfo;
    GenTL::PDSClose DSClose;
    GenTL::PDSStartAcquisition DSStartAcquisition;
    GenTL::PDSStopAcquisition DSStopAcquisition;
    GenTL::PDSGetInfo DSGetInfo;
    GenTL::PDSAnnounceBuffer DSAnnounceBuffer;

    QSet<GenTL::BUFFER_HANDLE> m_buffers; //all allocated buffers
    QSharedPointer<QLibrary> m_lib;
    bool m_acquisitionStarted;
    int m_payloadSize;
    uint64_t m_timeoutMS;
    ito::int8 m_usePreAllocatedBuffer; //0 if the image buffer is allocated by the camera, 1 if the buffer is allocated by the itom-plugin and has to be deleted after revoking the buffer, -1 if not decided yet
    bool m_endianessChanged;
    int m_verbose;
    bool m_flushAllBuffersToInput; //see init parameter with the same name

};

#endif // DATASTREAM_H

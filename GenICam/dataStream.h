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
    GenTLDataStream(QSharedPointer<QLibrary> lib, GenTL::DS_HANDLE handle, ito::RetVal &retval);
    ~GenTLDataStream();

    ito::RetVal allocateAndAnnounceBuffers(int nrOfBuffers, size_t bytesPerBuffer = 0); //bytesPerBuffer = 0 means that the payload for each buffer should be automatically detected
	ito::RetVal revokeAllBuffers();

    ito::RetVal queueOneBufferForAcquisition();
    ito::RetVal unlockBuffer(GenTL::BUFFER_HANDLE buffer);
    ito::RetVal checkForNewBuffer(GenTL::BUFFER_HANDLE &buffer);
    ito::RetVal startAcquisition(GenTL::ACQ_START_FLAGS startFlags = GenTL::ACQ_START_FLAGS_DEFAULT);
    ito::RetVal stopAcquisition(GenTL::ACQ_STOP_FLAGS stopFlags = GenTL::ACQ_STOP_FLAGS_DEFAULT);

	bool setPayloadSize(int payloadSize); //use this method to signal payload size from GenApi XML., returns true if payloadSize is different than before
	void setTimeoutSec(double timeout);

	ito::RetVal copyBufferToDataObject(const GenTL::BUFFER_HANDLE buffer, ito::DataObject &dobj);

protected:
	ito::RetVal copyMono8ToDataObject(const void* ptr, const size_t &width, const size_t &height, bool littleEndian, ito::DataObject &dobj);
	ito::RetVal copyMono10to16ToDataObject(const void* ptr, const size_t &width, const size_t &height, bool littleEndian, ito::DataObject &dobj);
	ito::RetVal copyMono12pToDataObject(const void* ptr, const size_t &width, const size_t &height, bool littleEndian, ito::DataObject &dobj);

    GenTL::DS_HANDLE m_handle;
    GenTL::EVENT_HANDLE m_newBufferEvent;

    GenTL::PDSAllocAndAnnounceBuffer DSAllocAndAnnounceBuffer;
	GenTL::PDSRevokeBuffer DSRevokeBuffer;
    GenTL::PGCRegisterEvent GCRegisterEvent;
    GenTL::PGCUnregisterEvent GCUnregisterEvent;
    GenTL::PEventGetData EventGetData;
    GenTL::PDSQueueBuffer DSQueueBuffer;
    GenTL::PDSFlushQueue DSFlushQueue;
    GenTL::PDSGetBufferInfo DSGetBufferInfo;
    GenTL::PDSClose DSClose;
    GenTL::PDSStartAcquisition DSStartAcquisition;
    GenTL::PDSStopAcquisition DSStopAcquisition;
    GenTL::PDSGetInfo DSGetInfo;

    QQueue<GenTL::BUFFER_HANDLE> m_idleBuffers; //buffers that can be queued for acquisition and are not locked (in announce position)
    QSet<GenTL::BUFFER_HANDLE> m_lockedBuffers; //in output buffer
    QSet<GenTL::BUFFER_HANDLE> m_queuedBuffers; //in input buffer
    QSharedPointer<QLibrary> m_lib;
    bool m_acquisitionStarted;
	int m_payloadSize;
	uint64_t m_timeoutMS;
    
};

#endif // DATASTREAM_H
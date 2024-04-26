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
            std::cout << "Available locations for XML parameter description\n----------------------------------------\n";

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
/*virtual*/ void GenTLDevice::callbackParameterChanged_(INode *pNode)
{
    CValuePtr ptrValue = pNode;

    if (m_verbose >= VERBOSE_DEBUG)
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

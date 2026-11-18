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

#ifndef DEVICE_H
#define DEVICE_H

#define NOMINMAX

#include <qstring.h>
#include <qpointer.h>
#include <qsharedpointer.h>
#include <qqueue.h>
#include <qset.h>
#include <qvector.h>
#include <qmap.h>
#include <qtimer.h>
#include <qhash.h>

#include "common/param.h"
#include "common/retVal.h"
#include "dataStream.h"
#include "framegrabber.h"
#include "basePort.h"

#include "GenApi/GenApi.h"

#include "GenTL_v1_5.h"



using namespace GENAPI_NAMESPACE;

////////////////////////////////////////////////////////////////////////////////////////////
/*
*/
class GenTLDevice : public BasePort
{
public:
    GenTLDevice(
        QSharedPointer<QLibrary> lib,
        GenTL::DEV_HANDLE devHandle,
        const QByteArray &deviceID,
        const QByteArray &modelName,
        const QByteArray &identifier,
        const QByteArray &serialNumber,
        int verbose,
        ito::RetVal &retval);

    ~GenTLDevice();

    QByteArray getDeviceID() const { return m_deviceID; }

    //!< returns the serial number of this device or an empty string if not available
    QByteArray getSerialNumber() const { return m_serialNumber; }

    QSharedPointer<GenTLDataStream> getDataStream(ito::int32 streamIndex, ito::RetVal &retval);

    int getPayloadSize() const;

    QByteArray getIdentifier() const { return m_identifier; }

    void resyncAllParameters();

    virtual void callbackParameterChanged_(INode *pNode); //this is the member, called from the static version callbackParameterChanged (this is necessary if more than one GenICam device is connected to the computer)

    QSharedPointer<GenTLFramegrabber> getFramegrabber(ito::RetVal &retval);

protected:
    QSharedPointer<QTimer> m_callbackParameterChangedTimer;
    GenTL::DEV_HANDLE m_cameraHandle;
    QByteArray m_deviceID;
    QByteArray m_serialNumber;
    QByteArray m_identifier;
    QByteArray m_modelName;
    GenTL::EVENT_HANDLE m_errorEvent;
};

#endif // DEVICE_H

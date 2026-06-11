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

#ifndef DEVICECONTAINER_H
#define DEVICECONTAINER_H

#define NOMINMAX

#include <qlibrary.h>
#include <qstring.h>
#include <qpointer.h>
#include <qsharedpointer.h>
#include <qqueue.h>
#include <qset.h>

#include "device.h"

#include "common/retVal.h"

#include "GenTL_v1_5.h"

////////////////////////////////////////////////////////////////////////////////////////////
/*
*/
class GenTLInterface
{

public:
    GenTLInterface(QSharedPointer<QLibrary> lib, GenTL::IF_HANDLE ifHandle, QByteArray ifID, int verbose, ito::RetVal &retval);
    ~GenTLInterface();

    GenTL::IF_HANDLE getHandle() const { return m_handle; }
    QByteArray getIfaceID() const { return m_IfaceID; }
    int getNumDevices() const;

    QSharedPointer<GenTLDevice> getDevice(const QByteArray &deviceID, GenTL::DEVICE_ACCESS_FLAGS deviceAccess, ito::RetVal &retval);

    static const QByteArray SerialNumberPrefix;

protected:
    ito::RetVal printDeviceInfo(const char* sDeviceID) const;

    GenTL::IF_HANDLE m_handle;
    QByteArray m_IfaceID;

    GenTL::PIFOpenDevice IFOpenDevice;
    GenTL::PIFClose IFClose;
    GenTL::PIFGetDeviceInfo IFGetDeviceInfo;
    GenTL::PIFGetDeviceID IFGetDeviceID;
    GenTL::PIFUpdateDeviceList IFUpdateDeviceList;
    GenTL::PIFGetNumDevices IFGetNumDevices;

    QList<QWeakPointer<GenTLDevice> > m_devices;

    int m_verbose;

private:
    QSharedPointer<QLibrary> m_lib;
};

////////////////////////////////////////////////////////////////////////////////////////////
/*
*/
class GenTLSystem
{

public:
    GenTLSystem();
    ~GenTLSystem();

    ito::RetVal init(const QString &filename);
    ito::RetVal openSystem();

    QString getCtiFile() const { return m_ctiFile; }
    QByteArray getStringInfo(GenTL::TL_INFO_CMD_LIST cmd, ito::RetVal &retval) const;
    GenTL::TL_HANDLE getHandle() const { return m_handle; }

    QSharedPointer<GenTLInterface> getInterface(const QByteArray &interfaceID, ito::RetVal &retval);

    void setVerbose(int verbose) { m_verbose = verbose; }

    QByteArray getInterfaceInfo(GenTL::INTERFACE_INFO_CMD_LIST cmd, const char *sIfaceID, ito::RetVal &retval) const;

protected:

    GenTL::PGCInitLib GCInitLib;
    GenTL::PGCCloseLib GCCloseLib;
    GenTL::PGCGetInfo GCGetInfo;
    //GenTL::PGCGetPortURL GCGetPortURL;
    GenTL::PTLOpen TLOpen;
    GenTL::PTLClose TLClose;
    GenTL::PTLUpdateInterfaceList TLUpdateInterfaceList;
    GenTL::PTLGetNumInterfaces TLGetNumInterfaces;
    GenTL::PTLGetInterfaceID TLGetInterfaceID;
    GenTL::PTLGetInterfaceInfo TLGetInterfaceInfo;
    GenTL::PTLOpenInterface TLOpenInterface;
    GenTL::PGCGetLastError GCGetLastError;

    QString m_ctiFile;
    bool m_initialized;
    bool m_systemInit;
    bool m_systemOpened;
    int m_verbose;

    QList<QWeakPointer<GenTLInterface> > m_interfaces;

    GenTL::TL_HANDLE m_handle;

private:
    QSharedPointer<QLibrary> m_lib;
};

////////////////////////////////////////////////////////////////////////////////////////////
/*
*/
class GenTLOrganizer
{

    public:
        static GenTLOrganizer* instance();

        QSharedPointer<GenTLSystem> getSystem(const QString &filename, ito::RetVal &retval);

    protected:

        QList<QWeakPointer<GenTLSystem> > m_systems;

    private:
        GenTLOrganizer(void);
        GenTLOrganizer(GenTLOrganizer  &/*copyConstr*/) {}
        ~GenTLOrganizer(void);

        static GenTLOrganizer *m_pOrganizer;

        //!< singleton nach: http://www.oop-trainer.de/Themen/Singleton.html
        class GenTLOrganizerSingleton
        {
            public:
                ~GenTLOrganizerSingleton()
                {
                    #pragma omp critical
                    {
                        if( GenTLOrganizer::m_pOrganizer != NULL)
                        {
                            delete GenTLOrganizer::m_pOrganizer;
                            GenTLOrganizer::m_pOrganizer = NULL;
                        }
                    }
                }
        };
        friend class GenTLOrganizerSingleton;
};

#endif // DEVICECONTAINER_H

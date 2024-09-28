/* ********************************************************************
    Plugin "ItomUSBDevice" for itom software
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

#ifndef ITOMUSBDEVICE_H
#define ITOMUSBDEVICE_H

#include "common/addInInterface.h"
#include "DataObject/dataobj.h"
#include "libusb.h" //from the libusb.info project!

#include <qsharedpointer.h>
#include <qbytearray.h>

struct USBDevice
{
    USBDevice() : vendorID(0), productID(0), busNr(0), deviceAddr(0) {};
    USBDevice(uint16_t vendor_id, uint16_t product_id, uint8_t bus_nr, uint8_t device_addr) :
        vendorID(vendor_id), productID(product_id), busNr(bus_nr), deviceAddr(device_addr) {}
    bool operator == (const USBDevice& dev) const
    {
        return (vendorID == dev.vendorID) &&
               (productID == dev.productID) &&
               (busNr == dev.busNr) &&
               (deviceAddr == dev.deviceAddr);
    }
    uint16_t vendorID;
    uint16_t productID;
    uint8_t busNr;
    uint8_t deviceAddr;
};

//----------------------------------------------------------------------------------------------------------------------------------
class ItomUSBDevice : public ito::AddInDataIO //, public DummyGrabberInterface
{
    Q_OBJECT

    protected:
        virtual ~ItomUSBDevice();
        ItomUSBDevice();

    public:
        friend class ItomUSBDeviceInterface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 0; } //!< indicates that this plugin has got a configuration dialog

    private:

        libusb_device_handle *m_pDevice;
        bool m_autoDetach;

        bool m_debugMode;   /*! Enables / Disables live connection to dockingwidge-protocol */
        static int m_instCounter;
        int m_timeoutMS;
        int m_endpoint_read;
        int m_endpoint_write;
        USBDevice m_currentDevice;

        static QVector<USBDevice> openedDevices;
        static QMutex openedDevicesReadWriteMutex;

    signals:
        void serialLog(QByteArray data, const char InOutChar);
        void uniqueIDChanged(const int);
        //void parametersChanged(QMap<QString, ito::tParam>); (defined in AddInBase)

    public slots:
/*
        ito::RetVal getParam(const char *name, QSharedPointer<double> val, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal getParam(const char *name, QSharedPointer<char> val, QSharedPointer<int> len, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal setParam(const char *name, const char *val, const int len, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal setParam(const char *name, const double val, ItomSharedSemaphore *waitCond = NULL);
*/
        ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond = NULL);

        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal close(ItomSharedSemaphore *waitConde = NULL);

        ito::RetVal startDevice(ItomSharedSemaphore *waitCond);
        ito::RetVal stopDevice(ItomSharedSemaphore *waitCond);
        ito::RetVal acquire(const int trigger, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal getVal(QSharedPointer<char> data, QSharedPointer<int> length, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal setVal(const char *data, const int length, ItomSharedSemaphore *waitCond);

        ito::RetVal execFunc(const QString funcName, QSharedPointer<QVector<ito::ParamBase> > paramsMand, QSharedPointer<QVector<ito::ParamBase> > paramsOpt, QSharedPointer<QVector<ito::ParamBase> > paramsOut, ItomSharedSemaphore *waitCond = NULL);

    private slots:
        void dockWidgetVisibilityChanged(bool visible);
};

//----------------------------------------------------------------------------------------------------------------------------------
class ItomUSBDeviceInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        ItomUSBDeviceInterface();
        ~ItomUSBDeviceInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // ItomUSBDevice_H

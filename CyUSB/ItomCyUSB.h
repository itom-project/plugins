/* ********************************************************************
    Plugin "ItomCyUSB" for itom software
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

#ifndef ItomCyUSB_H
#define ItomCyUSB_H

#include "common/addInInterface.h"
#include "DataObject/dataobj.h"
#include "ItomCyUSB.h"

#include <qsharedpointer.h>
#include <qbytearray.h>

#include <Windows.h>
#include "CyAPI.h" //http://www.cypress.com/file/135301?finished=1

//----------------------------------------------------------------------------------------------------------------------------------
class ItomCyUSB : public ito::AddInDataIO //, public
{
    Q_OBJECT

    protected:
        virtual ~ItomCyUSB();
        ItomCyUSB();

    public:
        friend class ItomCyUSBInterface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 0; } //!< indicates that this plugin has got a configuration dialog

    private:

        HANDLE cyHandle;
        CCyUSBDevice *m_cyDevices;
        CCyUSBEndPoint  **m_endPoints;

        enum endpointType {
            IsocIn = 0x0001,
            IsocOut = 0x0002,
            bulkIn = 0x0003,
            bulkOut = 0x0004,
            interruptIn = 0x0005,
            interruptOut = 0x0006,
            controlEndPoint = 0x0007
            };

        CCyIsocEndPoint *m_isocInEndPoint;
        CCyIsocEndPoint *m_isocOutEndPoint;
        CCyBulkEndPoint *m_bulkInEndPoint;
        CCyBulkEndPoint *m_bulkOutEndPoint;
        CCyInterruptEndPoint *m_interruptInEndPoint;
        CCyInterruptEndPoint *m_interruptOutEndPoint;
        CCyControlEndPoint *m_controlEndPoint;

        static QVector<CCyUSBDevice> openedDevices;
        static QMutex openedDevicesReadWriteMutex;

    signals:
        void serialLog(QByteArray data, const char InOutChar);
        //void uniqueIDChanged(const int);
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
class ItomCyUSBInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        ItomCyUSBInterface();
        ~ItomCyUSBInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // ItomCyUSB_H

/* ********************************************************************
    Plugin "HidApi" for itom software
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

#ifndef ITOMHIDAPI_H
#define ITOMHIDAPI_H

#include "common/addInInterface.h"
#include "DataObject/dataobj.h"
#include "hidapi.h"

#include <qsharedpointer.h>
#include <qbytearray.h>

//----------------------------------------------------------------------------------------------------------------------------------
class ItomHidApi : public ito::AddInDataIO
{
    Q_OBJECT

    protected:
        virtual ~ItomHidApi();
        ItomHidApi();

    public:
        friend class ItomHidApiInterface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 0; } //!< indicates that this plugin has got a configuration dialog

    private:

        hid_device *m_pDevice;

        bool m_debugMode;   /*! Enables / Disables live connection to dockingwidge-protocol */

    signals:
        void serialLog(QByteArray data, const char InOutChar);
        void uniqueIDChanged(const int);

    public slots:
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
class ItomHidApiInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        ItomHidApiInterface();
        ~ItomHidApiInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // ITOMHIDAPI_H

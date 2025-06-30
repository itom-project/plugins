/* ********************************************************************
    Plugin "SmarActMCS2" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2025, TRUMPF Lasersystems for Semiconductor Manufacturing SE,´Germany

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

#ifndef SMARACTMCS2_H
#define SMARACTMCS2_H

#include "common/addInInterface.h"
#include <qsharedpointer.h>
#include "dialogSmarActMCS2.h"

#include "SmarActControl.h"

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    SmarActMCS2Interface
  *
  *\brief    Interface-Class for SmarActMCS2-Class
  *
  *    \sa    AddInActuator, SmarActMCS2
  *
  */
class SmarActMCS2Interface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    public:
        SmarActMCS2Interface();
        ~SmarActMCS2Interface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);
};


//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    SmarActMCS2

  */
class SmarActMCS2 : public ito::AddInActuator
{
    Q_OBJECT

    protected:
        //! Destructor
        ~SmarActMCS2();
        //! Constructor
        SmarActMCS2();

    public:
        friend class SmarActMCS2Interface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog

    private:
        int m_async;    //!< variable to set up async and sync positioning --> Synchrone means program do not return until positioning was done.
        int m_nrOfAxes;

        SA_CTL_DeviceHandle_t m_insrumentHdl;
        QVector<int> m_factor;

        static QList<QString> openedDevices;

        ito::RetVal waitForDone(const int timeoutMS = -1, const QVector<int> axis = QVector<int>() /*if empty -> all axis*/, const int flags = 0 /*for your use*/);

        ito::RetVal updateStatus(); //optional method to obtain the status and position of all connected axes

    public slots:
        ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond);

        ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond);

        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);

        ito::RetVal close(ItomSharedSemaphore *waitCond);

        ito::RetVal calib(const int axis, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal calib(const QVector<int> axis, ItomSharedSemaphore *waitCond = NULL);

        ito::RetVal setOrigin(const int axis, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal setOrigin(const QVector<int> axis, ItomSharedSemaphore *waitCond = NULL);

        ito::RetVal getStatus(QSharedPointer<QVector<int> > status, ItomSharedSemaphore *waitCond);

        ito::RetVal getPos(const int axis, QSharedPointer<double> pos, ItomSharedSemaphore *waitCond);
        ito::RetVal getPos(const QVector<int> axis, QSharedPointer<QVector<double> > pos, ItomSharedSemaphore *waitCond);

        ito::RetVal setPosAbs(const int axis, const double pos, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal setPosAbs(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond = NULL);

        ito::RetVal setPosRel(const int axis, const double pos, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal setPosRel(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond = NULL);

        ito::RetVal execFunc(
            const QString funcName,
            QSharedPointer<QVector<ito::ParamBase>> paramsMand,
            QSharedPointer<QVector<ito::ParamBase>> paramsOpt,
            QSharedPointer<QVector<ito::ParamBase>> paramsOut,
            ItomSharedSemaphore* waitCond = nullptr);

    private slots:
        void dockWidgetVisibilityChanged(bool visible);
};

#endif // SMARACTMCS2_H

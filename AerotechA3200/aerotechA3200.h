/* ********************************************************************
    Plugin "AerotechA3200" for itom software
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

#include "common/addInInterface.h"

#define NOMINMAX //instead min, max is defined as macro in winDef.h, included by vld.h
#include <windows.h>
#include "AerSys.h"

#include <qsharedpointer.h>
#include <qmetatype.h>
#include <qlibrary.h>
#include <qevent.h>
//#include <WinNT.h>

class DockWidgetAerotechA3200;

//----------------------------------------------------------------------------------------------------------------------------------
/** @class AerotechA3200Interface
*/
class AerotechA3200Interface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        AerotechA3200Interface(QObject *parent = 0);
        ~AerotechA3200Interface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);    //!< Creates a new DummyMotor and gives him a unique identification

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);
};

//----------------------------------------------------------------------------------------------------------------------------------
/** @class AerotechA3200
*/
class AerotechA3200 : public ito::AddInActuator
{
    Q_OBJECT

    protected:
        AerotechA3200();    //!< Constructor
        ~AerotechA3200();    //! Destructor

        //void timerEvent( QTimerEvent *event );

        ito::RetVal checkError(int a3200ReturnValue);
        ito::RetVal axisFaultToRetVal(int axisFault, int axisID);
        ito::RetVal getAxisMask(const int *axes, const int numAxes, AXISMASK &mask);
        ito::RetVal getAxisMask2(const QVector<int> &axesIndices, AXISMASK &mask);

    public:
        friend class AerotechA3200Interface;
        const ito::RetVal showConfDialog(void);    //!< This calls the modal Configuration Dialog
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog

    private:
        int m_async;    //!< variable to set up async and sync positioning --> Synchrone means program do not return until positioning was done.

        QVector<double> m_offset;

        ito::RetVal waitForDone(const int timeoutMS = -1, const QVector<int> axis = QVector<int>() /*if empty -> all axis*/, const int flags = 0 /*for your use*/);

        ito::RetVal doUpdatePosAndState(const QVector<int> &axes);

        ito::RetVal enabledisable(int axis, int ziel);

        DockWidgetAerotechA3200 *m_pAerotechA3200Wid;

        HAERCTRL hAerCtrl;
        QVector<int> m_enabledAxes;
        QVector<int> m_allAxesVector;
        QStringList m_axisNames;

        TCHAR szMsg[MAX_TEXT_LEN]; // Buffer for the return error message from A3200


    signals:
        void dockWidgetAerotechA3200Init(QMap<QString, ito::Param> params, QStringList m_axisNames);

    public slots:
        ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond = NULL);

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

        //! Emits status and position if triggered. Used form the dockingwidget
        ito::RetVal RequestStatusAndPosition(bool sendActPosition, bool sendTargetPos);

    private slots:
        void dockWidgetVisibilityChanged(bool visible);
        void doAliveTimer();
};

//----------------------------------------------------------------------------------------------------------------------------------

/* ********************************************************************
    Plugin "USBMotion3XIII" for itom software
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

#ifndef USBMOTION3XIII_H
#define USBMOTION3XIII_H

#include "common/addInInterface.h"

#include "dialogUSBMotion3XIII.h"    //! This is the configuration dialog
#include "dockWidgetUSBMotion3XIII.h"    //! This is the control dialog

#include <qsharedpointer.h>
#include <qmetatype.h>
#include <qlibrary.h>
#include <qevent.h>

//----------------------------------------------------------------------------------------------------------------------------------
/** @class USBMotion3XIIIInterface
*/
class USBMotion3XIIIInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        USBMotion3XIIIInterface(QObject *parent = 0);
        ~USBMotion3XIIIInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);    //!< Creates a new actuator instance

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);
        ito::RetVal loadDLL();
        ito::RetVal unloadDLL();

        QLibrary mLib;

};

//----------------------------------------------------------------------------------------------------------------------------------
/** @class USBMotion3XIII
*/
class USBMotion3XIII : public ito::AddInActuator
{
    Q_OBJECT

    protected:
        ~USBMotion3XIII();    //! Destructor
        USBMotion3XIII();    //!< Constructor

        void timerEvent( QTimerEvent *event );

    public:
        friend class USBMotion3XIIIInterface;
        const ito::RetVal showConfDialog(void);    //!< This calls the modal Configuration Dialog
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog

    private:

        ito::RetVal checkConnection();
        ito::RetVal loadDriverSettingsToParams();
        ito::RetVal loadParamsToEEP();
        ito::RetVal errorCheck(unsigned int driverErrorNumber);
        double getTotalStepsPerUnit(int axis); //axis = 0,1,2

        ito::RetVal setMicroSteps(int axis, int steps);
        ito::RetVal setCoilCurrents(int axis, char changeBitMask, double agtat, double aleat, double v0, double threshold);
        ito::RetVal setSpeed(int axis, char changeBitMask, double vmin, double vmax);
        ito::RetVal setAcceleration(int axis, double amax);
        ito::RetVal setEnabled(int axis, int value);

        ito::RetVal changeStatusTimer(bool anyMotorIsMoving);
        ito::RetVal updateStatus();

        int m_axisUnit[3]; //0: deg, 1: mm

        int m_curDeviceIndex;
        QVector<unsigned char> m_availableAxis;
        int m_timerId;
        int m_timerInterval;

        DockWidgetUSBMotion3XIII *USBMotion3XIIIWid;

        int m_async;    //!< variable to set up async and sync positioning --> Synchrone means program do not return until positioning was done.

        ito::RetVal waitForDone(const int timeoutMS = -1, const QVector<int> axis = QVector<int>() /*if empty -> all axis*/, const int flags = 0 /*for your use*/);

        static QVector<QString> openedDevices;

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

    private slots:
        void dockWidgetVisibilityChanged( bool visible );
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // USBMOTION3XIII_H

/* ********************************************************************
    Plugin "Standa ST8SMC4USB" for itom software
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

#ifndef ST8SMC4USB_H
#define ST8SMC4USB_H

#include "common/addInInterface.h"

#include "dialogST8SMC4USB.h"
#include "dockWidgetST8SMC4USB.h"

#include <qsharedpointer.h>
#include <qvector.h>
#include <qpair.h>
#include <qbytearray.h>

#if defined(_WIN64)
    typedef unsigned __int64 ulong_t;
    typedef __int64 long_t;
#else
    typedef unsigned long long ulong_t;
    typedef long long long_t;
#endif

#include "ximc.h"

//----------------------------------------------------------------------------------------------------------------------------------
class ST8SMC4USB : public ito::AddInActuator
{
    Q_OBJECT

    protected:
        ~ST8SMC4USB(){};
        ST8SMC4USB();

    public:
        friend class ST8SMC4USBInterface;

        const ito::RetVal showConfDialog(void);    /*!<shows the configuration dialog*/
        int hasConfDialog(void) { return 1; } //!< indicates that this plugin has got a configuration dialog

    private:
        ito::AddInDataIO *m_pSer;

        int m_async;
        device_t m_device;
         engine_settings_t m_engine_settings;
        double m_unitPerSteps; //number of mm or degree per stepper motor full step

        QSharedPointer<ito::Param> endlineParam;

        QString getErrorString(const result_t result);
        QString getLogLevelString(int loglevel);

        ito::RetVal SMCSetPos(const QVector<int> axis, const QVector<double> posMM, bool relNotAbs, ItomSharedSemaphore *waitCond = nullptr);    /*!< Set a position (absolute or relative) */
        ito::RetVal SMCCheckStatus();
        ito::RetVal SMCCheckError(ito::RetVal retval);
        ito::RetVal synchronizeMotorSettings(double newAccel = -1.0, double newSpeed = -1.0, double newDecel = -1.0);

        double stepsToUnit(const get_position_t &steps, int microSteps);
        double stepsToUnit(int fullSteps, int uSteps, int microSteps);
        void unitToSteps(double unitStep, int microSteps, int &fullSteps, int &uSteps);

        int microStepsToMicrostepMode(const int microSteps);

        ito::RetVal waitForDone(const int timeoutMS = -1, const QVector<int> axis = QVector<int>() /*if empty -> all axis*/, const int flags = 0 /*for your use*/);

    public slots:
        ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore* waitCond = nullptr);
        ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore* waitCond = nullptr);

        ito::RetVal init(QVector<ito::ParamBase>* paramsMand, QVector<ito::ParamBase>* paramsOpt, ItomSharedSemaphore* waitCond = nullptr);
        ito::RetVal close(ItomSharedSemaphore *waitCond);

        //! Starts calibration for a single axis
        ito::RetVal calib(const int axis, ItomSharedSemaphore* waitCond = nullptr);
        //! Starts calibration for all axis -> calls single axis version
        ito::RetVal calib(const QVector<int> axis, ItomSharedSemaphore* waitCond = nullptr);
        //! Not implelemted yet
        ito::RetVal setOrigin(const int axis, ItomSharedSemaphore* waitCond = nullptr);
        //! Not implelemted yet
        ito::RetVal setOrigin(const QVector<int> axis, ItomSharedSemaphore* waitCond = nullptr);
        //! Reads out status request answer and gives back ito::retOk or ito::retError
        ito::RetVal getStatus(QSharedPointer<QVector<int> > status, ItomSharedSemaphore *waitCond);
        //! Get the position of a single axis
        ito::RetVal getPos(const int axis, QSharedPointer<double> pos, ItomSharedSemaphore *waitCond);
        //! Get the position of a all axis -> calls single axis version
        ito::RetVal getPos(const QVector<int> axis, QSharedPointer<QVector<double> > pos, ItomSharedSemaphore *waitCond);
        //! Set an absolute position and go there. Waits if m_async=0. Calls SMCSetPos of axis=0 else ito::retError
        ito::RetVal setPosAbs(const int axis, const double pos, ItomSharedSemaphore* waitCond = nullptr);
        //! Set an absolute position and go there. Waits if m_async=0. Calls SMCSetPos of axis[0]=0 && axis.size()=1 else ito::retError
        ito::RetVal setPosAbs(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore* waitCond = nullptr);
        //! Set a relative offset of current position and go there. Waits if m_async=0. Calls SMCSetPos of axis=0 else ito::retError
        ito::RetVal setPosRel(const int axis, const double pos, ItomSharedSemaphore* waitCond = nullptr);
        //! Set a relative offset of current position and go there. Waits if m_async=0. Calls SMCSetPos of axis[0]=0 && axis.size()=1 else ito::retError
        ito::RetVal setPosRel(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore* waitCond = nullptr);

        //! Emits status and position if triggered. Used form the dockingwidget
        ito::RetVal requestStatusAndPosition(bool sendCurrentPos, bool sendTargetPos);

    private slots:
        void dockWidgetVisibilityChanged( bool visible );
        void doAliveTimer();

};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // ST8SMC4USB_H

/* ********************************************************************
    Plugin "Newport SMC100" for itom software
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

#ifndef SMC100_H
#define SMC100_H

#include "common/addInInterface.h"

#include "dialogSMC100.h"
#include "dockWidgetSMC100.h"

#include <qsharedpointer.h>
#include <qvector.h>
#include <qpair.h>
#include <qbytearray.h>

//----------------------------------------------------------------------------------------------------------------------------------
class SMC100 : public ito::AddInActuator
{
    Q_OBJECT

    protected:
        ~SMC100() {}
        SMC100();

    public:

        friend class SMC100Interface;

        const ito::RetVal showConfDialog(void);    /*!<shows the configuration dialog*/
        int hasConfDialog(void) { return 1; } //!< indicates that this plugin has got a configuration dialog
        enum controllerStatus
        {
            ctrlStNotRef = 0,
            ctrlStConfig = 1,
            ctrlStDisabled = 2,
            ctrlStReady = 3,
            ctrlStMotion = 4
        };

        enum calibMode
        {
            calibModeNotDef = -1,
            calibModeMzAndInc = 0,
            calibModeCurrentPos = 1,
            calibModeMzOnly = 2,
            calibModeEorAndInc = 3,
            calibModeEorOnly = 4
        };


    private:
        ito::AddInDataIO *m_pSer;

        int m_numAxis;
        int m_async;

        QVector<int> m_addresses;
        QVector<QString> m_ids;
        QVector<int> m_controllerState;
        QVector<int> m_calibMode;
        QVector<double> m_acceleration;
        QVector<double> m_velocity;

        QSharedPointer<ito::Param> endlineParam;

        ito::RetVal SMCSendCommand(const QByteArray &cmd, bool checkError, int axis = -1);
        ito::RetVal SMCReadString(QByteArray &result, int &len, int timeoutMS, bool checkError, int axis = -1);
        ito::RetVal SMCSendQuestionWithAnswerDouble(const QByteArray &questionCommand, double &answer, int timeoutMS, bool checkError, int axis = -1);
        ito::RetVal SMCSendQuestionWithAnswerString(const QByteArray &questionCommand, QByteArray &answer, int timeoutMS, bool checkError, int axis = -1);

        ito::RetVal SMCSetPos(const QVector<int> axis, const QVector<double> posMM, bool relNotAbs, ItomSharedSemaphore *waitCond = NULL);    /*!< Set a position (absolute or relative) */
        ito::RetVal SMCCheckStatus(const QVector<int> axis);


        // Config Mode
        ito::RetVal SMCEnterConfigMode(const QVector<int> axis, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal SMCLeaveConfigMode(const QVector<int> axis, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal SMCResetController(const QVector<int> axis, ItomSharedSemaphore *waitCond = NULL);

        // Calib Mode
        ito::RetVal SMCSetCalibMode(const QVector<int> axisAndMode);
        ito::RetVal SMCGetCalibMode(const QVector<int> axis);

        // Velocity and acceleration
        ito::RetVal SMCGetVelocityAcceleration(bool vNota);
        ito::RetVal SMCSetVelocityAcceleration(bool vNota, const QVector<double> axis);

        ito::RetVal SMCCheckError(int axis = -1);

        ito::RetVal waitForDone(const int timeoutMS = -1, const QVector<int> axis = QVector<int>() /*if empty -> all axis*/, const int flags = 0 /*for your use*/);

    public slots:

        ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond = NULL);

        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal close(ItomSharedSemaphore *waitCond);

        //! Starts calibration for a single axis
        ito::RetVal calib(const int axis, ItomSharedSemaphore *waitCond = NULL);
        //! Starts calibration for all axis -> calls single axis version
        ito::RetVal calib(const QVector<int> axis, ItomSharedSemaphore *waitCond = NULL);
        //! Not implelemted yet
        ito::RetVal setOrigin(const int axis, ItomSharedSemaphore *waitCond = NULL);
        //! Not implelemted yet
        ito::RetVal setOrigin(const QVector<int> axis, ItomSharedSemaphore *waitCond = NULL);
        //! Reads out status request answer and gives back ito::retOk or ito::retError
        ito::RetVal getStatus(QSharedPointer<QVector<int> > status, ItomSharedSemaphore *waitCond);
        //! Get the position of a single axis
        ito::RetVal getPos(const int axis, QSharedPointer<double> pos, ItomSharedSemaphore *waitCond);
        //! Get the position of a all axis -> calls single axis version
        ito::RetVal getPos(const QVector<int> axis, QSharedPointer<QVector<double> > pos, ItomSharedSemaphore *waitCond);
        //! Set an absolute position and go there. Waits if m_async=0. Calls SMCSetPos of axis=0 else ito::retError
        ito::RetVal setPosAbs(const int axis, const double pos, ItomSharedSemaphore *waitCond = NULL);
        //! Set an absolute position and go there. Waits if m_async=0. Calls SMCSetPos of axis[0]=0 && axis.size()=1 else ito::retError
        ito::RetVal setPosAbs(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond = NULL);
        //! Set a relative offset of current position and go there. Waits if m_async=0. Calls SMCSetPos of axis=0 else ito::retError
        ito::RetVal setPosRel(const int axis, const double pos, ItomSharedSemaphore *waitCond = NULL);
        //! Set a relative offset of current position and go there. Waits if m_async=0. Calls SMCSetPos of axis[0]=0 && axis.size()=1 else ito::retError
        ito::RetVal setPosRel(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond = NULL);

        //! Emits status and position if triggered. Used form the dockingwidget
        ito::RetVal requestStatusAndPosition(bool sendCurrentPos, bool sendTargetPos);

    private slots:
        void dockWidgetVisibilityChanged( bool visible );
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // SMC100_H

/* ********************************************************************
    Plugin "Newport SMC100" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2014, Institut für Technische Optik (ITO),
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

    private:
        bool m_useOnTarget;
        bool m_getStatusInScan;
        bool m_getPosInScan;

        ito::AddInDataIO *m_pSer;

        double m_scale; //! in steps per mm
        int m_numAxis;
        int m_async;
        int m_delayAfterSendCommandMS;
        double m_delayProp; //s
        double m_delayOffset; //s
        bool m_hasHardwarePositionLimit;
        double m_posLimitLow;
        double m_posLimitHigh;

        QVector<int> m_addresses;

        ito::RetVal SMCDummyRead(void); /*!< reads buffer of serial port without delay in order to clear it */
        ito::RetVal SMCGetLastErrors( QVector<QPair<int,QByteArray> > &lastErrors );
        ito::RetVal SMCSendCommandInt(const QByteArray &cmd, int value, int axis = -1);
        ito::RetVal SMCSendCommandVoid(const QByteArray &cmd, int axis = -1);
        ito::RetVal SMCReadString(QByteArray &result, int &len, int timeoutMS);
        ito::RetVal SMCSendQuestionWithAnswerDouble( QByteArray questionCommand, double &answer, int timeoutMS );
        ito::RetVal SMCSendQuestionWithAnswerString( QByteArray questionCommand, QByteArray &answer, int timeoutMS );
        ito::RetVal SMCIdentifyAndInitializeSystem(int keepSerialConfig);
        ito::RetVal convertSMCErrorsToRetVal( QVector<QPair<int,QByteArray> > &lastErrors );
        ito::RetVal SMCSetOperationMode(bool localNotRemote);
        ito::RetVal SMCSetPos(const int axis, const double posMM, bool relNotAbs, ItomSharedSemaphore *waitCond = NULL);    /*!< Set a position (absolute or relative) */
        ito::RetVal SMCCheckStatus(void);

        ito::RetVal checkError(const ito::RetVal &retval);

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
        //! Set an absolut position and go thier. Waits if m_async=0. Calls SMCSetPos of axis=0 else ito::retError
        ito::RetVal setPosAbs(const int axis, const double pos, ItomSharedSemaphore *waitCond = NULL);
        //! Set an absolut position and go thier. Waits if m_async=0. Calls SMCSetPos of axis[0]=0 && axis.size()=1 else ito::retError
        ito::RetVal setPosAbs(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond = NULL);
        //! Set a relativ offset of current position and go thier. Waits if m_async=0. Calls SMCSetPos of axis=0 else ito::retError
        ito::RetVal setPosRel(const int axis, const double pos, ItomSharedSemaphore *waitCond = NULL);
        //! Set a relativ offset of current position and go thier. Waits if m_async=0. Calls SMCSetPos of axis[0]=0 && axis.size()=1 else ito::retError
        ito::RetVal setPosRel(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond = NULL);
        
        //! Emits status and position if triggered. Used form the dockingwidget
        ito::RetVal requestStatusAndPosition(bool sendCurrentPos, bool sendTargetPos);

    private slots:
        void dockWidgetVisibilityChanged( bool visible );
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // SMC100_H

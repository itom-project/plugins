/* ********************************************************************
    Plugin "PIPiezoControl" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut fuer Technische Optik (ITO),
    Universitaet Stuttgart, Germany

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

#ifndef PIPIEZOCTRL_H
#define PIPIEZOCTRL_H

#include "common/addInInterface.h"

#include "dialogPIPiezoCtrl.h"
#include "dockWidgetPIPiezoCtrl.h"

#include <qsharedpointer.h>
#include <qvector.h>
#include <qpair.h>
#include <qbytearray.h>

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    PIPiezoCtrl 
  *\brief    This class can be used to communicate with different PI-Piezo Controller (E-816, E-621, E-625, E-665 or E662) 
  *
  *         This class can be used to work with Piefocs and Piezo-Stages. The ITO-Controllers have only one axis with axis number 0.
  *            This system needs a serial port, which differs depending on type:
  *            baud = 9600 (E662) or 115200 (E-816, E-621, E-625, E-665)
  *            bits = 8
  *            parity = 0
  *            stopbits = 108
  *            flow = 1 (perhaps 2)
  *            endline = "\n"
  *            The class comes with a Config-Dialog and a Controll-Gui-Widget
  *
  * \todo setorigin
  *    \sa    AddInActuator, DummyMotor, dialogPIPiezoCtrl, DockWidgetPIPiezoCtrl
  *    \date    11.10.2010
  *    \author    Wolfram Lyda
  * \warning    NA
  *
  */
class PIPiezoCtrl : public ito::AddInActuator 
{
    Q_OBJECT

    protected:
        ~PIPiezoCtrl() {}
        PIPiezoCtrl();

    public:
#ifdef GCS2
        friend class PI_GCS2Interface;
#else
        friend class PIPiezoCtrlInterface;
#endif
        const ito::RetVal showConfDialog(void);    /*!<shows the configuration dialog*/
        int hasConfDialog(void) { return 1; } //!< indicates that this plugin has got a configuration dialog

    private:

        enum tControllerType { E662Family, C663Family, E625Family, E753Family, EUnknown };

        bool m_useOnTarget;
        bool m_getStatusInScan;
        bool m_getPosInScan;

#ifdef GCS2
        int m_deviceID;
#else
        ito::AddInDataIO *m_pSer;
#endif

        double m_scale; //! in steps per mm
        int m_numAxis;
        int m_async;
        int m_delayAfterSendCommandMS;
        double m_delayProp; //s
        double m_delayOffset; //s
        bool m_hasHardwarePositionLimit;
        double m_posLimitLow;
        double m_posLimitHigh;
        tControllerType m_ctrlType;
        DockWidgetPIPiezoCtrl *m_dockWidget;

        QByteArray m_AbsPosCmd;        /*!< This contains the command for absolut positioning. It is created & allocated in PISwitchType and freed in close(). This differs between E662 and (E-816, E-621, E-625, E-665) */
        QByteArray m_RelPosCmd;        /*!< This contains the command for relative positioning. It is created & allocated in PISwitchType and freed in close(). This differs between E662 and (E-816, E-621, E-625, E-665) */
        QByteArray m_PosQust;        /*!< This contains the command for position request. It is created & allocated in PISwitchType and freed in close(). This differs between E662 and (E-816, E-621, E-625, E-665) */
        QByteArray m_VelCmd;        /*!< This contains the command for position request. It is created & allocated in PISwitchType and freed in close(). This differs between E662 and (C-663) */
        QByteArray m_VelQust;        /*!< This contains the command for position request. It is created & allocated in PISwitchType and freed in close(). This differs between E662 and (C-663) */

        ito::RetVal PIDummyRead(void); /*!< reads buffer of serial port without delay in order to clear it */
        ito::RetVal PIGetLastErrors( QVector<QPair<int,QByteArray> > &lastErrors );
        ito::RetVal PISendCommand(const QByteArray &command );
        ito::RetVal PIReadString(QByteArray &result, int &len, const int timeoutMS);
        ito::RetVal PISendQuestionWithAnswerDouble(const QByteArray &questionCommand, double &answer, const int timeoutMS);
        ito::RetVal PISendQuestionWithAnswerDouble2(const QByteArray &questionCommand, const int axisId, double &answer, const int timeoutMS);
        ito::RetVal PISendQuestionWithAnswerInt2(const QByteArray& questionCommand, const int axisId, int& answer, const int timeoutMS);
        ito::RetVal PISendQuestionWithAnswerString(const QByteArray &questionCommand, QByteArray &answer, const int timeoutMS);
        ito::RetVal PIIdentifyAndInitializeSystem(int keepSerialConfig);
        ito::RetVal convertPIErrorsToRetVal( QVector<QPair<int,QByteArray> > &lastErrors );
        ito::RetVal PISetOperationMode(bool localNotRemote);
        ito::RetVal PISetPos(const int axis, const double posMM, bool relNotAbs, ItomSharedSemaphore *waitCond = NULL);    /*!< Set a position (absolute or relative) */
        ito::RetVal PICheckStatus(void);

        ito::RetVal PIFilterAnswerByteArray(QByteArray &answer, const int axisID);

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
        //! Set an absolut position and go thier. Waits if m_async=0. Calls PISetPos of axis=0 else ito::retError
        ito::RetVal setPosAbs(const int axis, const double pos, ItomSharedSemaphore *waitCond = NULL);
        //! Set an absolut position and go thier. Waits if m_async=0. Calls PISetPos of axis[0]=0 && axis.size()=1 else ito::retError
        ito::RetVal setPosAbs(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond = NULL);
        //! Set a relativ offset of current position and go thier. Waits if m_async=0. Calls PISetPos of axis=0 else ito::retError
        ito::RetVal setPosRel(const int axis, const double pos, ItomSharedSemaphore *waitCond = NULL);
        //! Set a relativ offset of current position and go thier. Waits if m_async=0. Calls PISetPos of axis[0]=0 && axis.size()=1 else ito::retError
        ito::RetVal setPosRel(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond = NULL);
        
        //! Emits status and position if triggered. Used form the dockingwidget
        ito::RetVal requestStatusAndPosition(bool sendCurrentPos, bool sendTargetPos);

    private slots:
        void dockWidgetVisibilityChanged( bool visible );
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // PIPIEZOCTRL_H

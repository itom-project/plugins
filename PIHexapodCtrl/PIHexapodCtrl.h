#ifndef PIHexapodCTRL_H
#define PIHexapodCTRL_H

#include "common/addInInterface.h"

#include "dialogPIHexapodCtrl.h"
#include "dockWidgetPIHexapodCtrl.h"

#include <qsharedpointer.h>
#include <qvector.h>
#include <qpair.h>
#include <qbytearray.h>
#include <QTcpSocket>

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    PIHexapodCtrl
  *\brief    This class can be used to communicate with different PI-Hexapod Controller (E-816, E-621, E-625, E-665 or E662)
  *
  *         This class can be used to work with Piefocs and Hexapod-Stages. The ITO-Controllers have only one axis with axis number 0.
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
  *    \sa    AddInActuator, DummyMotor, dialogPIHexapodCtrl, DockWidgetPIHexapodCtrl
  *    \date    11.10.2010
  *    \author    Wolfram Lyda
  * \warning    NA
  *
  */
class PIHexapodCtrl : public ito::AddInActuator
{
    Q_OBJECT

    protected:
        ~PIHexapodCtrl() {}
        PIHexapodCtrl();

    public:
        friend class PIHexapodCtrlInterface;
        const ito::RetVal showConfDialog(void);    /*!<shows the configuration dialog*/
        int hasConfDialog(void) { return 0; } //!< indicates that this plugin has got a configuration dialog

    private:

        ito::AddInDataIO *m_pSer;
        double m_scale; //! in steps per mm

        int m_async;
        int m_delayAfterSendCommandMS;
        double m_delayProp; //s
        double m_delayOffset; //s
        bool m_hasHardwarePositionLimit;

        bool m_useTCPIP;
        QTcpSocket *m_connection;
        QString m_tcpAddr;
        long m_tcpPort;
        bool m_closing;
        bool m_isInit;
        bool m_doWait;

        Qt::HANDLE m_threadID;

        QList<QByteArray> m_axesNames;
        int m_numAxis;

        DockWidgetPIHexapodCtrl *m_dockWidget;

        ito::RetVal PIGetLastErrors( QVector<QPair<int,QByteArray> > &lastErrors );

        ito::RetVal updatePivotPoint();
        ito::RetVal setPivotPoint(const double *values);
        ito::RetVal PIDummyRead(void); /*!< reads buffer of serial port without delay in order to clear it */
        ito::RetVal PISendCommand(const QByteArray &command);
        ito::RetVal PIReadString(QByteArray &result, int &len, const int timeoutMS);
        ito::RetVal PISendQuestionWithAnswerDouble(const QByteArray &questionCommand, double &answer, int timeoutMS );
        ito::RetVal PISendQuestionWithAnswerString(const QByteArray &questionCommand, QByteArray &answer, int timeoutMS );

        ito::RetVal PIIdentifyAndInitializeSystem(void);
        ito::RetVal convertPIErrorsToRetVal( QVector<QPair<int,QByteArray> > &lastErrors );
        ito::RetVal PISetPos(const QVector<int> &axis,  const QVector<double> &posMM, bool relNotAbs, ItomSharedSemaphore *waitCond = NULL);    /*!< Set a position (absolute or relative) */
        ito::RetVal PICheckStatus(int timeoutMS = 200);

        ito::RetVal waitForDone(const int timeoutMS = -1, const QVector<int> axis = QVector<int>() /*if empty -> all axis*/, const int flags = 0 /*for your use*/);

        ito::RetVal tcpOpenConnection(const QString &ipAddress, const long port, const char /*proto*/);
        ito::RetVal tcpReconnect(void);
        void        tcpChanged(QAbstractSocket::SocketState socketState);

        inline ito::RetVal checkAxisVector(const QVector<int> &axis);

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
        //! Set an absolute position and go there. Waits if m_async=0. Calls PISetPos of axis=0 else ito::retError
        ito::RetVal setPosAbs(const int axis, const double pos, ItomSharedSemaphore *waitCond = NULL);
        //! Set an absolute position and go there. Waits if m_async=0. Calls PISetPos of axis[0]=0 && axis.size()=1 else ito::retError
        ito::RetVal setPosAbs(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond = NULL);
        //! Set a relative offset of current position and go there. Waits if m_async=0. Calls PISetPos of axis=0 else ito::retError
        ito::RetVal setPosRel(const int axis, const double pos, ItomSharedSemaphore *waitCond = NULL);
        //! Set a relative offset of current position and go there. Waits if m_async=0. Calls PISetPos of axis[0]=0 && axis.size()=1 else ito::retError
        ito::RetVal setPosRel(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond = NULL);

        //! Emits status and position if triggered. Used form the dockingwidget
        ito::RetVal requestStatusAndPosition(bool sendCurrentPos, bool sendTargetPos);

        ito::RetVal execFunc(const QString funcName, QSharedPointer<QVector<ito::ParamBase> > paramsMand, QSharedPointer<QVector<ito::ParamBase> > paramsOpt, QSharedPointer<QVector<ito::ParamBase> > paramsOut, ItomSharedSemaphore *waitCond);


    private slots:
        void dockWidgetVisibilityChanged( bool visible );

        void threadSafeClose();
};

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    PIHexapodCtrlInterface
  *
  *\brief    Interface-Class for PIHexapodCtrlInterface-Class
  *
  *    \sa    AddInActuator, PIHexapodCtrl
  *    \date    11.10.2010
  *    \author    Wolfram Lyda
  * \warning    NA
  *
  */
class PIHexapodCtrlInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        PIHexapodCtrlInterface();
        ~PIHexapodCtrlInterface() {};
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);


    signals:

    public slots:
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // PIHexapodCTRL_H

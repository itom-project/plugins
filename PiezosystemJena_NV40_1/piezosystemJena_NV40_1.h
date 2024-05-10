#ifndef PIEZOSYSTEMJENA_NV40_1_H
#define PIEZOSYSTEMJENA_NV40_1_H

#include "common/addInInterface.h"

#include "dialogPiezosystemJena_NV40_1.h"


#include <qsharedpointer.h>
#include <qvector.h>
#include <qpair.h>
#include <qbytearray.h>

class DockWidgetPiezosystemJena_NV40_1;

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    PiezosystemJena_NV40_1
  */
class PiezosystemJena_NV40_1 : public ito::AddInActuator
{
    Q_OBJECT

    protected:
        ~PiezosystemJena_NV40_1() {}
        PiezosystemJena_NV40_1();

    public:
        friend class PiezosystemJena_NV40_1Interface;
        const ito::RetVal showConfDialog(void);    /*!<shows the configuration dialog*/
        int hasConfDialog(void) { return 1; } //!< indicates that this plugin has got a configuration dialog

    private:
        ito::AddInDataIO *m_pSer;
        int m_async;
        bool m_closedLoop;
        int m_delayAfterSendCommandMS;

        QByteArray m_receiveEndline;

        DockWidgetPiezosystemJena_NV40_1 *m_dockWidget;

        ito::RetVal serialDummyRead(QByteArray *content = NULL); /*!< reads buffer of serial port without delay in order to clear it */
        ito::RetVal serialSendCommand(const QByteArray &command);
        void sleep(int ms);
        ito::RetVal readString(QByteArray &result, int &len, int timeoutMS, const QByteArray endline = "");
        ito::RetVal sendQuestionWithAnswerDouble(const QByteArray &questionCommand, double &answer, int timeoutMS);
        ito::RetVal sendQuestionWithAnswerString(const QByteArray &questionCommand, QByteArray &answer, int timeoutMS);
        ito::RetVal identifyAndInitializeSystem(QString &identifier);
        ito::RetVal setPos(const int axis, const double posMM, bool relNotAbs, ItomSharedSemaphore *waitCond = NULL);    /*!< Set a position (absolute or relative) */

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

    private slots:
        void dockWidgetVisibilityChanged( bool visible );
};

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    PiezosystemJena_NV40_1Interface
  *
  *\brief    Interface-Class for PiezosystemJena_NV40_1-Class
  *
  *    \sa    AddInActuator
  * \warning    NA
  *
  */
class PiezosystemJena_NV40_1Interface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        PiezosystemJena_NV40_1Interface();
        ~PiezosystemJena_NV40_1Interface() {};
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);

    signals:

    public slots:
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // PIEZOSYSTEMJENA_NV40_1_H

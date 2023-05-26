#ifndef UHLTEXT_H
#define UHLTEXT_H

#include "common/addInInterface.h"

#include <qsharedpointer.h>
#include <qmetatype.h>

#include <QTimer>

#include "dialogUhl.h"
#include "dockWidgetUhl.h"

//----------------------------------------------------------------------------------------------------------------------------------
class UhlText : public ito::AddInActuator //, public DummyGrabberInterface
{
    Q_OBJECT

    protected:
        ~UhlText();
        UhlText();

//        void connectNotify ( const char * signal );
//        void disconnectNotify ( const char * signal );

    public:
        friend class UhlTextInterface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog


    private:
        unsigned char m_spitchx ; // spindle pitch X in 100nm
//        m_spitchy     =   22,
//        m_spitchz     =   23,
        unsigned char m_resolution; // step per digit of traverse path
        ito::AddInDataIO *m_pSer;
        int m_numAxis;
        double m_scale; // in steps per m
        long m_turnspeed;
        double m_accel;  // RAMP
        int m_async;
        int m_inverse[4];
        char m_nameAxis[4];

        float m_pitch[4];

        QTimer m_jogTimer;
        int m_jogging[4];
        bool m_jogDir[4];

        int m_posrequestlisteners;
        int m_joyEnabled;

//        double offset;
//        unsigned short locked;
//        const ito::RetVal UhlSetPos(QVector<int> axis, QVector<double> pos, const unsigned char absrelflag);
        const ito::RetVal UhlSetPos(QVector<int> axis, QVector<double> pos, const unsigned char absrelflag, ItomSharedSemaphore *waitCond);
        const ito::RetVal DummyRead(void);
        const ito::RetVal UhlReadString(char buf[], const int bufsize, const int tries, int * signcnt);
        const ito::RetVal UhlReadFloat(float *value);
        const ito::RetVal UhlReadLong(long *value);
        const ito::RetVal UhlStatus(void);
        const ito::RetVal UhlJoystickOn(void);
        const ito::RetVal UhlJoystickOff(bool waitforanswer = true);
        const ito::RetVal UhlCheckAxisNumber(const int axis);
        const ito::RetVal UhlCheckAxisNumber(QVector<int> axis);

        static void doNotDelSharedPtr(char * /*ptr*/) {} //workaround for deleter for QSharedPointer, such that the pointer is NOT deleted if shared-pointer's reference drops towards zero.
        ito::RetVal waitForDone(const int timeoutMS = -1, const QVector<int> axis = QVector<int>() /*if empty -> all axis*/, const int flags = 0 /*for your use*/);

/*    signals:

        void parametersChanged(QMap<QString, ito::Param> params);
//        void PositioningStatusChanged(bool Running);
//        void SentStatusChanged(int Status);
//        void SentPositionChanged(QVector<int> axis,QVector<double> pos);
*/
    public slots:
        ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond = NULL);    //* returns parameter of m_params with key name.*/
        ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond = NULL);    //* sets parameter of m_params with key name.*/

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

        ito::RetVal startJoyStickMovement(QVector<int> axis, QVector<double> vel);

        void triggerJogIncrement();

    private slots:
        void dockWidgetVisibilityChanged(bool visible); //overwritten from AddInBase
};

//----------------------------------------------------------------------------------------------------------------------------------
class UhlTextInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        UhlTextInterface();
        ~UhlTextInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);

    signals:

    public slots:
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // SERIALIO_H

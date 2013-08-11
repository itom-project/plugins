#ifndef USBMOTION3XIII_H
#define USBMOTION3XIII_H

/**\file DummyMotor.h
* \brief In this file the class the DummyMotor and its interface are defined
* 
*\sa DummyMotorInterface, DummyMotor
*\author Wolfram Lyda
*\date	Oct2011
*/

#include "common/addInInterface.h"

#include "dialogUSBMotion3XIII.h"	//! This is the configuration dialog
#include "dockWidgetUSBMotion3XIII.h"	//! This is the controll dialog

#include <qsharedpointer.h>
#include <qmetatype.h>
#include <qlibrary.h>
#include <qevent.h>

//----------------------------------------------------------------------------------------------------------------------------------
/** @class DummyMotorInterface
*   @brief DummyMotor functionality
*   
*   AddIn Interface for the DummyMotor class s. also \ref DummyMotor
*/
class USBMotion3XIIIInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_INTERFACES(ito::AddInInterfaceBase)

    protected:

    public:
        USBMotion3XIIIInterface(QObject *parent = 0);
        ~USBMotion3XIIIInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);	//!< Creates a new DummyMotor and gives him a unique identification

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);
        ito::RetVal loadDLL();
        ito::RetVal unloadDLL();

        QLibrary mLib;

};

//----------------------------------------------------------------------------------------------------------------------------------
/** @class DummyMotor
*   @brief DummyMotor functionality
*   
*   The DummyMotor-Class can be used where algorithms should be tested without actually
*   using a specific hardware. The DummyMotor basically accepts setPos and getPos commands
*   and keeps track of the position with an internal variable. The maximum number of 
*   axis is currently limited to 10 - just for programmers conveniance.
*/
class USBMotion3XIII : public ito::AddInActuator 
{
    Q_OBJECT

    protected:
        ~USBMotion3XIII();	//! Destructor
//        DummyMotor(int uniqueID, QObject *parent = 0);	//!< Constructur
        USBMotion3XIII();	//!< Constructur

        void timerEvent( QTimerEvent *event );

    public:
        friend class USBMotion3XIIIInterface;
        const ito::RetVal showConfDialog(void);	//!< This calls the modal Configuration Dialog
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog

    private:

        ito::RetVal checkConnection();
        ito::RetVal loadDriverSettingsToParams();
        ito::RetVal loadParamsToEEP();
        ito::RetVal errorCheck(unsigned int driverErrorNumber);
        int getTotalStepsPerTurn(int axis); //axis = 0,1,2

        ito::RetVal setMicroSteps(int axis, int steps);
        ito::RetVal setCoilCurrents(int axis, char changeBitMask, double agtat, double aleat, double v0, double threshold); 
        ito::RetVal setSpeed(int axis, char changeBitMask, double vmin, double vmax);
        ito::RetVal setAcceleration(int axis, double amax);
        ito::RetVal setEnabled(int axis, int value);

        ito::RetVal changeStatusTimer(bool anyMotorIsMoving);
        ito::RetVal updateStatus();

        int m_curDeviceIndex;
        QVector<unsigned char> m_availableAxis;
        int m_timerId;
        int m_timerInterval;

        DockWidgetUSBMotion3XIII *USBMotion3XIIIWid;

		int m_async;	//!< variable to set up async and sync positioning --> Synchrone means program do not return until positioning was done.

        ito::RetVal waitForDone(const int timeoutMS = -1, const QVector<int> axis = QVector<int>() /*if empty -> all axis*/, const int flags = 0 /*for your use*/);

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

        void setAbsTargetDegree(double target1, double target2, double target3);
        void setRelTargetDegree(unsigned int axisNo, double relStepDegree);

    private slots:
        void dockWidgetVisibilityChanged( bool visible );
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // USBMOTION3XIII_H

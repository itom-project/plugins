/* ********************************************************************
    Plugin "LeicaMotorFocus" for itom software
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

#ifndef LEICAMF_H
#define LEICAMF_H

#include "common/addInInterface.h"

#include "dialogLeicaMotorFocus.h"
#include "dockWidgetLeicaMotorFocus.h"
//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    LeicaMotorFocus
  *\brief    class to use a motorized Leica MZ12 or MZ12.5 as an ITOM-Addin. Child of AddIn - Library (DLL) - Interface
  *
  *         This class can be used to work with a motorized Leica MZ12(.5). The motor has only one axis (z) with the axis number 0.
  *            This system needs a serial port with:
  *            baud = 9600
  *            bits = 8
  *            parity = 0
  *            stopbits = 1
  *            flow = 1 (perhaps 2)
  *            endline = "\r\n"
  *            The class comes with a Config-Dialog and a Controll-Gui-Widget
  *
  * \todo setorigin
  *    \sa    AddInActuator, DummyMotor, dialogLeicaMotorFocus, DockWidgetLeicaMotorFocus
  *    \date    11.10.2010
  *    \author    Wolfram Lyda
  * \warning    NA
  *
  */
class LeicaMotorFocus : public ito::AddInActuator
{
    Q_OBJECT

    protected:
        ~LeicaMotorFocus() {};    /*! < Destructor*/
        LeicaMotorFocus();/*! < Constructor*/

    public:
        friend class LeicaMotorFocusInterface;
        const ito::RetVal showConfDialog(void);    /*! < Opens the modal configuration dialog*/
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog

    private:

        ito::AddInDataIO *m_pSer;    /*! < Handle to serial port for communication with the microscope*/
        double m_scale; /*!< Smallest increment of motor in mm*/
        int m_async;    /*!< Toggels wait until timeout / stop driving and dont wait*/
        int m_direction; /*!< Implement mirror used in getPos and LMFSetPos*/

        const ito::RetVal LMFDummyRead(void);    /*!< Clear serial port before writing*/
        const ito::RetVal LMFReadString(char *buf, const int bufsize, int * readsigns);    /*!< Reads a string from the serial port */
        const ito::RetVal LMFWriteCmd(int id, int cmd);    /*!< Writes a command without argument to serial port*/
        const ito::RetVal LMFWriteCmdArg(int id, int cmd, long arg);    /*!< Writes a command with argument of type long to serial port*/
        const ito::RetVal LMFQueryS(int id, int cmd, char *buf, int bufsize); /*!< Writes an command and reads the answer (string)*/
        const ito::RetVal LMFQueryL(int id, int cmd, long *plval);    /*!< Writes an command and reads the answer, transfers answer to long*/
        //const ito::RetVal LMFWaitForAnswer(const int timeout);    /*!< Checks if the motor is still moving else returns / droppes to timeout*/
        const ito::RetVal LMFStatus(int &status); /*!< Read statusreport from the stage*/

        const ito::RetVal LMFSetPos(QVector<int> axis, const double pos, const int absrelflag, ItomSharedSemaphore *waitCond);    /*!< Set a position (absolute or relative)*/

        ito::RetVal waitForDone(const int timeoutMS = -1, const QVector<int> axis = QVector<int>() /*if empty -> all axis*/, const int flags = 0 /*for your use*/);

        static void doNotDelSharedPtr(char * /*ptr*/) {} /*!<workaround for deleter for QSharedPointer, such that the pointer is NOT deleted if shared-pointer's reference drops towards zero.*/
		static QSharedPointer<QVector<ito::ParamBase> > emptySharedParamBaseVec;

    public slots:
        ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL); /*!< This function establish the communication between the plugin and the microscope*/
        ito::RetVal close(ItomSharedSemaphore *waitCond); /*!< This function kills the communication before the plugin is destroyed */

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
        //! Set an absolut position and go thier. Waits if m_async=0. Calls LMFSetPos of axis=0 else ito::retError
        ito::RetVal setPosAbs(const int axis, const double pos, ItomSharedSemaphore *waitCond = NULL);
        //! Set an absolut position and go thier. Waits if m_async=0. Calls LMFSetPos of axis[0]=0 && axis.size()=1 else ito::retError
        ito::RetVal setPosAbs(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond = NULL);
        //! Set a relativ offset of current position and go thier. Waits if m_async=0. Calls LMFSetPos of axis=0 else ito::retError
        ito::RetVal setPosRel(const int axis, const double pos, ItomSharedSemaphore *waitCond = NULL);
        //! Set a relativ offset of current position and go thier. Waits if m_async=0. Calls LMFSetPos of axis[0]=0 && axis.size()=1 else ito::retError
        ito::RetVal setPosRel(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond = NULL);
        //! Emits status and position if triggered. Used form the dockingwidget
        ito::RetVal requestStatusAndPosition(bool sendCurrentPos, bool sendTargetPos);

    private slots:
        void dockWidgetVisibilityChanged( bool visible );
};

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    LeicaMotorFocusInterface
  *
  *\brief    Interface-Class for LeicaMotorFocusInterface-Class
  *
  *    \sa    AddInActuator, LeicaMotorFocus
  *    \date    11.10.2010
  *    \author    Wolfram Lyda
  * \warning    NA
  *
  */
class LeicaMotorFocusInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        LeicaMotorFocusInterface();
        ~LeicaMotorFocusInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst); /*! < Create new instance of the LeicaMotorFocus*/

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);

    signals:

    public slots:
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // LEICAMF_H

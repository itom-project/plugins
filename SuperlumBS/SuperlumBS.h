/* ********************************************************************
    Plugin "SuperlumBS" for itom software
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

#ifndef SUPERLUMBS_H
#define SUPERLUMBS_h

#include "common/addInInterface.h"
#include "common/abstractAddInDockWidget.h"

#include "dialogSuperlumBS.h"
#include "dockWidgetSuperlumBS.h"

#include <qsharedpointer.h>
#include <qvector.h>
#include <qpair.h>
#include <qbytearray.h>

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    SuperlumBS
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
  *    \sa    AddInActuator, DummyMotor,
  *    \date    12.03.2014
  *    \author    Johann Krauter
  * \warning    NA
  *
  */
class SuperlumBS : public ito::AddInDataIO
{
    Q_OBJECT

    protected:
        ~SuperlumBS() {}
        SuperlumBS();

    public:
        friend class SuperlumBSInterface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 1; }        //!< indicates that this plugin has got a configuration dialog

    private:

        ito::AddInDataIO *m_pSer;
        int m_delayAfterSendCommandMS;

        enum DeviceType { BS_840_1_HP, Unknown};

        DeviceType m_deviceType;

        DockWidgetSuperlumBS *m_dockWidget;
        //ito::RetVal SerialDummyRead(void); /*!< reads buffer of serial port without delay in order to clear it */
        ito::RetVal SerialSendCommand(QByteArray command);
        ito::RetVal readString(QByteArray &command, QByteArray &result, int &len, int timeoutMS);
        ito::RetVal SendQuestionWithAnswerString(QByteArray questionCommand, QByteArray &answer, int timeoutMS);
        //ito::RetVal SetPos(const int axis, const double posMM, bool relNotAbs, ItomSharedSemaphore *waitCond = NULL);    /*!< Set a position (absolute or relative) */
        //ito::RetVal CheckStatus(void);
        ito::RetVal waitForDone(const int timeoutMS = -1, const QVector<int> axis = QVector<int>() /*if empty -> all axis*/, const int flags = 0 /*for your use*/);
        ito::RetVal IdentifyAndInitializeSystem();

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
  *\class    SuperlumBSInterface
  *
  *\brief    Interface-Class for SuperlumBSInterface-Class
  *
  *    \sa    AddInActuator, SuperlumBS
  *    \date    12.03.2014
  *    \author    Johann Krauter
  * \warning    NA
  *
  */
class SuperlumBSInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        SuperlumBSInterface();
        ~SuperlumBSInterface() {};
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);

    signals:


    public slots:
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // SUPERLUMBS_H

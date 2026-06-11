/* ********************************************************************
    Plugin "DummyMotor" for itom software
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

#ifndef DUMMYMOTOR_H
#define DUMMYMOTOR_H

/**\file DummyMotor.h
* \brief In this file the class the DummyMotor and its interface are defined
*
*\sa DummyMotorInterface, DummyMotor
*\author Wolfram Lyda
*\date    Oct2011
*/

#include "common/addInInterface.h"

#include "dialogDummyMotor.h"    //! This is the configuration dialog
#include "dockWidgetDummyMotor.h"    //! This is the control dialog

#include <qsharedpointer.h>
#include <qmetatype.h>
#ifdef WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

//----------------------------------------------------------------------------------------------------------------------------------
/** @class DummyMotorInterface
*   @brief DummyMotor functionality
*
*   AddIn Interface for the DummyMotor class s. also \ref DummyMotor
*/
class DummyMotorInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        DummyMotorInterface(QObject *parent = 0);
        ~DummyMotorInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);    //!< Creates a new DummyMotor and gives it a unique identification
        bool hasDockWidget() { return true; }

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);

    signals:

    public slots:
};

//----------------------------------------------------------------------------------------------------------------------------------
/** @class DummyMotor
*   @brief DummyMotor functionality
*
*   The DummyMotor-Class can be used where algorithms should be tested without actually
*   using a specific hardware. The DummyMotor basically accepts setPos and getPos commands
*   and keeps track of the position with an internal variable. The maximum number of
*   axis is currently limited to 10 - just for programmers convenience.
*/
class DummyMotor : public ito::AddInActuator
{
    Q_OBJECT

    public:
        friend class DummyMotorInterface;
        const ito::RetVal showConfDialog(void);    //!< This calls the modal Configuration Dialog
        int hasConfDialog(void) { return 1; } //!< indicates that this plugin has got a configuration dialog

    protected:
        ~DummyMotor(); //!< Destructor
        DummyMotor();    //!< Constructor

        ito::RetVal waitForDone(const int timeoutMS = -1, const QVector<int> axis = QVector<int>() /*if empty -> all axis*/, const int flags = 0 /*for your use*/);

    private:
        int m_numaxis;  //!< Number of axis currently available at this stage
        int m_async;    //!< variable to set up async and sync positioning --> Syncrone means program do not return until positioning was done.
        int m_scale;    // Its something to round from ITO mm into stepwith of the corresponding system

        // Cut here
        double m_distance;    //! Just to enable the WaitForAnswer to wait according to the Speed
        double m_startPos[10];
        // till here

    public slots:
        ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal close(ItomSharedSemaphore *waitCond);

        ito::RetVal calib(const int axis, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal calib(const QVector<int> axis, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal setOrigin(const int axis, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal setOrigin(const QVector<int> axis, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal getStatus(QSharedPointer<QVector<int> > status, ItomSharedSemaphore *waitCond);
        ito::RetVal getPos(const int axis, QSharedPointer<double> pos, ItomSharedSemaphore *waitCond);
        ito::RetVal getPos(const QVector<int> axis, QSharedPointer<QVector<double> > pos, ItomSharedSemaphore *waitCond);
        ito::RetVal setPosAbs(const int axis, const double pos, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal setPosAbs(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal setPosRel(const int axis, const double pos, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal setPosRel(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond = nullptr);

        ito::RetVal execFunc(const QString funcName, QSharedPointer<QVector<ito::ParamBase> > paramsMand, QSharedPointer<QVector<ito::ParamBase> > paramsOpt, QSharedPointer<QVector<ito::ParamBase> > paramsOut, ItomSharedSemaphore *waitCond);
        ito::RetVal requestStatusAndPosition(bool sendCurrentPos, bool sendTargetPos);    //!< Slot to trigger a Status and position request
        ito::RetVal startJoyStickMovement(QVector<int> axis, QVector<double> vel);

    private slots:
        void dockWidgetVisibilityChanged(bool visible); //overwritten from AddInBase
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // DUMMYMOTOR_H

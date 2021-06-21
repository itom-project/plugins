/* ********************************************************************
    Plugin "ThorlabsISM" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2020, Institut fuer Technische Optik (ITO),
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

#pragma once

#define NOMINMAX // https://stackoverflow.com/questions/22744262/cant-call-stdmax-because-minwindef-h-defines-max

#include "common/addInInterface.h"

#include <qbytearray.h>
#include <qlibrary.h>
#include <qlist.h>
#include <qpair.h>
#include <qsharedpointer.h>
#include <qvector.h>

#include "Thorlabs.MotionControl.KCube.DCServo.h"

//-------------------------------------------------------------------------------------
/** @class ThorlabsKCubeDCServoInterface
 */
class ThorlabsKCubeDCServoInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase")
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

protected:
public:
    ThorlabsKCubeDCServoInterface();
    ~ThorlabsKCubeDCServoInterface(){};
    ito::RetVal getAddInInst(ito::AddInBase** addInInst);

private:
    ito::RetVal closeThisInst(ito::AddInBase** addInInst);

    QLibrary mLib;

signals:

public slots:
};

//-------------------------------------------------------------------------------------
/** @class ThorlabsKCubeDCServo
 */
class ThorlabsKCubeDCServo : public ito::AddInActuator
{
    Q_OBJECT

protected:
    ThorlabsKCubeDCServo();
    ~ThorlabsKCubeDCServo()
    {
    }

public:
    friend class ThorlabsKCubeDCServoInterface;

    /*!<shows the configuration dialog*/
    const ito::RetVal showConfDialog(void); 

    //!< indicates that this plugin has got a configuration dialog
    int hasConfDialog(void)
    {
        return 1;
    } 

private:
    bool m_async;
    bool m_opened;
    int m_numaxis;
    char m_serialNo[16];

    //!< the absolute positions, where the origin command was triggered.
    QVector<double> m_originPositions;

    static QList<QByteArray> openedDevices;
    static int numberOfKinesisSimulatorConnections;

    ito::RetVal waitForDone(
        const int timeoutMS = -1, const int axis = -1, const int flags = -1 /*for your use*/);

    ito::RetVal waitForDone(
        const int timeoutMS = -1,
        const QVector<int> axis = QVector<int>() /*if empty -> all axis*/,
        const int flags = -1 /*for your use*/);

    ito::RetVal checkError(short value, const char* message);

public slots:
    ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore* waitCond = nullptr);

    ito::RetVal setParam(
        QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore* waitCond = nullptr);

    ito::RetVal init(
        QVector<ito::ParamBase>* paramsMand,
        QVector<ito::ParamBase>* paramsOpt,
        ItomSharedSemaphore* waitCond = nullptr);

    ito::RetVal close(ItomSharedSemaphore* waitCond);

    //! Starts calibration for a single axis
    ito::RetVal calib(const int axis, ItomSharedSemaphore* waitCond = nullptr);

    //! Starts calibration for all axis -> calls single axis version
    ito::RetVal calib(const QVector<int> axis, ItomSharedSemaphore* waitCond = nullptr);

    
    ito::RetVal setOrigin(const int axis, ItomSharedSemaphore* waitCond = nullptr);

    
    ito::RetVal setOrigin(const QVector<int> axis, ItomSharedSemaphore* waitCond = nullptr);

    //! Reads out status request answer and gives back ito::retOk or ito::retError
    ito::RetVal getStatus(QSharedPointer<QVector<int>> status, ItomSharedSemaphore* waitCond);

    //! Get the position of a single axis
    ito::RetVal getPos(const int axis, QSharedPointer<double> pos, ItomSharedSemaphore* waitCond);

    //! Get the position of a all axis -> calls single axis version
    ito::RetVal getPos(
        const QVector<int> axis,
        QSharedPointer<QVector<double>> pos,
        ItomSharedSemaphore* waitCond);

    //! Set an absolut position and go thier. Waits if m_async=0. Calls SMCSetPos of axis=0 else
    //! ito::retError
    ito::RetVal setPosAbs(
        const int axis, const double pos, ItomSharedSemaphore* waitCond = nullptr);

    //! Set an absolut position and go thier. Waits if m_async=0. Calls SMCSetPos of axis[0]=0 &&
    //! axis.size()=1 else ito::retError
    ito::RetVal setPosAbs(
        const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore* waitCond = nullptr);

    //! Set a relativ offset of current position and go thier. Waits if m_async=0. Calls SMCSetPos
    //! of axis=0 else ito::retError
    ito::RetVal setPosRel(
        const int axis, const double pos, ItomSharedSemaphore* waitCond = nullptr);

    //! Set a relativ offset of current position and go thier. Waits if m_async=0. Calls SMCSetPos
    //! of axis[0]=0 && axis.size()=1 else ito::retError
    ito::RetVal setPosRel(
        const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore* waitCond = nullptr);

    //! Emits status and position if triggered. Used form the dockingwidget
    ito::RetVal requestStatusAndPosition(bool sendCurrentPos, bool sendTargetPos);

private slots:
    void dockWidgetVisibilityChanged(bool visible);
};

/* ********************************************************************
    Plugin "FaulhaberMCS" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2024, Institut für Technische Optik (ITO),
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

#ifndef FAULHABERMCS_H
#define FAULHABERMCS_H

#include "Momancmd.h"
#include "Momanprot.h"
#include "common/addInInterface.h"
#include "dialogFaulhaberMCS.h"
#include "dockWidgetFaulhaberMCS.h"
#include <qsharedpointer.h>
#include <windows.h>

//----------------------------------------------------------------------------------------------------------------------------------
class FaulhaberMCSInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase")
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

public:
    FaulhaberMCSInterface();
    ~FaulhaberMCSInterface();
    ito::RetVal getAddInInst(ito::AddInBase** addInInst);
    bool hasDockWidget()
    {
        return true;
    }

private:
    ito::RetVal closeThisInst(ito::AddInBase** addInInst);
};

//----------------------------------------------------------------------------------------------------------------------------------
class FaulhaberMCS : public ito::AddInActuator
{
    Q_OBJECT;

protected:
    //! Destructor
    ~FaulhaberMCS();
    //! Constructor
    FaulhaberMCS();

public:
    friend class FaulhaberMCSInterface;
    const ito::RetVal showConfDialog(void);
    int hasConfDialog(void)
    {
        return 1;
    }; //!< indicates that this plugin has got a configuration dialog

private:
    int m_async; //!< variable to set up async and sync positioning --> Synchrone means program do
                 //!< not return until positioning was done.
    int m_nrOfAxes;

    ito::RetVal waitForDone(
        const int timeoutMS = -1,
        const QVector<int> axis = QVector<int>() /*if empty -> all axis*/,
        const int flags = 0 /*for your use*/);

    ito::RetVal waitForIntParam(
        const char* parameter,
        const int& newValue,
        const int& timeoutMS = 1000,
        const int& sleepMS = 10);

    ito::RetVal updateStatus(); // optional method to obtain the status and position of all
                                // connected axes

    HINSTANCE m_hProtocolDll;

    tdmmProtInitInterface mmProtInitInterface;
    tdmmProtCloseInterface mmProtCloseInterface;
    tdmmProtOpenCom mmProtOpenCom;
    tdmmProtCloseCom mmProtCloseCom;
    tdmmProtSendCommand mmProtSendCommand;
    tdmmProtReadAnswer mmProtReadAnswer;
    tdmmProtDecodeAnswStr mmProtDecodeAnswStr;
    tdmmProtGetStrObj mmProtGetStrObj;
    tdmmProtSetObj mmProtSetObj;
    tdmmProtGetAbortMessage mmProtGetAbortMessage;
    tdmmProtGetErrorMessage mmProtGetErrorMessage;
    tdmmProtGetObj mmProtGetObj;
    tdmmProtFindConnection mmProtFindConnection;
    tdmmProtSendMotionCommand mmProtSendMotionCommand;
    tdmmProtCheckMotionCommand mmProtCheckMotionCommand;

    int m_node;
    int m_statusWord;
    bool m_isComOpen;

    enum statuswordBits
    {
        readyToSwitchOn = 1 << 0,
        switchedOn = 1 << 1,
        operationEnabled = 1 << 2,
        fault = 1 << 3,
        voltageEnabled = 1 << 4,
        quickStop = 1 << 5,
        switchOnDisabled = 1 << 6,
        warning = 1 << 7,
        targetReached = 1 << 10,
        internalLimitActive = 1 << 11,
        setPointAcknowledged = 1 << 12,
        followingError = 1 << 13
    };


public slots:
    ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore* waitCond);

    ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore* waitCond);

    ito::RetVal init(
        QVector<ito::ParamBase>* paramsMand,
        QVector<ito::ParamBase>* paramsOpt,
        ItomSharedSemaphore* waitCond = NULL);

    ito::RetVal close(ItomSharedSemaphore* waitCond);

    ito::RetVal calib(const int axis, ItomSharedSemaphore* waitCond = NULL);
    ito::RetVal calib(const QVector<int> axis, ItomSharedSemaphore* waitCond = NULL);

    ito::RetVal setOrigin(const int axis, ItomSharedSemaphore* waitCond = NULL);
    ito::RetVal setOrigin(const QVector<int> axis, ItomSharedSemaphore* waitCond = NULL);

    ito::RetVal getStatus(QSharedPointer<QVector<int>> status, ItomSharedSemaphore* waitCond);

    ito::RetVal getPos(const int axis, QSharedPointer<double> pos, ItomSharedSemaphore* waitCond);
    ito::RetVal getPos(
        const QVector<int> axis,
        QSharedPointer<QVector<double>> pos,
        ItomSharedSemaphore* waitCond);

    ito::RetVal setPosAbs(const int axis, const double pos, ItomSharedSemaphore* waitCond = NULL);
    ito::RetVal setPosAbs(
        const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore* waitCond = NULL);

    ito::RetVal setPosRel(const int axis, const double pos, ItomSharedSemaphore* waitCond = NULL);
    ito::RetVal setPosRel(
        const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore* waitCond = NULL);

    // Faulhaber MCS methods
    ito::RetVal getSerialNumber(int& serialNum);
    ito::RetVal getVendorID(int& id);
    ito::RetVal getProductCode(int& code);
    ito::RetVal getRevisionNumber(int& num);
    ito::RetVal getDeviceName(const char*& name);
    ito::RetVal getSoftwareVersion(const char*& version);
    ito::RetVal getPosMCS(int& pos);
    ito::RetVal getTargetPosMCS(int& pos);
    ito::RetVal getAmbientTemperature(int& temp);
    ito::RetVal setPosAbsMCS(double& pos);
    ito::RetVal setPosRelMCS(double& pos);
    ito::RetVal quickstop();
    ito::RetVal getStatusword(int& status);
    ito::RetVal getTorqueGain(int& gain);
    ito::RetVal setTorqueGain(int& gain);
    ito::RetVal getTorqueIntegralTime(int& time);
    ito::RetVal setTorqueIntegralTime(int& time);
    ito::RetVal getVelocityGain(int& gain);
    ito::RetVal setVelocityGain(int& gain);
    ito::RetVal getVelocityIntegralTime(int& time);
    ito::RetVal setVelocityIntegralTime(int& time);
    ito::RetVal getVelocityDeviationThreshold(int& thres);
    ito::RetVal setVelocityDeviationThreshold(int& thres);
    ito::RetVal getVelocityDeviationTime(int& time);
    ito::RetVal setVelocityDeviationTime(int& time);
    ito::RetVal getVelocityWarningThreshold(int& thres);
    ito::RetVal setVelocityWarningThreshold(int& thres);
    ito::RetVal getVelocityIntegralPartOption(int& option);
    ito::RetVal setVelocityIntegralPartOption(int& option);

    void decodeStatusWord();


    int doubleToInteger(double& value);

private slots:
    void dockWidgetVisibilityChanged(bool visible);
};

#endif // FAULHABERMCS_H

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
    ito::AddInDataIO* m_pSerialIO;
    int m_delayAfterSendCommandMS;
    int m_requestTimeOutMS;
    int m_async; //!< variable to set up async and sync positioning --> Synchrone means program do
                 //!< not return until positioning was done.
    int m_numOfAxes;
    int m_waitForDoneTimeout = 60000;

    uint8_t m_node;


    uint8_t m_S = 0x53;
    uint8_t m_E = 0x45;
    uint8_t m_GET = 0x01;
    uint8_t m_SET = 0x02;

    int m_statusWord;

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

    ito::RetVal waitForDone(
        const int timeoutMS = -1,
        const QVector<int> axis = QVector<int>() /*if empty -> all axis*/,
        const int flags = 0 /*for your use*/);

    ito::RetVal updateStatus(); // optional method to obtain the status and position of all
                                // connected axes

    // SeralIO functions
    ito::RetVal sendCommandAndGetResponse(const QByteArray& command, QByteArray& response);
    ito::RetVal readResponse(QByteArray& result);

    ito::RetVal sendQuestionWithAnswerDouble(const QByteArray& questionCommand, double& answer);
    ito::RetVal sendQuestionWithAnswerDoubleArray(
        const QByteArray& questionCommand, double* answer, const int number);

    ito::RetVal readRegisterWithAnswerString(
        const uint16_t& address, const uint8_t& subindex, char*& answer);
    ito::RetVal readRegisterWithAnswerInteger(
        const uint16_t& address, const uint8_t& subindex, int& answer);

    ito::RetVal homingCurrentPosToZero(const int& axis);

    ito::RetVal readRegister(
        const uint16_t& address, const uint8_t& subindex, std::vector<uint8_t>& response);
    ito::RetVal parseResponse(const QByteArray& response, std::vector<uint8_t>& parsedResponse);

    uint8_t CRC(const std::vector<uint8_t>& message);

    int doubleToInteger(const double& value);

    ito::RetVal getSerialNumber(char*& serialNum);
    ito::RetVal getDeviceType(char*& serialNum);

public slots:
    ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore* waitCond);

    ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore* waitCond);

    ito::RetVal init(
        QVector<ito::ParamBase>* paramsMand,
        QVector<ito::ParamBase>* paramsOpt,
        ItomSharedSemaphore* waitCond = nullptr);

    ito::RetVal close(ItomSharedSemaphore* waitCond);

    ito::RetVal calib(const int axis, ItomSharedSemaphore* waitCond = nullptr);
    ito::RetVal calib(const QVector<int> axis, ItomSharedSemaphore* waitCond = nullptr);

    ito::RetVal setOrigin(const int axis, ItomSharedSemaphore* waitCond = nullptr);
    ito::RetVal setOrigin(const QVector<int> axis, ItomSharedSemaphore* waitCond = nullptr);

    ito::RetVal getStatus(QSharedPointer<QVector<int>> status, ItomSharedSemaphore* waitCond);

    ito::RetVal getPos(const int axis, QSharedPointer<double> pos, ItomSharedSemaphore* waitCond);
    ito::RetVal getPos(
        const QVector<int> axis,
        QSharedPointer<QVector<double>> pos,
        ItomSharedSemaphore* waitCond);

    ito::RetVal setPosAbs(
        const int axis, const double pos, ItomSharedSemaphore* waitCond = nullptr);
    ito::RetVal setPosAbs(
        const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore* waitCond = nullptr);

    ito::RetVal setPosRel(
        const int axis, const double pos, ItomSharedSemaphore* waitCond = nullptr);
    ito::RetVal setPosRel(
        const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore* waitCond = nullptr);

    // Faulhaber MCS methods
    /*
    ito::RetVal getVendorID(int& id);
    ito::RetVal getProductCode(int& code);
    ito::RetVal getRevisionNumber(int& num);
    ito::RetVal getDeviceName(const char*& name);
    ito::RetVal getSoftwareVersion(const char*& version);
    ito::RetVal getPosMCS(int& pos);
    ito::RetVal getTargetPosMCS(int& pos);
    ito::RetVal getAmbientTemperature(int& temp);

    ito::RetVal setPosAbsMCS(double& pos);
    ito::RetVal setPosRelMCS(const double& pos);

    ito::RetVal getMaxMotorSpeed(int& speed);
    ito::RetVal setMaxMotorSpeed(const int& speed);

    ito::RetVal getProfileVelocity(int& speed);
    ito::RetVal setProfileVelocity(const int& speed);

    ito::RetVal getAcceleration(int& acceleration);
    ito::RetVal setAcceleration(const int& acceleration);

    ito::RetVal getDeceleration(int& deceleration);
    ito::RetVal setDeceleration(const int& deceleration);

    ito::RetVal getQuickStopDeceleration(int& deceleration);
    ito::RetVal setQuickStopDeceleration(const int& deceleration);


    ito::RetVal setHomingMode(const uint8_t& mode);

    ito::RetVal getTorqueLimits(int limits[]);
    ito::RetVal setTorqueLimits(const int limits[]);

    ito::RetVal getOperationMode(int& mode);
    ito::RetVal setOperationMode(const uint8_t& mode);

    ito::RetVal getControlword(int& word);
    ito::RetVal setControlword(const uint8_t& word, const int& len);
    ito::RetVal updateStatusMCS();*/

private slots:
    void dockWidgetVisibilityChanged(bool visible);
};

#endif // FAULHABERMCS_H

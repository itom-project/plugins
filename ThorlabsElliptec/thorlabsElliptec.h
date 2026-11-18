/* ********************************************************************
    Plugin "ThorlabsElliptec" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2025, Institut für Technische Optik (ITO),
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

#pragma once

#include "common/addInInterface.h"
#include "dialogThorlabsElliptec.h"
#include "dockWidgetThorlabsElliptec.h"
#include <qsharedpointer.h>

//------------------------------------------------------------------------------
class ThorlabsElliptecInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase")
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

public:
    ThorlabsElliptecInterface();
    ~ThorlabsElliptecInterface();
    ito::RetVal getAddInInst(ito::AddInBase** addInInst);
    bool hasDockWidget()
    {
        return true;
    }

private:
    ito::RetVal closeThisInst(ito::AddInBase** addInInst);
};

//------------------------------------------------------------------------------
class ElliptecDevice
{
public:
    ElliptecDevice() {};

    ElliptecDevice(
        const int &modelId,
        const QString& name,
        const QString& description,
        bool indexed,
        bool linear,
        const QString& unit,
        int numIndexedPositions,
        const QString supportedCmds,
        int numMotors,
        bool allowCleaning,
        bool allowOptimization
    ) :
        m_modelId(modelId), m_name(name), m_description(description),
        m_indexed(indexed), m_linear(linear),
        m_unit(unit), m_numIndexedPositions(numIndexedPositions), m_numMotors(numMotors),
        m_allowCleaning(allowCleaning), m_allowOptimization(allowOptimization)
    {
        if (!m_indexed)
        {
            m_numIndexedPositions = 0;
        }

        m_supportedCmds = supportedCmds.split(";");
    }

    QString m_name;
    QString m_description;
    bool m_indexed;
    bool m_linear; //linear: true, rotation: false
    QString m_unit;
    int m_numIndexedPositions;
    int m_modelId;
    QStringList m_supportedCmds;
    int m_numMotors;
    bool m_allowCleaning;
    bool m_allowOptimization;
};

//------------------------------------------------------------------------------
class ThorlabsElliptec : public ito::AddInActuator
{
    Q_OBJECT;

protected:
    //! Destructor
    ~ThorlabsElliptec();
    //! Constructor
    ThorlabsElliptec();

public:
    friend class ThorlabsElliptecInterface;
    const ito::RetVal showConfDialog(void);

    //!< indicates that this plugin has got a configuration dialog
    int hasConfDialog(void)
    {
        return 1;
    };

private:
    struct CmdInfo
    {
        CmdInfo() {};
        CmdInfo(
            const QByteArray& sendCmd,
            const QByteArray& rcvCmd,
            int sendDataNumBytes,
            int rcvDataNumBytes,
            bool canReturnStatus)
        {
            this->sendCmd = sendCmd;
            this->rcvCmd = rcvCmd;
            this->sendDataNumBytes = sendDataNumBytes;
            this->rcvDataNumBytes = rcvDataNumBytes;
            this->canReturnStatus = canReturnStatus;
        };

        QByteArray sendCmd;
        QByteArray rcvCmd;
        int sendDataNumBytes;
        int rcvDataNumBytes;
        bool canReturnStatus; // if true, the command can also return with GS
    };

    ito::AddInDataIO* m_pSerialIO;

    //!< variable to set up async and sync positioning --> Synchrone means program do
    //!< not return until positioning was done.
    int m_async;

    const int m_serialBufferSize;
    QSharedPointer<int> m_serialBufferLength;
    QSharedPointer<char> m_serialBuffer;
    bool m_serialMutexLocked;
    int m_requestTimeOutMS;
    int m_waitForDoneTimeoutMS;

    int m_address;
    ElliptecDevice m_model;
    static QList<ElliptecDevice> elliptecModels;
    static QMap<QByteArray, CmdInfo> supportedCmds;

    static void initElliptecModels();
    static void initSupportedCmds();

    ito::RetVal waitForDone(
        const int timeoutMS = -1,
        const QVector<int> axis = QVector<int>() /*if empty -> all axis*/,
        const int flags = 0 /*for your use*/);

    bool getCmdInfo(const QByteArray& cmd, CmdInfo& info) const;
    ito::RetVal sendCommand(unsigned char address, const QByteArray& cmdId, const QByteArray& data = QByteArray());
    ito::RetVal sendCommandAndGetResponse(unsigned char address, const QByteArray& cmdId, const QByteArray& data, int timeoutMs, QByteArray &response);
    ito::RetVal sendCommandAndGetResponse(
        unsigned char address,
        const QByteArray& cmdId,
        int data,
        int timeoutMs,
        QByteArray& response);
    ito::RetVal readResponse(int timeoutMs, QByteArray& response);
    ito::RetVal parseStatusResponse(const QByteArray& response) const;
    double positionFromPosResponse(const QByteArray& response) const;
    QByteArray positionTo8ByteArray(double position) const;
    QByteArray intToByteArray(int value, int numBytes) const;
    int byteArrayToInt(const QByteArray& value) const;
    int getFrequencyFromWord(const QByteArray& ba);
    ito::RetVal identifyDevices();
    ito::RetVal updateMotorFrequencies();
    ito::RetVal saveUserData();

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

    ito::RetVal execFunc(
        const QString funcName,
        QSharedPointer<QVector<ito::ParamBase>> paramsMand,
        QSharedPointer<QVector<ito::ParamBase>> paramsOpt,
        QSharedPointer<QVector<ito::ParamBase>> paramsOut,
        ItomSharedSemaphore* waitCond = nullptr);

private slots:
    void dockWidgetVisibilityChanged(bool visible);
};
